#!/usr/bin/env python3
"""rigd — the Jetson-side orchestrator and web UI for the camera fleet.

One rigd runs on the Jetson. It:
  * monitors every camera node and reconnects them automatically;
  * holds ONE desired settings vector and continuously converges every camera
    onto it, so the pair is always identically configured regardless of what
    state a body rebooted into;
  * runs transects — synchronized capture, per-camera pull + rename + flight
    log, with nav and IMU stamped onto every frame;
  * exposes a structured event/diag/anomaly surface for humans and for an AI
    watcher to observe field faults and correct them live;
  * serves a tabbed web UI (review / fleet / nav / imu / controls / events)
    that runs with no AI in the loop.

Stdlib only, plus rig/nav.py (which itself needs pyserial). nav is optional:
without it, runs still work and the flight log simply carries empty nav columns.
"""

import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rigcore
from rigcore import (NODES, EventLog, NodeMonitor, RunBrowser, RunsError,
                     SettingsManager, TimeSync, free_mb, http_json, http_bytes)
from run import RunManager
import drain as draindrv

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = int(os.environ.get("RIGD_PORT", "9090"))
UI_PATH = os.path.join(HERE, "rig_ui.html")

# Free space on the JETSON volume below which a transect is at risk. Nothing
# measured this before: the disk_low detector and the UI's "Disk" field both
# report a Pi's PC-save spool, while every transect is written to ~/rig-runs on
# the Jetson. A full Jetson volume is not survivable in-run - each frame's write
# raises OSError and the frame is never retried, so it gets no flight_log row
# and no nav/IMU correlation, permanently - so the warning has to come early.
# Sized on the worst case: at transsize=0 a delivered JPEG measured 14.1 MB, so
# 5 GB is only ~180 stereo pairs.
RUNS_DISK_LOW_MB = 5000
# How long a card write may stay in progress before it is a stuck write rather
# than a busy one. This is the fault that took cam1 down on 2026-08-16: one
# frame wedged in the body's write buffer locks the whole property table, kills
# PC delivery, and makes ilxctl look dead - while `slotStatus` still reads OK,
# which is why nothing detected it. slotWriting is the field that does.
SLOT_WRITING_STUCK_S = 30.0
SLOT_WRITING_CRITICAL_S = 120.0


# ---------------------------------------------------------------------------
# Request validation.
#
# What failed: the HTTP surface trusted whatever JSON arrived. POST
# /api/run/start with a MALFORMED body started a real transect on the built-in
# defaults, because _read_body answered {} for "cannot parse" exactly as it did
# for "no body at all" (fuzz 2026-08-23, verified on the live rig). Downstream
# of that, every handler did b.get(...) on a value it never checked:
# {"on":"maybe"} enabled the exposure servo (bool("maybe") is True),
# {"value":"abc"} on /api/exposure travelled to ilxctl and stalled the SDK ~6 s
# inside SetDeviceProperty until cam1 flapped to "no connected camera", and
# ?since=abc / steps="x" came back as 500s carrying a Python exception string.
#
# Why it is shaped this way: one exception type, raised by small coercers, and
# ONE catch in each of do_GET/do_POST. Returning error tuples relies on every
# handler remembering to look at them - which is the bug being fixed. A
# BadRequest is always a clean 400 naming the field; nothing else changes.
# ---------------------------------------------------------------------------
class BadRequest(ValueError):
    """Client input this daemon refuses to act on. Answered 400, never 500."""


def want_bool(v, what):
    """A REAL bool: JSON true/false, or the integers 0/1. Nothing else.

    A truthy STRING is not consent - {"on":"maybe"} armed the auto-exposure
    servo on the live rig."""
    if isinstance(v, bool):
        return v
    if isinstance(v, int) and v in (0, 1):
        return bool(v)
    raise BadRequest("%s must be true or false (got %r)" % (what, v))


def want_int(v, what, lo=None, hi=None):
    """A whole number. JSON number or a clean decimal string; bools, fractions
    and anything non-numeric are refused HERE, before a node is contacted."""
    n = None
    if isinstance(v, bool):
        n = None
    elif isinstance(v, int):
        n = v
    elif isinstance(v, float):
        # json.loads accepts the non-standard literals Infinity/-Infinity/NaN
        # by default, and int() raises OverflowError / ValueError on them - so
        # {"steps": Infinity} left want_int by EXCEPTION and became a 500
        # carrying a Python exception string plus an "error"/"http" line in
        # rigd.jsonl, which is the exact outcome BadRequest exists to prevent
        # (audit 2026-08-24). want_float below already refused them.
        n = int(v) if (v == v and v not in (float("inf"), float("-inf"))
                       and v == int(v)) else None
    elif isinstance(v, str):
        try:
            n = int(v.strip(), 10)
        except ValueError:
            n = None
    if n is None:
        raise BadRequest("%s must be a whole number (got %r)" % (what, v))
    _bounds(n, what, lo, hi)
    return n


def _bounds(n, what, lo, hi):
    """One-sided or two-sided range message: "between 0 and None" told the
    operator nothing about which end they were on."""
    if lo is not None and hi is not None and not (lo <= n <= hi):
        raise BadRequest("%s must be between %s and %s (got %s)"
                         % (what, lo, hi, n))
    if lo is not None and n < lo:
        raise BadRequest("%s must be %s or more (got %s)" % (what, lo, n))
    if hi is not None and n > hi:
        raise BadRequest("%s must be %s or less (got %s)" % (what, hi, n))


def want_float(v, what, lo=None, hi=None):
    """A finite number. NaN/inf are refused too: they poison every comparison
    downstream (an IMU window at t0=nan matches no sample and raises nothing)."""
    try:
        n = float(v)
    except (TypeError, ValueError):
        raise BadRequest("%s must be a number (got %r)" % (what, v)) from None
    if n != n or n in (float("inf"), float("-inf")):
        raise BadRequest("%s must be a finite number (got %r)" % (what, v))
    _bounds(n, what, lo, hi)
    return n


def qint(q, key, default, lo=None, hi=None):
    """One integer query parameter, or `default` when it is absent/empty."""
    vals = q.get(key)
    if not vals or vals[0] == "":
        return default
    return want_int(vals[0], "?%s" % key, lo, hi)


def qfloat(q, key, default, lo=None, hi=None):
    vals = q.get(key)
    if not vals or vals[0] == "":
        return default
    return want_float(vals[0], "?%s" % key, lo, hi)


# The `which` names ilxctl's /api/exposure actually dispatches (src/camera.cpp
# Camera::setExposure). Anything outside this set is a typo, and a typo used to
# reach the body: it is refused at the door instead.
EXPOSURE_WHICH = ("iso", "shutter", "aperture", "program", "drive",
                  "filetype", "imagesize", "quality", "transsize",
                  "pcsave", "rawtype", "expcomp", "wb_mode", "colortemp")

# Lens endpoints: path -> (the body field ilxctl reads, the /api/status range
# key that bounds it, or None where the body publishes no range). Position and
# speed are per-camera and never fleet-converged, so the body's own published
# range is the only guard available - out-of-range values used to be forwarded
# and silently dropped, answering ok:true with empty results.
LENS_FIELD = {"/api/focus/drive": ("step", None),
              "/api/focus/position": ("value", "focusPosRange"),
              "/api/zoom/drive": ("speed", "zoomSpeedRange"),
              "/api/zoom/position": ("value", "zoomPosRange"),
              "/api/zoom/setting": ("value", None)}


def _clamp_to_range(v, rng):
    """(clamped, was_clamped) against an ilxctl {min,max,step} range triple."""
    if not isinstance(rng, dict):
        return v, False
    lo, hi = rng.get("min"), rng.get("max")
    out = v
    if isinstance(lo, (int, float)) and out < lo:
        out = int(lo)
    if isinstance(hi, (int, float)) and out > hi:
        out = int(hi)
    return out, out != v


# ---------------------------------------------------------------------------
# Anomaly detectors — cheap checks over current fleet state, each with the
# evidence and a suggested action an operator (or agent) can act on.
# ---------------------------------------------------------------------------
STATIC_FIX_PATH = os.path.expanduser("~/rig/static_fix.json")
# Host-vs-node clock offset that starts costing sync. The whole stereo budget
# is 10 ms; the fire schedule's lead is SYNC_LEAD_S 0.30 s minus the focus lead
# and the trigger latency, so an undisciplined host eats it outright. Measured
# on the Mac 2026-08-23: 187 ms behind NTP, drifting ~60 ppm - every fire late
# by ~33 ms on both nodes and the pair skew degraded 0.59 -> 1.78 ms.
HOST_CLOCK_WARN_S = 0.1
HOST_CLOCK_BAD_S = 0.5
# An offset measured over a link this slow is not a clock reading, it is the
# network. Say so instead of dropping the node out of the comparison silently.
CLOCK_RTT_LIMIT_MS = 20.0
# A node_clock_skew alarm must survive this many consecutive scans. The live
# journal carried single-sample "82.9 ms" skew alarms while chronyc showed the
# two Pis 20 us apart: one host-clock step landing between the two nodes' polls
# is enough to invent one.
CLOCK_SKEW_SCANS = 3


class LiveTap:
    """Per-camera live-view throttle: rigd is the SINGLE point where live
    view touches a node. One upstream fetch in flight per camera, every
    client shares it, and a cached frame is served while it is younger than
    the policy interval — so no browser tab, second operator, or stray curl
    loop can exceed the cap.

    Why a cap at all (bench, 2026-08-23): live view polled unbounded (~53 fps)
    on both cameras during a 2 Hz RAW transect starved cam1's piagent fire
    path (23 of 28 fires timed out) and starved cam2's SDK transfer path to
    zero delivered frames, then wedged its body. The same transect with live
    view at ~6 fps per camera was clean: 30/30, 0 late, 0.19 ms skew, 6 ms
    worst preview latency. Policy: 5 fps idle, 2 fps while a run is active
    (conservative until measured at more), 0 fps with no viewer (pull-based:
    nothing is fetched unless someone asks)."""

    IDLE_S = 0.20        # 5 fps, no run active
    RUN_S = 0.50         # 2 fps while a transect is recording
    TIMEOUT_S = 1.5      # an SDK stall must not freeze the picture for 8 s

    def __init__(self):
        self._ent = {}
        self._lock = threading.Lock()

    def _slot(self, name):
        with self._lock:
            e = self._ent.get(name)
            if e is None:
                e = self._ent[name] = {"lock": threading.Lock(), "data": None,
                                       "at": 0.0, "err": None}
            return e

    def get(self, m, run_active):
        """Returns (jpeg_bytes_or_None, error, age_ms, policy)."""
        interval = self.RUN_S if run_active else self.IDLE_S
        policy = "run" if run_active else "idle"
        e = self._slot(m.name_)
        with e["lock"]:          # concurrent clients queue here and share
            now = time.monotonic()
            if e["data"] is not None and now - e["at"] < interval:
                return e["data"], None, (now - e["at"]) * 1000.0, policy
            data, err = http_bytes("http://%s:8080/liveview.jpg" % m.host,
                                   timeout=self.TIMEOUT_S)
            if data:
                e["data"], e["at"], e["err"] = data, time.monotonic(), None
                return data, None, 0.0, policy
            e["err"] = err
            # Serve the last good frame briefly rather than nothing, but
            # never pretend it is fresh: the age header says so.
            if e["data"] is not None and now - e["at"] < 3.0:
                return e["data"], err, (now - e["at"]) * 1000.0, policy
            return None, err or "no liveview", None, policy


LIVETAP = LiveTap()


# "this wrapper has no way to ask whether a run is recording" - distinct from
# "no run is recording", because the two want opposite answers: unknown must
# not let a Start re-decide a live run's latch, and must not turn the idle
# static fix off either.
UNKNOWN_RUN = object()


class StaticFixNav:
    """Delegating wrapper around NavReader: when the gateway is DEAD and there
    is no live fix, fix_at()/snapshot() fall back to the operator-provided
    static position in ~/rig/static_fix.json ({lat, lon, label, ...}).

    Field case: no NMEA aboard, but the site position is known — e.g. the
    last live fix from a previous day. Honesty rules: a live fix ALWAYS wins;
    the static row carries only position/UTM (never depth, heading, or speed,
    which we do not know); `nav_epoch` is the original fix's capture epoch so
    `age_s` says exactly how old the position is; `valid` stays False and
    `fix_kind` says "static"; and health()/snapshot() name the static source so
    the UI preflight can say so out loud. The file is re-read on mtime change,
    so it can be edited without a restart.

    ARMING (audit 2026-08-23, nav finding #1). The stand-in used to apply to
    ANY row without a live fix, which meant a transect that started on good GPS
    and lost the bus for 40 s mid-line had those frames stamped with the static
    position instead of an empty one. Photogrammetrically that is worse than no
    position: the flight_log looks complete and nothing says those rows are a
    constant. PROTOCOL.md's rule is that a missing fix writes EMPTY cells, and
    the static fix is an armed FALLBACK, not a gap filler. So the stand-in is
    decided once, at run start, from whether the gateway was online then:
      * gateway offline at run start -> armed for the whole run (the no-NMEA
        deployment: every frame carries the operator's position, labelled);
      * gateway online at run start  -> NOT armed; a mid-run gap writes empty
        lat/long and raises nav_no_fix, exactly as the contract says.
    Outside a run the current gateway state decides, so the UI and a one-off
    /api/capture still show the armed position when there is genuinely no bus.
    The decision BELONGS TO THAT RUN (see begin_run): only the run that took it
    can release it, and while a run is recording with no decision latched at
    all the stand-in is refused rather than guessed."""

    # A start that took the latch and never bound a run to it (the handler
    # died between begin_run and bind_run) must not hold it for ever. It has
    # to be comfortably longer than a slow RunManager.start - which live-probes
    # every monitor with a 5 s timeout, outside its own lock, before the run
    # becomes visible - and short enough that a leaked latch self-heals.
    START_GRACE_S = 60.0

    def __init__(self, reader, navmod, events, run_active=None):
        self._r = reader
        self._nm = navmod
        self._ev = events
        self._sf = None
        self._sf_mtime = None
        # The latch AND ITS OWNER. _run_armed None = no latch; True/False = the
        # decision taken at run start. _run_id names the run that owns it once
        # the run exists, _run_gen is the start that took it, _pending_since
        # marks a start still in flight. The four move together under the lock.
        self._run_armed = None
        self._run_id = None
        self._run_gen = 0
        self._pending_since = None
        self._run_lock = threading.RLock()
        # Callable -> the run_id recording RIGHT NOW, or None. MUST NOT take
        # the run manager's lock: armed() runs on the pull workers' threads,
        # once per frame, and that lock is not reentrant. None (no hint wired)
        # reads as "unknown", which is the conservative answer everywhere.
        self._run_active = run_active
        self._load()

    def _load(self):
        try:
            mt = os.path.getmtime(STATIC_FIX_PATH)
        except OSError:
            self._sf = None
            self._sf_mtime = None
            return
        if mt == self._sf_mtime:
            return
        # Stamp the mtime BEFORE parsing, so a malformed file is parsed once
        # and warned about once. It used to be stamped only on success: every
        # snapshot(), fix_at() and health() call re-read the broken file and
        # re-emitted the warning, which at the UI's poll rate buried the event
        # journal under one line per 200 ms.
        self._sf_mtime = mt
        try:
            with open(STATIC_FIX_PATH) as fh:
                sf = json.load(fh)
            if not isinstance(sf, dict):
                raise ValueError("not a JSON object")
            lat, lon = float(sf["lat"]), float(sf["lon"])
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                raise ValueError("lat/lon out of range")
            e, n, zone = self._nm.latlon_to_utm(lat, lon)
            sf.update({"lat": lat, "lon": lon,
                       "xutm": e, "yutm": n, "utm_zone": zone})
            self._sf = sf
        except Exception as exc:  # noqa: BLE001
            self._sf = None
            self._ev.emit("warn", "nav",
                          "static_fix.json unreadable (%s): no static fallback "
                          "is armed - frames with no live fix will carry EMPTY "
                          "positions" % exc)

    # -- arming -------------------------------------------------------------
    # OWNERSHIP (audit 2026-08-24, review blocker). The latch used to be a bare
    # process-global tri-state with no run identity, and two call sites cleared
    # it unconditionally:
    #   * the 5 s _nav_time_loop tick, which cannot tell "the run ended" from
    #     "the run has not started yet" - RunManager.start live-probes every
    #     monitor with a 5 s timeout BEFORE self.active is assigned, so with one
    #     node unreachable status() reports active:False for seconds and the
    #     tick is guaranteed to land inside that window;
    #   * the /api/run/start error path, which fired on "run already active" -
    #     i.e. a duplicate Start (a double tap, a UI retry) dropped the latch of
    #     the transect that WAS recording.
    # Either way the run then ran with no latch, armed() fell back to live
    # gateway state, and a bus drop 20 minutes later wrote the armed static
    # LAUNCH-POINT position into every remaining flight_log row of a moving
    # survey - fabricated position data presented as real, which is exactly
    # what the arming rule exists to prevent. So the latch now names its owner,
    # and only its owner may release it.
    def begin_run(self, gateway_online):
        """Latch the fallback decision for one transect.

        Returns (armed, token). `token` is this caller's proof that IT took the
        latch; it is None when the latch is already spoken for - by a recording
        run or by another start still in flight - and then the caller must
        neither re-decide nor release it."""
        with self._run_lock:
            rec = self._recording()
            if self._run_id is not None and (rec is UNKNOWN_RUN
                                             or rec == self._run_id):
                # A transect that is still recording owns the decision.
                return self._run_armed, None
            if self._run_id is None and self._pending():
                # Another start is in flight and its run needs its decision.
                return self._run_armed, None
            # Either nothing is latched, or the run that latched it is over
            # (its Stop and this Start raced, and the tick has not caught up).
            # A new transect gets a NEW decision - inheriting the last one
            # would arm this line from a gateway state sampled during another.
            self._run_gen += 1
            self._run_armed = not gateway_online
            self._run_id = None
            self._pending_since = time.monotonic()
            return self._run_armed, self._run_gen

    def _pending(self):
        """A start took the latch and has not bound a run to it yet. Callers
        hold _run_lock."""
        return (self._pending_since is not None
                and time.monotonic() - self._pending_since < self.START_GRACE_S)

    def _recording(self):
        """The run_id recording right now, None when none is, or UNKNOWN when
        this wrapper was built without the hint."""
        if self._run_active is None:
            return UNKNOWN_RUN
        try:
            return self._run_active()
        except Exception:  # noqa: BLE001
            return UNKNOWN_RUN

    def bind_run(self, run_id, token=None):
        """Name the run that owns the latch. Idempotent, and the nav tick calls
        it too, so a start that answered its client but never reached this
        still ends up with an owned latch."""
        with self._run_lock:
            if not run_id or self._run_armed is None:
                return False
            if token is not None and token != self._run_gen:
                return False            # a later start holds the latch now
            if self._run_id is not None and self._run_id != run_id:
                return False            # another run already owns it
            self._run_id = run_id
            self._pending_since = None
            return True

    def end_run(self, run_id=None, token=None):
        """Release the latch, but only for a caller that owns it. Returns True
        if it was actually released."""
        with self._run_lock:
            if self._run_armed is None:
                return False
            if token is not None:
                # A start that was refused: only the start that took the latch
                # may drop it, and only while no run has claimed it since.
                if token != self._run_gen or self._run_id is not None:
                    return False
            elif self._run_id is not None:
                # Owned by a run. run_id=None is the housekeeping tick, which
                # has just seen that no run is active, so it may release.
                if run_id is not None and run_id != self._run_id:
                    return False
            elif self._pending():
                # A start is IN FLIGHT and the run it is about to create still
                # needs this decision. This is the clear that used to strip a
                # starting transect's latch during RunManager.start's probe.
                return False
            self._run_armed = None
            self._run_id = None
            self._pending_since = None
            return True

    def armed(self):
        """Whether a static stand-in may be used for the NEXT row."""
        self._load()
        if not self._sf:
            return False
        with self._run_lock:
            rec = self._recording()
            live = rec is not None and rec is not UNKNOWN_RUN
            if self._run_armed is not None:
                if live and self._run_id is not None and rec != self._run_id:
                    # The latch belongs to a transect that is over and a
                    # DIFFERENT one is recording (its Stop raced this Start, so
                    # the new run never got a decision of its own). One line's
                    # decision must not govern another's rows.
                    return False
                return self._run_armed
            if live:
                # A transect is RECORDING and no decision is latched: the latch
                # was never taken, or was dropped under us. Refuse rather than
                # let live gateway state decide - PROTOCOL.md's rule is that a
                # row with no fix carries EMPTY cells, and a missing position
                # is recoverable where a fabricated one is not.
                return False
        # Idle: the gateway's current state decides. NavReader.gateway_online
        # is a cheap derived property (last-receive age), not I/O.
        online = getattr(self._r, "gateway_online", None)
        if online is None:
            try:
                online = bool((self._r.health() or {}).get("online"))
            except Exception:  # noqa: BLE001
                online = False
        return not bool(online)

    def static_label(self):
        self._load()
        return (self._sf or {}).get("label") or \
            ("static fix" if self._sf else None)

    def _stand_in(self, row):
        sf = self._sf
        row.update({"lat": sf["lat"], "lon": sf["lon"], "long": sf["lon"],
                    "xutm": sf["xutm"], "yutm": sf["yutm"],
                    "utm_zone": sf["utm_zone"],
                    # A static position is NOT a fix. Anything that keys on
                    # `valid` (the UI pill, the preflight, ingest) has to keep
                    # seeing False, and fix_kind says which of the two "not
                    # valid" states this is.
                    "valid": False, "fix_kind": "static",
                    "static_fix": self.static_label()})
        row["nav_epoch"] = sf.get("captured_epoch")
        if row["nav_epoch"] and row.get("local_epoch"):
            row["age_s"] = abs(row["local_epoch"] - row["nav_epoch"])
        return row

    def fix_at(self, epoch=None, max_age_s=None):
        row = self._r.fix_at(epoch, max_age_s)
        if row.get("valid"):
            row["fix_kind"] = "live"
            return row
        if not self.armed():
            # PROTOCOL.md: never fabricate. Empty cells, and the anomaly
            # scanner raises nav_no_fix on the way past.
            row["fix_kind"] = "none"
            return row
        return self._stand_in(row)

    def snapshot(self):
        snap = self._r.snapshot() or {}
        if snap.get("valid"):
            snap["fix_kind"] = "live"
            return snap
        if not self.armed():
            snap["fix_kind"] = "none"
            return snap
        return self._stand_in(snap)

    def health(self):
        h = self._r.health()
        self._load()
        h["static_fix"] = self.static_label()
        h["static_fix_armed"] = self.armed()
        return h

    def __getattr__(self, name):
        # set_raw_hook, bus_table, stop, port, ... — the reader's surface.
        return getattr(self._r, name)


class Anomalies:
    def __init__(self, monitors, runmgr, nav, events, settings=None):
        self.monitors = monitors
        self.runmgr = runmgr
        self.nav = nav
        self.events = events
        self.settings = settings
        self._flap = {}          # node -> deque of transition times
        self._last = {}
        self._writing_since = {}  # node -> epoch its card write started
        self._disk = (0.0, None)  # (checked_at, free_mb) — see _runs_free_mb
        self._skew_streak = {}   # (nodeA,nodeB) -> consecutive scans over budget
        # scan() runs from the 2.5 s loop AND from /api/anomalies and /api/diag,
        # which the UI's preflight() issues together in one Promise.all. Three
        # ThreadingHTTPServer threads used to compute `cur - self._last` before
        # any of them assigned self._last, so a single new fault (capture_paused,
        # card_write_stuck) was emitted to the journal two or three times: the
        # run's events.log carried duplicates, recent_counts() double-counted
        # the kind, and a watcher polling /api/events saw several distinct
        # errors for one event. One lock, held across the whole scan, because
        # the read-then-assign of _last (and of _skew_streak) is the invariant.
        self._lock = threading.Lock()

    def _runs_free_mb(self):
        """Free space on the runs volume, cached for a second.

        scan() is called from /api/anomalies (every 2.5 s per browser) and from
        /api/diag; the syscall is cheap but there is no reason to make it once
        per client per poll."""
        at, val = self._disk
        now = time.time()
        if now - at < 1.0:
            return val
        val = free_mb(rigcore.RUNS_DIR)
        self._disk = (now, val)
        return val

    def scan(self):
        with self._lock:
            return self._scan_locked()

    def _scan_locked(self):
        out = []
        now = time.time()
        run = self.runmgr.status()
        for m in self.monitors:
            snap = m.snapshot()
            st = snap["state"]
            h = snap.get("health") or {}
            status = snap.get("status") or {}
            if st == NodeMonitor.OFFLINE:
                out.append(self._a("node_offline", m.name_, "node unreachable",
                                   {"last_seen_s": snap.get("age_s")},
                                   "check power, network, and that ilxctl/"
                                   "piagent are running", sev="bad"))
            elif st == NodeMonitor.REACHABLE:
                out.append(self._a("camera_absent", m.name_,
                                   "node up but camera not claimed",
                                   {"log": (status.get("log") or [])[-1:]},
                                   "check USB cable is a data cable, camera on, "
                                   "PC Remote mode", sev="bad"))
            elif st == NodeMonitor.ILX_DOWN:
                out.append(self._a(
                    "ilx_down", m.name_,
                    "ilxctl not answering (piagent is) - camera daemon wedged",
                    {"error": status.get("error"),
                     "last_seen_s": snap.get("age_s")},
                    "the SDK session is stuck (HANDOFF §2.2): power-cycle the "
                    "camera body first; if ilxctl still does not answer, on "
                    "the node: sudo pkill -9 -x ilxctl, USB unbind/bind, "
                    "sudo systemctl start ilxctl (or pull the Pi's PoE for "
                    "10 s). Fires still work; frames will NOT be delivered",
                    sev="bad"))
            if st == NodeMonitor.CONNECTED and \
                    str(status.get("slotStatus", "")).lower() in ("no card", "nocard"):
                out.append(self._a(
                    "card_missing", m.name_,
                    "%s has NO memory card" % m.name_,
                    {"slotStatus": status.get("slotStatus"),
                     "storeDest": status.get("storeDestLabel")},
                    "the shutter still fires on GPIO but nothing records and "
                    "nothing is delivered to the Pi - every shot is an orphan. "
                    "Insert a card before running",
                    sev="bad"))
            ts = getattr(self, "timesync", None)
            off = ts.exif_offset.get(m.name_) if ts else None
            if off is not None and abs(off) > 60:
                days = abs(off) / 86400.0
                out.append(self._a(
                    "camera_clock_wrong", m.name_,
                    "%s body clock is %s %s the rig (EXIF)"
                    % (m.name_, ("%.1f days" % days) if days >= 1
                       else ("%.0f s" % abs(off)),
                       "behind" if off < 0 else "ahead"),
                    {"offset_s": round(off, 2)},
                    "set date/time in the body's own menu (USB cannot: the "
                    "ILX-LR1 refuses DateTime_Settings). Only the card files' "
                    "own timestamps are wrong - the rig's edge times and "
                    "the ingest sidecars are unaffected",
                    sev="warn"))
            # Pi spool not draining: delete-after-pull frees each frame's
            # PC-save copy the moment it is on host disk, so cam_frames should
            # stay small during a run. A climbing count means deletes are
            # failing (read-only spool, permission) and the Pi WILL fill.
            cf = h.get("cam_frames")
            if isinstance(cf, (int, float)) and cf > 400 and run.get("active"):
                out.append(self._a(
                    "spool_not_draining", m.name_,
                    "%s PC-save spool holds %d files during a run" % (m.name_, cf),
                    {"cam_frames": cf, "disk_free_mb": h.get("disk_free_mb")},
                    "frames are not being deleted after pull - the Pi will "
                    "fill. Check the spool is writable; run "
                    "POST /api/spool/prune on the node to recover",
                    sev="bad" if cf > 800 else "warn"))
            pw = h.get("power") or {}
            # Only alarm on under-voltage happening NOW. The since-boot bit
            # LATCHES until reboot, so alarming on it kept a permanent warning
            # up after a single momentary sag and trained the operator to
            # ignore the pill (audit 2026-08-23). The since-boot history is in
            # /health for anyone who looks.
            if pw.get("undervolt_now"):
                out.append(self._a(
                    "node_undervoltage", m.name_,
                    "%s UNDER-VOLTAGE NOW" % m.name_,
                    {"throttled": pw.get("throttled")},
                    "the PoE port/cable is sagging under load: this is the "
                    "step before the node reboots mid-run. Fix the power "
                    "budget before a survey", sev="bad"))
            rb = getattr(m, "rebooted_at", None)
            if rb and now - rb < 600:
                out.append(self._a(
                    "node_rebooted", m.name_,
                    "%s lost power and restarted %d s ago" % (m.name_, now - rb),
                    {"rebooted_at": rb, "uptime_s": h.get("uptime_s")},
                    "a Pi does not reboot by itself: PoE budget collapsed or "
                    "the port/cable sagged. Check the switch's PoE input, per-"
                    "port draw and shed events; isolate this node's power to "
                    "confirm. Frames fired while it was down are lost",
                    sev="bad"))
            # The body answers but its property table is gone: no writable
            # map, no ISO, slotWriting unreported. That is the card-stall
            # aftermath (HANDOFF §2.1: one frame the card will not accept
            # locks the whole table) or a half-dead SDK session — NOT ten
            # fields that each need attention. Say the one true thing and
            # hush the per-field divergence while it lasts.
            # A locked property table (the card-stall aftermath): the body
            # answers connected:true but GetSelectDeviceProperties returns
            # nothing, so ilxctl emits an EMPTY writable map, iso collapses to
            # "ISO 0"/isoValue 0, and slotWriting is unreported. Keying on
            # iso in (None,"","?") never matched real hardware (audit
            # 2026-08-23) - the body sends "ISO 0", not None.
            blind = (st == NodeMonitor.CONNECTED and status
                     and not (status.get("writable") or {})
                     and (status.get("isoValue") in (0, None)
                          or status.get("iso") in ("ISO 0", "0", None, "", "?"))
                     and status.get("slotWritingLabel") in (None, "unknown", ""))
            # WHY the table is blind decides whether it is an alarm. Every
            # clause above is satisfied by ABSENCE, and there are two benign
            # ways to get an empty table:
            #   * the degraded status body ({connected, busy:true, model:"",
            #     id:"", log}) ilxctl answers with when the SDK mutex is held
            #     past 4.5 s. PROTOCOL.md is explicit that busy:true means "the
            #     node answered and told us nothing - do not reconcile against
            #     it, do not count the missing keys as divergence"; diagnosing
            #     a card stall from them does precisely that, and put a red
            #     "power the body off and reformat the card" alarm in front of
            #     the operator for a body that was merely slow (audit
            #     2026-08-24, disputed finding - upheld on the contract, which
            #     does not depend on how often the window is hit);
            #   * a card drain: the property table is empty in transfer mode BY
            #     DESIGN, which is why rigcore already skips those nodes for
            #     reconcile. "locked" carries no information there either.
            # A genuine table lock answers promptly with an empty table and no
            # busy flag, so neither exemption hides the real card stall - and
            # card_write_stuck below watches the write itself regardless.
            excused = (bool(status.get("busy"))
                       or bool(getattr(m, "suspend_control", False))
                       or status.get("controlMode") == "transfer")
            locked = bool(blind) and not excused
            if locked:
                out.append(self._a(
                    "body_locked", m.name_,
                    "%s property table unavailable - body busy/locked"
                    % m.name_,
                    {"slotStatus": status.get("slotStatus"),
                     "log": (status.get("log") or [])[-1:]},
                    "the classic card stall (a write the card will not take "
                    "locks the whole body; slotStatus still says OK): power "
                    "the camera fully OFF, pull the card, full-format it on a "
                    "computer, format again in-camera - or replace it with a "
                    "V60/UHS-II card. Frames will not deliver until then",
                    sev="bad"))
            conv = snap.get("convergence") or {}
            if st == NodeMonitor.CONNECTED and not blind and not excused \
                    and conv.get("synced") is False and conv.get("diverged"):
                # Name the ilxctl error where there is one: for filetype /
                # imagesize / transsize there is no readback, so the body's own
                # words are the only evidence the field did not take.
                errs = conv.get("blind_errors") or {}
                out.append(self._a("settings_divergent", m.name_,
                                   "settings will not converge: %s"
                                   % ",".join(conv["diverged"]),
                                   {"fields": conv["diverged"],
                                    "errors": errs,
                                    "unsettable": conv.get("unsettable") or []},
                                   "the two bodies are NOT identically "
                                   "configured - check the field is settable in "
                                   "the current exposure mode, that the body is "
                                   "not sitting on its own menu, and that PC "
                                   "remote priority is still held",
                                   sev="bad"))
            # Camera health that ilxctl now reports and nothing consumed.
            # ONLY when the node is genuinely CONNECTED and answering: a node
            # that dropped OFFLINE mid-write keeps its last-good status showing
            # WRITING/overheating, and firing card_write_stuck on that stale
            # snapshot doubled up on node_offline/node_rebooted and sent the
            # operator to reformat a card during a plain power loss (audit
            # 2026-08-23).
            oh = status.get("overheatingLabel") if st == NodeMonitor.CONNECTED else None
            if oh in ("pre-overheat", "OVERHEATING"):
                out.append(self._a("camera_overheating", m.name_,
                                   "body overheating (%s)" % oh,
                                   {"overheating": status.get("overheating"),
                                    "label": oh},
                                   "the body shuts itself down when it reaches "
                                   "the limit, mid-transect - reduce duty "
                                   "cycle, get air over the housing, or stop "
                                   "and cool it now",
                                   sev="bad" if oh == "OVERHEATING" else "warn"))
            sw = status.get("slotWritingLabel") if st == NodeMonitor.CONNECTED else None
            if sw == "WRITING":
                since = self._writing_since.setdefault(m.name_, now)
                held = now - since
                if held > SLOT_WRITING_STUCK_S:
                    out.append(self._a(
                        "card_write_stuck", m.name_,
                        "card write has not completed in %.0fs" % held,
                        {"held_s": round(held, 1),
                         "slotStatus": status.get("slotStatus"),
                         "storeDest": status.get("storeDest")},
                        "one frame stuck in the write buffer takes the whole "
                        "body down: the property table locks, PC delivery "
                        "stops and ilxctl wedges, while slotStatus still reads "
                        "OK. Power the body fully off, then read/format the "
                        "card in a computer. A UHS-I or worn card cannot "
                        "sustain 61 MP RAW and will stall exactly this way",
                        sev="bad" if held > SLOT_WRITING_CRITICAL_S else "warn"))
            else:
                self._writing_since.pop(m.name_, None)
            # pc_control_lost and battery_low are read off `status`, and
            # NodeMonitor never clears status on the OFFLINE transition - it
            # only overwrites it on the next reachable poll. A node that lost
            # PoE mid-run therefore kept raising "the body has taken control
            # priority back" and "battery low (12%)" for a body that was not
            # even powered, on top of node_offline/node_rebooted, sending the
            # operator to the camera menu during a plain power loss (audit
            # 2026-08-23). Same CONNECTED gate the card/overheat checks use.
            pk = status.get("priorityKeyLabel") if st == NodeMonitor.CONNECTED \
                else None
            if pk == "Camera position":
                out.append(self._a(
                    "pc_control_lost", m.name_,
                    "the body has taken control priority back",
                    {"priorityKey": status.get("priorityKey"), "label": pk},
                    "PC Remote priority was lost, so writes will be refused "
                    "and PC save will not deliver - it masquerades as an SDK "
                    "bug. Set the body's priority back to PC remote",
                    sev="bad"))
            batt = status.get("battery") if st == NodeMonitor.CONNECTED else None
            if isinstance(batt, (int, float)) and 0 <= batt <= 15:
                out.append(self._a("battery_low", m.name_,
                                   "battery low (%s%%)" % batt, {"battery": batt},
                                   "bring the 12 V harness supply up or swap "
                                   "battery", sev="bad"))
            disk = h.get("disk_free_mb")
            if isinstance(disk, (int, float)) and disk < 2000:
                out.append(self._a("disk_low", m.name_,
                                   "node disk low (%s MB)" % disk,
                                   {"disk_free_mb": disk},
                                   "clear old frames from the PC-save dir",
                                   sev="bad"))
            imu = h.get("imu") or {}
            if imu.get("present") and imu.get("age_s") not in (None,) \
                    and isinstance(imu.get("age_s"), (int, float)) \
                    and imu["age_s"] > 3:
                out.append(self._a("imu_stall", m.name_,
                                   "IMU samples stale (%.1fs)" % imu["age_s"],
                                   {"age_s": imu["age_s"]},
                                   "check IMU wiring / power on this node"))
        # nav staleness
        if self.nav:
            try:
                snap = self.nav.snapshot()
                # "No GPS fix" and "the gateway is not talking to us" need
                # different fixes, and conflating them sends the operator to
                # the wrong end of the boat. The iKonvert is powered from the
                # N2K bus, not from USB, so a dark bus means silence on a port
                # that still opens perfectly well.
                kind = (snap or {}).get("fix_kind")
                # A run recording frames with NO position is the expensive
                # case and it has to be said first: the static fix is only
                # armed when the gateway was already dead at run start, so a
                # mid-run bus drop writes EMPTY lat/long into the flight_log
                # (PROTOCOL.md: never fabricate) and those frames cannot be
                # placed afterwards. The old code reached this through an
                # `elif` under nav_gateway_down, so the operator was told the
                # gateway was down but never that the transect was losing
                # position (audit 2026-08-23, nav finding #1).
                if run.get("active") and snap and snap.get("lat") is None:
                    out.append(self._a(
                        "nav_no_fix", None,
                        "RECORDING WITH NO POSITION - frames are getting "
                        "empty lat/long",
                        {"snap": {k: snap.get(k) for k in
                                  ("sats", "fix_source", "age_s",
                                   "gateway_online")},
                         "fix_kind": kind},
                        "these frames cannot be placed afterwards. Restore the "
                        "N2K/GPS source now, or stop the line and restart it "
                        "with the static fix armed (~/rig/static_fix.json) so "
                        "at least the site position is recorded",
                        sev="bad"))
                elif snap and not snap.get("gateway_online"):
                    # With a static fix standing in, the operator has already
                    # said "no NMEA aboard, use this position" — that is a
                    # state to display, not an alarm to chase.
                    sf = snap.get("static_fix") if kind == "static" else None
                    out.append(self._a("nav_gateway_down", None,
                                       ("no live NMEA — static fix in use: %s"
                                        % sf) if sf
                                       else "iKonvert sending no data",
                                       {"health": self.nav.health(),
                                        "fix_kind": kind},
                                       "flight-log positions use the armed "
                                       "static fix; plug in the iKonvert for "
                                       "live nav" if sf else
                                       "the iKonvert draws power from the N2K "
                                       "bus, not USB - check bus power and the "
                                       "gateway's POWER LED",
                                       sev="warn" if sf else "bad"))
                elif snap and snap.get("lat") is None:
                    out.append(self._a("nav_no_fix", None, "no GPS fix",
                                       {"snap": {k: snap.get(k) for k in
                                                 ("sats", "fix_source", "age_s")},
                                        "fix_kind": kind},
                                       "check the N2K backbone / GPS source"))
            except Exception:  # noqa: BLE001
                pass
        paused = (run.get("sync") or {}).get("paused_for") if run.get("active") else None
        if paused:
            out.append(self._a(
                "capture_paused", paused.get("node"),
                "capture paused: %s is not answering fires (since %d s)"
                % (paused.get("node"), now - (paused.get("since") or now)),
                {"since": paused.get("since"), "after_shot": paused.get("after_shot")},
                "the run is holding so the other camera does not shoot "
                "unpaired frames; it resumes by itself when the node answers "
                "its health poll. If it rebooted, see node_rebooted",
                sev="bad"))
        # jitter from the active run
        if run.get("active"):
            for node, s in (run.get("stats") or {}).items():
                jm = s.get("interval_jitter_ms")
                if isinstance(jm, (int, float)) and jm > 120:
                    out.append(self._a("jitter_high", node,
                                       "capture interval jitter high (%s ms)" % jm,
                                       {"jitter_ms": jm},
                                       "expect this on USB firing; wire the GPIO "
                                       "harness for tight sync"))
                pf = s.get("failed", 0)
                if pf and pf > 2:
                    out.append(self._a("pull_fail", node,
                                       "%d frame pulls failed" % pf,
                                       {"failed": pf},
                                       "check node link speed and disk",
                                       sev="bad"))
        # A held settings preview, read from the SettingsManager rather than
        # from a node's convergence badge: the manager is the only authority on
        # whether a pin exists, and a badge on a camera that has since dropped
        # offline cannot be trusted to clear itself.
        pv = self.settings.preview_state() if self.settings else {}
        if pv.get("active"):
            out.append(self._a(
                "preview_pinned", pv.get("node"),
                "holding a settings preview (%s) - the pair is NOT matched"
                % ",".join("%s=%s" % kv for kv in
                           sorted((pv.get("pending") or {}).items())),
                {"pending": pv.get("pending"),
                 "fleet": pv.get("desired"),
                 "expires_in_s": pv.get("expires_in_s")},
                "in Controls, Deploy to fleet or Discard. Frames taken now are "
                "a mismatched stereo pair, and a run start will drop the "
                "preview; it also expires on its own"))
        # The Jetson volume the transects are actually written to.
        # ---- clock domains (contract C1) ----------------------------------
        # Every scheduled fire epoch, every GPIO edge epoch/epoch_hw and every
        # piagent /health time.epoch is on the NODE clock; the fire scheduler,
        # nav's ring and rigd's own time.time() are on the HOST clock. Two
        # things can go wrong and they need different alarms and different
        # fixes, so they are two detectors:
        #   node_clock_skew   the two nodes disagree with EACH OTHER -> lands
        #                     1:1 in inter-camera exposure skew (10 ms budget).
        #   host_clock_offset both nodes agree but the HOST does not -> the
        #                     fire schedule's lead is eaten (every fire late)
        #                     and nav lookups made at a node epoch hit the
        #                     host-keyed ring at the wrong index.
        # Both read the FILTERED offset (NodeMonitor.clock_offset_s: RTT-gated
        # median of the recent window), never m.clock's last raw sample. A raw
        # sample carries whatever the network did during that one poll, and the
        # live journal shows what that costs: single-scan "82.9 ms" skew alarms
        # while chronyc had the two Pis 20 us apart.
        clocked, unmeasurable = [], []
        for m in self.monitors:
            if m.snapshot()["state"] == NodeMonitor.OFFLINE:
                continue          # a node that is not answering has no clock
            info = m.clock_offset_info()
            if info.get("offset_s") is None:
                continue
            # Do NOT drop a slow-linked node out of the comparison silently:
            # over a 20 ms link the midpoint estimate is network, not clock,
            # and the operator needs to know the check could not be made
            # rather than seeing an all-clear (audit 2026-08-23).
            if (info.get("rtt_ms_best") or 0.0) >= CLOCK_RTT_LIMIT_MS:
                unmeasurable.append((m.name_, info))
                continue
            clocked.append((m.name_, info))
        for name, info in unmeasurable:
            out.append(self._a(
                "node_clock_unmeasurable", name,
                "%s clock cannot be measured: best RTT %.1f ms over the last "
                "%d samples" % (name, info["rtt_ms_best"], info["n"]),
                {"rtt_ms_best": info["rtt_ms_best"], "n": info["n"],
                 "offset_ms": round(info["offset_s"] * 1e3, 2)},
                "the 10 ms stereo sync budget cannot be verified on this node "
                "while its link is this slow - the offset above is mostly "
                "network. Check the switch port / cable / PoE load, then "
                "re-check chronyc tracking"))
        # Node-to-node skew, but only once it has PERSISTED: a single scan is
        # not evidence (see above). The streak is keyed on the node pair and
        # reset the moment a scan comes back inside budget.
        seen_pairs = set()
        for i in range(len(clocked)):
            for j in range(i + 1, len(clocked)):
                (na, ia), (nb, ib) = clocked[i], clocked[j]
                pair = (na, nb)
                seen_pairs.add(pair)
                skew = abs(ia["offset_s"] - ib["offset_s"]) * 1000.0
                noise = (ia["rtt_ms_best"] + ib["rtt_ms_best"]) / 2.0
                if skew <= max(5.0, noise):
                    self._skew_streak.pop(pair, None)
                    continue
                n = self._skew_streak.get(pair, 0) + 1
                self._skew_streak[pair] = n
                if n < CLOCK_SKEW_SCANS:
                    continue
                out.append(self._a(
                    "node_clock_skew", None,
                    "%s and %s clocks disagree by %.1f ms (%d scans running)"
                    % (na, nb, skew, n),
                    {"skew_ms": round(skew, 2),
                     "offsets_ms": {na: round(ia["offset_s"] * 1e3, 2),
                                    nb: round(ib["offset_s"] * 1e3, 2)},
                     "rtt_noise_ms": round(noise, 2), "scans": n,
                     "samples": {na: ia["n"], nb: ib["n"]}},
                    "the whole stereo sync budget is 10 ms: scheduled fires "
                    "land this far apart and the strobe walks out of the "
                    "exposure window. Re-point both nodes' chrony at ONE "
                    "reachable master (peer cam2 to cam1) and confirm with "
                    "chronyc tracking",
                    sev="bad" if skew > 8.0 else "warn"))
        for pair in [p for p in self._skew_streak if p not in seen_pairs]:
            self._skew_streak.pop(pair, None)
        # The host itself. The nodes are chrony-locked to each other, so a
        # common-mode node-minus-host offset never shows up as skew: it was
        # invisible until it had eaten the fire schedule. Measured on this Mac
        # 2026-08-23: 187 ms, drifting ~60 ppm, both nodes reporting late_ms
        # ~33 ms on every fire and nav lookups ~190 ms off (19 cm at 1 m/s).
        if clocked:
            offs = sorted(i["offset_s"] for _, i in clocked)
            k = len(offs)
            med = offs[k // 2] if k % 2 else (offs[k // 2 - 1] + offs[k // 2]) / 2.0
            if abs(med) > HOST_CLOCK_WARN_S:
                out.append(self._a(
                    "host_clock_offset", None,
                    "the rigd host clock is %.0f ms %s the cameras"
                    % (abs(med) * 1e3, "behind" if med > 0 else "ahead"),
                    {"offset_s": round(med, 4),
                     "offset_ms": round(med * 1e3, 2),
                     "per_node_ms": {n: round(i["offset_s"] * 1e3, 2)
                                     for n, i in clocked},
                     "samples": {n: i["n"] for n, i in clocked}},
                    "every fire is scheduled on the host clock and busy-waited "
                    "on the node clock, so this offset comes straight out of "
                    "the 10 ms stereo sync budget - and nav fixes are looked "
                    "up at a node epoch against a host-keyed ring, putting "
                    "every frame's position out by this much. Enable network "
                    "time on the host (System Settings > General > Date & "
                    "Time), then confirm the nodes' chronyc tracking",
                    sev="bad" if abs(med) > HOST_CLOCK_BAD_S else "warn"))
        free = self._runs_free_mb()
        if isinstance(free, (int, float)) and free < RUNS_DISK_LOW_MB:
            out.append(self._a(
                "runs_disk_low", None,
                "Jetson runs volume low (%d MB free)" % free,
                {"free_mb": free, "path": rigcore.RUNS_DIR,
                 # Both the format the fleet runs today and the one a survey
                 # actually wants, because the difference is 44x.
                 "frames_left_at_320kb": int(free / 0.32),
                 "frames_left_at_14mb": int(free / 14.1)},
                "~/rig-runs is on this volume. When it fills, every frame write "
                "fails and those frames are NOT retried - they get no "
                "flight_log row at all. Move finished transects off now",
                sev="bad"))
        # emit newly-appearing anomalies once
        cur = {(a["kind"], a["node"]) for a in out}
        for key in cur - set(self._last):
            a = next(x for x in out if (x["kind"], x["node"]) == key)
            self.events.emit("error" if a.get("sev") == "bad" else "warn",
                             a["kind"], a["msg"], node=a["node"],
                             **a.get("evidence", {}))
        self._last = {k: now for k in cur}
        return out

    @staticmethod
    def _a(kind, node, msg, evidence, action, sev="warn"):
        # `sev` travels with the anomaly instead of being re-derived from the
        # kind's spelling by the UI: "card_write_stuck" and "camera_overheating"
        # matched none of the old severity patterns and would have rendered
        # amber, alongside advisories, while taking a body down.
        return {"kind": kind, "node": node, "msg": msg, "evidence": evidence,
                "suggested_action": action, "sev": sev, "since": time.time()}


class Rig:
    def __init__(self):
        self.events = EventLog()
        self.monitors = [NodeMonitor(n, self.events) for n in NODES]
        self.settings = SettingsManager(self.monitors, self.events)
        self.timesync = TimeSync(self.events)
        self.nav = self._start_nav()
        self._drain_lock = threading.Lock()
        self._drain_status = {"active": False, "node": None, "last": None,
                              "queue": [], "cancel_requested": False,
                              "wedged": {}}
        # Set to ask the running drain to stop. C4: the Drainer finishes the
        # file it is on and never cancels between pull-verify and delete, so a
        # cancel can never cost a card original.
        self._drain_stop = threading.Event()
        # node -> the monitor's rebooted_at at the moment its drain hit the
        # "card index not ready" wedge. See _drain_wedge_reason().
        self._drain_wedged = {}
        self._imu_lock = threading.Lock()
        self._imu_node = None
        self._imu_dead_until = 0.0
        self.runmgr = RunManager(self.monitors, self.settings, self.timesync,
                                 self.events, self.nav)
        self.anomalies = Anomalies(self.monitors, self.runmgr, self.nav,
                                   self.events, self.settings)
        self.anomalies.timesync = self.timesync
        self.runs = RunBrowser(self.events)
        self._stop = threading.Event()
        self._stopped = threading.Event()
        # Before anything else can write to the tree: repair whatever the last
        # process left behind.
        self._recover_runs()
        for m in self.monitors:
            m.start()
        threading.Thread(target=self._reconcile_loop, daemon=True).start()
        threading.Thread(target=self._nav_time_loop, daemon=True).start()
        threading.Thread(target=self._startup_calibrate, daemon=True).start()
        # Anomalies used to be evaluated ONLY when a browser polled
        # /api/anomalies: with no tab open, a stuck card write, a rebooted
        # node or a paused capture raised no event at all. Scan on a timer so
        # the journal records them whether or not anyone is watching.
        threading.Thread(target=self._anomaly_loop, daemon=True).start()
        self.events.emit("info", "lifecycle", "rigd up", port=PORT,
                         nodes=[m.name_ for m in self.monitors],
                         nav=bool(self.nav))

    def _recover_runs(self):
        """Finalise runs left open by a crash, a kill, or a power cut.

        rigd had no restart-time recovery, so an abnormal exit mid-transect left
        run.json with final:false, `frames` stale by up to 9 entries (the index
        is rewritten every 10th frame) and no per-worker stats at all. The
        imagery, the flight_log rows and events.log all survive - they are
        flushed as they are produced - so the run can be rebuilt from what is on
        disk, which is what this does: recount from each camera's flight_log,
        mark it final, and say plainly that it was interrupted rather than
        leaving a manifest that disagrees with its own directory.

        Bounded to the newest 20 runs: an old survey tree must not turn a rigd
        restart into a minutes-long scan before the fleet comes up."""
        try:
            listing = self.runs.list_runs(limit=20)
        except Exception as e:  # noqa: BLE001
            self.events.emit("warn", "run_recover",
                             "could not scan %s: %s" % (rigcore.RUNS_DIR, e))
            return
        for row in listing.get("runs", []):
            if row.get("final") or not row.get("has_run_json"):
                continue
            rid = row["run_id"]
            try:
                detail = self.runs.detail(rid)
                path = os.path.join(self.runs.run_dir(rid), "run.json")
                with open(path) as fh:
                    doc = json.load(fh)
                rows = {c: v.get("rows", 0)
                        for c, v in (detail.get("per_camera") or {}).items()}
                doc["final"] = True
                doc["interrupted"] = True
                doc["interrupted_note"] = (
                    "rigd exited before this run was stopped. frames/stats were "
                    "rebuilt from the flight_log files on disk at %s; the "
                    "in-memory per-frame index ends at the last multiple of 10."
                    % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
                doc["recovered"] = {"at": time.time(), "flight_log_rows": rows,
                                    "frames_indexed": doc.get("frames")}
                doc["frames"] = sum(rows.values()) or doc.get("frames")
                tmp = path + ".tmp"
                with open(tmp, "w") as fh:
                    json.dump(doc, fh, indent=1)
                os.replace(tmp, path)
                self.events.emit(
                    "warn", "run_recover",
                    "run %s was never finalised (rigd exited mid-transect); "
                    "rebuilt from flight_logs: %s" %
                    (rid, ", ".join("%s=%d rows" % kv
                                    for kv in sorted(rows.items())) or "none"),
                    run_id=rid, rows=rows)
            except (RunsError, OSError, ValueError) as e:
                self.events.emit("warn", "run_recover",
                                 "could not finalise %s: %s" % (rid, e),
                                 run_id=rid)

    def _start_nav(self):
        try:
            import nav as navmod
        except Exception as e:  # noqa: BLE001
            self.events.emit("info", "nav", "nav.py not available: %s" % e)
            return None
        reader = navmod.NavReader()          # resolves /dev/serial/by-id itself
        try:
            reader.open()
            self.events.emit("info", "nav", "nav reader opened on %s @ %d"
                             % (reader.port, reader.baud))
        except Exception as e:  # noqa: BLE001
            # Not fatal. The reader thread retries with backoff, so a gateway
            # that is plugged in - or has its bus powered - after rigd starts is
            # picked up without a restart. Returning None here used to disable
            # nav for the whole lifetime of the process.
            self.events.emit("warn", "nav", "iKonvert not open yet: %s" % e)
        reader.start()
        wrapped = StaticFixNav(reader, navmod, self.events,
                               run_active=self._recording_run_id)
        if wrapped.static_label():
            self.events.emit("warn", "nav",
                             "STATIC FIX armed: flight-log positions fall "
                             "back to '%s' whenever there is no live fix"
                             % wrapped.static_label())
        return wrapped

    DRAIN_DEST = os.path.expanduser("~/rig-raw")
    DRAIN_FLAG = os.path.expanduser("~/rig/auto_drain")

    def auto_drain_default(self):
        # Auto-drain after a run when ~/rig/auto_drain exists (default: on).
        return os.path.exists(self.DRAIN_FLAG) or not os.path.exists(
            os.path.expanduser("~/rig/no_auto_drain"))

    def drain_status(self):
        with self._drain_lock:
            st = dict(self._drain_status)
            st["wedged"] = {n: dict(v) for n, v in self._drain_wedged.items()}
        st["cancel_requested"] = self._drain_stop.is_set()
        return st

    def cancel_drain(self):
        """Ask the running drain to stop (contract C4).

        There was no way out of a drain at all: FIELD-RUN's own numbers put a
        full card at 10-15 minutes, RunManager.start refuses for the whole of
        it, and the only escape an operator had was `launchctl kickstart -k`,
        which SIGTERMs rigd and kills the drain thread mid-pull. The Event is
        checked between FILES, never between a file's verify and its card
        delete, so cancelling can never cost a card original."""
        with self._drain_lock:
            if not self._drain_status["active"]:
                return {"ok": False, "error": "no drain is running"}
            node = self._drain_status["node"]
        self._drain_stop.set()
        self.events.emit("warn", "drain",
                         "drain cancel requested on %s - it stops after the "
                         "file it is on; already-verified files stay pulled "
                         "and the rest of the card is untouched" % node,
                         node=node)
        return {"ok": True, "cancelling": node}

    # The wedge (HANDOFF §2.2, reproduced live 2026-08-23 on cam2): a body
    # whose transfer subsystem is stuck never publishes "card index ready", so
    # drain.py's _wait_index gives up after 90 s - twice, because it retries -
    # and the whole auto-drain after every Stop burns 3 minutes and achieves
    # nothing. Only a power cycle clears it, so re-attempting it automatically
    # is pure cost. The skip is released the moment the monitor SEES the node
    # reboot (piagent host_uptime_s reset -> NodeMonitor.rebooted_at moves), or
    # when the operator asks for a drain by hand.
    WEDGE_MARK = "card index not ready"

    def _drain_wedge_reason(self, node):
        """Why an AUTO drain should skip this node, or None."""
        w = self._drain_wedged.get(node)
        if not w:
            return None
        m = next((x for x in self.monitors if x.name_ == node), None)
        if m is not None and getattr(m, "rebooted_at", None) != w["rebooted_at"]:
            self._drain_wedged.pop(node, None)      # seen to reboot: clear it
            return None
        return ("%s's last drain wedged its transfer subsystem (%s) and it has "
                "not been seen to power-cycle since - skipping the auto-drain. "
                "Power-cycle the camera, then drain it by hand from the "
                "Card drain panel in Review"
                % (node, w["at_iso"]))

    def start_drain(self, nodes, keep=False, auto=False):
        """Claim the nodes and launch the drain worker. `auto` marks the
        after-a-run drain, which honours the wedge skip; a manual drain is the
        operator overriding it and clears the mark."""
        want = list(nodes)
        skipped = {}
        for n in want:
            m = next((x for x in self.monitors if x.name_ == n), None)
            if m is None:
                skipped[n] = "unknown node"
            elif not m.is_connected():
                skipped[n] = "not connected - its card was NOT drained"
            elif auto:
                why = self._drain_wedge_reason(n)
                if why:
                    skipped[n] = why
            if n not in skipped and not auto:
                self._drain_wedged.pop(n, None)
        nodes = [n for n in want if n not in skipped]

        err = None
        with self._drain_lock:
            if self._drain_status["active"]:
                err = ("a drain is already running on %s"
                       % self._drain_status["node"])
            elif not nodes:
                err = "no node to drain"
            else:
                # Claim the nodes for the drain HERE, under the RUN MANAGER's
                # OWN lock, before any run can observe them free. The previous
                # fix claimed them under rigd's _drain_lock, which
                # RunManager.start never takes: it checks self.draining under
                # run.py's self._lock, so the two guards sat on different
                # mutexes and the TOCTOU survived (audit 2026-08-23).
                # status() is deliberately NOT called here - it takes the same
                # non-reentrant lock and would deadlock.
                with self.runmgr._lock:      # noqa: SLF001 - the shared claim
                    if self.runmgr.active:
                        err = "a run is active - cannot drain"
                    elif getattr(self.runmgr, "draining", None):
                        err = "a drain already holds %s" % self.runmgr.draining
                    else:
                        # The claim covers the whole QUEUE, not just the node
                        # being drained right now: clearing it between nodes
                        # reopened the same window for every node after the
                        # first.
                        self.runmgr.draining = nodes[0]
            if err is None:
                self._drain_stop = threading.Event()
                self._drain_status = {"active": True, "node": nodes[0],
                                      "last": None, "queue": list(nodes),
                                      "skipped": skipped,
                                      "cancel_requested": False}
                for m in self.monitors:
                    if m.name_ in nodes:
                        m.suspend_control = True
                stopev = self._drain_stop
        # Every refusal path used to return a structured error that the
        # /api/run/stop branch threw away, and none of them emitted an event -
        # so an auto-drain that never happened was invisible everywhere (audit
        # 2026-08-23). Say it out loud, naming the nodes that were dropped.
        if skipped:
            self.events.emit("warn", "drain",
                             "card drain skipping %s"
                             % ", ".join("%s (%s)" % kv
                                         for kv in sorted(skipped.items())))
        if err is not None:
            self.events.emit("warn", "drain",
                             "card drain not started: %s" % err)
            return {"ok": False, "error": err, "skipped": skipped}
        threading.Thread(target=self._drain_worker,
                         args=(nodes, keep, stopev), daemon=True).start()
        return {"ok": True, "draining": nodes, "skipped": skipped}

    def _drain_worker(self, nodes, keep, stopev):
      try:
        for node in nodes:
            host = next((m.host for m in self.monitors if m.name_ == node), None)
            if host is None:
                continue
            if stopev.is_set():
                self.events.emit("warn", "drain",
                                 "drain cancelled before %s - its card was "
                                 "NOT drained" % node, node=node)
                continue
            # draining + suspend_control were claimed in start_drain under
            # the lock; just point them at the current node. NOT cleared
            # between nodes - see the claim comment there. Still under the run
            # manager's lock: every write to `draining` goes through the same
            # mutex RunManager.start reads it under, so there is exactly one
            # rule to check rather than "this one is safe because...".
            # Sequential with _drain_lock below, never nested inside it -
            # start_drain takes them the other way round.
            with self.runmgr._lock:          # noqa: SLF001
                self.runmgr.draining = node
            with self._drain_lock:
                self._drain_status["node"] = node
            self.events.emit("info", "drain", "card drain started on %s" % node,
                             node=node)
            try:
                rep = draindrv.Drainer(
                    node, host, dest=self.DRAIN_DEST,
                    log=lambda m: self.events.emit("info", "drain", m)).run(
                        keep_card=keep, stop=stopev)
                sev = "warn" if rep.get("errors") else "info"
                self.events.emit(
                    sev, "drain",
                    "%s drain %s: %d pulled (%.1f GB), %d deleted, %d errors"
                    % (node, "cancelled" if rep.get("cancelled") else "done",
                       rep["pulled"], rep["bytes"] / 1e9, rep["deleted"],
                       len(rep["errors"])), node=node,
                    cancelled=bool(rep.get("cancelled")),
                    errors=rep["errors"][:5])
                self._note_wedge(node, rep)
                with self._drain_lock:
                    self._drain_status["last"] = {"node": node, "at": time.time(),
                                                  **{k: rep[k] for k in
                                                     ("pulled", "bytes", "deleted",
                                                      "verified")},
                                                  "cancelled": bool(rep.get("cancelled")),
                                                  "errors": len(rep["errors"])}
                # hand the pulled RAWs to ingest (best-effort; never blocks).
                # Per-node staging dir - the drain writes ~/rig-raw/<node>/.
                try:
                    import ingest
                    self._note_ingest(node, ingest.ingest(
                        os.path.join(self.DRAIN_DEST, node),
                        log=lambda *a: None))
                except Exception as e:  # noqa: BLE001
                    self.events.emit("warn", "drain",
                                     "ingest after drain failed: %s" % e)
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "drain",
                                 "drain on %s failed: %s" % (node, e), node=node)
            finally:
                for m in self.monitors:
                    if m.name_ == node:
                        m.suspend_control = False
      finally:
        # Whatever happened (an exception before the per-node try, a host
        # lookup miss), release the drain claim so a leaked active flag can
        # never block every future drain and run (audit 2026-08-23). Under the
        # run manager's lock, for the same reason the claim is.
        with self.runmgr._lock:              # noqa: SLF001
            self.runmgr.draining = None
        for m in self.monitors:
            m.suspend_control = False
        with self._drain_lock:
            self._drain_status["active"] = False
            self._drain_status["node"] = None
            self._drain_status["queue"] = []

    def _note_ingest(self, node, rep_i):
        """Say what the post-drain ingest actually matched.

        The automatic path threw away both the log and the return value, so a
        drain that emptied the card and then matched NOTHING - a truncated
        frame index, a body clock that has moved past the offset the mode
        found, missing flight rows - left only "cam1 drain done: 169 pulled,
        169 deleted, 0 errors" in the journal. The card originals are already
        gone at that point and the RAWs sit in the staging dir unnamed and
        unpaired, with nothing anywhere saying so; the operator's manual
        ingest (FIELD-RUN.md) is hours later. ingest returns `totals` for
        exactly this caller (audit 2026-08-24)."""
        t = (rep_i or {}).get("totals") or {}
        matched = t.get("matched", 0)
        # A card with files that matched nothing, an ambiguous attribution, or
        # a content conflict are all "look at this now"; the ordinary case is
        # one info line saying how many frames were placed.
        bad = bool(t.get("ambiguous") or t.get("conflicts")
                   or (t.get("cards") and not matched))
        self.events.emit(
            "warn" if bad else "info", "drain",
            "%s post-drain ingest: %d matched, %d unmatched, %d RAW placed, "
            "%d leftover of %d card files%s%s"
            % (node, matched, t.get("unmatched", 0), t.get("raw", 0),
               t.get("leftover", 0), t.get("cards", 0),
               ", %d CONFLICTS" % t["conflicts"] if t.get("conflicts") else "",
               (", AMBIGUOUS: %s" % "; ".join(t["ambiguous"]))
               if t.get("ambiguous") else ""),
            node=node, ingest=t)
        # ...and beside "deleted from card" in /api/drain and the UI's table.
        with self._drain_lock:
            last = self._drain_status.get("last")
            if isinstance(last, dict) and last.get("node") == node:
                last["ingest"] = t

    def _note_wedge(self, node, rep):
        """Remember (or clear) the transfer-subsystem wedge for this node."""
        wedged = any(self.WEDGE_MARK in str(e) for e in (rep.get("errors") or []))
        m = next((x for x in self.monitors if x.name_ == node), None)
        with self._drain_lock:      # drain_status() iterates this dict
            if not wedged:
                self._drain_wedged.pop(node, None)
                return
            self._drain_wedged[node] = {
                "at": time.time(),
                "at_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "rebooted_at": getattr(m, "rebooted_at", None) if m else None}
        self.events.emit(
            "warn", "drain",
            "%s's card index never came ready - its transfer subsystem is "
            "wedged. Auto-drain will SKIP this node until it is seen to "
            "power-cycle (or you drain it by hand); its card keeps every "
            "frame in the meantime" % node, node=node)

    def _startup_calibrate(self):
        """Measure per-camera trigger latency once the fleet is up.

        Done at program start as well as run start so the first transect of a
        session is already aligned rather than paying for the calibration in
        survey frames."""
        deadline = time.time() + 90
        while time.time() < deadline:
            live = [m for m in self.monitors if m.is_connected()]
            if len(live) >= 1:
                time.sleep(3)          # let the property tables settle
                # A run started during the boot window does its own
                # calibration; firing this one into it would hold FOCUS
                # (AE-lock) and race the run's frame naming.
                if self.runmgr.status().get("active"):
                    self.events.emit("info", "calibrate",
                                     "startup calibration skipped: a run is "
                                     "already active and calibrates itself")
                    return
                try:
                    self.runmgr.calibrate_trigger()
                except Exception as e:  # noqa: BLE001
                    self.events.emit("warn", "calibrate",
                                     "startup calibration failed: %s" % e)
                return
            time.sleep(2)
        self.events.emit("info", "calibrate",
                         "no camera connected within 90s; trigger latency "
                         "will be measured at the next run start")

    def _anomaly_loop(self):
        while True:
            time.sleep(2.5)
            try:
                self.anomalies.scan()
            except Exception as e:  # noqa: BLE001
                self.events.emit("warn", "anomaly", "scan error: %s" % e)

    def _reconcile_loop(self):
        while not self._stop.wait(3.0):
            try:
                self.settings.reconcile_all(force=False)
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "settings",
                                 "reconcile loop error: %s" % e)

    def _nav_time_loop(self):
        while not self._stop.wait(5.0):
            if not self.nav:
                continue
            # Keep the static-fix latch tied to the run that owns it: adopt
            # the active run's id (so a start that answered its client but
            # never bound still ends up owned), and release the latch when no
            # run is active - the run ended by a path other than POST
            # /api/run/stop (an internal stop, a rigd-side abort), and a latch
            # left behind would keep that transect's stand-in policy in force
            # over the idle fleet.
            #
            # This tick used to clear UNCONDITIONALLY on "not active", which is
            # also what RunManager.start reports for the whole of its multi-
            # second live probe of every monitor: with one node unreachable the
            # tick stripped the decision of the run that was starting, and a
            # bus drop later in that run then fabricated positions from the
            # static fix (audit 2026-08-24, review blocker). end_run_nav now
            # refuses while a start is in flight.
            try:
                st = self.runmgr.status()
                if st.get("active"):
                    self.bind_run_nav(st.get("run_id"))
                else:
                    self.end_run_nav()
            except Exception:  # noqa: BLE001
                pass
            try:
                ta = getattr(self.nav, "time_authority", None)
                # gps_epoch() always returns a value (Jetson time when there is
                # no fix), so it cannot tell us whether GPS is really flowing —
                # source() does. Only claim GPS time when a fresh fix is held.
                if ta and ta.source() == "gps":
                    self.timesync.feed_gps(ta.gps_epoch())
                else:
                    self.timesync.clear_gps()
            except Exception:  # noqa: BLE001
                pass

    # ---- diag snapshot ----------------------------------------------------
    def diag(self):
        return {
            "ts": time.time(),
            "rigd": {"port": PORT, "nav": bool(self.nav)},
            # Full gateway health, not just present/absent: which port it
            # resolved, whether bytes are actually arriving, and the last error.
            "nav": (self.nav.health() if self.nav else {"present": False}),
            "time": {"source": self.timesync.now()[1],
                     "gps_offset_s": round(self.timesync.gps_offset, 3),
                     "exif_offset": self.timesync.exif_offset},
            "nodes": [m.snapshot() for m in self.monitors],
            "desired": self.settings.get(),
            "preview": self.settings.preview_state(),
            "run": self.runmgr.status(),
            # The volume the transects are written to — the one nothing
            # measured. `disk_free_mb` on a node is that Pi's PC-save spool.
            "storage": {"runs_dir": rigcore.RUNS_DIR,
                        "jetson_free_mb": free_mb(rigcore.RUNS_DIR),
                        "low_threshold_mb": RUNS_DISK_LOW_MB},
            "anomaly_counts": self.events.recent_counts(),
            "anomalies": self.anomalies.scan(),
        }

    def fleet(self):
        return {"nodes": [self._node_view(m) for m in self.monitors],
                "run": self.runmgr.status(),
                "preview": self.settings.preview_state(),
                "jetson_free_mb": free_mb(rigcore.RUNS_DIR),
                "jetson_disk_low_mb": RUNS_DISK_LOW_MB,
                "time_source": self.timesync.now()[1]}

    def _node_view(self, m):
        snap = m.snapshot()
        status = snap.get("status") or {}
        h = snap.get("health") or {}
        run = self.runmgr.status()
        stats = (run.get("stats") or {}).get(m.name_, {}) if run.get("active") \
            else {}
        clock = m.clock_offset_info()
        if clock.get("offset_s") is not None:
            clock["offset_s"] = round(clock["offset_s"], 4)
        # ilxctl already formats display labels (iso="ISO 400", shutter="1/200",
        # aperture="F8"); use them directly and carry raw numerics alongside.
        return {
            "name": m.name_, "cam_num": m.node["cam_num"], "host": m.host,
            "state": snap["state"], "age_s": snap.get("age_s"),
            "connected": status.get("connected", False),
            "model": status.get("model"), "id": status.get("id"),
            "battery": status.get("battery"),
            "iso": status.get("iso"),
            "shutter": status.get("shutter"),
            "aperture": status.get("aperture"),
            "fnum": status.get("aperture"),
            "iso_raw": status.get("isoValue"),
            "shutter_raw": status.get("shutterValue"),
            "aperture_raw": status.get("apertureValue"),
            "store_dest": status.get("storeDest"),
            "store_dest_label": status.get("storeDestLabel"),
            # Body-menu-only on these bodies (enableFlag=DisplayOnly): shown so
            # the operator can SEE them, never offered as something the UI can
            # set. `writable` is carried so the UI can say why.
            "pcsave": status.get("pcsave"),
            "writable": status.get("writable") or {},
            # Format properties. ilxctl has no readback for these on this body
            # (they appear in /api/status only on the fake node), so the UI
            # shows the desired value marked "no readback" rather than implying
            # it has confirmed anything.
            "filetype": status.get("filetypeValue"),
            "imagesize": status.get("imagesizeValue"),
            "transsize": status.get("transsizeValue"),
            # Health fields ilxctl reports and nothing consumed until now.
            # slotWriting == WRITING persistently is a stuck card write, which
            # takes the whole body down while slotStatus still reads OK.
            "slot_status": status.get("slotStatus"),
            "slot_writing": status.get("slotWriting"),
            "slot_writing_label": status.get("slotWritingLabel"),
            "overheating": status.get("overheating"),
            "overheating_label": status.get("overheatingLabel"),
            "live_view": status.get("liveViewStatus"),
            "live_view_label": status.get("liveViewLabel"),
            "priority_key_label": status.get("priorityKeyLabel"),
            # Lens position, READ ONLY and PER CAMERA. These are deliberately
            # not converged (docs/future-tests.md §1: encoder parity between the
            # two bodies is unmeasured), but a body that came back from a
            # power cycle at a different focus than its partner invalidates the
            # stereo solution, so the numbers have to be visible even though
            # nothing may push one body's value to the other.
            "focus_mode": status.get("focusMode"),
            "focus_mode_label": status.get("focusModeLabel"),
            "focus_pos": status.get("focusPosCur", status.get("focusPos")),
            "zoom_pos": status.get("zoomPosCur", status.get("zoomPos")),
            "zoom_setting_label": status.get("zoomSettingLabel"),
            "convergence": snap.get("convergence"),
            # This node's clock vs the host's, from the /health poll. Two
            # numbers on purpose: clock_offset_ms is the LAST RAW sample (what
            # the most recent poll saw, network noise and all) and
            # clock_offset_s is the FILTERED figure every decision is made on
            # (contract C1: RTT-gated median of the last up-to-8 samples). They
            # differ by tens of ms on a busy link, and an operator comparing
            # the strip against an anomaly needs to see both rather than
            # wonder which one lied.
            "clock_offset_ms": (round(m.clock["offset_s"] * 1e3, 2)
                                if getattr(m, "clock", None) else None),
            "clock_rtt_ms": (round(m.clock["rtt_ms"], 2)
                             if getattr(m, "clock", None) else None),
            "clock_offset_s": clock["offset_s"],
            "clock_offset_info": clock,
            "gpio": h.get("gpio"), "imu": h.get("imu"),
            "disk_free_mb": h.get("disk_free_mb"),
            "cam_frames": h.get("cam_frames"),
            "stats": stats,
        }

    def fanout(self, api_path, body):
        """Forward a lens call to every connected camera (or one, if body has
        a 'node'). Lens position/speed stay PER CAMERA and out of `desired`
        (docs/future-tests.md §1), so this is the only path they take.

        What failed (fuzz 2026-08-23): a camera that was not connected was
        skipped silently and the answer was still {"ok":true,"results":{}}, so
        the UI's lens queue saw success, cleared its note and - for a STOP -
        stopped retrying, against a lens that was still driving when the node
        came back. Out-of-range values had the same shape: focus/position
        -1 or 99999999 answered ok:true and nothing moved. Now a skipped node
        is named, an addressed-but-skipped node is ok:false, and values are
        clamped into the body's own published range with the clamp reported."""
        target = body.get("node")
        if target is not None:
            if not isinstance(target, str) or not self._known(target):
                raise BadRequest("unknown node %r (known: %s)"
                                 % (target, ", ".join(m.name_ for m in
                                                      self.monitors)))
        payload = {k: v for k, v in body.items() if k != "node"}
        field, range_key = LENS_FIELD.get(api_path, (None, None))
        # Validate only what was SENT: ilxctl supplies its own default for an
        # absent field, and the defect being fixed is bad values reaching the
        # body, not missing ones.
        if field and field in payload:
            payload[field] = want_int(payload[field], '"%s"' % field)
        results, skipped, clamped = {}, {}, {}
        for m in self.monitors:
            if target and m.name_ != target:
                continue
            if not m.is_connected():
                skipped[m.name_] = "node not connected"
                continue
            p = dict(payload)
            if field and field in p and range_key:
                rng = (m.snapshot().get("status") or {}).get(range_key)
                v, was = _clamp_to_range(p[field], rng)
                if was:
                    clamped[m.name_] = {"from": p[field], "to": v,
                                        "range": rng}
                    p[field] = v
            results[m.name_] = http_json("http://%s:8080%s" % (m.host, api_path),
                                         p, timeout=10)
        if not results:
            return {"ok": False,
                    "error": ("%s: node not connected" % target) if target
                             else "no connected camera",
                    "results": {}, "skipped": skipped}
        ok = all(not (isinstance(r, dict) and r.get("ok") is False)
                 for r in results.values())
        out = {"ok": ok, "results": results}
        if skipped:
            out["skipped"] = skipped
        if clamped:
            out["clamped"] = clamped
        return out

    def _known(self, name):
        return any(m.name_ == name for m in self.monitors)

    # A failed IMU probe is remembered for this long. The IMU hangs off cam1;
    # when cam1 loses PoE its IP stops answering ARP and every http_json to it
    # blocks the full 3 s timeout. rig_ui polls /api/imu/window five times a
    # second while the IMU tab is open, so rigd accumulated ~15 handler threads
    # each parked on a dead socket - and because a browser allows six
    # connections per origin, the queued IMU requests also delayed /api/fleet,
    # /api/anomalies and /api/nav on the same tab, exactly when the operator
    # needed to see node_offline (audit 2026-08-23).
    IMU_DEAD_TTL_S = 10.0
    # How long stop() waits for an active run to finalise before exiting
    # anyway. Sized under launchd's 20 s default SIGKILL budget with room for
    # serve_forever's 0.5 s poll and the monitor shutdown behind it.
    STOP_DEADLINE_S = 12.0

    def _imu_host(self):
        """Host currently serving IMU samples.

        The IMU is the rig's master orientation source for every camera, so the
        node it hangs off is discovered rather than hardcoded: it can be moved
        to another Pi without a code change. The last node that answered is
        cached; a node the MONITOR already knows is OFFLINE is never probed at
        all, and a probe that finds nothing is remembered for IMU_DEAD_TTL_S so
        the next hundred polls answer present:false for free. The probe itself
        is serialised, so concurrent callers share one round trip instead of
        each opening their own."""
        now = time.monotonic()
        if now < getattr(self, "_imu_dead_until", 0.0):
            return None, None
        with self._imu_lock:
            # Re-check: a caller that queued behind the prober must not repeat
            # the work it just did.
            now = time.monotonic()
            if now < self._imu_dead_until:
                return None, None
            cached = getattr(self, "_imu_node", None)
            order = ([m for m in self.monitors if m.name_ == cached] +
                     [m for m in self.monitors if m.name_ != cached])
            probed = 0
            for m in order:
                if m.snapshot()["state"] == NodeMonitor.OFFLINE:
                    continue          # no HTTP to a node we know is not there
                probed += 1
                s = http_json("http://%s:8081/imu/latest" % m.host, timeout=3)
                if s and s.get("epoch") is not None:
                    self._imu_node = m.name_
                    self._imu_dead_until = 0.0
                    return m.host, s
            self._imu_node = None
            self._imu_dead_until = time.monotonic() + self.IMU_DEAD_TTL_S
            if probed:
                self.events.emit("info", "imu",
                                 "no node is serving IMU samples; not "
                                 "re-probing for %ds" % self.IMU_DEAD_TTL_S)
            return None, None

    def imu(self):
        host, s = self._imu_host()
        if not host:
            return {"present": False}
        s["node"] = self._imu_node
        return s

    def imu_window(self, t0, t1):
        """A batch of samples, so the UI can render at display rate without
        one HTTP round trip per frame. The driver runs at ~200 Hz; polling it
        60 times a second through two hops would cost far more than it shows."""
        host, _ = self._imu_host()
        if not host:
            return {"present": False, "samples": []}
        s = http_json("http://%s:8081/imu/window?t0=%.3f&t1=%.3f" % (host, t0, t1),
                      timeout=4)
        # piagent wraps the batch: {"present":bool,"samples":[...]}. Tolerate a
        # bare list too, so an older node still works during a rolling deploy.
        if isinstance(s, list):
            samples = s
        elif isinstance(s, dict):
            samples = s.get("samples") or []
        else:
            samples = []
        return {"present": True, "node": self._imu_node, "samples": samples}

    def nav_snapshot(self):
        if not self.nav:
            return {"present": False, "fix_kind": "none", "valid": False}
        try:
            s = self.nav.snapshot() or {}
            if s.get("lat") is not None and s.get("lon") is not None:
                import nav as navmod
                x, y, z = navmod.latlon_to_utm(s["lat"], s["lon"])
                s.update(xutm=x, yutm=y, utm_zone=z)
            # fix_kind is the field the UI colours the header pill from, and it
            # exists because "lat is not None" was doing that job: an armed
            # STATIC position wore the same green "nav fix" as a live GPS fix
            # and was plotted as a track (audit 2026-08-23). StaticFixNav sets
            # it; derive it here too so a bare NavReader (no static wrapper)
            # and a nav-less rigd still answer the contract.
            if "fix_kind" not in s:
                s["fix_kind"] = ("live" if s.get("valid") else
                                 "static" if (s.get("lat") is not None
                                              and s.get("static_fix"))
                                 else "none")
            if s["fix_kind"] != "live":
                s["valid"] = False       # a static row is never a fix
            s["present"] = True
            return s
        except Exception as e:  # noqa: BLE001
            return {"present": False, "fix_kind": "none", "valid": False,
                    "error": str(e)}

    # ---- static-fix arming (nav finding #1) -------------------------------
    def _recording_run_id(self):
        """The run_id of the transect recording right now, or None.

        Deliberately a plain attribute read rather than runmgr.status(): this
        is called from StaticFixNav.armed() on the pull workers' threads, once
        per frame, and status() takes the run manager's non-reentrant lock."""
        return (getattr(getattr(self, "runmgr", None), "active", None)
                or {}).get("run_id")

    def begin_run_nav(self):
        """Latch the static-fix fallback for the run about to start.

        Called from the /api/run/start handler because the decision is "was
        the gateway alive when the operator pressed record", which only makes
        sense at that instant: armed means the whole line carries the armed
        position, unarmed means a mid-run bus drop writes EMPTY lat/long and
        raises nav_no_fix rather than papering the gap over.

        Returns the ownership TOKEN to hand back to bind_run_nav (the start
        succeeded) or release_run_nav (it was refused). A None token means the
        latch was already owned - a duplicate Start while a run is recording -
        so this request took nothing and must release nothing."""
        if not hasattr(self.nav, "begin_run"):
            return None
        online = bool(getattr(self.nav, "gateway_online", False))
        armed, token = self.nav.begin_run(online)
        if armed and token is not None:
            self.events.emit(
                "warn", "nav",
                "no live NMEA at run start - the armed STATIC fix (%s) will "
                "stand in for every frame of this run; positions are one "
                "constant, not a track" % self.nav.static_label())
        return token

    def bind_run_nav(self, run_id, token=None):
        """Give the latch the identity of the run that now owns it."""
        if run_id and hasattr(self.nav, "bind_run"):
            return bool(self.nav.bind_run(run_id, token))
        return False

    def release_run_nav(self, token):
        """Undo a begin_run_nav whose start was refused. A None token is a
        no-op ON PURPOSE: this request never took the latch, so the run that
        did still owns its decision."""
        if token is not None and hasattr(self.nav, "end_run"):
            return bool(self.nav.end_run(token=token))
        return False

    def end_run_nav(self, run_id=None):
        if hasattr(self.nav, "end_run"):
            return bool(self.nav.end_run(run_id=run_id))
        return False

    def stop(self):
        """Shut down cleanly, finalising an active run first.

        A rigd that goes away mid-transect used to leave run.json with
        final:false and no per-worker stats, and - the part that costs imagery -
        left frames that had reached a node but not yet been pulled to be
        baselined as "old" by the NEXT run's PullWorker, so they were never
        pulled, renamed, or given a flight_log row. runmgr.stop() drains the
        pull workers and writes the manifest, so the ordinary exit path stops
        creating that seam. Idempotent: SIGTERM and the KeyboardInterrupt path
        can both land here.

        BOUNDED (audit 2026-08-23). launchd SIGKILLs 20 s after SIGTERM by
        default, and RunManager.stop against a wedged node can spend longer
        than that on node I/O alone: cap_thread.join(6) + a 6 s grace loop that
        never clears while pulled < fired + w.join(8) behind a 30 s http_bytes
        pull. rigd was killed before run.json was marked final, so the next
        start had to rebuild the manifest as interrupted. The finalisation now
        runs on its own thread with a deadline, so the process always reaches
        "rigd down" inside the budget; whatever the join did not cover is
        repaired by _recover_runs at the next start, which is strictly better
        than being killed mid-write. deploy/rigd.launchd.plist also raises
        ExitTimeOut to 60 so the normal (slow but healthy) stop is never cut
        short in the first place."""
        if self._stopped.is_set():
            return
        self._stopped.set()
        try:
            if self.runmgr.status().get("active"):
                self.events.emit("warn", "lifecycle",
                                 "rigd is stopping with a run active - "
                                 "finalising it before exit")
                done = threading.Event()

                def _fin():
                    try:
                        self.runmgr.stop()
                    except Exception as e:  # noqa: BLE001
                        self.events.emit("error", "lifecycle",
                                         "could not finalise the active run: "
                                         "%s" % e)
                    finally:
                        done.set()

                t = threading.Thread(target=_fin, name="rigd-finalise",
                                     daemon=True)
                t.start()
                if not done.wait(self.STOP_DEADLINE_S):
                    self.events.emit(
                        "error", "lifecycle",
                        "run finalisation did not complete within %ds (a node "
                        "is not answering) - exiting anyway; the next rigd "
                        "start rebuilds this run's manifest from the "
                        "flight_logs on disk" % self.STOP_DEADLINE_S)
        except Exception as e:  # noqa: BLE001
            self.events.emit("error", "lifecycle",
                             "could not finalise the active run: %s" % e)
        self._stop.set()
        for m in self.monitors:
            m.stop()
        self.events.emit("info", "lifecycle", "rigd down")


RIG = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _bytes(self, data, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _read_body(self):
        """The POST body as a dict, or raise BadRequest.

        "Cannot parse" and "no body" used to be the same answer, {}, so
        POST /api/run/start with a truncated JSON body recorded a REAL transect
        on the built-in defaults instead of refusing it (verified live). A body
        that is valid JSON but not an OBJECT ("null", "[]", "3") is refused for
        the same reason: every handler below immediately does b.get(...), which
        on a list is an AttributeError and a 500."""
        try:
            n = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            raise BadRequest("bad Content-Length header") from None
        if n <= 0:
            return {}
        if n > 1 << 20:
            # Not read, so the connection cannot be reused - say so rather
            # than leaving a megabyte of body framed as the next request.
            self.close_connection = True
            raise BadRequest("body too large (%d bytes; limit 1 MiB)" % n)
        try:
            raw = self.rfile.read(n)
        except OSError as e:
            raise BadRequest("could not read the request body: %s" % e) from None
        # parse_constant refuses Infinity/-Infinity/NaN, which json.loads
        # otherwise accepts silently: they are not JSON, no client of this
        # daemon sends them, and every coercer downstream would have to guard
        # them one at a time (want_int did not - audit 2026-08-24).
        def _no_const(name):
            raise BadRequest("JSON body contains %s, which is not a number"
                             % name)

        try:
            b = json.loads(raw.decode("utf-8") or "{}",
                           parse_constant=_no_const)
        except (ValueError, UnicodeDecodeError) as e:
            if isinstance(e, BadRequest):
                raise
            raise BadRequest("malformed JSON body: %s" % e) from None
        # "null" is refused with the rest: rig_ui's api() sends a GET when it
        # has no body and JSON.stringify of an object otherwise, so nothing
        # legitimate ever POSTs a bare null - it is only ever a client bug or
        # a fuzzer, and treating it as {} is what let a defaulted run start.
        if not isinstance(b, dict):
            raise BadRequest("JSON body must be an object, got %s"
                             % ("null" if b is None else type(b).__name__))
        return b

    def _mon(self, name):
        return next((m for m in RIG.monitors if m.name_ == name), None)

    def _node(self, name, what="node"):
        """A monitor for a node name the fleet actually has, or BadRequest."""
        m = self._mon(name) if isinstance(name, str) else None
        if m is None:
            raise BadRequest("%s: unknown node %r (known: %s)"
                             % (what, name,
                                ", ".join(x.name_ for x in RIG.monitors)))
        return m

    def do_GET(self):
        u = urlparse(self.path)
        p, q = u.path, parse_qs(u.query)
        try:
            if p == "/" or p == "/index.html":
                try:
                    with open(UI_PATH, "rb") as fh:
                        self._bytes(fh.read(), "text/html; charset=utf-8")
                except OSError:
                    self._bytes(b"rig_ui.html missing", "text/plain", 500)
            elif p == "/api/fleet":
                self._json(RIG.fleet())
            elif p == "/api/diag":
                self._json(RIG.diag())
            elif p == "/api/anomalies":
                self._json({"anomalies": RIG.anomalies.scan()})
            elif p == "/api/events":
                # ?since=abc used to reach int() and come back as a 500 with
                # "invalid literal for int()" in the body, which reads as a
                # rigd crash to anyone watching the journal.
                since = qint(q, "since", 0, lo=0)
                sev = (q.get("sev") or ["debug"])[0]
                self._json(RIG.events.since(since, sev))
            elif p == "/api/settings":
                self._json(RIG.settings.get())
            elif p == "/api/settings/preview":
                self._json(RIG.settings.preview_state())
            elif p == "/api/run":
                self._json(RIG.runmgr.status())
            elif p == "/api/runs":
                # A junk limit is the client's problem, not a 500: clamp it.
                try:
                    lim = int((q.get("limit") or ["50"])[0] or 50)
                except ValueError:
                    lim = 50
                self._json(RIG.runs.list_runs(limit=lim))
            elif p == "/api/run/detail":
                self._runs(lambda: RIG.runs.detail((q.get("id") or [""])[0]))
            elif p == "/api/run/shots":
                self._runs(lambda: RIG.runs.shots(
                    (q.get("id") or [""])[0],
                    (q.get("offset") or ["0"])[0],
                    (q.get("limit") or ["200"])[0]))
            elif p == "/api/run/frame":
                self._run_frame((q.get("id") or [""])[0],
                                (q.get("cam") or [""])[0],
                                (q.get("name") or [""])[0])
            elif p == "/api/run/flight_log":
                self._run_flight_log((q.get("id") or [""])[0],
                                     (q.get("cam") or [""])[0])
            elif p == "/api/imu":
                self._json(RIG.imu())
            elif p == "/api/imu/window":
                now = time.time()
                t0 = qfloat(q, "t0", 0.0)
                # A fresh client sends t0=0; give it the last second rather than
                # every sample the ring holds. A window that reaches further
                # back than the ring is capped here rather than asking piagent
                # for an hour of samples it would have to serialise.
                if t0 <= 0:
                    t0 = now - 1.0
                elif now - t0 > 600.0:
                    t0 = now - 600.0
                self._json(RIG.imu_window(t0, now))
            elif p == "/api/nav":
                self._json(RIG.nav_snapshot())
            elif p == "/api/drain":
                self._json(RIG.drain_status())
            elif p == "/api/nav/all":
                # The bus-sniffing table: every PGN seen on the N2K bus with
                # age/rate/source and raw payload, decoded or not.
                if RIG.nav and hasattr(RIG.nav, "bus_table"):
                    self._json(RIG.nav.bus_table())
                else:
                    self._json({"present": False, "pgns": []})
            elif p == "/nmea":
                try:
                    with open(os.path.join(HERE, "nmea_dash.html"), "rb") as fh:
                        self._bytes(fh.read(), "text/html; charset=utf-8")
                except OSError:
                    self._bytes(b"nmea_dash.html missing", "text/plain", 500)
            elif p == "/api/strobe":
                self._json({"ok": True, "strobe": RIG.runmgr.get_strobe()})
            elif p == "/api/status":
                m = self._node((q.get("node") or [""])[0], "?node")
                self._json(http_json("http://%s:8080/api/status" % m.host,
                                     timeout=6))
            elif p == "/api/shots":
                m = self._node((q.get("node") or [""])[0], "?node")
                self._json(m.shots())
            elif p == "/api/frame":
                self._proxy_frame((q.get("node") or [""])[0],
                                  (q.get("name") or [""])[0])
            elif p == "/api/liveview":
                self._proxy_liveview((q.get("node") or [""])[0])
            else:
                self._json({"ok": False, "error": "not found"}, 404)
        except BadRequest as e:
            # A client-side mistake is a 400 naming the field, never a 500
            # carrying a Python exception string, and it is not journalled as
            # a rigd error - a fuzzer must not be able to fill the event log.
            self._json({"ok": False, "error": str(e)}, 400)
        except Exception as e:  # noqa: BLE001
            RIG.events.emit("error", "http", "GET %s: %s" % (p, e))
            self._json({"ok": False, "error": str(e)}, 500)

    # ---- transect browser (read-only) ------------------------------------
    # Every one of these takes a run id / camera / filename straight from the
    # browser. RunBrowser validates each part against a strict character class
    # AND checks the resolved realpath is still inside the runs directory, so
    # neither "../.." nor a symlink planted in a run folder can read outside it.
    # RunsError is the one exception type they raise, and it is answered with a
    # 400/404 rather than the 500-with-a-stack-trace an unexpected error gets.
    def _runs(self, fn):
        try:
            self._json(fn())
        except RunsError as e:
            self._json({"ok": False, "error": str(e)},
                       404 if "no such" in str(e) else 400)

    def _run_frame(self, rid, cam, name):
        try:
            path = RIG.runs.frame_path(rid, cam, name)
        except RunsError as e:
            self._json({"ok": False, "error": str(e)},
                       404 if "no such" in str(e) else 400)
            return
        # A transect frame never changes once written: let the browser keep
        # it. Stepping back to a pair used to re-download and re-decode every
        # byte (14 MB at transsize=Original) because _bytes sends no-store.
        try:
            st = os.stat(path)
        except OSError as e:
            self._json({"ok": False, "error": str(e)}, 404)
            return
        etag = '"%s-%d-%d"' % (name, st.st_size, int(st.st_mtime))
        if self.headers.get("If-None-Match") == etag:
            self.send_response(304)
            self.send_header("ETag", etag)
            self.send_header("Cache-Control", "private, max-age=3600")
            self.end_headers()
            return
        try:
            with open(path, "rb") as fh:
                data = fh.read()
        except OSError as e:
            self._json({"ok": False, "error": str(e)}, 404)
            return
        ext = os.path.splitext(name)[1].lower()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg" if ext in (".jpg", ".jpeg")
                         else "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("ETag", etag)
        self.send_header("Cache-Control", "private, max-age=3600")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _run_flight_log(self, rid, cam):
        try:
            data, truncated = RIG.runs.flight_log(rid, cam)
        except RunsError as e:
            self._json({"ok": False, "error": str(e)},
                       404 if "no such" in str(e) or "no flight_log" in str(e)
                       else 400)
            return
        # text/csv, not a download: the operator reads it in the browser, and
        # nothing here should be able to hand the client an executable name.
        self._bytes(data, "text/csv; charset=utf-8")
        if truncated:
            RIG.events.emit("warn", "runs",
                            "flight_log for %s/%s exceeded the serve cap and "
                            "was truncated in transit - read it from disk"
                            % (rid, cam))

    def _proxy_frame(self, node, name):
        m = self._mon(node)
        if not m or not name:
            self._json({"ok": False, "error": "bad node/name"}, 400); return
        if (m.snapshot().get("status") or {}).get("ilx_down"):
            self._json({"ok": False, "error": "ilxctl not answering"}, 503)
            return
        # ilxctl already validates the name; we forward as-is (it 404s bad ones)
        data, err = http_bytes("http://%s:8080/shot/%s" % (m.host, name),
                               timeout=30)
        if err or not data:
            self._json({"ok": False, "error": err or "no frame"}, 404); return
        self._bytes(data, "image/jpeg")

    def _proxy_liveview(self, node):
        m = self._mon(node)
        if not m:
            self._json({"ok": False, "error": "bad node"}, 400); return
        if not m.is_connected():
            # Never forward to a node whose camera is not live: against a
            # wedged ilxctl every request strands an HTTP worker behind the
            # stuck SDK mutex and the 8 s wait freezes the UI's picture.
            self._json({"ok": False, "error": "camera not connected"}, 503)
            return
        active = bool(RIG.runmgr.status().get("active"))
        data, err, age_ms, policy = LIVETAP.get(m, active)
        if not data:
            self._json({"ok": False, "error": err or "no liveview"}, 503); return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Frame-Age-ms", "%.0f" % (age_ms or 0))
        self.send_header("X-Live-Policy", policy)
        if err:
            self.send_header("X-Live-Stale", err[:80])
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        u = urlparse(self.path)
        p = u.path
        try:
            # INSIDE the try, so a malformed body is a 400 with a message
            # rather than an exception on the way to the dispatch (D1).
            b = self._read_body()
            if p == "/api/settings":
                rep = RIG.settings.update(b)
                # A rejected value is returned, not swallowed: an out-of-range
                # desired value looks exactly like a camera fault from the
                # operator's side (red badge, anomaly pointing at the body),
                # and the only way out was to guess which field was bad.
                self._json({"ok": bool(rep["applied"]) or not rep["rejected"],
                            "applied": rep["applied"],
                            "rejected": rep["rejected"]})
            elif p == "/api/settings/preview":
                # Stage on the primary only. `desired` is untouched and that
                # node is pinned so the 3 s reconcile cannot yank the preview
                # back before the operator has looked at it.
                node = b.get("node")
                if node is not None:
                    node = self._node(node, '"node"').name_
                self._json(RIG.settings.preview(b, node=node))
            elif p == "/api/settings/commit":
                self._json(RIG.settings.commit())
            elif p in ("/api/settings/discard", "/api/settings/revert"):
                self._json(RIG.settings.discard())
            elif p == "/api/settings/auto":
                # bool("maybe") is True: {"on":"maybe"} armed the auto-exposure
                # servo on the live rig, which then drives the fleet's exposure
                # by itself for the rest of the session. A truthy string is not
                # consent - require a real bool.
                on = want_bool(b.get("on"), '"on"')
                RIG.settings.set_auto(on)
                self._json({"ok": True, "auto": on})
            elif p == "/api/ev":
                cur = RIG.settings.bump_ev(
                    want_int(b.get("steps", 0), '"steps"', -30, 30))
                self._json({"ok": True, "expcomp_mev": cur})
            elif p == "/api/run/start":
                # A pinned preview means cam1 is deliberately NOT on the fleet
                # vector. Recording a transect in that state produces stereo
                # pairs shot at two different exposures - unusable, and not
                # visible in the frames themselves. The UI blocks this in
                # pre-flight; releasing it here as well means an API client
                # that skips pre-flight cannot record a mismatched survey.
                if RIG.settings.pinned_node():
                    RIG.settings.discard("a run was started while a preview was "
                                         "pinned - the fleet must be matched to "
                                         "record")
                # Latch the static-fix decision BEFORE the run exists, so the
                # very first frame is judged by the same rule as the last. The
                # token is this request's proof of ownership: it binds the
                # latch to the run that started, or releases it if the start
                # was refused. On "run already active" the token is None and
                # nothing is released - dropping the RECORDING run's latch here
                # is how a duplicate Start used to end in fabricated positions
                # (audit 2026-08-24, review blocker). try/finally so a start
                # that raises cannot leak the latch either.
                tok = RIG.begin_run_nav()
                bound = False
                try:
                    res = RIG.runmgr.start(b)
                    if res.get("ok"):
                        bound = RIG.bind_run_nav(res.get("run_id"), tok)
                finally:
                    if not bound:
                        RIG.release_run_nav(tok)
                self._json(res)
            elif p == "/api/run/stop":
                # `drain` is honoured as an explicit override; absent, the
                # ~/rig/auto_drain policy decides.
                want_drain = (want_bool(b["drain"], '"drain"') if "drain" in b
                              else RIG.auto_drain_default())
                res = RIG.runmgr.stop()
                # Named, so a Stop that races another transect's Start cannot
                # release the latch that Start is holding.
                RIG.end_run_nav(run_id=res.get("run_id"))
                if res.get("ok") and want_drain:
                    # drain_started used to be hardcoded True while
                    # start_drain's return value was discarded, so a Stop with
                    # both nodes down reported a drain that never began and
                    # nothing anywhere said the cards were not emptied (audit
                    # 2026-08-23). Report what actually happened.
                    d = RIG.start_drain([n for n in res.get("summary", {})],
                                        auto=True)
                    res["drain"] = d
                    res["drain_started"] = bool(d.get("ok"))
                else:
                    res["drain_started"] = False
                self._json(res)
            elif p == "/api/drain":
                # Manual drain of one node or all connected. Refused during a
                # run. A manual drain is the operator overriding the wedge
                # skip, so it clears the mark (see Rig.start_drain).
                nodes = b.get("nodes")
                if nodes is None:
                    nodes = [m.name_ for m in RIG.monitors if m.is_connected()]
                elif not isinstance(nodes, list):
                    raise BadRequest('"nodes" must be a list of node names')
                else:
                    nodes = [self._node(n, '"nodes"').name_ for n in nodes]
                self._json(RIG.start_drain(
                    nodes, keep=want_bool(b.get("keep", False), '"keep"')))
            elif p == "/api/drain/cancel":
                self._json(RIG.cancel_drain())
            elif p == "/api/capture":
                # The rig is ALWAYS manual focus - never AF, on any path. This
                # is the one HTTP entry point that could still ask a body to
                # autofocus on the shutter press (capture_once -> ilxctl
                # /api/shutter {"af":true}), and nothing in the UI or the test
                # harness sends it. Refuse it here rather than leave the rule
                # depending on nobody typing it.
                if want_bool(b.get("af", False), '"af"'):
                    self._json({"ok": False, "error":
                                "this rig is manual focus on every path: "
                                "autofocus would move the lens between the "
                                "stereo pair's calibration and this frame. "
                                "Set focus with /api/focus/position"}, 400)
                    return
                # capture_once's result is passed through VERBATIM: it carries
                # per-node late_ms/skew and (run lane, 2026-08-23) host_offset_s,
                # the node-minus-host offset the fire was scheduled against.
                # Nothing here may reshape or round it.
                self._json(RIG.runmgr.capture_once(af=False))
            elif p == "/api/calibrate":
                # Never into a live transect: calibration holds FOCUS (an
                # AE-lock on the body) and its exposures race the run's own
                # frame naming — the run does its own calibration at start.
                if RIG.runmgr.status().get("active"):
                    self._json({"ok": False, "error":
                                "a run is active - calibration would corrupt "
                                "its exposures; stop the run first"}, 409)
                    return
                # samples < 1 measured nothing and answered ok:true with an
                # empty map; a junk value reached int() and became a 500.
                # Every calibration sample is a real exposure on both bodies,
                # so the upper bound is a shutter-count guard, not a nicety.
                samples = want_int(b.get("samples", 5), '"samples"', 1, 50)
                nodes = b.get("nodes")
                if nodes is None:
                    mons = None
                elif not isinstance(nodes, list):
                    raise BadRequest('"nodes" must be a list of node names')
                else:
                    mons = [self._node(n, '"nodes"') for n in nodes]
                self._json({"ok": True,
                            "latency_ms": {k: round(v * 1000, 2) for k, v in
                                           RIG.runmgr.calibrate_trigger(
                                               samples=samples, nodes=mons,
                                               force=True   # operator asked
                                           ).items()}})
            elif p == "/api/reconcile":
                # "Push the desired vector at the bodies NOW" — the manual kick
                # for the 3 s loop, used when a body has been nudged on its own
                # menu or a blind field is suspect. It is NOT the exposure
                # apply button: that is POST /api/settings, which is what the
                # UI's "Apply <cam>'s exposure to fleet" posts.
                #
                # exposure= must therefore be passed EXPLICITLY. reconcile_all's
                # contract is "exposure=None follows force", and a bare
                # force=True counts as an explicit fleet apply — so this call
                # re-pushed `desired`'s exposure onto every body and silently
                # destroyed a deliberate per-camera split (reproduced on a fake
                # rig: cam2 balanced to ISO 3200, one POST here and it was back
                # on the fleet's 400, with nothing in the journal to say so).
                # Exposure is per-camera BETWEEN explicit applies and this is
                # not one of them: force the rest of the vector, leave the
                # split alone. Regression: audit_rigd.py D8.
                RIG.settings.reconcile_all(force=True, exposure=False)
                self._json({"ok": True})
            elif p == "/api/strobe":
                # Strobe config: enable/node/delta_ms/pulse_ms. Validation and
                # the shutter-speed warning live in RunManager.set_strobe.
                self._json(RIG.runmgr.set_strobe(b or {}))
            elif p == "/api/run/open":
                # Open the run's folder in the host's file manager. The id is
                # validated by RunBrowser.run_dir (path-guarded), and the
                # RESOLVED path is passed as one argv - never through a shell.
                try:
                    path = RIG.runs.run_dir(str(b.get("id") or ""))
                except RunsError as e:
                    self._json({"ok": False, "error": str(e)}, 400)
                    return
                import subprocess
                opener = "open" if sys.platform == "darwin" else "xdg-open"
                if opener == "xdg-open" and not (os.environ.get("DISPLAY") or
                                                 os.environ.get(
                                                     "WAYLAND_DISPLAY")):
                    self._json({"ok": False, "path": path,
                                "error": "no display on the rigd host - "
                                         "browse or copy the path instead"})
                    return
                try:
                    subprocess.Popen([opener, path],
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                    self._json({"ok": True, "path": path})
                except OSError as e:
                    self._json({"ok": False, "path": path, "error": str(e)})
            elif p == "/api/focus/mode":
                # Focus MODE is fleet state, so it goes through `desired` and
                # is converged like any other field. Fanning it out instead
                # meant the reconcile loop pushed MF back over the operator's
                # choice within 3 s while the selector kept showing the choice,
                # and the disagreement never reached the convergence badge.
                # (The rig is always MF; this is how it STAYS MF, and how a
                # deliberate change is recorded rather than fought.)
                if "mode" not in b:
                    raise BadRequest('"mode" is required (the rig is always '
                                     "manual focus: 1 = MF)")
                rep = RIG.settings.update(
                    {"focus_mode": want_int(b["mode"], '"mode"')})
                self._json({"ok": bool(rep["applied"]), "applied":
                            rep["applied"], "rejected": rep["rejected"]})
            elif p == "/api/exposure":
                # Per-camera exposure, LIVE: the operator tunes one body while
                # its partner keeps the fleet vector, then either applies that
                # exposure to the fleet (POST /api/settings) or deliberately
                # leaves the bodies different. A node key is REQUIRED here so
                # no client can fleet-write exposure by accident; the split
                # shows up as convergence.exposure_split, never as a fault.
                #
                # Everything is checked HERE, before the node is contacted.
                # {"value":"abc"} used to be forwarded: ilxctl parsed it as 0,
                # SetDeviceProperty stalled the SDK ~6 s, the monitor's status
                # poll timed out behind it and cam1 flapped to "no connected
                # camera", 409ing every call for the next several polls - all
                # from one typo (verified live 2026-08-23). ilxctl rejects it
                # at the door now too, but old builds are in the field and a
                # camera is too expensive to spend on a typo either way.
                # The node key is REQUIRED so no client can fleet-write
                # exposure by accident (it is per-camera between applies).
                m = self._node(b.get("node"), '"node"')
                which = b.get("which")
                if which not in EXPOSURE_WHICH:
                    raise BadRequest('"which" must be one of %s (got %r)'
                                     % ("|".join(EXPOSURE_WHICH), which))
                value = want_int(b.get("value"), '"value"')
                # When the body publishes its own legal values for this field,
                # a value outside them cannot succeed - and on this hardware
                # trying costs an SDK stall, so it is refused rather than sent.
                status = m.snapshot().get("status") or {}
                choices = status.get(rigcore.CHOICE_KEY.get(which) or "")
                if isinstance(choices, list) and choices and value not in choices:
                    raise BadRequest(
                        "%s does not offer %s=%d; it offers %s"
                        % (m.name_, which, value,
                           ",".join(str(c) for c in choices[:16])))
                if not m.is_connected():
                    self._json({"ok": False, "error": "%s has no connected "
                                "camera" % m.name_}, 409)
                    return
                r = m.set_exposure(which, value)
                self._json(r if isinstance(r, dict)
                           else {"ok": False, "error": "no response"})
            elif p in ("/api/focus/drive", "/api/focus/position",
                       "/api/zoom/drive", "/api/zoom/position",
                       "/api/zoom/setting"):
                # Lens POSITION stays per-camera and out of `desired`: encoder
                # parity between the two bodies is unmeasured
                # (docs/future-tests.md §1), so nothing may push one body's
                # count to the other. Without a "node" key this fans out to
                # every connected camera as a convenience for a rig whose
                # lenses are set by hand; it is never re-asserted afterwards,
                # and the per-camera readback is in /api/fleet so a mismatch is
                # at least visible.
                self._json(RIG.fanout(p, b))
            elif p == "/api/node/connect":
                m = self._node(b.get("node"))
                r = http_json("http://%s:8080/api/connect" % m.host, body={},
                              timeout=30)
                self._json(r)
            elif p == "/api/node/focus":
                m = self._node(b.get("node"))
                # FOCUS is the AE-lock line, not autofocus: the rig is always
                # MF. A truthy string here would latch the hold with no way to
                # tell it had been asked for - require a real bool.
                r = http_json("http://%s:8081/gpio/focus" % m.host,
                              {"hold": want_bool(b.get("hold", False),
                                                 '"hold"')}, timeout=8)
                self._json(r)
            else:
                self._json({"ok": False, "error": "not found"}, 404)
        except BadRequest as e:
            # See do_GET: a client-side mistake is a clean 400 naming the
            # field, and is not journalled as a rigd error.
            self._json({"ok": False, "error": str(e)}, 400)
        except Exception as e:  # noqa: BLE001
            RIG.events.emit("error", "http", "POST %s: %s" % (p, e))
            self._json({"ok": False, "error": str(e)}, 500)


def main():
    global RIG
    RIG = Rig()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    print("rigd -> http://0.0.0.0:%d" % PORT)

    # systemctl stop/restart sends SIGTERM, and the default disposition is to
    # die on the spot: an active transect was left with run.json final:false,
    # no stats, and frames on the nodes that the next run would baseline away
    # as "already seen". Route it to the same finalising path as Ctrl-C.
    # serve_forever() must be woken from ANOTHER thread - calling shutdown()
    # from inside the handler deadlocks, because the handler runs on the very
    # thread shutdown() waits for.
    def _term(signum, _frame):
        RIG.events.emit("warn", "lifecycle",
                        "signal %d received - shutting down" % signum)
        threading.Thread(target=srv.shutdown, daemon=True).start()
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, _term)
        except (ValueError, OSError):
            pass          # not the main thread (embedded/test use): skip
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        RIG.stop()
        srv.server_close()


if __name__ == "__main__":
    main()

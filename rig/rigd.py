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
# Anomaly detectors — cheap checks over current fleet state, each with the
# evidence and a suggested action an operator (or agent) can act on.
# ---------------------------------------------------------------------------
STATIC_FIX_PATH = os.path.expanduser("~/rig/static_fix.json")


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


class StaticFixNav:
    """Delegating wrapper around NavReader: when there is NO valid live fix,
    fix_at()/snapshot() fall back to the operator-provided static position in
    ~/rig/static_fix.json ({lat, lon, label, ...}).

    Field case: no NMEA aboard, but the site position is known — e.g. the
    last live fix from a previous day. Honesty rules: a live fix ALWAYS wins;
    the static row carries only position/UTM (never depth, heading, or speed,
    which we do not know); `nav_epoch` is the original fix's capture epoch so
    `age_s` says exactly how old the position is; and health()/snapshot()
    name the static source so the UI preflight can say so out loud. The file
    is re-read on mtime change, so it can be edited without a restart."""

    def __init__(self, reader, navmod, events):
        self._r = reader
        self._nm = navmod
        self._ev = events
        self._sf = None
        self._sf_mtime = None
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
        try:
            with open(STATIC_FIX_PATH) as fh:
                sf = json.load(fh)
            lat, lon = float(sf["lat"]), float(sf["lon"])
            e, n, zone = self._nm.latlon_to_utm(lat, lon)
            sf.update({"lat": lat, "lon": lon,
                       "xutm": e, "yutm": n, "utm_zone": zone})
            self._sf = sf
            self._sf_mtime = mt
        except Exception as exc:  # noqa: BLE001
            self._sf = None
            self._ev.emit("warn", "nav",
                          "static_fix.json unreadable: %s" % exc)

    def static_label(self):
        self._load()
        return (self._sf or {}).get("label") or \
            ("static fix" if self._sf else None)

    def fix_at(self, epoch=None, max_age_s=None):
        row = self._r.fix_at(epoch, max_age_s)
        if row.get("valid"):
            return row
        self._load()
        if not self._sf:
            return row
        sf = self._sf
        row.update({"lat": sf["lat"], "lon": sf["lon"], "long": sf["lon"],
                    "xutm": sf["xutm"], "yutm": sf["yutm"],
                    "utm_zone": sf["utm_zone"]})
        row["nav_epoch"] = sf.get("captured_epoch")
        if row["nav_epoch"] and row.get("local_epoch"):
            row["age_s"] = abs(row["local_epoch"] - row["nav_epoch"])
        row["static_fix"] = self.static_label()
        return row

    def snapshot(self):
        snap = self._r.snapshot() or {}
        if not snap.get("valid"):
            self._load()
            if self._sf:
                sf = self._sf
                snap.update({"lat": sf["lat"], "lon": sf["lon"],
                             "xutm": sf["xutm"], "yutm": sf["yutm"],
                             "utm_zone": sf["utm_zone"],
                             "static_fix": self.static_label()})
        return snap

    def health(self):
        h = self._r.health()
        self._load()
        h["static_fix"] = self.static_label()
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
            if pw.get("undervolt_now") or pw.get("undervolt_since_boot"):
                out.append(self._a(
                    "node_undervoltage", m.name_,
                    "%s reports %s" % (m.name_, "UNDER-VOLTAGE NOW"
                                       if pw.get("undervolt_now")
                                       else "an under-voltage since boot"),
                    {"throttled": pw.get("throttled")},
                    "the PoE port/cable is sagging under load: this is the "
                    "step before the node reboots mid-run. Fix the power "
                    "budget before a survey",
                    sev="bad" if pw.get("undervolt_now") else "warn"))
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
            locked = (st == NodeMonitor.CONNECTED and status
                      and not status.get("writable")
                      and status.get("iso") in (None, "", "?")
                      and status.get("slotWritingLabel") in (None, "unknown"))
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
            if st == NodeMonitor.CONNECTED and not locked \
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
            oh = status.get("overheatingLabel")
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
            sw = status.get("slotWritingLabel")
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
            pk = status.get("priorityKeyLabel")
            if status.get("connected") and pk == "Camera position":
                out.append(self._a(
                    "pc_control_lost", m.name_,
                    "the body has taken control priority back",
                    {"priorityKey": status.get("priorityKey"), "label": pk},
                    "PC Remote priority was lost, so writes will be refused "
                    "and PC save will not deliver - it masquerades as an SDK "
                    "bug. Set the body's priority back to PC remote",
                    sev="bad"))
            batt = status.get("battery")
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
                if snap and not snap.get("gateway_online"):
                    # With a static fix armed the operator has already said
                    # "no NMEA aboard, use this position" — that is a state
                    # to display, not an alarm to chase.
                    sf = snap.get("static_fix")
                    out.append(self._a("nav_gateway_down", None,
                                       ("no live NMEA — static fix in use: %s"
                                        % sf) if sf
                                       else "iKonvert sending no data",
                                       {"health": self.nav.health()},
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
                                                 ("sats", "fix_source", "age_s")}},
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
        # Node clock agreement. Every scheduled fire and every epoch_hw edge
        # lives on the NODE's clock, so two nodes disagreeing with each other
        # lands 1:1 in inter-camera exposure skew — the whole sync budget is
        # 10 ms. With no local chrony master (the Jetson is gone in the
        # macOS-host topology) the Pis free-run apart silently; measured
        # 16.8 ms apart on 2026-08-20. Offsets are RTT-bounded /health
        # samples, so only differences well above the noise floor alarm.
        clocked = [(m.name_, m.clock) for m in self.monitors
                   if getattr(m, "clock", None)
                   and now - m.clock["at"] < 30
                   and m.clock["rtt_ms"] < 20]
        for i in range(len(clocked)):
            for j in range(i + 1, len(clocked)):
                (na, ca), (nb, cb) = clocked[i], clocked[j]
                skew = abs(ca["offset_s"] - cb["offset_s"]) * 1000.0
                noise = (ca["rtt_ms"] + cb["rtt_ms"]) / 2.0
                if skew > max(5.0, noise):
                    out.append(self._a(
                        "node_clock_skew", None,
                        "%s and %s clocks disagree by %.1f ms" % (na, nb, skew),
                        {"skew_ms": round(skew, 2),
                         "offsets_ms": {na: round(ca["offset_s"] * 1e3, 2),
                                        nb: round(cb["offset_s"] * 1e3, 2)},
                         "rtt_noise_ms": round(noise, 2)},
                        "scheduled fires land this far apart and the strobe "
                        "walks out of the exposure window. Re-point both "
                        "nodes' chrony at ONE reachable master (the rigd "
                        "host, or peer cam2 to cam1) and confirm with "
                        "chronyc tracking",
                        sev="bad" if skew > 8.0 else "warn"))
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
        self._drain_status = {"active": False, "node": None, "last": None}
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
        wrapped = StaticFixNav(reader, navmod, self.events)
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
            return dict(self._drain_status)

    def start_drain(self, nodes, keep=False):
        with self._drain_lock:
            if self._drain_status["active"]:
                return {"ok": False, "error": "a drain is already running on %s"
                        % self._drain_status["node"]}
            if self.runmgr.status().get("active"):
                return {"ok": False, "error": "a run is active - cannot drain"}
            nodes = [n for n in nodes
                     if any(m.name_ == n and m.is_connected() for m in self.monitors)]
            if not nodes:
                return {"ok": False, "error": "no connected node to drain"}
            self._drain_status = {"active": True, "node": nodes[0], "last": None,
                                  "queue": list(nodes)}
        threading.Thread(target=self._drain_worker, args=(nodes, keep),
                         daemon=True).start()
        return {"ok": True, "draining": nodes}

    def _drain_worker(self, nodes, keep):
        for node in nodes:
            host = next((m.host for m in self.monitors if m.name_ == node), None)
            if host is None:
                continue
            # Block runs on this node AND stop the monitor from reclaiming
            # the camera in remote mode while the drain holds it in transfer
            # mode.
            self.runmgr.draining = node
            for m in self.monitors:
                if m.name_ == node:
                    m.suspend_control = True
            with self._drain_lock:
                self._drain_status["node"] = node
            self.events.emit("info", "drain", "card drain started on %s" % node,
                             node=node)
            try:
                rep = draindrv.Drainer(
                    node, host, dest=self.DRAIN_DEST,
                    log=lambda m: self.events.emit("info", "drain", m)).run(
                        keep_card=keep)
                sev = "warn" if rep.get("errors") else "info"
                self.events.emit(
                    sev, "drain",
                    "%s drain done: %d pulled (%.1f GB), %d deleted, %d errors"
                    % (node, rep["pulled"], rep["bytes"] / 1e9, rep["deleted"],
                       len(rep["errors"])), node=node,
                    errors=rep["errors"][:5])
                with self._drain_lock:
                    self._drain_status["last"] = {"node": node, "at": time.time(),
                                                  **{k: rep[k] for k in
                                                     ("pulled", "bytes", "deleted",
                                                      "verified")},
                                                  "errors": len(rep["errors"])}
                # hand the pulled RAWs to ingest (best-effort; never blocks)
                try:
                    import ingest
                    ingest.ingest(self.DRAIN_DEST, log=lambda *a: None)
                except Exception as e:  # noqa: BLE001
                    self.events.emit("warn", "drain",
                                     "ingest after drain failed: %s" % e)
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "drain",
                                 "drain on %s failed: %s" % (node, e), node=node)
            finally:
                self.runmgr.draining = None
                for m in self.monitors:
                    if m.name_ == node:
                        m.suspend_control = False
        with self._drain_lock:
            self._drain_status["active"] = False
            self._drain_status["node"] = None

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
            # This node's clock vs ours, RTT-bounded, from the /health poll.
            # Two nodes disagreeing with each other is 1:1 exposure skew.
            "clock_offset_ms": (round(m.clock["offset_s"] * 1e3, 2)
                                if getattr(m, "clock", None) else None),
            "clock_rtt_ms": (round(m.clock["rtt_ms"], 2)
                             if getattr(m, "clock", None) else None),
            "gpio": h.get("gpio"), "imu": h.get("imu"),
            "disk_free_mb": h.get("disk_free_mb"),
            "cam_frames": h.get("cam_frames"),
            "stats": stats,
        }

    def fanout(self, api_path, body):
        """Forward an ilxctl call to every connected camera (or one, if body has
        a 'node'). Used for lens controls so cameras stay in lock-step."""
        target = body.get("node")
        payload = {k: v for k, v in body.items() if k != "node"}
        results = {}
        for m in self.monitors:
            if target and m.name_ != target:
                continue
            if not m.is_connected():
                continue
            results[m.name_] = http_json("http://%s:8080%s" % (m.host, api_path),
                                         payload, timeout=10)
        return {"ok": True, "results": results}

    def _imu_host(self):
        """Host currently serving IMU samples.

        The IMU is the rig's master orientation source for every camera, so the
        node it hangs off is discovered rather than hardcoded: it can be moved
        to another Pi without a code change. The last node that answered is
        cached, and only re-probed once it stops answering."""
        cached = getattr(self, "_imu_node", None)
        order = ([m for m in self.monitors if m.name_ == cached] +
                 [m for m in self.monitors if m.name_ != cached])
        for m in order:
            s = http_json("http://%s:8081/imu/latest" % m.host, timeout=3)
            if s and s.get("epoch") is not None:
                self._imu_node = m.name_
                return m.host, s
        self._imu_node = None
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
            return {"present": False}
        try:
            s = self.nav.snapshot() or {}
            if s.get("lat") is not None and s.get("lon") is not None:
                import nav as navmod
                x, y, z = navmod.latlon_to_utm(s["lat"], s["lon"])
                s.update(xutm=x, yutm=y, utm_zone=z)
            s["present"] = True
            return s
        except Exception as e:  # noqa: BLE001
            return {"present": False, "error": str(e)}

    def stop(self):
        """Shut down cleanly, finalising an active run first.

        A rigd that goes away mid-transect used to leave run.json with
        final:false and no per-worker stats, and - the part that costs imagery -
        left frames that had reached a node but not yet been pulled to be
        baselined as "old" by the NEXT run's PullWorker, so they were never
        pulled, renamed, or given a flight_log row. runmgr.stop() drains the
        pull workers and writes the manifest, so the ordinary exit path stops
        creating that seam. Idempotent: SIGTERM and the KeyboardInterrupt path
        can both land here."""
        if self._stopped.is_set():
            return
        self._stopped.set()
        try:
            if self.runmgr.status().get("active"):
                self.events.emit("warn", "lifecycle",
                                 "rigd is stopping with a run active - "
                                 "finalising it before exit")
                self.runmgr.stop()
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
        try:
            n = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            n = 0
        if n <= 0 or n > 1 << 20:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode() or "{}")
        except (ValueError, OSError):
            return {}

    def _mon(self, name):
        return next((m for m in RIG.monitors if m.name_ == name), None)

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
                since = int((q.get("since") or ["0"])[0])
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
                t0 = float((q.get("t0") or ["0"])[0] or 0)
                # A fresh client sends t0=0; give it the last second rather than
                # every sample the ring holds.
                if t0 <= 0:
                    t0 = now - 1.0
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
                m = self._mon((q.get("node") or [""])[0])
                if not m:
                    self._json({}, 404)
                else:
                    self._json(http_json("http://%s:8080/api/status" % m.host,
                                         timeout=6))
            elif p == "/api/shots":
                m = self._mon((q.get("node") or [""])[0])
                self._json(m.shots() if m else [])
            elif p == "/api/frame":
                self._proxy_frame((q.get("node") or [""])[0],
                                  (q.get("name") or [""])[0])
            elif p == "/api/liveview":
                self._proxy_liveview((q.get("node") or [""])[0])
            else:
                self._json({"ok": False, "error": "not found"}, 404)
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
        b = self._read_body()
        try:
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
                self._json(RIG.settings.preview(b, node=b.get("node")))
            elif p == "/api/settings/commit":
                self._json(RIG.settings.commit())
            elif p in ("/api/settings/discard", "/api/settings/revert"):
                self._json(RIG.settings.discard())
            elif p == "/api/settings/auto":
                RIG.settings.set_auto(bool(b.get("on")))
                self._json({"ok": True})
            elif p == "/api/ev":
                cur = RIG.settings.bump_ev(int(b.get("steps", 0)))
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
                self._json(RIG.runmgr.start(b or {}))
            elif p == "/api/run/stop":
                res = RIG.runmgr.stop()
                if res.get("ok") and b.get("drain", RIG.auto_drain_default()):
                    RIG.start_drain([n for n in res.get("summary", {})])
                    res["drain_started"] = True
                self._json(res)
            elif p == "/api/drain":
                # Manual drain of one node or all connected. Refused during a run.
                nodes = b.get("nodes") or [m.name_ for m in RIG.monitors
                                           if m.is_connected()]
                self._json(RIG.start_drain(nodes, keep=bool(b.get("keep"))))
            elif p == "/api/capture":
                self._json(RIG.runmgr.capture_once(af=bool(b.get("af"))))
            elif p == "/api/calibrate":
                # Never into a live transect: calibration holds FOCUS (an
                # AE-lock on the body) and its exposures race the run's own
                # frame naming — the run does its own calibration at start.
                if RIG.runmgr.status().get("active"):
                    self._json({"ok": False, "error":
                                "a run is active - calibration would corrupt "
                                "its exposures; stop the run first"}, 409)
                    return
                self._json({"ok": True,
                            "latency_ms": {k: round(v * 1000, 2) for k, v in
                                           RIG.runmgr.calibrate_trigger(
                                               samples=int(b.get("samples", 5)),
                                               force=True   # operator asked
                                           ).items()}})
            elif p == "/api/reconcile":
                RIG.settings.reconcile_all(force=True)
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
                rep = RIG.settings.update({"focus_mode": b.get("mode")})
                self._json({"ok": bool(rep["applied"]), "applied":
                            rep["applied"], "rejected": rep["rejected"]})
            elif p == "/api/exposure":
                # Per-camera exposure, LIVE: the operator tunes one body while
                # its partner keeps the fleet vector, then either applies that
                # exposure to the fleet (POST /api/settings) or deliberately
                # leaves the bodies different. A node key is REQUIRED here so
                # no client can fleet-write exposure by accident; the split
                # shows up as convergence.exposure_split, never as a fault.
                m = self._mon(b.get("node"))
                if not m:
                    self._json({"ok": False, "error": "a 'node' key naming one "
                                "camera is required on /api/exposure"}, 400)
                    return
                if not m.is_connected():
                    self._json({"ok": False, "error": "%s has no connected "
                                "camera" % m.name_}, 409)
                    return
                r = m.set_exposure(b.get("which"), b.get("value"))
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
                m = self._mon(b.get("node"))
                if not m:
                    self._json({"ok": False, "error": "bad node"}, 400); return
                r = http_json("http://%s:8080/api/connect" % m.host, body={},
                              timeout=30)
                self._json(r)
            elif p == "/api/node/focus":
                m = self._mon(b.get("node"))
                if not m:
                    self._json({"ok": False, "error": "bad node"}, 400); return
                r = http_json("http://%s:8081/gpio/focus" % m.host,
                              {"hold": bool(b.get("hold"))}, timeout=8)
                self._json(r)
            else:
                self._json({"ok": False, "error": "not found"}, 404)
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

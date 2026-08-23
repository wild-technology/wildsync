"""rigcore — the engine behind rigd: nodes, settings convergence, time, events.

Kept separate from the HTTP layer (rigd.py) so the moving parts can be reasoned
about and unit-tested on their own. Nothing here talks to a socket the browser
sees; it talks to the camera nodes and holds the fleet's agreed state.

All node I/O is over plain HTTP to two services per node:
    ilxctl  :8080  — the SDK/USB path (settings, frames, live view)
    piagent :8081  — the GPIO/IMU/health path
Both are treated as untrusted and slow: every call has a timeout and every
failure is a state transition, never an exception that escapes.
"""

import calendar
import csv
import json
import os
import re
import threading
import time
import urllib.error
import urllib.request
from collections import deque

# ---------------------------------------------------------------------------
# Fleet — see rig/PROTOCOL.md. cam_num is what frames get renamed with.
# ---------------------------------------------------------------------------
# cam1 is the primary: Pi 5, carries the IMU, previews global exposure changes,
# and is the intended streaming source. cam2 is a Pi 4 - same 40-pin header
# pinout, but its GPIO chip enumerates as gpiochip0 rather than gpiochip4, which
# piagent discovers rather than assumes.
_DEFAULT_NODES = [
    {"name": "cam1", "cam_num": 1, "host": "192.168.1.201"},
    {"name": "cam2", "cam_num": 2, "host": "192.168.1.202"},
    # cam3 (192.168.1.203) was an empty slot that fired a permanent
    # node_offline anomaly and a forever-"2/3" header. A third camera joins
    # via ~/rig/nodes.json in the field, not a code edit (see below).
]

# Hardcoding the addresses assumes every node holds its static lease, and a node
# that comes up on DHCP instead then looks permanently OFFLINE while sitting
# happily on the network. Allow an override so a field swap does not need a code
# edit: ~/rig/nodes.json (or $WILDSYNC_NODES) may hold either a full node list or
# just {"cam1": "192.168.1.171"} to move one host.
NODES_PATH = os.environ.get("WILDSYNC_NODES",
                            os.path.expanduser("~/rig/nodes.json"))


def _load_nodes():
    nodes = [dict(n) for n in _DEFAULT_NODES]
    try:
        with open(NODES_PATH) as fh:
            override = json.load(fh)
    except (OSError, ValueError):
        return nodes
    if isinstance(override, list):
        return [dict(n) for n in override if n.get("name") and n.get("host")]
    if isinstance(override, dict):
        by_name = {n["name"]: n for n in nodes}
        for name, val in override.items():
            host = val if isinstance(val, str) else (val or {}).get("host")
            if not host:
                continue
            if name in by_name:
                by_name[name]["host"] = host
            else:
                nodes.append({"name": name, "host": host,
                              "cam_num": (val or {}).get("cam_num", len(nodes) + 1)
                              if isinstance(val, dict) else len(nodes) + 1})
    return nodes


NODES = _load_nodes()
ILX_PORT = 8080
PIAGENT_PORT = 8081

RIG_HOME = os.path.expanduser("~/rig")
RUNS_DIR = os.path.expanduser("~/rig-runs")
DESIRED_PATH = os.path.join(RIG_HOME, "desired.json")
RIGD_LOG = os.path.join(RIG_HOME, "rigd.jsonl")

# Sony encodings (PROTOCOL.md). Settings the convergence engine keeps in step.
ISO_AUTO = 16777215
DRIVE_SINGLE = 1
DRIVE_CONT_LO = 65540


def shutter_encode(num, den):
    return ((int(num) & 0xFFFF) << 16) | (int(den) & 0xFFFF)


def shutter_decode(v):
    return (v >> 16) & 0xFFFF, v & 0xFFFF


# ---------------------------------------------------------------------------
# Event journal — the observability spine. One JSON object per line, a ring for
# the live API, monotonic sequence numbers for cursor polling.
# ---------------------------------------------------------------------------
class EventLog:
    SEV = {"debug": 0, "info": 1, "warn": 2, "error": 3, "critical": 4}

    # PROTOCOL.md calls ~/rig/rigd.jsonl "rolling", but nothing ever rolled it.
    # Measured at 188 bytes/event with the rig merely idling, and every reconcile
    # cycle, node transition and anomaly writes more: a multi-day deployment
    # fills the Jetson's root volume - which is the SAME volume ~/rig-runs writes
    # every transect frame to, so an unbounded journal ends a survey. Cap it and
    # keep exactly one previous generation, so the last ~32 MB of history
    # survives while the disk cannot be consumed.
    MAX_BYTES = 16 * 1024 * 1024

    def __init__(self, path=RIGD_LOG, ring=5000, max_bytes=MAX_BYTES):
        self.path = path
        self.max_bytes = max_bytes
        self._ring = deque(maxlen=ring)
        self._seq = 0
        self._lock = threading.Lock()
        self._run_fh = None            # optional per-run events.log
        os.makedirs(os.path.dirname(path), exist_ok=True)
        try:
            self._bytes = os.path.getsize(path)
        except OSError:
            self._bytes = 0

    def set_run_file(self, fh):
        with self._lock:
            self._run_fh = fh

    def emit(self, sev, kind, msg, node=None, **ctx):
        with self._lock:
            self._seq += 1
            rec = {"seq": self._seq, "ts": round(time.time(), 3),
                   "node": node, "sev": sev, "kind": kind, "msg": msg}
            if ctx:
                rec["ctx"] = ctx
            self._ring.append(rec)
            line = json.dumps(rec, separators=(",", ":"))
            # Both writes below guard OSError AND ValueError. A write to a file
            # that has been closed underneath us - which is exactly what the
            # per-run events.log does at run stop - raises ValueError("I/O
            # operation on closed file"), not OSError. That escaped, and killed
            # whichever thread was reporting the fault: when that thread was a
            # NodeMonitor the fleet silently stopped being watched for the rest
            # of the session, with the last thing in the log being the event
            # that killed it.
            try:
                with open(self.path, "a") as fh:
                    fh.write(line + "\n")
                self._bytes += len(line) + 1
                if self._bytes > self.max_bytes:
                    self._rotate_locked()
            except (OSError, ValueError):
                pass
            if self._run_fh:
                try:
                    self._run_fh.write("%s  %-8s %-18s %s%s\n" % (
                        time.strftime("%H:%M:%S", time.gmtime(rec["ts"])),
                        sev.upper(), kind, (("[%s] " % node) if node else ""),
                        msg))
                    self._run_fh.flush()
                except (OSError, ValueError):
                    pass
        return rec

    def _rotate_locked(self):
        """Roll the journal over. Caller holds the lock."""
        try:
            os.replace(self.path, self.path + ".1")
            self._bytes = 0
        except OSError:
            # Cannot rotate (read-only fs, permissions). Stop counting up so we
            # do not attempt it on every subsequent event.
            self._bytes = 0

    def since(self, seq, min_sev="debug", limit=500):
        floor = self.SEV.get(min_sev, 0)
        with self._lock:
            out = [r for r in self._ring
                   if r["seq"] > seq and self.SEV[r["sev"]] >= floor]
            nxt = self._seq
        return {"next": nxt, "events": out[-limit:]}

    def recent_counts(self, window_s=300):
        cut = time.time() - window_s
        counts = {}
        with self._lock:
            for r in self._ring:
                if r["ts"] >= cut and self.SEV[r["sev"]] >= 2:
                    counts[r["kind"]] = counts.get(r["kind"], 0) + 1
        return counts


# ---------------------------------------------------------------------------
# HTTP client to a node — every call bounded, failures returned not raised.
# ---------------------------------------------------------------------------
def http_json(url, body=None, timeout=8):
    req = urllib.request.Request(
        url, data=json.dumps(body).encode() if body is not None else None,
        method="POST" if body is not None else "GET",
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", "replace")
        return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read().decode("utf-8", "replace"))
        except Exception:  # noqa: BLE001
            return {"ok": False, "error": "http %d" % e.code, "_http": e.code}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": str(e), "_unreachable": True}


def http_bytes(url, timeout=20, cap=64 * 1024 * 1024):
    req = urllib.request.Request(url)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            declared = r.headers.get("Content-Length")
            data = r.read(cap + 1)
        if len(data) > cap:
            return None, "frame exceeds %d bytes" % cap
        # read(n) with an explicit size returns a SHORT buffer without raising
        # when the connection dies mid-body. Without this check a severed
        # transfer is indistinguishable from a complete one, and half a JPEG is
        # written into the survey folder and given a flight_log row - a record
        # that looks like real data. The node always sends Content-Length, so
        # compare against it and fail loudly instead.
        if declared is not None:
            try:
                want = int(declared)
            except ValueError:
                want = None
            if want is not None and len(data) != want:
                return None, ("truncated transfer: got %d of %d bytes"
                              % (len(data), want))
        return data, None
    except Exception as e:  # noqa: BLE001
        return None, str(e)


def free_mb(path):
    """Free megabytes on the volume holding `path`, or None if unreadable.

    Nothing in the rig measured the JETSON's free space - the "Disk" field in
    the UI and the disk_low detector both report a Pi's PC-save spool - yet
    ~/rig-runs on the Jetson is where every transect actually lands. A full
    Jetson volume is not caught in advance: each frame's write raises OSError,
    is counted as a pull_fail, and is NEVER retried (the frame is added to the
    worker's `seen` set before the download), so those frames get no flight_log
    row and no nav/IMU correlation. Cheap enough (one statvfs) to call from the
    anomaly scan; walks up to the nearest existing parent so a runs dir that has
    not been created yet still reports its future volume."""
    p = os.path.abspath(path)
    while True:
        try:
            st = os.statvfs(p)
        except OSError:
            parent = os.path.dirname(p)
            if not parent or parent == p:
                return None
            p = parent
            continue
        return int(st.f_bavail * st.f_frsize // (1024 * 1024))


# ---------------------------------------------------------------------------
# Per-node monitor: OFFLINE -> REACHABLE -> CAM_CONNECTED, with auto-reconnect.
# ---------------------------------------------------------------------------
class NodeMonitor(threading.Thread):
    OFFLINE = "OFFLINE"
    REACHABLE = "REACHABLE"          # piagent/ilxctl answer, camera not claimed
    CONNECTED = "CAM_CONNECTED"      # camera claimed and controllable
    # piagent answers but ilxctl does not (timeout / refused / garbage): the
    # daemon is wedged inside an SDK call (HANDOFF §2.2) or dead. This is NOT
    # "camera not claimed": POSTing /api/connect here strands one of ilxctl's
    # HTTP workers per attempt behind the stuck mutex, and the stale cached
    # status kept reporting connected:true so the UI kept requesting live
    # view — observed live on cam2, 2026-08-23, after a live-view storm.
    ILX_DOWN = "ILX_DOWN"

    def __init__(self, node, events, poll=2.0):
        super().__init__(daemon=True)
        self.node = node
        self.name_ = node["name"]
        self.host = node["host"]
        self.ilx = "http://%s:%d" % (self.host, ILX_PORT)
        self.pia = "http://%s:%d" % (self.host, PIAGENT_PORT)
        self.events = events
        self.poll = poll
        self._stopev = threading.Event()
        self._lock = threading.Lock()
        self.state = self.OFFLINE
        self.status = {}             # last ilxctl /api/status
        self.health = {}             # last piagent /health
        self.clock = None            # {"offset_s","rtt_ms","at"} vs our clock
        self._uptime = None          # piagent uptime_s from the last health
        self.rebooted_at = None      # epoch the node was last seen to restart
        self._connect_after = 0.0    # backoff gate
        self._backoff = 5.0
        self.last_seen = 0.0
        self.convergence = {"synced": None, "diverged": [], "last_check": 0}

    def stop(self):
        self._stopev.set()

    def snapshot(self):
        with self._lock:
            return {
                "name": self.name_, "host": self.host, "state": self.state,
                "last_seen": self.last_seen,
                "age_s": round(time.time() - self.last_seen, 1)
                if self.last_seen else None,
                "status": self.status, "health": self.health,
                "convergence": dict(self.convergence),
            }

    def _set_state(self, new, **ctx):
        if new != self.state:
            self.events.emit("info" if new != self.OFFLINE else "warn",
                             "node_transition",
                             "%s -> %s" % (self.state, new),
                             node=self.name_, **ctx)
            if new == self.CONNECTED:
                # Forget what we believe we pushed to this body. Fields with no
                # readback key (filetype, imagesize, transsize, expcomp) are
                # converged optimistically against this cache, so a body that
                # went away and came back - a power cycle, a USB replug, a
                # reboot to Sony's RAW+JPEG factory default - is remembered as
                # already correct and is NEVER re-pushed. The readable fields
                # snap back and the UI shows "synced", while the camera quietly
                # shoots the wrong file type for the whole survey. Clearing here
                # is what makes PROTOCOL.md's "settings re-push + verify after
                # every reconnect" actually true.
                self._pushed = {}
                # A body that went away may have reset. Exposure is normally
                # exempt from the continuous reconcile (per-camera between
                # applies), but a reconnected body gets one forced exposure
                # pass so a power-cycled camera rejoins the fleet's vector.
                self._exposure_force = True
            self.state = new

    def run(self):
        while not self._stopev.wait(self.poll):
            try:
                self._tick()
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "monitor", "tick error: %s" % e,
                                 node=self.name_)

    def _tick(self):
        t0 = time.time()
        health = http_json(self.pia + "/health", timeout=4)
        t1 = time.time()
        status = http_json(self.ilx + "/api/status", timeout=6)
        reachable = not status.get("_unreachable")
        pia_ok = not health.get("_unreachable")
        # The node's clock offset against ours, bounded by the poll's RTT.
        # Every scheduled fire and every epoch_hw edge lives on the NODE's
        # clock, so two nodes disagreeing with each other lands 1:1 in
        # inter-camera exposure skew — and with no local chrony master that
        # drift is silent. Sampled here because t0/t1 bracket the request.
        clock = None
        if pia_ok and isinstance(health.get("time"), dict) \
                and health["time"].get("epoch") is not None:
            clock = {"offset_s": health["time"]["epoch"] - (t0 + t1) / 2.0,
                     "rtt_ms": (t1 - t0) * 1000.0, "at": t1}
        # A node that restarts announces nothing; its piagent uptime going
        # backwards is the only tell. On this rig that means a POWER loss
        # (PoE budget collapse under synchronized fires took cam1's Pi 5 down
        # mid-run, 2026-08-23) - the event names it so it is never chased as
        # a software fault.
        rebooted = False
        if pia_ok:
            # Prefer the HOST uptime (piagent 2026-08-23+): the service's own
            # uptime resets on every deploy and read as a power loss.
            up = health.get("host_uptime_s")
            if up is None:
                up = health.get("uptime_s")
            if isinstance(up, (int, float)):
                if self._uptime is not None and up < self._uptime - 5:
                    rebooted = True
                self._uptime = up
        with self._lock:
            self.health = {} if health.get("_unreachable") else health
            if clock is not None:
                self.clock = clock
            if rebooted:
                self.rebooted_at = time.time()
            if reachable:
                self.status = status
                self.last_seen = time.time()
            elif pia_ok:
                # ilxctl is not answering while the Pi is. Do not keep serving
                # the last good status as if it were current: mark it stale and
                # NOT connected so is_connected(), the pull worker, the live-
                # view proxy and the UI all stop treating the body as live.
                self.status = {"connected": False, "stale": True,
                               "ilx_down": True,
                               "error": status.get("error") or "ilxctl not answering"}
        if rebooted:
            self.events.emit("error", "node_rebooted",
                             "%s restarted (piagent uptime reset to %.0f s) - "
                             "power loss: check the PoE budget/port and the "
                             "cable; both nodes firing in sync can collapse it"
                             % (self.name_, self._uptime or 0), node=self.name_)
        if not reachable and not pia_ok:
            self._set_state(self.OFFLINE)
            return
        if not reachable:
            # Never POST /api/connect at a wedged daemon; just keep probing.
            self._set_state(self.ILX_DOWN)
            return
        connected = bool(status.get("connected"))
        if connected:
            self._set_state(self.CONNECTED)
            self._backoff = 5.0
        else:
            self._set_state(self.REACHABLE)
            # ilxctl is up but the camera isn't claimed — try to (re)connect,
            # with backoff so a genuinely absent camera isn't hammered.
            now = time.time()
            if now >= self._connect_after:
                self.events.emit("info", "reconnect",
                                 "attempting camera connect", node=self.name_)
                r = http_json(self.ilx + "/api/connect", body={}, timeout=30)
                if r.get("ok") is False or r.get("_unreachable"):
                    err = str(r.get("error") or "")
                    # "already connected" after a USB drop means ilxctl kept a
                    # dead handle (OnDisconnected leaves m_handle set, so
                    # Camera::connect refuses). No retry can ever succeed;
                    # back off to the ceiling and say what actually fixes it.
                    if "already connected" in err.lower():
                        self._backoff = 60.0
                    else:
                        self._backoff = min(self._backoff * 1.6, 60.0)
                    self._connect_after = now + self._backoff
                    self.events.emit("warn", "reconnect",
                                     "connect failed: %s%s" % (
                                         err,
                                         " (dead SDK handle - restart ilxctl "
                                         "on the node)" if "already connected"
                                         in err.lower() else ""),
                                     node=self.name_, retry_s=round(self._backoff))
                else:
                    # ok:true only means ilxctl accepted the request. If the body
                    # never actually claims - another host holding it, PC-remote
                    # priority refused - /api/status stays connected:false and
                    # clearing the gate here re-POSTs connect on every single
                    # poll, forever: a USB storm on a camera someone else may be
                    # using, plus an event per attempt into an unrotated log.
                    # Keep backing off until a poll observes it truly connected.
                    self._backoff = min(self._backoff * 1.6, 60.0)
                    self._connect_after = now + self._backoff

    # ---- convenience node calls ------------------------------------------
    def is_connected(self):
        with self._lock:
            return self.state == self.CONNECTED

    def set_exposure(self, which, value):
        return http_json(self.ilx + "/api/exposure",
                         {"which": which, "value": value}, timeout=12)

    def set_focus_mode(self, mode):
        return http_json(self.ilx + "/api/focus/mode", {"mode": mode}, timeout=12)

    def set_store(self, dest):
        return http_json(self.ilx + "/api/store", {"dest": dest}, timeout=12)

    def shots(self):
        r = http_json(self.ilx + "/api/shots", timeout=10)
        return r if isinstance(r, list) else []

    def shutter(self, af=False):
        return http_json(self.ilx + "/api/shutter", {"af": af}, timeout=30)


# ---------------------------------------------------------------------------
# Desired-state convergence — one settings vector for the whole fleet.
# ---------------------------------------------------------------------------
# field -> (ilxctl 'which', status key the raw value is read back on).
# ilxctl emits a human label under the bare name (iso="ISO 400") and the raw
# numeric under "<name>Value" — convergence must compare the raw values.
CONVERGE_FIELDS = {
    "aperture": ("aperture", "apertureValue"),
    "shutter": ("shutter", "shutterValue"),
    "iso": ("iso", "isoValue"),
    "drive": ("drive", "driveValue"),
    "filetype": ("filetype", None),
    "imagesize": ("imagesize", None),
    "transsize": ("transsize", None),
    "store_dest": (None, "storeDest"),   # set via /api/store, read on storeDest
    # Focus MODE is fleet state (the rig is always MF - never AF, on any path),
    # so it converges like any other field: set via /api/focus/mode, read back on
    # focusMode. It used to be pushed unconditionally outside the convergence
    # map, which meant the operator's choice in the Controls selector was fought
    # by the reconcile loop within 3 s while the UI kept showing their choice,
    # and the disagreement never reached the convergence badge.
    # Focus/zoom POSITION are deliberately absent and must stay absent: lens
    # encoder parity between the two bodies is unmeasured (docs/future-tests.md
    # §1), so pushing one body's encoder count to the other could silently
    # change the stereo pair's interior orientation.
    "focus_mode": (None, "focusMode"),
    # White balance is fleet state: a stereo pair must render identically, so
    # the mode (256 = fixed color temperature) and the Kelvin value converge
    # like any readable field. The readback keys exist only on ilxctl builds
    # from 2026-08-20 on — the reconcile loop skips a node whose /api/status
    # lacks the key entirely, so a fleet mid-upgrade stays quiet instead of
    # alarming on nodes that simply predate the field.
    "wb_mode": ("wb_mode", "whiteBalance"),
    "colortemp": ("colortemp", "colorTemp"),
}

# Fields that older ilxctl builds do not know: absent readback key = node
# predates the field, skip silently (see CONVERGE_FIELDS note).
BUILD_GATED_FIELDS = ("wb_mode", "colortemp")

# Exposure is PER-CAMERA between explicit applies. The operator tunes one body
# live (POST /api/exposure with a node key) and either pushes that exposure to
# the fleet (POST /api/settings — the force path) or deliberately leaves the
# two bodies different, e.g. balancing them against unequal strobes or vignette.
# The continuous 3 s reconcile therefore only READS these fields and reports a
# split as convergence.exposure_split — information, not a fault. They are
# still written on: an explicit apply (force=True), a node reconnect (the body
# may have reset), or the in-pass reboot tell (a NON-exposure readable field
# reverted underneath us).
EXPOSURE_FIELDS = ("aperture", "shutter", "iso", "expcomp")

# ilxctl's `writable` map is keyed on the property's own name, which is not
# always the desired-vector field name. Getting this wrong reads back None and
# mis-reports a body-menu-only property as an ordinary divergence, re-alarming
# forever on something the operator cannot fix over USB.
WRITABLE_KEY = {"store_dest": "storeDest", "focus_mode": "focusMode",
                "wb_mode": "whiteBalance", "colortemp": "colorTemp"}
# enableFlag values from the SDK: 2 = DisplayOnly, i.e. readable but NOT
# settable over USB. `storeDest` and `pcsave` report this on these bodies.
ENABLE_DISPLAY_ONLY = 2

DEFAULT_DESIRED = {
    "aperture": 800,                 # f/8.0  (f x100)
    "shutter": shutter_encode(1, 200),
    "iso": 400,
    "expcomp": 0,
    "drive": DRIVE_SINGLE,
    "focus_mode": 1,                 # MF for a fixed survey rig
    "filetype": 1,                   # JPEG
    "imagesize": 1,                  # L
    # transsize=1 (Small) delivers 1616x1080 = 1.7 MP / ~320 KB to the host;
    # transsize=0 (Original) delivers 9504x6336 = 60.2 MP / 14.1 MB (both
    # measured 2026-08-16). Small is a review thumbnail, NOT survey-grade
    # imagery - but it is what the fleet is running today and what the operator
    # asked to keep, so it is not changed here silently. Both values are
    # settable over USB and both are now offered in the Controls tab: change it
    # there, deliberately, and watch the pull rate (14.1 MB/frame is 44x the
    # bytes per frame over the same USB link the capture path uses).
    "transsize": 1,
    "store_dest": 3,                 # both card + PC
    # Fixed white balance is rig policy (operator, 2026-08-21): AWB renders the
    # two bodies of a stereo pair differently shot-to-shot, which fights both
    # matching and any radiometric use of the JPEGs. 256 = fixed color
    # temperature mode; 5600 K is the flash/daylight point the survey uses.
    # RAW is WB-agnostic either way — this pins the JPEG rendering.
    "wb_mode": 256,
    "colortemp": 5600,
}

# Value bounds for the desired vector. Deliberately wide - the body's own choice
# list is the real authority and is checked first whenever we have one - but
# they catch what actually happens: the operator clears the ISO box, `+''` sends
# 0, and 0 is not a legal ISO (AUTO is 16777215). That value persisted to
# ~/rig/desired.json, survived every restart, and was re-pushed to both bodies
# every 3 s forever - and because the field is a READABLE one, the "reverted"
# check then cleared the blind-field cache on every pass too, restoring the
# multi-write-per-camera USB storm that cache exists to prevent, on the link the
# frame pulls share.
SETTING_BOUNDS = {
    "aperture": (50, 9900),          # f/0.5 .. f/99, f x100
    "iso": (25, 409600),             # plus the ISO_AUTO sentinel
    "expcomp": (-5000, 5000),        # mEV, +-5 EV
    "drive": (1, 0x7FFFFFFF),
    "filetype": (0, 5),              # None/JPEG/RAW/RAW+JPEG/RAW+HEIF/HEIF
    "imagesize": (1, 3),             # L/M/S
    "transsize": (0, 1),             # Original/Small
    "store_dest": (1, 3),            # PC/card/both
    "focus_mode": (1, 0xFFFF),
}
# Where the body publishes its own legal values for a field, when it does.
CHOICE_KEY = {"aperture": "apertureChoices", "shutter": "shutterChoices",
              "iso": "isoChoices", "drive": "driveChoices",
              "focus_mode": "focusModes"}

# How long a preview pin may hold one camera off the fleet's desired vector.
# The pin is what makes "see the change on cam1 before committing" possible, and
# it is also the one piece of state here that can quietly ruin a survey: while
# it is held cam1 shoots a DIFFERENT exposure from cam2, so every stereo pair
# taken in that window is unusable for photogrammetry. It therefore expires on
# its own, is dropped when a run starts, and lives only in memory so that a rigd
# restart releases it. 180 s is far longer than judging an exposure on the live
# view takes, and far shorter than a transect.
PREVIEW_TTL_S = 180.0
# Only exposure is previewable. focus_position / zoom_position are NOT here and
# NOT in DEFAULT_DESIRED - see the CONVERGE_FIELDS note and
# docs/future-tests.md §1.
PREVIEW_FIELDS = ("aperture", "shutter", "iso", "expcomp")
# A target the bodies will not accept does not become more true by being
# reported every 3 s; re-alarming that fast trains the operator to ignore the
# alert. Alarm on every CHANGE of the diverged field set, and otherwise at most
# this often.
DIVERGE_REALARM_S = 300.0


class SettingsManager:
    def __init__(self, monitors, events):
        self.monitors = monitors
        self.events = events
        self._lock = threading.Lock()
        self._load_note = None       # set by _load() when it fell back
        self.desired = self._load()
        self._auto = False           # exposure servo off by default (manual)
        # Staged preview state. Deliberately NOT persisted and NOT written to
        # DESIRED_PATH: a pin that survived a restart could hold the stereo pair
        # mismatched with nothing on screen to explain it. A fresh process is
        # always an unpinned process.
        self._pending = {}           # field -> staged value, cam1 only
        self._pin = None             # {"node","since","until"} while previewing
        if self._load_note:
            self.events.emit("warn", "settings", self._load_note,
                             desired=dict(self.desired))
            # Write the fallback out immediately, so the next restart is at
            # least reproducible rather than silently defaulting again.
            self._save()

    def _load(self):
        """Read the saved desired vector, falling back to the built-in defaults.

        The fallback is silent no longer. Convergence pushes whatever this
        returns to every camera within seconds, so a missing or unreadable file
        does not merely lose the settings - it actively overwrites the bodies
        with defaults. A survey configured for auto-ISO came back at ISO 400
        this way, with nothing in the log to explain it."""
        if not os.path.exists(DESIRED_PATH):
            self._load_note = ("no saved settings at %s - falling back to "
                               "built-in defaults, which WILL be pushed to every "
                               "camera" % DESIRED_PATH)
            return dict(DEFAULT_DESIRED)
        try:
            with open(DESIRED_PATH) as fh:
                d = json.load(fh)
            merged = dict(DEFAULT_DESIRED)
            merged.update({k: v for k, v in d.items() if k in DEFAULT_DESIRED
                           or k in ("expcomp", "focus_mode")})
            return merged
        except Exception as e:  # noqa: BLE001
            self._load_note = ("could not read %s (%s) - falling back to "
                               "built-in defaults, which WILL be pushed to "
                               "every camera" % (DESIRED_PATH, e))
            return dict(DEFAULT_DESIRED)

    def _save(self):
        try:
            os.makedirs(RIG_HOME, exist_ok=True)
            tmp = DESIRED_PATH + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(self.desired, fh, indent=2)
            os.replace(tmp, DESIRED_PATH)
        except OSError as e:
            self.events.emit("warn", "settings", "desired save failed: %s" % e)

    def get(self):
        # Expire a forgotten pin here too, not only in the reconcile loop, so
        # that anything reading the vector (the UI, /api/diag, the pre-flight
        # check) sees the truth. Deliberately without a reconcile: a GET must
        # not block on node I/O. The loop pulls cam1 back within 3 s.
        self._expire_pin(reconcile=False)
        with self._lock:
            d = dict(self.desired, _auto=self._auto)
            d["_preview"] = self._preview_state_locked()
            return d

    def set_auto(self, on):
        with self._lock:
            self._auto = bool(on)
        self.events.emit("info", "settings", "auto-exposure %s"
                         % ("on" if on else "off"))

    # ---- validation -------------------------------------------------------
    def _primary_status(self):
        """Status of the first connected camera, for its choice lists.

        The choice lists are the only authority on what a value MEANS on this
        body; bounds alone would accept ISO 300 on a body whose ladder has no
        such step."""
        for m in self.monitors:
            if m.is_connected():
                st = m.snapshot().get("status") or {}
                if st:
                    return m.name_, st
        return None, {}

    @staticmethod
    def _choice_values(raw):
        """ilxctl emits [{v,l},...]; the fake node and older builds emit bare
        numbers. Accept both, ignore anything that is neither."""
        out = []
        for c in (raw or []):
            try:
                out.append(int(c["v"] if isinstance(c, dict) else c))
            except (TypeError, ValueError, KeyError):
                continue
        return out

    def _validate_field(self, field, value, choices_from=None, choices=None):
        """(clean_value, None) or (None, why). Never raises."""
        if value is None:
            return None, ("no value - a cleared input box sends nothing, and "
                          "nothing is not a camera setting")
        if isinstance(value, bool):
            return None, "a boolean is not a camera setting value"
        try:
            v = int(value)
        except (TypeError, ValueError):
            return None, "not a number: %r" % (value,)
        if field == "shutter":
            num, den = shutter_decode(v)
            if num < 1 or den < 1:
                return None, ("%d decodes to %d/%d - both halves of the Sony "
                              "(num<<16)|den encoding must be at least 1"
                              % (v, num, den))
        elif field == "iso":
            lo, hi = SETTING_BOUNDS["iso"]
            if v != ISO_AUTO and not (lo <= v <= hi):
                return None, ("ISO %d is outside %d-%d (AUTO is the sentinel "
                              "%d)" % (v, lo, hi, ISO_AUTO))
        else:
            lo, hi = SETTING_BOUNDS.get(field, (None, None))
            if lo is not None and not (lo <= v <= hi):
                return None, "%d is outside the legal range %d-%d" % (v, lo, hi)
        legal = self._choice_values((choices or {}).get(CHOICE_KEY.get(field)))
        if legal and v not in legal:
            return None, ("%d is not one of the values %s offers for %s"
                          % (v, choices_from or "the camera", field))
        return v, None

    def update(self, changes, _staged=False):
        """User intent: validate, mutate desired, then force a reconcile.

        Returns {"applied": {...}, "rejected": {field: why}}. The rejections are
        returned rather than swallowed because an unattainable desired value is
        indistinguishable, from the operator's side, from a camera fault: the
        badge goes red and the anomaly points at the body."""
        applied, rejected = {}, {}
        prim, status = self._primary_status()
        for k, v in (changes or {}).items():
            if k not in DEFAULT_DESIRED and k not in ("expcomp", "focus_mode"):
                rejected[k] = "not a settings field"
                continue
            clean, why = self._validate_field(k, v, prim, status)
            if why:
                rejected[k] = why
                continue
            applied[k] = clean
        with self._lock:
            if applied:
                self.desired.update(applied)
                self._save()
        if rejected:
            self.events.emit("warn", "settings_rejected",
                             "refused into desired: %s"
                             % "; ".join("%s=%r (%s)" % (k, (changes or {})[k],
                                                         why)
                                         for k, why in rejected.items()),
                             rejected=rejected)
        if applied:
            self.events.emit("info", "settings", "desired updated: %s"
                             % ", ".join("%s=%s" % kv for kv in applied.items()))
            # A preview is a proposal about the vector that just changed
            # underneath it. Holding the pin here would leave cam1 on an
            # exposure nobody asked for while the fleet moved to a new one.
            if not _staged:
                self._release_pin("desired changed while a preview was pinned",
                                  reconcile=False)
            self.reconcile_all(force=True)
        return {"applied": applied, "rejected": rejected}

    def bump_ev(self, steps):
        """EV compensation nudge during a survey: ±1/3-stop steps."""
        with self._lock:
            self.desired["expcomp"] = int(self.desired.get("expcomp", 0)
                                          + steps * 333)
            cur = self.desired["expcomp"]
            self._save()
        self.events.emit("info", "settings", "EV bump %+d/3 -> %+d mEV"
                         % (steps, cur))
        self.reconcile_all(force=True)
        return cur

    # ---- staged preview: try it on cam1, then deploy to the fleet ----------
    # The product goal is "Cam1 allows live preview of settings changes before
    # deploying across fleet", and until now every change hit both bodies at
    # once: to judge an exposure the operator had to commit the whole fleet to
    # it and overwrite the saved survey configuration, with no prior value to
    # undo to. The staged flow is: preview (pending values -> the primary only,
    # `desired` untouched, that node pinned so the 3 s reconcile cannot yank it
    # back) -> commit (pending becomes desired, pin released, fleet converges)
    # or discard (pin released, cam1 snaps back).
    #
    # Everything about the pin is built so it cannot outlive the operator's
    # attention: it expires by itself (PREVIEW_TTL_S), it is dropped when a run
    # starts, it is dropped when `desired` changes underneath it, and it exists
    # only in memory so a rigd restart clears it. A forgotten preview can
    # therefore cost at most one TTL of mismatched pairs, never a whole survey.
    def _pick_preview_node(self, node=None):
        """The camera a preview lands on: the requested one, else the primary.

        The primary is cam1 by cam_num (PROTOCOL.md: it hosts the IMU and is the
        preview/stream source), falling back to the first connected camera so a
        one-body bench session can still preview."""
        conn = [m for m in self.monitors if m.is_connected()]
        if node:
            return next((m for m in conn if m.name_ == node), None)
        return next((m for m in conn if m.node.get("cam_num") == 1),
                    conn[0] if conn else None)

    def _preview_state_locked(self):
        """Caller holds the lock."""
        if not self._pin:
            return {"active": False}
        now = time.time()
        with_desired = {f: self.desired.get(f) for f in self._pending}
        return {"active": True, "node": self._pin["node"],
                "pending": dict(self._pending),
                "desired": with_desired,
                "since": round(self._pin["since"], 3),
                "expires_at": round(self._pin["until"], 3),
                "expires_in_s": round(max(0.0, self._pin["until"] - now), 1)}

    def preview_state(self):
        self._expire_pin(reconcile=False)
        with self._lock:
            return self._preview_state_locked()

    def pinned_node(self):
        with self._lock:
            return self._pin["node"] if self._pin else None

    def _expire_pin(self, reconcile=True):
        """Release a pin that has run out of time. Safe to call from anywhere."""
        with self._lock:
            if not self._pin or time.time() < self._pin["until"]:
                return False
        self._release_pin("preview expired after %ds without a decision - "
                          "cam1 is being pulled back to the fleet vector"
                          % int(PREVIEW_TTL_S), sev="warn", reconcile=reconcile)
        return True

    def _release_pin(self, why, sev="info", reconcile=True):
        """Drop the pin and the staged values. Returns the node, or None."""
        with self._lock:
            if not self._pin:
                self._pending = {}
                return None
            node = self._pin["node"]
            pending = dict(self._pending)
            self._pin = None
            self._pending = {}
        # Clear the marker on the node itself. The reconcile pass below only
        # visits CONNECTED cameras, so a camera that dropped out while pinned
        # would otherwise keep publishing preview:true for the rest of the
        # session - a permanent "the pair is not matched" alarm about a preview
        # that no longer exists.
        for m in self.monitors:
            if m.name_ == node:
                with m._lock:
                    if m.convergence.get("preview"):
                        m.convergence = {"synced": None, "diverged": [],
                                         "unsettable": [],
                                         "last_check": time.time()}
        self.events.emit(sev, "settings_preview", "preview released: %s" % why,
                         node=node, pending=pending)
        if reconcile:
            # Snap the previewed camera back now rather than up to 3 s later:
            # the operator is watching its live view and has just asked for it.
            self.reconcile_all(force=True)
        return node

    def preview(self, changes, node=None):
        """Apply pending exposure values to ONE camera, leaving desired alone."""
        self._expire_pin()
        prim, status = self._primary_status()
        clean, rejected = {}, {}
        for k, v in (changes or {}).items():
            if k not in PREVIEW_FIELDS:
                rejected[k] = ("only exposure is previewable (%s); focus and "
                               "zoom are per-camera and never fleet-applied"
                               % ", ".join(PREVIEW_FIELDS))
                continue
            c, why = self._validate_field(k, v, prim, status)
            if why:
                rejected[k] = why
            else:
                clean[k] = c
        m = self._pick_preview_node(node)
        if m is None:
            return {"ok": False, "error": "no connected camera to preview on",
                    "rejected": rejected}
        if not clean:
            return {"ok": False, "error": "nothing valid to preview",
                    "rejected": rejected, "node": m.name_}
        pushed, failed = {}, {}
        for f, v in clean.items():
            which = "expcomp" if f == "expcomp" else CONVERGE_FIELDS[f][0]
            r = m.set_exposure(which, v)
            if r.get("ok") is False or r.get("_unreachable"):
                failed[f] = str(r.get("error") or "rejected by the camera")
            else:
                pushed[f] = v
        if not pushed:
            # Nothing is on screen to judge, so pinning would only hold the
            # camera off the fleet vector for no benefit.
            return {"ok": False, "error": "the camera accepted none of it",
                    "node": m.name_, "failed": failed, "rejected": rejected}
        now = time.time()
        moved = None
        with self._lock:
            if self._pin and self._pin["node"] != m.name_:
                # The pin can only ever be on one camera. Moving it must not
                # carry the previous camera's staged values along - that one is
                # about to be re-converged, and committing values it is no
                # longer showing would deploy an exposure nobody looked at.
                moved = self._pin["node"]
                self._pending = {}
            self._pending.update(pushed)
            self._pin = {"node": m.name_,
                         "since": now if moved or not self._pin
                         else self._pin["since"],
                         "until": now + PREVIEW_TTL_S}
            state = self._preview_state_locked()
        if moved:
            self.events.emit("info", "settings_preview",
                             "preview moved to %s; %s goes back to the fleet "
                             "vector" % (m.name_, moved), node=moved)
        # Show the pin on the node immediately; the operator should not have to
        # wait out a reconcile tick to see that the pair is intentionally split.
        self._mark_pinned(m, state["pending"])
        self.events.emit("info", "settings_preview",
                         "preview on %s: %s (fleet stays on %s) - expires in %ds"
                         % (m.name_,
                            ", ".join("%s=%s" % kv for kv in
                                      sorted(pushed.items())),
                            ", ".join("%s=%s" % (f, state["desired"].get(f))
                                      for f in sorted(pushed)),
                            int(PREVIEW_TTL_S)),
                         node=m.name_, pending=pushed)
        return {"ok": True, "node": m.name_, "preview": state,
                "failed": failed, "rejected": rejected}

    def commit(self):
        """Promote the staged values into desired and converge the fleet."""
        self._expire_pin()
        with self._lock:
            pending = dict(self._pending)
            node = self._pin["node"] if self._pin else None
        if not pending:
            return {"ok": False, "error": "nothing staged to deploy"}
        # Release first: update() reconciles with force, and the previewed
        # camera has to be part of that pass or it stays on the staged values
        # by accident rather than by decision.
        self._release_pin("deployed to the fleet", reconcile=False)
        rep = self.update(pending, _staged=True)
        return {"ok": bool(rep["applied"]), "node": node,
                "applied": rep["applied"], "rejected": rep["rejected"]}

    def discard(self, why="discarded by the operator"):
        node = self._release_pin(why)
        if node is None:
            return {"ok": False, "error": "no preview to discard"}
        return {"ok": True, "node": node}

    def _mark_pinned(self, m, pending):
        """Publish the preview on the node's convergence badge.

        A pinned camera is NOT synced and must never be shown as synced - but it
        is not divergent either, and raising settings_divergent for a state the
        operator asked for is how alerts get ignored. It is its own state."""
        with m._lock:
            m.convergence = {"synced": None, "preview": True,
                             "preview_fields": sorted(pending),
                             "diverged": [], "unsettable": [],
                             "last_check": time.time()}
        m._diverge_strikes = 0

    def reconcile_all(self, force=False):
        self._expire_pin(reconcile=False)
        pinned = self.pinned_node()
        with self._lock:
            pending = dict(self._pending)
        for m in self.monitors:
            if not m.is_connected():
                continue
            if pinned == m.name_:
                # The whole point of the pin: do not fight the preview.
                self._mark_pinned(m, pending)
                continue
            try:
                self._reconcile_node(m, force)
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "settings",
                                 "reconcile error: %s" % e, node=m.name_)

    def _reconcile_node(self, m, force):
        with self._lock:
            want = dict(self.desired)
        status = m.snapshot()["status"]
        # field -> the body's own words for why it refused a field that has NO
        # readback key. For those fields this is the only evidence there will
        # ever be, and it used to be collected into a local that nothing read.
        blind_fail = {}
        # Some fields have no readback key on this body, so "have == target" can
        # never be true for them and they were re-pushed on every single pass -
        # four SetDeviceProperty writes per camera every 3 s, forever, over a USB
        # link we also want to pull frames across. Remember what was last pushed
        # and skip when nothing changed; a body reset clears it via `force`.
        pushed = getattr(m, "_pushed", None)
        if pushed is None or force:
            pushed = m._pushed = {}
        # A body that reset does not announce it. Clearing the blind-field cache
        # on the OFFLINE->CAM_CONNECTED transition is necessary but NOT
        # sufficient: a camera can power-cycle entirely between two 2 s polls, so
        # the monitor never observes a transition at all and the cache survives a
        # reboot it should not have. The reliable tell is the readable fields -
        # if aperture/shutter/ISO/drive/store have reverted underneath us, the
        # body has been reset or hand-nudged, and whatever we believe we pushed
        # to the fields with NO readback (filetype, imagesize, transsize,
        # expcomp) is no longer credible either. Sony's factory default is
        # RAW+JPEG, so the specific consequence of trusting the stale cache is a
        # whole transect silently shot in the wrong file type while the UI
        # reports "synced".
        # Whether exposure fields are written this pass. A deliberate exposure
        # split must not read as "the body reset", so the in-pass reboot tell
        # below counts NON-exposure readable fields only.
        exp_force = bool(force) or bool(getattr(m, "_exposure_force", False))
        if not force:
            reverted = [f for f, (w, k) in CONVERGE_FIELDS.items()
                        if k and f not in EXPOSURE_FIELDS
                        and want.get(f) is not None
                        and status.get(k) is not None
                        and status.get(k) != want[f]]
            if reverted:
                pushed = m._pushed = {}
                exp_force = True
        # A push result, judged once, in one place. Three outcomes matter and
        # they used to be conflated:
        #   * unreachable  - the POST never got there. Recording it in the
        #     blind-field cache marks an UNSENT write as applied, and because a
        #     blind field is then skipped until a force or a readable-field
        #     revert clears the cache, that lie does NOT self-heal: the body
        #     keeps shooting the wrong file type while the badge reads synced.
        #   * rejected     - the body said no. For a field with no readback this
        #     is the ONLY evidence that exists, so it has to be kept.
        #   * accepted     - cache it (blind fields only; readable ones are
        #     judged by the verification read below, which is stronger).

        def note(field, r, key, value):
            if r.get("_unreachable"):
                return
            if r.get("ok") is False:
                if not key:
                    blind_fail[field] = str(r.get("error") or "rejected")
                return
            if not key:
                pushed[field] = value
        for field, (which, key) in CONVERGE_FIELDS.items():
            target = want.get(field)
            if target is None:
                continue
            if field in EXPOSURE_FIELDS and not exp_force:
                continue          # per-camera between applies; still read below
            if field in BUILD_GATED_FIELDS and key not in status:
                continue    # node's ilxctl predates this field: skip quietly
            have = status.get(key) if key else None
            if key and have == target and not force:
                continue
            if not key and pushed.get(field) == target and not force:
                continue                       # blind field, already at target
            if field == "store_dest":
                # No forced re-push when it already reads right: storeDest is
                # DisplayOnly on these bodies (body-menu-only), so a redundant
                # write is a guaranteed error, every time, for nothing.
                if have != target:
                    note(field, m.set_store(target), key, target)
                continue
            if field == "focus_mode":
                note(field, m.set_focus_mode(target), key, target)
                continue
            if which:
                note(field, m.set_exposure(which, target), key, target)
        # expcomp has no readback key either, so it gets the same treatment
        # rather than an unconditional write on every reconcile. It is an
        # exposure field, so it also honours the per-camera exemption.
        want_ev = want.get("expcomp", 0)
        if exp_force and (pushed.get("expcomp") != want_ev or force):
            r = m.set_exposure("expcomp", want_ev)
            err = str(r.get("error", ""))
            if r.get("ok") is False and ("InvalidCalled" in err
                                         or "DisplayOnly" in err
                                         or "read-only" in err):
                # This body does not expose EV compensation in its current
                # exposure mode — on the ILX-LR1, full Manual (program=M)
                # makes expcomp enableFlag=DisplayOnly, and the live fleet
                # answered exactly that. Cache it so we stop writing; it is
                # not a divergence the operator can act on.
                pushed["expcomp"] = want_ev
            else:
                note("expcomp", r, None, want_ev)
        # Verify the readable ones settled. Read the CAMERA, not m.snapshot():
        # that returns the cached status dict the diff was computed from, which
        # only refreshes on the 2 s poll. Confirming against it means every user
        # settings change raises a spurious settings_divergent and flashes the
        # UI badge for a change that in fact applied cleanly.
        time.sleep(0.15)
        after = http_json(m.ilx + "/api/status", timeout=8)
        if not isinstance(after, dict) or after.get("_unreachable"):
            return                                  # offline: not a divergence
        # The reconnect flag is consumed by a pass that got far enough to push
        # and re-read; an unreachable node above keeps it for the next pass.
        m._exposure_force = False
        still, exp_split = [], []
        for field, (which, key) in CONVERGE_FIELDS.items():
            if field in BUILD_GATED_FIELDS and key not in after:
                continue    # node's ilxctl predates this field: not divergence
            if key and want.get(field) is not None \
                    and after.get(key) != want[field]:
                # An exposure field left alone this pass is a deliberate
                # per-camera split: report it as information. One that was
                # PUSHED and still disagrees is a real divergence.
                if field in EXPOSURE_FIELDS and not exp_force:
                    exp_split.append(field)
                else:
                    still.append(field)
        # Fold in the fields that have NO readback key and were refused. They
        # are invisible to the check above by construction - "have == target"
        # can never be true for them - so a body that rejects filetype /
        # imagesize / transsize (its menu is open, PC-remote priority was lost)
        # used to publish synced:true with a green dot on both the Fleet card
        # and the Controls strip while the two bodies recorded in DIFFERENT
        # formats and resolutions. A stereo pair shot that way is not a pair.
        for field in blind_fail:
            if field not in still:
                still.append(field)
        # A property the body refuses to expose for writing (a manual aperture
        # ring; storeDest and pcsave, which are body-menu-only on these bodies)
        # will never converge, and re-alarming every cycle trains the operator
        # to ignore the alert. Report it once, distinctly.
        writable = after.get("writable") or {}
        # dict.get evaluates its default eagerly, and expcomp (a blind_fail
        # candidate) is deliberately absent from CONVERGE_FIELDS — a bare
        # CONVERGE_FIELDS[f] here raised KeyError, aborted the whole pass,
        # froze the convergence badge at its previous value and suppressed
        # every divergence alarm for the node, once per 3 s, forever.
        unsettable = [f for f in still
                      if writable.get(WRITABLE_KEY.get(
                          f, CONVERGE_FIELDS.get(f, (None,))[0] or f))
                      == ENABLE_DISPLAY_ONLY]
        synced = not still
        with m._lock:
            m.convergence = {"synced": synced, "diverged": still,
                             "exposure_split": exp_split,
                             "unsettable": unsettable,
                             "blind_errors": dict(blind_fail),
                             "last_check": time.time()}
        if not synced:
            # Require two consecutive failures before alarming: one poll can
            # race a setting the body is still applying.
            m._diverge_strikes = getattr(m, "_diverge_strikes", 0) + 1
            if m._diverge_strikes >= 2:
                # Alarm on every CHANGE of what is wrong, then at most once per
                # DIVERGE_REALARM_S. An unattainable target (ISO 0 typed into
                # the box, a lens with no electronic aperture) otherwise emits
                # one warning per camera per 3 s into an unrotated journal, for
                # the whole deployment.
                sig = (tuple(sorted(unsettable)), tuple(sorted(still)))
                last_sig, last_at = getattr(m, "_diverge_sig", (None, 0.0))
                now = time.time()
                if sig != last_sig or (now - last_at) >= DIVERGE_REALARM_S:
                    m._diverge_sig = (sig, now)
                    if unsettable:
                        self.events.emit(
                            "warn", "settings_unsettable",
                            "%s cannot be set over USB on this body (body-menu "
                            "control only, e.g. storeDest or a manual aperture "
                            "ring)" % ",".join(unsettable),
                            node=m.name_, fields=unsettable)
                    rest = [f for f in still if f not in unsettable]
                    if rest:
                        self.events.emit(
                            "warn", "settings_divergent",
                            "fields not converged: %s" % ",".join(rest),
                            node=m.name_, fields=rest,
                            errors={f: blind_fail[f] for f in rest
                                    if f in blind_fail})
        else:
            m._diverge_strikes = 0
            m._diverge_sig = (None, 0.0)


# ---------------------------------------------------------------------------
# Time authority: GPS (from nav) with Jetson fallback, plus per-node EXIF offset.
# ---------------------------------------------------------------------------
class TimeSync:
    def __init__(self, events):
        self.events = events
        self._lock = threading.Lock()
        self.gps_offset = 0.0        # gps_epoch - jetson_epoch
        self.source = "jetson"
        self.exif_offset = {}        # node -> (camera_epoch - true_epoch)

    def feed_gps(self, gps_epoch):
        if not gps_epoch:
            return
        with self._lock:
            self.gps_offset = gps_epoch - time.time()
            self.source = "gps"

    def clear_gps(self):
        with self._lock:
            self.gps_offset = 0.0
            self.source = "jetson"

    def now(self):
        with self._lock:
            return time.time() + self.gps_offset, self.source

    def set_exif_offset(self, node, off):
        with self._lock:
            self.exif_offset[node] = off
        self.events.emit("info", "exif_offset",
                         "camera clock offset %.2fs" % off, node=node)

    def correct_exif(self, node, camera_epoch):
        with self._lock:
            off = self.exif_offset.get(node)
        if off is None or camera_epoch is None:
            return None
        return camera_epoch - off


# ---------------------------------------------------------------------------
# Transect browser — read-only view of ~/rig-runs.
#
# "Keep things organized by transect" is a goal clause with no implementation:
# runs were written to disk and never readable again from the UI. This is the
# read side, and it is READ-ONLY on purpose - a browser that can delete or
# rewrite a transect is a browser that can destroy the survey.
#
# The question it exists to answer, for a photogrammetry operator, is not "what
# files are there" but "did BOTH cameras produce a frame for every shot, and if
# not, where are the holes" - a single-camera shot is not a stereo pair and
# cannot be photogrammetrised, and today that is only discoverable by counting
# files in two directories by hand, after the boat is back.
# ---------------------------------------------------------------------------
# Every one of these bounds exists because the UI polls this during a live
# survey, on the same host that is writing the frames.
RUNS_LIST_MAX = 200              # runs returned by one list call
RUN_JSON_MAX_BYTES = 8 << 20     # a run.json carries up to 2000 index entries
FLIGHT_MAX_BYTES = 32 << 20
FLIGHT_MAX_ROWS = 50000          # 50k rows is ~14 h at 1 Hz on one camera
GAPS_MAX = 500                   # reported individually; the count is exact
FRAME_MAX_BYTES = 64 << 20
# A frame from each camera closer together than this belongs to the same shot.
# The pair itself is ~1 ms apart (measured 0.59 ms mean), but the timestamp
# written to flight_log falls back to corrected EXIF or to command time when
# there is no GPIO edge, and those are coarser - so the window has to tolerate
# the fallback tiers without swallowing the next shot at a 2 s interval.
PAIR_TOL_S = 0.75
_RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
FRAME_EXTS = (".jpg", ".jpeg", ".arw", ".heif", ".hif")


def _flight_dt_epoch(s):
    """`YYMMDD_hhmmss.ss` (UTC, PROTOCOL.md) -> epoch, or None.

    Never raises and never guesses: an unparseable stamp returns None and that
    row is reported as un-timed rather than being placed at an invented instant.
    """
    m = re.match(r"^(\d{2})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})(\.\d+)?$",
                 (s or "").strip())
    if not m:
        return None
    yy, mo, dd, hh, mi, ss = (int(m.group(i)) for i in range(1, 7))
    try:
        # timegm, not mktime: the column is UTC (run.py stamps it with
        # time.gmtime), and inverting it through local time would shift every
        # frame by the Jetson's timezone offset - which on a boat is whatever
        # the last person to set it chose.
        base = calendar.timegm((2000 + yy, mo, dd, hh, mi, ss, 0, 1, 0))
    except (ValueError, OverflowError):
        return None
    return base + (float(m.group(7)) if m.group(7) else 0.0)


class RunsError(Exception):
    """A request that must not be served — bad id, escape attempt, no such run."""


class RunBrowser:
    """Read-only access to the run directory tree, with the path guard.

    Every id and filename here arrives from a browser. The guard is belt and
    braces on purpose: a strict character class (no separators, no leading dot,
    so `..` and absolute paths are rejected before they touch the filesystem)
    AND a realpath containment check (so a symlink inside the runs dir cannot
    point out of it either)."""

    def __init__(self, events=None):
        self.events = events
        self._lock = threading.Lock()
        self._cache = {}             # run_id -> (signature, detail dict)

    # ---- path guard -------------------------------------------------------
    @staticmethod
    def root():
        # Read the module global at call time: the test harness rebinds
        # rigcore.RUNS_DIR to a temp dir, and capturing it at construction
        # would send every read at the real survey tree.
        return os.path.realpath(os.path.expanduser(RUNS_DIR))

    @classmethod
    def _contained(cls, path):
        root = cls.root()
        real = os.path.realpath(path)
        return real == root or real.startswith(root + os.sep)

    @classmethod
    def run_dir(cls, run_id):
        if not run_id or not _RUN_ID_RE.match(run_id):
            raise RunsError("bad run id")
        path = os.path.join(cls.root(), run_id)
        if not cls._contained(path):
            raise RunsError("bad run id")
        if not os.path.isdir(path):
            raise RunsError("no such run")
        return path

    @classmethod
    def child(cls, run_id, *parts):
        """A file inside a run, validated part by part."""
        base = cls.run_dir(run_id)
        for p in parts:
            if not p or not _NAME_RE.match(p):
                raise RunsError("bad name")
        path = os.path.join(base, *parts)
        if not cls._contained(path):
            raise RunsError("bad name")
        return path

    # ---- reads ------------------------------------------------------------
    @staticmethod
    def _read_json(path, cap=RUN_JSON_MAX_BYTES):
        try:
            if os.path.getsize(path) > cap:
                return None
            with open(path) as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    @staticmethod
    def _cam_dirs(root):
        try:
            return sorted(d for d in os.listdir(root)
                          if _NAME_RE.match(d) and os.path.isdir(
                              os.path.join(root, d))
                          and os.path.exists(os.path.join(root, d,
                                                          "flight_log.csv")))
        except OSError:
            return []

    def list_runs(self, limit=50):
        """Newest first, one summary line each. Bounded and non-blocking."""
        limit = max(1, min(int(limit or 50), RUNS_LIST_MAX))
        root = self.root()
        try:
            names = [d for d in os.listdir(root)
                     if _RUN_ID_RE.match(d)
                     and os.path.isdir(os.path.join(root, d))]
        except OSError:
            return {"runs": [], "root": root, "total": 0}
        # The id is YYMMDD_hhmm_label, so a reverse name sort is chronological;
        # mtime would reorder a run whose directory was touched afterwards.
        names.sort(reverse=True)
        out = []
        for rid in names[:limit]:
            path = os.path.join(root, rid)
            doc = self._read_json(os.path.join(path, "run.json")) or {}
            cams = self._cam_dirs(path)
            stats = doc.get("stats") or {}
            out.append({
                "run_id": rid,
                "label": doc.get("label"),
                "started": doc.get("started"),
                "nodes": doc.get("nodes") or cams,
                "cams": cams,
                "frames": doc.get("frames"),
                "final": bool(doc.get("final")),
                "interrupted": bool(doc.get("interrupted")),
                "time_source": (doc.get("time") or {}).get("source"),
                "skew_ms_max": (doc.get("sync") or {}).get("skew_ms_max"),
                "pulled": {n: (s or {}).get("pulled")
                           for n, s in stats.items()},
                "failed": {n: (s or {}).get("failed")
                           for n, s in stats.items()},
                "has_run_json": bool(doc),
            })
        return {"runs": out, "root": root, "total": len(names)}

    def _flight_rows(self, root, cam):
        """(rows, truncated). Bounded read; a malformed row is skipped, never
        guessed at."""
        path = os.path.join(root, cam, "flight_log.csv")
        rows, truncated = [], False
        try:
            if os.path.getsize(path) > FLIGHT_MAX_BYTES:
                truncated = True
            with open(path, newline="") as fh:
                for i, row in enumerate(csv.DictReader(fh)):
                    if i >= FLIGHT_MAX_ROWS:
                        truncated = True
                        break
                    rows.append(row)
        except (OSError, ValueError, csv.Error):
            pass
        return rows, truncated

    def detail(self, run_id):
        """One run: its manifest, per-camera counts, and pair completeness."""
        root = self.run_dir(run_id)
        # Cache on the mtimes/sizes of the files we read, so a UI polling this
        # every few seconds during a live transect re-parses only when the run
        # has actually grown.
        sig = []
        for rel in ["run.json"] + [os.path.join(c, "flight_log.csv")
                                   for c in self._cam_dirs(root)]:
            try:
                st = os.stat(os.path.join(root, rel))
                sig.append((rel, st.st_mtime_ns, st.st_size))
            except OSError:
                sig.append((rel, None, None))
        sig = tuple(sig)
        with self._lock:
            hit = self._cache.get(run_id)
        if hit and hit[0] == sig:
            return hit[1]
        doc = self._read_json(os.path.join(root, "run.json")) or {}
        cams = self._cam_dirs(root)
        percam, frames = {}, []
        for cam in cams:
            rows, truncated = self._flight_rows(root, cam)
            epochs = []
            untimed = 0
            for r in rows:
                ep = _flight_dt_epoch(r.get("datetime", ""))
                if ep is None:
                    untimed += 1
                    continue
                epochs.append(ep)
                frames.append((ep, cam, r.get("filename") or "",
                               r.get("capture_source") or ""))
            percam[cam] = {
                "rows": len(rows),
                "untimed_rows": untimed,
                "truncated": truncated,
                "first": min(epochs) if epochs else None,
                "last": max(epochs) if epochs else None,
                "sources": _count_by(r.get("capture_source") for r in rows),
                "files": _count_files(os.path.join(root, cam)),
            }
        detail = {
            "run_id": run_id,
            "path": root,
            "label": doc.get("label"),
            "started": doc.get("started"),
            "final": bool(doc.get("final")),
            "interrupted": bool(doc.get("interrupted")),
            "interrupted_note": doc.get("interrupted_note"),
            "nodes": doc.get("nodes") or cams,
            "cams": cams,
            "config": doc.get("config") or {},
            "time": doc.get("time") or {},
            "sync": doc.get("sync") or {},
            "stats": doc.get("stats") or {},
            "frames_indexed": doc.get("frames"),
            "per_camera": percam,
            "has_run_json": bool(doc),
        }
        pairs, shots_full = self._pairs(frames, cams, doc)
        detail["pairs"] = pairs
        detail["strobe"] = doc.get("strobe")
        with self._lock:
            if len(self._cache) > 16:
                self._cache.clear()
            self._cache[run_id] = (sig, detail, shots_full)
        return detail

    def shots(self, run_id, offset=0, limit=200):
        """The run's full grouped shot list, paginated. offset < 0 counts from
        the end (offset=-50 → the last 50 shots), which is what a live review
        screen wants without a prior total-count round trip."""
        self.detail(run_id)          # refresh the cache off the files' mtimes
        with self._lock:
            hit = self._cache.get(run_id)
        full = hit[2] if hit and len(hit) > 2 else []
        n = len(full)
        try:
            offset, limit = int(offset), int(limit)
        except (TypeError, ValueError):
            offset, limit = 0, 200
        if offset < 0:
            offset = max(0, n + offset)
        offset = min(offset, n)
        limit = max(1, min(limit, 1000))
        return {"run_id": run_id, "total": n, "offset": offset,
                "shots": full[offset:offset + limit]}

    @staticmethod
    def _pairs(frames, cams, doc):
        """Group frames into shots; return (summary, full_shot_list).

        This is the whole point of the view: a shot that only one camera
        recorded is not a stereo pair, and the operator needs to know that -
        and WHERE - while the boat is still on the line.

        Where run.json's per-frame index covers a shot, the µs-grade index
        epochs replace the centisecond-quantised flight_log datetimes for the
        spread (spread_src says which), and the shot gets a strobe verdict:
        the acceptance check of docs/strobe-trigger.md §4.2 — the strobe
        instant must sit inside the INTERSECTION of every member's measured
        [fall, rise] exposure window, all epoch_hw-derived. A verdict is only
        computed when every member is src=gpio_edge; a coarser source would
        make the check a guess wearing a measurement's clothes."""
        if not cams:
            return ({"cams": [], "shots": 0, "complete": 0, "incomplete": 0,
                     "gaps": [], "tolerance_s": PAIR_TOL_S}, [])
        interval = 0.0
        try:
            interval = float((doc.get("config") or {}).get("interval_s") or 0)
        except (TypeError, ValueError):
            interval = 0.0
        tol = PAIR_TOL_S if interval <= 0 \
            else max(0.05, min(PAIR_TOL_S, interval / 2.0))
        idx_by_file = {r.get("file"): r for r in (doc.get("index") or [])
                       if isinstance(r, dict) and r.get("file")}
        frames.sort(key=lambda f: f[0])
        shots = []
        for ep, cam, fname, src in frames:
            if shots and ep - shots[-1]["t0"] <= tol:
                shots[-1]["have"].setdefault(cam, fname)
                shots[-1]["t1"] = ep
            else:
                shots.append({"t0": ep, "t1": ep, "have": {cam: fname}})

        def shot_view(s):
            members = {c: idx_by_file.get(f) for c, f in s["have"].items()}
            idx_eps = {c: r["epoch"] for c, r in members.items()
                       if r and r.get("epoch") is not None}
            srcs = {c: r.get("src") for c, r in members.items() if r}
            if len(idx_eps) == len(s["have"]) and len(idx_eps) >= 1:
                spread = (max(idx_eps.values()) - min(idx_eps.values())) * 1000
                spread_src = "index"
            else:
                spread = (s["t1"] - s["t0"]) * 1000
                spread_src = "flight_log"
            out = {"epoch": round(s["t0"], 3), "files": dict(s["have"]),
                   "missing": [c for c in cams if c not in s["have"]],
                   "spread_ms": round(spread, 2), "spread_src": spread_src,
                   "srcs": srcs}
            strobe_ep = next((r["strobe"] for r in members.values()
                              if r and r.get("strobe") is not None), None)
            if strobe_ep is not None:
                windows = [(r["epoch"], r["rise"]) for r in members.values()
                           if r and r.get("epoch") is not None
                           and r.get("rise") is not None]
                measured = (not out["missing"]
                            and len(windows) == len(s["have"])
                            and all(v == "gpio_edge" for v in srcs.values())
                            and len(srcs) == len(s["have"]))
                if measured:
                    lo = max(w[0] for w in windows)
                    hi = min(w[1] for w in windows)
                    ok = lo <= strobe_ep <= hi
                    margin = min(strobe_ep - lo, hi - strobe_ep) * 1000
                    out["strobe"] = {"epoch": round(strobe_ep, 6), "ok": ok,
                                     "margin_ms": round(margin, 2)}
                else:
                    out["strobe"] = {"epoch": round(strobe_ep, 6), "ok": None,
                                     "margin_ms": None}
            return out

        full = [shot_view(s) for s in shots]
        gaps, complete = [], 0
        for i, (s, v) in enumerate(zip(shots, full)):
            if not v["missing"]:
                complete += 1
            elif len(gaps) < GAPS_MAX:
                gaps.append({"i": i, "epoch": v["epoch"],
                             "have": sorted(s["have"]),
                             "missing": v["missing"],
                             "files": v["files"],
                             "spread_ms": v["spread_ms"]})
        strobe_missed = sum(1 for v in full
                            if (v.get("strobe") or {}).get("ok") is False)
        summary = {"cams": cams, "shots": len(shots), "complete": complete,
                   "incomplete": len(shots) - complete,
                   # The tail of the transect, with the actual filenames, so
                   # the browser can put the pair on screen side by side -
                   # during a live survey that is "show me what the last shot
                   # looks like on both cameras", which is the check nobody can
                   # do from a file listing.
                   "recent": full[-12:],
                   "strobe_missed": strobe_missed,
                   "gaps": gaps, "gaps_truncated": len(shots) - complete
                   > len(gaps), "tolerance_s": round(tol, 3),
                   # Every shot as one character, oldest first: '.' complete,
                   # a digit = how many cameras were missing. Cheap enough to
                   # send on every poll; draws the whole transect at a glance.
                   "strip": "".join(
                       "." if len(s["have"]) == len(cams)
                       else str(min(9, len(cams) - len(s["have"])))
                       for s in shots[-4000:])}
        return summary, full

    def frame_path(self, run_id, cam, name):
        path = self.child(run_id, cam, name)
        if os.path.splitext(name)[1].lower() not in FRAME_EXTS:
            raise RunsError("not a frame")
        if not os.path.isfile(path):
            raise RunsError("no such frame")
        if os.path.getsize(path) > FRAME_MAX_BYTES:
            raise RunsError("frame too large to serve")
        return path

    def flight_log(self, run_id, cam):
        """Raw CSV bytes, bounded. Served as the file on disk, not a
        re-serialisation of it: the flight_log IS the deliverable."""
        path = self.child(run_id, cam, "flight_log.csv")
        if not os.path.isfile(path):
            raise RunsError("no flight_log for that camera")
        try:
            with open(path, "rb") as fh:
                data = fh.read(FLIGHT_MAX_BYTES + 1)
        except OSError as e:
            raise RunsError(str(e))
        if len(data) > FLIGHT_MAX_BYTES:
            return data[:FLIGHT_MAX_BYTES], True
        return data, False


def _count_by(values):
    out = {}
    for v in values:
        k = (v or "").strip() or "(empty)"
        out[k] = out.get(k, 0) + 1
    return out


def _count_files(path):
    try:
        return sum(1 for f in os.listdir(path)
                   if os.path.splitext(f)[1].lower() in FRAME_EXTS)
    except OSError:
        return None

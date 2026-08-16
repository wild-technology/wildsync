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

import json
import os
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
    {"name": "cam3", "cam_num": 3, "host": "192.168.1.203"},
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

    def __init__(self, path=RIGD_LOG, ring=5000):
        self.path = path
        self._ring = deque(maxlen=ring)
        self._seq = 0
        self._lock = threading.Lock()
        self._run_fh = None            # optional per-run events.log
        os.makedirs(os.path.dirname(path), exist_ok=True)

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
            try:
                with open(self.path, "a") as fh:
                    fh.write(line + "\n")
            except OSError:
                pass
            if self._run_fh:
                try:
                    self._run_fh.write("%s  %-8s %-18s %s%s\n" % (
                        time.strftime("%H:%M:%S", time.gmtime(rec["ts"])),
                        sev.upper(), kind, (("[%s] " % node) if node else ""),
                        msg))
                    self._run_fh.flush()
                except OSError:
                    pass
        return rec

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
            data = r.read(cap + 1)
        if len(data) > cap:
            return None, "frame exceeds %d bytes" % cap
        return data, None
    except Exception as e:  # noqa: BLE001
        return None, str(e)


# ---------------------------------------------------------------------------
# Per-node monitor: OFFLINE -> REACHABLE -> CAM_CONNECTED, with auto-reconnect.
# ---------------------------------------------------------------------------
class NodeMonitor(threading.Thread):
    OFFLINE = "OFFLINE"
    REACHABLE = "REACHABLE"          # piagent/ilxctl answer, camera not claimed
    CONNECTED = "CAM_CONNECTED"      # camera claimed and controllable

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
            self.state = new

    def run(self):
        while not self._stopev.wait(self.poll):
            try:
                self._tick()
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "monitor", "tick error: %s" % e,
                                 node=self.name_)

    def _tick(self):
        health = http_json(self.pia + "/health", timeout=4)
        status = http_json(self.ilx + "/api/status", timeout=6)
        reachable = not status.get("_unreachable")
        pia_ok = not health.get("_unreachable")
        with self._lock:
            self.health = {} if health.get("_unreachable") else health
            if reachable:
                self.status = status
                self.last_seen = time.time()
        if not reachable and not pia_ok:
            self._set_state(self.OFFLINE)
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
                    self._backoff = min(self._backoff * 1.6, 60.0)
                    self._connect_after = now + self._backoff
                    self.events.emit("warn", "reconnect",
                                     "connect failed: %s" % r.get("error"),
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
}

DEFAULT_DESIRED = {
    "aperture": 800,                 # f/8.0  (f x100)
    "shutter": shutter_encode(1, 200),
    "iso": 400,
    "expcomp": 0,
    "drive": DRIVE_SINGLE,
    "focus_mode": 1,                 # MF for a fixed survey rig
    "filetype": 1,                   # JPEG
    "imagesize": 1,                  # L
    "transsize": 1,                  # Small — fast review pulls
    "store_dest": 3,                 # both card + PC
}


class SettingsManager:
    def __init__(self, monitors, events):
        self.monitors = monitors
        self.events = events
        self._lock = threading.Lock()
        self._load_note = None       # set by _load() when it fell back
        self.desired = self._load()
        self._auto = False           # exposure servo off by default (manual)
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
        with self._lock:
            return dict(self.desired, _auto=self._auto)

    def set_auto(self, on):
        with self._lock:
            self._auto = bool(on)
        self.events.emit("info", "settings", "auto-exposure %s"
                         % ("on" if on else "off"))

    def update(self, changes):
        """User intent: mutate desired, then force an immediate reconcile."""
        applied = {}
        with self._lock:
            for k, v in changes.items():
                if k in DEFAULT_DESIRED or k in ("expcomp", "focus_mode"):
                    self.desired[k] = v
                    applied[k] = v
            self._save()
        if applied:
            self.events.emit("info", "settings", "desired updated: %s"
                             % ", ".join("%s=%s" % kv for kv in applied.items()))
        self.reconcile_all(force=True)
        return applied

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

    def reconcile_all(self, force=False):
        for m in self.monitors:
            if m.is_connected():
                try:
                    self._reconcile_node(m, force)
                except Exception as e:  # noqa: BLE001
                    self.events.emit("error", "settings",
                                     "reconcile error: %s" % e, node=m.name_)

    def _reconcile_node(self, m, force):
        with self._lock:
            want = dict(self.desired)
        status = m.snapshot()["status"]
        diverged = []
        # Some fields have no readback key on this body, so "have == target" can
        # never be true for them and they were re-pushed on every single pass -
        # four SetDeviceProperty writes per camera every 3 s, forever, over a USB
        # link we also want to pull frames across. Remember what was last pushed
        # and skip when nothing changed; a body reset clears it via `force`.
        pushed = getattr(m, "_pushed", None)
        if pushed is None or force:
            pushed = m._pushed = {}
        for field, (which, key) in CONVERGE_FIELDS.items():
            target = want.get(field)
            if target is None:
                continue
            have = status.get(key) if key else None
            if key and have == target and not force:
                continue
            if not key and pushed.get(field) == target and not force:
                continue                       # blind field, already at target
            if field == "store_dest":
                if have != target:
                    r = m.set_store(target)
                    if r.get("ok") is False:
                        diverged.append(field)
                continue
            if which:
                r = m.set_exposure(which, target)
                if r.get("ok") is False and not r.get("_unreachable"):
                    diverged.append(field)
                elif not key:
                    pushed[field] = target
        # expcomp has no readback key either, so it gets the same treatment
        # rather than an unconditional write on every reconcile.
        want_ev = want.get("expcomp", 0)
        if pushed.get("expcomp") != want_ev or force:
            r = m.set_exposure("expcomp", want_ev)
            if r.get("ok") is False and not r.get("_unreachable") \
                    and "InvalidCalled" not in str(r.get("error", "")):
                diverged.append("expcomp")
            else:
                pushed["expcomp"] = want_ev
        # focus mode.
        if want.get("focus_mode") is not None:
            if status.get("focusMode") != want["focus_mode"] or force:
                m.set_focus_mode(want["focus_mode"])
        # Verify the readable ones settled. Read the CAMERA, not m.snapshot():
        # that returns the cached status dict the diff was computed from, which
        # only refreshes on the 2 s poll. Confirming against it means every user
        # settings change raises a spurious settings_divergent and flashes the
        # UI badge for a change that in fact applied cleanly.
        time.sleep(0.15)
        after = http_json(m.ilx + "/api/status", timeout=8)
        if not isinstance(after, dict) or after.get("_unreachable"):
            return                                  # offline: not a divergence
        still = []
        for field, (which, key) in CONVERGE_FIELDS.items():
            if key and want.get(field) is not None \
                    and after.get(key) != want[field]:
                still.append(field)
        # A property the body refuses to expose for writing (a manual-aperture
        # lens, say) will never converge, and re-alarming every cycle trains the
        # operator to ignore the alert. Report it once, distinctly.
        writable = after.get("writable") or {}
        unsettable = [f for f in still
                      if writable.get(CONVERGE_FIELDS[f][0] or f) == 2]
        synced = not still
        with m._lock:
            m.convergence = {"synced": synced, "diverged": still,
                             "unsettable": unsettable,
                             "last_check": time.time()}
        if not synced:
            # Require two consecutive failures before alarming: one poll can
            # race a setting the body is still applying.
            m._diverge_strikes = getattr(m, "_diverge_strikes", 0) + 1
            if m._diverge_strikes >= 2:
                if unsettable:
                    self.events.emit(
                        "warn", "settings_unsettable",
                        "%s cannot be set over USB on this body (body-only "
                        "control, e.g. a manual aperture ring)"
                        % ",".join(unsettable), node=m.name_, fields=unsettable)
                rest = [f for f in still if f not in unsettable]
                if rest:
                    self.events.emit("warn", "settings_divergent",
                                     "fields not converged: %s" % ",".join(rest),
                                     node=m.name_, fields=rest)
        else:
            m._diverge_strikes = 0


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

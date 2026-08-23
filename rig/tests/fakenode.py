#!/usr/bin/env python3
"""fakenode — an in-process stand-in for one Wild Sync camera node.

Speaks both node APIs from rig/PROTOCOL.md on a loopback alias:

    ilxctl   http://127.0.0.x:8080   settings, shots, frame bytes, live view
    piagent  http://127.0.0.x:8081   health, GPIO edges, IMU, time probe

The ports are fixed at 8080/8081 because rig/run.py and rig/rigd.py hard-code
them; a distinct 127.0.0.x address per node is what makes several fakes able to
coexist. Nothing here ever binds a routable address, so a harness built on this
module cannot reach the real fleet even by accident.

Fidelity that matters
---------------------
* `/api/status` carries BOTH shapes for every settable field: the human label
  under the bare name (`"iso": "ISO 400"`) and the Sony raw number under
  `"<name>Value"` (`"isoValue": 400`). Code that converges on the label instead
  of the raw number is wrong and this fake makes that wrongness visible.
* Settings only move when something moves them: a push through /api/exposure,
  a `drift()` (hand-nudged body), or a reboot to factory defaults.
* Frames are real JPEGs with real EXIF DateTimeOriginal/SubSecTimeOriginal, so
  the EXIF capture-time path is exercised for real rather than stubbed.

Faults it can wear (per surface: "ilx", "pia", or "both")
---------------------------------------------------------
    hang_s        every response sleeps this long first (slow/hung node)
    http500       every response is a 500 with an error body
    badjson       every response is syntactically invalid JSON
    refuse        socket listener taken down (connection refused)
    connect_lies  POST /api/connect answers {ok:true} but the camera never claims
    truncate      /shot/<n> returns the first half of the JPEG, honestly framed
    cut           /shot/<n> promises the full length then closes the socket early
    vanish        /shot/<n> 404s although /api/shots still lists the frame
    fail_set      /api/exposure rejects with ok:false (field that will not take)

Standalone:  python3 rig/fakenode.py [--host 127.0.0.2]  (Ctrl-C to stop)
"""

import io
import json
import os
import random
import socket
import threading
import time
from collections import Counter, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote

ILX_PORT = 8080
PIA_PORT = 8081

# Linux binds any 127.0.0.0/8 address for free; macOS configures 127.0.0.1
# alone and raises EADDRNOTAVAIL for the rest. When the alias cannot be bound,
# every fake maps its nominal (127.0.0.x, port) to a deterministic port on
# 127.0.0.1 instead. loopback_map() is the single source of that mapping; the
# soaktest netguard applies the identical rewrite to outgoing URLs, so rig code
# keeps speaking the fixed-port addresses PROTOCOL.md documents on every OS.
_ALIAS_BINDABLE = None


def _alias_bindable():
    global _ALIAS_BINDABLE
    if _ALIAS_BINDABLE is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(("127.0.0.2", 0))
            _ALIAS_BINDABLE = True
        except OSError:
            _ALIAS_BINDABLE = False
        finally:
            s.close()
    return _ALIAS_BINDABLE


def loopback_map(host, port):
    """The (host, port) to actually bind/connect for a nominal 127.0.0.x:port
    node address: identity where loopback aliases exist, else 127.0.0.1 with a
    port unique to (pid, x, port). The pid term keeps two test PROCESSES (a
    lingering previous run, a devrig next to a soaktest) off each other's
    ports; within one process every suite maps the same nominal address to the
    same port, which is what lets the client-side URL rewrite stay stateless."""
    if not host.startswith("127.") or host == "127.0.0.1" or _alias_bindable():
        return host, port
    x = int(host.rsplit(".", 1)[1])
    return "127.0.0.1", 20000 + (os.getpid() % 40) * 640 \
        + 2 * x + (port - ILX_PORT)

# Sony encodings — PROTOCOL.md "Measured constants".
SHUTTER_1_200 = (1 << 16) | 200          # 65736
ISO_AUTO = 16777215

# What a body comes up in after a power cycle: nothing like the survey vector.
FACTORY = {
    "aperture": 560,                      # f/5.6
    "shutter": (1 << 16) | 60,            # 1/60
    "iso": ISO_AUTO,
    "drive": 1,
    "filetype": 3,                        # RAW+JPEG
    "imagesize": 1,
    "transsize": 0,                       # Original
    "store_dest": 2,                      # card only — PC save off after boot
    "focus_mode": 2,                      # AF-S
    "expcomp": 0,
}

SURVEY = {                                # a body already set up for the transect
    "aperture": 800, "shutter": SHUTTER_1_200, "iso": 400, "drive": 1,
    "filetype": 1, "imagesize": 1, "transsize": 1, "store_dest": 3,
    "focus_mode": 1, "expcomp": 0,
    # A fresh body boots in AWB (0); the rig converges it to fixed color
    # temperature (mode 256) per DEFAULT_DESIRED.
    "wb_mode": 0, "colortemp": 5500,
}

# which= name used by /api/exposure  ->  internal settings key
WHICH_TO_KEY = {
    "aperture": "aperture", "shutter": "shutter", "iso": "iso",
    "drive": "drive", "filetype": "filetype", "imagesize": "imagesize",
    "quality": "quality", "transsize": "transsize", "pcsave": "pcsave",
    "rawtype": "rawtype", "expcomp": "expcomp",
    "wb_mode": "wb_mode", "colortemp": "colortemp",
}


def _iso_label(v):
    return "ISO AUTO" if v == ISO_AUTO else "ISO %d" % v


def _shutter_label(v):
    num, den = (v >> 16) & 0xFFFF, v & 0xFFFF
    if den in (0, 1):
        return "%d\"" % num
    return "%d/%d" % (num, den)


def _aperture_label(v):
    return "F%.1f" % (v / 100.0)


def _drive_label(v):
    return {1: "Single", 65540: "Continuous Lo"}.get(v, "Drive %d" % v)


# ---------------------------------------------------------------------------
# JPEG frames — small but genuine, with the EXIF fields run.py reads.
# ---------------------------------------------------------------------------
def make_jpeg(epoch, size=(48, 36), label=None):
    """A real JPEG whose EXIF DateTimeOriginal/SubSec encode `epoch`.

    The stamp is written in LOCAL time because run._exif_capture_epoch parses it
    back with time.mktime (local). A camera whose clock is off is modelled by
    passing an already-skewed epoch, not by changing the encoding.
    """
    try:
        from PIL import Image
    except Exception:                                   # noqa: BLE001
        return _fallback_jpeg(epoch)
    im = Image.new("RGB", size, (random.randint(20, 200), 40, 60))
    try:
        ex = Image.Exif()
        ex[0x8769] = {
            0x9003: time.strftime("%Y:%m:%d %H:%M:%S", time.localtime(epoch)),
            0x9291: "%02d" % int(round((epoch % 1) * 100)) if epoch % 1 < 0.995
                    else "99",
        }
        ex[0x010F] = "SONY"
        ex[0x0110] = "ILX-LR1"
        if label:
            ex[0x010E] = str(label)
        buf = io.BytesIO()
        im.save(buf, "JPEG", exif=ex.tobytes(), quality=60)
        return buf.getvalue()
    except Exception:                                   # noqa: BLE001
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=60)
        return buf.getvalue()


def _fallback_jpeg(epoch):
    """Minimum viable JPEG (SOI ... EOI) for hosts without Pillow."""
    return (b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01"
            b"\x00\x00" + ("%.3f" % epoch).encode().ljust(64, b"\x00")
            + b"\xff\xd9")


# ---------------------------------------------------------------------------
class FakeNode:
    """One camera node: two HTTP surfaces, one settings vector, fault knobs."""

    def __init__(self, name, host, cam_num=3, connected=True, survey=True,
                 has_imu=False, has_gpio=True, clock_skew_s=0.0):
        self.name = name
        self.host = host
        self.cam_num = cam_num
        self.model = "ILX-LR1"
        self.id = "D516000F46%02X" % (0x70 + cam_num)
        self.has_imu = has_imu
        self.has_gpio = has_gpio
        self.clock_skew_s = clock_skew_s      # camera clock - true clock

        self._lock = threading.RLock()
        self.connected = bool(connected)
        self.settings = dict(SURVEY if survey else FACTORY)
        self.shots = []                       # [{name,size,bytes,epoch}]
        self._shot_seq = 0
        self.edges = deque(maxlen=5000)       # (seq, edge, epoch, fire_seq)
        self._edge_seq = 0
        self._fire_seq = 0
        self.strobe_fires = 0
        self.strobe_last = None               # epoch of the last strobe pulse
        self.imu = deque(maxlen=6000)
        self.boot_epoch = time.time()
        self.disk_free_mb = 40000
        self.battery = 87
        self.focus_held = False
        # PC-save off = the body still exposes, but no frame ever reaches the
        # Pi's save dir. That is cam3's real state whenever the body's PC-save
        # menu or PC control priority is lost (see PROTOCOL.md + field notes).
        self.pc_save = True
        # A synthetic card for the drain path: contentId -> {jpeg, raw bytes}.
        # Populated lazily; each fired frame also lands a RAW here.
        self.card = {}
        self._card_seq = 40000
        self.card_mode = "remote"
        self.log = ["ilxctl up", "USB device found"]
        # Force chosen /api/status keys to a fixed value, so a status whose
        # human label disagrees with its raw number can be staged on demand.
        self.label_override = {}
        # Keys REMOVED from /api/status entirely — simulates an ilxctl build
        # that predates a field (e.g. whiteBalance/colorTemp before
        # 2026-08-20). The reconcile loop must skip those quietly.
        self.hide_keys = set()

        self.counts = Counter()               # "GET /api/status" -> n
        self.call_log = deque(maxlen=4000)    # (epoch, surface, method, path)
        self.pushes = deque(maxlen=4000)      # (epoch, which, value) accepted or not
        self.faults = {"ilx": self._blank_faults(), "pia": self._blank_faults()}

        self._servers = {}
        self._threads = {}
        self._srv_lock = threading.Lock()   # serialises up()/down()/reboot()
        self.up()

    # ---- fault knobs ------------------------------------------------------
    @staticmethod
    def _blank_faults():
        return {"hang_s": 0.0, "http500": False, "badjson": False,
                "connect_lies": False, "truncate": False, "cut": False,
                "vanish": False, "fail_set": None, "flaky": 0.0}

    def set_fault(self, surface="both", **kw):
        surfaces = ("ilx", "pia") if surface == "both" else (surface,)
        with self._lock:
            for s in surfaces:
                for k, v in kw.items():
                    if k not in self.faults[s]:
                        raise KeyError("unknown fault %r" % k)
                    self.faults[s][k] = v

    def clear_faults(self):
        with self._lock:
            self.faults = {"ilx": self._blank_faults(),
                           "pia": self._blank_faults()}

    def _fault(self, surface):
        with self._lock:
            return dict(self.faults[surface])

    # ---- lifecycle --------------------------------------------------------
    def up(self):
        """Bring both listeners up (idempotent). A closed node stays closed:
        a reboot thread that outlives close() must not resurrect listeners on
        ports the NEXT suite's Env is about to bind."""
        with self._srv_lock:
            if getattr(self, "_closed", False):
                return
            for surface, port in (("ilx", ILX_PORT), ("pia", PIA_PORT)):
                if surface in self._servers:
                    continue
                srv = _mkserver(*loopback_map(self.host, port))
                srv.node = self
                srv.surface = surface
                t = threading.Thread(target=srv.serve_forever,
                                     kwargs={"poll_interval": 0.05},
                                     name="fake-%s-%s" % (self.name, surface),
                                     daemon=True)
                t.start()
                self._servers[surface] = srv
                self._threads[surface] = t

    def down(self):
        """Take both listeners away — clients see connection refused."""
        with self._srv_lock:
            for surface in list(self._servers):
                srv = self._servers.pop(surface)
                t = self._threads.pop(surface, None)
                srv.shutdown()
                srv.server_close()
                if t:
                    t.join(timeout=3)

    def reboot(self, down_s=1.0, factory=True, keep_shots=False, block=True):
        """Power-cycle: the camera AND the Pi go (shared PoE feed), then the
        node comes back with a freshly-booted body and a fresh piagent."""
        def _go():
            self.down()
            time.sleep(max(0.0, down_s))
            with self._lock:
                self.boot_epoch = time.time()
                self.connected = False
                if factory:
                    self.settings = dict(FACTORY)
                if not keep_shots:
                    self.shots = []
                    self._shot_seq = 0
                self.edges.clear()
                self._edge_seq = 0
                self._fire_seq = 0
                self.focus_held = False
                self.log = ["ilxctl up", "USB device found"]
            self.up()
            # ilxctl claims the camera again a beat after the service is up.
            time.sleep(0.2)
            with self._lock:
                self.connected = True
        if block:
            _go()
        else:
            threading.Thread(target=_go, daemon=True,
                             name="fake-reboot-%s" % self.name).start()

    def close(self):
        self._closed = True     # before down(): reboot threads check it in up()
        self.down()

    # ---- state the harness drives ----------------------------------------
    def add_frame(self, epoch=None, name=None, exif=True, size_bytes=None):
        """Land a new frame in the PC-save dir, as a capture would.

        With `save_delay_s` > 0 the frame lands that much later, the way a
        real body takes ~0.5-1.5 s of card write + PC-save before the file
        appears in the spool. The stop-grace barrier exists for exactly that
        window; instant delivery can never reproduce the race."""
        delay = getattr(self, "save_delay_s", 0)
        if delay > 0:
            t = threading.Timer(delay, self._add_frame_now,
                                args=(epoch or time.time(), name, exif,
                                      size_bytes))
            t.daemon = True
            t.start()
            return None
        return self._add_frame_now(epoch, name, exif, size_bytes)

    def _add_frame_now(self, epoch=None, name=None, exif=True,
                       size_bytes=None):
        with self._lock:
            self._shot_seq += 1
            ep = time.time() if epoch is None else epoch
            nm = name or "ILX%05d.JPG" % self._shot_seq
            data = (make_jpeg(ep + self.clock_skew_s) if exif
                    else _fallback_jpeg(ep))
            if size_bytes:
                data = (data * (size_bytes // len(data) + 1))[:size_bytes]
            self.shots.append({"name": nm, "size": len(data), "bytes": data,
                               "epoch": ep})
            # The same shot also lands a JPEG+RAW pair on the card (one
            # contentId), so the drain path has something to pull.
            self._card_seq += 1
            cid = self._card_seq
            base = "_CA%05d" % cid
            raw = (_fallback_jpeg(ep) * 40)[:200000]      # a stand-in "RAW"
            self.card[cid] = {1: (base + ".JPG", data, 0x3801),
                              2: (base + ".ARW", raw, 0xB101)}
            return nm

    def push_edge(self, epoch=None, edge="fall", fire_seq=None):
        with self._lock:
            self._edge_seq += 1
            self.edges.append((self._edge_seq, edge,
                               time.time() if epoch is None else epoch,
                               fire_seq))
            return self._edge_seq

    def push_imu(self, epoch=None, **over):
        s = {"epoch": time.time() if epoch is None else epoch,
             "pitch": 1.25, "roll": -0.5, "yaw": 210.0, "heading": 211.5,
             "ax": 0.01, "ay": -0.02, "az": 0.98,
             "gx": 0.1, "gy": -0.2, "gz": 0.05,
             "mx": 12.0, "my": -3.0, "mz": 40.0, "temp": 31.5}
        s.update(over)
        with self._lock:
            self.imu.append(s)
        return s

    def drift(self, **fields):
        """The body wanders off desired — a hand-nudged dial or a reboot."""
        with self._lock:
            if not fields:
                fields = {"iso": 3200, "shutter": (1 << 16) | 60}
            self.settings.update(fields)
            return dict(self.settings)

    def raw(self, key):
        with self._lock:
            return self.settings.get(key)

    def raw_all(self):
        with self._lock:
            return dict(self.settings)

    def count(self, key):
        with self._lock:
            return self.counts[key]

    def clear_counts(self):
        with self._lock:
            self.counts.clear()
            self.call_log.clear()
            self.pushes.clear()

    def calls_since(self, epoch, path=None):
        with self._lock:
            return [c for c in self.call_log
                    if c[0] >= epoch and (path is None or c[3] == path)]

    # ---- response builders ------------------------------------------------
    def status(self):
        with self._lock:
            s = self.settings
            doc = {
                "connected": self.connected,
                "model": self.model if self.connected else "",
                "id": self.id if self.connected else "",
                # label form (what the UI prints) ...
                "iso": _iso_label(s["iso"]),
                "shutter": _shutter_label(s["shutter"]),
                "aperture": _aperture_label(s["aperture"]),
                "fnum": _aperture_label(s["aperture"]),
                "drive": _drive_label(s["drive"]),
                "program": "M",
                # ... and raw Sony encodings alongside
                "isoValue": s["iso"],
                "shutterValue": s["shutter"],
                "apertureValue": s["aperture"],
                "driveValue": s["drive"],
                "fileType": s["filetype"], "filetypeValue": s["filetype"],
                "imageSize": s["imagesize"], "imagesizeValue": s["imagesize"],
                "transSize": s["transsize"], "transsizeValue": s["transsize"],
                "storeDest": s["store_dest"],
                "focusMode": s["focus_mode"],
                "expcompValue": s["expcomp"],
                # .get: tests inject partial settings dicts (factory-reset
                # scenarios predate these keys); a real body always reports.
                "whiteBalance": s.get("wb_mode", 0),
                "colorTemp": s.get("colortemp", 5500),
                "battery": self.battery,
                "remainShots": 1200,
                "slotStatus": "OK",
                "focusPosition": 500, "zoomPosition": 0,
                "interval": {"running": False, "shots": 0},
                "isoChoices": [100, 200, 400, 800, 1600, 3200, ISO_AUTO],
                "apertureChoices": [400, 560, 800, 1100],
                "shutterChoices": [SHUTTER_1_200, (1 << 16) | 60,
                                   (1 << 16) | 30],
                "log": list(self.log)[-20:],
            }
            if not self.connected:
                doc["iso"] = doc["shutter"] = doc["aperture"] = ""
            doc.update(self.label_override)
            for k in self.hide_keys:
                doc.pop(k, None)
            return doc

    def health(self):
        with self._lock:
            imu_age = (round(time.time() - self.imu[-1]["epoch"], 3)
                       if self.imu else None)
            return {
                "node": self.name,
                "uptime_s": round(time.time() - self.boot_epoch, 1),
                "gpio": {"chip": "gpiochip4", "available": self.has_gpio,
                         "ok": self.has_gpio, "focus_held": self.focus_held,
                         "monitor_running": self.has_gpio,
                         "interval": {"running": False, "fired": 0},
                         "edges_seen": self._edge_seq,
                         "strobe": {"bcm": 26, "claimed": self.strobe_fires > 0,
                                    "fires": self.strobe_fires,
                                    "last_epoch": self.strobe_last,
                                    "error": None}},
                "imu": {"present": self.has_imu,
                        "rate_hz": 50 if self.has_imu else None,
                        "age_s": imu_age},
                "disk_free_mb": self.disk_free_mb,
                "cam_frames": len(self.shots),
                "load1": 0.42,
                "time": {"epoch": time.time(), "source": "local"},
            }

    def pushed(self, *which):
        with self._lock:
            return [p for p in self.pushes if not which or p[1] in which]

    def apply_exposure(self, which, value):
        with self._lock:
            self.pushes.append((time.time(), which, value))
        f = self._fault("ilx")
        bad = f.get("fail_set")
        if bad and (bad is True or which in
                    (bad if isinstance(bad, (list, tuple, set)) else [bad])):
            return {"ok": False, "error": "SetDeviceProperty failed: InvalidCalled"}
        key = WHICH_TO_KEY.get(which)
        if key is None:
            return {"ok": False, "error": "unknown which %r" % which}
        with self._lock:
            if not self.connected:
                return {"ok": False, "error": "camera not connected"}
            self.settings[key] = int(value)
        return {"ok": True, "which": which, "value": int(value)}


# ---------------------------------------------------------------------------
def _mkserver(host, port):
    class _Srv(ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True
        address_family = socket.AF_INET
    last = None
    for _ in range(50):                 # TIME_WAIT after a reboot cycle
        try:
            return _Srv((host, port), _Handler)
        except OSError as e:
            last = e
            time.sleep(0.1)
    raise last


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "fakenode/1.0"

    def log_message(self, *a):
        pass

    # -- plumbing ----------------------------------------------------------
    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self._send_raw(body, "application/json", code)

    def _send_raw(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_cut(self, body, ctype):
        """Promise the whole frame, deliver half, hang up: a link that dies
        mid-transfer."""
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body[:max(1, len(body) // 2)])
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        self.close_connection = True

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

    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")

    def _dispatch(self, method):
        node = self.server.node
        surface = self.server.surface
        path = self.path.split("?", 1)[0]
        query = parse_qs(self.path.split("?", 1)[1]) if "?" in self.path else {}
        body = self._read_body() if method == "POST" else {}
        with node._lock:
            node.counts["%s %s" % (method, path)] += 1
            node.counts[surface] += 1
            node.call_log.append((time.time(), surface, method, path))
        f = node._fault(surface)
        if f["hang_s"]:
            time.sleep(f["hang_s"])
        if f["flaky"] and random.random() < f["flaky"]:
            self._send_json({"ok": False, "error": "transient"}, 503)
            return
        if f["http500"]:
            self._send_json({"ok": False, "error": "internal error"}, 500)
            return
        if f["badjson"]:
            self._send_raw(b'{"connected": tru, "iso": }', "application/json")
            return
        try:
            if surface == "ilx":
                self._ilx(node, method, path, query, body, f)
            else:
                self._pia(node, method, path, query, body, f)
        except Exception as e:                          # noqa: BLE001
            self._send_json({"ok": False, "error": "fake handler: %s" % e}, 500)

    # -- ilxctl ------------------------------------------------------------
    def _ilx(self, node, method, path, query, body, f):
        if method == "GET" and path == "/api/status":
            self._send_json(node.status())
        elif method == "GET" and path == "/api/shots":
            with node._lock:
                self._send_json([{"name": s["name"], "size": s["size"]}
                                 for s in node.shots])
        elif method == "GET" and path.startswith("/shot/"):
            name = unquote(path[len("/shot/"):])
            if (query.get("dir") or [""])[0] == "raw":
                with node._lock:
                    hit = next((data for c in node.card.values()
                                for (nm, data, fmt) in c.values() if nm == name), None)
                if hit is None:
                    self._send_json({"ok": False, "error": "no such frame"}, 404); return
                self._send_raw(hit, "image/jpeg"); return
            with node._lock:
                shot = next((s for s in node.shots if s["name"] == name), None)
            if shot is None or f["vanish"]:
                self._send_json({"ok": False, "error": "no such frame"}, 404)
                return
            data = shot["bytes"]
            if f["cut"]:
                self._send_cut(data, "image/jpeg")
            elif f["truncate"]:
                self._send_raw(data[:max(1, len(data) // 2)], "image/jpeg")
            else:
                self._send_raw(data, "image/jpeg")
        elif method == "GET" and path == "/liveview.jpg":
            self._send_raw(make_jpeg(time.time()), "image/jpeg")
        elif method == "POST" and path == "/api/card/mode":
            with node._lock:
                node.card_mode = (body or {}).get("mode", "remote")
            self._send_json({"ok": True, "mode": node.card_mode})
        elif method == "GET" and path == "/api/card/ready":
            with node._lock:
                self._send_json({"ok": True, "ready": node.card_mode == "transfer",
                                 "mode": node.card_mode})
        elif method == "GET" and path == "/api/card/list":
            with node._lock:
                if node.card_mode != "transfer":
                    self._send_json({"ok": False, "error": "InvalidCalled 0x8402"}, 503); return
                files = []
                for cid, c in sorted(node.card.items()):
                    for fid, (nm, data, fmt) in c.items():
                        files.append({"contentId": cid, "fileId": fid, "name": nm,
                                      "size": len(data), "format": fmt,
                                      "fileNumber": cid, "dirNumber": 100,
                                      "captured_utc": 1787000000})
            self._send_json({"ok": True, "count": len(files), "files": files})
        elif method == "POST" and path == "/api/card/pull":
            cid = int((body or {}).get("contentId", -1)); fid = int((body or {}).get("fileId", -1))
            with node._lock:
                ent = node.card.get(cid, {}).get(fid)
            if not ent:
                self._send_json({"ok": False, "error": "content not found"}, 503); return
            nm, data, fmt = ent
            import hashlib
            self._send_json({"ok": True, "name": nm, "bytes": len(data),
                             "sha256": hashlib.sha256(data).hexdigest(), "secs": 0.01})
        elif method == "POST" and path == "/api/card/delete":
            if (body or {}).get("confirm") != "delete":
                self._send_json({"ok": False, "error": "confirm"}, 400); return
            cid = int((body or {}).get("contentId", -1))
            with node._lock:
                existed = node.card.pop(cid, None)
            self._send_json({"ok": bool(existed), "deleted": cid})
        elif method == "POST" and path == "/api/shots/delete":
            if (body or {}).get("confirm") != "delete":
                self._send_json({"ok": False, "error": "confirm"}, 400); return
            self._send_json({"ok": True})
        elif method == "POST" and path == "/api/connect":
            if f["connect_lies"]:
                # ilxctl reports success, but PC-remote priority is not actually
                # held, so the body never shows up as claimed.
                self._send_json({"ok": True, "model": node.model})
                return
            with node._lock:
                node.connected = True
            self._send_json({"ok": True, "model": node.model})
        elif method == "POST" and path == "/api/disconnect":
            with node._lock:
                node.connected = False
            self._send_json({"ok": True})
        elif method == "POST" and path == "/api/exposure":
            self._send_json(node.apply_exposure(body.get("which"),
                                                body.get("value")))
        elif method == "POST" and path == "/api/store":
            with node._lock:
                node.settings["store_dest"] = int(body.get("dest", 3))
            self._send_json({"ok": True})
        elif method == "POST" and path == "/api/focus/mode":
            with node._lock:
                node.settings["focus_mode"] = int(body.get("mode", 1))
            self._send_json({"ok": True})
        elif method == "POST" and path in ("/api/focus/drive",
                                           "/api/focus/position",
                                           "/api/zoom/drive",
                                           "/api/zoom/position",
                                           "/api/zoom/setting"):
            self._send_json({"ok": True})
        elif method == "POST" and path in ("/api/shutter", "/api/shutter/hold"):
            with node._lock:
                connected = node.connected
            if not connected:
                self._send_json({"ok": False, "error": "not connected"}, 409)
                return
            if node.pc_save:
                node.add_frame()
            node.push_edge()            # the shutter fires either way
            self._send_json({"ok": True})
        elif method == "POST" and path.startswith("/api/interval"):
            self._send_json({"ok": True})
        else:
            self._send_json({"ok": False, "error": "not found"}, 404)

    # -- piagent -----------------------------------------------------------
    def _pia(self, node, method, path, query, body, f):
        if method == "GET" and path == "/health":
            self._send_json(node.health())
        elif method == "GET" and path == "/imu/latest":
            with node._lock:
                s = node.imu[-1] if node.imu else None
            self._send_json(dict(s) if s else {"present": False})
        elif method == "GET" and path == "/imu/window":
            t0 = float((query.get("t0") or [0])[0])
            t1 = float((query.get("t1") or [time.time()])[0])
            with node._lock:
                sel = [dict(s) for s in node.imu if t0 <= s["epoch"] <= t1]
            self._send_json({"present": node.has_imu, "samples": sel})
        elif method == "GET" and path == "/gpio/exposure/events":
            since = int((query.get("since") or [0])[0])
            # Same shape the real piagent serves: epoch_hw (kernel timestamp
            # converted to wall time — here identical to epoch) and the
            # fire_seq each edge belongs to, so identity-based pairing and the
            # strobe acceptance windows are exercised for real.
            with node._lock:
                evs = [{"i": i, "edge": ed, "epoch": ep, "epoch_hw": ep,
                        "raw_ts": None, "fire_seq": fs}
                       for (i, ed, ep, fs) in node.edges if i > since]
                nxt = node._edge_seq
            self._send_json({"next": nxt, "events": evs})
        elif method == "GET" and path == "/gpio/state":
            self._send_json(node.health()["gpio"])
        elif method == "POST" and path == "/gpio/focus":
            with node._lock:
                node.focus_held = bool(body.get("hold"))
                held = node.focus_held
            self._send_json({"ok": True, "focus_held": held})
        elif method == "POST" and path == "/gpio/fire":
            with node._lock:
                held = node.focus_held
            # The real piagent asserts FOCUS itself when the caller supplies a
            # per-shot focus_lead_ms; only a lead-less fire needs the hold.
            if not held and not int(body.get("focus_lead_ms") or 0):
                self._send_json({"ok": False, "error": "FOCUS not held"}, 409)
                return
            at = float(body.get("at_epoch") or 0) or time.time()
            # The real piagent busy-waits to its instant on a disciplined
            # clock; honour the schedule (bounded) so exposure windows, strobe
            # instants and late_ms all sit where they would on the rig. A bare
            # time.sleep() lands 1-15 ms late, which is enough to push a fake
            # exposure window past a scheduled strobe — spin the final approach
            # exactly as piagent does.
            if 0 < at - time.time() <= 2.0:
                while True:
                    rem = at - time.time()
                    if rem <= 0:
                        break
                    time.sleep(rem - 0.001 if rem > 0.002 else 0)
            now = time.time()
            with node._lock:
                node._fire_seq += 1
                seq = node._fire_seq
                edge0 = node._edge_seq
            node.add_frame(epoch=now)
            node.push_edge(epoch=now, fire_seq=seq)
            # End of exposure after the body's current shutter duration, so the
            # [fall, rise] window the strobe acceptance intersects is real.
            sh = node.settings.get("shutter") or SHUTTER_1_200
            num, den = (sh >> 16) & 0xFFFF, sh & 0xFFFF
            dur = (num / den) if den else 0.005
            node.push_edge(epoch=now + max(0.001, min(dur, 30.0)),
                           edge="rise", fire_seq=seq)
            resp = {"ok": True, "requested_epoch": at, "actual_epoch": now,
                    "late_ms": round((now - at) * 1000, 2),
                    "fire_seq": seq, "edge_seq": edge0,
                    "node_epoch": time.time()}
            strobe_at = float(body.get("strobe_at_epoch") or 0)
            if strobe_at:
                # The fake pulses exactly on schedule plus 0.2 ms of "hardware":
                # deterministic, so acceptance-window tests can assert on it.
                t = strobe_at + 0.0002
                with node._lock:
                    node.strobe_fires += 1
                    node.strobe_last = t
                resp["strobe_epoch"] = t
                resp["strobe_late_ms"] = 0.2
            self._send_json(resp)
        elif method == "POST" and path == "/gpio/strobe":
            # Strobe with no camera fire: the light must not depend on this
            # node's camera being claimable (see piagent.strobe_only).
            at = float(body.get("at_epoch") or 0)
            if at <= 0:
                self._send_json({"ok": False, "error": "at_epoch required"}, 400)
                return
            if 0 < at - time.time() <= 2.0:
                while True:
                    rem = at - time.time()
                    if rem <= 0:
                        break
                    time.sleep(rem - 0.001 if rem > 0.002 else 0)
            t = at + 0.0002
            with node._lock:
                node.strobe_fires += 1
                node.strobe_last = t
            self._send_json({"ok": True, "strobe_epoch": t,
                             "strobe_late_ms": 0.2})
        elif method == "POST" and path.startswith("/gpio/interval"):
            self._send_json({"ok": True})
        elif method == "POST" and path == "/timeprobe":
            t = time.time()
            self._send_json({"t0": body.get("t0"), "t_rx": t, "t_tx": time.time()})
        else:
            self._send_json({"ok": False, "error": "not found"}, 404)


# ---------------------------------------------------------------------------
def _demo():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.2")
    ap.add_argument("--frames", type=int, default=3)
    a = ap.parse_args()
    n = FakeNode("cam-demo", a.host, cam_num=3, has_imu=True)
    for i in range(a.frames):
        n.add_frame(epoch=time.time() - 10 + i)
        n.push_edge(epoch=time.time() - 10 + i)
        n.push_imu(epoch=time.time() - 10 + i)
    print("fake node on http://%s:%d (ilxctl) and :%d (piagent)"
          % (a.host, ILX_PORT, PIA_PORT))
    print("  curl http://%s:%d/api/status | head -c 400" % (a.host, ILX_PORT))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        n.close()


if __name__ == "__main__":
    _demo()

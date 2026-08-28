#!/usr/bin/env python3
"""Audit regression suite — rigd lane (findings D1..D8).

Every check here FAILS on the pre-fix rigd.py and passes after.

D1  A POST body that is malformed, or valid JSON that is not an object,
    answered exactly like an empty body ({}), so POST /api/run/start with a
    truncated body recorded a REAL transect on the built-in defaults.
D2  Unvalidated inputs: {"on":"maybe"} armed the auto-exposure servo,
    {"value":"abc"} on /api/exposure reached ilxctl and stalled the SDK ~6 s
    until the node flapped, ?since=abc / t0=abc / steps="x" came back 500 with
    a Python exception string, /api/calibrate took samples<1, and the lens
    fanout answered ok:true with empty results for a camera it had skipped.
D3  host_clock_offset did not exist (a common-mode node-vs-host offset is
    invisible to the node-to-node skew check); node_clock_skew fired on ONE
    sample and silently dropped a node whose link was slow.
D4  /api/run/stop reported drain_started:true whatever start_drain did, the
    drain claim was taken under rigd's own lock instead of the run manager's,
    there was no cancel, and a node whose transfer subsystem had wedged was
    re-drained after every single Stop.
D5  The static fix stood in for ANY row without a live fix, so a mid-run
    gateway gap was papered over with a constant position instead of leaving
    the flight_log cells empty and raising nav_no_fix.
D6  pc_control_lost/battery_low were evaluated on a dead node's stale status;
    Anomalies.scan() had no lock so a new anomaly was journalled two or three
    times; _imu_host re-probed a dead IMU for 3 s on every poll.
D9  (review blocker) The static-fix arming latch was a process-global with no
    run identity and two call sites cleared it unconditionally - the 5 s nav
    tick (which sees active:False for the whole of RunManager.start's live
    probe) and the /api/run/start error path (which fired on "run already
    active", i.e. a duplicate Start). A run then had no latch, armed() fell
    back to live gateway state, and a mid-run bus drop wrote the static
    LAUNCH-POINT position into every remaining flight_log row.
D10 The post-drain auto-ingest discarded both its log and its totals, so a
    drain that emptied the card and matched nothing said only "drain done".
D11 want_int leaked OverflowError/ValueError for JSON Infinity/NaN: a 500
    carrying a Python exception string, plus an error line in rigd.jsonl.
D12 The body_locked predicate is satisfied entirely by ABSENCE, so ilxctl's
    degraded "busy" body (PROTOCOL.md: "the node answered and told us
    nothing") was diagnosed as a stalled card - "power the body off and
    reformat the card" - and so was a node under a card drain.
D8  POST /api/reconcile called reconcile_all(force=True) with no exposure=
    decision, and a bare force=True IS an explicit fleet apply — so the manual
    "push the vector now" kick re-pushed the fleet's exposure onto every body
    and silently destroyed a deliberate per-camera split. It must force the
    rest of the vector and leave exposure alone; POST /api/settings is the
    exposure apply and must still propagate.

Hermetic: fakenode fakes on loopback only (soaktest's netguard is installed by
importing it), a throwaway temp home, an rigd HTTP server on an ephemeral
127.0.0.1 port. No sleeps beyond what the 0.3 s monitor poll needs.

Run standalone:  python3 rig/tests/audit_rigd.py
"""

import json
import os
import shutil
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

# Importing soaktest installs the loopback netguard before any rig code runs,
# so nothing in this file can reach 192.168.1.x even by accident.
from soaktest import check, sect, note, wait_for                  # noqa: E402
import rigcore                                                     # noqa: E402
import run as runmod                                               # noqa: E402
import rigd                                                        # noqa: E402
from fakenode import FakeNode, ISO_AUTO                            # noqa: E402


# ---------------------------------------------------------------------------
# Harness: real rigd (Rig + Handler) over two in-process fakes.
# ---------------------------------------------------------------------------
class FastMonitor(rigcore.NodeMonitor):
    """Rig() constructs its monitors with the shipping poll interval; a test
    that has to watch a state transition would spend seconds per check."""

    def __init__(self, node, events, poll=0.3):
        super().__init__(node, events, poll=poll)


class Harness:
    def __init__(self, nav=None, nodes=(("cam1", "127.0.0.2", 1),
                                        ("cam2", "127.0.0.3", 2))):
        self.dir = tempfile.mkdtemp(prefix="wildsync-audit-rigd-")
        rigcore.RIG_HOME = os.path.join(self.dir, "rig")
        rigcore.DESIRED_PATH = os.path.join(rigcore.RIG_HOME, "desired.json")
        rigcore.RIGD_LOG = os.path.join(rigcore.RIG_HOME, "rigd.jsonl")
        rigcore.RUNS_DIR = runmod.RUNS_DIR = os.path.join(self.dir, "runs")
        os.makedirs(rigcore.RIG_HOME, exist_ok=True)
        # Never the operator's real ~/rig or ~/rig-raw.
        self._sf_path = os.path.join(self.dir, "static_fix.json")
        self._saved = {"NODES": rigd.NODES, "MON": rigd.NodeMonitor,
                       "SF": rigd.STATIC_FIX_PATH,
                       "startnav": rigd.Rig._start_nav,
                       "calib": rigd.Rig._startup_calibrate,
                       "loop": rigd.Rig._anomaly_loop,
                       "RIG": rigd.RIG, "http_json": rigcore.http_json}
        rigd.STATIC_FIX_PATH = self._sf_path
        rigd.NODES = [{"name": n, "cam_num": c, "host": h}
                      for n, h, c in nodes]
        rigd.NodeMonitor = FastMonitor
        rigd.Rig._start_nav = lambda s: nav
        rigd.Rig._startup_calibrate = lambda s: None
        # The 2.5 s background scan would interleave with the deterministic
        # scans the anomaly checks drive by hand (the skew streak counts
        # scans). Those checks call scan() themselves.
        rigd.Rig._anomaly_loop = lambda s: None

        self.nodes = {n: FakeNode(n, h, cam_num=c, has_imu=(c == 1))
                      for n, h, c in nodes}
        self.rig = rigd.Rig()
        self.rig.DRAIN_DEST = os.path.join(self.dir, "rig-raw")
        self.rig.DRAIN_FLAG = os.path.join(rigcore.RIG_HOME, "auto_drain")
        open(self.rig.DRAIN_FLAG, "w").close()   # auto-drain on, deterministic
        rigd.RIG = self.rig
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), rigd.Handler)
        self.srv.daemon_threads = True
        self.port = self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    # -- handles ------------------------------------------------------------
    def mon(self, name):
        return next(m for m in self.rig.monitors if m.name_ == name)

    def node(self, name):
        return self.nodes[name]

    def wait_connected(self, name, timeout=8.0):
        m = self.mon(name)
        return wait_for(lambda: m.state == rigcore.NodeMonitor.CONNECTED,
                        timeout) and m.state == rigcore.NodeMonitor.CONNECTED

    def write_static_fix(self, text):
        with open(self._sf_path, "w") as fh:
            fh.write(text)
        # mtime resolution: make sure the next _load() sees a change.
        os.utime(self._sf_path, (time.time(), time.time() + 1))

    # -- HTTP ---------------------------------------------------------------
    def req(self, method, path, body=None, raw=None, timeout=20):
        """(status_code, parsed_json). `raw` sends bytes verbatim, which is how
        the malformed-body cases are staged."""
        data = None
        if raw is not None:
            data = raw if isinstance(raw, bytes) else raw.encode()
        elif body is not None:
            data = json.dumps(body).encode()
        r = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path),
                                   data=data, method=method)
        if data is not None:
            r.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(r, timeout=timeout) as fh:
                return fh.status, _body(fh.read())
        except urllib.error.HTTPError as e:
            return e.code, _body(e.read())

    def get(self, path, **kw):
        return self.req("GET", path, **kw)

    def post(self, path, body=None, **kw):
        return self.req("POST", path, body=body, **kw)

    def close(self):
        try:
            self.srv.shutdown()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            self.srv.server_close()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            self.rig.stop()
        except Exception:                                          # noqa: BLE001
            pass
        for n in self.nodes.values():
            n.close()
        rigd.NODES = self._saved["NODES"]
        rigd.NodeMonitor = self._saved["MON"]
        rigd.STATIC_FIX_PATH = self._saved["SF"]
        rigd.Rig._start_nav = self._saved["startnav"]
        rigd.Rig._startup_calibrate = self._saved["calib"]
        rigd.Rig._anomaly_loop = self._saved["loop"]
        rigd.RIG = self._saved["RIG"]
        rigcore.http_json = self._saved["http_json"]
        shutil.rmtree(self.dir, ignore_errors=True)


def _body(raw):
    try:
        return json.loads(raw.decode() or "null")
    except (ValueError, UnicodeDecodeError):
        return {"_raw": raw[:200].decode(errors="replace")}


def is400(code, doc):
    return 400 <= code < 500 and isinstance(doc, dict) \
        and doc.get("ok") is False and bool(doc.get("error"))


def raw_post(port, path, extra_len, payload=b""):
    """A POST whose Content-Length claims more than it sends, so the oversize
    guard can be exercised without the client racing the server's close (a
    real 1 MiB write dies with EPIPE half way through)."""
    import socket
    s = socket.create_connection(("127.0.0.1", port), timeout=10)
    try:
        s.sendall(("POST %s HTTP/1.1\r\nHost: 127.0.0.1\r\n"
                   "Content-Type: application/json\r\n"
                   "Content-Length: %d\r\n\r\n" % (path, extra_len)).encode())
        s.sendall(payload)
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        while True:
            try:
                chunk = s.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    head, _, body = buf.partition(b"\r\n\r\n")
    try:
        code = int(head.split(b" ")[1])
    except (IndexError, ValueError):
        code = 0
    return code, _body(body)


# The ilxctl/piagent paths a rigd HANDLER drives. Everything else a fake sees
# (/api/status, /health, /shots, /gpio/exposure/events) is a background poll.
CONTROL_PATHS = {"/api/exposure", "/api/focus/mode", "/api/focus/drive",
                 "/api/focus/position", "/api/zoom/drive",
                 "/api/zoom/position", "/api/zoom/setting", "/api/store",
                 "/api/connect", "/api/shutter", "/gpio/focus", "/gpio/fire",
                 "/api/card/mode", "/api/card/list"}


# ===========================================================================
# D1 — a body we cannot read is not an empty body
# ===========================================================================
BAD_BODIES = [("truncated object", '{"label": "t"'),
              ("bare array", "[]"),
              ("bare string", '"go"'),
              ("bare number", "7"),
              ("literal null", "null"),
              ("not JSON at all", "label=transect&interval=2")]

POST_PATHS = ["/api/run/start", "/api/run/stop", "/api/settings",
              "/api/settings/preview", "/api/settings/auto", "/api/ev",
              "/api/drain", "/api/drain/cancel", "/api/capture",
              "/api/calibrate", "/api/strobe", "/api/exposure",
              "/api/focus/mode", "/api/focus/drive", "/api/focus/position",
              "/api/zoom/drive", "/api/zoom/position", "/api/zoom/setting",
              "/api/node/connect", "/api/node/focus", "/api/run/open",
              "/api/reconcile"]


def t_bodies(h):
    sect("D1 rigd: a POST body that will not parse is refused, not defaulted")
    # The one that cost a transect: a truncated body started a real run.
    for name, raw in BAD_BODIES:
        code, doc = h.post("/api/run/start", raw=raw)
        check("run/start refuses a %s body" % name, is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:120]))
        check("run/start did not start a run on a %s body" % name,
              h.rig.runmgr.status().get("active") is not True,
              json.dumps(h.rig.runmgr.status().get("run_id")))
        if h.rig.runmgr.status().get("active"):
            h.rig.runmgr.stop()

    desired_before = json.dumps(h.rig.settings.get(), sort_keys=True)
    t0 = time.time()
    bad = []
    for p in POST_PATHS:
        for name, raw in BAD_BODIES:
            code, doc = h.post(p, raw=raw)
            if not is400(code, doc):
                bad.append("%s <- %s: %s %s" % (p, name, code,
                                                json.dumps(doc)[:60]))
    check("every POST endpoint answers 4xx {ok:false,error} to a non-object "
          "body", not bad, "; ".join(bad[:6]))
    check("the desired settings vector is untouched by every bad body",
          json.dumps(h.rig.settings.get(), sort_keys=True) == desired_before)
    # The monitors and any pull worker keep polling in the background, so
    # count only the paths a HANDLER drives - the ones that change a camera.
    drove = [c for n in h.nodes.values() for c in n.calls_since(t0)
             if c[3] in CONTROL_PATHS]
    check("no bad body reached a camera", not drove,
          "%d control calls: %s" % (len(drove), str(drove[:4])))
    code, doc = raw_post(h.port, "/api/run/start",
                         extra_len=(1 << 20) + 1, payload=b"xxxx")
    check("an oversize body is refused rather than framed as the next request",
          is400(code, doc), "%s %s" % (code, json.dumps(doc)[:80]))
    # An EMPTY body is still legal - the UI's doStop() sends one.
    code, doc = h.post("/api/run/stop")
    check("an empty body is still accepted (the UI's Stop sends one)",
          code == 200 and isinstance(doc, dict), "%s" % code)


# ===========================================================================
# D2 — value validation, before a node is contacted
# ===========================================================================
def t_inputs(h):
    sect("D2 rigd: bad values are refused at the door, not sent to a camera")
    h.wait_connected("cam1")
    h.wait_connected("cam2")
    cam1 = h.node("cam1")

    # --- the auto-exposure servo -----------------------------------------
    auto_before = h.rig.settings.get().get("auto")
    for v in ("maybe", "yes", "off", 2, [], {}, None):
        code, doc = h.post("/api/settings/auto", {"on": v})
        check("settings/auto refuses on=%r" % (v,), is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:90]))
    check("the exposure servo was NOT armed by a truthy string",
          h.rig.settings.get().get("auto") == auto_before,
          "auto=%r" % h.rig.settings.get().get("auto"))
    code, doc = h.post("/api/settings/auto", {"on": True})
    check("settings/auto still accepts a real bool", code == 200
          and doc.get("ok") is True, json.dumps(doc)[:80])
    h.post("/api/settings/auto", {"on": False})

    # --- exposure ---------------------------------------------------------
    t0 = time.time()
    cases = [({"node": "cam1", "which": "iso", "value": "abc"},
              "a non-numeric value"),
             ({"node": "cam1", "which": "iso", "value": None}, "a null value"),
             ({"node": "cam1", "which": "iso", "value": 1.5},
              "a fractional value"),
             ({"node": "cam1", "which": "isoo", "value": 400},
              "an unknown which"),
             ({"node": "cam9", "which": "iso", "value": 400},
              "an unknown node"),
             ({"which": "iso", "value": 400}, "no node key"),
             ({"node": "cam1", "which": "iso", "value": 999999},
              "a value the body does not offer")]
    for body, why in cases:
        code, doc = h.post("/api/exposure", body)
        check("exposure refuses %s" % why, is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:110]))
    sent = cam1.calls_since(t0, "/api/exposure")
    check("not one bad exposure reached ilxctl (the 6 s SDK stall)",
          not sent, "%d forwarded: %s" % (len(sent), str(sent[:3])))
    code, doc = h.post("/api/exposure",
                       {"node": "cam1", "which": "iso", "value": 800})
    check("a legal exposure still reaches the camera",
          code == 200 and doc.get("ok") is not False
          and cam1.raw("iso") == 800,
          "%s iso=%r" % (json.dumps(doc)[:60], cam1.raw("iso")))
    # ISO_AUTO is in the body's own choice list, so it must pass the gate.
    code, doc = h.post("/api/exposure",
                       {"node": "cam1", "which": "iso", "value": ISO_AUTO})
    check("a value the body DOES offer is not blocked by the choice gate",
          code == 200, "%s %s" % (code, json.dumps(doc)[:80]))
    h.post("/api/exposure", {"node": "cam1", "which": "iso", "value": 400})

    # --- query strings ----------------------------------------------------
    for path in ("/api/events?since=abc", "/api/events?since=-4",
                 "/api/imu/window?t0=abc", "/api/imu/window?t0=nan",
                 "/api/status?node=cam9", "/api/shots?node=../"):
        code, doc = h.get(path)
        check("GET %s is a clean 4xx, never a 500 with a traceback" % path,
              is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:100]))
    code, doc = h.get("/api/events?since=0")
    check("GET /api/events?since=0 still works", code == 200
          and isinstance(doc, dict), str(code))

    # --- ev / calibrate ---------------------------------------------------
    for v in ("x", None, 1.5, [1]):
        code, doc = h.post("/api/ev", {"steps": v})
        check("ev refuses steps=%r" % (v,), is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:80]))
    code, doc = h.post("/api/ev", {"steps": 1})
    check("ev still accepts a whole number of steps", code == 200
          and doc.get("ok") is True, json.dumps(doc)[:80])
    h.post("/api/ev", {"steps": -1})
    for body, why in [({"samples": 0}, "samples=0"), ({"samples": -3}, "-3"),
                      ({"samples": "five"}, "a word"),
                      ({"samples": 5, "nodes": ["cam9"]}, "an unknown node"),
                      ({"nodes": "cam1"}, "nodes that is not a list")]:
        code, doc = h.post("/api/calibrate", body)
        check("calibrate refuses %s" % why, is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:100]))

    # --- lens fanout ------------------------------------------------------
    # A camera that is not connected was SKIPPED SILENTLY and the answer was
    # still ok:true with empty results, so the UI's lens queue treated a lost
    # STOP as delivered.
    # Staged WITHOUT taking the listener down: fakenode.loopback_map gives
    # every process the same ports for a given os.getpid() % 40, so releasing
    # a port mid-suite lets a second soaktest process (three lanes share this
    # worktree) bind it and answer our monitor as a live camera — see the
    # staging note in t_drain. An unclaimed body is the real "not connected"
    # anyway; connect_lies keeps the monitor from re-claiming it.
    h.node("cam2").set_fault(surface="ilx", connect_lies=True)
    with h.node("cam2")._lock:
        h.node("cam2").connected = False
    check("cam2 seen as not connected",
          wait_for(lambda: not h.mon("cam2").is_connected(), 8.0),
          h.mon("cam2").state)
    code, doc = h.post("/api/focus/position", {"node": "cam2", "value": 500})
    check("an addressed but disconnected camera answers ok:false",
          code == 200 and doc.get("ok") is False
          and "not connected" in (doc.get("error") or ""),
          json.dumps(doc)[:140])
    code, doc = h.post("/api/focus/drive", {"step": 0})
    check("a fleet-wide lens call names the camera it skipped",
          isinstance(doc, dict) and "cam2" in (doc.get("skipped") or {}),
          json.dumps(doc)[:140])
    h.node("cam2").clear_faults()
    with h.node("cam2")._lock:
        h.node("cam2").connected = True
    check("cam2 back", h.wait_connected("cam2", 12.0))

    t0 = time.time()
    for body, why in [({"node": "cam1", "value": "abc"}, "a non-numeric value"),
                      ({"node": "cam1", "value": None}, "a null value"),
                      ({"node": "cam9", "value": 1}, "an unknown node")]:
        code, doc = h.post("/api/focus/position", body)
        check("focus/position refuses %s" % why, is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:100]))
    check("no bad lens value reached a camera",
          not h.node("cam1").calls_since(t0, "/api/focus/position"),
          str(h.node("cam1").calls_since(t0, "/api/focus/position")[:2]))

    # Out-of-range against the body's OWN published range. The fake does not
    # emit range triples, so stage one the way a real body reports it.
    h.node("cam1").label_override = {"focusPosRange": {"min": 0, "max": 1000,
                                                       "step": 1}}
    check("the staged range reached the monitor",
          wait_for(lambda: (h.mon("cam1").snapshot().get("status") or {})
                   .get("focusPosRange") is not None, 6.0))
    code, doc = h.post("/api/focus/position", {"node": "cam1",
                                               "value": 99999999})
    clamped = (doc.get("clamped") or {}).get("cam1") or {}
    check("an out-of-range lens position is clamped and SAID so, not silently "
          "dropped", code == 200 and clamped.get("to") == 1000,
          json.dumps(doc)[:160])
    code, doc = h.post("/api/focus/position", {"node": "cam1", "value": -1})
    clamped = (doc.get("clamped") or {}).get("cam1") or {}
    check("a negative lens position is clamped to the body's minimum",
          clamped.get("to") == 0, json.dumps(doc)[:160])
    h.node("cam1").label_override = {}


# ===========================================================================
# D3 — clock domains
# ===========================================================================
def _drifted_http_json(offsets):
    """Wrap rigcore.http_json so a fake's /health reports a clock `offset`
    seconds off the host's. The fakes have no clock knob and are not this
    lane's file, so the drift is injected on the wire instead."""
    orig = rigcore.http_json

    def w(url, *a, **kw):
        r = orig(url, *a, **kw)
        if isinstance(r, dict) and isinstance(url, str) and "/health" in url:
            for host, off in offsets.items():
                if url.startswith("http://%s:" % host) \
                        and isinstance(r.get("time"), dict) \
                        and r["time"].get("epoch") is not None:
                    r = dict(r)
                    r["time"] = dict(r["time"],
                                     epoch=r["time"]["epoch"] + off)
                    break
        return r
    return w


def _kinds(anoms, kind):
    return [a for a in anoms if a["kind"] == kind]


def _flush_clock(h):
    """Drop the monitors' clock windows so the next phase's samples are the
    only ones in the median. Without this each phase inherits up to eight
    samples of the previous phase's drift and the arithmetic under test is
    whatever the two windows happen to overlap on."""
    for m in h.rig.monitors:
        with m._lock:
            m._clock_hist.clear()
            m.clock = None


def _streak(an):
    """Anomalies._skew_streak — absent on pre-fix rigd, which is D3 itself.
    Tolerated so this suite reports honest FAILs there instead of aborting."""
    return getattr(an, "_skew_streak", None) or {}


def _clear_streak(an):
    if hasattr(an, "_skew_streak"):
        an._skew_streak.clear()


def _settled(h, pred, timeout=12.0):
    """Wait until every monitor has at least 3 usable samples AND pred holds."""
    return wait_for(lambda: all(m.clock_offset_info()["n"] >= 3
                                for m in h.rig.monitors) and pred(), timeout)


def t_clock(h):
    sect("D3 rigd: host_clock_offset, and a skew alarm that has to persist")
    h.wait_connected("cam1")
    h.wait_connected("cam2")
    an = h.rig.anomalies

    # -- both nodes agree with each other and disagree with the HOST -------
    # This is today's live fault: the Pis are chrony-locked to each other
    # (0.6 ms) while the Mac is 187 ms out and drifting. The node-to-node
    # check sees skew 0 and says nothing.
    rigcore.http_json = _drifted_http_json({"127.0.0.2": 0.2,
                                            "127.0.0.3": 0.2})
    _flush_clock(h)
    try:
        ok = _settled(h, lambda: all(
            0.15 < (m.clock_offset_s() or 0) < 0.25 for m in h.rig.monitors))
        check("both fakes now report a +0.2 s node-vs-host offset", ok,
              str([m.clock_offset_s() for m in h.rig.monitors]))
        a = an.scan()
        hco = _kinds(a, "host_clock_offset")
        check("host_clock_offset fires when the host is 200 ms off the fleet",
              len(hco) == 1, json.dumps([x["kind"] for x in a]))
        if hco:
            ev = hco[0]["evidence"]
            check("host_clock_offset reports the filtered median offset",
                  0.15 < abs(ev.get("offset_s", 0)) < 0.25,
                  json.dumps(ev)[:140])
            check("host_clock_offset is a warning at 0.2 s (bad above 0.5)",
                  hco[0]["sev"] == "warn", hco[0]["sev"])
            check("host_clock_offset names the host fix and the 10 ms budget",
                  "network time" in hco[0]["suggested_action"]
                  and "10 ms" in hco[0]["suggested_action"],
                  hco[0]["suggested_action"][:120])
        check("two nodes that agree with EACH OTHER raise no skew alarm",
              not _kinds(a, "node_clock_skew"),
              json.dumps([x["msg"] for x in _kinds(a, "node_clock_skew")]))
        check("the fleet view carries the filtered clock_offset_s next to the "
              "raw clock_offset_ms",
              all(v.get("clock_offset_s") is not None
                  and v.get("clock_offset_ms") is not None
                  for v in h.rig.fleet()["nodes"]),
              json.dumps([{k: v.get(k) for k in
                           ("name", "clock_offset_ms", "clock_offset_s")}
                          for v in h.rig.fleet()["nodes"]]))
        # A 0.6 s offset is the bad tier.
        rigcore.http_json = _drifted_http_json({"127.0.0.2": 0.6,
                                                "127.0.0.3": 0.6})
        _flush_clock(h)
        _settled(h, lambda: all((m.clock_offset_s() or 0) > 0.5
                                for m in h.rig.monitors))
        hco = _kinds(an.scan(), "host_clock_offset")
        check("host_clock_offset escalates to bad above 0.5 s",
              hco and hco[0]["sev"] == "bad",
              json.dumps([x["sev"] for x in hco]))
    finally:
        rigcore.http_json = _drifted_http_json({})

    # -- node-to-node skew has to PERSIST ---------------------------------
    _clear_streak(an)
    rigcore.http_json = _drifted_http_json({"127.0.0.3": 0.05})
    _flush_clock(h)
    try:
        ok = _settled(h, lambda: 0.03 < abs(
            (h.mon("cam2").clock_offset_s() or 0)
            - (h.mon("cam1").clock_offset_s() or 0)) < 0.07)
        check("cam2's clock is now ~50 ms off cam1's", ok,
              str([m.clock_offset_s() for m in h.rig.monitors]))
        first = _kinds(an.scan(), "node_clock_skew")
        second = _kinds(an.scan(), "node_clock_skew")
        third = _kinds(an.scan(), "node_clock_skew")
        check("one scan of skew is NOT an alarm (the live 82.9 ms false "
              "positives)", not first and not second,
              "scan1=%d scan2=%d" % (len(first), len(second)))
        check("node_clock_skew fires once it has held for 3 scans",
              len(third) == 1, json.dumps([x["msg"] for x in third]))
        if third:
            check("node_clock_skew keeps the 10 ms budget framing",
                  "10 ms" in third[0]["suggested_action"],
                  third[0]["suggested_action"][:100])
            check("node_clock_skew says how many scans it has held",
                  third[0]["evidence"].get("scans") == 3,
                  json.dumps(third[0]["evidence"])[:140])
        # Back in agreement: the streak resets, so the next disagreement has
        # to earn its alarm again. Scan while waiting - the streak only clears
        # on a scan that finds the clocks inside budget.
        rigcore.http_json = _drifted_http_json({})
        _flush_clock(h)
        check("the skew streak resets when the clocks agree again",
              wait_for(lambda: (an.scan(), not _streak(an))[1], 12.0),
              json.dumps({str(k): v for k, v in _streak(an).items()}))
    finally:
        rigcore.http_json = _drifted_http_json({})

    # -- a link too slow to measure is REPORTED, not dropped ---------------
    _clear_streak(an)
    h.node("cam2").set_fault(surface="pia", hang_s=0.05)
    _flush_clock(h)
    try:
        ok = wait_for(lambda: (h.mon("cam2").clock_offset_info()
                               .get("rtt_ms_best") or 0) >= 20.0, 12.0)
        check("cam2's /health RTT is now above the 20 ms measurement floor",
              ok, json.dumps(h.mon("cam2").clock_offset_info()))
        a = an.scan()
        unm = _kinds(a, "node_clock_unmeasurable")
        check("a node whose link is too slow is reported, not silently "
              "excluded", len(unm) == 1 and unm[0]["node"] == "cam2",
              json.dumps([(x["kind"], x["node"]) for x in a]))
        if unm:
            check("node_clock_unmeasurable says the sync budget cannot be "
                  "verified", "10 ms" in unm[0]["suggested_action"],
                  unm[0]["suggested_action"][:100])
    finally:
        h.node("cam2").set_fault(surface="pia", hang_s=0.0)


# ===========================================================================
# D4 — drain honesty, the claim race, cancel, and the wedge skip
# ===========================================================================
class StubDrainer:
    """Stands in for drain.Drainer so the drain path is instant and its stop
    Event is observable. drain.py's own behaviour is soaktest's suite_drain."""

    made = []
    files = 6
    errors = ()
    per_file_s = 0.02

    def __init__(self, node, host, dest=None, log=None):
        self.node, self.host, self.log = node, host, log or (lambda *a: None)
        self.pulled = 0
        self.stop = None
        StubDrainer.made.append(self)

    def run(self, keep_card=False, formats=None, limit=None, stop=None):
        self.stop = stop
        rep = {"node": self.node, "pulled": 0, "bytes": 0, "verified": 0,
               "deleted": 0, "skipped": 0, "errors": list(self.errors),
               "files": [], "cancelled": False}
        for _ in range(self.files):
            if stop is not None and stop.is_set():
                rep["cancelled"] = True
                break
            time.sleep(self.per_file_s)
            rep["pulled"] += 1
            rep["deleted"] += 1
            self.pulled = rep["pulled"]
        return rep


def t_drain(h):
    sect("D4 rigd: drain reported honestly, claimed atomically, cancellable")
    h.wait_connected("cam1")
    h.wait_connected("cam2")
    real = rigd.draindrv.Drainer
    rigd.draindrv.Drainer = StubDrainer
    try:
        # -- the claim is taken under the RUN MANAGER's lock ---------------
        # The previous fix took rigd's own _drain_lock, which
        # RunManager.start never acquires: the two guards sat on different
        # mutexes and the TOCTOU survived. Watch the assignment itself.
        rm = h.rig.runmgr
        held = []
        base = type(rm)

        class Watched(base):
            @property
            def draining(self):
                return self.__dict__.get("_draining")

            @draining.setter
            def draining(self, v):
                held.append((v, self._lock.locked()))
                self.__dict__["_draining"] = v

        prev = rm.__dict__.pop("draining", None)
        rm.__class__ = Watched
        rm.__dict__["_draining"] = prev
        try:
            StubDrainer.made = []
            StubDrainer.files = 2
            r = h.rig.start_drain(["cam1"])
            check("a drain of a connected node starts", r.get("ok") is True,
                  json.dumps(r)[:120])
            claims = [x for x in held if x[0] is not None]
            check("runmgr.draining is claimed while the RUN MANAGER's own "
                  "lock is held", claims and all(x[1] for x in claims),
                  str(held[:4]))
            check("a run refuses to start while the drain holds the node",
                  (h.rig.runmgr.start({"label": "x"}) or {}).get("ok") is False)
            check("the drain finishes",
                  wait_for(lambda: not h.rig.drain_status()["active"], 10.0),
                  json.dumps(h.rig.drain_status())[:120])
        finally:
            rm.__class__ = base
            rm.__dict__.pop("_draining", None)
            rm.draining = None

        # -- cancel ---------------------------------------------------------
        StubDrainer.made = []
        StubDrainer.files = 40
        StubDrainer.per_file_s = 0.03
        r = h.rig.start_drain(["cam1", "cam2"])
        check("a two-node drain starts", r.get("ok") is True, json.dumps(r)[:90])
        check("the drain is working",
              wait_for(lambda: StubDrainer.made
                       and StubDrainer.made[0].pulled >= 2, 6.0),
              str([d.pulled for d in StubDrainer.made]))
        code, doc = h.post("/api/drain/cancel")
        check("POST /api/drain/cancel is accepted",
              code == 200 and doc.get("ok") is True, json.dumps(doc)[:100])
        st = h.rig.drain_status()
        check("GET /api/drain shows cancel_requested",
              h.get("/api/drain")[1].get("cancel_requested") is True,
              json.dumps(st)[:140])
        check("the drain stops",
              wait_for(lambda: not h.rig.drain_status()["active"], 10.0),
              json.dumps(h.rig.drain_status())[:140])
        d0 = StubDrainer.made[0]
        check("cancel was delivered as a threading.Event on Drainer.run(stop=)",
              isinstance(d0.stop, threading.Event) and d0.stop.is_set(),
              repr(d0.stop))
        check("the cancel stopped the drain BETWEEN files, mid-card",
              0 < d0.pulled < StubDrainer.files, "%d of %d pulled"
              % (d0.pulled, StubDrainer.files))
        check("the second node's card was never touched after the cancel",
              len(StubDrainer.made) == 1 or StubDrainer.made[1].pulled == 0,
              str([d.pulled for d in StubDrainer.made]))
        check("the cancelled drain is reported as cancelled, not as done",
              (h.rig.drain_status().get("last") or {}).get("cancelled") is True,
              json.dumps(h.rig.drain_status().get("last"))[:140])
        code, doc = h.post("/api/drain/cancel")
        check("cancelling when nothing is draining is a clean refusal",
              doc.get("ok") is False, json.dumps(doc)[:100])

        # -- /api/run/stop tells the truth about the auto-drain ------------
        StubDrainer.made = []
        StubDrainer.files = 1
        StubDrainer.per_file_s = 0.0
        code, doc = h.post("/api/run/start", {"label": "audit-drain"})
        check("a run starts for the stop test", doc.get("ok") is True,
              json.dumps(doc)[:120])
        # Staging note — this was node.down(), and that is why this section
        # flaked in a FULL soaktest run while passing standalone. down()
        # RELEASES the fake's loopback listener, and fakenode.loopback_map
        # hands every process the same ports for a given os.getpid() % 40: a
        # second soaktest process (three lanes share this worktree) sitting in
        # _mkserver's 5 s bind-retry loop takes the port the moment we let go
        # and answers OUR monitor as a live camera. The node then reads
        # CAM_CONNECTED, the auto-drain runs, and both checks below fail —
        # reproduced deterministically by binding a second fake on the freed
        # port. What D4 is about is a node that CANNOT be drained, so stage
        # exactly that and never release the socket: the Pi answers, the body
        # is not claimed (PC-remote priority lost — the live failure this
        # "not connected" message was written for), and connect_lies stops the
        # monitor re-claiming it underneath the test.
        for n in ("cam1", "cam2"):
            fake = h.node(n)
            fake.set_fault(surface="ilx", connect_lies=True)
            with fake._lock:
                fake.connected = False
        check("neither camera is claimed, so neither one can be drained",
              wait_for(lambda: not any(m.is_connected()
                                       for m in h.rig.monitors), 10.0),
              " ".join("%s=%s" % (m.name_, m.state) for m in h.rig.monitors))
        seq = h.rig.events._seq
        code, doc = h.post("/api/run/stop", {})
        check("run/stop reports drain_started FALSE when no node could be "
              "drained", doc.get("drain_started") is False,
              json.dumps({k: doc.get(k) for k in
                          ("ok", "drain_started", "drain")})[:200])
        check("the refused auto-drain names the reason in the response",
              "not connected" in json.dumps(doc.get("drain") or {}),
              json.dumps(doc.get("drain"))[:200])
        evs = [e for e in h.rig.events.since(seq)["events"]
               if e.get("kind") == "drain"]
        check("the refused auto-drain leaves an event in the journal",
              any("not" in (e.get("msg") or "") for e in evs),
              json.dumps([e.get("msg") for e in evs])[:220])
        for n in ("cam1", "cam2"):
            fake = h.node(n)
            fake.clear_faults()
            with fake._lock:
                fake.connected = True
        check("cam1 back", h.wait_connected("cam1", 15.0))
        check("cam2 back", h.wait_connected("cam2", 15.0))

        # -- the wedge skip -------------------------------------------------
        StubDrainer.made = []
        StubDrainer.errors = ("card index not ready within 90s - the body's "
                              "transfer subsystem is wedged",)
        seq = h.rig.events._seq
        h.rig.start_drain(["cam2"])
        wait_for(lambda: not h.rig.drain_status()["active"], 10.0)
        check("a wedged drain is remembered",
              "cam2" in h.rig.drain_status().get("wedged", {}),
              json.dumps(h.rig.drain_status().get("wedged"))[:140])
        StubDrainer.errors = ()
        StubDrainer.made = []
        seq = h.rig.events._seq
        r = h.rig.start_drain(["cam1", "cam2"], auto=True)
        wait_for(lambda: not h.rig.drain_status()["active"], 10.0)
        check("the auto-drain SKIPS the wedged node", "cam2" in (r.get("skipped")
              or {}) and r.get("draining") == ["cam1"], json.dumps(r)[:200])
        check("the skip says power-cycle, and says it in an event",
              any("wedged" in (e.get("msg") or "") or
                  "power-cycle" in (e.get("msg") or "")
                  for e in h.rig.events.since(seq)["events"]),
              json.dumps([e.get("msg") for e in h.rig.events.since(seq)["events"]
                          if e.get("kind") == "drain"])[:220])
        check("only the un-wedged node was drained",
              [d.node for d in StubDrainer.made] == ["cam1"],
              str([d.node for d in StubDrainer.made]))
        # Seeing the node reboot releases the skip.
        h.mon("cam2").rebooted_at = time.time()
        StubDrainer.made = []
        r = h.rig.start_drain(["cam2"], auto=True)
        wait_for(lambda: not h.rig.drain_status()["active"], 10.0)
        check("a node seen to power-cycle is auto-drained again",
              r.get("ok") is True and "cam2" not in (r.get("skipped") or {}),
              json.dumps(r)[:160])
    finally:
        rigd.draindrv.Drainer = real
        StubDrainer.errors = ()
        StubDrainer.per_file_s = 0.02


# ===========================================================================
# D5 — the static fix is an armed fallback, not a gap filler
# ===========================================================================
class FakeReader:
    """The NavReader surface StaticFixNav delegates to."""

    def __init__(self):
        self.gateway_online = True
        self.fix = True

    def _row(self, at):
        r = {"lat": None, "lon": None, "long": None, "xutm": None,
             "yutm": None, "utm_zone": None, "depth_m": None,
             "heading_mag_deg": None, "sog_mps": None, "sats": None,
             "valid": False, "stale": True, "local_epoch": at, "epoch": at,
             "age_s": None, "gateway_online": self.gateway_online,
             "time_source": "gps" if self.fix else "jetson"}
        if self.fix:
            r.update(lat=40.5, lon=-105.5, long=-105.5, valid=True,
                     stale=False, age_s=0.05, sats=9)
        return r

    def fix_at(self, epoch=None, max_age_s=None):
        return self._row(time.time() if epoch is None else float(epoch))

    def snapshot(self):
        return self._row(time.time())

    def health(self):
        return {"present": True, "online": self.gateway_online}


class _NavMod:
    @staticmethod
    def latlon_to_utm(lat, lon):
        return 500000.0, 4485000.0, "13N"


# The arming API grew an owner (D9): begin_run returns (armed, token), end_run
# takes run_id=/token= and StaticFixNav takes run_active=. Pre-fix it is a bare
# bool, a no-argument end_run and a three-argument constructor - normalise, so
# a pre-fix run reports these defects as honest FAILs instead of a TypeError
# that skips the rest of the section.
def _mknav(h, reader, run_active=None):
    try:
        return rigd.StaticFixNav(reader, _NavMod, h.rig.events,
                                 run_active=run_active)
    except TypeError:
        return rigd.StaticFixNav(reader, _NavMod, h.rig.events)


def _begin(nav, online):
    r = nav.begin_run(online)
    return r if isinstance(r, tuple) else (r, None)


def _bind(nav, run_id, token=None):
    if hasattr(nav, "bind_run"):
        return nav.bind_run(run_id, token)
    return False


def _end(nav, **kw):
    try:
        return nav.end_run(**kw)
    except TypeError:
        return nav.end_run()


def t_static(h):
    sect("D5 rigd: the static fix stands in only when it was ARMED at start")
    h.write_static_fix(json.dumps({"lat": 40.4, "lon": -105.4,
                                   "label": "site A",
                                   "captured_epoch": time.time() - 86400}))
    rd = FakeReader()
    nav = rigd.StaticFixNav(rd, _NavMod, h.rig.events)

    # -- the no-NMEA deployment: armed at start, used all run ---------------
    rd.gateway_online = False
    rd.fix = False
    armed, tok = _begin(nav, rd.gateway_online)
    _bind(nav, "d5-no-nmea", tok)
    check("a run started with a dead gateway arms the static fallback", armed)
    row = nav.fix_at(time.time())
    check("an armed static fix supplies the position", row.get("lat") == 40.4,
          json.dumps({k: row.get(k) for k in ("lat", "fix_kind", "valid")}))
    check("a static row is NEVER valid:true", row.get("valid") is False,
          repr(row.get("valid")))
    check("a static row says fix_kind=static", row.get("fix_kind") == "static",
          repr(row.get("fix_kind")))
    check("a static row names the label so the flight log can say so",
          row.get("static_fix") == "site A", repr(row.get("static_fix")))
    check("age_s measures back to the ORIGINAL fix, not to now",
          row.get("age_s", 0) > 80000, repr(row.get("age_s")))
    _end(nav, run_id="d5-no-nmea")

    # -- the transient mid-run gap: NOT armed, so the cells stay empty ------
    rd.gateway_online = True
    rd.fix = True
    armed, tok = _begin(nav, rd.gateway_online)
    _bind(nav, "d5-live", tok)
    check("a run started on live GPS does NOT arm the fallback", armed is False)
    live = nav.fix_at(time.time())
    check("a live fix still wins and is fix_kind=live",
          live.get("valid") is True and live.get("fix_kind") == "live",
          json.dumps({k: live.get(k) for k in ("valid", "fix_kind")}))
    rd.gateway_online = False           # the bus drops mid-line
    rd.fix = False
    gap = nav.fix_at(time.time())
    check("a MID-RUN gap leaves lat/long empty instead of papering over it "
          "with a constant", gap.get("lat") is None and gap.get("long") is None,
          json.dumps({k: gap.get(k) for k in ("lat", "long", "static_fix")}))
    check("the mid-run gap is fix_kind=none", gap.get("fix_kind") == "none",
          repr(gap.get("fix_kind")))
    snap = nav.snapshot()
    check("snapshot() agrees, so nav_no_fix can see it",
          snap.get("lat") is None and snap.get("fix_kind") == "none",
          json.dumps({k: snap.get(k) for k in ("lat", "fix_kind")}))

    # -- nav_no_fix during that same unarmed run ---------------------------
    # Still inside the run (the fallback is NOT armed), the gateway is gone,
    # and frames are being written with empty positions. That has to be an
    # anomaly of its own - it used to sit behind nav_gateway_down's elif and
    # was never raised at all.
    saved_nav, saved_run = h.rig.anomalies.nav, h.rig.anomalies.runmgr

    class _Run:
        @staticmethod
        def status():
            return {"active": True, "run_id": "r1", "stats": {}, "sync": {}}

    h.rig.anomalies.nav, h.rig.anomalies.runmgr = nav, _Run
    try:
        a = h.rig.anomalies.scan()
        nf = _kinds(a, "nav_no_fix")
        check("a run recording with no position raises nav_no_fix",
              len(nf) == 1, json.dumps([x["kind"] for x in a]))
        if nf:
            check("nav_no_fix during a run is a BAD-severity alarm, not an "
                  "advisory", nf[0]["sev"] == "bad", nf[0]["sev"])
    finally:
        h.rig.anomalies.nav, h.rig.anomalies.runmgr = saved_nav, saved_run
    _end(nav, run_id="d5-live")

    # -- /api/nav exposes fix_kind -----------------------------------------
    saved = h.rig.nav
    h.rig.nav = nav
    try:
        rd.gateway_online = False       # no bus at all, and no run: idle arm
        rd.fix = False
        code, doc = h.get("/api/nav")
        check("/api/nav exposes fix_kind=static with valid:false",
              code == 200 and doc.get("fix_kind") == "static"
              and doc.get("valid") is False and doc.get("lat") == 40.4,
              json.dumps({k: doc.get(k) for k in
                          ("fix_kind", "valid", "lat", "static_fix")}))
        rd.gateway_online = True
        rd.fix = True
        code, doc = h.get("/api/nav")
        check("/api/nav says fix_kind=live on a real fix",
              doc.get("fix_kind") == "live" and doc.get("valid") is True,
              json.dumps({k: doc.get(k) for k in ("fix_kind", "valid")}))
    finally:
        h.rig.nav = saved

    # -- a malformed static_fix.json is warned about ONCE ------------------
    # Last, because it leaves the file unreadable for anything after it.
    h.write_static_fix("{ this is not json")
    seq = h.rig.events._seq
    nav2 = rigd.StaticFixNav(FakeReader(), _NavMod, h.rig.events)
    for _ in range(20):
        nav2.static_label()
        nav2.snapshot()
        nav2.health()
    warns = [e for e in h.rig.events.since(seq)["events"]
             if "static_fix.json" in (e.get("msg") or "")]
    check("a malformed static_fix.json is parsed once and warned about once, "
          "not once per poll", len(warns) == 1,
          "%d warnings from 60 reads" % len(warns))
    check("a malformed static_fix.json arms nothing",
          nav2.armed() is False and nav2.static_label() is None,
          repr(nav2.static_label()))


# ===========================================================================
# D9 — the static-fix latch belongs to the run that took it (review blocker)
# ===========================================================================
def t_latch(h):
    sect("D9 rigd: the static-fix latch is owned by its run")
    # The operator's launch point, as FIELD-RUN.md tells them to leave it.
    h.write_static_fix(json.dumps({"lat": 41.42, "lon": -71.45,
                                   "label": "Pier A",
                                   "captured_epoch": time.time() - 3600}))
    # The hint rigd wires in: the run_id recording right now, or None.
    live = {"id": None}
    rd = FakeReader()
    nav = _mknav(h, rd, run_active=lambda: live["id"])
    RID = "260824_0900_line3"

    # -- the traced sequence: the tick lands inside RunManager.start --------
    # Start is pressed with the gateway ONLINE, so the run is correctly NOT
    # armed. RunManager.start then live-probes every monitor with a 5 s
    # timeout, outside its own lock, and reports active:False throughout - so
    # the 5 s nav tick is guaranteed to land in that window when a node is
    # unreachable. It used to clear the latch there.
    rd.gateway_online = True
    rd.fix = True
    armed, tok = _begin(nav, rd.gateway_online)
    check("a Start on a live gateway latches NOT-armed", armed is False,
          repr(armed))
    check("the 5 s nav tick does NOT clear a latch while the start that took "
          "it is still in flight", _end(nav) is False,
          repr(getattr(nav, "_run_armed", None)))
    live["id"] = RID                        # RunManager.start returns; the
    _bind(nav, RID, tok)                    # run is recording, and is bound
    rd.gateway_online = False               # 20 min in, the N2K bus drops
    rd.fix = False
    row = nav.fix_at(time.time())
    check("a mid-run bus drop still writes EMPTY lat/long, not the armed "
          "static launch point",
          row.get("lat") is None and row.get("long") is None,
          json.dumps({k: row.get(k) for k in
                      ("lat", "long", "fix_kind", "static_fix")}))
    check("...and the row says fix_kind=none so nav_no_fix fires",
          row.get("fix_kind") == "none", repr(row.get("fix_kind")))

    # -- a second Start against that recording run --------------------------
    armed2, tok2 = _begin(nav, rd.gateway_online)
    check("a duplicate Start does not re-decide the recording run's latch",
          armed2 is False, repr(armed2))
    check("...and is handed no token, so it can release nothing",
          tok2 is None, repr(tok2))
    check("a Stop naming a DIFFERENT run cannot release this run's latch",
          _end(nav, run_id="some-other-run") is False,
          repr(getattr(nav, "_run_armed", None)))
    check("the recording run's own Stop does release it",
          _end(nav, run_id=RID) is True,
          repr(getattr(nav, "_run_armed", None)))
    live["id"] = None

    # -- belt and braces: recording with no latch at all --------------------
    # If the decision is missing while a run records - a run started by some
    # path other than /api/run/start, or a race nobody has thought of - the
    # stand-in is REFUSED, not guessed. A missing position is recoverable;
    # a fabricated one is not.
    rd.gateway_online = False
    rd.fix = False
    live["id"] = "260824_0930_unlatched"
    check("a run recording with NO latch writes empty cells rather than "
          "falling back to live gateway state",
          nav.armed() is False and nav.fix_at(time.time()).get("lat") is None,
          json.dumps({"armed": nav.armed(),
                      "lat": nav.fix_at(time.time()).get("lat")}))
    live["id"] = None
    check("...while the IDLE rule is unchanged: no bus and no run still shows "
          "the armed static position",
          nav.armed() is True and nav.fix_at(time.time()).get("lat") == 41.42,
          json.dumps({"armed": nav.armed(),
                      "lat": nav.fix_at(time.time()).get("lat")}))

    # -- the next transect gets its OWN decision ----------------------------
    # A Stop and the next Start can race: the latch of the run that just ended
    # is still bound when the new Start arrives (the nav tick runs every 5 s).
    # The new line must be judged on ITS gateway state, not inherit the last
    # line's - and the late Stop must not then strip what the new Start took.
    rd.gateway_online = False               # line A ran with no NMEA: armed
    rd.fix = False
    armedA, tokA = _begin(nav, rd.gateway_online)
    live["id"] = "260824_1000_lineA"
    _bind(nav, live["id"], tokA)
    check("line A is armed", armedA is True, repr(armedA))
    live["id"] = None                       # runmgr.stop() has returned...
    rd.gateway_online = True                 # ...the bus is back for line B
    rd.fix = True
    armedB, tokB = _begin(nav, rd.gateway_online)
    check("the next Start re-decides from ITS OWN gateway state instead of "
          "inheriting the finished line's", armedB is False, repr(armedB))
    check("...and line A's late Stop cannot strip it",
          _end(nav, run_id="260824_1000_lineA") is False,
          repr(getattr(nav, "_run_armed", None)))
    live["id"] = "260824_1005_lineB"
    _bind(nav, live["id"], tokB)
    rd.gateway_online = False               # the bus drops during line B
    rd.fix = False
    check("line B therefore writes EMPTY cells on a mid-run drop",
          nav.fix_at(time.time()).get("lat") is None,
          repr(nav.fix_at(time.time()).get("lat")))
    _end(nav, run_id=live["id"])
    live["id"] = None

    # -- one line's decision never governs another's rows -------------------
    # The tightest version of the same race: a Stop lands INSIDE the next
    # Start's window, so the new run records while the finished line's latch
    # is still bound to it. The new line gets empty cells (nav_no_fix says so)
    # rather than the last line's armed position.
    rd.gateway_online = False
    rd.fix = False
    armedC, tokC = _begin(nav, rd.gateway_online)
    live["id"] = "260824_1100_lineC"
    _bind(nav, live["id"], tokC)
    check("line C is armed", armedC is True, repr(armedC))
    live["id"] = "260824_1101_lineD"          # C stopped, D is recording
    check("a finished line's latch does not stand in for the line that is "
          "recording now", nav.fix_at(time.time()).get("lat") is None,
          json.dumps({"owner": getattr(nav, "_run_id", None),
                      "recording": live["id"],
                      "lat": nav.fix_at(time.time()).get("lat")}))
    live["id"] = None
    _end(nav)

    # -- the tick still does its original job -------------------------------
    rd2 = FakeReader()
    rd2.gateway_online = False
    rd2.fix = False
    nav2 = _mknav(h, rd2, run_active=lambda: None)
    armed3, tok3 = _begin(nav2, rd2.gateway_online)
    _bind(nav2, "r-abandoned", tok3)
    check("a no-NMEA run is still armed for its whole line", armed3 is True,
          repr(armed3))
    rd2.gateway_online = True
    rd2.fix = True
    _end(nav2)                              # the tick, with no run active
    check("a latch left by a run that ended by some other path is still "
          "released by the nav tick", nav2.armed() is False,
          repr(nav2.armed()))

    # -- and now through the real handler, with a real run ------------------
    saved = h.rig.nav
    rd3 = FakeReader()
    rd3.gateway_online = True
    rd3.fix = True
    h.rig.nav = _mknav(h, rd3,
                       run_active=getattr(h.rig, "_recording_run_id", None))
    started = False
    try:
        code, doc = h.post("/api/run/start", {"label": "audit-latch"})
        started = doc.get("ok") is True
        check("a run starts for the duplicate-Start test", started,
              json.dumps(doc)[:120])
        code, doc2 = h.post("/api/run/start", {"label": "audit-latch"})
        check("the second Start is refused as 'run already active'",
              doc2.get("ok") is False
              and "already active" in (doc2.get("error") or ""),
              json.dumps(doc2)[:120])
        rd3.gateway_online = False          # the bus drops mid-transect
        rd3.fix = False
        code, doc3 = h.get("/api/nav")
        check("the refused Start did NOT strip the recording run's latch: the "
              "bus drop leaves the position EMPTY",
              doc3.get("lat") is None and doc3.get("fix_kind") == "none",
              json.dumps({k: doc3.get(k) for k in
                          ("lat", "fix_kind", "static_fix")}))
    finally:
        if started:
            h.post("/api/run/stop", {"drain": False})
        h.rig.nav = saved


# ===========================================================================
# D10 — the post-drain ingest is not silent
# ===========================================================================
class StubIngest:
    """ingest.ingest's contract for the post-drain caller: a report whose
    `totals` say what was matched, for a caller that swallows the log."""

    totals = {}
    calls = []

    @staticmethod
    def ingest(card_dir, log=print, **kw):
        StubIngest.calls.append(card_dir)
        log("stub ingest")
        return {"runs": [], "leftover": [], "totals": dict(StubIngest.totals)}


def _ingest_events(h, seq):
    return [e for e in h.rig.events.since(seq)["events"]
            if "post-drain ingest" in (e.get("msg") or "")]


def t_ingest_report(h):
    sect("D10 rigd: the automatic post-drain ingest reports what it matched")
    import ingest as ingestmod
    h.wait_connected("cam1")
    h.rig._drain_wedged.pop("cam1", None)
    real_drainer, real_ingest = rigd.draindrv.Drainer, ingestmod.ingest
    rigd.draindrv.Drainer = StubDrainer
    ingestmod.ingest = StubIngest.ingest
    StubDrainer.files, StubDrainer.per_file_s = 2, 0.0
    try:
        # The expensive case: the card was emptied and NOTHING matched.
        StubDrainer.made, StubIngest.calls = [], []
        StubIngest.totals = {"runs": 1, "matched": 0, "unmatched": 169,
                             "raw": 0, "conflicts": 0, "cards": 169,
                             "cards_timed": 169, "leftover": 169,
                             "exif_mismatch": 0, "ambiguous": []}
        seq = h.rig.events._seq
        r = h.rig.start_drain(["cam1"])
        check("the drain starts", r.get("ok") is True, json.dumps(r)[:110])
        check("the drain finishes",
              wait_for(lambda: not h.rig.drain_status()["active"], 10.0),
              json.dumps(h.rig.drain_status())[:120])
        check("the post-drain ingest ran against the node's staging dir",
              StubIngest.calls
              and os.path.basename(StubIngest.calls[0]) == "cam1",
              str(StubIngest.calls))
        evs = _ingest_events(h, seq)
        check("a drain that emptied the card and matched NOTHING is no longer "
              "invisible", len(evs) == 1, "%d events" % len(evs))
        if evs:
            check("...it is a WARNING that names the counts",
                  evs[0].get("sev") == "warn" and "0 matched" in evs[0]["msg"]
                  and "169 unmatched" in evs[0]["msg"],
                  json.dumps(evs[0])[:190])
            check("...and carries the totals for the AI watcher to read",
                  ((evs[0].get("ctx") or {}).get("ingest") or {})
                  .get("cards") == 169, json.dumps(evs[0].get("ctx"))[:160])
        check("/api/drain shows the ingest result beside 'deleted from card'",
              ((h.get("/api/drain")[1].get("last") or {}).get("ingest") or {})
              .get("unmatched") == 169,
              json.dumps(h.get("/api/drain")[1].get("last"))[:190])

        # The ordinary case: matched, no conflicts - one info line.
        StubDrainer.made, StubIngest.calls = [], []
        StubIngest.totals = {"runs": 1, "matched": 169, "unmatched": 0,
                             "raw": 169, "conflicts": 0, "cards": 169,
                             "cards_timed": 169, "leftover": 0,
                             "exif_mismatch": 0, "ambiguous": []}
        seq = h.rig.events._seq
        h.rig.start_drain(["cam1"])
        wait_for(lambda: not h.rig.drain_status()["active"], 10.0)
        evs = _ingest_events(h, seq)
        check("a clean ingest says how many frames it placed, at info",
              len(evs) == 1 and evs[0].get("sev") == "info"
              and "169 RAW placed" in evs[0]["msg"],
              json.dumps(evs[0] if evs else {})[:190])

        # An ambiguous attribution is a "look now", even with matches.
        StubDrainer.made, StubIngest.calls = [], []
        StubIngest.totals = dict(StubIngest.totals,
                                 conflicts=2, ambiguous=["r1 cam1"])
        seq = h.rig.events._seq
        h.rig.start_drain(["cam1"])
        wait_for(lambda: not h.rig.drain_status()["active"], 10.0)
        evs = _ingest_events(h, seq)
        check("conflicts and ambiguous attributions are raised as warnings",
              len(evs) == 1 and evs[0].get("sev") == "warn"
              and "CONFLICTS" in evs[0]["msg"]
              and "AMBIGUOUS" in evs[0]["msg"],
              json.dumps(evs[0] if evs else {})[:190])
    finally:
        rigd.draindrv.Drainer = real_drainer
        ingestmod.ingest = real_ingest
        StubDrainer.files, StubDrainer.per_file_s = 6, 0.02


# ===========================================================================
# D11 — JSON Infinity/NaN are a clean 400, not a 500 with a traceback
# ===========================================================================
def t_nonfinite(h):
    sect("D11 rigd: JSON Infinity/NaN never reach a coercer as an exception")
    for v in (float("inf"), float("-inf"), float("nan")):
        try:
            rigd.want_int(v, '"steps"', -30, 30)
            ok, why = False, "accepted %r" % v
        except rigd.BadRequest as e:
            ok, why = True, str(e)
        except Exception as e:                                 # noqa: BLE001
            ok, why = False, "%s: %s" % (type(e).__name__, e)
        check("want_int(%r) is a BadRequest, not a leaked exception" % v,
              ok, why)

    # json.loads accepts these literals by default, so they arrive intact.
    for path, raw in (("/api/ev", b'{"steps": Infinity}'),
                      ("/api/ev", b'{"steps": NaN}'),
                      ("/api/exposure",
                       b'{"node":"cam1","which":"iso","value": -Infinity}'),
                      ("/api/calibrate", b'{"samples": NaN}')):
        seq = h.rig.events._seq
        code, doc = h.req("POST", path, raw=raw)
        check("POST %s %s -> 4xx, not a 500 with a Python exception"
              % (path, raw.decode()), is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:110]))
        errs = [e for e in h.rig.events.since(seq)["events"]
                if e.get("kind") == "http" and e.get("sev") == "error"]
        check("...and it writes no error line into rigd.jsonl",
              not errs, json.dumps(errs[:1])[:160])


# ===========================================================================
# D12 — a body that answered "busy" is not a stalled card
# ===========================================================================
class _StubMon:
    """One monitor with exactly the status body a check needs. Anomalies reads
    snapshot()/name_/clock_offset_info()/is_present() and nothing else."""

    def __init__(self, name, status, state=None, conv=None, present=True,
                 camera_enabled=True):
        self.name_ = name
        self.host = "127.0.0.9"
        self.suspend_control = False
        self.camera_enabled = camera_enabled
        self.optional = False
        self._present = present
        self._snap = {"state": state or rigcore.NodeMonitor.CONNECTED,
                      "status": status, "health": {}, "age_s": 0.1,
                      "convergence": conv or {}}

    def snapshot(self):
        return dict(self._snap)

    def clock_offset_info(self):
        return {"offset_s": None, "rtt_ms_best": None, "n": 0}

    def is_present(self):
        """Is the NODE in the fleet? False only for an optional node never
        seen. A stub that omits this does not stand in for a NodeMonitor any
        more, and the section dies with AttributeError rather than failing a
        check - which is exactly what happened when is_present() was added."""
        return self._present

    def is_capturing(self):
        """Does this node's CAMERA take part? Narrower: a camera switched off
        leaves the runs and the camera alarms, but its Pi stays in the fleet."""
        return self._present and self.camera_enabled


def t_busy(h):
    sect("D12 rigd: a busy/draining body is not diagnosed as a stalled card")
    an = h.rig.anomalies
    saved, saved_last = an.monitors, an._last
    # PROTOCOL.md: the degraded answer when the SDK mutex is held past 4.5 s.
    busy = {"connected": True, "busy": True, "model": "", "id": "", "log": []}
    # The real thing: the body answers promptly with an empty property table.
    stalled = {"connected": True, "writable": {}, "iso": "ISO 0",
               "isoValue": 0, "slotStatus": "OK"}
    try:
        an._last = {}
        an.monitors = [_StubMon("cam1", busy)]
        a = an.scan()
        check("the degraded busy status body does not raise body_locked",
              not _kinds(a, "body_locked"),
              json.dumps([x["kind"] for x in a])[:160])

        an._last = {}
        an.monitors = [_StubMon("cam1", busy,
                                conv={"synced": False, "diverged": ["iso"]})]
        a = an.scan()
        check("...nor are its missing keys read as a settings divergence",
              not _kinds(a, "settings_divergent"),
              json.dumps([x["kind"] for x in a])[:160])

        an._last = {}
        drained = _StubMon("cam1", dict(stalled, controlMode="transfer"))
        an.monitors = [drained]
        a = an.scan()
        check("a body in transfer mode (a card drain empties the property "
              "table by design) is not called a stalled card either",
              not _kinds(a, "body_locked"),
              json.dumps([x["kind"] for x in a])[:160])

        an._last = {}
        held = _StubMon("cam1", stalled)
        held.suspend_control = True
        an.monitors = [held]
        a = an.scan()
        check("nor is one a drain currently holds",
              not _kinds(a, "body_locked"),
              json.dumps([x["kind"] for x in a])[:160])

        an._last = {}
        an.monitors = [_StubMon("cam1", stalled)]
        a = an.scan()
        bl = _kinds(a, "body_locked")
        check("a REAL card stall is still raised, at bad severity, with the "
              "card remedy",
              len(bl) == 1 and bl[0]["sev"] == "bad"
              and "full-format" in bl[0]["suggested_action"],
              json.dumps([x["kind"] for x in a])[:160])
    finally:
        an.monitors = saved
        an._last = saved_last


# ===========================================================================
# D6 — stale-status anomalies, one event per anomaly, IMU negative cache
# ===========================================================================
def t_misc(h):
    sect("D6 rigd: no anomalies on a dead node's stale status; one event each")
    h.wait_connected("cam1")
    h.wait_connected("cam2")
    an = h.rig.anomalies
    n1 = h.node("cam1")
    # Stage the two fields on a LIVE node first, so the monitor's cached
    # status carries them, then take the node away. NodeMonitor never clears
    # status on the OFFLINE transition - that is the whole defect.
    n1.battery = 9
    n1.label_override = {"priorityKeyLabel": "Camera position"}
    check("the low battery / lost-priority status reached the monitor",
          wait_for(lambda: (h.mon("cam1").snapshot().get("status") or {})
                   .get("battery") == 9, 6.0))
    a = an.scan()
    check("both are raised while the node is genuinely CONNECTED",
          _kinds(a, "battery_low") and _kinds(a, "pc_control_lost"),
          json.dumps([x["kind"] for x in a]))
    # The node stops answering usefully on BOTH surfaces, which is the OFFLINE
    # rigd judges anomalies against. Staged as garbage answers rather than
    # n1.down(): down() releases the fake's loopback port, and another test
    # process on this shared worktree can bind it and answer as a live camera
    # (see the staging note in t_drain) — the socket stays ours this way.
    n1.set_fault(surface="both", badjson=True)
    check("cam1 goes OFFLINE",
          wait_for(lambda: h.mon("cam1").snapshot()["state"]
                   == rigcore.NodeMonitor.OFFLINE, 10.0),
          h.mon("cam1").snapshot()["state"])
    a = an.scan()
    kinds = [(x["kind"], x["node"]) for x in a]
    check("battery_low is NOT raised on a node that is not powered",
          not [x for x in _kinds(a, "battery_low") if x["node"] == "cam1"],
          json.dumps(kinds))
    check("pc_control_lost is NOT raised on a node that is not answering",
          not [x for x in _kinds(a, "pc_control_lost") if x["node"] == "cam1"],
          json.dumps(kinds))
    check("node_offline still says the one true thing",
          [x for x in _kinds(a, "node_offline") if x["node"] == "cam1"],
          json.dumps(kinds))

    # -- the IMU negative cache -------------------------------------------
    # The IMU hangs off cam1, which is now down: every probe used to block the
    # full 3 s, and the UI polls /api/imu/window five times a second.
    h.rig._imu_node = None
    h.rig._imu_dead_until = 0.0    # absent pre-fix; set so the cache starts cold
    t0 = time.time()
    answers = []
    for _ in range(6):
        answers.append(h.get("/api/imu/window?t0=0"))
    spent = time.time() - t0
    check("a dead IMU answers present:false rather than an error",
          all(c == 200 and d.get("present") is False for c, d in answers),
          json.dumps([(c, d.get("present")) for c, d in answers]))
    # Pre-fix: 6 x 2 nodes x 3 s. The first poll may still pay one probe.
    check("six IMU window polls against a dead node do not each pay the probe "
          "timeout", spent < 8.0, "%.1f s for 6 polls" % spent)
    check("the failed IMU probe is cached for at least 10 s",
          getattr(h.rig, "IMU_DEAD_TTL_S", 0) >= 10.0
          and getattr(h.rig, "_imu_dead_until", 0) > time.monotonic(),
          "ttl=%s" % getattr(h.rig, "IMU_DEAD_TTL_S", None))
    n1.clear_faults()
    check("cam1 back", h.wait_connected("cam1", 15.0))
    n1.label_override = {}
    n1.battery = 87

    # -- scan() is serialised ---------------------------------------------
    # Two /api/anomalies polls and the 2.5 s loop routinely overlap; both used
    # to compute `cur - self._last` before either assigned it, so one new
    # anomaly was journalled two or three times.
    an._last = {}
    n1.set_fault(surface="ilx", http500=True)     # -> ilx_down / camera_absent
    wait_for(lambda: not h.mon("cam1").is_connected(), 10.0)
    an._last = {}
    seq = h.rig.events._seq
    threads = [threading.Thread(target=an.scan) for _ in range(6)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    fired = {}
    for e in h.rig.events.since(seq)["events"]:
        if e.get("kind") in ("ilx_down", "camera_absent", "node_offline"):
            fired[e["kind"]] = fired.get(e["kind"], 0) + 1
    check("six concurrent scans emit each new anomaly exactly ONCE",
          fired and all(v == 1 for v in fired.values()), json.dumps(fired))
    n1.set_fault(surface="ilx", http500=False)
    h.wait_connected("cam1", 15.0)

    # -- D7: /api/capture passes the run manager's result through verbatim --
    saved = h.rig.runmgr.capture_once
    marker = {"ok": True, "results": {"cam1": {"ok": True, "late_ms": 2.0}},
              "host_offset_s": 0.187, "skew_ms": 0.6}
    seen = []

    def _cap(af=False, target=None):
        seen.append(af)
        return dict(marker)

    h.rig.runmgr.capture_once = _cap
    try:
        code, doc = h.post("/api/capture", {})
        check("D7 /api/capture surfaces host_offset_s untouched",
              code == 200 and doc == marker, json.dumps(doc)[:160])
        code, doc = h.post("/api/capture", {"af": "yes"})
        check("/api/capture refuses a non-bool af", is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:90]))
        code, doc = h.post("/api/capture", {"af": True})
        check("/api/capture refuses af:true - the rig is manual focus on "
              "every path", is400(code, doc),
              "%s %s" % (code, json.dumps(doc)[:120]))
        check("no autofocus request ever reached the run manager",
              seen == [False], str(seen))
    finally:
        h.rig.runmgr.capture_once = saved

    # -- the bounded shutdown ---------------------------------------------
    check("Rig.stop bounds run finalisation under launchd's 20 s budget",
          getattr(rigd.Rig, "STOP_DEADLINE_S", 999) <= 15.0,
          str(getattr(rigd.Rig, "STOP_DEADLINE_S", None)))
    plist = os.path.join(os.path.dirname(RIG), "deploy", "rigd.launchd.plist")
    try:
        with open(plist) as fh:
            txt = fh.read()
    except OSError:
        txt = ""
    check("the launchd agent sets ExitTimeOut so a healthy slow stop is not "
          "killed", "ExitTimeOut" in txt and "<integer>60</integer>" in txt,
          "ExitTimeOut present: %s" % ("ExitTimeOut" in txt))


# ===========================================================================
# D8 — POST /api/reconcile is not the exposure apply button
# ===========================================================================
# rigd called RIG.settings.reconcile_all(force=True) with no exposure= at all.
# reconcile_all's contract is "exposure=None follows force", and a bare
# force=True counts as an explicit fleet apply — so the manual "push the vector
# at the bodies now" kick silently re-pushed `desired`'s aperture/shutter/ISO/EV
# onto every camera and destroyed a deliberate per-camera exposure split (the
# operator balancing cam2 against an unequal strobe, or against vignette). The
# exposure apply button is POST /api/settings; this is not it.
#
# The two halves have to be checked TOGETHER, or the cheapest way to pass is to
# make /api/reconcile do nothing at all: the split must survive it, and a
# non-exposure divergence must still be re-pushed by the same POST.
def t_reconcile(h):
    sect("D8 POST /api/reconcile forces the vector without wiping a "
         "per-camera exposure split")
    check("cam1 connected", h.wait_connected("cam1", 15.0))
    check("cam2 connected", h.wait_connected("cam2", 15.0))
    n1, n2 = h.node("cam1"), h.node("cam2")
    m1, m2 = h.mon("cam1"), h.mon("cam2")
    code, desired0 = h.get("/api/settings")
    if code != 200 or not isinstance(desired0, dict) or "iso" not in desired0:
        check("GET /api/settings answers the desired vector", False,
              "%s %s" % (code, json.dumps(desired0)[:110]))
        return
    want_iso, want_ft = desired0["iso"], desired0["filetype"]

    # Settle first. The fakes boot on the SURVEY vector, which differs from
    # DEFAULT_DESIRED in filetype/imagesize/white balance, and a READABLE
    # non-exposure field off target is the documented "this body has reset"
    # tell: while one is outstanding the idle 3 s loop re-forces the whole
    # exposure vector by design, which would wipe the split below without the
    # seam under test being involved at all.
    h.post("/api/reconcile", {})
    synced = wait_for(lambda: (m1.convergence.get("synced") is True
                               and m2.convergence.get("synced") is True), 25.0)
    check("both bodies are converged before the split is staged", synced,
          "cam1=%s cam2=%s" % (json.dumps(m1.convergence)[:100],
                               json.dumps(m2.convergence)[:100]))
    check("no reconnect tell is pending (it forces one exposure pass by "
          "design)", not getattr(m1, "_exposure_force", False)
          and not getattr(m2, "_exposure_force", False))

    # -- the operator balances cam2 against an unequal strobe, per-camera ---
    code, doc = h.post("/api/exposure", {"node": "cam2", "which": "iso",
                                         "value": 3200})
    check("POST /api/exposure moves cam2 ALONE to ISO 3200",
          code == 200 and isinstance(doc, dict) and doc.get("ok") is True
          and n2.raw("iso") == 3200 and n1.raw("iso") == want_iso,
          "cam1 iso=%s cam2 iso=%s %s"
          % (n1.raw("iso"), n2.raw("iso"), json.dumps(doc)[:80]))

    # -- a NON-exposure divergence only a FORCED pass can repair ------------
    # filetype with its readback key hidden is an ilxctl build that predates
    # contract C2: converged BLIND off the last-pushed cache. Force once to
    # fill that cache, then move the body underneath it. The idle loop cannot
    # see this field and will not write it, so whatever repairs it came from
    # the POST under test rather than from the background loop — and a blind
    # field is not evidence of a reset, so staging it does not itself license
    # an exposure write.
    with n2._lock:
        n2.hide_keys = {"filetypeValue"}
    check("cam2 now looks like a pre-C2 ilxctl (no filetype readback)",
          wait_for(lambda: "filetypeValue" not in
                   (m2.snapshot().get("status") or {}), 8.0))
    h.post("/api/reconcile", {})           # fills the blind cache at target
    n2.drift(filetype=1)                   # body silently drops to JPEG-only
    check("cam2's file type is off the survey vector, and invisible",
          n2.raw("filetype") == 1 and want_ft != 1,
          "filetype=%s want=%s" % (n2.raw("filetype"), want_ft))

    # -- the POST under test ------------------------------------------------
    n1.clear_counts()
    n2.clear_counts()
    code, doc = h.post("/api/reconcile", {})
    check("POST /api/reconcile answers ok",
          code == 200 and isinstance(doc, dict) and doc.get("ok") is True,
          "%s %s" % (code, json.dumps(doc)[:80]))
    exp_pushes = (n1.pushed("iso", "aperture", "shutter", "expcomp")
                  + n2.pushed("iso", "aperture", "shutter", "expcomp"))
    check("D8 a deliberate per-camera exposure split SURVIVES "
          "POST /api/reconcile",
          n2.raw("iso") == 3200 and n1.raw("iso") == want_iso,
          "cam1 iso=%s cam2 iso=%s (pre-fix: cam2 back on desired %s)"
          % (n1.raw("iso"), n2.raw("iso"), want_iso))
    check("D8 it writes no exposure field to either body at all",
          not exp_pushes, "%d exposure pushes: %s"
          % (len(exp_pushes), str([(p[1], p[2]) for p in exp_pushes[:6]])))
    check("D8 ...while the SAME POST still re-pushes a non-exposure "
          "divergence", n2.raw("filetype") == want_ft
          and len(n2.pushed("filetype")) >= 1,
          "filetype=%s want=%s after %d filetype pushes"
          % (n2.raw("filetype"), want_ft, len(n2.pushed("filetype"))))

    sect("D8 POST /api/settings IS the exposure apply, and still propagates")
    code, doc = h.post("/api/settings", {"iso": 1600})
    check("an explicit exposure apply is accepted",
          code == 200 and isinstance(doc, dict)
          and (doc.get("applied") or {}).get("iso") == 1600,
          "%s %s" % (code, json.dumps(doc)[:110]))
    check("D8 the explicit apply reaches BOTH bodies and ends the split",
          n1.raw("iso") == 1600 and n2.raw("iso") == 1600,
          "cam1 iso=%s cam2 iso=%s" % (n1.raw("iso"), n2.raw("iso")))

    # The other half of the same rule through rigd's own surface: a
    # NON-exposure apply is not an exposure apply either. focus_mode is
    # already 1 (the rig is always MF), so this stages no divergence of its
    # own and the 3 s loop has nothing to react to while the check runs.
    h.post("/api/exposure", {"node": "cam2", "which": "iso", "value": 3200})
    code, doc = h.post("/api/settings", {"focus_mode": 1})
    check("a non-exposure apply through /api/settings leaves a fresh split "
          "alone",
          code == 200 and isinstance(doc, dict)
          and (doc.get("applied") or {}).get("focus_mode") == 1
          and n2.raw("iso") == 3200 and n1.raw("iso") == 1600,
          "cam1 iso=%s cam2 iso=%s applied=%s"
          % (n1.raw("iso"), n2.raw("iso"),
             json.dumps((doc or {}).get("applied"))))

    # -- leave the fleet matched for the sections that follow ---------------
    with n2._lock:
        n2.hide_keys = set()
    h.post("/api/settings", {"iso": want_iso})
    h.post("/api/reconcile", {})
    check("the fleet is left matched on the desired vector",
          wait_for(lambda: (n1.raw("iso") == want_iso
                            and n2.raw("iso") == want_iso
                            and n2.raw("filetype") == want_ft), 15.0),
          "cam1 iso=%s cam2 iso=%s cam2 filetype=%s"
          % (n1.raw("iso"), n2.raw("iso"), n2.raw("filetype")))


# ===========================================================================
def suite(opts=None):
    h = Harness()
    try:
        # Each section is independent, and on PRE-fix rigd several of them
        # raise (no Anomalies._skew_streak, no start_drain(auto=), no
        # /api/drain/cancel). Keep going so a pre-fix run reports the whole
        # list of reproduced defects rather than stopping at the first.
        # `reconcile` runs FIRST, on pristine fakes: it needs a fleet with no
        # outstanding divergence and no pending reconnect tell, both of which
        # legitimately force an exposure pass. It restores what it changed.
        for name, fn in (("reconcile", t_reconcile),
                         ("bodies", t_bodies), ("inputs", t_inputs),
                         ("clock", t_clock), ("drain", t_drain),
                         ("ingest", t_ingest_report),
                         ("nonfinite", t_nonfinite), ("busy", t_busy),
                         ("static", t_static), ("latch", t_latch),
                         ("misc", t_misc)):
            try:
                fn(h)
            except Exception as e:                             # noqa: BLE001
                import traceback
                traceback.print_exc()
                check("audit_rigd %s section ran to completion" % name,
                      False, repr(e))
    finally:
        h.close()
    note("audit_rigd drove the real rigd Handler over an ephemeral loopback "
         "port against two fakenode fakes; drain.Drainer was stubbed for the "
         "cancel/wedge paths (drain.py itself is soaktest's suite_drain)")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    t0 = time.time()
    suite(a)
    import soaktest
    print("\n%d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - t0))
    if soaktest.FAIL:
        print("FAILED: " + ", ".join(soaktest.FAIL))
    sys.exit(1 if soaktest.FAIL else 0)

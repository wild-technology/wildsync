#!/usr/bin/env python3
"""soaktest — the regression + soak gate the rig has to clear before the water.

Everything here runs against `rig/fakenode.py`: in-process HTTP stand-ins for
the camera nodes that speak the ilxctl + piagent APIs from rig/PROTOCOL.md and
can be made to misbehave on demand. No hardware, no fleet, no camera is touched
— a network guard installed at start-up refuses any request that is not to a
127.x loopback address, so this file cannot reach 192.168.1.x even by mistake.

What it drives, with real code (rigcore.NodeMonitor, rigcore.SettingsManager,
run.RunManager — none of it stubbed):

  fake      the fake node is itself trustworthy (both label and raw status keys,
            byte-exact frame transfer, real EXIF, faults that look like faults)
  monitor   OFFLINE -> REACHABLE -> CAM_CONNECTED and back under drop, hang,
            HTTP 500, malformed JSON, a lying /api/connect and a PoE reboot;
            backoff honoured, no wedge, no busy-loop
  settings  convergence: knocked out of sync, restored; identical across
            cameras; no thrash when a field is unset or the node is offline;
            divergence alerts name node + field
  runmgr    a run end to end: folder layout, Cam#_YYYYMMDD_hhmmss.ss.jpg,
            the 23-column flight_log contract byte-for-byte, a frame with no
            nav fix and no IMU sample, duplicate + late frames, stop mid-pull
  pull      fault injection on the pull path: disk full, permission denied,
            truncated JPEG, node vanishing mid-transfer — flight_log stays
            intact and every failure leaves an event
  resource  threads, file descriptors, event ring and memory stay bounded

Usage
    python3 rig/soaktest.py                 # full regression, a few minutes
    python3 rig/soaktest.py --quick         # skip the slow timeout probes
    python3 rig/soaktest.py --only monitor,settings
    python3 rig/soaktest.py --soak 300      # 300 s of randomized fault storms

Exit code is nonzero if any check fails. Checks that encode a contract from
PROTOCOL.md are reported a second time, with file:line, under DEFECTS — those
are bugs in code this harness does not own.
"""

import argparse
import csv
import errno
import gc
import json
import os
import random
import re
import shutil
import stat
import sys
import tempfile
import threading
import time
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
# rig/ itself has to be importable, not just rig/tests/: everything under test
# (rigcore, run, navlog) lives one level up. Without this the documented
# invocation `python3 rig/tests/soaktest.py` dies on `import rigcore` and the
# whole gate is silently unrunnable from the README's own instructions.
sys.path.insert(0, RIG)

# PROTOCOL.md is at <repo>/docs/, not beside this file. Pointed at HERE it never
# resolved, and every contract test that reads it failed as an unrelated
# FileNotFoundError mid-suite rather than as a checkable assertion.
PROTOCOL_PATH = os.path.join(os.path.dirname(RIG), "docs", "PROTOCOL.md")
FNAME_RE = re.compile(r"^Cam(\d+)_(\d{8})_(\d{6})\.(\d{2})\.jpg$")

PASS, FAIL, NOTES, DEFECTS = [], [], [], []


# ---------------------------------------------------------------------------
# Result reporting — same shape as rig/selftest.py.
# ---------------------------------------------------------------------------
def check(name, cond, detail=""):
    cond = bool(cond)
    (PASS if cond else FAIL).append(name)
    print("  %s %s%s" % ("PASS" if cond else "FAIL", name,
                         (" — " + detail) if detail else ""))
    return cond


def contract(name, cond, ref, scenario, detail=""):
    """A check that encodes a promise from PROTOCOL.md. If it fails, the defect
    lives in code this harness does not own — record where and how to hit it."""
    ok = check(name, cond, detail)
    if not ok:
        DEFECTS.append({"check": name, "ref": ref, "scenario": scenario,
                        "detail": detail})
    return ok


def note(msg):
    NOTES.append(msg)
    print("  note %s" % msg)


def sect(t):
    print("\n== %s" % t)


def safe_alive(t):
    """Thread.is_alive() raises TypeError on NodeMonitor/PullWorker once they
    have exited (both shadow Thread._stop with an Event). Treat that as dead —
    the TypeError only happens after the thread's state lock is released."""
    try:
        return t.is_alive()
    except TypeError:
        return False


def safe_join(t, timeout=2.0):
    try:
        t.join(timeout=timeout)
    except (TypeError, RuntimeError):       # shadowed _stop, or never started
        pass


def wait_for(pred, timeout=5.0, interval=0.05):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(interval)
    return pred()


# ---------------------------------------------------------------------------
# Network guard — installed before any rig code runs. The live fleet is off
# limits: another engineer owns the camera on 192.168.1.203.
# ---------------------------------------------------------------------------
BLOCKED = []


def _install_netguard():
    import urllib.request
    import rigcore as _rc

    def _ok(url):
        host = urlparse(url if isinstance(url, str) else
                        getattr(url, "full_url", "")).hostname or ""
        return host.startswith("127.")

    def _remap(url):
        # Fakes on hosts without loopback aliases (macOS) listen on 127.0.0.1
        # with per-node ports; rewrite outgoing URLs with the same mapping the
        # fake used to bind. Identity on Linux. The guard above always judges
        # the NOMINAL address, so nothing off-loopback can slip through here.
        if not isinstance(url, str):
            return url
        import fakenode as _fk
        u = urlparse(url)
        host, port = u.hostname or "", u.port
        if port is None or not host.startswith("127.") or host == "127.0.0.1":
            return url
        h2, p2 = _fk.loopback_map(host, port)
        if (h2, p2) == (host, port):
            return url
        return url.replace("//%s:%d" % (host, port),
                           "//%s:%d" % (h2, p2), 1)

    def _wrap(fn, label):
        def w(url, *a, **kw):
            if not _ok(url):
                BLOCKED.append(str(url))
                raise AssertionError(
                    "soaktest netguard: refused %s call to %s (fakes only)"
                    % (label, url))
            return fn(_remap(url), *a, **kw)
        return w

    _real_urlopen = urllib.request.urlopen
    urllib.request.urlopen = _wrap(_real_urlopen, "urlopen")
    _rc.http_json = _wrap(_rc.http_json, "http_json")
    _rc.http_bytes = _wrap(_rc.http_bytes, "http_bytes")
    return _rc


rigcore = _install_netguard()
import run as runmod                                          # noqa: E402
runmod.http_json = rigcore.http_json          # run.py imported them by value
runmod.http_bytes = rigcore.http_bytes
from fakenode import FACTORY, ISO_AUTO, FakeNode   # noqa: E402

# Unhandled exceptions in any background thread are a failure by themselves.
UNHANDLED = []
_orig_excepthook = threading.excepthook


def _thread_excepthook(args):
    if args.exc_type is SystemExit:
        return
    UNHANDLED.append("%s in %s: %s" % (args.exc_type.__name__,
                                       getattr(args.thread, "name", "?"),
                                       args.exc_value))
    _orig_excepthook(args)


threading.excepthook = _thread_excepthook


# ---------------------------------------------------------------------------
# Process resource sampling
# ---------------------------------------------------------------------------
def fd_count():
    try:
        return len(os.listdir("/proc/self/fd"))
    except OSError:
        return -1


def rss_mb():
    try:
        with open("/proc/self/statm") as fh:
            pages = int(fh.read().split()[1])
        return pages * os.sysconf("SC_PAGE_SIZE") / 1e6
    except (OSError, ValueError, IndexError):
        return -1.0


def thread_names():
    return sorted(t.name for t in threading.enumerate())


def sample():
    return {"threads": threading.active_count(), "fds": fd_count(),
            "rss_mb": round(rss_mb(), 1)}


# ---------------------------------------------------------------------------
# A fleet of fakes wired into the real engine.
# ---------------------------------------------------------------------------
NAV_FIX = {"lat": 40.0001234, "lon": -105.0004321, "depth_m": 12.34,
           "heading_mag_deg": 211.4, "sog_mps": 1.2, "sats": 9}
NAV_KEYS = ("lat", "lon", "xutm", "yutm", "utm_zone", "depth_m",
            "heading_mag_deg", "sog_mps", "sats")


class FakeNav:
    """A nav reader with the surface run.py drives: fix_at() for the capture
    instant, snapshot() for the UI, set_raw_hook() for nmea_raw.log."""

    def __init__(self, fix=True):
        self.fix = fix
        self.hook = None

    def _view(self, at):
        s = {k: None for k in NAV_KEYS}
        s.update({"local_epoch": at, "epoch": at, "age_s": None,
                  "valid": False, "stale": True, "gateway_online": True,
                  "time_source": "gps" if self.fix else "jetson"})
        if self.fix:
            s.update(NAV_FIX)
            s.update(valid=True, stale=False, age_s=0.05)
            try:
                import nav as navmod
                s["xutm"], s["yutm"], s["utm_zone"] = \
                    navmod.latlon_to_utm(s["lat"], s["lon"])
            except Exception:                                 # noqa: BLE001
                pass
        # flight_log-shaped aliases, as nav.NavReader.fix_at supplies them
        s["long"] = s["lon"]
        s["depth_from_xplore9"] = s["depth_m"]
        s["heading_mag_xplore"] = s["heading_mag_deg"]
        return s

    def fix_at(self, epoch=None, max_age_s=None):
        return self._view(time.time() if epoch is None else float(epoch))

    def snapshot(self):
        return self._view(time.time())

    def set_raw_hook(self, hook):
        self.hook = hook

    def feed(self, line):
        h = self.hook
        if h is None:
            return
        if hasattr(h, "write_line"):
            h.write_line(time.time(), time.monotonic(), line)
        else:
            h(time.time(), line)


def shipping_nav_reader():
    """The real rig/nav.py reader, constructed but never opened: no serial
    port, no fix. Used to check run.py and nav.py still agree on the API."""
    import nav as navmod
    return navmod.NavReader()


class Env:
    """Temp home, event log, fakes, monitors, settings, timesync, run manager."""

    def __init__(self, specs, poll=0.5, threaded=False, nav=None,
                 imu_node="cam3"):
        self.dir = tempfile.mkdtemp(prefix="wildsync-soak-")
        rigcore.RIG_HOME = os.path.join(self.dir, "rig")
        rigcore.DESIRED_PATH = os.path.join(rigcore.RIG_HOME, "desired.json")
        rigcore.RUNS_DIR = runmod.RUNS_DIR = os.path.join(self.dir, "runs")
        os.makedirs(rigcore.RIG_HOME, exist_ok=True)
        self.events = rigcore.EventLog(
            path=os.path.join(rigcore.RIG_HOME, "rigd.jsonl"), ring=3000)
        self.nodes, self.monitors = {}, []
        for spec in specs:
            name, host, cam_num = spec[0], spec[1], spec[2]
            kw = spec[3] if len(spec) > 3 else {}
            self.nodes[name] = FakeNode(name, host, cam_num=cam_num, **kw)
            m = rigcore.NodeMonitor({"name": name, "cam_num": cam_num,
                                     "host": host}, self.events, poll=poll)
            self.monitors.append(m)
        self.settings = rigcore.SettingsManager(self.monitors, self.events)
        self.timesync = rigcore.TimeSync(self.events)
        self.nav = nav
        self.runmgr = runmod.RunManager(self.monitors, self.settings,
                                        self.timesync, self.events, nav,
                                        imu_node=imu_node)
        self.threaded = threaded
        if threaded:
            for m in self.monitors:
                m.start()

    # -- handles ------------------------------------------------------------
    def mon(self, name):
        return next(m for m in self.monitors if m.name_ == name)

    def node(self, name):
        return self.nodes[name]

    def tick(self, n=1):
        for _ in range(n):
            for m in self.monitors:
                m._tick()

    def wait_state(self, name, state, timeout=6.0):
        m = self.mon(name)
        return wait_for(lambda: m.state == state, timeout) and m.state == state

    # -- events -------------------------------------------------------------
    def seq(self):
        return self.events._seq

    def evs(self, since=0, kind=None, node=None):
        out = self.events.since(since, limit=100000)["events"]
        if kind:
            out = [e for e in out if e["kind"] == kind]
        if node:
            out = [e for e in out if e["node"] == node]
        return out

    def close(self):
        try:
            if self.runmgr.active:
                self.runmgr.stop()
        except Exception:                                     # noqa: BLE001
            pass
        for m in self.monitors:
            m.stop()
        for m in self.monitors:
            safe_join(m, 3)
        for n in self.nodes.values():
            n.close()
        shutil.rmtree(self.dir, ignore_errors=True)


CONVERGE_KEYS = ("aperture", "shutter", "iso", "drive", "filetype",
                 "imagesize", "transsize", "store_dest")


def read_flight(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    return (rows[0] if rows else []), rows[1:]


def protocol_flight_header():
    """The header line PROTOCOL.md declares, verbatim."""
    with open(PROTOCOL_PATH) as fh:
        lines = fh.read().splitlines()
    for i, ln in enumerate(lines):
        if "flight_log.csv header (exact)" in ln:
            for cand in lines[i + 1:i + 6]:
                if cand.startswith("```") or not cand.strip():
                    continue
                return cand.strip()
    return None


# ===========================================================================
# Suite 1 — the fake node is worth trusting
# ===========================================================================
def suite_fake(opts):
    sect("fake node fidelity (ilxctl + piagent per PROTOCOL.md)")
    n = FakeNode("cam3", "127.0.0.9", cam_num=3, has_imu=True)
    try:
        st = rigcore.http_json("http://127.0.0.9:8080/api/status", timeout=4)
        check("status carries the human label keys",
              st.get("iso") == "ISO 400" and st.get("shutter") == "1/200"
              and st.get("aperture") == "F8.0",
              "%s / %s / %s" % (st.get("iso"), st.get("shutter"),
                                st.get("aperture")))
        check("status carries the raw Sony keys alongside",
              st.get("isoValue") == 400 and st.get("shutterValue") == 65736
              and st.get("apertureValue") == 800,
              "isoValue=%s shutterValue=%s apertureValue=%s"
              % (st.get("isoValue"), st.get("shutterValue"),
                 st.get("apertureValue")))
        check("label and raw are different types (the convergence trap)",
              isinstance(st.get("iso"), str) and isinstance(st.get("isoValue"), int))
        h = rigcore.http_json("http://127.0.0.9:8081/health", timeout=4)
        check("piagent health has gpio/imu/disk/time",
              all(k in h for k in ("gpio", "imu", "disk_free_mb", "time")),
              "chip=%s" % (h.get("gpio") or {}).get("chip"))

        t = time.time() - 3.0
        name = n.add_frame(epoch=t)
        shots = rigcore.http_json("http://127.0.0.9:8080/api/shots", timeout=4)
        check("/api/shots lists the frame with a size",
              isinstance(shots, list) and shots[-1]["name"] == name
              and shots[-1]["size"] > 0, name)
        data, err = rigcore.http_bytes("http://127.0.0.9:8080/shot/%s" % name,
                                       timeout=6)
        check("/shot/<name> returns the exact bytes",
              data is not None and len(data) == shots[-1]["size"]
              and data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9",
              err or "%d bytes" % (len(data) if data else 0))
        cap = runmod._exif_capture_epoch(data)
        check("frame EXIF decodes to the capture instant",
              cap is not None and abs(cap - t) < 0.02,
              "%.2f vs %.2f" % (cap or -1, t))

        # fault symptoms
        n.set_fault("ilx", http500=True)
        r = rigcore.http_json("http://127.0.0.9:8080/api/status", timeout=4)
        check("HTTP 500 arrives as an error body, not as unreachable",
              r.get("ok") is False and not r.get("_unreachable"), str(r)[:60])
        n.clear_faults()
        n.set_fault("ilx", badjson=True)
        r = rigcore.http_json("http://127.0.0.9:8080/api/status", timeout=4)
        check("malformed JSON is surfaced as a failure", r.get("ok") is False)
        n.clear_faults()
        n.set_fault("ilx", hang_s=2.0)
        t0 = time.time()
        r = rigcore.http_json("http://127.0.0.9:8080/api/status", timeout=1)
        dt = time.time() - t0
        check("a hung node trips the client timeout, bounded",
              r.get("_unreachable") and dt < 1.8, "%.2f s" % dt)
        n.clear_faults()
        n.down()
        r = rigcore.http_json("http://127.0.0.9:8080/api/status", timeout=2)
        check("a dropped node is refused, not hung", r.get("_unreachable"))
        n.up()
        check("node comes back after the drop",
              rigcore.http_json("http://127.0.0.9:8080/api/status",
                                timeout=4).get("connected") is True)
        n.reboot(down_s=0.4)
        st = rigcore.http_json("http://127.0.0.9:8080/api/status", timeout=4)
        check("a PoE power-cycle reboots the node into factory settings",
              st.get("isoValue") == FACTORY["iso"]
              and st.get("storeDest") == FACTORY["store_dest"],
              "isoValue=%s storeDest=%s" % (st.get("isoValue"),
                                            st.get("storeDest")))
        check("reboot clears the PC-save dir and the edge ring",
              rigcore.http_json("http://127.0.0.9:8080/api/shots",
                                timeout=4) == []
              and rigcore.http_json(
                  "http://127.0.0.9:8081/gpio/exposure/events?since=0",
                  timeout=4)["events"] == [])
        check("netguard blocked nothing so far (all traffic on loopback)",
              not BLOCKED, ", ".join(BLOCKED[:2]))
    finally:
        n.close()


# ===========================================================================
# Suite 2 — NodeMonitor state machine
# ===========================================================================
def suite_monitor(opts):
    sect("NodeMonitor: OFFLINE -> REACHABLE -> CAM_CONNECTED")
    env = Env([("cam3", "127.0.0.3", 3, {"has_imu": True})], poll=0.25)
    m, node = env.mon("cam3"), env.node("cam3")
    try:
        check("starts OFFLINE before any poll", m.state == m.OFFLINE)
        env.tick()
        check("a healthy node with a claimed camera goes CAM_CONNECTED",
              m.state == m.CONNECTED, m.state)
        check("status cached with the raw keys convergence needs",
              m.snapshot()["status"].get("isoValue") == 400)
        check("last_seen advanced", m.snapshot()["age_s"] is not None
              and m.snapshot()["age_s"] < 2)

        # --- camera drops off USB but the Pi is fine ----------------------
        s0 = env.seq()
        node.connected = False
        node.clear_counts()
        env.tick()
        check("camera unclaimed -> REACHABLE", m.state == m.REACHABLE, m.state)
        check("REACHABLE triggers exactly one /api/connect",
              node.count("POST /api/connect") == 1,
              "%d attempts" % node.count("POST /api/connect"))
        env.tick()
        check("a successful connect returns the node to CAM_CONNECTED",
              m.state == m.CONNECTED, m.state)
        kinds = [e["msg"] for e in env.evs(s0, kind="node_transition")]
        check("both transitions were journalled",
              kinds == ["CAM_CONNECTED -> REACHABLE",
                        "REACHABLE -> CAM_CONNECTED"], str(kinds))

        # --- node vanishes -------------------------------------------------
        s0 = env.seq()
        node.down()
        env.tick()
        check("an unreachable node goes OFFLINE", m.state == m.OFFLINE, m.state)
        env.tick(3)
        check("OFFLINE is announced once, not once per poll",
              len(env.evs(s0, kind="node_transition")) == 1,
              "%d transition events" % len(env.evs(s0, kind="node_transition")))
        check("an OFFLINE node reports is_connected() False",
              not m.is_connected())
        node.up()
        env.tick()
        check("node returns to CAM_CONNECTED when it comes back",
              m.state == m.CONNECTED, m.state)

        # --- connect keeps failing: backoff --------------------------------
        node.connected = False
        node.set_fault("ilx", fail_set=None)
        node.clear_counts()
        m._backoff = 5.0
        m._connect_after = 0.0
        node.set_fault("ilx", http500=True)          # /api/connect answers 500
        env.tick()
        b1 = m._backoff
        env.tick(8)                                   # well inside the gate
        check("failed connect backs off instead of retrying every poll",
              node.count("POST /api/connect") == 1,
              "%d attempts in 9 polls" % node.count("POST /api/connect"))
        check("first backoff is 5 s -> 8 s", abs(b1 - 8.0) < 0.01, str(b1))
        seq = [b1]
        for _ in range(9):
            m._connect_after = 0.0                    # pretend the gate expired
            env.tick()
            seq.append(m._backoff)
        check("backoff grows geometrically and caps at 60 s",
              seq == sorted(seq) and abs(seq[-1] - 60.0) < 0.01,
              "->".join("%.0f" % x for x in seq))
        node.clear_faults()
        m._connect_after = 0.0
        env.tick(2)
        check("recovery resets the backoff to 5 s",
              m.state == m.CONNECTED and m._backoff == 5.0,
              "%s backoff=%s" % (m.state, m._backoff))

        # --- ilxctl answers 500 while up -----------------------------------
        node.set_fault("ilx", http500=True)
        env.tick()
        check("an ilxctl serving 500s is REACHABLE, not OFFLINE",
              m.state == m.REACHABLE, m.state)
        node.clear_faults()
        env.tick()

        # --- malformed JSON -------------------------------------------------
        node.set_fault("ilx", badjson=True)
        env.tick()
        check("malformed status JSON does not raise out of the monitor",
              m.state in (m.OFFLINE, m.REACHABLE, m.ILX_DOWN), m.state)
        node.clear_faults()
        env.tick()

        # --- ilxctl wedged while piagent is fine -> ILX_DOWN, no connects ---
        # cam2 2026-08-23: ilxctl stuck inside an SDK call after a live-view
        # storm. The old classification (REACHABLE, "camera not claimed")
        # POSTed /api/connect every backoff, stranding one ilxctl HTTP worker
        # per attempt, and the stale cached status kept reporting
        # connected:true so the UI kept asking for live view.
        env.tick()
        node.clear_counts()
        node.set_fault("ilx", hang_s=8.0)
        env.tick(3)
        check("ilxctl hung + piagent up -> ILX_DOWN, not REACHABLE",
              m.state == m.ILX_DOWN, m.state)
        check("no /api/connect is POSTed at a wedged ilxctl",
              node.count("POST /api/connect") == 0, str(node.count("POST /api/connect")))
        snap = m.snapshot()
        check("the cached status is marked stale and NOT connected",
              snap["status"].get("connected") is False
              and snap["status"].get("ilx_down") is True
              and not m.is_connected(), json.dumps(snap["status"])[:120])
        node.clear_faults()
        env.tick(2)
        check("ilxctl answering again -> CAM_CONNECTED without a restart",
              m.state == m.CONNECTED, m.state)
        check("monitor recovers after malformed JSON", m.state == m.CONNECTED)

        # --- slow / hung node ----------------------------------------------
        if not opts.quick:
            node.set_fault("both", hang_s=9.0)
            t0 = time.time()
            env.tick()
            dt = time.time() - t0
            check("a hung node is bounded by the client timeouts (4 s + 6 s)",
                  dt < 13.0, "tick took %.1f s" % dt)
            check("a hung node ends OFFLINE, not wedged", m.state == m.OFFLINE,
                  m.state)
            node.clear_faults()
            env.tick()
            check("monitor recovers after the hang clears",
                  m.state == m.CONNECTED, m.state)

        # --- PoE power cycle: camera AND Pi reboot mid-run ------------------
        s0 = env.seq()
        node.reboot(down_s=1.0, block=False)
        seen = set()
        t_end = time.time() + 6.0
        while time.time() < t_end:
            env.tick()
            seen.add(m.state)
            time.sleep(0.1)
        check("a PoE reboot walks OFFLINE and back to CAM_CONNECTED",
              m.OFFLINE in seen and m.CONNECTED in seen, str(sorted(seen)))
        check("the reboot is visible in the journal",
              len(env.evs(s0, kind="node_transition")) >= 2)
        check("the rebooted body came back with factory settings to reconcile",
              node.raw("iso") == FACTORY["iso"], str(node.raw("iso")))

        # --- threaded: no wedge, no busy loop ------------------------------
        m2 = rigcore.NodeMonitor({"name": "cam3t", "cam_num": 3,
                                  "host": "127.0.0.3"}, env.events, poll=0.25)
        node.clear_counts()
        cpu0 = time.process_time()
        m2.start()
        time.sleep(2.5)
        cpu = time.process_time() - cpu0
        polls = node.count("GET /api/status")
        m2.stop()
        time.sleep(0.6)
        check("a running monitor polls at its period, not flat out",
              4 <= polls <= 16, "%d status polls in 2.5 s" % polls)
        check("polling costs almost no CPU", cpu < 1.0, "%.2f s CPU" % cpu)
        check("stop() unwedges the monitor thread promptly", not safe_alive(m2))
        joined = True
        try:
            m2.join(timeout=2)
        except TypeError:
            joined = False
        contract("a stopped monitor can be joined / probed with is_alive()",
                 joined,
                 "rigcore.py:199 (and the same at run.py:72)",
                 "NodeMonitor subclasses threading.Thread and then assigns "
                 "self._stop = threading.Event(), which shadows Thread._stop(), "
                 "the private method _wait_for_tstate_lock() calls once the "
                 "thread finishes. From then on BOTH monitor.join() and "
                 "monitor.is_alive() raise TypeError: 'Event' object is not "
                 "callable. Nothing in rigd joins them today, so it is a "
                 "landmine rather than a live fault — but the obvious fix for "
                 "the mid-run worker gap (a watchdog that checks is_alive() and "
                 "restarts a dead PullWorker) detonates on it immediately.",
                 "join() raised TypeError: 'Event' object is not callable")

        # --- /api/connect that lies -----------------------------------------
        node.connected = False
        node.set_fault("ilx", connect_lies=True)
        node.clear_counts()
        m3 = rigcore.NodeMonitor({"name": "cam3l", "cam_num": 3,
                                  "host": "127.0.0.3"}, env.events, poll=0.25)
        m3.start()
        time.sleep(4.0)
        m3.stop()
        safe_join(m3, 2)
        tries = node.count("POST /api/connect")
        contract(
            "a camera that never claims is retried with backoff, not every poll",
            tries <= 2,
            "rigcore.py:264-273",
            "ilxctl answers /api/connect {ok:true} but PC-remote priority was "
            "never granted, so /api/status stays connected:false. _connect_after "
            "is reset to 0 on the truthy reply, so the monitor re-POSTs "
            "/api/connect every 2 s forever, each one logging a 'reconnect' "
            "event -> USB storm on a camera another operator may be using, and "
            "unbounded growth of ~/rig/rigd.jsonl.",
            "%d connect attempts in 4 s (backoff 5->60 s allows <=2)" % tries)
        node.clear_faults()
        check("no unhandled exception escaped any monitor thread",
              not UNHANDLED, "; ".join(UNHANDLED[:3]))
    finally:
        env.close()


# ===========================================================================
# Suite 3 — SettingsManager convergence
# ===========================================================================
def suite_settings(opts):
    sect("SettingsManager: one desired vector, every camera identical")
    env = Env([("cam2", "127.0.0.2", 2), ("cam3", "127.0.0.3", 3)], poll=0.25)
    a, b = env.node("cam2"), env.node("cam3")
    ma = env.mon("cam2")
    S = env.settings
    try:
        env.tick()
        want = S.get()
        check("desired vector defaults to the survey setup",
              want["iso"] == 400 and want["shutter"] == 65736
              and want["aperture"] == 800 and want["store_dest"] == 3)

        # --- knocked out of sync -------------------------------------------
        # Exposure is per-camera between explicit applies (the operator may
        # deliberately balance the two bodies), so the idle reconcile corrects
        # NON-exposure fields only and reports an exposure split as
        # information, never a fault. First consume the one forced exposure
        # pass every monitor arms on its OFFLINE->CAM_CONNECTED transition.
        S.reconcile_all(force=False)
        env.tick()
        a.drift(store_dest=2)
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        check("a hand-nudged NON-exposure field is pulled back to desired",
              a.raw("store_dest") == want["store_dest"],
              "store=%s" % a.raw("store_dest"))
        # NOTE a store_dest/drive revert doubles as the in-pass reboot tell and
        # deliberately re-forces exposure; a genuine per-camera exposure change
        # touches exposure fields alone, which is what must survive.
        a.drift(iso=3200, shutter=(1 << 16) | 60, aperture=400)
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        check("a per-camera exposure change SURVIVES the idle reconcile",
              a.raw("iso") == 3200 and a.raw("aperture") == 400,
              "iso=%s aperture=%s (must stay the operator's values)"
              % (a.raw("iso"), a.raw("aperture")))
        S.reconcile_all(force=False)
        env.tick()
        check("the split is reported as exposure_split, not as divergence",
              ma.convergence.get("synced") is True
              and set(ma.convergence.get("exposure_split") or [])
              >= {"iso", "aperture"},
              str(ma.convergence))
        # An explicit apply is the force path: the fleet converges again.
        S.reconcile_all(force=True)
        env.tick()
        check("an explicit apply re-converges the split camera",
              all(a.raw(k) == want[k] for k in
                  ("iso", "shutter", "aperture", "store_dest")),
              "iso=%s shutter=%s aperture=%s store=%s"
              % (a.raw("iso"), a.raw("shutter"), a.raw("aperture"),
                 a.raw("store_dest")))

        # --- a camera that rebooted to defaults ------------------------------
        b.reboot(down_s=0.3)
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        check("a camera that rebooted to factory is reconverged",
              all(b.raw(k) == want[k] for k in CONVERGE_KEYS),
              json.dumps({k: b.raw(k) for k in CONVERGE_KEYS}))
        contract("every camera ends EXACTLY synced to every other",
                 all(a.raw(k) == b.raw(k) for k in CONVERGE_KEYS),
                 "PROTOCOL.md:100-121",
                 "two bodies, one rebooted to factory: after reconcile the raw "
                 "settings vectors must be byte-identical across cameras",
                 json.dumps({k: [a.raw(k), b.raw(k)] for k in CONVERGE_KEYS
                             if a.raw(k) != b.raw(k)}))

        # --- label vs raw ---------------------------------------------------
        a.label_override = {"iso": "ISO 12800", "shutter": "1/8000",
                            "aperture": "F22"}
        env.tick()
        a.clear_counts()
        S.reconcile_all(force=False)
        check("a lying human label does not trigger a re-push (raw is truth)",
              not a.pushed("iso", "shutter", "aperture", "drive"),
              "re-pushed %s although every raw value was already correct"
              % [p[1] for p in a.pushed("iso", "shutter", "aperture", "drive")])
        a.label_override = {}
        a.settings["iso"] = 100                    # raw wrong, label would say ok
        a.label_override = {"iso": "ISO 400"}
        env.tick()
        a.clear_counts()
        S.reconcile_all(force=True)     # exposure writes ride the force path
        env.tick()
        check("a correct-looking label with a wrong raw value IS corrected",
              a.raw("iso") == 400, "iso=%s" % a.raw("iso"))
        a.label_override = {}

        # --- no thrash when everything is already in sync -------------------
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        a.clear_counts()
        b.clear_counts()
        for _ in range(3):
            S.reconcile_all(force=False)
            env.tick()
        pushes = a.count("POST /api/exposure") + a.count("POST /api/store") \
            + a.count("POST /api/focus/mode")
        contract("a converged fleet is left alone (no writes when in sync)",
                 pushes == 0,
                 "rigcore.py:409-435",
                 "filetype/imagesize/transsize map to a None readback key, so "
                 "the `if key and have == target` skip can never fire for them, "
                 "and expcomp is pushed unconditionally at rigcore.py:428. A "
                 "fully converged fleet therefore takes 4 SetDeviceProperty "
                 "writes per camera every 3 s forever — permanent USB traffic "
                 "into a body that is mid-survey.",
                 "%d writes in 3 idle reconciles (expected 0): %s"
                 % (pushes, sorted({p[1] for p in a.pushed()})))

        # --- reconcile must confirm from the node, not from a stale cache ---
        # drive is a readable NON-exposure field, so the idle pass corrects it;
        # the contract is that the correction is confirmed by re-reading the
        # NODE, not the 2 s status cache the diff was computed from.
        a.drift(drive=2)
        env.tick()                                   # cache now holds drive=2
        S.reconcile_all(force=False)                 # pushes 1, node applies it
        contract("reconcile confirms by re-reading the node, not the cache",
                 ma.convergence.get("synced") is True,
                 "rigcore.py:437-451",
                 "after pushing, _reconcile_node sleeps 150 ms then re-reads "
                 "m.snapshot()['status'] — the SAME cached dict the diff was "
                 "computed from, refreshed only by the 2 s monitor poll. The "
                 "push has already been applied by the camera, yet the node is "
                 "marked diverged and a settings_divergent warning is emitted. "
                 "Every user settings change therefore raises a false alert and "
                 "flashes the UI badge to 'divergent'.",
                 "convergence=%s while the node already reads %s"
                 % (ma.convergence, a.raw("iso")))
        env.tick()
        S.reconcile_all(force=False)
        check("convergence does settle once the poll refreshes the cache",
              ma.convergence.get("synced") is True, str(ma.convergence))

        # --- a field that will not take -------------------------------------
        # Exposure is only written on the force path now, so the refused-push
        # alarm is exercised through an explicit apply.
        s0 = env.seq()
        a.drift(iso=6400)
        a.set_fault("ilx", fail_set=["iso"])
        env.tick()
        a.clear_counts()
        S.reconcile_all(force=True)
        env.tick()
        S.reconcile_all(force=True)
        divergent = env.evs(s0, kind="settings_divergent", node="cam2")
        check("a field that will not take raises settings_divergent for the node",
              bool(divergent) and "iso" in str(divergent[-1]),
              divergent[-1]["msg"] if divergent else "no event")
        check("the divergent field is exposed on the node for the UI",
              "iso" in (ma.convergence.get("diverged") or []),
              str(ma.convergence))
        check("a rejected field is retried once per reconcile, not spun on",
              len(a.pushed("iso")) <= 2,
              "%d iso writes over 2 reconciles" % len(a.pushed("iso")))
        a.clear_faults()
        env.tick()
        S.reconcile_all(force=True)
        env.tick()
        check("the field converges once the camera accepts it again",
              a.raw("iso") == 400, str(a.raw("iso")))

        # --- offline node ----------------------------------------------------
        b.down()
        env.tick()
        b.clear_counts()
        s0 = env.seq()
        before = b.raw_all()
        S.reconcile_all(force=False)
        S.reconcile_all(force=True)
        check("an offline node is skipped entirely (no calls, no exception)",
              b.count("ilx") == 0, "%d calls while down" % b.count("ilx"))
        check("an offline node is not flagged divergent",
              not env.evs(s0, kind="settings_divergent", node="cam3"))
        check("the online camera still converges while its neighbour is down",
              a.raw("iso") == 400 and ma.is_connected())
        b.up()
        env.tick(2)
        check("no settings were fabricated on the node while it was away",
              b.raw_all() == before)

        # --- desired persistence + user intent -------------------------------
        S.update({"iso": 800, "aperture": 1100})
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        check("a user change lands on every connected camera",
              a.raw("iso") == 800 and b.raw("iso") == 800
              and a.raw("aperture") == 1100 and b.raw("aperture") == 1100,
              "cam2 iso=%s cam3 iso=%s" % (a.raw("iso"), b.raw("iso")))
        check("desired persists to disk for a rigd restart",
              os.path.exists(rigcore.DESIRED_PATH))
        S2 = rigcore.SettingsManager(env.monitors, env.events)
        check("a fresh SettingsManager reloads the persisted vector",
              S2.get()["iso"] == 800 and S2.get()["aperture"] == 1100,
              json.dumps({k: S2.get()[k] for k in ("iso", "aperture")}))
        S.update({"bogus_field": 1})
        check("unknown keys are refused into desired",
              "bogus_field" not in S.get())

        # --- white balance: fleet-converged, and build-gated ----------------
        # Rig policy is fixed color temperature (mode 256) at 5600 K; the fake
        # boots in AWB, so convergence must have moved BOTH fields by now.
        check("white balance converges to fixed 5600 K on every camera",
              a.raw("wb_mode") == 256 and a.raw("colortemp") == 5600
              and b.raw("wb_mode") == 256 and b.raw("colortemp") == 5600,
              "cam2 wb=%s/%s cam3 wb=%s/%s" % (a.raw("wb_mode"),
              a.raw("colortemp"), b.raw("wb_mode"), b.raw("colortemp")))
        # An ilxctl build that predates the field reports NO whiteBalance /
        # colorTemp keys at all. The reconcile loop must skip it quietly —
        # no pushes, no divergence — instead of alarming on a node that is
        # simply not upgraded yet (the live fleet's exact state 2026-08-21).
        with b._lock:
            b.hide_keys = {"whiteBalance", "colorTemp"}
            b.settings["wb_mode"] = 0            # body actually in AWB
        b.clear_counts()
        S.reconcile_all(force=False)
        env.tick()
        S.reconcile_all(force=False)
        env.tick()
        conv = env.mon("cam3").snapshot()["convergence"]
        check("a pre-WB ilxctl build is skipped quietly, not diverged",
              conv.get("synced") is True
              and "wb_mode" not in (conv.get("diverged") or [])
              and "colortemp" not in (conv.get("diverged") or []),
              json.dumps(conv))
        check("no wb push was sent to the pre-WB build",
              not [p for p in b.pushes if p[1] in ("wb_mode", "colortemp")],
              str(list(b.pushes)[-3:]))
        with b._lock:
            b.hide_keys = set()
        cur = S.bump_ev(1)
        check("EV bump moves expcomp by 1/3 stop", cur == 333, str(cur))
        S.bump_ev(-1)
        check("no unhandled exception escaped reconcile",
              not UNHANDLED, "; ".join(UNHANDLED[:3]))
    finally:
        env.close()


# ===========================================================================
# Suite 4 — RunManager end to end
# ===========================================================================
def suite_runmgr(opts):
    sect("RunManager: layout, naming, flight_log contract")
    # Pure formatting first — no node needed.
    base = 1786000000.0
    edge = base + 0.999
    fn_e, dt_e = runmod._fmt_fname(3, edge), runmod._fmt_dt(edge)
    want_s = time.strftime("%H%M%S", time.gmtime(round(edge)))
    # Cam{N}_YYYYMMDD_hhmmss.ss.jpg -> "Cam3_" is 5 chars, the date 8 more, then
    # the separator at index 13, so hhmmss lives at [14:20]. The slice used to
    # read [13:19], i.e. "_07064", which could never equal a %H%M%S string: the
    # assertion failed for every input and reported run.py's (correct) rounding
    # as a defect. Derive the offset instead of hand-counting it.
    hh_at = len("Cam3_20260806_")
    contract("a frame in the last 5 ms of a second is not stamped a second early",
             fn_e[hh_at:hh_at + 6] == want_s,
             "run.py:54-64",
             "_split_epoch must round epoch*100 to centiseconds ONCE and derive "
             "both the seconds and the fraction from that result. Rounding the "
             "fraction independently of the seconds stamps any capture in the "
             "last 5 ms of a second a full second early — about 0.5%% of every "
             "survey's frames, silently, in both the filename and the "
             "flight_log datetime column that nav is correlated against.",
             "%.3f -> %s / %s (expected the %s second)"
             % (edge, fn_e, dt_e, want_s))
    check("filename and datetime describe the same instant",
          runmod._fmt_fname(3, base + .25)[5:].replace(".jpg", "")[2:]
          == runmod._fmt_dt(base + .25))

    nav = FakeNav(fix=False)
    env = Env([("cam2", "127.0.0.2", 2),
               ("cam3", "127.0.0.3", 3, {"has_imu": True, "clock_skew_s": 45.0})],
              poll=0.4, threaded=True, nav=nav)
    a, b = env.node("cam2"), env.node("cam3")
    try:
        check("both fakes reach CAM_CONNECTED",
              env.wait_state("cam2", "CAM_CONNECTED")
              and env.wait_state("cam3", "CAM_CONNECTED"))
        # ---- against the nav reader rig/nav.py actually ships --------------
        # run.py and nav.py have to agree on the interface (set_raw_hook /
        # fix_at); an unopened NavReader has no fix, which is also the "never
        # fabricate" case for the CSV.
        fd0 = fd_count()
        ok, why, navrows = False, "", []
        try:
            env.runmgr.nav = shipping_nav_reader()
            r0 = env.runmgr.start({"label": "navshape"})
            ok = bool(r0.get("ok"))
            a.add_frame(epoch=time.time(), name="ILXNAVAPI.JPG")
            fl0 = os.path.join(r0.get("root", ""), "cam2", "flight_log.csv")
            wait_for(lambda: os.path.exists(fl0)
                     and len(read_flight(fl0)[1]) > 0, 8)
            navrows = read_flight(fl0)[1] if os.path.exists(fl0) else []
            env.runmgr.stop()
            check("nmea_raw.log is opened at the path PROTOCOL.md specifies",
                  os.path.exists(os.path.join(r0.get("root", ""),
                                              "nmea_raw.log")))
        except Exception as e:                                # noqa: BLE001
            why = ("%s: %s | run left inactive=%s, event log still pointed at "
                   "the dead run dir=%s, +%d leaked fds"
                   % (type(e).__name__, e, env.runmgr.active is None,
                      env.events._run_fh is not None, fd_count() - fd0))
            if env.events._run_fh is not None:                # tidy up after it
                try:
                    env.events._run_fh.close()
                except OSError:
                    pass
                env.events.set_run_file(None)
        env.runmgr.nav = nav
        contract("a run drives the nav reader rig/nav.py actually ships",
                 ok,
                 "run.py:265-268 and run.py:313 vs nav.NavReader",
                 "RunManager.start tees NMEA into the run by calling "
                 "self.nav.set_raw_hook(...), and every frame calls "
                 "self.nav.fix_at(epoch). If nav.py's reader does not expose "
                 "exactly those, start() raises AttributeError out of "
                 "/api/run/start — after the run dir, events.log and "
                 "nmea_raw.log have been created and the event log redirected "
                 "into them — so rigd answers 500, no run can be started at "
                 "all, and each attempt leaks two file handles.",
                 why)
        if navrows:
            idx0 = {k: i for i, k in enumerate(read_flight(fl0)[0])}
            blanks = ("lat", "long", "xutm", "yutm", "utm_zone",
                      "depth_from_xplore9", "heading_mag_xplore")
            check("a nav reader with no fix writes empty nav columns, never a "
                  "fabricated position",
                  all(navrows[-1][idx0[c]] == "" for c in blanks),
                  json.dumps({c: navrows[-1][idx0[c]] for c in blanks
                              if navrows[-1][idx0[c]] != ""}))

        r = env.runmgr.start({"label": "transect-01"})
        check("run starts", r.get("ok") is True, json.dumps(r)[:120])
        root = r.get("root", "")
        check("run_id is YYMMDD_hhmm_label",
              re.match(r"^\d{6}_\d{4}_transect-01$", r.get("run_id", "")),
              r.get("run_id"))
        check("run root, events.log and nmea_raw.log exist",
              os.path.isdir(root)
              and os.path.exists(os.path.join(root, "events.log"))
              and os.path.exists(os.path.join(root, "nmea_raw.log")))
        cam_dirs = [os.path.join(root, n) for n in ("cam2", "cam3")]
        check("a per-camera folder is created for every live node",
              wait_for(lambda: all(os.path.isdir(d) for d in cam_dirs), 4))
        check("run.json is written at start",
              os.path.exists(os.path.join(root, "run.json")))

        # ---- phase A: a frame with no nav fix and no IMU sample -----------
        t_a = time.time()
        na = a.add_frame(epoch=t_a)
        fl_a = os.path.join(root, "cam2", "flight_log.csv")
        got = wait_for(lambda: os.path.exists(fl_a) and len(read_flight(fl_a)[1]),
                       8)
        check("the frame is pulled and logged", bool(got))
        hdr, rows = read_flight(fl_a)
        proto = protocol_flight_header()
        contract("flight_log header matches PROTOCOL.md byte for byte",
                 proto is not None and ",".join(hdr) == proto,
                 "run.py:27-32 vs PROTOCOL.md:203",
                 "the 23-column contract is what every downstream consumer "
                 "parses; any drift silently corrupts the survey dataset",
                 "csv=%r" % ",".join(hdr))
        check("header is exactly 23 columns", len(hdr) == 23, str(len(hdr)))
        check("every row has 23 fields",
              all(len(r_) == 23 for r_ in rows),
              str(sorted({len(r_) for r_ in rows})))
        rowa = rows[-1]
        m_ = FNAME_RE.match(rowa[0])
        check("filename matches Cam#_YYYYMMDD_hhmmss.ss.jpg exactly",
              bool(m_), rowa[0])
        if m_:
            check("filename carries this node's camera number",
                  m_.group(1) == "2", m_.group(1))
            check("the renamed file exists on disk with the logged name",
                  os.path.exists(os.path.join(root, "cam2", rowa[0])))
        idx = {k: i for i, k in enumerate(hdr)}
        blank = ("lat", "long", "xutm", "yutm", "utm_zone",
                 "depth_from_xplore9", "heading_mag_xplore")
        check("no nav fix -> nav columns are empty, never fabricated",
              all(rowa[idx[c]] == "" for c in blank),
              json.dumps({c: rowa[idx[c]] for c in blank if rowa[idx[c]] != ""}))
        imu_cols = ("pitch", "roll", "yaw", "heading_imu", "ax_g", "ay_g",
                    "az_g", "gx_dps", "gy_dps", "gz_dps", "imu_temp_c")
        check("no IMU sample -> IMU columns are empty, never fabricated",
              all(rowa[idx[c]] == "" for c in imu_cols),
              json.dumps({c: rowa[idx[c]] for c in imu_cols
                          if rowa[idx[c]] != ""}))
        check("datetime column is YYMMDD_hhmmss.ss UTC",
              re.match(r"^\d{6}_\d{6}\.\d{2}$", rowa[idx["datetime"]])
              and rowa[idx["datetime"]][:6] ==
              time.strftime("%y%m%d", time.gmtime(t_a)),
              rowa[idx["datetime"]])
        check("time_source is recorded", rowa[idx["time_source"]] in
              ("gps", "jetson"), rowa[idx["time_source"]])
        check("capture_source is one of gpio_edge|exif|command",
              rowa[idx["capture_source"]] in ("gpio_edge", "exif", "command"),
              rowa[idx["capture_source"]])

        # ---- EXIF clock calibration ---------------------------------------
        off = wait_for(lambda: env.timesync.exif_offset.get("cam3"), 12)
        check("per-node EXIF clock offset is measured at run start",
              off is not None and abs(off - 45.0) < 1.0,
              "measured %.2f s against a staged +45 s camera clock"
              % (off if off else -1))

        # ---- phase B: nav fix + IMU ---------------------------------------
        nav.fix = True
        fl_b = os.path.join(root, "cam3", "flight_log.csv")
        wait_for(lambda: os.path.exists(fl_b), 6)
        n_b0 = len(read_flight(fl_b)[1]) if os.path.exists(fl_b) else 0
        t_b = time.time()
        for k in range(7):
            b.push_imu(epoch=t_b - 0.6 + 0.2 * k)
        b.add_frame(epoch=t_b, name="ILXNAV01.JPG")
        wait_for(lambda: os.path.exists(fl_b)
                 and len(read_flight(fl_b)[1]) > n_b0, 10)
        rows_b = read_flight(fl_b)[1]
        check("the nav/IMU frame produced a new row", len(rows_b) > n_b0,
              "%d -> %d rows" % (n_b0, len(rows_b)))
        rowb = rows_b[-1]
        check("a fix populates lat/long and the UTM conversion",
              rowb[idx["lat"]] and rowb[idx["long"]] and rowb[idx["xutm"]]
              and rowb[idx["utm_zone"]],
              "lat=%s utm=%s %s" % (rowb[idx["lat"]], rowb[idx["xutm"]],
                                    rowb[idx["utm_zone"]]))
        check("depth and magnetic heading come from nav",
              rowb[idx["depth_from_xplore9"]] == "12.34"
              and rowb[idx["heading_mag_xplore"]] == "211.4",
              "%s / %s" % (rowb[idx["depth_from_xplore9"]],
                           rowb[idx["heading_mag_xplore"]]))
        check("the IMU sample nearest the capture instant is stamped on the row",
              rowb[idx["pitch"]] == "1.25" and rowb[idx["az_g"]] == "0.98"
              and rowb[idx["imu_temp_c"]] == "31.5",
              "pitch=%s az=%s" % (rowb[idx["pitch"]], rowb[idx["az_g"]]))

        # ---- duplicate + late frames ---------------------------------------
        # Wait for the log to go quiet before taking the baseline. Frames from
        # the preceding phases, and the EXIF-calibration shot the run fires at
        # start, can still be landing here; sampling the row count across a
        # fixed sleep then credits THOSE arrivals to the duplicate listing and
        # fails a dedupe that is in fact working (every filename unique, rows
        # equal to files on disk).
        def _settled(quiet_s=1.2, timeout=8.0):
            deadline = time.time() + timeout
            last = len(read_flight(fl_a)[1])
            stable_since = time.time()
            while time.time() < deadline:
                time.sleep(0.2)
                n = len(read_flight(fl_a)[1])
                if n != last:
                    last, stable_since = n, time.time()
                elif time.time() - stable_since >= quiet_s:
                    break
            return last

        before = _settled()
        with a._lock:                      # the same frame listed twice
            dup = dict(next(s for s in a.shots if s["name"] == na))
            a.shots.append(dup)
        time.sleep(1.5)
        check("a duplicate listing does not write a second row",
              len(read_flight(fl_a)[1]) == before,
              "%d -> %d rows" % (before, len(read_flight(fl_a)[1])))
        t_late = time.time() - 45.0
        a.add_frame(epoch=t_late, name="ILX09999.JPG")
        wait_for(lambda: len(read_flight(fl_a)[1]) > before, 8)
        rows_a = read_flight(fl_a)[1]
        check("a late frame is still logged", len(rows_a) > before)
        names = [r_[0] for r_ in rows_a]
        check("no duplicate filenames in the flight log",
              len(names) == len(set(names)), str(names))
        check("every logged filename exists on disk",
              all(os.path.exists(os.path.join(root, "cam2", n_))
                  for n_ in names))
        on_disk = sorted(f for f in os.listdir(os.path.join(root, "cam2"))
                         if f.endswith(".jpg"))
        check("exactly one flight_log row per image on disk",
              sorted(names) == on_disk,
              "%d rows vs %d files" % (len(names), len(on_disk)))

        # ---- GPIO edge attribution -----------------------------------------
        edges = []
        base = time.time()
        for k in range(3):
            ep = base + 0.10 * k
            edges.append(ep)
            b.push_edge(epoch=ep)
            b.add_frame(epoch=ep, name="ILXG%04d.JPG" % k)
        time.sleep(2.0)
        index = env.runmgr.active["index"] if env.runmgr.active else []
        g = [e for e in index if e["orig"].startswith("ILXG")]
        srcs = [e["src"] for e in g]
        contract("each frame is stamped with ITS OWN GPIO exposure edge",
                 len(g) == 3 and srcs.count("gpio_edge") == 3
                 and sorted(round(e["epoch"], 2) for e in g)
                 == sorted(round(x, 2) for x in edges),
                 "run.py:416-424",
                 "three frames land in one 0.4 s pull cycle with three pending "
                 "EXPOSURE edges. match_exposure_edge() drains the whole cursor "
                 "on the first frame and returns falls[-1], so frame 1 is "
                 "stamped with frame 3's edge and frames 2-3 silently fall back "
                 "to EXIF/command time. Capture timestamps — the entire point of "
                 "the GPIO harness — are wrong whenever more than one frame "
                 "arrives per poll.",
                 "sources=%s epochs=%s vs edges=%s"
                 % (srcs, [round(e["epoch"], 2) for e in g],
                    [round(x, 2) for x in edges]))

        # ---- a node that power-cycles mid-run (shared PoE feed) ------------
        n_before = len(read_flight(fl_a)[1])
        a.reboot(down_s=1.0, keep_shots=True, block=True)
        wait_for(lambda: env.mon("cam2").state == "CAM_CONNECTED", 10)
        a.add_frame(epoch=time.time(), name="ILXPOE01.JPG")
        got = wait_for(lambda: len(read_flight(fl_a)[1]) > n_before, 10)
        check("a node that power-cycles mid-run keeps being logged after it "
              "comes back", bool(got),
              "%d -> %d rows" % (n_before, len(read_flight(fl_a)[1])))
        rows_a = read_flight(fl_a)[1]

        # ---- stop during an in-flight pull ---------------------------------
        a.set_fault("ilx", hang_s=3.0)
        a.add_frame(epoch=time.time(), name="ILXSTOP1.JPG")
        time.sleep(0.8)                     # pull is now blocked in http_bytes
        t0 = time.time()
        res = env.runmgr.stop()
        dt = time.time() - t0
        check("stop() returns while a pull is still in flight",
              res.get("ok") is True and dt < 6.0, "%.1f s" % dt)
        doc = json.load(open(os.path.join(root, "run.json")))
        check("run.json is finalised", doc.get("final") is True)
        check("run.json carries the frame index and per-camera stats",
              doc.get("frames", 0) > 0 and "cam2" in (doc.get("stats") or {}))
        hdr2, rows2 = read_flight(fl_a)
        check("flight_log survives stop-during-pull intact",
              hdr2 == hdr and all(len(r_) == 23 for r_ in rows2)
              and len(rows2) >= len(rows_a))

        # ---- a stopped run's worker must not bleed into the next run -------
        env.runmgr.start({"label": "transect-02"})
        wait_for(lambda: "ILXSTOP1.JPG" in
                 [e["orig"] for e in (env.runmgr.active or {}).get("index", [])],
                 10)
        idx2 = [e["orig"] for e in (env.runmgr.active or {}).get("index", [])]
        contract("a stopped run's in-flight frame does not land in the next run",
                 "ILXSTOP1.JPG" not in idx2,
                 "run.py:298-321",
                 "stop() clears self.workers without joining the PullWorker "
                 "threads, and each worker holds a reference to the RunManager. "
                 "A pull still in flight (up to a 30 s http_bytes timeout) "
                 "finishes afterwards and calls index_frame(), which appends to "
                 "whatever run is active by then — so a frame from transect-01 "
                 "is recorded in transect-02's run.json.",
                 "index of the new run: %s" % idx2)
        a.clear_faults()
        env.runmgr.stop()

        # ---- the LAST fired shot must survive stop -------------------------
        # A real body takes ~0.5-1.5 s of card write + PC-save before a fired
        # frame reaches the spool. stop() used to race that pipeline and drop
        # the FINAL shot of every transect, deterministically (measured live
        # 2026-08-20: 141 fired/140 archived, 337/336). A fire committed
        # before stop belongs to the transect: stop waits, bounded, for it.
        a.save_delay_s = 0.9
        env.runmgr.start({"label": "transect-grace"})
        rootg = env.runmgr.active["root"]
        rg = env.runmgr.capture_once()
        check("grace-test fire succeeded",
              ((rg.get("results") or {}).get("cam2") or {}).get("ok") is True,
              json.dumps(rg)[:140])
        resg = env.runmgr.stop()      # immediately — frame lands ~0.9 s later
        a.save_delay_s = 0
        docg = json.load(open(os.path.join(rootg, "run.json")))
        pulledg = ((docg.get("stats") or {}).get("cam2") or {}).get("pulled", 0)
        check("the last fired shot is archived despite an immediate stop",
              resg.get("ok") is True and pulledg >= 1,
              "pulled=%s" % pulledg)

        # ---- a node that stops answering fires pauses the grid -------------
        # cam1 lost power mid-run (PoE collapse, 2026-08-23): its fires timed
        # out 10 s each, the backlog climbed past 9 shots and cam2 shot an
        # entire "transect" alone. Fires now fail fast, three in a row marks
        # the node dead, the grid pauses (no unpaired frames), and it resumes
        # by itself when the node's health poll is fresh again.
        b.set_fault("pia", hang_s=30.0)           # piagent dead, ilxctl fine
        env.runmgr.start({"label": "transect-pause", "interval_s": 0.4,
                          "auto_capture": True})
        got = wait_for(lambda: bool((env.runmgr.status().get("sync") or {})
                                    .get("paused_for")), 20)
        st = env.runmgr.status().get("sync") or {}
        check("three consecutive failed fires pause the capture grid",
              got and (st.get("paused_for") or {}).get("node") == "cam3",
              json.dumps(st.get("paused_for")))
        check("a paused run fired no more than a handful of unpaired shots",
              0 < st.get("unpaired_shots", 0) <= 4,
              "unpaired=%s" % st.get("unpaired_shots"))
        pulled_a = env.runmgr.status()["stats"]["cam2"]["pulled"]
        time.sleep(1.5)
        check("the healthy camera is NOT fired alone while paused",
              env.runmgr.status()["stats"]["cam2"]["pulled"] == pulled_a,
              "cam2 pulled %d -> %d" % (pulled_a,
                                        env.runmgr.status()["stats"]["cam2"]["pulled"]))
        check("capture_paused was journalled as an error",
              any(e["kind"] == "capture_paused" for e in env.evs()))
        b.clear_faults()
        env.tick()
        resumed = wait_for(lambda: not (env.runmgr.status().get("sync") or {})
                           .get("paused_for")
                           and env.runmgr.status()["stats"]["cam3"]["pulled"] >= 1, 15)
        check("capture resumes by itself when the node answers again",
              resumed, json.dumps((env.runmgr.status().get("sync") or {}).get("paused_for")))
        env.runmgr.stop()

        # ---- trigger latency is reused, not re-fired, within 24 h ---------
        # Every run start used to fire five calibration frames per camera and
        # hold FOCUS (AE-lock) to re-measure a figure that moves <0.5 ms in a
        # week: ~240 junk RAW+JPEG frames in three days. A value measured on
        # the SAME body id within 24 h is adopted instead.
        seq0 = env.seq()
        a.clear_counts(); b.clear_counts()
        env.runmgr.start({"label": "transect-reuse"})
        time.sleep(2.5)
        env.runmgr.stop()
        reused = [e for e in env.evs(since=seq0) if e["kind"] == "calibrate"
                  and "reused" in e["msg"]]
        fires = a.count("POST /gpio/fire") + b.count("POST /gpio/fire")
        check("a fresh same-body latency is reused at the next run start",
              len(reused) >= 1, "%d reuse events" % len(reused))
        check("no trigger-calibration frames are fired when reusing "
              "(only the single EXIF frame per camera remains)",
              fires <= 2, "%d fires at run start" % fires)
        check("the latency is persisted under RIG_HOME, not the live ~/rig",
              os.path.exists(os.path.join(rigcore.RIG_HOME, "trigger_latency.json"))
              and rigcore.RIG_HOME != os.path.expanduser("~/rig"))

        # ---- a node that is down when the run starts ----------------------
        b.down()
        wait_for(lambda: env.mon("cam3").state == "OFFLINE", 8)
        r3 = env.runmgr.start({"label": "transect-03"})
        b.up()
        wait_for(lambda: env.mon("cam3").state == "CAM_CONNECTED", 10)
        b.add_frame(epoch=time.time(), name="ILXLATE1.JPG")
        fl3 = os.path.join(r3["root"], "cam3", "flight_log.csv")
        got = wait_for(lambda: os.path.exists(fl3)
                       and len(read_flight(fl3)[1]) > 0, 8)
        contract("a node that comes up after the run started is still surveyed",
                 bool(got),
                 "run.py:269-280",
                 "run start snapshots the connected nodes once and builds a "
                 "PullWorker per node from that list; nothing revisits it. A "
                 "camera still booting when the transect is started (or one "
                 "power-cycled long enough to miss the start) gets no worker, "
                 "no cam folder and no flight_log for the entire run — every "
                 "frame it takes is left on the node and never appears in the "
                 "survey. run.json's `nodes` also under-reports the fleet.",
                 "cam3 dir=%s rows=%s"
                 % (os.path.isdir(os.path.join(r3["root"], "cam3")),
                    len(read_flight(fl3)[1]) if os.path.exists(fl3) else "n/a"))
        env.runmgr.stop()
        check("no unhandled exception escaped a worker thread",
              not UNHANDLED, "; ".join(UNHANDLED[:3]))
    finally:
        env.close()


# ===========================================================================
# Suite 5 — fault injection on the pull path
# ===========================================================================
def suite_pull(opts):
    sect("pull path under fault: disk full, no permission, truncation, vanish")
    env = Env([("cam2", "127.0.0.2", 2)], poll=0.4, threaded=True,
              nav=FakeNav(fix=True))
    a = env.node("cam2")
    try:
        env.wait_state("cam2", "CAM_CONNECTED")

        # ---- a transfer that dies mid-frame (no run needed) ---------------
        nm = a.add_frame(epoch=time.time(), name="ILXCUT.JPG")
        with a._lock:
            full = next(s["size"] for s in a.shots if s["name"] == nm)
        a.set_fault("ilx", cut=True)
        data, err = rigcore.http_bytes("http://127.0.0.2:8080/shot/%s" % nm,
                                       timeout=8)
        a.set_fault("ilx", cut=False)
        contract("a frame transfer that dies mid-stream is reported as an error",
                 err is not None or (data is not None and len(data) == full),
                 "rigcore.py:170-179",
                 "the node promises Content-Length: %d and then the link drops "
                 "half way. http_bytes calls r.read(cap+1) with an explicit "
                 "size, which returns the short body without raising, so the "
                 "caller cannot tell a completed transfer from a severed one "
                 "and writes the half frame to the survey folder." % full,
                 "got %s bytes of %d with err=%r"
                 % (len(data) if data is not None else None, full, err))

        r = env.runmgr.start({"label": "faults"})
        root = r["root"]
        cam = os.path.join(root, "cam2")
        fl = os.path.join(cam, "flight_log.csv")
        wait_for(lambda: os.path.exists(fl), 6)

        def rows():
            return read_flight(fl)[1]

        def good_frame(tag):
            a.add_frame(epoch=time.time(), name="ILXOK%s.JPG" % tag)
            n0 = len(rows())
            return wait_for(lambda: len(rows()) > n0, 8)

        check("baseline: a healthy frame lands", bool(good_frame("A")))

        # ---- disk full -----------------------------------------------------
        s0 = env.seq()
        n0 = len(rows())
        real_open = open

        def enospc(path, *a_, **k_):
            if isinstance(path, str) and path.startswith(cam) \
                    and path.endswith(".jpg"):
                raise OSError(errno.ENOSPC, "No space left on device", path)
            return real_open(path, *a_, **k_)

        runmod.open = enospc
        a.add_frame(epoch=time.time(), name="ILXFULL.JPG")
        time.sleep(2.0)
        runmod.open = real_open
        ev = env.evs(s0, kind="pull_fail")
        check("disk full during a pull raises a pull_fail event",
              bool(ev) and ev[-1]["sev"] in ("warn", "error"),
              ev[-1]["msg"] if ev else "no event")
        check("disk full writes no flight_log row", len(rows()) == n0,
              "%d -> %d" % (n0, len(rows())))
        check("the worker survives a disk-full frame", bool(good_frame("B")))

        # ---- permission denied ----------------------------------------------
        if os.geteuid() == 0:
            note("running as root — permission-denied injection skipped")
        else:
            s0 = env.seq()
            n0 = len(rows())
            mode = os.stat(cam).st_mode
            os.chmod(cam, stat.S_IRUSR | stat.S_IXUSR)
            a.add_frame(epoch=time.time(), name="ILXPERM.JPG")
            time.sleep(2.0)
            os.chmod(cam, mode)
            ev = env.evs(s0, kind="pull_fail")
            check("permission denied raises a pull_fail event", bool(ev),
                  ev[-1]["msg"] if ev else "no event")
            check("permission denied writes no flight_log row",
                  len(rows()) == n0, "%d -> %d" % (n0, len(rows())))
            check("the worker survives a permission-denied frame",
                  bool(good_frame("C")))

        # ---- node vanishes mid-pull -----------------------------------------
        s0 = env.seq()
        n0 = len(rows())
        a.set_fault("ilx", vanish=True)
        a.add_frame(epoch=time.time(), name="ILXGONE.JPG")
        time.sleep(2.0)
        a.set_fault("ilx", vanish=False)
        ev = env.evs(s0, kind="pull_fail")
        check("a frame that vanishes mid-pull raises pull_fail", bool(ev),
              ev[-1]["msg"] if ev else "no event")
        check("a vanished frame writes no row and leaves no stub file",
              len(rows()) == n0
              and not any(f.startswith("Cam2_") and
                          os.path.getsize(os.path.join(cam, f)) == 0
                          for f in os.listdir(cam)))
        check("the worker survives a vanished frame", bool(good_frame("D")))
        note("a frame whose pull fails is added to `seen` before the download "
             "(run.py:123-125), so it is never retried — a transient link error "
             "loses that frame from the survey permanently")

        # ---- node goes away entirely mid-pull -------------------------------
        s0 = env.seq()
        n0 = len(rows())
        a.add_frame(epoch=time.time(), name="ILXDOWN.JPG")
        a.set_fault("ilx", hang_s=1.5)
        time.sleep(0.6)
        a.down()
        time.sleep(1.5)
        a.clear_faults()
        a.up()
        env.wait_state("cam2", "CAM_CONNECTED", 8)
        check("a node lost mid-pull is journalled, not silent",
              bool(env.evs(s0, kind="pull_fail"))
              or bool(env.evs(s0, kind="node_transition")))
        check("the worker keeps working after the node returns",
              bool(good_frame("E")))

        # ---- truncated JPEG --------------------------------------------------
        s0 = env.seq()
        n0 = len(rows())
        a.set_fault("ilx", truncate=True)
        nt = a.add_frame(epoch=time.time(), name="ILXTRUNC.JPG")
        with a._lock:
            full = next(s["size"] for s in a.shots if s["name"] == nt)
        time.sleep(2.0)
        a.set_fault("ilx", truncate=False)
        new = rows()[n0:]
        onshelf = [f for f in os.listdir(cam) if f.endswith(".jpg")]
        bad = [f for f in onshelf
               if os.path.getsize(os.path.join(cam, f)) == full // 2]
        # A new row here is NOT a failure any more. The pull worker retries a
        # failed transfer with backoff, so once the fault is cleared above the
        # frame is recovered and legitimately logged - which is the whole point
        # of the retry. What must never happen is a TRUNCATED file on disk, or a
        # flight_log row pointing at a frame that is short or missing. Assert
        # that instead of "no row at all", which only passed before retry existed
        # and otherwise fails intermittently depending on whether a retry lands
        # inside the observation window.
        short_rows = [r_[0] for r_ in new
                      if not os.path.exists(os.path.join(cam, r_[0]))
                      or os.path.getsize(os.path.join(cam, r_[0])) != full]
        contract("a truncated frame is rejected, not logged as a good capture",
                 not bad and not short_rows,
                 "run.py:127-157 (the size argument is never used) + rigcore.py:170-179",
                 "ilxctl lists the frame as %d bytes; the transfer delivers "
                 "half of them (a partial PC-save, or a link that dies "
                 "mid-transfer — rigcore.http_bytes uses r.read(cap+1), which "
                 "returns short without raising). _handle() is handed the "
                 "expected `size` and never compares it, and nothing checks for "
                 "the JPEG EOI marker, so the corrupt frame is written to the "
                 "run folder and logged as a normal capture." % full,
                 "%d truncated file(s) on disk, %d new row(s) of which %d point "
                 "at a short or missing frame"
                 % (len(bad), len(new), len(short_rows)))

        # ---- flight_log integrity across every fault above -------------------
        hdr, rr = read_flight(fl)
        check("flight_log header still intact after every fault",
              ",".join(hdr) == protocol_flight_header())
        check("every flight_log row still has 23 fields",
              all(len(x) == 23 for x in rr),
              str(sorted({len(x) for x in rr})))
        names = [x[0] for x in rr]
        check("every filename still matches the rename convention",
              all(FNAME_RE.match(n_) for n_ in names),
              str([n_ for n_ in names if not FNAME_RE.match(n_)][:3]))
        check("no duplicate rows after the fault storm",
              len(names) == len(set(names)))
        check("every fault produced at least one event",
              len(env.evs(0, kind="pull_fail")) >= 3,
              "%d pull_fail events" % len(env.evs(0, kind="pull_fail")))

        # ---- two frames sharing one capture instant -------------------------
        # Pin the clock so the collision is deterministic; in the field the same
        # thing happens whenever two frames share a capture instant.
        n0 = len(rows())
        files0 = set(os.listdir(cam))
        real_now = env.timesync.now
        fixed = time.time()
        env.timesync.now = lambda: (fixed, "jetson")
        a.add_frame(epoch=fixed, name="ILXDUP1.JPG", exif=False)
        a.add_frame(epoch=fixed, name="ILXDUP2.JPG", exif=False)
        wait_for(lambda: len(rows()) >= n0 + 2, 8)
        env.timesync.now = real_now
        new_rows = rows()[n0:]
        new_files = set(os.listdir(cam)) - files0
        contract("two frames at the same capture instant get two distinct files",
                 len({r_[0] for r_ in new_rows}) == len(new_rows)
                 and len(new_files) == len(new_rows),
                 "run.py:59-62 (_fmt_fname) + run.py:139-146 (the unguarded open)",
                 "the destination name is derived only from the capture instant "
                 "at centisecond resolution and written with a bare open(dest, "
                 "'wb'). Two frames that resolve to the same instant — a "
                 "RAW+JPEG pair, two frames attributed to one fire command "
                 "(_pending_cmd is consumed by the first, the rest fall through "
                 "to `now`), or simply two frames pulled in the same 0.4 s cycle "
                 "with no edge and no EXIF — produce the same filename, so the "
                 "second silently overwrites the first image while the "
                 "flight_log gains two rows pointing at the one surviving file. "
                 "A frame is destroyed and the log says otherwise.",
                 "%d rows -> %d distinct names, %d new files: %s"
                 % (len(new_rows), len({r_[0] for r_ in new_rows}),
                    len(new_files), sorted(r_[0] for r_ in new_rows)))

        # ---- RAW sidecar ----------------------------------------------------
        files0 = set(os.listdir(cam))
        t_raw = time.time()
        a.add_frame(epoch=t_raw, name="ILXRAW01.JPG")
        a.add_frame(epoch=t_raw, name="ILXRAW01.ARW", exif=False)
        time.sleep(2.5)
        new_files = sorted(set(os.listdir(cam)) - files0)
        contract("a RAW sidecar keeps the raw extension, not .jpg",
                 any(f.lower().endswith((".arw", ".raw")) for f in new_files),
                 "run.py:59-62",
                 "with filetype RAW+JPEG (Sony 3 — and the factory default a "
                 "rebooted body comes up in) every capture lands two files in "
                 "the PC-save dir. _fmt_fname hard-codes '.jpg', so the .ARW is "
                 "copied into the run folder named CamN_....jpg: a file that "
                 "claims to be a JPEG, holds raw sensor data, is logged as a "
                 "normal frame, and (same stem, same instant) can overwrite the "
                 "real JPEG. PROTOCOL.md:165 requires the sidecar to keep the "
                 "same stem with the raw extension.",
                 "files written: %s" % new_files)

        env.runmgr.stop()
        check("no unhandled exception escaped the pull workers",
              not UNHANDLED, "; ".join(UNHANDLED[:3]))
    finally:
        if hasattr(runmod, "open"):       # undo the ENOSPC injection
            del runmod.open
        env.close()


# ===========================================================================
# Suite 6 — resources: threads, fds, rings, memory
# ===========================================================================
def suite_strobe(opts):
    sect("Strobe: scheduled pulse, per-node routing, acceptance verdicts")
    env = Env([("cam2", "127.0.0.2", 2), ("cam3", "127.0.0.3", 3)],
              poll=0.3, threaded=True)
    a, b = env.node("cam2"), env.node("cam3")
    R = env.runmgr
    try:
        env.wait_state("cam2", "CAM_CONNECTED")
        env.wait_state("cam3", "CAM_CONNECTED")

        # ---- config validation ---------------------------------------------
        check("strobe is OFF by default",
              R.get_strobe().get("enabled") is False, str(R.get_strobe()))
        r = R.set_strobe({"node": "cam9"})
        check("an unknown strobe node is refused", r.get("ok") is False,
              str(r.get("error")))
        r = R.set_strobe({"delta_ms": 300})
        check("a delta beyond 100 ms is refused", r.get("ok") is False,
              str(r.get("error")))
        r = R.set_strobe({"enabled": True, "node": "cam2", "delta_ms": 10})
        check("enabling at the survey shutter (1/200) warns to run 1/30 "
              "or slower",
              r.get("ok") is True and any("1/30" in w
                                          for w in r.get("warnings", [])),
              str(r.get("warnings")))

        # ---- the pulse rides ONLY the strobe node's fire --------------------
        res = R.capture_once()
        results = res.get("results") or {}
        check("both cameras fire with the strobe armed",
              all((results.get(n) or {}).get("ok") for n in ("cam2", "cam3")),
              json.dumps(results)[:200])
        check("the strobe pulses on the strobe node only",
              a.strobe_fires == 1 and b.strobe_fires == 0,
              "cam2=%d cam3=%d" % (a.strobe_fires, b.strobe_fires))
        R.set_strobe({"enabled": False})
        R.capture_once()
        check("a disabled strobe never pulses",
              a.strobe_fires == 1, str(a.strobe_fires))

        # ---- acceptance: inside the window at 1/30, a miss at 1/200 ---------
        # The verdict is docs/strobe-trigger.md §4.2: the strobe instant must
        # sit inside the intersection of every camera's measured [fall, rise]
        # window. 1/30 (the spec's recommended minimum) opens 33 ms — a 10 ms
        # delta lands inside; 1/200 opens 5 ms — the same delta lands after
        # the curtain has closed.
        env.settings.update({"shutter": (1 << 16) | 30})
        # delta 20 ms, not 10: the FAKE fires 5-10 ms late on a loaded host
        # (the real piagent lands within 0.2 ms), and a 10 ms strobe could
        # then precede the fall edge and be judged a miss for the wrong reason.
        R.set_strobe({"enabled": True, "node": "cam2", "delta_ms": 20})
        time.sleep(1.0)                        # let the shutter apply land

        def strobed_run(label):
            r0 = R.start({"label": label})
            wait_for(lambda: R.trig_latency.get("cam2") is not None
                     and R.trig_latency.get("cam3") is not None, 25)
            R.capture_once()
            fla = os.path.join(r0["root"], "cam2", "flight_log.csv")
            flb = os.path.join(r0["root"], "cam3", "flight_log.csv")
            ok = wait_for(lambda: os.path.exists(fla) and os.path.exists(flb)
                          and len(read_flight(fla)[1]) >= 1
                          and len(read_flight(flb)[1]) >= 1, 15)
            R.stop()
            return r0, ok

        r1, delivered = strobed_run("strobed")
        check("both cameras deliver the strobed frame", bool(delivered))
        import rigcore as _rc
        browser = _rc.RunBrowser(env.events)
        shots = (browser.shots(r1["run_id"]) or {}).get("shots") or []
        lit = [s for s in shots if s.get("strobe")]
        check("the strobed shot carries a strobe verdict", bool(lit),
              json.dumps(shots)[-300:])
        s1 = lit[-1] if lit else {}
        check("pair spread is computed from the µs index, not the flight_log",
              s1.get("spread_src") == "index", str(s1.get("spread_src")))
        check("a 20 ms strobe inside a 1/30 exposure is ACCEPTED",
              (s1.get("strobe") or {}).get("ok") is True,
              json.dumps(s1.get("strobe")))

        # A 40 ms strobe against a 1/200 (5 ms) window: outside it however
        # late the fake fires (the window closes by ~+15 ms worst case), so the
        # MISS verdict is deterministic on any host. The contract is about the
        # verdict, not the margin.
        R.set_strobe({"delta_ms": 40})
        env.settings.update({"shutter": (1 << 16) | 200})
        time.sleep(1.0)                        # let the apply reach the fakes
        r2, delivered = strobed_run("strobed-fast")
        check("both cameras deliver the fast-shutter frame", bool(delivered))
        shots2 = (browser.shots(r2["run_id"]) or {}).get("shots") or []
        lit2 = [s for s in shots2 if s.get("strobe")]
        check("a 40 ms strobe against a 1/200 shutter is judged a MISS",
              bool(lit2) and (lit2[-1].get("strobe") or {}).get("ok") is False,
              json.dumps(lit2[-1] if lit2 else None))
        d2 = browser.detail(r2["run_id"])
        check("the run summary counts the missed strobe",
              ((d2.get("pairs") or {}).get("strobe_missed") or 0) >= 1,
              str((d2.get("pairs") or {}).get("strobe_missed")))
        # ---- the light must not depend on the strobe node's camera ----------
        # Replay the 2026-08-16 shape: the strobe node's CAMERA faults (card
        # stall) while its Pi stays healthy. The pulse must ride the
        # standalone /gpio/strobe so the partner's frames stay lit.
        R.set_strobe({"enabled": True, "node": "cam2", "delta_ms": 10})
        with a._lock:
            a.connected = False          # camera gone; piagent still up
        env.wait_state("cam2", "REACHABLE")
        pulses0 = a.strobe_fires
        res = R.capture_once()
        check("the partner camera still fires without the strobe node",
              ((res.get("results") or {}).get("cam3") or {}).get("ok") is True,
              json.dumps(res.get("results"))[:150])
        check("the strobe still pulses with its camera faulted (standalone)",
              wait_for(lambda: a.strobe_fires == pulses0 + 1, 8),
              "pulses %d -> %d" % (pulses0, a.strobe_fires))
        with a._lock:
            a.connected = True
        R.set_strobe({"enabled": False})
        check("no unhandled exception escaped any thread",
              not UNHANDLED, "; ".join(UNHANDLED[:3]))
    finally:
        env.close()


def suite_resource(opts):
    sect("resource discipline: threads, file descriptors, rings")
    env = Env([("cam2", "127.0.0.2", 2), ("cam3", "127.0.0.3", 3,
                                          {"has_imu": True})],
              poll=0.3, threaded=True, nav=FakeNav(fix=True))
    try:
        env.wait_state("cam2", "CAM_CONNECTED")
        env.wait_state("cam3", "CAM_CONNECTED")
        time.sleep(1.0)
        gc.collect()
        base = sample()                 # steady state: fakes + monitors only
        for i in range(6):
            env.runmgr.start({"label": "cycle%02d" % i})
            env.node("cam2").add_frame(epoch=time.time())
            env.node("cam3").add_frame(epoch=time.time())
            time.sleep(1.0)
            env.runmgr.stop()
        # Settle rather than sleep a fixed time: in-flight sockets and the
        # per-run EXIF-calibration threads are allowed to retire, a leak is not.
        wait_for(lambda: threading.active_count() <= base["threads"]
                 and fd_count() <= base["fds"], 25)
        gc.collect()
        after = sample()
        check("6 run start/stop cycles leave no threads behind",
              after["threads"] - base["threads"] <= 1,
              "%d -> %d threads (%s)" % (base["threads"], after["threads"],
                                         ",".join(sorted(set(
                                             t.name.split("-")[0]
                                             for t in threading.enumerate()))))
              )
        check("6 run start/stop cycles leak no file descriptors",
              after["fds"] - base["fds"] <= 1,
              "%d -> %d fds" % (base["fds"], after["fds"]))
        check("the event ring is bounded",
              len(env.events._ring) <= env.events._ring.maxlen,
              "%d/%d" % (len(env.events._ring), env.events._ring.maxlen))

        # A run started against the rig's real current state — PC-save off on
        # the body, so no frame ever lands — plus a node that has gone slow.
        t0 = threading.active_count()
        cam3 = env.node("cam3")
        cam3.pc_save = False
        cam3.set_fault("ilx", hang_s=2.0)
        env.runmgr.start({"label": "slownode"})
        time.sleep(1.0)
        env.runmgr.stop()
        left = wait_for(lambda: threading.active_count() <= t0, 20)
        contract("a stopped run strands no calibration thread on a slow node",
                 bool(left),
                 "run.py:426-445",
                 "_calibrate_exif fires one frame and then polls `for _ in "
                 "range(200): sleep(0.05); m.shots()`. The bound is 200 "
                 "iterations, not wall-clock, and m.shots() carries a 10 s "
                 "timeout. With PC-save disabled on the body no frame ever "
                 "arrives (run.py's own comment says as much) and against a "
                 "node that has gone slow the loop runs 200 x (poll + latency) "
                 "— minutes to half an hour of a thread that outlives its run, "
                 "hammering that node, one per node per run start. It also "
                 "gives up silently: no event, so the operator never learns the "
                 "EXIF clock offset was never measured.",
                 "%d threads still up 20 s after stop (was %d)"
                 % (threading.active_count(), t0))
        cam3.clear_faults()
        cam3.pc_save = True
        wait_for(lambda: threading.active_count() <= t0, 25)
        jsonl = os.path.getsize(env.events.path) if \
            os.path.exists(env.events.path) else 0
        note("rigd.jsonl grew to %d bytes for %d events (%.0f B/event) and "
             "rigcore.EventLog has no rotation or size cap (PROTOCOL.md:146 "
             "calls it 'rolling'): at the observed idle rate a multi-day "
             "deployment fills the Jetson's disk" %
             (jsonl, env.events._seq, jsonl / max(1, env.events._seq)))
    finally:
        env.close()


# ===========================================================================
# Soak — randomized fault sequences, watching for drift
# ===========================================================================
FAULT_DECK = [
    ("clean", lambda n: n.clear_faults()),
    ("hang", lambda n: n.set_fault("ilx", hang_s=random.uniform(0.5, 2.5))),
    ("hang-pia", lambda n: n.set_fault("pia", hang_s=random.uniform(0.5, 2.0))),
    ("http500", lambda n: n.set_fault("ilx", http500=True)),
    ("badjson", lambda n: n.set_fault("ilx", badjson=True)),
    ("flaky", lambda n: n.set_fault("both", flaky=0.3)),
    ("truncate", lambda n: n.set_fault("ilx", truncate=True)),
    ("cut", lambda n: n.set_fault("ilx", cut=True)),
    ("vanish", lambda n: n.set_fault("ilx", vanish=True)),
    ("fail_set", lambda n: n.set_fault("ilx", fail_set=["iso", "aperture"])),
    ("connect_lies", lambda n: (n.set_fault("ilx", connect_lies=True),
                                setattr(n, "connected", False))),
    ("drift", lambda n: n.drift(iso=random.choice([100, 1600, ISO_AUTO]),
                                aperture=random.choice([400, 560, 1100]))),
    ("drop", lambda n: n.down()),
    ("reboot", lambda n: n.reboot(down_s=random.uniform(0.3, 2.0), block=False)),
]


def soak(seconds, opts):
    sect("soak: %d s of randomized fault sequences" % seconds)
    random.seed(opts.seed)
    env = Env([("cam2", "127.0.0.2", 2),
               ("cam3", "127.0.0.3", 3, {"has_imu": True})],
              poll=1.0, threaded=True, nav=FakeNav(fix=True))
    stop = threading.Event()

    def reconciler():
        while not stop.wait(3.0):
            try:
                env.settings.reconcile_all(force=False)
            except Exception as e:                            # noqa: BLE001
                UNHANDLED.append("reconcile loop: %r" % e)

    th = threading.Thread(target=reconciler, daemon=True, name="soak-reconcile")
    th.start()
    t_end = time.time() + seconds
    marks, applied = [], []
    gc.collect()
    time.sleep(2.0)
    base = sample()
    base["events"] = env.events._seq
    runs = frames = 0
    err = None
    try:
        while time.time() < t_end:
            node = random.choice(list(env.nodes.values()))
            name, fn = random.choice(FAULT_DECK)
            try:
                fn(node)
            except Exception as e:                            # noqa: BLE001
                UNHANDLED.append("fault %s: %r" % (name, e))
            applied.append(name)
            if random.random() < 0.35:
                if env.runmgr.active:
                    env.runmgr.stop()
                else:
                    env.runmgr.start({"label": "soak%03d" % runs})
                    runs += 1
            if env.runmgr.active and random.random() < 0.4:
                try:
                    env.runmgr.capture_once(af=False)
                except Exception as e:                        # noqa: BLE001
                    UNHANDLED.append("capture_once: %r" % e)
            for n in env.nodes.values():
                try:
                    if n._servers:
                        n.add_frame(epoch=time.time())
                        n.push_edge(epoch=time.time())
                        n.push_imu(epoch=time.time())
                        frames += 1
                except Exception:                             # noqa: BLE001
                    pass
            time.sleep(random.uniform(0.4, 1.6))
            if random.random() < 0.3:
                node.clear_faults()
                if not node._servers:
                    node.up()
            s = sample()
            s["t"] = round(time.time() - (t_end - seconds), 1)
            s["events"] = env.events._seq
            marks.append(s)
    except Exception as e:                                    # noqa: BLE001
        err = e
    finally:
        stop.set()
        for n in env.nodes.values():
            n.clear_faults()
            n.up()
        try:
            if env.runmgr.active:
                env.runmgr.stop()
        except Exception:                                     # noqa: BLE001
            pass

    time.sleep(15.0)                     # let stragglers retire before sampling
    gc.collect()
    end = sample()
    end["events"] = env.events._seq
    print("  faults applied: %s"
          % ", ".join("%s x%d" % (k, applied.count(k))
                      for k in sorted(set(applied))))
    print("  runs started %d, frames staged %d, events %d"
          % (runs, frames, end["events"] - base["events"]))
    print("  threads %d -> %d   fds %d -> %d   rss %.1f -> %.1f MB"
          % (base["threads"], end["threads"], base["fds"], end["fds"],
             base["rss_mb"], end["rss_mb"]))
    peak = max((m["threads"] for m in marks), default=end["threads"])
    check("soak completed without the harness itself failing", err is None,
          repr(err) if err else "")
    check("no unhandled exception in any thread during the soak",
          not UNHANDLED, "; ".join(UNHANDLED[:5]))
    check("threads settle back after the storm (no runaway growth)",
          end["threads"] - base["threads"] <= 8,
          "%d -> %d (peak %d)" % (base["threads"], end["threads"], peak))
    check("file descriptors do not leak",
          end["fds"] - base["fds"] <= 10,
          "%d -> %d" % (base["fds"], end["fds"]))
    check("resident memory does not run away",
          end["rss_mb"] - base["rss_mb"] < 60,
          "%.1f -> %.1f MB" % (base["rss_mb"], end["rss_mb"]))
    check("the event ring stays bounded",
          len(env.events._ring) <= env.events._ring.maxlen,
          "%d/%d entries" % (len(env.events._ring), env.events._ring.maxlen))
    check("every node is reachable again once the faults clear",
          wait_for(lambda: all(m.state == "CAM_CONNECTED"
                               for m in env.monitors), 15),
          ", ".join("%s=%s" % (m.name_, m.state) for m in env.monitors))
    env.settings.reconcile_all(force=False)
    time.sleep(2.0)
    env.settings.reconcile_all(force=False)
    want = env.settings.get()
    same = all(env.nodes["cam2"].raw(k) == env.nodes["cam3"].raw(k)
               for k in CONVERGE_KEYS)
    check("the fleet reconverges to one identical settings vector after the storm",
          same and all(env.nodes["cam2"].raw(k) == want[k]
                       for k in CONVERGE_KEYS),
          json.dumps({k: [env.nodes["cam2"].raw(k), env.nodes["cam3"].raw(k),
                          want[k]] for k in CONVERGE_KEYS
                      if env.nodes["cam2"].raw(k) != want[k]
                      or env.nodes["cam3"].raw(k) != want[k]}))
    if marks:
        print("  thread/fd/rss trace: %s"
              % "  ".join("%.0fs:%d/%d/%.0fMB"
                          % (m["t"], m["threads"], m["fds"], m["rss_mb"])
                          for m in marks[::max(1, len(marks) // 8)]))

    # Whatever the storm did, every flight_log it produced must still parse.
    proto = protocol_flight_header()
    logs, bad_hdr, bad_row, bad_name, missing, dupes, total = 0, [], [], [], [], [], 0
    for dirpath, _dirs, files in os.walk(rigcore.RUNS_DIR):
        if "flight_log.csv" not in files:
            continue
        logs += 1
        p = os.path.join(dirpath, "flight_log.csv")
        hdr, rows = read_flight(p)
        total += len(rows)
        if ",".join(hdr) != proto:
            bad_hdr.append(p)
        seen = set()
        for row in rows:
            if len(row) != 23:
                bad_row.append((p, len(row)))
                continue
            if not FNAME_RE.match(row[0]):
                bad_name.append(row[0])
            elif not os.path.exists(os.path.join(dirpath, row[0])):
                missing.append(row[0])
            if row[0] in seen:
                dupes.append(row[0])
            seen.add(row[0])
    check("every flight_log written during the soak still parses",
          logs > 0 and not bad_hdr and not bad_row,
          "%d logs, %d rows, %d bad headers, %d malformed rows"
          % (logs, total, len(bad_hdr), len(bad_row)))
    check("every logged filename is well-formed and present on disk",
          not bad_name and not missing,
          "%d malformed, %d missing (%s)" % (len(bad_name), len(missing),
                                             (bad_name + missing)[:2]))
    contract("no duplicate frame rows in any flight_log", not dupes,
             "run.py:59-62 (_fmt_fname) + run.py:139-146 (the unguarded open)",
             "under randomized load the centisecond-resolution filename "
             "collides: two frames render to the same CamN_YYYYMMDD_hhmmss.ss"
             ".jpg, the second overwrites the first on disk, and the flight_log "
             "carries two rows for the one surviving file. Every collision is a "
             "survey frame destroyed with no error anywhere.",
             "%d duplicated names in %d rows across %d logs: %s"
             % (len(dupes), total, logs, sorted(set(dupes))[:3]))
    kinds = {}
    for e in env.evs(0):
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    print("  event kinds: %s"
          % ", ".join("%s=%d" % kv for kv in sorted(kinds.items())))
    check("the storm was actually felt (transitions + failures journalled)",
          kinds.get("node_transition", 0) >= 2,
          "%d node transitions" % kinds.get("node_transition", 0))
    env.close()


# ===========================================================================
SUITES = [("fake", suite_fake), ("monitor", suite_monitor),
          ("strobe", suite_strobe),
          ("settings", suite_settings), ("runmgr", suite_runmgr),
          ("pull", suite_pull), ("resource", suite_resource)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--soak", type=int, default=0,
                    help="run the randomized soak for N seconds after the suites")
    ap.add_argument("--only", default="",
                    help="comma-separated suite names: "
                         + ",".join(k for k, _ in SUITES))
    ap.add_argument("--quick", action="store_true",
                    help="skip the slow client-timeout probes")
    ap.add_argument("--soak-only", action="store_true",
                    help="skip the regression suites, run only the soak")
    ap.add_argument("--seed", type=int, default=int(time.time()) & 0xFFFF)
    ap.add_argument("--allow-known", action="store_true",
                    help="exit 0 when the only failures are recorded defects")
    a = ap.parse_args()

    print("Wild Sync soaktest — %s (pid %d, seed %d)"
          % (time.strftime("%Y-%m-%d %H:%M:%S"), os.getpid(), a.seed))
    print("fakes only: every request is pinned to 127.0.0.0/8 by the netguard; "
          "the live fleet is never contacted")
    random.seed(a.seed)
    t0 = time.time()
    want = [s.strip() for s in a.only.split(",") if s.strip()]
    if not a.soak_only:
        for name, fn in SUITES:
            if want and name not in want:
                continue
            try:
                fn(a)
            except Exception as e:                            # noqa: BLE001
                import traceback
                traceback.print_exc()
                check("suite %s ran to completion" % name, False, repr(e))
    if a.soak:
        try:
            soak(a.soak, a)
        except Exception as e:                                # noqa: BLE001
            import traceback
            traceback.print_exc()
            check("soak ran to completion", False, repr(e))

    print("\n%s" % ("=" * 72))
    print("%d passed, %d failed in %.0f s" % (len(PASS), len(FAIL),
                                              time.time() - t0))
    if BLOCKED:
        print("NETGUARD BLOCKED (a test tried to leave loopback!): %s"
              % ", ".join(sorted(set(BLOCKED))))
    if NOTES:
        print("\nNOTES")
        for n in NOTES:
            print("  - %s" % n)
    if DEFECTS:
        print("\nDEFECTS IN CODE THIS HARNESS DOES NOT OWN (%d)" % len(DEFECTS))
        for d in DEFECTS:
            print("\n  * %s" % d["check"])
            print("    where:    %s" % d["ref"])
            print("    observed: %s" % (d["detail"] or "-"))
            print("    scenario: %s" % d["scenario"])
    if FAIL:
        print("\nFAILED: " + ", ".join(FAIL))
        known = {d["check"] for d in DEFECTS}
        if a.allow_known and set(FAIL) <= known:
            print("(only recorded defects failed; --allow-known set)")
            return 0
        return 1
    print("\nALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

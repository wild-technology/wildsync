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
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rigcore
from rigcore import (NODES, EventLog, NodeMonitor, SettingsManager, TimeSync,
                     http_json, http_bytes)
from run import RunManager

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = int(os.environ.get("RIGD_PORT", "9090"))
UI_PATH = os.path.join(HERE, "rig_ui.html")


# ---------------------------------------------------------------------------
# Anomaly detectors — cheap checks over current fleet state, each with the
# evidence and a suggested action an operator (or agent) can act on.
# ---------------------------------------------------------------------------
class Anomalies:
    def __init__(self, monitors, runmgr, nav, events):
        self.monitors = monitors
        self.runmgr = runmgr
        self.nav = nav
        self.events = events
        self._flap = {}          # node -> deque of transition times
        self._last = {}

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
                                   "piagent are running"))
            elif st == NodeMonitor.REACHABLE:
                out.append(self._a("camera_absent", m.name_,
                                   "node up but camera not claimed",
                                   {"log": (status.get("log") or [])[-1:]},
                                   "check USB cable is a data cable, camera on, "
                                   "PC Remote mode"))
            conv = snap.get("convergence") or {}
            if conv.get("synced") is False and conv.get("diverged"):
                out.append(self._a("settings_divergent", m.name_,
                                   "settings will not converge",
                                   {"fields": conv["diverged"]},
                                   "check the field is settable in the current "
                                   "exposure mode; inspect ilxctl log"))
            batt = status.get("battery")
            if isinstance(batt, (int, float)) and 0 <= batt <= 15:
                out.append(self._a("battery_low", m.name_,
                                   "battery low (%s%%)" % batt, {"battery": batt},
                                   "bring the 12 V harness supply up or swap "
                                   "battery"))
            disk = h.get("disk_free_mb")
            if isinstance(disk, (int, float)) and disk < 2000:
                out.append(self._a("disk_low", m.name_,
                                   "node disk low (%s MB)" % disk,
                                   {"disk_free_mb": disk},
                                   "clear old frames from the PC-save dir"))
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
                    out.append(self._a("nav_gateway_down", None,
                                       "iKonvert sending no data",
                                       {"health": self.nav.health()},
                                       "the iKonvert draws power from the N2K "
                                       "bus, not USB - check bus power and the "
                                       "gateway's POWER LED"))
                elif snap and snap.get("lat") is None:
                    out.append(self._a("nav_no_fix", None, "no GPS fix",
                                       {"snap": {k: snap.get(k) for k in
                                                 ("sats", "fix_source", "age_s")}},
                                       "check the N2K backbone / GPS source"))
            except Exception:  # noqa: BLE001
                pass
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
                                       "check node link speed and disk"))
        # emit newly-appearing anomalies once
        cur = {(a["kind"], a["node"]) for a in out}
        for key in cur - set(self._last):
            a = next(x for x in out if (x["kind"], x["node"]) == key)
            self.events.emit("warn", a["kind"], a["msg"], node=a["node"],
                             **a.get("evidence", {}))
        self._last = {k: now for k in cur}
        return out

    @staticmethod
    def _a(kind, node, msg, evidence, action):
        return {"kind": kind, "node": node, "msg": msg, "evidence": evidence,
                "suggested_action": action, "since": time.time()}


class Rig:
    def __init__(self):
        self.events = EventLog()
        self.monitors = [NodeMonitor(n, self.events) for n in NODES]
        self.settings = SettingsManager(self.monitors, self.events)
        self.timesync = TimeSync(self.events)
        self.nav = self._start_nav()
        self.runmgr = RunManager(self.monitors, self.settings, self.timesync,
                                 self.events, self.nav)
        self.anomalies = Anomalies(self.monitors, self.runmgr, self.nav,
                                   self.events)
        self._stop = threading.Event()
        for m in self.monitors:
            m.start()
        threading.Thread(target=self._reconcile_loop, daemon=True).start()
        threading.Thread(target=self._nav_time_loop, daemon=True).start()
        threading.Thread(target=self._startup_calibrate, daemon=True).start()
        self.events.emit("info", "lifecycle", "rigd up", port=PORT,
                         nodes=[m.name_ for m in self.monitors],
                         nav=bool(self.nav))

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
        return reader

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
            "run": self.runmgr.status(),
            "anomaly_counts": self.events.recent_counts(),
            "anomalies": self.anomalies.scan(),
        }

    def fleet(self):
        return {"nodes": [self._node_view(m) for m in self.monitors],
                "run": self.runmgr.status(),
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
            "convergence": snap.get("convergence"),
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
        self._stop.set()
        for m in self.monitors:
            m.stop()


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
            elif p == "/api/run":
                self._json(RIG.runmgr.status())
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

    def _proxy_frame(self, node, name):
        m = self._mon(node)
        if not m or not name:
            self._json({"ok": False, "error": "bad node/name"}, 400); return
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
        data, err = http_bytes("http://%s:8080/liveview.jpg" % m.host, timeout=8)
        if err or not data:
            self._json({"ok": False, "error": err or "no liveview"}, 503); return
        self._bytes(data, "image/jpeg")

    def do_POST(self):
        u = urlparse(self.path)
        p = u.path
        b = self._read_body()
        try:
            if p == "/api/settings":
                self._json({"ok": True, "applied": RIG.settings.update(b)})
            elif p == "/api/settings/auto":
                RIG.settings.set_auto(bool(b.get("on")))
                self._json({"ok": True})
            elif p == "/api/ev":
                cur = RIG.settings.bump_ev(int(b.get("steps", 0)))
                self._json({"ok": True, "expcomp_mev": cur})
            elif p == "/api/run/start":
                self._json(RIG.runmgr.start(b or {}))
            elif p == "/api/run/stop":
                self._json(RIG.runmgr.stop())
            elif p == "/api/capture":
                self._json(RIG.runmgr.capture_once(af=bool(b.get("af"))))
            elif p == "/api/calibrate":
                self._json({"ok": True,
                            "latency_ms": {k: round(v * 1000, 2) for k, v in
                                           RIG.runmgr.calibrate_trigger(
                                               samples=int(b.get("samples", 5))
                                           ).items()}})
            elif p == "/api/reconcile":
                RIG.settings.reconcile_all(force=True)
                self._json({"ok": True})
            elif p in ("/api/focus/mode", "/api/focus/drive", "/api/focus/position",
                       "/api/zoom/drive", "/api/zoom/position", "/api/zoom/setting"):
                # Lens controls fan out to every connected camera so the pair
                # stays identically framed and focused. A "node" key targets one.
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
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        RIG.stop()


if __name__ == "__main__":
    main()

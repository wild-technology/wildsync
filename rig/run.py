"""run.py — a transect run: firing, pulling, renaming, and the flight log.

A "run" is one survey line. Starting it lays down a directory, wires up a pull
worker per online camera, and (if asked) drives a synchronized capture loop.
Each frame that lands is renamed to the rig convention, timestamped with its
best available capture instant, and written as one row of that camera's
flight_log alongside the nav + IMU state at that instant.

The design keeps the cameras independent: one camera stalling or dropping does
not stall the others. Every worker owns its own thread, its own baseline of
"frames already seen", and its own flight_log handle.
"""

import csv
from collections import deque
import json
import os
import threading
import time

from rigcore import http_json, http_bytes, RUNS_DIR

try:
    import navlog
except ImportError:            # raw NMEA logging is optional
    navlog = None

FLIGHT_HEADER = [
    "filename", "datetime", "lat", "long", "xutm", "yutm", "utm_zone",
    "depth_from_xplore9", "pitch", "roll", "yaw", "heading_mag_xplore",
    "heading_imu", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
    "imu_temp_c", "capture_source", "time_source", "time_err_ms",
]


def _exif_capture_epoch(jpeg_bytes):
    """Camera capture epoch from EXIF, or None. Pure-bytes, no temp file."""
    try:
        from io import BytesIO
        from PIL import Image
        with Image.open(BytesIO(jpeg_bytes)) as im:
            ex = im.getexif().get_ifd(0x8769)
        dt = ex.get(0x9003)
        ss = str(ex.get(0x9291) or "0").strip()
        if not dt:
            return None
        base = time.mktime(time.strptime(dt, "%Y:%m:%d %H:%M:%S"))
        frac = float("0." + ss) if ss.isdigit() else 0.0
        return base + frac
    except Exception:  # noqa: BLE001
        return None


def _split_epoch(epoch):
    """(struct_time, '.ss') with the centisecond carry applied to BOTH halves.

    Rounding the fraction independently of the seconds is a one-second error
    waiting to happen: at ...000.999 the fraction rounds to 1.00, which used to
    be pasted straight on to an unrounded 'seconds' field, stamping the frame a
    full second early. That column is what nav is correlated against, so ~0.5%
    of frames would be mis-positioned. Round once, derive both from the result."""
    cs = int(round(epoch * 100))          # centiseconds since the epoch
    whole, frac = divmod(cs, 100)
    return time.gmtime(whole), ".%02d" % frac


def _fmt_dt(epoch):
    """UTC YYMMDD_hhmmss.ss for the flight log datetime column."""
    lt, frac = _split_epoch(epoch)
    return time.strftime("%y%m%d_%H%M%S", lt) + frac


def _fmt_fname(cam_num, epoch, ext=".jpg"):
    """CamN_YYYYMMDD_hhmmss.ss<ext>.

    `ext` follows the frame the camera actually sent: shooting RAW+JPEG (Sony's
    factory default, which a power-cycled body comes back in) delivers .ARW as
    well, and naming that .jpg buries raw data in a file claiming to be a JPEG."""
    lt, frac = _split_epoch(epoch)
    return "Cam%d_%s%s%s" % (cam_num, time.strftime("%Y%m%d_%H%M%S", lt), frac, ext)


def _uniq_dest(cam_dir, fname):
    """A path that does not already exist, keeping the stem readable.

    Centisecond resolution is not a unique key: a RAW+JPEG pair, or two frames
    arriving in one poll with no edge and no EXIF, can resolve to the same
    instant. Opening the same path twice silently destroys the first frame while
    the flight_log still gains two rows - the survey then has a record with no
    image. Never overwrite; disambiguate instead."""
    dest = os.path.join(cam_dir, fname)
    if not os.path.exists(dest):
        return fname, dest
    stem, ext = os.path.splitext(fname)
    for n in range(1, 1000):
        alt = "%s_%d%s" % (stem, n, ext)
        p = os.path.join(cam_dir, alt)
        if not os.path.exists(p):
            return alt, p
    return fname, dest                     # 1000 collisions: give up, overwrite


class PullWorker(threading.Thread):
    def __init__(self, mon, cam_dir, provider):
        super().__init__(daemon=True)
        self.mon = mon
        self.cam_num = mon.node["cam_num"]
        self.cam_dir = cam_dir
        self.provider = provider          # RunManager, for nav/imu/time/events
        self._stopev = threading.Event()
        self.seen = set()
        self.pulled = 0
        self.failed = 0
        self._last_cap = None
        self._cmds = deque(maxlen=256)   # expected exposure instants, oldest first
        self.intervals = []               # capture-epoch deltas, for jitter
        self._flight_fh = None
        self._flight = None
        # command epochs keyed by original camera filename, filled at fire time
        self.cmd_epoch = {}
        self._lock = threading.Lock()

    def note_command(self, epoch):
        """Queue an expected exposure instant.

        This used to overwrite a single slot, which is fine at one frame every
        few seconds and useless at 2 fps: by the time a frame arrives the slot
        holds the newest command, not the one that produced it."""
        with self._lock:
            self._cmds.append(epoch)

    def stop(self):
        self._stopev.set()

    def run(self):
        os.makedirs(self.cam_dir, exist_ok=True)
        path = os.path.join(self.cam_dir, "flight_log.csv")
        new = not os.path.exists(path)
        self._flight_fh = open(path, "a", newline="")
        self._flight = csv.writer(self._flight_fh)
        if new:
            self._flight.writerow(FLIGHT_HEADER)
            self._flight_fh.flush()
        # Baseline: everything already on the camera is "old".
        for s in self.mon.shots():
            self.seen.add(s["name"])
        self.provider.events.emit("info", "pull", "worker up, baseline %d frames"
                                  % len(self.seen), node=self.mon.name_)
        while not self._stopev.wait(0.4):
            try:
                self._poll_once()
            except Exception as e:  # noqa: BLE001
                self.provider.events.emit("error", "pull",
                                          "poll error: %s" % e, node=self.mon.name_)
        try:
            self._flight_fh.close()
        except OSError:
            pass

    def _poll_once(self):
        if not self.mon.is_connected():
            return
        shots = self.mon.shots()
        fresh = [s for s in shots if s["name"] not in self.seen]
        fresh.sort(key=lambda s: s["name"])
        for s in fresh:
            self.seen.add(s["name"])
            self._handle(s["name"], s.get("size", 0))

    def _handle(self, name, size):
        url = "http://%s:8080/shot/%s" % (self.mon.host, name)
        data, err = http_bytes(url, timeout=30)
        if err or not data:
            self.failed += 1
            self.provider.events.emit("warn", "pull_fail",
                                      "download %s failed: %s" % (name, err),
                                      node=self.mon.name_)
            return
        # A link that dies mid-frame yields a short read with no exception, and a
        # half-written frame that still gets a flight_log row is worse than a
        # missing one: it looks like real survey data. Reject it here.
        if size and len(data) != size:
            self.failed += 1
            self.provider.events.emit("error", "pull_fail",
                                      "%s truncated: got %d of %d bytes"
                                      % (name, len(data), size),
                                      node=self.mon.name_)
            return
        ext = os.path.splitext(name)[1].lower() or ".jpg"
        if ext in (".jpg", ".jpeg") and not (data[:2] == b"\xff\xd8"
                                             and data[-2:] == b"\xff\xd9"):
            self.failed += 1
            self.provider.events.emit("error", "pull_fail",
                                      "%s is not a complete JPEG (bad SOI/EOI)"
                                      % name, node=self.mon.name_)
            return
        # Best capture instant: (GPIO edge) > corrected EXIF > command epoch.
        source, epoch, terr = self._capture_instant(name, data)
        fname, dest = _uniq_dest(self.cam_dir,
                                 _fmt_fname(self.cam_num, epoch, ext))
        try:
            with open(dest, "wb") as fh:
                fh.write(data)
        except OSError as e:
            self.failed += 1
            self.provider.events.emit("error", "pull_fail",
                                      "write %s failed: %s" % (fname, e),
                                      node=self.mon.name_)
            return
        self.pulled += 1
        with self._lock:
            if self._last_cap is not None:
                self.intervals.append(epoch - self._last_cap)
                if len(self.intervals) > 500:
                    self.intervals.pop(0)
            self._last_cap = epoch
        self._write_flight(fname, name, epoch, source, terr)
        self.provider.on_frame(self.mon.name_, self.cam_num, fname, name, epoch)

    def _capture_instant(self, name, data):
        # 1. The GPIO exposure edge belonging to THIS frame's fire command.
        #
        # Matching frames to edges strictly FIFO looks right until the camera
        # drops one - then every later frame is paired with the previous
        # frame's edge and the whole run is silently skewed, with each row
        # still claiming capture_source=gpio_edge. Instead, each frame claims
        # the oldest queued command and takes the edge nearest that instant;
        # a command whose frame never arrives ages out on its own.
        while True:
            with self._lock:
                target = self._cmds[0] if self._cmds else None
            if target is None:
                break
            edge = self.provider.match_exposure_edge(self.mon, expected=target)
            if edge is not None:
                with self._lock:
                    if self._cmds:
                        self._cmds.popleft()
                return "gpio_edge", edge, 0.0
            if time.time() - target > 5.0:
                # Old enough that no frame can still be coming for it: this fire
                # produced no frame (or no edge). Drop it and try the next.
                with self._lock:
                    if self._cmds:
                        self._cmds.popleft()
                continue
            break
        # 2. EXIF, corrected by the per-node offset measured at run start.
        cam_epoch = _exif_capture_epoch(data)
        corrected = self.provider.timesync.correct_exif(self.mon.name_, cam_epoch)
        if corrected:
            return "exif", corrected, None
        # 3. The command epoch we fired at (or now, for card-review pulls).
        with self._lock:
            cmd = self._cmds.popleft() if self._cmds else None
        if cmd:
            return "command", cmd, None
        now, _ = self.provider.timesync.now()
        return "command", now, None

    def _write_flight(self, fname, orig, epoch, source, terr):
        nav = self.provider.nav_snapshot(epoch)
        imu = self.provider.imu_snapshot(epoch)
        _, tsource = self.provider.timesync.now()
        row = {k: "" for k in FLIGHT_HEADER}
        row.update({
            "filename": fname,
            "datetime": _fmt_dt(epoch),
            "capture_source": source,
            "time_source": tsource,
            "time_err_ms": "" if terr is None else round(terr * 1000, 1),
        })
        if nav:
            row.update({
                "lat": _r(nav.get("lat"), 7), "long": _r(nav.get("lon"), 7),
                "xutm": _r(nav.get("xutm"), 2), "yutm": _r(nav.get("yutm"), 2),
                "utm_zone": nav.get("utm_zone", ""),
                "depth_from_xplore9": _r(nav.get("depth_m"), 2),
                "heading_mag_xplore": _r(nav.get("heading_mag_deg"), 1),
            })
        if imu:
            row.update({
                "pitch": _r(imu.get("pitch"), 2), "roll": _r(imu.get("roll"), 2),
                "yaw": _r(imu.get("yaw"), 2),
                "heading_imu": _r(imu.get("heading"), 1),
                "ax_g": _r(imu.get("ax"), 4), "ay_g": _r(imu.get("ay"), 4),
                "az_g": _r(imu.get("az"), 4),
                "gx_dps": _r(imu.get("gx"), 3), "gy_dps": _r(imu.get("gy"), 3),
                "gz_dps": _r(imu.get("gz"), 3),
                "imu_temp_c": _r(imu.get("temp"), 1),
            })
        self._flight.writerow([row[k] for k in FLIGHT_HEADER])
        self._flight_fh.flush()
        self.provider.index_frame(self.cam_num, fname, orig, epoch, source)

    def stats(self):
        with self._lock:
            iv = list(self.intervals)
        import statistics as st
        jit = (st.pstdev(iv) * 1000) if len(iv) > 1 else None
        return {"pulled": self.pulled, "failed": self.failed,
                "last_capture": self._last_cap,
                "interval_mean_s": round(st.mean(iv), 3) if iv else None,
                "interval_jitter_ms": round(jit, 1) if jit else None}


def _r(v, n):
    return "" if v is None else round(v, n)


class RunManager:
    def __init__(self, monitors, settings, timesync, events, nav, imu_node="cam3"):
        self.monitors = monitors
        self.settings = settings
        self.timesync = timesync
        self.events = events
        self.nav = nav
        self.imu_node = imu_node
        self._lock = threading.Lock()
        self.active = None            # dict describing the current run
        self.workers = {}
        # node -> TRIGGER->EXPOSURE latency in seconds, measured not assumed.
        self.trig_latency = {}
        self._cap_thread = None
        self._cap_stop = None

    # ---- lifecycle --------------------------------------------------------
    def start(self, config):
        with self._lock:
            if self.active:
                return {"ok": False, "error": "run already active",
                        "run_id": self.active["run_id"]}
            now, _ = self.timesync.now()
            label = config.get("label", "transect")
            rid = time.strftime("%y%m%d_%H%M", time.gmtime(now)) + "_" + \
                _slug(label)
            root = os.path.join(RUNS_DIR, rid)
            os.makedirs(root, exist_ok=True)
            events_fh = open(os.path.join(root, "events.log"), "a")
            self.events.set_run_file(events_fh)
            # navlog stamps each line with wall AND monotonic time. Wall time is
            # not monotonic - chrony steps it, and a GPS fix arriving mid-run
            # shifts it - so a single-clock log cannot be used to measure
            # intervals afterwards, and cannot even reveal that a step happened.
            if navlog is not None:
                nmea_fh = navlog.open_run_log(root, run_id=rid)
                raw_hook = nmea_fh                  # exposes write_line()
            else:
                nmea_fh = open(os.path.join(root, "nmea_raw.log"), "a")
                raw_hook = lambda ep, ln: nmea_fh.write(
                    "%.3f %s\n" % (ep, ln)) or nmea_fh.flush()
            if self.nav:
                self.nav.set_raw_hook(raw_hook)
            live = [m for m in self.monitors if m.is_connected()]
            run = {"run_id": rid, "root": root, "label": label,
                   "started": now, "config": config,
                   "nodes": [m.name_ for m in live],
                   "events_fh": events_fh, "nmea_fh": nmea_fh,
                   "index": [], "alerts": []}
            self.active = run
            self.workers = {}
            for m in live:
                cam_dir = os.path.join(root, m.name_)
                w = PullWorker(m, cam_dir, self)
                w.start()
                self.workers[m.name_] = w
            # Measure per-camera EXIF clock offset so timestamps are meaningful.
            # Do it off-thread: it fires a frame and waits for the transfer,
            # which must never block the run from starting (and cannot complete
            # at all while PC-save is disabled on the body).
            for m in live:
                threading.Thread(target=self._calibrate_exif, args=(m,),
                                 daemon=True).start()
            self._write_run_json()
            # Begin this run's edge stream at "now" so no bench-test edge is
            # inherited as a survey frame's capture instant.
            self.reset_edge_cursors()
            self.events.emit("info", "run", "run started: %d cameras" % len(live),
                             nodes=run["nodes"])
            # Re-measure per-camera trigger latency for THIS run: it moves with
            # lens, drive mode and body state, so a figure from an hour ago is
            # not the figure that will align these exposures. Off-thread so the
            # HTTP caller is not held for the duration.
            if config.get("calibrate", True):
                threading.Thread(target=self._calibrate_then_reset,
                                 daemon=True).start()
            # Optional automatic synchronized capture loop.
            if config.get("auto_capture"):
                self._start_capture_loop(config)
            return {"ok": True, "run_id": rid, "nodes": run["nodes"],
                    "root": root}

    def stop(self):
        with self._lock:
            if not self.active:
                return {"ok": False, "error": "no active run"}
            self._stop_capture_loop()
            for w in self.workers.values():
                w.stop()
            time.sleep(0.6)
            self._write_run_json(final=True)
            run = self.active
            try:
                run["events_fh"].close(); run["nmea_fh"].close()
            except OSError:
                pass
            if self.nav:
                self.nav.set_raw_hook(None)
            self.events.set_run_file(None)
            rid = run["run_id"]
            summary = {n: w.stats() for n, w in self.workers.items()}
            self.active = None
            self.workers = {}
            self.events.emit("info", "run", "run stopped: %s" % rid,
                             summary=summary)
            return {"ok": True, "run_id": rid, "summary": summary}

    # ---- capture ----------------------------------------------------------
    # How far ahead the shared fire instant is scheduled. It must cover the HTTP
    # round trip to the slowest node, because every node has to receive the
    # request *before* the instant arrives or it fires late.
    SYNC_LEAD_S = 0.15
    # An EXPOSURE edge older than this cannot belong to a frame arriving now.
    EDGE_MAX_AGE_S = 30.0
    # FOCUS is asserted this far ahead of each TRIGGER. The camera will not
    # accept a trigger without it, but holding it down for the whole run
    # half-presses the body and AE-locks it, which would freeze auto-ISO at the
    # light level the run started in. Per-shot keeps metering live.
    FOCUS_LEAD_MS = 40

    def capture_once(self, af=False, target=None):
        """Fire every connected camera at ONE shared absolute instant.

        Dispatching in parallel and telling each node "fire now" only makes the
        requests concurrent - each camera still fires whenever its own request
        lands, which measured 10-80 ms apart across two nodes. Handing every
        node the same target epoch instead lets each piagent busy-wait to it
        locally, off a clock disciplined to the Jetson: measured 0.65 ms mean
        inter-camera skew, worst 3.10 ms. For a stereo pair at 1 m/s that is the
        difference between ~30 mm and ~0.7 mm of baseline error."""
        live = [m for m in self.monitors if m.is_connected()]
        if not live:
            return {"ok": False, "error": "no cameras connected"}
        results = {}
        threads = []
        if target is None:
            target = time.time() + self.SYNC_LEAD_S

        def _fire(m):
            has_gpio = (m.health.get("gpio", {}) or {}).get("available")
            if has_gpio:
                # Fire this node early by its own measured trigger latency so
                # the EXPOSURES coincide, not the pulses. piagent asserts FOCUS
                # itself, FOCUS_LEAD_MS ahead, so no run needs to hold it.
                lead = self.trig_latency.get(m.name_, 0.0)
                r = http_json("http://%s:8081/gpio/fire" % m.host,
                              {"at_epoch": target - lead, "pulse_ms": 5,
                               "focus_lead_ms": self.FOCUS_LEAD_MS},
                              timeout=10 + self.SYNC_LEAD_S)
                results[m.name_] = {"path": "gpio", "lead_ms": round(lead * 1000, 2),
                                    **r}
                # Queue the expected EXPOSURE instant, not the trigger instant:
                # this is the key the frame is later matched to an edge by, and
                # the edge lands one trigger-latency after the pulse. Because
                # each node is fired early by its own latency, that is `target`
                # for every camera - the whole point of the compensation.
                t = target
            else:
                # No harness on this node: it cannot be scheduled, so it fires on
                # arrival and its frames carry the usual USB uncertainty.
                t = time.time()
                r = m.shutter(af=af)
                results[m.name_] = {"path": "usb", "ok": r.get("ok", True)}
            w = self.workers.get(m.name_)
            if w:
                w.note_command(t)

        for m in live:
            th = threading.Thread(target=_fire, args=(m,), daemon=True)
            th.start(); threads.append(th)
        for th in threads:
            th.join(timeout=35)
        self.events.emit("info", "capture", "manual capture fired",
                         results={k: v.get("path") for k, v in results.items()})
        return {"ok": True, "results": results}

    def _start_capture_loop(self, config):
        period = float(config.get("interval_s", 2.0))
        count = int(config.get("frames", 0))
        self._cap_stop = threading.Event()

        def _loop():
            k = 0
            start = time.time() + 1.0
            inflight = []
            while not self._cap_stop.is_set():
                if count and k >= count:
                    break
                # Absolute grid, so lateness in one shot cannot accumulate.
                target = start + k * period
                # Wake a lead-time early: capture_once has to reach every node
                # BEFORE the instant, and calling it at the instant itself made
                # each shot SYNC_LEAD_S late. Dispatching it inline also
                # serialised the lead into the period, which is why a requested
                # 0.5 s came out at 0.637 s.
                wake = target - self.SYNC_LEAD_S
                while time.time() < wake and not self._cap_stop.is_set():
                    time.sleep(min(0.02, max(0, wake - time.time())))
                if self._cap_stop.is_set():
                    break
                th = threading.Thread(target=self.capture_once,
                                      kwargs={"af": False, "target": target},
                                      daemon=True)
                th.start()
                inflight.append(th)
                inflight = [t for t in inflight if t.is_alive()]
                if len(inflight) > 4:
                    # The cameras are not keeping up; stop piling on requests.
                    self.events.emit("warn", "capture",
                                     "capture backlog: %d shots still in flight"
                                     % len(inflight))
                    inflight[0].join(timeout=period)
                k += 1
            for t in inflight:
                t.join(timeout=5)
            self.events.emit("info", "capture", "auto-capture loop ended",
                             fired=k)

        self._cap_thread = threading.Thread(target=_loop, daemon=True)
        self._cap_thread.start()
        self.events.emit("info", "capture", "auto-capture loop started",
                         interval_s=period, frames=count)

    def _stop_capture_loop(self):
        if self._cap_stop:
            self._cap_stop.set()

    # ---- data providers used by workers -----------------------------------
    def nav_snapshot(self, epoch):
        """Nav state at the frame's capture instant, not at 'now'.

        A frame can be pulled seconds after the shutter fired, so stamping it
        with the current position would put the boat where it has since moved
        to. fix_at() reaches back to the sample nearest that instant and has
        already computed UTM and the staleness/validity flags."""
        if not self.nav:
            return None
        try:
            return self.nav.fix_at(epoch)
        except Exception:  # noqa: BLE001
            return None

    def imu_snapshot(self, epoch):
        imu_mon = next((m for m in self.monitors if m.name_ == self.imu_node),
                       None)
        if not imu_mon or not imu_mon.is_connected():
            return None
        r = http_json("http://%s:8081/imu/window?t0=%.3f&t1=%.3f"
                      % (imu_mon.host, epoch - 0.5, epoch + 0.5), timeout=4)
        samples = r.get("samples") if isinstance(r, dict) else None
        if samples:
            # nearest sample to the capture instant
            return min(samples, key=lambda s: abs(s.get("epoch", epoch) - epoch))
        r = http_json("http://%s:8081/imu/latest" % imu_mon.host, timeout=3)
        return r if isinstance(r, dict) and r.get("epoch") else None

    def match_exposure_edge(self, mon, expected=None, window=0.20):
        """Oldest unconsumed GPIO EXPOSURE fall-edge epoch for this node.

        Edges are buffered per node and handed out one per frame, in order.
        Draining the whole cursor and returning the LAST edge - as this used to -
        gives frame 1 frame 3's timestamp whenever more than one frame arrives in
        a poll, and leaves frames 2 and 3 falling back to EXIF. That defeats the
        entire point of the harness, and it does so precisely when the rig is
        working hardest. Frames and exposures are both strictly ordered per
        camera, so FIFO is the correct pairing."""
        buf = getattr(mon, "_edge_buf", None)
        if buf is None:
            buf = mon._edge_buf = []
        r = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                      % (mon.host, getattr(mon, "_edge_cursor", 0)), timeout=3)
        if isinstance(r, dict):
            mon._edge_cursor = r.get("next", getattr(mon, "_edge_cursor", 0))
            buf.extend(e["epoch"] for e in r.get("events", [])
                       if e.get("edge") == "fall" and e.get("epoch") is not None)
        # piagent's ring holds an hour of edges. Anything older than this frame
        # could plausibly be is not ours - it belongs to a bench test or an
        # earlier run - and pairing a frame with a minutes-old edge silently
        # writes a wildly wrong capture time into the flight_log while still
        # claiming capture_source=gpio_edge. Drop stale entries rather than
        # hand one out.
        cutoff = time.time() - self.EDGE_MAX_AGE_S
        while buf and buf[0] < cutoff:
            buf.pop(0)
        if len(buf) > 64:
            del buf[:-64]
        if not buf:
            return None
        if expected is None:
            return buf.pop(0)
        # Nearest edge to the instant this frame was scheduled to expose. The
        # window is far wider than the ~1 ms sync we achieve and far narrower
        # than the 500 ms between frames at 2 fps, so it cannot pick a
        # neighbour's edge.
        i = min(range(len(buf)), key=lambda k: abs(buf[k] - expected))
        if abs(buf[i] - expected) > window:
            return None
        return buf.pop(i)

    def calibrate_trigger(self, samples=5, hold_focus=True):
        """Measure each camera's TRIGGER -> EXPOSURE latency, per node.

        Two bodies do not answer a trigger in the same time: measured 22.18 ms
        on cam1 and 22.44 ms on cam2, and a different lens or body can widen
        that considerably. Scheduling both for the same instant therefore still
        leaves them exposing at different instants. Firing each node at
        (target - its own latency) makes the EXPOSURES coincide, which is the
        thing stereo actually requires.

        Run at rigd start and again at run start, because the figure moves with
        lens, drive mode and body state."""
        live = [m for m in self.monitors if m.is_connected()
                and (m.health.get("gpio", {}) or {}).get("available")]
        if not live:
            return {}
        out = {}
        for m in live:
            if hold_focus:
                http_json("http://%s:8081/gpio/focus" % m.host, {"hold": True},
                          timeout=8)
            time.sleep(0.3)
            lat = []
            for _ in range(samples):
                r0 = http_json("http://%s:8081/gpio/exposure/events" % m.host,
                               timeout=4)
                cur = r0.get("next", 0) if isinstance(r0, dict) else 0
                r = http_json("http://%s:8081/gpio/fire" % m.host,
                              {"at_epoch": 0, "pulse_ms": 5}, timeout=10)
                if not r.get("ok"):
                    continue
                time.sleep(1.0)
                ev = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                               % (m.host, cur), timeout=4)
                falls = [e["epoch"] for e in (ev.get("events") or [])
                         if e.get("edge") == "fall"]
                if falls and r.get("actual_epoch"):
                    lat.append(falls[0] - r["actual_epoch"])
                time.sleep(0.2)
            if hold_focus:
                http_json("http://%s:8081/gpio/focus" % m.host, {"hold": False},
                          timeout=8)
            if lat:
                lat.sort()
                out[m.name_] = lat[len(lat) // 2]        # median, outlier-proof
        if out:
            self.trig_latency.update(out)
            self.events.emit("info", "calibrate",
                             "trigger latency: " +
                             ", ".join("%s=%.2fms" % (k, v * 1000)
                                       for k, v in sorted(out.items())),
                             latency_ms={k: round(v * 1000, 2)
                                         for k, v in out.items()})
        return out

    def _calibrate_then_reset(self):
        """Calibrate, then re-baseline. Calibration frames are real exposures on
        the camera; they must not be mistaken for survey frames, so both the
        edge cursors and each worker's 'already seen' set are refreshed after."""
        try:
            self.calibrate_trigger()
            for w in list(self.workers.values()):
                try:
                    for s in w.mon.shots():
                        w.seen.add(s["name"])
                except Exception:  # noqa: BLE001
                    pass
            self.reset_edge_cursors()
        except Exception as e:  # noqa: BLE001
            self.events.emit("warn", "calibrate", "calibration failed: %s" % e)

    def reset_edge_cursors(self):
        """Start each node's edge stream at 'now'.

        Without this a run inherits every edge piagent has buffered since it
        started - bench tests included - and hands them out as capture instants
        for the first frames of the survey."""
        for m in self.monitors:
            m._edge_buf = []
            r = http_json("http://%s:8081/gpio/exposure/events" % m.host, timeout=3)
            m._edge_cursor = r.get("next", 0) if isinstance(r, dict) else 0

    def _calibrate_exif(self, m):
        """One frame to learn camera_clock - true_clock for this body."""
        before = {s["name"] for s in m.shots()}
        t_cmd = time.time()
        m.shutter(af=False)
        for _ in range(200):
            time.sleep(0.05)
            new = [s for s in m.shots() if s["name"] not in before]
            if new:
                name = sorted(x["name"] for x in new)[0]
                data, err = http_bytes("http://%s:8080/shot/%s"
                                       % (m.host, name), timeout=30)
                if err or not data:
                    return
                cam = _exif_capture_epoch(data)
                if cam:
                    # true capture ~ command + release lag (20 ms) + curtain
                    true_epoch = t_cmd + 0.024
                    self.timesync.set_exif_offset(m.name_, cam - true_epoch)
                return

    # ---- run.json / index -------------------------------------------------
    def on_frame(self, node, cam_num, fname, orig, epoch):
        self.events.emit("debug", "frame", "%s <- %s" % (fname, orig), node=node)

    def index_frame(self, cam_num, fname, orig, epoch, source):
        with self._lock:
            if self.active:
                self.active["index"].append(
                    {"cam": cam_num, "file": fname, "orig": orig,
                     "epoch": round(epoch, 3), "src": source})
                if len(self.active["index"]) % 10 == 0:
                    self._write_run_json()

    def _write_run_json(self, final=False):
        run = self.active
        if not run:
            return
        doc = {"run_id": run["run_id"], "label": run["label"],
               "started": run["started"], "nodes": run["nodes"],
               "config": run["config"],
               "frames": len(run["index"]), "index": run["index"][-2000:],
               "stats": {n: w.stats() for n, w in self.workers.items()},
               "final": final, "written": time.time()}
        try:
            tmp = os.path.join(run["root"], "run.json.tmp")
            with open(tmp, "w") as fh:
                json.dump(doc, fh, indent=1)
            os.replace(tmp, os.path.join(run["root"], "run.json"))
        except OSError:
            pass

    def status(self):
        with self._lock:
            if not self.active:
                return {"active": False}
            return {"active": True, "run_id": self.active["run_id"],
                    "label": self.active["label"],
                    "started": self.active["started"],
                    "nodes": self.active["nodes"],
                    "frames": len(self.active["index"]),
                    "stats": {n: w.stats() for n, w in self.workers.items()}}


def _slug(s):
    return "".join(c if c.isalnum() or c in "-_" else "-"
                   for c in str(s))[:40] or "run"

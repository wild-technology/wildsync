"""run.py — a transect run: firing, pulling, renaming, and the flight log.

A "run" is one survey line. Starting it lays down a directory, wires up a pull
worker per online camera, and (if asked) drives a synchronized capture loop.
Each frame that lands is renamed to the rig convention, timestamped with its
best available capture instant, and written as one row of that camera's
flight_log alongside the nav + IMU state at that instant.

The design keeps the cameras independent: one camera stalling or dropping does
not stall the others. Every worker owns its own thread, its own baseline of
"frames already seen", and its own flight_log handle.

Two rules run through the whole file and are worth stating once:

  * TIMEBASE. Every epoch handled internally - fire commands, EXPOSURE edges,
    the nav ring, the IMU window - is a RAW local (Jetson) epoch, because those
    are all local-clock domains. The GPS correction is applied exactly once, at
    the presentation boundary (the `datetime` column and the CamN_ filename),
    from an offset latched at run start so every row of one transect sits on one
    clock. `time_source` names the offset actually applied to that row.

  * IDENTITY. A frame is paired with the fire command that produced it by an
    explicit claim taken when the frame is first listed, not by whatever happens
    to be at the head of a queue when it is written. One frame that fires but
    never lands used to shift every later row by one shot period while still
    claiming capture_source=gpio_edge.
"""

import csv
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
    """UTC YYMMDD_hhmmss.ss for the flight log datetime column.

    `epoch` must already be in the run's PRESENTATION timebase (local epoch plus
    the latched GPS offset). Nothing inside this file may feed a corrected epoch
    back into nav.fix_at(), /imu/window or edge matching - those are local-clock
    domains and a corrected stamp silently blanks their columns."""
    lt, frac = _split_epoch(epoch)
    return time.strftime("%y%m%d_%H%M%S", lt) + frac


def _fmt_fname(cam_num, epoch, ext=".jpg"):
    """CamN_YYYYMMDD_hhmmss.ss<ext>, from the presentation timebase.

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
    """One camera's frames, for the life of one run.

    LOCK ORDER, and it matters: a worker never calls back into RunManager while
    holding its own lock. RunManager does the reverse - it reads a worker's
    counters with the run lock held - and stop() joins the workers only after
    releasing it. Any other order deadlocks a stop against an in-flight frame.
    """

    # A failed pull used to lose that frame for good: the name went into `seen`
    # BEFORE the download was attempted and nothing ever looked at it again.
    # The bytes stay on the node (its save dir is never pruned) and on the card,
    # so retry, mark the name seen only once the bytes are on disk, and give up
    # loudly rather than silently.
    #
    # The horizon is deliberately SHORT. It exists to ride out the two faults
    # that clear on their own within a poll or two - a frame listed while
    # ilxctl is still writing it, and a momentary link stall - and each attempt
    # already carries its own 30 s transfer timeout. Anything that survives four
    # attempts is a condition an operator has to fix (a full disk, a read-only
    # run directory, a body that has lost PC-save), and quietly re-reading the
    # same frame for a minute would put the puller behind a survey that is still
    # firing. A node that is OFFLINE burns no attempts at all - _poll_once
    # returns early - so a camera power-cycled mid-run (which reboots its Pi on
    # the shared PoE feed) resumes its retries when it comes back.
    MAX_ATTEMPTS = 4
    BACKOFF_S = (0.25, 0.35, 0.45, 0.55)

    # A queued fire command whose frame has not been listed by now produced no
    # frame at all (the body refused the release, PC-save was off, the card was
    # full). Frames are listed within ~1 s of the shutter - measured USB save
    # latency 0.44-0.83 s - so 10 s is far beyond any legitimate lag, and
    # leaving such a command in the queue is what shifts every later frame by
    # one shot period.
    CMD_MAX_AGE_S = 10.0

    def __init__(self, mon, cam_dir, provider, adopted=False):
        super().__init__(daemon=True)
        self.mon = mon
        self.cam_num = mon.node["cam_num"]
        self.cam_dir = cam_dir
        self.provider = provider          # RunManager, for nav/imu/time/events
        self.adopted = adopted            # joined an already-running transect
        self._stopev = threading.Event()
        # Set once the first poll has been through: a camera adopted mid-run
        # must get to list (and claim) its backlog before anything starts firing
        # calibration frames at it.
        self.primed = threading.Event()
        self.seen = set()
        self.pulled = 0
        self.failed = 0                   # frames given up on entirely
        self.retries = 0
        self.skipped_cal = 0              # calibration exposures kept out
        self.orphans = 0                  # fires that produced no frame
        self._last_cap = None
        # Expected exposure instants, oldest first. A list, not a deque, because
        # note_command inserts in TARGET order: it is called from the per-node
        # fire threads, so a slow /gpio/fire could otherwise append shot k after
        # shot k+1 and swap two frames' timestamps, filenames, nav fixes and IMU
        # attitude with each other.
        self._cmds = []
        self.intervals = []               # capture-epoch deltas, for jitter
        self._flight_fh = None
        self._flight = None
        # The command each frame CLAIMED, keyed by the original camera filename.
        # The claim is taken when the frame is first listed and released when it
        # is written or abandoned, so a frame whose pull is failing cannot hand
        # its command to the next frame in line.
        self.cmd_epoch = {}
        # name -> {"size", "attempts", "next_try"} for frames not yet on disk.
        self._pending = {}
        # The last few frames this worker has actually dealt with (pulled,
        # discarded or given up on). Calibration uses it to tell "this frame was
        # already in the survey pipeline before my shutter" from "this is mine";
        # `seen` cannot answer that, because it also holds the run-start
        # baseline of everything that was on the camera before the run.
        self._recent = []
        self._plaus_strikes = 0
        self._plaus_off = False
        self._lock = threading.Lock()

    # ---- fire commands ----------------------------------------------------
    def note_command(self, epoch, path="gpio", fire_seq=None, edge_seq=None):
        """Queue an expected exposure instant and return its record.

        Inserted in target order rather than appended: this is called from one
        thread per node per shot, so arrival order is not shot order under any
        load worth worrying about."""
        rec = {"epoch": float(epoch), "path": path, "fire_seq": fire_seq,
               "edge_seq": edge_seq, "queued": time.time()}
        with self._lock:
            i = len(self._cmds)
            while i > 0 and self._cmds[i - 1]["epoch"] > rec["epoch"]:
                i -= 1
            self._cmds.insert(i, rec)
            if len(self._cmds) > 256:
                del self._cmds[:-256]
        return rec

    def update_command(self, rec, **fields):
        with self._lock:
            rec.update(fields)

    def drop_command(self, rec):
        """Un-queue a command whose fire did not happen.

        A fire that returned ok:false exposed nothing, so its command belongs to
        no frame; left queued it is claimed by the NEXT frame, which is then
        stamped one shot period early for the rest of the run."""
        with self._lock:
            for i, r in enumerate(self._cmds):
                if r is rec:
                    del self._cmds[i]
                    return True
        return False

    def _claim(self, name):
        """Take the oldest unclaimed command for this frame, by name."""
        with self._lock:
            if name in self.cmd_epoch:
                return self.cmd_epoch[name]
            if not self._cmds:
                return None
            rec = self._cmds.pop(0)
            self.cmd_epoch[name] = rec
            return rec

    def _release_claim(self, name, requeue):
        """Give a claim back (requeue=True) or drop it (an orphaned fire)."""
        with self._lock:
            rec = self.cmd_epoch.pop(name, None)
            if rec is not None and requeue:
                i = len(self._cmds)
                while i > 0 and self._cmds[i - 1]["epoch"] > rec["epoch"]:
                    i -= 1
                self._cmds.insert(i, rec)
            return rec

    def _expire_commands(self):
        """Drop commands no frame can still be coming for, and say so."""
        cut = time.time() - self.CMD_MAX_AGE_S
        dropped = []
        with self._lock:
            while self._cmds and self._cmds[0]["epoch"] < cut:
                dropped.append(self._cmds.pop(0))
            self.orphans += len(dropped)
            total = self.orphans
        if dropped:
            self.provider.note_orphan(self.mon.name_, len(dropped))
            # Every one is counted in run.json; the alert itself is rate-limited
            # so a camera that has stopped delivering does not bury the journal
            # under one warning every shot.
            if self.provider.warn_once("orphan:%s" % self.mon.name_, 20.0):
                self.provider.events.emit(
                    "warn", "orphan_fire",
                    "%d fire(s) on %s produced no frame (%d this run) - the "
                    "shutter was commanded and nothing arrived; check PC-save, "
                    "card space and PC-remote priority"
                    % (len(dropped), self.mon.name_, total),
                    node=self.mon.name_, orphans=total)

    # ---- lifecycle --------------------------------------------------------
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
        listing = {s["name"] for s in self.mon.shots()}
        known = self.provider.known_frames(self.mon.name_)
        backlog = ()
        if self.adopted and known is not None:
            # This camera missed the start of the transect but rigd has watched
            # its save dir before, so anything in it we have never listed is a
            # frame it took DURING this run - while it was booting, or in the
            # seconds between coming back and being adopted. Baselining those
            # away (which a blanket "everything here is old" does) drops real
            # survey frames on the floor with no row and no event. Frames we
            # have already accounted for stay accounted for.
            backlog = sorted(listing - set(known))
            self.seen |= (listing - set(backlog))
        else:
            # First sight of this camera in this session: its save dir is never
            # pruned, so everything in it predates us and is not survey data.
            self.seen |= listing
        self.provider.remember_frames(self.mon.name_, listing)
        self.provider.events.emit(
            "info", "pull", "worker up, baseline %d frames%s"
            % (len(self.seen),
               (", %d taken before adoption still to pull" % len(backlog))
               if backlog else ""), node=self.mon.name_)
        while True:
            try:
                self._poll_once()
            except Exception as e:  # noqa: BLE001
                self.provider.events.emit("error", "pull",
                                          "poll error: %s" % e, node=self.mon.name_)
            self.primed.set()
            if self._stopev.wait(0.4):
                break
        try:
            self._flight_fh.close()
        except OSError:
            pass

    def _poll_once(self):
        if not self.mon.is_connected():
            return
        self._expire_commands()
        shots = self.mon.shots()
        self.provider.remember_frames(self.mon.name_,
                                      [s["name"] for s in shots])
        # A calibration exposure is landing on this node right now and has not
        # been named yet. Hold off listing anything new for the second or two it
        # takes the calibration to identify its own frame: the alternative is a
        # race the survey loses either way - either a calibration frame is
        # written into the transect as survey data, or a real frame is guessed
        # at and thrown away. Deferring is lossless: nothing is marked seen, so
        # the next poll picks up whatever this one skipped.
        quiet = self.provider.calibration_quiet(self.mon.name_)
        # Take the claim in listing order, which is capture order: the camera
        # numbers frames sequentially. Claiming here rather than at write time
        # is what keeps a frame whose download is being retried from donating
        # its command to the frame behind it.
        for s in (() if quiet else sorted(shots, key=lambda s: s["name"])):
            name = s["name"]
            if name in self.seen or name in self._pending:
                continue
            self._pending[name] = {"size": s.get("size", 0), "attempts": 0,
                                   "next_try": 0.0}
            self._claim(name)
        now = time.time()
        for name in sorted(self._pending):
            p = self._pending.get(name)
            if p is None or p["next_try"] > now:
                continue
            if self._stopev.is_set():
                return
            if self._handle(name, p["size"]):
                self.seen.add(name)
                self._note_recent(name)
                self._pending.pop(name, None)
                self._release_claim(name, requeue=False)
                continue
            p["attempts"] += 1
            if p["attempts"] >= self.MAX_ATTEMPTS:
                self.seen.add(name)
                self._note_recent(name)
                self._pending.pop(name, None)
                self.failed += 1
                # The frame is gone from the survey, so its command belongs to
                # nobody: drop it rather than let the next frame inherit it.
                lost = self._release_claim(name, requeue=False)
                if lost is not None:
                    with self._lock:
                        self.orphans += 1
                    self.provider.note_orphan(self.mon.name_, 1)
                self.provider.events.emit(
                    "error", "pull_fail",
                    "%s abandoned after %d attempts - that frame is NOT in the "
                    "survey; the JPEG is still on the node and on the card"
                    % (name, p["attempts"]), node=self.mon.name_, frame=name)
            else:
                self.retries += 1
                p["next_try"] = time.time() + self.BACKOFF_S[
                    min(p["attempts"] - 1, len(self.BACKOFF_S) - 1)]

    def _note_recent(self, name):
        with self._lock:
            self._recent.append(name)
            del self._recent[:-512]

    def recent_names(self):
        with self._lock:
            return set(self._recent) | set(self._pending)

    # ---- one frame --------------------------------------------------------
    def _handle(self, name, size):
        """Pull, rename and log one frame. True = done with it (or discarded)."""
        attempts = self._pending.get(name, {}).get("attempts", 0)

        def fail(sev, msg):
            # The first failure is the operator-visible one and the give-up is
            # an error; the attempts in between are progress notes under their
            # own kind, so a `pull_fail` in the journal always means something
            # that needs looking at.
            self.provider.events.emit(
                *((sev, "pull_fail", msg) if attempts == 0
                  else ("debug", "pull_retry", "retry %d: %s"
                        % (attempts + 1, msg))),
                node=self.mon.name_, attempt=attempts + 1)
            return False

        # A calibration exposure is a real frame on the camera and NOT survey
        # data: it is shot from the stationary start of the line, belongs to no
        # stereo pair, and inflates the transect's frame count. Keep it out by
        # the name the calibration itself recorded, before paying for the
        # transfer. Only ever by NAME - guessing from the capture instant would
        # eventually throw away a real frame, which is the one outcome worse
        # than logging a calibration one.
        if self.provider.is_calibration_frame(self.mon.name_, name):
            self._discard_calibration(name)
            return True
        url = "http://%s:8080/shot/%s" % (self.mon.host, name)
        data, err = http_bytes(url, timeout=30)
        if err or not data:
            return fail("warn", "download %s failed: %s" % (name, err))
        # A link that dies mid-frame yields a short read with no exception, and a
        # half-written frame that still gets a flight_log row is worse than a
        # missing one: it looks like real survey data. Reject it here.
        if size and len(data) != size:
            return fail("error", "%s truncated: got %d of %d bytes"
                        % (name, len(data), size))
        ext = os.path.splitext(name)[1].lower() or ".jpg"
        if ext in (".jpg", ".jpeg") and not (data[:2] == b"\xff\xd8"
                                             and data[-2:] == b"\xff\xd9"):
            return fail("error", "%s is not a complete JPEG (bad SOI/EOI)" % name)
        # Best capture instant: (GPIO edge) > corrected EXIF > command epoch.
        source, epoch, terr = self._capture_instant(name, data)
        # The one place the GPS correction is applied: filename and datetime.
        off, tsource = self.provider.time_base()
        stamp = epoch + off
        fname, dest = _uniq_dest(self.cam_dir,
                                 _fmt_fname(self.cam_num, stamp, ext))
        try:
            with open(dest, "wb") as fh:
                fh.write(data)
        except OSError as e:
            return fail("error", "write %s failed: %s" % (fname, e))
        self.pulled += 1
        with self._lock:
            if self._last_cap is not None:
                self.intervals.append(epoch - self._last_cap)
                if len(self.intervals) > 500:
                    self.intervals.pop(0)
            self._last_cap = epoch
        self._write_flight(fname, name, epoch, stamp, source, terr, tsource)
        self.provider.on_frame(self.mon.name_, self.cam_num, fname, name, epoch)
        return True

    def _discard_calibration(self, name):
        self.skipped_cal += 1
        # If it managed to claim a scheduled command before being recognised,
        # hand that command BACK: it belongs to a survey frame still to come.
        self._release_claim(name, requeue=True)
        self.provider.events.emit(
            "debug", "pull", "%s is a calibration exposure - not survey data, "
            "kept out of the transect" % name, node=self.mon.name_)

    def _capture_instant(self, name, data):
        """(source, LOCAL epoch, error_s) for this frame.

        Preference order is the contract's: GPIO EXPOSURE edge > corrected EXIF
        > the command epoch. What changed is how the frame finds its command."""
        cam_epoch = _exif_capture_epoch(data)
        corrected = self.provider.timesync.correct_exif(self.mon.name_, cam_epoch)
        cmd = self.cmd_epoch.get(name) or self._claim(name)
        # A claim this frame cannot own is worse than no claim: it hands the
        # frame the previous shot's edge, one shot period early, still labelled
        # capture_source=gpio_edge with time_err_ms=0. EXIF is coarse (whole
        # seconds plus SubSec) but it is INDEPENDENT of the queue, so it can
        # tell the two directions apart:
        #   command much OLDER than the frame  -> that fire's frame never landed
        #   command much NEWER than the frame  -> this frame was never scheduled
        #                                         (a body shutter press, the
        #                                         camera's own Interval REC)
        for _ in range(4):
            if cmd is None or corrected is None or self._plaus_off:
                break
            tol = self._claim_tolerance()
            delta = cmd["epoch"] - corrected
            if delta > tol:
                self._release_claim(name, requeue=True)
                self.provider.events.emit(
                    "debug", "capture",
                    "%s exposed %.1fs before the oldest queued fire - treating "
                    "it as an unscheduled frame" % (name, delta),
                    node=self.mon.name_)
                cmd = None
                break
            if delta < -tol:
                if not self._orphan_claim(name, cmd, delta):
                    break
                cmd = self._claim(name)
                continue
            self._plaus_strikes = 0
            break
        if cmd is not None:
            edge = self.provider.match_exposure_edge(
                self.mon, expected=cmd["epoch"], fire_seq=cmd.get("fire_seq"),
                after_seq=cmd.get("edge_seq"))
            if edge is not None:
                return "gpio_edge", edge, 0.0
        else:
            # No fire command is queued for this frame, but the harness may still
            # have caught its EXPOSURE edge. That happens for every frame the
            # Jetson did not itself schedule: the camera's own Interval REC,
            # piagent's node-side /gpio/interval, a shot taken from the body, and
            # every frame a node produced before it was adopted into a run that
            # had already started. Falling straight through to EXIF there throws
            # away a kernel-timestamped edge in favour of the camera's own coarse
            # clock (whole-second DateTimeOriginal plus a calibrated offset) -
            # which is precisely the accuracy the harness exists to provide.
            # The window here used to be 1.0 s, which is wide enough for a frame
            # to adopt a NEIGHBOUR'S exposure - a calibration fire, or the shot
            # before it at 2 fps - and then be written as capture_source=
            # gpio_edge at that instant, with nav and IMU looked up there too
            # (and, past the IMU's 100 ms window, no attitude at all). EXIF with
            # SubSec is good to ~10 ms, so a tight window costs nothing when the
            # camera gives sub-second precision, and when it does not the honest
            # answer is capture_source=exif rather than a confidently wrong edge.
            edge = (self.provider.match_exposure_edge(
                        self.mon, expected=corrected, window=0.20)
                    if corrected else
                    self.provider.match_exposure_edge(self.mon, expected=None))
            if edge is not None:
                return "gpio_edge", edge, 0.0
        if corrected:
            return "exif", corrected, self.provider.exif_err(self.mon.name_)
        if cmd is not None:
            # The command instant itself. How far that can be from the true
            # exposure depends on how it was fired: a scheduled GPIO shot lands
            # within a few ms of its target (measured 0.59 ms mean skew, 1.8 ms
            # worst), a USB release anywhere in PROTOCOL.md's 0-200 ms.
            return "command", cmd["epoch"], (0.2 if cmd["path"] == "usb"
                                             else 0.025)
        # Card-review pull: no command, no edge, no EXIF. RAW local time, so it
        # stays in the same domain as every other epoch in this file.
        return "command", time.time(), None

    def _claim_tolerance(self):
        """How far a frame's EXIF may sit from its claimed fire command.

        Wide enough to absorb the camera clock's own quantisation (whole-second
        DateTimeOriginal when SubSec is absent) plus the ~25 ms EXIF-offset
        calibration error, narrow enough to catch the one-shot-period shift that
        a dropped frame causes."""
        period = self.provider.shot_period()
        return max(1.5, 0.6 * period)

    def _orphan_claim(self, name, cmd, delta):
        """Discard a claimed command whose own frame never arrived."""
        self._plaus_strikes += 1
        if self._plaus_strikes > 5:
            self._plaus_off = True
            # Five in a row means the cross-check itself is wrong - almost
            # always an EXIF offset measured against the wrong frame - and
            # eating the whole command queue on the strength of it would be
            # worse than the off-by-one it defends against.
            self.provider.events.emit(
                "error", "capture",
                "EXIF disagrees with every queued fire on %s (last %+.1fs) - "
                "the camera-clock offset is not trustworthy; command/frame "
                "cross-checking is off for the rest of this run"
                % (self.mon.name_, delta), node=self.mon.name_)
            return False
        self._release_claim(name, requeue=False)
        with self._lock:
            self.orphans += 1
            total = self.orphans
        self.provider.note_orphan(self.mon.name_, 1)
        if self.provider.warn_once("orphan:%s" % self.mon.name_, 20.0):
            self.provider.events.emit(
                "warn", "orphan_fire",
                "a fire on %s produced no frame (%s exposed %.1fs after it; %d "
                "orphaned this run) - dropping it so later frames keep their own "
                "capture instants" % (self.mon.name_, name, -delta, total),
                node=self.mon.name_, orphans=total)
        return True

    def _write_flight(self, fname, orig, epoch, stamp, source, terr, tsource):
        # nav and IMU are looked up with the RAW local epoch: both rings are
        # keyed by local receive time, so handing them a GPS-corrected stamp
        # blanks the columns as soon as the offset exceeds their staleness
        # windows (3 s for nav, 100 ms for the IMU).
        nav = self.provider.nav_snapshot(epoch)
        imu = self.provider.imu_snapshot(epoch)
        row = {k: "" for k in FLIGHT_HEADER}
        row.update({
            "filename": fname,
            "datetime": _fmt_dt(stamp),
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
        claim = self.cmd_epoch.get(orig)
        self.provider.index_frame(self.cam_num, fname, orig, epoch, source,
                                  node=self.mon.name_,
                                  path=(claim or {}).get("path"))

    def stats(self):
        with self._lock:
            iv = list(self.intervals)
            queued = len(self._cmds)
        import statistics as st
        jit = (st.pstdev(iv) * 1000) if len(iv) > 1 else None
        return {"pulled": self.pulled, "failed": self.failed,
                "retrying": len(self._pending), "retries": self.retries,
                "skipped_calibration": self.skipped_cal,
                "orphan_fires": self.orphans, "queued_commands": queued,
                "last_capture": self._last_cap,
                "interval_mean_s": round(st.mean(iv), 3) if iv else None,
                "interval_jitter_ms": round(jit, 1) if jit else None}


def _r(v, n):
    return "" if v is None else round(v, n)


class RunManager:
    def __init__(self, monitors, settings, timesync, events, nav, imu_node=None):
        self.monitors = monitors
        self.settings = settings
        self.timesync = timesync
        self.events = events
        self.nav = nav
        # Which node carries the IMU is DISCOVERED, not assumed. This used to
        # default to "cam3" - a slot that has no hardware and reports OFFLINE -
        # so imu_snapshot() never found a connected node and every IMU column of
        # every flight_log came out empty, silently, on a rig whose IMU was
        # sitting on cam1 sampling happily at 200 Hz. A pinned name is also
        # wrong on its own terms: the IMU is a USB device that can be moved to
        # another Pi without a code change. `imu_node` is now only an optional
        # override for tests; None means "ask the fleet".
        self.imu_node = imu_node
        self._imu_node_cache = None
        self._lock = threading.Lock()
        self.active = None            # dict describing the current run
        self.workers = {}
        # node -> TRIGGER->EXPOSURE latency in seconds, measured not assumed.
        self.trig_latency = {}
        self.trig_measured_at = {}
        # node -> uncertainty of that node's EXIF clock offset, in seconds.
        self.exif_uncertainty = {}
        self._cap_thread = None
        self._cap_stop = None
        self._calib_stop = None
        self._calib_thread = None
        self._adopt_stop = None
        self._adopt_thread = None
        # Calibration exposures, so the pull workers can keep them out of the
        # survey: exact camera filenames where we learned them, plus the fire
        # instants as a net for the ones the listing raced.
        self._cal_names = {}
        self._cal_busy = {}
        self._known_frames = {}
        # Sticky per-node GPIO capability and the rate limiters for the alerts
        # that would otherwise fire once per shot at 2 fps.
        self._gpio_ok = {}
        self._warned_at = {}

    # ---- timebase ---------------------------------------------------------
    def _live_time_base(self):
        """(gps_offset, source) as TimeSync sees it right now.

        Derived from one now() call so the offset and the label it is reported
        under can never disagree: reading .gps_offset and .source separately can
        straddle a fix arriving and produce a row stamped 'gps' with no
        correction applied, which is exactly the defect this replaces."""
        t0 = time.time()
        try:
            gnow, src = self.timesync.now()
        except Exception:  # noqa: BLE001
            return 0.0, "jetson"
        return gnow - t0, src

    def time_base(self):
        """The (offset, source) THIS transect stamps every row with.

        Latched at run start. A fix arriving - or dropping - mid-run must not
        move the clock under a survey that is already half written: rows before
        and after would then be minutes apart in the CSV and silently
        incomparable. The live offset is watched, reported, and recorded in
        run.json so a whole run can be shifted in post if it matters."""
        run = self.active
        if run:
            return run["time_off"], run["time_src"]
        return self._live_time_base()

    def exif_err(self, node):
        return self.exif_uncertainty.get(node)

    # ---- what we have already seen on each camera --------------------------
    # Every frame name any worker has ever listed, per node, for the life of
    # this rigd. It is what lets an adopted worker tell "this was already on the
    # card before we ever saw this camera" from "this was taken during the run,
    # while the camera was away". A dict is used as an ordered set so the oldest
    # entries can be trimmed; 10k names is a few hundred KB and many sessions.
    KNOWN_FRAMES_MAX = 10000

    def known_frames(self, node):
        with self._lock:
            d = self._known_frames.get(node)
            return dict(d) if d is not None else None

    def remember_frames(self, node, names):
        with self._lock:
            d = self._known_frames.setdefault(node, {})
            for n in names:
                d[n] = None
            over = len(d) - self.KNOWN_FRAMES_MAX
            if over > 0:
                for k in list(d)[:over]:
                    del d[k]

    def shot_period(self):
        """Nominal seconds between scheduled shots, or 0 when not auto-capturing."""
        run = self.active
        cfg = (run or {}).get("config") or {}
        if not cfg.get("auto_capture"):
            return 0.0
        try:
            return float(cfg.get("interval_s", 0) or 0)
        except (TypeError, ValueError):
            return 0.0

    # ---- lifecycle --------------------------------------------------------
    def start(self, config):
        with self._lock:
            if self.active:
                return {"ok": False, "error": "run already active",
                        "run_id": self.active["run_id"]}
            # Latch the timebase for the whole run, before anything is named.
            time_off, time_src = self._live_time_base()
            now = time.time() + time_off
            label = config.get("label", "transect")
            rid, root = self._new_run_root(now, label)
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
                   "time_off": time_off, "time_src": time_src,
                   "nodes": [m.name_ for m in live],
                   "events_fh": events_fh, "nmea_fh": nmea_fh,
                   "index": [], "alerts": [], "fired": {}, "orphans": {},
                   "skews_ms": [], "late_shots": 0, "failed_fires": 0}
            self.active = run
            self.workers = {}
            for m in live:
                cam_dir = os.path.join(root, m.name_)
                w = PullWorker(m, cam_dir, self)
                w.start()
                self.workers[m.name_] = w
            self._write_run_json()
            # Begin this run's edge stream at "now" so no bench-test edge is
            # inherited as a survey frame's capture instant.
            self.reset_edge_cursors()
            self.events.emit("info", "run", "run started: %d cameras" % len(live),
                             nodes=run["nodes"], run_id=rid,
                             time_source=time_src,
                             gps_offset_s=round(time_off, 3))
            # Calibration and the survey capture loop are NOT concurrent. Run
            # start fires ~6 non-survey exposures per camera (one for the EXIF
            # clock offset, five for trigger latency); with the capture loop
            # already running those interleave with real survey frames, steal
            # their queued commands and their EXPOSURE edges, and the bulk
            # re-baseline that used to follow marked whatever had landed in the
            # meantime as "already seen" - deleting real survey frames from the
            # transect. One ordered pass, then the loop.
            self._cap_stop = threading.Event()
            self._calib_stop = threading.Event()
            self._calib_thread = threading.Thread(
                target=self._calibrate_and_arm, args=(live, config), daemon=True)
            self._calib_thread.start()
            # Keep watching for cameras that were not up at start.
            self._adopt_stop = threading.Event()
            self._adopt_thread = threading.Thread(target=self._adopt_loop,
                                                  daemon=True)
            self._adopt_thread.start()
            return {"ok": True, "run_id": rid, "nodes": run["nodes"],
                    "root": root, "time_source": time_src,
                    "gps_offset_s": round(time_off, 3)}

    def _new_run_root(self, now, label):
        """A run_id whose directory does not already exist.

        Minute resolution is not unique, and the realistic collision is a false
        start: Start, notice cam2 has not converged, Stop, fix it, Start again
        inside the same minute. Both transects then shared one directory, their
        frames and flight_log rows interleaved under one header, and - worse -
        _write_run_json() at the second start atomically replaced the FIRST
        run's run.json, destroying the only copy of its per-frame index and its
        renamed->original filename mapping. Never reuse an existing root."""
        base = time.strftime("%y%m%d_%H%M", time.gmtime(now)) + "_" + _slug(label)
        os.makedirs(RUNS_DIR, exist_ok=True)
        for n in range(0, 27):
            rid = base if n == 0 else "%s_%s" % (base, chr(ord("a") + n))
            root = os.path.join(RUNS_DIR, rid)
            try:
                os.mkdir(root)
            except FileExistsError:
                continue
            if n:
                self.events.emit(
                    "warn", "run",
                    "a run directory named %s already exists - this transect is "
                    "%s so the earlier run's frames, flight_log and run.json "
                    "stay intact" % (base, rid), run_id=rid)
            return rid, root
        rid = "%s_%d" % (base, int(time.time()))
        root = os.path.join(RUNS_DIR, rid)
        os.makedirs(root, exist_ok=True)
        return rid, root

    def _adopt_loop(self):
        """Run housekeeping, every 3 s: adopt late cameras, watch the accounting.

        The worker set used to be fixed at run start. A camera still booting
        when the transect began - or power-cycled long enough to miss it, which
        on this rig happens every time a body is power-cycled, because the PoE
        feed reboots its Pi too - got no PullWorker for the whole run. It kept
        firing and its frames kept landing on the node, but none were pulled,
        renamed, or written to a flight_log: an entire camera silently missing
        from the survey, with run.json under-reporting the fleet as well."""
        while not self._adopt_stop.wait(3.0):
            with self._lock:
                run = self.active
                if not run:
                    return
                root = run["root"]
                missing = [m for m in self.monitors
                           if m.is_connected() and m.name_ not in self.workers]
                for m in missing:
                    w = PullWorker(m, os.path.join(root, m.name_), self,
                                   adopted=True)
                    w.start()
                    self.workers[m.name_] = w
                    if m.name_ not in run["nodes"]:
                        run["nodes"].append(m.name_)
                    m._edge_buf = []
                    self.events.emit(
                        "warn", "run",
                        "%s joined the run late - now being pulled and logged; "
                        "any frames it took before this are NOT in the survey"
                        % m.name_, node=m.name_)
            if missing:
                for m in missing:
                    threading.Thread(target=self._adopt_calibrate, args=(m,),
                                     daemon=True).start()
                self._write_run_json()
            try:
                self._watch_timebase()
                self._watch_frame_accounting()
            except Exception as e:  # noqa: BLE001
                self.events.emit("error", "run", "housekeeping error: %s" % e)

    def _watch_timebase(self):
        """Say it out loud if the master clock moves under an open transect."""
        run = self.active
        if not run:
            return
        off, src = self._live_time_base()
        if src == run["time_src"] and abs(off - run["time_off"]) <= 0.05:
            return
        if not self.warn_once("timebase", 0):
            return
        self.events.emit(
            "warn", "timebase",
            "master clock changed mid-run (%s %+.3fs -> %s %+.3fs). This "
            "transect keeps its start-of-run base so every row stays "
            "comparable; run.json records both, so the whole run can be "
            "shifted in post if the GPS base is wanted"
            % (run["time_src"], run["time_off"], src, off),
            run_time_source=run["time_src"],
            run_offset_s=round(run["time_off"], 3),
            live_time_source=src, live_offset_s=round(off, 3))

    def _watch_frame_accounting(self):
        """Fires issued vs frames landed, per camera.

        A body that stays CAM_CONNECTED and settings-converged but stops
        delivering - PC-save or PC-remote priority lost, card full, shutter
        refused - produced no alert of any kind: the run kept "firing", the
        flight_log simply stopped growing."""
        with self._lock:
            run = self.active
            if not run:
                return
            pairs = [(n, run["fired"].get(n, 0), w) for n, w in
                     self.workers.items()]
        for node, fired, w in pairs:
            if fired < 5:
                continue
            inflight = len(w._pending) + 2      # pull lag is ~1 s at 0.4 s polls
            missing = fired - w.pulled - w.skipped_cal - inflight
            if missing < 3:
                continue
            if not self.warn_once("frames_missing:%s" % node, 30.0):
                continue
            self.events.emit(
                "warn", "frames_missing",
                "%s: %d shots fired, %d frames landed - %d never arrived. Check "
                "PC-save and PC control priority on the body, card space, and "
                "the USB link" % (node, fired, w.pulled, missing),
                node=node, fired=fired, pulled=w.pulled, missing=missing)

    def warn_once(self, key, every_s):
        """Rate limiter: an alert repeated per shot at 2 fps is not an alert."""
        now = time.time()
        with self._lock:
            last = self._warned_at.get(key)
            if last is not None and (every_s <= 0 or now - last < every_s):
                return False
            self._warned_at[key] = now
        return True

    def stop(self):
        # Signal under the lock, then WAIT outside it. The workers call back
        # into index_frame()/on_frame(), which take this same lock, so joining
        # while holding it deadlocks every in-flight frame against the stop that
        # is trying to wait for it. Signal -> release -> join -> finalise.
        with self._lock:
            if not self.active:
                return {"ok": False, "error": "no active run"}
            self._stop_capture_loop()
            if self._calib_stop:
                # A stopped run must not keep firing calibration frames into a
                # closed transect, and a calibration polling a node that has
                # gone slow must not outlive the run that started it.
                self._calib_stop.set()
            self._calib_stop = None
            if self._adopt_stop:
                self._adopt_stop.set()
            workers = dict(self.workers)
            for w in workers.values():
                w.stop()
        # Actually WAIT for the pull threads to finish their current frame. A
        # bare sleep(0.6) is not a barrier: a worker mid-download when the
        # operator hit stop would keep running, append a row to a flight_log
        # that had already been summarised, and land a frame in a run directory
        # whose run.json was written and marked final - a transect on disk whose
        # own manifest disagrees with its contents.
        for w in workers.values():
            w.join(timeout=8.0)
        stuck = [n for n, w in workers.items() if w.is_alive()]
        if stuck:
            self.events.emit("warn", "run",
                             "pull worker(s) still finishing at stop: %s"
                             % ",".join(stuck), nodes=stuck)
        with self._lock:
            if not self.active:
                return {"ok": False, "error": "no active run"}
            self._write_run_json(final=True)
            run = self.active
            rid = run["run_id"]
            summary = {n: w.stats() for n, w in workers.items()}
            self.events.emit("info", "run", "run stopped: %s" % rid,
                             summary=summary,
                             time_source=run["time_src"],
                             gps_offset_s=round(run["time_off"], 3))
            # Detach the handles BEFORE closing them. The other order leaves a
            # window in which a concurrent emit() writes to a closed file, and
            # EventLog.emit only guards OSError - a ValueError there kills the
            # emitting thread, and if that thread is a NodeMonitor the fleet
            # silently stops being watched for the rest of the session.
            if self.nav:
                self.nav.set_raw_hook(None)
            self.events.set_run_file(None)
            try:
                run["events_fh"].close(); run["nmea_fh"].close()
            except OSError:
                pass
            self.active = None
            self.workers = {}
            return {"ok": True, "run_id": rid, "summary": summary}

    # ---- capture ----------------------------------------------------------
    # How far ahead the shared fire instant is scheduled. It must cover the HTTP
    # round trip to the slowest node, because every node has to receive the
    # request *before* the instant arrives or it fires late.
    SYNC_LEAD_S = 0.15
    # An EXPOSURE edge older than this cannot belong to a frame arriving now.
    EDGE_MAX_AGE_S = 30.0
    # A node reporting it fired further than this from its target is not merely
    # jittery: the whole point of handing every node one absolute instant is
    # that each busy-waits to it locally. Measured lateness is +0.1-0.3 ms mean.
    LATE_BUDGET_MS = 2.0
    # The product spec: every image within 10 ms of its pair. Realised skew is
    # computed per shot from what the nodes report, not assumed from the plan.
    SKEW_BUDGET_MS = 10.0
    # FOCUS is asserted this far ahead of each TRIGGER. The camera will not
    # accept a trigger without it, but holding it down for the whole run
    # half-presses the body and AE-locks it, which would freeze auto-ISO at the
    # light level the run started in. Per-shot keeps metering live.
    #
    # The VALUE of the lead is the single biggest lever left on sync accuracy,
    # because everything host-side is already sub-millisecond (measured fire
    # lateness +0.1-0.3 ms mean, chrony holds both Pis to the Jetson at ~85 us
    # RMS). All remaining skew is inside the bodies, and it is a strong function
    # of how long FOCUS has been asserted when TRIGGER lands. Measured on the
    # live pair, 25-30 scheduled pairs per point, skew from kernel EXPOSURE-edge
    # timestamps:
    #
    #     lead     per-body latency sd    mean |skew|    worst |skew|
    #      40 ms        2.4 ms              3.14 ms        7.22 ms
    #      80 ms        0.5-0.7 ms          0.59 ms        2.10 ms
    #     120 ms        0.6-0.8 ms          0.59 ms        1.82 ms
    #     200 ms        0.6-0.7 ms          0.68 ms        1.91 ms
    #     250 ms        0.9 ms              1.83 ms        4.27 ms
    #     400-800 ms    0.6-0.9 ms          1.45-1.59 ms   3.3-3.7 ms
    #
    # 40 ms - the previous value - does not give the body time to settle before
    # the trigger arrives, and cost 5x the skew for nothing. The 80-200 ms
    # plateau is flat, so 120 ms sits mid-band with margin either side; past
    # ~250 ms the body appears to re-meter and the jitter comes back. Lengthening
    # the lead is safe here because the rig is always MF (a long half-press
    # cannot provoke an AF hunt), and it stays per-shot so AE is never locked for
    # the run. If skew ever regresses, re-measure this curve before suspecting
    # the scheduler.
    FOCUS_LEAD_MS = 120

    def _fire_path(self, m):
        """"gpio" or "usb" for this shot, with hysteresis.

        rigcore blanks a node's whole health dict on ONE failed /health poll,
        and OFFLINE needs BOTH ilxctl and piagent to be unreachable - so a
        single 4 s timeout used to demote a camera from the GPIO path (22.1 ms,
        sd 0.4-0.9 ms) to the USB path (68-76 ms, sd 5-11 ms) with no alert:
        ~50 ms of inter-camera skew, every affected pair over the 10 ms spec.
        A blank health dict means UNKNOWN, not "no harness"."""
        health = m.health or {}
        gpio = health.get("gpio")
        if isinstance(gpio, dict):
            avail = bool(gpio.get("available"))
            was = self._gpio_ok.get(m.name_)
            self._gpio_ok[m.name_] = avail
            if was and not avail and self.warn_once("usb_path:%s" % m.name_, 30):
                self.events.emit(
                    "warn", "sync_degraded",
                    "%s has lost its GPIO harness and is now firing over USB - "
                    "0-200 ms inter-camera skew, well outside the 10 ms spec. "
                    "Frames taken now are marked path=usb in run.json" % m.name_,
                    node=m.name_)
            return "gpio" if avail else "usb"
        if self._gpio_ok.get(m.name_):
            if self.warn_once("health_blank:%s" % m.name_, 30):
                self.events.emit(
                    "warn", "sync_degraded",
                    "%s did not answer /health - keeping it on the GPIO path it "
                    "was last known to have, rather than silently dropping it to "
                    "unsynchronised USB firing" % m.name_, node=m.name_)
            return "gpio"
        return "usb"

    def _trigger_lead(self, node):
        """This node's measured TRIGGER->EXPOSURE latency, never a silent zero.

        Defaulting to 0.0 makes a partially-calibrated fleet WORSE than an
        uncalibrated one: the calibrated body is fired 22.2 ms early and its
        partner is not, so the pair exposes ~22 ms apart instead of the 0.26 ms
        body-to-body spread you get by compensating nobody."""
        lat = self.trig_latency.get(node)
        if lat is not None:
            return lat
        known = sorted(self.trig_latency.values())
        if known:
            med = known[len(known) // 2]
            if self.warn_once("no_latency:%s" % node, 60):
                self.events.emit(
                    "error", "calibration_missing",
                    "%s has no trigger-latency measurement of its own - firing "
                    "it on the fleet median (%.1f ms). Bodies measured 22.18 vs "
                    "22.44 ms, so expect a systematic skew until it calibrates"
                    % (node, med * 1000), node=node,
                    fleet_median_ms=round(med * 1000, 2))
            return med
        if self.warn_once("no_latency_any:%s" % node, 60):
            self.events.emit(
                "error", "calibration_missing",
                "no trigger-latency measured for any camera - firing "
                "uncompensated; the pair will expose up to the body-to-body "
                "spread (~22 ms) apart", node=node)
        return 0.0

    def capture_once(self, af=False, target=None):
        """Fire every connected camera at ONE shared absolute instant.

        Dispatching in parallel and telling each node "fire now" only makes the
        requests concurrent - each camera still fires whenever its own request
        lands, which measured 10-80 ms apart across two nodes. Handing every
        node the same target epoch instead lets each piagent busy-wait to it
        locally, off a clock disciplined to the Jetson: measured 0.65 ms mean
        inter-camera skew, worst 3.10 ms. For a stereo pair at 1 m/s that is the
        difference between ~30 mm and ~0.7 mm of baseline error.

        The outcome of every fire is recorded and judged. `ok`, `late_ms` and
        `actual_epoch` used to be thrown away and this returned ok:True come
        what may, so a camera that stopped firing altogether raised nothing."""
        live = [m for m in self.monitors if m.is_connected()]
        if not live:
            return {"ok": False, "error": "no cameras connected"}
        results = {}
        threads = []
        if target is None:
            target = time.time() + self.SYNC_LEAD_S

        def _fire(m):
            w = self.workers.get(m.name_)
            path = self._fire_path(m)
            if path == "gpio":
                # Fire this node early by its own measured trigger latency so
                # the EXPOSURES coincide, not the pulses. piagent asserts FOCUS
                # itself, FOCUS_LEAD_MS ahead, so no run needs to hold it.
                lead = self._trigger_lead(m.name_)
                # Queue the expected EXPOSURE instant BEFORE dispatching, not
                # from the completion path: these threads run one per node per
                # shot, so queueing on completion let a slow /gpio/fire land
                # shot k+1's command ahead of shot k's and swapped two frames'
                # timestamps, filenames, nav fixes and IMU attitudes.
                # It is the EXPOSURE instant, not the trigger instant: each node
                # is fired early by its own latency, so that is `target` for
                # every camera - the whole point of the compensation.
                rec = w.note_command(target, path="gpio") if w else None
                r = http_json("http://%s:8081/gpio/fire" % m.host,
                              {"at_epoch": target - lead, "pulse_ms": 5,
                               "focus_lead_ms": self.FOCUS_LEAD_MS},
                              timeout=10 + self.SYNC_LEAD_S)
                ok = bool(r.get("ok"))
                actual = r.get("actual_epoch")
                if ok and w and rec is not None:
                    # piagent hands back the identity of this fire and the edge
                    # cursor it started from, so the frame can be paired with
                    # ITS OWN exposure edge instead of the nearest one.
                    w.update_command(rec, fire_seq=r.get("fire_seq"),
                                     edge_seq=r.get("edge_seq"))
                elif w and rec is not None:
                    # It exposed nothing, so no frame will ever claim this.
                    w.drop_command(rec)
                results[m.name_] = {
                    "path": "gpio", "ok": ok,
                    "lead_ms": round(lead * 1000, 2),
                    "late_ms": r.get("late_ms"), "actual_epoch": actual,
                    "exposure_epoch": (actual + lead) if (ok and actual) else None,
                    "error": r.get("error")}
            else:
                # No harness on this node: it cannot be scheduled, so it fires on
                # arrival and its frames carry the usual USB uncertainty.
                t = time.time()
                rec = w.note_command(t, path="usb") if w else None
                r = m.shutter(af=af)
                ok = r.get("ok", True) is not False
                if not ok and w and rec is not None:
                    w.drop_command(rec)
                # No exposure_epoch: a USB release exposes somewhere in
                # PROTOCOL.md's 0-200 ms after the command and nothing reports
                # WHEN. Publishing the dispatch instant as if it were the
                # exposure would put a made-up number into the run's sync
                # record; the frame's own EXPOSURE edge is where that comes
                # from, if the harness caught one.
                results[m.name_] = {"path": "usb", "ok": ok,
                                    "error": r.get("error")}

        for m in live:
            th = threading.Thread(target=_fire, args=(m,), daemon=True)
            th.start(); threads.append(th)
        for th in threads:
            th.join(timeout=35)
        return self._judge_shot(target, results)

    def _judge_shot(self, target, results):
        """Score one shot: who fired, how late, and how far apart they exposed."""
        failed = {k: v.get("error") or "no response"
                  for k, v in results.items() if not v.get("ok")}
        fired = [k for k, v in results.items() if v.get("ok")]
        # Only nodes that reported an exposure instant (the GPIO path) are
        # compared. A shot that mixes paths yields no skew figure rather than a
        # guessed one - the USB camera's degradation is reported as
        # sync_degraded, which is what it is.
        exps = {k: v["exposure_epoch"] for k, v in results.items()
                if v.get("exposure_epoch")}
        skew_ms = None
        if len(exps) > 1:
            skew_ms = round((max(exps.values()) - min(exps.values())) * 1000, 2)
        lates = {k: v["late_ms"] for k, v in results.items()
                 if v.get("late_ms") is not None}
        worst_late = max((abs(x) for x in lates.values()), default=None)
        with self._lock:
            run = self.active
            if run:
                for n in fired:
                    run["fired"][n] = run["fired"].get(n, 0) + 1
                if skew_ms is not None:
                    run["skews_ms"].append(skew_ms)
                    if len(run["skews_ms"]) > 1000:
                        del run["skews_ms"][:-1000]
                if failed:
                    run["failed_fires"] += len(failed)
                if worst_late is not None and worst_late > self.LATE_BUDGET_MS:
                    run["late_shots"] += 1
        if failed and self.warn_once("capture_fail:%s" % ",".join(sorted(failed)),
                                     10.0):
            # Every failure is counted in run.json; the alert is rate-limited so
            # a camera that has stopped firing at 2 fps does not push everything
            # else out of the journal.
            self.events.emit(
                "error", "capture_fail",
                "%s did not fire: %s (%d failed fires this run)"
                % (",".join(sorted(failed)),
                   "; ".join("%s=%s" % kv for kv in sorted(failed.items())),
                   (self.active or {}).get("failed_fires", len(failed))),
                nodes=sorted(failed), results=failed)
        if skew_ms is not None and skew_ms > self.SKEW_BUDGET_MS \
                and self.warn_once("skew", 10.0):
            self.events.emit(
                "warn", "sync_skew",
                "cameras exposed %.1f ms apart, over the %.0f ms spec"
                % (skew_ms, self.SKEW_BUDGET_MS), skew_ms=skew_ms,
                exposures={k: round(v, 4) for k, v in exps.items()})
        if worst_late is not None and worst_late > self.LATE_BUDGET_MS \
                and self.warn_once("late", 10.0):
            self.events.emit(
                "warn", "gpio_late",
                "a node fired %.1f ms off its scheduled instant (budget %.0f ms)"
                % (worst_late, self.LATE_BUDGET_MS), late_ms=lates)
        self.events.emit("info", "capture", "capture fired",
                         results={k: v.get("path") for k, v in results.items()},
                         skew_ms=skew_ms)
        return {"ok": not failed, "results": results, "skew_ms": skew_ms,
                "target": round(target, 4), "fired": sorted(fired),
                "failed": sorted(failed)}

    def _start_capture_loop(self, config):
        period = float(config.get("interval_s", 2.0))
        count = int(config.get("frames", 0))
        stopev = self._cap_stop
        if stopev is None or stopev.is_set() or not self.active:
            return                       # the run was stopped during calibration

        def _loop():
            k = 0
            start = time.time() + 1.0
            inflight = []
            while not stopev.is_set():
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
                while time.time() < wake and not stopev.is_set():
                    time.sleep(min(0.02, max(0, wake - time.time())))
                if stopev.is_set():
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
        already computed UTM and the staleness/validity flags. `epoch` is a RAW
        local epoch: nav's ring is keyed by local receive time."""
        if not self.nav:
            return None
        try:
            return self.nav.fix_at(epoch)
        except Exception:  # noqa: BLE001
            return None

    # An IMU sample further than this from the capture instant is not that
    # frame's attitude. At ~200 Hz the nearest sample is normally <5 ms away, so
    # anything approaching this bound means the IMU stalled - and writing a
    # half-second-old attitude into a photogrammetry row while a boat rolls is
    # worse than writing nothing, because nothing is visibly missing.
    IMU_MAX_AGE_S = 0.10

    def imu_monitor(self):
        """The connected node currently serving IMU samples, or None.

        Resolved from the health each NodeMonitor already polls, so it costs no
        extra HTTP, and re-resolved whenever the cached node stops reporting an
        IMU - moving the sensor to another Pi needs no code change and no
        restart."""
        if self.imu_node:                      # explicit override (tests)
            m = next((m for m in self.monitors if m.name_ == self.imu_node), None)
            return m if (m and m.is_connected()) else None

        def has_imu(m):
            return bool(m.is_connected()
                        and ((m.health.get("imu") or {}).get("present")))

        cached = self._imu_node_cache
        if cached is not None:
            m = next((m for m in self.monitors if m.name_ == cached), None)
            if m is not None and has_imu(m):
                return m
        for m in self.monitors:
            if has_imu(m):
                if self._imu_node_cache != m.name_:
                    self._imu_node_cache = m.name_
                    self.events.emit("info", "imu", "IMU source is %s" % m.name_,
                                     node=m.name_)
                return m
        self._imu_node_cache = None
        return None

    def imu_snapshot(self, epoch):
        """Attitude at a RAW local capture instant. The IMU ring is stamped on
        the node against the same chrony-disciplined clock, so a GPS-corrected
        epoch here would fall outside every window and blank the columns."""
        imu_mon = self.imu_monitor()
        if not imu_mon:
            return None
        r = http_json("http://%s:8081/imu/window?t0=%.3f&t1=%.3f"
                      % (imu_mon.host, epoch - self.IMU_MAX_AGE_S,
                         epoch + self.IMU_MAX_AGE_S), timeout=4)
        samples = r.get("samples") if isinstance(r, dict) else None
        if samples:
            # nearest sample to the capture instant
            best = min(samples,
                       key=lambda s: abs(s.get("epoch", epoch) - epoch))
            age = abs((best.get("epoch") or epoch) - epoch)
            if age <= self.IMU_MAX_AGE_S:
                best = dict(best)
                best["_age_s"] = age
                best["_node"] = imu_mon.name_
                return best
        # No sample near the capture instant. Do NOT fall back to /imu/latest:
        # that returns whatever arrived most recently, which during a stall is
        # an arbitrarily old attitude that would be written as if it were this
        # frame's. PROTOCOL.md: never fabricate - leave the columns empty and
        # say so once.
        self._imu_gap = getattr(self, "_imu_gap", 0) + 1
        if self._imu_gap in (1, 10, 100) or self._imu_gap % 500 == 0:
            self.events.emit("warn", "imu_stall",
                             "no IMU sample within %d ms of capture (%d frames "
                             "affected) - attitude columns left empty"
                             % (self.IMU_MAX_AGE_S * 1000, self._imu_gap),
                             node=imu_mon.name_, frames=self._imu_gap)
        return None

    def match_exposure_edge(self, mon, expected=None, window=0.20,
                            fire_seq=None, after_seq=None):
        """The GPIO EXPOSURE fall-edge epoch belonging to one frame, or None.

        Three ways to pair, strongest first:
          * `fire_seq` - piagent attributes each fall edge to the fire that
            caused it, so this is identity, not inference.
          * `after_seq` - the edge index at fire time; this fire's exposure can
            only be a LATER edge, which bounds the search by construction.
          * nearest edge to `expected` within `window` - for a node running an
            older piagent, and for frames the Jetson never scheduled.
        Draining the whole cursor and returning the LAST edge - as this once did
        - gives frame 1 frame 3's timestamp whenever more than one frame arrives
        in a poll, and leaves frames 2 and 3 falling back to EXIF."""
        buf = getattr(mon, "_edge_buf", None)
        if buf is None:
            buf = mon._edge_buf = []
        r = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                      % (mon.host, getattr(mon, "_edge_cursor", 0)), timeout=3)
        if isinstance(r, dict):
            mon._edge_cursor = r.get("next", getattr(mon, "_edge_cursor", 0))
            for e in r.get("events", []):
                if e.get("edge") != "fall":
                    continue
                # epoch_hw is the kernel's own interrupt timestamp converted to
                # wall time on the node; epoch is Python's stamp after reading
                # the gpiomon pipe, which carries the node's scheduling latency
                # (median 0.1-0.3 ms here, occasionally hundreds of ms under
                # load). That latency is uncorrelated between the two Pis, so it
                # lands directly in the stereo pair's apparent skew and in the
                # capture instant written to flight_log. Prefer the hardware
                # stamp; fall back for a node still running an older piagent.
                t = e.get("epoch_hw")
                if t is None:
                    t = e.get("epoch")
                if t is not None:
                    buf.append({"t": t, "i": e.get("i"),
                                "fire_seq": e.get("fire_seq")})
        # piagent's ring holds an hour of edges. Anything older than this frame
        # could plausibly be is not ours - it belongs to a bench test or an
        # earlier run - and pairing a frame with a minutes-old edge silently
        # writes a wildly wrong capture time into the flight_log while still
        # claiming capture_source=gpio_edge. Drop stale entries rather than
        # hand one out.
        cutoff = time.time() - self.EDGE_MAX_AGE_S
        while buf and buf[0]["t"] < cutoff:
            buf.pop(0)
        if len(buf) > 64:
            del buf[:-64]
        if not buf:
            return None
        if fire_seq is not None:
            for k, e in enumerate(buf):
                if e.get("fire_seq") == fire_seq:
                    return buf.pop(k)["t"]
            # Its edge has not been read yet; do not fall back to a neighbour's.
            if after_seq is not None:
                return None
        cand = [k for k, e in enumerate(buf)
                if after_seq is None or e.get("i") is None
                or e["i"] > after_seq]
        if not cand:
            return None
        if expected is None:
            return buf.pop(cand[0])["t"]
        # Nearest edge to the instant this frame was scheduled to expose. The
        # window is far wider than the ~1 ms sync we achieve and far narrower
        # than the 500 ms between frames at 2 fps, so it cannot pick a
        # neighbour's edge.
        i = min(cand, key=lambda k: abs(buf[k]["t"] - expected))
        if abs(buf[i]["t"] - expected) > window:
            return None
        return buf.pop(i)["t"]

    # ---- calibration ------------------------------------------------------
    # A calibration exposure is real on the camera but is not survey data. It is
    # kept out of the transect by the exact filename it produced; while one is
    # in flight the node's pull worker holds off listing new frames so the
    # calibration always gets to name its own frame first.
    CAL_QUIET_S = 3.0
    # Wall-clock bound on waiting for a calibration frame. The old bound was 200
    # poll iterations, not seconds: against a node gone slow (a 10 s /api/shots
    # timeout per iteration) that is half an hour of a thread outliving its run,
    # and with PC-save off on the body no frame ever arrives at all.
    CAL_FRAME_WAIT_S = 12.0

    def begin_calibration_fire(self, node, hold_s=None):
        """A calibration exposure is about to happen (or is still landing)."""
        with self._lock:
            self._cal_busy[node] = time.time() + (hold_s or self.CAL_QUIET_S)

    def end_calibration_fire(self, node):
        with self._lock:
            self._cal_busy.pop(node, None)

    def calibration_quiet(self, node):
        with self._lock:
            return self._cal_busy.get(node, 0.0) > time.time()

    def note_calibration_frame(self, node, name):
        with self._lock:
            names = self._cal_names.setdefault(node, [])
            if name not in names:
                names.append(name)
            del names[:-128]

    def note_calibration_frames(self, node, before, after):
        """Name the frames one calibration fire produced, and only those.

        Two filters, and both earn their place:
          * a frame the node's worker has already taken into the survey
            pipeline cannot be this fire's - the worker is held off the node
            from before the shutter until the frame is named, so anything it is
            already handling existed first. Without this, a frame that landed
            between the `before` snapshot and the fire is named as calibration -
            and a real survey frame named as calibration is DELETED from the
            survey, the one outcome worse than a calibration frame surviving in
            it. It deliberately does NOT consult `seen`, which also holds the
            run-start baseline: if the worker's baseline listing happens to race
            ahead of the calibration shutter, the calibration frame is in `seen`
            and the fire could then never name its own frame - leaving the
            puller held off the node until the wait times out.
          * the stem filter keeps a RAW+JPEG pair together while refusing to
            adopt an unrelated frame that happened to land in the same second.
        """
        w = self.workers.get(node)
        known = w.recent_names() if w is not None else set()
        new = sorted(n for n in after if n not in before and n not in known)
        if not new:
            return []
        stem = os.path.splitext(new[0])[0]
        named = [n for n in new if os.path.splitext(n)[0] == stem]
        for n in named:
            self.note_calibration_frame(node, n)
        return named

    def is_calibration_frame(self, node, name):
        """Is this frame one of ours, fired to measure the rig rather than survey?"""
        with self._lock:
            return name in self._cal_names.get(node, ())

    def _calib_over(self, stopev, run_at_entry):
        """Should a calibration pass give up right now?

        A stopped run must not keep firing frames into a transect that is
        already summarised and marked final, and a pass polling a node that has
        gone slow must not outlive the run that started it. rigd also calibrates
        with no run open at all (at startup, and from /api/calibrate) - that
        case has nothing to abort against, which is why both conditions are
        anchored to what was true when the pass began."""
        if stopev is not None and stopev.is_set():
            return True
        return run_at_entry is not None and self.active is not run_at_entry

    def _calibrate_and_arm(self, live, config):
        """Run-start calibration, then release the survey capture loop.

        Ordered, not concurrent: the EXIF clock offset needs a frame it can
        attribute to its own shutter command, and the trigger-latency pass fires
        five more frames per camera. Overlapping them (as separate threads used
        to) let the EXIF pass measure the camera clock against somebody else's
        exposure. Overlapping either with auto-capture was worse still."""
        stopev = self._calib_stop
        run_at_entry = self.active
        try:
            ths = []
            for m in live:
                t = threading.Thread(target=self._calibrate_exif, args=(m,),
                                     daemon=True)
                t.start(); ths.append(t)
            for t in ths:
                t.join(timeout=self.CAL_FRAME_WAIT_S + 8)
            if self._calib_over(stopev, run_at_entry):
                return
            # Re-measure per-camera trigger latency for THIS run: it moves with
            # lens, drive mode and body state, so a figure from an hour ago is
            # not the figure that will align these exposures.
            if config.get("calibrate", True):
                self.calibrate_trigger(nodes=live)
            # This run's edge stream starts after the calibration exposures, so
            # no survey frame can be handed a calibration edge.
            self.reset_edge_cursors()
        except Exception as e:  # noqa: BLE001
            self.events.emit("warn", "calibrate", "calibration failed: %s" % e)
        finally:
            if config.get("auto_capture"):
                self._start_capture_loop(config)

    def _adopt_calibrate(self, m):
        """Calibrate a camera that joined mid-transect, as far as is safe."""
        # Let its worker list (and claim) what is already on the node first.
        # Calibration quiets the puller while its own frames land, and firing
        # into a camera whose backlog has not been listed yet would hold that
        # backlog behind ~7 s of calibration for no reason.
        run_at_entry = self.active
        w = self.workers.get(m.name_)
        if w is not None:
            w.primed.wait(timeout=8.0)
        if self._calib_over(self._calib_stop, run_at_entry):
            return
        self._calibrate_exif(m)
        if m.name_ in self.trig_latency:
            return
        cap = self._cap_stop
        if cap is not None and not cap.is_set() and self.shot_period():
            # Trigger calibration fires five frames and holds FOCUS while it
            # does - which AE-locks the body. Doing that in the middle of a
            # running survey line corrupts the exposures around it, so say what
            # the cost is instead and let the fleet median carry the node.
            self.events.emit(
                "error", "calibration_missing",
                "%s joined mid-transect with no trigger-latency measurement of "
                "its own; it will fire on the fleet median. Stop and restart "
                "the line to calibrate it properly" % m.name_, node=m.name_)
            return
        self.calibrate_trigger(nodes=[m])

    def calibrate_trigger(self, samples=5, hold_focus=True, nodes=None):
        """Measure each camera's TRIGGER -> EXPOSURE latency, per node.

        Two bodies do not answer a trigger in the same time: measured 22.18 ms
        on cam1 and 22.44 ms on cam2, and a different lens or body can widen
        that considerably. Scheduling both for the same instant therefore still
        leaves them exposing at different instants. Firing each node at
        (target - its own latency) makes the EXPOSURES coincide, which is the
        thing stereo actually requires.

        Run at rigd start and again at run start, because the figure moves with
        lens, drive mode and body state. Every frame it costs is registered as a
        calibration exposure so no pull worker writes it into the transect."""
        stopev = self._calib_stop
        run_at_entry = self.active
        pool = nodes if nodes is not None else self.monitors
        live = [m for m in pool if m.is_connected()
                and (m.health.get("gpio", {}) or {}).get("available")]
        if not live:
            return {}
        out = {}
        for m in live:
            if self._calib_over(stopev, run_at_entry):
                break
            if hold_focus:
                http_json("http://%s:8081/gpio/focus" % m.host, {"hold": True},
                          timeout=8)
            time.sleep(0.3)
            lat = []
            for _ in range(samples):
                if self._calib_over(stopev, run_at_entry):
                    break
                r0 = http_json("http://%s:8081/gpio/exposure/events" % m.host,
                               timeout=4)
                cur = r0.get("next", 0) if isinstance(r0, dict) else 0
                # Quiet the puller BEFORE the shutter, not after: on the live
                # rig a frame is listed ~0.5 s after the release and the worker
                # polls every 0.4 s, so "after" is already too late. Snapshot
                # the save dir inside that quiet, immediately before the pulse.
                self.begin_calibration_fire(m.name_)
                named = []
                try:
                    before = {s["name"] for s in m.shots()}
                    t_fire = time.time()
                    r = http_json("http://%s:8081/gpio/fire" % m.host,
                                  {"at_epoch": 0, "pulse_ms": 5}, timeout=10)
                    if not r.get("ok"):
                        self.events.emit("warn", "capture_fail",
                                         "calibration fire on %s refused: %s"
                                         % (m.name_, r.get("error")),
                                         node=m.name_)
                        continue
                    # Watch for THIS fire's frame and name it the moment it
                    # appears, rather than waiting out a flat second: the puller
                    # is held off the node until it is named, and every extra
                    # 100 ms of that is 100 ms a real frame taken during
                    # calibration waits to be pulled.
                    while time.time() < t_fire + 1.2:
                        time.sleep(0.05)
                        named = self.note_calibration_frames(
                            m.name_, before, [s["name"] for s in m.shots()])
                        if named:
                            self.end_calibration_fire(m.name_)
                            break
                    # The exposure edge lands ~22 ms after the pulse and gpiomon
                    # rings it within a millisecond, but give piagent a full
                    # second from the fire before reading: this measurement is
                    # the thing that aligns the two bodies.
                    time.sleep(max(0.0, t_fire + 1.0 - time.time()))
                    ev = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                                   % (m.host, cur), timeout=4)
                    # piagent attributes each fall edge to the fire that caused
                    # it, so use that identity when the node offers it rather
                    # than assuming the first edge back is ours.
                    falls = [e for e in (ev.get("events") or [])
                             if e.get("edge") == "fall"
                             and (r.get("fire_seq") is None
                                  or e.get("fire_seq") is None
                                  or e.get("fire_seq") == r.get("fire_seq"))]
                    if falls and r.get("actual_epoch"):
                        e0 = falls[0]
                        lat.append((e0.get("epoch_hw") or e0.get("epoch"))
                                   - r["actual_epoch"])
                    if not named:
                        # Slow body: still nothing. Name whatever has landed
                        # while the node is STILL quiet - a second pass after
                        # the puller is let back on would adopt a real survey
                        # frame that arrived in the meantime and delete it from
                        # the transect.
                        self.note_calibration_frames(
                            m.name_, before, [s["name"] for s in m.shots()])
                finally:
                    self.end_calibration_fire(m.name_)
                time.sleep(0.2)
            if hold_focus:
                http_json("http://%s:8081/gpio/focus" % m.host, {"hold": False},
                          timeout=8)
            if lat:
                lat.sort()
                out[m.name_] = lat[len(lat) // 2]        # median, outlier-proof
        if out:
            now = time.time()
            self.trig_latency.update(out)
            for k in out:
                self.trig_measured_at[k] = now
            self.events.emit("info", "calibrate",
                             "trigger latency: " +
                             ", ".join("%s=%.2fms" % (k, v * 1000)
                                       for k, v in sorted(out.items())),
                             latency_ms={k: round(v * 1000, 2)
                                         for k, v in out.items()})
        # A node that produced no samples is the dangerous case: it gets no
        # compensation at all unless somebody says so. _trigger_lead falls back
        # to the fleet median, but the operator has to know the pair is not
        # individually aligned.
        missed = [m.name_ for m in live if m.name_ not in out]
        if missed:
            self.events.emit(
                "error", "calibration_missing",
                "no trigger-latency samples from %s - %s. Until it measures, "
                "that camera fires on the fleet median or uncompensated, which "
                "is a systematic inter-camera skew of ~22 ms"
                % (",".join(missed),
                   "keeping its previous measurement"
                   if all(n in self.trig_latency for n in missed)
                   else "it has never been measured"),
                nodes=missed)
        return out

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
        """One frame to learn camera_clock - true_clock for this body.

        Measured against the GPIO EXPOSURE edge when the harness has one, as
        PROTOCOL.md requires: timing it from the pre-call command epoch instead
        folds the blocking USB dispatch (measured 0.44-0.83 s round trip) into
        the offset, biasing every capture_source=exif stamp early by it, per
        node and therefore differentially between the two cameras."""
        stopev = self._calib_stop
        run_at_entry = self.active
        r0 = http_json("http://%s:8081/gpio/exposure/events" % m.host, timeout=3)
        cur = r0.get("next", 0) if isinstance(r0, dict) else 0
        self.begin_calibration_fire(m.name_)
        try:
            # Snapshot the save dir as late as possible - after the puller is
            # held off and immediately before the release - so the window in
            # which an unrelated frame can land and be mistaken for ours is one
            # HTTP round trip rather than several.
            before = {s["name"] for s in m.shots()}
            t_cmd = time.time()
            rs = m.shutter(af=False)
            t_done = time.time()
            if isinstance(rs, dict) and rs.get("ok") is False:
                self.events.emit("warn", "capture_fail",
                                 "EXIF calibration shutter refused on %s: %s"
                                 % (m.name_, rs.get("error")), node=m.name_)
                return
            deadline = t_cmd + self.CAL_FRAME_WAIT_S
            while time.time() < deadline:
                if self._calib_over(stopev, run_at_entry):
                    return
                # Keep the node quiet while we are still waiting for OUR
                # frame, so it cannot be pulled into the transect before it is
                # named - but only for as long as a frame could plausibly still
                # be landing (measured USB save latency 0.44-0.83 s). Past that
                # the body is not delivering at all, and holding the puller off
                # a camera that IS delivering survey frames would cost real
                # data to protect against a frame that is never coming.
                if time.time() < t_cmd + self.CAL_QUIET_S:
                    self.begin_calibration_fire(m.name_, hold_s=1.0)
                time.sleep(0.05)
                after = [s["name"] for s in m.shots()]
                named = self.note_calibration_frames(m.name_, before, after)
                if not named:
                    continue
                name = named[0]
                data, err = http_bytes("http://%s:8080/shot/%s"
                                       % (m.host, name), timeout=30)
                if err or not data:
                    self.events.emit("warn", "calibrate",
                                     "EXIF calibration frame unreadable on %s: %s"
                                     % (m.name_, err), node=m.name_)
                    return
                cam = _exif_capture_epoch(data)
                if not cam:
                    self.events.emit("warn", "calibrate",
                                     "no EXIF timestamp in %s's calibration "
                                     "frame - camera-clock offset not measured"
                                     % m.name_, node=m.name_)
                    return
                true_epoch, err_s = self._true_exposure(m, cur, t_cmd, t_done)
                self.exif_uncertainty[m.name_] = err_s
                self.timesync.set_exif_offset(m.name_, cam - true_epoch)
                return
            self.events.emit(
                "warn", "calibrate",
                "no calibration frame from %s within %.0fs - its camera-clock "
                "offset is unmeasured, so any frame without an EXPOSURE edge "
                "falls back to the command epoch. Check PC-save on the body"
                % (m.name_, self.CAL_FRAME_WAIT_S), node=m.name_)
        finally:
            self.end_calibration_fire(m.name_)

    def _true_exposure(self, m, cursor, t_cmd, t_done):
        """(true capture epoch, uncertainty_s) for the calibration frame.

        A USB-fired release drives the same EXPOSURE line as a GPIO one, so the
        harness dates it to the kernel interrupt - three orders of magnitude
        better than any estimate from the command. Read the edges with a private
        cursor so this never consumes an edge a survey frame is waiting for."""
        ev = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                       % (m.host, cursor), timeout=4)
        falls = [(e.get("epoch_hw") or e.get("epoch"))
                 for e in (ev.get("events") or []) if e.get("edge") == "fall"]
        falls = [f for f in falls if f and f >= t_cmd - 0.05]
        if falls:
            return min(falls), 0.002
        # No harness (or no edge): PROTOCOL.md's fallback, command epoch + the
        # 20 ms hardware release lag + half the 4.5 ms curtain transit. The
        # dispatch itself is the error bar, so record it rather than imply this
        # is as good as an edge.
        return t_cmd + 0.024, max(0.05, t_done - t_cmd)

    # ---- run.json / index -------------------------------------------------
    def note_orphan(self, node, n=1):
        with self._lock:
            if self.active:
                self.active["orphans"][node] = \
                    self.active["orphans"].get(node, 0) + n

    def on_frame(self, node, cam_num, fname, orig, epoch):
        self.events.emit("debug", "frame", "%s <- %s" % (fname, orig), node=node)

    def index_frame(self, cam_num, fname, orig, epoch, source, node=None,
                    path=None):
        with self._lock:
            if self.active:
                # Microseconds, not milliseconds: the capture instant is a
                # kernel interrupt timestamp and the whole rig is judged on a
                # 10 ms budget, so rounding the machine-readable index to 1 ms
                # throws away an eighth of the tolerance the harness buys.
                rec = {"cam": cam_num, "file": fname, "orig": orig,
                       "epoch": round(epoch, 6), "src": source}
                if path:
                    # Which way this frame was fired. A USB-fired frame carries
                    # 0-200 ms of skew against its pair and has to be
                    # identifiable when the survey is processed.
                    rec["path"] = path
                self.active["index"].append(rec)
                if len(self.active["index"]) % 10 == 0:
                    self._write_run_json()

    def _write_run_json(self, final=False):
        run = self.active
        if not run:
            return
        live_off, live_src = self._live_time_base()
        skews = sorted(run["skews_ms"])
        doc = {"run_id": run["run_id"], "label": run["label"],
               "started": run["started"], "nodes": run["nodes"],
               "config": run["config"],
               # The correction actually applied to every datetime and filename
               # in this run, so the whole transect can be re-based in post.
               "time": {"source": run["time_src"],
                        "gps_offset_s": round(run["time_off"], 6),
                        "live_source": live_src,
                        "live_gps_offset_s": round(live_off, 6)},
               "sync": {"shots": len(run["skews_ms"]),
                        "skew_ms_max": skews[-1] if skews else None,
                        "skew_ms_p95": (skews[int(len(skews) * 0.95)]
                                        if skews else None),
                        "late_shots": run["late_shots"],
                        "failed_fires": run["failed_fires"],
                        "fired": dict(run["fired"]),
                        "orphan_fires": dict(run["orphans"]),
                        "trigger_latency_ms": {k: round(v * 1000, 2) for k, v
                                               in self.trig_latency.items()}},
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
            run = self.active
            skews = sorted(run["skews_ms"])
            return {"active": True, "run_id": run["run_id"],
                    "label": run["label"],
                    "started": run["started"],
                    "nodes": run["nodes"],
                    "frames": len(run["index"]),
                    "time_source": run["time_src"],
                    "gps_offset_s": round(run["time_off"], 3),
                    "sync": {"shots": len(skews),
                             "skew_ms_max": skews[-1] if skews else None,
                             "late_shots": run["late_shots"],
                             "failed_fires": run["failed_fires"],
                             "orphan_fires": dict(run["orphans"])},
                    "stats": {n: w.stats() for n, w in self.workers.items()}}


def _slug(s):
    return "".join(c if c.isalnum() or c in "-_" else "-"
                   for c in str(s))[:40] or "run"

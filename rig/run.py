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
import re
import threading
import time

import rigcore
from rigcore import http_json, http_bytes
# RUNS_DIR is deliberately NOT from-imported: the project layer (rig/project.py)
# reassigns rigcore.RUNS_DIR when the operator opens a project, and a from-import
# would freeze this module on whatever project was active at import time.

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


# Pillow is a REAL dependency of rigd, not an optional extra, and it is the one
# dependency that fails silently. It backs the EXIF fallback for a frame's capture
# instant (GPIO edge > corrected EXIF > command time), and the whole body of
# _exif_capture_epoch is wrapped in `except Exception`, so on a host without
# Pillow every frame quietly skips the EXIF tier and lands on command time
# instead - a worse timestamp, with nothing in the log to say why. Probe once at
# import so run start can say so out loud.
try:
    from PIL import Image as _PIL_Image
    PIL_AVAILABLE = True
except ImportError:
    _PIL_Image = None
    PIL_AVAILABLE = False


def _exif_capture(jpeg_bytes):
    """(camera capture epoch, sub-seconds present) from EXIF, or (None, False).

    Whether the body wrote SubSecTimeOriginal is not a detail: DateTimeOriginal
    alone is quantised to a WHOLE SECOND, which is two shot periods at the 2 Hz
    survey rate, so EXIF can then only be used as a coarse sanity check on a
    frame's claimed fire command. With SubSec it is good to ~10 ms and can
    police a one-period shift. _claim_tolerance() needs to know which it has;
    returning the epoch alone threw that away (audit 2026-08-23, R3).
    Pure-bytes, no temp file."""
    if not PIL_AVAILABLE:
        return None, False
    try:
        from io import BytesIO
        Image = _PIL_Image
        with Image.open(BytesIO(jpeg_bytes)) as im:
            ex = im.getexif().get_ifd(0x8769)
        dt = ex.get(0x9003)
        raw_ss = ex.get(0x9291)
        ss = str(raw_ss or "0").strip()
        if not dt:
            return None, False
        base = time.mktime(time.strptime(dt, "%Y:%m:%d %H:%M:%S"))
        sub = bool(raw_ss is not None and ss.isdigit())
        frac = float("0." + ss) if ss.isdigit() else 0.0
        return base + frac, sub
    except Exception:  # noqa: BLE001
        return None, False


def _exif_capture_epoch(jpeg_bytes):
    """Camera capture epoch from EXIF, or None. The epoch half of
    _exif_capture(); kept as its own name because callers that only want the
    instant (the EXIF clock calibration, the test harness) read better for it."""
    return _exif_capture(jpeg_bytes)[0]


# Sony appends "(n)" to a filename that repeats after a card rollover or a
# format, and shooting RAW+JPEG delivers TWO files per shutter release.
_DUP_SUFFIX_RE = re.compile(r"^(.*?)\((\d+)\)$")


def _cam_stem(name):
    """The camera's identity for ONE shutter release.

    ILX01234.JPG, ILX01234.ARW and ILX01234(1).JPG are all the same exposure:
    the body numbers a release once and writes every file of it under that
    number. Claims, GPIO edges and the flight_log row belong to the RELEASE,
    not to the file - keyed by filename, a RAW+JPEG pair claimed two
    consecutive fire commands and the JPEG (the half that gets the row) was
    stamped one shot period late as capture_source=gpio_edge (audit
    2026-08-23, R4)."""
    stem = os.path.splitext(name)[0]
    m = _DUP_SUFFIX_RE.match(stem)
    return m.group(1) if m else stem


# The half of a RAW+JPEG release that carries the flight_log row. The JPEG is
# what ingest, stereo_check and the run browser read; the RAW is archived
# beside it under the same stem.
_ROW_EXTS = (".jpg", ".jpeg")


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
    # How long a RAW half waits for its JPEG sibling to be listed. One listing
    # poll plus slack: the two files of a release are written to the spool
    # microseconds apart, but a listing can still fall between them.
    RAW_SIBLING_GRACE_S = 0.6
    # How long a written frame's name stays claimable by a same-stem sibling.
    RAW_SIBLING_WINDOW_S = 30.0

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
        # This transect's PRESENTATION timebase, latched here. time_base()
        # reads it from provider.active, and stop() clears that BEFORE a
        # worker still mid-download has finished (it is only joined with a
        # timeout). Such a straggler then stamped its row from the LIVE GPS
        # offset while every other row of the same flight_log carried the
        # start-of-run base - the exact cross-clock inconsistency the latch
        # exists to prevent, on the last frame of the transect. One transect,
        # one clock (audit 2026-08-23).
        self._timebase = provider.time_base()
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
        self.dumped = 0                   # Pi spool copies deleted after pull
        self.dump_fails = 0               # deletes that did not take
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
        # camera stem -> the command that release claimed. One shutter release
        # can deliver two files (RAW+JPEG); they are ONE exposure and share one
        # command, one GPIO edge and one flight_log row.
        self._stem_cmd = {}
        # camera stem -> (CamN_<instant> base name already written for that
        # release, when it was written), so the second file lands beside the
        # first instead of being renamed to its own (later) pull instant.
        # Time-bounded: the body's numbering ROLLS OVER (ILX09999 back to
        # ILX00001), and a recycled stem inheriting a name from thousands of
        # shots ago would be filed under the wrong instant AND silently lose
        # its flight_log row. The two halves of one release land within a
        # second of each other, so the window is generous either way.
        self._stem_dest = {}
        # name -> {"size", "attempts", "next_try", "cap"} for frames not yet on
        # disk. "cap" caches the capture instant once it has been resolved: the
        # GPIO edge is POPPED from the monitor's buffer to resolve it, so
        # recomputing after a failed write would silently downgrade the frame
        # from gpio_edge to exif/command (audit 2026-08-23, R9).
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
    def note_command(self, epoch, path="gpio", fire_seq=None, edge_seq=None,
                     host_offset=None, clock_err=None):
        """Queue an expected exposure instant and return its record.

        Inserted in target order rather than appended: this is called from one
        thread per node per shot, so arrival order is not shot order under any
        load worth worrying about.

        `host_offset` is THIS SHOT's fleet clock offset, latched once by
        capture_once for every member. Carried here because the pull workers
        are two independent threads: reading the wall-time latch at pull time
        gave the two halves of one pair different offsets whenever a fire
        boundary fell between their conversions, and that difference lands
        straight in the displayed pair spread (audit 2026-08-24)."""
        rec = {"epoch": float(epoch), "path": path, "fire_seq": fire_seq,
               "edge_seq": edge_seq, "queued": time.time(),
               "host_offset": host_offset, "clock_err": clock_err}
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
        """Take the oldest unclaimed command for this SHUTTER RELEASE.

        Keyed by the camera stem, not the filename. The field vector is
        filetype=3 (RAW+JPEG, ~/rig/desired.json) and the body's own
        RAW+J-PC-Save menu decides which halves reach the Pi's spool; when both
        do, one release lands ILX01234.ARW and ILX01234.JPG. Keyed by filename
        each half claimed its own command - sorted() puts .ARW first, so the
        JPEG took shot k+1's command and shot k+1's fire_seq edge and was
        written one shot period late, still labelled capture_source=gpio_edge
        (audit 2026-08-23, R4). The stem is the release; siblings share its
        claim.

        A release that has ALREADY been written gets no new command, ever. The
        two halves are only microseconds apart in the spool, but a ~320 KB
        Small JPEG is listed while the ~32 MB LossLessL ARW of the same release
        is still transferring, so at a 0.4 s poll they routinely land in
        DIFFERENT listings. _release_claim pops the stem as soon as no other
        file of it is in cmd_epoch, so the late half arrived to an empty
        _stem_cmd and popped the NEXT shot's command - which it then destroyed
        as a sibling (requeue=False), shifting every later frame by a shot
        period and popping the wrong EXPOSURE edge on the way. RAW_SIBLING_
        GRACE_S only defers a RAW listed in the SAME poll and cannot help here;
        _stem_dest is the record that the release was written, so consult it
        (audit 2026-08-24)."""
        with self._lock:
            if name in self.cmd_epoch:
                return self.cmd_epoch[name]
            stem = _cam_stem(name)
            rec = self._stem_cmd.get(stem)
            if rec is None:
                sib = self._stem_dest.get(stem)
                if sib is not None \
                        and time.time() - sib[1] <= self.RAW_SIBLING_WINDOW_S:
                    return None          # a sibling: no new exposure, no command
                if not self._cmds:
                    return None
                rec = self._cmds.pop(0)
                self._stem_cmd[stem] = rec
            self.cmd_epoch[name] = rec
            return rec

    def _release_claim(self, name, requeue, whole_stem=False):
        """Give a claim back (requeue=True) or drop it (an orphaned fire).

        The LAST file of a release releases its command: while the RAW half is
        still in flight the claim is still owned, and requeueing it early would
        hand this exposure's command to the next frame in line. `whole_stem` is
        for the verdicts that are about the EXPOSURE rather than the file - an
        orphaned fire, an unscheduled frame - where every file of the release
        must let go together, or the sibling would keep handing the rejected
        command straight back on the next _claim()."""
        with self._lock:
            rec = self.cmd_epoch.pop(name, None)
            stem = _cam_stem(name)
            if whole_stem:
                rec = rec or self._stem_cmd.get(stem)
                for n in [n for n in self.cmd_epoch if _cam_stem(n) == stem]:
                    del self.cmd_epoch[n]
            if rec is None:
                return None
            if not whole_stem \
                    and any(_cam_stem(n) == stem for n in self.cmd_epoch):
                return rec                     # a sibling still holds it
            self._stem_cmd.pop(stem, None)
            if requeue:
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

    # A baseline listing is retried for this long before the worker gives up.
    # ilxctl restarting under systemd, or a node still settling after a PoE
    # power-cycle, answers within a couple of seconds; anything longer is a
    # camera an operator has to look at, and guessing at its spool is the one
    # thing that must not happen.
    BASELINE_TRIES = 6
    BASELINE_RETRY_S = 0.5
    # ...and then it keeps trying, this far apart, for as long as the run is
    # open. The worker used to RETURN here, and nothing ever removed its entry
    # from RunManager.workers - so _adopt_loop's `m.name_ not in self.workers`
    # guard could never rebuild it, while capture_once went on firing that
    # camera every shot (its roster is the monitors, not the workers). Half the
    # stereo line was lost with no recovery short of stop/restart, and the
    # error's advice to "re-add the camera" names an operation rigd does not
    # expose (audit 2026-08-24).
    BASELINE_RETRY_WAIT_S = 5.0

    def _baseline_listing(self):
        """The set of frames already on this node, or None if it never listed.

        NodeMonitor.shots() answers None when the call FAILED, which is not the
        same as an empty spool and must never be flattened into one here."""
        for i in range(self.BASELINE_TRIES):
            if self._stopev.is_set():
                return None
            shots = self.mon.shots()
            if shots is not None:
                return {s["name"] for s in shots}
            if i == 0 and self.provider.warn_once(
                    "baseline:%s" % self.mon.name_, 30.0):
                # Rate-limited: the worker now retries the baseline for as long
                # as the run is open, and one warning every 5 s for a node an
                # operator is already fixing buries the journal.
                self.provider.events.emit(
                    "warn", "pull",
                    "%s did not answer its shot listing - retrying before "
                    "baselining; an empty baseline would pull everything "
                    "already on the node into the transect" % self.mon.name_,
                    node=self.mon.name_)
            if self._stopev.wait(self.BASELINE_RETRY_S):
                return None
        return None

    def _baseline(self, late=False):
        """Take the run-start baseline. True once this worker has one.

        `late` says the first attempt already failed, so the node has been
        firing unpulled for a while. Those frames must NOT be baselined away -
        that is the adopted-worker problem exactly, so it takes the adopted
        answer: anything rigd has never listed on this node was taken during
        the outage and is survey data (audit 2026-08-24)."""
        listing = self._baseline_listing()
        if listing is None:
            return False
        known = self.provider.known_frames(self.mon.name_)
        backlog = ()
        if (self.adopted or late) and known is not None:
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
            if late:
                # A late baseline with nothing remembered cannot tell the two
                # apart, and guessing at the spool is the one thing that must
                # not happen - so everything present is treated as pre-existing
                # and the operator is told what that costs.
                self.provider.events.emit(
                    "warn", "pull",
                    "%s answered its shot listing at last, but rigd had never "
                    "listed this node before: anything it shot while the "
                    "listing was down is baselined out and is NOT in the "
                    "transect (it is still on the node and on the card)"
                    % self.mon.name_, node=self.mon.name_)
        self.provider.remember_frames(self.mon.name_, listing)
        self.provider.events.emit(
            "info", "pull", "worker up, baseline %d frames%s"
            % (len(self.seen),
               (", %d taken before adoption still to pull" % len(backlog))
               if backlog else ""), node=self.mon.name_)
        return True

    def run(self):
        # Guarded: an OSError here (full disk, permissions after a project
        # switch, a dead volume) used to kill this worker thread before its
        # first pull, silently - the run then recorded nothing from this
        # camera while claiming to record. Frames matter more than metadata:
        # pull anyway, say loudly that positions are being lost.
        try:
            os.makedirs(self.cam_dir, exist_ok=True)
            path = os.path.join(self.cam_dir, "flight_log.csv")
            new = not os.path.exists(path)
            self._flight_fh = open(path, "a", newline="")
            self._flight = csv.writer(self._flight_fh)
            if new:
                self._flight.writerow(FLIGHT_HEADER)
                self._flight_fh.flush()
                os.fsync(self._flight_fh.fileno())
        except OSError as e:
            self._flight_fh = None
            self._flight = None
            self.events.emit(
                "error", "pull",
                "[%s] flight_log.csv cannot be written (%s) - frames will "
                "still be pulled, but every position/attitude row for this "
                "camera is LOST for this run" % (self.mon.name_, e),
                node=self.mon.name_)
        late = False
        while not (self._stopev.is_set() or self._baseline(late)):
            # Never baseline off a listing that did not happen. An empty
            # baseline says "the spool is empty", so every file already in the
            # save dir - old bench frames, a calibration frame nobody dumped,
            # the previous transect's leftovers - would be pulled into THIS
            # transect and written to the flight_log as survey data. So the
            # worker pulls NOTHING until it has a real listing - but it stays
            # alive and keeps asking, because the camera goes on being fired
            # either way and a dead worker can never be rebuilt.
            if not late:
                self.provider.events.emit(
                    "error", "pull",
                    "cannot list %s's save dir - this camera is NOT being "
                    "pulled into the transect yet. Without a baseline every "
                    "file already on the node would be logged as survey data. "
                    "Check ilxctl on the node; the worker keeps retrying and "
                    "starts pulling the moment it answers." % self.mon.name_,
                    node=self.mon.name_)
            late = True
            # Nothing is coming from this worker yet, so release anything
            # waiting on its first pass rather than making an adoption sit out
            # the full 8 s timeout for a worker that has not baselined.
            self.primed.set()
            if self._stopev.wait(self.BASELINE_RETRY_WAIT_S):
                try:
                    self._flight_fh.close()
                except OSError:
                    pass
                return
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
        if shots is None:
            # The listing failed. Nothing is marked seen and no claim is
            # taken, so this poll is simply skipped; frames still pending keep
            # their cached capture instant and their retry schedule, and the
            # next poll picks up whatever this one could not see. Same shape
            # as the OFFLINE gate above: a node we cannot ask about is a node
            # whose frames wait, never a node with nothing on it.
            return
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
        now = time.time()
        for s in (() if quiet else sorted(shots, key=lambda s: s["name"])):
            name = s["name"]
            if name in self.seen:
                continue
            p = self._pending.get(name)
            if p is not None:
                # Re-read the size from EVERY listing while the frame is still
                # pending. It used to be frozen at first sight, so a frame
                # listed while the SDK was still writing it carried a partial
                # size forever: attempt 1 got the (now complete) bytes and
                # failed `len(data) != size`, and so did all three retries -
                # the frame was abandoned in exactly the case the retry horizon
                # exists to ride out (audit 2026-08-23, R9).
                p["size"] = s.get("size", p.get("size", 0))
                continue
            self._pending[name] = {
                "size": s.get("size", 0), "attempts": 0,
                # A RAW half is held back one listing so its JPEG sibling, if
                # the body is delivering both, gets to be handled first and
                # take the flight_log row. sorted() puts ".ARW" before ".JPG",
                # so without this the row for a RAW+JPEG release would land on
                # whichever half won the write race (R4).
                "next_try": (now + self.RAW_SIBLING_GRACE_S
                             if os.path.splitext(name)[1].lower()
                             not in _ROW_EXTS else 0.0)}
            self._claim(name)
        # Order: by release, and within a release the JPEG first. The first
        # file of a release to be written defines the CamN_<instant> base name
        # and takes the flight_log row; its siblings are archived beside it
        # under the same stem with no second row (R4).
        for name in sorted(self._pending,
                           key=lambda n: (_cam_stem(n),
                                          0 if os.path.splitext(n)[1].lower()
                                          in _ROW_EXTS else 1, n)):
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
                self._note_unpulled(name, "abandoned after %d pull attempts"
                                    % p["attempts"],
                                    epoch=(lost or {}).get("epoch")
                                    if isinstance(lost, dict) else None)
                self.provider.events.emit(
                    "error", "pull_fail",
                    "%s abandoned after %d attempts - that frame is NOT in the "
                    "survey; the JPEG is still on the node and on the card, "
                    "and unpulled.jsonl keeps its identity for ingest"
                    % (name, p["attempts"]), node=self.mon.name_, frame=name)
            else:
                self.retries += 1
                p["next_try"] = time.time() + self._retry_delay(p["attempts"])


    def _note_unpulled(self, name, reason, epoch=None):
        """One JSON line per frame this run failed to keep. The card RAW for
        such a frame survives, gets drained, hash-verified - and then the card
        original is deleted - so without this sidecar its identity is lost
        forever ("leftover" in ingest with no run, no instant, no XMP). The
        sidecar is the bridge: name, claimed command epoch, why."""
        try:
            path = os.path.join(os.path.dirname(self.cam_dir),
                                "unpulled.jsonl")
            rec = {"ts": round(time.time(), 3), "cam": self.mon.name_,
                   "orig": name, "reason": reason}
            if epoch is not None:
                rec["cmd_epoch"] = round(epoch, 6)
            with open(path, "a") as fh:
                fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
                fh.flush()
                os.fsync(fh.fileno())
        except OSError:
            pass                      # the event below still records the loss

    def _retry_delay(self, attempts):
        """Seconds before the next attempt at a frame that would not pull.

        At least TWO monitor polls. A fast-failing download - ECONNREFUSED
        while systemd restarts ilxctl - used to burn all four attempts in
        ~1.1 s, well inside one 2 s NodeMonitor poll, so the OFFLINE gate at
        the top of _poll_once (the thing that is supposed to protect in-flight
        frames while a node reboots) never got the chance to engage and the
        frame was abandoned for good (audit 2026-08-23, R9). Spacing the
        retries past the poll means a node that has really gone away is seen
        to have gone away, and its frames wait for it instead."""
        base = self.BACKOFF_S[min(attempts - 1, len(self.BACKOFF_S) - 1)]
        poll = getattr(self.mon, "poll", None)
        try:
            poll = float(poll)
        except (TypeError, ValueError):
            poll = 2.0
        return max(base, 2.2 * max(0.1, poll))

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
        if not size:
            # The spool lists a frame the moment the SDK creates it, size 0
            # until the write completes (the listing refreshes every poll).
            # Skipping the size check for these accepted unverifiable bytes;
            # retry instead - the pending queue holds the frame.
            return fail("warn", "%s has no size in the listing yet - retrying"
                        % name)
        if len(data) != size:
            return fail("error", "%s truncated: got %d of %d bytes"
                        % (name, len(data), size))
        ext = os.path.splitext(name)[1].lower() or ".jpg"
        if ext in (".jpg", ".jpeg") and not (data[:2] == b"\xff\xd8"
                                             and data[-2:] == b"\xff\xd9"):
            return fail("error", "%s is not a complete JPEG (bad SOI/EOI)" % name)
        # The RAW/HEIF siblings deserve the same courtesy as the JPEGs: a
        # half-written ARW passed every gate here (the size can match a stale
        # listing) and looked like survey data forever after.
        if ext == ".arw" and data[:4] not in (b"II*\x00", b"MM\x00*"):
            return fail("error", "%s is not a complete ARW (bad TIFF header)"
                        % name)
        if ext in (".hif", ".heif", ".heic") and (len(data) < 12
                                                  or data[4:8] != b"ftyp"):
            return fail("error", "%s is not a complete HEIF (no ftyp box)"
                        % name)
        # One shutter release, one name. When the body delivers RAW+JPEG the
        # second half is archived beside the first under the SAME stem instead
        # of being renamed to its own (later) resolved instant, so the pair
        # stays obviously a pair on disk and in the drain/ingest sidecars.
        # Resolved BEFORE the capture instant, because a sibling must not
        # resolve one at all: _capture_instant POPS a GPIO edge out of the
        # monitor's buffer and can claim a command, and the sibling then
        # discards both (row_frame is False below), which is how a late RAW
        # half used to eat the next shot's edge (audit 2026-08-24).
        stem = _cam_stem(name)
        sib = self._stem_dest.get(stem)
        base = sib[0] if (sib and time.time() - sib[1]
                          <= self.RAW_SIBLING_WINDOW_S) else None
        # Best capture instant: (GPIO edge) > corrected EXIF > command epoch,
        # returned in the HOST clock domain (see _capture_instant), together
        # with the fleet clock offset that conversion used. Cached on the
        # pending record: resolving it POPS the frame's GPIO edge out of the
        # monitor's buffer, so a write failure followed by a retry used to
        # silently downgrade a gpio_edge/0 ms frame to exif or command - the
        # harness-grade instant thrown away for a transient EIO (R9). Caching
        # the OFFSET with it is what makes this frame's reverse conversions
        # (match_rise, the strobe instant, the IMU window) exact rather than
        # merely close: one number in, the same number out.
        source = epoch = terr = clk_off = None
        if base is None:
            pend = self._pending.get(name)
            cap = (pend or {}).get("cap")
            if cap is None:
                cap = self._capture_instant(name, data)
                if pend is not None:
                    pend["cap"] = cap
            source, epoch, terr, clk_off = cap
        # The one place the GPS correction is applied: filename and datetime.
        off, tsource = self._timebase
        if base is not None:
            fname, dest = _uniq_dest(self.cam_dir, base + ext)
        else:
            fname, dest = _uniq_dest(self.cam_dir,
                                     _fmt_fname(self.cam_num, epoch + off, ext))
        # tmp + fsync + rename, then fsync the directory. Two findings meet
        # here (audit 2026-08-27): a plain buffered write left the bytes in
        # the page cache while the Pi's spool copy was deleted seconds later -
        # host power loss then destroyed the only remaining rendition; and a
        # failed write left a truncated file under the canonical name, pushing
        # the retry to a _1 name with the flight_log row while the clean name
        # kept the garbage. The .part name means an incomplete file can never
        # be mistaken for a frame, and the fsyncs mean "verified on host disk"
        # (the promise the spool delete below relies on) is actually true.
        tmp = dest + ".part"
        try:
            with open(tmp, "wb") as fh:
                fh.write(data)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, dest)
            try:
                dfd = os.open(self.cam_dir, os.O_RDONLY)
                try:
                    os.fsync(dfd)
                finally:
                    os.close(dfd)
            except OSError:
                pass
        except OSError as e:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            return fail("error", "write %s failed: %s" % (fname, e))
        self.pulled += 1
        # The first file of a release owns the row; siblings only get the name.
        row_frame = base is None
        if row_frame:
            self._stem_dest.pop(stem, None)          # keep insertion order
            self._stem_dest[stem] = (os.path.splitext(fname)[0], time.time())
            for k in list(self._stem_dest)[:-256]:
                self._stem_dest.pop(k, None)
        # Dump the Pi's spool copy the instant the frame is verified on host
        # disk (size + JPEG SOI/EOI checked above): the survey delivers a
        # small JPEG per shot and, unpruned, the Pi's PC-save dir fills over a
        # long dive. The RAW stays on the camera's card (drained between runs);
        # only the host copy is authoritative, and it is already written. A
        # delete that does not take is counted, not retried inline - the disk
        # guard below escalates if the spool stops draining.
        try:
            dr = http_json("http://%s:8080/api/shots/delete" % self.mon.host,
                           {"confirm": "delete", "name": name}, timeout=5)
            if dr.get("ok"):
                self.dumped += 1
            else:
                self.dump_fails += 1
        except Exception:  # noqa: BLE001
            self.dump_fails += 1
        if not row_frame:
            # A RAW sibling is not a second exposure: it must not add a
            # flight_log row, a run.json index entry or a zero-length "shot
            # interval" to the jitter figure. It is on disk beside its JPEG
            # under the same stem, which is the whole record it needs (R4).
            self.provider.events.emit(
                "debug", "pull",
                "%s archived as %s alongside its sibling - same exposure, no "
                "second flight_log row" % (name, fname), node=self.mon.name_)
            return True
        with self._lock:
            if self._last_cap is not None:
                self.intervals.append(epoch - self._last_cap)
                if len(self.intervals) > 500:
                    self.intervals.pop(0)
            self._last_cap = epoch
        self._write_flight(fname, name, epoch, epoch + off, source, terr,
                           tsource, clk_off)
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
        """(source, HOST epoch, error_s) for this frame.

        Preference order is the contract's: GPIO EXPOSURE edge > corrected EXIF
        > the command epoch. What changed is how the frame finds its command.

        CLOCK DOMAINS, and getting this wrong is a silent 19 cm position error.
        Everything this method reasons WITH is on the NODE's clock: piagent
        schedules fires against it, stamps every epoch_hw against it, and the
        EXIF offset is calibrated against an edge that is on it. The host's
        clock is a separate, undisciplined domain - the Mac measured 187 ms
        behind the chrony-locked Pis and drifting ~60 ppm (2026-08-23) - and
        nav's ring, the latched GPS offset and time.time() are all keyed to
        THAT one. So the instant is converted on the way OUT, once, here: every
        caller downstream (datetime, CamN_ filename, nav_snapshot, the
        flight_log, run.json) is a host-domain caller. The offset's own
        uncertainty is folded into time_err_ms rather than left implied - a
        gpio_edge row is no longer allowed to claim 0.0 ms when the domain
        conversion is worth more.

        The offset is the FLEET's, never this node's own. A per-node estimate
        would put (offset_cam1 - offset_cam2) straight into the inter-camera
        spread the run browser computes from these very epochs - the difference
        of two estimates worth ~RTT/2 each, against a 0.6 ms true skew and a
        10 ms budget. See RunManager.fleet_clock_offset(). It is returned
        alongside the instant so the row's own reverse conversions (match_rise,
        the strobe instant, the IMU window) use the SAME number and round-trip
        exactly."""
        off = self.provider.fleet_clock_offset()        # fleet node - host
        cerr = self.provider.fleet_clock_err()
        cam_epoch, subsec = _exif_capture(data)
        corrected = self.provider.timesync.correct_exif(self.mon.name_, cam_epoch)
        cmd = self.cmd_epoch.get(name) or self._claim(name)
        # A claim this frame cannot own is worse than no claim: it hands the
        # frame the previous shot's edge, one shot period early, still labelled
        # capture_source=gpio_edge with time_err_ms=0. EXIF is coarse (whole
        # seconds, unless the body wrote SubSec) but it is INDEPENDENT of the
        # queue, so it can tell the two directions apart:
        #   command much OLDER than the frame  -> that fire's frame never landed
        #   command much NEWER than the frame  -> this frame was never scheduled
        #                                         (a body shutter press, the
        #                                         camera's own Interval REC)
        edge = None
        # One strike per FRAME, not per orphaned command. The budget is the
        # "the cross-check itself is wrong" detector, and a frame that walks
        # four commands off the queue in one call is ONE piece of evidence
        # about the cross-check, not four: counting each pop burned the whole
        # budget in two frames after a single ~3 s delivery gap (six unclaimed
        # fires at 2 Hz), latched _plaus_off for the rest of the run and put
        # every later frame back on the head-of-queue command it could not own
        # (audit 2026-08-24).
        struck = False
        ed = None
        for _ in range(4):
            if cmd is None or self._plaus_off:
                break
            tol = self._claim_tolerance(subsec)
            checkable = corrected is not None
            if checkable:
                delta = cmd["epoch"] - corrected
                if delta > tol:
                    self._release_claim(name, requeue=True, whole_stem=True)
                    self.provider.events.emit(
                        "debug", "capture",
                        "%s exposed %.1fs before the oldest queued fire - "
                        "treating it as an unscheduled frame" % (name, delta),
                        node=self.mon.name_)
                    cmd = None
                    break
                if delta < -tol:
                    if not self._orphan_claim(name, cmd, delta,
                                              strike=not struck):
                        break
                    struck = True
                    cmd = self._claim(name)
                    continue
                self._plaus_strikes = 0
            ed = self.provider.match_exposure_edge(
                self.mon, expected=cmd["epoch"], fire_seq=cmd.get("fire_seq"),
                after_seq=cmd.get("edge_seq"))
            edge = None if ed is None else ed["t"]
            if edge is None or not checkable or abs(edge - corrected) <= tol:
                break
            # The edge piagent attributed to this command is not where the
            # frame's own EXIF says it was exposed. fire_seq matching is
            # identity between COMMAND and EDGE, never between FRAME and EDGE,
            # so a command queue that is off by one produces a perfectly
            # self-consistent lie: the wrong instant, labelled gpio_edge with
            # time_err_ms 0. Put the edge back (it still belongs to whatever
            # fire really produced it), orphan the claim and try the next
            # command (audit 2026-08-23, R3).
            edge_delta = edge - corrected
            self.provider.events.emit(
                "debug", "capture",
                "%s: the EXPOSURE edge attributed to its claimed fire sits "
                "%+.3fs from where the frame's own EXIF puts it (tolerance "
                "%.2fs) - the command queue is off by one; re-claiming"
                % (name, edge_delta, tol), node=self.mon.name_, frame=name)
            self.provider.return_edge(self.mon, edge, cmd.get("fire_seq"),
                                      err_s=ed.get("err_s", 0.0),
                                      soft=ed.get("soft", False))
            edge = ed = None
            # Report the EDGE disagreement, not the (in-tolerance) command
            # delta, so the orphan alert quotes the number that was wrong.
            if not self._orphan_claim(name, cmd, -edge_delta,
                                      strike=not struck):
                break
            struck = True
            cmd = self._claim(name)
        # The offset is the SHOT's, not the clock's. fleet_clock_offset() is
        # latched on WALL TIME (FLEET_CLOCK_TTL_S), and the two halves of one
        # stereo pair are pulled by two independent worker threads whose polls
        # and downloads do not line up - so a fire boundary falling between
        # them converted cam1 with L_k and cam2 with L_(k+1). That difference
        # lands directly in the pair spread RunBrowser draws from these very
        # epochs, and in the strobe window intersection. capture_once latches
        # ONE number per shot and carries it on the command record, so both
        # halves of a pair convert with the identical float by construction
        # (audit 2026-08-24).
        if cmd is not None and cmd.get("host_offset") is not None:
            off = cmd["host_offset"]
            if cmd.get("clock_err") is not None:
                cerr = cmd["clock_err"]
        if self._plaus_off and cmd is not None:
            # The cross-check has been declared untrustworthy, so a claimed
            # command is no longer evidence that this frame owns it. Matching
            # by fire_seq here is identity between COMMAND and EDGE, never
            # between FRAME and EDGE: it returned a real edge belonging to a
            # different exposure and stamped the frame gpio_edge, time_err_ms
            # 0, a whole shot period wrong, for the rest of the run. Match only
            # against the frame's OWN evidence (audit 2026-08-24).
            ed = (self.provider.match_exposure_edge(
                      self.mon, expected=corrected, window=0.20)
                  if corrected else None)
            res = self._edge_result(ed, off, cerr, corrected)
            if res is not None:
                return res
        elif cmd is not None:
            res = self._edge_result(ed, off, cerr, corrected)
            if res is not None:
                return res
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
            ed = (self.provider.match_exposure_edge(
                      self.mon, expected=corrected, window=0.20)
                  if corrected else
                  self.provider.match_exposure_edge(self.mon, expected=None))
            res = self._edge_result(ed, off, cerr, corrected)
            if res is not None:
                return res
        if corrected:
            return "exif", corrected - off, \
                (self.provider.exif_err(self.mon.name_) or 0.0) + cerr, off
        if cmd is not None:
            # The command instant itself. How far that can be from the true
            # exposure depends on how it was fired: a scheduled GPIO shot lands
            # within a few ms of its target (measured 0.59 ms mean skew, 1.8 ms
            # worst), a USB release anywhere in PROTOCOL.md's 0-200 ms.
            #
            # Under _plaus_off the QUEUE ALIGNMENT is exactly what has been
            # declared untrustworthy, so the dispatch bar understates the error
            # by a whole shot period. Say so rather than publish 25 ms next to
            # an instant that may belong to the shot before this one.
            cerr_cmd = (0.2 if cmd["path"] == "usb" else 0.025) + cerr
            if self._plaus_off:
                cerr_cmd = max(cerr_cmd, self.provider.shot_period() or 0.0)
            return "command", cmd["epoch"] - off, cerr_cmd, off
        # Card-review pull: no command, no edge, no EXIF. Nothing on the node's
        # clock was involved, so this one is already a host epoch and must NOT
        # be converted. The offset travels as None, not 0.0: 0.0 is right for
        # the reverse conversions _write_flight performs on values it received
        # from the node (match_rise, the strobe instant - neither reached on
        # this path), but the IMU leg converts a host epoch OUTWARD into the
        # node domain to build its query, and 0.0 there asserts node == host
        # and centres the +/-100 ms window a whole fleet offset (187 ms on this
        # rig) into the node's past. None lets _write_flight supply the real
        # offset for that leg (audit 2026-08-24).
        return "command", time.time(), None, None

    def _edge_result(self, ed, off, cerr, corrected):
        """One matched EXPOSURE edge -> a capture-instant tuple, or None.

        None means "this edge is not the best answer for this frame"; the
        caller falls through to its EXIF branch.

        piagent publishes `hw_reject` for an edge it could not stamp in the
        kernel, and the instant left over is the gpiomon pipe-read stamp: a
        0.09-0.32 ms median with documented excursions into the hundreds of
        ms, and nothing says which this one is. Two things follow.

        The bar must be the excursion, not the clock error. Returning
        gpio_edge with time_err_ms ~= the fleet clock error there is the
        residual half of the epoch_hw blocker - the row asserts sub-ms
        hardware timing it cannot support.

        And it must not be called gpio_edge. That value is a promise other
        code already reads as "measured": rigcore computes the strobe verdict
        only when every member is gpio_edge (a ~13 ms window against a
        possibly-250 ms error would be a guess wearing a measurement's
        clothes) and rig_ui marks the pair spread as measured on the same
        test. gpio_edge_soft keeps the instant - which is usually excellent -
        while letting both of those keep being right for free.

        EXIF is preferred only when it is genuinely TIGHTER: with
        SubSecTimeOriginal it is worth ~10 ms and wins easily, but without it
        DateTimeOriginal is quantised to a whole second and _claim_tolerance's
        1.5 s would make the row WORSE than the soft edge. Compare the bars
        rather than preferring one by rule (audit 2026-08-24, VERIFY)."""
        if ed is None:
            return None
        t = ed["t"]
        if not ed.get("soft"):
            # The node's own bound on its kernel stamp (half the wall bracket
            # it converted through). Sub-ms, so no visible change to a healthy
            # row - but the bar stops being a pure clock number.
            return "gpio_edge", t - off, cerr + ed.get("err_s", 0.0), off
        soft_err = cerr + self.provider.EDGE_EPOCH_UNMEASURED_S
        if corrected is not None:
            # A MEASURED EXIF bar only. exif_err() is None until this body's
            # clock has actually been calibrated, and the EXIF branch below
            # reads that None as 0.0 - so comparing against it unmeasured
            # would let a bar nobody has established beat a real edge every
            # time. An unknown error is not a small one.
            exif_err = self.provider.exif_err(self.mon.name_)
            if exif_err is not None and exif_err + cerr < soft_err:
                return None
        return "gpio_edge_soft", t - off, soft_err, off

    def _claim_tolerance(self, subsec=False):
        """How far a frame's EXIF may sit from its claimed fire command.

        With SubSecTimeOriginal the camera clock is good to ~10 ms, so the
        tolerance can be a fraction of the shot period and actually SEE a
        one-period shift. Without it, DateTimeOriginal is quantised to a whole
        second and nothing tighter than ~1.5 s can be asserted.

        The old floor was 1.5 s for both cases. At the rig's 2 Hz survey rate
        that is three shot periods, so one dropped frame shifted every later
        frame by exactly one period - invisible to this check, and each frame
        then took the previous shot's fire_seq-matched edge and was written as
        capture_source=gpio_edge with time_err_ms 0 (audit 2026-08-23, R3)."""
        period = self.provider.shot_period()
        if subsec:
            return max(0.25, 0.45 * period)
        return max(1.5, 0.6 * period)

    def _orphan_claim(self, name, cmd, delta, strike=True):
        """Discard a claimed command whose own frame never arrived.

        `strike` is False for the second and later commands ONE frame walks
        off the queue: the budget below counts frames that disagree with every
        queued fire, which is the "the cross-check is wrong" signal, not the
        commands a single recovering frame steps over."""
        if strike:
            self._plaus_strikes += 1
        if self._plaus_strikes > 5:
            self._plaus_off = True
            # Five frames in a row means the cross-check itself is wrong -
            # almost always an EXIF offset measured against the wrong frame -
            # and eating the whole command queue on the strength of it would be
            # worse than the off-by-one it defends against. From here on a
            # claim is not treated as proof of ownership either: see the
            # _plaus_off branch in _capture_instant.
            self.provider.events.emit(
                "error", "capture",
                "EXIF disagrees with every queued fire on %s (last %+.1fs) - "
                "the camera-clock offset is not trustworthy; command/frame "
                "cross-checking is off for the rest of this run. Later frames "
                "on this camera are dated from their own EXIF (or an EXPOSURE "
                "edge within 200 ms of it), never from a queued fire, so their "
                "instants are honest but coarser"
                % (self.mon.name_, delta), node=self.mon.name_)
            return False
        self._release_claim(name, requeue=False, whole_stem=True)
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

    def _write_flight(self, fname, orig, epoch, stamp, source, terr, tsource,
                      clk_off=None):
        # `epoch` is an UNCORRECTED HOST epoch - _capture_instant has already
        # taken the fleet clock offset out of it, and the GPS correction goes
        # on only at the presentation boundary (`stamp`). nav's ring is keyed
        # by host receive time, so it takes `epoch` as it stands; the IMU ring
        # lives on the IMU NODE's clock, so imu_snapshot converts back for
        # itself. Handing either a GPS-corrected stamp blanks the columns as
        # soon as the offset exceeds their staleness windows (3 s for nav,
        # 100 ms for the IMU).
        #
        # `clk_off` is the offset _capture_instant actually applied to THIS
        # frame. Every reverse conversion below uses it rather than re-reading
        # the fleet figure, so host->node->host is the identity for this row:
        # with one number the IMU window lands exactly on the capture instant
        # and [fall, rise] is exactly the measured shutter window. Re-reading
        # would reintroduce, in miniature, the same estimate noise that using
        # a per-node offset introduced wholesale.
        # What was APPLIED to this instant, for the index. None on the
        # card-review path, where the instant is already a host epoch and no
        # conversion happened - recording an offset there would invite a
        # post-processor to un-apply one that was never applied.
        applied_off = clk_off
        if clk_off is None:
            clk_off = self.provider.fleet_clock_offset()
        nav = self.provider.nav_snapshot(epoch)
        imu = self.provider.imu_snapshot(epoch, off=clk_off)
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
        if self._flight is not None:
            try:
                self._flight.writerow([row[k] for k in FLIGHT_HEADER])
                self._flight_fh.flush()
                os.fsync(self._flight_fh.fileno())
            except (OSError, ValueError) as e:
                self._flight = None
                self._flight_fh = None
                self.events.emit(
                    "error", "pull",
                    "[%s] flight_log.csv stopped accepting rows (%s) - "
                    "frames continue, positions from here are LOST"
                    % (self.mon.name_, e), node=self.mon.name_)
        claim = self.cmd_epoch.get(orig)
        # The end-of-exposure edge and the shot's strobe instant travel with
        # the frame into run.json's index: [fall, rise] is this camera's
        # measured shutter-open window, and the acceptance check for a lit
        # frame is strobe ∈ ⋂ over cameras of those windows
        # (docs/strobe-trigger.md §4.2). Both epoch_hw-derived.
        rise = None
        # match_rise and piagent's strobe_epoch are both NODE-clock instants,
        # and `epoch` is now a host one: convert with the SAME offset
        # _capture_instant used, or [fall, rise] comes out shifted by the
        # host-node offset and the strobe acceptance check - which intersects
        # both cameras' windows against one strobe instant - judges the wrong
        # window. One fleet offset for all of it, so the intersection is taken
        # in a single consistent domain.
        off_node = clk_off
        if source == "gpio_edge":
            rise = self.provider.match_rise(self.mon, epoch + off_node,
                                            (claim or {}).get("fire_seq"))
            if rise is not None:
                rise -= off_node
        strobe = (claim or {}).get("strobe_epoch")
        if strobe:
            strobe -= off_node
        if strobe and source == "gpio_edge" and rise \
                and not (epoch <= strobe <= rise):
            self.provider.events.emit(
                "warn", "strobe_miss",
                "%s: strobe fired %+.1f ms from this frame's exposure window "
                "[0..%.1f ms] - the frame is unlit or half-lit"
                % (fname, (strobe - epoch) * 1000, (rise - epoch) * 1000),
                node=self.mon.name_, frame=fname)
        self.provider.index_frame(self.cam_num, fname, orig, epoch, source,
                                  node=self.mon.name_,
                                  path=(claim or {}).get("path"),
                                  rise=rise, strobe=strobe,
                                  clk_off=applied_off)

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
                "dumped": self.dumped, "dump_fails": self.dump_fails,
                "last_capture": self._last_cap,
                "interval_mean_s": round(st.mean(iv), 3) if iv else None,
                "interval_jitter_ms": round(jit, 1) if jit else None}


def _r(v, n):
    return "" if v is None else round(v, n)


def _applied_span(span):
    """run.json's record of the offsets a run actually applied to its frames.

    [first, last, min, max] as collected by index_frame, rendered as nulls
    before any frame has landed. One live-read scalar could not describe it:
    the applied number moves across a transect (this host free-runs ~60 ppm
    and was seen to step 187 -> 69 ms), so a post-processor re-basing off a
    single figure reintroduces the run's own drift (audit 2026-08-24)."""
    span = list(span or [None, None, None, None])
    return {"first": _r6(span[0]), "last": _r6(span[1]),
            "min": _r6(span[2]), "max": _r6(span[3]),
            "moved_ms": (None if span[2] is None or span[3] is None
                         else _r6((span[3] - span[2]) * 1000.0, 2))}


def _r6(v, n=6):
    """round() for a JSON record: unknown stays null, never 0.000000.

    _r's "" is a CSV blank. In run.json an unmeasured node rendered as 0.0
    reads as "this node's clock agrees with the host exactly", which is a
    measurement nobody made (audit 2026-08-24)."""
    return None if v is None else round(v, n)


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
        # (offset, measured_at) for fleet_clock_offset(). Its own lock: the
        # pull workers read it while holding nothing, and taking the run lock
        # from a worker inverts the documented lock order.
        self._fleet_clock = None
        self._fleet_clock_lock = threading.Lock()
        self.draining = None       # node name while a card drain holds it
        self.active = None            # dict describing the current run
        self.workers = {}
        # node -> TRIGGER->EXPOSURE latency in seconds, measured not assumed.
        self.trig_latency = {}
        # Persisted per-body trigger latency. Each calibration fires five
        # frames per camera and ran at every rigd start AND every run start:
        # ~240 RAW+JPEG frames of non-survey data in three days, plus a FOCUS
        # hold (AE-lock) each time. The figure is a property of the body and
        # its lens and moved <0.5 ms across a week of measurements, so a value
        # measured within TRIG_LAT_MAX_AGE_S for the SAME camera id is reused.
        self._load_trig_latency()
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
        # node -> Lock. Only one calibration pass may hold a node at a time.
        self._cal_locks = {}
        self._known_frames = {}
        # Sticky per-node GPIO capability and the rate limiters for the alerts
        # that would otherwise fire once per shot at 2 fps.
        self._gpio_ok = {}
        self._warned_at = {}
        # Strobe (docs/strobe-trigger.md): a scheduled open-drain pulse on the
        # strobe node at T + delta, where T is the shot's shared target instant.
        # Persisted so a rigd restart mid-survey keeps the operator's choice.
        self.strobe = self._load_strobe()

    # ---- strobe config ------------------------------------------------------
    STROBE_DEFAULT = {"enabled": False, "node": "cam1",
                      "delta_ms": 10.0, "pulse_ms": 5}

    @staticmethod
    def _strobe_path():
        import rigcore
        return os.path.join(rigcore.RIG_HOME, "strobe.json")

    def _load_strobe(self):
        cfg = dict(self.STROBE_DEFAULT)
        try:
            with open(self._strobe_path()) as fh:
                saved = json.load(fh)
            cfg.update({k: saved[k] for k in cfg if k in saved})
        except (OSError, ValueError):
            pass
        return cfg

    def get_strobe(self):
        with self._lock:
            return dict(self.strobe)

    def set_strobe(self, changes):
        """Validate and persist the strobe configuration.

        delta_ms is bounded to the measured-safe window (§4.1: 8–12 ms clears
        curtain travel plus skew; the wide bound below is for bench work, and
        the acceptance check in the run browser is the real judge). Warnings —
        a shutter faster than 1/30 leaves the flash almost no margin — are
        returned for the UI to show, never silently enforced."""
        rep, warns = {}, []
        with self._lock:
            cfg = dict(self.strobe)
        if "enabled" in changes:
            rep["enabled"] = bool(changes["enabled"])
        if "node" in changes:
            names = [m.name_ for m in self.monitors]
            if changes["node"] not in names:
                return {"ok": False,
                        "error": "unknown strobe node %r (fleet: %s)"
                                 % (changes["node"], ", ".join(names))}
            rep["node"] = changes["node"]
        if "delta_ms" in changes:
            try:
                d = float(changes["delta_ms"])
            except (TypeError, ValueError):
                return {"ok": False, "error": "delta_ms is not a number"}
            if not (0.0 < d <= 100.0):
                return {"ok": False,
                        "error": "delta_ms %.1f is outside 0..100 ms" % d}
            if not (8.0 <= d <= 12.0):
                warns.append("delta_ms %.1f is outside the measured-safe "
                             "8-12 ms window (docs/strobe-trigger.md §4.1)" % d)
            rep["delta_ms"] = d
        if "pulse_ms" in changes:
            try:
                pm = int(changes["pulse_ms"])
            except (TypeError, ValueError):
                return {"ok": False, "error": "pulse_ms is not a number"}
            if not (1 <= pm <= 50):
                return {"ok": False,
                        "error": "pulse_ms %d is outside 1..50" % pm}
            rep["pulse_ms"] = pm
        cfg.update(rep)
        if cfg.get("enabled"):
            d = self.settings.get() if self.settings else {}
            sh = d.get("shutter")
            if sh:
                num, den = (sh >> 16) & 0xFFFF, sh & 0xFFFF
                dur_ms = (num / den * 1000.0) if den else 0
                if 0 < dur_ms < 33.0:
                    warns.append(
                        "shutter %s is faster than 1/30 - at delta %.0f ms the "
                        "flash may land after the first curtain closes; run "
                        "1/30 or slower (docs/strobe-trigger.md §4.1)"
                        % ("%d/%d" % (num, den), cfg["delta_ms"]))
        with self._lock:
            self.strobe = cfg
        try:
            import rigcore
            os.makedirs(rigcore.RIG_HOME, exist_ok=True)
            tmp = self._strobe_path() + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(cfg, fh, indent=2)
            os.replace(tmp, self._strobe_path())
        except OSError as e:
            warns.append("strobe config not persisted: %s" % e)
        if rep:
            self.events.emit("info", "strobe", "strobe config: %s"
                             % ", ".join("%s=%s" % kv for kv in cfg.items()),
                             **{k: v for k, v in cfg.items()})
        return {"ok": True, "strobe": cfg, "warnings": warns}

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

    # ---- host <-> node clock ----------------------------------------------
    # THE THIRD CLOCK. The two Pis are chrony-locked to each other (~0.6 ms)
    # but nothing disciplines this host to them: measured 2026-08-23, the Mac
    # was 187 ms behind NTP and drifting ~60 ppm. Every scheduled fire epoch,
    # every GPIO edge epoch_hw and every piagent /health time.epoch is on the
    # NODE clock; nav's ring, the GPS offset and time.time() are on the HOST
    # clock. Treating them as one domain cost two separate defects on the live
    # rig: the 0.30 s fire lead was overrun by the offset (both nodes reported
    # late_ms ~33 ms on EVERY fire and skew degraded 0.59 -> 1.78 ms), and nav
    # was looked up ~190 ms away from the true exposure - 19 cm at 1 m/s.
    # NodeMonitor.clock_offset_s() is the filtered node-minus-host offset.
    def _mon(self, node):
        return node if hasattr(node, "name_") else \
            next((x for x in self.monitors if x.name_ == node), None)

    def _clock_offset_raw(self, node):
        """That node's measured offset, or None when it has none.

        Distinct from node_clock_offset()'s 0.0: "unmeasured" must not be
        averaged into a fleet median as if the node agreed with this host,
        which on a rig 187 ms off its Pis would drag the whole conversion
        toward the wrong clock every time one node went quiet."""
        fn = getattr(self._mon(node), "clock_offset_s", None)
        if fn is None:
            return None
        try:
            o = fn()
        except Exception:  # noqa: BLE001
            return None
        return None if o is None else float(o)

    def node_clock_offset(self, node):
        """(node clock - host clock) in seconds for ONE node; 0.0 if unknown.

        Diagnostics only - run.json's per-node clock record, and the fleet
        median below. Nothing that stamps a capture instant may use it: see
        fleet_clock_offset() for why a per-node estimate must never reach a
        frame's epoch.

        Unknown is deliberately 0.0 and not an exception: a node that has not
        been polled yet, or an older rigcore without the filter, must degrade
        to the previous behaviour rather than stop the survey."""
        o = self._clock_offset_raw(node)
        return 0.0 if o is None else o

    def clock_err(self, node):
        """Uncertainty of that node's offset, in seconds: half the best RTT in
        the filter window. The offset is estimated from a request/response
        midpoint, so an asymmetric path can bias it by up to RTT/2 - that is
        the honest error bar to fold into a converted instant's time_err_ms."""
        fn = getattr(self._mon(node), "clock_offset_info", None)
        if fn is None:
            return 0.0
        try:
            rtt = (fn() or {}).get("rtt_ms_best")
        except Exception:  # noqa: BLE001
            return 0.0
        return (float(rtt) / 2000.0) if rtt else 0.0

    # A host clock this far from the nodes has already eaten the fire lead.
    CLOCK_WARN_S = 0.1
    # The correction is bounded. A wild offset is a broken clock, not a
    # measurement: applying it would schedule fires minutes away (piagent
    # refuses those anyway) or, negative and large, in the past.
    CLOCK_CLAMP = (-1.0, 5.0)

    def _median_clamped(self, offs):
        offs = sorted(offs)
        if not offs:
            return 0.0
        n = len(offs)
        med = offs[n // 2] if n % 2 else (offs[n // 2 - 1] + offs[n // 2]) / 2.0
        return max(self.CLOCK_CLAMP[0], min(self.CLOCK_CLAMP[1], med))

    def common_clock_offset(self, mons):
        """ONE node-minus-host offset for a shot, across its live members.

        The median, never per-node. Firing each camera against its own offset
        estimate would inject that estimate's noise straight into the
        inter-camera skew - the single number this rig exists to keep under
        10 ms - and the offsets are common-mode anyway (the Pis are chrony
        peers; it is the host that has drifted). One shared value moves both
        exposures together and cancels out of the skew entirely."""
        return self._median_clamped(self.node_clock_offset(m) for m in mons)

    # How long one fleet-offset latch is reused. The underlying figure is
    # already an RTT-gated median over a 60 s window of node samples, so it
    # cannot move meaningfully faster than this; the latch exists so every
    # frame of ONE shot converts with the SAME number even though the two
    # cameras' frames are pulled by two independent worker threads.
    FLEET_CLOCK_TTL_S = 2.0

    def _fleet_clock_members(self):
        """The monitors the fleet offset is measured across.

        This run's members while a run is open (that is the set the fire
        schedule averages), the whole fleet otherwise - and in both cases only
        the nodes that actually have a measurement, because an unmeasured node
        reads 0.0 and would pull the median toward this host's own clock."""
        run = self.active
        names = set((run or {}).get("nodes") or ())
        offs = [(m, self._clock_offset_raw(m)) for m in self.monitors
                if m.is_present()]
        mine = [o for m, o in offs
                if o is not None and (not names or m.name_ in names)]
        return mine or [o for _, o in offs if o is not None]

    def fleet_clock_offset(self, mons=None):
        """THE node-minus-host offset every host<->fleet conversion uses.

        One number for the whole fleet, latched for FLEET_CLOCK_TTL_S, and the
        same median the fire schedule targets with. Per-node offsets are the
        trap: RunBrowser._pairs and the UI's jitter chip compute a pair's
        inter-camera spread FROM the indexed capture epochs, so converting each
        camera's frame with its OWN estimate makes the DISPLAYED skew carry
        (offset_cam1 - offset_cam2) - the difference of two noisy estimates,
        each worth about half that node's RTT (cam1 ~2.9 ms, cam2 ~10.3 ms) -
        against a true skew of ~0.6 ms and a 10 ms budget. The docstring on
        common_clock_offset() already made this argument for the fire schedule;
        it holds with equal force for the instants those fires produce, and for
        the strobe acceptance check, which intersects the two cameras'
        [fall, rise] windows against one strobe instant. The offsets are
        common-mode anyway - the Pis are chrony peers, it is the host that has
        drifted - so one shared value cancels out of every inter-camera figure
        exactly, and leaves only the (real, reported) host-domain shift.

        Pass `mons` from the fire path to LATCH the shot's own value. That
        latch is keyed on WALL TIME, which is not shot identity: capture_once
        carries the latched number on each node's command record and
        _capture_instant prefers it, so a scheduled frame converts with its own
        shot's offset however late it is pulled. This TTL is the fallback for
        frames that have no command at all - an unscheduled release, a card
        review, a pre-adoption backlog."""
        now = time.time()
        if mons is None:
            with self._fleet_clock_lock:
                c = self._fleet_clock
                if c is not None and now - c[1] < self.FLEET_CLOCK_TTL_S:
                    return c[0]
            offs = self._fleet_clock_members()
        else:
            offs = [o for o in (self._clock_offset_raw(m) for m in mons)
                    if o is not None]
            offs = offs or self._fleet_clock_members()
        if not offs:
            # Nothing measured anywhere - every node just dropped, or none has
            # been polled yet. Hold the last latch rather than publishing 0.0:
            # "no measurement" is not "the host agrees with the fleet", and
            # latching 0.0 would step every instant of the next two seconds by
            # the host's whole error (187 ms on this rig).
            with self._fleet_clock_lock:
                return self._fleet_clock[0] if self._fleet_clock else 0.0
        off = self._median_clamped(offs)
        with self._fleet_clock_lock:
            self._fleet_clock = (off, now)
        return off

    def fleet_clock_err(self):
        """The error bar on that ONE offset, in seconds.

        The applied number is a median over the fleet, so it carries the
        uncertainty of the worst contributor - reporting cam1's row as
        +/-1.4 ms when the offset actually applied was pulled by cam2's
        +/-5.2 ms estimate would be a smaller number than the truth. Both
        halves of a pair get the same bar, which is what a shared conversion
        means."""
        run = self.active
        names = set((run or {}).get("nodes") or ())
        errs = [self.clock_err(m) for m in self.monitors
                if (not names or m.name_ in names)
                and self._clock_offset_raw(m) is not None]
        return max(errs) if errs else 0.0

    def _warn_host_clock(self, offset):
        """Say once per run that this host's clock is eating the fire lead."""
        if abs(offset) <= self.CLOCK_WARN_S:
            return
        run = self.active
        key = "hostclock:%s" % ((run or {}).get("run_id") or "-")
        if not self.warn_once(key, 0):
            return
        self.events.emit(
            "warn", "host_clock",
            "host clock is %.0f ms off the nodes - the fire schedule is being "
            "corrected for it, but enable network time on this host (the Pis "
            "are chrony-locked to each other and nothing disciplines the Mac). "
            "Frame timestamps are converted to the host domain; nav and the "
            "GPS offset stay comparable" % (offset * 1000.0),
            offset_ms=round(offset * 1000.0, 1))

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
    # A period this short guarantees a 409 on every other fire: piagent holds
    # its fire lock from FOCUS_LEAD_MS before the target until the pulse ends,
    # so two shots closer together than that overlap inside the node. Refused.
    # Properties, not plain constants: SYNC_LEAD_S and FOCUS_LEAD_MS are
    # declared further down the class body (with the measured skew curve that
    # justifies them), so they do not exist yet at this point in it.
    @property
    def MIN_INTERVAL_S(self):
        return self.FOCUS_LEAD_MS / 1000.0 + 0.08

    # Below this there is no dispatch margin left - the request has to reach
    # the node before (target - FOCUS lead - trigger latency) - so the grid
    # runs on arrival time instead of on the schedule. Warned, not refused:
    # bench work at 0.4 s is legitimate and the operator is told what it costs.
    @property
    def SAFE_INTERVAL_S(self):
        return self.SYNC_LEAD_S + 0.2

    MAX_INTERVAL_S = 3600.0
    MAX_FRAMES = 100000

    @staticmethod
    def _clean_label(v):
        """A label that can safely become a run_id, a directory and a CSV cell.

        Control characters and newlines would break the events.log and the run
        browser's own listing; _slug() then reduces whatever survives to
        [A-Za-z0-9-_] for the directory name."""
        lbl = "".join(c for c in str(v if v is not None else "")
                      if c.isprintable()).strip()
        return lbl[:60] or "transect"

    def _validate_config(self, config):
        """(normalised config, error). Refuse a run that cannot work.

        start() used to store the client's dict verbatim: /api/run/start with
        interval_s 0 spun the capture loop flat out with no sleep, spawning a
        thread per iteration and flooding both piagents with fires that 409'd
        each other, and a negative value did the same (audit 2026-08-23, R7).
        The UI's own iv>0 check was the only guard anywhere."""
        cfg = dict(config or {})
        cfg["label"] = self._clean_label(cfg.get("label"))
        warns = []
        try:
            frames = int(cfg.get("frames", 0) or 0)
        except (TypeError, ValueError):
            return None, "frames is not a number", warns
        if not (0 <= frames <= self.MAX_FRAMES):
            return None, ("frames %d is outside 0..%d (0 means 'until stopped')"
                          % (frames, self.MAX_FRAMES)), warns
        cfg["frames"] = frames
        if cfg.get("auto_capture"):
            try:
                iv = float(cfg.get("interval_s", 2.0))
            except (TypeError, ValueError):
                return None, "interval_s is not a number", warns
            if iv != iv or iv in (float("inf"), float("-inf")):
                return None, "interval_s is not a finite number", warns
            if iv < self.MIN_INTERVAL_S:
                return None, ("interval_s %.3f is below the minimum %.2f s - "
                              "the node holds its fire lock from %.0f ms before "
                              "each target, so shots closer than that refuse "
                              "each other" % (iv, self.MIN_INTERVAL_S,
                                              self.FOCUS_LEAD_MS)), warns
            if iv > self.MAX_INTERVAL_S:
                return None, ("interval_s %.1f is above the maximum %.0f s"
                              % (iv, self.MAX_INTERVAL_S)), warns
            if iv < self.SAFE_INTERVAL_S:
                warns.append(
                    "interval_s %.2f is below the %.2f s that leaves the fire "
                    "schedule its dispatch margin (SYNC_LEAD_S + 0.2); shots "
                    "may run on arrival time rather than on the grid"
                    % (iv, self.SAFE_INTERVAL_S))
            cfg["interval_s"] = iv
        return cfg, None, warns

    def start(self, config):
        cfg, err, warns = self._validate_config(config)
        if err:
            return {"ok": False, "error": err}
        config = cfg
        with self._lock:
            if self.active:
                return {"ok": False, "error": "run already active",
                        "run_id": self.active["run_id"]}
            if getattr(self, "draining", None):
                return {"ok": False, "error": "card drain in progress on %s - "
                        "cannot start a run (the camera is in transfer mode)"
                        % self.draining}
            monitors = list(self.monitors)
        # Belt and braces beyond the draining flag: refuse if ANY connected
        # node's body actually reports transfer control mode. A LIVE probe, not
        # the ≤2 s-stale monitor cache: firing a transect into a transfer-mode
        # body records nothing, so a fresh check is worth one HTTP per node at
        # start (audit 2026-08-23, critical). Outside the lock - it does I/O.
        stuck = []
        for m in monitors:
            st = http_json("http://%s:8080/api/status" % m.host, timeout=5)
            if isinstance(st, dict) and st.get("controlMode") == "transfer":
                stuck.append(m.name_)
        with self._lock:
            # Re-check under the lock: the probe released it, so a run or a
            # drain could have started in the gap.
            if self.active:
                return {"ok": False, "error": "run already active",
                        "run_id": self.active["run_id"]}
            if getattr(self, "draining", None):
                return {"ok": False, "error": "card drain in progress on %s"
                        % self.draining}
            if stuck:
                return {"ok": False, "error": "%s in transfer mode (card "
                        "drain / stuck session) - cannot shoot; wait for the "
                        "drain or power-cycle" % ",".join(stuck)}
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
            live = [m for m in self.monitors
                    if m.is_connected() and m.is_capturing()]
            # run.json keeps only the LAST 2000 index entries, so a long
            # transect loses its head - and ingest and stereo_check read that
            # index to match card RAWs to frames. index.jsonl is the complete
            # record: one JSON object per indexed frame, appended and flushed
            # as the frame is written, so it survives a kill as well as a long
            # line (audit 2026-08-23, contract C3).
            try:
                index_fh = open(os.path.join(root, "index.jsonl"), "a")
            except OSError:
                index_fh = None
            self._cap_stop = threading.Event()
            run = {"run_id": rid, "root": root, "label": label,
                   "started": now, "config": config,
                   "time_off": time_off, "time_src": time_src,
                   "nodes": [m.name_ for m in live],
                   "events_fh": events_fh, "nmea_fh": nmea_fh,
                   "index_fh": index_fh,
                   # The capture loop's stop event, latched on the run it
                   # belongs to: a calibration thread that finishes after this
                   # run has been stopped must not arm the NEXT one (R5).
                   "cap_stop": self._cap_stop,
                   "index": [], "alerts": [], "fired": {}, "orphans": {},
                   "skews_ms": [], "late_shots": 0, "failed_fires": 0,
                   "unpaired_shots": 0, "fail_streak": {}, "paused_for": None,
                   # [first, last, min, max] of the fleet clock offset actually
                   # applied to a frame in this run. The applied number moves
                   # (the host free-runs and can step), so run.json's single
                   # scalar could not describe the transect it claimed to.
                   "clk_applied": [None, None, None, None]}
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
            if not PIL_AVAILABLE:
                self.events.emit(
                    "warn", "run",
                    "Pillow is not installed, so the EXIF fallback for a frame's "
                    "capture instant is unavailable: any frame without a GPIO "
                    "EXPOSURE edge will be stamped with the command time instead "
                    "of the camera's own clock. Install python3-pil on this host.")
            # Calibration and the survey capture loop are NOT concurrent. Run
            # start fires ~6 non-survey exposures per camera (one for the EXIF
            # clock offset, five for trigger latency); with the capture loop
            # already running those interleave with real survey frames, steal
            # their queued commands and their EXPOSURE edges, and the bulk
            # re-baseline that used to follow marked whatever had landed in the
            # meantime as "already seen" - deleting real survey frames from the
            # transect. One ordered pass, then the loop.
            self._calib_stop = threading.Event()
            self._calib_thread = threading.Thread(
                target=self._calibrate_and_arm, args=(live, config, run),
                daemon=True)
            self._calib_thread.start()
            # Keep watching for cameras that were not up at start.
            self._adopt_stop = threading.Event()
            self._adopt_thread = threading.Thread(target=self._adopt_loop,
                                                  daemon=True)
            self._adopt_thread.start()
            for w in warns:
                self.events.emit("warn", "run", w, run_id=rid)
            return {"ok": True, "run_id": rid, "nodes": run["nodes"],
                    "root": root, "time_source": time_src,
                    "gps_offset_s": round(time_off, 3),
                    "warnings": warns,
                    "host_offset_s": round(self.fleet_clock_offset(live), 6)}

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
        os.makedirs(rigcore.RUNS_DIR, exist_ok=True)
        for n in range(0, 27):
            rid = base if n == 0 else "%s_%s" % (base, chr(ord("a") + n))
            root = os.path.join(rigcore.RUNS_DIR, rid)
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
        root = os.path.join(rigcore.RUNS_DIR, rid)
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
                # is_capturing(), not just is_connected(): a camera the
                # operator switched off is perfectly connected - the Pi and
                # ilxctl are fine, only the camera is out of play - so an
                # is_connected() test here adopts it back within 3 s and
                # silently undoes the roster filter at run start. A camera
                # switched back ON mid-run is still adopted through this same
                # path, which is the behaviour you want.
                missing = [m for m in self.monitors
                           if m.is_connected() and m.is_capturing()
                           and m.name_ not in self.workers]
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
        # Keyed by RUN, not by process. _warned_at is process-lifetime state
        # and start() never cleared it, so only the FIRST transect of the day
        # whose master clock moved mid-line got the alert; every later one was
        # silently rate-limited out (audit 2026-08-23, R7).
        if not self.warn_once("timebase:%s" % run["run_id"], 0):
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
            self._stop_capture_loop()          # signals _cap_stop only
            if self._calib_stop:
                # A stopped run must not keep firing calibration frames into a
                # closed transect, and a calibration polling a node that has
                # gone slow must not outlive the run that started it.
                self._calib_stop.set()
            self._calib_stop = None
            if self._adopt_stop:
                self._adopt_stop.set()
            cap_thread = self._cap_thread
        # Join the capture LOOP before snapshotting `fired`: _stop_capture_loop
        # only SETS the event, and a capture_once already dispatched but not
        # yet counted would otherwise be missed by the fired snapshot and the
        # grace wait below, dropping the final shot of the transect silently
        # (audit 2026-08-23). Join outside the lock - the loop's in-flight
        # capture threads call back into index_frame(), which takes it.
        if cap_thread is not None:
            cap_thread.join(timeout=6.0)
        with self._lock:
            workers = dict(self.workers)
            fired = dict((self.active.get("fired") or {}))
        # A fire's frames take ~0.5-1.5 s to reach the node's spool (card
        # write + PC-save + the 0.4 s listing poll). Stopping the workers the
        # instant the operator lets go raced that pipeline and dropped the
        # FINAL shot of every transect, deterministically (measured
        # 2026-08-20: 141 fired / 140 archived, 337/336). A fire committed
        # before stop belongs to the transect: no new fires can start now, so
        # give the workers a bounded grace to catch up before telling them to
        # stop. Outside the lock — indexing a frame takes it.
        deadline = time.time() + 6.0
        lag = [n for n, w in workers.items()
               if w.stats().get("pulled", 0) < fired.get(n, 0)]
        while lag and time.time() < deadline:
            time.sleep(0.2)
            lag = [n for n, w in workers.items()
                   if w.stats().get("pulled", 0) < fired.get(n, 0)]
        if lag:
            self.events.emit(
                "warn", "run",
                "stopping with fired frames still undelivered from %s "
                "after 6 s grace - check the node spool" % ",".join(lag),
                nodes=lag)
        with self._lock:
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
            # The counts go in the MESSAGE, not only the payload: the journal
            # line is what the operator (and the Diagnostics log) reads, and
            # "run stopped: <id>" answered the question everyone actually has
            # - how many frames did each camera deliver, and did any fail -
            # with silence.
            per_cam = ", ".join(
                "%s %d image%s%s" % (
                    n.capitalize(), s.get("pulled", 0),
                    "" if s.get("pulled", 0) == 1 else "s",
                    (" (%d FAILED)" % s["failed"]) if s.get("failed") else "")
                for n, s in sorted(summary.items()))
            self.events.emit("info", "run",
                             "run stopped: %s - %s" % (rid, per_cam or "no cameras"),
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
                if run.get("index_fh") is not None:
                    run["index_fh"].close()
            except OSError:
                pass
            self.active = None
            self.workers = {}
            return {"ok": True, "run_id": rid, "summary": summary}

    # ---- capture ----------------------------------------------------------
    # How far ahead the shared fire instant is scheduled. The request must
    # reach the node BEFORE (target − FOCUS_LEAD_MS − that node's trigger
    # latency): piagent wakes early to place the FOCUS lead, and a late
    # arrival does not shorten the lead — the TRIGGER lands late by the full
    # overshoot, which is direct inter-camera skew. At FOCUS_LEAD_MS=120 and
    # ~22 ms latency, 0.15 s left ~8 ms for dispatch + JSON + TCP; 0.30 s
    # restores the ~150 ms margin the original figure had at the 40 ms lead.
    SYNC_LEAD_S = 0.30
    # An EXPOSURE edge older than this cannot belong to a frame arriving now.
    EDGE_MAX_AGE_S = 30.0
    # What an edge's `epoch` is worth when the node could NOT stamp it in
    # hardware. piagent publishes epoch_hw (the kernel interrupt instant) and,
    # when it has none, `hw_reject` saying why; the remaining `epoch` is
    # Python's stamp after the gpiomon pipe read, whose latency is a measured
    # 0.09 ms (cam1) / 0.32 ms (cam2) median WITH excursions into the hundreds
    # of ms under load - and nothing measures which one this edge got. So the
    # honest bar is the excursion scale, never the fleet clock error alone.
    # Writing such an edge as capture_source=gpio_edge with time_err_ms ~= the
    # clock error is the residual half of the 2026-08-24 epoch_hw blocker: the
    # node stopped throwing good stamps away, but the host still dressed a
    # software stamp as a hardware one (audit 2026-08-24, VERIFY).
    EDGE_EPOCH_UNMEASURED_S = 0.25
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
        run = self.active
        members = list((run or {}).get("nodes") or ())
        live = [m for m in self.monitors
                if m.is_connected() and m.is_capturing()]
        # EVERY run member is judged, not just the ones we can reach. A member
        # that has dropped to not-connected (body off USB, ilxctl wedged, the
        # Pi rebooted on the shared PoE feed) used to be silently absent from
        # `live`: it produced no result, so it never accumulated a failed fire,
        # fail_streak never reached 3, the pause never engaged, and the other
        # camera shot the rest of the transect alone - 28 single-camera frames
        # reported as unpaired_shots:1 (devrig, 2026-08-23). An unreachable
        # member is a FAILED FIRE. It is not contacted: there is nothing to
        # contact, and a doomed HTTP round trip per shot is what piled the
        # backlog up in the first place.
        # A camera the operator switched off is NOT a failed fire: it is not
        # contacted and it is not judged. Without this it lands in `offline`
        # below, accumulates a failed fire every shot, drives fail_streak to 3
        # and pauses the whole run - the rig stopping itself because it was
        # told not to use a camera.
        _off_by_switch = {m.name_ for m in self.monitors if not m.is_capturing()}
        members = [n for n in members if n not in _off_by_switch]
        offline = [n for n in members
                   if n not in {m.name_ for m in live}]
        if not live and not offline:
            return {"ok": False, "error": "no cameras connected"}
        results = {n: {"path": None, "ok": False, "error": "node offline"}
                   for n in offline}
        threads = []
        # The target is a NODE-clock instant. piagent busy-waits to it against
        # the node's own clock, and this host's clock is not disciplined to
        # theirs (187 ms behind, 2026-08-23), so a host-clock target arrived
        # already 187 ms into its own 300 ms lead: both nodes reported
        # late_ms ~33 ms on every fire and the realised skew degraded from
        # 0.59 to 1.78 ms. One shared offset for the whole shot - never
        # per-node - so it cancels out of the inter-camera skew.
        # Latch this shot's fleet offset and CARRY IT ON THE COMMAND: the
        # wall-time latch (FLEET_CLOCK_TTL_S) is not shot identity, and the two
        # halves of one pair are converted by two independent worker threads
        # whose polls and downloads do not line up - a fire boundary landing
        # between them converted cam1 with this shot's number and cam2 with the
        # next shot's, and that difference is exactly the pair spread the run
        # browser displays. Carried on the record, both halves of a pair get
        # the identical float by construction (audit 2026-08-24).
        host_offset = self.fleet_clock_offset(live)
        clock_err = self.fleet_clock_err()
        self._warn_host_clock(host_offset)
        if target is None:
            target = time.time() + self.SYNC_LEAD_S + host_offset
        else:
            target = target + host_offset

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
                rec = (w.note_command(target, path="gpio",
                                      host_offset=host_offset,
                                      clock_err=clock_err) if w else None)
                body = {"at_epoch": target - lead, "pulse_ms": 5,
                        "focus_lead_ms": self.FOCUS_LEAD_MS}
                strobe = self.get_strobe()
                if strobe.get("enabled") and m.name_ == strobe.get("node"):
                    # The flash is placed against the SHARED target instant, so
                    # it lands relative to both bodies' exposures, not this
                    # one's curtain (docs/strobe-trigger.md §4, topology A).
                    body["strobe_at_epoch"] = target + strobe["delta_ms"] / 1e3
                    body["strobe_pulse_ms"] = strobe.get("pulse_ms", 5)
                # A fire answers within lead + FOCUS lead + a few ms. Waiting
                # 10 s on a dead node held each shot thread open for 10 s,
                # piled the backlog to 9+ in-flight shots and let the other
                # camera shoot alone for a whole transect (cam1 power loss,
                # 2026-08-23). Fail fast instead; the loop pauses on a streak.
                r = http_json("http://%s:8081/gpio/fire" % m.host, body,
                              timeout=max(2.0, self.SYNC_LEAD_S
                                          + self.FOCUS_LEAD_MS / 1000.0 + 1.5))
                ok = bool(r.get("ok"))
                actual = r.get("actual_epoch")
                if body.get("strobe_at_epoch") and ok:
                    if r.get("strobe_error"):
                        if self.warn_once("strobe_err:%s" % m.name_, 30):
                            self.events.emit(
                                "warn", "strobe_fail",
                                "strobe scheduled on %s but did not pulse: %s"
                                % (m.name_, r["strobe_error"]), node=m.name_)
                    elif (r.get("strobe_late_ms") or 0) > 5.0:
                        self.events.emit(
                            "warn", "strobe_late",
                            "strobe on %s pulsed %.1f ms late - the flash may "
                            "have missed the exposure window"
                            % (m.name_, r["strobe_late_ms"]), node=m.name_,
                            late_ms=r["strobe_late_ms"])
                if ok and w and rec is not None:
                    # piagent hands back the identity of this fire and the edge
                    # cursor it started from, so the frame can be paired with
                    # ITS OWN exposure edge instead of the nearest one.
                    w.update_command(rec, fire_seq=r.get("fire_seq"),
                                     edge_seq=r.get("edge_seq"),
                                     strobe_epoch=r.get("strobe_epoch"))
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
                # arrival and its frames carry the usual USB uncertainty. The
                # queued instant is still recorded on the NODE clock, like
                # every other command epoch, so _capture_instant can compare it
                # against edges and EXIF without mixing domains.
                t = time.time() + host_offset
                rec = (w.note_command(t, path="usb",
                                      host_offset=host_offset,
                                      clock_err=clock_err) if w else None)
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
        # The survey's light must not depend on the strobe node's CAMERA: if
        # that body is faulted (the 2026-08-16 card fault) or harness-less,
        # its healthy piagent still hosts the flash. Pulse it standalone so
        # every other camera's frames stay lit.
        strobe = self.get_strobe()
        if strobe.get("enabled"):
            sm = next((m for m in self.monitors
                       if m.name_ == strobe.get("node")), None)
            fired_with_strobe = (sm is not None and sm in live
                                 and self._fire_path(sm) == "gpio")
            # A strobe node that is fully OFFLINE (neither ilxctl nor piagent
            # answering) has no flash to fire. Calling it anyway blocked every
            # capture_once for the full timeout, piled shot threads past the
            # backlog guard and defeated stop()'s 6 s barrier - the camera fire
            # path was cut to a fail-fast timeout for exactly this reason and
            # the strobe-only call kept a 10 s one (audit 2026-08-23, R10).
            if sm is not None and not fired_with_strobe \
                    and getattr(sm, "state", None) == rigcore.NodeMonitor.OFFLINE:
                if self.warn_once("strobe_alone:%s" % sm.name_, 30):
                    self.events.emit(
                        "warn", "strobe_fail",
                        "strobe node %s is offline - this shot is unlit; the "
                        "flash is not being called" % sm.name_, node=sm.name_)
                sm = None
            if sm is not None and not fired_with_strobe:
                def _strobe_alone():
                    r = http_json("http://%s:8081/gpio/strobe" % sm.host,
                                  {"at_epoch":
                                   target + strobe["delta_ms"] / 1e3,
                                   "pulse_ms": strobe.get("pulse_ms", 5)},
                                  # Same fail-fast budget as /gpio/fire: the
                                  # pulse is scheduled, so anything past the
                                  # lead plus the delta is a dead host.
                                  timeout=max(2.0, self.SYNC_LEAD_S
                                              + strobe["delta_ms"] / 1e3 + 1.5))
                    if not (isinstance(r, dict) and r.get("ok")):
                        if self.warn_once("strobe_alone:%s" % sm.name_, 30):
                            self.events.emit(
                                "warn", "strobe_fail",
                                "standalone strobe on %s failed: %s"
                                % (sm.name_,
                                   (r or {}).get("error", "unreachable")),
                                node=sm.name_)
                    else:
                        # Kept OUT of `results`: _judge_shot scores camera
                        # fires, and a phantom "_strobe" node would pollute
                        # the run's fired/failed counts.
                        self.last_standalone_strobe = {
                            "node": sm.name_, "at": time.time(),
                            "strobe_epoch": r.get("strobe_epoch")}
                th = threading.Thread(target=_strobe_alone, daemon=True)
                th.start(); threads.append(th)
        for th in threads:
            th.join(timeout=35)
        rep = self._judge_shot(target, results, roster=members)
        # What was actually applied, so a transect can be re-based in post and
        # an operator can see the correction rather than guess at it.
        rep["host_offset_s"] = round(host_offset, 6)
        return rep

    def _judge_shot(self, target, results, roster=None):
        """Score one shot: who fired, how late, and how far apart they exposed.

        `roster` is the run's full member list. A shot is judged against the
        members the transect was started with, not against whoever happened to
        answer: a member missing from `results` altogether is still a member
        that did not expose, and the shot is still not a pair."""
        failed = {k: v.get("error") or "no response"
                  for k, v in results.items() if not v.get("ok")}
        fired = [k for k, v in results.items() if v.get("ok")]
        expected = set(roster or ()) | set(results)
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
                if fired and len(fired) < len(expected):
                    # Some members exposed and some did not: this shot can
                    # never be a pair. Counted against the ROSTER, so a member
                    # that produced no result at all (offline, never contacted)
                    # is counted too - the old test needed a `failed` entry,
                    # which a node that had dropped out of `live` never
                    # produced, and 28 single-camera shots were reported as
                    # unpaired_shots:1 (devrig, 2026-08-23).
                    run["unpaired_shots"] += 1
                for n in failed:
                    run["fail_streak"][n] = run["fail_streak"].get(n, 0) + 1
                for n in fired:
                    run["fail_streak"][n] = 0
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

    # Resume probing after a pause: 2 s, then doubling to a minute. A node
    # that answers its health poll but refuses every fire must not be retried
    # at shot rate - see _probe_fire.
    RESUME_BACKOFF_S = (2.0, 60.0)

    def _probe_fire(self, m):
        """Fire ONE node on its own to prove it can still expose.

        (ok, error). The resume test used to be "has /health answered since the
        pause". A node whose piagent answers polls but refuses every fire - a
        lost gpiod line, a stray /gpio/interval holding the fire lock, a body
        that will not release - therefore resumed on EVERY poll, fired the
        healthy camera three more times alone, hit the streak again and paused
        again: a livelock that shot ~75% of the grid unpaired (audit
        2026-08-23, R1). Health answering is necessary, not sufficient; the
        streak is forgiven only by a fire that actually worked.

        The probe fires the RETURNING node alone, never its partner, and its
        frame is registered as a calibration exposure so this diagnostic can
        never enter the transect as survey data."""
        self.begin_calibration_fire(m.name_)
        try:
            before = self.spool_names(m)
            if before is None:
                # No baseline, so the frame this probe is about to produce
                # could not be told apart from a survey frame that happens to
                # land beside it. Refuse the probe rather than fire blind: the
                # node simply stays paused and is probed again on the next
                # backoff step.
                self.events.emit(
                    "warn", "capture_fail",
                    "%s did not answer its shot listing - the resume probe is "
                    "skipped: without a before-listing its own frame cannot be "
                    "kept out of the transect" % m.name_, node=m.name_)
                return False, "shot listing failed"
            t_fire = time.time()
            if self._fire_path(m) == "gpio":
                r = http_json("http://%s:8081/gpio/fire" % m.host,
                              {"at_epoch": time.time() + self.SYNC_LEAD_S
                               + self.common_clock_offset([m]),
                               "pulse_ms": 5,
                               "focus_lead_ms": self.FOCUS_LEAD_MS},
                              timeout=max(2.0, self.SYNC_LEAD_S
                                          + self.FOCUS_LEAD_MS / 1000.0 + 1.5))
                ok = bool(isinstance(r, dict) and r.get("ok"))
            else:
                # No harness on this node, so there is no /gpio/fire to probe
                # with. Probe the path it would actually be fired on, or a
                # USB-path camera could never clear its streak and the grid
                # would stay paused for the rest of the line.
                r = m.shutter(af=False)
                ok = (r or {}).get("ok", True) is not False
            # A fire whose ANSWER was lost may still have released the shutter.
            # piagent's own comment names the case ("abandoned by the host's
            # 2 s timeout"), and the whole quarantine used to hang off `ok`:
            # the diagnostic exposure then landed after end_calibration_fire
            # had already let the puller back on the node, was never named, and
            # entered the transect as survey data with a real gpio_edge instant
            # (audit 2026-08-24). Two things separate "may have fired" from
            # "provably did not": the transport failed (an explicit refusal -
            # 409 busy, "trigger line unavailable" - is piagent telling us it
            # unclaimed the line), and enough time passed for the release,
            # since the answer is only built after the pulse. ECONNREFUSED
            # comes back in milliseconds and is left alone.
            fired_maybe = (not ok and isinstance(r, dict)
                           and bool(r.get("_unreachable"))
                           and time.time() - t_fire >= self.SYNC_LEAD_S)
            if ok or fired_maybe:
                # Name the frame it produced before letting the puller back on
                # the node, exactly as calibrate_trigger does.
                named = self._name_probe_frame(m, before)
                if fired_maybe:
                    self.events.emit(
                        "warn", "calibrate",
                        "%s's resume probe was abandoned by its timeout (%s) "
                        "but may still have released the shutter; %s is kept "
                        "out of the transect as a probe exposure - it is still "
                        "on the node and on the card"
                        % (m.name_, (r or {}).get("error"),
                           ", ".join(named) if named else "no new frame"),
                        node=m.name_, frames=named)
                # The pulse is not the proof - the FRAME is. piagent answers
                # ok:true whenever the trigger line was asserted, with no idea
                # whether a camera exposed (its own contract says so), so a
                # body that dropped off USB while its Pi stayed healthy passed
                # every probe, resumed, failed three fires, re-paused - an
                # oscillation that shot ~40% of the healthy camera's frames
                # unpaired (audit 2026-08-27). A resume is earned only by the
                # probe's own frame arriving in the spool: camera, exposure
                # and delivery proven in one observation.
                if ok and not named:
                    ok = False
                    r = {"error": "the fire was accepted but no frame "
                                  "appeared - the camera is not exposing or "
                                  "not delivering"}
            return ok, (r or {}).get("error")
        except Exception as e:  # noqa: BLE001
            return False, str(e)
        finally:
            self.end_calibration_fire(m.name_)

    def _name_probe_frame(self, m, before):
        """Wait for the probe's own frame and name it; returns those names.

        The hold is REFRESHED on every unsuccessful pass, exactly as
        _calibrate_trigger_node does. begin_calibration_fire stores an ABSOLUTE
        expiry, and _probe_fire takes its hold before the before-listing and
        the fire, both of which block - so the hold expired (listing + lead)
        early, the puller went back on the node, claimed the probe frame first,
        and note_calibration_frames could no longer name it (its recent_names
        filter deliberately excludes anything the worker is already handling)
        (audit 2026-08-24)."""
        deadline = time.time() + self.CAL_QUIET_S
        while time.time() < deadline:
            time.sleep(0.05)
            after = self.spool_names(m)
            if after is not None:
                named = self.note_calibration_frames(m.name_, before, after)
                if named:
                    return named
            self.begin_calibration_fire(m.name_, hold_s=1.0)
        after = self.spool_names(m)
        if after is None:
            self.events.emit(
                "warn", "calibrate",
                "%s stopped answering its shot listing during the resume probe "
                "- the probe frame is unnamed and may enter the transect as "
                "survey data" % m.name_, node=m.name_)
            return []
        named = self.note_calibration_frames(m.name_, before, after)
        if not named:
            # The listing worked and showed nothing new. Either the probe
            # exposed nothing (the interesting diagnostic) or its frame was
            # already claimed by the puller, in which case it is in the
            # transect and nobody said so.
            self.events.emit(
                "warn", "calibrate",
                "%s's resume probe produced no new frame within %.1fs - if it "
                "did expose, that frame is unaccounted for and may be in the "
                "transect as survey data" % (m.name_, self.CAL_QUIET_S),
                node=m.name_)
        return named

    def _start_capture_loop(self, config, run_at_entry=None):
        period = float(config.get("interval_s", 2.0))
        count = int(config.get("frames", 0))
        with self._lock:
            run = self.active
            # A calibration thread reaches its finally block long after the run
            # that started it may have been stopped - and possibly after the
            # NEXT run has started. Arming that run with THIS run's interval,
            # frame budget and a stop event it does not own launched a second
            # capture loop on somebody else's transect (audit 2026-08-23, R5).
            # The loop is armed for the run it was calibrated for, or not at
            # all.
            if run_at_entry is not None and run is not run_at_entry:
                self.events.emit(
                    "info", "capture",
                    "calibration finished after its run was stopped - not "
                    "arming a capture loop (run %s is no longer active)"
                    % (run_at_entry or {}).get("run_id"))
                return
            stopev = (run or {}).get("cap_stop") or self._cap_stop
        if stopev is None or stopev.is_set() or not run:
            return                       # the run was stopped during calibration

        def _loop():
            k = 0
            g = 0
            start = time.time() + 1.0
            inflight = []
            while not stopev.is_set():
                if count and k >= count:
                    break
                # A node that has failed 3 fires in a row is not "slow", it is
                # gone (power loss, link down). Firing the other camera alone
                # produces unpaired frames that are not a transect. Pause the
                # grid, say so once, and resume by itself the moment the node
                # answers its health poll again.
                dead = self._dead_fire_node()
                if dead:
                    m = next((x for x in self.monitors if x.name_ == dead), None)
                    with self._lock:
                        run = self.active
                        if not run:
                            break
                        pf = run["paused_for"]
                        fresh = not pf or pf.get("node") != dead
                        if fresh:
                            # If this same node resumed only moments ago, the
                            # resume was evidently premature: inherit doubled
                            # backoff instead of restarting at the floor, so a
                            # flapping node converges to the 60 s ceiling
                            # instead of oscillating at the 2 s floor.
                            lr = run.get("last_resume") or {}
                            b0 = self.RESUME_BACKOFF_S[0]
                            if lr.get("node") == dead and                                     time.time() - lr.get("at", 0) < 120.0:
                                b0 = min(lr.get("backoff_s", b0) * 2,
                                         self.RESUME_BACKOFF_S[1])
                            pf = run["paused_for"] = {
                                "node": dead, "since": time.time(),
                                "after_shot": k,
                                "backoff_s": b0,
                                "next_probe": time.time() + b0}
                    if fresh:
                        self.events.emit(
                            "error", "capture_paused",
                            "%s stopped answering fires - capture paused, "
                            "other camera NOT fired alone. Resumes by itself "
                            "when the camera reconnects AND a probe fire "
                            "delivers a frame" % dead, node=dead)
                    # Resume needs TWO things, not one. A health answer newer
                    # than the pause says the node is reachable; only a fire
                    # that returns ok:true says it can still expose. Probing
                    # on reachability alone livelocked (see _probe_fire).
                    snap = m.snapshot() if m is not None else {}
                    seen = snap.get("last_seen") or 0
                    # Three conditions, each covering a distinct drop class:
                    # health (the Pi answers), last_seen newer than the pause
                    # (ilxctl answers - it is the only writer of last_seen),
                    # and is_connected (the SDK actually holds the body). The
                    # third is what keeps a camera-side drop - USB out, body
                    # power lost, dead SDK handle - from being probed at all:
                    # the probe cannot succeed without a body, and probing
                    # anyway is how the pause/resume oscillation started.
                    back = (bool(snap.get("health")) and seen > pf["since"]
                            and m is not None and m.is_connected())
                    if back and time.time() >= pf["next_probe"]:
                        ok, err = self._probe_fire(m)
                        if ok:
                            with self._lock:
                                run = self.active
                                if run:
                                    run["fail_streak"][dead] = 0
                                    run["last_resume"] = {
                                        "node": dead, "at": time.time(),
                                        "backoff_s": pf.get("backoff_s",
                                                            self.RESUME_BACKOFF_S[0])}
                                    run["paused_for"] = None
                            self.events.emit(
                                "info", "capture_resumed",
                                "%s answered a probe fire - capture resumed"
                                % dead, node=dead)
                            start = time.time() + 1.0   # re-anchor the grid
                            g = 0
                            continue
                        with self._lock:
                            run = self.active
                            cur = run and run["paused_for"]
                            wait_s = min(pf["backoff_s"] * 2,
                                         self.RESUME_BACKOFF_S[1])
                            if cur and cur.get("node") == dead:
                                cur["backoff_s"] = wait_s
                                cur["next_probe"] = time.time() + wait_s
                        if self.warn_once("probe:%s" % dead, 30.0):
                            self.events.emit(
                                "warn", "capture_paused",
                                "%s answers its health poll but refused a probe "
                                "fire (%s) - staying paused; next probe in "
                                "%.0f s. The camera is reachable and cannot "
                                "expose: check the GPIO harness and the body"
                                % (dead, err or "no response", wait_s),
                                node=dead)
                    stopev.wait(min(period, 1.0))
                    start = time.time() + 1.0     # re-anchor the grid
                    g = 0
                    continue
                with self._lock:
                    run = self.active
                    if run and run["paused_for"]:
                        self.events.emit(
                            "info", "capture_resumed",
                            "%s answering again - capture resumed"
                            % run["paused_for"]["node"],
                            node=run["paused_for"]["node"])
                        run["paused_for"] = None
                # Absolute grid, so lateness in one shot cannot accumulate.
                # `g` indexes the grid (re-anchored after a pause); `k` counts
                # shots fired, for the frame budget and the summary.
                target = start + g * period
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
                g += 1
            for t in inflight:
                t.join(timeout=5)
            self.events.emit("info", "capture", "auto-capture loop ended",
                             fired=k)

        self._cap_thread = threading.Thread(target=_loop, daemon=True)
        self._cap_thread.start()
        self.events.emit("info", "capture", "auto-capture loop started",
                         interval_s=period, frames=count)

    def _dead_fire_node(self):
        """The first run member with >=3 consecutive failed fires, else None.

        Three timeouts in a row ARE the evidence: waiting for the monitor to
        also notice (its poll can lag 10 s behind a node that just died) let
        a whole 8-shot budget fire into one camera before the pause engaged
        (validation-2, 2026-08-23). Resuming is where the health poll
        belongs - see the loop: a health answer newer than the pause makes the
        node eligible for a PROBE FIRE, and only a probe that returns ok:true
        clears the streak (R1)."""
        with self._lock:
            run = self.active
            if not run:
                return None
            return next((n for n, k in run["fail_streak"].items() if k >= 3),
                        None)

    def _stop_capture_loop(self):
        if self._cap_stop:
            self._cap_stop.set()

    # ---- data providers used by workers -----------------------------------
    def nav_snapshot(self, epoch):
        """Nav state at the frame's capture instant, not at 'now'.

        A frame can be pulled seconds after the shutter fired, so stamping it
        with the current position would put the boat where it has since moved
        to. fix_at() reaches back to the sample nearest that instant and has
        already computed UTM and the staleness/validity flags. `epoch` is an
        uncorrected HOST epoch: nav's ring is keyed by host receive time, and
        _capture_instant has already taken the node-clock offset out. Handing
        it a node epoch matched every frame to a fix ~190 ms away from the true
        exposure - 19 cm of boat travel at 1 m/s (audit 2026-08-23, R2)."""
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

    def imu_snapshot(self, epoch, off=None):
        """Attitude at a HOST capture instant.

        The IMU ring is stamped by piagent on the IMU NODE's clock, and the
        capture instant arrives here in the host domain (_capture_instant
        converts on the way out), so it is converted BACK for the query and
        the returned sample is converted forward again. Skipping that put the
        whole 200 ms window on the wrong side of the host-node offset - which
        on this rig is 187 ms, i.e. the entire +/-100 ms window - and blanked
        every attitude column while reporting an IMU stall (R2).

        ONE offset for both legs, and `off` is the caller's own so a row's
        round trip is exact. With the IMU node's PRIVATE estimate the two legs
        were two different numbers whenever the estimate moved between them -
        the window was centred a little off the instant it was supposed to be
        centred on, and the sample came back stamped at an instant that is not
        where it was looked for. The fleet median is also the RIGHT number
        here: the IMU node is chrony-locked to the camera nodes, so its ring
        and their edges are one domain, and it is the host that has drifted
        away from all of them."""
        imu_mon = self.imu_monitor()
        if not imu_mon:
            return None
        if off is None:
            off = self.fleet_clock_offset()            # fleet node - host
        at = epoch + off
        r = http_json("http://%s:8081/imu/window?t0=%.3f&t1=%.3f"
                      % (imu_mon.host, at - self.IMU_MAX_AGE_S,
                         at + self.IMU_MAX_AGE_S), timeout=4)
        samples = r.get("samples") if isinstance(r, dict) else None
        if samples:
            # nearest sample to the capture instant
            best = min(samples, key=lambda s: abs(s.get("epoch", at) - at))
            age = abs((best.get("epoch") or at) - at)
            if age <= self.IMU_MAX_AGE_S:
                best = dict(best)
                if best.get("epoch") is not None:
                    best["epoch"] = best["epoch"] - off      # host domain
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

    @staticmethod
    def _edge_stamp(e, hw_meta):
        """(instant, err_s, soft) for one piagent EXPOSURE event, or None.

        `soft` says the node could NOT stamp this edge in hardware, so the
        instant is the gpiomon pipe-read stamp carrying an UNMEASURED latency
        - the thing a capture instant must never silently be.

        The three states are deliberately distinguished, because two of them
        used to be indistinguishable here and needed opposite treatment:
          * hw_meta absent          - an older piagent that never published the
                                      hw fields. `epoch` is all there is and
                                      always was; keep the previous behaviour.
          * hw_meta, epoch_hw set   - the kernel instant, plus the node's own
                                      error bar on it (hw_err_ms, sub-ms).
          * hw_meta, epoch_hw null  - REFUSED (hw_reject names why). Soft.
        """
        t = e.get("epoch_hw")
        if t is not None:
            err = e.get("hw_err_ms")
            return (t, (err / 1000.0) if isinstance(err, (int, float)) else 0.0,
                    False)
        t = e.get("epoch")
        if t is None:
            return None
        # No epoch_hw. On an hw_meta node that is a refusal, not an old build.
        return (t, 0.0, bool(hw_meta))

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
        in a poll, and leaves frames 2 and 3 falling back to EXIF.

        Returns the whole buffer entry - {t, err_s, soft, ...} - not a bare
        float: an edge the node could not stamp in hardware is worth a quarter
        of a second, not the fleet clock error, and the caller cannot tell the
        two apart from the instant alone (audit 2026-08-24, VERIFY)."""
        buf = getattr(mon, "_edge_buf", None)
        if buf is None:
            buf = mon._edge_buf = []
        r = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                      % (mon.host, getattr(mon, "_edge_cursor", 0)), timeout=3)
        if isinstance(r, dict):
            mon._edge_cursor = r.get("next", getattr(mon, "_edge_cursor", 0))
            # The version marker. Absent => a node running an older piagent,
            # where `epoch` is the only stamp there has ever been and a null
            # epoch_hw says nothing; present => this node refused THIS stamp
            # and said why. Both look like "no epoch_hw" without it.
            hw_meta = bool(r.get("hw_meta"))
            for e in r.get("events", []):
                if e.get("edge") == "rise":
                    # End-of-exposure edges, kept separately: together with the
                    # fall they bound the shutter-open window the strobe
                    # acceptance check needs (strobe ∈ ⋂ [fall_i, rise_i]).
                    st = self._edge_stamp(e, hw_meta)
                    if st is not None:
                        rbuf = getattr(mon, "_rise_buf", None)
                        if rbuf is None:
                            rbuf = mon._rise_buf = []
                        rbuf.append({"t": st[0], "i": e.get("i"),
                                     "fire_seq": e.get("fire_seq"),
                                     "err_s": st[1], "soft": st[2]})
                        if len(rbuf) > 64:
                            del rbuf[:-64]
                    continue
                if e.get("edge") != "fall":
                    continue
                # epoch_hw is the kernel's own interrupt timestamp converted to
                # wall time on the node; epoch is Python's stamp after reading
                # the gpiomon pipe, which carries the node's scheduling latency
                # (median 0.1-0.3 ms here, occasionally hundreds of ms under
                # load). That latency is uncorrelated between the two Pis, so it
                # lands directly in the stereo pair's apparent skew and in the
                # capture instant written to flight_log. Prefer the hardware
                # stamp; fall back to `epoch` for a node still running an older
                # piagent - and, on a node that DOES publish hw_meta, mark that
                # fallback `soft` so no caller can mistake it for a measurement.
                st = self._edge_stamp(e, hw_meta)
                if st is not None:
                    buf.append({"t": st[0], "i": e.get("i"),
                                "fire_seq": e.get("fire_seq"),
                                "err_s": st[1], "soft": st[2]})
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
                    return buf.pop(k)
            # Its edge has not been read yet; do not fall back to a neighbour's.
            if after_seq is not None:
                return None
        cand = [k for k, e in enumerate(buf)
                if after_seq is None or e.get("i") is None
                or e["i"] > after_seq]
        if not cand:
            return None
        if expected is None:
            return buf.pop(cand[0])
        # Nearest edge to the instant this frame was scheduled to expose. The
        # window is far wider than the ~1 ms sync we achieve and far narrower
        # than the 500 ms between frames at 2 fps, so it cannot pick a
        # neighbour's edge.
        i = min(cand, key=lambda k: abs(buf[k]["t"] - expected))
        if abs(buf[i]["t"] - expected) > window:
            return None
        return buf.pop(i)

    def return_edge(self, mon, t, fire_seq=None, err_s=0.0, soft=False):
        """Put a matched EXPOSURE edge back in the buffer, in time order.

        match_exposure_edge POPS what it hands out, which is right: an edge
        belongs to exactly one frame. But a fire_seq match is identity between
        the COMMAND and the EDGE, never between the FRAME and the edge, so a
        frame holding the wrong command takes a real edge that belongs to a
        different frame. When the EXIF cross-check catches that, the edge has
        to go back - dropping it would leave the frame that really produced it
        with no edge at all (audit 2026-08-23, R3).

        err_s/soft travel back with it. An edge that returns stripped of them
        is re-handed to the next frame as a hardware-grade instant, which is
        the very claim it failed to support (audit 2026-08-24, VERIFY)."""
        buf = getattr(mon, "_edge_buf", None)
        if buf is None:
            buf = mon._edge_buf = []
        i = len(buf)
        while i > 0 and buf[i - 1]["t"] > t:
            i -= 1
        # i=None so after_seq bounding treats it as always-eligible: the index
        # it came back with has already been consumed from the cursor.
        buf.insert(i, {"t": t, "i": None, "fire_seq": fire_seq,
                       "err_s": err_s, "soft": soft})
        if len(buf) > 64:
            del buf[:-64]

    def match_rise(self, mon, fall_t, fire_seq=None):
        """The end-of-exposure edge belonging to a frame whose fall edge is
        known, or None. Prefers fire_seq identity; else the first rise after
        the fall within one plausible exposure (5 s covers multi-second
        shutters without adopting a later frame's rise at survey rates)."""
        buf = getattr(mon, "_rise_buf", None)
        if not buf:
            return None
        # A rise can only close THIS exposure if it sits within a plausible
        # shutter-plus-overhead of the fall. Audit of the 2026-08-20 2 Hz run
        # found 10 cam2 rows whose "rise" was the NEXT frame's (523 ms windows
        # at a 24 ms shutter): the ringing harness lost the genuine rise and
        # the node tagged the following one with the stale fire_seq. A wrong
        # window would make the strobe verdict lie, so an implausible rise is
        # discarded and the window left unmeasured instead.
        limit = max(0.2, 3.0 * self._shutter_s(mon) + 0.05)

        def take(k):
            e = buf.pop(k)
            t = e["t"]
            # A rise the node could not stamp in hardware cannot close an
            # exposure window: the window is ~13 ms and this stamp's error is
            # unmeasured up to hundreds of ms, so the strobe acceptance check
            # built on it would be a guess wearing a measurement's clothes.
            # Leave the window unmeasured instead (audit 2026-08-24, VERIFY).
            if e.get("soft"):
                return None
            if 0 < t - fall_t <= limit:
                return t
            if self.warn_once("rise_lost:%s" % mon.name_, 60):
                self.events.emit(
                    "warn", "edge", "%s: end-of-exposure edge lost - the next "
                    "frame's rise was tagged to this fire (%.0f ms after the "
                    "fall, limit %.0f); window left unmeasured"
                    % (mon.name_, (t - fall_t) * 1000, limit * 1000),
                    node=mon.name_)
            return None
        if fire_seq is not None:
            for k, e in enumerate(buf):
                if e.get("fire_seq") == fire_seq:
                    return take(k)
        cand = [k for k, e in enumerate(buf)
                if 0 < e["t"] - fall_t <= limit and not e.get("soft")]
        if not cand:
            return None
        k = min(cand, key=lambda k: buf[k]["t"] - fall_t)
        return buf.pop(k)["t"]

    @staticmethod
    def _shutter_s(mon):
        """The body's current shutter in seconds (Sony (num<<16)|den), default
        1/60 when unreadable."""
        try:
            v = int((mon.snapshot()["status"] or {}).get("shutterValue") or 0)
            num, den = (v >> 16) & 0xFFFF, v & 0xFFFF
            if num and den:
                return num / den
        except Exception:  # noqa: BLE001
            pass
        return 1.0 / 60

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

    def node_calibration_lock(self, node):
        """One calibration at a time per node.

        _cal_busy is a single per-node TIMESTAMP, not a counter, so two passes
        overlapping on one node cancel each other: rigd's startup
        calibrate_trigger and a run start's _calibrate_exif each call
        begin/end_calibration_fire, and pass 1's end released the puller while
        pass 2's frame was still landing - that frame then entered the transect
        as survey data (audit 2026-08-23, R6). Serialising the passes removes
        the nesting entirely, which is simpler than making the flag re-entrant
        and also stops two passes fighting over the same body."""
        with self._lock:
            lk = self._cal_locks.get(node)
            if lk is None:
                lk = self._cal_locks[node] = threading.Lock()
            return lk

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

    @staticmethod
    def spool_names(m):
        """The names in one node's save dir, or None if the listing failed.

        Every calibration path below reasons by DIFFERENCE between two of
        these. A failed listing flattened to an empty set is the dangerous
        direction in BOTH of them: an empty `before` makes every frame already
        on the node look like the one this fire just produced, and
        note_calibration_frames() then hands those names to the puller as
        calibration exposures - which DELETES real survey frames from the
        transect, the outcome this file calls the one worse than a calibration
        frame surviving in it. So the difference is only ever taken between
        two listings that actually happened."""
        shots = m.shots()
        return None if shots is None else {s["name"] for s in shots}

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

    def _calibrate_and_arm(self, live, config, run_at_entry=None):
        """Run-start calibration, then release the survey capture loop.

        Ordered, not concurrent: the EXIF clock offset needs a frame it can
        attribute to its own shutter command, and the trigger-latency pass fires
        five more frames per camera. Overlapping them (as separate threads used
        to) let the EXIF pass measure the camera clock against somebody else's
        exposure. Overlapping either with auto-capture was worse still."""
        stopev = self._calib_stop
        if run_at_entry is None:
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
                # Tagged with the run this pass was started for: this thread
                # can reach here seconds after the operator stopped that run
                # and started another one (R5).
                self._start_capture_loop(config, run_at_entry)

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
        cap = self._cap_stop
        grid_live = (cap is not None and not cap.is_set()
                     and bool(self.shot_period()))
        if grid_live:
            # CALIBRATE WITHOUT FIRING while the grid is running. _calibrate_exif
            # takes a `before` listing, presses the USB shutter and names
            # whatever is new - but with auto-capture live, a scheduled GPIO
            # fire lands inside that same window on ~50% of shots at a 2 s
            # period and on nearly every shot at 0.5 s. It sorts first by
            # camera filename, so named[0] is the SURVEY frame: it is recorded
            # as a calibration exposure and the puller then DELETES it from
            # the transect, and the real calibration frame inherits its command
            # and its GPIO edge (audit 2026-08-23, R5). A frame lost from the
            # survey is worse than an unmeasured camera clock, and the cost of
            # not measuring is bounded and sayable: frames without an EXPOSURE
            # edge fall back to the command epoch. Trigger latency can still be
            # adopted from the persisted per-body figure, which fires nothing.
            reused = self._reuse_trig_latency(m)
            self.events.emit(
                "error", "calibration_missing",
                "%s joined mid-transect while the capture grid is running: its "
                "camera-clock (EXIF) offset is NOT being measured, because the "
                "calibration shutter cannot be told apart from a survey fire "
                "and would cost a real frame. Frames of its without an EXPOSURE "
                "edge fall back to the command epoch (0-200 ms). %s Stop and "
                "restart the line to calibrate it properly"
                % (m.name_,
                   "Trigger latency reused from this body's saved measurement."
                   if reused else
                   "It also has no trigger latency of its own and will fire on "
                   "the fleet median."), node=m.name_)
            return
        self._calibrate_exif(m)
        if m.name_ in self.trig_latency:
            return
        self.calibrate_trigger(nodes=[m])

    TRIG_LAT_MAX_AGE_S = 24 * 3600

    @property
    def TRIG_LAT_PATH(self):
        # Under rigcore.RIG_HOME so the test harness's redirect isolates it:
        # fakes must never persist a latency the live fleet could read.
        return os.path.join(rigcore.RIG_HOME, "trigger_latency.json")

    def _load_trig_latency(self):
        self._trig_saved = {}
        try:
            with open(self.TRIG_LAT_PATH) as fh:
                self._trig_saved = json.load(fh)
        except (OSError, ValueError):
            self._trig_saved = {}

    def _save_trig_latency(self, out):
        now = time.time()
        for name, lat in out.items():
            m = next((x for x in self.monitors if x.name_ == name), None)
            cam_id = ((m.snapshot().get("status") or {}).get("id")
                      if m else None)
            self._trig_saved[name] = {"ms": round(lat * 1000, 3), "at": now,
                                      "camera_id": cam_id}
        try:
            os.makedirs(os.path.dirname(self.TRIG_LAT_PATH), exist_ok=True)
            tmp = self.TRIG_LAT_PATH + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(self._trig_saved, fh, indent=1)
            os.replace(tmp, self.TRIG_LAT_PATH)
        except OSError as e:
            self.events.emit("warn", "calibrate",
                             "could not persist trigger latency: %s" % e)

    def _reuse_trig_latency(self, m):
        """Adopt a saved latency for this node when it is fresh and was
        measured on the SAME body (camera id). Returns True if adopted."""
        ent = self._trig_saved.get(m.name_)
        if not ent:
            return False
        cam_id = (m.snapshot().get("status") or {}).get("id")
        if ent.get("camera_id") != cam_id or cam_id is None:
            return False
        age = time.time() - float(ent.get("at") or 0)
        if age > self.TRIG_LAT_MAX_AGE_S:
            return False
        self.trig_latency[m.name_] = float(ent["ms"]) / 1000.0
        self.trig_measured_at[m.name_] = float(ent["at"])
        self.events.emit("info", "calibrate",
                         "%s trigger latency reused: %.2f ms measured %.1f h "
                         "ago on this body (%s) - no calibration frames fired"
                         % (m.name_, ent["ms"], age / 3600.0, cam_id),
                         node=m.name_)
        return True

    def calibrate_trigger(self, samples=5, hold_focus=True, nodes=None,
                          force=False):
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
        # Calibration FIRES the shutter. A camera switched off must not be in
        # the pool on either path - the implicit fleet one or an explicitly
        # passed list - or the operator's "do not use this camera" turns into
        # shutter actuations on it at every program start and run start.
        pool = nodes if nodes is not None else self.monitors
        live = [m for m in pool if m.is_connected() and m.is_capturing()
                and (m.health.get("gpio", {}) or {}).get("available")]
        if not force:
            live = [m for m in live if not self._reuse_trig_latency(m)]
        if not live:
            return {}
        out = {}
        for m in live:
            if self._calib_over(stopev, run_at_entry):
                break
            # One calibration pass per node at a time: rigd's startup pass and
            # a run start's pass used to overlap on the same body and cancel
            # each other's puller quiet (R6).
            lk = self.node_calibration_lock(m.name_)
            if not lk.acquire(timeout=self.CAL_FRAME_WAIT_S):
                self.events.emit(
                    "warn", "calibrate",
                    "another calibration is still running on %s - skipping the "
                    "trigger-latency pass rather than interleaving two sets of "
                    "calibration frames on one body" % m.name_, node=m.name_)
                continue
            try:
                self._calibrate_trigger_node(m, samples, hold_focus, stopev,
                                             run_at_entry, out)
            finally:
                lk.release()
        if out:
            now = time.time()
            self.trig_latency.update(out)
            for k in out:
                self.trig_measured_at[k] = now
            self._save_trig_latency(out)
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

    def _calibrate_trigger_node(self, m, samples, hold_focus, stopev,
                                run_at_entry, out):
        """One node's TRIGGER->EXPOSURE latency pass. Caller holds its lock."""
        if hold_focus:
            http_json("http://%s:8081/gpio/focus" % m.host, {"hold": True},
                      timeout=8)
        time.sleep(0.3)
        lat = []
        try:
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
                    before = self.spool_names(m)
                    if before is None:
                        # No before-listing, no way to tell this fire's frame
                        # from a survey frame. Skip the sample rather than fire
                        # a frame that cannot be kept out of the transect.
                        self.events.emit(
                            "warn", "calibrate",
                            "%s did not answer its shot listing - skipping a "
                            "trigger-latency sample rather than firing a frame "
                            "that cannot be identified" % m.name_, node=m.name_)
                        continue
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
                    #
                    # The deadline is CAL_QUIET_S, not the 1.2 s it used to be.
                    # 1.2 s covers the Small JPEGs the rig ships with (~0.5 s
                    # to list) but NOT the Original-size frames HANDOFF §3
                    # names as the target: 14 MB over a Pi 4's USB lists at
                    # ~1.5-2 s, so the quiet ended, the puller went back on the
                    # node and pulled the calibration frame into the transect
                    # as survey data - all five per node, with skipped_cal
                    # still reading 0 (audit 2026-08-23, R6). Keep the node
                    # quiet until the frame is SEEN or the bound expires.
                    while time.time() < t_fire + self.CAL_QUIET_S:
                        time.sleep(0.05)
                        after = self.spool_names(m)
                        named = ([] if after is None else
                                 self.note_calibration_frames(
                                     m.name_, before, after))
                        if named:
                            self.end_calibration_fire(m.name_)
                            break
                        # Keep refreshing the hold: begin_calibration_fire sets
                        # an absolute expiry, and this loop can outlive the one
                        # taken before the shutter.
                        self.begin_calibration_fire(m.name_, hold_s=1.0)
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
                    # A soft edge (no epoch_hw on a node that publishes the
                    # hw fields) carries the gpiomon pipe-read latency, and
                    # this median BECOMES each node's fire compensation - so
                    # folding an unmeasured, per-node read latency in here
                    # shifts one camera against the other for the whole run.
                    # Skip the sample rather than bias the alignment; the
                    # median is over five fires and tolerates losing some.
                    hw_meta = bool(ev.get("hw_meta"))
                    falls = [e for e in (ev.get("events") or [])
                             if e.get("edge") == "fall"
                             and not (hw_meta and e.get("epoch_hw") is None)
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
                        after = self.spool_names(m)
                        if after is not None:
                            self.note_calibration_frames(m.name_, before, after)
                        else:
                            self.events.emit(
                                "warn", "calibrate",
                                "%s stopped answering its shot listing during "
                                "trigger calibration - its calibration frame is "
                                "unnamed and may enter the transect"
                                % m.name_, node=m.name_)
                finally:
                    self.end_calibration_fire(m.name_)
                time.sleep(0.2)
        finally:
            if hold_focus:
                # Released in a finally: an exception here used to leave the
                # body half-pressed (AE-locked) for the rest of the session.
                http_json("http://%s:8081/gpio/focus" % m.host, {"hold": False},
                          timeout=8)
        if lat:
            lat.sort()
            out[m.name_] = lat[len(lat) // 2]        # median, outlier-proof

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
        # One calibration pass per node (R6): rigd's startup calibrate_trigger
        # and this pass used to run concurrently on the same body, and pass 1's
        # end_calibration_fire released the puller while pass 2's frame was
        # still landing.
        lk = self.node_calibration_lock(m.name_)
        if not lk.acquire(timeout=self.CAL_FRAME_WAIT_S):
            self.events.emit(
                "warn", "calibrate",
                "another calibration is still running on %s - the camera-clock "
                "offset is not being measured this pass" % m.name_,
                node=m.name_)
            return
        r0 = http_json("http://%s:8081/gpio/exposure/events" % m.host, timeout=3)
        cur = r0.get("next", 0) if isinstance(r0, dict) else 0
        self.begin_calibration_fire(m.name_)
        try:
            # Snapshot the save dir as late as possible - after the puller is
            # held off and immediately before the release - so the window in
            # which an unrelated frame can land and be mistaken for ours is one
            # HTTP round trip rather than several.
            before = self.spool_names(m)
            if before is None:
                # Without a before-listing this calibration frame could not be
                # told apart from a survey frame, and naming a survey frame as
                # calibration DELETES it from the transect. No measurement is
                # worth that: leave the camera-clock offset unmeasured (frames
                # then fall back to the command epoch, which is honest) and say
                # so.
                self.events.emit(
                    "warn", "calibrate",
                    "%s did not answer its shot listing - the EXIF calibration "
                    "shutter is NOT fired; its camera-clock offset stays "
                    "unmeasured" % m.name_, node=m.name_)
                return
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
                after = self.spool_names(m)
                named = ([] if after is None else
                         self.note_calibration_frames(m.name_, before, after))
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
                cam = _exif_capture(data)[0]
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
            lk.release()

    def _true_exposure(self, m, cursor, t_cmd, t_done):
        """(true capture epoch, uncertainty_s) for the calibration frame.

        A USB-fired release drives the same EXPOSURE line as a GPIO one, so the
        harness dates it to the kernel interrupt - three orders of magnitude
        better than any estimate from the command. Read the edges with a private
        cursor so this never consumes an edge a survey frame is waiting for."""
        ev = http_json("http://%s:8081/gpio/exposure/events?since=%d"
                       % (m.host, cursor), timeout=4)
        # Hardware-stamped falls only. This value becomes the body's EXIF
        # clock offset, which then dates every capture_source=exif frame of
        # the run, so an unmeasured pipe-read latency folded in here is a
        # standing per-node bias - not a one-frame error. When the node cannot
        # stamp, take the command-epoch fallback below, which at least
        # PUBLISHES its own error bar (audit 2026-08-24, VERIFY).
        hw_meta = bool(ev.get("hw_meta"))
        falls = [(e.get("epoch_hw") or e.get("epoch"))
                 for e in (ev.get("events") or []) if e.get("edge") == "fall"
                 and not (hw_meta and e.get("epoch_hw") is None)]
        # The FLEET offset, like every other host<->node conversion in this
        # file. What comes out of here becomes the camera-clock offset that
        # _capture_instant later compares node-domain edges and commands
        # against; measuring it against a different (per-node) conversion than
        # the one those comparisons use makes the EXIF tier disagree with the
        # edge tier by the difference of two estimates.
        off = self.fleet_clock_offset()                  # fleet node - host
        falls = [f for f in falls if f and f >= t_cmd + off - 0.05]
        if falls:
            return min(falls), 0.002 + self.fleet_clock_err()
        # No harness (or no edge): PROTOCOL.md's fallback, command epoch + the
        # 20 ms hardware release lag + half the 4.5 ms curtain transit. The
        # dispatch itself is the error bar, so record it rather than imply this
        # is as good as an edge.
        #
        # Returned on the NODE clock, like the edge branch above. This value
        # becomes the EXIF offset (camera clock - true clock), and everything
        # _capture_instant compares a corrected EXIF instant against - queued
        # command epochs, EXPOSURE edges - is node-domain. Leaving the fallback
        # on the host clock made the offset silently mean two different things
        # depending on whether the harness happened to catch that one edge.
        return t_cmd + off + 0.024, \
            max(0.05, t_done - t_cmd) + self.fleet_clock_err()

    # ---- run.json / index -------------------------------------------------
    def note_orphan(self, node, n=1):
        with self._lock:
            if self.active:
                self.active["orphans"][node] = \
                    self.active["orphans"].get(node, 0) + n

    def on_frame(self, node, cam_num, fname, orig, epoch):
        self.events.emit("debug", "frame", "%s <- %s" % (fname, orig), node=node)

    def index_frame(self, cam_num, fname, orig, epoch, source, node=None,
                    path=None, rise=None, strobe=None, clk_off=None):
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
                if rise is not None:
                    # End of exposure: [epoch, rise] is this camera's measured
                    # shutter-open window, for the strobe acceptance check.
                    rec["rise"] = round(rise, 6)
                if strobe is not None:
                    # The shot's strobe pulse instant, carried on the strobe
                    # node's frame; the pair-level verdict joins it against
                    # every member's window when the run is browsed.
                    rec["strobe"] = round(strobe, 6)
                if clk_off is not None:
                    # The fleet clock offset THIS frame was converted with.
                    # run.json's clock block used to carry one live-read scalar
                    # and assert applied:true, but the applied number moves
                    # across a transect - the host free-runs ~60 ppm and was
                    # seen to STEP 187 -> 69 ms mid-session - so re-basing the
                    # whole run with that scalar reintroduces up to the run's
                    # own drift, undetectably. Six bytes a row makes the
                    # re-base exact and a mid-run step visible by inspection
                    # (audit 2026-08-24).
                    rec["clk_off"] = round(clk_off, 6)
                    span = self.active.setdefault("clk_applied",
                                                  [None, None, None, None])
                    if span[0] is None:
                        span[0] = span[2] = span[3] = clk_off
                    span[1] = clk_off
                    span[2] = min(span[2], clk_off)
                    span[3] = max(span[3], clk_off)
                self.active["index"].append(rec)
                # The complete, append-only index. run.json's "index" is
                # capped at the last 2000 entries and is rewritten whole every
                # ten frames; this one is one line per frame, flushed as it is
                # written, so a 4000-shot transect and a run that is killed
                # both keep every entry (contract C3). Readers prefer it and
                # fall back to run.json's index when it is absent.
                fh = self.active.get("index_fh")
                if fh is not None:
                    try:
                        fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
                        fh.flush()
                        os.fsync(fh.fileno())
                    except (OSError, ValueError, TypeError) as e:
                        self.active["index_fh"] = None
                        self.events.emit(
                            "error", "run",
                            "index.jsonl is no longer being written (%s) - "
                            "run.json still holds the last 2000 frames" % e)
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
               # Read without get_strobe(): callers already hold self._lock.
               "strobe": dict(self.strobe),
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
                        "unpaired_shots": run.get("unpaired_shots", 0),
                        "paused_for": run.get("paused_for"),
                        "host_offset_s": round(self.fleet_clock_offset(), 6),
                        "fired": dict(run["fired"]),
                        "orphan_fires": dict(run["orphans"]),
                        "trigger_latency_ms": {k: round(v * 1000, 2) for k, v
                                               in self.trig_latency.items()}},
               # Which node-clock correction was applied to every capture
               # instant in this run, so a transect recorded against a
               # mis-set host clock can be understood (and re-based) in post
               # rather than silently believed (audit 2026-08-23, R2).
               # node_offsets_s is DIAGNOSTIC - what each node's own estimate
               # said, and null (never 0.0) for a node with no measurement:
               # node_clock_offset() flattens unknown to 0.0 so an unpolled
               # node cannot stop a survey, but published beside a measured
               # +0.187 that reads as "this node's clock agrees with the host
               # exactly" - a phantom 187 ms inter-camera skew in the one
               # record a post-processor re-bases from (audit 2026-08-24).
               #
               # host_offset_s is the offset in force as this document was
               # written; applied_offset_s is the span actually applied across
               # the run, because the host free-runs (~60 ppm here) and can
               # step, so ONE scalar cannot describe a whole transect. The
               # exact number each frame used is on the frame, in index.jsonl's
               # "clk_off" - that is what makes a re-base exact.
               "clock": {"node_offsets_s":
                         {m.name_: _r6(self._clock_offset_raw(m))
                          for m in self.monitors if m.name_ in run["nodes"]},
                         "host_offset_s": round(self.fleet_clock_offset(), 6),
                         "applied_offset_s": _applied_span(
                             run.get("clk_applied")),
                         "err_ms": round(self.fleet_clock_err() * 1000, 2),
                         "applied": True},
               "frames": len(run["index"]),
               "index_jsonl": "index.jsonl",
               "index": run["index"][-2000:],
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
                             "unpaired_shots": run.get("unpaired_shots", 0),
                             "paused_for": run.get("paused_for"),
                             # The node-minus-host correction being applied to
                             # this run's fire schedule and capture instants.
                             "host_offset_s": round(
                                 self.fleet_clock_offset(), 6),
                             "orphan_fires": dict(run["orphans"])},
                    "stats": {n: w.stats() for n, w in self.workers.items()}}


def _slug(s):
    """The run-id alphabet. Delegates to rigcore.run_id_slug — one sanitiser.

    This used to be a second copy of the logic, and the two disagreed on
    exactly the labels that matter: str.isalnum() is True for 'é' and for
    'サ', so the WRITER produced ids like "260815_1930_Récif-Nord" that the
    READERS' _RUN_ID_RE refused — the transect could not be listed, opened or
    finalised. Readers were widened to keep such runs browsable; the writer
    stops producing them (audit 2026-08-23)."""
    return rigcore.run_id_slug(s)

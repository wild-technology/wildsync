#!/usr/bin/env python3
"""Audit regression suite — run (findings R1-R26, rig/run.py).

Every check below FAILS on the pre-fix code and PASSES after. What each one
reproduces, in the words of the defect it pins:

R1  A run member that is not connected was simply left out of capture_once's
    roster: it produced no result, so it never accumulated a failed fire,
    fail_streak never reached 3, the pause never engaged, and the other camera
    shot the rest of the transect alone - 28 single-camera frames reported as
    unpaired_shots:1. Also the pause/resume livelock: a node whose health
    answers but whose fires are refused resumed on every poll.
R2  Host and node clocks are different domains. (a) the fire schedule was a
    HOST instant handed to a node that busy-waits on its own clock, so the
    Mac's 187 ms offset ate the 300 ms lead; (b) a capture instant taken from
    a GPIO edge is a NODE instant and was fed straight to nav's host-keyed
    ring, the datetime and the filename.
R3  At 2 Hz the claim tolerance floored at 1.5 s - three shot periods - so one
    dropped frame shifted every later frame one period early, still stamped
    capture_source=gpio_edge / time_err_ms 0.
R4  RAW+JPEG in the spool: two files per release each claimed a fire command,
    so the JPEG took the NEXT shot's command and edge.
R5  A stopped run's calibration thread armed a capture loop on whatever run
    was active when it reached its finally block.
R7  interval_s/frames were stored verbatim: 0 or negative spun the loop flat
    out. Also the mid-run master-clock warning was once per PROCESS.
R8  run.json truncates its index to the last 2000 entries; index.jsonl is the
    complete append-only record (contract C3).
R10 A standalone strobe on an unreachable strobe node blocked every
    capture_once for 10.3 s.
R11 (2026-08-24) Host<->fleet clock conversion. (a) each frame's capture
    instant was converted with ITS OWN node's offset ESTIMATE, so the pair
    spread the run browser draws from those epochs carried
    (est_cam1 - est_cam2) - the difference of two estimates worth ~RTT/2
    each - against a 10 ms budget; (b) imu_snapshot converted host->node with
    the IMU node's estimate and back with it, while the instant it was handed
    had been converted with the CAMERA node's, so the +/-100 ms window was
    displaced by their difference; (c) NodeMonitor.shots() returned [] when
    the listing FAILED, so PullWorker baselined an empty spool and pulled
    every file already on the node into the transect as survey data.

R12 (2026-08-24) The pre-deployment review of the R1-R11 fix. (a) a RAW half
    listed in a LATER poll than its JPEG found _stem_cmd already popped, took
    the NEXT shot's command and that shot's EXPOSURE edge, then destroyed both
    as a sibling; (b) _plaus_strikes counted ORPHANED COMMANDS, so one ~3 s
    delivery gap latched _plaus_off for the run and every later frame took the
    head-of-queue command by fire_seq identity - R3 reinstated; (c) the fleet
    clock latch is keyed on wall time, not on the shot, so the two halves of
    one pair could convert with different offsets; (d) _probe_fire quarantined
    its frame only when the fire REPORTED ok, and never refreshed its puller
    hold; (e) a worker that could not baseline gave up for the whole run while
    the camera kept being fired; (f) run.json published "never measured" as
    0.000000 and one live-read scalar for an offset that moves across a run;
    (g) the card-review fallback's clk_off=0.0 was handed to imu_snapshot as
    the outward host->node conversion offset.

R13 (2026-10-15) The frames an ADOPTED worker finds already on its node
    (the "taken before adoption" backlog) each claimed a fire command, so the
    first command queued after adoption went to a frame exposed before the
    worker existed and every later frame of the camera took the command of the
    shot after its own - a persistent one-period shift labelled gpio_edge.
R14 PullWorker has no `events` of its own: the flight_log failure handlers
    (header open at start, row write mid-run) raised AttributeError - killing
    the worker before its first pull, or losing a frame after its spool copy
    was deleted - and the stop path closed a flight_log that was already None.
R15 A standalone strobe (a camera-less strobe node - the Jetson's XR256)
    answered with its pulse instant and nothing read it, so no frame carried a
    `strobe` and the per-pair acceptance verdict never existed.
R16 `pulled` counted FILES: a RAW+JPEG release counted twice, so stop()'s
    final-shot grace ended before the last shot was listed and the
    frames_missing alarm was blind to a camera that had stopped delivering.
R17 run.json's sync.shots / skew_ms_max / skew_ms_p95 covered only the last
    1000 shots of a run, hiding the worst skew of a longer transect.
R18 start() held the run lock across reset_edge_cursors(), one blocking HTTP
    call per monitor (3 s for an unplugged slot), stalling every status poll
    and every pull worker's known-frames bookkeeping.
R19 A camera switched off after three failed fires kept its stale fail_streak,
    so _dead_fire_node() kept the grid paused for a node that is neither
    judged nor probed - until stop/start.
R20 A body that wrote no SubSecTimeOriginal published whole-second EXIF with
    the few-ms bar of the edge it was calibrated against, and that bar beat a
    soft edge in _edge_result.
R21 Frames taken BETWEEN runs (bench Capture, a bare calibrate) were never in
    known_frames, so a camera joining the next run late had them classed as
    "taken during the run" and pulled into the transect as survey data.
R22 A camera filename reused after a counter rollover was skipped for the rest
    of the run (`seen` never aged), never pulled and never deleted.
R23 A failed run-start EXIF calibration kept the previous run's offset and its
    error bar, while the event said the offset was unmeasured.
R24 Frames still pending when a run stopped left no unpulled.jsonl record, and
    the next run baselined them away.
R25 A pull worker that outlived stop() indexed its frame (and counted its
    orphans) into the NEXT run.
R26 String "false" armed auto-capture on /api/run/start and the strobe on
    /api/strobe; Infinity escaped int() as OverflowError.

Hermetic: in-process fakes on loopback only (soaktest's netguard is installed
by importing it), temp run roots, no sleeps beyond what the code under test
needs. Runs in ~60 s.

Run standalone:  python3 rig/tests/audit_run.py
"""

import io
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

import soaktest                                          # noqa: E402
from soaktest import (check, sect, wait_for, Env, FakeNav,  # noqa: E402,F401
                      read_flight, note)
import fakenode                                          # noqa: E402
import run as runmod                                     # noqa: E402
import rigcore                                           # noqa: E402


# ---------------------------------------------------------------------------
# A fake whose PIAGENT surfaces speak the NODE's own clock.
#
# The stock FakeNode stamps /health time.epoch and GPIO edges with time.time()
# - i.e. it pretends host and node share one clock, which is the very
# assumption the R2 defect is made of. node_offset_s = node clock - host clock.
# ---------------------------------------------------------------------------
class OffsetNode(fakenode.FakeNode):
    def __init__(self, *a, node_offset_s=0.0, **kw):
        self.node_offset_s = float(node_offset_s)
        super().__init__(*a, **kw)

    def health(self):
        h = super().health()
        t = dict(h.get("time") or {})
        t["epoch"] = time.time() + self.node_offset_s
        h["time"] = t
        return h

    def push_edge(self, epoch=None, edge="fall", fire_seq=None):
        ep = (time.time() if epoch is None else epoch) + self.node_offset_s
        return super().push_edge(epoch=ep, edge=edge, fire_seq=fire_seq)


class _UseOffsetNodes:
    """Env builds fakes through soaktest.FakeNode; swap the class, restore it."""

    def __enter__(self):
        self._prev = soaktest.FakeNode
        soaktest.FakeNode = OffsetNode
        return self

    def __exit__(self, *a):
        soaktest.FakeNode = self._prev
        return False


class RecordingNav(FakeNav):
    """A nav reader that remembers every instant fix_at() was asked about."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.asked = []

    def fix_at(self, epoch=None, max_age_s=None):
        self.asked.append(time.time() if epoch is None else float(epoch))
        return super().fix_at(epoch, max_age_s)


def _wait_calibrated(env, timeout=20.0):
    """Run-start calibration is a thread; nothing may be injected under it."""
    return wait_for(lambda: not (env.runmgr._calib_thread is not None
                                 and env.runmgr._calib_thread.is_alive()),
                    timeout)


def _drain_spool(env, node_name, w, timeout=12.0):
    """Wait until the worker has dealt with everything already on the node.

    Run start fires one EXIF-calibration frame per camera. Left in the spool it
    is listed alongside the injected frames and takes a claim of its own before
    being recognised and discarded, which shifts the command/frame alignment
    the R3 and R4 scenarios are about. Drain first, then inject."""
    def done():
        names = {sh["name"] for sh in env.node(node_name).shots}
        return not (names - w.seen) and not w._pending
    return wait_for(done, timeout)


def _rows(root, node):
    path = os.path.join(root, node, "flight_log.csv")
    if not os.path.exists(path):
        return [], []
    hdr, rows = read_flight(path)
    return hdr, [dict(zip(hdr, r)) for r in rows]


def _index(env, root):
    """The run's frame index, read the way contract C3 tells readers to:
    index.jsonl when it exists, run.json's capped "index" otherwise. Reading
    only index.jsonl would make every pre-fix comparison vacuously true.

    run.json is only rewritten every tenth frame, so flush it first when a run
    is still open - otherwise the fallback reads an index that is merely
    unwritten rather than truncated."""
    if env is not None and env.runmgr.active is not None:
        try:
            env.runmgr._write_run_json()
        except Exception:                                     # noqa: BLE001
            pass
    path = os.path.join(root, "index.jsonl")
    if os.path.exists(path):
        with open(path) as fh:
            return [json.loads(l) for l in fh if l.strip()]
    rj = os.path.join(root, "run.json")
    if os.path.exists(rj):
        return json.load(open(rj)).get("index") or []
    return []


# ===========================================================================
# R1 — an offline run member is a failed fire, and resume needs a real fire
# ===========================================================================
def _r1_offline_member_pauses(opts):
    sect("R1 a run member that drops offline pauses the grid (never shoots alone)")
    env = Env([("cam1", "127.0.0.41", 1), ("cam2", "127.0.0.42", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        check("both cameras connected", env.wait_state("cam1", "CAM_CONNECTED")
              and env.wait_state("cam2", "CAM_CONNECTED"))
        r = env.runmgr.start({"label": "r1", "interval_s": 0.5,
                              "auto_capture": True, "calibrate": False})
        check("run started with both members", r.get("ok")
              and sorted(r.get("nodes") or []) == ["cam1", "cam2"],
              json.dumps(r)[:160])
        _wait_calibrated(env, 20)
        got = wait_for(lambda: (env.runmgr.active or {}).get("fired", {})
                       .get("cam2", 0) >= 2, 15)
        check("paired shots are being fired before the fault", got,
              json.dumps((env.runmgr.active or {}).get("fired")))

        # cam2 loses power. Its monitor drops out of `live`; pre-fix it simply
        # stopped being fired at and nothing anywhere noticed.
        env.node("cam2").down()
        check("cam2's monitor leaves CAM_CONNECTED",
              wait_for(lambda: not env.mon("cam2").is_connected(), 12))

        paused = wait_for(lambda: ((env.runmgr.status().get("sync") or {})
                                   .get("paused_for") or {}).get("node"), 15)
        st = env.runmgr.status().get("sync") or {}
        check("an OFFLINE run member is judged as a failed fire and pauses "
              "the grid", paused == "cam2",
              "paused_for=%s" % json.dumps(st.get("paused_for")))
        check("the pause engaged within a handful of shots",
              paused == "cam2" and 0 < st.get("unpaired_shots", 0) <= 6,
              "unpaired_shots=%s failed_fires=%s"
              % (st.get("unpaired_shots"), st.get("failed_fires")))
        check("every single-camera shot is counted as unpaired",
              paused == "cam2"
              and 1 <= st.get("unpaired_shots", 0) <= st.get("failed_fires", 0),
              "unpaired=%s failed=%s" % (st.get("unpaired_shots"),
                                         st.get("failed_fires")))

        # And the surviving camera must NOT keep shooting the line alone. The
        # shot already dispatched when the streak hit 3 is still in flight and
        # legitimately lands after the pause; what must stop is the GRID.
        time.sleep(1.2)
        fired0 = dict((env.runmgr.active or {}).get("fired") or {})
        time.sleep(2.5)
        fired1 = dict((env.runmgr.active or {}).get("fired") or {})
        check("the healthy camera fires no further shots while paused",
              fired1.get("cam1", 0) == fired0.get("cam1", 0),
              "cam1 fired %s -> %s over 2.5 s at a 0.5 s period"
              % (fired0.get("cam1"), fired1.get("cam1")))
        check("capture_paused names the node in the journal",
              any(e["kind"] == "capture_paused" and e.get("node") == "cam2"
                  for e in env.evs()))
    finally:
        env.close()


def _r1_resume_needs_a_real_fire(opts):
    sect("R1 resume is gated on a probe FIRE, not on a health answer")
    env = Env([("cam1", "127.0.0.43", 1), ("cam2", "127.0.0.44", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.wait_state("cam2", "CAM_CONNECTED")
        env.runmgr.start({"label": "r1b", "interval_s": 0.5,
                          "auto_capture": True, "calibrate": False})
        _wait_calibrated(env, 20)
        wait_for(lambda: (env.runmgr.active or {}).get("fired", {})
                 .get("cam2", 0) >= 1, 15)

        # cam2's piagent answers /health perfectly but refuses every fire: a
        # lost gpiod line, a stray interval holding the fire lock, a body that
        # will not release. This is the livelock case - a health-based resume
        # forgave the streak on every poll and fired cam1 alone three more
        # times per cycle, for ~75% of the grid.
        real = runmod.http_json
        host = env.node("cam2").host

        def _patched(url, *a, **kw):
            if isinstance(url, str) and url.startswith(
                    "http://%s:8081/gpio/fire" % host):
                return {"ok": False, "error": "trigger line unavailable"}
            return real(url, *a, **kw)
        runmod.http_json = _patched
        try:
            paused = wait_for(lambda: ((env.runmgr.status().get("sync") or {})
                                       .get("paused_for") or {}).get("node"),
                              15)
            check("a node that answers health but refuses fires still pauses",
                  paused == "cam2", str(paused))
            time.sleep(1.2)     # let the shot already in flight land
            un0 = (env.runmgr.status().get("sync") or {}).get("unpaired_shots", 0)
            fired0 = dict((env.runmgr.active or {}).get("fired") or {})
            time.sleep(5.0)         # two probe backoff windows
            un1 = (env.runmgr.status().get("sync") or {}).get("unpaired_shots", 0)
            fired1 = dict((env.runmgr.active or {}).get("fired") or {})
            still = ((env.runmgr.status().get("sync") or {})
                     .get("paused_for") or {}).get("node")
            check("a refusing node does not resume the grid on its health poll",
                  still == "cam2", str(still))
            check("cam1 is not fired alone while cam2 refuses",
                  fired1.get("cam1", 0) == fired0.get("cam1", 0) and un1 == un0,
                  "cam1 fired %s -> %s over 5 s, unpaired %s -> %s"
                  % (fired0.get("cam1"), fired1.get("cam1"), un0, un1))
            check("the refusal is reported, not silently retried",
                  any("refused a probe fire" in e["msg"] for e in env.evs()))
        finally:
            runmod.http_json = real
        # Now let it fire again: the probe succeeds and the grid resumes.
        skipped0 = (env.runmgr.status().get("stats", {})
                    .get("cam2", {}).get("skipped_calibration", 0))
        resumed = wait_for(lambda: not (env.runmgr.status().get("sync") or {})
                           .get("paused_for"), 30)
        check("a successful probe fire resumes the grid", resumed,
              json.dumps((env.runmgr.status().get("sync") or {})
                         .get("paused_for")))
        skipped1 = wait_for(lambda: (env.runmgr.status().get("stats", {})
                                     .get("cam2", {})
                                     .get("skipped_calibration", 0)) > skipped0,
                            10)
        check("the probe frame is quarantined, not written into the transect",
              bool(skipped1), "skipped_calibration %s -> %s"
              % (skipped0, (env.runmgr.status().get("stats", {})
                            .get("cam2", {}).get("skipped_calibration"))))
    finally:
        env.close()


# ===========================================================================
# R2a — the fire schedule is expressed in NODE time
# ===========================================================================
def _r2a_fire_schedule_in_node_time(opts):
    sect("R2a the fire target is a NODE instant, so the lead survives the offset")
    OFF = 0.2
    with _UseOffsetNodes():
        env = Env([("cam1", "127.0.0.45", 1, {"node_offset_s": OFF}),
                   ("cam2", "127.0.0.46", 2, {"node_offset_s": OFF})],
                  poll=0.3, threaded=True, nav=None, imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.wait_state("cam2", "CAM_CONNECTED")
        got = wait_for(lambda: all((m.clock_offset_s() or 0) > OFF - 0.05
                                   for m in env.monitors), 12)
        info = env.mon("cam1").clock_offset_info()
        check("the monitors measure the node-minus-host offset (contract C1)",
              got and abs((env.mon("cam1").clock_offset_s() or 0) - OFF) < 0.05,
              "offset=%s n=%s rtt_best=%s"
              % (info.get("offset_s"), info.get("n"), info.get("rtt_ms_best")))

        t0 = time.time()
        rep = env.runmgr.capture_once()
        check("both cameras fired", rep.get("ok") is True, json.dumps(rep)[:180])
        check("the shot records the offset it was scheduled with",
              abs(rep.get("host_offset_s", 0) - OFF) < 0.05,
              "host_offset_s=%s" % rep.get("host_offset_s"))

        # The node's own clock at dispatch. What must survive is the FOCUS
        # lead: piagent asserts FOCUS focus_lead_ms ahead of at_epoch and a
        # late arrival does not shorten it - the TRIGGER simply lands late by
        # the whole overrun, which is direct inter-camera skew.
        node_now = t0 + OFF
        lead_s = runmod.RunManager.FOCUS_LEAD_MS / 1000.0
        margins = {}
        for name, res in (rep.get("results") or {}).items():
            at_epoch = rep["target"] - (res.get("lead_ms") or 0) / 1000.0
            margins[name] = round(at_epoch - node_now, 4)
        check("the at_epoch every node receives is still a full FOCUS lead "
              "in its OWN future",
              bool(margins) and all(v >= lead_s for v in margins.values()),
              "margins=%s need>=%.3f" % (json.dumps(margins), lead_s))
        check("ONE common offset is used, so it cancels out of the skew",
              len(set(round(v, 3) for v in margins.values())) == 1,
              json.dumps(margins))
        check("a host clock off the nodes is said out loud",
              any(e["kind"] == "host_clock" for e in env.evs()),
              "%d host_clock events"
              % len([e for e in env.evs() if e["kind"] == "host_clock"]))
    finally:
        env.close()


# ===========================================================================
# R2b — a node-clock capture instant is converted before it is used
# ===========================================================================
def _r2b_capture_instant_converted(opts):
    sect("R2b a GPIO edge is converted to the host domain before nav/datetime")
    OFF = 0.25
    nav = RecordingNav()
    with _UseOffsetNodes():
        env = Env([("cam1", "127.0.0.47", 1, {"node_offset_s": OFF})],
                  poll=0.3, threaded=True, nav=nav, imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        wait_for(lambda: (env.mon("cam1").clock_offset_s() or 0) > OFF - 0.05, 12)
        r = env.runmgr.start({"label": "r2b", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)

        nav.asked = []
        t_host = time.time()
        SEQ = 4242
        # A scheduled fire: the command is a NODE instant (that is what
        # capture_once now queues) and piagent's edge lands on the same clock.
        w.note_command(t_host + OFF, path="gpio", fire_seq=SEQ, edge_seq=0)
        node.push_edge(epoch=t_host, fire_seq=SEQ)     # -> stored at +OFF
        # exif=False so this test exercises the edge path alone.
        name = node.add_frame(epoch=t_host, exif=False, name="ILX07777.JPG")
        got = wait_for(lambda: len(_rows(root, "cam1")[1]) >= 1, 15)
        _hdr, rows = _rows(root, "cam1")
        check("the injected frame produced a flight_log row", got and bool(rows),
              "%d rows" % len(rows))
        if not rows:
            return
        row = rows[-1]
        check("its capture instant came from the GPIO edge",
              row["capture_source"] == "gpio_edge", row["capture_source"])

        mine = [e for e in _index(env, root) if e.get("orig") == name]
        check("the indexed capture epoch is the HOST instant, not the node one",
              bool(mine) and abs(mine[-1]["epoch"] - t_host) < 0.05,
              "epoch=%s host=%.3f node=%.3f"
              % (mine[-1]["epoch"] if mine else None, t_host, t_host + OFF))
        near = [a for a in nav.asked if abs(a - t_host) < 0.05]
        wrong = [a for a in nav.asked if abs(a - (t_host + OFF)) < 0.05]
        check("nav was looked up at the converted (host) instant",
              bool(near) and not wrong,
              "asked deltas=%s (want ~0.000, not +%.3f)"
              % ([round(a - t_host, 3) for a in nav.asked], OFF))
        off_s, _src = env.runmgr.time_base()
        # Stamp it from the frame's OWN indexed epoch, not from the test's
        # t_host. Both are host-domain instants, but they differ by the residual
        # of the clock-offset ESTIMATE (sub-millisecond on loopback fakes, where
        # the true offset is knowable but the measured one is noise), and
        # _fmt_dt rounds to CENTISECONDS - so comparing two rounded strings for
        # exact equality is a hard 5 ms boundary standing in for a tolerance,
        # and any capture landing within the residual of a centisecond flips
        # one of them. Observed 2026-08-24 on an idle serial gate run:
        # "260824_071437.23 vs 260824_071437.24", a 0.43 ms difference
        # (epoch=...677.234574 vs host=...677.235) reported as a wrong datetime,
        # while the sibling check above passed the same pair against its 0.05 s
        # tolerance. Against the indexed epoch this is an exact identity the
        # code either satisfies or does not: datetime == _fmt_dt(epoch + gps).
        # The "converted, not node-domain" half of the contract is not weakened
        # - it is carried by the check above, which pins that same indexed epoch
        # to the host instant within 0.05 s against a defect worth OFF (0.25 s).
        want = runmod._fmt_dt(mine[-1]["epoch"] + off_s)
        check("the datetime column is stamped from the converted instant",
              row["datetime"] == want, "%s vs %s" % (row["datetime"], want))
        terr = row["time_err_ms"]
        check("time_err_ms carries the offset's own uncertainty, not a bare 0",
              terr not in ("", None) and float(terr) > 0.0,
              "time_err_ms=%r" % terr)
        doc = json.load(open(os.path.join(root, "run.json")))
        clk = doc.get("clock") or {}
        check("run.json records which conversion was applied",
              clk.get("applied") is True
              and abs((clk.get("node_offsets_s") or {}).get("cam1", 0) - OFF)
              < 0.05, json.dumps(clk))
    finally:
        env.close()


# ===========================================================================
# R3 — one dropped frame must not shift every later frame
# ===========================================================================
def _r3_dropped_frame_no_shift(opts):
    sect("R3 a dropped frame at 2 Hz does not shift the frames behind it")
    env = Env([("cam1", "127.0.0.48", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r3", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)
        # The run is a 2 Hz survey line as far as _claim_tolerance is
        # concerned; the grid itself is driven by hand so the injection is
        # deterministic.
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5
        check("the run reports a 2 Hz shot period",
              abs(env.runmgr.shot_period() - 0.5) < 1e-9)
        # A known camera-clock offset: EXIF is then the frame's own instant.
        env.timesync.set_exif_offset("cam1", 0.0)

        PERIOD = 0.5
        t0 = time.time() - 2.0
        want = {}
        for k in range(4):
            ep = t0 + k * PERIOD
            seq = 9100 + k
            w.note_command(ep, path="gpio", fire_seq=seq, edge_seq=0)
            node.push_edge(epoch=ep, fire_seq=seq)
            if k == 1:
                continue        # shot 1's frame never reaches the spool
            node.add_frame(epoch=ep, name="ILX081%02d.JPG" % k)
            want["ILX081%02d.JPG" % k] = ep
        got = wait_for(lambda: len(_rows(root, "cam1")[1]) >= 3, 15)
        by_orig = {e["orig"]: e for e in _index(env, root)}
        check("all three landed frames were indexed",
              bool(got) and all(nm in by_orig for nm in want),
              "indexed=%s" % sorted(by_orig))
        bad = {nm: round(by_orig[nm]["epoch"] - ep, 3)
               for nm, ep in want.items()
               if nm in by_orig and abs(by_orig[nm]["epoch"] - ep) > 0.05}
        check("every frame keeps its OWN capture instant after the drop",
              not bad, "off by %s (a shift of -%.2f s is one shot period)"
              % (json.dumps(bad), PERIOD))
        check("the dropped fire is orphaned, not inherited by the next frame",
              w.stats().get("orphan_fires", 0) >= 1, json.dumps(w.stats()))
        check("every landed frame is still dated from its own GPIO edge",
              all(by_orig[nm]["src"] == "gpio_edge" for nm in want
                  if nm in by_orig),
              json.dumps({nm: by_orig[nm]["src"] for nm in want
                          if nm in by_orig}))
        try:
            sub, whole = w._claim_tolerance(True), w._claim_tolerance(False)
        except TypeError as e:      # pre-fix: the tolerance ignores SubSec
            sub, whole = None, None
            detail = str(e)
        else:
            detail = "subsec=%.3f whole-second=%.3f" % (sub, whole)
        check("SubSec EXIF tightens the claim tolerance below one shot period",
              sub is not None and sub < PERIOD and whole >= 1.5, detail)
    finally:
        env.close()


# ===========================================================================
# R4 — RAW+JPEG is ONE release: one claim, one edge, one row
# ===========================================================================
def _r4_raw_jpeg_one_release(opts):
    sect("R4 a RAW+JPEG pair is one exposure: one claim, one flight_log row")
    env = Env([("cam1", "127.0.0.49", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r4", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5
        env.timesync.set_exif_offset("cam1", 0.0)

        t0 = time.time() - 2.0
        # Shot A delivers both halves; shot B delivers only a JPEG. Pre-fix the
        # .ARW claimed shot A's command (sorted() puts "A" before "J") and A's
        # .JPG took shot B's command and B's edge, one period late.
        for k, seq in ((0, 9200), (1, 9201)):
            w.note_command(t0 + k * 0.5, path="gpio", fire_seq=seq, edge_seq=0)
            node.push_edge(epoch=t0 + k * 0.5, fire_seq=seq)
        node.add_frame(epoch=t0, name="ILX09001.ARW")
        node.add_frame(epoch=t0, name="ILX09001.JPG")
        node.add_frame(epoch=t0 + 0.5, name="ILX09002.JPG")

        # Three FILES, two exposures. Waits on the files and the rows, not on
        # `pulled`: that counts exposures, and a count of 3 never comes.
        def _landed():
            fs = os.listdir(os.path.join(root, "cam1"))
            return (len([f for f in fs if f.lower().endswith(".jpg")]) >= 2
                    and any(f.lower().endswith(".arw") for f in fs)
                    and len(_rows(root, "cam1")[1]) >= 2)
        wait_for(_landed, 15)
        _hdr, rows = _rows(root, "cam1")
        by_orig = {e["orig"]: e for e in _index(env, root)}
        check("the RAW half adds no second flight_log row",
              len(rows) == 2,
              "%d rows: %s" % (len(rows), [x["filename"] for x in rows]))
        check("only the JPEG halves are indexed",
              sorted(by_orig) == ["ILX09001.JPG", "ILX09002.JPG"],
              str(sorted(by_orig)))
        ok1 = ("ILX09001.JPG" in by_orig
               and abs(by_orig["ILX09001.JPG"]["epoch"] - t0) < 0.05)
        ok2 = ("ILX09002.JPG" in by_orig
               and abs(by_orig["ILX09002.JPG"]["epoch"] - (t0 + 0.5)) < 0.05)
        check("each release keeps its own command and edge (no double claim)",
              ok1 and ok2, "A=%s (want %.3f)  B=%s (want %.3f)"
              % ((by_orig.get("ILX09001.JPG") or {}).get("epoch"), t0,
                 (by_orig.get("ILX09002.JPG") or {}).get("epoch"), t0 + 0.5))
        files = sorted(os.listdir(os.path.join(root, "cam1")))
        raws = [f for f in files if f.lower().endswith(".arw")]
        jpgs = [f for f in files if f.lower().endswith(".jpg")]
        check("the RAW is archived beside its JPEG under the same stem",
              len(raws) == 1 and len(jpgs) == 2
              and os.path.splitext(raws[0])[0] in
              [os.path.splitext(j)[0] for j in jpgs], str(files))
    finally:
        env.close()


# ===========================================================================
# R5 — a stopped run's calibration thread must not arm the next run
# ===========================================================================
def _r5_calibration_arms_only_its_own_run(opts):
    sect("R5 a stopped run's calibration thread does not arm the NEXT run")
    env = Env([("cam1", "127.0.0.50", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        # Run A: auto-capture at 2 s, stopped before its calibration finished.
        env.runmgr.start({"label": "runA", "interval_s": 2.0,
                          "auto_capture": True, "calibrate": False})
        run_a = env.runmgr.active
        cfg_a = dict(run_a["config"])
        env.runmgr.stop()
        # Run B: a different line entirely, no auto-capture at all.
        env.runmgr.start({"label": "runB", "interval_s": 0.6,
                          "auto_capture": False, "calibrate": False})
        run_b = env.runmgr.active
        _wait_calibrated(env, 20)
        seq0 = env.seq()
        # A's calibration thread finally reaches its finally block.
        try:
            env.runmgr._start_capture_loop(cfg_a, run_a)
        except TypeError:
            # Pre-fix: the loop is not tagged with the run it belongs to at
            # all, so it arms whatever happens to be active. That IS the
            # defect - drive it and let the checks below report it.
            env.runmgr._start_capture_loop(cfg_a)
        time.sleep(0.8)
        armed = [e for e in env.evs(since=seq0)
                 if e["kind"] == "capture" and "loop started" in e["msg"]]
        check("no capture loop is armed on the run it does not belong to",
              not armed, json.dumps([e["msg"] for e in armed]))
        check("the run that IS active is untouched",
              env.runmgr.active is run_b
              and not (env.runmgr.active or {}).get("fired"),
              json.dumps((env.runmgr.active or {}).get("fired")))
        check("the refusal is journalled rather than silent",
              any("not arming" in e["msg"] for e in env.evs(since=seq0)))
        env.runmgr.stop()
    finally:
        env.close()


# ===========================================================================
# R7 — RunManager.start validates what it is handed
# ===========================================================================
def _r7_validation(opts):
    sect("R7 interval_s / frames / label are validated at run start")
    env = Env([("cam1", "127.0.0.51", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "bad", "interval_s": 0,
                              "auto_capture": True})
        check("interval_s 0 with auto_capture is refused",
              r.get("ok") is False and "interval_s" in (r.get("error") or ""),
              json.dumps(r))
        r = env.runmgr.start({"label": "bad", "interval_s": -3,
                              "auto_capture": True})
        check("a negative interval_s is refused", r.get("ok") is False,
              json.dumps(r))
        r = env.runmgr.start({"label": "bad", "interval_s": "soon",
                              "auto_capture": True})
        check("a non-numeric interval_s is refused", r.get("ok") is False,
              json.dumps(r))
        r = env.runmgr.start({"label": "bad", "interval_s": 7200,
                              "auto_capture": True})
        check("an interval_s beyond an hour is refused", r.get("ok") is False,
              json.dumps(r))
        r = env.runmgr.start({"label": "bad", "frames": -1})
        check("a negative frame budget is refused", r.get("ok") is False,
              json.dumps(r))
        r = env.runmgr.start({"label": "bad", "frames": 10 ** 9})
        check("an absurd frame budget is refused", r.get("ok") is False,
              json.dumps(r))
        check("no run was created by any of the refusals",
              env.runmgr.active is None)

        dirty = "line" + chr(10) + "7 " + chr(7)
        r = env.runmgr.start({"label": dirty, "interval_s": 0.4,
                              "auto_capture": False, "calibrate": False})
        check("a legal run still starts", r.get("ok") is True,
              json.dumps(r)[:140])
        lbl = (env.runmgr.active or {}).get("label", "")
        check("the label is sanitised of control characters",
              lbl and all(c.isprintable() for c in lbl), repr(lbl))
        # Mid-run master-clock warning: once per RUN, not once per process.
        env.timesync.feed_gps(time.time() + 12.0)
        env.runmgr._watch_timebase()
        env.runmgr._watch_timebase()
        n1 = len([e for e in env.evs() if e["kind"] == "timebase"])
        env.runmgr.stop()
        env.timesync.clear_gps()
        env.runmgr.start({"label": "second", "calibrate": False})
        env.timesync.feed_gps(time.time() + 30.0)
        env.runmgr._watch_timebase()
        n2 = len([e for e in env.evs() if e["kind"] == "timebase"])
        check("the mid-run clock warning is rate-limited per RUN, not per "
              "process", n1 == 1 and n2 == 2,
              "%d timebase events in run 1, %d after run 2" % (n1, n2))
        env.runmgr.stop()
        env.timesync.clear_gps()
    finally:
        env.close()


# ===========================================================================
# One transect, one clock: a straggler pull keeps the run's latched timebase
# ===========================================================================
def _timebase_latched_per_worker(opts):
    sect("a pull that outlives stop() keeps the RUN's timebase, not the live one")
    env = Env([("cam1", "127.0.0.57", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.runmgr.start({"label": "tb", "calibrate": False})
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        run_base = env.runmgr.time_base()
        latched = getattr(w, "_timebase", None)
        check("the worker latches the run's timebase when it is built",
              latched is not None and latched == run_base,
              "worker=%s run=%s" % (latched, run_base))
        env.runmgr.stop()
        # A GPS fix arrives after stop, while a straggler is still downloading.
        env.timesync.feed_gps(time.time() + 500.0)
        live = env.runmgr.time_base()
        check("stop() moves the LIVE base out from under the straggler",
              abs(live[0] - run_base[0]) > 100,
              "live=%.1f run=%.1f" % (live[0], run_base[0]))
        check("but the worker still stamps from the run's base",
              getattr(w, "_timebase", None) == run_base,
              "worker=%s" % (getattr(w, "_timebase", None),))
        env.timesync.clear_gps()
    finally:
        env.close()


# ===========================================================================
# R8 — index.jsonl is the complete record (contract C3)
# ===========================================================================
def _r8_index_jsonl(opts):
    sect("R8 index.jsonl holds every frame, past run.json's 2000-entry cap")
    env = Env([("cam1", "127.0.0.52", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r8", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        N = 2400
        real = env.runmgr._write_run_json
        env.runmgr._write_run_json = lambda *a, **k: None   # bulk, then once
        try:
            for i in range(N):
                env.runmgr.index_frame(1, "Cam1_x%04d.jpg" % i,
                                       "ILX%05d.JPG" % i, 1700000000.0 + i,
                                       "gpio_edge", node="cam1", path="gpio")
        finally:
            env.runmgr._write_run_json = real
        env.runmgr._write_run_json()
        path = os.path.join(root, "index.jsonl")
        recs = _index(env, root)
        mine = [e for e in recs if str(e.get("orig", "")).startswith("ILX0")]
        check("index.jsonl exists beside run.json", os.path.exists(path))
        check("every indexed frame is on its own line, none truncated",
              len(mine) >= N, "%d of %d entries" % (len(mine), N))
        check("the FIRST frame is still there (run.json's index is not)",
              bool(mine) and mine[0]["orig"] == "ILX00000.JPG",
              mine[0]["orig"] if mine else "-")
        check("each line carries the run.json index fields (contract C3)",
              bool(mine) and all(k in mine[0]
                                 for k in ("cam", "file", "orig", "epoch", "src")),
              json.dumps(mine[0]) if mine else "-")
        doc = json.load(open(os.path.join(root, "run.json")))
        check("run.json is unchanged: last 2000 entries, full frame count",
              len(doc["index"]) == 2000 and doc["frames"] >= N,
              "index=%d frames=%d" % (len(doc["index"]), doc["frames"]))
        check("run.json points readers at the complete index",
              doc.get("index_jsonl") == "index.jsonl", str(doc.get("index_jsonl")))
        n_before = len(recs)
        env.runmgr.stop()
        check("the index survives stop() intact",
              len(_index(None, root)) >= n_before,
              "%d -> %d" % (n_before, len(_index(None, root))))
    finally:
        env.close()


# ===========================================================================
# R10 — a dead strobe node must not hold every shot open
# ===========================================================================
def _r10_strobe_blackhole(opts):
    sect("R10 a standalone strobe on a wedged node does not hold the shot open")
    env = Env([("cam1", "127.0.0.53", 1), ("cam2", "127.0.0.54", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.wait_state("cam2", "CAM_CONNECTED")
        rs = env.runmgr.set_strobe({"enabled": True, "node": "cam2",
                                    "delta_ms": 10.0, "pulse_ms": 5})
        check("strobe configured on cam2", rs.get("ok") is True,
              json.dumps(rs)[:120])
        # The strobe node's CAMERA is gone but its Pi is still there and has
        # gone unresponsive - the case the standalone strobe path exists for.
        # A refused connection fails in microseconds; a wedged node is what
        # actually spends the timeout, so that is what this stages.
        env.node("cam2").connected = False
        env.node("cam2").set_fault("pia", hang_s=12.0)
        check("cam2 leaves CAM_CONNECTED but stays reachable",
              bool(wait_for(lambda: not env.mon("cam2").is_connected(), 15))
              and env.mon("cam2").state != rigcore.NodeMonitor.OFFLINE,
              env.mon("cam2").state)
        t0 = time.time()
        rep = env.runmgr.capture_once()
        dt = time.time() - t0
        check("capture_once is not held open by the wedged strobe node",
              dt < 4.0,
              "%.2f s (the camera fire path was cut to a fail-fast timeout; "
              "the standalone-strobe call kept a 10 s one -> 10.3 s per shot)"
              % dt)
        check("cam1 still fired",
              ((rep.get("results") or {}).get("cam1") or {}).get("ok") is True,
              json.dumps(rep)[:160])
        check("the failed flash is reported",
              any(e["kind"] == "strobe_fail" for e in env.evs()))
        env.node("cam2").clear_faults()
    finally:
        env.close()


def _r10_strobe_offline_skipped(opts):
    sect("R10 an OFFLINE strobe node is not called at all")
    env = Env([("cam1", "127.0.0.55", 1), ("cam2", "127.0.0.56", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.wait_state("cam2", "CAM_CONNECTED")
        env.runmgr.set_strobe({"enabled": True, "node": "cam2",
                               "delta_ms": 10.0, "pulse_ms": 5})
        env.node("cam2").down()
        check("cam2 reaches OFFLINE",
              bool(wait_for(lambda: env.mon("cam2").state
                            == rigcore.NodeMonitor.OFFLINE, 15)),
              env.mon("cam2").state)
        seq0 = env.seq()
        rep = env.runmgr.capture_once()
        check("cam1 still fired",
              ((rep.get("results") or {}).get("cam1") or {}).get("ok") is True,
              json.dumps(rep)[:160])
        check("the operator is told the shot is unlit, naming the offline node",
              any(e["kind"] == "strobe_fail" and "offline" in e["msg"]
                  and "unlit" in e["msg"] for e in env.evs(since=seq0)),
              json.dumps([e["msg"] for e in env.evs(since=seq0)
                          if e["kind"] == "strobe_fail"])[:200])
    finally:
        env.close()



# ---------------------------------------------------------------------------
# A node whose HARDWARE clock and whose /health clock disagree.
#
# OffsetNode above models a node that is simply on a different clock. This one
# models the thing the fleet actually looks like: the two Pis are chrony PEERS,
# so their true offset from this host is the SAME number, and what differs
# per node is the host's ESTIMATE of it. clock_offset_s() is a request/
# response midpoint, so an asymmetric path biases it by up to RTT/2 - live
# RTTs are cam1 ~2.9 ms and cam2 ~10.3 ms. `est_err_s` is that bias: the
# hardware (edges, IMU) stays on node_offset_s, /health reports
# node_offset_s + est_err_s.
# ---------------------------------------------------------------------------
class SkewNode(fakenode.FakeNode):

    def __init__(self, *a, node_offset_s=0.0, est_err_s=0.0, **kw):
        self.node_offset_s = float(node_offset_s)
        self.est_err_s = float(est_err_s)
        super().__init__(*a, **kw)

    def health(self):
        h = super().health()
        t = dict(h.get("time") or {})
        t["epoch"] = time.time() + self.node_offset_s + self.est_err_s
        h["time"] = t
        return h

    def push_edge(self, epoch=None, edge="fall", fire_seq=None):
        """`epoch` is given in HOST time; it is stored on the NODE clock."""
        ep = (time.time() if epoch is None else epoch) + self.node_offset_s
        return super().push_edge(epoch=ep, edge=edge, fire_seq=fire_seq)

    def push_imu(self, epoch=None, **over):
        """Same: piagent stamps the ring on the node's own clock."""
        ep = (time.time() if epoch is None else epoch) + self.node_offset_s
        return super().push_imu(epoch=ep, **over)


class _UseSkewNodes:
    def __enter__(self):
        self._prev = soaktest.FakeNode
        soaktest.FakeNode = SkewNode
        return self

    def __exit__(self, *a):
        soaktest.FakeNode = self._prev
        return False


def _wait_offsets(env, names, want, tol=0.05, timeout=15.0):
    """Wait until every named monitor has a settled clock estimate."""
    def ready():
        for n in names:
            o = env.mon(n).clock_offset_s()
            if o is None or abs(o - want[n]) > tol:
                return False
        return True
    return wait_for(ready, timeout)


# ===========================================================================
# R11a - the host<->fleet conversion must not enter the inter-camera skew
# ===========================================================================
def _r11a_pair_skew_free_of_clock_estimates(opts):
    sect("R11a one common clock offset, so the pair skew stays the rig's own")
    # Chrony peers: the TRUE offset is identical on both nodes. Only the
    # host's estimate of it differs, and only by the asymmetry of the path -
    # here 0 ms on cam1 and 50 ms on cam2, an exaggeration of the live
    # +/-RTT/2 (1.5 ms and 5.2 ms) so the mechanism is unmistakable rather
    # than lost in the fakes' own sub-millisecond noise.
    TRUE, ERR1, ERR2 = 0.25, 0.0, 0.050
    with _UseSkewNodes():
        env = Env([("cam1", "127.0.0.61", 1,
                    {"node_offset_s": TRUE, "est_err_s": ERR1}),
                   ("cam2", "127.0.0.62", 2,
                    {"node_offset_s": TRUE, "est_err_s": ERR2})],
                  poll=0.3, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        check("both cameras connected",
              env.wait_state("cam1", "CAM_CONNECTED")
              and env.wait_state("cam2", "CAM_CONNECTED"))
        check("the two nodes' clock ESTIMATES differ, as two real nodes' do",
              bool(_wait_offsets(env, ("cam1", "cam2"),
                                 {"cam1": TRUE + ERR1, "cam2": TRUE + ERR2},
                                 tol=0.02)),
              "cam1=%s cam2=%s" % (env.mon("cam1").clock_offset_s(),
                                   env.mon("cam2").clock_offset_s()))
        r = env.runmgr.start({"label": "r11a", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        for n in ("cam1", "cam2"):
            env.runmgr.workers[n].primed.wait(timeout=8)
            _drain_spool(env, n, env.runmgr.workers[n])

        # ONE stereo pair, exposed at the SAME true instant on both bodies:
        # the true inter-camera skew of this pair is exactly zero.
        t_host = time.time()
        SEQ = 5151
        for i, n in enumerate(("cam1", "cam2")):
            w = env.runmgr.workers[n]
            w.note_command(t_host + TRUE, path="gpio", fire_seq=SEQ,
                           edge_seq=0)
            env.node(n).push_edge(epoch=t_host, fire_seq=SEQ)
            env.node(n).add_frame(epoch=t_host, exif=False,
                                  name="ILXPAIR%d.JPG" % i)

        def _got():
            idx = _index(env, root)
            return {e["cam"]: e for e in idx
                    if str(e.get("orig", "")).startswith("ILXPAIR")}
        wait_for(lambda: len(_got()) == 2, 20)
        got = _got()
        check("both halves of the pair were indexed", len(got) == 2,
              json.dumps(sorted(got)))
        if len(got) != 2:
            return
        srcs = sorted(e.get("src") for e in got.values())
        check("both halves came from their own GPIO edge",
              srcs == ["gpio_edge", "gpio_edge"], str(srcs))
        eps = [e["epoch"] for e in got.values()]
        spread_ms = abs(eps[0] - eps[1]) * 1000.0
        # The run browser computes the displayed pair spread from exactly
        # these two numbers (rigcore.RunBrowser._pairs -> spread_src="index"),
        # and the product spec is 10 ms. Converting each camera's frame with
        # its OWN estimate puts (est_cam1 - est_cam2) straight into it: 50 ms
        # here, five times the whole budget, on a pair whose true skew is 0.
        check("the DISPLAYED pair skew carries no clock-estimate difference",
              spread_ms <= 3.0,
              "spread=%.1f ms (estimates differ by %.0f ms; true skew 0)"
              % (spread_ms, abs(ERR1 - ERR2) * 1000))
        errs = {r_["filename"]: r_["time_err_ms"]
                for _n in ("cam1", "cam2") for r_ in _rows(root, _n)[1]}
        check("both rows carry a non-zero, honest clock error bar",
              bool(errs) and all(v not in ("", None) and float(v) > 0.0
                                 for v in errs.values()),
              json.dumps(errs))
        doc = json.load(open(os.path.join(root, "run.json")))
        clk = doc.get("clock") or {}
        applied = clk.get("host_offset_s")
        check("run.json records the ONE offset that was applied",
              applied is not None
              and abs(applied - (TRUE + (ERR1 + ERR2) / 2.0)) < 0.02,
              json.dumps(clk))
        check("and still records each node's own estimate as diagnostics",
              len(clk.get("node_offsets_s") or {}) == 2,
              json.dumps(clk.get("node_offsets_s")))
    finally:
        env.close()


# ===========================================================================
# R11b - host -> node -> host must be the identity for one row
# ===========================================================================
def _r11b_imu_window_round_trip(opts):
    sect("R11b the IMU window is centred on the capture instant, exactly")
    # The IMU lives on cam1; the frame is taken by cam2. With per-node
    # offsets the capture instant leaves _capture_instant converted by cam2's
    # estimate and imu_snapshot converts it BACK with cam1's, so the +/-100 ms
    # window is centred (est_cam1 - est_cam2) away from the instant it is
    # supposed to be centred on and the nearest sample is the wrong one.
    TRUE, ERR1, ERR2 = 0.25, 0.0, 0.050
    with _UseSkewNodes():
        env = Env([("cam1", "127.0.0.63", 1,
                    {"node_offset_s": TRUE, "est_err_s": ERR1,
                     "has_imu": True}),
                   ("cam2", "127.0.0.64", 2,
                    {"node_offset_s": TRUE, "est_err_s": ERR2})],
                  poll=0.3, threaded=True, nav=FakeNav(), imu_node="cam1")
    try:
        check("both cameras connected",
              env.wait_state("cam1", "CAM_CONNECTED")
              and env.wait_state("cam2", "CAM_CONNECTED"))
        _wait_offsets(env, ("cam1", "cam2"),
                      {"cam1": TRUE + ERR1, "cam2": TRUE + ERR2}, tol=0.02)
        r = env.runmgr.start({"label": "r11b", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam2"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam2", w)

        t_host = time.time()
        SEQ = 6262
        # Three attitudes, 50 ms apart. THE one belonging to this frame is
        # yaw=111 at the capture instant; the neighbours are what a window
        # displaced by the difference of two clock estimates picks up instead.
        for dt, yaw in ((-0.050, 222.0), (0.0, 111.0), (0.050, 333.0)):
            env.node("cam1").push_imu(epoch=t_host + dt, yaw=yaw,
                                      heading=yaw + 1.0)
        w.note_command(t_host + TRUE, path="gpio", fire_seq=SEQ, edge_seq=0)
        env.node("cam2").push_edge(epoch=t_host, fire_seq=SEQ)
        env.node("cam2").add_frame(epoch=t_host, exif=False,
                                   name="ILXIMU01.JPG")
        wait_for(lambda: any(r_["filename"] for r_ in _rows(root, "cam2")[1]),
                 20)
        rows = _rows(root, "cam2")[1]
        check("the frame produced a flight_log row", bool(rows),
              "%d rows" % len(rows))
        if not rows:
            return
        row = rows[-1]
        check("its capture instant came from the GPIO edge",
              row["capture_source"] == "gpio_edge", row["capture_source"])
        check("the attitude written into the row is the sample AT the capture "
              "instant, not a neighbour 50 ms away",
              row["yaw"] == "111.0",
              "yaw=%r (222.0 = 50 ms early, 333.0 = 50 ms late, '' = the "
              "window missed the ring entirely)" % row["yaw"])
    finally:
        env.close()


# ===========================================================================
# R11c - a baseline listing that did not happen is not an empty spool
# ===========================================================================
def _r11c_baseline_listing_is_never_guessed(opts):
    sect("R11c a failed shot listing must not baseline the spool as empty")
    env = Env([("cam1", "127.0.0.65", 1)], poll=0.3, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    real_http_json = rigcore.http_json
    gate = {"until": 0.0}

    def flaky(url, *a, **kw):
        # ilxctl restarting under systemd: /api/status keeps answering (the
        # node is CONNECTED and joins the run) while the listing does not.
        if url.endswith("/api/shots") and time.time() < gate["until"]:
            return {"ok": False, "error": "injected: /api/shots unavailable"}
        return real_http_json(url, *a, **kw)

    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        # What is already in the save dir before the transect starts: bench
        # frames, a calibration frame nobody dumped, the last run's leftovers.
        old = [node.add_frame(epoch=time.time() - 600 + i,
                              name="ILXOLD%02d.JPG" % i) for i in range(3)]
        rigcore.http_json = flaky
        gate["until"] = time.time() + 1.4
        r = env.runmgr.start({"label": "r11c", "calibrate": False})
        root = r["root"]
        check("the run started with the camera as a member",
              r.get("ok") and r.get("nodes") == ["cam1"], json.dumps(r)[:160])
        w = env.runmgr.workers.get("cam1")
        check("a worker was started for it", w is not None)
        if w is None:
            return
        got = wait_for(lambda: w.primed.is_set(), 20)
        check("the worker got past its baseline", bool(got))
        check("the failed listing was journalled, not swallowed",
              any(e["kind"] == "pull" and "shot listing" in e["msg"]
                  for e in env.evs()),
              str([e["msg"][:70] for e in env.evs() if e["kind"] == "pull"]))
        # Give it several poll cycles with the listing healthy again.
        new = node.add_frame(epoch=time.time(), name="ILXNEW01.JPG")
        wait_for(lambda: len(_rows(root, "cam1")[1]) >= 1, 20)
        time.sleep(1.5)
        idx = _index(env, root)
        pulled = sorted(str(e.get("orig")) for e in idx)
        check("frames that were on the node BEFORE the run are not survey data",
              not any(o in pulled for o in old),
              "pulled=%s (pre-existing: %s)" % (pulled, old))
        check("and the frame taken DURING the run still is",
              new in pulled, "pulled=%s" % pulled)
    finally:
        rigcore.http_json = real_http_json
        env.close()


# ===========================================================================
# R12a - a RAW half listed AFTER its JPEG was written takes no command
#
# With RAW+J PC Save delivering both halves, a ~320 KB Small JPEG completes
# and is listed while the ~32 MB LossLessL ARW of the SAME release is still
# transferring, so at a 0.4 s poll the two halves land in DIFFERENT listings.
# _release_claim pops the stem as soon as no other file of it is in cmd_epoch,
# so the late ARW found an empty _stem_cmd, popped the NEXT shot's command,
# popped that shot's EXPOSURE edge by fire_seq, and then DESTROYED the command
# as a sibling (requeue=False) - shifting every later frame by a shot period.
# ===========================================================================
def _r12a_late_raw_sibling_takes_no_command(opts):
    sect("R12a a RAW half listed after its JPEG claims no command of its own")
    env = Env([("cam1", "127.0.0.66", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r12a", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5
        env.timesync.set_exif_offset("cam1", 0.0)

        PERIOD = 0.5
        t0 = time.time() - 2.0
        want = {}
        for k in range(3):
            ep = t0 + k * PERIOD
            w.note_command(ep, path="gpio", fire_seq=9300 + k, edge_seq=0)
            node.push_edge(epoch=ep, fire_seq=9300 + k)
            want["ILX093%02d.JPG" % k] = ep
        # Shot 0's ~320 KB JPEG completes and is listed first; its ~32 MB
        # LossLessL half is still on the wire.
        node.add_frame(epoch=t0, name="ILX09300.JPG")
        wait_for(lambda: "ILX09300.JPG" in w.seen, 12)
        # ...and only now is the RAW half listed. exif=False because Pillow
        # has no Sony raw decoder, so an ARW's own EXIF is unreadable - which
        # is what let it accept a command by fire_seq identity with nothing to
        # cross-check against.
        node.add_frame(epoch=t0, name="ILX09300.ARW", exif=False)
        wait_for(lambda: "ILX09300.ARW" in w.seen, 12)
        # Shots 1 and 2 land normally behind it.
        node.add_frame(epoch=t0 + PERIOD, name="ILX09301.JPG")
        node.add_frame(epoch=t0 + 2 * PERIOD, name="ILX09302.JPG")

        wait_for(lambda: len(_rows(root, "cam1")[1]) >= 3, 20)
        by_orig = {e["orig"]: e for e in _index(env, root)}
        check("the late RAW half adds no flight_log row",
              len(_rows(root, "cam1")[1]) == 3,
              "%d rows: %s" % (len(_rows(root, "cam1")[1]),
                               [x["filename"] for x in _rows(root, "cam1")[1]]))
        check("only the JPEG halves are indexed",
              "ILX09300.ARW" not in by_orig, str(sorted(by_orig)))
        bad = {nm: round(by_orig[nm]["epoch"] - ep, 3)
               for nm, ep in want.items()
               if nm in by_orig and abs(by_orig[nm]["epoch"] - ep) > 0.05}
        check("the frames behind it keep their OWN capture instants",
              len(by_orig) >= 3 and not bad,
              "off by %s (a shift of -%.2f s is one shot period)"
              % (json.dumps(bad), PERIOD))
        check("every frame is still dated from its own GPIO edge",
              all(by_orig[nm]["src"] == "gpio_edge" for nm in want
                  if nm in by_orig),
              json.dumps({nm: by_orig.get(nm, {}).get("src") for nm in want}))
        check("no command was destroyed and nothing was orphaned",
              w.stats().get("orphan_fires", 0) == 0
              and w.stats().get("queued_commands", 0) == 0,
              json.dumps(w.stats()))
        check("every frame keeps the fire identity of its own command",
              all(by_orig[nm].get("path") == "gpio" for nm in want
                  if nm in by_orig),
              json.dumps({nm: by_orig.get(nm, {}).get("path") for nm in want}))
        check("no scheduled frame was mistaken for an unscheduled one",
              not any(e["kind"] == "capture" and "unscheduled" in e["msg"]
                      for e in env.evs()),
              str([e["msg"][:70] for e in env.evs()
                   if e["kind"] == "capture"]))
        files = sorted(os.listdir(os.path.join(root, "cam1")))
        raws = [f for f in files if f.lower().endswith(".arw")]
        jpgs = [os.path.splitext(f)[0] for f in files
                if f.lower().endswith(".jpg")]
        check("the RAW is archived beside its JPEG under the same stem",
              len(raws) == 1 and os.path.splitext(raws[0])[0] in jpgs,
              str(files))
    finally:
        env.close()


# ===========================================================================
# R12b - one delivery gap must not latch the command/frame cross-check
#
# _plaus_strikes counted ORPHANED COMMANDS, not frames that failed the check,
# so ONE ~3 s delivery gap (six unclaimed fires at 2 Hz, all younger than
# CMD_MAX_AGE_S) burned the whole budget across two returning frames.
# _plaus_off then latched for the life of the run and every later frame took
# the head-of-queue command, matched THAT command's edge by fire_seq identity
# and was written capture_source=gpio_edge / time_err_ms 0 while being a whole
# shot period wrong - the exact defect this pass exists to remove.
# ===========================================================================
def _r12b_delivery_gap_does_not_latch_the_cross_check(opts):
    sect("R12b a six-frame delivery gap does not latch the cross-check off")
    env = Env([("cam1", "127.0.0.67", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r12b", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5
        env.timesync.set_exif_offset("cam1", 0.0)

        PERIOD, N_BEFORE, N_LOST, N_AFTER = 0.5, 2, 6, 6
        want = {}
        for k in range(N_BEFORE + N_LOST + N_AFTER):
            ep = time.time()
            seq = 9400 + k
            w.note_command(ep, path="gpio", fire_seq=seq, edge_seq=0)
            node.push_edge(epoch=ep, fire_seq=seq)
            # The body keeps exposing and the harness keeps seeing the edge;
            # for N_LOST shots the file never reaches the spool (PC-save
            # hiccup, card write burst, USB stall).
            if not (N_BEFORE <= k < N_BEFORE + N_LOST):
                nm = "ILX094%02d.JPG" % k
                node.add_frame(epoch=ep, name=nm)
                want[nm] = ep
            time.sleep(PERIOD)
        wait_for(lambda: len({e["orig"] for e in _index(env, root)}
                             & set(want)) >= len(want), 25)
        by_orig = {e["orig"]: e for e in _index(env, root)}
        check("every landed frame was indexed",
              all(nm in by_orig for nm in want),
              "missing %s" % sorted(set(want) - set(by_orig)))
        check("the cross-check is still on after the gap",
              not w._plaus_off,
              "plaus_off=%s strikes=%s" % (w._plaus_off, w._plaus_strikes))
        bad = {nm: round(by_orig[nm]["epoch"] - ep, 3)
               for nm, ep in want.items()
               if nm in by_orig and abs(by_orig[nm]["epoch"] - ep) > 0.05}
        check("no frame after the gap is shifted by a shot period",
              not bad, "off by %s (one period is %.2f s)"
              % (json.dumps(bad), PERIOD))
    finally:
        env.close()


def _r12b2_untrusted_claim_is_never_gpio_edge(opts):
    sect("R12b2 with the cross-check off, a claim is not proof of ownership")
    env = Env([("cam1", "127.0.0.68", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r12b2", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5
        env.timesync.set_exif_offset("cam1", 0.0)
        # However the latch got set, the latched state must not stamp a frame
        # from a command it cannot be shown to own.
        w._plaus_off = True

        PERIOD = 0.5
        t_frame = time.time()
        # The queue head is a DIFFERENT exposure, one period earlier, and its
        # own EXPOSURE edge is in the buffer under its fire_seq. Pre-fix the
        # frame matched that edge by identity and was written gpio_edge at it.
        w.note_command(t_frame - PERIOD, path="gpio", fire_seq=9500,
                       edge_seq=0)
        node.push_edge(epoch=t_frame - PERIOD, fire_seq=9500)
        node.push_edge(epoch=t_frame, fire_seq=9501)
        node.add_frame(epoch=t_frame, name="ILX09600.JPG")
        wait_for(lambda: any(e["orig"] == "ILX09600.JPG"
                             for e in _index(env, root)), 20)
        got = {e["orig"]: e for e in _index(env, root)}.get("ILX09600.JPG")
        check("the frame was indexed", got is not None)
        if got is None:
            return
        off = round(got["epoch"] - t_frame, 3)
        check("it is NOT stamped from the queue head's exposure",
              abs(off) < 0.05,
              "src=%s off=%+.3f s (one period is %.2f s)"
              % (got.get("src"), off, PERIOD))
        rows = {x["filename"]: x for x in _rows(root, "cam1")[1]}
        row = rows.get(got["file"])
        check("and its error bar is not a claimed-zero",
              row is not None and row.get("time_err_ms") not in (None, "")
              and float(row["time_err_ms"]) > 0.0,
              json.dumps({"src": got.get("src"),
                          "time_err_ms": (row or {}).get("time_err_ms")}))
    finally:
        env.close()


# ===========================================================================
# R12c - the clock offset is the SHOT's, not the wall clock's
#
# fleet_clock_offset()'s latch is keyed on wall time (FLEET_CLOCK_TTL_S), and
# the two halves of one pair are pulled by two independent worker threads, so
# a fire boundary falling between their conversions converted cam1 with L_k
# and cam2 with L_(k+1). That difference lands straight in the pair spread the
# run browser draws from these very epochs.
# ===========================================================================
def _r12c_pair_offset_is_shot_scoped(opts):
    sect("R12c both halves of a pair convert with their own SHOT's offset")
    env = Env([("cam1", "127.0.0.69", 1), ("cam2", "127.0.0.70", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        check("both cameras connected",
              env.wait_state("cam1", "CAM_CONNECTED")
              and env.wait_state("cam2", "CAM_CONNECTED"))
        r = env.runmgr.start({"label": "r12c", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        ws = {n: env.runmgr.workers[n] for n in ("cam1", "cam2")}
        for n, w in ws.items():
            w.primed.wait(timeout=8)
            _drain_spool(env, n, w)
            env.timesync.set_exif_offset(n, 0.0)
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5

        SHOT_OFF = 0.187          # this shot's latched fleet offset
        STEP = 0.500              # the host clock steps between the two pulls
        t_exp = time.time()
        for i, n in enumerate(("cam1", "cam2")):
            try:
                ws[n].note_command(t_exp, path="gpio", fire_seq=9700 + i,
                                   edge_seq=0, host_offset=SHOT_OFF,
                                   clock_err=0.002)
            except TypeError as e:
                # Pre-fix: nothing carries the shot's offset to the worker.
                check("the shot's clock offset travels with its command",
                      False, str(e))
                return
            env.node(n).push_edge(epoch=t_exp, fire_seq=9700 + i)
        env.runmgr._fleet_clock = (SHOT_OFF, time.time())
        env.node("cam1").add_frame(epoch=t_exp, name="ILX09700.JPG")
        wait_for(lambda: any(e["orig"] == "ILX09700.JPG"
                             for e in _index(env, root)), 20)
        # The wall-clock latch moves before cam2's half is pulled - a host
        # clock step, or simply the next shot's own latch.
        env.runmgr._fleet_clock = (SHOT_OFF + STEP, time.time())
        env.node("cam2").add_frame(epoch=t_exp, name="ILX09800.JPG")
        wait_for(lambda: any(e["orig"] == "ILX09800.JPG"
                             for e in _index(env, root)), 20)
        idx = {e["orig"]: e for e in _index(env, root)}
        got = [idx.get("ILX09700.JPG"), idx.get("ILX09800.JPG")]
        check("both halves of the pair were indexed", all(got),
              str(sorted(idx)))
        if not all(got):
            return
        spread_ms = abs(got[0]["epoch"] - got[1]["epoch"]) * 1000.0
        check("the DISPLAYED pair spread carries no latch difference",
              spread_ms <= 10.0,
              "spread=%.1f ms (the latch moved %.0f ms between the two pulls; "
              "true skew 0)" % (spread_ms, STEP * 1000))
        check("each row records the offset it was actually converted with",
              all(abs((e.get("clk_off") if e.get("clk_off") is not None
                       else -99) - SHOT_OFF) < 1e-6 for e in got),
              json.dumps([e.get("clk_off") for e in got]))
    finally:
        env.close()


# ===========================================================================
# R12d - a resume probe abandoned by its timeout may still have fired
#
# The whole quarantine hung off `ok`. piagent's own comment names the case
# ("abandoned by the host's 2 s timeout"): the TRIGGER pulse is placed and the
# shutter releases, but urlopen times out first, so no before/after diff was
# taken, end_calibration_fire ran at once, and the diagnostic exposure entered
# the transect as survey data with a real gpio_edge instant.
# ===========================================================================
def _r12d_abandoned_probe_frame_is_quarantined(opts):
    sect("R12d a probe fire abandoned by its timeout still quarantines its frame")
    env = Env([("cam1", "127.0.0.71", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    real_run_http_json = runmod.http_json
    node = env.node("cam1")

    def slow_fire(url, *a, **kw):
        if url.endswith(":8081/gpio/fire"):
            # The node executes the release and answers into a socket the host
            # has already given up on.
            time.sleep(runmod.RunManager.SYNC_LEAD_S + 0.2)
            node.add_frame(epoch=time.time(), name="ILXPROBE1.JPG")
            return {"ok": False, "error": "timed out", "_unreachable": True}
        return real_run_http_json(url, *a, **kw)

    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r12d", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        skipped0 = w.stats().get("skipped_calibration", 0)
        runmod.http_json = slow_fire
        ok, err = env.runmgr._probe_fire(env.mon("cam1"))
        check("the probe still reports failure, so the grid stays paused",
              ok is False, "ok=%s err=%s" % (ok, err))
        got = wait_for(lambda: "ILXPROBE1.JPG" in w.seen, 20)
        check("the probe's frame was dealt with", bool(got))
        idx = [e["orig"] for e in _index(env, root)]
        check("the abandoned probe's exposure is NOT in the transect",
              "ILXPROBE1.JPG" not in idx, "indexed: %s" % idx)
        check("it is counted as a calibration exposure, not a survey frame",
              w.stats().get("skipped_calibration", 0) > skipped0,
              json.dumps(w.stats()))
        check("and the journal says why",
              any(e["kind"] == "calibrate" and "abandoned" in e["msg"]
                  for e in env.evs()),
              str([e["msg"][:70] for e in env.evs()
                   if e["kind"] == "calibrate"]))
    finally:
        runmod.http_json = real_run_http_json
        env.close()


def _r12d2_probe_hold_outlives_its_own_listing(opts):
    sect("R12d2 the resume probe refreshes its puller hold while it waits")
    env = Env([("cam1", "127.0.0.72", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.runmgr.start({"label": "r12d2", "calibrate": False})
        _wait_calibrated(env, 20)
        m = env.mon("cam1")
        # begin_calibration_fire stores an ABSOLUTE expiry, and _probe_fire
        # takes its hold before a listing and a fire that both block - so the
        # naming loop outlives the quiet it was supposed to run under.
        env.runmgr.begin_calibration_fire("cam1")
        env.runmgr._cal_busy["cam1"] = time.time() + 0.15   # nearly expired
        namer = getattr(env.runmgr, "_name_probe_frame", None)
        if not check("the probe's naming wait is separable from the fire",
                     namer is not None,
                     "_name_probe_frame=%r (inlined in _probe_fire, with no "
                     "hold refresh, before the fix)" % namer):
            return
        t0 = time.time()
        namer(m, env.runmgr.spool_names(m))
        check("the puller was still held when the naming loop gave up",
              env.runmgr.calibration_quiet("cam1"),
              "quiet ended %.2fs before the loop did"
              % (time.time() - t0))
        check("and it says so when nothing new landed",
              any(e["kind"] == "calibrate" and "unaccounted" in e["msg"]
                  for e in env.evs()),
              str([e["msg"][:60] for e in env.evs()
                   if e["kind"] == "calibrate"]))
    finally:
        env.runmgr.end_calibration_fire("cam1")
        env.close()


# ===========================================================================
# R12e - a worker that cannot baseline keeps trying; it does not give up
#
# It used to RETURN, and nothing removed its entry from RunManager.workers, so
# _adopt_loop could never rebuild it while capture_once went on firing that
# camera every shot. Half the stereo line, with no recovery short of a restart.
# ===========================================================================
def _r12e_worker_retries_its_baseline(opts):
    sect("R12e a worker that cannot baseline keeps trying, and then pulls")
    env = Env([("cam1", "127.0.0.73", 1)], poll=0.3, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    real_http_json = rigcore.http_json
    gate = {"until": 0.0}

    def flaky(url, *a, **kw):
        # ilxctl is down for longer than the 6 x 0.5 s baseline budget while
        # /api/status keeps answering, so the camera is a run member.
        if url.endswith("/api/shots") and time.time() < gate["until"]:
            return {"ok": False, "error": "injected: /api/shots unavailable"}
        return real_http_json(url, *a, **kw)

    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        old = [node.add_frame(epoch=time.time() - 600 + i,
                              name="ILXOLD%02d.JPG" % i) for i in range(2)]
        # rigd has listed this node before this transect (an earlier run, or
        # an earlier pass of this one), which is what lets a late baseline
        # tell "already here" from "shot while the listing was down".
        env.runmgr.remember_frames("cam1", old)
        rigcore.http_json = flaky
        gate["until"] = time.time() + 6.0            # well past the 3 s budget
        r = env.runmgr.start({"label": "r12e", "calibrate": False})
        root = r["root"]
        w = env.runmgr.workers.get("cam1")
        check("a worker was started for the camera", w is not None)
        if w is None:
            return
        check("it gave up on its first baseline, loudly",
              wait_for(lambda: any(e["kind"] == "pull" and e["sev"] == "error"
                                   and "NOT being pulled" in e["msg"]
                                   for e in env.evs()), 15))
        check("but the worker is still alive and still the camera's worker",
              soaktest.safe_alive(w) and env.runmgr.workers.get("cam1") is w)
        # The camera goes on being fired the whole time the listing is down,
        # so its frames pile up on the node. Those are survey data.
        during = node.add_frame(epoch=time.time(), name="ILXGAP01.JPG")
        wait_for(lambda: time.time() > gate["until"], 10)
        got = wait_for(lambda: len(_rows(root, "cam1")[1]) >= 1, 25)
        check("the worker baselines on a later attempt and pulls again",
              bool(got), json.dumps(w.stats()))
        pulled = sorted(str(e.get("orig")) for e in _index(env, root))
        check("frames that were on the node BEFORE the run are not survey data",
              not any(o in pulled for o in old),
              "pulled=%s (pre-existing: %s)" % (pulled, old))
        check("and a frame shot while the listing was down still is",
              during in pulled, "pulled=%s" % pulled)
    finally:
        rigcore.http_json = real_http_json
        env.close()


# ===========================================================================
# R12f - the run record must not turn "never measured" into 0.000000, and
# must carry the offset each frame was ACTUALLY converted with
# ===========================================================================
def _r12f_run_json_clock_record(opts):
    sect("R12f run.json's clock record is honest about what was measured")
    TRUE = 0.25
    with _UseSkewNodes():
        env = Env([("cam1", "127.0.0.74", 1,
                    {"node_offset_s": TRUE, "est_err_s": 0.0}),
                   ("cam2", "127.0.0.75", 2,
                    {"node_offset_s": TRUE, "est_err_s": 0.0})],
                  poll=0.3, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        check("both cameras connected",
              env.wait_state("cam1", "CAM_CONNECTED")
              and env.wait_state("cam2", "CAM_CONNECTED"))
        _wait_offsets(env, ("cam1", "cam2"), {"cam1": TRUE, "cam2": TRUE},
                      tol=0.02)
        r = env.runmgr.start({"label": "r12f", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        for n in ("cam1", "cam2"):
            w = env.runmgr.workers[n]
            w.primed.wait(timeout=8)
            _drain_spool(env, n, w)
        # cam2's piagent answers without a `time` block (an older piagent, or
        # one whose clock read failed), so nothing is ever sampled for it. It
        # is still a run member and still delivering frames.
        env.mon("cam2").stop()
        env.mon("cam2")._clock_hist.clear()
        w1 = env.runmgr.workers["cam1"]
        # Node-domain command and edge, host-domain frame, as R11a does it.
        ep = time.time()
        try:
            w1.note_command(ep + TRUE, path="gpio", fire_seq=9900, edge_seq=0,
                            host_offset=TRUE, clock_err=0.001)
        except TypeError:            # pre-fix: no per-shot offset at all
            w1.note_command(ep + TRUE, path="gpio", fire_seq=9900, edge_seq=0)
        env.node("cam1").push_edge(epoch=ep, fire_seq=9900)
        env.node("cam1").add_frame(epoch=ep, exif=False, name="ILX09900.JPG")
        wait_for(lambda: any(e["orig"] == "ILX09900.JPG"
                             for e in _index(env, root)), 20)
        env.runmgr._write_run_json()
        doc = json.load(open(os.path.join(root, "run.json")))
        clk = doc.get("clock") or {}
        offs = clk.get("node_offsets_s") or {}
        check("an unmeasured node is published as null, not 0.000000",
              "cam2" in offs and offs["cam2"] is None,
              json.dumps(offs))
        check("a measured node still carries its own estimate",
              offs.get("cam1") is not None
              and abs(offs["cam1"] - TRUE) < 0.02, json.dumps(offs))
        span = clk.get("applied_offset_s") or {}
        check("run.json records the SPAN of offsets actually applied",
              span.get("first") is not None
              and abs(span["first"] - TRUE) < 1e-6
              and span.get("moved_ms") is not None, json.dumps(clk))
        rec = {e["orig"]: e for e in _index(env, root)}.get("ILX09900.JPG")
        check("and each frame carries the offset it was converted with",
              rec is not None and rec.get("clk_off") is not None
              and abs(rec["clk_off"] - TRUE) < 1e-6, json.dumps(rec))
    finally:
        env.close()


# ===========================================================================
# R12g - the card-review fallback's 0.0 was a domain sentinel, but
# _write_flight also handed it to imu_snapshot as the OUTWARD host->node
# conversion offset, centring the +/-100 ms IMU window a whole fleet offset
# into the node's past.
# ===========================================================================
def _r12g_card_review_imu_window(opts):
    sect("R12g a card-review row does not query the IMU a fleet offset early")
    TRUE = 0.25
    with _UseSkewNodes():
        env = Env([("cam1", "127.0.0.76", 1,
                    {"node_offset_s": TRUE, "est_err_s": 0.0,
                     "has_imu": True})],
                  poll=0.3, threaded=True, nav=FakeNav(), imu_node="cam1")
    try:
        check("the camera connected", env.wait_state("cam1", "CAM_CONNECTED"))
        _wait_offsets(env, ("cam1",), {"cam1": TRUE}, tol=0.02)
        r = env.runmgr.start({"label": "r12g", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        node = env.node("cam1")
        _drain_spool(env, "cam1", w)
        # No command, no EXPOSURE edge, and no EXIF calibration for this node:
        # the card-review fallback, the one path that returns a HOST instant.
        env.timesync.exif_offset.pop("cam1", None)
        env.runmgr.reset_edge_cursors()
        asked = []
        real_imu = env.runmgr.imu_snapshot

        def recording_imu(epoch, off=None, node=None):
            asked.append((epoch, off))
            return real_imu(epoch, off=off, node=node)

        env.runmgr.imu_snapshot = recording_imu
        node.add_frame(epoch=time.time(), name="ILXCARD01.JPG", exif=False)
        wait_for(lambda: any(e["orig"] == "ILXCARD01.JPG"
                             for e in _index(env, root)), 20)
        check("the card-review frame was written", bool(asked),
              "imu_snapshot was not called")
        if not asked:
            return
        ep, off = asked[-1]
        check("the IMU window is converted into the NODE domain, not left "
              "asserting node == host",
              off is not None and abs(off - TRUE) < 0.05,
              "off=%s (the fleet offset is %.3f s; 0.0 centres the +/-100 ms "
              "window %.0f ms into the node's past)"
              % (off, TRUE, TRUE * 1000))
    finally:
        env.close()


# ===========================================================================
# R13-R26 (2026-10-15) - the run-group audit
# ===========================================================================
def _unit_worker(env, node_name="cam1", **kw):
    """A PullWorker on a real (unticked) monitor with `_handle` replaced by a
    recorder, so claims can be read without a download. The monitor's listing
    is a list the test edits."""
    mon = env.mon(node_name)
    listing = {"names": []}
    mon.is_connected = lambda: True
    mon.shots = lambda: [{"name": n, "size": 10} for n in listing["names"]]
    cam_dir = os.path.join(env.dir, "unit", node_name)
    w = runmod.PullWorker(mon, cam_dir, env.runmgr, **kw)
    w.handled = {}

    def _handle(name, size):
        w.handled[name] = w.cmd_epoch.get(name)
        return True
    w._handle = _handle
    return w, listing


def _run_catching(worker):
    """worker.run() on this thread; the exceptions that escape it, as text."""
    try:
        worker.run()
    except Exception as e:                                   # noqa: BLE001
        return ["%s: %s" % (type(e).__name__, e)]
    return []


def _forget_but(env, names, node="cam1"):
    """rigd's memory of a node's spool, reset to exactly `names`. A worker's
    own baseline records everything it lists, so each worker under test needs
    the memory put back to what an EARLIER run would have left."""
    with env.runmgr._lock:
        env.runmgr._known_frames[node] = {n: None for n in names}


# ===========================================================================
# R13 - an adopted worker's backlog frames own no fire command
# ===========================================================================
def _r13_adoption_backlog_claims_no_command(opts):
    sect("R13 frames exposed before adoption do not claim the first command")
    env = Env([("cam1", "127.0.0.160", 1)], poll=0.4, threaded=False,
              nav=FakeNav(), imu_node="cam9")
    try:
        backlog = ["ILX00001.JPG", "ILX00002.JPG", "ILX00003.JPG"]
        # rigd has watched this node's save dir before (an earlier run in this
        # process), which is what makes the unlisted names a "backlog".
        env.runmgr.remember_frames("cam1", ["ILXOLD00.JPG"])

        w, listing = _unit_worker(env, adopted=True)
        listing["names"] = ["ILXOLD00.JPG"] + backlog
        _forget_but(env, ["ILXOLD00.JPG"])
        check("the adopted worker baselines and finds the backlog",
              w._baseline() is True)
        # The camera fires: a command is queued in the window between
        # adoption and the worker's first poll.
        t_cmd = time.time() + 0.3
        w.note_command(t_cmd, path="gpio", fire_seq=77, edge_seq=0)
        w._poll_once()
        check("no backlog frame claimed the queued command",
              all(w.handled.get(n) is None for n in backlog)
              and all(n in w.handled for n in backlog),
              "claims=%s" % {n: (w.handled.get(n) or {}).get("fire_seq")
                             for n in backlog})
        with w._lock:
            queued = [r.get("fire_seq") for r in w._cmds]
        check("the command is still queued for the next survey frame",
              queued == [77], str(queued))
        listing["names"].append("ILX00004.JPG")      # the first survey frame
        w._poll_once()
        check("and the post-adoption survey frame claims it",
              (w.handled.get("ILX00004.JPG") or {}).get("fire_seq") == 77,
              str(w.handled.get("ILX00004.JPG")))

        # Control: a worker that existed from run start and baselined late
        # owns the commands of the frames shot during its listing outage.
        w2, listing2 = _unit_worker(env, adopted=False)
        listing2["names"] = ["ILXOLD00.JPG"] + backlog
        _forget_but(env, ["ILXOLD00.JPG"])
        check("a late baseline succeeds", w2._baseline(late=True) is True)
        for k in range(3):
            w2.note_command(time.time() + 0.1 * k, path="gpio",
                            fire_seq=10 + k, edge_seq=0)
        w2._poll_once()
        got = [(w2.handled.get(n) or {}).get("fire_seq") for n in backlog]
        check("a LATE worker's backlog frames still claim their own commands",
              got == [10, 11, 12], str(got))

        # Rollover: a backlog name that has been pulled and deleted comes back
        # as a new exposure and is an ordinary frame.
        w3, listing3 = _unit_worker(env, adopted=True)
        listing3["names"] = ["ILXOLD00.JPG"] + backlog
        _forget_but(env, ["ILXOLD00.JPG"])
        w3._baseline()

        def _handle_gone(name, size):
            w3.handled[name] = w3.cmd_epoch.get(name)
            getattr(w3, "_gone", set()).add(name)        # confirmed delete
            return True
        w3._handle = _handle_gone
        w3._poll_once()
        listing3["names"] = ["ILXOLD00.JPG"]  # deleted from the node...
        w3._poll_once()
        listing3["names"] = ["ILXOLD00.JPG"] + backlog     # ...and reissued
        w3.note_command(time.time() + 0.3, path="gpio", fire_seq=88,
                        edge_seq=0)
        w3.handled.clear()
        w3._poll_once()                       # the numbers have wrapped
        check("a recycled backlog name is no longer barred from a command",
              (w3.handled.get("ILX00001.JPG") or {}).get("fire_seq") == 88,
              str(w3.handled))
    finally:
        env.close()


# ===========================================================================
# R14 - the flight_log failure handlers must not raise
# ===========================================================================
def _r14_flight_log_failures_do_not_raise(opts):
    sect("R14 a flight_log that cannot be written degrades, it does not crash")
    env = Env([("cam1", "127.0.0.161", 1)], poll=0.4, threaded=False,
              nav=FakeNav(), imu_node="cam9")
    try:
        # (1) startup: flight_log.csv is a directory, so open() raises OSError.
        w, _l = _unit_worker(env)
        os.makedirs(os.path.join(w.cam_dir, "flight_log.csv"))
        w._baseline = lambda late=False: True
        polls = []
        w._poll_once = lambda: polls.append(1)
        errs = []
        seq0 = env.seq()
        th = threading.Thread(target=lambda: errs.extend(_run_catching(w)),
                              daemon=True)
        th.start()
        primed = w.primed.wait(5)
        w.stop()
        th.join(5)
        check("a flight_log that cannot be opened does not kill the worker",
              not errs and primed and bool(polls), "errors=%s primed=%s polls=%d"
              % (errs, primed, len(polls)))
        check("the loss is reported through the run's event log",
              any(e["kind"] == "pull" and "cannot be written" in e["msg"]
                  for e in env.evs(since=seq0)),
              str([e["msg"][:60] for e in env.evs(since=seq0)]))

        # (2) mid-run: the row write fails after the frame is on host disk.
        w2, _l2 = _unit_worker(env)

        class _Full:
            def writerow(self, row):
                raise OSError(28, "No space left on device")
        w2._flight = _Full()
        w2._flight_fh = io.StringIO()
        seq1 = env.seq()
        t = time.time()
        raised = None
        try:
            w2._write_flight("Cam1_x.jpg", "ILX00001.JPG", t, t, "exif", 0.0,
                             w2._timebase[1], 0.0)
        except Exception as e:                               # noqa: BLE001
            raised = "%s: %s" % (type(e).__name__, e)
        check("a failed row write returns instead of losing the frame",
              raised is None, str(raised))
        check("the flight_log is dropped and the loss reported once",
              w2._flight is None and w2._flight_fh is None
              and any(e["kind"] == "pull" and "stopped accepting rows"
                      in e["msg"] for e in env.evs(since=seq1)))

        # (3) the stop path of a worker whose flight_log degraded mid-run.
        w3, _l3 = _unit_worker(env)
        w3._baseline = lambda late=False: True
        w3._poll_once = lambda: None
        errs3 = []
        th3 = threading.Thread(target=lambda: errs3.extend(_run_catching(w3)),
                               daemon=True)
        th3.start()
        w3.primed.wait(5)
        try:
            w3._flight_fh.close()
        except Exception:                                     # noqa: BLE001
            pass
        w3._flight_fh = None
        w3._flight = None
        w3.stop()
        th3.join(5)
        check("stopping a worker whose flight_log already failed is clean",
              not errs3 and not th3.is_alive(), str(errs3))
    finally:
        env.close()


# ===========================================================================
# R15 - a standalone strobe's instant reaches the cameras' frames
# ===========================================================================
def _r15_standalone_strobe_reaches_the_frames(opts):
    sect("R15 a camera-less strobe node's pulse instant is recorded per frame")
    env = Env([("cam1", "127.0.0.162", 1), ("cam2", "127.0.0.163", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    real = runmod.http_json
    mode = {"m": "real"}
    host2 = "http://%s:8081/gpio/strobe" % "127.0.0.163"

    def _hj(url, *a, **kw):
        if url.startswith(host2):
            if mode["m"] == "late":
                return {"ok": True, "strobe_epoch": time.time(),
                        "strobe_late_ms": 9.0}
            if mode["m"] == "fail":
                return {"ok": False, "error": "injected: no pulse"}
            if mode["m"] == "fast":
                return {"ok": True, "strobe_epoch": 5555.0,
                        "strobe_late_ms": 0.2}
        return real(url, *a, **kw)

    def _held_shot(w):
        """One capture_once with the puller held off, so the command record
        can be read before a frame claims it. -> (report, newest record)."""
        env.runmgr.begin_calibration_fire("cam1", hold_s=30)
        try:
            rep = env.runmgr.capture_once()
            with w._lock:
                rec = w._cmds[-1] if w._cmds else None
            return rep, rec
        finally:
            env.runmgr.end_calibration_fire("cam1")

    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.wait_state("cam2", "CAM_CONNECTED")
        rs = env.runmgr.set_strobe({"enabled": True, "node": "cam2",
                                    "delta_ms": 10.0, "pulse_ms": 5})
        check("strobe configured on cam2", rs.get("ok") is True)
        # The strobe node's camera is gone but its Pi answers: the
        # camera-less XR256 host of docs/FIELD-RUN.md.
        # ...and ilxctl's reconnect reports success without the body being
        # claimed, so it stays out of the run for the whole test.
        env.node("cam2").set_fault("ilx", connect_lies=True)
        env.node("cam2").connected = False
        check("cam2 leaves CAM_CONNECTED but stays reachable",
              bool(wait_for(lambda: not env.mon("cam2").is_connected(), 15))
              and env.mon("cam2").state != rigcore.NodeMonitor.OFFLINE)
        r = env.runmgr.start({"label": "r15", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        runmod.http_json = _hj

        rep, rec = _held_shot(w)
        want = rep["target"] + 0.010
        got = (rec or {}).get("strobe_epoch")
        check("the shot's command record carries the strobe instant",
              got is not None and abs(got - want) < 0.05,
              "strobe_epoch=%s want %.4f" % (got, want))
        ok = wait_for(lambda: len(_index(env, root)) >= 1, 15)
        ent = (_index(env, root) or [{}])[-1]
        off = ent.get("clk_off") or 0.0
        check("the indexed frame carries it, for the pair verdict",
              ok and ent.get("strobe") is not None
              and abs(ent["strobe"] - (want - off)) < 0.05,
              json.dumps(ent)[:200])
        _drain_spool(env, "cam1", w)

        # A late flash is reported, as the in-fire path reports it.
        mode["m"] = "late"
        seq0 = env.seq()
        _held_shot(w)
        check("a standalone flash pulsed late is reported, naming the node",
              any(e["kind"] == "strobe_late" and e.get("node") == "cam2"
                  for e in env.evs(since=seq0)),
              str([e["kind"] for e in env.evs(since=seq0)][-6:]))
        _drain_spool(env, "cam1", w)

        # A refused flash records nothing and says so.
        mode["m"] = "fail"
        seq1 = env.seq()
        rep, rec = _held_shot(w)
        check("a failed standalone flash attaches no instant",
              rec is not None and rec.get("strobe_epoch") is None,
              str(rec))
        check("and strobe_fail is still raised",
              any(e["kind"] == "strobe_fail" for e in env.evs(since=seq1)))
        _drain_spool(env, "cam1", w)

        # Ordering: the flash answers BEFORE the camera's command is queued.
        mode["m"] = "fast"
        real_lead = env.runmgr._trigger_lead

        def _slow_lead(node):
            time.sleep(0.25)
            return real_lead(node)
        env.runmgr._trigger_lead = _slow_lead
        try:
            rep, rec = _held_shot(w)
        finally:
            env.runmgr._trigger_lead = real_lead
        check("the instant is attached when the flash replies first",
              rec is not None and rec.get("strobe_epoch") == 5555.0, str(rec))
    finally:
        runmod.http_json = real
        env.close()


# ===========================================================================
# R16 - `pulled` counts exposures, not files
# ===========================================================================
def _r16_pulled_counts_exposures(opts):
    sect("R16 RAW+JPEG is one exposure to stop()'s grace and the alarm")
    env = Env([("cam1", "127.0.0.164", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r16", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        node = env.node("cam1")
        t0 = time.time() - 2.0
        env.timesync.set_exif_offset("cam1", 0.0)
        # Two exposures, three files: release A delivers both halves, B a JPEG.
        node.add_frame(epoch=t0, name="ILX09101.ARW")
        node.add_frame(epoch=t0, name="ILX09101.JPG")
        node.add_frame(epoch=t0 + 0.5, name="ILX09102.JPG")

        def _landed():
            fs = os.listdir(os.path.join(root, "cam1"))
            return (len([f for f in fs if f.lower().endswith(".jpg")]) >= 2
                    and any(f.lower().endswith(".arw") for f in fs)
                    and len(_rows(root, "cam1")[1]) >= 2
                    and not w._pending)
        check("three files landed for two exposures", bool(wait_for(_landed, 15)))
        check("pulled counts exposures, not files",
              w.stats()["pulled"] == 2, "pulled=%s" % w.stats()["pulled"])

        # stop()'s grace keeps waiting for a shot that was fired and has not
        # been delivered: three exposures fired, two delivered.
        with env.runmgr._lock:
            env.runmgr.active["fired"]["cam1"] = 3
            fired = dict(env.runmgr.active["fired"])
        lag = [n for n, x in env.runmgr.workers.items()
               if x.stats().get("pulled", 0) < fired.get(n, 0)]
        check("the final-shot grace still sees the undelivered shot",
              lag == ["cam1"], str(lag))

        # The accounting alarm: 8 fired, 2 exposures landed, 1 calibration
        # frame kept out, nothing pending (inflight allowance 2) -> 3 never
        # arrived. Counting files said 2.
        with env.runmgr._lock:
            env.runmgr.active["fired"]["cam1"] = 8
        env.runmgr._warned_at.pop("frames_missing:cam1", None)
        seq0 = env.seq()
        env.runmgr._watch_frame_accounting()
        check("frames_missing fires for a camera that stopped delivering",
              any(e["kind"] == "frames_missing" and e.get("node") == "cam1"
                  for e in env.evs(since=seq0)),
              str([e["kind"] for e in env.evs(since=seq0)]))
        with env.runmgr._lock:
            env.runmgr.active["fired"]["cam1"] = 2
    finally:
        env.close()


# ===========================================================================
# R17 - the run's sync record covers the whole run
# ===========================================================================
def _r17_run_skew_record_is_not_windowed(opts):
    sect("R17 sync.shots / skew_ms_max / skew_ms_p95 cover every shot")
    env = Env([("cam1", "127.0.0.165", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        r = env.runmgr.start({"label": "r17", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        rm = env.runmgr

        def shot(i, skew_ms):
            t = 1000.0 + i
            rm._judge_shot(t, {"cam1": {"ok": True, "path": "gpio",
                                        "exposure_epoch": t},
                               "cam2": {"ok": True, "path": "gpio",
                                        "exposure_epoch": t + skew_ms / 1e3}},
                           roster=["cam1", "cam2"])
        def skew_of(i):
            return 14.0 if 100 <= i < 200 else 0.5      # the bad ones are OLD
        for i in range(800):
            shot(i, skew_of(i))
        st = rm.status()["sync"]
        check("a run of 1000 shots or fewer is unchanged",
              st["shots"] == 800 and st["skew_ms_max"] == 14.0, str(st))
        for i in range(800, 1500):
            shot(i, skew_of(i))
        st = rm.status()["sync"]
        check("sync.shots counts all 1500 shots", st["shots"] == 1500,
              "shots=%s" % st["shots"])
        check("skew_ms_max still holds the worst shot of the whole run",
              st["skew_ms_max"] == 14.0, "skew_ms_max=%s" % st["skew_ms_max"])
        rm._write_run_json()
        doc = json.load(open(os.path.join(root, "run.json")))["sync"]
        check("run.json agrees: shots, max, and a p95 over the whole run",
              doc["shots"] == 1500 and doc["skew_ms_max"] == 14.0
              and doc["skew_ms_p95"] == 14.0, json.dumps(doc)[:200])
    finally:
        env.close()


# ===========================================================================
# R18 - start() does its edge-cursor I/O outside the run lock
# ===========================================================================
def _r18_start_io_is_outside_the_run_lock(opts):
    sect("R18 reset_edge_cursors runs outside the run lock")
    env = Env([("cam1", "127.0.0.166", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        rm = env.runmgr
        locked, first = [], threading.Event()

        def slow_reset():
            locked.append(rm._lock.locked())
            first.set()
            time.sleep(0.6)             # an unplugged slot costs 3 s of this
        rm.reset_edge_cursors = slow_reset
        out = {}
        th = threading.Thread(
            target=lambda: out.update(rm.start({"label": "r18",
                                                "calibrate": False})),
            daemon=True)
        th.start()
        entered = first.wait(10)
        t0 = time.time()
        rm.known_frames("cam1")        # takes the run lock, as every worker does
        dt = time.time() - t0
        th.join(20)
        check("the run started", out.get("ok") is True, json.dumps(out)[:120])
        check("the edge-cursor reset did not hold the run lock",
              entered and not any(locked), "lock held at call: %s" % locked)
        check("a pull worker's bookkeeping is not stalled behind it",
              dt < 0.2, "known_frames blocked %.2f s" % dt)
        _wait_calibrated(env, 20)
        n = len(locked)
        r2 = rm.start({"label": "r18b", "calibrate": False})
        check("a double-submitted Start is refused",
              r2.get("ok") is False and "already active" in str(r2.get("error")),
              json.dumps(r2)[:120])
        check("and it does not reset the live run's edge cursors",
              len(locked) == n, "resets %d -> %d" % (n, len(locked)))
    finally:
        env.close()


# ===========================================================================
# R19 - a switched-off camera's stale fail_streak does not hold the grid
# ===========================================================================
def _r19_switched_off_camera_does_not_pause_the_grid(opts):
    sect("R19 a camera switched off after failing is not a dead node")
    env = Env([("cam1", "127.0.0.167", 1), ("cam2", "127.0.0.168", 2)],
              poll=0.4, threaded=False, nav=FakeNav(), imu_node="cam9")
    try:
        rm = env.runmgr
        rm.active = {"run_id": "r19", "fired": {}, "skews_ms": [],
                     "failed_fires": 0, "unpaired_shots": 0, "fail_streak": {},
                     "late_shots": 0, "orphans": {}}
        try:
            for _ in range(3):
                rm._judge_shot(time.time(),
                               {"cam1": {"ok": True, "path": "gpio"},
                                "cam2": {"ok": False, "error": "no frame"}},
                               roster=["cam1", "cam2"])
            check("three failed fires make cam2 the dead node",
                  rm._dead_fire_node() == "cam2", str(rm._dead_fire_node()))
            env.mon("cam2").camera_enabled = False
            check("switched off, it no longer holds the grid paused",
                  rm._dead_fire_node() is None, str(rm._dead_fire_node()))
            env.mon("cam2").camera_enabled = True
            check("switched back on, an unproven body pauses it again",
                  rm._dead_fire_node() == "cam2", str(rm._dead_fire_node()))
        finally:
            rm.active = None
    finally:
        env.close()


# ===========================================================================
# R20 - a body without SubSec does not publish a few-ms EXIF bar
# ===========================================================================
def _jpeg_without_subsec(epoch):
    from PIL import Image
    im = Image.new("RGB", (48, 36), (90, 40, 60))
    ex = Image.Exif()
    ex[0x8769] = {0x9003: time.strftime("%Y:%m:%d %H:%M:%S",
                                        time.localtime(epoch))}
    buf = io.BytesIO()
    im.save(buf, "JPEG", exif=ex.tobytes(), quality=60)
    return buf.getvalue()


def _no_subsec(node):
    """Make `node` deliver frames whose EXIF has DateTimeOriginal only."""
    def add(epoch=None, name=None, exif=True, size_bytes=None):
        with node._lock:
            node._shot_seq += 1
            ep = time.time() if epoch is None else epoch
            nm = name or "ILX%05d.JPG" % node._shot_seq
            data = _jpeg_without_subsec(ep + node.clock_skew_s)
            node.shots.append({"name": nm, "size": len(data), "bytes": data,
                               "epoch": ep})
        return nm
    node.add_frame = add


def _edge_frame(env, root, node, w, name, *, fire_seq, t, edge=True):
    """One scheduled shot -> (index entry, flight_log row)."""
    w.note_command(t, path="gpio", fire_seq=fire_seq, edge_seq=0)
    if edge:
        node.push_edge(epoch=t, fire_seq=fire_seq)
    node.add_frame(epoch=t, name=name)
    wait_for(lambda: any(e["orig"] == name for e in _index(env, root)), 20)
    got = {e["orig"]: e for e in _index(env, root)}.get(name)
    row = None if got is None else {x["filename"]: x for x in
                                    _rows(root, "cam1")[1]}.get(got["file"])
    return got, row


def _r20_whole_second_exif_has_a_whole_second_bar(opts):
    sect("R20 EXIF without SubSec is worth ~1.5 s and cannot beat a soft edge")
    if not runmod.PIL_AVAILABLE:
        note("R20 skipped: Pillow is not available on this host")
        return
    env = Env([("cam1", "127.0.0.169", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        _no_subsec(node)
        r = env.runmgr.start({"label": "r20", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        env.runmgr.active["config"]["auto_capture"] = True
        env.runmgr.active["config"]["interval_s"] = 0.5
        err = env.runmgr.exif_err("cam1")
        check("calibrating on a SubSec-less frame records a whole-second bar",
              err is not None and err >= 1.5, "exif_uncertainty=%s" % err)

        # The node cannot stamp its edges in the kernel: the edge is soft
        # (250 ms) and EXIF is the only other instant.
        node.hw_reject = "domain"
        t = time.time()
        got, row = _edge_frame(env, root, node, w, "ILXR2000.JPG",
                               fire_seq=8000, t=t)
        check("a whole-second EXIF stamp does not displace the soft edge",
              got is not None and got.get("src") == "gpio_edge_soft",
              "src=%r" % (got or {}).get("src"))
        terr = None if row is None else row.get("time_err_ms")
        check("and the row carries the soft edge's bar",
              terr not in (None, "") and 240.0 <= float(terr) < 1000.0,
              "time_err_ms=%r" % terr)

        # No edge at all: the exif row's bar is the whole-second one.
        got2, row2 = _edge_frame(env, root, node, w, "ILXR2001.JPG",
                                 fire_seq=8001, t=time.time() + 1.0,
                                 edge=False)
        terr2 = None if row2 is None else row2.get("time_err_ms")
        check("an exif row publishes at least 1500 ms",
              got2 is not None and got2.get("src") == "exif"
              and terr2 not in (None, "") and float(terr2) >= 1500.0,
              "src=%r time_err_ms=%r" % ((got2 or {}).get("src"), terr2))

        # A body that DOES write SubSec keeps its tight bar.
        env.runmgr.stop()
        del node.add_frame
        node.hw_reject = None
        env.runmgr.start({"label": "r20b", "calibrate": False})
        _wait_calibrated(env, 20)
        err2 = env.runmgr.exif_err("cam1")
        check("with SubSec the bar stays at a few ms",
              err2 is not None and err2 < 0.05, "exif_uncertainty=%s" % err2)
    finally:
        env.close()


# ===========================================================================
# R21 - frames taken between runs are known, so a late joiner leaves them
# ===========================================================================
def _r21_between_run_frames_are_not_backlog(opts):
    sect("R21 an idle spool census keeps bench frames out of a late join")
    env = Env([("cam1", "127.0.0.170", 1), ("cam2", "127.0.0.171", 2)],
              poll=0.4, threaded=True, nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        env.wait_state("cam2", "CAM_CONNECTED")
        rm = env.runmgr
        n1, n2 = env.node("cam1"), env.node("cam2")
        census = getattr(rm, "_census_once", lambda: None)

        # (a) an earlier run listed this node.
        n2.add_frame(epoch=time.time() - 900, name="ILX00100.JPG")
        census()
        check("the idle census lists a connected node",
              "ILX00100.JPG" in (rm.known_frames("cam2") or {}),
              str(rm.known_frames("cam2")))
        # (b) bench shots between runs, seen by the next census tick.
        n2.add_frame(epoch=time.time() - 30, name="ILX00101.JPG")
        n2.add_frame(epoch=time.time() - 29, name="ILX00102.JPG")
        census()
        bench = ["ILX00101.JPG", "ILX00102.JPG"]
        check("bench frames are known once the census has run",
              all(b in (rm.known_frames("cam2") or {}) for b in bench))

        # (c) a bench Capture is recorded within seconds, not at the next tick.
        rm.CENSUS_AFTER_SHOT_S = 0.3
        rm.capture_once()
        newest = lambda n: n.shots[-1]["name"] if n.shots else None  # noqa: E731
        check("a bench Capture is censused shortly after the shot",
              bool(wait_for(lambda: newest(n1) in (rm.known_frames("cam1")
                                                   or {}), 8))
              and bool(wait_for(lambda: newest(n2) in (rm.known_frames("cam2")
                                                       or {}), 8)),
              "cam1 %s known=%s" % (newest(n1), list(rm.known_frames("cam1")
                                                     or {})[-3:]))

        # (d) a listing that finishes after a run has begun is discarded.
        mon1 = env.mon("cam1")

        def _list_while_run_begins():
            rm.active = {"run_id": "racing"}
            return [{"name": "ILXRACE01.JPG", "size": 1}]
        mon1.shots = _list_while_run_begins
        try:
            census()
        finally:
            del mon1.shots
            rm.active = None
        check("a census listing taken as a run begins is discarded",
              "ILXRACE01.JPG" not in (rm.known_frames("cam1") or {}))

        # (e) cam2 is away at run start and joins late.
        prerun = [s["name"] for s in n2.shots
                  if s["name"] not in ("ILX00100.JPG",) + tuple(bench)]
        n2.down()
        check("cam2 reaches OFFLINE",
              bool(wait_for(lambda: env.mon("cam2").state
                            == rigcore.NodeMonitor.OFFLINE, 15)))
        r = rm.start({"label": "r21", "calibrate": False})
        root = r["root"]
        check("the run starts without cam2", r.get("nodes") == ["cam1"],
              json.dumps(r.get("nodes")))
        # While a run is open the census lists nothing.
        calls = []
        real_shots = mon1.shots
        mon1.shots = lambda: calls.append(1) or real_shots()
        try:
            census()
        finally:
            del mon1.shots
        check("no census listing is taken while a run is open", not calls)
        during = n2.add_frame(epoch=time.time(), name="ILX00103.JPG")
        n2.up()
        check("cam2 is adopted", bool(wait_for(lambda: "cam2" in rm.workers,
                                               20)))
        w2 = rm.workers.get("cam2")
        if w2 is None:
            return
        w2.primed.wait(timeout=10)
        known_old = ["ILX00100.JPG"] + bench + prerun
        check("the pre-run frames are baselined, not backlog",
              all(n in w2.seen for n in known_old),
              "seen=%s" % sorted(w2.seen))
        check("the worker reports exactly one frame to recover",
              any(e["kind"] == "pull" and "1 taken before adoption"
                  in e["msg"] for e in env.evs(node="cam2")),
              str([e["msg"] for e in env.evs(node="cam2")
                   if e["kind"] == "pull"]))
        got = wait_for(lambda: len(_rows(root, "cam2")[1]) >= 1, 25)
        origs = sorted(str(e.get("orig")) for e in _index(env, root)
                       if e.get("cam") == 2)
        check("only the frame shot during the run is pulled into it",
              bool(got) and origs == [during], "cam2 pulled %s" % origs)
    finally:
        env.close()


# ===========================================================================
# R22 - a reused camera filename is a new exposure
# ===========================================================================
def _r22_reissued_filename_is_pulled_again(opts):
    sect("R22 a name pulled and deleted, then reissued, is pulled again")
    env = Env([("cam1", "127.0.0.172", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    real_rig = rigcore.http_json
    real = runmod.http_json
    gate = {"fail_delete": False, "empty_until": 0.0}
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        node.add_frame(epoch=time.time() - 600, name="ILXOLD00.JPG")
        host = node.host

        def _hj(url, *a, **kw):
            # The stock fake acknowledges a delete and keeps the file; make it
            # behave like ilxctl so a deleted name can actually come back.
            if url.startswith("http://%s:8080/api/shots/delete" % host):
                body = a[0] if a else kw.get("body")
                if gate["fail_delete"]:
                    return {"ok": False, "error": "injected: unlink failed"}
                with node._lock:
                    node.shots[:] = [s for s in node.shots
                                     if s["name"] != body["name"]]
                return {"ok": True}
            return real(url, *a, **kw)

        def _listing(url, *a, **kw):
            if url.endswith("/api/shots") and time.time() < gate["empty_until"]:
                return []                 # opendir failed: reads as empty
            return real_rig(url, *a, **kw)
        runmod.http_json = _hj
        rigcore.http_json = _listing

        r = env.runmgr.start({"label": "r22", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        env.timesync.set_exif_offset("cam1", 0.0)

        node.add_frame(epoch=time.time(), name="ILX09999.JPG")
        check("the frame is pulled", bool(wait_for(lambda: w.pulled == 1, 15)),
              json.dumps(w.stats()))
        check("and deleted from the node",
              bool(wait_for(lambda: not any(s["name"] == "ILX09999.JPG"
                                            for s in node.shots), 5)))
        time.sleep(1.0)               # a poll lists the spool without the name
        # The numbering wraps ~9999 exposures later, long after the 30 s
        # window in which a same-stem file is taken for a RAW sibling.
        w._stem_dest.clear()
        node.add_frame(epoch=time.time(), name="ILX09999.JPG")   # wrapped
        check("the same name reissued is pulled as a new exposure",
              bool(wait_for(lambda: w.pulled == 2, 15)), json.dumps(w.stats()))
        rows = _rows(root, "cam1")[1]
        origs = sorted(str(e.get("orig")) for e in _index(env, root))
        check("two rows, two distinct files, one camera name",
              len(rows) == 2 and len({x["filename"] for x in rows}) == 2
              and origs == ["ILX09999.JPG", "ILX09999.JPG"],
              "rows=%d origs=%s" % (len(rows), origs))
        check("nothing was abandoned or left unpulled",
              w.failed == 0 and not os.path.exists(
                  os.path.join(root, "unpulled.jsonl")))

        # A delete that fails leaves the file on the node: listing it again
        # must not pull it twice.
        gate["fail_delete"] = True
        node.add_frame(epoch=time.time(), name="ILX09998.JPG")
        check("a frame whose delete fails is pulled once",
              bool(wait_for(lambda: w.pulled == 3 and w.dump_fails >= 1, 15)),
              json.dumps(w.stats()))
        time.sleep(1.6)
        check("and is not pulled again while it stays on the node",
              w.pulled == 3, "pulled=%s" % w.pulled)
        gate["fail_delete"] = False

        # A listing that fails open (reads as empty) must not make the
        # run-start baseline look new when it comes back.
        gate["empty_until"] = time.time() + 1.5
        time.sleep(3.0)
        check("an empty listing does not re-pull the baseline",
              w.pulled == 3 and "ILXOLD00.JPG" in w.seen
              and not any(str(e.get("orig")) == "ILXOLD00.JPG"
                          for e in _index(env, root)),
              json.dumps(w.stats()))
    finally:
        runmod.http_json = real
        rigcore.http_json = real_rig
        env.close()


# ===========================================================================
# R23 - a failed run-start calibration does not keep the last run's offset
# ===========================================================================
def _r23_failed_calibration_clears_the_old_offset(opts):
    sect("R23 an offset from the previous run is not carried into this one")
    env = Env([("cam1", "127.0.0.173", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        mon = env.mon("cam1")
        env.timesync.set_exif_offset("cam1", 3.0)       # last run's figure
        env.runmgr.exif_uncertainty["cam1"] = 0.003
        mon.shutter = lambda af=False: {"ok": False,
                                        "error": "injected: shutter refused"}
        try:
            env.runmgr.start({"label": "r23", "calibrate": False})
            _wait_calibrated(env, 20)
        finally:
            del mon.shutter
        t = time.time()
        check("no offset survives a failed calibration",
              env.timesync.correct_exif("cam1", t) is None
              and "cam1" not in env.timesync.exif_offset,
              str(env.timesync.exif_offset))
        check("and no error bar is left to describe it",
              env.runmgr.exif_err("cam1") is None,
              str(env.runmgr.exif_err("cam1")))
        env.runmgr.stop()

        # Positive control: a working calibration measures it again.
        env.runmgr.start({"label": "r23b", "calibrate": False})
        _wait_calibrated(env, 20)
        off = env.timesync.exif_offset.get("cam1")
        check("a successful calibration sets the offset and its bar",
              off is not None and abs(off) < 1.5
              and env.runmgr.exif_err("cam1") is not None,
              "offset=%s err=%s" % (off, env.runmgr.exif_err("cam1")))

        # A camera that rejoins mid-run keeps THIS run's offset.
        env.runmgr.active["config"].update({"auto_capture": True,
                                            "interval_s": 0.5})
        env.runmgr._adopt_calibrate(mon)
        check("a mid-run rejoin does not clear the run's offset",
              env.timesync.exif_offset.get("cam1") == off,
              "%s -> %s" % (off, env.timesync.exif_offset.get("cam1")))
    finally:
        env.close()


# ===========================================================================
# R24 - frames pending at stop leave a record
# ===========================================================================
def _r24_pending_at_stop_is_recorded(opts):
    sect("R24 a frame still pending when the run stops lands in unpulled.jsonl")
    env = Env([("cam1", "127.0.0.174", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    real_hb = runmod.http_bytes
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        # A run whose every frame was pulled leaves no sidecar.
        r = env.runmgr.start({"label": "r24a", "calibrate": False})
        root_a = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        node.add_frame(epoch=time.time(), name="ILXOK001.JPG")
        wait_for(lambda: w.pulled == 1, 15)
        env.runmgr.stop()
        check("a normal stop writes no unpulled.jsonl",
              not os.path.exists(os.path.join(root_a, "unpulled.jsonl")))

        # A run stopped while a download keeps failing.
        r = env.runmgr.start({"label": "r24b", "calibrate": False})
        root = r["root"]
        _wait_calibrated(env, 20)
        w = env.runmgr.workers["cam1"]
        w.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w)
        host = node.host

        def _hb(url, *a, **kw):
            if url.startswith("http://%s:8080/shot/" % host):
                return None, "injected: link down"
            return real_hb(url, *a, **kw)
        runmod.http_bytes = _hb
        t0 = time.time()
        w.note_command(t0, path="gpio", fire_seq=5100, edge_seq=0)
        node.add_frame(epoch=t0, name="ILXPEND1.JPG")
        check("the frame is claimed and failing its first pull",
              bool(wait_for(lambda: (w._pending.get("ILXPEND1.JPG") or {})
                            .get("attempts", 0) >= 1, 10)),
              str(w._pending))
        seq0 = env.seq()
        env.runmgr.stop()
        path = os.path.join(root, "unpulled.jsonl")
        recs = ([json.loads(l) for l in open(path) if l.strip()]
                if os.path.exists(path) else [])
        check("stop leaves a record of the frame it could not pull",
              len(recs) == 1 and recs[0].get("orig") == "ILXPEND1.JPG"
              and recs[0].get("cam") == "cam1", json.dumps(recs)[:200])
        check("the record says why and carries the claimed command",
              bool(recs) and "stopped" in recs[0].get("reason", "")
              and abs(recs[0].get("cmd_epoch", 0) - t0) < 1e-3,
              json.dumps(recs)[:200])
        check("the stop names the loss to the operator",
              any(e["kind"] == "pull_fail" and "ILXPEND1.JPG" in e["msg"]
                  for e in env.evs(since=seq0)))
        check("the frame is not counted as failed or pulled",
              w.failed == 0 and w.pulled == 0, json.dumps(w.stats()))
    finally:
        runmod.http_bytes = real_hb
        env.close()


# ===========================================================================
# R25 - a straggling worker's frame is not indexed into the next run
# ===========================================================================
def _r25_straggler_does_not_index_into_the_next_run(opts):
    sect("R25 a worker that outlives stop() cannot touch the next run")
    env = Env([("cam1", "127.0.0.175", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        rm = env.runmgr
        ra = rm.start({"label": "r25a", "calibrate": False})
        _wait_calibrated(env, 20)
        w_old = rm.workers["cam1"]
        w_old.primed.wait(timeout=8)
        rm.stop()
        rb = rm.start({"label": "r25b", "calibrate": False})
        root_b = rb["root"]
        check("the second run is a different run",
              ra["run_id"] != rb["run_id"], "%s / %s"
              % (ra["run_id"], rb["run_id"]))
        _wait_calibrated(env, 20)
        w_new = rm.workers["cam1"]
        w_new.primed.wait(timeout=8)
        _drain_spool(env, "cam1", w_new)

        # The old worker was still downloading when stop() gave up on it; its
        # download finishes now. Its flight_log is still open in real life.
        w_old._flight = None
        t = time.time()
        seq0 = env.seq()
        w_old._write_flight("Cam1_late.jpg", "ILXLATE1.JPG", t, t, "exif",
                            0.0, w_old._timebase[1], 0.0)
        w_old.note_command(time.time() - 60, path="gpio")      # long expired
        w_old._expire_commands()
        names = [str(e.get("orig")) for e in _index(env, root_b)]
        check("its frame is not in the next run's index",
              "ILXLATE1.JPG" not in names, str(names))
        with rm._lock:
            orph = dict(rm.active["orphans"])
        check("its orphan count is not charged to the next run",
              not orph.get("cam1"), str(orph))
        check("the late frame is reported, naming both",
              any(e["kind"] == "run" and "Cam1_late.jpg" in e["msg"]
                  and ra["run_id"] in e["msg"] for e in env.evs(since=seq0)),
              str([e["msg"][:80] for e in env.evs(since=seq0)][-3:]))

        # Control: the new run's own worker indexes normally.
        w_new._write_flight("Cam1_ok.jpg", "ILXOK001.JPG", t, t, "exif", 0.0,
                            w_new._timebase[1], 0.0)
        names = [str(e.get("orig")) for e in _index(env, root_b)]
        check("the current run's worker still indexes", "ILXOK001.JPG" in names,
              str(names))
    finally:
        env.close()


# ===========================================================================
# R26 - a string is not consent
# ===========================================================================
def _r26_strings_do_not_arm_anything(opts):
    sect("R26 \"false\" does not start auto-capture or arm the strobe")
    env = Env([("cam1", "127.0.0.176", 1)], poll=0.4, threaded=False,
              nav=FakeNav(), imu_node="cam9")
    try:
        rm = env.runmgr
        bad = []
        for v in ("false", "0", "no", "off", "true", [], {}):
            cfg, err, _w = rm._validate_config({"auto_capture": v,
                                                "interval_s": 2})
            if cfg is not None or "auto_capture" not in str(err):
                bad.append(v)
        check("a non-boolean auto_capture is refused, naming the field",
              not bad, "accepted: %r" % bad)
        good = []
        for v, want in ((True, True), (False, False), (1, True), (0, False)):
            cfg, err, _w = rm._validate_config({"auto_capture": v,
                                                "interval_s": 2})
            good.append(cfg is not None and cfg["auto_capture"] is want)
        check("real booleans (and 0/1) still validate, as real booleans",
              all(good), str(good))
        try:
            cfg, err, _w = rm._validate_config({"frames": float("inf")})
            ok = cfg is None and "frames" in str(err)
        except OverflowError as e:
            ok = False
            err = "OverflowError: %s" % e
        check("frames=Infinity is refused, not an OverflowError", ok, str(err))

        before = rm.get_strobe()
        rs = rm.set_strobe({"enabled": "false"})
        check("a string 'enabled' is refused and the strobe stays as it was",
              rs.get("ok") is False and rm.get_strobe() == before,
              json.dumps(rs)[:120])
        try:
            rp = rm.set_strobe({"pulse_ms": float("inf")})
        except OverflowError as e:
            rp = {"ok": "OverflowError: %s" % e}
        check("pulse_ms=Infinity is refused, not an OverflowError",
              rp.get("ok") is False, json.dumps(rp)[:120])
        ok_on = rm.set_strobe({"enabled": True})
        ok_off = rm.set_strobe({"enabled": False})
        check("true/false still arm and disarm it",
              ok_on.get("ok") and ok_on["strobe"]["enabled"] is True
              and ok_off.get("ok") and ok_off["strobe"]["enabled"] is False)
    finally:
        env.close()


# ---------------------------------------------------------------------------
def suite(opts):
    _r1_offline_member_pauses(opts)
    _r1_resume_needs_a_real_fire(opts)
    _r2a_fire_schedule_in_node_time(opts)
    _r2b_capture_instant_converted(opts)
    _r3_dropped_frame_no_shift(opts)
    _r4_raw_jpeg_one_release(opts)
    _r5_calibration_arms_only_its_own_run(opts)
    _r7_validation(opts)
    _timebase_latched_per_worker(opts)
    _r8_index_jsonl(opts)
    _r10_strobe_blackhole(opts)
    _r10_strobe_offline_skipped(opts)
    _r11a_pair_skew_free_of_clock_estimates(opts)
    _r11b_imu_window_round_trip(opts)
    _r11c_baseline_listing_is_never_guessed(opts)
    _r12a_late_raw_sibling_takes_no_command(opts)
    _r12b_delivery_gap_does_not_latch_the_cross_check(opts)
    _r12b2_untrusted_claim_is_never_gpio_edge(opts)
    _r12c_pair_offset_is_shot_scoped(opts)
    _r12d_abandoned_probe_frame_is_quarantined(opts)
    _r12d2_probe_hold_outlives_its_own_listing(opts)
    _r12e_worker_retries_its_baseline(opts)
    _r12f_run_json_clock_record(opts)
    _r12g_card_review_imu_window(opts)
    _r13_adoption_backlog_claims_no_command(opts)
    _r14_flight_log_failures_do_not_raise(opts)
    _r15_standalone_strobe_reaches_the_frames(opts)
    _r16_pulled_counts_exposures(opts)
    _r17_run_skew_record_is_not_windowed(opts)
    _r18_start_io_is_outside_the_run_lock(opts)
    _r19_switched_off_camera_does_not_pause_the_grid(opts)
    _r20_whole_second_exif_has_a_whole_second_bar(opts)
    _r21_between_run_frames_are_not_backlog(opts)
    _r22_reissued_filename_is_pulled_again(opts)
    _r23_failed_calibration_clears_the_old_offset(opts)
    _r24_pending_at_stop_is_recorded(opts)
    _r25_straggler_does_not_index_into_the_next_run(opts)
    _r26_strings_do_not_arm_anything(opts)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    _t0 = time.time()
    suite(opts)
    print("\naudit_run: %d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - _t0))
    sys.exit(1 if soaktest.FAIL else 0)

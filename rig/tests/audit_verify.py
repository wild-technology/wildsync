#!/usr/bin/env python3
"""Audit regression suite — VERIFY lane (cross-lane contracts).

The six audit-fix lanes each owned one file. This suite pins the seams
BETWEEN them: the places where one lane changed a wire contract and the lane
that consumes it had to change to match. A lane suite cannot catch these,
because each half passes its own tests in isolation.

V1 (2026-08-24) THE epoch_hw CONTRACT, host half.
    The piagent lane fixed the node half of the epoch_hw blocker: a kernel
    edge stamp is no longer thrown away during the pipe-read excursions it
    exists to correct (the band was 0.25 s, cutting INSIDE a distribution
    whose documented excursions reach the hundreds of ms, and discarding the
    GOOD value while keeping the late one). It then published, per edge:

        hw_meta   (envelope) this node publishes the hw fields at all
        epoch_hw  the kernel instant, or null when it could not produce one
        hw_err_ms the node's own bound on epoch_hw (sub-ms)
        hw_lag_ms the measured pipe-read latency
        hw_reject why the stamp was refused: no_stamp | stamp_ahead | domain

    and stated in its handoff that the HOST must act on it, which no lane
    then did. Without the host half the blocker is only half dead: an edge
    the node openly refused to stamp was still written
    capture_source=gpio_edge with the fleet clock error as its whole error
    bar - a row asserting sub-millisecond hardware timing on a stamp that
    carries an unmeasured pipe-read latency. `hw_reject` being set and
    `hw_meta` being absent were also indistinguishable to the host (both
    read as "no epoch_hw"), and they need OPPOSITE treatment.

    These checks fail on the code as the six lanes left it and pass after.

Hermetic: in-process fakes on loopback only, temp run roots. ~35 s.

Run standalone:  python3 rig/tests/audit_verify.py
"""

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

import soaktest                                          # noqa: E402
from soaktest import check, sect, wait_for, Env, FakeNav  # noqa: E402
from audit_run import (_wait_calibrated, _drain_spool,    # noqa: E402
                       _index, _rows)


def _one_frame(env, root, node_name, w, name, *, fire_seq, exif=True,
               t=None, timeout=20.0):
    """Fire one scheduled shot and return (index entry, flight_log row)."""
    t = time.time() if t is None else t
    w.note_command(t, path="gpio", fire_seq=fire_seq, edge_seq=0)
    env.node(node_name).push_edge(epoch=t, fire_seq=fire_seq)
    env.node(node_name).add_frame(epoch=t, name=name, exif=exif)
    wait_for(lambda: any(e["orig"] == name for e in _index(env, root)),
             timeout)
    got = {e["orig"]: e for e in _index(env, root)}.get(name)
    row = None
    if got is not None:
        row = {x["filename"]: x
               for x in _rows(root, node_name)[1]}.get(got["file"])
    return got, row


def _env(label, host, cam=1):
    env = Env([("cam1", host, cam)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    env.wait_state("cam1", "CAM_CONNECTED")
    r = env.runmgr.start({"label": label, "calibrate": False})
    _wait_calibrated(env, 20)
    w = env.runmgr.workers["cam1"]
    w.primed.wait(timeout=8)
    _drain_spool(env, "cam1", w)
    env.runmgr.active["config"]["auto_capture"] = True
    env.runmgr.active["config"]["interval_s"] = 0.5
    return env, r["root"], w


# ===========================================================================
# V1a - a node that CAN stamp is unchanged, except that the row now carries
# the node's own bound on the stamp instead of the clock error alone.
# ===========================================================================
def _v1a_hardware_stamp_is_used_and_bounded(opts):
    sect("V1a a hardware-stamped edge is used, and carries the node's own bar")
    env, root, w = _env("v1a", "127.0.0.150")
    try:
        node = env.node("cam1")
        # The fake now serves the real contract: epoch_hw is the true instant
        # and `epoch` is that instant PLUS the pipe-read latency, the way
        # gpiomon and Python see it. A host reading `epoch` is late by the lag.
        node.hw_lag_ms = 40.0
        t = time.time()
        got, row = _one_frame(env, root, "cam1", w, "ILXV1A00.JPG",
                              fire_seq=7100, t=t)
        check("the frame was indexed", got is not None)
        if got is None:
            return
        check("it is capture_source=gpio_edge",
              got.get("src") == "gpio_edge", got.get("src"))
        # 40 ms of read latency, and the host must not have taken it.
        off_ms = (got["epoch"] - t) * 1000.0
        check("the host used the KERNEL stamp, not the late pipe-read stamp",
              abs(off_ms) < 20.0,
              "indexed epoch is %+.1f ms from the true instant (the fake's "
              "pipe-read lag is 40.0 ms)" % off_ms)
        terr = None if row is None else row.get("time_err_ms")
        check("the row carries a real error bar",
              terr not in (None, "") and float(terr) >= 0.0, "%r" % terr)
        check("and that bar is still tight - a good stamp costs nothing",
              terr not in (None, "") and float(terr) < 50.0, "%r ms" % terr)
    finally:
        env.close()


# ===========================================================================
# V1b - the blocker's residual half. The node says outright that it could not
# stamp this edge; the host must not dress the leftover software stamp as a
# hardware capture instant.
# ===========================================================================
def _v1b_refused_stamp_is_not_gpio_edge(opts):
    sect("V1b an edge the node REFUSED to stamp is not written as gpio_edge")
    env, root, w = _env("v1b", "127.0.0.151")
    try:
        node = env.node("cam1")
        # piagent could not produce a kernel stamp for any edge - e.g. a
        # gpiomon build that prints no timestamp. It says so per edge.
        node.hw_reject = "no_stamp"
        node.hw_lag_ms = 300.0
        # This body's clock has NOT been calibrated against an edge, so there
        # is no EXIF instant at all to fall through to - the deployed state
        # for any body whose calibration frame was lost, and the state every
        # frame is in before the first calibration lands. The soft edge is the
        # best instant available and must be published AS a soft one.
        env.timesync.exif_offset.pop("cam1", None)
        env.runmgr.exif_uncertainty.pop("cam1", None)
        got, row = _one_frame(env, root, "cam1", w, "ILXV1B00.JPG",
                              fire_seq=7200)
        check("the frame was indexed", got is not None)
        if got is None:
            return
        check("it is NOT capture_source=gpio_edge",
              got.get("src") != "gpio_edge",
              "src=%r - a refused stamp was published as a hardware instant"
              % got.get("src"))
        check("it is published as gpio_edge_soft",
              got.get("src") == "gpio_edge_soft", got.get("src"))
        terr = None if row is None else row.get("time_err_ms")
        check("its error bar is the documented read excursion, not the clock",
              terr not in (None, "") and float(terr) >= 250.0,
              "time_err_ms=%r (must be >= 250 ms: the pipe-read stamp's "
              "latency is unmeasured into the hundreds of ms)" % terr)
        check("no exposure window is claimed for it",
              got.get("rise") is None,
              "rise=%r - a ~13 ms window cannot be measured with a stamp "
              "whose error is unmeasured" % got.get("rise"))
    finally:
        env.close()


# ===========================================================================
# V1c - the two states that used to be indistinguishable. An OLD piagent
# (no hw_meta) publishes no epoch_hw either, and there `epoch` is all there
# has ever been - the previous behaviour is correct and must not regress.
# ===========================================================================
def _v1c_old_node_is_not_treated_as_a_refusal(opts):
    sect("V1c a node with no hw fields at all keeps the old behaviour")
    env, root, w = _env("v1c", "127.0.0.152")
    try:
        node = env.node("cam1")
        # Serve the pre-contract envelope: no hw_meta, no per-edge hw fields,
        # no epoch_hw - a node still running an older piagent.
        node.hw_strip = True
        got, row = _one_frame(env, root, "cam1", w, "ILXV1C00.JPG",
                              fire_seq=7300)
        check("the frame was indexed", got is not None)
        if got is None:
            return
        check("it is still capture_source=gpio_edge",
              got.get("src") == "gpio_edge",
              "src=%r - an older piagent's edge is not a refusal and must "
              "not be downgraded" % got.get("src"))
        terr = None if row is None else row.get("time_err_ms")
        check("and its bar is the clock error, as before",
              terr not in (None, "") and float(terr) < 250.0, "%r ms" % terr)
    finally:
        env.close()


# ===========================================================================
# V1d - a measured EXIF bar that is genuinely tighter wins; an UNMEASURED one
# does not. exif_err() is None until the body's clock is calibrated and the
# EXIF branch reads that None as 0.0, so an unguarded comparison would let a
# bar nobody has established beat a real edge every time.
# ===========================================================================
def _v1d_exif_wins_only_when_it_is_measurably_tighter(opts):
    sect("V1d EXIF is preferred over a soft edge only when measurably tighter")
    env, root, w = _env("v1d", "127.0.0.153")
    try:
        node = env.node("cam1")
        node.hw_reject = "domain"
        # A calibrated body with SubSec: EXIF is worth ~10 ms, far tighter
        # than the 250 ms a soft edge can promise, so the row should be exif.
        env.timesync.set_exif_offset("cam1", 0.0)
        env.runmgr.exif_uncertainty["cam1"] = 0.01
        got, _ = _one_frame(env, root, "cam1", w, "ILXV1D00.JPG",
                            fire_seq=7400)
        check("a tightly calibrated body falls through to EXIF",
              got is not None and got.get("src") == "exif",
              "src=%r" % (got or {}).get("src"))

        # Same node, same refusal, but the EXIF bar is now the coarse
        # whole-second one. 1.5 s is WORSE than the soft edge; preferring it
        # by rule would make the row worse, so the edge must win.
        env.runmgr.exif_uncertainty["cam1"] = 1.5
        got2, _ = _one_frame(env, root, "cam1", w, "ILXV1D01.JPG",
                             fire_seq=7401)
        check("a coarse EXIF bar does NOT displace the soft edge",
              got2 is not None and got2.get("src") == "gpio_edge_soft",
              "src=%r" % (got2 or {}).get("src"))

        # And with NO measured bar at all, an unknown error is not a small
        # one: the soft edge stands.
        env.runmgr.exif_uncertainty.pop("cam1", None)
        got3, _ = _one_frame(env, root, "cam1", w, "ILXV1D02.JPG",
                             fire_seq=7402)
        check("an UNMEASURED EXIF bar does not displace it either",
              got3 is not None and got3.get("src") == "gpio_edge_soft",
              "src=%r" % (got3 or {}).get("src"))
    finally:
        env.close()


# ===========================================================================
# V1e - the calibration constants. A soft edge folded into the trigger-latency
# median or the EXIF clock offset is not a one-frame error: both are per-node
# constants applied to the whole run, so an unmeasured, uncorrelated read
# latency there shifts one camera against the other for the entire transect.
# ===========================================================================
def _v1e_soft_edges_never_enter_a_calibration(opts):
    sect("V1e a soft edge is not folded into a per-node calibration constant")
    env = Env([("cam1", "127.0.0.154", 1)], poll=0.4, threaded=True,
              nav=FakeNav(), imu_node="cam9")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        node.hw_reject = "no_stamp"
        node.hw_lag_ms = 400.0
        mgr = env.runmgr
        m = next(x for x in mgr.monitors if x.name_ == "cam1")
        # _true_exposure is the EXIF clock calibration's measurement of when
        # the calibration frame really exposed. With the node unable to stamp,
        # it must take the command-epoch fallback - which publishes its own
        # error bar - rather than a 400 ms-late pipe-read stamp.
        cur = 0
        t_cmd = time.time()
        node.push_edge(epoch=t_cmd + 0.02, fire_seq=None)
        time.sleep(0.2)
        true_ep, err = mgr._true_exposure(m, cur, t_cmd, t_cmd + 0.5)
        drift_ms = abs(true_ep - (t_cmd + 0.02)) * 1000.0
        check("the EXIF calibration did not adopt the unstamped edge",
              err > 0.002,
              "uncertainty %.4f s: the edge branch reports 0.002 + clock "
              "err, the honest fallback reports the dispatch bar" % err)
        check("and it did not silently inherit the 400 ms read latency",
              drift_ms < 400.0,
              "the measured 'true exposure' is %.0f ms from the real one"
              % drift_ms)
    finally:
        env.close()


# ---------------------------------------------------------------------------
def suite(opts):
    _v1a_hardware_stamp_is_used_and_bounded(opts)
    _v1b_refused_stamp_is_not_gpio_edge(opts)
    _v1c_old_node_is_not_treated_as_a_refusal(opts)
    _v1d_exif_wins_only_when_it_is_measurably_tighter(opts)
    _v1e_soft_edges_never_enter_a_calibration(opts)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    _t0 = time.time()
    suite(opts)
    print("\naudit_verify: %d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - _t0))
    sys.exit(1 if soaktest.FAIL else 0)

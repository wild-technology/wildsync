#!/usr/bin/env python3
"""audit_piagent — regression tests for the 2026-08 piagent/imu_yb audit fixes.

Runs entirely on the host: piagent imports cleanly with gpiod/gpioset absent
(GPIO degrades to available=False, the IMU acquire loop probes and finds
nothing), so every fix is exercised against stubbed drivers and synthetic
0x7E/0x23 byte streams — no hardware, no network beyond 127.0.0.1.

Covers, and FAILS on the pre-fix code for:
  P1  FOCUS hold lease + watchdog (auto-release, keepalive, focus_held_s)
  P2  rise edges no longer inherit the open fire_seq forever
  P3  /health imu: attitude_hz vs frame_hz, imu_rate_low, no probe fallback
      once measured; attitude stall drops/re-acquires the reader
  P4  FIRE_MAX_FUTURE_S=5.5 and node_epoch on every fire answer
  P5  epoch_hw plausibility band + /health stamps time.epoch first
  P6  imu_yb checksum gate, euler/baro range checks, cadence-bounded dating
  P7  contract: at_epoch 0.3–5.5 s ahead is always accepted
  P8  a late gpiomon pipe read KEEPS the kernel stamp (the 0.25 s band cut
      inside the measured read-latency distribution); hw_lag_ms/hw_err_ms/
      hw_reject published per edge so an hw-less edge cannot be written as a
      hardware capture instant with a clock-error-only bar
  P9  the FOCUS lease covers a node-side interval whose period outlives it,
      and dies with the hold on the gpioset release path too
  P10 second IMU slot (imu2 / rig/imu_olive.py): absent-by-default is
      byte-compatible with pre-imu2 payloads, /imu2/* endpoints, imu2 nested
      in /health's imu section, per-slot rate floors, the acquire loop brings
      a probed olive online end-to-end, and undecodable bytes are counted
      and surfaced (raw tail), never raised or published

Usage:  python3 rig/tests/audit_piagent.py        (or via soaktest's registry)
"""

import json
import math
import os
import struct
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer
from urllib import request as _rq

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

# soaktest installs the loopback netguard at import; reusing its helpers keeps
# this suite's reporting identical to the main gate's.
from soaktest import check, sect, note, wait_for      # noqa: E402

# piagent resolves LOG_PATH and CAM_SAVE_DIR from HOME at import time; point
# them into a throwaway dir so this suite never writes to the operator's ~/rig.
_TMP = tempfile.mkdtemp(prefix="wildsync-audit-piagent-")
_OLD_HOME = os.environ.get("HOME")
os.environ["HOME"] = _TMP
try:
    import piagent                                    # noqa: E402
    import imu_yb                                     # noqa: E402
finally:
    if _OLD_HOME is None:
        os.environ.pop("HOME", None)
    else:
        os.environ["HOME"] = _OLD_HOME


# ---------------------------------------------------------------------------
# Synthetic device frames (imu_yb docstring: 0x7E 0x23 <type> <sub> <payload>
# <checksum>; checksum = 8-bit sum of everything after SOF).
# ---------------------------------------------------------------------------
def _frame(ftype, sub, payload):
    body = bytes([imu_yb.SOF, imu_yb.ADDR, ftype, sub]) + payload
    return body + bytes([sum(body[1:]) & 0xFF])


def euler_frame(roll_rad, pitch_rad, yaw_rad):
    return _frame(0x11, 0x26, struct.pack("<3f", roll_rad, pitch_rad, yaw_rad))


def baro_frame(temp_c, press_pa):
    return _frame(0x15, 0x32, struct.pack("<4f", 0.0, temp_c, press_pa,
                                          press_pa))


class StubDriver:
    """Stands in for _LineDriver so Gpio logic runs with no gpiod on the Mac."""
    ok = True

    def __init__(self):
        self.sets = []

    def set(self, bcm, value):
        self.sets.append((bcm, value))
        return True

    def pulse(self, bcm, hold_s):
        self.sets.append((bcm, "pulse"))
        return time.time()

    def shot(self, focus_bcm, trigger_bcm, lead_s, pulse_s, focus_after=None):
        self.sets.append((trigger_bcm, "shot"))
        if focus_after:
            focus_after()
        return time.time()


def _mk_gpio():
    g = piagent.Gpio()
    g.driver = StubDriver()
    return g


def _hwd(fn, raw, epoch):
    """_edge_hw's verdict as a dict, whatever shape the code under test uses.

    Pre-fix _edge_hw answered a bare float-or-None, which is precisely the
    thing being fixed (a dropped stamp was indistinguishable from a node that
    never had one), so the P8 checks have to be able to read both shapes."""
    r = fn(raw, epoch)
    if isinstance(r, dict):
        return r
    return {"hw": r, "lag_ms": None, "err_ms": None, "reject": None}


class FakeReader:
    """imu_yb.ImuReader stand-in with scriptable rates/staleness."""

    def __init__(self, rate=0.0, frame=99.7, measured=True,
                 att_age=None, frozen=None, rejects=0):
        self._rate, self._frame = rate, frame
        self._measured = measured
        self._att_age = att_age          # None = attitude never ringed
        self._frozen, self._rejects = frozen, rejects

    def latest(self):
        # Perpetually fresh, like trailing inertial frames keep it on the wire.
        return {"epoch": time.time(), "roll": 1.0}

    def rate_hz(self):
        return self._rate

    def frame_rate_hz(self):
        return self._frame

    def rate_measured(self):
        return self._measured

    def orientation_frozen_s(self):
        return self._frozen

    def rejected_frames(self):
        return self._rejects

    def last_attitude_epoch(self):
        return None if self._att_age is None else time.time() - self._att_age

    def checksum_state(self):
        return {"algo": "sum_after_sof", "dormant": False,
                "rejects": self._rejects, "learning": False}

    def stop(self):
        pass

    def close(self):
        pass


def _post(url, body):
    req = _rq.Request(url, data=json.dumps(body).encode(),
                      headers={"Content-Type": "application/json"})
    with _rq.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


def _get(url):
    with _rq.urlopen(url, timeout=5) as r:
        return json.loads(r.read().decode())


# ---------------------------------------------------------------------------
# The suite
# ---------------------------------------------------------------------------
def suite(opts):
    cleanup = []

    # ---- P1: FOCUS hold lease + watchdog ---------------------------------
    sect("audit piagent: FOCUS hold watchdog (P1)")
    g1 = _mk_gpio()
    cleanup.append(g1)
    old_gpio = piagent.GPIO
    piagent.GPIO = g1
    srv = ThreadingHTTPServer(("127.0.0.1", 0), piagent.Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    try:
        r = _post(base + "/gpio/focus", {"hold": True, "ttl_s": 1})
        check("hold with ttl_s accepted and reported held",
              r.get("ok") and r.get("focus_held") is True)
        check("/gpio/focus answers focus_held_s",
              isinstance(r.get("focus_held_s"), (int, float)))
        st = g1.state()
        check("/gpio/state exposes focus_held_s while held",
              isinstance(st.get("focus_held_s"), (int, float))
              and st["focus_held_s"] >= 0)
        # Keepalive: a renewal must extend the lease past the original expiry.
        time.sleep(0.6)
        _post(base + "/gpio/focus", {"hold": True, "ttl_s": 1})
        time.sleep(0.7)                      # 1.3 s after the FIRST grant
        check("renewed hold survives past the original ttl", g1.focus_held())
        # No further renewals: the watchdog must release the line by itself.
        released = wait_for(lambda: not g1.focus_held(), timeout=5.0)
        check("held FOCUS auto-releases once the lease lapses", released,
              "a dead host must not leave the body half-pressed")
        if released:
            check("watchdog released the line itself (driver saw IDLE)",
                  (piagent.BCM_FOCUS, piagent._LineDriver.IDLE)
                  in g1.driver.sets)
        check("focus_held_s reads None after release",
              g1.state().get("focus_held_s") is None)
        try:
            with open(piagent.LOG_PATH) as fh:
                logged = "focus_hold_expired" in fh.read()
        except OSError:
            logged = False
        check("expiry is logged as focus_hold_expired", logged)
        # An explicit release then a fresh hold must not inherit a stale lease.
        _post(base + "/gpio/focus", {"hold": True, "ttl_s": 60})
        check("fresh hold with long ttl stays held", g1.focus_held())
        _post(base + "/gpio/focus", {"hold": False})
        check("explicit release still works", not g1.focus_held())
    finally:
        srv.shutdown()
        srv.server_close()
        piagent.GPIO = old_gpio

    # The lease must not cut a legitimate hold short. run.py's trigger
    # calibration holds FOCUS for the whole sample loop and sends no keepalive
    # (and /api/calibrate does not clamp `samples`), so a fire that RELIES on
    # the hold — focus_lead_ms 0 — has to renew it; a survey fire, which
    # brings its own per-shot lead, must not, or a stale hold would ride the
    # whole transect exactly as it did before the watchdog.
    g1b = _mk_gpio()
    cleanup.append(g1b)
    g1b.available = True
    try:
        g1b.focus(True, ttl_s=1.0)
    except TypeError:                # pre-fix focus() has no lease at all
        g1b.focus(True)
    end = time.time() + 2.2                  # > 2 lease periods
    while time.time() < end and g1b.focus_held():
        g1b.fire(0, 5)                       # calibration-style fire
        time.sleep(0.2)
    check("a fire that relies on the held FOCUS renews the lease",
          g1b.focus_held(),
          "the watchdog would otherwise refuse every later calibration "
          "sample with 'FOCUS not held'")
    end = time.time() + 4.0
    while time.time() < end and g1b.focus_held():
        g1b.fire(0, 5, focus_lead_ms=120)    # survey-style fire
        time.sleep(0.2)
    check("a survey fire (own FOCUS lead) does NOT renew a stale hold",
          not g1b.focus_held(),
          "the per-shot FOCUS restore is what kept a stale hold alive all run")

    # ---- P2: rise edges and the open fire_seq window ---------------------
    sect("audit piagent: rise edge fire_seq window (P2)")
    g2 = _mk_gpio()
    cleanup.append(g2)
    t = time.time()
    g2._pending_fire = (41, t + 1.0)
    g2._record_edge(t, "fall", None, None)
    g2._record_edge(t + 0.024, "rise", None, None)
    g2._record_edge(t + 0.523, "rise", None, None)   # spurious late rise
    evs = g2.exposure_events(0)["events"]
    check("fall claims the pending fire", evs[0]["fire_seq"] == 41)
    check("the first rise closes the exposure with the same id",
          evs[1]["fire_seq"] == 41)
    check("a later spurious rise is NOT tagged with the stale id",
          evs[2]["fire_seq"] is None,
          "got %r - the host's identity match would adopt it and discard the "
          "genuine window" % (evs[2]["fire_seq"],))
    # Bounded in time as well: an exposure "open" for 11 s has lost its rise.
    g2._pending_fire = (42, time.time() + 1.0)
    g2._record_edge(time.time(), "fall", None, None)
    g2._edge_fire_at = time.time() - 11.0            # simulate a lost rise
    g2._record_edge(time.time(), "rise", None, None)
    evs = g2.exposure_events(0)["events"]
    check("a rise beyond the open-window bound is untagged",
          evs[-1]["fire_seq"] is None)

    # ---- P3: IMU health reporting ----------------------------------------
    sect("audit piagent: IMU attitude cadence in /health (P3)")
    imu = piagent.Imu()
    imu.reader = FakeReader(rate=0.0, frame=99.7, measured=True, att_age=10.0)
    imu.info = {"present": True, "sample_rate_hz": 49.8}
    h = imu.health()
    check("measured 0 Hz is reported as 0, not the probe figure",
          h.get("rate_hz") == 0.0,
          "got %r - a stalled attitude must not read as healthy" %
          (h.get("rate_hz"),))
    check("attitude_hz and frame_hz are reported separately",
          h.get("attitude_hz") == 0.0 and h.get("frame_hz") == 99.7)
    check("imu_rate_low raised on a stalled attitude",
          h.get("imu_rate_low") is True)
    imu.reader = FakeReader(rate=49.8, frame=174.0, measured=True,
                            att_age=0.02)
    h = imu.health()
    # 49.8 Hz IS this device's healthy attitude cadence (one quat + one euler
    # per ~25 Hz cycle; probe() measures exactly this). The 60 Hz threshold is
    # the FRAME-rate spec, and comparing attitude against it left imu_rate_low
    # true for the whole session on a good unit.
    check("a healthy 49.8 Hz attitude stream is NOT flagged low",
          h.get("imu_rate_low") is False,
          "the device cannot exceed ~50 Hz attitude; an alarm that is always "
          "on is an alarm nobody reads")
    check("/health publishes the floor it judged the attitude rate against",
          abs((h.get("attitude_floor_hz") or 0) - 29.9) < 0.2,
          "got %r" % (h.get("attitude_floor_hz"),))
    check("/health publishes the frame-rate floor separately",
          h.get("frame_floor_hz") == 60.0)
    imu.reader = FakeReader(rate=20.0, frame=174.0, measured=True,
                            att_age=0.02)
    check("a HALVED attitude rate is still flagged low",
          imu.health().get("imu_rate_low") is True,
          "the honest threshold must still catch a departure from normal")
    imu.reader = FakeReader(rate=49.8, frame=20.0, measured=True,
                            att_age=0.02)
    check("a collapsed FRAME rate is flagged against the 60 Hz spec",
          imu.health().get("imu_rate_low") is True)
    imu.reader = FakeReader(rate=49.8, frame=174.0, measured=True,
                            att_age=0.02)
    imu.info = {"present": True, "sample_rate_hz": 2.0}
    check("a probe taken on a sick device cannot lower the bar on itself",
          abs((imu.health().get("attitude_floor_hz") or 0) - 30.0) < 0.2,
          "got %r - 2 Hz is outside the sane band, so the nominal cadence "
          "must be used" % (imu.health().get("attitude_floor_hz"),))
    imu.info = {"present": True, "sample_rate_hz": 49.8}
    imu.reader = FakeReader(rate=49.8, measured=True, att_age=0.02)
    h = imu.health()
    check("attitude_age_s reported from the ring, not latest()",
          isinstance(h.get("attitude_age_s"), (int, float)))
    check("/health says whether frame-checksum verification is active",
          (h.get("checksum") or {}).get("algo") == "sum_after_sof",
          "the gate can go dormant on an unknown device; 'corrupt frames are "
          "rejected' must be answerable, not assumed")
    check("/health reports the decoder's rejected-frame count",
          h.get("rejected_frames") == 0)
    imu.reader = FakeReader(rate=75.0, measured=True, att_age=0.01)
    check("imu_rate_low clear at 75 Hz",
          imu.health().get("imu_rate_low") is False)
    imu.reader = FakeReader(rate=0.0, measured=False)
    h = imu.health()
    check("probe fallback only while not yet measured",
          h.get("attitude_hz") == 49.8 and h.get("imu_rate_low") is None)
    # Attitude stall with live inertial traffic: the acquire loop must drop
    # the reader (latest() alone stays perpetually fresh and used to hide it).
    imu.reader = FakeReader(rate=0.0, measured=True, att_age=30.0)
    dropped = wait_for(lambda: imu.reader is None, timeout=10.0, interval=0.2)
    check("attitude stall triggers the drop/re-acquire path", dropped,
          "reader kept despite a 30 s old attitude and an empty ring")
    imu.shutdown()

    # ---- P4/P7: fire schedule bounds + node_epoch ------------------------
    sect("audit piagent: fire epoch bounds and node_epoch (P4/P7)")
    check("FIRE_MAX_FUTURE_S lowered to 5.5 s",
          piagent.Gpio.FIRE_MAX_FUTURE_S == 5.5,
          "is %r" % (piagent.Gpio.FIRE_MAX_FUTURE_S,))
    check("FIRE_MAX_PAST_S unchanged at 2 s",
          piagent.Gpio.FIRE_MAX_PAST_S == 2.0)
    for ahead in (0.35, 2.0, 5.4):
        _, fault = piagent.Gpio._epoch_fault(time.time() + ahead)
        check("contract: at_epoch +%.2f s ahead is accepted" % ahead,
              fault is None)
    for ahead in (6.0, 9.5):
        _, fault = piagent.Gpio._epoch_fault(time.time() + ahead)
        check("at_epoch +%.1f s ahead is refused" % ahead, fault is not None)
        if fault:
            check("the refusal carries node_epoch (+%.1f s)" % ahead,
                  isinstance(fault.get("node_epoch"), float))
    _, fault = piagent.Gpio._epoch_fault(time.time() - 1.0)
    check("at_epoch 1 s in the past is still accepted", fault is None)
    _, fault = piagent.Gpio._epoch_fault(time.time() - 3.0)
    check("at_epoch 3 s in the past is refused", fault is not None)

    g3 = _mk_gpio()
    cleanup.append(g3)
    g3.available = False
    r = g3.fire(0, 5)
    check("no-gpio refusal carries node_epoch",
          not r.get("ok") and isinstance(r.get("node_epoch"), float))
    g4 = _mk_gpio()
    cleanup.append(g4)
    g4.available = True
    r = g4.fire(0, 5)                        # FOCUS not held, no lead
    check("FOCUS-not-held 409 carries node_epoch",
          r.get("code") == 409 and isinstance(r.get("node_epoch"), float))
    g4._fire_lock.acquire()
    try:
        r = g4.fire(0, 5, focus_lead_ms=120)
        check("busy 409 carries node_epoch",
              r.get("busy") and isinstance(r.get("node_epoch"), float))
    finally:
        g4._fire_lock.release()
    g4.focus(True)
    r = g4.fire(0, 5)
    check("successful fire still answers ok/fire_seq/node_epoch",
          r.get("ok") and r.get("fire_seq") == 1
          and isinstance(r.get("node_epoch"), float))
    g4.focus(False)

    # ---- P5: epoch_hw hardening + /health stamp order --------------------
    sect("audit piagent: epoch_hw band and /health stamp order (P5)")
    g5 = _mk_gpio()
    cleanup.append(g5)
    fn = getattr(g5, "_edge_hw", None)
    now = time.time()
    good = _hwd(fn, time.monotonic() - 0.01, now)["hw"] \
        if callable(fn) else None
    check("a plausible kernel stamp converts to wall time",
          good is not None and abs(good - now) < 0.1)
    bad = _hwd(fn, time.monotonic() + 5.0, time.time())["hw"] \
        if callable(fn) else "no fn"
    check("a stamp ahead of the read instant is dropped, not published",
          callable(fn) and bad is None,
          "a stamp that postdates the read is a wrong clock domain, not a "
          "late reader")
    # Seconds out is a DOMAIN error (a mis-scaled legacy gpiomon line), which
    # is all an upper bound can honestly catch - see P8 for why the bound may
    # not sit inside the measured read-latency distribution.
    bad2 = _hwd(fn, time.monotonic() - 60.0, time.time())["hw"] \
        if callable(fn) else "no fn"
    check("a stamp a minute behind the read is dropped as out-of-domain",
          callable(fn) and bad2 is None)
    check("rejected stamps are counted in /gpio/state",
          g5.state().get("edges_hw_rejected", 0) >= 2)

    orig_count = piagent._count_frames
    piagent._count_frames = lambda: (time.sleep(0.25), 3)[1]
    try:
        h = piagent.health()
        t_after = time.time()
    finally:
        piagent._count_frames = orig_count
    check("/health stamps time.epoch BEFORE the slow work",
          t_after - h["time"]["epoch"] >= 0.2,
          "stamp sat %.0f ms before the response - it used to be ~0 (stamped "
          "last), which biased the host's clock-offset sample by half the "
          "server-side work" % ((t_after - h["time"]["epoch"]) * 1000))
    check("/health reports the post-stamp work duration",
          h["time"].get("work_ms", 0) >= 200)

    # ---- P8: a late pipe read must not cost the kernel stamp --------------
    # The 0.25 s plausibility band was measured against the wrong quantity.
    # `lag` = epoch - epoch_hw is the gpiomon pipe-read latency itself, which
    # this file records as a 0.09/0.32 ms median "with occasional excursions
    # into the hundreds of ms under load". During such an excursion the kernel
    # stamp is the CORRECT value and `epoch` is the late one - and the band
    # threw the correct one away exactly then, after which run.py fell back to
    # `epoch` and still wrote capture_source=gpio_edge with a clock-error-only
    # bar: a row ~300 ms wrong that claims to be good.
    sect("audit piagent: epoch_hw survives a loaded Pi (P8)")
    g8 = _mk_gpio()
    cleanup.append(g8)
    fn8 = g8._edge_hw
    raw = time.monotonic()
    epoch = time.time() + 0.30                # the read landed 300 ms late
    v = _hwd(fn8, raw, epoch)
    check("a 300 ms pipe-read excursion KEEPS the kernel stamp",
          v["hw"] is not None and abs(v["hw"] - (epoch - 0.30)) < 0.02,
          "dropped it - that is the one moment epoch_hw is worth the most, "
          "and the host then writes the 300 ms-late epoch as gpio_edge")
    check("the measured read latency is published with the edge",
          isinstance(v.get("lag_ms"), float) and 250 <= v["lag_ms"] <= 350,
          "got %r" % (v.get("lag_ms"),))
    check("the edge carries the node's own error bar on epoch_hw",
          isinstance(v.get("err_ms"), float) and 0.0 < v["err_ms"] < 50.0,
          "hw_err_ms is the wall bracket used to convert the stamp, not the "
          "read latency, and it is floored at half a clock tick rather than "
          "claiming a perfect conversion; got %r" % (v.get("err_ms"),))
    check("a kept stamp is not marked rejected", v.get("reject") is None)

    # Through the monitor loop's own path: whatever _edge_hw answers is what
    # _record_edge stores and /gpio/events publishes.
    raw8, ep8 = time.monotonic(), time.time() + 0.30
    g8._record_edge(ep8, "fall", raw8, fn8(raw8, ep8))
    evs8 = g8.exposure_events(0)
    e8 = evs8["events"][-1]
    check("/gpio/events publishes epoch_hw for the late-read edge",
          e8.get("epoch_hw") is not None)
    check("/gpio/events publishes hw_lag_ms and hw_err_ms per edge",
          isinstance(e8.get("hw_lag_ms"), float)
          and isinstance(e8.get("hw_err_ms"), float),
          "the host needs the lag to widen time_err_ms when it falls back")
    check("the events envelope marks this node as publishing the hw fields",
          evs8.get("hw_meta") == 1,
          "without it the host cannot tell 'old piagent, no field' from "
          "'new piagent, no hardware instant for this edge'")

    # A rejection that still happens must be EXPLICIT, not a null the host
    # reads as "older node, fall back to epoch and call it gpio_edge".
    raw_ahead = time.monotonic() + 5.0
    ep_ahead = time.time()
    ahead_v = fn8(raw_ahead, ep_ahead)
    ahead = _hwd(fn8, raw_ahead, time.time())
    check("a refused stamp says WHY, in the edge itself",
          ahead["hw"] is None and ahead.get("reject") == "stamp_ahead",
          "got %r" % (ahead.get("reject"),))
    g8._record_edge(ep_ahead, "fall", raw_ahead, ahead_v)
    e8b = g8.exposure_events(0)["events"][-1]
    check("an hw-less edge is published with hw_reject set",
          e8b.get("epoch_hw") is None
          and e8b.get("hw_reject") == "stamp_ahead")
    check("an hw-less edge offers no error bar to borrow",
          e8b.get("hw_err_ms") is None and e8b.get("hw_lag_ms") is None,
          "epoch is late by an UNMEASURED amount there; a small bar would be "
          "a fabricated one")
    st8 = g8.state()
    check("/gpio/state breaks the rejections down by reason",
          (st8.get("edges_hw_reject_reasons")
           or {}).get("stamp_ahead", 0) >= 1,
          "a bare count cannot tell a degrading node from a broken one")
    check("/gpio/state exposes the read-latency excursion itself",
          (st8.get("edge_hw_lag_ms_max") or 0) >= 250,
          "this is the load signal the fleet view can alarm on before it "
          "reaches the capture instant; got %r"
          % (st8.get("edge_hw_lag_ms_max"),))

    # ---- P9: the FOCUS lease and the node-side intervalometer -------------
    sect("audit piagent: FOCUS lease vs interval schedule (P9)")
    # A period longer than the lease: the loop's own fires are the only thing
    # renewing it, so at period > ttl the watchdog released FOCUS in the gap
    # and every frame after the first came back "FOCUS not held" while
    # /gpio/state still reported running:true. Scaled down (ttl 1 s, period
    # 1.6 s) so the suite does not have to sit through 30 s of lease.
    g9 = _mk_gpio()
    cleanup.append(g9)
    g9.available = True
    old_ttl = piagent.FOCUS_HOLD_DEFAULT_TTL_S
    piagent.FOCUS_HOLD_DEFAULT_TTL_S = 1.0
    try:
        g9.focus(True)                       # a plain operator hold, 1 s lease
        r9 = g9.interval_start(0, 1.6, 3)
        check("interval_start accepts a period longer than the lease",
              r9.get("ok"), "%r" % (r9,))
        done = wait_for(lambda: not g9.interval_status().get("running"),
                        timeout=8.0, interval=0.1)
        st9 = g9.interval_status()
        check("every frame of a long-period interval fires",
              done and st9.get("fired") == 3,
              "fired %r of 3 - the lease lapsed between frames and the "
              "watchdog took FOCUS out from under the schedule"
              % (st9.get("fired"),))
        check("the interval still holds FOCUS at the end of the schedule",
              g9.focus_held())
        left = (g9._focus_hold_expiry or 0) - time.monotonic()
        check("the schedule's long lease shortens once the schedule ends",
              left <= piagent.FOCUS_HOLD_DEFAULT_TTL_S + 0.2,
              "%.1f s of hold left with nothing proving the owner is alive"
              % (left,))
        g9.focus(False)

        # And when FOCUS really does go away, the loop stops and says so
        # instead of logging one failure per remaining frame while still
        # reporting running:true.
        g9b = _mk_gpio()
        cleanup.append(g9b)
        g9b.available = True
        g9b.focus(True)
        g9b.interval_start(0, 0.3, 40)       # 12 s of schedule
        time.sleep(0.5)
        g9b.focus(False)
        stopped = wait_for(lambda: not g9b.interval_status().get("running"),
                           timeout=4.0, interval=0.1)
        check("an interval whose FOCUS is gone stops instead of spinning",
              stopped, "it kept claiming running:true with fired stuck")
        check("...and reports why it stopped",
              (g9b.interval_status().get("error") or "").find("FOCUS") >= 0,
              "got %r" % (g9b.interval_status().get("error"),))
        g9b.interval_stop()          # pre-fix it would spin for another 12 s
    finally:
        piagent.FOCUS_HOLD_DEFAULT_TTL_S = old_ttl

    # The lease must die with the hold on BOTH release paths. On a node
    # without python3-libgpiod, focus(False) goes through _release_focus,
    # which cleared the timestamps but not the granted length - so the next
    # hold that omitted ttl_s inherited it (600 s where the caller and
    # PROTOCOL.md both say 30). Driven directly: the gpioset path cannot be
    # exercised on the Mac, where there is no gpioset to spawn.
    g10 = _mk_gpio()
    cleanup.append(g10)
    with g10._lock:
        g10._focus_lease(True, 600.0)        # a long calibration hold
        inherited_from = g10._focus_hold_ttl
        g10._release_focus()                 # the gpioset release path
        g10._focus_lease(True, None)         # a fresh hold, no ttl_s
        fresh = g10._focus_hold_ttl
    check("a fresh hold after a release gets the default lease, not the last",
          inherited_from == 600.0
          and fresh == piagent.FOCUS_HOLD_DEFAULT_TTL_S,
          "inherited %r s - a host that dies then leaves the body "
          "half-pressed and AE-locked for that long" % (fresh,))

    # ---- P6: imu_yb decoder ----------------------------------------------
    sect("audit imu_yb: checksum, range checks, cadence dating (P6)")
    d = imu_yb._Decoder()
    check("NaN euler frame is rejected by the decoder",
          d.feed("euler", struct.pack("<3f", float("nan"), 0.0, 0.0), 1.0)
          is False)
    check("out-of-range euler frame is rejected",
          d.feed("euler", struct.pack("<3f", 12.6, 0.0, 0.0), 1.0) is False,
          "roll 12.6 rad = 722 deg cannot be an attitude")
    check("non-finite baro frame is rejected",
          d.feed("baro", struct.pack("<4f", 0.0, float("inf"), 1e5, 1e5), 1.0)
          is False)
    check("a sane euler frame still decodes",
          d.feed("euler", struct.pack("<3f", 0.1, -0.2, 1.0), 2.0) is True
          and abs(d.s["roll"] - math.degrees(0.1)) < 1e-6)

    r1 = imu_yb.ImuReader(port="synthetic")
    ing = getattr(r1, "_ingest", None)
    check("ImuReader exposes the chunk pipeline for verification",
          callable(ing))
    if callable(ing):
        t0 = 1000.0
        for i in range(70):                  # learn + enforce the checksum
            r1._ingest(euler_frame(0.001 * i, 0.0, 0.0), t0 + i * 0.02)
        n_ring = len(r1._ring)
        check("checksum algorithm learned from the stream",
              r1.checksum_state().get("algo") == "sum_after_sof"
              and n_ring == 70, "%r" % (r1.checksum_state(),))
        good = euler_frame(0.5, 0.0, 0.0)
        corrupt = good[:-2] + bytes([good[-2] ^ 0xFF]) + good[-1:]
        r1._ingest(corrupt, t0 + 70 * 0.02)
        check("a corrupted frame (stale checksum) never reaches the ring",
              len(r1._ring) == n_ring and r1.rejected_frames() >= 1)
        nan_ck = euler_frame(float("nan"), 0.0, 0.0)   # valid ck, NaN payload
        r1._ingest(nan_ck, t0 + 71 * 0.02)
        check("a NaN attitude with a valid checksum never reaches the ring",
              len(r1._ring) == n_ring
              and not any(s.get("roll") is not None
                          and math.isnan(s["roll"])
                          for _e, s in r1._ring))
        # rate_measured: 0.0 must be distinguishable from "not yet measured".
        check("rate not claimed measured before a full window",
              r1.rate_measured() is False)
        r1._ctr_t0 = time.time() - 2.0
        r1._ingest(euler_frame(0.6, 0.0, 0.0), t0 + 72 * 0.02)
        check("rate reported as measured after a window closes",
              r1.rate_measured() is True)

        # Cadence-bounded dating: two device cycles in ONE chunk must not be
        # dated a byte-time apart (the idle gap is invisible on the wire).
        r2 = imu_yb.ImuReader(port="synthetic")
        tb = 2000.0
        for i in range(80):                  # learn checksum + 20 ms cadence
            r2._ingest(euler_frame(0.001 * i, 0.0, 0.0), tb + i * 0.02)
        two = euler_frame(0.30, 0.0, 0.0) + euler_frame(0.31, 0.0, 0.0)
        t_two = tb + 80 * 0.02
        r2._ingest(two, t_two)
        e_old, e_new = r2._ring[-2][0], r2._ring[-1][0]
        check("newest frame of a batched chunk keeps its byte-time date",
              abs(e_new - t_two) < 1e-6)
        check("older frame of a batched chunk is dated ~a device cycle back",
              (e_new - e_old) > 0.008,
              "gap %.4f s - byte-time alone gives ~0.0015 s, i.e. the older "
              "attitude would be dated up to a whole idle gap late"
              % (e_new - e_old))
        check("cadence dating never reorders same-kind frames",
              e_old > r2._ring[-3][0])
        # ...and it must not reach back past a frame already IN the ring: a
        # read stall followed by one big buffered chunk walks several cycles
        # back, which would append the ring out of order and claim an
        # attitude was measured before one the host may already have bound to
        # a photo.
        r3 = imu_yb.ImuReader(port="synthetic")
        tc = 3000.0
        for i in range(80):
            r3._ingest(euler_frame(0.001 * i, 0.0, 0.0), tc + i * 0.02)
        t_prev = r3._ring[-1][0]
        batch = b"".join(euler_frame(0.4 + 0.01 * k, 0.0, 0.0)
                         for k in range(4))
        r3._ingest(batch, t_prev + 0.002)     # 4 cycles in one late read
        eps = [e for (e, _s) in r3._ring[-5:]]
        check("a batched chunk never dates a frame before one already ringed",
              all(b > a for a, b in zip(eps, eps[1:])),
              "ring epochs %s" % ([round(e - tc, 5) for e in eps],))

    # _reader_fresh: attitude epoch decides, not latest().
    fresh_fn = getattr(piagent.Imu, "_reader_fresh", None)
    check("acquire-loop freshness keys on the last attitude ring publish",
          callable(fresh_fn)
          and fresh_fn(FakeReader(att_age=0.05)) is True
          and fresh_fn(FakeReader(att_age=30.0)) is False)

    # ---- P10: second IMU slot (imu2 / imu_olive) --------------------------
    # The Olive olixVision at cam2 gets its own slot, its own endpoints and a
    # health sub-object nested in the imu section. The contract under test is
    # ADDITIVE ONLY: with the unit absent (this host), every pre-imu2 payload
    # is byte-identical to before, and nothing a wedged slot 2 does can take
    # /health or /imu/* down with it.
    sect("audit piagent: second IMU slot imu2 (P10)")
    import imu_olive                                  # noqa: E402

    check("piagent exposes an IMU2 slot, enabled (auto-probe) by default",
          getattr(piagent, "IMU2", None) is not None
          and piagent.IMU2.enabled is True)
    h2 = piagent.IMU2.health()
    check("imu2 absent on this host reads present:false",
          h2.get("present") is False and h2.get("enabled") is True)

    h = piagent.health()
    check("/health nests imu2 INSIDE the imu section (it rides rigd's "
          "existing forwarding of h['imu'] to the fleet view)",
          (h.get("imu") or {}).get("imu2", {}).get("present") is False)
    check("the imu section is otherwise unchanged (additive only)",
          set(h["imu"].keys()) - {"imu2"} == {"present"},
          "unexpected keys %r" % (set(h["imu"].keys()) - {"imu2", "present"},))

    srv2 = ThreadingHTTPServer(("127.0.0.1", 0), piagent.Handler)
    srv2.daemon_threads = True
    threading.Thread(target=srv2.serve_forever, daemon=True).start()
    base2 = "http://127.0.0.1:%d" % srv2.server_address[1]
    try:
        check("/imu/latest payload is byte-compatible with pre-imu2 piagent",
              _get(base2 + "/imu/latest") == {"present": False},
              "the olive must never leak into the payload rigd elects the "
              "MASTER orientation source from")
        check("/imu2/latest answers the shape /imu/latest always had",
              _get(base2 + "/imu2/latest") == {"present": False})
        check("/imu2/window answers the /imu/window shape",
              _get(base2 + "/imu2/window?t0=0&t1=1")
              == {"present": False, "samples": []})
        # With a live slot-2 reader the endpoint serves its samples.
        piagent.IMU2.reader = FakeReader(rate=100.0, frame=100.0,
                                         measured=True, att_age=0.05)
        try:
            s2 = _get(base2 + "/imu2/latest")
            check("/imu2/latest serves the slot-2 reader's sample",
                  s2.get("roll") == 1.0 and "epoch" in s2)
        finally:
            piagent.IMU2.reader = None
    finally:
        srv2.shutdown()
        srv2.server_close()

    # A wedged slot 2 must cost /health an error FIELD, not the endpoint —
    # /health also carries the clock stamp the host's offset model feeds on.
    class _Wedged:
        def health(self):
            raise RuntimeError("olive wedged")
    old_imu2 = piagent.IMU2
    piagent.IMU2 = _Wedged()
    try:
        h = piagent.health()
        check("a wedged imu2 cannot take /health down",
              h["imu"]["imu2"].get("present") is False
              and "olive wedged" in (h["imu"]["imu2"].get("error") or ""))
    finally:
        piagent.IMU2 = old_imu2

    # Config gate: the PIAGENT_IMU2 spellings.
    ioff = piagent.Imu2(spec="off")
    check("PIAGENT_IMU2=off disables the slot without starting a thread",
          ioff.enabled is False
          and ioff.health() == {"present": False, "enabled": False})
    idev = piagent.Imu2(spec="olive:/dev/ttyACM7")
    iudp = piagent.Imu2(spec="olive:udp:47901")
    check("olive:<device> and olive:udp:<port> specs parse through",
          idev.spec == "/dev/ttyACM7" and iudp.spec == "udp:47901")
    idev.shutdown()
    iudp.shutdown()

    # Slot-2 floors are the SLOT's, not the YB's: a device whose healthy
    # cadence has never been measured must not inherit the 60 Hz frame spec.
    i2f = piagent.Imu2(spec="off")
    i2f.reader = FakeReader(rate=49.8, frame=20.0, measured=True,
                            att_age=0.02)
    i2f.info = {"present": True, "sample_rate_hz": 49.8}
    h2f = i2f.health()
    check("imu2 judges the frame rate against its own floor (10 Hz, not 60)",
          h2f.get("frame_floor_hz") == 10.0
          and h2f.get("imu_rate_low") is False,
          "floor %r low %r - the same reader IS low for slot 1"
          % (h2f.get("frame_floor_hz"), h2f.get("imu_rate_low")))

    # End-to-end through the slot machinery — probe, construct, start,
    # sample — against imu_olive's synthetic stream. No hardware involved.
    isim = piagent.Imu2(spec="olive:sim")
    got = wait_for(lambda: isim.reader is not None, timeout=15.0,
                   interval=0.2)
    check("the acquire loop brings a probed olive online by itself", got)
    if got:
        s = wait_for(lambda: (isim.latest() or {}).get("roll") is not None,
                     timeout=5.0, interval=0.1) and isim.latest()
        check("slot-2 samples carry the imu_yb key set flight_log reads",
              bool(s) and all(k in s for k in
                              ("pitch", "roll", "yaw", "heading", "ax", "ay",
                               "az", "gx", "gy", "gz", "temp", "epoch")),
              "got %r" % (sorted(s) if s else None,))
        check("slot-2 samples are in rig units (the sim streams SI: "
              "|a| must land at ~1 g, angles in degrees)",
              bool(s) and 0.8 < math.sqrt((s["ax"] or 0) ** 2
                                          + (s["ay"] or 0) ** 2
                                          + (s["az"] or 0) ** 2) < 1.2
              and abs(s["roll"]) < 45,
              "%r" % ({k: s.get(k) for k in ("ax", "ay", "az", "roll")}
                      if s else None,))
        hs = wait_for(lambda: isim.health().get("imu_rate_low") is not None,
                      timeout=5.0, interval=0.2) and isim.health()
        check("slot-2 health measures a live attitude rate off the ring",
              bool(hs) and (hs.get("attitude_hz") or 0) > 20
              and hs.get("imu_rate_low") is False,
              "%r" % ({k: hs.get(k) for k in
                       ("attitude_hz", "imu_rate_low", "attitude_floor_hz")}
                      if hs else None,))
    isim.shutdown()

    # The tolerant driver itself: bytes it cannot decode must be COUNTED and
    # SHOWN (that hex tail is tomorrow's bring-up evidence), never raised out
    # of the ingest path or published as samples.
    ro = imu_olive.ImuReader(port="sim")
    ro._tr = imu_olive.SimTransport()        # transport identity only
    ro._ingest(b"\x00\x01\xfe\xba" * 200, time.time())
    sto = ro.checksum_state()
    check("imu_olive: undecodable bytes are counted, not raised or ringed",
          ro.rejected_frames() >= 1 and len(ro._ring) == 0)
    check("imu_olive: the undecoded raw tail is surfaced for bring-up",
          bool(sto.get("raw_tail_hex")) and sto.get("dormant") is True,
          "%r" % (sto,))

    # ---- teardown ---------------------------------------------------------
    for g in cleanup:
        g._watchdog_run = False
        g._mon_run = False
    piagent.GPIO._watchdog_run = False
    piagent.IMU.shutdown()
    piagent.IMU2.shutdown()
    note("piagent audit suite done (temp home: %s)" % _TMP)


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    t0 = time.time()
    suite(a)
    import soaktest as sk
    print("\n%d passed, %d failed in %.0f s"
          % (len(sk.PASS), len(sk.FAIL), time.time() - t0))
    if sk.FAIL:
        print("FAILED: " + ", ".join(sk.FAIL))
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Audit regression suite — nav lane (findings N1/N2/N3).

N1  TimeAuthority starvation: two trusted senders whose time words differ by
    more than CONFIRM_TOL_S (a GNSS 129029 next to a plotter's relayed
    126992, interleaved at 1 Hz) shared ONE candidate slot, so no two
    consecutive accepted feeds ever agreed and a good GPS was rejected for
    the whole run with no alert.  Now candidates are per (pgn, src): each
    sender confirms against itself, the authority is the best-ranked live
    sender (PGN priority, then cadence consistency), and the disagreement is
    exposed in status()["disagreement"] and warned about.

N2  gps_offset bakes the sender->bus->gateway->USB latency into every stamp.
    Nothing is subtracted (never fabricate) — but the assumed bound is now
    visible: status()/health()["time"] carry latency_bound_s and the measured
    offset_scatter_s so consumers know datetime is absolute to ~0.5 s.

N3  navlog.replay() + fix_at(t) used to judge TimeAuthority freshness and
    gateway_online against time.time(), so a desk replay of yesterday's log
    always reported time_source='jetson', offset 0, gateway_online False.
    fix_at now judges at the queried instant itself (confirmations are kept
    as spans), and replay() parks the reader's judgement clock at the last
    recorded line (NavReader.set_clock) so the instant-less views —
    snapshot(), health(), gateway_online — describe the end of the log too.

Hermetic: no serial ports, no network, no rig processes; NavReader instances
are never open()ed/start()ed — lines are pushed with feed_line()/replay().

Run standalone:  python3 rig/tests/audit_nav.py
"""

import base64
import os
import shutil
import struct
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

from soaktest import check, sect, note          # noqa: E402
import nav                                       # noqa: E402
import navlog                                    # noqa: E402

LAT, LON = 43.642567, -79.387139


# ---------------------------------------------------------------------------
# PGN payload builders (layouts cross-checked against nav's decoders)
# ---------------------------------------------------------------------------

def line(pgn, payload, src=3):
    return "!PDGY,%d,2,%d,255,1.234,%s" % (
        pgn, src, base64.b64encode(payload).decode())


def p_129025(lat=LAT, lon=LON):
    return struct.pack("<ii", round(lat * 1e7), round(lon * 1e7))


def p_129029(gps_epoch, method=1, sats=12):
    days = int(gps_epoch // 86400)
    tod = gps_epoch - days * 86400
    d = struct.pack("<BHIqqq", 1, days, round(tod * 1e4),
                    round(LAT * 1e16), round(LON * 1e16), round(76.543 * 1e6))
    d += bytes([(method << 4) & 0xFF, 0xFC, sats])
    d += struct.pack("<hh", 90, 150) + struct.pack("<i", -3520) + bytes([0])
    return d


def p_126992(gps_epoch, source=0):
    days = int(gps_epoch // 86400)
    tod = gps_epoch - days * 86400
    return struct.pack("<BBHI", 1, source & 0x0F, days, round(tod * 1e4))


def gnss(epoch, method=1, sats=9):
    """A decoded 129029 dict, for driving TimeAuthority.feed() directly."""
    return {"epoch": epoch, "method": method, "sats": sats}


def reader():
    return nav.NavReader(port="/dev/null", auto_reopen=False)


# Compat shims: on PRE-fix code TimeAuthority.source() takes no instant and
# feed() takes no src.  Falling back keeps this suite runnable there so it
# reports honest FAILs (reproducing the defects) instead of crashing.

def src_at(ta, t):
    try:
        return ta.source(t)
    except TypeError:
        return ta.source()


def feed(ta, pgn, decoded, t, src=None):
    try:
        return ta.feed(pgn, decoded, t, src=src)
    except TypeError:
        return ta.feed(pgn, decoded, t)


def set_clock(rd, fn):
    """True if the reader accepted a judgement clock (absent pre-fix)."""
    setter = getattr(rd, "set_clock", None)
    if not callable(setter):
        return False
    setter(fn)
    return True


def replay_nofreeze(path, rd):
    try:
        return navlog.replay(path, rd, freeze_clock=False)
    except TypeError:
        return navlog.replay(path, rd)


# ---------------------------------------------------------------------------
# N1 — disagreeing trusted senders must still confirm, visibly
# ---------------------------------------------------------------------------

def _n1_disagreeing_senders(opts):
    sect("audit N1: two disagreeing trusted senders")
    rd = reader()
    warns = []
    rd.time_authority.on_warn = warns.append
    base = time.time()
    # GNSS sender (src 3, 129029) offset +2.0 s; plotter (src 42, 126992
    # source nibble 0 = GPS-sourced relay) offset +2.6 s — the |delta| =
    # 0.6 s > CONFIRM_TOL_S starvation scenario from the finding.
    for i in range(6):
        t = base + i
        rd.feed_line(line(129029, p_129029(t + 2.0), src=3), t)
        rd.feed_line(line(126992, p_126992(t + 0.5 + 2.6), src=42), t + 0.5)
    last = base + 5.5
    ta = rd.time_authority
    check("disagreeing senders no longer starve confirmation",
          src_at(ta, last) == "gps", str(ta.status()))
    check("authority is the 129029 (satellite-instant) sender",
          abs(ta.offset_to(last) - 2.0) < 0.05,
          "offset %r" % ta.offset_to(last))
    st = ta.status()
    dis = st.get("disagreement")
    check("disagreement exposed in status()", isinstance(dis, dict), str(st))
    others = (dis or {}).get("others") or []
    check("disagreement names the disagreeing sender (pgn 126992 src 42)",
          any(o.get("pgn") == 126992 and o.get("src") == 42 for o in others),
          str(dis))
    check("disagreement spread ~0.6 s",
          dis is not None and abs(dis.get("spread_s", 0) - 0.6) < 0.1,
          str(dis))
    check("chosen sender recorded", (dis or {}).get("chosen", {}).get("pgn")
          == 129029, str(dis))
    check("warn emitted naming the disagreeing sender",
          any("126992" in m and "42" in m for m in warns), str(warns))
    check("health()['time'] carries the disagreement",
          rd.health()["time"].get("disagreement") is not None)
    check("health()['time'] names confirming sender",
          rd.health()["time"].get("from_src") == 3,
          str(rd.health()["time"].get("from_src")))


def _n1_confirmation_still_guarded(opts):
    sect("audit N1: confirmation guard survives the rework")
    now = time.time()
    # one sender, one frame: still not authority
    ta = nav.TimeAuthority()
    feed(ta, 129029, gnss(now + 2.0), now, src=7)
    check("single frame is still jetson", src_at(ta, now) == "jetson")
    # same sender disagreeing with itself: still refused
    feed(ta, 129029, gnss(now + 1.0 + 40.0), now + 1.0, src=7)   # off 40.0
    check("self-disagreeing pair stays jetson",
          src_at(ta, now + 1.0) == "jetson")
    # then agreeing with itself twice: confirmed
    feed(ta, 129029, gnss(now + 2.0 + 40.0), now + 2.0, src=7)   # off 40.0
    check("self-agreeing pair confirms", src_at(ta, now + 2.0) == "gps")
    check("confirmed offset is the agreed one",
          abs(ta.offset_to(now + 2.0) - 40.0) < 1e-6)
    # a second sender must not be able to confirm by agreeing with the FIRST
    # sender's candidate (cross-sender confirmation was the old semantics
    # and would let two one-frame glitches from different devices promote)
    tb = nav.TimeAuthority()
    feed(tb, 129029, gnss(now + 5.0), now, src=1)
    feed(tb, 129029, gnss(now + 1.0 + 5.0), now + 1.0, src=2)
    check("cross-sender agreement alone does not confirm",
          src_at(tb, now + 1.0) == "jetson", str(tb.status()))
    # no disagreement dict when senders agree within tolerance
    tc = nav.TimeAuthority()
    for i in range(4):
        feed(tc, 129029, gnss(now + i + 2.0), now + i, src=1)
        feed(tc, 126992, {"epoch": now + i + 0.5 + 2.1, "source": 0},
                now + i + 0.5, src=9)
    check("agreeing senders -> no disagreement reported",
          tc.status().get("disagreement") is None, str(tc.status()))


def _n1_cadence_tiebreak(opts):
    sect("audit N1: cadence consistency breaks a same-PGN tie")
    now = time.time()
    ta = nav.TimeAuthority()
    ta.on_warn = lambda m: None          # keep the suite's output clean
    # src 5: jittery cadence, offset +10.0; src 6: metronome, offset +10.8
    feeds = [(now + 0.10, 5, 10.0), (now + 0.50, 6, 10.8),
             (now + 0.90, 5, 10.0), (now + 1.50, 6, 10.8),
             (now + 2.30, 5, 10.0), (now + 2.50, 6, 10.8),
             (now + 2.90, 5, 10.0), (now + 3.50, 6, 10.8)]
    for t, src, off in feeds:
        feed(ta, 129029, gnss(t + off), t, src=src)
    check("steadier sender wins the same-priority tie",
          abs(ta.offset_to(now + 3.5) - 10.8) < 0.05,
          "offset %r senders %r" % (ta.offset_to(now + 3.5),
                                    ta.status().get("senders")))


# ---------------------------------------------------------------------------
# N2 — the latency bias bound must be visible; nothing subtracted
# ---------------------------------------------------------------------------

def _n2_latency_bound(opts):
    sect("audit N2: latency bias bound exposed, never subtracted")
    now = time.time()
    ta = nav.TimeAuthority()
    # confirmed offsets 2.00 / 2.04 / 2.02: mid-jitter, one sender
    for i, off in enumerate((2.00, 2.04, 2.02)):
        feed(ta, 129029, gnss(now + i + off), now + i, src=3)
    st = ta.status()
    # getattr: on pre-fix code the constant does not exist — the checks must
    # FAIL there, not crash
    bound = getattr(nav.TimeAuthority, "LATENCY_BOUND_S", None)
    check("latency_bound_s exposed in status()",
          bound is not None and st.get("latency_bound_s") == bound
          and st.get("latency_bound_s", 0) > 0, str(st.get("latency_bound_s")))
    check("offset applied verbatim — latency NOT silently subtracted",
          abs(ta.offset_to(now + 2) - 2.02) < 1e-6,
          "offset %r" % ta.offset_to(now + 2))
    check("offset_scatter_s measures the sender's spread",
          st.get("offset_scatter_s") is not None
          and abs(st["offset_scatter_s"] - 0.04) < 0.005,
          str(st.get("offset_scatter_s")))
    check("latency_subtracted_s is explicitly 0.0 (nothing fabricated)",
          st.get("latency_subtracted_s") == 0.0,
          str(st.get("latency_subtracted_s")))
    check("abs_error_bound_s = latency bound + measured scatter",
          bound is not None and st.get("abs_error_bound_s") is not None
          and abs(st["abs_error_bound_s"] - (bound + 0.04)) < 0.005,
          str(st.get("abs_error_bound_s")))
    rd = reader()
    t = rd.health()["time"]
    check("health()['time'] carries latency_bound_s",
          bound is not None and t.get("latency_bound_s") == bound, str(t))
    check("no authority -> no abs_error_bound_s claimed",
          t.get("abs_error_bound_s") is None, str(t.get("abs_error_bound_s")))
    note("latency bound %s s is the documented sender->bus->gateway->USB "
         "assumption; consumers must treat GPS-corrected stamps as absolute "
         "to that bound" % bound)


def _n1_starvation_is_measurable(opts):
    """The starvation N1 removes must still be REPORTABLE when it happens for
    another reason (a single sender that never agrees with itself), so rigd
    can raise nav_time_unconfirmed instead of silently running on jetson."""
    sect("audit N1: unconfirmed starvation is exposed as a duration")
    now = time.time()
    ta = nav.TimeAuthority()
    ta.on_warn = lambda m: None
    for i in range(12):                       # offset walks 5 s every feed
        feed(ta, 129029, gnss(now + i + 5.0 * i), now + i, src=3)
    st = ta.status()
    check("never confirmed -> candidate_pending", st.get("candidate_pending")
          is True, str(st))
    check("unconfirmed_for_s reports how long it has starved",
          st.get("unconfirmed_for_s") is not None
          and abs(st["unconfirmed_for_s"] - 11.0) < 0.1,
          str(st.get("unconfirmed_for_s")))
    check("still jetson while unconfirmed",
          src_at(ta, now + 11.0) == "jetson", str(st.get("source")))
    tb = nav.TimeAuthority()
    for i in range(4):
        feed(tb, 129029, gnss(now + i + 3.0), now + i, src=3)
    check("healthy authority reports no starvation",
          tb.status().get("candidate_pending") is False
          and tb.status().get("unconfirmed_for_s") is None,
          str(tb.status()))


# ---------------------------------------------------------------------------
# N3 — replay must reproduce the live time/gateway judgements
# ---------------------------------------------------------------------------

def _n3_replay_clock(opts):
    sect("audit N3: replayed log reproduces live judgements at the instant")
    tmp = tempfile.mkdtemp(prefix="audit-nav-")
    try:
        t0 = time.time() - 3 * 86400        # the run happened three days ago
        path = os.path.join(tmp, "nmea_raw.log")
        lg = navlog.RawNmeaLog(path, note_open=False)
        lg.write_line(t0, 1.0, "$PDGY,000000,12,0,4,100,34,0")
        lg.write_line(t0 + 0.1, 1.1, line(129025, p_129025()))
        lg.write_line(t0 + 0.2, 1.2, line(129029, p_129029(t0 + 0.2 + 2.0)))
        lg.write_line(t0 + 1.2, 2.2, line(129029, p_129029(t0 + 1.2 + 2.0)))
        lg.close()

        rd = reader()
        n = navlog.replay(path, rd)
        check("replay fed the whole log", n == 4, str(n))
        t_cap = t0 + 1.0
        fx = rd.fix_at(t_cap)
        check("replayed instant reports time_source gps",
              fx["time_source"] == "gps", str(fx["time_source"]))
        check("replayed instant keeps the recorded gps offset",
              abs(fx["gps_offset_s"] - 2.0) < 0.05,
              str(fx["gps_offset_s"]))
        check("replayed corrected epoch = instant + offset",
              abs(fx["epoch"] - (t_cap + 2.0)) < 0.05, str(fx["epoch"]))
        check("replayed instant reports gateway_online",
              fx["gateway_online"] is True)
        check("replayed position still resolves",
              fx["valid"] and abs(fx["lat"] - LAT) < 1e-6, str(fx["lat"]))
        # past the recorded span the authority must lapse ON THE REPLAYED
        # CLOCK, exactly as it did live
        t_late = t0 + 1.2 + nav.TimeAuthority.STALE_S + 1.0
        fx2 = rd.fix_at(t_late)
        check("staleness honored relative to the replayed epoch",
              fx2["time_source"] == "jetson" and fx2["gps_offset_s"] == 0.0,
              "%s %s" % (fx2["time_source"], fx2["gps_offset_s"]))
        # the instant-less views (snapshot/health/gateway_online) have no
        # epoch to judge at, so replay parks the reader's judgement clock at
        # the last recorded line: they describe the END of the log, not this
        # afternoon.  Pre-fix these were time.time() and said stale/offline.
        check("frozen replay clock: gateway_online as at end of log",
              rd.gateway_online is True, str(rd.gateway_online))
        snap = rd.snapshot()
        check("frozen replay clock: snapshot reports the recorded time source",
              snap["time_source"] == "gps", str(snap["time_source"]))
        check("frozen replay clock: snapshot keeps the recorded offset",
              abs(snap["gps_offset_s"] - 2.0) < 0.05,
              str(snap["gps_offset_s"]))
        h = rd.health()
        check("frozen replay clock: health last_rx_age_s is log-relative",
              h["last_rx_age_s"] is not None and h["last_rx_age_s"] < 5.0,
              str(h["last_rx_age_s"]))
        check("frozen replay clock: health()['time'] not stale",
              h["time"]["source"] == "gps", str(h["time"]["source"]))
        # ...and it is opt-out, and reversible: a reader handed back the wall
        # clock judges three-day-old data stale/offline again
        restored = set_clock(rd, None)
        check("set_clock(None) restores the wall clock",
              restored and rd.gateway_online is False
              and rd.time_authority.source() == "jetson",
              "set_clock present: %r" % restored)
        rd2 = reader()
        replay_nofreeze(path, rd2)
        check("freeze_clock=False keeps wall-clock judgement",
              rd2.gateway_online is False
              and rd2.time_authority.source() == "jetson")
        check("freeze_clock=False still reproduces fix_at at the instant",
              rd2.fix_at(t_cap)["time_source"] == "gps",
              str(rd2.fix_at(t_cap)["time_source"]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _n3_live_pulled_frame(opts):
    sect("audit N3: live frame pulled after a GPS drop keeps its authority")
    now = time.time()
    rd = reader()
    # GPS confirmed around the exposure instant (8 s ago), then went silent;
    # run.py pulls and stamps the frame NOW.
    for i in range(3):
        t = now - 8.0 + i
        rd.feed_line(line(129029, p_129029(t + 2.0)), t)
        rd.feed_line(line(129025, p_129025()), t)
    t_exp = now - 7.0
    fx = rd.fix_at(t_exp, max_age_s=10.0)
    check("exposure instant inside the confirmed span stays gps",
          fx["time_source"] == "gps" and abs(fx["gps_offset_s"] - 2.0) < 0.05,
          "%s %s" % (fx["time_source"], fx["gps_offset_s"]))
    check("but the authority is stale NOW (wall clock)",
          rd.time_authority.source() == "jetson")


def _n3_offset_step_not_backdated(opts):
    """Judging at the instant is only worth anything if the recorded history
    is honest: an offset STEP mid-run (a sender failover, or GPS stepping the
    host clock) must not re-stamp frames exposed before the step."""
    sect("audit N3: an offset step does not rewrite earlier instants")
    now = time.time()
    ta = nav.TimeAuthority()
    ta.on_warn = lambda m: None
    for i in range(4):                          # offset +2.0 s, t = now..now+3
        feed(ta, 129029, gnss(now + i + 2.0), now + i, src=3)
    early = now + 3.0
    check("offset before the step", abs(ta.offset_to(early) - 2.0) < 1e-6,
          str(ta.offset_to(early)))
    for i in range(4, 8):                       # steps to +9.0 s at now+4
        feed(ta, 129029, gnss(now + i + 9.0), now + i, src=3)
    late = now + 7.0
    check("offset after the step", abs(ta.offset_to(late) - 9.0) < 1e-6,
          str(ta.offset_to(late)))
    check("the pre-step instant keeps its pre-step offset",
          abs(ta.offset_to(early) - 2.0) < 1e-6,
          "offset at the earlier instant is now %r" % ta.offset_to(early))


# ---------------------------------------------------------------------------

def suite(opts):
    _n1_disagreeing_senders(opts)
    _n1_confirmation_still_guarded(opts)
    _n1_cadence_tiebreak(opts)
    _n1_starvation_is_measurable(opts)
    _n2_latency_bound(opts)
    _n3_replay_clock(opts)
    _n3_live_pulled_frame(opts)
    _n3_offset_step_not_backdated(opts)


if __name__ == "__main__":
    import argparse
    import soaktest
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    suite(opts)
    print("\naudit_nav: %d passed, %d failed"
          % (len(soaktest.PASS), len(soaktest.FAIL)))
    sys.exit(1 if soaktest.FAIL else 0)

#!/usr/bin/env python3
"""Audit regression suite — rigcore lane (findings K1-K10).

K1  NodeMonitor.clock_offset_s()/clock_offset_info() (contract C1): the host
    Mac's clock is not disciplined (187 ms behind NTP, ~60 ppm) while the two
    Pis are chrony-locked, so node-minus-host is load-bearing for the fire
    schedule and for nav lookups. One raw sample rides that poll's RTT; the
    published figure is an RTT-gated median of the recent window.

K2  A forced reconcile used to re-push the whole EXPOSURE vector to every
    body, so a format/WB/focus-mode apply, an EV bump or a discarded preview
    silently wiped a deliberate per-camera exposure split. And a PERSISTENT
    non-exposure readback mismatch (a DisplayOnly storeDest, ilxctl's 0/-1
    "never read that property" sentinels) made every idle reconcile a forced
    pass, so the split was overwritten every 3 s.

K3  filetype/imagesize/transsize/rawtype (+ expcomp) are readable on ilxctl
    builds that emit <name>Value (contract C2) and blind on older ones. A body
    that ACCEPTS the write and does not apply it used to be reported "synced"
    while the transect recorded no RAW.

K4  focus_mode may only ever be 1 (MF). POST /api/focus/mode 2 put the live
    fleet in AF-S on 2026-08-23 and the lens positions moved mid-excursion.

K5  reconcile_all must not push into a body a card drain holds in transfer
    mode (suspend_control).

K6  EventLog resolves RIGD_LOG at construction, not at class definition, and
    the journal really does rotate at MAX_BYTES.

K7  A run id built from a non-ASCII label never matched _RUN_ID_RE, so the
    transect was invisible to the browser and was never finalised by startup
    recovery.

K8  desired.json is validated on load, and a file that will not parse is kept
    (renamed .bad-<ts>) instead of being overwritten with built-in defaults -
    which used to record JPEG-only.

K9  bump_ev goes through validation (+-5000 mEV).

K10 RunBrowser prefers index.jsonl (contract C3), tolerates a missing or
    corrupt run.json, and does not print a measured 0.00 ms inter-camera
    spread for a shot only one camera recorded.

K12 Releasing a preview pin because `desired` changed must also RE-CONVERGE
    the camera it was pinned on: the apply's own pass writes only the exposure
    fields it named, so a format/WB/focus-mode apply left the previewed body
    on the preview's exposure with the pin, the badge and the run-start
    discard all gone at the same instant.

K13 ilxctl answers /api/status with a degraded body (connected + busy, no
    property block) when it cannot take the SDK mutex inside 4.5 s. Its
    `connected` is isConnected(), which is TRUE for a body wedged in
    RemoteTransfer, and its missing controlMode reads exactly like "not in
    transfer" - so a body that CANNOT shoot was promoted to CAM_CONNECTED and
    the whole settings vector was pushed into it.

K14 piagent stamps /health's epoch on entry and publishes the handler's
    work_ms. It is published as a diagnostic (work_ms/link_ms) and
    deliberately NOT folded into offset_s or out of rtt_ms - see the comment
    in NodeMonitor._tick.

Hermetic: temp dirs only, loopback fakes only (soaktest's netguard is
installed by importing it), monitors are ticked by hand rather than started.

Run standalone:  python3 rig/tests/audit_rigcore.py
"""

import json
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

from soaktest import check, note, sect          # noqa: E402
import rigcore                                   # noqa: E402
from fakenode import FakeNode                    # noqa: E402


# ---------------------------------------------------------------------------
# A minimal fleet: fakes + monitors + SettingsManager on a throwaway RIG_HOME.
# Deliberately NOT soaktest.Env — nothing here needs a RunManager, and staying
# off run.py keeps the suite fast and independent of that lane.
# ---------------------------------------------------------------------------
_GLOBALS = ("RIG_HOME", "DESIRED_PATH", "RUNS_DIR", "RIGD_LOG")


class Fleet:
    def __init__(self, specs=(("cam1", "127.0.0.11", 1),
                              ("cam2", "127.0.0.12", 2)),
                 desired_json=None, make_settings=True):
        self._saved = {k: getattr(rigcore, k) for k in _GLOBALS}
        self.dir = tempfile.mkdtemp(prefix="wildsync-audit-rigcore-")
        rigcore.RIG_HOME = os.path.join(self.dir, "rig")
        rigcore.DESIRED_PATH = os.path.join(rigcore.RIG_HOME, "desired.json")
        rigcore.RUNS_DIR = os.path.join(self.dir, "runs")
        rigcore.RIGD_LOG = os.path.join(rigcore.RIG_HOME, "rigd.jsonl")
        os.makedirs(rigcore.RIG_HOME, exist_ok=True)
        os.makedirs(rigcore.RUNS_DIR, exist_ok=True)
        if desired_json is not None:
            with open(rigcore.DESIRED_PATH, "w") as fh:
                fh.write(desired_json)
        self.events = rigcore.EventLog(path=rigcore.RIGD_LOG, ring=3000)
        self.nodes, self.monitors = {}, []
        for name, host, cam_num in specs:
            self.nodes[name] = FakeNode(name, host, cam_num=cam_num)
            self.monitors.append(rigcore.NodeMonitor(
                {"name": name, "cam_num": cam_num, "host": host},
                self.events, poll=0.25))
        self.settings = (rigcore.SettingsManager(self.monitors, self.events)
                         if make_settings else None)

    def mon(self, name):
        return next(m for m in self.monitors if m.name_ == name)

    def node(self, name):
        return self.nodes[name]

    def tick(self, n=1):
        for _ in range(n):
            for m in self.monitors:
                m._tick()

    def seq(self):
        return self.events._seq

    def evs(self, since=0, kind=None, node=None):
        out = self.events.since(since, limit=100000)["events"]
        if kind:
            out = [e for e in out if e["kind"] == kind]
        if node:
            out = [e for e in out if e["node"] == node]
        return out

    def settle(self):
        """Consume the one forced exposure pass each monitor arms on its
        OFFLINE->CAM_CONNECTED transition, and land the fleet on desired."""
        self.tick()
        for _ in range(3):
            self.settings.reconcile_all(force=True)
            self.tick()
        self.settings.reconcile_all(force=False)
        self.tick()

    def close(self):
        for m in self.monitors:
            m.stop()
        for n in self.nodes.values():
            n.close()
        shutil.rmtree(self.dir, ignore_errors=True)
        for k, v in self._saved.items():
            setattr(rigcore, k, v)


def _call(fn, *a, **kw):
    """Call something that may not exist yet on pre-fix code: a missing API is
    a FAILED check, not a suite that dies half way through."""
    try:
        return fn(*a, **kw), None
    except Exception as e:                                    # noqa: BLE001
        return None, "%s: %s" % (type(e).__name__, e)


def _ilx_calls(node, since):
    return len([c for c in node.calls_since(since) if c[1] == "ilx"])


# ===========================================================================
# K1 — filtered node-minus-host clock offset (contract C1)
# ===========================================================================
def k1_clock_offset(opts):
    sect("K1 clock offset: RTT-gated median, not the last raw sample (C1)")
    f = Fleet(specs=(("cam1", "127.0.0.11", 1),), make_settings=False)
    try:
        m = f.mon("cam1")
        if not hasattr(m, "clock_offset_s") or not hasattr(
                m, "clock_offset_info"):
            check("NodeMonitor exposes clock_offset_s/clock_offset_info", False,
                  "contract C1 API missing")
            return
        v, err = _call(m.clock_offset_s)
        info, _ = _call(m.clock_offset_info)
        check("no samples yet -> None, not a fabricated zero",
              v is None and (info or {}).get("n") == 0, err or json.dumps(info))

        now = time.time()
        # Eight polls of a node 0.187 s ahead of us; ONE of them sat behind a
        # 400 ms network stall, which lands 1:1 in that sample's midpoint
        # estimate. The stalled sample must not move the published figure.
        good = [0.186, 0.188, 0.187, 0.187, 0.186, 0.189, 0.188]
        with m._lock:
            for i, off in enumerate(good):
                m._clock_hist.append({"offset_s": off, "rtt_ms": 3.0 + i * 0.1,
                                      "at": now - (7 - i) * 2.0})
            m._clock_hist.append({"offset_s": 0.387, "rtt_ms": 400.0,
                                  "at": now - 0.2})
        v, err = _call(m.clock_offset_s)
        info, _ = _call(m.clock_offset_info)
        check("a 400 ms RTT spike is gated out of the offset",
              v is not None and abs(v - 0.187) < 0.002,
              "offset=%s info=%s" % (v, json.dumps(info)))
        check("clock_offset_info reports the surviving sample count and best RTT",
              (info or {}).get("n") == len(good)
              and abs(((info or {}).get("rtt_ms_best") or 99) - 3.0) < 0.01
              and (info or {}).get("age_s") is not None,
              json.dumps(info))

        # Ageing out: nothing sampled in the last minute is not an offset any
        # more, and the fire scheduler must not lean on a stale one.
        with m._lock:
            m._clock_hist.clear()
            m._clock_hist.append({"offset_s": 0.187, "rtt_ms": 3.0,
                                  "at": now - 120.0})
        v, _ = _call(m.clock_offset_s)
        info, _ = _call(m.clock_offset_info)
        check("a sample older than 60 s is not served as current",
              v is None and (info or {}).get("n") == 0, json.dumps(info))

        # An even-sized window medians to the mean of the middle pair.
        with m._lock:
            m._clock_hist.clear()
            for off in (0.10, 0.20):
                m._clock_hist.append({"offset_s": off, "rtt_ms": 2.0,
                                      "at": now})
        v, _ = _call(m.clock_offset_s)
        check("an even window medians the middle pair", v is not None
              and abs(v - 0.15) < 1e-9, str(v))

        # And the real sampler feeds it: two ticks against a live fake.
        with m._lock:
            m._clock_hist.clear()
        f.tick(2)
        v, _ = _call(m.clock_offset_s)
        check("the monitor's own polls feed the filter",
              v is not None and abs(v) < 5.0, str(v))
        check("the raw per-poll sample is still published for diagnostics",
              isinstance(m.clock, dict) and "offset_s" in (m.clock or {}),
              str(m.clock))
    finally:
        f.close()


# ===========================================================================
# K2 — a per-camera exposure split survives everything but an exposure apply
# ===========================================================================
def _split(f, cam="cam2", iso=3200, aperture=400):
    """The operator balances one body against unequal strobes."""
    f.node(cam).drift(iso=iso, aperture=aperture)
    f.tick()


def k2_exposure_split(opts):
    sect("K2 per-camera exposure is not wiped by a non-exposure apply")
    f = Fleet()
    try:
        S, b = f.settings, f.node("cam2")
        f.settle()

        _split(f)
        S.update({"colortemp": 5000})
        f.tick()
        check("a WB/format apply leaves a deliberate per-camera split alone",
              b.raw("iso") == 3200 and b.raw("aperture") == 400,
              "iso=%s aperture=%s (a {'colortemp':5000} POST used to re-push "
              "the whole exposure vector to every body)"
              % (b.raw("iso"), b.raw("aperture")))
        check("the non-exposure field the operator DID apply still lands",
              b.raw("colortemp") == 5000, str(b.raw("colortemp")))

        S.update({"focus_mode": 1})
        f.tick()
        check("a focus-mode apply leaves the split alone",
              b.raw("iso") == 3200, "iso=%s" % b.raw("iso"))

        S.update({"filetype": 3, "imagesize": 3})
        f.tick()
        check("a capture-format apply leaves the split alone",
              b.raw("iso") == 3200, "iso=%s" % b.raw("iso"))

        ev0 = S.get().get("expcomp", 0)
        S.bump_ev(1)
        f.tick()
        check("an EV bump moves expcomp only, not the split",
              b.raw("iso") == 3200 and S.get()["expcomp"] == ev0 + 333,
              "iso=%s expcomp=%s" % (b.raw("iso"), S.get()["expcomp"]))

        # A preview pinned on cam1, then discarded: cam2 was never part of it.
        rep = S.preview({"iso": 800}, node="cam1")
        f.tick()
        S.discard()
        f.tick()
        check("discarding a preview on cam1 leaves cam2's split alone",
              b.raw("iso") == 3200,
              "iso=%s (preview=%s)" % (b.raw("iso"), json.dumps(rep)[:120]))
        check("the previewed camera itself IS pulled back to the fleet vector",
              f.node("cam1").raw("iso") == S.get()["iso"],
              "cam1 iso=%s want=%s"
              % (f.node("cam1").raw("iso"), S.get()["iso"]))

        # ...but an explicit EXPOSURE apply is exactly what re-converges it.
        S.update({"iso": 800})
        f.tick()
        check("an explicit exposure apply DOES re-converge the split camera",
              b.raw("iso") == 800, "iso=%s" % b.raw("iso"))

        # A body that reconnected may have reset: that tell still forces one
        # full exposure pass, which is the only implicit exposure write left.
        _split(f, iso=1600)
        f.mon("cam2")._exposure_force = True
        S.reconcile_all(force=False)
        f.tick()
        check("a reconnect tell still re-pushes exposure to that body",
              b.raw("iso") == 800, "iso=%s" % b.raw("iso"))
    finally:
        f.close()


def k2_sentinel_revert(opts):
    sect("K2 a persistent mismatch is not a 'the body reset' tell")
    f = Fleet()
    try:
        S, b, mb = f.settings, f.node("cam2"), f.mon("cam2")
        f.settle()
        _split(f)

        # Trigger B of the finding: ilxctl publishes storeDest/driveValue/
        # focusMode as 0 (camera.cpp statusJson defaults) when the property
        # read failed, e.g. a body whose property table is busy or locked.
        # That is "not reported", not evidence of a revert.
        b.label_override = {"storeDest": 0, "driveValue": 0, "focusMode": 0}
        f.tick()
        s0 = f.seq()
        b.clear_counts()
        for _ in range(3):
            S.reconcile_all(force=False)
            f.tick()
        check("ilxctl's 0 'not reported' sentinels do not force an exposure pass",
              b.raw("iso") == 3200 and b.raw("aperture") == 400,
              "iso=%s aperture=%s after 3 idle reconciles"
              % (b.raw("iso"), b.raw("aperture")))
        check("nor do they restart the blind-field write storm",
              len(b.pushed("filetype", "imagesize", "transsize",
                           "rawtype")) == 0,
              "%d blind-field writes" % len(b.pushed("filetype", "imagesize",
                                                     "transsize", "rawtype")))
        b.label_override = {}
        f.tick()

        # Trigger A: storeDest is DisplayOnly on these bodies, so a body-menu
        # toggle disagrees with desired on EVERY pass, forever.
        b.label_override = {"storeDest": 2,
                            "writable": {"storeDest":
                                         rigcore.ENABLE_DISPLAY_ONLY}}
        f.tick()
        for _ in range(3):
            S.reconcile_all(force=False)
            f.tick()
        check("a DisplayOnly field stuck off-target does not wipe the split",
              b.raw("iso") == 3200 and b.raw("aperture") == 400,
              "iso=%s aperture=%s" % (b.raw("iso"), b.raw("aperture")))
        check("it is reported as unsettable, not silently ignored",
              "store_dest" in (mb.convergence.get("unsettable") or []),
              json.dumps(mb.convergence))
        b.label_override = {}
        f.tick()

        # A field that simply will not converge (a refused write) is a standing
        # divergence, not a NEW revert, so it must not re-force exposure either.
        b.set_fault("ilx", fail_set=["drive"])
        b.drift(drive=2)
        f.tick()
        # The FIRST pass sees a new revert and legitimately re-forces exposure
        # (that is the reboot tell). By the second, drive is a field that is
        # KNOWN not to converge - a standing divergence, not a fresh reset.
        for _ in range(2):
            S.reconcile_all(force=False)
            f.tick()
        _split(f)                       # operator balances the pair NOW
        for _ in range(3):
            S.reconcile_all(force=False)
            f.tick()
        check("a standing divergence is not re-read as a reset every pass",
              b.raw("iso") == 3200,
              "iso=%s (drive is stuck at %s)" % (b.raw("iso"), b.raw("drive")))
        check("the stuck field is still reported diverged",
              "drive" in (mb.convergence.get("diverged") or []),
              json.dumps(mb.convergence))
        b.clear_faults()
        f.tick()

        # But a genuine change under our feet IS still the reboot tell.
        S.reconcile_all(force=False)
        f.tick()
        S.reconcile_all(force=False)
        f.tick()
        _split(f, iso=6400)
        b.drift(store_dest=2)
        f.tick()
        S.reconcile_all(force=False)
        f.tick()
        check("a NEW non-exposure revert still re-forces exposure (reboot tell)",
              b.raw("store_dest") == S.get()["store_dest"]
              and b.raw("iso") == S.get()["iso"],
              "store=%s iso=%s" % (b.raw("store_dest"), b.raw("iso")))
    finally:
        f.close()


# ===========================================================================
# K3 — capture-format readback (contract C2) with a blind fallback
# ===========================================================================
def k3_format_readback(opts):
    sect("K3 filetype/imagesize/transsize/expcomp readback when reported (C2)")
    f = Fleet()
    try:
        S, a, b = f.settings, f.node("cam1"), f.node("cam2")
        ma, mb = f.mon("cam1"), f.mon("cam2")
        f.settle()

        # The whole point of the readback: a body that ACCEPTS the write and
        # does not apply it. Blind, that was cached as done and published
        # synced:true while the transect recorded no RAW.
        s0 = f.seq()
        a.label_override = {"filetypeValue": 1}       # body says JPEG-only
        f.tick()
        # Two passes: the alarm deliberately requires two consecutive failures
        # so one poll cannot race a setting the body is still applying.
        for _ in range(3):
            S.reconcile_all(force=False)
            f.tick()
        check("a body that silently declines filetype is reported DIVERGENT",
              "filetype" in (ma.convergence.get("diverged") or []),
              json.dumps(ma.convergence))
        check("and is not published as synced",
              ma.convergence.get("synced") is False,
              json.dumps(ma.convergence))
        check("the divergence reaches the journal",
              any("filetype" in str(e) for e in
                  f.evs(s0, kind="settings_divergent", node="cam1")),
              str([e["msg"] for e in f.evs(s0, kind="settings_divergent")]))
        a.label_override = {}
        f.tick()
        S.reconcile_all(force=True)
        f.tick()
        S.reconcile_all(force=False)
        f.tick()
        check("it clears once the body reports the pushed value",
              ma.convergence.get("synced") is True, json.dumps(ma.convergence))

        # expcompValue gets the same treatment.
        a.label_override = {"expcompValue": 1200}
        f.tick()
        S.update({"expcomp": 0})
        f.tick()
        S.reconcile_all(force=True, exposure=True)
        f.tick()
        check("an expcomp the body does not apply is reported, not cached",
              "expcomp" in (ma.convergence.get("diverged") or []),
              json.dumps(ma.convergence))
        a.label_override = {}
        f.tick()

        # An ilxctl build that predates the keys: converge BLIND, do not skip
        # the field (the body still has to shoot the survey's format) and do
        # not write on every idle pass.
        with b._lock:
            b.hide_keys = {"filetypeValue", "imagesizeValue", "transsizeValue"}
        b.drift(filetype=1, imagesize=1, transsize=0)
        f.tick()
        S.reconcile_all(force=False)
        f.tick()
        S.reconcile_all(force=False)
        f.tick()
        check("a pre-C2 ilxctl build still converges the format fields blind",
              b.raw("filetype") == S.get()["filetype"]
              and b.raw("imagesize") == S.get()["imagesize"]
              and b.raw("transsize") == S.get()["transsize"],
              "filetype=%s imagesize=%s transsize=%s"
              % (b.raw("filetype"), b.raw("imagesize"), b.raw("transsize")))
        check("and is not flagged divergent for the keys it cannot report",
              not ({"filetype", "imagesize", "transsize"}
                   & set(mb.convergence.get("diverged") or [])),
              json.dumps(mb.convergence))
        b.clear_counts()
        for _ in range(3):
            S.reconcile_all(force=False)
            f.tick()
        blind = b.pushed("filetype", "imagesize", "transsize", "rawtype",
                         "expcomp")
        check("a converged blind build is left alone (no writes when in sync)",
              len(blind) == 0,
              "%d writes in 3 idle reconciles: %s"
              % (len(blind), sorted({p[1] for p in blind})))
        with b._lock:
            b.hide_keys = set()
        f.tick()

        # rawtype has no readback on ANY build the fleet runs today: it must
        # take the blind path rather than diverging against a missing key.
        a.clear_counts()
        for _ in range(3):
            S.reconcile_all(force=False)
            f.tick()
        raws = a.pushed("rawtype")
        check("rawtype (no readback key anywhere) does not thrash or diverge",
              len(raws) == 0
              and "rawtype" not in (ma.convergence.get("diverged") or []),
              "%d rawtype writes; convergence=%s"
              % (len(raws), json.dumps(ma.convergence)))
        check("rawtype still reached the body",
              a.raw("rawtype") == S.get()["rawtype"], str(a.raw("rawtype")))

        # quality/pcsave are OPTIONAL: unmanaged until the operator pins one.
        check("quality/pcsave are unmanaged by default",
              "quality" not in S.get() and "pcsave" not in S.get(),
              json.dumps(sorted(S.get())))
        rep = S.update({"quality": 2})
        check("but they can be pinned into the desired vector",
              rep["applied"].get("quality") == 2 and S.get()["quality"] == 2,
              json.dumps(rep))
        f.tick()
        S.reconcile_all(force=True)
        f.tick()
        check("a pinned optional field is pushed to the fleet",
              a.raw("quality") == 2 and b.raw("quality") == 2,
              "cam1=%s cam2=%s" % (a.raw("quality"), b.raw("quality")))
    finally:
        f.close()


# ===========================================================================
# K4 — the rig is ALWAYS manual focus
# ===========================================================================
def k4_focus_mode_policy(opts):
    sect("K4 focus_mode: MF (1) is the only value the fleet will accept")
    f = Fleet()
    try:
        S, a, b = f.settings, f.node("cam1"), f.node("cam2")
        f.settle()
        before = (a.raw("focus_mode"), b.raw("focus_mode"))
        for bad in (2, 3, 4, 0):
            rep = S.update({"focus_mode": bad})
            ok = ("focus_mode" in rep["rejected"]
                  and "focus_mode" not in rep["applied"])
            check("focus_mode %d is refused with the operator rule named" % bad,
                  ok and "manual focus" in rep["rejected"].get("focus_mode", ""),
                  json.dumps(rep))
        f.tick()
        S.reconcile_all(force=True)
        f.tick()
        check("no body was left in AF by a refused apply",
              a.raw("focus_mode") == 1 and b.raw("focus_mode") == 1,
              "cam1=%s cam2=%s (was %s)"
              % (a.raw("focus_mode"), b.raw("focus_mode"), str(before)))
        check("desired still holds MF", S.get()["focus_mode"] == 1,
              str(S.get()["focus_mode"]))
        check("MF itself is still accepted",
              S.update({"focus_mode": 1})["applied"].get("focus_mode") == 1)
        # The preview path shares _validate_field, and focus is not previewable
        # at all (focus/zoom are per-camera, never fleet-applied).
        rep = S.preview({"focus_mode": 2}, node="cam1")
        check("focus_mode cannot be smuggled in through the preview path",
              "focus_mode" in (rep.get("rejected") or {}), json.dumps(rep))
    finally:
        f.close()

    # A desired.json hand-edited (or written by an older build) to AF must be
    # corrected on load, loudly, not merged verbatim and pushed to both bodies.
    f = Fleet(desired_json=json.dumps({"iso": 800, "focus_mode": 2}))
    try:
        S = f.settings
        check("a desired.json with focus_mode != 1 is corrected to MF on load",
              S.get()["focus_mode"] == 1, str(S.get()["focus_mode"]))
        check("the correction is journalled as a warning",
              any("focus_mode" in str(e) for e in f.evs(0, kind="settings")),
              str([e["msg"][:90] for e in f.evs(0)]))
        check("the rest of the saved vector still loads",
              S.get()["iso"] == 800, str(S.get()["iso"]))
    finally:
        f.close()


# ===========================================================================
# K5 — a body a card drain owns is not written to
# ===========================================================================
def k5_suspend_control(opts):
    sect("K5 reconcile_all skips a node a card drain holds in transfer mode")
    f = Fleet()
    try:
        S, a, b = f.settings, f.node("cam1"), f.node("cam2")
        ma, mb = f.mon("cam1"), f.mon("cam2")
        f.settle()

        mb.suspend_control = True
        b.drift(store_dest=2, drive=2, filetype=1)
        a.drift(store_dest=2)
        f.tick()
        t0 = time.time()
        b.clear_counts()
        a.clear_counts()
        S.reconcile_all(force=True)
        S.reconcile_all(force=False)
        check("no settings call is made into a draining node",
              _ilx_calls(b, t0) == 0,
              "%d ilxctl calls while the drain owns it" % _ilx_calls(b, t0))
        check("nothing on the drained body was moved",
              b.raw("store_dest") == 2 and b.raw("drive") == 2
              and b.raw("filetype") == 1,
              "store=%s drive=%s filetype=%s"
              % (b.raw("store_dest"), b.raw("drive"), b.raw("filetype")))
        check("its neighbour still converges normally",
              a.raw("store_dest") == S.get()["store_dest"],
              "cam1 store=%s" % a.raw("store_dest"))

        # A body left in transfer mode with NO drain flag (a crashed drain, a
        # rigd restart mid-drain) must be skipped too.
        mb.suspend_control = False
        b.label_override = {"controlMode": "transfer"}
        f.tick()
        with mb._lock:
            mb.state = mb.CONNECTED      # a crashed drain's exact shape
        t0 = time.time()
        b.clear_counts()
        S.reconcile_all(force=True)
        check("a body still in transfer mode is skipped even with no drain flag",
              _ilx_calls(b, t0) == 0,
              "%d settings calls into a transfer-mode session"
              % _ilx_calls(b, t0))
        b.label_override = {}
        f.tick()

        # Releasing the drain drops the blind-field cache (the SDK session was
        # torn down and rebuilt) but must NOT arm a forced exposure pass.
        # First put the body back on the fleet vector, so the only thing that
        # differs across the drain is the operator's deliberate split.
        for _ in range(2):
            S.reconcile_all(force=True)
            f.tick()
        S.reconcile_all(force=False)
        f.tick()
        mb.suspend_control = True
        f.tick()
        b.drift(iso=3200)               # balanced against unequal strobes
        mb.suspend_control = False
        f.tick()
        b.clear_counts()
        S.reconcile_all(force=False)
        f.tick()
        check("releasing a drain does not wipe a per-camera exposure split",
              b.raw("iso") == 3200, "iso=%s" % b.raw("iso"))
        check("but the blind-field cache IS dropped and re-verified after it",
              len(b.pushed("rawtype")) >= 1,
              "%d blind re-pushes after the drain released the body"
              % len(b.pushed("rawtype")))
        check("and the body still holds the fleet's non-exposure vector",
              b.raw("store_dest") == S.get()["store_dest"]
              and b.raw("filetype") == S.get()["filetype"],
              "store=%s filetype=%s"
              % (b.raw("store_dest"), b.raw("filetype")))
    finally:
        f.close()


# ===========================================================================
# K6 — EventLog: path bound at construction, and rotation really happens
# ===========================================================================
def k6_eventlog(opts):
    sect("K6 EventLog resolves RIGD_LOG at construction and rotates")
    saved = {k: getattr(rigcore, k) for k in _GLOBALS}
    d = tempfile.mkdtemp(prefix="wildsync-audit-evlog-")
    try:
        rigcore.RIG_HOME = os.path.join(d, "rig")
        rigcore.RIGD_LOG = os.path.join(rigcore.RIG_HOME, "rigd.jsonl")
        os.makedirs(rigcore.RIG_HOME, exist_ok=True)
        ev, err = _call(rigcore.EventLog)
        if ev is None:
            check("EventLog() constructs against a rebound RIGD_LOG", False,
                  err or "")
            return
        # Check the path BEFORE emitting: on pre-fix code this points at the
        # operator's real ~/rig/rigd.jsonl and this suite must not write there.
        bound = os.path.realpath(ev.path) == os.path.realpath(rigcore.RIGD_LOG)
        check("EventLog() honours a rebound rigcore.RIGD_LOG (devrig's case)",
              bound,
              "wrote to %s instead of %s" % (ev.path, rigcore.RIGD_LOG))
        if bound:
            ev.emit("info", "audit", "hello from the fake fleet")
            with open(rigcore.RIGD_LOG) as fh:
                body = fh.read()
            check("the event landed in the temp journal",
                  "hello from the fake fleet" in body, body[-120:])

        # Rotation. PROTOCOL.md calls the journal "rolling"; the soak note says
        # it has no cap. It does — MAX_BYTES with one kept generation.
        path = os.path.join(d, "roll.jsonl")
        ev2 = rigcore.EventLog(path=path, max_bytes=1500)
        for i in range(60):
            ev2.emit("info", "audit", "filler event %03d - padding the line "
                                      "out so the cap is reached quickly" % i)
        live = os.path.getsize(path) if os.path.exists(path) else -1
        check("the journal rotates at MAX_BYTES instead of growing forever",
              os.path.exists(path + ".1") and 0 <= live <= 1500 + 400,
              "live=%d bytes, .1 exists=%s"
              % (live, os.path.exists(path + ".1")))
        check("exactly one previous generation is kept",
              not os.path.exists(path + ".2"))
        check("the in-memory ring still has every event for the API",
              len(ev2.since(0, limit=100000)["events"]) == 60,
              str(len(ev2.since(0, limit=100000)["events"])))
        check("MAX_BYTES is a real cap, not a placeholder",
              isinstance(rigcore.EventLog.MAX_BYTES, int)
              and rigcore.EventLog.MAX_BYTES > 0,
              str(rigcore.EventLog.MAX_BYTES))
    finally:
        shutil.rmtree(d, ignore_errors=True)
        for k, v in saved.items():
            setattr(rigcore, k, v)


# ===========================================================================
# K7 — run ids the browser and startup recovery can actually read
# ===========================================================================
def k7_run_ids(opts):
    sect("K7 a non-ASCII label still produces a browsable, recoverable run id")
    slug = getattr(rigcore, "run_id_slug", None)
    if slug is None:
        check("rigcore exposes the single run-id sanitiser (run_id_slug)",
              False, "missing")
    else:
        cases = ["Récif-Nord", "サンゴ礁", "Bahía Isla", "reef 3/north",
                 "  ", "", None, "a" * 90, "Ræv-Ø_2"]
        bad = [c for c in cases
               if not rigcore._RUN_ID_RE.match(
                   time.strftime("%y%m%d_%H%M") + "_" + slug(c))]
        check("every label produces an id the browser's regex accepts",
              not bad, "rejected: %r" % bad)
        check("the accent is folded, not deleted",
              slug("Récif-Nord") == "Recif-Nord", slug("Récif-Nord"))
        check("a label with nothing ASCII in it still yields a usable id",
              slug("サンゴ礁") == "run", slug("サンゴ礁"))
        check("separators can never survive the slug",
              "/" not in slug("a/b") and "\\" not in slug("a\\b")
              and ".." not in slug("a..b"), slug("a/b"))
        # The writer has to agree with the readers, or the reader-side widening
        # above is the only thing keeping such a transect openable. run.py's
        # _slug had its own copy of this logic and kept non-ASCII alnum
        # characters (str.isalnum() is True for 'é'); it now delegates here.
        import run as runmod
        check("run.py's _slug is this sanitiser, not a second copy",
              runmod._slug("Récif-Nord") == slug("Récif-Nord")
              and runmod._slug("サンゴ礁") == slug("サンゴ礁"),
              "run._slug(%r) = %r" % ("Récif-Nord", runmod._slug("Récif-Nord")))

    # And what is already on the card: an accented run directory must be
    # listable and openable, or rigd's startup recovery (which iterates
    # list_runs) leaves it final:false with stale stats forever.
    f = Fleet(make_settings=False)
    try:
        _write_run(os.path.join(rigcore.RUNS_DIR, "260823_1642_Récif-Nord"),
                   cams=("cam1", "cam2"), shots=2)
        rid = os.listdir(rigcore.RUNS_DIR)[0]     # whatever the fs stored
        br = rigcore.RunBrowser(f.events)
        listed = [r["run_id"] for r in br.list_runs()["runs"]]
        check("an accented run directory is listed by the browser",
              rid in listed, "on disk %r, listed %r" % (rid, listed))
        d, err = _call(br.detail, rid)
        check("and it can be opened (startup recovery can finalise it)",
              d is not None and (d or {}).get("run_id") == rid, err or "")

        # The path guard must still be a guard.
        for evil in ("..", ".", "../etc", "a/b", ".hidden", "", "x" * 200,
                     "ok\n", "a\\b", '"x'):
            _, err = _call(br.run_dir, evil)
            check("the path guard still refuses %r" % evil, err is not None,
                  (err or "ACCEPTED IT").split(":")[0])
    finally:
        f.close()


# ===========================================================================
# K8 — desired.json is validated, and a bad one is kept, not overwritten
# ===========================================================================
def k8_desired_json(opts):
    sect("K8 desired.json: validate on load, never destroy an unreadable one")
    D = rigcore.DEFAULT_DESIRED
    check("the built-in fallback is the survey vector FIELD-RUN documents",
          D.get("filetype") == 3 and D.get("imagesize") == 3
          and D.get("transsize") == 1 and D.get("rawtype") == 5
          and D.get("wb_mode") == 256 and D.get("colortemp") == 5600
          and D.get("focus_mode") == 1 and D.get("store_dest") == 3,
          json.dumps({k: D.get(k) for k in
                      ("filetype", "imagesize", "transsize", "rawtype",
                       "wb_mode", "colortemp", "focus_mode", "store_dest")}))
    check("the fallback records RAW (a lost desired.json must not turn it off)",
          D.get("filetype") in (2, 3, 4), str(D.get("filetype")))

    # Case A — values the POST path would refuse, merged verbatim before.
    junk = json.dumps({"iso": 0, "aperture": "1100", "drive": 0,
                       "focus_mode": 2, "expcomp": 99000, "shutter": 65736,
                       "store_dest": 3, "bogus": 7})
    f = Fleet(desired_json=junk, make_settings=False)
    try:
        S = rigcore.SettingsManager(f.monitors, f.events)
        g = S.get()
        check("an illegal ISO is dropped, not pushed to both bodies every 3 s",
              g["iso"] == rigcore.DEFAULT_DESIRED["iso"], str(g["iso"]))
        check("a string value is coerced to the int a readback can equal",
              g["aperture"] == 1100 and isinstance(g["aperture"], int),
              repr(g["aperture"]))
        check("an out-of-range drive is dropped", g["drive"] ==
              rigcore.DEFAULT_DESIRED["drive"], str(g["drive"]))
        check("an out-of-range expcomp is dropped", abs(g["expcomp"]) <= 5000,
              str(g["expcomp"]))
        check("focus_mode is corrected to MF", g["focus_mode"] == 1,
              str(g["focus_mode"]))
        check("an unknown key never enters the desired vector",
              "bogus" not in g, json.dumps(sorted(g)))
        check("the values that WERE legal are kept",
              g["shutter"] == 65736 and g["store_dest"] == 3,
              "shutter=%s store=%s" % (g["shutter"], g["store_dest"]))
        check("the refusals are journalled for the operator",
              any(e["kind"] == "settings" and "refused" in e["msg"]
                  for e in f.evs(0)),
              str([e["msg"][:80] for e in f.evs(0)]))
    finally:
        f.close()

    # Case B — a file that will not parse at all (the classic trailing comma).
    broken = '{"iso": 1600, "aperture": 1100,}'
    f = Fleet(desired_json=broken, make_settings=False)
    try:
        s0 = f.seq()
        S = rigcore.SettingsManager(f.monitors, f.events)
        home = rigcore.RIG_HOME
        kept = [n for n in os.listdir(home)
                if n.startswith("desired.json.bad")]
        check("the unreadable file is kept aside as desired.json.bad-<ts>",
              len(kept) == 1, str(sorted(os.listdir(home))))
        if kept:
            with open(os.path.join(home, kept[0])) as fh:
                check("the operator's original bytes are intact in the .bad copy",
                      fh.read() == broken)
        check("defaults are NOT written over the operator's settings file",
              not os.path.exists(rigcore.DESIRED_PATH),
              "desired.json present=%s" % os.path.exists(rigcore.DESIRED_PATH))
        check("the failure is raised as an error, not a warning",
              any(e["sev"] == "error" and e["kind"] == "settings"
                  for e in f.evs(s0)),
              str([(e["sev"], e["msg"][:70]) for e in f.evs(s0)]))
        check("the fleet runs on the built-in defaults in the meantime",
              S.get()["filetype"] == rigcore.DEFAULT_DESIRED["filetype"],
              str(S.get()["filetype"]))
        # An explicit save by the operator is what resolves it.
        S.update({"iso": 800})
        check("an operator save writes a fresh desired.json",
              os.path.exists(rigcore.DESIRED_PATH))
        S2 = rigcore.SettingsManager(f.monitors, f.events)
        check("and it reloads cleanly next start", S2.get()["iso"] == 800,
              str(S2.get()["iso"]))
    finally:
        f.close()

    # A file that is valid JSON but not an object.
    f = Fleet(desired_json="[1, 2, 3]", make_settings=False)
    try:
        S = rigcore.SettingsManager(f.monitors, f.events)
        check("a JSON array where an object belongs is handled like junk",
              S.get()["iso"] == rigcore.DEFAULT_DESIRED["iso"]
              and any(n.startswith("desired.json.bad")
                      for n in os.listdir(rigcore.RIG_HOME)),
              str(sorted(os.listdir(rigcore.RIG_HOME))))
    finally:
        f.close()


# ===========================================================================
# K9 — bump_ev is bounded
# ===========================================================================
def k9_bump_ev(opts):
    sect("K9 bump_ev goes through validation (+-5000 mEV)")
    f = Fleet()
    try:
        S = f.settings
        f.settle()
        S.update({"expcomp": 0})
        for _ in range(20):
            S.bump_ev(1)
        hi = rigcore.SETTING_BOUNDS["expcomp"][1]
        check("+1/3 EV presses clamp at the legal ceiling",
              0 < S.get()["expcomp"] <= hi, str(S.get()["expcomp"]))
        for _ in range(40):
            S.bump_ev(-1)
        lo = rigcore.SETTING_BOUNDS["expcomp"][0]
        check("and at the legal floor", lo <= S.get()["expcomp"] < 0,
              str(S.get()["expcomp"]))
        S.update({"expcomp": 0})
        S.bump_ev(1000)
        check("one huge steps value cannot poison desired.json",
              abs(S.get()["expcomp"]) <= hi, str(S.get()["expcomp"]))
        with open(rigcore.DESIRED_PATH) as fh:
            saved = json.load(fh)
        check("the persisted value is inside the bound too",
              abs(saved.get("expcomp", 0)) <= hi, str(saved.get("expcomp")))
        S.bump_ev("nonsense")
        check("a non-numeric steps value is a no-op, not a crash",
              abs(S.get()["expcomp"]) <= hi, str(S.get()["expcomp"]))
    finally:
        f.close()


# ===========================================================================
# K10 — RunBrowser: index.jsonl, a broken run.json, a one-camera shot
# ===========================================================================
def _write_run(root, cams=("cam1", "cam2"), shots=2, lone_shot=False,
               run_json="full", index_jsonl=True, truncate_index=True):
    """A run directory the browser can read. Epochs are µs-distinct per camera
    in index.jsonl and centisecond-quantised in flight_log.csv, exactly like
    run.py writes them."""
    os.makedirs(root, exist_ok=True)
    base = 1787503200.0
    index = []
    for cam_i, cam in enumerate(cams):
        d = os.path.join(root, cam)
        os.makedirs(d, exist_ok=True)
        rows = ["datetime,filename,capture_source"]
        for i in range(shots):
            ep = base + i * 2.0 + cam_i * 0.0006
            name = "%s_%d.jpg" % (cam.capitalize(), i)
            rows.append("%s,%s,gpio_edge"
                        % (time.strftime("%y%m%d_%H%M%S",
                                         time.gmtime(ep))
                           + ".%02d" % int((ep % 1) * 100), name))
            index.append({"cam": cam, "file": name, "orig": name,
                          "epoch": round(ep, 6), "src": "gpio_edge"})
        if lone_shot and cam_i == 0:
            ep = base + shots * 2.0
            name = "%s_lone.jpg" % cam.capitalize()
            rows.append("%s,%s,gpio_edge"
                        % (time.strftime("%y%m%d_%H%M%S", time.gmtime(ep))
                           + ".%02d" % int((ep % 1) * 100), name))
            index.append({"cam": cam, "file": name, "orig": name,
                          "epoch": round(ep, 6), "src": "gpio_edge"})
        with open(os.path.join(d, "flight_log.csv"), "w") as fh:
            fh.write("\n".join(rows) + "\n")
    if index_jsonl:
        with open(os.path.join(root, rigcore.INDEX_JSONL), "w") as fh:
            for r in index:
                fh.write(json.dumps(r) + "\n")
            fh.write('{"cam": "cam1", "file": "torn.jpg"')   # torn final line
    if run_json == "full":
        doc = {"label": "audit", "final": True, "frames": len(index),
               "index": index[-1:] if truncate_index else index,
               "config": {"interval_s": 2.0}}
        with open(os.path.join(root, "run.json"), "w") as fh:
            json.dump(doc, fh)
    elif run_json == "corrupt":
        with open(os.path.join(root, "run.json"), "w") as fh:
            fh.write("{not json at all,,,")
    return index


def k10_runbrowser(opts):
    sect("K10 RunBrowser: index.jsonl (C3), a broken run.json, a lone frame")
    f = Fleet(make_settings=False)
    try:
        br = rigcore.RunBrowser(f.events)
        R = rigcore.RUNS_DIR

        # 1. run.json's index is capped at the last 2000 entries; index.jsonl
        #    carries the whole transect. The browser must prefer it.
        rid = "260823_1640_indexed"
        _write_run(os.path.join(R, rid), shots=3, truncate_index=True)
        d, err = _call(br.detail, rid)
        check("detail() reads the run", d is not None, err or "")
        d = d or {}
        check("index.jsonl is preferred over run.json's truncated index",
              d.get("index_source") == "index.jsonl", str(d.get("index_source")))
        shots = (br.shots(rid) or {}).get("shots") or []
        srcs = [s.get("spread_src") for s in shots]
        check("every shot gets its µs spread, not just the last 2000",
              srcs and all(x == "index" for x in srcs), str(srcs))
        check("a torn final line in index.jsonl is skipped, not fatal",
              len(shots) == 3, "%d shots" % len(shots))

        # 2. Same run without index.jsonl: the old behaviour still works.
        rid2 = "260823_1641_nojsonl"
        _write_run(os.path.join(R, rid2), shots=3, index_jsonl=False,
                   truncate_index=True)
        d2 = br.detail(rid2)
        shots2 = (br.shots(rid2) or {}).get("shots") or []
        check("with no index.jsonl the reader falls back to run.json",
              d2.get("index_source") == "run.json", str(d2.get("index_source")))
        check("and the truncated head really is coarser (why C3 exists)",
              [s.get("spread_src") for s in shots2].count("flight_log") >= 1,
              str([s.get("spread_src") for s in shots2]))

        # 3. A shot only one camera recorded is not a pair and has no spread.
        rid3 = "260823_1644_lone"
        _write_run(os.path.join(R, rid3), shots=2, lone_shot=True)
        shots3 = (br.shots(rid3) or {}).get("shots") or []
        lone = [s for s in shots3 if s.get("missing")]
        check("the one-camera shot is reported as missing a camera",
              len(lone) == 1 and lone[0]["missing"] == ["cam2"],
              json.dumps(lone))
        check("and carries NO measured inter-camera spread",
              bool(lone) and lone[0].get("spread_ms") is None
              and lone[0].get("spread_src") == "single",
              "spread_ms=%s spread_src=%s"
              % (lone[0].get("spread_ms") if lone else "?",
                 lone[0].get("spread_src") if lone else "?"))
        pairs = br.detail(rid3)["pairs"]
        check("the run summary counts it as incomplete",
              pairs["incomplete"] == 1 and pairs["complete"] == 2,
              json.dumps({k: pairs[k] for k in ("shots", "complete",
                                                "incomplete")}))

        # 4. A run whose run.json is corrupt, and one with none at all.
        rid4 = "260823_1645_corrupt"
        _write_run(os.path.join(R, rid4), shots=2, run_json="corrupt")
        rid5 = "260823_1646_norunjson"
        _write_run(os.path.join(R, rid5), shots=2, run_json=None)
        listing = {r["run_id"]: r for r in br.list_runs()["runs"]}
        check("a run with a corrupt run.json is still listed",
              rid4 in listing and listing[rid4]["has_run_json"] is False,
              json.dumps(listing.get(rid4)))
        check("a run with no run.json at all is still listed",
              rid5 in listing and listing[rid5]["cams"] == ["cam1", "cam2"],
              json.dumps(listing.get(rid5)))
        for rid_x in (rid4, rid5):
            d_x, err = _call(br.detail, rid_x)
            check("detail() survives %s" % rid_x,
                  d_x is not None and (d_x or {}).get("pairs", {}).get("shots")
                  == 2, err or json.dumps((d_x or {}).get("pairs", {}))[:120])
            check("and recovers the frame count from index.jsonl for %s"
                  % rid_x, (d_x or {}).get("frames_indexed") == 4,
                  str((d_x or {}).get("frames_indexed")))

        # 5. An empty run directory must not raise either.
        rid6 = "260823_1647_empty"
        os.makedirs(os.path.join(R, rid6), exist_ok=True)
        d6, err = _call(br.detail, rid6)
        check("an empty run directory reads as an empty run, not an error",
              d6 is not None and (d6 or {}).get("cams") == [], err or "")
    finally:
        f.close()


# ===========================================================================
# K9b — "already connected" is the SDK's own auto-reconnect, not a dead handle
# ===========================================================================
def _stub_node(status_seq, connect_reply):
    """A rigcore.http_json stand-in: no sockets, no fake node, just the two
    answers the connect branch reads. status_seq is consumed one entry per
    /api/status call so the poll's read and the branch's re-read can differ."""
    seq = list(status_seq)

    def stub(url, body=None, timeout=8):
        if url.endswith("/health"):
            return {"node": "cam1", "uptime_s": 10.0, "host_uptime_s": 900.0,
                    "time": {"epoch": time.time()}}
        if url.endswith("/api/status"):
            return dict(seq.pop(0) if seq else {"connected": False})
        if url.endswith("/api/connect"):
            return dict(connect_reply)
        return {"ok": True}
    return stub


def k9b_already_connected(opts):
    sect("K9 'already connected' is not diagnosed as a dead SDK handle")
    f = Fleet(specs=(("cam1", "127.0.0.11", 1),), make_settings=False)
    real = rigcore.http_json
    try:
        m = f.mon("cam1")
        # ilxctl runs with CrReconnecting_ON: after a USB blip the SDK's own
        # auto-reconnect races our POST and wins. The live journal carried 300
        # "restart ilxctl on the node" warnings (60 s backoff each) for bodies
        # that were perfectly healthy.
        rigcore.http_json = _stub_node(
            [{"connected": False}, {"connected": True}],
            {"ok": False, "error": "already connected"})
        s0 = f.seq()
        m._connect_after = 0.0
        m._tick()
        evs = f.evs(s0, kind="reconnect")
        check("a body that IS claimed on the re-read raises no warning",
              not [e for e in evs if e["sev"] in ("warn", "error")],
              str([(e["sev"], e["msg"][:70]) for e in evs]))
        check("and it is reported as the race it is",
              any(e["sev"] == "info" and "auto-reconnect" in e["msg"]
                  for e in evs),
              str([e["msg"][:70] for e in evs]))
        check("no 60 s backoff is imposed on a healthy body",
              m._backoff <= 5.0, "backoff=%s" % m._backoff)

        # A body that STAYS unclaimed after answering "already connected" is
        # the real dead-handle case and must still say so.
        m2 = rigcore.NodeMonitor({"name": "cam1", "cam_num": 1,
                                  "host": "127.0.0.11"}, f.events, poll=0.25)
        rigcore.http_json = _stub_node(
            [{"connected": False}, {"connected": False}],
            {"ok": False, "error": "already connected"})
        s0 = f.seq()
        m2._connect_after = 0.0
        m2._tick()
        evs = f.evs(s0, kind="reconnect")
        check("a genuinely dead SDK handle still names the fix",
              any(e["sev"] == "warn" and "restart ilxctl" in e["msg"]
                  for e in evs),
              str([(e["sev"], e["msg"][:70]) for e in evs]))
        check("and still backs off to the ceiling", m2._backoff == 60.0,
              "backoff=%s" % m2._backoff)
    finally:
        rigcore.http_json = real
        f.close()


# ===========================================================================
# K11 — a failed shot listing is None, never an empty spool
# ===========================================================================
def k11_shots_none_on_error(opts):
    sect("K11 shots() says 'I could not ask', not 'the spool is empty'")
    f = Fleet(specs=(("cam1", "127.0.0.11", 1),))
    try:
        f.tick()
        m = f.mon("cam1")
        node = f.node("cam1")
        node.add_frame(epoch=time.time(), name="ILXOLD01.JPG")
        node.add_frame(epoch=time.time(), name="ILXOLD02.JPG")
        got = m.shots()
        check("a healthy node lists its spool as a list",
              isinstance(got, list) and len(got) == 2,
              "%r" % (got if not isinstance(got, list) else len(got),))

        # An EMPTY spool is a list too - the honest empty answer must stay
        # distinguishable from the failure below.
        f2 = Fleet(specs=(("cam2", "127.0.0.12", 2),))
        try:
            f2.tick()
            empty = f2.mon("cam2").shots()
            check("an empty spool is an empty LIST, not None",
                  isinstance(empty, list) and empty == [], "%r" % (empty,))
        finally:
            f2.close()

        # ilxctl restarting under systemd, a 500, a hang, a node that has just
        # gone: every one of them means "I could not ask".
        for label, fault in (("HTTP 500", {"http500": True}),
                             ("malformed JSON", {"badjson": True})):
            node.set_fault("ilx", **fault)
            try:
                out = m.shots()
            finally:
                node.clear_faults()
            check("a %s listing is None, not []" % label, out is None,
                  "%r" % (out,))
        node.down()
        try:
            out = m.shots()
        finally:
            node.up()
        check("an unreachable node's listing is None, not []", out is None,
              "%r" % (out,))
        note("[] from a failed listing is what let PullWorker baseline an "
             "empty spool and pull every old frame on the node into the "
             "transect as survey data (audit 2026-08-24)")
    finally:
        f.close()



# ===========================================================================
# K12 — releasing a preview pin must RE-CONVERGE the camera it was pinned on
# ===========================================================================
def k12_preview_release_reconverges(opts):
    sect("K12 a non-exposure apply releases the pin AND pulls that body back")
    f = Fleet()
    try:
        S, c1, c2 = f.settings, f.node("cam1"), f.node("cam2")
        f.settle()
        want_iso = S.get()["iso"]
        want_ap = S.get()["aperture"]
        # cam2 carries a DELIBERATE per-camera split (K2). Nothing done for the
        # pinned body may touch it.
        _split(f, cam="cam2", iso=1600, aperture=560)

        rep = S.preview({"iso": 3200}, node="cam1")
        f.tick()
        check("the preview lands on cam1 and nowhere else",
              c1.raw("iso") == 3200 and c2.raw("iso") == 1600,
              "cam1=%s cam2=%s %s" % (c1.raw("iso"), c2.raw("iso"),
                                      json.dumps(rep)[:120]))

        # The operator now applies something unrelated from the Controls tab.
        # The format panel POSTs only the fields it made dirty, e.g. Kelvin.
        S.update({"colortemp": 5000})
        f.tick()
        check("the pin is dropped", S.pinned_node() is None,
              str(S.preview_state()))
        check("and the camera it was pinned on IS pulled back to the fleet "
              "vector", c1.raw("iso") == want_iso,
              "cam1 iso=%s want=%s (a {'colortemp':5000} apply wrote no "
              "exposure to anyone, so cam1 kept the preview's ISO with the "
              "pin, the badge and the pre-flight discard all gone)"
              % (c1.raw("iso"), want_iso))
        check("while cam2's deliberate split is untouched",
              c2.raw("iso") == 1600 and c2.raw("aperture") == 560,
              "iso=%s aperture=%s" % (c2.raw("iso"), c2.raw("aperture")))
        check("and the field the operator actually applied still landed",
              c1.raw("colortemp") == 5000 and c2.raw("colortemp") == 5000,
              "cam1=%s cam2=%s" % (c1.raw("colortemp"), c2.raw("colortemp")))

        # Same hole through the focus-mode selector (POST /api/focus/mode).
        S.preview({"iso": 3200}, node="cam1")
        f.tick()
        S.update({"focus_mode": 1})
        f.tick()
        check("a focus-mode apply releases the pin and re-converges too",
              S.pinned_node() is None and c1.raw("iso") == want_iso,
              "cam1 iso=%s pinned=%s" % (c1.raw("iso"), S.pinned_node()))
        check("and still leaves cam2's split alone", c2.raw("iso") == 1600,
              "iso=%s" % c2.raw("iso"))

        # A PARTIAL exposure apply covers the named field on every body, but
        # the previewed camera is still sitting on the staged fields it did
        # NOT name.
        S.preview({"iso": 3200, "aperture": 400}, node="cam1")
        f.tick()
        S.update({"iso": 800})
        f.tick()
        check("an exposure apply also re-converges the fields the preview "
              "staged but the apply did not name",
              c1.raw("aperture") == want_ap and c1.raw("iso") == 800,
              "cam1 aperture=%s want=%s iso=%s"
              % (c1.raw("aperture"), want_ap, c1.raw("iso")))

        # Moving the pin to the other body says "cam1 goes back to the fleet
        # vector" in the journal. Nothing used to make that true: exposure is
        # exempt from the continuous reconcile, so the camera that LOST the pin
        # sat on the preview's exposure, unpinned and unbadged, until the next
        # explicit apply.
        S.update({"iso": want_iso})            # both bodies back on desired
        f.tick()
        S.preview({"iso": 3200}, node="cam1")
        f.tick()
        S.preview({"iso": 100}, node="cam2")
        f.tick()
        check("moving the pin puts the camera that lost it back on the fleet "
              "vector", c1.raw("iso") == want_iso and c2.raw("iso") == 100,
              "cam1 iso=%s want=%s cam2 iso=%s"
              % (c1.raw("iso"), want_iso, c2.raw("iso")))
        S.discard()
        f.tick()

        # ...and committing a preview still deploys it to the whole fleet.
        S.preview({"iso": 1600}, node="cam1")
        f.tick()
        out = S.commit()
        f.tick()
        check("commit still deploys the staged values to the fleet",
              S.get()["iso"] == 1600 and c1.raw("iso") == 1600
              and c2.raw("iso") == 1600,
              "desired=%s cam1=%s cam2=%s %s"
              % (S.get()["iso"], c1.raw("iso"), c2.raw("iso"),
                 json.dumps(out)[:120]))
    finally:
        f.close()


# ===========================================================================
# K13 — a status with no property block decides nothing
# ===========================================================================
def _degraded_status(connected=True, **extra):
    """Exactly what camera.cpp's statusJson answers when it cannot take the
    SDK mutex inside its 4.5 s bound: connected + busy + log, and no property
    block at all - no controlMode, no isoValue, no writable map."""
    doc = {"connected": connected, "busy": True, "model": "", "id": "",
           "log": ["USB device found"]}
    doc.update(extra)
    return doc


def _status_stub(status_seq, calls=None):
    """A rigcore.http_json stand-in serving a scripted /api/status. The last
    entry repeats, so a fault can be held for as many ticks as the test needs."""
    seq = list(status_seq)

    def stub(url, body=None, timeout=8):
        if calls is not None:
            calls.append(url)
        if url.endswith("/health"):
            return {"node": "cam1", "uptime_s": 10.0, "host_uptime_s": 900.0,
                    "time": {"epoch": time.time(), "work_ms": 0.4}}
        if url.endswith("/api/status"):
            return dict(seq.pop(0) if len(seq) > 1 else seq[0])
        return {"ok": True}
    return stub


def k13_busy_status_is_not_connected(opts):
    sect("K13 a busy ilxctl reports no property block - that is not 'healthy'")
    f = Fleet(specs=(("cam1", "127.0.0.11", 1),), make_settings=False)
    real = rigcore.http_json
    healthy = {"connected": True, "isoValue": 400, "controlMode": "remote"}
    try:
        # 1. A rigd restart mid-drain leaves the body in RemoteTransfer with no
        #    drain holding it, and an abandoned card index build holds the SDK
        #    mutex past ilxctl's 4.5 s bound. connected:true in that body is
        #    isConnected(), which is TRUE in transfer mode.
        m = f.mon("cam1")
        s0 = f.seq()
        rigcore.http_json = _status_stub([_degraded_status()])
        m._tick()
        check("a body that reported no property block is NOT promoted to "
              "CAM_CONNECTED",
              m.snapshot()["state"] != rigcore.NodeMonitor.CONNECTED,
              "state=%s (connected:true here is isConnected(), which stays "
              "true for a body wedged in transfer mode - promoting off it put "
              "a camera that cannot shoot into the run roster)"
              % m.snapshot()["state"])
        check("and is_connected() agrees", not m.is_connected(),
              "state=%s" % m.snapshot()["state"])
        check("the daemon is still reported as answering, not as OFFLINE",
              m.snapshot()["state"] == rigcore.NodeMonitor.REACHABLE
              and any(e["sev"] == "warn" and "property block" in e["msg"]
                      for e in f.evs(s0)),
              "state=%s evs=%s" % (m.snapshot()["state"],
                                   [(e["sev"], e["msg"][:50])
                                    for e in f.evs(s0)]))

        # 2. If a later ilxctl DOES manage to name the mode in that body, the
        #    stuck-in-transfer recovery must still fire on it.
        m2 = rigcore.NodeMonitor({"name": "cam1", "cam_num": 1,
                                  "host": "127.0.0.11"}, f.events, poll=0.25)
        calls = []
        rigcore.http_json = _status_stub(
            [_degraded_status(controlMode="transfer")], calls=calls)
        m2._connect_after = 0.0
        m2._tick()
        check("a degraded body that still names transfer mode is recovered",
              m2.snapshot()["state"] == rigcore.NodeMonitor.REACHABLE
              and any(u.endswith("/api/card/mode") for u in calls),
              "state=%s calls=%s" % (m2.snapshot()["state"],
                                     [u.rsplit("/", 2)[-2:] for u in calls]))

        # 3. An ordinary SDK stall (a card index build, the ~6 s
        #    SetDeviceProperty stall measured live) must NOT flap a healthy
        #    body out of a live run's roster.
        m3 = rigcore.NodeMonitor({"name": "cam1", "cam_num": 1,
                                  "host": "127.0.0.11"}, f.events, poll=0.25)
        rigcore.http_json = _status_stub([healthy])
        m3._tick()
        check("a healthy body is CAM_CONNECTED", m3.is_connected(),
              "state=%s" % m3.snapshot()["state"])
        rigcore.http_json = _status_stub([_degraded_status()])
        m3._tick()
        m3._tick()
        check("a short busy spell holds the state it already had",
              m3.is_connected(), "state=%s" % m3.snapshot()["state"])

        # 4. ...but a body whose property table stays locked (a stuck card
        #    write reads slotStatus OK forever) must stop being fired at.
        s0 = f.seq()
        hold = getattr(rigcore.NodeMonitor, "BUSY_HOLD_S", None)
        if hold is None or getattr(m3, "_busy_since", None) is None:
            # Pre-fix code tracks no busy streak at all: a failed check, not a
            # suite that dies half way through.
            check("a body busy for longer than BUSY_HOLD_S leaves the fleet",
                  False, "NodeMonitor has no busy streak (_busy_since=%r, "
                         "BUSY_HOLD_S=%r)"
                  % (getattr(m3, "_busy_since", "missing"), hold))
            check("and says so, once, in words that name the cause", False,
                  "not reached")
        else:
            m3._busy_since -= hold + 1.0
            m3._tick()
            check("a body busy for longer than BUSY_HOLD_S leaves the fleet",
                  not m3.is_connected(), "state=%s" % m3.snapshot()["state"])
            check("and says so, once, in words that name the cause",
                  any(e["sev"] == "warn" and "property block" in e["msg"]
                      for e in f.evs(s0)),
                  str([(e["sev"], e["msg"][:60]) for e in f.evs(s0)]))

        # 5. Coming back ends the streak rather than leaving it armed.
        rigcore.http_json = _status_stub([healthy])
        m3._tick()
        check("a real answer restores it and clears the busy streak",
              m3.is_connected()
              and getattr(m3, "_busy_since", "missing") is None,
              "state=%s busy_since=%r"
              % (m3.snapshot()["state"], getattr(m3, "_busy_since", "missing")))
    finally:
        rigcore.http_json = real
        f.close()


def k13b_no_reconcile_into_a_busy_body(opts):
    sect("K13 you cannot converge against a status that reports nothing")
    f = Fleet()
    try:
        S, a, b, mb = f.settings, f.node("cam1"), f.node("cam2"), f.mon("cam2")
        f.settle()
        # cam2's last poll came back degraded. Every managed field then reads
        # back None != target, which used to push the WHOLE vector - possibly
        # into a still-live transfer session, which PROTOCOL.md requires the
        # reconcile to leave alone.
        with mb._lock:
            mb.status = _degraded_status()
            mb.state = rigcore.NodeMonitor.CONNECTED
        a.clear_counts()
        b.clear_counts()
        S.reconcile_all(force=True)
        check("nothing is pushed into a body that reported no property block",
              len(b.pushed()) == 0,
              "pushes=%s" % [p[1:] for p in b.pushed()][:6])
        check("...while the body that DID answer is still converged",
              len(a.pushed()) > 0, "pushes=%d" % len(a.pushed()))
    finally:
        f.close()


# ===========================================================================
# K14 — piagent's handler time is published, not folded into the offset
# ===========================================================================
def k14_clock_work_ms(opts):
    sect("K14 /health work_ms is a diagnostic, not a correction (C1)")
    f = Fleet(specs=(("cam1", "127.0.0.11", 1),), make_settings=False)
    real = rigcore.http_json
    WORK = 40.0          # ms of handler time, stamped by the node itself
    try:
        m = f.mon("cam1")

        def stub(url, body=None, timeout=8):
            if url.endswith("/health"):
                # piagent stamps time.epoch on ENTRY and only then does the
                # work (GPIO, IMU, statvfs, a listdir of the spool).
                t = time.time()
                time.sleep(WORK / 1000.0)
                return {"node": "cam1", "uptime_s": 10.0,
                        "host_uptime_s": 900.0,
                        "time": {"epoch": t, "source": "local",
                                 "work_ms": WORK}}
            if url.endswith("/api/status"):
                return {"connected": True, "isoValue": 400,
                        "controlMode": "remote"}
            return {"ok": True}
        rigcore.http_json = stub
        m._tick()
        info = m.clock_offset_info()
        check("the poll carries the node's own work_ms",
              (m.clock or {}).get("work_ms") == WORK, str(m.clock))
        check("clock_offset_info publishes it next to the best RTT",
              info.get("work_ms") == WORK, json.dumps(info))
        check("and splits out what was actually the LINK, so a slow handler "
              "does not send the operator to the switch port",
              info.get("link_ms") is not None
              and abs(info["link_ms"]
                      - max(0.0, info["rtt_ms_best"] - WORK)) < 0.002
              # Most of that round trip was the handler, not the wire. The
              # bound is loose on purpose: this suite runs under CPU load
              # alongside the others, and loopback scheduling noise lands in
              # link_ms.
              and info["link_ms"] < WORK,
              json.dumps(info))

        # DECISION (audit 2026-08-24): the estimator is NOT corrected with it.
        # The stamp sits at the handler's START, so the midpoint estimate is
        # low by work/2 - but the TCP handshake and piagent's accept + thread
        # spawn sit inside the same bracket ahead of the stamp and bias it the
        # other way by the same order, and only one of the two is measurable.
        # What matters is that the published error bar still BOUNDS the bias:
        # run.py's clock_err is rtt_ms_best/2, and rtt_ms_best keeps the whole
        # measured round trip.
        off = m.clock["offset_s"]
        check("the offset is still the plain midpoint estimate",
              abs(off - (-WORK / 2000.0)) < 0.010,
              "offset=%.4f s (a work/2 correction would read ~0.000)" % off)
        check("and rtt_ms still measures the whole round trip, so rtt/2 "
              "bounds that bias", info["rtt_ms_best"] >= WORK
              and info["rtt_ms_best"] / 2000.0 >= abs(off),
              "rtt_best=%s off=%.4f" % (info["rtt_ms_best"], off))
        note("work_ms is published rather than applied: correcting the one "
             "measurable term of L + A/2 - work/2 does not shrink the sum, "
             "and taking work out of rtt_ms would shrink both time_err_ms "
             "and rigd's node_clock_skew noise floor")
    finally:
        rigcore.http_json = real
        f.close()


SUITE = [
    ("K1 clock offset", k1_clock_offset),
    ("K2 exposure split", k2_exposure_split),
    ("K2 revert tell", k2_sentinel_revert),
    ("K3 format readback", k3_format_readback),
    ("K4 focus policy", k4_focus_mode_policy),
    ("K5 suspend_control", k5_suspend_control),
    ("K6 event log", k6_eventlog),
    ("K7 run ids", k7_run_ids),
    ("K8 desired.json", k8_desired_json),
    ("K9 bump_ev", k9_bump_ev),
    ("K9 already-connected", k9b_already_connected),
    ("K10 run browser", k10_runbrowser),
    ("K11 shots() failure", k11_shots_none_on_error),
    ("K12 preview release", k12_preview_release_reconverges),
    ("K13 busy status", k13_busy_status_is_not_connected),
    ("K13 busy reconcile", k13b_no_reconcile_into_a_busy_body),
    ("K14 clock work_ms", k14_clock_work_ms),
]


def suite(opts=None):
    """Entry point the soaktest runner registers (contract C5)."""
    for name, fn in SUITE:
        try:
            fn(opts)
        except Exception as e:                                # noqa: BLE001
            check("%s ran to completion" % name, False,
                  "%s: %s" % (type(e).__name__, e))


if __name__ == "__main__":
    import soaktest
    t0 = time.time()
    suite(None)
    print("\n%d passed, %d failed in %d s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - t0))
    if soaktest.FAIL:
        print("FAILED: " + "; ".join(soaktest.FAIL))
    sys.exit(1 if soaktest.FAIL else 0)

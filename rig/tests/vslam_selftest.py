#!/usr/bin/env python3
"""vslam_selftest — exercise rig/vslam.py with no hardware and no network.

What it checks:

  * the cv2/numpy-missing degradation path (via monkeypatched import): the
    engine must return a clean "unavailable: <reason>" status, never crash —
    a fresh host has no OpenCV and the map overlay must simply say so
  * synthetic tracking: a textured plane viewed by a virtual camera that
    translates sideways (window crops of one big texture — exactly a
    fronto-parallel plane under pure translation); the engine must track
    >80% of frames into keyframes with a direction-consistent, x-dominant
    trajectory
  * GPS scale: the same sequence with synthetic flight_log UTM must come out
    metric (path length ~ GPS length), and without GPS must say scale=unit
  * corrupt/truncated frames are skipped and counted, tracking continues
  * VslamRunner end to end against a temp run dir that GROWS while it runs:
    ordering, the half-written-file settle, duplicate names, an out-of-order
    straggler, a corrupt file, and a JSON-safe capped snapshot
  * STEREO: two virtual cameras with a known 0.5 m baseline over the same
    scene — the self-estimated extrinsics must recover the baseline
    direction, the PnP chain must come out METRIC within 10%, the GPS
    cross-check ratio must sit near 1.0, a single-plane (mono-degenerate)
    scene must still track, and missing cam2 halves must fall back cleanly
  * auto mode: a cam1+cam2 dir decides stereo, a cam1-only dir decides mono
  * the JSON/PLY writers

If cv2 or numpy is absent the image-pipeline sections print SKIPPED and the
selftest still PASSES on the degradation checks alone — that is the honest
result for that host, not a failure.

Usage:  python3 rig/tests/vslam_selftest.py
Exit code is nonzero if any check fails.
"""
import builtins
import json
import math
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)

import vslam  # noqa: E402 - module under test; must import WITHOUT cv2/numpy

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("  %s %s%s" % ("PASS" if cond else "FAIL", name,
                         (" — " + detail) if detail else ""))
    return cond


def sect(t):
    print("\n== %s" % t)


def have_cv():
    try:
        import numpy  # noqa: F401
        import cv2    # noqa: F401
        return True
    except ImportError:
        return False


# ---------------------------------------------------------------------------

def test_degradation():
    sect("cv2/numpy-missing degradation (monkeypatched import)")
    real_import = builtins.__import__

    def fake(name, *a, **kw):
        if name in ("cv2", "numpy"):
            raise ImportError("simulated absence of %s" % name)
        return real_import(name, *a, **kw)

    builtins.__import__ = fake
    try:
        # a FRESH backend so no cached cv2 sneaks past the patch — the cache
        # is per-instance by design, exactly so this path stays testable.
        be = vslam.CpuOrbBackend()
        err = be.ensure()
        check("ensure() reports reason", bool(err) and "install" in err,
              str(err))
        eng = vslam.VslamEngine(backend=be)
        up = eng.feed(b"\xff\xd8not a real jpeg")
        check("feed() returns None, no crash", up is None)
        check("status is unavailable:*",
              eng.status.startswith("unavailable:"), eng.status)
        check("error cached, not re-raised", be.ensure() == err)
    finally:
        builtins.__import__ = real_import


def test_pure_math():
    sect("pure-python pose math (no cv2 needed)")
    q = vslam._quat_from_r([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    check("identity quaternion", abs(q[0] - 1.0) < 1e-9 and
          all(abs(v) < 1e-9 for v in q[1:]), str(q))
    a = math.radians(90)
    Rz = [[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0],
          [0, 0, 1]]
    q = vslam._quat_from_r(Rz)
    check("90deg z quaternion", abs(q[0] - math.sqrt(0.5)) < 1e-9 and
          abs(q[3] - math.sqrt(0.5)) < 1e-9, str(q))
    ep = vslam._fname_epoch("Cam1_20260827_024128.48.jpg")
    # 1e-3, not 1e-9: a double at ~1.8e9 s magnitude resolves to ~2.4e-7 s,
    # so .48 is not exactly representable after the add.
    check("filename epoch parses", ep is not None and abs(ep % 1 - 0.48) < 1e-3,
          str(ep))
    check("collision suffix _1 parses",
          vslam._fname_epoch("Cam1_20260827_024128.48_1.jpg") == ep)
    check("foreign name rejected", vslam._fname_epoch("IMG_1234.jpg") is None)
    # similarity fit: known transform (rot 90, scale 2, shift) recovered
    src = [(0, 0), (1, 0), (0, 1), (2, 2)]
    dst = [(-2 * y + 10, 2 * x + 5) for x, y in src]
    f = vslam._fit_similarity_2d(src, dst)
    check("similarity fit recovers a,b",
          f is not None and abs(f["a_re"]) < 1e-9 and abs(f["a_im"] - 2) < 1e-9
          and abs(f["tx"] - 10) < 1e-9 and abs(f["ty"] - 5) < 1e-9
          and f["rms_m"] < 1e-9, str(f))
    check("gps meta parse tolerates blanks",
          vslam._meta_gps({"xutm": "", "yutm": "4.0"}) is None and
          vslam._meta_gps({"xutm": "1.5", "yutm": "2.5"}) == (1.5, 2.5))


# ---------------------------------------------------------------------------
# synthetic imagery: TWO textured planes (background at depth Z, sparse
# foreground blobs at Z/2) viewed by a virtual camera translating sideways —
# sliding window crops with the near layer shifting at twice the pixel rate.
# The depth structure matters: a single fronto-parallel plane under uniform
# shift is homography-degenerate and rotation-ambiguous for the 5-point
# solver, so every estimator sits on a knife edge there (that exact planar
# case is pinned separately in vslam.py's MAGSAC comment). Blobs, not raw
# noise: distinct local structure gives ORB unambiguous matches at JPEG q95.

def _layers():
    import numpy as np
    import cv2
    rng = np.random.default_rng(7)
    bg = np.tile(np.linspace(60, 190, 1400, dtype=np.uint8), (900, 1))
    for _ in range(450):
        c = (int(rng.integers(0, 1400)), int(rng.integers(0, 900)))
        cv2.circle(bg, c, int(rng.integers(3, 14)),
                   int(rng.integers(0, 255)), -1)
    fg = np.zeros_like(bg)
    fgm = np.zeros_like(bg)
    for _ in range(160):
        c = (int(rng.integers(0, 1400)), int(rng.integers(0, 900)))
        r = int(rng.integers(4, 12))
        cv2.circle(fg, c, r, int(rng.integers(0, 255)), -1)
        cv2.circle(fgm, c, r, 255, -1)
    return (cv2.GaussianBlur(bg, (3, 3), 0), cv2.GaussianBlur(fg, (3, 3), 0),
            fgm)


def _frames(n=15, shift=12, w=640, h=480):
    import cv2
    bg, fg, fgm = _layers()
    out = []
    for i in range(n):
        x0 = 40 + i * shift             # far plane: f*t/Z px per frame
        fx0 = 40 + i * 2 * shift        # near plane at Z/2: twice that
        crop = bg[200:200 + h, x0:x0 + w].copy()
        m = fgm[200:200 + h, fx0:fx0 + w] > 0
        crop[m] = fg[200:200 + h, fx0:fx0 + w][m]
        ok, buf = cv2.imencode(".jpg", crop,
                               [cv2.IMWRITE_JPEG_QUALITY, 95])
        assert ok
        out.append(bytes(buf))
    return out


def _mk_engine():
    # proc_scale 1 / preprocess none: the synthetic geometry is exact and the
    # test asserts on it; the turbid-water flatten is exercised implicitly by
    # the CLI against real runs.
    return vslam.VslamEngine(
        intrinsics=vslam.Intrinsics(width=640, height=480, focal_px=500.0),
        backend=vslam.make_backend("cpu", proc_scale=1.0, preprocess="none"))


def _engine_kw():
    return dict(intrinsics=vslam.Intrinsics(width=640, height=480,
                                            focal_px=500.0),
                backend=vslam.make_backend("cpu", proc_scale=1.0,
                                           preprocess="none"))


def _mk_stereo(calib_pairs=3, baseline_m=0.5):
    return vslam.StereoVslamEngine(baseline_m=baseline_m,
                                   calib_pairs=calib_pairs, **_engine_kw())


# stereo geometry (all derived from fx=500 and the far plane at Z=25 m):
#   1 px of far-plane shift = Z/fx = 0.05 m of camera motion
#   step_px=12  -> the camera advances 0.60 m per frame
#   base_px=10  -> cam2 sits 0.50 m to the right of cam1 (disparity fx*b/Z)
#   the near plane at Z/2 moves/disparates at exactly twice the far rate
STEP_M = 0.6
BASE_M = 0.5


def _stereo_frames(n=12, step_px=12, base_px=10, w=640, h=480, layers=True):
    import cv2
    bg, fg, fgm = _layers()
    f1s, f2s = [], []
    for i in range(n):
        for lst, off in ((f1s, 0), (f2s, base_px)):
            x0 = 40 + i * step_px + off
            fx0 = 40 + i * 2 * step_px + 2 * off
            crop = bg[200:200 + h, x0:x0 + w].copy()
            if layers:
                m = fgm[200:200 + h, fx0:fx0 + w] > 0
                crop[m] = fg[200:200 + h, fx0:fx0 + w][m]
            ok, buf = cv2.imencode(".jpg", crop,
                                   [cv2.IMWRITE_JPEG_QUALITY, 95])
            assert ok
            lst.append(bytes(buf))
    return f1s, f2s


def test_tracking():
    sect("synthetic tracking, no GPS")
    frames = _frames()
    eng = _mk_engine()
    ups = []
    for i, f in enumerate(frames):
        u = eng.feed(f, t=float(i), name="synth_%02d.jpg" % i)
        if u is not None:
            ups.append(u)
    n = len(frames)
    check(">80%% of %d frames became keyframes" % n,
          eng.stats["keyframes"] >= 0.8 * n,
          "%d keyframes, stats=%s" % (eng.stats["keyframes"], eng.stats))
    pos = [(u.x, u.y, u.z) for u in ups]
    steps = [tuple(b[k] - a[k] for k in range(3))
             for a, b in zip(pos, pos[1:])]
    mean = [sum(s[k] for s in steps) / len(steps) for k in range(3)]
    mlen = math.sqrt(sum(v * v for v in mean)) or 1.0
    dots = [sum(s[k] * mean[k] for k in range(3)) /
            ((math.sqrt(sum(v * v for v in s)) or 1e-12) * mlen)
            for s in steps]
    check("direction-consistent (all steps along mean)",
          min(dots) > 0.9, "min cos %.3f" % min(dots))
    check("x-dominant motion (sideways pan)",
          abs(mean[0]) > 3 * abs(mean[1]) and abs(mean[0]) > 3 * abs(mean[2]),
          "mean step (%.3f %.3f %.3f)" % tuple(mean))
    check("unit scale declared without GPS", eng.scale_source == "unit"
          and all(u.scale_source == "unit" for u in ups))
    check("status reports scale", "scale=unit" in eng.status, eng.status)
    check("landmarks triangulated", len(eng.landmarks) > 50,
          "%d landmarks" % len(eng.landmarks))
    return frames


def test_gps_scale(frames):
    sect("GPS scale fit")
    eng = _mk_engine()
    step_m = 0.4
    ups = []
    for i, f in enumerate(frames):
        meta = {"xutm": "%.2f" % (500000.0 + i * step_m), "yutm": "4588707.83"}
        u = eng.feed(f, meta=meta, t=float(i), name="synth_%02d.jpg" % i)
        if u is not None:
            ups.append(u)
    check("scale source becomes gps",
          eng.scale_source == "gps" and ups[-1].scale_source == "gps",
          eng.scale_source)
    pos = [(u.x, u.y, u.z) for u in ups]
    vo_len = sum(math.dist(a, b) for a, b in zip(pos, pos[1:]))
    gps_len = step_m * (len(frames) - 1)
    check("VO path length ~ GPS length",
          0.6 * gps_len < vo_len < 1.4 * gps_len,
          "vo %.2f m vs gps %.2f m" % (vo_len, gps_len))
    # stationary GPS (every dock run): scale fit must refuse the jitter and
    # never collapse the chain to a point
    eng2 = _mk_engine()
    for i, f in enumerate(frames[:6]):
        eng2.feed(f, meta={"xutm": "294952.55", "yutm": "4588707.83"},
                  t=float(i), name="s%02d.jpg" % i)
    check("stationary fix refused (scale stays >= unit-ish)",
          eng2.scale > 0.5, "scale=%.3f source=%s" % (eng2.scale,
                                                      eng2.scale_source))


def test_corrupt(frames):
    sect("corrupt / truncated frame handling")
    eng = _mk_engine()
    eng.feed(frames[0], t=0.0, name="a.jpg")
    eng.feed(frames[1], t=1.0, name="b.jpg")
    kf_before = eng.stats["keyframes"]
    check("garbage-after-SOI skipped",
          eng.feed(b"\xff\xd8" + b"\x00garbage" * 64, t=2.0,
                   name="bad1.jpg") is None and eng.stats["corrupt"] == 1)
    check("truncated real JPEG skipped",
          eng.feed(frames[2][:len(frames[2]) // 2], t=3.0,
                   name="bad2.jpg") is None and eng.stats["corrupt"] == 2)
    u = eng.feed(frames[2], t=4.0, name="c.jpg")
    check("tracking survives corruption",
          u is not None and eng.stats["keyframes"] == kf_before + 1,
          eng.status)
    check("corrupt frames never became keyframes/lost",
          eng.stats["lost"] == 0 and eng.stats["reinits"] == 0)


def test_runner(frames):
    sect("VslamRunner against a growing temp run dir")
    tmp = tempfile.mkdtemp(prefix="vslam_selftest_")
    try:
        cam = os.path.join(tmp, "cam1")
        os.makedirs(cam)
        names = ["Cam1_20260827_0300%02d.00.jpg" % i for i in range(len(frames))]
        with open(os.path.join(cam, "flight_log.csv"), "w") as fh:
            fh.write("filename,datetime,lat,long,xutm,yutm,utm_zone\n")
            for i, n in enumerate(names):
                fh.write("%s,,41.42,-71.45,%.2f,4588707.83,19T\n"
                         % (n, 294900.0 + i * 0.4))
        eng = _mk_engine()
        r = vslam.VslamRunner(engine=eng, poll_s=0.1)
        r.start(tmp, cam="cam1")
        # trickle files in while the watcher runs, like a live survey:
        for i, n in enumerate(names[:10]):
            with open(os.path.join(cam, n), "wb") as fh:
                fh.write(frames[i])
            time.sleep(0.08)
        # a corrupt frame with a legal name...
        with open(os.path.join(cam, "Cam1_20260827_030010.50.jpg"), "wb") as fh:
            fh.write(b"\xff\xd8" + b"\x00junk" * 100)
        # ...a duplicate rewrite of an already-fed name (must not re-feed)...
        with open(os.path.join(cam, names[3]), "wb") as fh:
            fh.write(frames[3])
        # ...an out-of-order straggler dated BEFORE everything above...
        with open(os.path.join(cam, "Cam1_20260827_025959.00.jpg"), "wb") as fh:
            fh.write(frames[10])
        # ...and one more good frame to prove tracking continued.
        with open(os.path.join(cam, names[11]), "wb") as fh:
            fh.write(frames[11])
        deadline = time.time() + 20
        while time.time() < deadline:
            if r.watch["files_seen"] >= 13 and r.watch["pending"] == 0:
                break
            time.sleep(0.1)
        time.sleep(0.3)                       # let the last feed finish
        r.stop()
        snap = r.snapshot()
        s = snap["stats"]
        check("all files observed", s["files_seen"] >= 13,
              "files_seen=%d" % s["files_seen"])
        check("keyframes from trickle", s["keyframes"] >= 8,
              "keyframes=%d" % s["keyframes"])
        check("corrupt counted once", s["corrupt"] == 1, str(s["corrupt"]))
        check("out-of-order straggler skipped", s["out_of_order"] == 1,
              str(s["out_of_order"]))
        check("duplicate not re-fed", s["fed"] <= s["files_seen"] - 1,
              "fed=%d" % s["fed"])
        check("gps scale picked up from flight_log",
              snap["scale_source"] == "gps", snap["scale_source"])
        check("status is tracking", snap["status"].startswith("tracking"),
              snap["status"])
        blob = json.dumps(snap)
        check("snapshot JSON-safe and capped", len(blob) < 2_000_000,
              "%d bytes" % len(blob))
        check("trajectory entries carry pose + quat",
              all(k in snap["trajectory"][0]
                  for k in ("t", "file", "x", "y", "z", "qw", "qx", "qy", "qz")))
        check("utm_align present (boat moved)",
              snap["utm_align"] is not None and snap["utm_align"]["n"] >= 3,
              str(snap["utm_align"]))
        # writers
        jpath = os.path.join(tmp, "out.json")
        ppath = os.path.join(tmp, "out.ply")
        with open(jpath, "w") as fh:
            json.dump(snap, fh)
        vslam.write_ply(ppath, snap)
        with open(ppath) as fh:
            head = fh.read(200)
        check("ply header sane", head.startswith("ply\n") and
              "element vertex" in head)
        check("ply carries points", os.path.getsize(ppath) > 1000,
              "%d bytes" % os.path.getsize(ppath))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_reinit(frames):
    sect("tracking loss -> segment reinit")
    eng = _mk_engine()
    eng.feed(frames[0], t=0.0, name="a.jpg")
    eng.feed(frames[1], t=1.0, name="b.jpg")
    # an unrelated scene ORB cannot match against the keyframe, repeatedly:
    import numpy as np
    import cv2
    rng = np.random.default_rng(99)
    alien = np.tile(np.linspace(190, 60, 640, dtype=np.uint8), (480, 1))
    for _ in range(300):
        cv2.circle(alien, (int(rng.integers(0, 640)), int(rng.integers(0, 480))),
                   int(rng.integers(3, 12)), int(rng.integers(0, 255)), -1)
    ok, buf = cv2.imencode(".jpg", alien, [cv2.IMWRITE_JPEG_QUALITY, 95])
    for i in range(eng.reinit_after):
        eng.feed(bytes(buf), t=2.0 + i, name="alien%d.jpg" % i)
    check("reinit after lost streak", eng.stats["reinits"] == 1
          and len(eng.segments) == 2,
          "reinits=%d segments=%d" % (eng.stats["reinits"], len(eng.segments)))
    check("lost frames counted", eng.stats["lost"] >= eng.reinit_after - 1,
          str(eng.stats["lost"]))


# ---------------------------------------------------------------------------
# stereo

def _feed_stereo(eng, f1s, f2s, drop=(), gps_step=STEP_M):
    ups = []
    for i, f1 in enumerate(f1s):
        f2 = None if i in drop else f2s[i]
        meta = {"xutm": "%.3f" % (500000.0 + i * gps_step),
                "yutm": "4588707.83"}
        u = eng.feed_pair(f1, f2, meta=meta, t=float(i),
                          name="st_%02d.jpg" % i)
        if u is not None:
            ups.append(u)
    return ups


def _path_len(ups):
    pos = [(u.x, u.y, u.z) for u in ups]
    return sum(math.dist(a, b) for a, b in zip(pos, pos[1:]))


def test_stereo_metric(f1s, f2s):
    sect("stereo: metric scale from the baseline (no GPS in the chain)")
    eng = _mk_stereo()
    ups = _feed_stereo(eng, f1s, f2s)
    R12, t12 = eng._ext
    check("extrinsics t direction ~ (-baseline, 0, 0)",
          abs(t12[0] + BASE_M) < 0.05 and abs(t12[1]) < 0.05
          and abs(t12[2]) < 0.05, "t12=(%.3f %.3f %.3f)" % tuple(t12))
    check("extrinsics scatter small",
          eng.stats["calib_scatter_deg"] is not None
          and eng.stats["calib_scatter_deg"] < 5.0,
          "%s deg" % eng.stats["calib_scatter_deg"])
    n_expect = len(f1s) - eng.calib_pairs + 1     # calib eats the first pairs
    check("keyframes after calibration", len(ups) >= n_expect - 1,
          "%d updates (expect ~%d)" % (len(ups), n_expect))
    exp = STEP_M * (len(ups) - 1)
    got = _path_len(ups)
    check("METRIC path length within 10%",
          0.9 * exp < got < 1.1 * exp, "vo %.2f m vs true %.2f m" % (got, exp))
    check("scale source is stereo", eng.scale_source == "stereo"
          and all(u.scale_source == "stereo" for u in ups))
    check("UNCALIBRATED baseline flagged loudly",
          "UNCALIBRATED" in eng.calib_status and
          "UNCALIBRATED" in eng.status, eng.calib_status)
    check("gps demoted to cross-check, ratio ~1",
          eng.stats["gps_ratio"] is not None
          and 0.9 < eng.stats["gps_ratio"] < 1.1,
          "ratio=%s" % eng.stats["gps_ratio"])
    check("stereo cloud populated", eng.stats["stereo_points"] > 50,
          "%d points" % eng.stats["stereo_points"])


def test_stereo_planar():
    sect("stereo: planar scene (the case where mono E is degenerate)")
    # single fronto-parallel plane, uniform shift — the exact configuration
    # that deterministically broke the mono essential-matrix chain before
    # MAGSAC, and remains its knife edge. PnP against a metric cloud must
    # not care at all.
    f1s, f2s = _stereo_frames(n=10, layers=False)
    eng = _mk_stereo()
    ups = _feed_stereo(eng, f1s, f2s)
    n_expect = len(f1s) - eng.calib_pairs + 1
    check("planar scene tracks >80%", len(ups) >= 0.8 * n_expect,
          "%d updates of ~%d (lost=%d)" % (len(ups), n_expect,
                                           eng.stats["lost"]))
    pos = [(u.x, u.y, u.z) for u in ups]
    steps = [tuple(b[k] - a[k] for k in range(3))
             for a, b in zip(pos, pos[1:])]
    mean = [sum(s[k] for s in steps) / len(steps) for k in range(3)]
    mlen = math.sqrt(sum(v * v for v in mean)) or 1.0
    dots = [sum(s[k] * mean[k] for k in range(3)) /
            ((math.sqrt(sum(v * v for v in s)) or 1e-12) * mlen)
            for s in steps]
    check("planar trajectory direction-consistent", min(dots) > 0.9,
          "min cos %.3f" % min(dots))
    exp = STEP_M * (len(ups) - 1)
    got = _path_len(ups)
    check("planar metric length within 15%",
          0.85 * exp < got < 1.15 * exp,
          "vo %.2f m vs true %.2f m" % (got, exp))


def test_stereo_unpaired(f1s, f2s):
    sect("stereo: missing cam2 halves fall back to PnP-vs-last-cloud")
    eng = _mk_stereo()
    ups = _feed_stereo(eng, f1s, f2s, drop=(6, 7))
    check("unpaired frames counted", eng.stats["unpaired"] == 2,
          str(eng.stats["unpaired"]))
    check("chain continues through the gap",
          len(ups) >= len(f1s) - eng.calib_pairs,
          "%d updates" % len(ups))
    exp = STEP_M * (len(ups) - 1)
    got = _path_len(ups)
    check("metric length still within 10%",
          0.9 * exp < got < 1.1 * exp, "vo %.2f m vs true %.2f m" % (got, exp))
    # corrupt cam2 half: demoted to unpaired, cam1 chain unharmed
    eng.feed_pair(f1s[-1], b"\xff\xd8garbage", t=99.0, name="c2bad.jpg")
    check("corrupt cam2 half dropped, not fatal",
          eng.stats["cam2_dropped"] == 1 and eng.stats["corrupt"] == 0,
          "cam2_dropped=%d" % eng.stats["cam2_dropped"])


def test_runner_auto(f1s, f2s, mono_frames):
    sect("VslamRunner auto mode")
    # (a) cam1+cam2 with matching instants -> stereo
    tmp = tempfile.mkdtemp(prefix="vslam_auto_")
    try:
        names1 = ["Cam1_20260827_0400%02d.00.jpg" % i for i in range(8)]
        names2 = ["Cam2_20260827_0400%02d.00.jpg" % i for i in range(8)]
        os.makedirs(os.path.join(tmp, "cam1"))
        os.makedirs(os.path.join(tmp, "cam2"))
        with open(os.path.join(tmp, "cam1", "flight_log.csv"), "w") as fh:
            fh.write("filename,datetime,lat,long,xutm,yutm,utm_zone\n")
            for i, n in enumerate(names1):
                fh.write("%s,,41.42,-71.45,%.3f,4588707.83,19T\n"
                         % (n, 294900.0 + i * STEP_M))
        for i in range(8):
            with open(os.path.join(tmp, "cam1", names1[i]), "wb") as fh:
                fh.write(f1s[i])
            with open(os.path.join(tmp, "cam2", names2[i]), "wb") as fh:
                fh.write(f2s[i])
        r = vslam.VslamRunner(mode="auto", baseline_m=BASE_M, calib_pairs=3,
                              engine_kw=_engine_kw())
        r.run_once(tmp)
        snap = r.snapshot()
        check("auto chose stereo", snap["mode"] == "stereo",
              str(snap["stats"].get("mode_reason")))
        check("pairing rate 100%", snap["pairing_rate"] == 1.0,
              str(snap["pairing_rate"]))
        check("stereo keyframes from run dir",
              snap["stats"]["keyframes"] >= 4,
              "keyframes=%d" % snap["stats"]["keyframes"])
        check("snapshot carries baseline + calib flags",
              snap["baseline_m"] == BASE_M
              and snap["baseline_calibrated"] is False
              and "UNCALIBRATED" in snap["calib"], snap["calib"])
        check("gps cross-check in snapshot",
              snap["stats"]["gps_ratio"] is not None
              and 0.85 < snap["stats"]["gps_ratio"] < 1.15,
              "ratio=%s" % snap["stats"]["gps_ratio"])
        check("auto-stereo snapshot JSON-safe",
              len(json.dumps(snap)) < 2_000_000)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # (b) cam1 only -> mono
    tmp = tempfile.mkdtemp(prefix="vslam_auto_m_")
    try:
        os.makedirs(os.path.join(tmp, "cam1"))
        for i in range(6):
            with open(os.path.join(tmp, "cam1",
                                   "Cam1_20260827_0500%02d.00.jpg" % i),
                      "wb") as fh:
                fh.write(mono_frames[i])
        r = vslam.VslamRunner(mode="auto", engine_kw=_engine_kw())
        r.run_once(tmp)
        snap = r.snapshot()
        check("auto chose mono (no cam2 dir)", snap["mode"] == "mono",
              str(snap["stats"].get("mode_reason")))
        check("mono tracked", snap["stats"]["keyframes"] >= 4,
              "keyframes=%d" % snap["stats"]["keyframes"])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------

def main():
    print("vslam selftest — %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    test_degradation()
    test_pure_math()
    if have_cv():
        frames = test_tracking()
        test_gps_scale(frames)
        test_corrupt(frames)
        test_reinit(frames)
        test_runner(frames)
        f1s, f2s = _stereo_frames()
        test_stereo_metric(f1s, f2s)
        test_stereo_planar()
        test_stereo_unpaired(f1s, f2s)
        test_runner_auto(f1s, f2s, frames)
    else:
        print("\n== synthetic tracking .. runner: SKIPPED (cv2/numpy not "
              "installed on this host)")
        print("   the degradation checks above ARE the correct behaviour "
              "for this host; install with:")
        print("   pip3 install opencv-python-headless numpy")
        # the real backend must agree with the monkeypatched one
        err = vslam.CpuOrbBackend().ensure()
        check("real backend honestly unavailable", bool(err), str(err))
    print("\n%d passed, %d failed" % (len(PASS), len(FAIL)))
    if FAIL:
        print("FAILED: %s" % ", ".join(FAIL))
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())

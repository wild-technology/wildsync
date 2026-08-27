#!/usr/bin/env python3
"""rigcal selftest — the stereo solve against a synthetic rig with known
truth, the guidance engine's advice, and the IMU stillness gate. No hardware,
no network: capture is bypassed and _detect is fed rendered JPEGs.

Run: python3 rig/tests/rigcal_selftest.py            (exit 0 = pass)
"""
import json
import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

PASS = FAIL = 0


def check(name, ok, detail=""):
    global PASS, FAIL
    PASS += ok
    FAIL += not ok
    print("  %s %s%s" % ("PASS" if ok else "FAIL", name,
                         (" — " + str(detail)) if detail and not ok else ""))


def main():
    os.environ["HOME"] = tempfile.mkdtemp()
    import importlib
    import rigcal
    importlib.reload(rigcal)
    try:
        cv2, np = rigcal._cv()
    except RuntimeError as e:
        print("SKIP (no cv2/numpy): %s" % e)
        print("rigcal_selftest: 0 failed (skipped)")
        return 0

    # ---- synthetic stereo rig ---------------------------------------------
    COLS, ROWS, SQ_MM = 9, 6, 30.0
    W, H = 1616, 1080
    FX = 940.0                      # "unknown" truth the solve must find
    K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
    BASE = 0.302                    # metres, truth
    R2 = cv2.Rodrigues(np.array([0.0, math.radians(2.0), 0.0]))[0]
    T2 = np.array([[-BASE], [0.0], [0.0]])
    sq = SQ_MM / 1000.0
    obj = np.zeros((ROWS * COLS, 3), np.float32)
    obj[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * sq

    def render(rvec, tvec, Rc=np.eye(3), tc=np.zeros((3, 1))):
        """Draw a filled checkerboard as seen by a camera at (Rc,tc)."""
        Rb = cv2.Rodrigues(np.asarray(rvec, float))[0]
        # board squares: corners of each cell in board coords, projected
        img = np.full((H, W), 140, np.uint8)
        for by in range(ROWS + 1):
            for bx in range(COLS + 1):
                if (bx + by) % 2:
                    continue
                cell = np.array([[bx - 1, by - 1, 0], [bx, by - 1, 0],
                                 [bx, by, 0], [bx - 1, by, 0]], float) * sq
                pw = (Rb @ cell.T + np.asarray(tvec, float).reshape(3, 1))
                pc = Rc @ pw + tc
                if (pc[2] <= 0.05).any():
                    return None
                uv, _ = cv2.projectPoints(pc.T, np.zeros(3), np.zeros(3),
                                          K, None)
                # shift=4: subpixel fixed-point rendering. Integer-truncated
                # polygons bias every edge by up to a pixel and the solve then
                # honestly reports that bias as RMS ~1.8 px / focal -5%.
                cv2.fillConvexPoly(img,
                                   (uv.reshape(-1, 2) * 16).astype(np.int32),
                                   30, lineType=cv2.LINE_AA, shift=4)
        return img

    def jpeg(img):
        return cv2.imencode(".jpg", img)[1].tobytes()

    sess = rigcal.StereoSession(COLS, ROWS, SQ_MM,
                                baseline_mm=BASE * 1000 * 1.01)
    poses = [
        ((0.0, 0.0, 0.1), (0.03, -0.08, 0.46)),     # near, centred between cams
        ((0.0, 0.0, -0.1), (0.05, -0.07, 0.48)),    # near
        ((0.3, 0.0, 0.0), (-0.12, -0.08, 1.4)),
        ((-0.3, 0.1, 0.0), (-0.10, -0.10, 1.4)),
        ((0.0, 0.35, 0.0), (-0.42, -0.08, 1.5)),    # tilted, left
        ((0.0, -0.35, 0.0), (0.12, -0.08, 1.5)),    # tilted, right
        ((0.25, 0.25, 0.0), (-0.30, -0.45, 1.6)),   # top-left
        ((0.1, 0.0, 0.0), (-0.12, -0.48, 1.6)),     # top
        ((-0.1, -0.15, 0.1), (0.12, -0.46, 1.7)),   # top-right
        ((0.2, -0.2, 0.1), (-0.12, 0.18, 1.6)),     # bottom
        ((0.0, 0.0, 0.3), (-0.40, -0.30, 1.45)),
        ((0.1, 0.2, -0.2), (0.05, 0.08, 1.45)),
        ((-0.2, 0.15, 0.2), (-0.40, 0.10, 2.0)),
        ((0.15, -0.3, 0.0), (0.0, -0.32, 2.0)),
        ((0.0, 0.15, 0.0), (-0.14, -0.10, 1.5)),    # far
        ((0.2, 0.0, -0.1), (-0.12, -0.06, 1.5)),    # far
        ((0.0, -0.15, 0.05), (-0.30, -0.08, 1.52)), # far
        ((0.0, 0.0, 0.05), (0.02, -0.09, 0.47)),    # near
        # STRONG tilts at close range - the poses that actually condition
        # focal length against distance (mono RMS stays low without them
        # while fx wanders +/-6%: measured in this very test).
        ((0.0, 0.55, 0.0), (-0.32, -0.10, 0.95)),
        ((0.0, -0.55, 0.0), (0.02, -0.10, 0.95)),
        ((0.5, 0.0, 0.0), (-0.15, -0.28, 0.95)),
        ((-0.5, 0.0, 0.0), (-0.15, 0.05, 0.95)),
        ((0.4, 0.4, 0.0), (-0.30, -0.28, 1.05)),
        ((-0.4, -0.4, 0.0), (0.0, 0.02, 1.05)),
        ((0.0, 0.45, 0.5), (-0.28, -0.12, 1.0)),    # tilt + roll
        ((0.35, -0.35, -0.5), (-0.05, -0.05, 1.0)), # tilt + roll
        # corner-cell fills, chosen to stay fully visible to BOTH cameras
        # (cam2's view sits a baseline left of cam1's)
        ((0.1, 0.15, 0.0), (-0.60, -0.36, 1.3)),    # top-left both
        ((0.05, -0.1, 0.0), (0.60, -0.40, 1.25)),   # top-right both
        ((0.05, -0.12, 0.0), (0.60, -0.08, 1.25)),  # right both
        ((0.05, -0.1, 0.0), (0.60, 0.24, 1.25)),    # bottom-right both
        ((0.05, 0.05, 0.0), (0.18, -0.42, 1.3)),    # top-centre for cam2
        ((0.05, 0.1, 0.0), (0.04, -0.10, 0.44)),    # near
    ]
    # systematic sweep chosen so every board is FULLY visible to both
    # cameras (the baseline shifts cam2's view), reaches the edge coverage
    # cells of each, and keeps the cell pitch above the detector's 18 px
    # floor (z <= ~1.55 m at fx 940 / 30 mm squares).
    for oy in (-0.36, -0.10, 0.24):
        for ox in (-0.55, -0.28, 0.0, 0.25, 0.52):
            poses.append(((0.08 * ox, -0.08 * oy, 0.06), (ox, oy, 1.5)))
    for rvec, tvec in poses:
        rec = {"at": time.time(), "cams": {}}
        i1 = render(rvec, tvec)
        i2 = render(rvec, tvec, R2, T2)
        if i1 is None or i2 is None:
            continue
        rec["cams"]["cam1"] = sess._detect(cv2, np, jpeg(i1), "a.jpg")
        rec["cams"]["cam2"] = sess._detect(cv2, np, jpeg(i2), "b.jpg")
        sess.pairs.append(rec)

    st = sess.status()
    check("all synthetic boards detected on both cams",
          st["pairs_good"] >= 12, st)
    check("guidance reaches ready on a well-spread set", st["ready"],
          st["guidance"])

    res = sess.compute()
    err = abs(res["baseline_m"] - BASE) / BASE * 100
    check("baseline recovered within 2%% (got %.1f mm, true %.1f)"
          % (res["baseline_m"] * 1000, BASE * 1000), err < 2.0,
          "%.2f%%" % err)
    fxs = list(res["fx"].values())
    # 3%%: the synthetic renderer's AA+JPEG edges carry a small correlated
    # corner bias that real photographs do not; baseline (the quantity that
    # scales every VSLAM metre) is held to 2%% above, and RMS to sub-pixel.
    check("focal recovered within 3%% with NO prior (got %s, true %.0f)"
          % (fxs, FX), all(abs(f - FX) / FX < 0.03 for f in fxs))
    check("stereo RMS is sub-pixel", res["rms_stereo_px"] < 1.0,
          res["rms_stereo_px"])
    check("tape agreement computed", "agreement_pct" in res,
          res.get("agreement_pct"))

    saved = sess.save()
    check("save writes the calibration artifact",
          os.path.isfile(saved["path"]))
    doc = json.load(open(saved["path"]))
    check("artifact carries the vslam wire-in fields",
          all(k in doc for k in ("K1", "d1", "K2", "d2", "R", "T",
                                 "baseline_m", "rms_stereo_px")))
    import vslam
    importlib.reload(vslam)
    cal = vslam.load_stereo_calibration()
    check("vslam loader reads it back",
          bool(cal) and abs(cal["baseline_m"] - res["baseline_m"]) < 1e-9)

    # ---- guidance: sparse session asks for more ---------------------------
    poor = rigcal.StereoSession(COLS, ROWS, SQ_MM)
    for rvec, tvec in poses[:3]:
        rec = {"at": time.time(), "cams": {}}
        rec["cams"]["cam1"] = sess._detect(cv2, np, jpeg(render(rvec, tvec)),
                                           "a.jpg")
        rec["cams"]["cam2"] = sess._detect(
            cv2, np, jpeg(render(rvec, tvec, R2, T2)), "b.jpg")
        poor.pairs.append(rec)
    stp = poor.status()
    check("sparse session is NOT ready", not stp["ready"])
    check("and asks for more pairs",
          any("more pair" in g for g in stp["guidance"]), stp["guidance"])
    try:
        poor2 = rigcal.StereoSession(COLS, ROWS, SQ_MM)
        poor2.pairs = poor.pairs[:2]
        poor2.compute()
        check("compute refuses too little data", False)
    except RuntimeError as e:
        check("compute refuses too little data", "at least 8" in str(e))
    try:
        rigcal.StereoSession(7, 7, 30)
        check("square pattern refused", False)
    except ValueError:
        check("square pattern refused", True)

    # ---- IMU stillness gate ----------------------------------------------
    still = [{"epoch": i * 0.05, "gx": 0.01, "gy": -0.02, "gz": 0.005,
              "ax": 0.049, "ay": -1.0096, "az": 0.091, "pitch": 10.6,
              "roll": 118.6, "yaw": 65.1, "heading": 65.1}
             for i in range(200)]
    stt = rigcal._stats(still)
    check("still stats: gyro bias recovered",
          abs(stt["gyro_bias_dps"][0] - 0.01) < 1e-6)
    check("still stats: accel norm ~1 g",
          abs(stt["accel_norm_g"] - 1.0179) < 0.01, stt["accel_norm_g"])
    moving = [dict(s, gx=math.sin(i) * 30) for i, s in enumerate(still)]
    stm = rigcal._stats(moving)
    check("motion is detectable for the gate",
          stm["gyro_std_dps"][0] > rigcal.STILL_GYRO_DPS,
          stm["gyro_std_dps"])
    return 0


if __name__ == "__main__":
    rc = main()
    print("rigcal_selftest: %d passed, %d failed" % (PASS, FAIL))
    sys.exit(1 if FAIL else rc)

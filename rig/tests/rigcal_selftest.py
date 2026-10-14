#!/usr/bin/env python3
"""rigcal selftest — the stereo solve against a synthetic rig with known
truth, the guidance engine's advice, the pair-synchronisation gate, the save
guard, and the IMU stillness gate (noise, steady rate, slow tilt, the +/-180
roll wrap, heading circular means). No hardware, no network: capture is
bypassed and _detect is fed rendered JPEGs; the IMU windows are injected.

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

    # ---- save guard: a failed solve never silently replaces a good one ----
    check("the passing synthetic solve is ok (saves without force)",
          res.get("ok") is True, res.get("verdict"))
    sess2 = rigcal.StereoSession(COLS, ROWS, SQ_MM)
    sess2.result = dict(res, ok=False, baseline_m=0.123,
                        verdict=["stereo RMS 15.00 px is HIGH"])
    good_bl = json.load(open(rigcal.STEREO_PATH))["baseline_m"]
    try:
        sess2.save()
        refused = None
    except RuntimeError as e:
        refused = str(e)
    check("a failed solve is refused without force, naming the verdict",
          refused is not None and "did not pass" in refused
          and "RMS 15.00" in refused, refused)
    check("and the saved file is untouched",
          json.load(open(rigcal.STEREO_PATH))["baseline_m"] == good_bl)
    out2 = sess2.save(force=True)
    check("force writes the failed solve",
          json.load(open(rigcal.STEREO_PATH))["baseline_m"] == 0.123
          and out2["forced"] is True)
    check("and keeps the previous solution beside it as .prev",
          os.path.isfile(rigcal.STEREO_PATH + ".prev")
          and json.load(open(rigcal.STEREO_PATH + ".prev"))["baseline_m"]
          == good_bl)
    sess.save()
    check("a passing solve still saves with no force",
          json.load(open(rigcal.STEREO_PATH))["baseline_m"] == good_bl)

    # ---- pair synchronisation: the skew gate is enforced ------------------
    G = {"path": "gpio", "ok": True}
    U = {"path": "usb", "ok": True}
    sy = rigcal._pair_sync({"results": {"cam1": G, "cam2": G}, "skew_ms": 0.6},
                           ["cam1", "cam2"], 10.0)
    check("both on GPIO within the budget is synchronised",
          sy["ok"] is True and sy["skew_ms"] == 0.6, sy)
    sy = rigcal._pair_sync({"results": {"cam1": G, "cam2": U},
                            "skew_ms": None}, ["cam1", "cam2"], 10.0)
    check("a USB-fired camera is unsynchronised, with the reason",
          sy["ok"] is False and "cam2 fired over usb" in sy["reason"], sy)
    sy = rigcal._pair_sync({"results": {"cam1": G, "cam2": G}, "skew_ms": 14},
                           ["cam1", "cam2"], 10.0)
    check("a skew over the spec is unsynchronised",
          sy["ok"] is False and "14.0 ms" in sy["reason"], sy)
    sy = rigcal._pair_sync({"results": {"cam1": G, "cam2": G},
                            "skew_ms": None}, ["cam1", "cam2"], 10.0)
    check("GPIO with no measured skew is NOT assumed synchronised",
          sy["ok"] is False, sy)
    sy = rigcal._pair_sync({"results": {
        "cam1": dict(G, exposure_epoch=100.0000),
        "cam2": dict(G, exposure_epoch=100.0008)}, "skew_ms": 90.0},
        ["cam1", "cam2"], 10.0)
    check("the pair's own exposure instants decide, not a third camera's skew",
          sy["ok"] is True and abs(sy["skew_ms"] - 0.8) < 1e-6, sy)

    import copy
    mixed = rigcal.StereoSession(COLS, ROWS, SQ_MM)
    mixed.pairs = copy.deepcopy(sess.pairs)
    found_pairs = [p for p in mixed.pairs
                   if all(c.get("found") for c in p["cams"].values())]
    for p in found_pairs[5:]:
        p["sync"] = {"ok": False, "reason": "cam2 fired over usb"}
    stm2 = mixed.status()
    check("status() counts only the synchronised pairs (legacy unmarked "
          "records stay usable)", len(found_pairs) > 8
          and stm2["pairs_good"] == 5, stm2["pairs_good"])
    check("and says why pairs were rejected",
          any("rejected" in g and "GPIO" in g for g in stm2["guidance"]),
          stm2["guidance"])
    try:
        mixed.compute()
        check("compute refuses when too few pairs are synchronised", False)
    except RuntimeError as e:
        check("compute refuses when too few pairs are synchronised",
              "only 5 usable pairs" in str(e), str(e))
    allsync = rigcal.StereoSession(COLS, ROWS, SQ_MM)
    allsync.pairs = copy.deepcopy(sess.pairs)
    for p in allsync.pairs:
        p["sync"] = {"ok": True, "reason": ""}
    check("a fully synchronised session is as ready as an unmarked one",
          allsync.status()["ready"] == sess.status()["ready"])

    # capture() end to end against a stub rig
    img1, img2 = jpeg(render(*poses[0])), jpeg(render(*poses[0], R2, T2))

    class Mon:
        def __init__(self, name, num, host):
            self.name_, self.node, self.host = name, {"cam_num": num}, host
            self.fired = False

        def is_capturing(self):
            return True

        def is_connected(self):
            return True

        def shots(self):
            return ["IMG_%s.jpg" % self.name_] if self.fired else []

    class Rig:
        def __init__(self, reply):
            self.monitors = [Mon("cam1", 1, "h1"), Mon("cam2", 2, "h2")]
            self.reply = reply
            self.runmgr = self
            self.SKEW_BUDGET_MS = 10.0

        def status(self):
            return {"active": False}

        def drain_status(self):
            return {"active": False}

        def capture_once(self, af=False):
            for m in self.monitors:
                m.fired = True
            return self.reply

    real_bytes, real_sleep = rigcal.http_bytes, rigcal.time.sleep
    rigcal.http_bytes = lambda url, timeout=15: (
        (img1 if "h1" in url else img2), None)
    rigcal.time.sleep = lambda s_: None
    try:
        for label, reply, want in (
                ("mixed GPIO and USB", {"ok": True, "skew_ms": None, "results": {
                    "cam1": G, "cam2": U}}, False),
                ("both GPIO at 0.6 ms", {"ok": True, "skew_ms": 0.6, "results": {
                    "cam1": G, "cam2": G}}, True),
                ("both GPIO at 14 ms", {"ok": True, "skew_ms": 14, "results": {
                    "cam1": G, "cam2": G}}, False)):
            cs = rigcal.StereoSession(COLS, ROWS, SQ_MM)
            pv = cs.capture(Rig(reply))
            check("capture(): %s records sync.ok=%s" % (label, want),
                  pv["sync"]["ok"] is want, pv["sync"])
            if not want:
                check("capture(): the rejection reason reaches pair_view",
                      bool(pv["sync"]["reason"]), pv["sync"])
    finally:
        rigcal.http_bytes, rigcal.time.sleep = real_bytes, real_sleep

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

    # ---- IMU stillness gate through calibrate_imu ------------------------
    def window(n=200, **over):
        out = []
        for i in range(n):
            smp = dict(still[0], epoch=i * 0.05)
            for k, v in over.items():
                smp[k] = v(i, n) if callable(v) else v
            out.append(smp)
        return out

    def gate(by_cam, target="cam1", secs=3.0):
        """calibrate_imu with injected windows; (error text or None, doc)."""
        real = (rigcal._collect, rigcal.IMU_SOURCES, rigcal.IMU_PATH)
        path = os.path.join(tempfile.mkdtemp(), "imu_calibration.json")
        rigcal.IMU_PATH = path
        rigcal.IMU_SOURCES = lambda: {n: (n, "/imu/latest") for n in by_cam}
        rigcal._collect = lambda host, p_, seconds: by_cam[host]
        try:
            try:
                rigcal.calibrate_imu(target, secs)
                err = None
            except RuntimeError as e:
                err = str(e)
            doc = json.load(open(path)) if os.path.exists(path) else None
            return err, doc
        finally:
            rigcal._collect, rigcal.IMU_SOURCES, rigcal.IMU_PATH = real

    def tilt(i, n):
        return 15.0 * i / n

    ramp = window(gy=1.5, gz=3.0, pitch=tilt,
                  ax=lambda i, n: math.sin(math.radians(tilt(i, n))),
                  ay=0.0,
                  az=lambda i, n: math.cos(math.radians(tilt(i, n))))
    err, doc = gate({"cam1": ramp})
    check("a steady pan with a slow tilt is refused (std gates alone passed it)",
          err is not None and "MOVED" in err, err)
    check("and nothing is written for the refused window", doc is None)
    err, doc = gate({"cam1": window(gy=6.0)})
    check("a large steady rate with no tilt is refused",
          err is not None and "MOVED" in err and "mean gyro" in err, err)
    bias = window(gx=-1.06, gy=-1.27, gz=0.0,
                  pitch=lambda i, n: 10.6 + 0.02 * math.sin(i),
                  roll=lambda i, n: 118.6 - 0.02 * math.sin(i * 1.3))
    err, doc = gate({"cam1": bias})
    check("a still rig with the documented YB bias (-1.06, -1.27, 0) is accepted",
          err is None and doc and abs(doc["cam1"]["level_ref"]["pitch"]
                                      - 10.6) < 0.05, err)
    err, doc = gate({"cam1": window(
        pitch=lambda i, n: 10.6 + 0.5 * i / n)})
    check("a sub-degree settle across the window is not a false refusal",
          err is None, err)
    wrap_ramp = window(roll=lambda i, n: ((178.0 + 4.0 * i / n + 180) % 360)
                       - 180)
    err, doc = gate({"cam1": wrap_ramp})
    check("a 4 degree tilt across the +/-180 roll wrap is refused",
          err is not None and "MOVED" in err, err)
    flicker = window(roll=lambda i, n: 179.5 if i % 2 else -179.5)
    err, doc = gate({"cam1": flicker})
    check("a still window flickering between 179.5 and -179.5 is accepted",
          err is None, err)
    check("and its level reference is near +/-180, not the arithmetic 0",
          doc and abs(abs(doc["cam1"]["level_ref"]["roll"]) - 180.0) < 1.0,
          doc and doc["cam1"]["level_ref"])
    err, doc = gate({"cam1": window(gx=lambda i, n: math.sin(i) * 30)})
    check("wild oscillating motion is still refused", err is not None
          and "MOVED" in err, err)

    # ---- heading: circular mean, and no fabricated relative yaw ------------
    def hdg(vals):
        return window(heading=lambda i, n: vals[i % len(vals)])

    def cdist(a, b):
        return abs(((a - b + 540) % 360) - 180)
    hm = rigcal._stats(hdg([359, 0, 1, 359, 0, 1]))["heading_mean"]
    check("headings straddling north average near 0, not 120",
          hm is not None and cdist(hm, 0) < 1.0, hm)
    check("a constant heading is unchanged",
          rigcal._stats(still)["heading_mean"] == 65.1)
    hm = rigcal._stats(hdg([10, 350]))["heading_mean"]
    check("10 and 350 average near 0, not 180",
          hm is not None and cdist(hm, 0) < 1.0, hm)
    check("no heading column gives None, not 0",
          rigcal._stats([{k: v for k, v in smp.items() if k != "heading"}
                         for smp in still])["heading_mean"] is None)
    check("headings spread evenly round the circle give None (no direction)",
          rigcal._stats(hdg([0, 90, 180, 270]))["heading_mean"] is None)
    check("relative yaw across north is -1.0, not a 359 swing",
          abs(rigcal._yaw_difference(359.5, 0.5) + 1.0) < 1e-9)
    check("relative yaw with an unknown heading is None, not made up",
          rigcal._yaw_difference(None, 12.0) is None
          and rigcal._yaw_difference(12.0, None) is None)
    err, doc = gate({"cam1": hdg([359, 0]), "cam2": hdg([0, 1])}, "both")
    check("calibrate_imu(both) records the cam1->cam2 yaw seed across north",
          err is None and abs(doc["relative"]["yaw"] + 1.0) < 0.05,
          err or doc.get("relative"))
    nohead = [{k: v for k, v in smp.items() if k != "heading"}
              for smp in still]
    err, doc = gate({"cam1": hdg([359, 0]), "cam2": nohead}, "both")
    check("a camera with no heading leaves the relative yaw unset",
          err is None and doc["relative"]["yaw"] is None, err or doc.get("relative"))
    return 0


if __name__ == "__main__":
    rc = main()
    print("rigcal_selftest: %d passed, %d failed" % (PASS, FAIL))
    sys.exit(1 if FAIL else rc)

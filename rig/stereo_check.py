#!/usr/bin/env python3
"""Stereo-pair verification for an ingested transect.

Proves, from image content alone, that the frames the rig logged as a pair
really were taken together from the two rig positions: for every pair the
relative pose (rotation R, translation direction t) between cam1 and cam2 is
recovered from SIFT matches via the essential matrix. On a rigid rig that
pose is the SAME for every pair; if the pairing were off by a shot, the boat's
motion between shots would enter t and R and the pose would scatter. The
control runs the identical estimate on deliberately mis-paired frames
(cam1[i] with cam2[i+1]).

With the rig's known baseline (224 mm) the recovered unit translation is
scaled to metric and inlier points are triangulated: median scene range per
pair is the physical plausibility check (seafloor distance must be positive,
consistent, and survey-like).

    python3 rig/stereo_check.py ~/rig-runs/260820_1925_transect-01 [--every 10] [--baseline-mm 224]
"""
import argparse
import glob
import math
import os
import statistics
import sys

import cv2
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ingest import read_frame_index  # noqa: E402 - C3: the FULL frame index

SENSOR_W_MM = 35.7         # ILX-LR1 full-frame width
FLAT_PORT = 1.33           # refraction through a flat port scales focal length
PORT = [FLAT_PORT]         # overridable: --port-factor 1.0 for a dome port


def pairs_of(root):
    # run.json carries only the LAST 2000 index entries, so a long transect's
    # head silently vanished from the check; read_frame_index prefers the
    # append-only <run>/index.jsonl and falls back to run.json (contract C3).
    rows = {1: [], 2: []}
    for e in read_frame_index(root):
        rows.setdefault(e["cam"], []).append(e)
    rows[1].sort(key=lambda r: r["epoch"]); rows[2].sort(key=lambda r: r["epoch"])
    out = []
    j = 0
    for a in rows[1]:
        while j < len(rows[2]) and rows[2][j]["epoch"] < a["epoch"] - 0.05:
            j += 1
        if j < len(rows[2]) and abs(rows[2][j]["epoch"] - a["epoch"]) <= 0.05:
            out.append((a, rows[2][j])); j += 1
    return out


def card_jpg(root, row):
    return os.path.join(root, "cam%d" % row["cam"],
                        os.path.splitext(row["file"])[0] + ".card.JPG")


def load(path, scale):
    """Turbid-water survey frames are a grey fog with faint texture under a
    strong port vignette (SIFT found 1-15 features per raw frame). Flatten the
    illumination field (high-pass), then equalise local contrast, before any
    feature work."""
    im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if im is None:
        return None
    im = im.astype(np.float32)
    bg = cv2.GaussianBlur(im, (0, 0), 60)
    flat = np.clip((im - bg) * 3 + 128, 0, 255).astype(np.uint8)
    flat = cv2.resize(flat, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8)).apply(flat)


def intrinsics(path, scale):
    # one open, not two — and callers must check the file exists first
    with Image.open(path) as im:
        ex = im.getexif().get_ifd(0x8769)
        w, h = im.size
    f_mm = float(ex.get(0x920A) or 29.0)
    f_px = f_mm / SENSOR_W_MM * w * PORT[0] * scale
    return np.array([[f_px, 0, w * scale / 2], [0, f_px, h * scale / 2], [0, 0, 1]])


def rel_pose(ia, ib, K, sift, bf):
    ka, da = sift.detectAndCompute(ia, None)
    kb, db = sift.detectAndCompute(ib, None)
    if da is None or db is None or len(ka) < 50 or len(kb) < 50:
        return None
    good = [m for m, n in bf.knnMatch(da, db, k=2) if m.distance < 0.75 * n.distance]
    if len(good) < 20:
        return None
    pa = np.float32([ka[m.queryIdx].pt for m in good])
    pb = np.float32([kb[m.trainIdx].pt for m in good])
    E, mask = cv2.findEssentialMat(pa, pb, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None or E.shape != (3, 3):
        return None
    n_in, R, t, mask2 = cv2.recoverPose(E, pa, pb, K, mask=mask)
    inl = mask2.ravel().astype(bool)
    # triangulate inliers at the rig baseline to get metric range
    P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = K @ np.hstack([R, t])
    X = cv2.triangulatePoints(P1, P2, pa[inl].T, pb[inl].T)
    X = (X[:3] / X[3]).T
    depth = X[:, 2]
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(R) - 1) / 2))))
    return {"matches": len(good), "inliers": int(inl.sum()),
            "rot_deg": ang, "t": t.ravel(), "depth_unit": np.median(depth[depth > 0]) if (depth > 0).any() else float("nan"),
            "pos_frac": float((depth > 0).mean())}


def summarize(res, baseline_m):
    ts = np.array([r["t"] for r in res])
    mean_t = ts.mean(axis=0); mean_t /= np.linalg.norm(mean_t)
    ang = [math.degrees(math.acos(max(-1, min(1, float(np.dot(t, mean_t)))))) for t in ts]
    rots = [r["rot_deg"] for r in res]
    inl = [r["inliers"] for r in res]
    rng = [r["depth_unit"] * baseline_m for r in res if not math.isnan(r["depth_unit"])]
    # rigidity score: fraction of pairs within 10 deg (t) and 5 deg (R) of the
    # cluster centre — a rigid rig puts most pairs here; mis-pairing does not
    med_rot = statistics.median(rots)
    tight = sum(1 for t, r in zip(ts, rots)
                if math.degrees(math.acos(max(-1, min(1, float(np.dot(t, mean_t)))))) <= 10
                and abs(r - med_rot) <= 5)
    return {"n": len(res), "rigid_frac": tight / len(res),
            "inliers_median": statistics.median(inl),
            "t_dir": mean_t, "t_scatter_deg_median": statistics.median(ang),
            "t_scatter_deg_p90": sorted(ang)[int(0.9 * (len(ang) - 1))],
            "rot_median_deg": statistics.median(rots),
            "rot_scatter_deg": statistics.pstdev(rots) if len(rots) > 1 else 0.0,
            "range_m_median": statistics.median(rng) if rng else float("nan"),
            "range_m_spread": (statistics.pstdev(rng) if len(rng) > 1 else 0.0),
            "pos_frac_median": statistics.median(r["pos_frac"] for r in res)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_root")
    ap.add_argument("--every", type=int, default=10, help="evaluate every Nth pair")
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--baseline-mm", type=float, default=224.0)
    ap.add_argument("--montage", help="write a match visualisation for the first pair here")
    ap.add_argument("--port-factor", type=float, default=FLAT_PORT,
                    help="focal scale for the housing port: 1.33 flat, 1.0 dome")
    a = ap.parse_args()
    PORT[0] = a.port_factor
    prs = pairs_of(a.run_root)
    sel = prs[::a.every]
    print("run %s: %d logged pairs, evaluating %d" % (os.path.basename(a.run_root), len(prs), len(sel)))
    sift = cv2.SIFT_create(nfeatures=8000, contrastThreshold=0.01)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    K = None
    good_res, ctrl_res = [], []
    missing = 0
    for i, (r1, r2) in enumerate(sel):
        p1, p2 = card_jpg(a.run_root, r1), card_jpg(a.run_root, r2)
        # An unmatched row has no .card.JPG on disk (and a partly-ingested run
        # has gaps): skip the pair instead of tracebacking out of the whole
        # check on the very first Image.open (audit 2026-08-23, low).
        if not (os.path.isfile(p1) and os.path.isfile(p2)):
            missing += 1
            continue
        if K is None:
            K = intrinsics(p1, a.scale)
        i1, i2 = load(p1, a.scale), load(p2, a.scale)
        if i1 is None or i2 is None:
            continue
        res = rel_pose(i1, i2, K, sift, bf)
        if res:
            res["pair"] = os.path.splitext(r1["file"])[0]
            good_res.append(res)
            print("  %-32s matches %4d inliers %4d  rot %.2f deg  t=(%+.2f %+.2f %+.2f)  range %.2f m"
                  % (res["pair"], res["matches"], res["inliers"], res["rot_deg"], *res["t"],
                     res["depth_unit"] * a.baseline_mm / 1000.0))
            if a.montage and i == 0:
                ka, da = sift.detectAndCompute(i1, None); kb, db = sift.detectAndCompute(i2, None)
                g = [m for m, n in bf.knnMatch(da, db, k=2) if m.distance < 0.75 * n.distance]
                vis = cv2.drawMatches(i1, ka, i2, kb, g[:150], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                cv2.imwrite(a.montage, vis)
        # control: deliberately mis-paired (next shot on cam2)
        k = prs.index((r1, r2)) + 1
        if k < len(prs):
            p2c = card_jpg(a.run_root, prs[k][1])
            i2c = load(p2c, a.scale)
            if i2c is not None:
                rc = rel_pose(i1, i2c, K, sift, bf)
                if rc:
                    ctrl_res.append(rc)
    print()
    if missing:
        print("skipped %d selected pairs with no .card.JPG on disk "
              "(unmatched or not yet ingested)" % missing)
    for label, res in (("LOGGED PAIRS", good_res), ("CONTROL: cam1[i] vs cam2[i+1] (mis-paired)", ctrl_res)):
        if not res:
            print("%s: no usable estimates" % label); continue
        s = summarize(res, a.baseline_mm / 1000.0)
        print("%s (n=%d)  RIGID-CLUSTER FRACTION %.0f%%" % (label, s["n"], 100 * s["rigid_frac"]))
        print("  inliers median %d | translation direction (%+.2f %+.2f %+.2f) scatter median %.1f deg, p90 %.1f deg"
              % (s["inliers_median"], *s["t_dir"], s["t_scatter_deg_median"], s["t_scatter_deg_p90"]))
        print("  inter-camera rotation median %.2f deg (scatter sd %.2f) | scene range at %.0f mm baseline: median %.2f m (sd %.2f) | %.0f%% points in front"
              % (s["rot_median_deg"], s["rot_scatter_deg"], a.baseline_mm, s["range_m_median"], s["range_m_spread"], 100 * s["pos_frac_median"]))


if __name__ == "__main__":
    main()

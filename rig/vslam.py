#!/usr/bin/env python3
"""vslam — live camera-derived trajectory from the review JPEGs of a run.

During a survey the nodes trickle ~200 KB 1616x1080 review JPEGs onto the host
(<run>/cam1/Cam1_YYYYMMDD_hhmmss.ss.jpg) at ~2 Hz, alongside a flight_log.csv
row per frame with nav state (lat/long/xutm/yutm/heading/pitch/roll). This
module turns that trickle into an incremental visual-odometry trajectory the
operator can see live, in one of two modes:

  mono    ORB features, ratio-tested Hamming matches against the previous
          keyframe, essential matrix + MAGSAC, pose chain, sparse landmark
          cloud. Monocular VO has no scale; when the flight_log carries UTM
          the scale of each segment is fitted to the GPS baseline over a
          sliding window, otherwise the chain is unit-norm and says so.
  stereo  the rig fires cam1+cam2 synchronized (~0.4 ms skew), so same-
          instant pairs triangulate a METRIC cloud from the inter-camera
          baseline and the chain is PnP (3D->2D) against the previous
          pair's cloud — metric scale with no GPS, and no planar-scene
          degeneracy. The inter-camera pose is self-estimated from the
          first pairs; the baseline LENGTH is a rig constant that is
          currently UNMEASURED (default 0.30 m placeholder, loudly flagged
          until measured — docs/vslam-jetson.md). GPS demotes to a
          cross-check: stats.gps_ratio = stereo length / GPS length.
  auto    stereo when the pair dir exists and >50% of frames pair up by
          capture instant; mono otherwise.

Runs today on the macOS host (CPU); it is finalized on the Jetson Orin Nano,
where the same VslamEngine can be handed a CUDA backend — everything that
touches cv2/numpy lives behind the small backend interface (CpuOrbBackend) for
exactly that swap. See docs/vslam-jetson.md for the finalization plan, the
calibration procedure and the rigd /api/vslam integration spec.

cv2/numpy may be absent on a fresh host: imports are lazy and every entry
point degrades to a clean "unavailable: <reason>" status instead of crashing.
Install on the Mac with `pip3 install opencv-python-headless numpy`; the
Jetson gets cv2 from JetPack/apt (python3-opencv), no pip build.

    python3 rig/vslam.py <run_dir> [--mode auto|mono|stereo] [--cam cam1]
                         [--follow] [--baseline 0.30]
                         [--json out.json] [--ply out.ply]

processes an existing run (or tails a growing one with --follow), printing one
line per keyframe and writing trajectory JSON / a PLY point cloud at the end.
No network, no hardware: input is only files under the run dir.

Selftest (no hardware, passes with or without cv2 installed):

    python3 rig/tests/vslam_selftest.py
"""
import argparse
import calendar
import csv
import json
import math
import os
import re
import sys
import threading
import time

# ---------------------------------------------------------------------------
# backend — the ONLY code that touches cv2/numpy.
#
# The Jetson finalization swaps this class, not the engine: a CUDA backend
# (cv2.cuda ORB + BFMatcher, or VPI, or a cuVSLAM adapter) implements the same
# five methods and make_backend() returns it. The engine passes plain Python
# lists across this boundary and does its pose math in pure Python, so the
# engine itself imports nothing and is testable anywhere.

class CpuOrbBackend:
    """OpenCV CPU backend: ORB -> BF-Hamming ratio matches -> 5-point E.

    Interface contract (a replacement backend implements exactly this):
      ensure()                    -> None when usable, else a reason string
      decode(bytes)               -> opaque gray frame or None (corrupt)
      detect(gray)                -> ([(x, y)...], opaque descriptors)
      match(d1, d2)               -> [(i1, i2)...] ratio-tested index pairs
      rel_pose(p1, p2, K)         -> (R 3x3 lists, t unit 3-list,
                                      inlier index list) or None
      triangulate(p1, p2, K, R, t)-> list ALIGNED with the input pairs:
                                      [x, y, z] in the FIRST camera frame at
                                      the scale of |t|, or None where the
                                      point fails cheirality/depth (stereo
                                      needs the index mapping back to cam1
                                      keypoints; mono just drops the Nones)
      pnp(obj, img, K)            -> (R 3x3 lists, t 3-list, inlier index
                                      list) or None; x_cam = R X + t
      rel_pose_h(p1, p2, K)       -> like rel_pose but via homography
                                      decomposition — the planar-scene
                                      fallback the stereo extrinsics
                                      bootstrap needs (flat sand is
                                      homography-degenerate for rel_pose)
    """

    name = "cpu-orb"

    def __init__(self, n_features=1500, ratio=0.75, proc_scale=0.5,
                 preprocess="flat"):
        self.n_features = n_features
        self.ratio = ratio
        # Survey frames through a port in turbid water are a grey fog with
        # faint texture under strong vignette — stereo_check.py measured
        # 1-15 raw SIFT features per frame until it flattened the illumination
        # field and equalised local contrast. Same treatment here ("flat"),
        # switchable off for synthetic imagery in tests ("none").
        self.preprocess = preprocess
        # Feature work runs on a downscaled frame (0.5 of 1616x1080 is plenty
        # for 1500 ORB points and halves the flatten+CLAHE cost, the most
        # expensive step). decode() applies it; detect() coordinates are in
        # the SCALED image, so the caller must scale K identically — the
        # engine does, via Intrinsics.k33(proc_scale).
        self.proc_scale = proc_scale
        self._cv2 = self._np = None
        self._err = None
        self._orb = self._bf = self._clahe = None

    def ensure(self):
        """Import lazily, once. The import error is cached: retrying a missing
        package on every frame of a 2 Hz stream buys nothing but log spam."""
        if self._cv2 is not None:
            return None
        if self._err is not None:
            return self._err
        try:
            import numpy as np
            import cv2
        except ImportError as e:
            self._err = ("%s — host: pip3 install opencv-python-headless "
                         "numpy; Jetson: apt python3-opencv (JetPack)" % e)
            return self._err
        self._np, self._cv2 = np, cv2
        self._orb = cv2.ORB_create(nfeatures=self.n_features)
        # crossCheck off: the ratio test in match() is the filter.
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self._clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
        return None

    def decode(self, data):
        cv2, np = self._cv2, self._np
        im = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if im is None or im.size == 0:
            return None
        if self.proc_scale != 1.0:
            im = cv2.resize(im, None, fx=self.proc_scale, fy=self.proc_scale,
                            interpolation=cv2.INTER_AREA)
        if self.preprocess == "flat":
            f = im.astype(np.float32)
            bg = cv2.GaussianBlur(f, (0, 0), 30)
            im = np.clip((f - bg) * 3 + 128, 0, 255).astype(np.uint8)
            im = self._clahe.apply(im)
        return im

    def detect(self, gray):
        kps, desc = self._orb.detectAndCompute(gray, None)
        if desc is None or len(kps) == 0:
            return [], None
        return [k.pt for k in kps], desc

    def match(self, d1, d2):
        out = []
        for pair in self._bf.knnMatch(d1, d2, k=2):
            # k=2 can return 1 element near the descriptor-set edge; a match
            # with no second-best to ratio against is not evidence.
            if len(pair) == 2 and pair[0].distance < self.ratio * pair[1].distance:
                out.append((pair[0].queryIdx, pair[0].trainIdx))
        return out

    def rel_pose(self, p1, p2, K):
        cv2, np = self._cv2, self._np
        if len(p1) < 8:                       # 5-pt minimum + RANSAC headroom
            return None
        a = np.float32(p1)
        b = np.float32(p2)
        Km = np.float64(K)
        # MAGSAC, not plain RANSAC: a survey seafloor is close to planar, and
        # a planar scene under smooth motion is homography-degenerate for the
        # 5-point solver — classic RANSAC deterministically picked a 13-inlier
        # forward-motion model on a 972-match pure-sideways pair (the selftest
        # pins this exact case). MAGSAC's marginalising score is robust to it
        # (889/972 on the same pair). Fall back for a pre-4.5 cv2.
        method = getattr(cv2, "USAC_MAGSAC", cv2.RANSAC)
        E, mask = cv2.findEssentialMat(a, b, Km, method=method,
                                       prob=0.999, threshold=1.0)
        if E is None or E.shape != (3, 3):
            return None
        n_in, R, t, mask2 = cv2.recoverPose(E, a, b, Km, mask=mask)
        if n_in < 1:
            return None
        inl = [i for i, v in enumerate(mask2.ravel()) if v]
        return R.tolist(), [float(v) for v in t.ravel()], inl

    def triangulate(self, p1, p2, K, R, t):
        cv2, np = self._cv2, self._np
        Km = np.float64(K)
        P1 = Km @ np.hstack([np.eye(3), np.zeros((3, 1))])
        P2 = Km @ np.hstack([np.float64(R), np.float64(t).reshape(3, 1)])
        # cv2 5.0 asserts equal float depth of P and points: cast both to f64.
        X = cv2.triangulatePoints(P1, P2, np.float64(p1).T, np.float64(p2).T)
        w = X[3]
        w[w == 0] = 1e-12
        X = (X[:3] / w).T
        # Cheirality plus a depth ceiling: at unit |t| a survey step of ~0.5 m
        # over a 2-6 m seafloor puts real points at z ~ 4-12; z > 500 is a
        # near-infinity match (surface glare, particulate) that would fling
        # the cloud kilometres out once scaled. Output stays aligned with the
        # input correspondences (None holes) — see the interface contract.
        return [[float(x), float(y), float(z)] if 0.05 < z < 500.0 else None
                for x, y, z in X]

    def rel_pose_h(self, p1, p2, K):
        """Relative pose via homography decomposition, for scenes where the
        5-point solver is degenerate (one dominant plane — flat seafloor,
        our synthetic planar fixtures): measured on such a pair, MAGSAC E
        kept 3 of 1011 matches while the homography kept 939. Of the up-to-4
        (R, t, n) decompositions the physical one is picked by cheirality
        (fraction of inliers triangulating in FRONT of both cameras: the
        true candidate scores ~1.0, its mirror ~0.0, the spurious pair
        ~0.5); anything below 0.7 support is refused rather than guessed.
        Only the t DIRECTION is meaningful (|t| is scaled by the unknown
        plane distance), same contract as rel_pose."""
        cv2, np = self._cv2, self._np
        if len(p1) < 8:
            return None
        a, b = np.float32(p1), np.float32(p2)
        Km = np.float64(K)
        H, mask = cv2.findHomography(a, b, cv2.RANSAC, 2.0)
        if H is None or mask is None:
            return None
        inl = [i for i, v in enumerate(mask.ravel()) if v]
        if len(inl) < 8:
            return None
        _, Rs, ts, _ = cv2.decomposeHomographyMat(H, Km)
        ai = np.float64(a[inl]).T
        bi = np.float64(b[inl]).T
        P1 = Km @ np.hstack([np.eye(3), np.zeros((3, 1))])
        best = None
        for R, t in zip(Rs, ts):
            tn = float(np.linalg.norm(t))
            if tn < 1e-9:
                continue                      # pure-rotation: no baseline info
            tu = (t / tn).reshape(3, 1)
            X = cv2.triangulatePoints(P1, Km @ np.hstack([R, tu]), ai, bi)
            w = X[3].copy()
            w[w == 0] = 1e-12
            X3 = X[:3] / w
            front = float(((X3[2] > 0) & ((R @ X3 + tu)[2] > 0)).mean())
            if best is None or front > best[0]:
                best = (front, R.tolist(), [float(v) for v in tu.ravel()])
        if best is None or best[0] < 0.7:
            return None
        return best[1], best[2], inl

    def pnp(self, obj, img, K):
        """3D->2D pose (x_cam = R X + t) via RANSAC PnP + iterative refine.
        This is what makes the stereo chain immune to the planar-scene
        degeneracy that bites the essential matrix: PnP against a metric
        cloud is well-posed no matter how flat the seafloor is."""
        cv2, np = self._cv2, self._np
        if len(obj) < 6:
            return None
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            np.float64(obj), np.float64(img), np.float64(K), None,
            reprojectionError=2.0, iterationsCount=200)
        if not ok or inl is None or len(inl) < 4:
            return None
        R = cv2.Rodrigues(rvec)[0]
        return (R.tolist(), [float(v) for v in tvec.ravel()],
                [int(i) for i in np.ravel(inl)])


def make_backend(name="cpu", **kw):
    """Single construction point so the Jetson can add 'cuda' etc. without the
    engine or rigd learning new class names."""
    if name == "cpu":
        return CpuOrbBackend(**kw)
    raise ValueError("unknown vslam backend %r (have: cpu)" % name)


# ---------------------------------------------------------------------------
# intrinsics

class Intrinsics:
    """Pinhole guess for the review JPEG stream.

    Default: 1616x1080 with ~59 deg HFOV -> fx ~ 1429 px. That is a GUESS
    (spec'd assumption for the ILX-LR1 review stream); real calibration is a
    Jetson-day task (docs/vslam-jetson.md). The essential matrix is tolerant
    of moderate focal error — the trajectory shape survives, absolute angles
    do not. port_factor: refraction through a flat housing port multiplies
    the effective in-water focal by ~1.33 (stereo_check.py uses the same
    constant); leave 1.0 in air / dome port.
    """

    def __init__(self, width=1616, height=1080, hfov_deg=59.0, focal_px=None,
                 port_factor=1.0):
        self.width, self.height = int(width), int(height)
        if focal_px is None:
            focal_px = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.focal_px = float(focal_px) * float(port_factor)

    def k33(self, scale=1.0):
        """3x3 K as plain lists, scaled to the backend's processing size —
        detect() coordinates live in the scaled image, so K must too."""
        f = self.focal_px * scale
        return [[f, 0.0, self.width * scale / 2.0],
                [0.0, f, self.height * scale / 2.0],
                [0.0, 0.0, 1.0]]


# ---------------------------------------------------------------------------
# pure-python 3-vector / 3x3 helpers — keeps the engine importable (and its
# pose chain testable) with no numpy on the box. Point counts here are tiny
# (one pose + a few hundred landmarks per keyframe at 2 Hz).

def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _mat_t(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _quat_from_r(R):
    """(w, x, y, z) from a rotation matrix; standard Shepperd branch on the
    largest diagonal element for numerical safety near 180-deg rotations."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (R[2][1] - R[1][2]) / s,
                (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s)
    i = max(range(3), key=lambda k: R[k][k])
    if i == 0:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        return ((R[2][1] - R[1][2]) / s, 0.25 * s,
                (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        return ((R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s,
                0.25 * s, (R[1][2] + R[2][1]) / s)
    s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
    return ((R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s,
            (R[1][2] + R[2][1]) / s, 0.25 * s)


def _meta_gps(meta):
    """(xutm, yutm) floats from a flight_log row dict, or None. The log writes
    empty strings when nav is absent, so parse defensively."""
    if not meta:
        return None
    try:
        x, y = float(meta.get("xutm") or ""), float(meta.get("yutm") or "")
    except (TypeError, ValueError):
        return None
    return (x, y)


class PoseUpdate:
    """One accepted keyframe pose. World frame = first camera frame of the
    run: x right, y DOWN, z forward (OpenCV camera convention) — the
    horizontal plane of a roughly level camera is therefore (x, z), which is
    what the UTM alignment in VslamRunner.snapshot() uses."""

    __slots__ = ("t", "file", "x", "y", "z", "qw", "qx", "qy", "qz",
                 "keyframe", "matches", "inliers", "scale", "scale_source",
                 "segment", "gps")

    def as_dict(self):
        return {k: getattr(self, k) for k in self.__slots__}


# ---------------------------------------------------------------------------
# engine

# GPS steps under this are jitter, not motion (all dock/bench runs to date sit
# at ONE fix for the whole run) — fitting scale to them would collapse the
# trajectory to a point. Below it the last good scale is held, or unit.
MIN_GPS_STEP_M = 0.05
SCALE_WINDOW = 6          # keyframe pairs in the sliding scale fit
LANDMARK_CAP = 20000      # thin by 2 above this; a multi-hour survey must
                          # not grow the cloud without bound


class VslamEngine:
    """Incremental monocular VO over the review-JPEG trickle.

    feed(path_or_bytes, meta=None) -> PoseUpdate | None. None means "frame
    consumed but no new keyframe": still (below parallax), lost (counted,
    reinit after a streak), corrupt (counted), or backend unavailable
    (status says so). All state is single-threaded by design — VslamRunner
    serialises feed() and guards snapshots with its own lock.
    """

    mode = "mono"

    def __init__(self, intrinsics=None, backend=None, min_matches=25,
                 min_inliers=15, min_inlier_frac=0.3, min_parallax_px=10.0,
                 reinit_after=3):
        self.intrinsics = intrinsics or Intrinsics()
        self.backend = backend or make_backend()
        self.min_matches = min_matches
        self.min_inliers = min_inliers
        # Absolute count is not enough: RANSAC can return a small spurious
        # consensus (46 of 680 matches, observed on repetitive blob texture)
        # whose pose is garbage — composing it kinks the chain ~90 deg in one
        # step. A healthy solve keeps 85-95% of ratio-tested matches; under
        # 30% the model, not the scene, is the problem -> treat as lost.
        self.min_inlier_frac = min_inlier_frac
        self.min_parallax_px = min_parallax_px
        self.reinit_after = reinit_after
        # camera-to-world pose of the current keyframe
        self._R = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
        self._C = [0.0, 0.0, 0.0]
        self._kf = None                     # dict: kps/desc/t/file/gps
        self._lost_streak = 0
        self.scale = 1.0
        self.scale_source = "unit"          # unit | gps | gps-hold
        self._gps_steps = []                # sliding window of metric steps
        self.landmarks = []
        self.segments = []                  # [{'start_t','start_file','keyframes'}]
        self.stats = {"frames": 0, "keyframes": 0, "corrupt": 0, "still": 0,
                      "lost": 0, "reinits": 0, "last_matches": 0,
                      "last_inliers": 0, "proc_ms": 0.0}
        self._status = "initializing (waiting for first frame)"

    @property
    def status(self):
        return self._status

    # -- feed -------------------------------------------------------------

    def feed(self, frame, meta=None, t=None, name=None):
        """One frame in arrival order. frame: filesystem path or raw JPEG
        bytes. meta: the frame's flight_log row (dict) if known. t/name:
        timestamp + display name; derived from the path when absent."""
        t0 = time.monotonic()
        err = self.backend.ensure()
        if err:
            self._status = "unavailable: %s" % err
            return None
        data, name, t = self._read(frame, name, t)
        self.stats["frames"] += 1
        if data is None:
            return None                     # _read counted + set status
        # run.py writes review frames straight to their final path (no tmp +
        # rename), so a watcher can catch a half-written file: a JPEG without
        # its EOI marker is truncation, not scenery. Cheaper and more decisive
        # than letting libjpeg salvage a grey half-frame into the map.
        if data[:2] == b"\xff\xd8" and not data.rstrip(b"\0").endswith(b"\xff\xd9"):
            return self._corrupt(name, "truncated (no JPEG EOI)")
        gray = self.backend.decode(data)
        if gray is None:
            return self._corrupt(name, "decode failed")
        kps, desc = self.backend.detect(gray)
        if desc is None or len(kps) < 12:
            # featureless (fog, lens cap, over/underexposure) — not corrupt,
            # but nothing to track either. Treat as a lost frame.
            return self._lost(name, "only %d features" % len(kps))
        if self._kf is None:
            return self._bootstrap(kps, desc, t, name, meta)
        matches = self.backend.match(self._kf["desc"], desc)
        self.stats["last_matches"] = len(matches)
        if len(matches) < self.min_matches:
            return self._lost(name, "%d matches" % len(matches),
                              kps=kps, desc=desc, t=t, meta=meta)
        p_prev = [self._kf["kps"][i] for i, _ in matches]
        p_cur = [kps[j] for _, j in matches]
        # keyframe gate: median pixel displacement. Below it the geometry is
        # noise (E from ~zero parallax is garbage) — the dock runs are exactly
        # this case for hours, and must yield a clean "still" not drift.
        par = sorted(math.hypot(a[0] - b[0], a[1] - b[1])
                     for a, b in zip(p_prev, p_cur))[len(matches) // 2]
        if par < self.min_parallax_px:
            self._lost_streak = 0
            self.stats["still"] += 1
            self._status = "tracking — %d keyframes, still (parallax %.1f px)" % (
                self.stats["keyframes"], par)
            self._tick(t0)
            return None
        K = self.intrinsics.k33(getattr(self.backend, "proc_scale", 1.0))
        rp = self.backend.rel_pose(p_prev, p_cur, K)
        if rp is None:
            return self._lost(name, "essential matrix failed",
                              kps=kps, desc=desc, t=t, meta=meta)
        R_rel, t_rel, inl = rp
        self.stats["last_inliers"] = len(inl)
        need = max(self.min_inliers, int(self.min_inlier_frac * len(matches)))
        if len(inl) < need:
            return self._lost(name, "%d/%d inliers" % (len(inl), len(matches)),
                              kps=kps, desc=desc, t=t, meta=meta)
        up = self._advance(R_rel, t_rel, inl, p_prev, p_cur, K,
                           kps, desc, t, name, meta, len(matches))
        self._tick(t0)
        return up

    # -- internals ---------------------------------------------------------

    def _read(self, frame, name, t):
        if isinstance(frame, (bytes, bytearray)):
            return bytes(frame), name or "<bytes>", t if t is not None else time.time()
        path = os.fspath(frame)
        if name is None:
            name = os.path.basename(path)
        if t is None:
            t = _fname_epoch(name)
            if t is None:
                try:
                    t = os.path.getmtime(path)
                except OSError:
                    t = time.time()
        try:
            with open(path, "rb") as fh:
                data = fh.read()
        except OSError as e:
            # feed() counts the frame after _read returns; only the corrupt
            # tally and status belong here.
            self._corrupt(name, "unreadable: %s" % e)
            return None, name, t
        return data, name, t

    def _corrupt(self, name, why):
        self.stats["corrupt"] += 1
        self._status = "tracking — %d keyframes (skipped %s: %s)" % (
            self.stats["keyframes"], name, why)
        return None

    def _lost(self, name, why, kps=None, desc=None, t=None, meta=None):
        self.stats["lost"] += 1
        self._lost_streak += 1
        # A streak means the previous keyframe is gone for good (occlusion,
        # a turn over featureless bottom, a lighting step) — re-anchor a NEW
        # segment at the last known pose instead of matching into the void
        # forever. Requires a usable current frame to anchor on.
        if (self._lost_streak >= self.reinit_after and desc is not None
                and self._kf is not None):
            self.stats["reinits"] += 1
            self._lost_streak = 0
            self._set_kf(kps, desc, t, name, meta)
            self.segments.append({"start_t": t, "start_file": name,
                                  "keyframes": 0})
            self._status = ("tracking — %d keyframes, reinit segment %d at %s"
                            % (self.stats["keyframes"], len(self.segments), name))
            return None
        self._status = "lost (%d since keyframe: %s)" % (self._lost_streak, why)
        return None

    def _bootstrap(self, kps, desc, t, name, meta):
        self._set_kf(kps, desc, t, name, meta)
        self.segments.append({"start_t": t, "start_file": name, "keyframes": 1})
        self.stats["keyframes"] += 1
        self._status = "tracking — 1 keyframe (origin %s)" % name
        return self._pose_update(t, name, 0, 0, meta)

    def _set_kf(self, kps, desc, t, name, meta):
        self._kf = {"kps": kps, "desc": desc, "t": t, "file": name,
                    "gps": _meta_gps(meta)}

    def _advance(self, R_rel, t_rel, inl, p_prev, p_cur, K,
                 kps, desc, t, name, meta, n_matches):
        # recoverPose convention: x_cur = R x_prev + t (|t| = 1). Camera-to-
        # world chain: R_wc' = R_wc R^T ; C' = C + R_wc (-R^T t) * scale.
        Rt = _mat_t(R_rel)
        step_cam = [-v for v in _mat_vec(Rt, t_rel)]  # cur center, prev frame
        gps = _meta_gps(meta)
        self._update_scale(gps)
        step_w = _mat_vec(self._R, step_cam)
        C_prev, R_prev = self._C, self._R
        self._C = [c + s * self.scale for c, s in zip(self._C, step_w)]
        self._R = _mat_mul(self._R, Rt)
        # landmarks: unit-scale points in the PREVIOUS keyframe camera frame;
        # place them in world with the previous pose and the fitted scale.
        pts = self.backend.triangulate([p_prev[i] for i in inl],
                                       [p_cur[i] for i in inl], K, R_rel,
                                       t_rel)
        for p in pts:
            if p is None:                     # cheirality/depth-filtered hole
                continue
            pw = _mat_vec(R_prev, [v * self.scale for v in p])
            self.landmarks.append([pw[0] + C_prev[0], pw[1] + C_prev[1],
                                   pw[2] + C_prev[2]])
        if len(self.landmarks) > LANDMARK_CAP:
            self.landmarks = self.landmarks[::2]
        self._lost_streak = 0
        self._set_kf(kps, desc, t, name, meta)
        self.segments[-1]["keyframes"] += 1
        self.stats["keyframes"] += 1
        self._status = "tracking — %d keyframes, scale=%s" % (
            self.stats["keyframes"], self.scale_source)
        return self._pose_update(t, name, n_matches, len(inl), meta)

    def _update_scale(self, gps_now):
        """Sliding similarity fit, scale part: recoverPose translations are
        unit-norm, so each keyframe pair contributes exactly 1.0 of VO length
        — the metric scale of a segment is then the mean GPS step over the
        window. Rotation-only keyframes and brief GPS holds are damped by the
        window; a stationary fix (every dock run) is refused outright by
        MIN_GPS_STEP_M and the previous scale is held."""
        prev = self._kf["gps"] if self._kf else None
        if gps_now is not None and prev is not None:
            self._gps_steps.append(math.hypot(gps_now[0] - prev[0],
                                              gps_now[1] - prev[1]))
            self._gps_steps = self._gps_steps[-SCALE_WINDOW:]
        if self._gps_steps:
            mean = sum(self._gps_steps) / len(self._gps_steps)
            if mean >= MIN_GPS_STEP_M:
                self.scale, self.scale_source = mean, "gps"
                return
            if self.scale_source in ("gps", "gps-hold"):
                self.scale_source = "gps-hold"
                return
        # no GPS ever seen (or none yet): unit-norm chain, and say so.
        if self.scale_source == "unit":
            self.scale = 1.0

    def _pose_update(self, t, name, n_matches, n_inl, meta):
        u = PoseUpdate()
        u.t, u.file = t, name
        u.x, u.y, u.z = self._C
        u.qw, u.qx, u.qy, u.qz = _quat_from_r(self._R)
        u.keyframe = True
        u.matches, u.inliers = n_matches, n_inl
        u.scale, u.scale_source = self.scale, self.scale_source
        u.segment = len(self.segments)
        u.gps = _meta_gps(meta)
        return u

    def _tick(self, t0):
        # EMA over tracked frames (still + keyframe): the number the Jetson
        # fps budget in docs/vslam-jetson.md is checked against.
        ms = (time.monotonic() - t0) * 1000.0
        ema = self.stats["proc_ms"]
        self.stats["proc_ms"] = ms if ema == 0 else 0.8 * ema + 0.2 * ms


class StereoVslamEngine(VslamEngine):
    """Stereo VO over synchronized cam1+cam2 pairs (~0.4 ms measured skew).

    Kills both monocular weaknesses at once: the pair triangulates a METRIC
    point cloud from the inter-camera baseline (no GPS needed for scale),
    and the frame-to-frame chain is PnP against the previous pair's cloud —
    well-posed on planar scenes where the essential matrix degenerates.

    Extrinsics: the relative pose cam1->cam2 is SELF-ESTIMATED once, from
    the first `calib_pairs` good pairs (MAGSAC essential matrix per pair,
    medoid over the candidates' translation directions). That fixes the
    rotation and the translation DIRECTION from image evidence; the LENGTH
    is `baseline_m` — a rig constant that is currently UNMEASURED (also see
    docs/future-tests.md §1: lens/encoder parity unverified), so the 0.30 m
    default is a placeholder and every status/snapshot flags it loudly until
    baseline_calibrated=True. A checkerboard stereo calibration replaces the
    whole estimate on Jetson day (docs/vslam-jetson.md). All trajectory
    DISTANCES scale linearly with baseline error; SHAPE does not.

    GPS is a cross-check here, not the scale source: stats.gps_ratio =
    stereo-derived track length / GPS track length, once the boat has moved
    a couple of metres. Ratio far from 1.0 = baseline (or pairing) is wrong.

    feed_pair(f1, f2=None, ...) is the entry point; f2=None (a missing cam2
    half) still advances the pose by PnP against the LAST paired cloud —
    counted in stats.unpaired — but cannot refresh the cloud, so a long
    unpaired streak eventually loses overlap and reinitializes.
    """

    mode = "stereo"

    def __init__(self, intrinsics=None, backend=None, baseline_m=0.30,
                 baseline_calibrated=False, calib_pairs=5, min_pnp_points=12,
                 **kw):
        super().__init__(intrinsics=intrinsics, backend=backend, **kw)
        self.baseline_m = float(baseline_m)
        self.baseline_calibrated = bool(baseline_calibrated)
        self.calib_pairs = int(calib_pairs)
        self.min_pnp_points = min_pnp_points
        self._calib = []            # (R, t_unit, n_inliers) candidates
        self._ext = None            # (R12, t12_metric): x_cam2 = R12 x_cam1 + t12
        self.scale_source = "stereo"
        self.stats.update({"pairs": 0, "unpaired": 0, "cam2_dropped": 0,
                           "stereo_points": 0, "gps_ratio": None,
                           "calib_scatter_deg": None})
        self._vo_len = self._gps_len = 0.0

    @property
    def calib_status(self):
        base = ("baseline %.3f m (measured)" % self.baseline_m
                if self.baseline_calibrated else
                "baseline %.3f m PLACEHOLDER — UNCALIBRATED" % self.baseline_m)
        if self._ext is None:
            return "extrinsics pending (%d/%d pairs); %s" % (
                len(self._calib), self.calib_pairs, base)
        return ("extrinsics self-estimated from %d pairs (t scatter %s deg); %s"
                % (self.calib_pairs, self.stats["calib_scatter_deg"], base))

    def feed(self, frame, meta=None, t=None, name=None):
        # mono entry point degrades to an unpaired stereo feed, so a caller
        # holding "an engine" never has to care which one it got.
        return self.feed_pair(frame, None, meta=meta, t=t, name=name)

    # -- feed --------------------------------------------------------------

    def feed_pair(self, f1, f2=None, meta=None, t=None, name=None):
        """One synchronized pair (f2=None when the cam2 half is missing).
        Same None semantics as VslamEngine.feed."""
        t0 = time.monotonic()
        err = self.backend.ensure()
        if err:
            self._status = "unavailable: %s" % err
            return None
        data, name, t = self._read(f1, name, t)
        self.stats["frames"] += 1
        if data is None:
            return None
        if data[:2] == b"\xff\xd8" and not data.rstrip(b"\0").endswith(b"\xff\xd9"):
            return self._corrupt(name, "truncated (no JPEG EOI)")
        gray = self.backend.decode(data)
        if gray is None:
            return self._corrupt(name, "decode failed")
        kps, desc = self.backend.detect(gray)
        if desc is None or len(kps) < 12:
            return self._lost2(name, "only %d features" % len(kps))
        half = self._load_half(f2) if f2 is not None else None
        self.stats["pairs" if half else "unpaired"] += 1
        K = self.intrinsics.k33(getattr(self.backend, "proc_scale", 1.0))
        # extrinsics: estimated ONCE from the first good pairs, then frozen —
        # the rig is rigid; re-estimating every pair would just add noise.
        if self._ext is None:
            if half:
                self._calib_step(kps, desc, half, K)
            if self._ext is None:
                self._status = ("calibrating stereo extrinsics (%d/%d pairs)"
                                % (len(self._calib), self.calib_pairs))
                return None
        # metric cloud for THIS pair (empty when unpaired/cam2 dropped)
        pts3d = self._pair_cloud(kps, desc, half, K) if half else {}
        if half:
            self.stats["stereo_points"] = len(pts3d)
        if self._kf is None:
            if not pts3d:
                self._status = "waiting for a stereo pair to bootstrap"
                return None
            return self._bootstrap2(kps, desc, pts3d, t, name, meta)
        m01 = self.backend.match(self._kf["desc"], desc)
        self.stats["last_matches"] = len(m01)
        if len(m01) < self.min_matches:
            return self._lost2(name, "%d matches" % len(m01), kps=kps,
                               desc=desc, pts3d=pts3d, t=t, meta=meta)
        par = sorted(math.hypot(self._kf["kps"][i][0] - kps[j][0],
                                self._kf["kps"][i][1] - kps[j][1])
                     for i, j in m01)[len(m01) // 2]
        if par < self.min_parallax_px:
            self._lost_streak = 0
            self.stats["still"] += 1
            self._status = ("tracking — %d keyframes, still (parallax %.1f px)"
                            % (self.stats["keyframes"], par))
            self._tick(t0)
            return None
        # 3D-2D: previous cloud points seen again in the current cam1 image
        obj, img = [], []
        for i, j in m01:
            P = self._kf["pts3d"].get(i)
            if P is not None:
                obj.append(P)
                img.append(kps[j])
        if len(obj) < self.min_pnp_points:
            return self._lost2(name, "%d PnP points" % len(obj), kps=kps,
                               desc=desc, pts3d=pts3d, t=t, meta=meta)
        rp = self.backend.pnp(obj, img, K)
        if rp is None:
            return self._lost2(name, "PnP failed", kps=kps, desc=desc,
                               pts3d=pts3d, t=t, meta=meta)
        R_rel, t_rel, inl = rp
        self.stats["last_inliers"] = len(inl)
        if len(inl) < max(self.min_inliers,
                          int(self.min_inlier_frac * len(obj))):
            return self._lost2(name, "%d/%d PnP inliers" % (len(inl), len(obj)),
                               kps=kps, desc=desc, pts3d=pts3d, t=t, meta=meta)
        up = self._advance2(R_rel, t_rel, pts3d, kps, desc, t, name, meta,
                            len(m01), len(inl))
        self._tick(t0)
        return up

    # -- internals ---------------------------------------------------------

    def _load_half(self, f2):
        """cam2 half -> (kps2, desc2) or None. A bad cam2 frame (unreadable,
        truncated, featureless) demotes the pair to unpaired rather than
        poisoning the cam1 chain."""
        try:
            if isinstance(f2, (bytes, bytearray)):
                d = bytes(f2)
            else:
                with open(os.fspath(f2), "rb") as fh:
                    d = fh.read()
        except OSError:
            self.stats["cam2_dropped"] += 1
            return None
        if d[:2] == b"\xff\xd8" and not d.rstrip(b"\0").endswith(b"\xff\xd9"):
            self.stats["cam2_dropped"] += 1
            return None
        g = self.backend.decode(d)
        if g is None:
            self.stats["cam2_dropped"] += 1
            return None
        kps2, desc2 = self.backend.detect(g)
        if desc2 is None or len(kps2) < 12:
            self.stats["cam2_dropped"] += 1
            return None
        return kps2, desc2

    def _calib_step(self, kps, desc, half, K):
        kps2, desc2 = half
        m12 = self.backend.match(desc, desc2)
        if len(m12) < self.min_matches:
            return
        p1 = [kps[i] for i, _ in m12]
        p2 = [kps2[j] for _, j in m12]
        need = max(self.min_inliers, int(self.min_inlier_frac * len(m12)))
        rp = self.backend.rel_pose(p1, p2, K)
        if rp is None or len(rp[2]) < need:
            # a flat seafloor is homography-degenerate for the 5-point
            # solver (measured: 3/1011 inliers on a planar pair) — fall back
            # to homography decomposition, which is exact on such scenes.
            rp = self.backend.rel_pose_h(p1, p2, K)
        if rp is None or len(rp[2]) < need:
            return
        R, tu, inl = rp
        self._calib.append((R, tu, len(inl)))
        if len(self._calib) >= self.calib_pairs:
            self._finalize_extrinsics()

    def _finalize_extrinsics(self):
        """Medoid over the candidates' translation directions: on a rigid rig
        every same-instant pair must agree (stereo_check.py's whole premise),
        so picking the candidate closest to all the others rejects one bad
        estimate without averaging rotation matrices. The mean angular
        distance is kept as calib_scatter_deg — a big number means the
        pairing, the sync, or the rig is not what we think it is."""
        def ang(u, v):
            d = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v))))
            return math.degrees(math.acos(d))
        cost, k = min((sum(ang(c[1], o[1]) for o in self._calib), i)
                      for i, c in enumerate(self._calib))
        R, tu, _ = self._calib[k]
        self.stats["calib_scatter_deg"] = round(
            cost / max(1, len(self._calib) - 1), 2)
        self._ext = (R, [v * self.baseline_m for v in tu])

    def _pair_cloud(self, kps, desc, half, K):
        """Metric 3D points in the CURRENT cam1 frame keyed by cam1 keypoint
        index — the map the next frame's PnP consumes."""
        kps2, desc2 = half
        m12 = self.backend.match(desc, desc2)
        if len(m12) < 8:
            return {}
        R12, t12 = self._ext
        tri = self.backend.triangulate([kps[i] for i, _ in m12],
                                       [kps2[j] for _, j in m12], K, R12, t12)
        return {i: P for (i, _), P in zip(m12, tri) if P is not None}

    def _set_skf(self, kps, desc, pts3d, t, name, meta):
        # the kf carries its own pose: unpaired frames advance the RUNNING
        # pose while the cloud (and its anchor pose) stays put, and PnP
        # results are relative to the cloud's frame, not the running one.
        self._kf = {"kps": kps, "desc": desc, "pts3d": pts3d, "t": t,
                    "file": name, "gps": _meta_gps(meta),
                    "pose_R": [row[:] for row in self._R],
                    "pose_C": list(self._C)}

    def _bootstrap2(self, kps, desc, pts3d, t, name, meta):
        self._set_skf(kps, desc, pts3d, t, name, meta)
        self.segments.append({"start_t": t, "start_file": name, "keyframes": 1})
        self.stats["keyframes"] += 1
        self._status = self._track_status("origin %s" % name)
        return self._pose_update(t, name, 0, 0, meta)

    def _lost2(self, name, why, kps=None, desc=None, pts3d=None, t=None,
               meta=None):
        self.stats["lost"] += 1
        self._lost_streak += 1
        # reinit needs a frame that can anchor a NEW cloud: without pts3d
        # (unpaired, cam2 dropped) there is nothing for the next PnP to chain
        # against, so keep counting until a paired frame comes by.
        if (self._lost_streak >= self.reinit_after and pts3d
                and self._kf is not None):
            self.stats["reinits"] += 1
            self._lost_streak = 0
            self._set_skf(kps, desc, pts3d, t, name, meta)
            self.segments.append({"start_t": t, "start_file": name,
                                  "keyframes": 0})
            self._status = ("tracking — %d keyframes, reinit segment %d at %s"
                            % (self.stats["keyframes"], len(self.segments),
                               name))
            return None
        self._status = "lost (%d since keyframe: %s)" % (self._lost_streak, why)
        return None

    def _advance2(self, R_rel, t_rel, pts3d, kps, desc, t, name, meta,
                  n_matches, n_inl):
        # PnP convention: x_cur = R X_kf + t, already METRIC. Compose off the
        # KF's STORED pose, never the running pose — see _set_skf.
        Rt = _mat_t(R_rel)
        step_cam = [-v for v in _mat_vec(Rt, t_rel)]
        kfR, kfC = self._kf["pose_R"], self._kf["pose_C"]
        step_w = _mat_vec(kfR, step_cam)
        self._C = [c + s for c, s in zip(kfC, step_w)]
        self._R = _mat_mul(kfR, Rt)
        gps_now = _meta_gps(meta)
        if pts3d:
            # landmarks: this pair's cloud is in the CURRENT cam1 frame
            for P in pts3d.values():
                pw = _mat_vec(self._R, P)
                self.landmarks.append([pw[0] + self._C[0], pw[1] + self._C[1],
                                       pw[2] + self._C[2]])
            if len(self.landmarks) > LANDMARK_CAP:
                self.landmarks = self.landmarks[::2]
            # GPS cross-check between consecutive CLOUD keyframes: both are
            # chord distances over the same instants, so the ratio is fair.
            if gps_now is not None and self._kf["gps"] is not None:
                self._gps_len += math.hypot(gps_now[0] - self._kf["gps"][0],
                                            gps_now[1] - self._kf["gps"][1])
                self._vo_len += math.sqrt(sum((a - b) ** 2
                                              for a, b in zip(self._C, kfC)))
                if self._gps_len >= 2.0:
                    self.stats["gps_ratio"] = round(
                        self._vo_len / self._gps_len, 3)
            self._set_skf(kps, desc, pts3d, t, name, meta)
        self._lost_streak = 0
        self.segments[-1]["keyframes"] += 1
        self.stats["keyframes"] += 1
        self._status = self._track_status(None)
        return self._pose_update(t, name, n_matches, n_inl, meta)

    def _track_status(self, extra):
        b = ("baseline %.3f m" % self.baseline_m if self.baseline_calibrated
             else "baseline UNCALIBRATED %.2f m" % self.baseline_m)
        s = ("tracking — %d keyframes, stereo metric (%s)"
             % (self.stats["keyframes"], b))
        return s + (", " + extra if extra else "")


# ---------------------------------------------------------------------------
# runner — tails a run directory as review frames trickle in

# Cam1_20260827_024128.48.jpg, plus run.py's collision suffix _N. The fixed-
# width timestamp makes plain string order chronological order, which the
# runner leans on for both sorting and the out-of-order gate.
FNAME_RE = re.compile(r"^Cam(\d+)_(\d{8})_(\d{6})\.(\d{2})(?:_\d+)?\.jpg$")


def _fname_epoch(name):
    m = FNAME_RE.match(name or "")
    if not m:
        return None
    try:
        st = time.strptime(m.group(2) + m.group(3), "%Y%m%d%H%M%S")
    except ValueError:
        return None
    # filenames are written from the presentation timebase in UTC (run.py
    # formats with gmtime), so decode with timegm, not mktime.
    return calendar.timegm(st) + int(m.group(4)) / 100.0


def _decimate(seq, cap):
    """Even stride keeping first and last — a live overlay cares most about
    the ends of the track."""
    if len(seq) <= cap:
        return list(seq)
    step = len(seq) / float(cap)
    out = [seq[int(i * step)] for i in range(cap - 1)]
    out.append(seq[-1])
    return out


def _fit_similarity_2d(src, dst):
    """Least-squares 2D similarity dst ~ a*src + b with a, b complex
    (rotation+scale+translation, no reflection) — places the VO (x, z) track
    onto UTM for the map overlay without touching the VO state itself."""
    n = len(src)
    if n < 3:
        return None
    sm = [sum(p[i] for p in src) / n for i in (0, 1)]
    dm = [sum(p[i] for p in dst) / n for i in (0, 1)]
    var = num_re = num_im = 0.0
    for (sx, sy), (dx, dy) in zip(src, dst):
        sx, sy, dx, dy = sx - sm[0], sy - sm[1], dx - dm[0], dy - dm[1]
        var += sx * sx + sy * sy
        num_re += sx * dx + sy * dy          # conj(s) * d
        num_im += sx * dy - sy * dx
    if var < 1e-9:
        return None
    a_re, a_im = num_re / var, num_im / var
    tx = dm[0] - (a_re * sm[0] - a_im * sm[1])
    ty = dm[1] - (a_im * sm[0] + a_re * sm[1])
    rms = 0.0
    for (sx, sy), (dx, dy) in zip(src, dst):
        ex = a_re * sx - a_im * sy + tx - dx
        ey = a_im * sx + a_re * sy + ty - dy
        rms += ex * ex + ey * ey
    return {"a_re": a_re, "a_im": a_im, "tx": tx, "ty": ty,
            "rms_m": math.sqrt(rms / n), "n": n}


STEREO_CAL_PATH = os.path.expanduser("~/rig/stereo_calibration.json")


def load_stereo_calibration(path=STEREO_CAL_PATH):
    """The Calib tab's saved checkerboard solution, or None.

    Returns {"R": 3x3, "T": [3] metres, "baseline_m": float, "K1","d1","K2",
    "d2", "rms_stereo_px", "date"} - the wire-in docs/vslam-jetson.md
    specifies: construct the stereo engine with these extrinsics frozen
    (baseline_calibrated=True) instead of the self-estimate."""
    try:
        with open(path) as fh:
            doc = json.load(fh)
        if not (isinstance(doc, dict) and doc.get("R") and doc.get("T")
                and doc.get("baseline_m")):
            return None
        return doc
    except (OSError, ValueError):
        return None


class VslamRunner:
    """Threaded watcher: feeds a run dir's review JPEGs to an engine in
    timestamp order as they appear, and keeps the JSON-safe state a rigd
    /api/vslam endpoint will serve (docs/vslam-jetson.md).

    mode: 'mono' (cam only), 'stereo' (cam + pair_cam, frames associated by
    capture instant from the filenames), or 'auto' — stereo when the pair
    dir exists and >50% of the first frames find a same-instant partner,
    mono otherwise. Pairing tolerance is half the median inter-frame gap
    (the two halves of one fire land well inside that; the NEXT frame is a
    full gap away), overridable with pair_tol_s. In follow mode a lone cam1
    frame waits one grace period for its cam2 half before being fed
    unpaired — the halves cross USB independently and rarely land together.
    """

    def __init__(self, engine=None, poll_s=1.0, on_update=None, mode="mono",
                 pair_cam="cam2", pair_tol_s=None, baseline_m=0.30,
                 baseline_calibrated=False, calib_pairs=5, engine_kw=None):
        self.poll_s = poll_s
        self.on_update = on_update            # callable(PoseUpdate) — CLI print
        self.mode = mode                      # as requested
        self.pair_cam = pair_cam
        self.pair_tol_s = pair_tol_s
        self.baseline_m = baseline_m
        self.baseline_calibrated = baseline_calibrated
        self.calib_pairs = calib_pairs
        self.engine_kw = dict(engine_kw or {})
        self.engine = engine
        self.decided = None                   # 'mono'|'stereo' once chosen
        if engine is not None:
            # an injected engine is authoritative — its class IS the mode
            self.decided = getattr(engine, "mode", "mono")
        elif mode in ("mono", "stereo"):
            self.decided = mode
            self.engine = self._make_engine(mode)
        self.run_dir = self.cam = None
        self.trajectory = []
        self.watch = {"files_seen": 0, "fed": 0, "out_of_order": 0,
                      "duplicates": 0, "pending": 0, "fps": 0.0,
                      "cam2_orphans": 0, "pairs_fed": 0}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._seen = {}                       # cam -> set of names
        self._pending = {}                    # cam -> {name: size last scan}
        self._q1 = []                         # ready cam frames, name-sorted
        self._ready_at = {}                   # cam name -> monotonic ready t
        self._ready2 = {}                     # pair_cam name -> epoch
        self._e1_hist = []                    # recent cam epochs (cadence)
        self._last_fed = ""                   # lexicographic == chronological
        self._flight = {}
        self._flight_sig = None
        self._fed_times = []

    def _make_engine(self, mode):
        if mode == "stereo":
            # A saved checkerboard calibration (the UI's Calib tab writes
            # ~/rig/stereo_calibration.json) beats both the placeholder
            # baseline and the self-estimated extrinsics - freeze it in,
            # unless the caller already supplied a calibrated baseline.
            cal = None if self.baseline_calibrated else \
                load_stereo_calibration()
            if cal:
                eng = StereoVslamEngine(baseline_m=float(cal["baseline_m"]),
                                        baseline_calibrated=True,
                                        calib_pairs=self.calib_pairs,
                                        **self.engine_kw)
                eng._ext = ([list(map(float, row)) for row in cal["R"]],
                            [float(v) for v in cal["T"]])
                return eng
            return StereoVslamEngine(baseline_m=self.baseline_m,
                                     baseline_calibrated=self.baseline_calibrated,
                                     calib_pairs=self.calib_pairs,
                                     **self.engine_kw)
        return VslamEngine(**self.engine_kw)

    # -- lifecycle ---------------------------------------------------------

    def start(self, run_dir, cam="cam1"):
        if self._thread is not None:
            raise RuntimeError("runner already started")
        self.run_dir, self.cam = os.path.abspath(run_dir), cam
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="vslam-%s" % cam)
        self._thread.start()

    def stop(self, timeout=10.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def run_once(self, run_dir, cam="cam1"):
        """Process everything already on disk, in order, on the caller's
        thread — the CLI's non-follow mode. No settle delay and no pairing
        grace: a finished run's files are complete by definition."""
        self.run_dir, self.cam = os.path.abspath(run_dir), cam
        self._poll(settle=False, follow=False)

    # -- watcher -----------------------------------------------------------

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._poll(settle=True, follow=True)
            except Exception as e:  # noqa: BLE001 - a watcher that dies
                # silently leaves the operator staring at a frozen track with
                # no hint; surface the error in status instead and keep going.
                with self._lock:
                    self.watch["error"] = "%s: %s" % (type(e).__name__, e)
            self._stop.wait(self.poll_s)

    def _refresh_flight(self):
        """flight_log.csv (the primary cam's) grows during the run; re-parse
        when its size/mtime move. Whole-file re-read: a full survey is a few
        thousand short rows, well under the cost of one ORB pass."""
        path = os.path.join(self.run_dir, self.cam, "flight_log.csv")
        try:
            st = os.stat(path)
        except OSError:
            return
        sig = (st.st_size, st.st_mtime_ns)
        if sig == self._flight_sig:
            return
        try:
            with open(path, newline="") as fh:
                rows = {r.get("filename"): r for r in csv.DictReader(fh)}
        except (OSError, csv.Error):
            return                            # mid-write; next poll gets it
        self._flight_sig = sig
        self._flight = rows

    def _scan_cam(self, cam, settle):
        """Newly READY files of one cam dir (None = dir absent, distinct
        from empty). run.py writes straight to the final name, so a file
        whose size still moves between polls is mid-write: ready = same
        nonzero size on two consecutive scans (~one poll of extra latency;
        the EOI check in feed() backstops the rest)."""
        cam_num = "".join(c for c in cam if c.isdigit())
        prefix = "Cam%s_" % cam_num
        seen = self._seen.setdefault(cam, set())
        pend = self._pending.setdefault(cam, {})
        try:
            names = os.listdir(os.path.join(self.run_dir, cam))
        except OSError:
            return None
        out = []
        for n in names:
            if not n.startswith(prefix) or n in seen or not FNAME_RE.match(n):
                continue
            try:
                size = os.path.getsize(os.path.join(self.run_dir, cam, n))
            except OSError:
                continue
            if settle:
                last = pend.get(n)
                pend[n] = size
                if last != size or size == 0:
                    continue
                del pend[n]
            seen.add(n)
            out.append(n)
        with self._lock:
            self.watch["files_seen"] += len(out)
            self.watch["pending"] = sum(len(p) for p in self._pending.values())
        return out

    def _tol(self):
        """Pairing tolerance: half the observed median inter-frame gap, so a
        same-instant partner always qualifies and the NEXT frame never does.
        0.25 s (half the 2 Hz default cadence) until enough gaps are seen."""
        if self.pair_tol_s:
            return self.pair_tol_s
        es = sorted(self._e1_hist[-40:])
        gaps = [b - a for a, b in zip(es, es[1:]) if 1e-3 < b - a < 30]
        if not gaps:
            return 0.25
        return max(0.02, 0.5 * sorted(gaps)[len(gaps) // 2])

    def _poll(self, settle, follow):
        self._refresh_flight()
        new1 = self._scan_cam(self.cam, settle)
        if new1 is None:
            return                            # run/cam dir not created yet
        want2 = (self.decided == "stereo"
                 or (self.decided is None and self.mode == "auto"))
        new2 = self._scan_cam(self.pair_cam, settle) if want2 else None
        now = time.monotonic()
        for n in sorted(new1):
            if n <= self._last_fed:
                # a straggler older than what the engine already tracked
                # cannot be inserted into an incremental chain — count it
                # and move on (the settle delay makes this rare at 2 Hz).
                with self._lock:
                    self.watch["out_of_order"] += 1
                continue
            self._q1.append(n)
            self._ready_at[n] = now
            e = _fname_epoch(n)
            if e is not None:
                self._e1_hist.append(e)
        self._q1.sort()
        self._e1_hist = self._e1_hist[-60:]
        for n in sorted(new2 or []):
            e = _fname_epoch(n)
            if e is not None:
                self._ready2[n] = e
        if self.decided is None:
            self._decide(pair_dir_exists=new2 is not None, follow=follow)
            if self.decided is None:
                return                        # keep queueing until decidable
        if self.decided == "mono":
            for n in self._q1:
                if self._stop.is_set() and follow:
                    return
                self._feed(n, None)
            self._q1.clear()
            self._ready_at.clear()
            return
        # stereo: pair each cam frame with the nearest-instant ready partner
        tol = self._tol()
        grace = max(1.0, 2 * self.poll_s) if follow else 0.0
        while self._q1:
            if self._stop.is_set() and follow:
                return
            n1 = self._q1[0]
            e1 = _fname_epoch(n1) or 0.0
            partner, best = None, tol + 1.0
            for n2, e2 in self._ready2.items():
                d = abs(e2 - e1)
                if d <= tol and d < best:
                    partner, best = n2, d
            if partner is None and (now - self._ready_at.get(n1, now)) < grace:
                break         # wait a beat for the other half; keep ORDER
            self._q1.pop(0)
            self._ready_at.pop(n1, None)
            if partner is not None:
                self._ready2.pop(partner)
            self._feed(n1, partner)
            # cam2 frames that can no longer pair with anything upcoming
            dead = [n2 for n2, e2 in self._ready2.items() if e2 < e1 - tol]
            for n2 in dead:
                self._ready2.pop(n2)
                with self._lock:
                    self.watch["cam2_orphans"] += 1

    def _decide(self, pair_dir_exists, follow):
        """auto: stereo when the pair dir exists and >50% of the queued
        frames have a same-instant partner. In follow mode wait for at least
        4 frames so one straggler cannot flip the answer; a one-shot pass
        has everything on disk already and decides immediately."""
        if not pair_dir_exists:
            if self._q1 or not follow:
                self._choose("mono", "no %s dir" % self.pair_cam)
            return
        if follow and len(self._q1) < 4:
            return
        if not self._q1:
            if not follow:
                self._choose("mono", "no frames at all")
            return
        tol = self._tol()
        paired = sum(1 for n1 in self._q1
                     if any(abs(e2 - (_fname_epoch(n1) or 0.0)) <= tol
                            for e2 in self._ready2.values()))
        rate = paired / len(self._q1)
        self._choose("stereo" if rate > 0.5 else "mono",
                     "pairing %.0f%% over %d frames" % (100 * rate,
                                                        len(self._q1)))

    def _choose(self, mode, why):
        self.decided = mode
        with self._lock:
            self.watch["mode_reason"] = why
        if self.engine is None:
            self.engine = self._make_engine(mode)

    def _feed(self, name, partner):
        meta = self._flight.get(name)
        p1 = os.path.join(self.run_dir, self.cam, name)
        p2 = (os.path.join(self.run_dir, self.pair_cam, partner)
              if partner else None)
        with self._lock:
            if self.decided == "stereo":
                up = self.engine.feed_pair(p1, p2, meta=meta)
                if partner:
                    self.watch["pairs_fed"] += 1
            else:
                up = self.engine.feed(p1, meta=meta)
            self._last_fed = name
            self.watch["fed"] += 1
            now = time.monotonic()
            self._fed_times = [t for t in self._fed_times if now - t < 20.0]
            self._fed_times.append(now)
            self.watch["fps"] = round(len(self._fed_times) / 20.0, 2)
            if up is not None:
                e = up.as_dict()
                del e["keyframe"]             # always True in the trajectory
                self.trajectory.append(e)
        if up is not None and self.on_update:
            self.on_update(up)

    # -- state out ---------------------------------------------------------

    def snapshot(self, traj_cap=1500, lm_cap=4000):
        """JSON-safe dict for a rigd endpoint: capped, rounded, and complete
        enough to draw the overlay (trajectory + cloud + UTM placement).
        json.dumps(snapshot()) stays well under a poll-friendly ~1 MB."""
        with self._lock:
            eng = self.engine
            if eng is None:                   # auto, not enough frames yet
                return {"status": "auto: undecided (waiting for frames)",
                        "mode": None, "run_dir": self.run_dir,
                        "cam": self.cam, "stats": dict(self.watch),
                        "segments": [], "trajectory": [], "landmarks": [],
                        "utm_align": None}
            traj = _decimate(self.trajectory, traj_cap)
            lms = _decimate(eng.landmarks, lm_cap)
            out = {
                "status": eng.status,
                "mode": self.decided,
                "backend": getattr(eng.backend, "name", "?"),
                "run_dir": self.run_dir, "cam": self.cam,
                "scale": round(eng.scale, 4),
                "scale_source": eng.scale_source,
                "stats": dict(eng.stats, **self.watch),
                "segments": [dict(s) for s in eng.segments],
                "trajectory": [
                    {k: (round(v, 4) if isinstance(v, float) else v)
                     for k, v in e.items() if k != "gps"}
                    for e in traj],
                "landmarks": [[round(v, 3) for v in p] for p in lms],
            }
            if eng.mode == "stereo":
                out["baseline_m"] = eng.baseline_m
                out["baseline_calibrated"] = eng.baseline_calibrated
                out["calib"] = eng.calib_status
                p, u = eng.stats["pairs"], eng.stats["unpaired"]
                out["pairing_rate"] = (round(p / (p + u), 3) if p + u
                                       else None)
            # place VO (x, z) onto (xutm, yutm) when the boat has genuinely
            # moved; a dock run (single fix) never passes the spread gate.
            # With stereo the fitted |a| doubles as a baseline check: it
            # should sit near 1.0, and its deviation ~= the baseline error.
            pts = [(e["x"], e["z"], e["gps"]) for e in self.trajectory
                   if e.get("gps")]
        out["utm_align"] = None
        if len(pts) >= 3:
            es = [g[0] for _, _, g in pts]
            ns = [g[1] for _, _, g in pts]
            if max(es) - min(es) > 1.0 or max(ns) - min(ns) > 1.0:
                out["utm_align"] = _fit_similarity_2d(
                    [(x, z) for x, z, _ in pts], [g for _, _, g in pts])
        out["stats"]["proc_ms"] = round(out["stats"]["proc_ms"], 1)
        return out


# ---------------------------------------------------------------------------
# outputs

def write_ply(path, snapshot):
    """ASCII PLY of the landmark cloud (grey) and trajectory (orange), for
    MeshLab/CloudCompare on the bench — no viewer dependency in the field."""
    lms = snapshot.get("landmarks") or []
    traj = snapshot.get("trajectory") or []
    with open(path, "w") as fh:
        fh.write("ply\nformat ascii 1.0\ncomment wildsync vslam v1\n"
                 "element vertex %d\n" % (len(lms) + len(traj)))
        fh.write("property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                 "end_header\n")
        for p in lms:
            fh.write("%.3f %.3f %.3f 170 170 170\n" % (p[0], p[1], p[2]))
        for e in traj:
            fh.write("%.3f %.3f %.3f 255 96 32\n" % (e["x"], e["y"], e["z"]))


# ---------------------------------------------------------------------------
# cli

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="incremental VO over a run's review JPEGs")
    ap.add_argument("run_dir")
    ap.add_argument("--cam", default="cam1")
    ap.add_argument("--mode", choices=["auto", "mono", "stereo"],
                    default="auto",
                    help="auto = stereo when cam2 frames pair up, else mono")
    ap.add_argument("--baseline", type=float, default=0.30,
                    help="stereo baseline in metres — a rig constant that is "
                         "currently UNMEASURED; 0.30 is a placeholder and the "
                         "output says so until --baseline-calibrated")
    ap.add_argument("--baseline-calibrated", action="store_true",
                    help="assert --baseline is a measured value")
    ap.add_argument("--follow", action="store_true",
                    help="keep tailing the dir for new frames (Ctrl-C to stop)")
    ap.add_argument("--json", help="write the final snapshot here")
    ap.add_argument("--ply", help="write landmark cloud + trajectory here")
    ap.add_argument("--focal", type=float, default=None,
                    help="focal length in px at 1616 wide (default: from --hfov)")
    ap.add_argument("--hfov", type=float, default=59.0)
    ap.add_argument("--port-factor", type=float, default=1.0,
                    help="1.33 for a flat underwater port, 1.0 in air/dome")
    ap.add_argument("--poll", type=float, default=1.0)
    ap.add_argument("--pre", choices=["flat", "none"], default="flat",
                    help="'flat' = illumination flatten + CLAHE (turbid water)")
    ap.add_argument("--proc-scale", type=float, default=0.5,
                    help="feature-work downscale (0.5 proven in stereo_check)")
    ap.add_argument("--features", type=int, default=1500,
                    help="ORB feature budget; stereo cross-camera matching "
                         "on real water frames roughly doubles per 2x here "
                         "at near-zero CPU cost (measured 64->163 matches "
                         "from 1500->5000 at +14 ms/pair)")
    a = ap.parse_args(argv)

    intr = Intrinsics(focal_px=a.focal, hfov_deg=a.hfov,
                      port_factor=a.port_factor)
    r = VslamRunner(poll_s=a.poll, mode=a.mode, baseline_m=a.baseline,
                    baseline_calibrated=a.baseline_calibrated,
                    engine_kw=dict(
                        intrinsics=intr,
                        backend=make_backend("cpu", n_features=a.features,
                                             proc_scale=a.proc_scale,
                                             preprocess=a.pre)))

    def show(u):
        kf = r.engine.stats["keyframes"] if r.engine else 0
        print("kf %-4d %-32s matches %4d inliers %4d  "
              "pos (%+8.2f %+8.2f %+8.2f)  scale %s" %
              (kf, u.file, u.matches, u.inliers,
               u.x, u.y, u.z, u.scale_source))

    r.on_update = show
    if a.follow:
        r.start(a.run_dir, cam=a.cam)
        print("following %s/%s (poll %.1fs, Ctrl-C to stop)"
              % (a.run_dir, a.cam, a.poll))
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        r.stop()
    else:
        r.run_once(a.run_dir, cam=a.cam)

    snap = r.snapshot(traj_cap=10 ** 6, lm_cap=10 ** 6)
    s = snap["stats"]
    if "frames" not in s:                     # auto never decided (no frames)
        print("done: %s" % snap["status"])
        return 0
    print("done: mode=%s, %d frames -> %d keyframes in %d segment(s); "
          "%d still, %d lost, %d corrupt, %d reinits; scale=%s; %s"
          % (snap.get("mode"), s["frames"], s["keyframes"],
             len(snap["segments"]), s["still"], s["lost"], s["corrupt"],
             s["reinits"], snap["scale_source"], snap["status"]))
    if snap.get("mode") == "stereo":
        print("stereo: pairing %s (%d pairs, %d unpaired, %d cam2 dropped, "
              "%d orphans); %s"
              % (snap.get("pairing_rate"), s["pairs"], s["unpaired"],
                 s["cam2_dropped"], s.get("cam2_orphans", 0), snap["calib"]))
        if s.get("gps_ratio") is not None:
            print("gps cross-check: stereo/GPS track length ratio %.3f "
                  "(1.0 = baseline correct)" % s["gps_ratio"])
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(snap, fh)
        print("wrote %s" % a.json)
    if a.ply:
        write_ply(a.ply, snap)
        print("wrote %s (%d landmarks + %d poses)"
              % (a.ply, len(snap["landmarks"]), len(snap["trajectory"])))
    if snap["status"].startswith("unavailable:"):
        print(snap["status"], file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())

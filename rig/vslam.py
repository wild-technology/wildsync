#!/usr/bin/env python3
"""vslam — live camera-derived trajectory from the review JPEGs of a run.

During a survey the nodes trickle ~200 KB 1616x1080 review JPEGs onto the host
(<run>/cam1/Cam1_YYYYMMDD_hhmmss.ss.jpg) at ~2 Hz, alongside a flight_log.csv
row per frame with nav state (lat/long/xutm/yutm/heading/pitch/roll). This
module turns that trickle into an incremental monocular visual-odometry
trajectory the operator can see live: ORB features, ratio-tested Hamming
matches against the previous keyframe, essential matrix + RANSAC, pose chain,
and a sparse triangulated landmark cloud. Monocular VO has no scale; when the
flight_log carries UTM the scale of each new trajectory segment is fitted to
the GPS baseline over a sliding window, otherwise the chain is unit-norm and
the status says so.

Runs today on the macOS host (CPU); it is finalized on the Jetson Orin Nano,
where the same VslamEngine can be handed a CUDA backend — everything that
touches cv2/numpy lives behind the small backend interface (CpuOrbBackend) for
exactly that swap. See docs/vslam-jetson.md for the finalization plan, the
calibration procedure and the rigd /api/vslam integration spec.

cv2/numpy may be absent on a fresh host: imports are lazy and every entry
point degrades to a clean "unavailable: <reason>" status instead of crashing.
Install on the Mac with `pip3 install opencv-python-headless numpy`; the
Jetson gets cv2 from JetPack/apt (python3-opencv), no pip build.

    python3 rig/vslam.py <run_dir> [--cam cam1] [--follow]
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
      triangulate(p1, p2, K, R, t)-> [[x, y, z]...] in the FIRST camera frame,
                                      unit-|t| scale, cheirality-filtered
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
        # the cloud kilometres out once scaled.
        return [[float(x), float(y), float(z)] for x, y, z in X
                if 0.05 < z < 500.0]


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


class VslamRunner:
    """Threaded watcher: feeds a run dir's review JPEGs to a VslamEngine in
    timestamp order as they appear, and keeps the JSON-safe state a rigd
    /api/vslam endpoint will serve (docs/vslam-jetson.md)."""

    def __init__(self, engine=None, poll_s=1.0, on_update=None):
        self.engine = engine or VslamEngine()
        self.poll_s = poll_s
        self.on_update = on_update            # callable(PoseUpdate) — CLI print
        self.run_dir = self.cam = None
        self.trajectory = []
        self.watch = {"files_seen": 0, "fed": 0, "out_of_order": 0,
                      "duplicates": 0, "pending": 0, "fps": 0.0}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._seen = set()
        self._pending = {}                    # name -> size at last scan
        self._last_fed = ""                   # lexicographic == chronological
        self._flight = {}
        self._flight_sig = None
        self._fed_times = []

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
        thread — the CLI's non-follow mode. No settle delay: a finished run's
        files are complete by definition."""
        self.run_dir, self.cam = os.path.abspath(run_dir), cam
        self._poll(settle=False)

    # -- watcher -----------------------------------------------------------

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._poll(settle=True)
            except Exception as e:  # noqa: BLE001 - a watcher that dies
                # silently leaves the operator staring at a frozen track with
                # no hint; surface the error in status instead and keep going.
                with self._lock:
                    self.watch["error"] = "%s: %s" % (type(e).__name__, e)
            self._stop.wait(self.poll_s)

    def _cam_dir(self):
        return os.path.join(self.run_dir, self.cam)

    def _refresh_flight(self):
        """flight_log.csv grows during the run; re-parse when its size/mtime
        move. Whole-file re-read: a full survey is a few thousand short rows,
        well under the cost of one ORB pass."""
        path = os.path.join(self._cam_dir(), "flight_log.csv")
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

    def _poll(self, settle):
        self._refresh_flight()
        cam_num = "".join(c for c in self.cam if c.isdigit())
        prefix = "Cam%s_" % cam_num
        try:
            names = os.listdir(self._cam_dir())
        except OSError:
            return                            # run dir not created yet
        fresh = []
        for n in names:
            if not n.startswith(prefix) or n in self._seen or not FNAME_RE.match(n):
                continue
            path = os.path.join(self._cam_dir(), n)
            try:
                size = os.path.getsize(path)
            except OSError:
                continue
            if settle:
                # run.py writes straight to the final name, so a file whose
                # size still moves between polls is mid-write. Ready = same
                # nonzero size on two consecutive scans (~one poll of extra
                # latency; the EOI check in feed() backstops the rest).
                last = self._pending.get(n)
                self._pending[n] = size
                if last != size or size == 0:
                    continue
                del self._pending[n]
            fresh.append(n)
        with self._lock:
            self.watch["pending"] = len(self._pending)
        for n in sorted(fresh):
            if self._stop.is_set() and settle:
                return
            self._ingest(n)

    def _ingest(self, name):
        self._seen.add(name)
        with self._lock:
            self.watch["files_seen"] += 1
        if name <= self._last_fed:
            # a straggler older than what the engine has already tracked
            # cannot be inserted into an incremental chain — count it and
            # move on (the settle delay makes this rare at 2 Hz).
            with self._lock:
                self.watch["out_of_order"] += 1
            return
        path = os.path.join(self._cam_dir(), name)
        meta = self._flight.get(name)
        with self._lock:
            up = self.engine.feed(path, meta=meta)
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
            traj = _decimate(self.trajectory, traj_cap)
            lms = _decimate(eng.landmarks, lm_cap)
            out = {
                "status": eng.status,
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
            # place VO (x, z) onto (xutm, yutm) when the boat has genuinely
            # moved; a dock run (single fix) never passes the spread gate.
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
    a = ap.parse_args(argv)

    intr = Intrinsics(focal_px=a.focal, hfov_deg=a.hfov,
                      port_factor=a.port_factor)
    eng = VslamEngine(intrinsics=intr,
                      backend=make_backend("cpu", proc_scale=a.proc_scale,
                                           preprocess=a.pre))

    def show(u):
        print("kf %-4d %-32s matches %4d inliers %4d  "
              "pos (%+8.2f %+8.2f %+8.2f)  scale %s" %
              (eng.stats["keyframes"], u.file, u.matches, u.inliers,
               u.x, u.y, u.z, u.scale_source))

    r = VslamRunner(engine=eng, poll_s=a.poll, on_update=show)
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
    print("done: %d frames -> %d keyframes in %d segment(s); "
          "%d still, %d lost, %d corrupt, %d reinits; scale=%s; %s"
          % (s["frames"], s["keyframes"], len(snap["segments"]), s["still"],
             s["lost"], s["corrupt"], s["reinits"], snap["scale_source"],
             snap["status"]))
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

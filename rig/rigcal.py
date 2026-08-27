"""Rig calibration — stereo geometry from a checkerboard, and per-camera IMU
references. Backend for the UI's Calib tab.

WHY this exists: the pair shoots through corrected underwater optics, so the
absolute focal length is NOT known from any datasheet - the dome changes it,
and it changes again between air and water. Everything downstream that wants
metres (stereo VSLAM, photogrammetry sanity checks) therefore depends on a
calibration measured through the actual optics, ideally in the actual medium.
Nothing here assumes a focal prior: intrinsics come entirely from the board.

STEREO SESSION FLOW
    start(cols, rows, square_mm[, baseline_mm]) -> capture() per pose ->
    status() drives the UI's live guidance (coverage grids, scale/tilt
    diversity, what is still missing) -> compute() -> save().

Captures ride the rig's own synchronized fire path (RunManager.capture_once),
so each pair is a genuine simultaneous stereo exposure - a handheld board is
fine. Frames are read back from the node spools exactly like the Review tab
reads them; nothing here touches the cards.

The saved artifact (~/rig/stereo_calibration.json) is the wire-in the stereo
VSLAM documents (docs/vslam-jetson.md): K/dist per camera, R, T, baseline.
vslam.load_stereo_calibration() consumes it.

IMU CALIBRATION
    Each camera now carries its own IMU (cam1: YB on /imu, cam2: Olive on
    /imu2). calibrate_imu() samples a still window, refuses if the rig moved,
    and records gyro bias, accel norm, and the mounting attitude reference
    (the pitch/roll the IMU reports when the rig is level - the number every
    later attitude is read against). "both" also records the relative
    cam1->cam2 orientation seed the fusion filter needs (docs/olive-imu.md).

cv2/numpy are imported lazily and their absence degrades to a clear
"unavailable" answer - same discipline as rig/vslam.py.
"""

import json
import math
import os
import threading
import time

import rigcore
from rigcore import http_json, http_bytes

CAL_DIR = os.path.expanduser("~/rig")
STEREO_PATH = os.path.join(CAL_DIR, "stereo_calibration.json")
IMU_PATH = os.path.join(CAL_DIR, "imu_calibration.json")

# Session targets - the guidance engine's definition of "enough data".
MIN_PAIRS = 12
MIN_NEAR = 2          # board wide in frame (close) - anchors focal/distortion
MIN_FAR = 2           # board small in frame (far) - anchors focal
MIN_TILTED = 3        # oblique views - separates focal from principal point
NEAR_FRAC = 0.28      # board bbox width / image width considered "near"
FAR_FRAC = 0.13
TILT_DEG = 14.0

def _shot_name(entry):
    # /api/shots entries have been a bare name, [name, size] and
    # {"name":..,"size":..} across builds - take the name from any of them.
    if isinstance(entry, dict):
        return entry.get("name")
    if isinstance(entry, (list, tuple)) and entry:
        return entry[0]
    return entry


_lock = threading.RLock()
_session = None       # the one live StereoSession (operator workflow is serial)
_imu_busy = False


def _cv():
    """(cv2, numpy) or raises RuntimeError with the honest reason."""
    try:
        import cv2
        import numpy as np
        return cv2, np
    except Exception as e:  # noqa: BLE001
        raise RuntimeError("OpenCV/numpy are not importable on this host "
                           "(%s) - pip3 install opencv-python-headless numpy"
                           % e)


def _atomic_json(path, doc):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def saved_summary():
    """What is on disk, for the tab's 'current calibration' panel."""
    out = {"stereo": None, "imu": None}
    for key, path in (("stereo", STEREO_PATH), ("imu", IMU_PATH)):
        try:
            with open(path) as fh:
                doc = json.load(fh)
            if key == "stereo":
                out[key] = {k: doc.get(k) for k in
                            ("date", "baseline_m", "rms_stereo_px",
                             "pairs_used", "pattern", "square_mm",
                             "agreement_pct", "notes")}
            else:
                out[key] = {"date": doc.get("date"),
                            "cams": sorted(k for k in doc
                                           if k.startswith("cam")),
                            "relative": doc.get("relative") is not None}
        except (OSError, ValueError):
            pass
    return out


# ---------------------------------------------------------------------------
# Stereo session
# ---------------------------------------------------------------------------
class StereoSession:
    def __init__(self, cols, rows, square_mm, baseline_mm=None, notes=""):
        cols, rows = int(cols), int(rows)
        if not (3 <= cols <= 25 and 3 <= rows <= 25) or cols == rows:
            # cols == rows is refused on purpose: a square inner-corner grid
            # is 90-degree ambiguous and silently flips corner ordering
            # between views, which poisons stereoCalibrate.
            raise ValueError("pattern must be the board's INNER corner counts,"
                             " 3-25 per side and not square (e.g. 9x6)")
        if not (1.0 <= float(square_mm) <= 500.0):
            raise ValueError("square size must be 1-500 mm (measure it - "
                             "printed boards scale)")
        self.cols, self.rows = cols, rows
        self.square_mm = float(square_mm)
        self.baseline_mm = float(baseline_mm) if baseline_mm else None
        self.notes = str(notes or "")
        self.started = time.time()
        self.pairs = []           # per capture: dict per cam + meta
        self.result = None
        self.busy = False

    # -- capture ------------------------------------------------------------
    def capture(self, rig):
        """Fire one synchronized pair, detect corners, record the pose."""
        cv2, np = _cv()
        if rig.runmgr.status().get("active"):
            raise RuntimeError("a run is recording - no calibration captures")
        if rig.drain_status().get("active"):
            raise RuntimeError("a drain is running - wait for it")
        mons = {m.name_: m for m in rig.monitors if m.is_connected()}
        if len(mons) < 2:
            raise RuntimeError("both cameras must be connected (%d are)"
                               % len(mons))
        before = {}
        for name, m in mons.items():
            names = m.shots()
            if names is None:
                raise RuntimeError("%s did not answer its shot listing" % name)
            before[name] = {_shot_name(s) for s in names}
        r = rig.runmgr.capture_once(af=False)
        if not (isinstance(r, dict) and r.get("ok", True) is not False):
            raise RuntimeError("capture failed: %s"
                               % (r or {}).get("error", "no answer"))
        rec = {"at": time.time(), "cams": {}, "skew_ms": (r or {}).get("skew_ms")}
        deadline = time.time() + 8.0
        pending = set(mons)
        while pending and time.time() < deadline:
            time.sleep(0.25)
            for name in list(pending):
                m = mons[name]
                names = m.shots() or []
                fresh = [_shot_name(s) for s in names]
                new = [n for n in fresh if n not in before[name]
                       and os.path.splitext(n)[1].lower()
                       in (".jpg", ".jpeg")]
                if not new:
                    continue
                data, err = http_bytes(
                    "http://%s:8080/shot/%s" % (m.host, sorted(new)[0]),
                    timeout=15)
                if not data:
                    raise RuntimeError("%s: could not fetch %s (%s)"
                                       % (name, new[0], err))
                rec["cams"][name] = self._detect(cv2, np, data,
                                                 sorted(new)[0])
                pending.discard(name)
        for name in pending:
            rec["cams"][name] = {"found": False, "name": None,
                                 "error": "no frame delivered within 8 s "
                                          "(PC-save on? spool reachable?)"}
        with _lock:
            self.pairs.append(rec)
        return self.pair_view(len(self.pairs) - 1)

    def _detect(self, cv2, np, jpeg, name):
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8),
                           cv2.IMREAD_GRAYSCALE)
        if img is None:
            return {"found": False, "name": name, "error": "undecodable JPEG"}
        h, w = img.shape[:2]
        pattern = (self.cols, self.rows)
        found, corners = False, None
        # SB detector first: it is the one that survives the underwater
        # contrast; the classic detector + subpix refine is the fallback for
        # older OpenCV builds.
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                img, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_EXHAUSTIVE)
        if not found:
            found, corners = cv2.findChessboardCorners(
                img, pattern,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                cv2.cornerSubPix(
                    img, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                     30, 1e-3))
        if not found or corners is None:
            return {"found": False, "name": name, "size": [w, h],
                    "error": "checkerboard not found"}
        pts = corners.reshape(-1, 2)
        # Canonicalize the 180-degree ordering ambiguity: the detector may
        # return the grid starting from either end, PER VIEW - and if cam1's
        # corner[0] is cam2's corner[-1], stereoCalibrate is fed a coherent-
        # looking but physically crossed correspondence set (caught by the
        # synthetic selftest at RMS 15 px). The flip is chosen from the ROW
        # direction's dominant image-axis sign - a rule both cameras of a
        # rigid pair answer identically for any board attitude short of a
        # 90-degree in-plane split between the views (corner-position sum
        # comparison was tried first and proved unstable for oblique boards).
        row = pts[self.cols - 1] - pts[0]
        flip = (row[0] < 0) if abs(row[0]) >= abs(row[1]) else (row[1] < 0)
        if flip:
            pts = pts[::-1].copy()
        # A board touching the frame edge is refused: a partially visible
        # board can still yield a full-count "found" grid (the detector
        # hallucinates or shifts the lattice) and one such view poisons the
        # whole solve - observed in the synthetic selftest as RMS 35 px.
        if pts[:, 0].min() < 6 or pts[:, 1].min() < 6 \
                or pts[:, 0].max() > w - 6 or pts[:, 1].max() > h - 6:
            return {"found": False, "name": name, "size": [w, h],
                    "error": "board touches the frame edge - keep it fully "
                             "inside BOTH views"}
        # Cell-pitch gate. On sub-~18 px cells the detector demonstrably
        # locks onto a phantom half-cell lattice with DUPLICATED corners
        # (measured on synthetic data: det[0]==det[1], every point off by
        # half a pitch) - data that poisons the solve while looking "found".
        # A board that far away is simply below what the optics can measure.
        import numpy as _np
        grid = _np.asarray(pts, float).reshape(self.rows, self.cols, 2)
        dx = _np.linalg.norm(_np.diff(grid, axis=1), axis=2)
        dy = _np.linalg.norm(_np.diff(grid, axis=0), axis=2)
        pitch = float(_np.median(_np.concatenate([dx.ravel(), dy.ravel()])))
        if pitch < 18.0 or float(min(dx.min(), dy.min())) < 4.0:
            return {"found": False, "name": name, "size": [w, h],
                    "error": "board is too SMALL in frame to measure "
                             "(%.0f px/square, need 18+) - move it closer"
                             % pitch}
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        cx, cy = pts.mean(axis=0)
        # Tilt: the perspective foreshortening of the two board axes. Row and
        # column direction vectors' length ratio maps to view obliqueness -
        # cheap, monotonic, and needs no pose solve.
        row0 = pts[self.cols - 1] - pts[0]
        rowN = pts[-1] - pts[-self.cols]
        r0, rn = np.linalg.norm(row0), np.linalg.norm(rowN)
        ratio = min(r0, rn) / max(r0, rn) if max(r0, rn) > 0 else 1.0
        tilt = math.degrees(math.acos(max(0.0, min(1.0, ratio))))
        return {"found": True, "name": name, "size": [int(w), int(h)],
                "corners": pts.astype(float).tolist(),
                "cell": [min(2, int(3 * cx / w)), min(2, int(3 * cy / h))],
                "frac": round(float(x1 - x0) / w, 3),
                "tilt_deg": round(tilt, 1)}

    # -- guidance -----------------------------------------------------------
    def pair_view(self, i):
        p = self.pairs[i]
        return {"index": i, "at": p["at"],
                "cams": {n: {k: v for k, v in c.items() if k != "corners"}
                         for n, c in p["cams"].items()}}

    def status(self):
        with _lock:
            pairs = list(self.pairs)
        good = [p for p in pairs
                if all(c.get("found") for c in p["cams"].values())
                and len(p["cams"]) == 2]
        cov = {}
        near = far = tilted = 0
        for p in good:
            for name, c in p["cams"].items():
                g = cov.setdefault(name, [[0] * 3 for _ in range(3)])
                col, row = c["cell"]
                g[row][col] += 1
            c1 = next(iter(p["cams"].values()))
            if c1["frac"] >= NEAR_FRAC:
                near += 1
            if c1["frac"] <= FAR_FRAC:
                far += 1
            if max(c["tilt_deg"] for c in p["cams"].values()) >= TILT_DEG:
                tilted += 1
        guide = []
        if len(good) < MIN_PAIRS:
            guide.append("capture %d more pair%s with the board fully visible "
                         "to BOTH cameras"
                         % (MIN_PAIRS - len(good),
                            "" if MIN_PAIRS - len(good) == 1 else "s"))
        POS = {(0, 0): "top-left", (0, 1): "top", (0, 2): "top-right",
               (1, 0): "left", (1, 2): "right", (2, 0): "bottom-left",
               (2, 1): "bottom", (2, 2): "bottom-right"}
        for name in sorted(cov):
            g = cov[name]
            missing = [POS[(r, c)] for r in range(3) for c in range(3)
                       if g[r][c] == 0 and (r, c) != (1, 1)]
            if missing and len(good) >= 4:
                guide.append("move the board to the %s of %s's view"
                             % (", ".join(missing[:3]), name))
        if len(good) >= 4:
            if near < MIN_NEAR:
                guide.append("bring the board CLOSER (fill over ~a third of "
                             "the frame) for %d more capture%s"
                             % (MIN_NEAR - near,
                                "" if MIN_NEAR - near == 1 else "s"))
            if far < MIN_FAR:
                guide.append("hold the board FARTHER (small in frame) for %d "
                             "more capture%s"
                             % (MIN_FAR - far, "" if MIN_FAR - far == 1
                                else "s"))
            if tilted < MIN_TILTED:
                guide.append("tilt the board ~30 degrees toward the cameras "
                             "for %d more capture%s - straight-on views "
                             "cannot separate focal length from distance"
                             % (MIN_TILTED - tilted,
                                "" if MIN_TILTED - tilted == 1 else "s"))
        bad = len(pairs) - len(good)
        if bad and len(pairs) >= 3 and bad / len(pairs) > 0.5:
            guide.append("most captures miss the board on one camera - keep "
                         "it inside the stereo overlap (both live views) and "
                         "avoid glare off the board")
        ready = not guide
        if ready:
            guide.append("enough data - Compute when you are done adding "
                         "poses (more well-spread pairs only help)")
        return {"active": True, "pattern": [self.cols, self.rows],
                "square_mm": self.square_mm,
                "baseline_mm_measured": self.baseline_mm,
                "pairs": len(pairs), "pairs_good": len(good),
                "near": near, "far": far, "tilted": tilted,
                "targets": {"pairs": MIN_PAIRS, "near": MIN_NEAR,
                            "far": MIN_FAR, "tilted": MIN_TILTED},
                "coverage": cov, "guidance": guide, "ready": ready,
                "last_pairs": [self.pair_view(i)
                               for i in range(max(0, len(pairs) - 5),
                                              len(pairs))],
                "result": self.result, "busy": self.busy}

    def discard(self, index):
        with _lock:
            if 0 <= index < len(self.pairs):
                self.pairs.pop(index)
                self.result = None

    # -- solve --------------------------------------------------------------
    def compute(self):
        cv2, np = _cv()
        with _lock:
            good = [p for p in self.pairs
                    if len(p["cams"]) == 2
                    and all(c.get("found") for c in p["cams"].values())]
            self.busy = True
        try:
            if len(good) < 8:
                raise RuntimeError("only %d usable pairs - need at least 8 "
                                   "(12+ recommended)" % len(good))
            names = sorted(good[0]["cams"])
            size = tuple(good[0]["cams"][names[0]]["size"])
            # Object grid in METRES so T comes out in metres directly.
            sq = self.square_mm / 1000.0
            obj = np.zeros((self.rows * self.cols, 3), np.float32)
            obj[:, :2] = (np.mgrid[0:self.cols, 0:self.rows]
                          .T.reshape(-1, 2) * sq)
            objpts = [obj] * len(good)
            imgpts = {n: [np.array(p["cams"][n]["corners"], np.float32)
                          .reshape(-1, 1, 2) for p in good] for n in names}
            intr = {}
            for n in names:
                # NO intrinsic guess anywhere: the corrected underwater
                # optics mean the focal is precisely the unknown.
                rms, K, d, _r, _t = cv2.calibrateCamera(
                    objpts, imgpts[n], size, None, None)
                intr[n] = (rms, K, d)
            flags = cv2.CALIB_FIX_INTRINSIC
            rms2, K1, d1, K2, d2, R, T, _E, _F = cv2.stereoCalibrate(
                objpts, imgpts[names[0]], imgpts[names[1]],
                intr[names[0]][1], intr[names[0]][2],
                intr[names[1]][1], intr[names[1]][2], size, flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                          200, 1e-6))
            baseline = float(np.linalg.norm(T))
            res = {
                "date": time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime()),
                "pattern": [self.cols, self.rows],
                "square_mm": self.square_mm,
                "image_size": list(size), "pairs_used": len(good),
                "cams": names, "notes": self.notes,
                "K1": K1.tolist(), "d1": d1.ravel().tolist(),
                "K2": K2.tolist(), "d2": d2.ravel().tolist(),
                "R": R.tolist(), "T": T.ravel().tolist(),
                "baseline_m": round(baseline, 5),
                "rms_cam_px": {n: round(intr[n][0], 3) for n in names},
                "rms_stereo_px": round(float(rms2), 3),
                "fx": {n: round(float(intr[n][1][0][0]), 1) for n in names},
                "fy": {n: round(float(intr[n][1][1][1]), 1) for n in names},
            }
            verdict = []
            if rms2 > 1.0:
                verdict.append("stereo RMS %.2f px is HIGH (want < 1.0) - "
                               "discard blurred/edge pairs and add spread"
                               % rms2)
            if self.baseline_mm:
                agree = 100.0 * baseline / (self.baseline_mm / 1000.0)
                res["agreement_pct"] = round(agree, 1)
                if abs(agree - 100.0) > 5.0:
                    verdict.append(
                        "solved baseline %.1f mm vs measured %.1f mm (%.1f%%)"
                        " - re-measure the tape OR re-measure the board's "
                        "square size; one of them is off" %
                        (baseline * 1000, self.baseline_mm, agree))
            fxs = list(res["fx"].values())
            if max(fxs) / max(1e-9, min(fxs)) > 1.05:
                verdict.append("the two bodies' focal lengths differ by >5%% "
                               "(%.0f vs %.0f px) - identical lenses should "
                               "match; check the lens/port setup" %
                               (fxs[0], fxs[1]))
            res["verdict"] = verdict or ["calibration looks sound"]
            res["ok"] = not verdict
            with _lock:
                self.result = res
            return res
        finally:
            with _lock:
                self.busy = False

    def save(self):
        with _lock:
            res = self.result
        if not res:
            raise RuntimeError("nothing computed yet")
        _atomic_json(STEREO_PATH, res)
        return {"path": STEREO_PATH, "baseline_m": res["baseline_m"]}


def stereo_start(cols, rows, square_mm, baseline_mm=None, notes=""):
    global _session
    with _lock:
        _session = StereoSession(cols, rows, square_mm, baseline_mm, notes)
    return _session.status()


def stereo():
    with _lock:
        s = _session
    return s.status() if s else {"active": False, "saved": saved_summary()}


def stereo_session():
    with _lock:
        if _session is None:
            raise RuntimeError("no calibration session - Start one first")
        return _session


# ---------------------------------------------------------------------------
# IMU calibration
# ---------------------------------------------------------------------------
# Which IMU endpoint each camera's Pi serves: cam1 carries the YB on slot 1,
# cam2 the Olive on slot 2. Hosts come from the live node table so a field
# re-address (~/rig/nodes.json) carries over.
def IMU_SOURCES():
    paths = {"cam1": "/imu/latest", "cam2": "/imu2/latest"}
    return {n["name"]: (n["host"], paths.get(n["name"], "/imu/latest"))
            for n in rigcore.NODES if n["name"] in paths}
STILL_GYRO_DPS = 2.0        # allowed |gyro| std while "still"
STILL_ACC_G = 0.02          # allowed |accel| std


def _collect(host, path, seconds):
    out = []
    end = time.time() + seconds
    last = None
    while time.time() < end:
        r = http_json("http://%s:8081%s" % (host, path), timeout=3)
        if isinstance(r, dict) and r.get("epoch") and r["epoch"] != last:
            last = r["epoch"]
            out.append(r)
        time.sleep(0.05)
    return out


def _stats(samples):
    def col(k):
        return [s[k] for s in samples if isinstance(s.get(k), (int, float))]

    def mean(v):
        return sum(v) / len(v) if v else None

    def std(v):
        if len(v) < 2:
            return None
        m = mean(v)
        return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))
    gx, gy, gz = col("gx"), col("gy"), col("gz")
    ax, ay, az = col("ax"), col("ay"), col("az")
    amag = [math.sqrt(x * x + y * y + z * z)
            for x, y, z in zip(ax, ay, az)] if ax and ay and az else []
    return {
        "n": len(samples),
        "gyro_bias_dps": [round(mean(v), 4) if v else None
                          for v in (gx, gy, gz)],
        "gyro_std_dps": [round(std(v), 4) if v and std(v) is not None
                         else None for v in (gx, gy, gz)],
        "accel_norm_g": round(mean(amag), 5) if amag else None,
        "accel_std_g": round(std(amag), 5) if amag and std(amag) is not None
        else None,
        "level_ref": {"pitch": round(mean(col("pitch")), 3)
                      if col("pitch") else None,
                      "roll": round(mean(col("roll")), 3)
                      if col("roll") else None},
        "heading_mean": round(mean(col("heading")), 2)
        if col("heading") else None,
    }


def calibrate_imu(target="both", seconds=10.0):
    """Sample still windows and write the references. Raises on motion."""
    global _imu_busy
    seconds = max(3.0, min(30.0, float(seconds)))
    sources = IMU_SOURCES()
    targets = sorted(sources) if target == "both" else [target]
    for t in targets:
        if t not in sources:
            raise ValueError("unknown target %r" % t)
    with _lock:
        if _imu_busy:
            raise RuntimeError("an IMU calibration is already running")
        _imu_busy = True
    try:
        res = {}
        threads = {}
        rescol = {}

        def work(name):
            host, path = sources[name]
            rescol[name] = _collect(host, path, seconds)
        for name in targets:
            th = threading.Thread(target=work, args=(name,), daemon=True)
            th.start()
            threads[name] = th
        for th in threads.values():
            th.join(seconds + 10)
        for name in targets:
            samples = rescol.get(name) or []
            if len(samples) < max(10, seconds * 3):
                raise RuntimeError(
                    "%s: only %d IMU samples in %.0f s - is its IMU present "
                    "and streaming? (check /health)"
                    % (name, len(samples), seconds))
            st = _stats(samples)
            gs = [g for g in st["gyro_std_dps"] if g is not None]
            if (gs and max(gs) > STILL_GYRO_DPS) or \
                    (st["accel_std_g"] or 0) > STILL_ACC_G:
                raise RuntimeError(
                    "%s: the rig MOVED during the window (gyro std %s dps, "
                    "accel std %s g) - hold it still, on a solid surface, "
                    "and run again" % (name, st["gyro_std_dps"],
                                       st["accel_std_g"]))
            res[name] = st
        doc = {}
        try:
            with open(IMU_PATH) as fh:
                doc = json.load(fh)
                if not isinstance(doc, dict):
                    doc = {}
        except (OSError, ValueError):
            pass
        doc["date"] = time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime())
        for name in targets:
            doc[name] = res[name]
        if len(targets) >= 2 and "cam1" in res and "cam2" in res:
            # The cross-IMU orientation seed (Euler-difference approximation
            # of R_12; the fusion filter refines it - docs/olive-imu.md). Both
            # windows were sampled simultaneously, so this is a true
            # same-instant comparison.
            a, b = res["cam1"], res["cam2"]
            if a["level_ref"]["pitch"] is not None \
                    and b["level_ref"]["pitch"] is not None:
                doc["relative"] = {
                    "pitch": round(a["level_ref"]["pitch"]
                                   - b["level_ref"]["pitch"], 3),
                    "roll": round(a["level_ref"]["roll"]
                                  - b["level_ref"]["roll"], 3),
                    "yaw": round(((a["heading_mean"] or 0)
                                  - (b["heading_mean"] or 0) + 540) % 360
                                 - 180, 2),
                    "method": "euler-difference seed; refine in fusion",
                }
        _atomic_json(IMU_PATH, doc)
        return {"ok": True, "saved": IMU_PATH,
                **{n: res[n] for n in targets},
                "relative": doc.get("relative")}
    finally:
        with _lock:
            _imu_busy = False

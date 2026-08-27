# VSLAM-lite — live camera trajectory from the review stream

Written 2026-08-27. Status: **framework v1, runs on the macOS host (CPU)**;
finalized on the Jetson Orin Nano, where the backend may be swapped for CUDA.
Code: `rig/vslam.py`. Selftest: `python3 rig/tests/vslam_selftest.py` (no
hardware, no network, passes with or without OpenCV installed).

During a run the nodes trickle ~200 KB 1616x1080 review JPEGs into
`<run>/cam1/` at ~2 Hz, with a `flight_log.csv` row per frame. vslam consumes
exactly that — nothing else crosses the wire — and produces an incremental
camera-derived trajectory + sparse landmark cloud the operator can see live.
It is a cross-check on nav (GPS+IMU says where the boat went; the CAMERA says
where the camera went) and the seed for later photogrammetry planning.

    pip3 install opencv-python-headless numpy     # Mac host (one-time)
    python3 rig/vslam.py ~/rig-runs/<run> [--cam cam1] [--follow]
                         [--json out.json] [--ply out.ply]

Without cv2/numpy nothing crashes: every entry point reports
`unavailable: <reason>` and the selftest still passes on the degradation
checks. The Jetson gets cv2 from JetPack (`apt python3-opencv`), no pip build.

## What v1 does / does not do

Does:
* monocular VO on cam1: ORB (~1500) → BF-Hamming ratio match against the
  previous keyframe → essential matrix (**MAGSAC** — see below) → recoverPose
  → pose chain, quaternion per keyframe
* keyframe policy: median-parallax gate (a stationary rig yields "still"
  frames forever, **zero drift by construction** — verified on the dock runs)
* metric scale from GPS: sliding-window fit of segment scale to the
  flight_log UTM baseline; without GPS (or with a stationary fix) the chain
  is unit-norm and the status says `scale=unit`
* sparse landmark cloud via triangulation, capped (thins at 20 k points)
* tracking-loss handling: lost streak → reinitialize a new segment at the
  last pose; segments list preserved
* survives the trickle: out-of-order arrival, duplicate names, half-written
  files (run.py writes straight to the final path — the runner settles on
  stable size and feed() rejects JPEGs without an EOI marker), corrupt frames
* turbid-water preprocessing: illumination flatten + CLAHE at 0.5 scale —
  the same treatment stereo_check.py needed (raw survey frames gave SIFT
  1-15 features; flattened, hundreds)

Does not (v2/Jetson-day candidates, in rough priority order):
* no bundle adjustment / loop closure — drift grows with segment length
* no rotation-to-UTM in the ENGINE: the pose chain lives in the first-camera
  frame; the runner's snapshot carries a 2D similarity (`utm_align`) fitted
  trajectory→UTM for the overlay when the boat has actually moved >1 m
* per-pair translation is unit-norm, so within a scale window all speed
  information comes from GPS; landmark-based relative scale would fix this
* no stereo. cam2 at the known 224 mm baseline (stereo_check.py) gives
  metric scale with NO GPS and per-keyframe metric depth — the natural v2:
  feed same-instant cam2 frames as a second view of each keyframe, keep the
  VslamEngine interface unchanged
* planar-scene homography fallback (decomposeHomographyMat) when E is
  degenerate — MAGSAC already handles the common case, see below
* intrinsics are a guess (below) until calibration day

## Architecture

    review JPEGs + flight_log.csv          (files on disk; the ONLY input)
            │
    VslamRunner        thread; tails the run dir, orders by filename
            │          timestamp, settles half-written files, joins the
            │          flight_log row, serialises feed(), keeps trajectory
            ▼
    VslamEngine        pure-Python state machine: keyframe policy, pose
            │          chain, GPS scale window, segments, landmark cap.
            │          Imports NOTHING — all cv2/numpy behind the backend.
            ▼
    CpuOrbBackend      the swap point. Five methods:
                         ensure() -> None | reason        (lazy import)
                         decode(bytes) -> gray | None
                         detect(gray) -> ([(x,y)], desc)
                         match(d1, d2) -> [(i1,i2)]
                         rel_pose(p1,p2,K) -> (R,t,inliers) | None
                         triangulate(p1,p2,K,R,t) -> [[x,y,z]]
                       make_backend("cpu") is the single construction point;
                       the Jetson adds make_backend("cuda"/"vpi"/"cuvslam")
                       and NOTHING else changes.

World frame = first camera frame (OpenCV convention: x right, y **down**,
z forward). A roughly level camera puts the horizontal plane at (x, z) —
that is what `utm_align` maps to (xutm, yutm).

**MAGSAC, not classic RANSAC** for the essential matrix: a survey seafloor is
near-planar, and a planar scene under smooth motion is homography-degenerate
for the 5-point solver. Classic RANSAC deterministically returned a 13-inlier
forward-motion model on a 972-match pure-sideways synthetic pair; MAGSAC gave
889/972 on the same data. The selftest pins a two-depth-layer scene; the
single-plane degenerate case is documented at the call site. A second guard,
`min_inlier_frac` (30%), rejects any solve that keeps a small consensus of a
large match set — observed to kink the chain 90° in one step otherwise.

## Intrinsics and calibration (Jetson-day task)

Default guess: 1616x1080, HFOV ~59° → fx ≈ 1429 px (`Intrinsics`; `--focal`
and `--hfov` override; `--port-factor 1.33` for a flat underwater port, the
same constant stereo_check.py uses). The trajectory SHAPE tolerates moderate
focal error; absolute angles and the landmark cloud do not.

Calibration procedure (in this order of preference):

1. **Checkerboard**: 9x6 board, ~30 review JPEGs from a live node at varying
   pose/distance, `cv2.calibrateCamera` at 1616x1080. Store
   `{fx, fy, cx, cy, dist}` in `rig/intrinsics_cam1.json`; wire it into
   `Intrinsics` (small change — accept a dist vector, undistort points before
   `rel_pose`). Calibrate THROUGH the housing port in water for survey use,
   or multiply the in-air focal by 1.33 for a flat port (approximation).
2. **EXIF**: the card JPEGs carry FocalLength (0x920A); f_px =
   f_mm / 35.7 × 1616 (stereo_check.py does exactly this). Good enough to
   replace the HFOV guess if no board is available.
3. Verify either with stereo_check-style epipolar residuals on a real run.

## GPS scale strategy (and its limits)

`recoverPose` translation is unit-norm, so each keyframe pair contributes
exactly 1.0 of VO path length. When the current and previous keyframes both
carry UTM, the metric step enters a sliding window (6 pairs); scale = window
mean, applied to each new segment step. Guards:

* window mean < 0.05 m → GPS is jitter, not motion (every dock run to date
  is a single repeated fix) → hold the last good scale, or stay `unit`
* no GPS at all → unit-norm, `scale_source: "unit"`, status says so
* rotation-only keyframes (boat turning in place) contribute VO length with
  ~no GPS step; the window damps this, it does not eliminate it — expect
  scale dips in tight turns until landmark-based relative scale exists (v2)

`snapshot()["utm_align"]` is fitted separately (least-squares 2D similarity,
VO (x, z) → (xutm, yutm), only when GPS spread > 1 m and ≥ 3 keyframes) so
the map overlay can PLACE the track without the engine ever knowing about UTM.

## Measured performance and the Jetson budget

Measured on this Mac (M-series, CPU, `proc_scale` 0.5 → feature work at
808x540, flatten+CLAHE on, ORB 1500):

| what | number |
|---|---|
| per-frame processing, EMA | **46.7 ms** (≈ 21 fps capability) |
| 336-frame dock run, wall | 14.4 s end to end |
| trickle rate to keep up with | 2 Hz → 500 ms budget per frame |
| snapshot JSON at defaults | ~100-125 KB |

Orin Nano CPU (6x A78AE) is roughly 2-4x slower per core → est. 100-200 ms
per frame, still 2.5-5x headroom at 2 Hz **on CPU alone**. So: bring-up on
the Jetson needs NO CUDA work to function; CUDA is an optimization, not a
dependency. Re-measure with `stats.proc_ms` from a real run before deciding.

## Jetson finalization plan

Day-1 bring-up (CPU, no new code):
1. `sudo apt install python3-opencv python3-numpy` (JetPack's build; note the
   apt cv2 has **no CUDA** — that is fine for the CPU backend).
2. `python3 rig/tests/vslam_selftest.py` → must be 38/38.
3. `python3 rig/vslam.py <a drained run> --json /tmp/v.json --ply /tmp/v.ply`
   and check `stats.proc_ms` against the table above.
4. Calibrate intrinsics (section above), re-run, keep the JSON.

Backend candidates behind the SAME five-method interface, in ascending
effort:

| backend | what changes | notes |
|---|---|---|
| `cv2.cuda` ORB + BFMatcher | `detect`/`match` only | needs a CUDA-enabled OpenCV (self-build or jetson-containers; the apt cv2 won't have it). Cheapest real speedup; E/recoverPose stay CPU (they are not the bottleneck) |
| VPI (PVA/GPU) | `detect`/`match` | ships in JetPack with Python bindings; check the installed VPI version exposes ORB before committing. Frees both CPU and GPU (PVA offload) |
| Isaac ROS Visual SLAM (cuVSLAM) | whole engine becomes a thin adapter | full VIO with loop closure, wants stereo + IMU (we have both, 224 mm baseline + flight_log IMU). Heavy: ROS2 + containers on the critical path. Only if v1 drift is operationally painful |

Decision gate: if CPU `proc_ms` < 250 on the Orin, ship v1 as-is and spend
the effort on stereo scale instead — it buys accuracy, CUDA only buys speed
we may not need at 2 Hz.

## rigd integration spec (NOT implemented — wire-in plan, one sitting)

Server (`rig/rigd.py`):
* module-level `_vslam = None`; on **run start** (the same place the run dir
  is created): `_vslam = VslamRunner(); _vslam.start(run_dir, cam="cam1")`.
  On **run stop**: `_vslam.stop()`, keep the object so the endpoint serves
  the final state until the next run starts.
* `GET /api/vslam` → `200 {"active": bool, **_vslam.snapshot()}`, or
  `200 {"active": false, "status": "idle"}` when no runner has ever started.
  snapshot() is already JSON-safe, capped (~100 KB) and lock-guarded; the
  handler is one `self._json(...)` call. Do NOT raise the caps for the UI —
  the browser's 6-connections-per-origin limit already bit /api/imu/window;
  keep this payload lean and poll at ≥ 2.5 s like /api/anomalies.
* if cv2 is missing on the host the endpoint still answers, with
  `status: "unavailable: ..."` — the UI shows that string verbatim.

UI (`rig/rig_ui.html`, map tab):
* poll `/api/vslam` every 2.5 s **only while the map tab is visible**.
* if `utm_align` present: transform each trajectory point
  `e = a_re*x − a_im*z + tx`, `n = a_im*x + a_re*z + ty` and draw the
  polyline on the existing UTM map, breaking the line at `segment`
  boundaries (current segment solid, earlier segments faded).
* else: draw in a fixed-size inset (VO units, x right / z up) with a badge
  showing `scale=unit` or `gps` — never place an unaligned track on the map.
* status chip: the `status` string + `keyframes`/`fed`/`fps` + lost/reinit
  counters; corrupt > 0 in red (it means truncated review frames, which is a
  transfer problem, not a vision problem).

## Operator notes / failure modes

* stationary rig → "still (parallax 0.0 px)", 1 keyframe, empty cloud. That
  is CORRECT, not broken (verified: 260827_0241_stability-2fps → 32 still).
* dock runs with a single GPS fix never get metric scale — `scale=unit` on
  the badge is the honest answer there.
* hand-carried tests with view gaps produce many segments (260820_1925:
  336 frames → 76 keyframes in 56 segments, 195 lost). At sea with
  continuous 2 Hz coverage segments should be long; many reinits during a
  real transect = look at exposure or turbidity first.
* `--pre none` turns the flatten+CLAHE off (bench scenes in air track fine
  without it and it saves ~40% of the frame budget).

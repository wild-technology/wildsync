# VSLAM-lite — live camera trajectory from the review stream

Written 2026-08-27 (stereo mode added same day). Status: **framework v1,
runs on the macOS host (CPU)**; finalized on the Jetson Orin Nano, where the
backend may be swapped for CUDA. Code: `rig/vslam.py`. Selftest:
`python3 rig/tests/vslam_selftest.py` (no hardware, no network, passes with
or without OpenCV installed — 61 checks with, 12 without).

During a run the nodes trickle ~200 KB 1616x1080 review JPEGs into
`<run>/cam1/` and `<run>/cam2/` at ~2 Hz, with a `flight_log.csv` row per
frame. vslam consumes exactly that — nothing else crosses the wire — and
produces an incremental camera-derived trajectory + sparse landmark cloud
the operator can see live. It is a cross-check on nav (GPS+IMU says where
the boat went; the CAMERA says where the camera went) and the seed for later
photogrammetry planning. Two modes plus auto-detection:

* **mono** — cam1 only; essential-matrix chain; scale from GPS (or unit)
* **stereo** — synchronized cam1+cam2 pairs (~0.4 ms measured skew); METRIC
  scale from the inter-camera baseline, PnP chain; GPS demotes to a
  cross-check. See the Stereo section below for its (real) caveats.
* **auto** (default) — stereo when the cam2 dir exists and >50% of frames
  find a same-instant partner by filename; mono otherwise.

    pip3 install opencv-python-headless numpy     # Mac host (one-time)
    python3 rig/vslam.py ~/rig-runs/<run> [--mode auto|mono|stereo]
                         [--cam cam1] [--follow] [--baseline 0.30]
                         [--features 1500] [--json out.json] [--ply out.ply]

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

Also does (stereo mode — see its own section for honest caveats):
* pair association by capture instant from the filenames (tolerance = half
  the median inter-frame gap), missing halves tolerated and counted
* self-estimated cam1→cam2 extrinsics from the first N good pairs, with a
  homography-decomposition fallback for planar scenes
* METRIC per-pair triangulated cloud from the baseline; frame-to-frame
  chain by PnP on the previous cloud — no GPS needed for scale, and no
  planar-scene degeneracy
* GPS as a cross-check: `stats.gps_ratio` = stereo length / GPS length

Does not (v2/Jetson-day candidates, in rough priority order):
* no bundle adjustment / loop closure — drift grows with segment length
* no rotation-to-UTM in the ENGINE: the pose chain lives in the first-camera
  frame; the runner's snapshot carries a 2D similarity (`utm_align`) fitted
  trajectory→UTM for the overlay when the boat has actually moved >1 m
* mono per-pair translation is unit-norm, so within a scale window all speed
  information comes from GPS; stereo mode is the fix when pairs exist
* no measured baseline and no checkerboard stereo extrinsics yet — stereo
  distances are placeholder-scaled until then (loudly flagged)
* no epipolar-GUIDED cross-camera matching: prototyped against the transect
  run with self-estimated extrinsics and it was a wash (gate ~= plain ratio
  matching); becomes worthwhile once calibrated extrinsics make the gate
  trustworthy
* intrinsics are a guess (below) until calibration day

## Architecture

    review JPEGs + flight_log.csv          (files on disk; the ONLY input)
            │
    VslamRunner        thread; tails cam1 (and cam2 in stereo/auto), orders
            │          by filename timestamp, settles half-written files,
            │          associates pairs by capture instant, decides
            │          mono-vs-stereo in auto, joins the flight_log row,
            │          serialises feed, keeps trajectory
            ▼
    VslamEngine /      pure-Python state machines: keyframe policy, pose
    StereoVslamEngine  chain, scale (GPS window / metric baseline),
            │          segments, landmark cap, extrinsics bootstrap.
            │          Import NOTHING — all cv2/numpy behind the backend.
            ▼
    CpuOrbBackend      the swap point. Seven methods:
                         ensure() -> None | reason        (lazy import)
                         decode(bytes) -> gray | None
                         detect(gray) -> ([(x,y)], desc)
                         match(d1, d2) -> [(i1,i2)]
                         rel_pose(p1,p2,K) -> (R,t,inliers) | None
                         rel_pose_h(p1,p2,K) -> same, via homography
                             decomposition (planar-scene fallback)
                         triangulate(p1,p2,K,R,t) -> [[x,y,z]|None] aligned
                             with the input pairs (stereo keeps the index
                             map for PnP; mono drops the None holes)
                         pnp(obj,img,K) -> (R,t,inliers) | None
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
the map overlay can PLACE the track without the engine ever knowing about
UTM. In stereo mode the fitted |a| doubles as a baseline check: it should
sit near 1.0 and its deviation ≈ the baseline error.

## Stereo mode — how it works and what to believe

Why stereo at all: it kills both monocular weaknesses at once. The pair
triangulates a METRIC cloud from the inter-camera baseline (scale with no
GPS), and the frame-to-frame chain is PnP (3D→2D) against the previous
cloud — well-posed on a flat seafloor where the essential matrix
degenerates. The selftest proves both on synthetic imagery: metric path
recovered within 2% on a two-plane scene, and clean tracking on the exact
single-plane scene that broke the mono chain.

**Self-estimated extrinsics (the big caveat).** The relative pose cam1→cam2
is estimated ONCE from the first `calib_pairs` (5) good pairs: MAGSAC
essential matrix per pair, homography-decomposition fallback when the scene
is one plane (measured: MAGSAC kept 3/1011 matches on a planar pair, the
homography kept 939 and cheirality picks the right decomposition at
front-fraction 1.0 vs ≤0.59), then a medoid over the candidates' translation
directions. That fixes rotation and the translation DIRECTION from image
evidence only. The translation LENGTH is `--baseline`, a rig constant that
is **currently unmeasured** (docs/future-tests.md §1 also flags lens parity
as unverified) — the 0.30 m default is a placeholder and every status line,
snapshot and CLI summary carries `UNCALIBRATED` until it is measured.
**Every stereo distance scales linearly with baseline error; the SHAPE of
the track does not.** Measure the physical baseline with a tape (±2 mm on
0.3 m = 0.7% distance error) as the day-one stopgap; the checkerboard
procedure below replaces the whole estimate.

**Pairing.** Frames associate by capture instant parsed from the filenames
(the two bodies fire ~0.4 ms apart; the filename stamps land well inside
half a shot period). Tolerance = half the median inter-frame gap. In follow
mode a lone cam1 frame waits one grace period (2 polls) for its half before
being fed unpaired; unpaired frames still advance the pose by PnP against
the LAST paired cloud (counted in `stats.unpaired`) but cannot refresh it.
cam2 frames that can never pair are counted as orphans.

**GPS becomes a cross-check**, not the scale source: `stats.gps_ratio` =
stereo track length / GPS track length over the same keyframes, reported
once the boat has moved ≥2 m. Ratio far from 1.0 means the baseline (or the
pairing) is wrong — on synthetic data with a correct baseline it reads
0.98–1.0.

**Measured on real runs (2026-08-27):**

| run | result |
|---|---|
| 260827_0241 stability (bench) | pairing 97%, extrinsics **0/5 — correct**: the two bodies lay on the desk pointed at different things (checked the frames); no overlap, no calibration, no fake output |
| 260820_1925 transect (hand-held, in water) | pairing 100%; extrinsics self-estimated (t scatter 9–16°); at the default 1500 ORB features only 4 keyframes — cross-camera matching is the bottleneck (6–48 ratio matches vs 100–300 temporal); at `--features 4000` → 16 keyframes, 2083 landmarks |

The transect numbers are the honest v1 state: cross-camera appearance
differs a lot (vignette, exposure, viewpoint — stereo_check.py needed SIFT
at 8000 features with contrast 0.01 for the same job), so the per-pair
cloud is thin and the PnP chain loses easily on this gappy hand-carried
data. Feature budget scales it almost for free (64→163 mean cross matches
from 1500→5000 features at +14 ms/pair): run stereo with `--features
3000`–`5000` until the Jetson gets a stronger detector. Mono remains the
more robust live overlay until the extrinsics are calibrated — auto mode
picks stereo when pairs exist, so pass `--mode mono` explicitly if the
overlay matters more than metric scale on a given day.

**Checkerboard stereo calibration (Jetson day, replaces self-estimation):**

1. Rigid 9x6 board visible to BOTH cameras at once; ~30 synchronized review
   pairs at varying pose/distance (in water, through the ports, for survey
   use).
2. `cv2.stereoCalibrate` at 1616x1080 → per-camera K + dist, and R, T
   between the cameras. |T| IS the baseline — compare against the tape
   measurement as a sanity check.
3. Store as `rig/stereo_extrinsics.json`; wire-in = construct
   `StereoVslamEngine` with the stored R, T instead of the self-estimate
   (set `_ext` directly and `baseline_calibrated=True`) — the engine
   already treats extrinsics as an opaque frozen constant, nothing else
   changes.
4. Re-run a drained run; `gps_ratio` should sit within a few % of 1.0 on a
   moving-GPS transect. Then enable epipolar-guided cross-camera matching
   (prototyped, parked — see the does-not list).

**Expected accuracy at a ~0.3 m baseline, 1–3 m altitude** (fx ≈ 950 px at
proc scale 0.5 in water; depth error σ_z ≈ z²·σ_d / (f·b), σ_d ≈ 0.7 px):

| altitude | depth noise per point | stereo overlap (59° HFOV) |
|---|---|---|
| 1 m | ~2.5 mm | ~73% |
| 2 m | ~10 mm | ~87% |
| 3 m | ~23 mm | ~91% |

Per-step pose error over dozens of PnP inliers lands at the cm level, so at
2 Hz survey cadence expect ~1–2% drift per unbroken segment. Systematic
error is dominated by the extrinsics until calibrated: baseline error maps
1:1 onto every distance, and the self-estimated t-direction scatter (9–16°
measured) tilts the whole track. After checkerboard calibration both
collapse to the sub-percent level.

## Measured performance and the Jetson budget

Measured on this Mac (M-series, CPU, `proc_scale` 0.5 → feature work at
808x540, flatten+CLAHE on, ORB 1500):

| what | number |
|---|---|
| mono per-frame, EMA | **46.7 ms** (≈ 21 fps capability) |
| stereo per-frame at `--features 4000`, wall | **~65 ms** (two decodes+detects, pair match, triangulation, PnP; BFMatcher threads across cores) |
| 336-frame run, wall | 14.4 s mono / 21.8 s stereo end to end |
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
2. `python3 rig/tests/vslam_selftest.py` → must be 61/61.
3. `python3 rig/vslam.py <a drained run> --json /tmp/v.json --ply /tmp/v.ply`
   and check `stats.proc_ms` against the table above.
4. Calibrate intrinsics (section above), re-run, keep the JSON.

Backend candidates behind the SAME five-method interface, in ascending
effort:

| backend | what changes | notes |
|---|---|---|
| `cv2.cuda` ORB + BFMatcher | `detect`/`match` only | needs a CUDA-enabled OpenCV (self-build or jetson-containers; the apt cv2 won't have it). Cheapest real speedup; E/recoverPose stay CPU (they are not the bottleneck) |
| VPI (PVA/GPU) | `detect`/`match` | ships in JetPack with Python bindings; check the installed VPI version exposes ORB before committing. Frees both CPU and GPU (PVA offload) |
| Isaac ROS Visual SLAM (cuVSLAM) | whole engine becomes a thin adapter | full VIO with loop closure, wants stereo + IMU (we have both: nominal ~224 mm baseline per stereo_check.py — unverified — plus flight_log IMU). Heavy: ROS2 + containers on the critical path. Only if v1 drift is operationally painful |

Decision gate: if CPU `proc_ms` < 250 on the Orin, ship v1 as-is and spend
the effort on stereo scale instead — it buys accuracy, CUDA only buys speed
we may not need at 2 Hz.

## rigd integration spec (NOT implemented — wire-in plan, one sitting)

Server (`rig/rigd.py`):
* module-level `_vslam = None`; on **run start** (the same place the run dir
  is created): `_vslam = VslamRunner(mode="auto"); _vslam.start(run_dir,
  cam="cam1")`. On **run stop**: `_vslam.stop()`, keep the object so the
  endpoint serves the final state until the next run starts. Auto decides
  stereo/mono from the first frames on its own; the snapshot's `mode`,
  `baseline_m`, `baseline_calibrated`, `calib` and `pairing_rate` fields
  report what it chose and how trustworthy the metric scale is.
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
* stereo badge: show `mode`; when stereo, `pairing_rate`, `gps_ratio`, and
  the `calib` string — render the word UNCALIBRATED in amber verbatim, it
  is the difference between "shape only" and "trust the metres".

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
* stereo "calibrating stereo extrinsics (0/N)" that never advances = the
  two cameras do not overlap (verify by eyeballing a same-instant pair
  before blaming the code — the bench run's bodies pointed at different
  walls) or cross-camera matches are starving (`--features 4000`).
* stereo distances carry the UNCALIBRATED flag until the baseline is
  measured — treat the track SHAPE as real and the metres as provisional;
  `gps_ratio` on a moving-GPS run tells you how provisional.

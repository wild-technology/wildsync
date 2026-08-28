# cam3 as a Basler node — wildsync integration plan and strobe trigger

Research brief for accepting the **Basler a2A4504-18umBAS** (USB3, 20.2 MP
mono, global shutter, pypylon) as **cam3** — a third synchronized camera on
its own Pi 5 — and for the question this camera reopens: **can it trigger
the XR256 directly**, instead of the Jetson interface board planned in
`docs/xr256-jetson-trigger.md`, or via the VA Imaging strobe controller?

**Status 2026-08-28: research and planning only. Nothing coded.** Line
references are to the working tree on `cam3-node`, which already carries the
(uncommitted) cam3 *slot* work: the `~/rig/nodes.json` patch mechanism with
`cam_num` defaulting (`rig/rigcore.py:59-99`), `drain.py` resolving the
fleet from rigcore instead of its own literal (`rig/drain.py:65-77`), chrony
slot 3 in `deploy/pi-resync/user-data.template`, and `docs/PI-SETUP.md`.
`PI-SETUP.md` was written commissioning cam3 as a third **ILX** node; this
brief re-targets that slot to the Basler. Its Pi-side content (flash,
cloud-init, network, chrony, PoE) stands; its ILX items (body, `ilxctl`,
Sony SDK, card) do not apply to this node. Camera bring-up and the
standalone test gate live in `docs/BASLER-SETUP.md`.

---

## 0. Shape of the answer

The two-service node architecture survives intact, and that is the whole
trick:

| Service | On cam3 (Basler) | Change |
|---|---|---|
| `piagent` on :8081 | **Stock piagent.** GPIO fire, edge monitor, health, strobe path — unchanged. | none (env only) |
| `ilxctl` on :8080 | Replaced by **`baslerctl`** — a pypylon service implementing the minimum ilxctl-shaped HTTP contract. | new, ~one file |

The wiring makes this possible (`BASLER-SETUP.md` §6): BCM 27 → camera
Line 2 as hardware trigger (falling edge, matching piagent's active-low
pulse), camera Line 3 `ExposureActive` → BCM 22 (open-collector into
gpiomon's pull-up — **fall = exposure start, rise = end, the exact ILX
harness polarity**). piagent cannot tell this camera from a Sony body: the
fire busy-wait, `fire_seq`/`edge_seq` accounting, `epoch_hw` bracketing, and
the strobe scheduler all run verbatim. BCM 17 (FOCUS) drives an unconnected
pin — harmless, and the FOCUS lease endpoints keep working, which
`calibrate_trigger` requires (`rig/run.py:3660`).

What the host sees: a node that reaches `CAM_CONNECTED`, fires over the
`gpio` path with real exposure edges, spools JPEGs, and participates in
`skew_ms`, the strobe acceptance intersection (already N-way,
`rig/rigcore.py:2255-2274`), and trigger-latency calibration — because all
of that machinery is roster-driven, not pair-driven.

---

## 1. Joining the fleet

Already supported, by design of the uncommitted slot work — **no code**:

```json
// ~/rig/nodes.json on the host
{"cam3": {"host": "192.168.1.203", "cam_num": 3}}
```

- `cam_num: 3` is contiguous, so `selftest`'s "cam_num is 1..N and unique"
  check passes. `_DEFAULT_NODES` stays two-wide, deliberately
  (`rig/rigcore.py:33-46`) — a host without cam3 cabled reads 2/2, the boat
  reads 3/3.
- **Keep the node name `cam3`.** `run.py` names run dirs by *node name*
  (`rig/run.py:1963`, `:2056`) while `ingest.py` reconstructs them as
  `"cam%d" % cam` (`rig/ingest.py:243`); the two coincide only while
  `name == "cam" + str(cam_num)`. That convention is now load-bearing —
  either honor it forever or fix ingest (§4.7).
- chrony: slot 3 already exists (`deploy/pi-resync/user-data.template`,
  `SLOTS="1 2 3"`). A cam3 outside the peer mesh puts two permanent
  `node_clock_skew` alarms on every scan (`rig/rigd.py:1005-1033`) — the
  mesh is not optional.
- `deploy/deploy.sh` `NODES` already lists cam3, but `deploy_node`
  hard-requires the Sony SDK and installs `ilxctl.service`
  (`deploy/deploy.sh:100-153`) — cam3 needs a `baslerctl` deploy branch
  (§4.8).

---

## 2. `baslerctl` — the :8080 contract

The gate is `NodeMonitor`: a node whose :8080 doesn't satisfy the
ilxctl shape sits in `ILX_DOWN` forever, `is_connected()` stays false, and
it is excluded from runs, fires, pulls, and adoption entirely
(`rig/rigcore.py:517-521, 734-736`; `rig/run.py:2392, 1929, 1963`). So the
minimum viable `baslerctl`, distilled from `rigcore.py:320-333, :543,
:587` and the fakenode fidelity lane (`rig/tests/soaktest.py:410-489`):

| Endpoint | Behavior on the Basler node |
|---|---|
| `GET /api/status` | `connected: true` once the grab loop is armed; **both** label and raw exposure shapes (`"iso": "ISO 400"` *and* `"isoValue": 400`); never `busy: true` without `isoValue` (that's the degraded-status tell); `controlMode` ≠ `"transfer"`; a `writable` map (§3). |
| `POST /api/connect` | Open camera by serial, apply UserSet1, arm the triggered grab loop, return ok. Idempotent — the monitor retries with backoff. |
| `GET /api/shots` | Spool listing `[{name, size}]`, real sizes. On any internal failure: HTTP error, **never `[]`** — the host distinguishes "empty" from "unknown" (`rig/rigcore.py:748-771`). |
| `GET /shot/<name>` | Exact bytes. JPEGs must carry SOI/EOI — `run.py:790-806` validates. |
| `POST /api/shots/delete` | `{"confirm":"delete","name":…}` — delete after the host's verified write. |
| `POST /api/shutter` | `ExecuteSoftwareTrigger()`. This is the degraded `usb` fire path (`rig/run.py:2311-2342`) — worse timing, but it must work. |
| `POST /api/exposure` | Map `shutter → ExposureTime (µs)`, `iso → Gain (dB)` (§3). |
| `GET /liveview.jpg` | Downscaled latest frame — rigd proxies it (`rig/rigd.py:256`). |
| Card/format/store endpoints | Honest 4xx "no card on this node". `drain.py` is ILX-transfer-specific and simply doesn't apply to cam3. |

The grab loop itself: hardware-triggered acquisition permanently armed
(`TriggerSource=Line2`), `OneByOne` strategy, retrieve → encode Mono8 JPEG
(libjpeg-turbo worker pool at the quality Phase 5 chose — mandatory, the
camera has no in-camera compression and raw frames are 20.3 MB;
`BASLER-SETUP.md` §5) → EXIF `DateTimeOriginal`+subsec from the
chunk-timestamp mapping → atomic `.part`/rename into the spool dir. The fidelity lane checks EXIF decodes to
the capture instant within 20 ms — the camera's 1 GHz timestamp clock,
latched against the host clock, clears that easily.

piagent side, env only (`/etc/default/piagent`): point `CAM_SAVE_DIR` at the
Basler spool so `/health`'s `disk_free_mb`/`cam_frames` mean something
(today it statvfs's `~/Pictures/ILX-LR1` and counts ILX extensions,
`rig/piagent.py:154, 1878-1884` — the extension tuple needs `.jpg` only, so
a spool of JPEGs counts fine).

**Timing note that makes the whole thing honest:** the camera's fixed
trigger→exposure latency (`BslExposureStartDelay`, ~112 µs) plus line
propagation is *measured, not assumed* — `calibrate_trigger` fires the node
and takes `edge.epoch_hw − actual_epoch` medians exactly as for the ILX
(`rig/run.py:3583-3652`), and it works unmodified because the edges come
from gpiomon. cam3's lead will be ~0.1 ms where the ILX bodies are ~50 ms;
the per-node lead architecture absorbs that spread without comment.

---

## 3. Settings reconcile — opting out of Sony

`reconcile_all` walks `CONVERGE_FIELDS` (`rig/rigcore.py:776-830`) on every
connected node; a field that reads back different from the fleet vector
raises the red `settings_divergent` alarm (`rig/rigd.py:907-925`). The code
already contains the two escape hatches a non-Sony node needs
(`rig/rigcore.py:851-874`):

1. **Absent readback key = "predates the field", skipped silently** (the
   `BUILD_GATED_FIELDS` precedent). `baslerctl` simply omits keys for
   `drive, program, filetype, imagesize, transsize, rawtype, store_dest,
   wb_mode, colortemp, focus_mode`.
2. **`writable` map with `ENABLE_DISPLAY_ONLY (2)` = reported as
   `settings_unsettable`, not divergence** — use for anything worth
   displaying but not settable (e.g. `aperture` on a fixed C-mount lens).

What *does* converge: `shutter` → `ExposureTime` (Sony `(num<<16)|den`
encoding decoded to seconds → µs), `iso` → `Gain` with a declared
convention — **ISO 100 ≡ 0 dB, +6 dB per stop** (ISO 400 = 12 dB), stated in
`baslerctl` and here, so photometry across the fleet stays comparable.
Exposure is per-camera between applies anyway (`EXPOSURE_FIELDS`), so cam3
holding a different shutter than the ILX pair is already a supported state.

---

## 4. Host changes — the honest list

Everything the survey found that assumes two cameras or an ILX backend, with
severity. (1)–(3) block a useful cam3; the rest are staged.

1. **`rigcal` stereo sessions hard-break at three cams.**
   `StereoSession.capture` stores *every* connected monitor into each pose
   (`rig/rigcal.py:150-193`), then `status()`/`compute()` require
   `len(cams) == 2` (`rigcal.py:288-290, 369-401`) — with cam3 connected,
   **zero usable pairs, calibration dead**. Fix: capture from an explicit
   pair (default `("cam1","cam2")`) rather than "all connected". Small,
   surgical, must land before cam3 first joins a rig that still calibrates.
2. **Map tab literals.** `flight_log?cam=cam1`/`cam2` fetched by name
   (`rig/rig_ui.html:3044-3046, 3059, 3083, 3134`) — cam3's trail is
   invisible. Iterate `FLEET.nodes`.
3. **Review pair layout.** `.pairwrap` is a hard cam/chip/cam 3-column grid
   and the divider is injected at `i===1` (`rig_ui.html:118, 1045`) — a
   third `.camcol` wraps and breaks the row. Move to `auto-fit` like the
   Controls grid already does (`rig_ui.html:155`).
4. **Calib tab fixed slots** `clCov1/2`, `imuCal1/2`, result table
   `res.cams[0]/[1]` (`rig_ui.html:571-589, 3313-3316`) — degrade, don't
   break; render per-fleet when touched.
5. **`rigcal.IMU_SOURCES`** maps only cam1/cam2 (`rigcal.py:483`) — cam3 is
   silently IMU-less, which is *correct* until the day it grows one.
6. **`stereo_check.py`** is strictly the ILX pair (`{1:[],2:[]}`,
   `SENSOR_W_MM = 35.7`, `stereo_check.py:36-62`) — leave pairwise; it's an
   ILX tool. Ditto `vslam.py` stereo VO (two-view by construction;
   `--cam/--pair-cam` already parameterize which two).
7. **`ingest.py:243`'s `cam%d`** — hold the `name == camN` convention (§1)
   now; fix ingest to read run dirs by node name when it's next open.
8. **`deploy/deploy.sh`** — a `deploy_basler_node` branch: skip Sony SDK
   and `ilxctl.service`, install pylon udev rules + `baslerctl.service` +
   the venv, keep the shared piagent/chrony/power steps.
9. **`FRAME_EXTS` / content checks** — no change needed while `baslerctl`
   spools JPEG (`rigcore.py:1910`; `run.py:790-806`). Revisit only if raw
   Mono12 capture ever becomes a field format.
10. **`docs/PROTOCOL.md`** — document cam3's divergent :8080 (which
    endpoints exist, which return 4xx) and the nodes.json join; the fleet
    table check in `selftest.py:155-180` compares `_DEFAULT_NODES` against
    the documented *pair*, so the doc must present cam3 as the optional
    third, not a default row.

**Conformance gate:** `rig/tests/fakenode.py` is the de-facto protocol
spec; run the soaktest lanes that matter for a new backend — fake-node
fidelity (`soaktest.py:410-489`), NodeMonitor state machine (`:496-708`),
run layout/naming (`:961-1415`), the ENOSPC pull lane (`:1416-1520`, the
`.jpg.part` injector matches `baslerctl`'s atomic writes too), and strobe
routing (`:1789-1912`). A `baslerctl` that passes the fidelity lane's
expectations against a real camera is fleet-ready by definition.

---

## 5. Implementation phases

| Phase | Work | Gate |
|---|---|---|
| A | Camera standalone on the Pi | `BASLER-SETUP.md` §7 exit criteria |
| B | `baslerctl` minimal (§2) + piagent env; bench node answers both ports | fakenode-fidelity expectations met against real hardware; `calibrate_trigger` semantics rehearsed standalone |
| C | rigcal pair fix (§4.1); join fleet via nodes.json, **no runs yet** | header 3/3, `CAM_CONNECTED`, clock skew quiet 1 h, no red anomalies, Diag/Alerts sane |
| D | First 3-camera run | per-shot `skew_ms` includes cam3; `Cam3_*` frames land, flight_log rows complete, ingest associates; soaktest still green |
| E | UI passes (§4.2–4.4), PROTOCOL.md (§4.10), deploy branch (§4.8) | boat-ready |
| F | Strobe (§7) | its own bench gates |

---

## 6. What cam3 costs and buys

Costs: one more clock-skew pair to keep quiet, ~680 mA of USB budget on a
PoE-powered Pi (`BASLER-SETUP.md` §2.1), a second camera-control codebase
(small, but real), and the `name == camN` convention hardening into law.

Buys: a **global-shutter** third view (the ILX pair is mechanical/rolling —
the Basler freezes without the X-sync ceiling), 16 µs floor exposures for
strobe photometry, a 20 MP mono frame for VSLAM/photogrammetry
experiments — and, per §7, **the strobe problem gets a better home than the
Jetson.**

---

## 7. The strobe question: camera-direct, VA controller, or Pi

`xr256-jetson-trigger.md` put the XR256 on the Jetson because wiring into
the *existing* Pi enclosures is a field problem, and a bare 3.3 V GPIO can't
meet the PI input's sourcing 4–24 V floor anyway. cam3 is new hardware whose
enclosure is still on the drawing board — that changes the answer.

### 7.1 VA Imaging controller (VA-STRB-D1-V1): wrong box for this light

The controller (`docs/Industrial Strobe Controller | LED-Light |
Trigger.pdf`) is a **trigger-in → LED-power-out pulse switch for
driver-less heads**: opto-isolated 5–24 V NPN-style trigger input (≤10 µs
response, 50 kHz), DIP-switch pulse width 10 µs–1023 ms, and a single 2-pin
output that passes the 12–48 V supply rail to a dumb LED at **max 2 A
continuous / 4 A transient**. It has no logic/trigger output, no delay
feature, no remote programmability.

The XR256 is the opposite architecture: an integrated 180 A OverDrive
driver + SafeStrobe behind a logic trigger input. Chaining
controller→XR256 would point a power stage at a logic pin — switch
topology, off-state leakage, and edge behavior all undocumented — and its
one feature (pulse width) is ignored in PI mode and duplicated in
pulse-following. **Don't wire it.** Keep the box: if photometry
(`xr256-jetson-trigger.md` §6 test 11) says 0.5 J isn't enough and a
driver-less fill light joins the rig, this is precisely that light's
driver, triggered in parallel from the same edge. (Its trigger input is
also directly drivable by the Basler's open-collector GPIO — that pairing
is the vendor's own canonical hookup.)

### 7.2 Camera-direct: electrically yes, galvanically no

The a2A4504-18umBAS has **no opto output** — one opto *input* (Line 1) and
two open-collector GPIO lines (sink ≤50 mA, tolerate 24 V pull-ups,
internal ≈650 Ω/3.3 V pull-up, rise <2.5 µs, jitter <100 ns). Three facts:

- **Bare GPIO fails PI**: idles at 3.3 V — under the 4 V floor.
- **GPIO + external pull-up to 24 V into PI**: in-spec (safe range
  3.3–24 V) and fires — with two bench checks: the high level under the PI
  input's *unspecified* current draw, and whether the internal 3.3 V
  pull-up dividers the high level down (the 24 V rating implies it's
  handled; scope it).
- **GPIO straight into the pulse-following NPN terminal**: zero external
  parts — the line *is* a sinking NPN open-collector, and the camera's
  Timer (ExposureStart-keyed, delay+width programmable at 0.01 µs
  resolution) generates the flash-width pulse in hardware.

So the camera *can* trigger the XR256 directly. But the direct wire ties
the camera's GPIO ground — which is Pi ground, USB ground, rig ground —
into the terminal block of a light that pulls **20 A recharge bursts** on
its 24 V return. That coupling is exactly what the original analysis
declared non-negotiable to isolate (~0.66 V resistive + ~2 V L·di/dt of
ground bounce, `xr256-jetson-trigger.md` §2). The verdict doesn't change
because the trigger source got more convenient: **one optocoupler stage
between signal source and the 24 V domain stays mandatory in every
variant.**

There is also a line-budget conflict: cam3 has exactly two GPIO lines, and
the rig design (§0) spends both — Line 2 trigger-in, Line 3
ExposureActive→BCM 22. A camera-driven strobe re-purposes Line 3
(`LineSource=Timer1Active`), which forfeits the gpiomon exposure edges and
with them the `gpio_edge` acceptance verdict — unless edge feedback moves
to camera chunk timestamps, which is honest (`epoch_hw` with a real
`hw_err_ms` from the 1 GHz clock latch) but is new host-visible plumbing.

### 7.3 Recommendation: strobe on cam3's Pi, existing piagent path

Wire the strobe to **cam3's Pi**, not the Jetson and not the camera:

```
Pi BCM (WILDSYNC_STROBE_BCM) ──R(~270 Ω)──▶ opto LED (~8 mA)   [3.3 V domain]
                                   │
                              optocoupler                       [the only crossing]
                                   │
                     +24 V ──▶ collector; emitter ──▶ XR256 PI (pin 3)
                              PI ──4.7 kΩ──▶ XR256 GND (pin 5)  [24 V domain]
```

Why this wins:

- **The software already exists and is tested.** piagent's scheduled strobe
  (`strobe_at_epoch` riding `/gpio/fire`, standalone `POST /gpio/strobe`),
  host config (`set_strobe` — cam3 is a valid strobe node the moment it's
  in the fleet, `rig/run.py:1482-1488`), and the N-way `[fall, rise]`
  acceptance intersection all run **unchanged**. The entire Jetson software
  work list (`xr256-jetson-trigger.md` §5: Tegra gpiochip port, camera-less
  node concept, ILX_DOWN gating, chrony re-reference, Tegra scheduling QA)
  **evaporates**. Set `WILDSYNC_STROBE_BCM` on cam3, pick the pin with the
  boot-state audit, done.
- **The interface board gets simpler.** A Pi GPIO sources ~8 mA happily, so
  the opto LED is driven directly — the Jetson's mandatory buffer stage
  (its header pins are ~1 mA signal-only) is deleted. The board is
  literally the §2.1 reference design minus its first transistor.
- **Timing is already proven on this class of hardware.** The Pis' plain
  busy-wait (piagent has no SCHED_FIFO, despite what the Jetson brief
  assumed — `rig/piagent.py:855-958` is a coarse sleep + spin) delivers the
  sub-ms `late_ms` envelope the acceptance math was built on; δ margin is
  milliseconds.
- One caveat inherited knowingly: strobe availability now couples to cam3's
  *node* (not its camera — `ILX_DOWN` deliberately doesn't suppress the
  standalone strobe, `rig/run.py:2531-2543`; the same holds for a wedged
  `baslerctl`). If cam3's Pi dies, the flash dies with it. The Jetson plan
  had the same property on different hardware.

**Precision upgrade, later, if ever needed:** the camera-Timer path (§7.2,
pulse-following NPN through the same opto board) replaces piagent's
software-scheduled edge with a hardware-timed pulse — sub-µs δ and width —
at the cost of the Line 3 re-purpose and the chunk-timestamp edge plumbing.
The current acceptance budget is milliseconds wide; nothing today justifies
that trade. Characterize it once on the bench (`BASLER-SETUP.md` §7
Phase 6) so the option is quantified, then leave it on the shelf.

What carries over from `xr256-jetson-trigger.md` untouched: §1 (datasheet
dictates), §3 (PI mode, dial 250 µs starting point), §4 (the 24 V rail —
regulated ±5 %, bulk caps at the light, fused, single-point light-side
ground), §6's phased tests (re-hosted: "Jetson" → "cam3 Pi"; the pin audit
is now a Pi 5 `pinctrl-rp1` pin; the photometry go/no-go and burst-draw
measurements are identical), and every §7 open question except the Jetson
HTE item.

---

## 8. Open questions

1. **cam3's optical role** — lens choice (C-mount, must cover 1.1"), and
   whether the Basler frame joins photogrammetry/VSLAM or serves as strobe
   photometry + context. Decides nothing above; decides Gain/exposure
   defaults and the eventual Calib-tab treatment.
2. **XR256 PI input current** — still unspecified; sizes the opto stage and
   the §7.2 pull-up variant. Bench item, unchanged.
3. **ISO↔dB convention** (§3) — confirm ISO 100 ≡ 0 dB is the mapping the
   photometry work wants, before it fossilizes into `baslerctl`.
4. **Pi 5 PoE HAT USB budget** — verify the chosen HAT's 5 V rail truly
   supports `usb_max_current_enable=1` plus the camera's peaks
   (`BASLER-SETUP.md` §2.1); otherwise a powered hub joins the enclosure.
5. **Spool pruning** — nothing prunes the ILX spool today
   (`PI-SETUP.md` §1); the Basler spool has the same property and no
   camera-card backup tier behind it. Decide whether `baslerctl` prunes
   after verified host pull (the delete endpoint already implies yes).
6. **Strobe pin choice on the Pi 5** — which BCM becomes
   `WILDSYNC_STROBE_BCM`, with the boot-state audit across power cycles
   (the FOCUS-incident rule) before the opto board is connected.

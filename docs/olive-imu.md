# Olive olixVision X1 IMU — setup, configuration and fusion roadmap

Each camera carries an **Olive Robotics olixVision X1** IMU on its node, and each
camera's `flight_log.csv` takes its attitude from that camera's own IMU. (The
2026-08-20 field transects used one Yahboom YB-MRA02 on cam1 instead: its
solution is in both cameras' logs. The Olives were first run on the rig on
2026-08-27.) This file covers how the Olive connects, how it is installed and
checked, the driver's configuration, and the fusion design that is **not yet
implemented**.

## How it connects

The unit's USB-C is a CDC-Ethernet gadget, not a serial port. `deploy.sh node`
pins it on the node as interface `olive0` (Pi `192.168.7.2/24`, unit
`192.168.7.100`, a subnet of its own that cannot collide with the rig's
192.168.1.0/24), matched on the `cdc_ether` driver in
`/etc/netplan/60-wildsync-olive.yaml`. The unit serves, among others, a web UI on
80, ssh on 22, JupyterLab on 8888, and WebSocket streams on 5400, 5500, 5530 and
7070. **Port 5500 is the sensor stream**: protobuf-framed binary messages
carrying quaternion, accelerometer (m/s²), gyro (rad/s), magnetometer (mG) and
identity strings (`AHRS`, `x1-pro`). The ROS 2 / DDS path the unit advertises
needs a DDS stack the Pi image cannot build, so it is not used.

`rig/olive_ws_bridge.py` (`deploy/olive-bridge.service`, installed on every node;
it idles harmlessly where no Olive exists) decodes that stream without a schema,
converts it to the rig's units (degrees, g, deg/s) and sends one flat JSON
datagram per sample to `udp://127.0.0.1:9901`. `piagent` reads it through its
Olive driver (`rig/imu_olive.py`) and serves it like any other IMU. The unit's own
clock is wrong by about two years, so samples carry the arrival epoch only.

| piece | what it does |
|---|---|
| `rig/olive_ws_bridge.py`, `deploy/olive-bridge.service` | the WebSocket-to-UDP relay above |
| `rig/imu_olive.py` | the driver; the same duck-typed interface as `imu_yb.ImuReader` (`probe()`, `latest()`/`window()`, `rate_hz()`/`frame_rate_hz()`, `rejected_frames()`, `last_attitude_epoch()`, `checksum_state()`, `orientation_frozen_s()`); three transports (serial CDC-ACM, UDP, simulated) behind one tolerant codec |
| `rig/piagent.py` | two IMU slots: the primary (`/imu/*`) and a second (`/imu2/*`), each with the full acquire/health machinery; `/health`'s `imu` section reports both |
| `deploy/deploy.sh node` | writes `/etc/default/piagent` so the Olive is the node's IMU (below) |
| `rig/tests/audit_piagent.py` | offline tests for the slots: endpoint shapes, wedged-slot isolation, spec parsing, per-slot floors, an end-to-end simulated acquisition, garbage-ingest safety |

## Installing and checking

`deploy/deploy.sh node camN` configures the Olive: the `olive0` link, the bridge
unit and `PIAGENT_IMU=olive:udp:9901` with `PIAGENT_IMU2=off` in
`/etc/default/piagent`, which puts the Olive on the primary slot. For a node that
still carries the Yahboom unit, `WILDSYNC_NODE_IMU=yb deploy/deploy.sh node camN`
leaves the Yahboom primary and puts the Olive on the second slot
(`PIAGENT_IMU2=olive:udp:9901`). The check, from `docs/PI-SETUP.md` §6.1:

```sh
ssh ubuntu@192.168.1.20N 'ip -br addr show olive0; systemctl is-active olive-bridge'
curl -s 192.168.1.20N:8081/health | python3 -c "import sys,json; print(json.load(sys.stdin)['imu'])"
```

`present: true` with an `attitude_hz` around 15 is healthy. If the unit is not
found, check its USB-C cable and `systemctl status olive-bridge` on the node. The
IMU tab shows each Olive live, and the Calib tab's **Calibrate both** records the
gyro bias, accelerometer norm and mounting attitude of each IMU from a ten-second
still window (refused if anything moves; saved to `~/rig/imu_calibration.json`).
The Olive's AHRS stream reports a zeroed gyro at rest, which is its own on-device
bias correction.

**Physical check.** With the boat still, `|(ax,ay,az)|` should be about 1 g and
roll and pitch near the mount's true attitude; rotate the housing by hand and
watch roll, pitch and yaw move in the right sense, in degrees. `unit_mode` in the
sample should read `si` or `rig`; `unknown` means the unit gate could not see
gravity and the numbers are unverified pass-through.

## Configuration: `PIAGENT_IMU` and `PIAGENT_IMU2`

Set in the piagent service environment (`/etc/default/piagent`) and restart
piagent; the slot re-probes every ~4 s, so plugging a unit in brings it online
with no configuration.

| value | behaviour |
|---|---|
| *(unset)* or `auto` | auto-probe: `/dev/ttyACM*` scan, then a UDP dwell on port 9901 |
| `off` / `0` / `none` | slot disabled; no thread, `/health` reports `present:false, enabled:false` |
| `olive` | auto-probe, explicitly |
| `olive:/dev/ttyACM0` | that serial device only |
| `olive:udp` / `olive:udp:9901` | UDP listener on the given port (default from `OLIVE_UDP_PORT`, 9901) |
| `olive:sim` | synthetic stream (offline testing) |

`PIAGENT_IMU` selects the primary-slot driver: unset or `yb` keeps the Yahboom
reader on `/imu/*`; `olive[:spec]` (the same spec forms) puts the Olive on the
primary slot and turns slot 2 off. Run the driver by hand with `python3 rig/imu_olive.py` (add a device, `udp:9901`
or `sim`): `present: true` with the codec and rates, or `present: false` with the
tail of the undecoded stream as `raw_tail_hex`.

### Why `/imu2/*` is a separate endpoint

`/imu/latest`'s success payload **is** the bare sample dict (no envelope), so
there is nowhere to hang a sub-object without polluting the key namespace
`flight_log` reads by name. And rigd elects the fleet's **master orientation
source** by probing `/imu/latest` on every node and taking the first answer with
an `epoch`; a unit answering there would turn master election into a race. With
the second slot absent every pre-existing payload is byte-identical.

## Sample shape (`/imu/latest`, `/imu2/latest`, window entries)

Same keys as the YB where the quantity is the same — flight_log's readers need
nothing new: `epoch, pitch, roll, yaw, heading` (deg), `ax, ay, az` (g),
`gx, gy, gz` (dps), `mx, my, mz` (as received; units unverified), `temp` (°C),
`pressure_pa` (null — no barometer claimed), `qw..qz` (when the stream carries
quaternions), `fresh`. Olive extras: `src` (transport), `unit_mode`, `device_ts`
(when the stream carries one).

**Timestamping caveat.** Olive epochs are ARRIVAL-stamped (the read/recvfrom
instant). Unlike imu_yb there is no measured wire cadence to count byte-times
against, so these are good to a few ms on an idle Pi and worse under load. Do not
bind them to capture instants at better than ~10 ms. The flight log takes the
nearest sample within 100 ms.

## The tolerant codec

`rig/imu_olive.py` accepts several encodings, auditioned until one produces
`LOCK_N=4` consecutive good records; the winner is then enforced and failures
become counted rejects:

* `json` — one object per line/datagram, keys by name (ROS shapes included)
* `nmea_csv` — `$TAG,f1,...*hh`, XOR checksum **verified when present**
* `csv` — bare comma-separated floats
* `bin_u8` / `bin_u16` — `[len][len bytes of float32]` over the byte stream
* `udp_f32` / `udp_f64` — a bare float vector per datagram (CDR header skipped)

The bridge emits JSON, which is what the rig uses. Unlabeled numeric records use
one documented field order (`_map_numeric`): `[device_ts?] [qw qx qy qz | roll
pitch yaw] [ax ay az] [gx gy gz] [mx my mz] [temp]`, with a quaternion claimed
only at unit norm within 1e-3. Units are inferred from gravity (`_UnitGate`):
median |accel| of the first 16 inertial records ≈ 1 ⇒ already g/deg/dps, ≈ 9.8 ⇒
SI (rad, m/s², rad/s), converted before anything is published; samples are
buffered until the verdict so maybe-radians never reach a consumer. If a stream
disagrees with a guess, each has one home in `rig/imu_olive.py`: field order in
`_map_numeric`, bare-list quaternion order in `_vec`, units in `_UnitGate`,
framing in `_Codec`. Every decode failure increments `rejected_frames()`;
`checksum_state()` carries the locked codec, `unit_mode`, error counters and, while
undecoded, the raw tail. **A rising reject count with a locked codec means a
plausibility gate is catching a wrong guess — read the raw tail, do not loosen the
gate.**

## Fusion roadmap — yb IMU + Olive IMU + magnetic heading + GPS

### What each source already provides

| source | quantities | rate | clock | where it enters |
|---|---|---|---|---|
| YB-MRA02 (cam1 Pi) | roll/pitch/yaw(+quat), accel g, gyro dps, mag raw, baro | ~50 Hz attitude / ~174 Hz frames | node clock, wire-dated to ~1 ms + fusion lag unknown | `/imu/window` (host converts node→host epoch via rigcore offsets) |
| Olive X1 (each camera's Pi) | same keys; about 15 Hz attitude as received at piagent | as received | arrival-stamped (see caveat) | `/imu/window` (primary slot) or `/imu2/window` |
| Boat heading | PGN 127250 vessel heading (+deviation, variation when the sender knows them) | ~10 Hz | host at NMEA arrival | `rig/nav.py` NavReader |
| GPS | PGN 129025 lat/lon rapid (~10 Hz), 129029 full GNSS (1 Hz, alt + fix quality), 129026 COG/SOG (~10 Hz) | 1–10 Hz | host; TimeAuthority disciplines from 126992 | `rig/nav.py` |

All fusion happens HOST-SIDE in rigd's nav layer (`rig/nav.py` grows a
`NavFusion` class; rigd feeds it, `run.py` reads it where `imu_snapshot` is
read today). Nodes stay dumb samplers — they already timestamp and ring;
nothing on a Pi needs to change for any phase below.

### Filter: error-state Kalman filter (ESKF), one vehicle, two witnesses

Both IMUs ride ONE rigid vessel, so there is one true attitude/velocity/
position and two witnesses of it with their own biases and mounting
rotations. Do not run two independent filters and average — that discards
exactly the cross-checking a second IMU is for.

**Nominal state** (propagated nonlinearly): attitude quaternion `q_nb`
(vehicle body→NED), velocity `v_n` (NED, m/s), position `p` (lat, lon —
work in the local UTM tangent plane via `latlon_to_utm`, alt from 129029 or
pinned to 0 for a boat).

**Error state** (the KF vector, 16):

    δx = [ δθ(3)  δv(3)  δp(3)  b_g1(3)  b_a1(3)  b_g2? ... ]

Start with the biases of the PROPAGATING IMU only (16 states); add the
second IMU's gyro bias (19) once residuals demand it.

**Prediction:** strapdown mechanization at the faster IMU's rate (expected:
the olive) — integrate gyro into `q_nb`, rotate accel to NED, remove
gravity, integrate to `v_n`, `p`. `Q` from the datasheet noise densities
until Allan-variance numbers exist from a dockside log (an hour still at
the dock gives both IMUs' bias instability for free).

**Measurement models** (all with 3σ chi-square gating so one bad sentence
cannot yank the state):

* **Second IMU attitude** `z = q_1b→2b ⊗ q_nb` — the OTHER unit's fused
  attitude through the fixed mounting rotation `R_12` (survey it: both
  units' roll/pitch at the dock give it in one sitting). Innovation here is
  the disagreement between the two IMUs, which is the health metric nothing
  else provides: alarm when it exceeds the stereo rig's angular error
  budget.
* **Gravity / roll-pitch:** each IMU's accel when `| |a| − 1g | < 0.05 g`
  (reject during turns/waves by that gate) observes roll/pitch directly.
* **Magnetic heading:** `z = ψ + variation(WMM or PGN 127250's own field) +
  deviation(table)`. This is the ONLY absolute yaw source — both IMU yaws
  drift or reference their own magnetometers with unknown calibration.
  Calibrate the deviation table with one slow circle while logging 127250
  against GPS COG at speed.
* **GPS position:** 129025 lat/lon → tangent-plane `p`; R from fix quality
  (129029): ~2–5 m single-point, sub-m RTK if the receiver has it.
* **GPS velocity:** COG/SOG (129026) → `v_n` measurement. On a boat COG ≠
  heading (current + leeway): couple COG to the VELOCITY state only, never
  to yaw, or the filter learns the crab angle as a heading error.

**Timing discipline:** every measurement enters at its host-epoch time using
the machinery that already exists — rigcore's node clock offsets for the IMU
rings, TimeAuthority for nav. Process the measurement queue in time order
with a small (~200 ms) reordering buffer; the flight_log query then asks the
filter for the state AT the capture instant (interpolate the two bracketing
filter states) instead of today's nearest-raw-sample.

**Outputs and flight_log:** keep every existing raw column untouched; APPEND
fused columns (`fused_roll, fused_pitch, fused_heading, fused_lat,
fused_lon, fused_err_deg, fused_err_m, imu_disagree_deg`) at the END of the
row — the CSV header is versioned by column presence, additive only, same
rule as every wire change in this repo.

### Phases (each independently useful)

* **Phase 0 — in place:** each IMU is ring-buffered and served by its node, and
  each camera's flight log takes attitude from its own node's IMU. No fusion, no
  risk.
* **Phase 1 — offline:** a standalone script replays a run's flight_log +
  `/imu2` ring dump + navlog through the ESKF; validate against
  `stereo_check` reprojection before anything touches the live path.
* **Phase 2 — online:** `NavFusion` in rigd, flight_log gains the fused
  columns; raw columns stay authoritative until a season of agreement.
* **Phase 3 — timestamps:** device_ts correlation for the olive (and the
  YB's fusion-lag estimate) so the filter's time base stops inheriting
  arrival jitter.

A complementary filter (Mahony-style) is NOT the plan for the host: the
devices already run their own onboard fusion — the host's job is exactly the
part a complementary filter cannot do (bias estimation, absolute heading,
position, and cross-IMU consistency with honest covariances).

## Open items

1. **Device timestamp.** The unit's own clock is ~2 years wrong, so samples carry
   arrival epochs only. Whether the stream carries a usable device timestamp that
   could be correlated with arrival time (constant slope + offset) is not
   established.
2. **Rate and floors.** Measure the real cadence over a long run and tighten
   `Imu2.FRAME_FLOOR_HZ` (a deliberately slack 10 Hz) and `PROBE_BAND` in
   `rig/piagent.py` from it, as the YB's floors were derived from its own
   measured normal.
3. **Magnetometer.** Units and frame of the `mx..mz` values, and whether the X1's
   own fusion uses them, are unconfirmed.
4. **Mounting rotation `R_12`** between the two IMU housings: survey it at the
   dock; the fusion's cross-IMU residual needs it first.

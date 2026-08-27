# Olive olixVision X1 — second IMU: bring-up runbook + fusion roadmap

Written 2026-08-27, the night before first power-up. The unit is physically
mounted at cam2 but not yet cabled, so **everything below the "software in
place" section is a plan against an unconfirmed wire format** — the driver was
built tolerant for exactly that reason, and the first job tomorrow is to let
it tell us what the device actually speaks.

## Software in place tonight

| piece | what it does |
|---|---|
| `rig/imu_olive.py` | driver; same duck-typed interface as `imu_yb.ImuReader` (`probe()`, `read()/latest()/window()`, `rate_hz()/frame_rate_hz()/rate_measured()`, `rejected_frames()`, `last_attitude_epoch()`, `checksum_state()`, `orientation_frozen_s()`); three transports (serial CDC-ACM, UDP, sim) behind one tolerant codec |
| `rig/piagent.py` | a second IMU slot `Imu2` (the acquire/health machinery of `Imu`, parameterized); config via `PIAGENT_IMU2`; new `GET /imu2/latest` and `GET /imu2/window?t0=&t1=`; `/health` `imu` section gains a nested `imu2` sub-object |
| `rig/tests/audit_piagent.py` | P10 section: absent-by-default byte-compatibility, endpoint shapes, wedged-slot isolation, spec parsing, per-slot floors, an end-to-end sim acquisition, garbage-ingest safety. Suite: **104 passed, 0 failed** offline |

**Byte-compatibility contract:** with the olive absent, every pre-existing
payload (`/imu/latest`, `/imu/window`, `/health`'s `imu` keys) is unchanged —
the only addition anywhere is the `imu2` sub-object inside `imu` and the two
new endpoints. rigd, run.py and flight_log need **no changes** for tonight's
deploy; they keep reading the YB exactly as before.

### Why `/imu2/*` endpoints and not an `imu2` key inside `/imu/*`

`/imu/latest`'s success payload **is** the bare sample dict (no envelope), so
there is nowhere to hang a sub-object without polluting the key namespace
`flight_log` reads by name. Worse, rigd elects the fleet's **master
orientation source** by probing `/imu/latest` on every node and taking the
first answer with an `epoch` — if cam2's olive answered there, master
election would become a race between the two units. Separate endpoints keep
the olive invisible to that election until the host is explicitly taught
about it.

### Config: `PIAGENT_IMU2`

| value | behaviour |
|---|---|
| *(unset)* or `auto` | auto-probe: `/dev/ttyACM*` scan, then a UDP dwell on port 9901 |
| `off` / `0` / `none` | slot disabled; no thread, `/health` reports `present:false, enabled:false` |
| `olive` | auto-probe, explicitly |
| `olive:/dev/ttyACM0` | that serial device only |
| `olive:udp` / `olive:udp:9901` | UDP listener on the given port (default from `OLIVE_UDP_PORT`, 9901) |
| `olive:sim` | synthetic stream (offline testing) |

Set it in the piagent service environment (same mechanism as `PIAGENT_PORT`),
restart piagent — or set nothing: auto-probe re-runs every ~4 s, so simply
plugging the unit in should bring it online with **zero config**.

## Bring-up tomorrow

### Case A — USB-C into cam2's Pi (CDC-ACM serial)

1. Cable the unit; on cam2's Pi: `ls /dev/ttyACM*` and `dmesg | tail -20`.
   No ttyACM ⇒ cable/power problem (or the unit's USB is configured as
   network-only — see the ROS note below), stop here and fix that first.
2. Probe by hand before trusting the automation:
   `python3 ~/rig/imu_olive.py` (or with the device:
   `python3 ~/rig/imu_olive.py /dev/ttyACM0`). Three possible outcomes:
   * `present: true` with `codec`, `sample_rate_hz`, `frame_rate_hz`, then 20
     live sample lines — done, skip to step 4.
   * `present: false` with a note `N bytes seen ... undecoded` and a
     `raw_tail_hex` — **this is the designed outcome for a format the codec
     has not met.** Copy the hex tail; the codec cascade and where to extend
     it are in the next section. Nothing needs a redeploy to keep probing.
   * `no data` — the port enumerates but streams nothing: the unit likely
     needs its streaming mode enabled in Olive's own configuration (olixOS
     web UI / `olix` CLI over the same USB network interface). Configure it
     to stream IMU data on the serial CDC or as UDP, then re-probe.
3. If the by-hand probe decodes, piagent's auto-probe will too (same code).
   The slot re-probes every ~4 s; watch it come online:
   `curl -s cam2:8081/health | python3 -m json.tool` → `imu.imu2.present`.
   While it is still absent, `imu.imu2.probe.notes` / `.probe.raw_tail_hex`
   carry the last probe's evidence, so this is diagnosable from the host
   without a shell on the Pi.
4. Verify payloads:
   * `curl -s cam2:8081/imu2/latest` → a sample dict (shape below).
   * `curl -s "cam2:8081/imu2/window?t0=$(date +%s -d '-10 sec')&t1=$(date +%s)"`
     → `{present:true, samples:[...]}` (ring holds ≥ 60 s).
   * `/health` `imu.imu2`: `attitude_hz`, `frame_hz`, `imu_rate_low`,
     `rejected_frames`, `checksum` (which for the olive carries the codec
     lock, `unit_mode`, error counters and — while undecoded — the raw tail).
5. **Physical sanity, do not skip:** with the boat still, `|(ax,ay,az)| ≈
   1 g` and roll/pitch near the mount's true attitude; rotate the housing by
   hand and watch roll/pitch/yaw move in the right sense and in degrees;
   `unit_mode` should read `si` (ROS-native stream) or `rig` — if it reads
   `unknown`, the unit-inference gate could not see gravity and the numbers
   are pass-through: treat them as unverified.

Port-sharing note: imu_yb's own probe scans `/dev/ttyACM*` too (every ~4 s,
0.5 s dwell). imu_olive opens its port `exclusive=True`, so once the olive
reader holds the port the YB probe fails instantly instead of siphoning
bytes. If the olive probe itself reports `could not exclusively lock`, the
other slot's dwell was mid-flight — it clears on the next probe cycle.

### Case B — Ethernet to the host (UDP raw streaming)

The X1 is ROS 2 native. The rig deliberately runs **no DDS stack** on the
nodes, so the Ethernet path is either the unit's raw/UDP streaming mode or a
minimal bridge:

1. Preferred: configure the unit (olixOS) to stream IMU data as UDP to the
   listener's IP, port 9901. The codec accepts JSON, `$`-CSV, or a bare
   float32/float64 vector per datagram (a leading 4-byte CDR encapsulation
   header is skipped automatically).
2. Fallback bridge (if only ROS 2 topics are available): on any machine that
   can see the unit's DDS domain, ~20 lines of rclpy — subscribe
   `sensor_msgs/Imu`, `json.dumps` the fields, `sock.sendto` one line per
   message to the listener. The driver's JSON mapper already understands the
   ROS message shape (`orientation{x,y,z,w}`, `angular_velocity{x,y,z}`,
   `linear_acceleration{x,y,z}`) verbatim.
3. Who listens: any piagent with `PIAGENT_IMU2=olive:udp:9901` serves it on
   its `/imu2/*` — point the unit at that node. If instead the stream should
   terminate on the run host itself, `imu_olive.py` runs standalone anywhere
   (`python3 rig/imu_olive.py udp:9901`); wiring a host-side slot into rigd
   is deliberately NOT built tonight — decide after we see which transport
   the hardware favours. **Watch the subnet trap** (memory:
   rig-network-subnet-collision) before assuming the datagrams can reach the
   host at all.

### Case C — no hardware at all (works today)

`PIAGENT_IMU2=olive:sim` or `python3 rig/imu_olive.py sim` exercises the
whole path: NMEA-style CSV in SI units → checksum verify → unit inference →
rig-unit samples at ~50 Hz. This is what the audit suite runs.

## The tolerant codec — what it accepts, where to fix a wrong guess

Candidate encodings audition until one produces `LOCK_N=4` consecutive good
records; the winner is then enforced and failures become counted rejects:

* `json` — one object per line/datagram, keys by name (ROS shapes included)
* `nmea_csv` — `$TAG,f1,...*hh`, XOR checksum **verified when present**
* `csv` — bare comma-separated floats
* `bin_u8` / `bin_u16` — `[len][len bytes of float32]` over the byte stream
* `udp_f32` / `udp_f64` — a bare float vector per datagram (CDR header
  skipped)

Unlabeled numeric records use ONE documented field-order guess
(`_map_numeric`): `[device_ts?] [qw qx qy qz | roll pitch yaw] [ax ay az]
[gx gy gz] [mx my mz] [temp]` — quaternion claimed only at unit norm within
1e-3 (a fusion output is normalized; a loose band let euler records
masquerade). Units are inferred from gravity (`_UnitGate`): median |accel| of
the first 16 inertial records ≈ 1 ⇒ already g/deg/dps, ≈ 9.8 ⇒ SI
(rad, m/s², rad/s), converted before anything is published. Samples are
**buffered until the verdict** so maybe-radians can never reach a consumer.

When tomorrow's stream disagrees with a guess, each has exactly one home:

| wrong guess | fix in `rig/imu_olive.py` |
|---|---|
| field order | `_map_numeric` |
| bare-list quaternion element order | `_vec` (the `n == 4` branch) |
| units | `_UnitGate` (or force by post-lock config once known) |
| framing | add the real frame spec to `_Codec` and let it win the audition |

Every decode failure increments `rejected_frames()`; `checksum_state()`
carries `algo` (locked codec), `unit_mode`, `errors`/`last_error`,
`bytes_seen`, and `raw_tail_hex` while undecoded. **A rising reject count
with a locked codec means a wrong guess is being caught by the plausibility
gates — that is the signal to go read the raw tail, not to loosen the gate.**

## Sample shape (`/imu2/latest`, `/imu2/window` entries)

Same keys as the YB where the quantity is the same — flight_log's readers
need nothing new: `epoch, pitch, roll, yaw, heading` (deg), `ax, ay, az`
(g), `gx, gy, gz` (dps), `mx, my, mz` (as received; units unverified),
`temp` (°C), `pressure_pa` (null — no baro claimed yet), `qw..qz` (when the
stream carries quaternions), `fresh`. Olive extras: `src` (transport),
`unit_mode`, `device_ts` (when the stream carries one).

**Timestamping caveat, do not forget it:** olive epochs are ARRIVAL-stamped
(the read/recvfrom instant). Unlike imu_yb there is no measured wire cadence
to count byte-times against yet, so these are bring-up-grade epochs — a few
ms on an idle Pi, worse under load. Do not bind them to capture instants at
better than ~10 ms until the device_ts correlation below is done.

## Post-bring-up tightening (in order)

1. Measure the real cadences; tighten `Imu2.FRAME_FLOOR_HZ` (now a
   deliberately-slack 10 Hz) and `PROBE_BAND` in `rig/piagent.py` from the
   probe figures, the same way the YB's floors were derived from its own
   measured normal.
2. Map `device_ts` (if present) against arrival time over a few minutes:
   constant slope + offset ⇒ convert to wall time and replace arrival
   epochs, imu_yb-style honesty about the residual.
3. Confirm mag units/frame and whether the X1's fusion already uses it.
4. Decide the Ethernet-vs-serial question on measured jitter, not taste.

## Fusion roadmap — yb IMU + Olive IMU + magnetic heading + GPS

### What each source already provides

| source | quantities | rate | clock | where it enters |
|---|---|---|---|---|
| YB-MRA02 (cam1 Pi) | roll/pitch/yaw(+quat), accel g, gyro dps, mag raw, baro | ~50 Hz attitude / ~174 Hz frames | node clock, wire-dated to ~1 ms + fusion lag unknown | `/imu/window` (host converts node→host epoch via rigcore offsets) |
| Olive X1 (cam2 Pi or host) | same keys; quality/rate TBD (expect 100–400 Hz) | TBD | arrival-stamped (see caveat) | `/imu2/window` |
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

* **Phase 0 — tonight's code already does it:** both IMUs ring-buffered and
  served; a run records both raw streams. No fusion, no risk.
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

## Open questions for tomorrow

1. Which transport does the unit actually present — CDC-ACM serial, USB
   network interface, or Ethernet-only ROS 2? (Determines Case A vs B.)
2. What does it stream un-configured, if anything — and what does the raw
   tail look like if the codec doesn't lock? (Paste the hex into the next
   session; the codec cascade is built to be extended in one place.)
3. Real rates and units (`unit_mode` verdict), then tighten the floors.
4. Does its stream carry a device timestamp, and is it disciplined?
5. Where should the UDP stream terminate if Case B wins — a camera Pi
   (served via `/imu2/*` like tonight's plumbing assumes) or the run host
   (needs the standalone-reader decision)?
6. Mounting rotation `R_12` between the two IMU housings — survey at the
   dock; the fusion's cross-IMU residual needs it first.

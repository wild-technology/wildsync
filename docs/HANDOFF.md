# Handoff — Wild Sync stereo rig

State as of 2026-08-16. Everything below was measured on the live rig, not
inferred. `docs/PROTOCOL.md` is the interface contract; this document is the
operational picture: what is built, what is proven, and what is left.

---

## 1. Fleet

| node | address | hardware | camera | gpiochip | IMU | notes |
|---|---|---|---|---|---|---|
| **cam1** | 192.168.1.201 | Raspberry Pi 5 | `D516000F467B` | `gpiochip4` | yes | primary: IMU host, exposure preview, intended stream source |
| **cam2** | 192.168.1.202 | Raspberry Pi 4B | `D516000F46F7` | `gpiochip0` | no | |
| cam3 | 192.168.1.203 | — | — | — | — | slot reserved, no hardware |

Both nodes hold a static address **and** a DHCP lease, so a bad netplan cannot
strand one (`/etc/netplan/99-wildsync-static.yaml`). SSH as `ubuntu`, key-based.

Node addresses can be overridden without a code change — `~/rig/nodes.json` on
the Jetson, e.g. `{"cam2": "192.168.1.171"}`.

The Pi 4 and Pi 5 have identical 40-pin header pinouts; only the gpiochip name
differs, and `piagent` discovers it rather than assuming.

Services: `ilxctl` (:8080) and `piagent` (:8081) on each node, `rigd` (:9090) on
the Jetson. All systemd, all `Restart=always`.

## 2. Wiring

Molex Micro-Fit 3.0 6-pin on the camera, latch tab **down** (Sony manual p.414):

| harness pin | signal | Pi header | BCM | direction |
|---|---|---|---|---|
| 2 | GND | 9 | — | common ground — required |
| 4 | `FOCUS` | 11 | 17 | camera input, active Low |
| 5 | `TRIGGER` | 13 | 27 | camera input, active Low, needs FOCUS already Low |
| 6 | `EXPOSURE` | 15 | 22 | camera output, open-drain, Low during exposure |

Pins 1/3 are DC in; the cameras are powered separately. `docs/harness-and-strobe.md`
has the electrical detail and the timing diagram.

**PoE couples camera and Pi power.** Power-cycling a camera reboots its Pi. Plan
diagnostics around that, and expect the camera to come up while the Pi is still
booting.

## 3. How synchronised capture works

1. `rigd` measures each camera's TRIGGER→EXPOSURE latency (`calibrate_trigger`),
   at program start and again at run start. Currently ~22.4 ms and ~22.2 ms.
2. For each shot it picks one absolute target instant and tells every node to
   fire at `target − that node's own latency`, so the **exposures** coincide
   rather than the pulses.
3. `piagent` busy-waits to its instant on a chrony-disciplined clock, asserts
   `FOCUS` 40 ms ahead, pulses `TRIGGER`, and releases both — all as in-process
   register writes via libgpiod.
4. The camera's `EXPOSURE` edge is the recorded capture instant. Each frame
   claims the oldest queued fire command and takes the edge nearest it.

Two design points worth keeping:

- **FOCUS is per-shot, not held for the run.** Holding it is a permanent
  half-press, which AE-locks the body and would freeze auto-ISO at whatever the
  light was when the run started.
- **Never shell out per shot.** `piagent` used `gpioset` subprocesses; that cost
  2.83 ms on the Pi 5 and 12.39 ms on the Pi 4, a silent ~9.6 ms inter-camera
  skew. The in-process path removed it.

## 4. What is proven

- Camera control, live view, and settings convergence across both bodies.
- GPIO trigger: 22.1 ms, sd 0.4–0.9 ms. Inter-camera skew 0.65 ms mean.
- Full pathway: fire → EXPOSURE edge → frame transfer → `CamN_YYYYMMDD_hhmmss.ss.jpg`
  → 23-column `flight_log.csv`, both cameras, `capture_source=gpio_edge` on every row.
- Desired-state convergence survives a power cycle: both bodies came back and
  re-applied f/8 · 1/200 · ISO AUTO with no intervention.
- IMU (Yahboom YB-MRA02, 0x7E protocol @115200 on cam1) at ~200 Hz.
- Regression suites green: `selftest` 48/48, `nav`, `navlog`, `navtest` all pass.

## 5. What is not

- **iKonvert NMEA 2000 gateway.** The driver, raw logging, PGN decode, UTM
  (cross-checked against pyproj to <5 cm) and time authority are written and
  unit-tested, but **no `$PDGY` byte has ever come out of the unit**. The gateway
  is bus-powered through a galvanic isolator, so with the N2K connector dry only
  the FTDI chip is alive — silence is the correct signature, not a fault.
  To finish: put 12 V on the micro-C connector, check the POWER LED, then
  `python3 rig/nav.py --probe`. Expect `online_no_bus` and a 1 Hz
  `$PDGY,000000,,,,,,,`. Baud is **230400** (RAW/PDGY mode), not 115200.
  Verify the Xplore 9 depth offset convention against the sounder's own display
  before trusting `depth_from_xplore9`.
- **Sustained 2 fps.** Achieved ~0.57 s per frame against a requested 0.5 s, with
  occasional dropped frames (20 vs 21 across the pair) and skew degrading to
  1.5 ms mean / 10 ms worst. The scheduled-fire path itself holds sub-ms; the
  limit is frame delivery over USB while the camera is also writing to card.
  Try: PC-save only (`store_dest=1`), smaller JPEG, and confirm the camera can
  sustain the delivery rate before blaming the scheduler.
- **cam3.** No hardware. The slot exists in `rigcore._DEFAULT_NODES` and will
  report OFFLINE until populated.
- Open findings from the soak harness, none of them data-destroying: the worker
  set is fixed at run start (a node that joins mid-run gets no folder), `stop()`
  does not join workers, `EventLog` has no rotation.

## 6. Diagnostics that pay for themselves

- `GET /api/status` on a node returns `writable:{}` — the per-property
  `CrPropertyEnableFlag`. All `2` (DisplayOnly) with `priorityKey:1` means the
  **body** is locked, not that the software is broken.
- Every read-only error carries the flag plus `opmode`, `keylock`, `slot1`,
  `program` and `CAUTION ON BODY`. A caution is a modal on a screenless body:
  unseated lens, locked iris, card format. Power-cycle or HDMI to clear.
- `GET /gpio/state` reports `harness_safe`, `trigger_path`, `edges_bounced`.
  A rising `edges_bounced` means a ringing EXPOSURE line — cam2's harness does
  this; the 1 ms debounce absorbs it, but the wire wants shielding.
- `GET /api/diag` on the Jetson: fleet, time model, nav gateway health, anomalies.
- Frame counts: a node whose `/api/shots` stops growing while the camera clearly
  fires usually means `SetSaveInfo` failed — check the save directory exists.

## 7. Immediate next steps

1. Power the N2K bus and finish nav verification (§5).
2. Chase the 2 fps frame drops — instrument delivery rate vs fire rate per node.
3. Commission cam3 when hardware arrives: `deploy.sh provision cam3`.
4. Shield or shorten cam2's EXPOSURE lead.
5. Re-run `soaktest.py --soak 300` and clear the remaining findings.

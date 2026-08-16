# Wild Sync

Control software for a multi-camera **Sony ILX-LR1** underwater stereo rig used
for seafloor photogrammetry.

A Jetson orchestrates a fleet of Raspberry Pi camera nodes over Ethernet. Each
node drives one ILX-LR1 over USB through Sony's Camera Remote SDK and fires it
through a GPIO trigger harness. The Jetson schedules every camera to expose at
one shared instant, pulls the frames, renames them, and writes a per-camera
flight log binding each frame to position and orientation.

```
                    ┌──────────────────────────────────────────┐
   browser  ───────►│  Jetson :9090   rigd + web UI            │
                    │  fleet state · desired settings · runs   │
                    └───┬───────────────────────┬──────────────┘
                        │ Ethernet              │
              ┌─────────▼─────────┐   ┌─────────▼─────────┐
              │ cam1  Pi 5        │   │ cam2  Pi 4        │
              │  ilxctl  :8080 ───┼───┼─► USB ─► ILX-LR1  │
              │  piagent :8081 ───┼───┼─► GPIO harness    │
              │  + IMU (master)   │   │                   │
              └───────────────────┘   └───────────────────┘
```

## Measured performance

All figures from the live rig, measured 2026-08-16 against each camera's own
`EXPOSURE` line (hardware ground truth), not host-side estimates.

| | |
|---|---|
| GPIO trigger → exposure | **21.8–21.9 ms** (cam1 21.94, cam2 21.83) |
| USB release → exposure | 68–76 ms, sd 5–11 ms, quantised to the camera's ~30 Hz internal tick |
| **Inter-camera skew, scheduled fire** | **0.59 ms mean, 1.82 ms worst** |
| Host fire lateness vs scheduled instant | +0.1–0.3 ms mean, 0.4 ms max |
| Node clock discipline (chrony → Jetson) | ~85 µs RMS |
| IMU sample rate | ~174 Hz measured |

Use the GPIO path for survey work. USB release is the fallback when a node has
no harness, and costs roughly 30 ms of timing uncertainty.

**The sustained-rate figures are withdrawn.** Every one on record was taken with
`transsize=Small` — 320 KB review thumbnails, not survey frames — so they say
nothing about the real archive path. Re-measure before relying on a frame rate.

### How synchronised capture works

1. `rigd` measures each camera's TRIGGER→EXPOSURE latency (`calibrate_trigger`),
   at program start and again at run start.
2. For each shot it picks one absolute target instant and tells every node to fire
   at `target − that node's own latency`, so the **exposures** coincide rather than
   the pulses.
3. `piagent` busy-waits to its instant on a chrony-disciplined clock, asserts
   `FOCUS` 120 ms ahead, pulses `TRIGGER`, and releases both — all as in-process
   register writes via libgpiod.
4. The camera's `EXPOSURE` edge is the recorded capture instant.

Three things worth keeping:

- **`FOCUS_LEAD_MS` is the single biggest lever on sync accuracy.** At 40 ms the
  skew was 3.14 ms mean / 7.22 ms worst; the 80–200 ms plateau gives 0.59 ms /
  1.82 ms. Everything host-side is already sub-millisecond, so all remaining skew
  is inside the camera bodies.
- **`FOCUS` is per-shot, not held for the run.** Holding it is a permanent
  half-press, which AE-locks the body and freezes auto-ISO.
- **Never shell out per shot.** `gpioset` subprocesses cost 2.83 ms on the Pi 5 and
  12.39 ms on the Pi 4 — a silent ~9.6 ms inter-camera skew. The in-process path
  removed it.

## Layout

```
src/            ilxctl — C++ camera daemon (one per node, :8080)
rig/            Python: rigd (Jetson), piagent (node), nav, run, UI
rig/tests/      regression + soak suites (no hardware required)
deploy/         deploy.sh and the systemd units
docs/           PROTOCOL.md (interface contract), HANDOFF.md (state + TODO), hardware notes
```

`docs/PROTOCOL.md` is the contract every component is built against — node HTTP
APIs, the 23-column `flight_log.csv` header, run layout, the time model, and the
measured Sony property encodings. Read it before changing anything that crosses
a process boundary.

## Where things run

| Process | Host | Port | Language | Owns |
|---|---|---|---|---|
| `rigd` | Jetson | 9090 | Python 3, stdlib | fleet state, desired settings, runs, web UI |
| `ilxctl` | each Pi | 8080 | C++17 | the camera over USB via Sony's SDK — settings, live view, image transfer |
| `piagent` | each Pi | 8081 | Python 3, stdlib | GPIO harness (FOCUS/TRIGGER/EXPOSURE), IMU, node health |

All three are systemd units with `Restart=always`. `rigd` talks to both node
services over plain HTTP; nothing else crosses a machine boundary.

State on disk:

| Path | Host | What |
|---|---|---|
| `~/rig-runs/<run_id>/` | Jetson | one directory per transect: `run.json`, `events.log`, `nmea_raw.log`, and a `camN/` per camera holding renamed frames + `flight_log.csv` |
| `~/rig/desired.json` | Jetson | the fleet's agreed settings vector, survives restart |
| `~/rig/rigd.jsonl` | Jetson | rolling structured event journal |
| `~/rig/nodes.json` | Jetson | optional address override, e.g. `{"cam2": "192.168.1.171"}` |
| `~/Pictures/ILX-LR1/` | each Pi | PC-save staging dir the camera writes into; `rigd` pulls from here. **Nothing prunes it yet.** |

## Dependencies

**Jetson (`rigd`):** Python 3.12, `python3-serial` (nav), **`python3-pil`**.

Pillow is easy to miss and fails quietly — it backs the EXIF fallback for a
frame's capture instant, and without it frames with no GPIO edge are stamped with
the command time instead. `rigd` now emits a `warn` at run start if it is absent,
but install it up front:

```sh
sudo apt install -y python3-serial python3-pil
```

**Camera nodes (`piagent`):** Python 3.12, `python3-libgpiod` (v1.6.3 API),
`gpiod`/`gpioset`/`gpiomon` CLI tools. `deploy.sh provision` handles chrony, the
usbfs buffer and the gpio udev rule.

`piagent` and `ilxctl` need no Python packages beyond libgpiod. Services are
stdlib-only by policy — see `docs/PROTOCOL.md`.

## Build

`ilxctl` builds on Linux (aarch64) and macOS. It is **always built on the machine
it runs on**: the Makefile probes `-mcpu=native`, so a binary built on one node
SIGILLs on another.

Sony's Camera Remote SDK is not redistributed here. Stage it from your own copy
of the official package:

```sh
make sdk CRSDK_DIR=/path/to/unpacked/RemoteCli
make
```

## Deploy

```sh
deploy/deploy.sh provision cam2    # first time on a bare node: SDK, chrony, usbfs
deploy/deploy.sh node cam2         # update sources, rebuild, restart services
deploy/deploy.sh jetson            # install rigd here
```

Then open `http://<jetson>:9090`.

`deploy.sh node` ships `src/*`, `rig/piagent.py` and `rig/imu_yb.py`, rebuilds
`ilxctl` on the node and restarts both services. It does **not** ship `rigd` —
that runs from this checkout on the Jetson, so a `rigd` change takes effect on
`sudo systemctl restart rigd`.

## Working on it

The edit-test loop needs no hardware. Change the code, run the suites, then
deploy only when you want it on the rig.

```sh
# 1. full offline gate — this is what must stay green
python3 rig/tests/soaktest.py

# 2. iterate on one area
python3 rig/tests/soaktest.py --only runmgr     # or: settings, pull, nav
python3 rig/tests/soaktest.py --soak 300        # randomized fault injection

# 3. see live state without a browser
curl -s localhost:9090/api/diag      | python3 -m json.tool
curl -s localhost:9090/api/anomalies | python3 -m json.tool
curl -s 192.168.1.201:8081/gpio/state | python3 -m json.tool
```

Logs: `journalctl -u rigd -f` on the Jetson, `journalctl -u ilxctl -f` /
`-u piagent -f` on a node. Structured events also land in `~/rig/rigd.jsonl` and
are served cursor-style from `GET /api/events?since=<seq>`.

## Tests

Everything runs without hardware, against an in-process fake node:

```sh
python3 rig/tests/selftest.py --offline    # contracts: naming, header, encodings
python3 rig/tests/soaktest.py              # state machine, convergence, pull faults
python3 rig/tests/soaktest.py --soak 300   # randomized fault injection
python3 rig/nav.py --selftest              # PGN decode, UTM, time authority
python3 rig/tests/navtest.py               # nav integration over a real pty
python3 rig/navlog.py --selftest           # raw NMEA log + replay
```

Current baseline: **soaktest 137/0, selftest 48/0**, nav/navtest/navlog all pass.


## Operating notes

Each of these has already cost a day. None of them look like what they are —
every one presents as a software fault.

- **A released GPIO is not high-Z.** The pad keeps the last requester's bias, and
  a Low `FOCUS` line is a permanent half-press: the camera reports every property
  read-only, hands control priority back to the body, and ignores the shutter on
  *both* USB and GPIO. `piagent` holds the lines open-drain-idle for exactly this
  reason. `/gpio/state` exposes `harness_safe`.
- **A stuck card write takes the whole body down, and nothing obvious says so.**
  One frame the card will not accept makes the body go busy: the property table
  locks (`storeDest` becomes DisplayOnly, `storeChoices` and `driveChoices` come
  back empty), PC delivery stops, format is refused — and the shutter still fires
  and `EXPOSURE` edges still land, so it looks exactly like a software fault.
  `slotStatus` still reads `OK`, because the card is *recognised* fine; it is the
  write that is hung. Read `slotWriting` instead. A 61 MP RAW is ~60–125 MB, so
  sustained RAW-to-card needs a card that can actually take it.
- **`usbfs_memory_mb` must be 150, and it does not persist by itself.** The 16 MB
  Linux default drops the SDK's transfer session; the camera still connects, still
  answers property reads and still writes to its own card, so it presents as a
  broken camera. It is set on the kernel command line — `modprobe.d` does nothing
  here, because `usbcore` is built into the Pi kernel.
- **`ilxctl` cannot recover from an unclean shutdown.** The camera's PTP session
  stays stuck and the next start blocks before binding :8080. Recover with a USB
  unbind/rebind — see `docs/HANDOFF.md` §2.2.

See `docs/HANDOFF.md` for current fleet state, open work and the discoveries
behind these notes; `docs/future-tests.md` for what still needs measuring.

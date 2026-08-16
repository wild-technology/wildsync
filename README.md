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

All figures from the live rig, timed against each camera's own `EXPOSURE` line
(hardware ground truth), not host-side estimates.

| | |
|---|---|
| GPIO trigger → exposure | **22.1 ms, sd 0.4–0.9 ms** |
| USB release → exposure | 68–76 ms, sd 5–11 ms, quantised to the camera's ~30 Hz internal tick |
| Inter-camera skew, scheduled fire | **0.65 ms mean, 3.1 ms worst** |
| Inter-camera skew, sustained 2 fps run | 1.5 ms mean, 10 ms worst |
| Node clock discipline (chrony → Jetson) | 5–55 µs |
| IMU sample rate | ~200 Hz (UI renders at display rate) |

Use the GPIO path for survey work. USB release is the fallback when a node has
no harness, and costs roughly 30 ms of timing uncertainty.

## Layout

```
src/            ilxctl — C++ camera daemon (one per node, :8080)
rig/            Python: rigd (Jetson), piagent (node), nav, run, UI
rig/tests/      regression + soak suites (no hardware required)
deploy/         deploy.sh and the systemd units
docs/           PROTOCOL.md (interface contract), HANDOFF.md, hardware notes
```

`docs/PROTOCOL.md` is the contract every component is built against — node HTTP
APIs, the 23-column `flight_log.csv` header, run layout, the time model, and the
measured Sony property encodings. Read it before changing anything that crosses
a process boundary.

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

## Tests

Everything runs without hardware, against an in-process fake node:

```sh
python3 rig/tests/selftest.py --offline    # contracts: naming, header, encodings
python3 rig/tests/soaktest.py              # state machine, convergence, pull faults
python3 rig/tests/soaktest.py --soak 300   # randomized fault injection
python3 rig/nav.py --selftest              # PGN decode, UTM, time authority
python3 rig/tests/navtest.py               # nav integration over a real pty
```

## Operating notes

Two things about this hardware will otherwise cost you a day each:

- **A released GPIO is not high-Z.** The pad keeps the last requester's bias, and
  a Low `FOCUS` line is a permanent half-press: the camera reports every property
  read-only, hands control priority back to the body, and ignores the shutter on
  *both* USB and GPIO. `piagent` holds the lines open-drain-idle for exactly this
  reason. `/gpio/state` exposes `harness_safe`.
- **A caution modal on a screenless body locks the whole property table.** An
  unseated lens, a locked iris, or a card format will do it, and it is invisible
  over USB. `ilxctl` surfaces it: every read-only error carries the property's
  `enableFlag` plus `CAUTION ON BODY`. Clearing it needs a power cycle or HDMI.

See `docs/HANDOFF.md` for current fleet state and what remains.

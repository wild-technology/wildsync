# Wild Sync

Control software for a multi-camera **Sony ILX-LR1** underwater stereo rig used
for seafloor photogrammetry.

A **Jetson Orin NX** runs `rigd`, the orchestrator and web UI, and controls a
fleet of Raspberry Pi camera nodes over Ethernet. Each node drives one ILX-LR1
over USB through Sony's Camera Remote SDK, fires it through a GPIO trigger
harness, and reads that camera's **Olive olixVision IMU**. The Jetson schedules
every camera to expose at one shared instant, fires the **XR256** strobe into
that instant, pulls the frames, renames them, and writes a per-camera flight log
binding each frame to position and orientation. Position, heading and GPS time
come from a **Subsonus** RTK antenna over NMEA 2000, through an **iKonvert**
gateway on the Jetson's USB.

```
                     ┌──────────────────────────────────────────────┐
   browser ─────────►│ Jetson Orin NX   rigd + web UI :9090         │
                     │ chrony master · piagent (strobe role) :8081  │──► XR256 (opto board)
   Subsonus ─ N2K ─► │ iKonvert on USB                              │
                     └───┬───────────────────────────┬──────────────┘
                         │ Ethernet (rig switch)     │
               ┌─────────▼─────────┐       ┌─────────▼─────────┐
               │ cam1  Pi 5        │       │ cam2  Pi 4        │
               │  ilxctl  :8080 ───┼─ USB ─┼─► ILX-LR1         │
               │  piagent :8081 ───┼─ GPIO ┼─► trigger harness │
               │  Olive IMU (USB)  │       │  Olive IMU (USB)  │
               └───────────────────┘       └───────────────────┘
```

Each camera shoots through a Sony FE PZ 16–35 mm lens and a Nauticam WACP-C
water-contact optic in a Sexton housing. The operator's guide is
[`docs/USER-GUIDE.html`](docs/USER-GUIDE.html) (the field manual) and the boat
checklist is [`docs/FIELD-RUN.md`](docs/FIELD-RUN.md).

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
| IMU (Olive, via olive-bridge) | ~15 Hz attitude stream |

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
| `rigd` | Jetson | 9090 | Python 3, stdlib | fleet state, desired settings, runs, web UI, nav (iKonvert) |
| `piagent` (strobe role) | Jetson | 8081 | Python 3, stdlib | the XR256 trigger line, nothing else (`PIAGENT_ROLE=strobe`) |
| `ilxctl` | each Pi | 8080 | C++17 | the camera over USB via Sony's SDK — settings, live view, image transfer |
| `piagent` | each Pi | 8081 | Python 3, stdlib | GPIO harness (FOCUS/TRIGGER/EXPOSURE), the node's IMU, node health |
| `olive-bridge` | each Pi | — | Python 3, stdlib | relays the Olive IMU's stream (`olive0`, 192.168.7.100:5500) to piagent on UDP 9901 |

All are systemd units with `Restart=always`. `rigd` talks to the node services
over plain HTTP; nothing else crosses a machine boundary. On the Jetson, the
strobe agent is registered with rigd as the node `jetson` (localhost, camera
switched off) by `deploy.sh strobe`.

State on disk:

| Path | Host | What |
|---|---|---|
| `~/wildsync-projects/<name>/` | Jetson | one folder per project: transects (`run.json`, `events.log`, `nmea_raw.log`, a `camN/` per camera with renamed frames + `flight_log.csv`), drained RAWs, exports |
| `~/rig/desired.json` | Jetson | the fleet's agreed settings vector, survives restart |
| `~/rig/rigd.jsonl` | Jetson | rolling structured event journal |
| `~/rig/nodes.json` | Jetson | node address overrides and extra nodes, e.g. `{"cam3": {"host": "192.168.1.203", "cam_num": 3}}` |
| `~/rig/camera_enabled.json` | Jetson | per-camera on/off, e.g. `{"jetson": false}` |
| `~/rig/strobe.json` | Jetson | strobe node, delta and pulse settings |
| `~/Pictures/ILX-LR1/` | each Pi | the PC-save spool the camera writes into; rigd pulls each frame and then deletes it |

## Requirements

**Jetson Orin NX:** JetPack 6 (Ubuntu 22.04, Python 3.10 — the code also runs
on 3.9 and on the 3.12 the nodes use). `deploy.sh jetson` installs the rest:
`python3-serial` (nav), `python3-pil` (EXIF fallback for capture times — easy to
miss, and without it frames with no GPIO edge get the command time instead),
`python3-numpy`, `python3-opencv` (calibration, VSLAM) and `chrony`.
`deploy.sh strobe` adds `python3-libgpiod` and `gpiod`.

**Camera nodes:** Ubuntu 24.04 server, Python 3.12, `python3-libgpiod`
(v1.6.3 API), `gpiod` tools, `build-essential`, `libusb-1.0-0`, `libxml2`,
`chrony` — all installed by `deploy.sh provision`. Services are stdlib-only by
policy; see `docs/PROTOCOL.md`.

**Sony Camera Remote SDK (v2.02.00):** proprietary, supplied by Sony under its
own licence agreement (linked from <http://www.sony.net/CameraRemoteSDK/>), and
not part of this repository: neither its headers (`include/CRSDK/`) nor its
libraries (`lib/`, `lib-linux/`) are tracked. Obtain the package from Sony,
unzip it, unzip the `RemoteCli.zip` inside it, and stage it with
`make sdk CRSDK_DIR=/path/to/RemoteCli` (see
[Building `ilxctl`](#building-ilxctl)). The code was built against v2.02.00. The
nodes need the **Linux aarch64 (Linux64ARMv8)** package, both to build `ilxctl`
and to provision them; a development build on a Mac needs the macOS package.

`rigd` also runs on macOS (bench and development; `rig/tests/devrig.py` runs
the full UI against two fake cameras on any host). On a Mac, stage the node SDK
with `make sdk-linux CRSDK_DIR=/path/to/RemoteCli` (the RemoteCli folder of the
Linux64ARMv8 package), and install rigd as a launchd agent from
`deploy/rigd.launchd.plist` instead of `deploy.sh jetson`.

## Install

Everything below runs from a checkout on the Jetson unless it says otherwise.

### 1. The Jetson

Flash JetPack 6, wire the Jetson to the rig switch, and give it a fixed address
on 192.168.1.0/24 (the field manual assumes `192.168.1.29`). Then:

```sh
git clone https://github.com/wild-technology/wildsync.git ~/wildsync
cd ~/wildsync
make sdk-linux CRSDK_DIR=/path/to/RemoteCli   # Sony SDK for the nodes (Linux64ARMv8 package)
deploy/deploy.sh jetson        # deps, chrony master for the rig, rigd service
```

`deploy.sh jetson` prints the UI address when rigd is up. Plug the iKonvert
into a Jetson USB port; rigd finds it under `/dev/serial/by-id/`. Its DIP
switches must all be ON (RAW mode) or nav stays silent.

### 2. Each camera node

Build the SD card and first boot per [`docs/PI-SETUP.md`](docs/PI-SETUP.md)
(§2–§5), plug the camera and its Olive IMU into the Pi, then from the Jetson:

```sh
deploy/deploy.sh trust     cam1    # push the Jetson's ssh key (asks for the Pi password)
deploy/deploy.sh provision cam1    # SDK, usbfs, chrony, build tools
deploy/deploy.sh node      cam1    # build ilxctl, udev rules, Olive link, start services
```

Repeat for `cam2` (and `cam3` if fitted). `node` ends with
`ilxctl: active   piagent: active`. It also points the node's chrony at the
Jetson and makes the Olive the node's IMU. `ilxctl` is **always built on the
node**: the Makefile probes `-mcpu=native`, so a binary built elsewhere SIGILLs.

### 3. The XR256 strobe

Wire the trigger board and 24 V supply per
[`docs/xr256-jetson-trigger.md`](docs/xr256-jetson-trigger.md). Set the chosen
header pin to GPIO with `sudo /opt/nvidia/jetson-io/jetson-io.py`, find its line
offset with `gpioinfo`, then:

```sh
WILDSYNC_STROBE_LINE=<offset> deploy/deploy.sh strobe
```

This installs `piagent-strobe.service` (active-high, for the board's rising-edge
input), registers the Jetson with rigd as the camera-less node `jetson`, and
restarts rigd. Measure the pin's boot-time state before connecting the board
(brief §2.2): it must not pulse during a power cycle.

### 4. First run

Open `http://192.168.1.29:9090` (or `localhost:9090` on the Jetson). The header
should read **2/2 cameras**. Then follow the field manual's pre-dive checklist.

### Updating

`deploy.sh node camN` ships `src/*` and the node's Python files, rebuilds
`ilxctl` on the node and restarts its services. rigd runs from this checkout,
so after a `git pull` on the Jetson a rigd change takes effect on
`sudo systemctl restart rigd` (and the strobe agent on
`sudo systemctl restart piagent-strobe`).

## Building `ilxctl`

`ilxctl` is C++17 and builds with `make` on macOS (Apple Silicon, clang) and on
Linux aarch64 (g++). On the rig it is always built on the node by
`deploy.sh node`; to build it on a development machine:

```sh
# once per checkout, and again after unpacking a different SDK release
make sdk CRSDK_DIR=/path/to/RemoteCli    # headers -> include/CRSDK/, libraries -> lib/
make                                     # -> build/ilxctl, SDK libraries staged beside it
```

`CRSDK_DIR` is the unpacked `RemoteCli` folder (the one that holds `app/CRSDK`
and `external/crsdk`) from the Sony package that matches the machine you build
on: the macOS package on a Mac, the Linux64ARMv8 package on Linux aarch64. This
is the same command on both. `make` stops with a pointer back to `make sdk` if
the SDK is not staged. `make sdk-linux` is the variant for a Mac that
provisions Pi nodes: it stages the Linux libraries into `lib-linux/` (and the
headers) without disturbing the Mac's own `lib/`. Everything `make sdk` and
`make sdk-linux` stage is git-ignored.

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

Logs: `journalctl -u rigd -f` (and `-u piagent-strobe`) on the Jetson, `journalctl -u ilxctl -f` /
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

All of these must pass. `soaktest` includes the per-file audit suites
(`rig/tests/audit_*.py`); its one hardware-dependent check, `ilxctl builds
(make)`, needs the Sony SDK staged.


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
- **`usbfs_memory_mb` must be at least 150 (1000 on a Basler node), and it does not persist by itself.** The 16 MB
  Linux default drops the SDK's transfer session; the camera still connects, still
  answers property reads and still writes to its own card, so it presents as a
  broken camera. It is set on the kernel command line — `modprobe.d` does nothing
  here, because `usbcore` is built into the Pi kernel.
- **`ilxctl` cannot recover from an unclean shutdown.** The camera's PTP session
  stays stuck and the next start blocks before binding :8080. Recover with a USB
  unbind/rebind — see `docs/HANDOFF.md` §2.2.

See `docs/HANDOFF.md` for current fleet state, open work and the discoveries
behind these notes; `docs/future-tests.md` for what still needs measuring.

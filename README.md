# Wild Sync

Control software for a multi-camera **Sony ILX-LR1** underwater stereo rig used
for seafloor photogrammetry.

A **rig host** runs `rigd`, the orchestrator and web UI, and controls a fleet of
Raspberry Pi camera nodes over Ethernet. The host is a **Mac** (Apple Silicon)
or a **Jetson Orin NX**. Each node drives one ILX-LR1 over USB through Sony's
Camera Remote SDK, fires it through a GPIO trigger harness, and reads that
camera's IMU (an **Olive olixVision**, or a Yahboom unit on a node set up for
it). The host schedules every camera to expose at one shared instant, pulls the
frames, renames them, and writes a per-camera flight log binding each frame to
position and orientation. Position, heading and GPS time come from an
Advanced Navigation **GNSS Compass** over NMEA 2000, through an **iKonvert**
gateway on the host's USB (or, not yet used in the field, from the unit
directly over Ethernet; `docs/INSTALL.md` §7.1). On a Jetson host the **XR256**
strobe can be fired into the shared instant as well.

```
                     ┌──────────────────────────────────────────────┐
   browser ─────────►│ rig host: Mac or Jetson Orin NX              │
                     │ rigd + web UI :9090                          │
 GNSS Compass ─N2K─► │ iKonvert on USB                              │
                     │ Jetson only: chrony master, strobe agent     │──► XR256 (opto board)
                     │ (piagent, strobe role) :8081                 │
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
water-contact optic in a Sexton housing. To install it, start at
[`docs/INSTALL.md`](docs/INSTALL.md) (hardware, host, nodes, clocks, IMUs,
navigation gateway, strobe, verification). The operator's guide is
[`docs/USER-GUIDE.html`](docs/USER-GUIDE.html) (the field manual) and the boat
checklist is [`docs/FIELD-RUN.md`](docs/FIELD-RUN.md).

**What has been used in the field.** The 20 August 2026 transects (Narragansett
Bay) were recorded with a Mac host, a Raspberry Pi beside each camera (Pi 5 on
cam1, Pi 4 on cam2), one IMU, the static-fix position fallback (no live
NMEA 2000 in either transect) and no strobe. The Jetson host, a per-camera
Olive IMU and the XR256 strobe role are supported by the current software, but
the repository holds no field run recorded with them, and the XR256 has not yet
been fired with the camera array. [`docs/INSTALL.md`](docs/INSTALL.md) lists
each configuration with its status.

## Versions

Releases are git tags: `task6-report-2026-08-31` (the state described in the
31 August 2026 Task 6 report), `v1.0.0` (21 September 2026) and `v1.1.0` (this
release). [`CHANGELOG.md`](CHANGELOG.md) lists what changed between them and
explains why commit IDs quoted before 5 October 2026 no longer resolve; refer to
a version by its tag. Every run records the software that produced it:
`run.json` carries a `software` object (version, commit, dirty flag), and
`python3 rig/buildinfo.py` prints the same stamp for a checkout.

## Measured performance

Bench figures, measured on the live rig 2026-08-16 against each camera's own
`EXPOSURE` line (hardware ground truth), not host-side estimates. The host then
was a Jetson.

| | |
|---|---|
| GPIO trigger → exposure | **21.8–21.9 ms** (cam1 21.94, cam2 21.83) |
| USB release → exposure | 68–76 ms, sd 5–11 ms, quantised to the camera's ~30 Hz internal tick |
| **Inter-camera skew, scheduled fire** | **0.59 ms mean, 1.82 ms worst** |
| Host fire lateness vs scheduled instant | +0.1–0.3 ms mean, 0.4 ms max |
| Node clock discipline (chrony → Jetson host) | ~85 µs RMS |
| IMU (Olive, via olive-bridge) | ~15 Hz attitude stream |

Use the GPIO path for survey work. USB release is the fallback when a node has
no harness, and costs roughly 30 ms of timing uncertainty.

Field figures, recomputed from the GPIO-edge timestamps recorded in the two
transects of 2026-08-20 (Mac host; every frame in both runs was stamped from a
GPIO edge):

| Run | Interval | Pairs | Inter-camera spread, mean / worst |
|---|---|---|---|
| `260820_1925_transect-01` | 0.5 s | 336 | 0.95 ms / 4.86 ms |
| `260820_1919_transect-01` | 2.0 s | 140 | 2.03 ms / 5.36 ms |

The pair budget is 10 ms. The bench worst case (1.82 ms) was measured on a
characterisation sweep, not on a transect; quote the field worst case for a
survey.

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

### Design rules

- **Manual focus, always.** Autofocus is refused on every path: by `rigd`, and by
  `ilxctl` itself (HTTP 403) unless it was started with `--allow-autofocus`,
  which is for the bench and shows as `afAllowed: true` to the whole fleet.
- **Focus and zoom are per camera.** They are not in the fleet settings vector
  and the UI offers no "apply focus to fleet", because lens-encoder parity
  between the two bodies is unmeasured (`docs/future-tests.md` §1).
- **Empty means empty.** A value the rig did not have is written as an empty
  cell, never invented; position from the armed static fix is labelled as such.
- **A camera that drops out pauses the grid** rather than letting the other one
  shoot the line alone, and resumes only when a probe fire succeeds.
- **Inter-camera exposure spread budget: 10 ms.** Time comes from GPS (PGN
  126992/129029) when the NMEA 2000 bus is live and from the host clock
  otherwise; `time_source` in each flight-log row says which.

## Layout

```
src/               ilxctl — C++ camera daemon (one per node, :8080)
rig/               Python: rigd (host), piagent (node), nav, run, ingest, UI
rig/tests/         regression + soak suites (no hardware required)
deploy/            deploy.sh, the systemd units and the launchd agent
third_party/       httplib.h — cpp-httplib, vendored (MIT)
docs/              INSTALL.md (start here), PROTOCOL.md (interface contract),
                   FIELD-RUN.md, PI-SETUP.md, the field manual, hardware notes
requirements.txt   Python packages beyond the standard library
LICENSE            MIT
THIRD_PARTY_NOTICES.md, sbom.spdx.json   third-party components
CHANGELOG.md       what changed in each version
```

`docs/PROTOCOL.md` is the contract every component is built against — node HTTP
APIs, the 23-column `flight_log.csv` header, run layout, the time model, and the
measured Sony property encodings. Read it before changing anything that crosses
a process boundary.

## Where things run

| Process | Host | Port | Language | Owns |
|---|---|---|---|---|
| `rigd` | rig host (Mac or Jetson) | 9090 | Python 3 | fleet state, desired settings, runs, web UI, nav (iKonvert, or the GNSS Compass / Subsonus over Ethernet) |
| `piagent` (strobe role) | Jetson host only | 8081 | Python 3, stdlib | the XR256 trigger line, nothing else (`PIAGENT_ROLE=strobe`) |
| `ilxctl` | each Pi | 8080 | C++17 | the camera over USB via Sony's SDK — settings, live view, image transfer |
| `piagent` | each Pi | 8081 | Python 3, stdlib | GPIO harness (FOCUS/TRIGGER/EXPOSURE), the node's IMU, node health |
| `olive-bridge` | each Pi | — | Python 3, stdlib | relays the Olive IMU's stream (`olive0`, 192.168.7.100:5500) to piagent on UDP 9901 |

On a Jetson host `rigd` and the strobe agent are systemd units; on a Mac `rigd`
is a launchd agent (`deploy/rigd.launchd.plist`). The node services are systemd
units. All restart automatically. `rigd` talks to the node services over plain
HTTP; nothing else crosses a machine boundary. On a Jetson host the strobe agent
is registered with rigd as the node `jetson` (localhost, camera switched off) by
`deploy.sh strobe`.

State on disk:

| Path | Host | What |
|---|---|---|
| `~/wildsync-projects/<name>/` | rig host | one folder per project: transects (`run.json`, `events.log`, `nmea_raw.log`, `SHA256SUMS`, a `camN/` per camera with renamed frames + `flight_log.csv`), drained RAWs, exports. Transects from before projects existed are in `~/rig-runs`, drained RAWs in `~/rig-raw` |
| `~/rig/desired.json` | rig host | the fleet's agreed settings vector, survives restart |
| `~/rig/rigd.jsonl` | rig host | rolling structured event journal |
| `~/rig/nodes.json` | rig host | node address overrides and extra nodes, e.g. `{"cam3": {"host": "192.168.1.203", "cam_num": 3}}` |
| `~/rig/camera_enabled.json` | rig host | per-camera on/off, e.g. `{"jetson": false}` |
| `~/rig/strobe.json` | rig host | strobe node, delta and pulse settings |
| `~/rig/static_fix.json` | rig host | the position used when no live NMEA 2000 is available |
| `~/Pictures/ILX-LR1/` | each Pi | the PC-save spool the camera writes into; rigd pulls each frame and then deletes it |

## Requirements

**Mac host:** macOS on Apple Silicon with Python 3 and the packages in
[`requirements.txt`](requirements.txt), installed in a venv because Homebrew's
Python refuses a global `pip install` (`docs/INSTALL.md` section 3a has the
commands). No camera is attached to the host, so a Mac that only runs
`rigd` does not build `ilxctl`; it stages the Linux SDK for the nodes (below).

**Jetson Orin NX host:** JetPack 6 (Ubuntu 22.04, Python 3.10 — the code also
runs on 3.9 and on the 3.12 the nodes use). `deploy.sh jetson` installs the
rest: `python3-serial` (nav), `python3-pil` (EXIF fallback for capture times —
easy to miss, and without it frames with no GPIO edge get the command time
instead), `python3-numpy`, `python3-opencv` (calibration, VSLAM) and `chrony`.
`deploy.sh strobe` adds `python3-libgpiod` and `gpiod`.

**Camera nodes:** Ubuntu 24.04 server, Python 3.12, `python3-libgpiod`
(v1.6.3 API), `gpiod` tools, `build-essential`, `libusb-1.0-0`, `libxml2`,
`chrony` — all installed by `deploy.sh provision`. The node services use the
Python standard library only (pyserial where a node reads a serial IMU); see
`docs/PROTOCOL.md`.

**Sony Camera Remote SDK (v2.02.00):** proprietary, supplied by Sony under its
own licence agreement (linked from <http://www.sony.net/CameraRemoteSDK/>), and
not part of this repository: neither its headers (`include/CRSDK/`) nor its
libraries (`lib/`, `lib-linux/`) are tracked. Obtain the package from Sony,
unzip it, unzip the `RemoteCli.zip` inside it, and stage it with
`make sdk CRSDK_DIR=/path/to/RemoteCli` (see
[Building `ilxctl`](#building-ilxctl)). The code was built against v2.02.00. The
nodes need the **Linux aarch64 (Linux64ARMv8)** package, both to build `ilxctl`
and to provision them; a development build on a Mac needs the macOS package.

**Python packages beyond the standard library:** listed in
[`requirements.txt`](requirements.txt), grouped by the part of the system that
imports them. Where `deploy.sh` installs them it uses apt packages (see
the Jetson host and Camera nodes above), not pip; `pip install -r
requirements.txt` is for a Mac (in a venv, `docs/INSTALL.md` section 3a) or
other development host.

`rig/tests/devrig.py` runs the full UI against two fake cameras on any host,
with no hardware.

## Install

[`docs/INSTALL.md`](docs/INSTALL.md) is the installation and configuration guide,
in order: hardware, the Sony SDK and `ilxctl`, the host (Mac with launchd, or
Jetson with systemd), provisioning and deploying the camera nodes, clocks, IMUs,
the navigation gateway, the strobe, and verification. The commands in short:

```sh
git clone https://github.com/wild-technology/wildsync.git ~/wildsync && cd ~/wildsync
make sdk-linux CRSDK_DIR=/path/to/RemoteCli   # Sony SDK for the nodes (Linux64ARMv8 package)
deploy/deploy.sh jetson                       # Jetson host only: deps, chrony master, rigd service
                                              # (Mac host: launchd agent, INSTALL.md section 3)
deploy/deploy.sh trust     cam1               # per node: push this host's ssh key
deploy/deploy.sh provision cam1               # per node: SDK, usbfs, chrony, build tools
deploy/deploy.sh node      cam1               # per node: build ilxctl, units, start services
WILDSYNC_STROBE_LINE=<offset> deploy/deploy.sh strobe   # Jetson host with an XR256
```

`ilxctl` is **always built on the node**: the Makefile probes `-mcpu=native`, so
a binary built elsewhere SIGILLs. `deploy.sh node camN` also ships `src/*` and
the node's Python files, so it is the update command too; rigd runs from the
checkout, so a rigd change takes effect when rigd is restarted
(`sudo systemctl restart rigd` on a Jetson, `launchctl kickstart -k
gui/$(id -u)/org.wildtechnology.wildsync.rigd` on a Mac).

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

The rig's own diagnostics are the first stop — `/api/diag`, `/api/anomalies`,
`/gpio/state` and the event journal — before logging in to a machine.

Logs: `journalctl -u rigd -f` (and `-u piagent-strobe`) on a Jetson host,
`~/rig/rigd-launchd.log` on a Mac host, `journalctl -u ilxctl -f` /
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

All of these must pass: each ends with `ALL PASS` or `PASS` and exits 0 when it
is green. The number of checks grows with the suites, so none is quoted here.
`soaktest` includes the per-file audit suites (`rig/tests/audit_*.py`); its one
hardware-dependent check, `ilxctl builds (make)`, needs the Sony SDK staged.


## Operating notes

Each of these presents as a software fault and is not one.

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
  sustained RAW-to-card needs a card that can actually take it (a V60/UHS-II
  card is the rig's standard; about 250 rapid fires preceded the stall seen on
  2026-08-16, so keep bench test batches modest and watch `slotWriting`).
- **`usbfs_memory_mb` must be at least 150, and it does not persist by itself.** The 16 MB
  Linux default drops the SDK's transfer session; the camera still connects, still
  answers property reads and still writes to its own card, so it presents as a
  broken camera. It is set on the kernel command line — `modprobe.d` does nothing
  here, because `usbcore` is built into the Pi kernel.
- **`ilxctl` cannot recover from an unclean shutdown.** The camera's PTP session
  stays stuck and the next start blocks in the SDK connect. Recover with a USB
  unbind/rebind — see `docs/PI-SETUP.md` §10.1.

`docs/PI-SETUP.md` §10 lists the node faults and their fixes; `docs/future-tests.md`
lists what still needs measuring.

## Licence

Wild Sync is released under the MIT licence ([`LICENSE`](LICENSE)), Copyright
(c) 2026 Wild Technologies. The licence covers this repository's own source code
and documentation. It does **not** cover:

- the **Sony Camera Remote SDK**, which is not in this repository and stays
  under Sony's own licence agreement;
- the **vendor documentation** the docs cite (Smart Vision Lights, VA Imaging,
  Digital Yacht), which is not redistributed here and stays the copyright of
  its publishers (where to get it: `THIRD_PARTY_NOTICES.md`);
- the third-party components listed in
  [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md), each under its own
  licence. `third_party/httplib.h` is cpp-httplib, vendored under its MIT
  licence.

`sbom.spdx.json` is the same component list as an SPDX 2.3 document.

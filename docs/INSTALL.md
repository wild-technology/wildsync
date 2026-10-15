# Installing and configuring Wild Sync

One guide, in the order a new installation is done: what hardware is needed and
how it is connected, the Sony SDK and `ilxctl`, the rig host, the camera nodes,
time synchronisation, the IMUs, the navigation gateway, the strobe node, and
how to verify the result. Each step says which commands to run and links to the
detailed document instead of repeating it.

All commands run from the root of a checkout
(`git clone https://github.com/wild-technology/wildsync.git ~/wildsync`) unless
the text says they run on a node. `camN` stands for a node name (`cam1`, `cam2`;
`cam3` for a third camera) and `192.168.1.20N` for its address.

## 0. What has been run, and what has not

The software supports more configurations than have been used in the field. This
table says which is which; anything not listed as used has not been exercised in
a recorded field run.

| Configuration | Status |
|---|---|
| **Mac host** (Apple Silicon, launchd), a Raspberry Pi per camera (Pi 5 on cam1, Pi 4 on cam2) running `ilxctl` and `piagent`, two ILX-LR1, hardware GPIO trigger | **Used in the field.** The two Narragansett Bay transects of 2026-08-20 were recorded this way, with the software of that date (before `v1.0.0`; the run files carry no build stamp) |
| One IMU (a Yahboom YB unit on cam1); its solution is written to both cameras' flight logs | **Used in the field** (2026-08-20) |
| Position from the static-fix fallback (`~/rig/static_fix.json`) | **Used in the field.** Neither transect received live NMEA 2000; every flight-log row carries the fallback position |
| Live NMEA 2000 through an iKonvert in RAW mode | Verified on the bench only (run `260820_0536_gps-e2e`) |
| **Jetson Orin NX host** (JetPack 6, systemd, `deploy.sh jetson`) | Supported by the current software. A Jetson was the host for the earlier bench work and the 2026-08-16 measurements in the README; the Mac replaced it on 2026-08-23 and the Jetson install was added back on 2026-09-21. No recorded field run used a Jetson host |
| **Per-camera Olive IMUs** (`olive-bridge`, `deploy.sh node` default) | Supported. First run on the rig on 2026-08-27, after the field transects; no recorded field run |
| **XR256 strobe** (`piagent` in its strobe role, `deploy.sh strobe`) | Software added on 2026-09-21 and covered by the offline tests. **No strobe has been connected or fired in any field run, and the XR256 has not yet been fired with the camera array** |
| **GNSS Compass as the rig's NTP server** (section 5: `deploy.sh clock`, `deploy.sh mac`, `node_clock_wrong` + the clock step) | Added 2026-10-15, after cam1 came back from a power cycle 47 days behind. Exercised on the rig that day with the Mac host: `deploy.sh clock cam1` put cam1 on the Compass (stratum 2, refid `C0A8013C`); a `date -s` a day back was stepped back by chrony within ~42 s; with chrony paused, `POST /clock/step` from the host stepped the same fault in one request and a second request was refused as under threshold. On 2026-10-16 the Mac's time server was switched to the Compass by hand (the two `systemsetup` lines `deploy.sh mac` runs; an undisciplined night had taken the Mac to 860 ms behind GPS): it came to within 10 ms in seconds, and rigd's measured host offset against cam1 read +11 ms where it had read +99 ms. **Not yet exercised:** `deploy.sh mac` as a whole (its launchd-agent part), the step button from the rig UI against a live node (the nodes' `piagent` gains `/clock/step` at their next `deploy.sh node`), and cam2, which was off the wire. No field run yet |

The chrony topology (section 5) follows the same split: the mesh-only Mac-host
topology was the one in use on 2026-08-20; the Compass reference has not yet
been under a recorded field run.

## 1. Hardware and how it is connected

### Parts

| Part | Notes |
|---|---|
| 2 × **Sony ILX-LR1** | one body per camera node; set to PC Remote on the body. Both bodies need a fast card (V60 / UHS-II): a slow card stalls on RAW writes and takes the whole body down (`README.md`, Operating notes) |
| Sony **FE PZ 16–35 mm F4 G** lens, **Nauticam WACP-C** water-contact optic, **Sexton** housing | per camera. Anything set on the body itself must be set before the housing is sealed |
| 2 × **Raspberry Pi** with a PoE HAT (802.3at) | cam1 is a Pi 5 and cam2 a Pi 4; both work. Ubuntu 24.04 Server arm64 on a V60 / UHS-II SD card, 64 GB or more. `docs/PI-SETUP.md` §1 is the full parts list |
| GPIO trigger harness | four leads per node, below |
| USB data cable, body to Pi | not a charge-only cable |
| IMU per camera | an **Olive olixVision X1** on USB-C to the Pi (default), or a Yahboom YB-MRA02 on USB-A |
| **Rig host** | a Mac (Apple Silicon) or a Jetson Orin NX, wired to the rig switch |
| Rig switch and PoE | a PoE switch for the nodes. cam1 is on its own 802.3at injector, cam2 on the switch: two nodes firing in sync overran the switch's PoE budget and it dropped a port every time until cam1 was isolated (`docs/FIELD-RUN.md`, Topology; `docs/PI-SETUP.md` §8) |
| 12 V supply for the camera bodies | 10–18 V DC at the camera connector, at least 40 W momentary (`docs/harness-and-strobe.md` §2) |
| Navigation | an Advanced Navigation **GNSS Compass** (a Subsonus is read the same way), either on an NMEA 2000 backbone through a Digital Yacht **iKonvert** gateway to the host's USB ([product page](https://digitalyacht.co.uk/product/ikonvert-nmea-2000-to-usb/), [quick start guide](https://digitalyachtamerica.com/download/ikonvert-usb-quick-start-guide-v2_02/?wpdmdl=5667)), or on the rig switch over Ethernet, static at 192.168.1.60 (section 7.1, not yet used in the field) |
| Strobe (optional) | an XR256, its trigger interface board and a regulated 24 V supply (`docs/xr256-jetson-trigger.md`) |

### Network

Everything is on 192.168.1.0/24, wired: the nodes are `192.168.1.201` (cam1),
`192.168.1.202` (cam2) and `192.168.1.203` (a third node); `pi-camN` implies
`192.168.1.(200+N)`, and that convention is load-bearing. A Jetson host is
assumed to be at `192.168.1.29`. If the host is a laptop that is also on Wi-Fi
using the same range (a Starlink router, for instance), the route to the nodes
goes out of the wrong interface: turn Wi-Fi off, or move that LAN off
192.168.1.0/24 (`docs/PI-SETUP.md` §4).

### The harness: one per camera, Pi to camera

The camera's 6-pin power/control connector (Molex Micro-Fit 3.0) carries the
trigger. Wire each Pi's header to it (`docs/PI-SETUP.md` §1.1,
`docs/harness-and-strobe.md` §2):

| Pi BCM | Header pin | Camera pin | Signal |
|---|---|---|---|
| 17 | 11 | 4 | `FOCUS`, open-drain output |
| 27 | 13 | 5 | `TRIGGER`, open-drain output; pulsed Low |
| 22 | 15 | 6 | `EXPOSURE`, input (bias pull-up); Low while the front curtain is open |
| GND | any GND | 2 | ground and signal return |

The camera is powered at pins 3 (+) and 2 (−); the grounds meet at pin 2 only.
**Never drive `FOCUS` or `TRIGGER` high**: "off" is high-Z, not logic 1. A Low
`FOCUS` line is a permanent half-press that takes the body out on both USB and
GPIO; `piagent` parks the lines safely and reports `harness_safe` in
`/gpio/state`. `EXPOSURE` is the hardware timestamp of every frame.

The IMU plugs into a USB port of the same Pi.

## 2. The Sony SDK and `ilxctl`

`ilxctl` is the C++ camera daemon that runs on each node. It is built against
the **Sony Camera Remote SDK v2.02.00**, which is proprietary, supplied by Sony
under its own licence agreement (<http://www.sony.net/CameraRemoteSDK/>) and
**not part of this repository**.

1. Obtain the package from Sony. Sony ships a package per platform: the
   **Linux64ARMv8** one is needed to build `ilxctl` on the nodes and to
   provision them, from either kind of host; the macOS one is needed only to
   build `ilxctl` on a Mac.
2. Unzip it, then unzip the `RemoteCli.zip` inside it. That `RemoteCli` folder is
   `CRSDK_DIR`.
3. Stage the SDK for the nodes:

   ```sh
   make sdk-linux CRSDK_DIR=/path/to/RemoteCli    # the RemoteCli folder of the Linux64ARMv8 package
   ```

   This puts the headers in `include/CRSDK/` and the Linux libraries in
   `lib-linux/`. Everything it stages is git-ignored. `deploy.sh provision`
   refuses to run until both are present.

`ilxctl` is **always built on the node** by `deploy.sh node`: the Makefile probes
`-mcpu=native`, so a binary built on another machine SIGILLs. To build it on a
development machine instead (a Mac, for the bench), stage that platform's SDK and
build:

```sh
make sdk CRSDK_DIR=/path/to/RemoteCli    # headers -> include/CRSDK/, libraries -> lib/
make                                     # -> build/ilxctl, SDK libraries staged beside it
```

`make` stops with a pointer back to `make sdk` if the SDK is not staged. See
`README.md`, "Building `ilxctl`", for the details.

## 3. The rig host

The host runs `rigd`, which owns the fleet, the settings, the runs, the web UI
and navigation. It needs no camera. Install the Python packages that `rigd`
and the calibration tools import (`requirements.txt` says what each one is for
and what stops working without it).

### 3a. Mac host (launchd)

```sh
mkdir -p ~/rig
/opt/homebrew/bin/python3 -m venv ~/rig/venv
~/rig/venv/bin/python3 -m pip install -r requirements.txt   # the interpreter the launchd job runs
sed "s|__REPO__|$(pwd)|g; s|__HOME__|$HOME|g" deploy/rigd.launchd.plist \
  > ~/Library/LaunchAgents/org.wildtechnology.wildsync.rigd.plist
launchctl bootstrap gui/$(id -u) \
  ~/Library/LaunchAgents/org.wildtechnology.wildsync.rigd.plist
```

Homebrew's Python is externally managed (PEP 668), so `pip install` into it
stops with `externally-managed-environment` and installs nothing; the packages
go in the venv instead. The plist runs `~/rig/venv/bin/python3` (the `sed`
fills in `__HOME__`), which is a link to the Homebrew interpreter; to use a
different Python, create the venv with that one. An agent installed before this
needs its plist regenerated with the `sed` above and a `launchctl bootout`,
then `bootstrap`, to pick the venv up.

The agent is `org.wildtechnology.wildsync.rigd` (restart it with
`launchctl kickstart -k gui/$(id -u)/org.wildtechnology.wildsync.rigd`, stop and
remove it with `launchctl bootout gui/$(id -u)/org.wildtechnology.wildsync.rigd`),
it restarts if `rigd` dies, and its log is `~/rig/rigd-launchd.log`. The UI is at
`http://localhost:9090`.

`deploy/deploy.sh mac` does the `sed` and `launchctl` steps above for you
(once the venv exists) and also points the Mac's own clock at the rig's GNSS
Compass, the rig's NTP server (section 5; it asks for an administrator
password). rigd corrects for a host offset, but the correction carries its own
uncertainty, and `time.apple.com` is not reachable from the rig segment, so a
Mac left on Apple's server free-runs on the boat. Stage the Linux SDK (section
2) so the Mac can provision the nodes.

### 3b. Jetson host (systemd)

Flash JetPack 6, wire the Jetson to the rig switch and give it a fixed address on
192.168.1.0/24 (the field manual assumes `192.168.1.29`). Then:

```sh
git clone https://github.com/wild-technology/wildsync.git ~/wildsync
cd ~/wildsync
make sdk-linux CRSDK_DIR=/path/to/RemoteCli   # Sony SDK for the nodes (Linux64ARMv8 package)
deploy/deploy.sh jetson        # deps, chrony master for the rig, rigd service
```

`deploy.sh jetson` installs `python3-serial`, `python3-pil`, `python3-numpy`,
`python3-opencv`, `chrony` and `rsync` with apt; makes the Jetson a chrony server
for the rig's subnet; installs `rigd.service` for this checkout and user; writes
`BUILD_INFO.json` (the software stamp, section 9); and restarts `rigd`. It prints
the UI address when `rigd` is up (`http://192.168.1.29:9090`, or
`localhost:9090` on the Jetson). The optional desktop launcher
`deploy/wildsync.desktop` carries the path of the checkout it was written for;
edit its `Exec=` line before using it.

## 4. The camera nodes

### 4a. Build the card

Flash Ubuntu 24.04 Server arm64 to the node's SD card and put the headless
cloud-init files on its boot partition: the hostname (`pi-camN`), the `ubuntu`
user with the host's ssh key, and the static address `192.168.1.20N` alongside a
DHCP lease. `docs/PI-SETUP.md` §2–§5 is the step-by-step, including the first
boot and the one trap worth reading before you start: **a fresh Pi boots months
in the past** (no RTC battery), and `apt` then fails on every repository (§5.1).
Those cloud-init files do not write the nodes' chrony peer mesh
(`/etc/chrony/conf.d/10-wildsync.conf`). Run `deploy/pi-resync/install.sh
/Volumes/system-boot` on every card, new or already booted: it writes the mesh
and re-applies the ssh key. `deploy.sh provision` warns when a node has no mesh
file.

Then fit the card, connect the node to the rig switch, apply PoE, connect the
camera (USB and harness) and the IMU, and set the camera body to PC Remote. Set
each body's date and time in its own menu: USB cannot set it (Sony refuses), and
the archive is corrected by timestamps regardless, but the card's own filenames
stay sensible.

### 4b. Provision and deploy, per node, from the host

```sh
deploy/deploy.sh trust     cam1    # push the host's ssh key (asks for the Pi password)
deploy/deploy.sh provision cam1    # SDK, usbfs, chrony, build tools
deploy/deploy.sh node      cam1    # build ilxctl, udev rules, Olive link, start services
```

On a Jetson host, run `deploy/deploy.sh jetson` (section 3b) before
`deploy.sh node`: `node` points a node's chrony at the host only when the host
is already the rig's chrony server. If a node was deployed first, run `node` for
it again. (`deploy.sh all` already runs `jetson` first.)

Repeat for `cam2` (and `cam3` if fitted). `deploy/deploy.sh all` runs `jetson`
and then `node` for every reachable node, so the host is already the chrony
server when each node is pointed at it; use it from a Jetson host only, since
`jetson` installs with apt and a systemd unit. It tries every node even if one
fails, and exits nonzero (naming the failed nodes) if any step failed. An
unreachable node is skipped and is not a failure.

- `trust` pushes the host's ssh key; every other step assumes passwordless ssh.
- `provision` sends the SDK (`lib-linux/` and `include/`), the Makefile and
  `third_party/` to the node; sets `usbfs_memory_mb` to at least 150 on the kernel
  command line (it does not persist any other way, so **reboot the node once
  after the first provision**); fixes the clock before running `apt`; installs
  `build-essential`, `rsync`, `chrony`, `python3-libgpiod`, `gpiod`,
  `libusb-1.0-0` and `libxml2`; and prints warnings for anything that did not
  install.
- `node` ships `src/*` and the node's Python files, builds `ilxctl` on the node,
  installs the udev rules for the GPIO chips and the Sony body, the three systemd
  units (`ilxctl`, `piagent`, `olive-bridge`), the Olive link (section 6) and the
  chrony host line (section 5), and restarts the services. It ends with
  `ilxctl: active   piagent: active`. It is also the update command: run it
  again after a `git pull`. It writes `~/wildsync/BUILD_INFO.json` on the node,
  the record of what was deployed there.

Refusals are deliberate: `provision` stops, with the remedy, when the Linux SDK
or the headers are not staged, and `node` stops if the node has no SDK
(`provision` was not run) or the build fails, leaving the running services alone.

### 4c. Join the fleet

The default fleet is cam1 and cam2. A third node joins through
`~/rig/nodes.json` on the host, never a code edit; `docs/PI-SETUP.md` §7 has the
file, why, and how a camera is switched off for a dive. Each node's settings,
anomalies and the UI header (`2/2 cameras`) are described in the field manual.

## 5. Time synchronisation

The stereo budget is 10 ms between the two exposures, and every scheduled fire
and every hardware exposure timestamp lives on the **node's** clock, so clock
disagreement between the nodes lands one for one in the exposure skew. A Pi has
no RTC battery, so a node that is power-cycled boots where its filesystem left
it (cam1 came back 47 days behind on 2026-10-15), and until something steps it
every scheduled fire is refused: `piagent` accepts an `at_epoch` only within
−2/+5.5 s of its own clock. The rig segment has no internet, so the reference
has to live on the switch:

- **The GNSS Compass is the rig's time server, under either host.** The
  Advanced Navigation GNSS Compass at `192.168.1.60` (§7.1) is a stratum-1 NTP
  server on GPS time (reference ID `GPS`; 0.5 ms round trip from a node). Every
  node holds it as its preferred chrony source,
  `/etc/chrony/conf.d/20-wildsync-gnss.conf`, written by the card step
  (`deploy/pi-resync/user-data.template`) and by `deploy.sh clock camN` (which
  `deploy.sh node camN` runs first) — same file, same content. The same step
  sets `chrony.conf` to `makestep 1 -1`: a clock more than a second out is
  stepped whenever a reference answers, however long the node has been up.
  Ubuntu's `makestep 1 3` stops stepping after three updates, which chrony
  parses *after* the drop-ins, so a node that had peered a wrong neighbour
  before the Compass appeared could never follow it. A step over a second is
  never inside the stereo budget; it is a clock that was wrong.
- **Every node, as the fallback:** a full peer mesh across
  `192.168.1.201/.202/.203` plus an orphan local reference (`10-wildsync.conf`,
  written by the card step; `deploy.sh clock` writes it too on a node whose
  card never ran that step, as cam1's had not). With the Compass off the nodes
  stay about a millisecond apart (0.6 ms measured with two nodes); free-running
  they measured 16.8 ms apart. A rig with no Compass deploys with
  `WILDSYNC_NTP=` (empty) and keeps only the mesh.
- **Mac host:** `deploy.sh mac` points macOS's own time service at the Compass
  (`systemsetup -setnetworktimeserver 192.168.1.60`, which asks for an
  administrator password; System Settings → General → Date & Time shows it)
  and installs the launchd agent (§3a). The Mac is not a chrony master and
  need not be: the nodes already hold GPS time. What this buys is a host on
  the same second as the nodes, so the host offset that rigd measures and
  corrects every fire target and every capture instant for
  (`host_clock_offset`; `docs/PROTOCOL.md`, "CLOCK DOMAINS") is tens of
  milliseconds with a small uncertainty, instead of the 100–190 ms an
  undisciplined Mac measured (187 ms on 2026-08-23; 100 ms behind GPS on
  2026-10-15, with `time.apple.com` out of reach from the rig segment anyway).
- **Jetson host:** `deploy.sh jetson` makes the Jetson a chrony server for the
  LAN *and* a client of the Compass, and `deploy.sh node`, run from that
  Jetson, adds `20-wildsync-host.conf` pointing each node at it as a second
  server. The strobe fires on the host's clock, so one reference keeps it
  honest.
- **When a node is still wrong** (the Compass was off while it booted, and is
  still off): rigd raises `node_clock_wrong` on that node, bad, once its clock
  is more than 2 s from the host's, and the anomaly carries a **set camN clock
  from host** button (`POST /api/node/clock/step {"node": "camN"}`). The node
  steps its clock to the host's (`sudo date -s` on the node — the hand fix of
  2026-10-15) and chrony slews the residual once a source answers. rigd refuses
  it while a run is active; a node under 1 s off refuses it itself, since that
  residual is chrony's. If *every* node shows the same offset and their
  `chronyc tracking` says GPS, it is the host that is wrong: run `deploy.sh
  mac` / `deploy.sh jetson` instead.

Check on a node (`docs/PI-SETUP.md` §9):

```sh
for ip in 192.168.1.201 192.168.1.202 192.168.1.203; do
  echo "== $ip"; ssh ubuntu@$ip 'chronyc tracking | head -3; chronyc -n sources'
done
```

With the Compass up every node prints Reference ID `C0A8013C (192.168.1.60)`,
Stratum 2, and `sources` marks `192.168.1.60` with `*`. With it off, all nodes
must still print the **same** Reference ID — the elected orphan's address — the
proof that the group has exactly one master; two different refids mean a split.
On a Mac, `sntp 192.168.1.60` prints the correction the Mac's clock still needs
(tens of milliseconds or less once `deploy.sh mac` has run and macOS has
slewed); on a Jetson host, `chronyc tracking` should show it synchronised and
`chronyc clients` should list the nodes. While running, rigd's
`node_clock_wrong`, `node_clock_skew` and `host_clock_offset` alerts are the
detectors.

Time in the data: when the NMEA 2000 bus is live, GPS time (PGN 126992/129029)
disciplines the log and `time_source` reads `gps`; otherwise it reads `host`, the
host's own clock. Runs recorded before `v1.1.0` wrote `jetson` for `host`.

## 6. The IMUs

Each camera's flight log takes its attitude from that camera's own IMU.

- **Olive olixVision X1 (default).** `deploy.sh node` configures it: it pins the
  IMU's USB network link as `olive0` (Pi `192.168.7.2/24`, IMU `192.168.7.100`),
  installs `olive-bridge.service`, which relays the IMU's stream to piagent on
  local UDP 9901, and writes `PIAGENT_IMU=olive:udp:9901` to `/etc/default/piagent`.
  Check it:

  ```sh
  ssh ubuntu@192.168.1.20N 'ip -br addr show olive0; systemctl is-active olive-bridge'
  curl -s 192.168.1.20N:8081/health | python3 -c "import sys,json; print(json.load(sys.stdin)['imu'])"
  ```

  `present: true` with an `attitude_hz` around 15 is healthy.
- **Yahboom YB-MRA02** (the unit used in the 2026-08-20 transects): deploy a node
  that still carries it with `WILDSYNC_NODE_IMU=yb deploy/deploy.sh node cam1`.
- **Calibration.** In the UI, Calib tab, with the rig still and level, press
  **Calibrate both** (ten seconds; refused if anything moves). The stereo
  checkerboard calibration is in the same tab and is done in the water.

Details, configuration values and the fusion roadmap: `docs/olive-imu.md`.

## 7. The navigation gateway (iKonvert, RAW mode)

The iKonvert is a Digital Yacht NMEA 2000 to USB gateway. It carries the
GNSS Compass's position, heading and GPS time, and the boat's other N2K data
such as depth, into `rigd`.

1. Connect the iKonvert to the NMEA 2000 backbone and its USB to the host. **Its
   processor is powered from the N2K bus, not from USB**: with the bus unpowered
   the port opens and returns nothing at any baud. The unit's POWER LED dark means
   no N2K power.
2. Set **all four DIP switches ON (RAW mode, 230400 baud)** and power-cycle the
   iKonvert. In any other mode it emits ordinary NMEA 0183 sentences and `rigd`
   decodes nothing.
3. `rigd` finds the gateway on its own: a `/dev/ikonvert` link if one exists,
   otherwise the FTDI FT232R under `/dev/serial/by-id/` on Linux or
   `/dev/cu.usbserial-*` on a Mac, never a bare `/dev/ttyUSB<n>`.
4. Verify: the UI header reads `nav fix` (a live position), the Nav tab shows
   data, and `/api/diag`'s `time` block names the GPS time source.

**Without live navigation** (the 2026-08-20 transects): put today's launch point
in `~/rig/static_fix.json` (`{lat, lon, label, ...}`). The header then reads
`nav STATIC`, the preflight says "STATIC fix in use", and every flight-log row
carries that one position, with depth and heading empty. A mid-line loss of a
live bus is not filled from it: those rows are empty and `nav_no_fix` is raised
(`docs/FIELD-RUN.md`, anomalies).

### 7.1 The GNSS Compass over Ethernet (no iKonvert)

Supported, not yet used in the field. The unit on the rig is an Advanced
Navigation GNSS Compass (its web UI header reads "GNSS COMPASS"); the Subsonus
surface unit is the other Advanced Navigation unit the same code reads. Both
speak Advanced Navigation's packet protocol (ANPP) over TCP, port 16718 by
default (a Subsonus streams on 16718–16721), and send a System State packet
(ID 20; 20 Hz on a Subsonus out of the box). `rigd` can read that stream
instead of the iKonvert (`rig/nav_subsonus.py`); position, heading, attitude,
COG/SOG and GPS time then arrive without the NMEA 2000 bus. The GNSS Compass
sends the 100-byte System State layout, confirmed on the rig on 2026-10-15;
the Subsonus sends a 116-byte one, taken from the published reference and not
confirmed on a unit. Depth and magnetic heading are not in that packet:
`depth_from_xplore9` and `heading_mag_xplore` stay empty (the true heading is
in `/api/nav`).

1. Plug the GNSS Compass's Ethernet into the rig switch. Ours keeps a static
   address, `192.168.1.60/24` (gateway and DNS `192.168.1.1`); it shipped on
   `192.168.2.220`, unreachable from the rig subnet, and was re-addressed over
   ANPP packet 11. If it is ever moved to another subnet, re-address it from
   any host on its current subnet, either in its web UI (Configuration →
   Ethernet Settings; login required) or from a script over ANPP packet 11 on
   TCP 16718. The packet's fields are permanent (u8), mode (u8; bit 0 is DHCP,
   keep the other bits 0 on write), then IP address, netmask, gateway and DNS
   (each a big-endian u32), then the three serial-number words echoed from a
   read. The unit re-addresses itself immediately and sends no acknowledgement.
   A Subsonus also answers to its zeroconf name, `an-subsonus-1.local` (or the
   name set on the unit).
2. Tell `rigd` where it is — either, on the host:

   ```sh
   echo '{"source": "anpp://192.168.1.60"}' > ~/rig/nav.json
   ```

   or `WILDSYNC_NAV=anpp://192.168.1.60` in the service environment —
   and restart `rigd`. `WILDSYNC_NAV` wins over `nav.json`; with neither,
   `rigd` looks for the iKonvert as before. The port defaults to 16718.
3. Verify: `python3 rig/nav_subsonus.py anpp://<address>` prints a line a
   second with the packet count, fix type (`3d` or better on the GNSS Compass;
   `rtk_fixed` only when corrections reach it, which needs an NTRIP source
   reachable from the unit), UTC status, position and heading. `suspect` must
   stay 0: a non-zero count means the packet layout is not the one the reader
   expects (it then records no position rather than a wrong one). The GNSS
   Compass's 100-byte layout was confirmed on the rig; the Subsonus's 116-byte
   layout was taken from the published ANPP reference, whose field table is
   inconsistent, and has not been confirmed on a unit. The same unit is the
   rig's NTP server (stratum 1 on GPS time; section 5). In the UI the header
   reads `nav fix` and `/api/diag`'s `nav` block shows `transport: anpp-tcp`
   with `packets` climbing and `layout: common-100` on a GNSS Compass
   (`subsonus-116` on a Subsonus).
4. If the socket connects but `packets` stays 0, the unit is not streaming on
   that port: enable the System State packet (ID 20) on the ANPP port in its
   web UI.

Every packet is written to the run's `nmea_raw.log` as an `!ANPP,<id>,<hex>`
line, so `navlog.replay()` reproduces an Ethernet nav run as it does an
iKonvert run. The reader never writes anything to the unit; re-addressing it
(step 1) is a separate, deliberate act.

## 8. The strobe node

Skip this section on a rig without a strobe (the 2026-08-20 configuration): leave
Controls → Strobe disabled. **The XR256 path has not yet been fired with the
camera array** (section 0).

The XR256 is triggered from a Jetson host's GPIO through an opto-isolated
interface board. A bare Jetson pin cannot trigger it (the input wants a sourcing
4–24 V edge), and the 24 V rail must never tie to the Jetson, the Pis, the PoE
switch or the camera harness.

1. Build the trigger board and a regulated 24 V supply, and wire them per
   `docs/xr256-jetson-trigger.md` (§2 and §4). Set the duration dial on the light
   to its working step (250 µs is the starting point) and **never to Auto**, which
   free-runs; write the step down, because the dial is unreachable once the
   housing is sealed.
2. Set the chosen header pin to GPIO with `sudo /opt/nvidia/jetson-io/jetson-io.py`
   and find its line offset with `gpioinfo`. **Measure the pin's state across a
   power cycle before connecting the board** (§2.2 of the brief): it must not
   pulse at boot.
3. Install the strobe agent:

   ```sh
   WILDSYNC_STROBE_LINE=<offset> deploy/deploy.sh strobe
   ```

   (`WILDSYNC_GPIOCHIP`, default `gpiochip0`, and `WILDSYNC_STROBE_ACTIVE`,
   default `high`, are the other settings.) It installs `piagent-strobe.service`
   (active-high, for the board's rising-edge input), registers the Jetson with
   rigd as the camera-less node `jetson` (`~/rig/nodes.json` and
   `~/rig/camera_enabled.json`) and restarts rigd.
4. In the UI, Controls → Strobe: enabled, Node `jetson`, delta 10 ms; the header
   pill reads `strobe T+10ms`. Set the shutter to **1/30 or slower** on both
   bodies (the fleet default 1/200 is too fast; the preflight warns). Take one
   frame with the strobe disabled, then Capture one with it enabled: the chip
   between the two frames must read **LIT**. Point the light away from people.

The bench plan that gates field use is `docs/xr256-jetson-trigger.md` §6. A
flash on a contact-closure sync port wired to a camera Pi's GPIO is the earlier
design, still supported: `docs/strobe-trigger.md`. The operating procedure is in
the field manual, section 05.

## 9. Verifying the installation

Do these in order; each gates the next.

**1. The offline test gate** (any host, no hardware, against in-process fake
nodes; `soaktest` includes the per-file audit suites, and its one
hardware-dependent check, `ilxctl builds (make)`, needs the Sony SDK staged):

```sh
python3 rig/tests/selftest.py --offline    # contracts: naming, header, encodings
python3 rig/tests/soaktest.py              # state machine, convergence, pull faults
python3 rig/nav.py --selftest              # PGN decode, UTM, time authority
python3 rig/tests/navtest.py               # nav integration over a real pty
python3 rig/navlog.py --selftest           # raw NMEA log + replay
```

Each ends with `ALL PASS` or `PASS` and exits 0. `python3 rig/tests/soaktest.py
--soak 300` adds randomized fault injection, and the calibration and VSLAM
suites are `python3 rig/tests/rigcal_selftest.py` and `python3
rig/tests/vslam_selftest.py` (the latter runs with or without OpenCV installed).

**2. What software is installed.** `python3 rig/buildinfo.py` prints the stamp
this checkout would record (version, commit, dirty flag). Every run records it
in `run.json` (`software`) and `events.log`.

**3. Each node** (`docs/PI-SETUP.md` §9). On the node:

```sh
systemctl is-active ilxctl piagent olive-bridge       # active, active, active
cat /sys/module/usbcore/parameters/usbfs_memory_mb    # at least 150
grep -o 'usbcore.usbfs_memory_mb=[0-9]*' /proc/cmdline    # present == survives reboot
```

From the host:

```sh
curl -s 192.168.1.201:8081/gpio/state | python3 -m json.tool
curl -s 192.168.1.201:8081/health     | python3 -m json.tool
curl -s 192.168.1.201:8080/api/status | python3 -m json.tool
curl -s localhost:9090/api/fleet      | python3 -m json.tool
```

Look for `harness_safe` true, `imu.present` true, `connected` true on the camera
and `afAllowed` **false** (true means `ilxctl` was started with
`--allow-autofocus`, which must never happen on a field node).

**4. Fire it once.** A node that answers every endpoint and has never pulsed
`TRIGGER` has proved nothing about the harness. With the body attached, press
**Capture one** in the UI and confirm an exposure edge comes back, then work the
pre-dive checklist in `docs/FIELD-RUN.md` (power, clocks, cards, settings, nav,
disk, host clock, strobe, calibration). The UI preflight lists every warning
before a run starts.

**5. The live validation script**, against the running rig (read-only unless
asked to fire):

```sh
python3 rig/tests/live_validate.py            # passive checks only
python3 rig/tests/live_validate.py --fire 10  # + 10 scheduled captures, skew and lateness
python3 rig/tests/live_validate.py --run 10   # + a 10-frame transect at 2 s
```

It checks that both cameras are connected and at their documented addresses, that
settings have converged, that both bodies are on manual focus and agree on format,
white balance and exposure, that no clock-skew alert is raised, and, with `--fire`
or `--run`, that skew and lateness are inside the budget and every pair is
complete. `--fire` and `--run` fire the cameras for real. Without `--drain` a
transect is not followed by an automatic card drain.

**6. A recorded run is verifiable.** After a run and a card drain, ingest the
RAWs (`docs/FIELD-RUN.md`, "Card drain") and check the run's hashes:

```sh
python3 rig/ingest.py --verify <run_id>
```

Exit 0 means every listed file matches its recorded SHA-256 (`docs/PROTOCOL.md`,
"Run provenance").

## 10. Where the details are

| For | Read |
|---|---|
| Operating the rig: the screen, a transect, the strobe, the cards, alerts | `docs/USER-GUIDE.html` (field manual) |
| The boat checklist and recovery commands | `docs/FIELD-RUN.md` |
| A node from a bare SD card; troubleshooting | `docs/PI-SETUP.md` |
| IMUs | `docs/olive-imu.md` |
| The XR256 and the contact-closure flash | `docs/xr256-jetson-trigger.md`, `docs/strobe-trigger.md` |
| The camera harness, leads and timing | `docs/harness-and-strobe.md` |
| Interfaces, run layout, time model, provenance | `docs/PROTOCOL.md` |
| What still needs measuring | `docs/future-tests.md` |
| Python packages, licences, components | `requirements.txt`, `LICENSE`, `THIRD_PARTY_NOTICES.md`, `sbom.spdx.json` |
| What changed in each version | `CHANGELOG.md` |

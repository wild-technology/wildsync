# Pi node setup — bare SD card to a running camera node

The build sheet for one camera node: a Raspberry Pi that sits beside one camera
and answers `:8080` (`ilxctl`, the camera over USB) and `:8081` (`piagent`, the
GPIO harness, the node's IMU and its health) to the rig host. It assumes nothing
is installed and takes you to a node that shows up in the rig's fleet header.

The rig host — the machine that runs `rigd` — is a **Mac** or a **Jetson Orin
NX**. The node procedure is the same for both; where they differ the text says
so. The whole installation, in order (hardware, host, nodes, clocks, IMUs,
navigation, strobe, verification), is [`docs/INSTALL.md`](INSTALL.md); this file
is its detailed reference for the nodes.

The examples build the third node, **cam3**; for cam1 or cam2 substitute the
node's own name and address.

| Part | Sections | Applies to |
|---|---|---|
| **I — the node** | §1–§5, §7–§8 | Card, cloud-init, network, the clock trap, fleet join, power. |
| **II — Sony ILX-LR1** | §6, §9, §10 | The camera stack: `ilxctl` + the 4-lead harness. |

Parts I and II interleave across §1–§10 because a node build does not split
cleanly in two — §6 and §9–§10 are Sony-specific, the rest is common ground.
Do the whole file in order.

Cross-references, not duplicates — read these rather than trusting a paraphrase:

| For | Read |
|---|---|
| Fleet table, GPIO pinout, node HTTP APIs, anomaly list | `docs/PROTOCOL.md` |
| Boat topology, pre-dive checklist, what the UI tells you | `docs/FIELD-RUN.md` |
| The whole installation in order | `docs/INSTALL.md` |
| Un-wedging `ilxctl`, the faults behind the warnings here | §10 below, `README.md` (Operating notes) |
| Build, `make sdk`, deploy commands, operating notes | `README.md` |

Throughout, **camN** = cam3 and **192.168.1.20N** = 192.168.1.203. The
convention is fixed and load-bearing: `pi-camN` implies `192.168.1.(200+N)`,
and `deploy/pi-resync/user-data.template` derives a node's chrony slot from its
hostname.

---

## 1. What you need

| Item | Note |
|---|---|
| Raspberry Pi 5 (4 GB or 8 GB) | cam1 is a Pi 5, **cam2 is a Pi 4 — both work**. The 40-pin header pinout is identical; only the GPIO chip differs (Pi 5 `gpiochip4`/pinctrl-rp1, Pi 4 `gpiochip0`/pinctrl-bcm2711). `piagent` **discovers** it at runtime by finding the pinctrl chip with ≥40 lines (`rig/piagent.py:discover_gpiochip`) — never hard-code it. |
| Ubuntu **24.04 LTS arm64 preinstalled SERVER** image | Not desktop. The fleet contract is Ubuntu 24.04 + Python 3.12 + libgpiod v1.6.3 (`docs/PROTOCOL.md`). A desktop image adds a display stack these nodes never use and changes the boot-time and power picture for nothing. |
| SD card, **V60 / UHS-II**, 64 GB+ | The card in the *Pi* holds the OS and the PC-save spool. The card in the *camera body* is the archive and must also be fast — cam1's old body card stalled on L-size RAW writes (`docs/FIELD-RUN.md` pre-dive §3). rigd deletes each frame from `~/Pictures/ILX-LR1` once it has pulled it; the spool only grows when pulls fail (`spool_not_draining`). |
| PoE HAT (802.3at) | See §8. cam3 runs on its **own dedicated injector**. |
| GPIO trigger harness | 4 leads. Pinout in §1.1 and `docs/PROTOCOL.md`. |
| Sony ILX-LR1 body + USB cable to the Pi | One body per node. `ilxctl` takes the single camera on the bus; `--camera`/`--match` exist for the dev bench only. |
| **Olive olixVision IMU** + USB-C cable | One per camera, into one of the Pi's USB ports. `deploy.sh node` configures it (§6.1). |
| A host on the **rig switch** | The machine that runs rigd (a Mac or a Jetson Orin NX), wired to the rig switch. A laptop host must be wired too, not on Starlink Wi-Fi — see §4. |

### 1.1 GPIO harness pinout

Wired and verified against Sony p.414 (`docs/PROTOCOL.md`):

| BCM | Header pin | Dir | Harness pin | Signal |
|---|---|---|---|---|
| 17 | 11 | out (open-drain) | 4 | FOCUS |
| 27 | 13 | out (open-drain) | 5 | TRIGGER — pulse LOW ≥1 ms, FOCUS must already be LOW |
| 22 | 15 | in (bias pull-up) | 6 | EXPOSURE — LOW while the front curtain is fully open |

Plus GND. **Never drive FOCUS or TRIGGER high** — "off" is high-Z, not logic 1.
A Low FOCUS line is a permanent half-press and takes the body out on *both*
USB and GPIO (§10).

---

## 2. Flash the card, headless

Image: `ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz` from
<https://cdimage.ubuntu.com/releases/24.04/release/>. The point release moves —
take the exact filename and the `SHA256SUMS` from that directory listing on the
day, do not paste an old one.

Verify before writing. A truncated download flashes fine and fails later as
something that looks like bad hardware:

```sh
cd ~/Downloads
curl -O https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz
curl -O https://cdimage.ubuntu.com/releases/24.04/release/SHA256SUMS
shasum -a 256 ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz
grep 'ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz' SHA256SUMS
# the two hashes must match, character for character
```

### Route A — Raspberry Pi Imager

1. Choose OS → **Use custom** → the `.img.xz` you just verified.
2. Choose Storage → the SD card. Check the size and the device name.
3. **Skip Imager's own OS customisation.** It writes its own `user-data` and
   will overwrite what §3 puts on the card. If you do use it, apply §3 *after*
   the flash completes and the card re-mounts.
4. Write, then let it verify.

### Route B — command line on a Mac

```sh
diskutil list external physical        # find the card — check the SIZE
diskutil unmountDisk /dev/disk4        # unmount, do NOT eject
xz -dc ~/Downloads/ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz \
  | sudo dd of=/dev/rdisk4 bs=4m
# ...then §3, then:
diskutil eject /dev/disk4
```

- **`of=` takes the wrong disk silently and destroys it.** `dd` will not ask.
  Run `diskutil list external physical` *with the card out and again with it in*
  and use the device that appeared. Confirm the size matches the card. There is
  no undo.
- `/dev/rdiskN` (raw), not `/dev/diskN` — the raw device skips the buffer cache
  and is roughly an order of magnitude faster here.
- `bs=4m` is BSD `dd` syntax (lowercase `m`). macOS `dd` has no
  `status=progress`; press **Ctrl-T** for a progress line.
- After `dd` returns, macOS re-mounts the FAT partition as
  `/Volumes/system-boot`. Do §3 before ejecting. (If you ejected already:
  `diskutil mountDisk /dev/disk4`.)

---

## 3. Headless cloud-init on `system-boot`

Ubuntu's raspi image boots cloud-init's NoCloud datasource against the **FAT
`system-boot` partition** — the small one the flashing computer mounts. It reads `user-data`,
`meta-data` and `network-config` from there on first boot. Editing those three
files is the whole headless setup; nothing needs a keyboard or a monitor.

### `user-data`

```yaml
#cloud-config
hostname: pi-cam3
manage_etc_hosts: true

# Create the groups BEFORE the user. cloud-init's user step fails on a group
# that does not exist, and that leaves an image which boots and then refuses
# your key. Whether this particular image has already created gpio/i2c/spi is
# not worth betting a re-flash on; declaring them here is idempotent. `gpio` is
# not cosmetic: deploy.sh installs a udev rule giving that group
# /dev/gpiochip*, which is what lets piagent drop its sudo fallback.
groups:
  - gpio
  - i2c
  - spi

users:
  - name: ubuntu
    gecos: wildsync camera node
    shell: /bin/bash
    sudo: ALL=(ALL) NOPASSWD:ALL
    lock_passwd: false
    passwd: __PASSWD_HASH__
    groups: [adm, sudo, plugdev, video, dialout, gpio, i2c, spi]
    ssh_authorized_keys:
      - __HOST_KEY__

ssh_pwauth: true
chpasswd:
  expire: false
```

- The group list matches `deploy/pi-resync/user-data.template` — keep the fleet
  uniform. `plugdev` is the group `deploy/ilxctl.service` names for USB access
  to the body; `dialout` is for serial IMUs.
- `__HOST_KEY__` is the contents of `~/.ssh/id_ed25519.pub` **on the rigd host**
  — the machine that will run `deploy.sh`. Everything in `deploy/` assumes
  passwordless ssh from that host.
- `__PASSWD_HASH__` is optional but worth having: it is the only way back in
  with an HDMI screen and a keyboard when the network is the thing that is
  broken. Generate it locally with `openssl passwd -6` (typed interactively) and
  paste the hash. **Never commit a hash or a password to this repo.** Drop
  `passwd:`, `lock_passwd:` and `ssh_pwauth:` entirely if you want key-only
  (`deploy/pi-resync/install.sh` then refuses the card, because it carries the
  existing hash over; add one by hand before running it).
- `hostname:`, not `preserve_hostname: true`. A fresh card has to be named
  here, and the name is what the chrony step keys off. `deploy/pi-resync/install.sh`
  keeps a card's `hostname:` (with `manage_etc_hosts: true`) when it rewrites
  `user-data`, and writes `preserve_hostname: true` only for a card that has
  none, i.e. one already booted from an earlier run of the template.

### `network-config`

```yaml
version: 2
ethernets:
  # BOTH names, deliberately — see §5.2. Ubuntu's raspi image ships `eth0`, but
  # this image's cmdline.txt does not set net.ifnames=0, so predictable naming
  # is live and a Pi 5's RP1 NIC can come up as `end0`. Only the name that
  # exists is configured; the other generates an inert .network file and costs
  # nothing. Guessing wrong on a headless card costs a re-flash.
  eth0:
    dhcp4: true
    optional: true
    addresses:
      - 192.168.1.203/24
  end0:
    dhcp4: true
    optional: true
    addresses:
      - 192.168.1.203/24
```

- **Static address *alongside* the DHCP lease**, which is what cam1 and cam2
  do: "a bad netplan cannot strand a node" (`docs/PROTOCOL.md`). The rig
  addresses the node by `.203`; the lease is the way back in if the static
  config is wrong.
- **No default route and no nameservers here.** The rig link is a LAN path to
  the cameras, not the node's route to the world, and a second default route on
  192.168.1.0/24 is exactly the collision in §4.
- **Which name actually wins is measured, not assumed** — see §5.2. On cam3
  (Pi 5, Ubuntu 24.04.4, kernel 6.8.0-1047-raspi) it was `eth0`. Confirm on
  first boot with `ip -br link`. If a future board answers to neither, key the
  block on the MAC (`match: {macaddress: ...}`) rather than guessing again.

### `meta-data`

Leave the image's `meta-data` as it is on a **fresh** card. It only needs
touching when re-applying cloud-config to a card that has already booted — that
is what `deploy/pi-resync/install.sh` does, by writing a new `instance-id` so
cloud-init re-runs its per-instance modules.

Then eject: `diskutil eject /dev/disk4`.

---

## 4. ⚠ The subnet collision — read before you plug anything in

**The rig switch and the Starlink Wi-Fi both use 192.168.1.0/24, with a gateway
at 192.168.1.1.** They are two separate L2 segments wearing the same address
range. The Pis live only on the wired one.

With both interfaces claiming the same directly-connected subnet, macOS keeps
the 192.168.1.0/24 route on the primary service — Wi-Fi — so **every packet you
send to a camera leaves via Wi-Fi and dies.** Setting a static IP on the
Ethernet NIC is necessary but **not sufficient**. The symptom is a node that
pings from nothing, ssh that hangs, and a rig header stuck one camera short,
all while the Pi is perfectly healthy.

A Jetson host is wired to the rig switch and has no Wi-Fi on that range, so it is not affected. A **laptop** host on both networks is. To reach the cameras from a Mac:

```sh
sudo networksetup -setairportpower en0 off      # Wi-Fi OFF; static IP alone is not enough
ping6 -c 3 "ff02::1%en5"                        # enumerate L2 neighbours on the wired iface
```

The `ping6` multicast trick lists who is actually on that segment without
needing DHCP — Pi OUIs are `d8:3a:dd` and `e4:5f:01`. Note that IPv6 link-local
is **not** a fallback path to the agent: `rig/piagent.py` binds `("0.0.0.0",
PORT)` and is IPv4-only.

The permanent fix is to move the Starlink LAN off 192.168.1.0/24 (Starlink app →
Settings), since the node addresses are pinned to that range.

---

## 5. First boot and verification

Fit the card, connect Ethernet to the rig switch, apply PoE. First boot expands
the root filesystem and runs cloud-init; give it a few minutes and **do not pull
power** while it does. Then, from the rigd host:

```sh
ssh ubuntu@192.168.1.203
```

On the node:

```sh
cloud-init status --wait                     # must end 'status: done'
hostnamectl --static                         # pi-cam3
ip -4 -br addr show eth0                     # 192.168.1.203/24 (plus any lease)
tr -d '\0' < /proc/device-tree/model; echo   # e.g. Raspberry Pi 5 Model B Rev 1.0
free -h                                      # RAM — confirms which Pi you actually got
python3 --version                            # 3.12 per the fleet contract
id ubuntu                                    # gpio, i2c, spi, plugdev, dialout present
sudo -n true && echo "passwordless sudo OK"
```

`cloud-init status` reporting `error` does **not** always mean the user/group
step failed — read `sudo cloud-init status --long` before concluding anything.
On cam3's real first boot it was `package_update_upgrade_install` that failed,
and that failure has a specific cause worth knowing before you debug the wrong
thing. A node that boots but refuses your key is a different fault, almost
always a group in `users:` that did not exist (§3).

### ⚠ 5.1 The clock trap — a fresh node boots months in the past

**Fix the clock before you let `apt` run.** The Pi 5 has an RTC but no battery
on this build, so Ubuntu's `fixrtc` seeds the clock from the *image build date*.
cam3 was flashed and booted on **2026-08-28** and came up believing it was
**2026-02-10** — 198 days behind. Every consequence follows from that:

```
E: Release file for http://ports.ubuntu.com/... noble-updates/InRelease is not
   valid yet (invalid for another 198d 23h 8min 28s).
Err:11 ... cpp-13-aarch64-linux-gnu arm64 13.3.0-6ubuntu2~24.04
  404  Not Found
```

apt refuses a Release file dated in its future, silently falls back to the
package lists baked into the image, and every download 404s because those
versions have since been superseded. Exit code 100. **Nothing** in
`build-essential`, `chrony`, `python3-libgpiod` or `gpiod` installed — while
`rsync`, `libusb-1.0-0` and `libxml2` reported fine, because those ship in the
base image and were never downloaded. That mix is the tell.

It is worse than a failed install. The `runcmd` had already disabled
`systemd-timesyncd` to hand the clock to chrony, and chrony was never installed
— so the node sat with **no time discipline at all**, on a rig whose entire
value is a 10 ms inter-camera budget.

Recovery, on the node:

```sh
sudo timedatectl set-ntp true                # borrow timesyncd just to get the date right
timedatectl show -p NTPSynchronized --value  # wait for 'yes' (took ~10 s)
sudo apt-get update && sudo apt-get install -y \
    build-essential chrony python3-libgpiod gpiod libusb-1.0-0 libxml2 rsync
sudo timedatectl set-ntp false               # then hand the clock back to chrony
sudo systemctl disable --now systemd-timesyncd
sudo systemctl enable --now chrony
```

`deploy/deploy.sh provision` now does the `set-ntp`/`apt-get update` dance up
front, refuses to disable `timesyncd` until chrony is confirmed installed, and
says so loudly if it is not. Before that fix it printed success in exactly this
state. If you are provisioning a node from an older checkout, do the above by
hand first.

### 5.2 What the interface is actually called

`network-config` in §3 defines **both** `eth0` and `end0` on purpose. Ubuntu's
stock image ships `eth0`, but the image's `cmdline.txt` does not set
`net.ifnames=0`, so predictable naming is live and the Pi 5's RP1 NIC could
come up either way. Guessing wrong on a headless card costs a re-flash to find
out.

**Measured on cam3 (Pi 5 Model B Rev 1.0, Ubuntu 24.04.4, kernel
6.8.0-1047-raspi): `eth0` won.** The `end0` stanza generated an inert
`.network` file and cost nothing. Keep both until a Pi 5 is seen to do
otherwise — the hedge is free, the re-flash is not.

Record the model string: it is the fleet-table entry, and it is what tells the
next person which `gpiochip` to expect.

---

## 6. Install the rig software

Three commands, in this order, from the checkout on the rigd host (a Mac, or a
Jetson after `deploy/deploy.sh jetson` has run there once):

```sh
cd /path/to/wildsync
deploy/deploy.sh trust     cam3     # push this host's ssh key (asks for the Pi password)
deploy/deploy.sh provision cam3     # Sony SDK, usbfs 150 MB, chrony
deploy/deploy.sh node      cam3     # build ilxctl, install units, start everything
```

`trust` is a no-op if §3 already installed the host's key — the point is that
`provision` and `node` assume passwordless ssh and will simply hang on a
password prompt otherwise.

### The prerequisite `provision` enforces

`provision` needs the **Linux aarch64** Camera Remote SDK to push to the node,
and **refuses to run** without it:

```
REFUSING provision: no Linux aarch64 libCr_Core.so to send.
```

```sh
make sdk-linux CRSDK_DIR=/path/to/RemoteCli    # RemoteCli folder of the Linux64ARMv8 package
```

The SDK is Sony's proprietary package (v2.02.00) and is not in this repository:
obtain it from Sony, unzip it, and unzip the `RemoteCli.zip` inside; that
`RemoteCli` folder is `CRSDK_DIR`. The command stages the `.so` files into
`lib-linux/`, which `provision` prefers, and the SDK headers into
`include/CRSDK/`, which `provision` also pushes because the node builds
`ilxctl` from them (`provision` refuses to run when either is missing). It
works the same on the Jetson and on a Mac, and leaves a Mac's own `lib/`
(macOS dylibs for a local `ilxctl` build) untouched.

Never push macOS `.dylib` files to a node: that once produced a node build with
a missing `libCr_Core.so` that looked like a successful provision (found
2026-08-27), which is what the refusal exists to prevent.

`ilxctl` is **always built on the node** — the Makefile probes `-mcpu=native`
and a binary built elsewhere SIGILLs. Never scp the binary between machines.

### What `node` installs

`deploy_node()` builds `ilxctl` on the box, installs the gpio udev rule, and
copies in `piagent.service`, `olive-bridge.service` and `ilxctl.service`. Watch
its last line:

```
  ilxctl: active   piagent: active
```

`ilxctl` runs as `ubuntu`, not root. `node` installs the udev rule that lets
it open the camera (`/etc/udev/rules.d/99-wildsync-sony.rules`: Sony vendor
`054c`, group `plugdev`) and adds `ubuntu` to `plugdev`. If the unit is active
but finds no camera, check that rule and that the body is in PC Remote mode
before anything else. cam1 and cam2 were originally set up with hand-installed
units; compare once with `ssh ubuntu@192.168.1.201 'systemctl cat ilxctl'`.

### 6.1 The Olive IMU

Each camera carries an Olive olixVision IMU on its node. Plug its USB-C into
one of the Pi's USB ports; `node` does the rest:

- it pins the IMU's USB network link as `olive0`, Pi `192.168.7.2/24`, IMU
  `192.168.7.100` (`/etc/netplan/60-wildsync-olive.yaml`, applied only when it
  changes);
- `olive-bridge.service` relays the IMU's stream to piagent on local UDP 9901;
- `/etc/default/piagent` gets `PIAGENT_IMU=olive:udp:9901`, so the Olive is
  the node's primary IMU and its attitude goes into that camera's flight log.

Check it:

```sh
ssh ubuntu@192.168.1.20N 'ip -br addr show olive0; systemctl is-active olive-bridge'
curl -s 192.168.1.20N:8081/health | python3 -c "import sys,json; print(json.load(sys.stdin)['imu'])"
```

`present: true` with an `attitude_hz` around 15 is healthy. For a node that
still carries the older Yahboom IMU, deploy it with
`WILDSYNC_NODE_IMU=yb deploy/deploy.sh node camN`.

### Chrony, after provisioning

The node's clock topology has three layers (`docs/INSTALL.md` §5), applied by
the card step and re-applied by `deploy.sh clock camN` (which `deploy.sh node`
runs first):

1. **The GNSS Compass — every node, either host.** `192.168.1.60` is a
   stratum-1 NTP server on GPS time; `/etc/chrony/conf.d/20-wildsync-gnss.conf`
   names it with `prefer`. A Pi has no RTC battery and boots where its
   filesystem left it, so `chrony.conf` is set to `makestep 1 -1`: any error
   over a second is stepped the moment a reference answers, however many
   updates the node has already made (Ubuntu's `makestep 1 3` gives up after
   three, and is parsed after the drop-ins, so a drop-in cannot override it).
2. **The peer mesh — every node, either host.** The card's own step
   (`deploy/pi-resync/user-data.template`) writes a full mesh of peers across
   `192.168.1.201/.202/.203` plus an orphan local reference
   (`10-wildsync.conf`), so the nodes stay within about a millisecond of each
   other with the Compass off. `deploy.sh clock` writes the same file on a node
   whose card never ran that step (cam1's had not, 2026-10-15: it still carried
   the two-node `10-jetson.conf` star).
3. **The Jetson as a server — Jetson host only.** `deploy.sh jetson` makes the
   Jetson a chrony server for 192.168.1.0/24 (itself on the Compass), and
   `deploy.sh node`, run **from that Jetson**, adds
   `/etc/chrony/conf.d/20-wildsync-host.conf` pointing the node at it. The
   Jetson fires the XR256 on its own clock and the nodes fire the cameras on
   theirs, so one reference keeps the strobe timing honest.

A **Mac host is not a chrony master** and need not be: the nodes hold GPS time
directly. `deploy.sh mac` points the Mac's own clock at the Compass so the host
offset rigd measures and corrects every fire and capture instant for
(`host_clock_offset`; `docs/PROTOCOL.md`, "CLOCK DOMAINS") is small; nothing on
the Mac answers NTP and `deploy.sh clock` writes no host line when run from it.
A node that is still wrong because the Compass was off while it booted gets
rigd's `node_clock_wrong` anomaly and its **set camN clock from host** button
(`POST /api/node/clock/step`), which steps the node to the host's clock.

`provision` writes no chrony configuration of its own, and removes the
`wildsync.conf` that earlier versions wrote. Verify per §9: `chronyc -n
sources` on a node should show the Compass selected (`*`) and its peers behind
it.

---

## 7. Joining the fleet — `~/rig/nodes.json`, not a code edit

On the **rigd host**:

```sh
mkdir -p ~/rig
cat > ~/rig/nodes.json <<'JSON'
{"cam3": {"host": "192.168.1.203", "cam_num": 3}}
JSON
```

**Why this and not a line in `rigcore.py`.** cam3 was in `_DEFAULT_NODES` once
and was removed on 2026-08-19: the empty third slot fired a **permanent
`node_offline` anomaly and a forever-"2/3" header** for hardware that did not
exist (`docs/PROTOCOL.md`, and the comment still standing in
`rig/rigcore.py:_DEFAULT_NODES`). Defaults describe hardware that is always
there. A third camera that may or may not be on the boat this trip belongs in
the host's own config, where removing it is deleting a file rather than
reverting a commit. **Do not add cam3 back to `_DEFAULT_NODES`.**

Two shapes, and the difference matters (`rigcore._load_nodes`):

- **A JSON object is additive** — the keys merge onto the defaults. An unknown
  name is appended as a new node. This is the form above, and the one you want.
- **A JSON array replaces the fleet entirely.** Use it only when you mean to
  redefine every node.

`cam_num` is not decoration: it is what frames are renamed with —
`Cam3_YYYYmmdd_HHMMSS...` (`rig/run.py:_fmt_fname`). Set it explicitly rather
than letting it be inferred.

`NODES` is built at import, so **rigd must be restarted** to see the file:

```sh
launchctl kickstart -k gui/$(id -u)/org.wildtechnology.wildsync.rigd   # Mac host
sudo systemctl restart rigd                                            # Jetson host
curl -s localhost:9090/api/fleet | python3 -m json.tool
```

The UI header pill should now read **`3/3 cameras`**. `3/3` is
connected/total — `2/3` means the third node is in the fleet but not connected,
which is a node problem (§9, §10), not a `nodes.json` problem. No change at all
in the header means rigd did not read the file: check the path and that it is
valid JSON (an unparseable file is silently ignored — `_load_nodes` swallows
`OSError`/`ValueError` and falls back to the defaults).

**cam3 is a third survey camera, not half of a new pair.** The stereo pairing in
`rigcal.py` / `vslam.py` / `ingest.py` is cam1↔cam2 and stays that way.

### 7.1 Two ways a camera leaves the fleet

They are different, and the difference is the whole point.

| | `optional`, never seen | camera switched off |
|---|---|---|
| Means | that node is not on the boat | that camera is not wanted on this dive |
| Set by | `"optional": true` in `~/rig/nodes.json` (the **default** for any node added there) | `POST /api/camera/enabled`, or the header buttons in the UI |
| Persisted in | `~/rig/nodes.json` | `~/rig/camera_enabled.json` (survives a rigd restart) |
| Node polled? | no — it does not exist yet | **yes** |
| Clock disciplined? | n/a | **yes** — it may be hosting the strobe |
| Joins runs? | no | no |
| Camera anomalies? | none | **none** — "camera not claimed" and "no card" are the intended state |
| Can host the strobe? | no | **yes** |
| Header reads | `2/2` | `2/2` |

A camera switched off stops drawing power and stops spooling frames nobody will
use — **without costing you the box**. That distinction is what lets a node be a
strobe host with its camera dark. The code models it as two predicates on
`NodeMonitor`: `is_present()` (is the *node* in the fleet) and `is_capturing()`
(does its *camera* take part).

```sh
# turn cam3's camera off for this dive; the Pi stays live
curl -s -XPOST localhost:9090/api/camera/enabled \
  -H 'Content-Type: application/json' \
  -d '{"node":"cam3","enabled":false}' | python3 -m json.tool
```

The UI offers this only for `cam_num > 2`. cam1 and cam2 are the stereo pair
every paired product depends on, and hiding the switch is cheaper than explaining
mid-dive why shooting half a pair is a bad idea.

**This is also how a camera-less strobe host joins the fleet** — register it in
`nodes.json`, switch its camera off, and it is polled, clock-disciplined and
selectable as the strobe node while raising no camera alarms. A node with no
camera otherwise sits in `ILX_DOWN` forever. See `docs/xr256-jetson-trigger.md`.

---

## 8. Power — cam3 gets its own injector, and is NOT trimmed

**cam3 runs on its own dedicated 802.3at PoE injector, exactly like cam1.** Not
a switch port shared with the rest of the rig.

The measured history (bench, 2026-08-23, `docs/FIELD-RUN.md` and
`deploy/pi-power.sh`): with two nodes firing **synchronously**, one node's PoE
port dropped every time. Either node alone was clean. It was not a Pi brown-out
— the **PSE shed the port**: the Ubiquiti switch's PoE budget at the
synchronized current spike. Isolating cam1 onto its own injector removed it
outright: **60 synchronized fires, 0 loss.** A third body firing on the same
instant makes that spike larger, not smaller.

**Do not apply power trimming to cam3.** Concretely:

- Do **not** run `deploy/pi-power.sh cam3`. It caps `arm_freq`, sets
  `dtoverlay=disable-wifi` / `disable-bt`, and pins the cpufreq governor to
  `powersave`. cam3 is listed in that script's address table only so the tool
  can *reach* the box — e.g. to strip a trim someone applied by mistake with
  `deploy/pi-power.sh cam3 --strip`, which removes the config block and the
  governor unit and reboots (`--no-reboot` skips that; an unknown option stops
  the script before it touches the node). Being addressable is not permission
  to trim.
- Do **not** pass `--pi5-power-trim` when writing cam3's card with
  `deploy/pi-resync/install.sh`.
- `pi-power.sh` is **opt-in, per node, always run by hand**. `deploy.sh` does
  not call it and must not start.

The trim is a real cost — capped clocks, no radios — paid to squeeze a node into
a PoE budget that cannot be changed. cam3's budget *can* be changed, and has
been, by giving it its own injector. Trimming it would surrender performance to
solve a problem it does not have.

The tells if the power path is wrong anyway: the `node_rebooted` anomaly (a Pi
lost power mid-run) and `node_undervoltage` (the rail is sagging). Both are in
`docs/FIELD-RUN.md`'s anomaly table.

---

## 9. Verification checklist

**On the node:**

```sh
systemctl is-active ilxctl piagent olive-bridge   # active, active, active
cat /sys/module/usbcore/parameters/usbfs_memory_mb   # 150 or more (see §10)
grep -o 'usbcore.usbfs_memory_mb=[0-9]*' /proc/cmdline   # present == survives reboot
```

**From the rigd host:**

```sh
curl -s 192.168.1.203:8081/gpio/state | python3 -m json.tool
curl -s 192.168.1.203:8081/health     | python3 -m json.tool
curl -s 192.168.1.203:8080/api/status | python3 -m json.tool
curl -s localhost:9090/api/fleet      | python3 -m json.tool
```

What to actually look at:

| Endpoint | Check |
|---|---|
| `:8081/gpio/state` | `harness_safe` true — the lines are open-drain-idle, no stuck half-press. `chip` is `gpiochip4` on a Pi 5 (`gpiochip0` on a Pi 4) — discovered, not configured. |
| `:8081/health` | `imu.present` true (the Olive, §6.1), `gpio.ok` true, `gpio.focus_held` false, `disk_free_mb` sane, `power.undervolt_now` false. `node` is the **hostname** (`piagent.py` sets `NODE = socket.gethostname()`), so it reads `pi-cam3` while rigd calls the node `cam3` — that is expected, not a mismatch. |
| `:8080/api/status` | `connected` true, `model`, `id` (record the body serial for the fleet table), `slotStatus`, `remainShots`. **`afAllowed` must be `false`** — `true` means the daemon was started with `--allow-autofocus`, which must never happen on a field node. |
| `:9090/api/fleet` | one entry per camera node (plus `jetson` once the strobe agent is installed on a Jetson host); cam3 present with `cam_num` 3. |

**Clocks.** Every scheduled fire and every `epoch_hw` edge lives on the *node's*
clock, so inter-node clock skew lands 1:1 in true exposure skew while the rig's
own skew figures under-report it. The pair budget is **10 ms**. Free-running,
with no master and no peer mesh, the nodes measured **16.8 ms apart**
(2026-08-20) — outside budget, and invisible in the rig's own numbers. Peered in
orphan mode they sit at 0.6 ms.

```sh
for ip in 192.168.1.201 192.168.1.202 192.168.1.203; do
  echo "== $ip"; ssh ubuntu@$ip 'chronyc tracking | head -3; chronyc sources'
done
```

All three must print the **same Reference ID** — `C0A8013C (192.168.1.60)`,
the GNSS Compass, at Stratum 2 whenever it is up (§6, Chrony); with it off,
the Jetson's address on a Jetson rig, otherwise one node's (the mesh elects the
lowest address) — which is the one-line proof the group has exactly one master.
Two distinct refids is a split. After a master handover the followers slew
rather than step, so give it a minute before trusting the budget; a clock more
than a second out steps at once (`makestep 1 -1`). rigd's `node_clock_skew`
(not `chronyc`) is the detector that runs during a survey, and it needs three
consecutive scans to raise or clear; a node seconds off raises
`node_clock_wrong` on the first scan instead.

**Then fire it once.** A node that answers every endpoint and has never pulsed
TRIGGER has proved nothing about the harness. Do a bench fire with the body
attached and confirm an `EXPOSURE` edge comes back (`docs/FIELD-RUN.md`
pre-dive).

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Camera connects, answers property reads, writes to its own card — but PC image transfer dies mid-frame | `usbfs_memory_mb` is back at the 16 MB default. It **must** be set on the **kernel command line** (`usbcore.usbfs_memory_mb=150` in `/boot/firmware/cmdline.txt`), not in `modprobe.d` — `usbcore` is built into the Pi kernel, so `/etc/modprobe.d/usbfs.conf` is never read and the runtime `echo` silently reverted on every reboot. Check `/proc/cmdline`; re-run `deploy.sh provision cam3` and **reboot**. Presents as a broken camera. |
| Body reports every property read-only, hands control priority back, ignores the shutter on **both** USB and GPIO | A Low `FOCUS` line — a permanent half-press. **A released GPIO is not high-Z**: the pad keeps the last requester's bias. `piagent` holds the lines open-drain-idle for exactly this reason; check `harness_safe` in `/gpio/state`. Anything that drove the line by hand (a stray `gpioset`, a crashed test) can leave it there. |
| `ilxctl` "active" at 0% CPU, camera unusable; `systemctl restart` hangs | Stuck PTP session from an unclean shutdown. **Recovery recipe: §10.1** — `pkill -9 -x ilxctl`, USB unbind, USB bind, `systemctl start ilxctl`. `ilxctl` binds `:8080` *before* its startup connect (`src/main.cpp`), so the symptom is `/api/status` answering with `connected:false` rather than a dead port — but the PTP session still needs the unbind/bind to clear. `deploy/ilxctl.service` sets `RestartPreventExitStatus=SIGKILL` so a deliberate `-9` stays down long enough for the recipe to work. |
| `node_clock_skew` anomaly | Two nodes disagree beyond the noise floor (warn >5 ms, bad >8 ms), off the RTT-gated median, after 3 consecutive scans. Usually one node lost the Compass or the peer group — check that all three `chronyc tracking` refids match (§9) and that no leftover `/etc/chrony/conf.d/wildsync.conf` or `10-jetson.conf` (written by earlier versions; `deploy.sh clock` removes both) points a node at one host with `prefer`. A *common-mode* host-vs-node offset is invisible to this check by construction; that one is `host_clock_offset` (put the host on the Compass: `deploy.sh mac` / `jetson`). |
| `node_clock_wrong` anomaly | That node's clock is more than 2 s from the host's — a Pi power-cycled while the Compass was unreachable boots where its filesystem left it (47 days behind, cam1, 2026-10-15) — and every scheduled fire on it is refused. Click **set camN clock from host** on the anomaly (`POST /api/node/clock/step`; refused during a run), or `deploy.sh clock camN` once the Compass is back, then confirm `chronyc -n sources` shows `192.168.1.60` selected. |
| `node_clock_unmeasurable` | That node's best `/health` RTT is ≥20 ms, so the 10 ms budget cannot be verified at all. Check the PoE/switch path, not the clock. |
| Unit is `active`, but "no camera found" / `EnumCameraObjects` fails | USB permissions or the SDK adapter path. `deploy.sh node` installs the udev rule for the Sony body (`/etc/udev/rules.d/99-wildsync-sony.rules`, vendor `054c`, group `plugdev`) and adds `ubuntu` to `plugdev`; check both (`ls -l /dev/bus/usb/*/*`, `id ubuntu`) and that the body is in PC Remote mode. The adapter path is the `WorkingDirectory=` of `deploy/ilxctl.service` (`CrError_Adaptor_Create`, 0x8703, reads as "no camera found"). |
| Header stays `2/3` | The node is in the fleet but not connected — a node fault, not a config one. Work §9 from the top. |
| Header does not change at all after adding `~/rig/nodes.json` | rigd was not restarted, or the file is unparseable (silently ignored, defaults used), or you edited a checkout rather than `~/rig/nodes.json`. |
| Node unreachable from the host, but healthy on a screen | §4. Wi-Fi is holding the 192.168.1.0/24 route. |

### 10.1 Un-wedging `ilxctl`

`ilxctl` cannot recover from an unclean shutdown. Kill it mid-SDK-session and the
camera's PTP session stays stuck; the next start blocks in the SDK connect and
the process sits `active` at 0% CPU, looking exactly like a dead camera.
`systemctl restart` also hangs on the stuck stop — `deploy/ilxctl.service` bounds
that with `TimeoutStopSec=45`, so a `deploy.sh node` against a wedged daemon
waits up to 45 s at its restart step. It is slow, not stuck.

On the node, find the body on the USB bus (`lsusb -d 054c:` lists Sony devices;
the recipe below assumes product id `0e90`, so adjust it if the body shows
another), then:

```sh
DEV=$(for d in /sys/bus/usb/devices/*/; do [ -f "$d/idVendor" ] || continue;
  [ "$(cat $d/idVendor)" = 054c ] && [ "$(cat $d/idProduct)" = 0e90 ] && basename $d; done | head -1)
sudo pkill -9 -x ilxctl; sleep 1
echo -n "$DEV" | sudo tee /sys/bus/usb/drivers/usb/unbind; sleep 3
echo -n "$DEV" | sudo tee /sys/bus/usb/drivers/usb/bind;   sleep 4
sudo systemctl start ilxctl
```

`RestartPreventExitStatus=SIGKILL` in the unit keeps the deliberate `-9` down
until the final `systemctl start`; without it systemd would put a new daemon
back on the still-stuck bus inside those few seconds. The same side effect
applies to an OOM kill or any other external SIGKILL: the daemon stays down
until someone starts it, and rigd reports `ILX_DOWN` rather than a silent flap.
Power-cycling the camera body clears a stuck session as well, and is the first
thing to try when the body is within reach (field manual, "Recovery").

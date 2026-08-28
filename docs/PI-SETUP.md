# Pi node setup — bare SD card to a running camera node

Written while commissioning **cam3**, the third survey camera (branch
`cam3-node`). Until now every node was built by hand and nothing wrote the steps
down, so a new node meant reconstructing them from `deploy/deploy.sh` and two
working Pis.

This is the build sheet. It assumes nothing is installed and takes you to a node
that answers `:8080` and `:8081` and shows up in the rig's fleet header.

**cam3 carries two camera stacks.** As of 2026-08-28 the **Basler a2A4504** is
cam3's primary camera and the end-goal for testing; the **Sony ILX-LR1** stack
stays installed and working as the fallback, in case the rig reverts. Both are
built here, and they coexist on one node — nothing below asks you to choose.

| Part | Sections | Applies to |
|---|---|---|
| **I — the node** | §1–§5, §7–§8 | Both. Card, cloud-init, network, the clock trap, fleet join, power. |
| **II — Sony ILX-LR1** | §6, §9, §10 | The fallback stack: `ilxctl` + the 4-lead harness. |
| **III — Basler a2A4504** | §11–§16 | The primary stack: pylon, pypylon, the spool, measured results. |

Parts I and II interleave across §1–§10 because a node build does not split
cleanly in two — §6 and §9–§10 are Sony-specific, the rest is common ground.
Part III is self-contained. If you only want the Basler, do Part I and skip
straight to §11; if you want both, as cam3 has, do the whole file in order.

Cross-references, not duplicates — read these rather than trusting a paraphrase:

| For | Read |
|---|---|
| Fleet table, GPIO pinout, node HTTP APIs, anomaly list | `docs/PROTOCOL.md` |
| Boat topology, pre-dive checklist, what the UI tells you | `docs/FIELD-RUN.md` |
| Un-wedging `ilxctl`, the discoveries behind the warnings here | `docs/HANDOFF.md` §2.2, §3 |
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
| SD card, **V60 / UHS-II**, 64 GB+ | The card in the *Pi* holds the OS and the PC-save spool. The card in the *camera body* is the archive and must also be fast — cam1's old body card stalled on L-size RAW writes (`docs/FIELD-RUN.md` pre-dive §3). Size the Pi card against the spool: nothing prunes `~/Pictures/ILX-LR1` automatically. |
| PoE HAT (802.3at) | See §8. cam3 runs on its **own dedicated injector**. |
| GPIO trigger harness | 4 leads. Pinout in §1.1 and `docs/PROTOCOL.md`. |
| Sony ILX-LR1 body + USB cable to the Pi | The **fallback** stack. One body per node. `ilxctl` takes the single camera on the bus; `--camera`/`--match` exist for the dev bench only. |
| **Basler a2A4504-18umBAS** + **USB 3** cable | The **primary** camera (§11). USB3 Vision, 20.2 MP mono global shutter, IMX541, 4504×4504 Mono8. Must go in one of the Pi 5's **USB 3** ports — measured 360 MB/s. Bus-powered; see §11 on the current ceiling. |
| Lens covering a **1.1″** sensor | The Basler's format. Without one you get a frame with no scene structure, and every JPEG size measurement off it is meaningless (§13). |
| A host on the **rig switch** | Not on Starlink Wi-Fi. See §4 — this is the trap that eats an afternoon. |

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

### Route B — command line on the Mac

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
`system-boot` partition** — the small one the Mac mounts. It reads `user-data`,
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
      - __MAC_KEY__

ssh_pwauth: true
chpasswd:
  expire: false
```

- The group list matches `deploy/pi-resync/user-data.template` — keep the fleet
  uniform. `plugdev` is the group `deploy/ilxctl.service` names for USB access
  to the body; `dialout` is for serial IMUs.
- `__MAC_KEY__` is the contents of `~/.ssh/id_ed25519.pub` **on the rigd host**
  — the machine that will run `deploy.sh`. Everything in `deploy/` assumes
  passwordless ssh from that host.
- `__PASSWD_HASH__` is optional but worth having: it is the only way back in
  with an HDMI screen and a keyboard when the network is the thing that is
  broken. Generate it locally with `openssl passwd -6` (typed interactively) and
  paste the hash. **Never commit a hash or a password to this repo.** Drop
  `passwd:`, `lock_passwd:` and `ssh_pwauth:` entirely if you want key-only.
- `hostname:`, not `preserve_hostname: true`. The resync template preserves the
  hostname because it re-provisions an already-named node; a fresh card has to
  be named here, and the name is what the chrony step keys off.

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

To reach the cameras from the Mac:

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

Three commands, in this order, from the checkout on the rigd host:

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

`provision_node()` **refuses to run** unless `lib/libCr_Core.so` exists in the
checkout:

```
REFUSING provision: <repo>/lib has no libCr_Core.so (Linux SDK).
```

The nodes need the **Linux aarch64** Camera Remote SDK. `make sdk` stages
whatever platform the *host* builds for — on the Mac that is `.dylib` files —
and `provision` `rsync -az --delete`s `lib/` to the node wholesale. Rsyncing
macOS dylibs **bricked a node build with a missing `libCr_Core.so` while looking
like a completely successful provision** (audit 2026-08-27). The refusal exists
so that cannot happen twice; do not work around it.

Two clean ways to satisfy it:

1. **Provision from a Linux host.** `make sdk CRSDK_DIR=/path/to/unpacked/RemoteCli`
   there stages `.so` files and the Linux `CrAdapter/`, and `provision` pushes
   the right thing.
2. **From the Mac, use a separate copy of the checkout** staged with the Linux
   SDK package, and run `deploy/deploy.sh provision cam3` from that copy. Do not
   stage the Linux SDK into your working checkout: `lib/CrAdapter/` holds one
   platform's adapters at a time, so overwriting it breaks the Mac's own
   `ilxctl` build until you re-run `make sdk` against the macOS package.

`ilxctl` is **always built on the node** — the Makefile probes `-mcpu=native`
and a binary built elsewhere SIGILLs. Never scp the binary between machines.

### What `node` installs

`deploy_node()` builds `ilxctl` on the box, installs the gpio udev rule, and
copies in `piagent.service`, `olive-bridge.service` and `ilxctl.service`. Watch
its last line:

```
  ilxctl: active   piagent: active
```

**`ilxctl.service` did not exist in this repo until this branch.** cam1 and cam2
have been running hand-installed units nobody ever captured; `deploy/ilxctl.service`
is a reconstruction from the daemon's own behaviour, and its header says so.
Before assuming the fleet is uniform, diff it against the real thing:

```sh
ssh ubuntu@192.168.1.201 'systemctl cat ilxctl'
```

The open question is USB permissions: there is **no udev rule for the Sony body
anywhere in this tree** (`deploy.sh` installs only `99-gpio.rules`), so whether
a non-root `ilxctl` can claim the camera depends on how cam1's hand-installed
unit does it. That is called out in the unit's own comments with the exact
commands to settle it. Until it is settled on hardware, treat "unit active, no
camera found" as the expected failure mode and check there first.

### Chrony, after provisioning

`provision` writes `/etc/chrony/conf.d/wildsync.conf` pointing the node at
whatever host ssh'd in, with `prefer` — i.e. the Mac, which is deliberately
**not** a rig clock master. The card's own step
(`deploy/pi-resync/user-data.template`) writes the real topology: a full mesh of
peers across `192.168.1.201/.202/.203` plus an orphan local reference, and
chrony elects one master by comparing reference IDs. Re-apply that step (or
remove `wildsync.conf`) after any `provision`, then verify per §9. This is the
known gap the template documents; it is inert only while nothing on the Mac
answers NTP.

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
launchctl kickstart -k gui/$(id -u)/org.wildtechnology.wildsync.rigd   # macOS host
sudo systemctl restart rigd                                            # Linux host
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
  can *reach* the box — e.g. to strip a trim someone applied by mistake.
  Being addressable is not permission to trim.
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
systemctl is-active ilxctl piagent          # active, active
cat /sys/module/usbcore/parameters/usbfs_memory_mb   # 1000 (see below)
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
| `:8081/health` | `gpio.ok` true, `gpio.focus_held` false, `disk_free_mb` sane, `power.undervolt_now` false. `node` is the **hostname** (`piagent.py` sets `NODE = socket.gethostname()`), so it reads `pi-cam3` while rigd calls the node `cam3` — that is expected, not a mismatch. |
| `:8080/api/status` | `connected` true, `model`, `id` (record the body serial for the fleet table), `slotStatus`, `remainShots`. **`afAllowed` must be `false`** — `true` means the daemon was started with `--allow-autofocus`, which must never happen on a field node. |
| `:9090/api/fleet` | three entries; cam3 present with `cam_num` 3. |

**Clocks.** Every scheduled fire and every `epoch_hw` edge lives on the *node's*
clock, so inter-node clock skew lands 1:1 in true exposure skew while the rig's
own skew figures under-report it. The pair budget is **10 ms**. Free-running
after the Jetson master left the topology, the nodes measured **16.8 ms apart**
(2026-08-20) — outside budget, and invisible in the rig's own numbers. Peered in
orphan mode they sit at 0.6 ms.

```sh
for ip in 192.168.1.201 192.168.1.202 192.168.1.203; do
  echo "== $ip"; ssh ubuntu@$ip 'chronyc tracking | head -3; chronyc sources'
done
```

All three must print the **same Reference ID** — that is the one-line proof the
group has exactly one master. Two distinct refids is a split. After a master
handover the followers slew rather than step, so give it a minute before
trusting the budget. rigd's `node_clock_skew` (not `chronyc`) is the detector
that runs during a survey, and it needs three consecutive scans to raise or
clear.

**Then fire it once.** A node that answers every endpoint and has never pulsed
TRIGGER has proved nothing about the harness. Do a bench fire with the body
attached and confirm an `EXPOSURE` edge comes back (`docs/FIELD-RUN.md`
pre-dive).

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Camera connects, answers property reads, writes to its own card — but PC image transfer dies mid-frame | `usbfs_memory_mb` is back at the 16 MB default. It **must** be set on the **kernel command line** (`usbcore.usbfs_memory_mb=150` in `/boot/firmware/cmdline.txt`), not in `modprobe.d` — `usbcore` is built into the Pi kernel, so `/etc/modprobe.d/usbfs.conf` is never read and the runtime `echo` silently reverted on every reboot (HANDOFF §3.3). Check `/proc/cmdline`; re-run `deploy.sh provision cam3` and **reboot**. Presents as a broken camera. |
| Body reports every property read-only, hands control priority back, ignores the shutter on **both** USB and GPIO | A Low `FOCUS` line — a permanent half-press. **A released GPIO is not high-Z**: the pad keeps the last requester's bias. `piagent` holds the lines open-drain-idle for exactly this reason; check `harness_safe` in `/gpio/state`. Anything that drove the line by hand (a stray `gpioset`, a crashed test) can leave it there. |
| `ilxctl` "active" at 0% CPU, camera unusable; `systemctl restart` hangs | Stuck PTP session from an unclean shutdown. **Recovery recipe: `docs/HANDOFF.md` §2.2** — `pkill -9 -x ilxctl`, USB unbind, USB bind, `systemctl start ilxctl`. Note `ilxctl` now binds `:8080` *before* its startup connect (`src/main.cpp`), so the modern symptom is `/api/status` answering with `connected:false` rather than a dead port — but the PTP session still needs the unbind/bind to clear. `deploy/ilxctl.service` sets `RestartPreventExitStatus=SIGKILL` so a deliberate `-9` stays down long enough for the recipe to work. |
| `node_clock_skew` anomaly | Two nodes disagree beyond the noise floor (warn >5 ms, bad >8 ms), off the RTT-gated median, after 3 consecutive scans. Usually one node lost the peer group — check that all three `chronyc tracking` refids match (§9) and that `provision`'s `wildsync.conf` is not pointing a node at the Mac with `prefer`. A *common-mode* host-vs-node offset is invisible to this check by construction; that one is `host_clock_offset` (turn on network time on the host). |
| `node_clock_unmeasurable` | That node's best `/health` RTT is ≥20 ms, so the 10 ms budget cannot be verified at all. Check the PoE/switch path, not the clock. |
| Unit is `active`, but "no camera found" / `EnumCameraObjects` fails | USB permissions or the SDK adapter path. There is no udev rule for the Sony body in this tree — see the `SupplementaryGroups=`/`WorkingDirectory=` comments in `deploy/ilxctl.service` and settle it against cam1 (`systemctl show -p User -p SupplementaryGroups ilxctl`). |
| Header stays `2/3` | The node is in the fleet but not connected — a node fault, not a config one. Work §9 from the top. |
| Header does not change at all after adding `~/rig/nodes.json` | rigd was not restarted, or the file is unparseable (silently ignored, defaults used), or you edited a checkout rather than `~/rig/nodes.json`. |
| Node unreachable from the host, but healthy on a screen | §4. Wi-Fi is holding the 192.168.1.0/24 route. |

---
---

# Part III — Basler a2A4504-18umBAS

Everything below was **executed and measured on cam3 on 2026-08-28**, not
derived from datasheets. Where a number here disagrees with
`docs/BASLER-SETUP.md`'s estimate, this file is the measurement and that file is
the plan. Design rationale — why this camera, why the strobe moves to cam3's Pi,
what `baslerctl` must implement — stays in `docs/BASLER-SETUP.md` and
`docs/basler-cam3-node.md`; this is the build sheet.

Prerequisite: Part I complete (§1–§5, §7–§8). The Basler does **not** need the
Sony SDK, `ilxctl`, or §6 — but on cam3 both stacks are installed together.

## 11. The camera, and the power question

| | Measured on cam3 |
|---|---|
| Model / serial | `a2A4504-18umBAS` / `40256384` |
| USB id | `2676:ba05` (Basler AG), enumerates as **SuperSpeed** on a USB 3 port |
| Firmware | `p=vu3_imx541m_bas/s=r/v=2.1.0/i=6518.29/h=e035835` |
| Sensor | 4504 × 4504 (max 4512 × 4512), Mono8, global shutter |
| `BslExposureStartDelay` | **111.0 µs** (`docs/BASLER-SETUP.md` predicted ≈112 µs) |
| Free-run rate | `ResultingFrameRate` **17.75 fps**, sustained **17.74 fps** measured |
| Sustained throughput | **~360 MB/s** over USB3, **355/355 frames, 0 failures** in 20 s |
| Camera temp | 34.3 °C idle, 36.8 °C after a 20 s free-run (60 °C housing limit) |

### 11.1 It runs on USB bus power — measured, not assumed

The datasheet figure is ~680 mA typical, and a Pi 5 that has not negotiated the
5 A USB-C contract caps **total** USB draw at 600 mA. A PoE-powered Pi never
negotiates that contract, so on paper this camera does not fit.

**In practice it does.** With `usb_max_current_enable=0` — i.e. the 600 mA cap
in force — cam3 enumerated at SuperSpeed and then sustained 360 MB/s for 20 s
and a 2 fps spool run for 2 minutes with **`vcgencmd get_throttled` reading
`0x0` throughout**: no undervoltage, no over-current, no link reset, no dropped
frames. 680 mA is a typical-draw figure the camera evidently does not reach in
this mode.

So: **do not set `usb_max_current_enable=1` and do not add a powered hub** on
the strength of the datasheet alone. Both are still the fallback if a longer or
hotter run misbehaves, and `usb_max_current_enable=1` is only safe on a supply
that can actually deliver 1.6 A — an open question for the PoE HAT's 5 V rail.
The tell is `get_throttled` going non-zero, or a link reset in `dmesg`.

### 11.2 If nothing enumerates at all

A power-ceiling failure still *tries*: you get an enumeration attempt and then
an over-current or link-reset line in `dmesg`. **Silence is not a power
problem.** On cam3 the first plug-in produced zero USB events, and the cause was
the cable — the a2A4504 uses the wide two-part **USB 3.0 Micro-B** socket, and a
plug can sit in the narrow USB-2 half or merely look seated. Re-seating it fixed
it immediately.

```sh
lsusb -d 2676:                       # the camera, or nothing
lsusb -t | grep 5000M                # must be a 5000M bus
sudo dmesg | grep -iE 'usb|over-?current' | tail
```

`new SuperSpeed USB device` is the line you want. `new high-speed USB device`
means it fell back to USB 2 — a cable or port problem, and the bandwidth budget
in §13 does not hold.

### 11.3 GPIO harness — mirrors the ILX, deliberately

Wired to the **same Pi pins with the same polarity** as the Sony harness (§1.1),
so stock `piagent` drives this camera unmodified. Per
`docs/basler-cam3-node.md`; **not yet wired or verified on cam3.**

| Pi BCM | Dir | Camera M8 pin | Camera line | Meaning |
|---|---|---|---|---|
| 27 | out, open-drain | 4 | Line 2 · input | Pulse LOW ≥1 ms; camera fires on the **falling** edge (`TriggerSource=Line2`, `FallingEdge`) |
| 22 | in, pull-up | 5 | Line 3 · output | `LineSource=ExposureActive` — LOW while exposing. Same polarity as the ILX |
| GND | — | 6 | GPIO ground | Pi ↔ camera **only**. Never into the strobe's 24 V domain |

BCM 17 (FOCUS) connects to nothing — the Basler has no half-press, and `piagent`
driving an unconnected pin is harmless. Line 1 (the opto input) stays empty: the
GPIO path is ~10× faster.

## 12. Install the pylon Suite

`pylon-26.08.1_linux-aarch64_setup.tar` from Basler. The outer `.tar` contains
an inner `.tar.gz` plus Basler's own `INSTALL` — follow that, not a paraphrase.

```sh
scp ~/Downloads/pylon-26.08.1_linux-aarch64_setup.tar ubuntu@192.168.1.203:
ssh ubuntu@192.168.1.203
mkdir -p ~/pylon-stage && cd ~/pylon-stage
tar -xf ~/pylon-26.08.1_linux-aarch64_setup.tar        # -> inner .tar.gz + INSTALL
sudo mkdir -p /opt/pylon
sudo tar -C /opt/pylon -xzf ~/pylon-stage/pylon-26.08.1_linux-aarch64.tar.gz
sudo chmod 755 /opt/pylon                              # ~1.8 GB installed
```

### ⚠ 12.1 Do not run `setup-usb.sh` over ssh

Basler's `setup-usb.sh` installs the USB3 Vision udev rules — and **hangs
forever on a non-TTY ssh**, burning a core. Its `askNoYes` helper is
`while true; do read -p ...`, which spins on EOF instead of exiting, so a piped
or non-interactive session never terminates. It happened twice on cam3 and left
runaway processes both times.

Install the rule directly instead. It is the part that matters, and the script's
other job (persisting `usbfs` via **GRUB**) is a no-op on a Pi, which boots from
`cmdline.txt`:

```sh
sudo cp /opt/pylon/share/pylon/69-basler-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# then unplug and replug the camera
```

The rule is one line — `SUBSYSTEM=="usb", ATTRS{idVendor}=="2676", MODE:="0666"`
— so any user can open the camera; no group membership needed.

Limits pylon wants (open files, and real-time priority for its transfer
threads):

```sh
printf '*  soft nofile 524288\n*  hard nofile 524288\n' | sudo tee /etc/security/limits.d/90-pylon-nofile.conf
printf '*  soft rtprio 99\n*  hard rtprio 99\n'         | sudo tee /etc/security/limits.d/91-pylon-rtprio.conf
```

If you *do* want the script interactively, give it a TTY: `ssh -t`.

### 12.2 usbfs must be 1000 MB — and it is a floor, not a setting

One Mono8 frame is **20.3 MB**, which does not fit the 16 MB default at all, and
does not fit the **150 MB** the Sony path asks for either.

```sh
echo 1000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
sudo sed -i 's/usbcore\.usbfs_memory_mb=[0-9]*/usbcore.usbfs_memory_mb=1000/' /boot/firmware/cmdline.txt
```

1000 satisfies **both** cameras — Sony's 150 is a minimum, and raising it costs
the ILX path nothing. It must live on the **kernel command line**, never
`modprobe.d` (§10, HANDOFF §3.3).

`deploy.sh provision` used to hard-set 150, so re-provisioning a dual-camera node
silently broke Basler transfers while the Sony path looked fine. It now treats
its own value as a **floor** and prints `usbfs: leaving 1000 MB (already above
this script's 150 MB floor)`. If you see it *lower* the value, you are on an old
checkout.

## 13. pypylon, and the encoder

Ubuntu 24.04 is externally managed, so pypylon goes in a venv. pypylon bundles
its own pylon runtime and never touches `/opt/pylon`; the two coexist.

```sh
sudo apt-get install -y python3-venv libturbojpeg0
python3 -m venv ~/pylon-venv
~/pylon-venv/bin/pip install pypylon numpy opencv-python-headless pillow piexif
~/pylon-venv/bin/pip install "PyTurboJPEG<2"      # see below
```

Confirm the USB transport layer is present — `['BaslerUsb', 'BaslerGigE']`:

```sh
~/pylon-venv/bin/python -c "from pypylon import pylon; \
  print([t.GetDeviceClass() for t in pylon.TlFactory.GetInstance().EnumerateTls()])"
```

### 13.1 Three library traps, all hit on cam3

- **`PyTurboJPEG<2` is required.** PyTurboJPEG 2.0 needs libjpeg-turbo ≥ 3.0;
  Ubuntu 24.04 ships 2.x, and 2.0 fails at import with *"requires libjpeg-turbo
  3.0 or later"*. 1.8.3 works.
- **PyTurboJPEG needs an explicit grayscale format.** `encode()` defaults to
  `TJPF_BGR` and wants a 3-D array, so a `(H, W)` Mono8 frame raises
  *"Invalid shape for image data"*. Pass
  `pixel_format=TJPF_GRAY, jpeg_subsample=TJSAMP_GRAY` and reshape to
  `(H, W, 1)`.
- **`piexif.insert(exif, jpeg_bytes)` raises** — *"Give a 3rd argument to
  'insert' to output file"*. The bytes-in/bytes-out form needs an `io.BytesIO`
  sink as the third argument. This one shipped 240 frames with **no EXIF at
  all** on the first Phase 5 run, because it sat behind a bare
  `except: return jpeg_bytes`. See §14.2.

### 13.2 Encode benchmark — measured, replacing the estimate

The Pi 5 **dropped** the hardware JPEG encoder that Pi 0–4 had, so every encode
is CPU. Median of 20 real-scene 4504×4504 Mono8 frames on cam3:

| encoder | q | ms/frame | MB/frame | ratio | MB/s @ 2 fps |
|---|---|---|---|---|---|
| `cv2.imencode` | 85 | 57.8 | 1.073 | 18.9× | 2.15 |
| **`cv2.imencode`** | **90** | **62.0** | **1.572** | 12.9× | **3.14** |
| `cv2.imencode` | 95 | 71.3 | 2.887 | 7.0× | 5.77 |
| PyTurboJPEG | 85 | 66.7 | 1.078 | 18.8× | 2.16 |
| PyTurboJPEG | 90 | 69.6 | 1.588 | 12.8× | 3.18 |
| PyTurboJPEG | 95 | 79.8 | 3.000 | 6.8× | 6.00 |
| *pure noise (ceiling)* | *90* | *155.7* | *15.955* | *1.3×* | *31.91* |

Two results worth carrying forward:

- **`cv2.imencode` beats PyTurboJPEG** by ~13 % at identical file sizes.
  `docs/BASLER-SETUP.md` §5.2 assumed the opposite. OpenCV 5.0 bundles a newer
  libjpeg-turbo than the system 2.x PyTurboJPEG binds against. **Use cv2**;
  PyTurboJPEG stays installed as a cross-check.
- **58–80 ms/frame, not the 150–400 ms envelope** that doc had to guess. At
  2 fps that is ~12 % of *one* core, so the 2-worker pool has margin rather than
  being the bottleneck.

**Chosen: q90.** 1.57 MB/frame, 12.9×, 3.14 MB/s at 2 fps.

The *pure noise* row is a hard entropy ceiling — no real photograph compresses
worse — so spool math is bounded between 1.6 and 16 MB/frame. **Benchmark on a
real scene**: file size is entropy-driven, and cam3's first attempt ran against
a nearly-black frame (mean 8.0, 96 % of pixels in the darkest histogram bin)
whose sizes were meaninglessly small. Open the iris, then:

```sh
# auto-exposure converged on cam3 at 66973 us / 0.0 dB -> mean 89.5, std 77.3
ExposureAuto = Continuous ; GainAuto = Continuous ; AutoTargetBrightness = 0.35
```

67 ms of exposure is fine at 2 fps but would motion-blur a moving platform —
the strobe path is what buys a short exposure.

## 14. The spool — `rig/basler_spool.py`

`rig/basler_spool.py` is the Phase 5 rehearsal of what `baslerctl` will do:
grab → JPEG encode (2-worker pool) → EXIF → atomic write. Ship it with the rest
of `rig/` and run it in the venv.

```sh
scp rig/basler_spool.py ubuntu@192.168.1.203:/home/ubuntu/wildsync/rig/
ssh ubuntu@192.168.1.203 \
  '~/pylon-venv/bin/python ~/wildsync/rig/basler_spool.py --seconds 120 --fps 2 --quality 90'
```

Three things in it are **contracts with the rest of the rig**, not preferences:

- **Naming** — `CamN_YYYYMMDD_hhmmss.ss.jpg`, byte-identical to
  `run.py:_fmt_fname`. `ingest.py` keys attribution off `cam%d` and the stem, so
  a divergent name silently unpairs frames.
- **Atomic write** — `.part` → `fsync` → `rename` → `fsync(dir)`, identical to
  `run.py`'s pull path, for the reasons in §10 and the audit of 2026-08-27.
- **JPEG before the card** — raw is 20.3 MB; at 2 fps that would be 41 MB/s onto
  the SD card and ~146 GB per hour of host outage.

### 14.1 Measured result (2 min at 2 fps, q90)

| | idle | under `stress-ng --cpu 2` |
|---|---|---|
| Grabbed / written | 240 / 240 @ 1.99 fps | **240 / 240 @ 1.99 fps** |
| Grab failures / drops | 0 / 0 | **0 / 0** |
| Per frame | 1.532 MB (3.05 MB/s) | 1.474 MB (2.94 MB/s) |
| Encode ms p50 / p95 / max | 67.5 / 70.8 / 100.9 | 68.0 / 70.0 / 72.1 |
| Write ms p50 / p95 / max | 40.8 / 56.9 / 124.7 | 42.4 / **202.9 / 1060.8** |
| Encode queue depth | max 1 of 6 | max 1 of 6 |
| Stray `.part` | 0 | 0 |
| SoC temp | 50.5 °C | 59.8 °C |

Encode barely notices two competing CPU hogs. **SD write latency does** — p95
went 57 → 203 ms and the worst single write took **1.06 s**, two full periods at
2 fps. The 2-worker pool absorbed it (the queue never exceeded 1 of 6), but that
is the number to watch on a 30-minute run, and the reason the queue is bounded:
it converts a slow card into a *countable drop* instead of RAM growth and an
OOM kill.

> **OOM is a real failure mode here.** The first benchmark attempt held 20 raw
> frames (20 × 20.3 MB) live and was OOM-killed three times on an 8 GB Pi.
> Stream frames; never accumulate them.

### 14.2 EXIF, and a lesson about silent fallbacks

`cv2` writes **no EXIF whatsoever**. `rigd`'s EXIF fallback is what stamps a
frame's capture instant when no GPIO edge is available, so an unstamped frame
silently inherits the command time instead.

The first Phase 5 run reported a clean **PASS** while writing 240 frames with no
EXIF at all, because the `piexif` call in §13.1 raised and a bare
`except: return jpeg_bytes` swallowed it. The fallback turned a hard requirement
failure into a passing run. `basler_spool.py` now **counts** EXIF failures and
fails the run on any of them. Verify on the pulled frames, not on the node's
own say-so.

## 15. Verify the whole path — camera to Mac

Do these in order; each gates the next. The first four need no camera.

| Phase | Proves | cam3 result, 2026-08-28 |
|---|---|---|
| **0** emulator | venv + scripts before hardware. `PYLON_CAMEMU=2` | **PASS** — 2 devices, open, grab, numpy `(1040,1024)` Mono8 |
| **1** bring-up | Enumeration, single grab, power margin | **PASS** — full-res 20.3 MB in 61 ms, `throttled=0x0` |
| **2** free-run | Bandwidth + thermals | **PASS** — 17.74 fps, 355/355, ~360 MB/s, 36.8 °C |
| **5** compress + spool | Production behaviour | **PASS** — §14.1 |
| **cam → Mac** | The delivery path end to end | **PASS** — below |

```sh
# on the node: manifest before pulling
ssh ubuntu@192.168.1.203 'cd ~/Pictures/Basler && md5sum *.jpg | sort > /tmp/node.md5'

# pull (openrsync on macOS has no --info=progress2; plain -a)
rsync -a ubuntu@192.168.1.203:/home/ubuntu/Pictures/Basler/ ~/rig-raw/cam3-phase5/

# compare
scp ubuntu@192.168.1.203:/tmp/node.md5 /tmp/node.md5
cd ~/rig-raw/cam3-phase5 && md5 -r *.jpg | awk '{print $1"  "$2}' | sort > /tmp/mac.md5
diff /tmp/node.md5 /tmp/mac.md5 && echo "byte-identical"
```

Measured: **240 frames, 351 MB, pulled in 3.5 s (~100 MB/s), all byte-identical.**
Then verify the frames themselves rather than trusting the transfer:

- 240/240 decode, 0 corrupt, all 4504 × 4504
- sizes 1.39 – 1.89 MB, median 1.48 MB
- EXIF present on every frame — `DateTimeOriginal`, `SubSecTimeOriginal`,
  `Make=Basler`, `Model=a2A4504-18umBAS`
- **every frame's EXIF timestamp matches its own filename** — the check that
  catches a naming/stamping drift no checksum can see

### 15.1 Basler troubleshooting

| Symptom | Cause / fix |
|---|---|
| Nothing in `lsusb`, **no `dmesg` USB lines at all** | Not a power fault — a power fault still *tries*. Cable or port. Re-seat the USB 3.0 Micro-B firmly (§11.2); confirm a `5000M` bus. |
| `new high-speed USB device` instead of `SuperSpeed` | Fell back to USB 2. Wrong port or a USB-2 cable; the §13 bandwidth budget no longer holds. |
| `setup-usb.sh` never returns, one core pinned | Non-TTY ssh + its `while true; do read` loop (§12.1). Kill it, install the `.rules` file directly. |
| `PyTurboJPEG ... requires libjpeg-turbo 3.0 or later` | Ubuntu 24.04 ships 2.x. `pip install "PyTurboJPEG<2"`. |
| `Invalid shape for image data` from `encode()` | Grayscale needs `TJPF_GRAY` + a `(H,W,1)` array (§13.1). |
| `Give a 3rd argument to 'insert' to output file` | `piexif` bytes-in/bytes-out needs an `io.BytesIO` sink (§13.1, §14.2). |
| `PermissionError: '/home/ubuntu/Pictures/Basler'` | `/home/ubuntu/Pictures` is root-owned: `install -d -o ubuntu … Pictures/ILX-LR1` sets the **leaf** owner but creates parents as root. `sudo chown -R ubuntu:ubuntu ~/Pictures`. |
| Process killed, `dmesg` shows OOM | Raw frames accumulated. 20.3 MB each; stream, and keep the encode queue bounded (§14.1). |
| Frames land but carry no EXIF | §14.2 — and check the run's `EXIF failures` line rather than assuming. |
| Transfers stall or truncate mid-frame | `usbfs_memory_mb` below 1000 (§12.2). Check `/proc/cmdline`, not just the sysfs value. |

## 16. What is not done yet

Honest state of the Basler path as of 2026-08-28:

- **Phases 3, 4 and 6 have not run.** Software trigger, hardware trigger, and the
  Line 3 Timer strobe option are untested. The §11.3 harness is **not wired**.
- **`baslerctl` does not exist.** cam3 currently runs `ilxctl`, which serves
  `:8080` for a Sony body that is not attached. Until `baslerctl` implements the
  ilxctl-shaped contract, cam3 cannot join a run — a node failing that shape
  sits in `ILX_DOWN` forever. `rig/basler_spool.py` is the rehearsal of its save
  path only.
- **`~/rig/nodes.json` has not been created**, so rigd still sees a two-camera
  fleet. Adding cam3 now would only park a camera in `ILX_DOWN`. Do it when
  `baslerctl` answers (§7).
- **The `Statistic_*` counters** named in `docs/BASLER-SETUP.md` Phase 2 do not
  resolve on the USB transport layer — those are GigE-side names. The 355/355
  and 240/240 figures above are direct frame counts, not a read of those
  counters. Find the USB equivalents before calling Phase 2 formally closed.
- **Long-run behaviour is unmeasured.** The longest run so far is 2 minutes. The
  30-minute run in `docs/BASLER-SETUP.md` step 13, and the SD write latency in
  §14.1, are the things most likely to bite.
- **Optics are unresolved** — lens choice, and whether cam3 feeds photogrammetry
  or strobe photometry, are still open in `docs/BASLER-SETUP.md`.

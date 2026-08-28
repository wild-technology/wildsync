#!/bin/bash
# deploy.sh — push the rig software to a camera node and (optionally) the Jetson.
#
#   ./deploy.sh provision cam2   first-time setup of a bare node (SDK + build)
#   ./deploy.sh node cam1        update sources, rebuild ilxctl, restart services
#   ./deploy.sh jetson           install rigd on this Jetson
#   ./deploy.sh all              every reachable camera node + the Jetson
#
# ilxctl is ALWAYS built on the target. Nodes and the Jetson are all aarch64 but
# different microarchitectures, and the Makefile probes -mcpu=native, so a binary
# built on one SIGILLs on the other. Never scp the binary between machines.

set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# The fleet, in deploy order. A space-separated string, not an array and not
# `declare -A`: the rigd host is a Mac now, and macOS ships bash 3.2, which has
# neither (the old associative form died with "cam1: unbound variable" under
# set -u). `all` iterates this, so a new node is one edit here plus one line in
# _ip() - previously `all` carried its own literal "cam1 cam2" pair and adding
# cam3 to _ip() alone would have left it silently out of every `all` run.
NODES="cam1 cam2 cam3"

# Sony's USB bulk-transfer buffer. The Linux default of 16 MB drops the session
# mid-frame on large transfers; Sony's own guidance is 150 MB.
USBFS_MB=150

_ip() {
  local ip=""
  case "$1" in
    cam1) ip=192.168.1.201;;
    cam2) ip=192.168.1.202;;
    # cam3: third survey camera, Pi 5, pi-cam3. Deliberately NOT added to
    # rigcore.py's _DEFAULT_NODES - a code-side empty slot fired a permanent
    # node_offline anomaly and a forever "2/3" header. A third camera joins the
    # rig's own view via ~/rig/nodes.json on the host; this table is only how
    # deploy reaches the box over ssh.
    cam3) ip=192.168.1.203;;
  esac
  [ -n "$ip" ] || { echo "unknown node '$1'" >&2; return 1; }
  echo "$ip"
}

# Everything a node needs that `node` does not ship: the SDK headers/libs, the
# Makefile, and the third-party header. Only needed once per machine.
provision_node() {
  local name="$1" ip
  ip="$(_ip "$name")" || return 1
  echo "=== provision $name ($ip) ==="
  ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  unreachable, skip"; return; }
  # The Pis need the LINUX aarch64 SDK (.so). This checkout's lib/ holds
  # whatever platform the HOST builds with - on the Mac that is dylibs, and
  # rsyncing those bricked the node build with a missing libCr_Core.so while
  # looking like a successful provision (audit 2026-08-27).
  #
  # lib-linux/ is the way out of that on a Mac host, and since the Jetson was
  # retired the Mac is the only host there is - so `make sdk` (which stages for
  # THIS platform) could never provision a node, and the refusal below had no
  # remedy on the machine actually running it. `make sdk-linux` fills
  # lib-linux/ from the Linux64ARMv8 package, leaving lib/ alone so ilxctl
  # still builds here. Prefer it when present; fall back to lib/ so a genuinely
  # Linux host (a node, or the old Jetson) keeps working unchanged.
  local sdklib=""
  if [ -f "$REPO"/lib-linux/libCr_Core.so ]; then
    sdklib="$REPO/lib-linux"
  elif [ -f "$REPO"/lib/libCr_Core.so ]; then
    sdklib="$REPO/lib"
  else
    echo "  REFUSING provision: no Linux aarch64 libCr_Core.so to send." >&2
    echo "  Looked in $REPO/lib-linux and $REPO/lib." >&2
    echo "  Sony ships PLATFORM-SPECIFIC packages and the Mac one will not do:" >&2
    echo "    make sdk-linux CRSDK_DIR=/path/to/CrSDK_..._Linux64ARMv8" >&2
    echo "  (see docs/PI-SETUP.md), or provision from a Linux host." >&2
    return 1
  fi
  echo "  SDK source: $sdklib"
  ssh ubuntu@"$ip" 'mkdir -p ~/wildsync/{src,rig,include,lib,third_party}'
  rsync -az --delete "$REPO"/include/     ubuntu@"$ip":/home/ubuntu/wildsync/include/
  rsync -az --delete "$sdklib"/           ubuntu@"$ip":/home/ubuntu/wildsync/lib/
  rsync -az --delete "$REPO"/third_party/ ubuntu@"$ip":/home/ubuntu/wildsync/third_party/
  scp -q "$REPO"/Makefile ubuntu@"$ip":/home/ubuntu/wildsync/Makefile
  ssh ubuntu@"$ip" "bash -s" <<REMOTE
set -uo pipefail
# USBFS_MB is a FLOOR, never an assignment. cam3 also carries a Basler
# a2A4504 (20.2 MP), whose single 20.3 MB frame needs ~1000 MB of URB space,
# and this step used to hard-set 150 - so re-provisioning a node silently
# broke Basler transfers while the Sony path looked fine. Raise, never lower.
cur=\$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || echo 0)
case "\$cur" in ''|*[!0-9]*) cur=0;; esac
# ...and the PERSISTED value, which can legitimately be higher than the running
# one: a node whose cmdline.txt already says 1000 but has not rebooted yet is
# still running 150, so a runtime-only floor would quietly write 1000 back down
# to 150 and undo the Basler setting on the next boot.
pers=\$(sed -n 's/.*usbcore\.usbfs_memory_mb=\([0-9]*\).*/\1/p' \
        /boot/firmware/cmdline.txt 2>/dev/null | head -1)
case "\$pers" in ''|*[!0-9]*) pers=0;; esac
[ "\$pers" -gt "\$cur" ] && cur=\$pers
if [ "\$cur" -gt $USBFS_MB ]; then
  echo "  usbfs: leaving \$cur MB (already above this script's $USBFS_MB MB floor)"
  USBFS_EFFECTIVE=\$cur
else
  USBFS_EFFECTIVE=$USBFS_MB
fi
echo \$USBFS_EFFECTIVE | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null
# Persist it on the KERNEL COMMAND LINE, not in modprobe.d. usbcore is built
# into the Pi kernel rather than loaded as a module, so /etc/modprobe.d/usbfs.conf
# is never read and the setting silently reverted to the 16 MB default on every
# reboot - taking PC image transfer with it, while the camera still connects,
# still answers property reads and still writes to its own card. That failure
# looks exactly like a broken camera and cost a session to find.
echo "options usbcore usbfs_memory_mb=\$USBFS_EFFECTIVE" | \
  sudo tee /etc/modprobe.d/usbfs.conf >/dev/null
CMDLINE=/boot/firmware/cmdline.txt
if [ -f "\$CMDLINE" ]; then
  if ! grep -q "usbcore.usbfs_memory_mb=" "\$CMDLINE"; then
    sudo sed -i "s/\$/ usbcore.usbfs_memory_mb=\$USBFS_EFFECTIVE/" "\$CMDLINE"
    echo "  added usbcore.usbfs_memory_mb=\$USBFS_EFFECTIVE to \$CMDLINE (next boot)"
  else
    sudo sed -i "s/usbcore.usbfs_memory_mb=[0-9]*/usbcore.usbfs_memory_mb=\$USBFS_EFFECTIVE/" "\$CMDLINE"
  fi
else
  echo "  WARNING: \$CMDLINE not found - usbfs setting will NOT survive a reboot"
fi
# THE CLOCK MUST BE RIGHT BEFORE apt RUNS. A freshly imaged Pi has no RTC
# battery, so 'fixrtc' seeds the clock from the image build date - cam3 booted
# on 2026-08-28 believing it was 2026-02-10. apt then rejects every repo with
# "Release file is not valid yet (invalid for another 198d)", falls back to the
# stale package lists baked into the image, and every download 404s because
# those versions have been superseded. Observed on cam3's first boot: exit 100,
# and NOTHING in build-essential/chrony/libgpiod installed.
sudo timedatectl set-ntp true >/dev/null 2>&1 || true
for _ in 1 2 3 4 5 6 7 8 9 10 11 12; do
  [ "\$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = yes ] && break
  sleep 5
done
sudo DEBIAN_FRONTEND=noninteractive apt-get update -q >/dev/null 2>&1 || true

# One master clock for the rig. Public NTP costs milliseconds of jitter and,
# worse, disciplines a node to a different reference than its stereo partner -
# that difference lands straight in the inter-camera skew.
#
# ORDER IS LOAD-BEARING, and it used to be wrong: this disabled timesyncd
# BEFORE checking that chrony had installed, behind '|| true' on every step. A
# node whose chrony install failed was left with NO time discipline at all
# while provision printed success - which is precisely the state cam3 was found
# in. Fail loud instead, and never surrender the working clock until the
# replacement is actually running.
# The FULL node dependency set, not just chrony. Nothing else in this repo
# ever installed these: Ubuntu Server ships no compiler, so 'make' on the node
# (the next step, and the only way ilxctl can be built - the Makefile probes
# -mcpu=native) fails on a fresh node; piagent needs libgpiod; the Sony SDK
# needs libusb/libxml2; and provision itself rsyncs, which needs rsync ON the
# node. All of these had to be installed by hand to bring cam3 up on
# 2026-08-28, which is the definition of a provision step that does not
# provision. python3-libgpiod is v1.6.3 on 24.04, the version PROTOCOL.md pins.
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
  build-essential rsync chrony python3-libgpiod gpiod libusb-1.0-0 libxml2 \
  >/dev/null 2>&1 || true
for pkg in build-essential rsync python3-libgpiod gpiod; do
  dpkg-query -W -f='${Status}' "\$pkg" 2>/dev/null | grep -q "ok installed" || \
    echo "  WARNING: \$pkg did NOT install - the node build will fail" >&2
done
if ! dpkg-query -W -f='${Status}' chrony 2>/dev/null | grep -q "ok installed"; then
  echo "  ERROR: chrony did NOT install - leaving systemd-timesyncd running." >&2
  echo "  The node keeps a clock, but not the rig's. Check the date (\$(date -Is))" >&2
  echo "  and this node's route to the archive, then re-run provision." >&2
else
  # NO chrony conf is written here. The node's clock topology is the peer mesh
  # laid down on its card (/etc/chrony/conf.d/10-wildsync.conf,
  # deploy/pi-resync/user-data.template). This step used to write
  # 'server <ssh client IP> iburst prefer' - i.e. the Mac, PREFERRED - and the
  # Mac is deliberately not a chrony master. Inert only while nothing on the
  # Mac answers NTP; the moment something does, every node abandons the peer
  # group for the undisciplined host clock, as a common-mode offset that
  # rigd's node_clock_skew cannot see by construction.
  sudo rm -f /etc/chrony/conf.d/wildsync.conf
  if [ ! -f /etc/chrony/conf.d/10-wildsync.conf ]; then
    echo "  WARNING: no 10-wildsync.conf on this node - it has no rig peer mesh." >&2
    echo "  Write its card with deploy/pi-resync/install.sh, or copy that file." >&2
  fi
  sudo systemctl enable --now chrony >/dev/null 2>&1 || true
  # Only NOW is the replacement actually running - check before surrendering
  # the working clock, rather than trusting that enable succeeded.
  if [ "\$(systemctl is-active chrony)" = active ]; then
    sudo systemctl disable --now systemd-timesyncd >/dev/null 2>&1 || true
  else
    echo "  WARNING: chrony installed but not active - leaving timesyncd up" >&2
  fi
fi
echo "  usbfs=\$(cat /sys/module/usbcore/parameters/usbfs_memory_mb)  chrony=\$(systemctl is-active chrony)  date=\$(date -Is)"
REMOTE
  echo "  provisioned $name — now run: $0 node $name"
}

deploy_node() {
  local name="$1" ip
  ip="$(_ip "$name")" || return 1
  echo "=== $name ($ip) ==="
  ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  unreachable, skip"; return; }
  # Create both dirs BEFORE any scp: without src/ the copy fails, and since this
  # script is not `set -e` the build would silently proceed against stale sources.
  ssh ubuntu@"$ip" 'mkdir -p ~/wildsync/src ~/wildsync/rig'
  scp -q "$REPO"/src/*.cpp "$REPO"/src/*.h ubuntu@"$ip":/home/ubuntu/wildsync/src/
  scp -q "$REPO"/rig/{piagent.py,imu_yb.py,imu_olive.py,olive_ws_bridge.py} ubuntu@"$ip":/home/ubuntu/wildsync/rig/
  scp -q "$REPO"/docs/PROTOCOL.md ubuntu@"$ip":/home/ubuntu/wildsync/rig/
  scp -q "$REPO"/deploy/piagent.service ubuntu@"$ip":/tmp/piagent.service
  scp -q "$REPO"/deploy/olive-bridge.service ubuntu@"$ip":/tmp/olive-bridge.service
  scp -q "$REPO"/deploy/ilxctl.service ubuntu@"$ip":/tmp/ilxctl.service
  ssh ubuntu@"$ip" 'bash -s' <<'REMOTE'
set -uo pipefail
cd ~/wildsync
if [ ! -f lib/libCr_Core.so ]; then
  echo "  SDK not staged on this node — run: deploy.sh provision <node>"; exit 1
fi
if ! make 2>&1 | tail -20; then
  echo "  BUILD FAILED on this node - not touching the running services." >&2
  exit 1
fi
# udev rule so the gpio group owns the chips (removes the sudo requirement)
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-gpio.rules >/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=gpio || true
sudo cp /tmp/piagent.service /etc/systemd/system/piagent.service
sudo cp /tmp/olive-bridge.service /etc/systemd/system/olive-bridge.service
# ilxctl.service ships from the repo as of cam3. Until then this step was a
# bare `systemctl restart ilxctl || echo "not installed here"` and cam1/cam2
# ran HAND-INSTALLED units nothing in the tree had ever captured - a new node
# had no way to come up at all. Overwriting is the point of a deploy, but it
# does mean the first run against cam1/cam2 replaces their hand-rolled unit
# with this one: `systemctl cat ilxctl` on those nodes BEFORE deploying, and
# reconcile any difference deliberately (see the header of deploy/ilxctl.service).
sudo cp /tmp/ilxctl.service /etc/systemd/system/ilxctl.service
# StartLimitBurst=5 with RestartSec=5 means a fast-exiting ilxctl burns its
# five starts in ~25 s and systemd then REFUSES manual starts with
# "start-limit-hit" - which breaks this deploy and, worse, the last step of
# HANDOFF 2.2's un-wedge recipe. Clear the counter before every restart.
sudo systemctl reset-failed ilxctl 2>/dev/null || true
# Uniform fleet config: the imu2 slot listens on localhost UDP everywhere;
# on a node with no Olive the bridge idles and the slot reports absent.
echo 'PIAGENT_IMU2=olive:udp:9901' | sudo tee /etc/default/piagent >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable olive-bridge >/dev/null 2>&1
sudo systemctl restart olive-bridge
sudo systemctl enable piagent
sudo systemctl restart piagent   # enable --now is a no-op on a running unit: the new file was never loaded
sudo systemctl enable ilxctl >/dev/null 2>&1
# Against a WEDGED daemon this now waits out TimeoutStopSec=45 instead of
# hanging for ever (HANDOFF 2.2). It is slow, not stuck - do not Ctrl-C it.
sudo systemctl restart ilxctl
sleep 4
echo "  ilxctl: $(systemctl is-active ilxctl)   piagent: $(systemctl is-active piagent)"
REMOTE
  echo "  done $name"
}

deploy_jetson() {
  echo "=== jetson (local) ==="
  # Host dependencies. A fresh JetPack image has none of these, and the old
  # path installed nothing: nav then died on import serial, EXIF reads
  # silently returned None (Pillow), and the host free-ran with no time
  # discipline while being the Pis' chrony master - the exact failure the
  # field guide warns about on the Mac (audit 2026-08-27).
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
    python3-serial python3-pil python3-numpy python3-opencv chrony \
    >/dev/null 2>&1 || echo "  WARNING: apt install failed - check network"
  # The host is the rig's clock master: serve the LAN, keep a local stratum
  # so the rig stays coherent with no upstream at sea, and let GPS-derived
  # corrections come from rigd's own timesync layer.
  printf 'allow 192.168.1.0/24\nlocal stratum 10\n' | \
    sudo tee /etc/chrony/conf.d/wildsync-host.conf >/dev/null
  sudo systemctl enable --now chrony >/dev/null 2>&1 || true
  sudo systemctl restart chrony >/dev/null 2>&1 || true
  # Render the unit for THIS checkout and user - the template carries
  # placeholders, never a hardcoded machine.
  sed "s|__REPO__|$REPO|g; s|__USER__|$(id -un)|g" \
    "$REPO"/deploy/rigd.service | sudo tee /etc/systemd/system/rigd.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable rigd >/dev/null 2>&1
  # enable --now is a NO-OP on an already-running unit: redeploys silently
  # kept the old code. Restart is the deploy.
  sudo systemctl restart rigd
  sleep 3
  echo "  rigd: $(systemctl is-active rigd)  -> http://$(hostname -I | awk '{print $1}'):9090"
  echo "  if the nodes are unreachable over ssh from this host, run '$0 trust <camN>' for each of: $NODES"
}

# First contact from a NEW host (the Jetson): the Pis only trust keys that
# were pushed to them, and every node/provision/power script assumes
# passwordless ssh. Run once per node; asks for the Pi's password.
trust_node() {
  local name="$1" ip
  ip="$(_ip "$name")" || return 1
  echo "=== trust $name ($ip) - pushing this host's ssh key ==="
  [ -f ~/.ssh/id_ed25519.pub ] || [ -f ~/.ssh/id_rsa.pub ] || \
    ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
  ssh-copy-id -o StrictHostKeyChecking=accept-new ubuntu@"$ip"
}

case "${1:-}" in
  provision) provision_node "${2:?usage: deploy.sh provision <camN>}";;
  trust)     trust_node "${2:?usage: deploy.sh trust <camN>}";;
  node)      deploy_node "${2:?usage: deploy.sh node <camN>}";;
  jetson)    deploy_jetson;;
  all)       for n in $NODES; do deploy_node "$n"; done; deploy_jetson;;
  *) echo "usage: $0 {provision <camN>|trust <camN>|node <camN>|jetson|all}"; exit 1;;
esac

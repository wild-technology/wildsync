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
# Node addresses. A plain case, not an associative array: the rigd host is a
# Mac now, and macOS ships bash 3.2, which has no `declare -A` (the old form
# died with "cam1: unbound variable" under set -u).

# Sony's USB bulk-transfer buffer. The Linux default of 16 MB drops the session
# mid-frame on large transfers; Sony's own guidance is 150 MB.
USBFS_MB=150

_ip() {
  local ip=""
  case "$1" in
    cam1) ip=192.168.1.201;;
    cam2) ip=192.168.1.202;;
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
  if [ ! -f "$REPO"/lib/libCr_Core.so ]; then
    echo "  REFUSING provision: $REPO/lib has no libCr_Core.so (Linux SDK)." >&2
    echo "  Stage Sony's Linux aarch64 CameraRemote SDK libs in lib/ first" >&2
    echo "  (see README 'make sdk'), or provision from a Linux host." >&2
    return 1
  fi
  ssh ubuntu@"$ip" 'mkdir -p ~/wildsync/{src,rig,include,lib,third_party}'
  rsync -az --delete "$REPO"/include/     ubuntu@"$ip":/home/ubuntu/wildsync/include/
  rsync -az --delete "$REPO"/lib/         ubuntu@"$ip":/home/ubuntu/wildsync/lib/
  rsync -az --delete "$REPO"/third_party/ ubuntu@"$ip":/home/ubuntu/wildsync/third_party/
  scp -q "$REPO"/Makefile ubuntu@"$ip":/home/ubuntu/wildsync/Makefile
  ssh ubuntu@"$ip" "bash -s" <<REMOTE
set -uo pipefail
echo $USBFS_MB | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null
# Persist it on the KERNEL COMMAND LINE, not in modprobe.d. usbcore is built
# into the Pi kernel rather than loaded as a module, so /etc/modprobe.d/usbfs.conf
# is never read and the setting silently reverted to the 16 MB default on every
# reboot - taking PC image transfer with it, while the camera still connects,
# still answers property reads and still writes to its own card. That failure
# looks exactly like a broken camera and cost a session to find.
echo "options usbcore usbfs_memory_mb=$USBFS_MB" | \
  sudo tee /etc/modprobe.d/usbfs.conf >/dev/null
CMDLINE=/boot/firmware/cmdline.txt
if [ -f "\$CMDLINE" ]; then
  if ! grep -q "usbcore.usbfs_memory_mb=" "\$CMDLINE"; then
    sudo sed -i "s/\$/ usbcore.usbfs_memory_mb=$USBFS_MB/" "\$CMDLINE"
    echo "  added usbcore.usbfs_memory_mb=$USBFS_MB to \$CMDLINE (takes effect next boot)"
  else
    sudo sed -i "s/usbcore.usbfs_memory_mb=[0-9]*/usbcore.usbfs_memory_mb=$USBFS_MB/" "\$CMDLINE"
  fi
else
  echo "  WARNING: \$CMDLINE not found - usbfs setting will NOT survive a reboot"
fi
# One master clock for the rig. Public NTP costs milliseconds of jitter and,
# worse, disciplines a node to a different reference than its stereo partner -
# that difference lands straight in the inter-camera skew.
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q chrony >/dev/null 2>&1 || true
printf 'server %s iburst prefer minpoll 4 maxpoll 6\n' "\$(echo \$SSH_CONNECTION | awk '{print \$1}')" \
  | sudo tee /etc/chrony/conf.d/wildsync.conf >/dev/null
sudo systemctl disable --now systemd-timesyncd >/dev/null 2>&1 || true
sudo systemctl enable --now chrony >/dev/null 2>&1 || true
echo "  usbfs=\$(cat /sys/module/usbcore/parameters/usbfs_memory_mb)  chrony=\$(systemctl is-active chrony)"
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
  scp -q "$REPO"/rig/{piagent.py,imu_yb.py} ubuntu@"$ip":/home/ubuntu/wildsync/rig/
  scp -q "$REPO"/docs/PROTOCOL.md ubuntu@"$ip":/home/ubuntu/wildsync/rig/
  scp -q "$REPO"/deploy/piagent.service ubuntu@"$ip":/tmp/piagent.service
  ssh ubuntu@"$ip" 'bash -s' <<'REMOTE'
set -uo pipefail
cd ~/wildsync
if [ ! -f lib/libCr_Core.so ]; then
  echo "  SDK not staged on this node — run: deploy.sh provision <node>"; exit 1
fi
make 2>&1 | tail -1
# udev rule so the gpio group owns the chips (removes the sudo requirement)
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-gpio.rules >/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=gpio || true
sudo cp /tmp/piagent.service /etc/systemd/system/piagent.service
sudo systemctl daemon-reload
sudo systemctl enable piagent
sudo systemctl restart piagent   # enable --now is a no-op on a running unit: the new file was never loaded
sudo systemctl restart ilxctl 2>/dev/null || \
  echo "  note: ilxctl.service not installed here — see docs/HANDOFF.md"
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
  echo "  if the nodes are unreachable over ssh from this host, run: $0 trust cam1 && $0 trust cam2"
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
  all)       for n in cam1 cam2; do deploy_node "$n"; done; deploy_jetson;;
  *) echo "usage: $0 {provision <camN>|trust <camN>|node <camN>|jetson|all}"; exit 1;;
esac

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
declare -A NODE_IP=( [cam1]=192.168.1.201 [cam2]=192.168.1.202 [cam3]=192.168.1.203 )

# Sony's USB bulk-transfer buffer. The Linux default of 16 MB drops the session
# mid-frame on large transfers; Sony's own guidance is 150 MB.
USBFS_MB=150

_ip() {
  local ip="${NODE_IP[$1]:-}"
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
sudo systemctl enable --now piagent
sudo systemctl restart ilxctl 2>/dev/null || \
  echo "  note: ilxctl.service not installed here — see docs/HANDOFF.md"
sleep 4
echo "  ilxctl: $(systemctl is-active ilxctl)   piagent: $(systemctl is-active piagent)"
REMOTE
  echo "  done $name"
}

deploy_jetson() {
  echo "=== jetson (local) ==="
  sudo cp "$REPO"/deploy/rigd.service /etc/systemd/system/rigd.service
  sudo systemctl daemon-reload
  sudo systemctl enable --now rigd
  sleep 3
  echo "  rigd: $(systemctl is-active rigd)  -> http://$(hostname -I | awk '{print $1}'):9090"
}

case "${1:-}" in
  provision) provision_node "${2:?usage: deploy.sh provision <camN>}";;
  node)      deploy_node "${2:?usage: deploy.sh node <camN>}";;
  jetson)    deploy_jetson;;
  all)       for n in cam1 cam2 cam3; do deploy_node "$n"; done; deploy_jetson;;
  *) echo "usage: $0 {provision <camN>|node <camN>|jetson|all}"; exit 1;;
esac

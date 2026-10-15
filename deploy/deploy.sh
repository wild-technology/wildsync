#!/bin/bash
# deploy.sh — push the rig software to a camera node and (optionally) the Jetson.
#
#   ./deploy.sh provision cam2   first-time setup of a bare node (SDK + build)
#   ./deploy.sh node cam1        update sources, rebuild ilxctl, restart services
#   ./deploy.sh clock cam1       only the node's chrony: GNSS Compass + peer mesh
#   ./deploy.sh mac              this Mac as the host: its clock on the Compass,
#                                the rigd launchd agent (docs/INSTALL.md 3a)
#   ./deploy.sh jetson           install rigd on this Jetson (also a chrony master)
#   ./deploy.sh strobe           install the XR256 strobe agent on this Jetson
#   ./deploy.sh trust cam1       push this host's ssh key to a node (once per node)
#   ./deploy.sh all              this host (mac or jetson), then every reachable
#                                camera node
#
# ilxctl is ALWAYS built on the target. Nodes and the Jetson are all aarch64 but
# different microarchitectures, and the Makefile probes -mcpu=native, so a binary
# built on one SIGILLs on the other. Never scp the binary between machines.

set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# The fleet, in deploy order. A space-separated string, not an array and not
# `declare -A`: this script must also run from a macOS host, whose bash 3.2 has
# neither (the old associative form died with "cam1: unbound variable" under
# set -u). `all` iterates this, so a new node is one edit here plus one line in
# _ip() - previously `all` carried its own literal "cam1 cam2" pair and adding
# cam3 to _ip() alone would have left it silently out of every `all` run.
NODES="cam1 cam2 cam3"

# The rig's time reference: the Advanced Navigation GNSS Compass on the rig
# switch, which is a stratum-1 NTP server on GPS time (refid "GPS", 0.5 ms
# RTT from a node; measured 2026-10-15). Every node holds it directly as its
# preferred chrony source (clock_node), the host holds it too (deploy_mac /
# deploy_jetson), and the nodes' peer mesh is the fallback when it is off.
# This is what makes a power-cycled Pi (no RTC battery: it boots where its
# filesystem left it, 47 days behind on cam1 on 2026-10-15) come back onto
# the right time by itself, with no internet on the rig segment and whatever
# the host is. Override per run - WILDSYNC_NTP=192.168.1.61 ./deploy.sh ... -
# or set it empty to deploy a rig that has no Compass (the nodes then keep
# only the mesh). The same address is in docs/INSTALL.md 5 and the card
# template (deploy/pi-resync/user-data.template, GNSS_NTP).
NTP_SERVER="${WILDSYNC_NTP-192.168.1.60}"

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

# One transfer or remote step of a node deploy. This script is deliberately not
# `set -e` (deploy_jetson and deploy_strobe lean on unchecked commands and
# '|| echo WARNING' fallbacks, and bash 3.2 on the Mac treats -e inconsistently
# inside functions), so every step that must not be skipped says so here. It
# used to be bare: a failed src/ scp let the node build and restart on STALE
# sources, a failed rsync of lib/ was followed by "provisioned", and either way
# the script exited 0, so `all` and any wrapper saw success. Call it as
# `_try cmd ... || return 1` so a failure stops the node BEFORE the remote
# build-and-restart script runs.
_try() {
  "$@" || { echo "  FAILED on ${name:-?}: $*" >&2; return 1; }
}

# Everything a node needs that `node` does not ship: the SDK headers/libs, the
# Makefile, and the third-party header. Only needed once per machine.
provision_node() {
  local name="$1" ip rc
  ip="$(_ip "$name")" || return 1
  echo "=== provision $name ($ip) ==="
  ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  unreachable, skip"; return 0; }
  # The Pis need the LINUX aarch64 SDK (.so). This checkout's lib/ holds
  # whatever platform the HOST builds with - on the Mac that is dylibs, and
  # rsyncing those bricked the node build with a missing libCr_Core.so while
  # looking like a successful provision (audit 2026-08-27).
  #
  # lib-linux/ is the way out of that on a Mac host: `make sdk` (which stages
  # for THIS platform) could never provision a node from there, and the
  # refusal below had no remedy on that machine. On the Jetson (Linux aarch64)
  # plain `make sdk` stages .so and lib/ works directly. `make sdk-linux` fills
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
    echo "    make sdk-linux CRSDK_DIR=/path/to/RemoteCli   # from the Linux64ARMv8 package" >&2
    echo "  (see docs/PI-SETUP.md), or provision from a Linux host." >&2
    return 1
  fi
  # The node builds ilxctl from include/CRSDK, which is not tracked in git: it
  # is staged from Sony's package by `make sdk` or `make sdk-linux`. Without it
  # the rsync below would succeed and the node build would then fail.
  if [ ! -f "$REPO"/include/CRSDK/CameraRemote_SDK.h ]; then
    echo "  REFUSING provision: the Sony SDK headers are not staged in $REPO/include/CRSDK." >&2
    echo "  They are not part of this repository. Stage them from the SDK package:" >&2
    echo "    make sdk-linux CRSDK_DIR=/path/to/RemoteCli   # from the Linux64ARMv8 package" >&2
    return 1
  fi
  echo "  SDK source: $sdklib"
  _try ssh ubuntu@"$ip" 'mkdir -p ~/wildsync/{src,rig,include,lib,third_party}' || return 1
  _try rsync -az --delete "$REPO"/include/     ubuntu@"$ip":/home/ubuntu/wildsync/include/ || return 1
  _try rsync -az --delete "$sdklib"/           ubuntu@"$ip":/home/ubuntu/wildsync/lib/ || return 1
  _try rsync -az --delete "$REPO"/third_party/ ubuntu@"$ip":/home/ubuntu/wildsync/third_party/ || return 1
  _try scp -q "$REPO"/Makefile ubuntu@"$ip":/home/ubuntu/wildsync/Makefile || return 1
  # This heredoc is UNQUOTED on purpose: $USBFS_MB is expanded here, on the
  # operator's host. Everything else in the body must therefore be escaped
  # (\$var, \${var}) or free of $ and backticks altogether - an unescaped
  # ${Status} aborted the whole script under set -u with an empty body, and
  # backticks in a comment ran deploy.sh locally. The node-deploy heredoc
  # below is quoted instead; it needs no local expansion.
  ssh ubuntu@"$ip" "bash -s" <<REMOTE
set -uo pipefail
# USBFS_MB is a FLOOR, never an assignment. Another USB3 camera on the same
# node can need a much larger value (around 1000 MB of URB space for a large
# single frame), and this step used to hard-set 150 - so re-provisioning such
# a node silently broke that camera's transfers while the Sony path looked
# fine. Raise, never lower.
cur=\$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || echo 0)
case "\$cur" in ''|*[!0-9]*) cur=0;; esac
# ...and the PERSISTED value, which can legitimately be higher than the running
# one: a node whose cmdline.txt already says 1000 but has not rebooted yet is
# still running 150, so a runtime-only floor would quietly write 1000 back down
# to 150 and undo the larger setting on the next boot.
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
  dpkg-query -W -f='\${Status}' "\$pkg" 2>/dev/null | grep -q "ok installed" || \
    echo "  WARNING: \$pkg did NOT install - the node build will fail" >&2
done
if ! dpkg-query -W -f='\${Status}' chrony 2>/dev/null | grep -q "ok installed"; then
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
  # The one deliberate master is a Jetson host that ran 'deploy.sh jetson':
  # 'deploy.sh node' from THAT host writes 20-wildsync-host.conf pointing at it.
  # (No backticks in this body: the heredoc is unquoted, so the operator's
  # shell would run them as commands before ssh ever saw the text.)
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
  # The remote script ends in an echo, so a nonzero status here is the ssh
  # session itself failing - the one case where "provisioned" would be false.
  rc=$?
  [ "$rc" -eq 0 ] || { echo "  FAILED provision $name (remote exit $rc)" >&2; return 1; }
  echo "  provisioned $name — now run: $0 node $name"
}

# A node's clock topology, and nothing else: no build, no service restart
# except chrony's. Called by deploy_node and as `deploy.sh clock camN` on its
# own, which is the recovery for a node found days off. Three layers, in the
# order chrony prefers them:
#   20-wildsync-gnss.conf  the GNSS Compass (NTP_SERVER), stratum 1 on GPS,
#                          `prefer` - the reference under EITHER host
#   20-wildsync-host.conf  the rigd host, only where it is a chrony master
#                          (deploy.sh jetson wrote wildsync-host.conf there;
#                          a Mac never is one, see deploy_mac)
#   10-wildsync.conf       the peer mesh + orphan local reference, so the
#                          nodes stay ~0.6 ms apart with the Compass off
# plus `makestep 1 -1` in chrony.conf: ALWAYS step a clock more than 1 s out.
# Ubuntu ships `makestep 1 3`, which chrony parses AFTER the conf.d drop-ins
# (confdir comes first in chrony.conf), so a drop-in cannot override it - and
# after three clock updates the node may never step again. A node that peered
# a neighbour's wrong clock for three updates before the Compass answered
# would then be stuck days off, slewing at most 83 s per day. A step over 1 s
# is never inside the 10 ms stereo budget anyway: it is a clock that was
# wrong, and every scheduled fire on it was being refused.
clock_node() {
  local name="$1" ip master="" peers="" n
  ip="$(_ip "$name")" || return 1
  ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  $name unreachable, skip clock"; return 0; }
  # The rigd host as a server, only where it is a chrony master. The strobe
  # fires on THIS host's clock and the cameras on the nodes' clocks, so on a
  # Jetson rig one reference keeps delta honest. Left alone (not removed) when
  # this host is not one, so a node deployed from a Jetson keeps that line
  # across a redeploy from a laptop.
  if [ -f /etc/chrony/conf.d/wildsync-host.conf ] && command -v ip >/dev/null 2>&1; then
    master="$(ip route get "$ip" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -1)"
  fi
  for n in $NODES; do [ "$n" = "$name" ] || peers="$peers $(_ip "$n")"; done
  peers="${peers# }"
  echo "  clock: gnss=${NTP_SERVER:-none} host-master=${master:-none} peers=$peers"
  ssh ubuntu@"$ip" "WILDSYNC_NTP='$NTP_SERVER' WILDSYNC_MASTER='$master' WILDSYNC_PEERS='$peers' bash -s" <<'REMOTE'
set -uo pipefail
CONFD=/etc/chrony/conf.d
if ! command -v chronyd >/dev/null 2>&1; then
  echo "  ERROR: chrony is not installed on this node - it keeps no rig clock. Run: deploy.sh provision <node>" >&2
  exit 1
fi
sudo mkdir -p "$CONFD"
# 1. The peer mesh is the card's job (deploy/pi-resync/user-data.template);
#    written here only when the card never did it - cam1's card never ran that
#    step and still carried the two-node star (10-jetson.conf) on 2026-10-15.
#    Same content as the template, peers taken from this script's fleet table.
if [ ! -f "$CONFD/10-wildsync.conf" ]; then
  echo "  no 10-wildsync.conf on this node (its card never ran deploy/pi-resync) - writing the peer mesh"
  { echo "# wildsync clock topology - generated $(date -Is) by deploy.sh clock on $(hostname -s)"
    echo "# the card step (deploy/pi-resync/user-data.template) writes the same - do not hand-edit"
    for p in $WILDSYNC_PEERS; do printf 'peer %s iburst minpoll 4 maxpoll 6\n' "$p"; done
    echo 'local stratum 10 orphan'
    echo 'allow 192.168.1.0/24'
    echo 'makestep 1 -1'
  } | sudo tee "$CONFD/10-wildsync.conf" >/dev/null
fi
# 2. Supersede every drop-in that named ONE master: 10-jetson.conf (the
#    two-node star) and wildsync.conf (an old provision's `server <ssh client>
#    prefer` - the Mac, preferred). The template removes the same two.
sudo rm -f "$CONFD/10-jetson.conf" "$CONFD/wildsync.conf"
# 3. The GNSS Compass, preferred. Removed (not left stale) when deploying
#    with WILDSYNC_NTP empty, so a rig without one does not wait on it.
if [ -n "$WILDSYNC_NTP" ]; then
  { echo "# wildsync: the rig's GNSS Compass - a stratum-1 GPS NTP server on the rig switch."
    echo "# generated $(date -Is) by deploy.sh clock; the card step writes the same - do not hand-edit"
    printf 'server %s iburst minpoll 4 maxpoll 6 prefer\n' "$WILDSYNC_NTP"
  } | sudo tee "$CONFD/20-wildsync-gnss.conf" >/dev/null
else
  sudo rm -f "$CONFD/20-wildsync-gnss.conf"
fi
# 4. The rigd host, where it is a chrony master (see the caller).
if [ -n "$WILDSYNC_MASTER" ]; then
  printf 'server %s iburst minpoll 4 maxpoll 6 prefer\n' "$WILDSYNC_MASTER" | \
    sudo tee "$CONFD/20-wildsync-host.conf" >/dev/null
fi
# 5. Always step a clock more than 1 s out (see the caller's comment).
if grep -q '^[[:space:]]*makestep[[:space:]]' /etc/chrony/chrony.conf; then
  sudo sed -i 's/^[[:space:]]*makestep[[:space:]].*/makestep 1 -1/' /etc/chrony/chrony.conf
else
  echo 'makestep 1 -1' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
fi
# 6. Restart, and prove what it selected. iburst gets the first four samples
#    inside ~2 s; a node that was days off is stepped on the first of them.
sudo systemctl restart chrony
if [ "$(systemctl is-active chrony)" != active ]; then
  echo "  ERROR: chrony did not start after the topology change:" >&2
  systemctl status chrony --no-pager 2>&1 | tail -5 >&2
  exit 1
fi
for _ in 1 2 3 4 5 6 7 8 9 10 11 12; do
  chronyc -n sources 2>/dev/null | grep -q '^\^\*' && break
  sleep 1
done
echo "  chrony: $(chronyc -n tracking 2>/dev/null | sed -n 's/^Reference ID *: *\([^ ]*\).*/refid \1/p') stratum $(chronyc -n tracking 2>/dev/null | sed -n 's/^Stratum *: *//p') makestep=$(sudo chronyd -p 2>/dev/null | grep '^makestep' | tail -1 | cut -d' ' -f2- | tr ' ' '/') date=$(date -Is)"
chronyc -n sources 2>/dev/null | sed -n '/^[\^=#]/p' | sed 's/^/    /'
if [ -n "$WILDSYNC_NTP" ] && ! chronyc -n sources 2>/dev/null | grep -q "^\^\* $WILDSYNC_NTP"; then
  echo "  WARNING: $WILDSYNC_NTP is not the selected source (unreachable, or still converging) - this node is on its peer mesh" >&2
fi
REMOTE
  local rc=$?
  [ "$rc" -eq 0 ] || { echo "  FAILED clock $name (remote exit $rc)" >&2; return 1; }
}

deploy_node() {
  local name="$1" ip rc
  ip="$(_ip "$name")" || return 1
  echo "=== $name ($ip) ==="
  # An unreachable node is skipped, not failed: `all` is documented as "every
  # reachable camera node", and a node that is powered off for a dive is normal.
  ping -c1 -W2 "$ip" >/dev/null 2>&1 || { echo "  unreachable, skip"; return 0; }
  # Create both dirs BEFORE any scp: without src/ the copy fails, and since this
  # script is not `set -e` the build would silently proceed against stale sources.
  # Every transfer is checked for the same reason: all of them finish before the
  # remote script builds and restarts anything, so a failure leaves the running
  # services alone.
  _try ssh ubuntu@"$ip" 'mkdir -p ~/wildsync/src ~/wildsync/rig' || return 1
  _try scp -q "$REPO"/src/*.cpp "$REPO"/src/*.h ubuntu@"$ip":/home/ubuntu/wildsync/src/ || return 1
  _try scp -q "$REPO"/rig/{piagent.py,imu_yb.py,imu_olive.py,olive_ws_bridge.py} ubuntu@"$ip":/home/ubuntu/wildsync/rig/ || return 1
  _try scp -q "$REPO"/docs/PROTOCOL.md ubuntu@"$ip":/home/ubuntu/wildsync/rig/ || return 1
  _try scp -q "$REPO"/deploy/piagent.service ubuntu@"$ip":/tmp/piagent.service || return 1
  _try scp -q "$REPO"/deploy/olive-bridge.service ubuntu@"$ip":/tmp/olive-bridge.service || return 1
  _try scp -q "$REPO"/deploy/ilxctl.service ubuntu@"$ip":/tmp/ilxctl.service || return 1
  # The node's tree has no .git, so record which commit was deployed to it
  # (rig/buildinfo.py; read it with: cat ~/wildsync/BUILD_INFO.json).
  # (A non-git checkout stays a warning, not a failure: that is a documented
  # way to run this script. A failed transfer of the file is not.)
  if python3 "$REPO"/rig/buildinfo.py --write /tmp/wildsync-build-info.json >/dev/null 2>&1; then
    _try scp -q /tmp/wildsync-build-info.json ubuntu@"$ip":/home/ubuntu/wildsync/BUILD_INFO.json
    rc=$?
  else
    echo "  WARNING: $REPO is not a git checkout - no BUILD_INFO.json for $name" >&2
    rc=0
  fi
  rm -f /tmp/wildsync-build-info.json
  [ "$rc" -eq 0 ] || return 1
  # Which IMU serves this node's /imu/* (and so its flight-log attitude). The
  # rig's IMUs are Olive olixVision units, one per camera, relayed by
  # olive-bridge to localhost UDP. Override per run for a Yahboom node:
  #   WILDSYNC_NODE_IMU=yb deploy/deploy.sh node cam1
  local imu="${WILDSYNC_NODE_IMU:-olive:udp:9901}"
  if [ "$imu" = yb ]; then
    printf 'PIAGENT_IMU2=olive:udp:9901\n' > /tmp/wildsync-piagent.env
  else
    printf 'PIAGENT_IMU=%s\nPIAGENT_IMU2=off\n' "$imu" > /tmp/wildsync-piagent.env
  fi
  _try scp -q /tmp/wildsync-piagent.env ubuntu@"$ip":/tmp/wildsync-piagent.env
  rc=$?
  rm -f /tmp/wildsync-piagent.env
  [ "$rc" -eq 0 ] || return 1
  # The node's clock topology, BEFORE anything is built or restarted: a node
  # whose clock is days off (the power-cycled Pi) gets it stepped by chrony
  # here, so the build and the services below start on the right time.
  clock_node "$name" || return 1
  ssh ubuntu@"$ip" bash -s <<'REMOTE'
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
# USB access for ilxctl, which runs as `ubuntu` (deploy/ilxctl.service), not
# root. /dev/bus/usb/* is root-only for writes on Ubuntu Server and a system
# service gets no seat ACL, so without this a fresh node starts ilxctl cleanly
# and then finds no camera. Sony's vendor ID (054c, as in the unbind recipe of
# docs/PI-SETUP.md 10.1) is matched without a product ID because the body enumerates
# with a different product ID per USB mode.
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="054c", GROUP="plugdev", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-wildsync-sony.rules >/dev/null
sudo usermod -aG plugdev ubuntu
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=usb || true
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
# the un-wedge recipe of docs/PI-SETUP.md 10.1. Clear the counter before every
# restart.
sudo systemctl reset-failed ilxctl 2>/dev/null || true
# IMU selection, rendered on the deploying host (see WILDSYNC_NODE_IMU above).
sudo cp /tmp/wildsync-piagent.env /etc/default/piagent
# The Olive's USB-C is a CDC-Ethernet gadget. Pin it as olive0 on its own
# subnet (Pi .2, unit .100) so olive-bridge finds it at 192.168.7.100:5500.
# Written only when it differs, so a routine deploy never re-applies netplan.
OLIVE_NP=/etc/netplan/60-wildsync-olive.yaml
cat > /tmp/60-wildsync-olive.yaml <<'NP'
network:
  version: 2
  ethernets:
    olive0:
      match:
        driver: cdc_ether
      set-name: olive0
      addresses: [192.168.7.2/24]
      dhcp4: false
      optional: true
NP
if ! sudo cmp -s /tmp/60-wildsync-olive.yaml "$OLIVE_NP"; then
  sudo install -m 600 /tmp/60-wildsync-olive.yaml "$OLIVE_NP"
  sudo netplan generate && sudo netplan apply && echo "  olive0 link configured (192.168.7.2/24)"
fi
# The clock topology (GNSS Compass, host master, peer mesh) was applied by
# clock_node before this script ran - nothing about chrony is touched here.
sudo systemctl daemon-reload
sudo systemctl enable olive-bridge >/dev/null 2>&1
sudo systemctl restart olive-bridge
sudo systemctl enable piagent
sudo systemctl restart piagent   # enable --now is a no-op on a running unit: the new file was never loaded
sudo systemctl enable ilxctl >/dev/null 2>&1
# Against a WEDGED daemon this now waits out TimeoutStopSec=45 instead of
# hanging for ever (docs/PI-SETUP.md 10.1). It is slow, not stuck - do not Ctrl-C it.
sudo systemctl restart ilxctl
sleep 4
echo "  ilxctl: $(systemctl is-active ilxctl)   piagent: $(systemctl is-active piagent)"
REMOTE
  # "SDK not staged" and "BUILD FAILED" both exit 1 above, before any service is
  # touched; this is where that becomes the script's own failure instead of a
  # "done" line followed by exit 0.
  rc=$?
  [ "$rc" -eq 0 ] || { echo "  FAILED $name (remote exit $rc)" >&2; return 1; }
  echo "  done $name"
}

# The software stamp every run records (run.json "software"). rigd asks git
# first; BUILD_INFO.json is what it falls back to when git cannot answer (no
# .git beside the code, git missing, or git refusing the checkout). One step
# for both host flavours (deploy_mac, deploy_jetson); the node copy is written
# by deploy_node.
_stamp_host() {
  local sw
  if sw="$(python3 "$REPO"/rig/buildinfo.py --write "$REPO"/BUILD_INFO.json 2>/dev/null)"; then
    echo "  software: $sw"
  else
    echo "  WARNING: $REPO is not a git checkout, so BUILD_INFO.json was not written;" >&2
    echo "  runs record an existing $REPO/BUILD_INFO.json, or the software as unknown" >&2
  fi
}

# The interpreter a host unit runs must import every package in
# requirements.txt, checked under THAT interpreter before the unit is
# (re)started. Without the check a Homebrew bump (2026-10-16: 3.14 -> 3.15
# under the launchd agent, empty site-packages) was deployed as a rigd that
# launchd called "running" and that said nothing anywhere. rig/pydeps.py is the
# same check rigd.py makes at start-up and prints the same one line; on a
# refusal nothing is restarted and the running rigd is left as it was.
_require_pydeps() {  # <interpreter>
  local py="$1"
  if [ ! -x "$py" ]; then
    echo "  REFUSED: $py (the interpreter the rigd unit runs) does not exist" >&2
    return 1
  fi
  if ! "$py" "$REPO"/rig/pydeps.py; then
    echo "  REFUSED: rigd not (re)started; fix the line above and deploy again" >&2
    return 1
  fi
}

# The interpreter deploy/rigd.service runs: the first word of its ExecStart.
_unit_python() {
  sed -n 's/^ExecStart=\([^ ]*\).*/\1/p' "$REPO"/deploy/rigd.service | head -1
}

deploy_jetson() {
  echo "=== jetson (local) ==="
  # Host dependencies. A fresh JetPack image has none of these, and the old
  # path installed nothing: nav then died on import serial, EXIF reads
  # silently returned None (Pillow), and the host free-ran with no time
  # discipline while being the Pis' chrony master - the exact failure the
  # field guide warns about on the Mac (audit 2026-08-27).
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
    python3-serial python3-pil python3-numpy python3-opencv chrony rsync \
    >/dev/null 2>&1 || echo "  WARNING: apt install failed - check network"
  _require_pydeps "$(_unit_python)" || return 1
  # The host is the rig's clock master: serve the LAN, keep a local stratum
  # so the rig stays coherent with no upstream at sea, and hold the GNSS
  # Compass itself (NTP_SERVER) so the host and the nodes - which prefer the
  # Compass directly, clock_node - trace to the same GPS second.
  { printf 'allow 192.168.1.0/24\nlocal stratum 10\n'
    [ -z "$NTP_SERVER" ] || printf 'server %s iburst minpoll 4 maxpoll 6 prefer\n' "$NTP_SERVER"
  } | sudo tee /etc/chrony/conf.d/wildsync-host.conf >/dev/null
  sudo systemctl enable --now chrony >/dev/null 2>&1 || true
  sudo systemctl restart chrony >/dev/null 2>&1 || true
  # Render the unit for THIS checkout and user - the template carries
  # placeholders, never a hardcoded machine.
  sed "s|__REPO__|$REPO|g; s|__USER__|$(id -un)|g" \
    "$REPO"/deploy/rigd.service | sudo tee /etc/systemd/system/rigd.service >/dev/null
  _stamp_host
  sudo systemctl daemon-reload
  sudo systemctl enable rigd >/dev/null 2>&1
  # enable --now is a NO-OP on an already-running unit: redeploys silently
  # kept the old code. Restart is the deploy.
  sudo systemctl restart rigd
  sleep 3
  echo "  rigd: $(systemctl is-active rigd)  -> http://$(hostname -I | awk '{print $1}'):9090"
  echo "  if the nodes are unreachable over ssh from this host, run '$0 trust <camN>' for each of: $NODES"
}

# This Mac as the rig host. It is NOT a chrony master and never was: the rig's
# time reference is the GNSS Compass (NTP_SERVER), which every node holds
# directly (clock_node). What the Mac must do is hold the SAME reference, so
# that the host offset rigd measures - and corrects every fire target and every
# capture instant for - is tens of milliseconds with a small uncertainty, not
# the 100-190 ms of an undisciplined Mac (2026-08-23: 187 ms behind, ~60 ppm;
# 2026-10-15: 100 ms behind GPS and Apple alike, with time.apple.com beyond the
# rig segment anyway). macOS's own time service takes one server; this points
# it at the Compass (System Settings > General > Date & Time shows it), which
# needs an administrator password. Then the launchd agent of docs/INSTALL.md
# 3a, rendered for this checkout and (re)started, and the software stamp -
# deploy_jetson is the twin.
deploy_mac() {
  echo "=== mac (local) ==="
  [ "$(uname -s)" = Darwin ] || { echo "  this is not a Mac - use: $0 jetson" >&2; return 1; }
  if [ -n "$NTP_SERVER" ]; then
    echo "  pointing this Mac's clock at $NTP_SERVER (systemsetup asks for an administrator password)"
    if sudo systemsetup -setnetworktimeserver "$NTP_SERVER" >/dev/null 2>&1 && \
       sudo systemsetup -setusingnetworktime on >/dev/null 2>&1; then
      echo "  network time on, server $NTP_SERVER"
    else
      echo "  WARNING: systemsetup failed - set the server to $NTP_SERVER by hand:" >&2
      echo "  System Settings > General > Date & Time > Source, then confirm with: sntp $NTP_SERVER" >&2
    fi
    # This Mac against the Compass right now: sntp prints the correction this
    # clock needs (+ = this Mac is behind) and the Compass's own error bar.
    # Expect it to shrink over the next minutes as macOS slews.
    sntp -t 3 "$NTP_SERVER" 2>/dev/null | head -1 | sed 's/^/  sntp vs compass: /'
  else
    echo "  WILDSYNC_NTP is empty - this Mac's time server is left alone"
  fi
  local venv="$HOME/rig/venv/bin/python3"
  local plist="$HOME/Library/LaunchAgents/org.wildtechnology.wildsync.rigd.plist"
  local label="org.wildtechnology.wildsync.rigd"
  if [ ! -x "$venv" ]; then
    echo "  no $venv - create it first (docs/INSTALL.md 3a):" >&2
    echo "    /opt/homebrew/bin/python3 -m venv ~/rig/venv && ~/rig/venv/bin/python3 -m pip install -r $REPO/requirements.txt" >&2
    return 1
  fi
  mkdir -p "$HOME/rig" "$HOME/Library/LaunchAgents"
  # Rendered for THIS checkout and home - the template carries placeholders.
  # Rendered aside first: the interpreter it names is checked before the plist
  # in place, or the running agent, is touched.
  local rendered="$plist.new"
  sed "s|__REPO__|$REPO|g; s|__HOME__|$HOME|g" "$REPO"/deploy/rigd.launchd.plist > "$rendered"
  local py
  py="$(plutil -extract ProgramArguments.0 raw -o - "$rendered" 2>/dev/null)" || py=""
  [ -n "$py" ] || py="$venv"
  if ! _require_pydeps "$py"; then
    rm -f "$rendered"
    return 1
  fi
  mv -f "$rendered" "$plist"
  _stamp_host
  # bootout then bootstrap, not kickstart: launchd reads a plist only at
  # bootstrap, so a regenerated one (new venv, moved checkout) needs the
  # round trip. Restart is the deploy; rigd finalises an active run on SIGTERM.
  launchctl bootout "gui/$(id -u)/$label" >/dev/null 2>&1 || true
  if launchctl bootstrap "gui/$(id -u)" "$plist"; then
    sleep 3
    echo "  rigd: $(launchctl print "gui/$(id -u)/$label" 2>/dev/null | sed -n 's/^[[:space:]]*state = //p' | head -1) -> http://localhost:9090  (log ~/rig/rigd-launchd.log)"
  else
    echo "  FAILED: launchctl bootstrap $plist" >&2
    return 1
  fi
  echo "  if the nodes are unreachable over ssh from this host, run '$0 trust <camN>' for each of: $NODES"
}

# The XR256 strobe agent on THIS host (the Jetson): piagent in its strobe
# role, claiming one GPIO line and nothing else, registered with rigd as the
# camera-less node "jetson". Required: which line drives the trigger board.
#   WILDSYNC_STROBE_LINE=<offset> [WILDSYNC_GPIOCHIP=gpiochip0] \
#   [WILDSYNC_STROBE_ACTIVE=high] deploy/deploy.sh strobe
# Find the offset with `gpioinfo` after setting the header pin to GPIO with
# jetson-io; docs/xr256-jetson-trigger.md section 2.2 has the pin rules.
deploy_strobe() {
  echo "=== strobe agent (local) ==="
  local line="${WILDSYNC_STROBE_LINE:-}"
  local chip="${WILDSYNC_GPIOCHIP:-gpiochip0}"
  local active="${WILDSYNC_STROBE_ACTIVE:-high}"
  if [ -z "$line" ]; then
    echo "  set WILDSYNC_STROBE_LINE to the trigger pin's line offset on $chip" >&2
    echo "  (gpioinfo $chip lists them; see docs/xr256-jetson-trigger.md 2.2)" >&2
    return 1
  fi
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q python3-libgpiod gpiod \
    >/dev/null 2>&1 || echo "  WARNING: apt install failed - check network"
  sudo groupadd -f gpio
  sudo usermod -aG gpio "$(id -un)"
  echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | \
    sudo tee /etc/udev/rules.d/99-gpio.rules >/dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=gpio || true
  printf 'PIAGENT_ROLE=strobe\nWILDSYNC_GPIOCHIP=%s\nWILDSYNC_STROBE_LINE=%s\nWILDSYNC_STROBE_ACTIVE=%s\n' \
    "$chip" "$line" "$active" | sudo tee /etc/default/piagent-strobe >/dev/null
  sed "s|__REPO__|$REPO|g; s|__USER__|$(id -un)|g" \
    "$REPO"/deploy/piagent-strobe.service | \
    sudo tee /etc/systemd/system/piagent-strobe.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable piagent-strobe >/dev/null 2>&1
  sudo systemctl restart piagent-strobe
  # Register with rigd: node "jetson" on localhost, camera switched off, so it
  # is a strobe host and never a camera (no ilxctl runs here).
  mkdir -p ~/rig
  python3 - <<'PY'
import json, os
home = os.path.expanduser("~/rig")
def merge(name, patch):
    path = os.path.join(home, name)
    try:
        with open(path) as fh:
            cur = json.load(fh)
    except (OSError, ValueError):
        cur = {}
    if not isinstance(cur, dict):
        print("  %s is a list - add the jetson entry by hand" % path)
        return
    cur.update(patch)
    with open(path, "w") as fh:
        json.dump(cur, fh, indent=1)
merge("nodes.json", {"jetson": {"host": "127.0.0.1", "cam_num": 9,
                                "optional": True}})
merge("camera_enabled.json", {"jetson": False})
PY
  if systemctl is-active --quiet rigd; then
    _require_pydeps "$(_unit_python)" && sudo systemctl restart rigd
  fi
  sleep 3
  echo "  piagent-strobe: $(systemctl is-active piagent-strobe)  ($chip line $line, active-$active)"
  echo "  in the UI: Controls > Strobe > Node = jetson"
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
  clock)     clock_node "${2:?usage: deploy.sh clock <camN>}";;
  mac)       deploy_mac;;
  jetson)    deploy_jetson;;
  strobe)    deploy_strobe;;
  # The host goes FIRST - this Mac, or this Jetson, whichever is running the
  # script: clock_node points a node at the host's chrony only if
  # /etc/chrony/conf.d/wildsync-host.conf already exists, and only
  # deploy_jetson writes it. Nodes first left that file absent on a first
  # install, so every node silently skipped the host master and the strobe (host
  # clock) and the cameras (mesh clock) were on different references. (Before
  # deploy_mac existed, `all` on a Mac ran the Jetson steps - apt-get and
  # systemd - and failed its way through them.) One bad node must not stop the
  # others, so failures are collected and `all` exits nonzero at the end.
  all)       fail=""
             if [ "$(uname -s)" = Darwin ]; then
               deploy_mac || fail="$fail mac"
             else
               deploy_jetson || fail="$fail jetson"
             fi
             for n in $NODES; do deploy_node "$n" || fail="$fail $n"; done
             if [ -n "$fail" ]; then echo "FAILED:$fail" >&2; exit 1; fi;;
  *) echo "usage: $0 {provision <camN>|trust <camN>|node <camN>|clock <camN>|mac|jetson|strobe|all}"; exit 1;;
esac

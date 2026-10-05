#!/bin/bash
# Install the re-provision user-data onto a node's mounted system-boot card.
#
#   deploy/pi-resync/install.sh /Volumes/system-boot
#
# Fills the template from the card's own existing user-data (password hash and
# the key already on the card are carried over unchanged) plus this host's
# ~/.ssh/id_ed25519.pub, backs up the originals, and bumps the cloud-init
# instance-id so the node re-runs the users + chrony steps on its next boot.
# Run once per card.
set -euo pipefail
V="${1:-/Volumes/system-boot}"
HERE="$(cd "$(dirname "$0")" && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"

[ -f "$V/user-data" ] || { echo "no user-data under $V — is the card mounted?"; exit 1; }
[ -f "$HOME/.ssh/id_ed25519.pub" ] || { echo "no ~/.ssh/id_ed25519.pub on this host - run ssh-keygen -t ed25519"; exit 1; }

HASH="$(grep -m1 '^[[:space:]]*passwd:' "$V/user-data" | sed 's/^[[:space:]]*passwd:[[:space:]]*//')"
CKEY="$(grep -m1 'ssh-ed25519' "$V/user-data" | sed 's/^[[:space:]]*-[[:space:]]*//')"
HKEY="$(cat "$HOME/.ssh/id_ed25519.pub")"
[ -n "$HASH" ] && [ -n "$CKEY" ] || { echo "could not read the existing passwd/key from $V/user-data"; exit 1; }

cp "$V/user-data" "$V/user-data.bak-$STAMP"
cp "$V/meta-data" "$V/meta-data.bak-$STAMP"

# awk, not sed: the hash contains $ and / which sed's s/// would mangle.
awk -v h="$HASH" -v c="$CKEY" -v k="$HKEY" '
  { gsub(/__PASSWD_HASH__/, h); gsub(/__CARD_KEY__/, c); gsub(/__HOST_KEY__/, k); print }
' "$HERE/user-data.template" > "$V/user-data"
printf 'instance-id: wildsync-resync-%s\n' "$STAMP" > "$V/meta-data"

# --pi5-power-trim: cap the Pi 5's peak draw on a node whose PoE port is at
# its ceiling (cam1: Pi 5 + body on one 802.3at port browned out under
# synchronized fires, 2026-08-23). The rig's loads are I/O-bound, so the
# 2.4 GHz boost buys nothing; Wi-Fi/BT are unused (Ethernet only). ~2-3 W
# off the peak. Appended under a [pi5] section so it is inert on a Pi 4.
#
# OPT-IN PER NODE, and NOT for cam3. cam3 runs on its own dedicated PoE
# injector — the real fix for the brown-out — so it has the headroom and must
# keep full clocks, Wi-Fi and BT. Do not pass this flag when writing cam3's
# card. cam1 still shares its port and may still use it.
if [ "${2:-}" = "--pi5-power-trim" ]; then
  cp "$V/config.txt" "$V/config.txt.bak-$STAMP"
  if ! grep -q "wildsync power trim" "$V/config.txt"; then
    cat >> "$V/config.txt" <<'TRIM'

[pi5]
# wildsync power trim (deploy/pi-resync/install.sh --pi5-power-trim)
arm_freq=1800
dtoverlay=disable-wifi
dtoverlay=disable-bt
[all]
TRIM
  fi
  echo "config.txt: Pi 5 power trim applied (backup config.txt.bak-$STAMP)"
fi

echo "written: $V/user-data ($(grep -c ssh-ed25519 "$V/user-data") keys), $V/meta-data ($(cat "$V/meta-data"))"
echo "originals: user-data.bak-$STAMP, meta-data.bak-$STAMP"
echo "next: eject the card, boot the Pi, then from this host:  ssh ubuntu@<pi-ip>"

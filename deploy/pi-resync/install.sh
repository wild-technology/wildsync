#!/bin/bash
# Install the re-provision user-data onto a node's mounted system-boot card.
#
#   deploy/pi-resync/install.sh /Volumes/system-boot
#
# Fills the template from the card's own existing user-data (password hash,
# the key already on the card and the hostname, when it has one, are carried
# over unchanged) plus this host's ~/.ssh/id_ed25519.pub, backs up the
# originals, and bumps the cloud-init instance-id so the node re-runs the users
# + chrony steps on its next boot. Run once per card.
set -euo pipefail
V="${1:-/Volumes/system-boot}"
HERE="$(cd "$(dirname "$0")" && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"

[ -f "$V/user-data" ] || { echo "no user-data under $V — is the card mounted?"; exit 1; }
[ -f "$HOME/.ssh/id_ed25519.pub" ] || { echo "no ~/.ssh/id_ed25519.pub on this host - run ssh-keygen -t ed25519"; exit 1; }

# '|| true': under pipefail a grep with no match fails the assignment, and
# set -e then killed the script silently, before the message below could say
# what was missing.
HASH="$(grep -m1 '^[[:space:]]*passwd:' "$V/user-data" | sed 's/^[[:space:]]*passwd:[[:space:]]*//' || true)"
CKEY="$(grep -m1 'ssh-ed25519' "$V/user-data" | sed 's/^[[:space:]]*-[[:space:]]*//' || true)"
HKEY="$(cat "$HOME/.ssh/id_ed25519.pub")"
if [ -z "$HASH" ] || [ -z "$CKEY" ]; then
  echo "could not read the existing passwd/key from $V/user-data:"
  [ -n "$HASH" ] || echo "  no passwd: line - this template carries the card's password hash over, so a key-only card needs a hash added by hand (or re-image it, docs/PI-SETUP.md section 3) before re-running"
  [ -n "$CKEY" ] || echo "  no ssh-ed25519 key line"
  exit 1
fi

# A card that was named by cloud-init (docs/PI-SETUP.md section 3: hostname: plus
# manage_etc_hosts:) must come out of this rewrite still named. The template
# used to hard-code preserve_hostname: true, which dropped hostname: and left
# the node on the image default - and cloud-init ignores hostname: whenever
# preserve_hostname is set, so the name has to be written back as hostname:.
# A card with no hostname: (already booted, written by an earlier run of this
# template) keeps preserve_hostname: true and its current name.
NAME="$(grep -m1 '^hostname:' "$V/user-data" \
  | sed -e 's/^hostname:[[:space:]]*//' -e 's/[[:space:]]*#.*$//' -e 's/[[:space:]]*$//' \
  | tr -d "\"'" || true)"
case "$NAME" in
  *[!A-Za-z0-9.-]*) echo "unusable hostname '$NAME' in $V/user-data (letters, digits, . and - only)"; exit 1;;
esac

cp "$V/user-data" "$V/user-data.bak-$STAMP"
cp "$V/meta-data" "$V/meta-data.bak-$STAMP"

# awk, not sed: the hash contains $ and / which sed's s/// would mangle.
awk -v h="$HASH" -v c="$CKEY" -v k="$HKEY" -v nm="$NAME" '
  /^__NAME_BLOCK__$/ {
    if (nm != "") { print "hostname: " nm; print "manage_etc_hosts: true" }
    else          { print "preserve_hostname: true" }
    next
  }
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

echo "written: $V/user-data ($(grep -c '^[[:space:]]*-[[:space:]]*ssh-ed25519' "$V/user-data") keys), $V/meta-data ($(cat "$V/meta-data"))"
echo "originals: user-data.bak-$STAMP, meta-data.bak-$STAMP"
echo "next: eject the card, boot the Pi, then from this host:  ssh ubuntu@<pi-ip>"

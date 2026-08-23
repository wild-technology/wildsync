#!/bin/bash
# pi-power.sh — trim a node's electrical peak so two nodes firing in sync
# stay inside a shared PoE budget that cannot be changed.
#
#   deploy/pi-power.sh cam1            # apply (config.txt + governor), then reboot
#   deploy/pi-power.sh cam2 --no-reboot
#
# What it does, and why each line exists (bench, 2026-08-23: with both nodes
# firing synchronously one node's PoE port dropped every time; either node
# alone was clean; the topology cannot change):
#   * CPU clock capped — Pi 5 1.5 GHz, Pi 4 boost off (1.5 GHz). The rig's
#     loads are I/O-bound; the fire path is a sub-ms spin.
#   * Wi-Fi and Bluetooth off — Ethernet only, the radios just draw.
#   * cpufreq governor 'powersave' — holds the floor frequency (Pi 4 600 MHz,
#     Pi 5 1.5 GHz) between bursts instead of racing to the cap on every
#     HTTP request; the node still serves everything (measured after).
#   * PoE HAT fan only above 60 C — the fan is ~0.4 W at full speed and the
#     capped SoC runs cooler anyway.
# Nothing here touches the camera body's draw, which is the larger share.
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
name="${1:?usage: pi-power.sh <camN> [--no-reboot]}"
case "$name" in cam1) ip=192.168.1.201;; cam2) ip=192.168.1.202;; *) echo "unknown node $name" >&2; exit 1;; esac
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null ubuntu@$ip"

$SSH 'bash -s' <<'REMOTE'
set -u
CFG=/boot/firmware/config.txt
sudo cp "$CFG" "$CFG.bak-power-$(date +%Y%m%d-%H%M%S)"
# strip any earlier power block (ours), then append the current one
sudo python3 - <<'PY'
import re, io
p = "/boot/firmware/config.txt"
s = open(p).read()
s = re.sub(r"\n# >>> wildsync power.*?# <<< wildsync power\n", "\n", s, flags=re.S)
s = re.sub(r"\n\[pi5\]\n# wildsync power trim \(deploy/pi-resync/install.sh --pi5-power-trim\)\narm_freq=\d+\ndtoverlay=disable-wifi\ndtoverlay=disable-bt\n\[all\]\n", "\n", s)
block = """
# >>> wildsync power (deploy/pi-power.sh) — keep two synchronized nodes inside the PoE budget
[pi5]
arm_freq=1500
dtparam=fan_temp0=60000
[pi4]
arm_boost=0
[all]
dtoverlay=disable-wifi
dtoverlay=disable-bt
# <<< wildsync power
"""
open(p, "w").write(s.rstrip("\n") + "\n" + block)
PY
# governor at the floor, every boot
sudo tee /etc/systemd/system/wildsync-power.service >/dev/null <<'UNIT'
[Unit]
Description=Wild Sync node power trim (cpufreq powersave)
After=multi-user.target
[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo powersave > $g; done'
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl daemon-reload
sudo systemctl enable --now wildsync-power.service >/dev/null 2>&1
echo "$(hostname): config.txt power block written; governor now $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor); cur $(vcgencmd measure_clock arm | cut -d= -f2) Hz"
REMOTE

if [ "${2:-}" != "--no-reboot" ]; then
  echo "rebooting $name for config.txt to take effect"
  $SSH 'sudo reboot' 2>/dev/null
fi

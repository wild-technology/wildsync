#!/bin/bash
# Desktop launcher: make sure rigd is up, then open the UI.
#
# rigd is a systemd unit with Restart=always, so it is normally already running
# and this just opens a browser. It is started here anyway because the one thing
# an operator must never have to do is find out from an empty page that the
# service is down.
URL="http://localhost:9090"

systemctl is-active --quiet rigd || systemctl start rigd 2>/dev/null || \
  pkexec systemctl start rigd 2>/dev/null || true

# Wait for the port rather than racing it: rigd takes a couple of seconds to
# bind, and a browser opened too early shows a connection error the operator
# then has to reason about.
for _ in $(seq 1 30); do
  curl -fsS -m1 -o /dev/null "$URL/api/fleet" 2>/dev/null && break
  sleep 0.5
done

if ! curl -fsS -m2 -o /dev/null "$URL/api/fleet" 2>/dev/null; then
  MSG="rigd is not responding on :9090.
Check:  systemctl status rigd
Logs:   journalctl -u rigd -n 50"
  command -v zenity >/dev/null && zenity --error --width=360 --text="$MSG" || echo "$MSG"
  exit 1
fi

xdg-open "$URL" >/dev/null 2>&1 || exec ${BROWSER:-firefox} "$URL"

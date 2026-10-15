#!/bin/bash
# Wild Sync.app - the Mac host's Desktop launcher: make sure rigd is up, then
# open the UI. `deploy/deploy.sh launcher` copies this file into the bundle as
# Contents/MacOS/wildsync-launcher; deploy/launch-wildsync.sh is the Linux twin.
#
# rigd is the launchd agent of docs/INSTALL.md 3a (KeepAlive), so it is
# normally already answering and this only opens the browser. When it is not:
#   * agent loaded  -> `launchctl kickstart` WITHOUT -k: starts a stopped job,
#                      never kills a running rigd (an active transect is safe);
#   * agent not loaded but its plist installed -> `launchctl bootstrap`;
#   * no plist      -> say so: deploy/deploy.sh mac installs it.
# If rigd still does not answer, an alert shows launchd's state and last exit
# code and the last lines of ~/rig/rigd-launchd.log - which is where rigd's one
# line goes when it refuses an interpreter missing requirements.txt (exit 78,
# rig/pydeps.py). The one thing an operator must never have to do is find out
# from an empty page that the service is down.
#
# Bash 3.2 (macOS /bin/bash). The WILDSYNC_* overrides exist for the tests
# (rig/tests/audit_launcher.py); an operator never sets them.

URL="${WILDSYNC_URL:-http://localhost:9090}"
LABEL="${WILDSYNC_RIGD_LABEL:-org.wildtechnology.wildsync.rigd}"
PLIST="${WILDSYNC_RIGD_PLIST:-$HOME/Library/LaunchAgents/$LABEL.plist}"
LOG="${WILDSYNC_RIGD_LOG:-$HOME/rig/rigd-launchd.log}"
WAIT_S="${WILDSYNC_LAUNCHER_WAIT_S:-20}"
OPEN="${WILDSYNC_OPEN:-/usr/bin/open}"
DOMAIN="gui/$(id -u)"
JOB="$DOMAIN/$LABEL"

up() {
  curl -fsS -m 2 -o /dev/null "$URL/api/fleet" 2>/dev/null
}

fail() {
  if [ -n "${WILDSYNC_LAUNCHER_NODIALOG:-}" ]; then
    printf '%s\n' "$1" >&2
  else
    printf '%s\n' "$1" >&2
    /usr/bin/osascript - "$1" >/dev/null 2>&1 <<'OSA'
on run argv
  display alert "Wild Sync" message (item 1 of argv) as critical
end run
OSA
  fi
  exit 1
}

notify() {
  # The bundle has no Dock icon or window (LSUIElement), so a start that takes
  # a while would otherwise look like a double-click that did nothing.
  [ -n "${WILDSYNC_LAUNCHER_NODIALOG:-}" ] && return 0
  /usr/bin/osascript - "$1" >/dev/null 2>&1 <<'OSA' &
on run argv
  display notification (item 1 of argv) with title "Wild Sync"
end run
OSA
}

if ! up; then
  if launchctl print "$JOB" >/dev/null 2>&1; then
    notify "rigd is not answering - starting it (up to ${WAIT_S} s)"
    launchctl kickstart "$JOB" >/dev/null 2>&1
  elif [ -f "$PLIST" ]; then
    notify "rigd is not loaded - starting it (up to ${WAIT_S} s)"
    launchctl bootstrap "$DOMAIN" "$PLIST" >/dev/null 2>&1
  else
    fail "The rigd agent is not installed: $PLIST does not exist.

Install it from the Wild Sync checkout with:
  deploy/deploy.sh mac
(docs/INSTALL.md 3a)."
  fi
  # Bounded by the clock, not by passes: a wedged rigd that accepts the
  # connection and never answers costs each probe its full 2 s timeout.
  end=$((SECONDS + WAIT_S))
  while [ "$SECONDS" -lt "$end" ]; do
    up && break
    sleep 0.5
  done
fi

if ! up; then
  state="$(launchctl print "$JOB" 2>/dev/null | \
    sed -n 's/^[[:space:]]*state = //p; s/^[[:space:]]*last exit code = /last exit code /p' | \
    head -2 | paste -sd ',' - | sed 's/,/, /g')"
  [ -n "$state" ] || state="not loaded"
  tail_lines=""
  if [ -f "$LOG" ]; then
    tail_lines="$(grep -v '^[[:space:]]*$' "$LOG" | tail -n 3 | cut -c1-400)"
  fi
  hint=""
  case "$state" in
    *"last exit code 78"*)
      hint="
rigd refused its Python interpreter: a package from requirements.txt is
missing. The log line below names it and the fix (docs/INSTALL.md 3a).";;
  esac
  fail "rigd is not answering on $URL.

launchd: $LABEL - $state$hint

Last lines of $LOG:
${tail_lines:-(empty)}

Restart it with:
  launchctl kickstart -k $JOB"
fi

exec "$OPEN" "$URL"

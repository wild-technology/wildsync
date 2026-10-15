#!/usr/bin/env python3
"""Audit regression suite - the Mac host's Desktop launcher, Wild Sync.app (L1..L6).

`deploy/deploy.sh launcher` builds ~/Desktop/Wild Sync.app around
deploy/launch-wildsync-mac.sh. Opening it must never disturb a running rigd,
must start a stopped one, and must say why when rigd will not answer.

L1  the bundle: built aside and swapped in, Info.plist lints and names an
    executable and an icon that exist, a rebuild replaces it cleanly.
L2  rigd answering: the UI is opened and launchctl is never touched.
L3  rigd down, agent loaded: `launchctl kickstart` WITHOUT -k (never a kill),
    then the UI once the port answers.
L4  rigd down, agent not loaded, plist installed: `launchctl bootstrap`.
L5  no plist: nothing is started; the message names deploy.sh mac.
L6  rigd never answers: exit 1, nothing opened; the message carries launchd's
    state and last exit code, the exit-78 hint, and the log's last line (the
    rig/pydeps.py line).
L7  a wedged rigd that accepts connections and never answers: the wait is
    bounded by the clock, not by probe count x a 2 s curl timeout.

Hermetic: launchctl and the opener are stubs that log their argv, rigd is a
loopback HTTP server in this process, the bundle is built in a temp dir, no
alert is shown (WILDSYNC_LAUNCHER_NODIALOG). Nothing touches the real agent.

Run standalone:  python3 rig/tests/audit_launcher.py
"""

import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
REPO = os.path.dirname(RIG)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

import soaktest                                          # noqa: E402
from soaktest import check, sect                         # noqa: E402

LAUNCHER = os.path.join(REPO, "deploy", "launch-wildsync-mac.sh")
DEPLOY = os.path.join(REPO, "deploy", "deploy.sh")
LABEL = "org.example.wildsync-audit"
PYDEPS_LINE = ("rigd: /opt/homebrew/bin/python3 (Python 3.15) cannot import "
               "pyserial (import serial) - run rigd from the venv of "
               "docs/INSTALL.md 3a")


class _Rigd(object):
    """A loopback stand-in for rigd's /api/fleet. `up` is a callable deciding
    whether it answers 200 (else 503), so a test can bring it up mid-wait."""

    def __init__(self, up):
        owner = self

        class H(BaseHTTPRequestHandler):
            def do_GET(self):
                ok = self.path.startswith("/api/fleet") and owner.up()
                self.send_response(200 if ok else 503)
                self.send_header("Content-Length", "2")
                self.end_headers()
                self.wfile.write(b"{}")

            def log_message(self, *a):
                pass

        self.up = up
        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = "http://127.0.0.1:%d" % self.srv.server_address[1]
        self.t = threading.Thread(target=self.srv.serve_forever, daemon=True)
        self.t.start()

    def close(self):
        self.srv.shutdown()
        self.srv.server_close()


def _write(path, text, mode=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write(text)
    if mode:
        os.chmod(path, mode)


def _stubs(tmp, loaded, print_body=""):
    """launchctl: logs argv; `print` succeeds (with print_body) only when the
    job is `loaded`; `kickstart`/`bootstrap` touch FLAG. The opener logs."""
    bindir = os.path.join(tmp, "bin")
    log = os.path.join(tmp, "calls.log")
    flag = os.path.join(tmp, "started.flag")
    body_file = os.path.join(tmp, "print.txt")
    _write(body_file, print_body)
    _write(os.path.join(bindir, "launchctl"), """#!/bin/sh
echo "launchctl $*" >> "%s"
case "$1" in
  print) [ "%s" = yes ] || exit 113; cat "%s";;
  kickstart|bootstrap) : > "%s";;
esac
exit 0
""" % (log, "yes" if loaded else "no", body_file, flag), 0o755)
    opener = os.path.join(bindir, "opener")
    _write(opener, '#!/bin/sh\necho "open $*" >> "%s"\n' % log, 0o755)
    return bindir, log, flag, opener


def _run(tmp, url, bindir, opener, plist, logfile, wait_s=2):
    env = dict(os.environ,
               PATH=bindir + os.pathsep + os.environ.get("PATH", ""),
               HOME=os.path.join(tmp, "home"),
               WILDSYNC_URL=url, WILDSYNC_RIGD_LABEL=LABEL,
               WILDSYNC_RIGD_PLIST=plist, WILDSYNC_RIGD_LOG=logfile,
               WILDSYNC_LAUNCHER_WAIT_S=str(wait_s), WILDSYNC_OPEN=opener,
               WILDSYNC_LAUNCHER_NODIALOG="1")
    t0 = time.time()
    p = subprocess.run(["/bin/bash", LAUNCHER], env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True,
                       timeout=60)
    return p, time.time() - t0


def _calls(log):
    try:
        with open(log) as fh:
            return [ln.rstrip("\n") for ln in fh]
    except OSError:
        return []


# ---------------------------------------------------------------------------
def _l1_bundle(opts):
    sect("L1 deploy.sh launcher builds Wild Sync.app")
    if sys.platform != "darwin":
        check("L1 deploy.sh launcher refuses off macOS",
              subprocess.run(["bash", DEPLOY, "launcher"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE
                             ).returncode != 0)
        return
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    try:
        env = dict(os.environ, WILDSYNC_LAUNCHER_DIR=tmp)
        p = subprocess.run(["bash", DEPLOY, "launcher"], env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        app = os.path.join(tmp, "Wild Sync.app")
        plist = os.path.join(app, "Contents", "Info.plist")
        check("L1 deploy.sh launcher exits 0", p.returncode == 0,
              p.stderr[-300:])
        lint = subprocess.run(["plutil", "-lint", plist],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        check("L1 Info.plist lints", lint.returncode == 0)

        def key(k):
            r = subprocess.run(["plutil", "-extract", k, "raw", "-o", "-",
                                plist], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, universal_newlines=True)
            return r.stdout.strip()
        exe = os.path.join(app, "Contents", "MacOS", key("CFBundleExecutable"))
        icon = os.path.join(app, "Contents", "Resources",
                            key("CFBundleIconFile") + ".icns")
        check("L1 CFBundleExecutable exists and is executable",
              os.path.isfile(exe) and os.access(exe, os.X_OK), exe)
        with open(exe) as fh, open(LAUNCHER) as src:
            check("L1 ... and is deploy/launch-wildsync-mac.sh",
                  fh.read() == src.read())
        check("L1 CFBundleIconFile resolves to the raven icon",
              os.path.isfile(icon) and os.path.getsize(icon) == os.path.getsize(
                  os.path.join(REPO, "deploy", "icons", "wildsync.icns")), icon)
        arch = subprocess.run(["plutil", "-extract", "LSArchitecturePriority.0",
                               "raw", "-o", "-", plist], stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, universal_newlines=True)
        check("L1 LSArchitecturePriority puts arm64 first (a script bundle "
              "otherwise runs under Rosetta, absent on a fresh Apple Silicon "
              "Mac)", arch.stdout.strip() == "arm64", arch.stdout)
        check("L1 an APPL bundle named Wild Sync, version from buildinfo",
              key("CFBundlePackageType") == "APPL"
              and key("CFBundleName") == "Wild Sync"
              and key("CFBundleShortVersionString") not in ("", "0"))
        _write(os.path.join(app, "Contents", "stale"), "x")
        p = subprocess.run(["bash", DEPLOY, "launcher"], env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        check("L1 a rebuild replaces the bundle (no stale files, no temp left)",
              p.returncode == 0
              and not os.path.exists(os.path.join(app, "Contents", "stale"))
              and sorted(os.listdir(tmp)) == ["Wild Sync.app"],
              repr(os.listdir(tmp)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _l2_up(opts):
    sect("L2 rigd answering: open the UI, leave launchctl alone")
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    rigd = _Rigd(lambda: True)
    try:
        bindir, log, _flag, opener = _stubs(tmp, loaded=True)
        p, dt = _run(tmp, rigd.url, bindir, opener,
                     os.path.join(tmp, "agent.plist"),
                     os.path.join(tmp, "rigd-launchd.log"))
        calls = _calls(log)
        check("L2 exits 0", p.returncode == 0, p.stderr[-300:])
        check("L2 opens the UI URL", calls == ["open " + rigd.url], repr(calls))
        check("L2 launchctl never called (a running rigd is not touched)",
              not any(c.startswith("launchctl") for c in calls), repr(calls))
        check("L2 promptly", dt < 5, "%.1f s" % dt)
    finally:
        rigd.close()
        shutil.rmtree(tmp, ignore_errors=True)


def _l3_kickstart(opts):
    sect("L3 rigd down, agent loaded: kickstart without -k, then open")
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    flag = os.path.join(tmp, "started.flag")
    rigd = _Rigd(lambda: os.path.exists(flag))
    try:
        bindir, log, flag2, opener = _stubs(tmp, loaded=True,
                                            print_body="\tstate = not running\n")
        assert flag2 == flag
        p, _dt = _run(tmp, rigd.url, bindir, opener,
                      os.path.join(tmp, "agent.plist"),
                      os.path.join(tmp, "rigd-launchd.log"))
        calls = _calls(log)
        kick = [c for c in calls if c.startswith("launchctl kickstart")]
        check("L3 exits 0 once rigd answers", p.returncode == 0,
              p.stderr[-300:])
        check("L3 kickstart, exactly once, of the agent's job",
              len(kick) == 1 and kick[0].endswith("/" + LABEL), repr(calls))
        check("L3 never with -k (that would kill a running rigd)",
              not any(" -k" in c or "kill" in c or "bootout" in c
                      for c in calls), repr(calls))
        check("L3 then opens the UI", calls[-1:] == ["open " + rigd.url],
              repr(calls))
    finally:
        rigd.close()
        shutil.rmtree(tmp, ignore_errors=True)


def _l4_bootstrap(opts):
    sect("L4 rigd down, agent not loaded, plist installed: bootstrap")
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    flag = os.path.join(tmp, "started.flag")
    rigd = _Rigd(lambda: os.path.exists(flag))
    try:
        bindir, log, _f, opener = _stubs(tmp, loaded=False)
        plist = os.path.join(tmp, "Launch Agents", "agent.plist")
        _write(plist, "<plist/>")
        p, _dt = _run(tmp, rigd.url, bindir, opener, plist,
                      os.path.join(tmp, "rigd-launchd.log"))
        calls = _calls(log)
        boot = [c for c in calls if c.startswith("launchctl bootstrap")]
        check("L4 exits 0 once rigd answers", p.returncode == 0,
              p.stderr[-300:])
        check("L4 bootstraps the installed plist (path with a space intact)",
              len(boot) == 1 and boot[0].endswith(" " + plist), repr(calls))
        check("L4 then opens the UI", calls[-1:] == ["open " + rigd.url],
              repr(calls))
    finally:
        rigd.close()
        shutil.rmtree(tmp, ignore_errors=True)


def _l5_not_installed(opts):
    sect("L5 no plist: nothing started, the message names deploy.sh mac")
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    rigd = _Rigd(lambda: False)
    try:
        bindir, log, _f, opener = _stubs(tmp, loaded=False)
        plist = os.path.join(tmp, "missing.plist")
        p, dt = _run(tmp, rigd.url, bindir, opener, plist,
                     os.path.join(tmp, "rigd-launchd.log"))
        calls = _calls(log)
        check("L5 exits 1", p.returncode == 1, "rc=%r" % p.returncode)
        check("L5 says the agent is not installed and how to install it",
              plist in p.stderr and "deploy/deploy.sh mac" in p.stderr
              and "3a" in p.stderr, p.stderr[-400:])
        check("L5 nothing bootstrapped, kickstarted or opened",
              not any(c.startswith(("launchctl bootstrap",
                                    "launchctl kickstart", "open"))
                      for c in calls), repr(calls))
        check("L5 without waiting out the timeout", dt < 2, "%.1f s" % dt)
    finally:
        rigd.close()
        shutil.rmtree(tmp, ignore_errors=True)


def _l6_never_up(opts):
    sect("L6 rigd never answers: state, exit 78 hint, log tail; nothing opened")
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    rigd = _Rigd(lambda: False)
    try:
        body = ("\tstate = spawn scheduled\n\tprogram = /x/python3\n"
                "\tlast exit code = 78\n")
        bindir, log, _f, opener = _stubs(tmp, loaded=True, print_body=body)
        logfile = os.path.join(tmp, "rigd-launchd.log")
        _write(logfile, "rigd -> http://0.0.0.0:9090\n\n%s\n%s\n\n"
               % (PYDEPS_LINE, PYDEPS_LINE))
        p, dt = _run(tmp, rigd.url, bindir, opener,
                     os.path.join(tmp, "agent.plist"), logfile, wait_s=1)
        calls = _calls(log)
        err = p.stderr
        check("L6 exits 1 after the wait", p.returncode == 1 and dt < 10,
              "rc=%r %.1f s" % (p.returncode, dt))
        check("L6 nothing opened", not any(c.startswith("open")
                                           for c in calls), repr(calls))
        check("L6 names the URL, launchd's state and last exit code",
              rigd.url in err and "spawn scheduled" in err
              and "last exit code 78" in err, err[-600:])
        check("L6 the exit-78 hint points at requirements.txt",
              "requirements.txt" in err and "refused its Python" in err,
              err[-600:])
        check("L6 the log's last line (rig/pydeps.py's) is shown",
              PYDEPS_LINE in err, err[-600:])
        check("L6 the restart command is offered, not run",
              "launchctl kickstart -k gui/" in err
              and not any(" -k" in c for c in calls), repr(calls))
    finally:
        rigd.close()
        shutil.rmtree(tmp, ignore_errors=True)


def _l7_wedged(opts):
    sect("L7 a wedged rigd (accepts, never answers): the wait is bounded")
    tmp = tempfile.mkdtemp(prefix="audit_launcher_")
    import socket
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(64)                  # the kernel completes handshakes; no reply
    url = "http://127.0.0.1:%d" % srv.getsockname()[1]
    try:
        bindir, log, _f, opener = _stubs(tmp, loaded=True,
                                         print_body="\tstate = running\n")
        p, dt = _run(tmp, url, bindir, opener,
                     os.path.join(tmp, "agent.plist"),
                     os.path.join(tmp, "rigd-launchd.log"), wait_s=2)
        calls = _calls(log)
        # WAIT_S=2: one 2 s probe before, about WAIT_S of waiting, one after.
        # Counting passes instead (the first version) took ~14 s here and
        # ~104 s at the default 20.
        check("L7 gives up within the bound, not per-probe x passes",
              p.returncode == 1 and dt < 10, "rc=%r %.1f s"
              % (p.returncode, dt))
        check("L7 kickstart is a no-op on a running job: no -k, no bootout",
              not any(" -k" in c or "bootout" in c for c in calls),
              repr(calls))
    finally:
        srv.close()
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
def suite(opts):
    _l1_bundle(opts)
    _l2_up(opts)
    _l3_kickstart(opts)
    _l4_bootstrap(opts)
    _l5_not_installed(opts)
    _l6_never_up(opts)
    _l7_wedged(opts)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    _t0 = time.time()
    suite(opts)
    print("\naudit_launcher: %d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - _t0))
    sys.exit(1 if soaktest.FAIL else 0)

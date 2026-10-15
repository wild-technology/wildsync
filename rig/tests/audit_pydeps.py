#!/usr/bin/env python3
"""Audit regression suite - the host interpreter's Python packages (Y1..Y4).

2026-10-16, Mac host: Homebrew moved /opt/homebrew/bin/python3 from 3.14 to
3.15 under the launchd agent. The new interpreter had none of requirements.txt
(pyserial, Pillow, numpy, opencv-python-headless), every one of them is
imported guardedly, and launchd reported the job "running" while nothing in
~/rig/rigd-launchd.log or ~/rig/rigd.jsonl named the cause.

Y1  rig/pydeps.py: requirements.txt is read (pip names mapped to the module
    each imports as), a module that will not import - absent OR broken - is
    reported, the report is ONE line naming the package, the interpreter and
    the docs/INSTALL.md 3a fix, and require() exits 78.
Y2  `python3 rigd.py` on an interpreter missing a package exits 78 within
    seconds with that line, before it binds its port or writes the journal;
    `import rigd` (tests, devrig) is not affected.
Y3  `deploy.sh mac` runs the check under the interpreter the rendered plist
    names, BEFORE launchctl bootout/bootstrap, and on a refusal leaves the
    installed plist and the running agent untouched.
Y4  `deploy.sh jetson` runs it under the unit's ExecStart interpreter before
    the unit is written or restarted; rigd.service does not restart on 78.

None of this needs the packages to be missing at test time: imports are
monkeypatched in-process, and the subprocess cases block or fake a module with
sys.modules / a PYTHONPATH of stub modules. deploy.sh runs from a temp copy of
the tree with launchctl, sudo, systemctl, uname and sleep stubbed on PATH, so
nothing on this machine is installed, restarted or written outside a temp dir.

Run standalone:  python3 rig/tests/audit_pydeps.py
"""

import io
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
REPO = os.path.dirname(RIG)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

import soaktest                                          # noqa: E402
from soaktest import check, sect                         # noqa: E402
import pydeps                                            # noqa: E402

REQ = os.path.join(REPO, "requirements.txt")
EXPECTED = {"pyserial": "serial", "Pillow": "PIL", "numpy": "numpy",
            "opencv-python-headless": "cv2"}


def _blocking_importer(blocked, exc=None):
    def imp(mod):
        if mod in blocked:
            if exc is not None:
                raise exc
            raise ModuleNotFoundError("No module named %r" % mod)
        return object()
    return imp


# ---------------------------------------------------------------------------
def _y1_module(opts):
    sect("Y1 pydeps: requirements.txt, the missing list, the one line, exit 78")
    reqs = pydeps.requirements(REQ)
    check("Y1 requirements.txt maps every package to the module it imports as",
          dict(reqs) == EXPECTED, repr(reqs))
    check("Y1 FALLBACK lists the same packages as requirements.txt",
          sorted(pydeps.FALLBACK) == sorted(EXPECTED), repr(pydeps.FALLBACK))
    check("Y1 an unreadable requirements file falls back to FALLBACK",
          dict(pydeps.requirements(os.path.join(REPO, "no-such-file")))
          == EXPECTED)
    check("Y1 requirement lines with versions/markers/comments parse",
          [pydeps._pip_name(x) for x in
           ("numpy>=1.24  # x", "pyserial ; sys_platform != 'win32'",
            "  # only a comment", "-r other.txt", "Pillow[extra]==10")]
          == ["numpy", "pyserial", None, None, "Pillow"])

    miss = pydeps.missing(reqs, _blocking_importer({"serial"}))
    check("Y1 one absent module is reported, the others are not",
          [m[:2] for m in miss] == [("pyserial", "serial")], repr(miss))
    check("Y1 nothing missing -> []",
          pydeps.missing(reqs, _blocking_importer(set())) == [])
    broken = pydeps.missing(reqs, _blocking_importer(
        {"cv2"}, ImportError("numpy.core.multiarray failed to import")))
    check("Y1 a present-but-broken module counts as missing",
          [m[1] for m in broken] == ["cv2"], repr(broken))
    allmiss = pydeps.missing(reqs, _blocking_importer(set(EXPECTED.values())))
    check("Y1 an empty interpreter is missing all four",
          len(allmiss) == 4, repr(allmiss))

    line = pydeps.message(miss, "rigd", executable="/opt/homebrew/bin/python3",
                          version="3.15", in_venv=False, platform="darwin")
    check("Y1 the message is one line", "\n" not in line, line)
    check("Y1 it names the package, the import name and the interpreter",
          "pyserial (import serial)" in line
          and "/opt/homebrew/bin/python3 (Python 3.15)" in line, line)
    check("Y1 not in a venv on a Mac: the fix is the ~/rig/venv of INSTALL 3a",
          "docs/INSTALL.md 3a" in line and "~/rig/venv" in line
          and "requirements.txt" in line, line)
    vline = pydeps.message(miss, "rigd", executable="/u/rig/venv/bin/python3",
                           version="3.14", in_venv=True, platform="darwin")
    check("Y1 in a venv: the fix is that venv's own pip install -r",
          "/u/rig/venv/bin/python3 -m pip install -r" in vline
          and "3a" in vline, vline)
    jline = pydeps.message(miss, "rigd", executable="/usr/bin/python3",
                           version="3.10", in_venv=False, platform="linux")
    check("Y1 on Linux: the fix is deploy.sh jetson's apt packages",
          "deploy/deploy.sh jetson" in jline and "python3-serial" in jline,
          jline)
    bline = pydeps.message(broken, "rigd", executable="p", version="3.14",
                           in_venv=True, platform="darwin")
    check("Y1 a broken module carries its error, a missing one does not",
          "numpy.core.multiarray failed to import" in bline
          and "No module named" not in line, bline)

    err = io.StringIO()
    code = None
    try:
        pydeps.require("rigd", stream=err, reqs=reqs,
                       importer=_blocking_importer({"PIL"}), environ={})
    except SystemExit as e:
        code = e.code
    out = err.getvalue()
    check("Y1 require() exits 78 (EX_CONFIG) when a module is missing",
          code == 78 == pydeps.EXIT_CODE, repr(code))
    check("Y1 ... having written exactly one line naming it",
          out.count("\n") == 1 and "Pillow (import PIL)" in out, out)
    err = io.StringIO()
    got = pydeps.require("rigd", stream=err, reqs=reqs,
                         importer=_blocking_importer(set()), environ={})
    check("Y1 require() with everything importable returns [] and is silent",
          got == [] and err.getvalue() == "", err.getvalue())
    err = io.StringIO()
    got = pydeps.require("rigd", stream=err, reqs=reqs,
                         importer=_blocking_importer({"serial"}),
                         environ={pydeps.SKIP_ENV: "1"})
    check("Y1 WILDSYNC_SKIP_PYDEPS=1 returns (degraded) but still prints",
          len(got) == 1 and "pyserial" in err.getvalue()
          and pydeps.SKIP_ENV in err.getvalue(), err.getvalue())


# ---------------------------------------------------------------------------
def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _port_bound(port):
    s = socket.socket()
    s.settimeout(0.3)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


# Run rigd.py exactly as launchd/systemd do (as __main__, from rig/), with one
# module made unimportable the way an empty site-packages makes it: a None in
# sys.modules raises ModuleNotFoundError on import.
_SHIM = r"""
import runpy, sys
sys.modules[%r] = None
sys.path.insert(0, %r)
sys.argv = [%r]
runpy.run_path(%r, run_name=%r)
"""


def _y2_rigd_script(opts):
    sect("Y2 python3 rigd.py exits 78 with the line, before it binds or logs")
    tmp = tempfile.mkdtemp(prefix="audit_pydeps_")
    try:
        home = os.path.join(tmp, "home")
        os.makedirs(os.path.join(home, "rig"))
        with open(os.path.join(home, "rig", "nodes.json"), "w") as fh:
            fh.write("[]")
        port = _free_port()
        env = dict(os.environ, HOME=home, RIGD_PORT=str(port),
                   WILDSYNC_NODES=os.path.join(home, "rig", "nodes.json"))
        env.pop(pydeps.SKIP_ENV, None)
        rigd_py = os.path.join(RIG, "rigd.py")
        t0 = time.time()
        p = subprocess.Popen(
            [sys.executable, "-c",
             _SHIM % ("serial", RIG, rigd_py, rigd_py, "__main__")],
            cwd=RIG, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True)
        try:
            out, err = p.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            p.kill()
            out, err = p.communicate()
        dt = time.time() - t0
        check("Y2 rigd.py with pyserial unimportable exits 78",
              p.returncode == 78, "rc=%r stderr=%s" % (p.returncode, err[-400:]))
        check("Y2 ... within seconds, not a hang", dt < 10, "%.1f s" % dt)
        lines = [ln for ln in err.splitlines() if ln.strip()]
        check("Y2 ... with ONE stderr line naming pyserial and INSTALL 3a",
              len(lines) == 1 and "pyserial (import serial)" in lines[0]
              and "docs/INSTALL.md 3a" in lines[0], repr(lines))
        check("Y2 ... and nothing on stdout (it never reached main())",
              out.strip() == "", out[:200])
        check("Y2 the journal was never written",
              not os.path.exists(os.path.join(home, "rig", "rigd.jsonl")),
              repr(os.listdir(os.path.join(home, "rig"))))
        check("Y2 the port was never bound", not _port_bound(port))

        # Imported as a module (audit_rigd, devrig) the check does not run:
        # the same blocked module, `import rigd` succeeds.
        p = subprocess.run(
            [sys.executable, "-c",
             "import sys; sys.modules['serial'] = None; "
             "sys.path.insert(0, %r); import rigd; print('imported')" % RIG],
            cwd=RIG, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True, timeout=60)
        check("Y2 `import rigd` is not gated (tests and devrig import it)",
              p.returncode == 0 and "imported" in p.stdout,
              "rc=%r %s" % (p.returncode, p.stderr[-300:]))
        with open(rigd_py) as fh:
            src = fh.read()
        i_check = src.find("pydeps.require(")
        i_core = src.find("\nimport rigcore")
        check("Y2 the check precedes every rig import in rigd.py",
              0 < i_check < i_core, "check@%d rigcore@%d" % (i_check, i_core))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# deploy.sh, run for real from a temp copy of the tree with the outside world
# stubbed: every stub appends one line to LOG, so the order of the check and
# the launchctl/systemctl calls is observable.
_STUB = """#!/bin/sh
echo "%s $*" >> "$WS_AUDIT_LOG"
%s
"""
_STUBS = {
    "launchctl": 'case "$1" in print) echo "    state = running";; esac\nexit 0',
    "uname": "echo Darwin",
    "sleep": "exit 0",
    "systemctl": "exit 0",        # unprivileged `systemctl is-active rigd`
    # sudo runs nothing: it logs, and swallows stdin for `sudo tee <file>`.
    "sudo": 'case "$1" in tee) cat >/dev/null;; esac\nexit 0',
    "hostname": "echo 127.0.0.1",
}
# The interpreter the unit runs: logs, then the real python with the stub
# modules ahead of site-packages.
_PY_STUB = """#!/bin/sh
echo "python $*" >> "$WS_AUDIT_LOG"
PYTHONPATH="$WS_AUDIT_MODS" exec %s "$@"
"""


def _write(path, text, mode=None):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(path, "w") as fh:
        fh.write(text)
    if mode:
        os.chmod(path, mode)


def _mods(root, block):
    """A PYTHONPATH dir faking all four packages, or blocking `block`."""
    d = os.path.join(root, "mods-" + (block or "all"))
    for mod in EXPECTED.values():
        path = (os.path.join(d, mod, "__init__.py") if mod == "PIL"
                else os.path.join(d, mod + ".py"))
        body = ("raise ModuleNotFoundError(\"No module named '%s'\")\n" % mod
                if mod == block else "")
        _write(path, body)
    return d


def _deploy_tree(tmp, interp):
    """A minimal copy of the tree deploy.sh needs; rigd.service's ExecStart
    rewritten to the stub interpreter."""
    tree = os.path.join(tmp, "tree")
    for rel in ("deploy/deploy.sh", "deploy/rigd.launchd.plist",
                "rig/pydeps.py", "requirements.txt"):
        dst = os.path.join(tree, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(os.path.join(REPO, rel), dst)
    with open(os.path.join(REPO, "deploy", "rigd.service")) as fh:
        unit = fh.read()
    _write(os.path.join(tree, "deploy", "rigd.service"),
           unit.replace("ExecStart=/usr/bin/python3",
                        "ExecStart=" + interp))
    return tree


def _run_deploy(tmp, what, block):
    home = os.path.join(tmp, "home")
    bindir = os.path.join(tmp, "bin")
    log = os.path.join(tmp, "calls.log")
    interp = os.path.join(home, "rig", "venv", "bin", "python3")
    for name, body in _STUBS.items():
        _write(os.path.join(bindir, name), _STUB % (name, body), 0o755)
    _write(interp, _PY_STUB % sys.executable, 0o755)
    tree = _deploy_tree(tmp, interp)
    env = dict(os.environ, HOME=home, WS_AUDIT_LOG=log,
               WS_AUDIT_MODS=_mods(tmp, block), WILDSYNC_NTP="",
               PATH=bindir + os.pathsep + os.environ.get("PATH", ""))
    env.pop(pydeps.SKIP_ENV, None)
    p = subprocess.run(["bash", os.path.join(tree, "deploy", "deploy.sh"),
                        what], cwd=tmp, env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True,
                       timeout=120)
    try:
        with open(log) as fh:
            calls = [ln.rstrip("\n") for ln in fh]
    except OSError:
        calls = []
    return p, calls, home


def _first(calls, prefix):
    for i, c in enumerate(calls):
        if c.startswith(prefix):
            return i
    return -1


def _y3_deploy_mac(opts):
    sect("Y3 deploy.sh mac checks the plist's interpreter before launchctl")
    label = "org.wildtechnology.wildsync.rigd.plist"
    tmp = tempfile.mkdtemp(prefix="audit_pydeps_mac_")
    try:
        # Refused: pyserial blocked. An agent is already installed.
        plist = os.path.join(tmp, "home", "Library", "LaunchAgents", label)
        _write(plist, "<the plist already in place>")
        p, calls, home = _run_deploy(tmp, "mac", "serial")
        check("Y3 refused: deploy.sh mac exits nonzero", p.returncode != 0,
              "rc=%r" % p.returncode)
        check("Y3 refused: the check ran under the unit's interpreter",
              _first(calls, "python ") >= 0
              and "pydeps.py" in calls[_first(calls, "python ")], repr(calls))
        check("Y3 refused: launchctl was never called (agent untouched)",
              _first(calls, "launchctl") < 0, repr(calls))
        check("Y3 refused: stderr carries the line naming pyserial and 3a",
              "pyserial (import serial)" in p.stderr
              and "docs/INSTALL.md 3a" in p.stderr, p.stderr[-600:])
        with open(plist) as fh:
            kept = fh.read()
        check("Y3 refused: the installed plist is left as it was",
              kept == "<the plist already in place>", kept[:80])
        check("Y3 refused: no rendered plist is left beside it",
              not os.path.exists(plist + ".new"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    tmp = tempfile.mkdtemp(prefix="audit_pydeps_mac_")
    try:
        p, calls, home = _run_deploy(tmp, "mac", None)
        plist = os.path.join(home, "Library", "LaunchAgents", label)
        ichk = _first(calls, "python ")
        iout = _first(calls, "launchctl bootout")
        iboot = _first(calls, "launchctl bootstrap")
        check("Y3 accepted: deploy.sh mac exits 0", p.returncode == 0,
              "rc=%r %s" % (p.returncode, p.stderr[-400:]))
        check("Y3 accepted: check, then bootout, then bootstrap",
              0 <= ichk < iout < iboot, repr(calls))
        try:
            with open(plist) as fh:
                text = fh.read()
        except OSError:
            text = ""
        check("Y3 accepted: the plist is installed, rendered for this home",
              os.path.join(home, "rig", "venv", "bin", "python3") in text
              and "__HOME__" not in text, text[:120])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _y4_deploy_jetson(opts):
    sect("Y4 deploy.sh jetson checks ExecStart's interpreter before restart")
    with open(os.path.join(REPO, "deploy", "rigd.service")) as fh:
        unit = fh.read()
    check("Y4 rigd.service does not restart on exit 78",
          "RestartPreventExitStatus=78" in unit)
    tmp = tempfile.mkdtemp(prefix="audit_pydeps_jet_")
    try:
        p, calls, _home = _run_deploy(tmp, "jetson", "cv2")
        check("Y4 refused: deploy.sh jetson exits nonzero", p.returncode != 0,
              "rc=%r" % p.returncode)
        check("Y4 refused: stderr names opencv-python-headless (import cv2)",
              "opencv-python-headless (import cv2)" in p.stderr,
              p.stderr[-600:])
        check("Y4 refused: the unit was neither written nor restarted",
              not any("systemd/system/rigd.service" in c
                      or "restart rigd" in c for c in calls), repr(calls))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    tmp = tempfile.mkdtemp(prefix="audit_pydeps_jet_")
    try:
        p, calls, _home = _run_deploy(tmp, "jetson", None)
        ichk = _first(calls, "python ")
        irst = _first(calls, "sudo systemctl restart rigd")
        check("Y4 accepted: deploy.sh jetson exits 0", p.returncode == 0,
              "rc=%r %s" % (p.returncode, p.stderr[-400:]))
        check("Y4 accepted: the check runs before systemctl restart rigd",
              0 <= ichk < irst, repr(calls))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
def suite(opts):
    _y1_module(opts)
    _y2_rigd_script(opts)
    if shutil.which("bash"):
        _y3_deploy_mac(opts)
        _y4_deploy_jetson(opts)
    else:
        check("Y3/Y4 need bash to run deploy.sh", False, "no bash on PATH")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    _t0 = time.time()
    suite(opts)
    print("\naudit_pydeps: %d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - _t0))
    sys.exit(1 if soaktest.FAIL else 0)

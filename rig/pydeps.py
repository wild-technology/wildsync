#!/usr/bin/env python3
"""pydeps - refuse to run rigd on an interpreter that lacks requirements.txt.

rigd imports pyserial, Pillow, numpy and OpenCV guardedly, so an interpreter
that has none of them does not crash: it comes up with nav, the EXIF fallback
and the Calib tab quietly dead. That is how the Mac host ran on 2026-10-16
after Homebrew moved /opt/homebrew/bin/python3 from 3.14 to 3.15 under the
launchd agent: a new interpreter with an empty site-packages, launchd reporting
the job "running", and nothing in ~/rig/rigd-launchd.log or ~/rig/rigd.jsonl
that named the cause. A host interpreter without the packages is a broken
install, not a degraded mode, so it is refused up front with ONE line that
names what is missing and how to fix it:

  * rigd.py runs require() before it imports anything else of the rig's, and
    exits EXIT_CODE (78, EX_CONFIG) with that line on stderr - the launchd log
    on a Mac, journalctl on a Jetson (deploy/rigd.service does not restart on
    that status, so the unit lands in "failed" with the line beside it);
  * deploy/deploy.sh mac / jetson run this file under the interpreter the
    unit will run, before launchctl bootstrap / systemctl restart, and refuse
    the deploy with the same line, leaving the running rigd as it was.

The module list is read from requirements.txt at the top of the tree (pip
names mapped to the names they import as), so adding a package there is
enough to have it checked; FALLBACK covers a deployment without the file.

WILDSYNC_SKIP_PYDEPS=1 starts rigd anyway (the line is still printed), for a
development box that wants the degraded behaviour on purpose.

    python3 rig/pydeps.py      # exit 0 silently, else 78 and the line on stderr

Standard library only, Python 3.10-safe.
"""

import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REQUIREMENTS = os.path.join(os.path.dirname(HERE), "requirements.txt")

# pip distribution name (lower-cased) -> the module it is imported as. A name
# not listed here is imported as itself with '-' read as '_'.
IMPORT_NAME = {
    "pyserial": "serial",
    "pillow": "PIL",
    "opencv-python-headless": "cv2",
    "opencv-python": "cv2",
    "numpy": "numpy",
}
FALLBACK = ("pyserial", "Pillow", "numpy", "opencv-python-headless")

EXIT_CODE = 78          # EX_CONFIG, sysexits.h
SKIP_ENV = "WILDSYNC_SKIP_PYDEPS"


def _pip_name(line):
    """The distribution name on one requirements.txt line, or None."""
    line = line.split("#", 1)[0].strip()
    if not line or line.startswith("-"):
        return None
    for i, ch in enumerate(line):
        if ch in "<>=!~;[ \t@":
            line = line[:i]
            break
    return line or None


def requirements(path=None):
    """[(pip_name, module)] in requirements.txt order; FALLBACK when the file
    is unreadable or lists nothing."""
    names = []
    try:
        with open(path or REQUIREMENTS) as fh:
            for raw in fh:
                n = _pip_name(raw)
                if n and n not in names:
                    names.append(n)
    except OSError:
        pass
    if not names:
        names = list(FALLBACK)
    return [(n, IMPORT_NAME.get(n.lower(), n.replace("-", "_")))
            for n in names]


def missing(reqs=None, importer=None):
    """[(pip_name, module, error)] for every requirement that will not import.
    Anything an import raises counts: a module that is present but broken (an
    OpenCV wheel built against another numpy) is as unusable as an absent
    one."""
    importer = importer or importlib.import_module
    out = []
    for pip_name, mod in (requirements() if reqs is None else reqs):
        try:
            importer(mod)
        except Exception as e:                       # noqa: BLE001
            out.append((pip_name, mod, "%s: %s" % (type(e).__name__, e)))
    return out


def _fix(executable, in_venv, platform):
    req = REQUIREMENTS
    if in_venv:
        return ("install them into this venv: %s -m pip install -r %s "
                "(docs/INSTALL.md 3a)" % (executable, req))
    if platform == "darwin":
        return ("run rigd from the venv of docs/INSTALL.md 3a: "
                "python3 -m venv ~/rig/venv && ~/rig/venv/bin/python3 -m pip "
                "install -r %s, then deploy/deploy.sh mac" % req)
    return ("install them for this interpreter: deploy/deploy.sh jetson "
            "(apt python3-serial python3-pil python3-numpy python3-opencv, "
            "docs/INSTALL.md 3b), or a venv as in docs/INSTALL.md 3a")


def message(miss, prog="rigd", executable=None, version=None, in_venv=None,
            platform=None):
    """The one line: who, which interpreter, what is missing, the fix."""
    executable = executable or sys.executable or "python3"
    version = version or "%d.%d" % sys.version_info[:2]
    if in_venv is None:
        in_venv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    platform = platform or sys.platform
    parts = []
    for pip_name, mod, err in miss:
        p = pip_name if pip_name == mod else "%s (import %s)" % (pip_name, mod)
        # "No module named x" says nothing the name does not; anything else is
        # a broken install and the reason is the useful part.
        if err and not err.startswith("ModuleNotFoundError"):
            p += " [%s]" % err.replace("\n", " ")[:160]
        parts.append(p)
    return ("%s: %s (Python %s) cannot import %s - %s"
            % (prog, executable, version, ", ".join(parts),
               _fix(executable, in_venv, platform)))


def require(prog="rigd", stream=None, reqs=None, importer=None, environ=None):
    """Return [] when every requirement imports. Otherwise print the line and
    raise SystemExit(EXIT_CODE), unless SKIP_ENV is set (then print it with a
    note and return what is missing)."""
    miss = missing(reqs, importer)
    if not miss:
        return []
    stream = stream or sys.stderr
    environ = os.environ if environ is None else environ
    line = message(miss, prog)
    if environ.get(SKIP_ENV, "").strip() not in ("", "0"):
        stream.write("%s (%s is set: starting anyway, degraded)\n"
                     % (line, SKIP_ENV))
        stream.flush()
        return miss
    stream.write(line + "\n")
    stream.flush()
    raise SystemExit(EXIT_CODE)


def main():
    # Never honour the skip here: deploy.sh asks this to decide whether the
    # unit may be (re)started, and an operator's shell variable must not turn
    # that into a yes.
    require("rigd", environ={})
    return 0


if __name__ == "__main__":
    sys.exit(main())

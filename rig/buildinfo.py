#!/usr/bin/env python3
"""buildinfo — which Wild Sync software is running: version, commit, dirty.

Every run records the software that produced it (run.json "software", one
events.log line at run start), so a transect can be traced to the exact tree
that recorded it. rigd resolves the stamp ONCE, at start-up, from the checkout
it is running from:

  1. git, when this tree is a git checkout and git answers within a short
     timeout: HEAD, `git describe --tags --always --dirty`, and dirty =
     tracked files differ from HEAD (untracked files do not count, the same
     rule as `describe --dirty`);
  2. else BUILD_INFO.json at the top of the tree, written from git when the
     tree was deployed or packed (deploy/deploy.sh does this; so does
     `python3 rig/buildinfo.py --write BUILD_INFO.json`);
  3. else "unknown".

`source` says which of the three answered. Nothing here can raise or block for
long: every git call is bounded, every failure falls through to the next
source, and a run never waits on it.

    python3 rig/buildinfo.py                       # print the stamp
    python3 rig/buildinfo.py --write BUILD_INFO.json

Stdlib only.
"""

import json
import os
import subprocess
import sys
import threading
import time

# The release this tree is. Bumped by hand at release time; git describe says
# where the tree actually sits relative to the tags.
VERSION = "1.1.0"

UNKNOWN = "unknown"
BUILD_INFO_NAME = "BUILD_INFO.json"
GIT_TIMEOUT_S = 2.0

# The top of the tree this file belongs to: rig/ lives directly under it, both
# in a checkout and in the ~/wildsync tree deploy.sh lays down on a node.
TREE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git(tree, args, timeout):
    """stdout of one bounded git call, or None on any failure."""
    env = dict(os.environ)
    # `describe --dirty` refreshes the index; without this it may take
    # index.lock and collide with an operator's git command in the same tree.
    env["GIT_OPTIONAL_LOCKS"] = "0"
    env["LC_ALL"] = "C"
    for k in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE"):
        env.pop(k, None)
    try:
        p = subprocess.run(["git", "-C", tree] + list(args),
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, timeout=timeout,
                           env=env, check=False)
    except (OSError, ValueError, subprocess.SubprocessError):
        return None
    if p.returncode != 0:
        return None
    return p.stdout.decode("utf-8", "replace").strip()


def from_git(tree=TREE, timeout=GIT_TIMEOUT_S):
    """The stamp from git, or None when this tree is not its own checkout.

    The toplevel check matters: a copy of rig/ dropped inside some OTHER
    repository would otherwise be stamped with that repository's commit."""
    out = _git(tree, ["rev-parse", "--show-toplevel", "HEAD"], timeout)
    if not out:
        return None
    lines = out.splitlines()
    if len(lines) != 2:
        return None
    top, commit = lines[0].strip(), lines[1].strip()
    try:
        if os.path.realpath(top) != os.path.realpath(tree):
            return None
    except (OSError, ValueError):
        return None
    if len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
        return None
    desc = _git(tree, ["describe", "--tags", "--always", "--dirty"], timeout)
    return {"version": VERSION, "commit": commit,
            "dirty": desc.endswith("-dirty") if desc else None,
            "describe": desc or UNKNOWN, "source": "git"}


def from_file(path):
    """The stamp recorded in a BUILD_INFO.json, or None."""
    try:
        with open(path) as fh:
            doc = json.load(fh)
    except (OSError, ValueError):
        return None
    if not isinstance(doc, dict):
        return None
    commit = doc.get("commit")
    if not isinstance(commit, str) or not commit:
        return None
    dirty = doc.get("dirty")
    return {"version": str(doc.get("version") or VERSION),
            "commit": commit,
            "dirty": dirty if isinstance(dirty, bool) else None,
            "describe": str(doc.get("describe") or UNKNOWN),
            "source": "build-info"}


def unknown():
    return {"version": VERSION, "commit": UNKNOWN, "dirty": None,
            "describe": UNKNOWN, "source": "none"}


def resolve(tree=TREE, timeout=GIT_TIMEOUT_S):
    """git, then BUILD_INFO.json, then "unknown". Never raises."""
    try:
        return (from_git(tree, timeout)
                or from_file(os.path.join(tree, BUILD_INFO_NAME))
                or unknown())
    except Exception:  # noqa: BLE001 - a stamp must never stop rigd
        return unknown()


_STAMP = None
_STAMP_LOCK = threading.Lock()


def stamp():
    """The process-wide stamp: resolved on first call, then reused.

    rigd calls this at start-up, so a run start only ever copies a dict."""
    global _STAMP
    with _STAMP_LOCK:
        if _STAMP is None:
            _STAMP = resolve()
        return dict(_STAMP)


def describe(s):
    """One line for events.log."""
    return ("wildsync %s, commit %s%s (%s), from %s"
            % (s.get("version"), s.get("commit"),
               "" if s.get("dirty") is not True else " + local changes",
               s.get("describe"), s.get("source")))


def write(path, tree=TREE, timeout=GIT_TIMEOUT_S):
    """Write BUILD_INFO.json from git. Returns the stamp, or None when this
    tree is not a git checkout (nothing is written then)."""
    s = from_git(tree, timeout)
    if s is None:
        return None
    doc = {k: s[k] for k in ("version", "commit", "dirty", "describe")}
    doc["written"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(doc, fh, indent=1)
        fh.write("\n")
    os.replace(tmp, path)
    return s


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--write", metavar="PATH",
                    help="write BUILD_INFO.json for this tree, from git")
    a = ap.parse_args()
    if a.write:
        s = write(a.write)
        if s is None:
            print("not a git checkout (or git unavailable): %s not written"
                  % a.write, file=sys.stderr)
            return 1
        print(describe(s))
        return 0
    print(json.dumps(resolve(), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

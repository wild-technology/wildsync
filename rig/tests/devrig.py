#!/usr/bin/env python3
"""devrig — the full rigd + web UI against two in-process fake nodes.

For UI work on a machine with no rig attached:

    python3 rig/tests/devrig.py [--port 9092] [--frames 6]

then open http://localhost:<port>. Everything is real except the cameras:
rigd, the settings engine, runs, pulls, the transect browser and the strobe
path all execute for real against rig/tests/fakenode.py.

Safety: the same netguard as soaktest wraps every HTTP entry point before any
rig code runs, so nothing here can reach the live fleet even by accident. On
macOS the fakes' 127.0.0.x aliases are remapped through fakenode.loopback_map
(Linux binds them natively).

State lives in a throwaway temp home (or --home) — never ~/rig."""

import argparse
import json
import os
import sys
import tempfile
import time
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)


def _install_guard():
    import urllib.request
    import fakenode as _fk

    def _remap(url):
        if not isinstance(url, str):
            return url
        u = urlparse(url)
        host, port = u.hostname or "", u.port
        if port is None or not host.startswith("127.") or host == "127.0.0.1":
            return url
        h2, p2 = _fk.loopback_map(host, port)
        if (h2, p2) == (host, port):
            return url
        return url.replace("//%s:%d" % (host, port),
                           "//%s:%d" % (h2, p2), 1)

    def _wrap(fn, label):
        def w(url, *a, **kw):
            host = urlparse(url if isinstance(url, str) else
                            getattr(url, "full_url", "")).hostname or ""
            if not host.startswith("127."):
                raise AssertionError(
                    "devrig netguard: refused %s call to %s (fakes only)"
                    % (label, url))
            return fn(_remap(url), *a, **kw)
        return w

    urllib.request.urlopen = _wrap(urllib.request.urlopen, "urlopen")
    import rigcore
    rigcore.http_json = _wrap(rigcore.http_json, "http_json")
    rigcore.http_bytes = _wrap(rigcore.http_bytes, "http_bytes")
    return rigcore


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9092)
    ap.add_argument("--frames", type=int, default=6,
                    help="seed frames per camera before rigd starts")
    ap.add_argument("--home", default="",
                    help="state dir (default: fresh temp dir)")
    a = ap.parse_args()

    home = a.home or tempfile.mkdtemp(prefix="wildsync-devrig-")
    nodes = [{"name": "cam1", "cam_num": 1, "host": "127.0.0.2"},
             {"name": "cam2", "cam_num": 2, "host": "127.0.0.3"}]
    npath = os.path.join(home, "nodes.json")
    with open(npath, "w") as fh:
        json.dump(nodes, fh)
    # Both read at import time, so they must be set before any rig import.
    os.environ["WILDSYNC_NODES"] = npath
    os.environ["RIGD_PORT"] = str(a.port)

    rigcore = _install_guard()
    rigcore.RIG_HOME = os.path.join(home, "rig")
    rigcore.RUNS_DIR = os.path.join(home, "runs")
    rigcore.DESIRED_PATH = os.path.join(rigcore.RIG_HOME, "desired.json")
    rigcore.RIGD_LOG = os.path.join(rigcore.RIG_HOME, "rigd.jsonl")
    os.makedirs(rigcore.RIG_HOME, exist_ok=True)

    from fakenode import FakeNode
    n1 = FakeNode("cam1", "127.0.0.2", cam_num=1, has_imu=True)
    n2 = FakeNode("cam2", "127.0.0.3", cam_num=2)
    now = time.time()
    for i in range(a.frames):
        ep = now - 60 + i * 2
        for n in (n1, n2):
            n.add_frame(epoch=ep)
    for n in (n1, n2):
        n.push_imu(epoch=now)

    import run as runmod
    runmod.RUNS_DIR = rigcore.RUNS_DIR

    print("devrig: fleet cam1=%s cam2=%s  home=%s" %
          (n1.host, n2.host, home))
    print("devrig: open http://localhost:%d" % a.port)
    import rigd
    try:
        rigd.main()
    finally:
        n1.close()
        n2.close()


if __name__ == "__main__":
    main()

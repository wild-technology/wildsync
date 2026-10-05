#!/usr/bin/env python3
"""Audit regression suite — run provenance.

What a transect on disk says about where it came from, and whether that can
be trusted after the fact:

P1  time_source. The host-clock label is "host". Runs recorded before v1.1.0
    wrote "jetson" for the same thing whatever the host was (the 2026-08-20
    transects were recorded on a Mac), so nothing may write "jetson" any more
    and every reader must still accept it: the run browser, ingest, and the
    helpers the two labels are tested through.

P2  software stamp. Every run records the code that made it: run.json
    "software" {version, commit, dirty, describe, source} and one events.log
    line, resolved once at start-up from git, else BUILD_INFO.json, else
    "unknown" - and nothing about resolving it may hang or fail a run.

Hermetic: in-process fakes on loopback only, temp run roots, never ~/rig or
~/rig-runs.

Run standalone:  python3 rig/tests/audit_provenance.py
"""

import csv
import io
import json
import os
import shutil
import sys
import tempfile
import time
import tokenize

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

import soaktest                                          # noqa: E402
from soaktest import check, sect, wait_for, Env, FakeNav  # noqa: E402
import rigcore                                           # noqa: E402
import run as runmod                                     # noqa: E402


def _tmp():
    return tempfile.mkdtemp(prefix="wildsync-audit-prov-")


def _read_flight(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def _short_run(label, specs=None, frames=2, nav=None):
    """One real run against fakes: start, land `frames` frames per camera,
    stop. Returns (env, result of start, run root). The caller closes env."""
    specs = specs or [("cam1", "127.0.0.2", 1), ("cam2", "127.0.0.3", 2)]
    env = Env(specs, poll=0.3, threaded=True,
              nav=nav if nav is not None else FakeNav(fix=False),
              imu_node="cam9")
    for spec in specs:
        env.wait_state(spec[0], "CAM_CONNECTED")
    r = env.runmgr.start({"label": label, "calibrate": False})
    root = r.get("root", "")
    t = time.time()
    for k in range(frames):
        for spec in specs:
            env.node(spec[0]).add_frame(epoch=t + k * 0.5)
    for spec in specs:
        fl = os.path.join(root, spec[0], "flight_log.csv")
        wait_for(lambda: os.path.exists(fl) and len(_read_flight(fl)) >= frames,
                 15)
    env.runmgr.stop()
    return env, r, root


# A run as the pre-v1.1.0 code left it on disk: the legacy "jetson" label, no
# software stamp, no simulated marker, the 8-column ingest manifest and no
# SHA256SUMS. Every reader must keep loading this shape.
LEGACY_MANIFEST_COLS = ["run_id", "cam", "card", "new_base", "epoch",
                        "residual_ms", "raw", "jpg"]


def write_legacy_run(runs_dir, rid="260820_1925_transect-01", shots=3):
    from fakenode import make_jpeg
    root = os.path.join(runs_dir, rid)
    base = 1787253940.0
    index = []
    for cam in (1, 2):
        d = os.path.join(root, "cam%d" % cam)
        os.makedirs(d, exist_ok=True)
        rows = []
        for i in range(shots):
            ep = base + i * 0.5 + (cam - 1) * 0.001
            fn = runmod._fmt_fname(cam, ep)
            with open(os.path.join(d, fn), "wb") as fh:
                fh.write(make_jpeg(ep))
            index.append({"cam": cam, "file": fn, "orig": "ILX%05d.JPG" % i,
                          "epoch": round(ep, 6), "src": "gpio_edge"})
            row = {k: "" for k in runmod.FLIGHT_HEADER}
            row.update({"filename": fn, "datetime": runmod._fmt_dt(ep),
                        "lat": "41.4237752", "long": "-71.4537835",
                        "capture_source": "gpio_edge",
                        "time_source": "jetson", "time_err_ms": "0.4"})
            rows.append(row)
        with open(os.path.join(d, "flight_log.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=runmod.FLIGHT_HEADER)
            w.writeheader()
            w.writerows(rows)
    with open(os.path.join(root, "index.jsonl"), "w") as fh:
        for e in index:
            fh.write(json.dumps(e) + "\n")
    doc = {"run_id": rid, "label": "transect-01", "started": base,
           "nodes": ["cam1", "cam2"], "config": {"interval_s": 0.5},
           "time": {"source": "jetson", "gps_offset_s": 0.0,
                    "live_source": "jetson", "live_gps_offset_s": 0.0},
           "frames": len(index), "index_jsonl": "index.jsonl",
           "index": index, "final": True, "written": base + 60}
    with open(os.path.join(root, "run.json"), "w") as fh:
        json.dump(doc, fh, indent=1)
    with open(os.path.join(root, "events.log"), "w") as fh:
        fh.write("19:25:40  INFO     run                run started: 2 cameras\n")
    with open(os.path.join(root, "ingest_manifest.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(LEGACY_MANIFEST_COLS)
        for e in index:
            w.writerow([rid, e["cam"], "DSC%05d" % index.index(e),
                        os.path.splitext(e["file"])[0], "%.6f" % e["epoch"],
                        "1.0", "linked", "linked"])
    return root, doc


def _string_literals(path):
    """Every string literal in a Python file, with its line number."""
    out = []
    with open(path, "rb") as fh:
        for tok in tokenize.tokenize(fh.readline):
            if tok.type == tokenize.STRING:
                out.append((tok.start[0], tok.string))
    return out


# ---------------------------------------------------------------------------
# P1 — time_source: "host" is written, "jetson" is still read
# ---------------------------------------------------------------------------
def _p1_time_source(opts):
    sect("P1 time_source: new runs write 'host'; the legacy 'jetson' still "
         "loads everywhere")
    ts = rigcore.TimeSync(rigcore.EventLog(path=os.path.join(_tmp(), "j.jsonl")))
    check("TimeSync starts on the host clock and calls it 'host'",
          ts.now()[1] == "host" and ts.source == "host", ts.source)
    ts.feed_gps(time.time() + 1.0)
    gps = ts.now()[1]
    ts.clear_gps()
    check("losing GPS falls back to 'host', never 'jetson'",
          gps == "gps" and ts.now()[1] == "host", "%s -> %s" % (gps, ts.source))
    import nav as navmod
    ta = navmod.TimeAuthority()
    check("nav's TimeAuthority reports 'host' with no GPS",
          ta.source() == "host" and ta.status()["source"] == "host",
          str(ta.status().get("source")))
    check("both labels are the host clock to a reader; gps is not",
          rigcore.is_host_time_source("host")
          and rigcore.is_host_time_source("jetson")
          and not rigcore.is_host_time_source("gps")
          and not rigcore.is_host_time_source(None))

    env, r, root = _short_run("p1-host")
    try:
        check("run start reports time_source 'host'",
              r.get("time_source") == "host", json.dumps(r)[:160])
        rows = _read_flight(os.path.join(root, "cam1", "flight_log.csv"))
        check("every flight_log row of a new run says 'host'",
              rows and all(x["time_source"] == "host" for x in rows),
              str(sorted({x["time_source"] for x in rows})))
        doc = json.load(open(os.path.join(root, "run.json")))
        check("run.json's time block says 'host' (applied and live)",
              (doc.get("time") or {}).get("source") == "host"
              and (doc.get("time") or {}).get("live_source") == "host",
              json.dumps(doc.get("time")))
        with open(os.path.join(root, "events.log")) as fh:
            ev = fh.read()
        check("events.log never carries the legacy label for a new run",
              "jetson" not in ev, ev[:200])
    finally:
        env.close()

    # Nothing in the rig's own modules may emit the legacy label. The two
    # LEGACY_* constants name it so readers can accept it, and ingest's
    # self-test feeds it as legacy data; any other "jetson" string literal
    # is a writer.
    allowed = {"rigcore.py": 1, "nav.py": 1, "ingest.py": 1}
    found = {}
    for fn in sorted(os.listdir(RIG)):
        if not fn.endswith(".py"):
            continue
        for ln, lit in _string_literals(os.path.join(RIG, fn)):
            if lit.strip("\"'") == "jetson":
                found.setdefault(fn, []).append(ln)
    extra = {fn: lines for fn, lines in found.items()
             if len(lines) > allowed.get(fn, 0)}
    check("no rig module writes 'jetson' (only the LEGACY_* names and "
          "ingest's legacy self-test data mention it)",
          not extra, json.dumps(found))
    with open(os.path.join(RIG, "rig_ui.html")) as fh:
        ui = fh.read()
    check("the UI never keys a decision on the legacy label",
          "==='jetson'" not in ui and "=='jetson'" not in ui
          and '==="jetson"' not in ui)

    # The legacy run on disk: listed, opened, read by ingest.
    runs = _tmp()
    saved = rigcore.RUNS_DIR
    rigcore.RUNS_DIR = runs
    try:
        root, _ = write_legacy_run(runs)
        rid = os.path.basename(root)
        rb = rigcore.RunBrowser()
        listing = {x["run_id"]: x for x in rb.list_runs()["runs"]}
        row = listing.get(rid) or {}
        check("a legacy 'jetson' run is listed with its label intact",
              row.get("time_source") == "jetson" and row.get("final") is True,
              json.dumps(row)[:200])
        det = rb.detail(rid)
        pc = det.get("per_camera") or {}
        check("a legacy run opens in the run browser with every row",
              sum((v or {}).get("rows", 0) for v in pc.values()) == 6
              and (det.get("pairs") or {}).get("complete") == 3,
              json.dumps({k: (v or {}).get("rows") for k, v in pc.items()}))
        import ingest
        doc, rows, flight = ingest.load_run(root)
        check("ingest loads a legacy run and its 'jetson' rows",
              sum(len(v) for v in rows.values()) == 6
              and {x["time_source"] for x in flight.values()} == {"jetson"},
              str(len(flight)))
    finally:
        rigcore.RUNS_DIR = saved
        shutil.rmtree(runs, ignore_errors=True)


# ---------------------------------------------------------------------------
# P2 — the software stamp
# ---------------------------------------------------------------------------
def _git(*args):
    import subprocess
    try:
        p = subprocess.run(["git", "-C", os.path.dirname(RIG)] + list(args),
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return p.stdout.decode().strip() if p.returncode == 0 else None


def _p2_software_stamp(opts):
    sect("P2 software stamp: run.json 'software' and one events.log line, "
         "from git, else BUILD_INFO.json, else 'unknown'")
    import subprocess
    import buildinfo
    s = buildinfo.resolve()
    head = _git("rev-parse", "HEAD")
    desc = _git("describe", "--tags", "--always", "--dirty")
    if head:
        check("in a checkout the stamp comes from git and names HEAD",
              s["source"] == "git" and s["commit"] == head, json.dumps(s))
        check("describe and dirty agree with git describe --dirty",
              s["describe"] == desc
              and s["dirty"] is desc.endswith("-dirty"),
              "%s vs %s" % (s["describe"], desc))
    else:
        soaktest.note("git unavailable here: the git branch of the software "
                      "stamp was not exercised")
    check("the stamp carries the release version",
          s["version"] == buildinfo.VERSION == "1.1.0", s["version"])

    # A tree with no .git: "unknown", then whatever BUILD_INFO.json says.
    tree = _tmp()
    os.makedirs(os.path.join(tree, "rig"))
    shutil.copy(os.path.join(RIG, "buildinfo.py"), os.path.join(tree, "rig"))
    u = buildinfo.resolve(tree)
    check("no .git and no BUILD_INFO.json -> 'unknown', not a guess",
          u["commit"] == "unknown" and u["describe"] == "unknown"
          and u["dirty"] is None and u["source"] == "none", json.dumps(u))
    if head:
        bi = os.path.join(tree, "BUILD_INFO.json")
        w = buildinfo.write(bi)
        f = buildinfo.resolve(tree)
        check("BUILD_INFO.json written from git is what a .git-less tree "
              "reports",
              w is not None and f["source"] == "build-info"
              and f["commit"] == head and f["describe"] == desc,
              json.dumps(f))
        with open(bi, "w") as fh:
            fh.write("{torn")
        check("a corrupt BUILD_INFO.json falls through to 'unknown'",
              buildinfo.resolve(tree)["source"] == "none")

        # rig/ copied inside some OTHER repository must not take its commit.
        outer = _tmp()
        subprocess.run(["git", "init", "-q", outer],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["git", "-C", outer, "-c", "user.name=t",
                        "-c", "user.email=t@example.invalid", "commit", "-q",
                        "--allow-empty", "-m", "outer"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        inner = os.path.join(outer, "vendor", "wildsync")
        os.makedirs(os.path.join(inner, "rig"))
        check("a tree nested inside another repository is not stamped with "
              "that repository's commit",
              buildinfo.from_git(inner) is None
              and buildinfo.resolve(inner)["source"] == "none",
              json.dumps(buildinfo.resolve(inner)))
        shutil.rmtree(outer, ignore_errors=True)
    shutil.rmtree(tree, ignore_errors=True)

    # A git that hangs: bounded by the timeout, then the fallback.
    fake = _tmp()
    with open(os.path.join(fake, "git"), "w") as fh:
        fh.write("#!/bin/sh\nexec sleep 20\n")
    os.chmod(os.path.join(fake, "git"), 0o755)
    old_path = os.environ.get("PATH", "")
    os.environ["PATH"] = fake + os.pathsep + old_path
    try:
        t0 = time.time()
        h = buildinfo.resolve(os.path.dirname(RIG), timeout=0.5)
        dt = time.time() - t0
    finally:
        os.environ["PATH"] = old_path
    check("a hung git costs at most its timeout and never raises",
          dt < 3.0 and h["source"] in ("build-info", "none"),
          "%.2f s, source %s" % (dt, h["source"]))
    shutil.rmtree(fake, ignore_errors=True)

    env, r, root = _short_run("p2-software")
    try:
        doc = json.load(open(os.path.join(root, "run.json")))
        sw = doc.get("software") or {}
        check("run.json carries the software stamp",
              set(sw) >= {"version", "commit", "dirty", "describe", "source"}
              and sw == env.runmgr.software, json.dumps(sw))
        with open(os.path.join(root, "events.log")) as fh:
            lines = [ln for ln in fh if " software " in ln]
        check("events.log has exactly one software line, naming the commit",
              len(lines) == 1 and sw.get("commit", "?") in lines[0],
              "".join(lines)[:200])
        rb = rigcore.RunBrowser()
        check("the run browser's detail carries the stamp",
              rb.detail(r["run_id"]).get("software") == sw)
        old = env.runmgr.software
        env.runmgr.software = buildinfo.unknown()
        try:
            r2 = env.runmgr.start({"label": "p2-unknown", "calibrate": False})
            env.runmgr.stop()
        finally:
            env.runmgr.software = old
        doc2 = json.load(open(os.path.join(r2.get("root", ""), "run.json")))
        check("an unresolvable stamp still records a run, as 'unknown'",
              r2.get("ok") is True
              and (doc2.get("software") or {}).get("commit") == "unknown",
              json.dumps(doc2.get("software")))
    finally:
        env.close()
    with open(os.path.join(os.path.dirname(RIG), "deploy", "deploy.sh")) as fh:
        dsh = fh.read()
    with open(os.path.join(os.path.dirname(RIG), ".gitignore")) as fh:
        gi = fh.read()
    check("deploy.sh writes BUILD_INFO.json for the host and the nodes, and "
          "git ignores it",
          dsh.count("rig/buildinfo.py --write") == 2
          and "BUILD_INFO.json" in gi.split())


# ---------------------------------------------------------------------------
def suite(opts):
    _p1_time_source(opts)
    _p2_software_stamp(opts)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    _t0 = time.time()
    suite(opts)
    print("\naudit_provenance: %d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - _t0))
    sys.exit(1 if soaktest.FAIL else 0)

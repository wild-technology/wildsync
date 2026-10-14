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

P3  simulated runs. A run recorded against the test fakes carries the marker
    inside its own files (run.json "simulated", index.jsonl per frame, one
    events.log line), derived only from what the nodes report, so a real run
    can never be marked and a simulated one never left unmarked.

P4  hashes. The drain records every verified file's SHA-256 before the card
    delete; ingest writes raw_sha256/jpg_sha256 after the original eight
    manifest columns and a shasum-compatible <run>/SHA256SUMS; --verify
    re-hashes; a recorded hash is never silently replaced; runs without any
    of this still load.

P5  exit status. ingest.py exits 1 when a run was refused as ambiguous, a run
    name already held different bytes, a placed file differs from the drain's
    hash, or the card held files and nothing matched; routine leftovers and a
    few unmatched frames stay exit 0 (the operator chains drain && ingest).

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


def _short_run(label, specs=None, frames=2, nav=None, prep=None):
    """One real run against fakes: start, land `frames` frames per camera,
    stop. Returns (env, result of start, run root). The caller closes env.
    `prep(env)` runs before the monitors' first poll."""
    specs = specs or [("cam1", "127.0.0.2", 1), ("cam2", "127.0.0.3", 2)]
    env = Env(specs, poll=0.3, threaded=False,
              nav=nav if nav is not None else FakeNav(fix=False),
              imu_node="cam9")
    if prep is not None:
        prep(env)
    env.threaded = True
    for m in env.monitors:
        m.start()
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
# P3 — simulated runs are marked inside their own files
# ---------------------------------------------------------------------------
def _index_lines(root):
    with open(os.path.join(root, "index.jsonl")) as fh:
        return [json.loads(ln) for ln in fh if ln.strip()]


def _as_real_node(node):
    """Make a fake answer exactly as the real piagent and ilxctl do: the same
    documents with no "simulated" key. Test-only - FakeNode itself has no way
    to stop saying it is simulated."""
    from fakenode import FakeNode
    node.status = lambda: {k: v for k, v in FakeNode.status(node).items()
                           if k != "simulated"}
    node.health = lambda: {k: v for k, v in FakeNode.health(node).items()
                           if k != "simulated"}


def _p3_simulated(opts):
    sect("P3 simulated runs: marked in run.json, index.jsonl and events.log "
         "from what the nodes report; real runs never marked")
    from fakenode import FakeNode, FACTORY
    fk = FakeNode("camS", "127.0.0.9", cam_num=9)
    try:
        fk.label_override = {"simulated": False}
        fk.hide_keys = {"simulated"}
        check("a fake says it is simulated on both surfaces, and no override "
              "or hidden key can stop it",
              fk.status().get("simulated") is True
              and fk.health().get("simulated") is True)
    finally:
        fk.close()
    with open(os.path.join(RIG, "piagent.py")) as fh:
        pia = fh.read()
    src = ""
    srcdir = os.path.join(os.path.dirname(RIG), "src")
    for fn in sorted(os.listdir(srcdir)):
        if fn.endswith((".cpp", ".h")):
            with open(os.path.join(srcdir, fn), errors="replace") as fh:
                src += fh.read()
    check("the real piagent and ilxctl never send a 'simulated' key",
          "simulated" not in pia and "simulated" not in src)

    env, r, root = _short_run("p3-sim")
    try:
        doc = json.load(open(os.path.join(root, "run.json")))
        check("run.json of a run against the fakes says simulated, naming "
              "the nodes",
              doc.get("simulated") is True
              and doc.get("simulated_nodes") == ["cam1", "cam2"],
              json.dumps({k: doc.get(k) for k in ("simulated",
                                                   "simulated_nodes")}))
        idx = _index_lines(root)
        check("every index.jsonl frame of it says simulated",
              idx and all(e.get("simulated") is True for e in idx),
              str(len(idx)))
        with open(os.path.join(root, "events.log")) as fh:
            sim_lines = [ln for ln in fh if "SIMULATED" in ln]
        check("events.log has one SIMULATED line for it",
              len(sim_lines) == 1 and "cam1, cam2" in sim_lines[0],
              "".join(sim_lines)[:200])
        with open(os.path.join(root, "cam1", "flight_log.csv"),
                  newline="") as fh:
            hdr = next(csv.reader(fh))
        check("the flight_log shape is unchanged (23 columns)",
              hdr == runmod.FLIGHT_HEADER and len(hdr) == 23, str(len(hdr)))
        rb = rigcore.RunBrowser()
        row = {x["run_id"]: x for x in rb.list_runs()["runs"]}.get(r["run_id"])
        det = rb.detail(r["run_id"])
        check("the run browser lists and opens it as simulated",
              (row or {}).get("simulated") is True
              and det.get("simulated") is True
              and det.get("simulated_nodes") == ["cam1", "cam2"],
              json.dumps(row)[:200])
        check("each fake's monitor recorded it", all(
            m.simulated and m.snapshot().get("simulated") for m in env.monitors))

        # Never unmarked: crash recovery rewrites run.json but keeps it.
        doc["final"] = False
        with open(os.path.join(root, "run.json"), "w") as fh:
            json.dump(doc, fh)
        import types
        import rigd
        fake_rig = types.SimpleNamespace(runs=rigcore.RunBrowser(),
                                         events=env.events)
        rigd.Rig._recover_runs(fake_rig)
        doc2 = json.load(open(os.path.join(root, "run.json")))
        check("crash recovery keeps the simulated marker",
              doc2.get("interrupted") is True and doc2.get("simulated") is True
              and doc2.get("simulated_nodes") == ["cam1", "cam2"],
              json.dumps({k: doc2.get(k) for k in ("interrupted", "simulated")}))
    finally:
        env.close()

    # The same run against nodes that answer like the real daemons.
    def realise(env):
        for n in env.nodes.values():
            _as_real_node(n)
    env, r, root = _short_run("p3-real", prep=realise)
    try:
        doc = json.load(open(os.path.join(root, "run.json")))
        check("a run against real-shaped nodes carries no simulated key",
              "simulated" not in doc and "simulated_nodes" not in doc
              and not any(m.simulated for m in env.monitors),
              json.dumps({k: doc.get(k) for k in ("simulated",)}))
        idx = _index_lines(root)
        check("...nor do its index.jsonl frames",
              idx and not any("simulated" in e for e in idx), str(len(idx)))
        with open(os.path.join(root, "events.log")) as fh:
            ev = fh.read()
        check("...nor its events.log", "SIMULATED" not in ev)
        row = {x["run_id"]: x for x in
               rigcore.RunBrowser().list_runs()["runs"]}.get(r["run_id"])
        check("the run browser reports it as a real run",
              (row or {}).get("simulated") is False, json.dumps(row)[:160])

        # A node that turns out to be simulated mid-run (its first answers
        # lost) marks the run from that frame on, and the run stays marked.
        r2 = env.runmgr.start({"label": "p3-late", "calibrate": False})
        root2 = r2.get("root", "")
        m1 = env.mon("cam1")
        n1 = env.node("cam1")
        n1.add_frame(epoch=time.time())
        wait_for(lambda: len(_index_lines(root2)) >= 1, 10)
        m1.simulated = True
        n1.add_frame(epoch=time.time() + 0.5)
        wait_for(lambda: len(_index_lines(root2)) >= 2, 10)
        m1.simulated = False          # a monitor never clears it; the run
        env.runmgr.stop()             # must not depend on that either
        doc3 = json.load(open(os.path.join(root2, "run.json")))
        idx3 = _index_lines(root2)
        check("a node that reports simulated mid-run marks the run, and the "
              "mark survives to the final run.json",
              doc3.get("simulated") is True
              and doc3.get("simulated_nodes") == ["cam1"]
              and doc3.get("final") is True,
              json.dumps({k: doc3.get(k) for k in ("simulated",
                                                    "simulated_nodes")}))
        check("frames before it are unmarked, frames from it on are marked",
              len(idx3) >= 2 and "simulated" not in idx3[0]
              and idx3[-1].get("simulated") is True,
              json.dumps(idx3)[:300])
    finally:
        env.close()


# ---------------------------------------------------------------------------
# P4 — drain hash record, manifest hashes, SHA256SUMS, --verify
# ---------------------------------------------------------------------------
def _drainer(node, nominal, dest):
    import drain as draindrv
    from fakenode import loopback_map
    h, port = loopback_map(nominal, 8080)
    dr = draindrv.Drainer(node, h, dest=dest, log=lambda *a: None)
    dr.base = "http://%s:%d" % (h, port)
    return dr


def _cli(*args):
    import subprocess
    p = subprocess.run([sys.executable, os.path.join(RIG, "ingest.py")]
                       + list(args), stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=120)
    return p.returncode, p.stdout.decode("utf-8", "replace")


def _oneline(out, n=4):
    """The last lines of a command's output as one detail string, so its own
    FAILED/MISSING lines can never read as a failing check in the log."""
    return " | ".join(ln.strip() for ln in out.strip().splitlines()[-n:])


def _shasum_check(root):
    """(tool, ok) from the system's own checker, or (None, None)."""
    import subprocess
    for tool in (["shasum", "-a", "256", "-c"], ["sha256sum", "-c"]):
        if shutil.which(tool[0]):
            p = subprocess.run(tool + ["SHA256SUMS"], cwd=root,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, timeout=120)
            return tool[0], p.returncode == 0, _oneline(p.stdout.decode())
    return None, None, ""


def _p4_hashes(opts):
    sect("P4 hashes: drain record, manifest raw_sha256/jpg_sha256, "
         "<run>/SHA256SUMS, --verify, legacy runs")
    import drain as draindrv
    import ingest
    from fakenode import make_jpeg

    # -- the drain's durable per-file record --------------------------------
    env = Env([("cam1", "127.0.0.2", 1)], poll=0.3, threaded=True)
    raw = os.path.join(env.dir, "raw")
    try:
        env.wait_state("cam1", "CAM_CONNECTED")
        node = env.node("cam1")
        for _ in range(3):
            node.add_frame(epoch=time.time())
        n_card = len(node.card)
        dr = _drainer("cam1", "127.0.0.2", raw)
        rep = dr.run(keep_card=True)
        rec = draindrv.read_sums(os.path.join(raw, "SHA256SUMS"))
        want = {"cam1/" + f: draindrv.file_sha256(
            os.path.join(raw, "cam1", f)) for f in rep["files"]}
        check("the drain records the hash of every file it lands, as "
              "cam1/<name> beside the node directories",
              rep["files"] and rec == want and n_card == rep["pulled"],
              "%d files, %d recorded" % (len(rep["files"]), len(rec)))
        check("the staging directory itself holds only card files",
              "SHA256SUMS" not in os.listdir(os.path.join(raw, "cam1")))
        with open(os.path.join(raw, "SHA256SUMS")) as fh:
            n_lines = len(fh.readlines())
        dr2 = _drainer("cam1", "127.0.0.2", raw)
        dr2.run(keep_card=True)        # the same contents, re-verified
        with open(os.path.join(raw, "SHA256SUMS")) as fh:
            n_lines2 = len(fh.readlines())
        check("re-verifying identical files does not repeat their lines",
              n_lines2 == n_lines, "%d -> %d" % (n_lines, n_lines2))
        v = draindrv.verify_sums(raw)
        check("the drain record verifies", v["ok"] == len(rec)
              and not v["failed"] and not v["missing"], json.dumps(v)[:200])
        tool, ok, out = _shasum_check(raw)
        if tool:
            check("%s -c accepts the drain record" % tool, ok, out)
        # The record is part of the delete gate: no record, no delete.
        blocked = os.path.join(env.dir, "raw-blocked")
        os.makedirs(os.path.join(blocked, "SHA256SUMS"))   # unwritable as a file
        node.add_frame(epoch=time.time())
        before = set(node.card)
        rep3 = _drainer("cam1", "127.0.0.2", blocked).run()
        check("a drain that cannot write its hash record keeps the card "
              "originals", set(node.card) == before and rep3["deleted"] == 0
              and any("hash record" in e for e in rep3["errors"]),
              json.dumps(rep3["errors"][:2]))
    finally:
        env.close()

    # -- ingest: manifest columns, SHA256SUMS, cross-check, --verify --------
    tmp = _tmp()
    runs = os.path.join(tmp, "runs")
    root, doc = write_legacy_run(runs)
    rid = os.path.basename(root)
    # cam1's card files, staged the way the drain leaves them
    stage = os.path.join(tmp, "raw", "cam1")
    os.makedirs(stage)
    body_off = -86400.0 * 2 + 12.3            # the body's clock is days wrong
    staged = []
    for i, e in enumerate([x for x in doc["index"] if x["cam"] == 1]):
        stem = "DSC%05d" % (100 + i)
        with open(os.path.join(stage, stem + ".JPG"), "wb") as fh:
            fh.write(make_jpeg(e["epoch"] + body_off))
        with open(os.path.join(stage, stem + ".ARW"), "wb") as fh:
            fh.write(b"ARW-%d-" % i * 4000)
        staged += [stem + ".JPG", stem + ".ARW"]
    draindrv.append_sums(os.path.join(tmp, "raw", "SHA256SUMS"), [
        ("cam1/" + f, draindrv.file_sha256(os.path.join(stage, f)))
        for f in staged])
    out = ingest.ingest(stage, runs_dir=runs, log=lambda *a: None,
                        offsets=[])
    s1 = out["runs"][0]["summary"].get(1, {}) if out["runs"] else {}
    check("ingest matches the staged cam1 files into the legacy run",
          s1.get("matched") == 3 and s1.get("raw") == 3, json.dumps(s1))
    with open(os.path.join(root, "ingest_manifest.csv"), newline="") as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        mrows = [dict(zip(hdr, x)) for x in rd]
    check("the manifest keeps the original eight columns in place and "
          "appends raw_sha256, jpg_sha256",
          hdr[:8] == LEGACY_MANIFEST_COLS
          and hdr[8:] == ["raw_sha256", "jpg_sha256"], str(hdr))
    c1 = [m for m in mrows if m["cam"] == "1"]
    c2 = [m for m in mrows if m["cam"] == "2"]
    ok_h = all(m["raw_sha256"] == draindrv.file_sha256(os.path.join(
        root, "cam1", m["new_base"] + ".ARW")) and m["jpg_sha256"]
        == draindrv.file_sha256(os.path.join(root, "cam1", m["new_base"]
                                             + ".card.JPG")) for m in c1)
    check("each RAW and card JPEG row carries the SHA-256 of the file in "
          "the run", len(c1) == 3 and ok_h, json.dumps(c1[:1]))
    check("rows kept from the older manifest load with empty hash cells",
          len(c2) == 3 and all(m["raw_sha256"] == "" == m["jpg_sha256"]
                               and m["raw"] == "linked" for m in c2),
          json.dumps(c2[:1]))
    listed = draindrv.read_sums(os.path.join(root, "SHA256SUMS"))
    expect = {"run.json", "index.jsonl", "events.log", "ingest_manifest.csv",
              "cam1/flight_log.csv", "cam2/flight_log.csv"}
    frames = [k for k in listed if k.lower().endswith((".jpg", ".arw"))]
    check("SHA256SUMS covers every frame and the final run's records, not "
          "the XMP sidecars or itself",
          expect <= set(listed) and len(frames) == 6 + 3 + 3
          and not any(k.endswith(".xmp") or k == "SHA256SUMS"
                      for k in listed), json.dumps(sorted(listed))[:300])
    check("ingest found no difference from the drain's hashes",
          out["totals"].get("hash_mismatch") == 0, json.dumps(out["totals"]))
    tool, ok, sout = _shasum_check(root)
    if tool:
        check("%s -c accepts <run>/SHA256SUMS" % tool, ok, sout)
    else:
        soaktest.note("no shasum/sha256sum on this host: the shasum "
                      "compatibility of SHA256SUMS was not exercised")
    rc, cout = _cli("--runs", runs, "--verify", rid)
    check("ingest.py --verify passes an untouched run (exit 0)",
          rc == 0 and "VERIFIED" in cout and "NOT VERIFIED" not in cout,
          _oneline(cout))

    # A changed frame: --verify catches it, and neither a re-ingest nor
    # --write-sums re-records it.
    victim = os.path.join(root, "cam2", sorted(
        f for f in os.listdir(os.path.join(root, "cam2"))
        if f.endswith(".jpg"))[0])
    with open(victim, "r+b") as fh:
        fh.seek(40)
        fh.write(b"\x00\x01\x02")
    rc, cout = _cli("--runs", runs, "--write-sums", rid, "--verify", rid)
    vrel = "cam2/" + os.path.basename(victim)
    check("a changed frame fails --verify (exit 1) and --write-sums does "
          "not re-bless it",
          rc == 1 and ("FAILED " + vrel) in cout
          and draindrv.read_sums(os.path.join(root, "SHA256SUMS"))[vrel]
          == listed[vrel], _oneline(cout))
    os.unlink(victim)
    rc, cout = _cli("--runs", runs, "--verify", rid)
    check("a deleted frame is reported MISSING, not forgotten",
          rc == 1 and ("MISSING " + vrel) in cout, _oneline(cout))

    # A staged file that changed after the drain verified it.
    stage2 = os.path.join(tmp, "raw2", "cam1")
    shutil.copytree(stage, stage2)
    draindrv.append_sums(os.path.join(tmp, "raw2", "SHA256SUMS"), [
        ("cam1/" + f, draindrv.file_sha256(os.path.join(stage2, f)))
        for f in staged])
    root2, _ = write_legacy_run(runs, rid="260820_1919_transect-01")
    with open(os.path.join(stage2, staged[1]), "ab") as fh:
        fh.write(b"tail")
    logs = []
    out2 = ingest.ingest(stage2, runs_dir=runs, only=[os.path.basename(root2)],
                         log=logs.append, offsets=[])
    check("ingest flags a placed file whose bytes differ from the drain's "
          "verified hash", out2["totals"].get("hash_mismatch") == 1
          and any("differs from the hash the drain verified" in m
                  for m in logs), json.dumps(out2["totals"]))

    # A run with no SHA256SUMS at all (every run before v1.1.0).
    root3, _ = write_legacy_run(runs, rid="260820_0536_gps-e2e")
    rc, cout = _cli("--runs", runs, "--verify", os.path.basename(root3))
    check("--verify on a run that never recorded hashes says so (exit 2)",
          rc == 2 and "no SHA256SUMS" in cout, _oneline(cout))
    saved = rigcore.RUNS_DIR
    rigcore.RUNS_DIR = runs
    try:
        rb = rigcore.RunBrowser()
        ids = {x["run_id"] for x in rb.list_runs()["runs"]}
        check("runs with and without hashes all list and open",
              {rid, os.path.basename(root2), os.path.basename(root3)} <= ids
              and rb.detail(os.path.basename(root3)).get("frames_indexed")
              == 6, str(sorted(ids)))
    finally:
        rigcore.RUNS_DIR = saved
    shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# P5 — ingest.py exit status
# ---------------------------------------------------------------------------
def _cli_home(home, *args):
    """_cli with HOME pointed at a temp dir, so the journal ingest reads for
    exif offsets (~/rig/rigd.jsonl) is empty and the case is hermetic."""
    import subprocess
    env = dict(os.environ, HOME=home, PYTHONDONTWRITEBYTECODE="1")
    p = subprocess.run([sys.executable, os.path.join(RIG, "ingest.py")]
                       + list(args), stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=120, env=env)
    return p.returncode, p.stdout.decode("utf-8", "replace")


def _p5_case(tmp, name, shots=6, stage_name="cam1", series="DSC", skip=(),
             spacing=None, record=True, cam=1, body_off=-172800.0 + 12.3):
    """A fresh run plus a staging dir of `cam`'s card files (JPG + ARW per
    frame, stems that are NOT the PC-save counter so only time can match).
    Returns (runs, stage, root, doc, staged names)."""
    from fakenode import make_jpeg
    import drain as draindrv
    base = os.path.join(tmp, name)
    runs = os.path.join(base, "runs")
    root, doc = write_legacy_run(runs, shots=shots)
    stage = os.path.join(base, "raw", stage_name)
    os.makedirs(stage, exist_ok=True)
    staged = []
    rows = [x for x in doc["index"] if x["cam"] == cam]
    for i, e in enumerate(rows):
        if i in skip:
            continue
        ep = e["epoch"] + body_off + (i * i * 7.0 if spacing else 0.0)
        stem = "%s%05d" % (series, 70000 + i + (0 if cam == 1 else 500))
        with open(os.path.join(stage, stem + ".JPG"), "wb") as fh:
            fh.write(make_jpeg(ep))
        with open(os.path.join(stage, stem + ".ARW"), "wb") as fh:
            fh.write(b"ARW-%d-" % i * 4000)
        staged += [stem + ".JPG", stem + ".ARW"]
    if record:
        draindrv.append_sums(os.path.join(base, "raw", "SHA256SUMS"), [
            (stage_name + "/" + f, draindrv.file_sha256(os.path.join(stage, f)))
            for f in staged])
    return runs, stage, root, doc, staged


def _p5_ingest_exit(opts):
    sect("P5 exit status: ingest.py exits 1 on refused / conflicting / "
         "hash-mismatched / nothing-matched, 0 otherwise")
    tmp = _tmp()
    home = os.path.join(tmp, "home")
    os.makedirs(home)
    try:
        runs, stage, root, doc, _ = _p5_case(tmp, "clean")
        rc, out = _cli_home(home, "--runs", runs, stage)
        check("a clean match exits 0", rc == 0, _oneline(out))
        rid = os.path.basename(root)
        rc, out = _cli_home(home, "--runs", runs, "--verify", rid)
        check("--verify keeps its own exit codes (0 for a verified run)",
              rc == 0 and "VERIFIED" in out, _oneline(out))

        runs, stage, root, doc, _ = _p5_case(tmp, "leftovers", skip=(2,))
        from fakenode import make_jpeg
        with open(os.path.join(stage, "DSC08999.JPG"), "wb") as fh:
            fh.write(make_jpeg(doc["started"] - 900 - 172800.0 + 12.3))
        rc, out = _cli_home(home, "--runs", runs, stage)
        check("matched frames plus unmatched rows and a leftover card file "
              "still exit 0 (routine)", rc == 0 and "unmatched 1" in out,
              _oneline(out, 6))

        runs, stage, root, doc, _ = _p5_case(tmp, "nomatch", spacing=True)
        rc, out = _cli_home(home, "--runs", runs, stage)
        check("card files that align with no row (NO MATCH) exit 1",
              rc == 1 and "NO MATCH" in out, _oneline(out, 6))

        runs, stage, root, doc, _ = _p5_case(tmp, "ambig", stage_name="download")
        _p5_case_extra = _p5_case(tmp, "ambig2", cam=2, series="_CA",
                                  stage_name="download")
        # both bodies' files in one non-camN directory, no journal
        for f in os.listdir(_p5_case_extra[1]):
            shutil.copy(os.path.join(_p5_case_extra[1], f), stage)
        rc, out = _cli_home(home, "--runs", runs, stage)
        check("a combined directory with no exif_offset is REFUSED: exit 1, "
              "nothing renamed", rc == 1 and "REFUSED" in out
              and not any(f.endswith(".ARW") for f in os.listdir(
                  os.path.join(root, "cam1"))), _oneline(out, 6))

        runs, stage, root, doc, _ = _p5_case(tmp, "conflict")
        first = [x for x in doc["index"] if x["cam"] == 1][0]
        with open(os.path.join(root, "cam1",
                               os.path.splitext(first["file"])[0] + ".ARW"),
                  "wb") as fh:
            fh.write(b"DIFFERENT BYTES")
        rc, out = _cli_home(home, "--runs", runs, stage)
        check("a run name already holding different bytes (CONFLICT) exits 1",
              rc == 1 and "CONFLICT" in out, _oneline(out, 6))

        runs, stage, root, doc, staged = _p5_case(tmp, "drainhash")
        with open(os.path.join(stage, staged[1]), "ab") as fh:
            fh.write(b"changed after the drain verified it")
        rc, out = _cli_home(home, "--runs", runs, stage)
        check("a staged file that differs from the drain's hash exits 1",
              rc == 1 and "HASH MISMATCH" in out, _oneline(out, 6))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
def suite(opts):
    _p1_time_source(opts)
    _p2_software_stamp(opts)
    _p3_simulated(opts)
    _p4_hashes(opts)
    _p5_ingest_exit(opts)


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

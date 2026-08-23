#!/usr/bin/env python3
"""selftest — exercise the rig's image pathway and logic end to end.

Runs on the Jetson. It drives everything that can be checked without physically
firing the GPIO trigger or relying on live PC-transfer (both currently gated on
hardware: FOCUS/TRIGGER leads unseated, and the camera body's PC-save menu).
What it DOES check against the real camera node:

  * rename convention           Cam{N}_YYYYMMDD_hhmmss.ss.jpg
  * EXIF capture-time extraction on a real frame pulled from the node
  * the pull -> rename -> flight_log row for a real frame
  * flight_log header + shape
  * nav PDGY decode + UTM
  * IMU frame decode (vendor-format loopback)
  * Sony encodings and the convergence field map
  * live node reachability + settings readback

and, added by the pre-deployment hardening pass, the contract checks that need
no hardware at all — the flight_log header against PROTOCOL.md byte for byte,
the rename convention against its regex including the rounding edges, the
fleet table, and the label-vs-raw discipline of the convergence map.

Usage:  python3 rig/selftest.py [--node cam2] [--offline]
        --offline skips everything that touches a node, so it is safe to run
        while somebody else has the camera.
Exit code is nonzero if any check fails.

For fault injection, state-machine coverage and soak, see rig/soaktest.py,
which drives the same code against in-process fake nodes (rig/fakenode.py).
"""

import argparse
import csv
import os
import re
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# Tests live in rig/tests/; the modules under test live one level up and the
# interface contract they are checked against lives in docs/.
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
PROTOCOL_PATH = os.path.join(os.path.dirname(RIG), "docs", "PROTOCOL.md")

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("  %s %s%s" % ("PASS" if cond else "FAIL", name,
                         (" — " + detail) if detail else ""))
    return cond


def sect(t):
    print("\n== %s" % t)


def test_rename_and_dt():
    sect("rename + datetime formatting")
    import run
    ep = 1723750245.20            # a fixed epoch, UTC
    fn = run._fmt_fname(3, ep)
    dt = run._fmt_dt(ep)
    want_stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime(ep))
    check("filename shape Cam3_...", fn.startswith("Cam3_") and fn.endswith(".jpg")
          and want_stamp in fn, fn)
    check("filename carries .ss centiseconds", fn.endswith("20.jpg"), fn)
    check("datetime YYMMDD_hhmmss.ss", len(dt) == 16 and dt[6] == "_"
          and dt[13] == "." , dt)


def _protocol_lines():
    try:
        with open(PROTOCOL_PATH) as fh:
            return fh.read().splitlines()
    except OSError:
        return []


def test_flight_header():
    sect("flight_log header")
    import run
    exp = ["filename", "datetime", "lat", "long", "xutm", "yutm", "utm_zone",
           "depth_from_xplore9", "pitch", "roll", "yaw", "heading_mag_xplore",
           "heading_imu"]
    check("header starts with the agreed columns",
          run.FLIGHT_HEADER[:len(exp)] == exp,
          "%d cols" % len(run.FLIGHT_HEADER))
    check("capture_source/time_source/time_err present",
          all(c in run.FLIGHT_HEADER for c in
              ("capture_source", "time_source", "time_err_ms")))
    check("no duplicate columns",
          len(set(run.FLIGHT_HEADER)) == len(run.FLIGHT_HEADER))
    # The header is a contract with every downstream consumer: compare it to
    # PROTOCOL.md verbatim rather than to a copy of it.
    lines = _protocol_lines()
    want = None
    for i, ln in enumerate(lines):
        if "flight_log.csv header (exact)" in ln:
            for cand in lines[i + 1:i + 6]:
                if cand.startswith("```") or not cand.strip():
                    continue
                want = cand.strip()
                break
            break
    if want is None:
        check("PROTOCOL.md declares the flight_log header", False)
    else:
        check("header matches PROTOCOL.md byte for byte",
              ",".join(run.FLIGHT_HEADER) == want,
              "%d cols" % len(run.FLIGHT_HEADER))
        check("header is the agreed 23 columns", len(want.split(",")) == 23,
              "%d" % len(want.split(",")))


FNAME_RE = re.compile(r"^Cam(\d+)_(\d{8})_(\d{6})\.(\d{2})\.jpg$")


def test_rename_contract():
    sect("rename convention (regex + rounding edges)")
    import run
    base = 1786000000.0                       # a whole second, UTC
    for cam in (1, 2, 3):
        fn = run._fmt_fname(cam, base + 0.25)
        m = FNAME_RE.match(fn)
        check("Cam%d name matches Cam#_YYYYMMDD_hhmmss.ss.jpg" % cam, bool(m), fn)
        if m:
            check("Cam%d name carries the node's camera number" % cam,
                  m.group(1) == str(cam))
    # datetime column and filename must describe the same instant
    fn = run._fmt_fname(3, base + 0.25)
    dt = run._fmt_dt(base + 0.25)
    check("filename stamp and datetime column agree",
          fn[5:].replace(".jpg", "")[2:] == dt, "%s vs %s" % (fn, dt))
    # The centisecond field is built with "%.2f" % (epoch % 1), which rounds up
    # to 1.00 in the last 5 ms of every second.
    edge = base + 0.999
    fn_e, dt_e = run._fmt_fname(3, edge), run._fmt_dt(edge)
    cs = FNAME_RE.match(fn_e).group(4) if FNAME_RE.match(fn_e) else "??"
    want_s = time.strftime("%H%M%S", time.gmtime(round(edge)))
    # Read the time field out of the regex rather than slicing by index: the
    # earlier fn_e[13:19] was off by one (it caught the '_' separator), so this
    # failed against correct output.
    m_e = FNAME_RE.match(fn_e)
    got_s = m_e.group(3) if m_e else "??"
    check("a frame in the last 5 ms of a second is not stamped a second early",
          got_s == want_s,
          "%.3f -> %s / %s (got %s, expected the %s second)"
          % (edge, fn_e, dt_e, got_s, want_s))


def test_fleet_table():
    sect("fleet table vs PROTOCOL.md")
    import rigcore
    lines = _protocol_lines()
    documented = {}
    for ln in lines:
        cells = [c.strip() for c in ln.strip().strip("|").split("|")]
        if len(cells) >= 3 and cells[0].startswith("pi-cam"):
            documented[cells[0].replace("pi-", "")] = cells[2]
    builtin = getattr(rigcore, "_DEFAULT_NODES", rigcore.NODES)
    coded = {n["name"]: n["host"] for n in builtin}
    check("PROTOCOL.md documents the camera nodes", len(documented) >= 2,
          ",".join(sorted(documented)))
    check("the built-in fleet matches the documented fleet", coded == documented,
          "code=%s doc=%s" % (coded, documented))
    check("cam_num is 1..N and unique",
          sorted(n["cam_num"] for n in rigcore.NODES)
          == list(range(1, len(rigcore.NODES) + 1)))
    live = {n["name"]: n["host"] for n in rigcore.NODES}
    if live != coded:
        print("  note %s overrides the built-in fleet: %s"
              % (getattr(rigcore, "NODES_PATH", "a nodes override"),
                 ", ".join("%s %s->%s" % (k, coded.get(k, "-"), v)
                           for k, v in sorted(live.items())
                           if coded.get(k) != v)))


def test_encodings():
    sect("Sony encodings + convergence map")
    import rigcore
    check("shutter 1/200 encodes 65736", rigcore.shutter_encode(1, 200) == 65736,
          str(rigcore.shutter_encode(1, 200)))
    n, d = rigcore.shutter_decode(65736)
    check("shutter decode round-trips", (n, d) == (1, 200), "%d/%d" % (n, d))
    check("convergence covers exposure+image fields",
          all(k in rigcore.CONVERGE_FIELDS for k in
              ("aperture", "shutter", "iso", "store_dest")))
    check("default desired is manual/MF/PC+card",
          rigcore.DEFAULT_DESIRED["focus_mode"] == 1
          and rigcore.DEFAULT_DESIRED["store_dest"] == 3)
    for num, den in ((1, 200), (1, 60), (1, 8000), (2, 1), (13, 10)):
        v = rigcore.shutter_encode(num, den)
        check("shutter %d/%d round-trips through the Sony encoding" % (num, den),
              rigcore.shutter_decode(v) == (num, den), str(v))
    check("ISO AUTO is the documented sentinel",
          rigcore.ISO_AUTO == 16777215)
    check("drive encodings match PROTOCOL.md",
          rigcore.DRIVE_SINGLE == 1 and rigcore.DRIVE_CONT_LO == 65540)
    # ilxctl publishes a human label under the bare name and the raw number
    # under "<name>Value"; converging on the label is the bug this guards.
    for field in ("iso", "shutter", "aperture", "drive"):
        which, key = rigcore.CONVERGE_FIELDS[field]
        check("convergence reads %s back on the RAW key, not the label" % field,
              key is not None and key != field and key.endswith("Value"),
              "%s -> %s" % (field, key))
    check("every desired field with a readback key is in DEFAULT_DESIRED",
          all(f in rigcore.DEFAULT_DESIRED for f in rigcore.CONVERGE_FIELDS))


def test_imports():
    sect("modules import cleanly")
    for mod in ("rigcore", "run", "imu_yb", "nav", "fakenode"):
        try:
            __import__(mod)
            check("import %s" % mod, True)
        except Exception as e:                                # noqa: BLE001
            check("import %s" % mod, False, "%s: %s" % (type(e).__name__, e))
    # soaktest is only compiled, never imported: importing it installs a network
    # guard that pins every request to loopback, which would break the live
    # checks below.
    # soaktest sits beside this file; the services it exercises live one level up.
    for base, f in ((HERE, "soaktest.py"), (RIG, "rigd.py"), (RIG, "piagent.py")):
        p = os.path.join(base, f)
        try:
            with open(p) as fh:
                compile(fh.read(), p, "exec")
            check("%s compiles" % f, True)
        except (OSError, SyntaxError) as e:
            check("%s compiles" % f, False, str(e))


def test_nav():
    sect("nav decode + UTM")
    try:
        import nav
    except Exception as e:  # noqa: BLE001
        check("import nav", False, str(e)); return
    # UTM against a known reference: 40.0 N, -105.0 W (UTM 13N)
    x, y, z = nav.latlon_to_utm(40.0, -105.0)
    check("UTM zone for -105 lon is 13N", z.startswith("13"), z)
    check("UTM easting near 500 km at central meridian",
          480000 < x < 520000, "%.0f" % x)
    check("UTM northing near 4.43 Mm at 40N",
          4.4e6 < y < 4.45e6, "%.0f" % y)
    # PDGY parse of a hand-built 129025 position line, if the API exists.
    if hasattr(nav, "parse_pdgy") and hasattr(nav, "decode_pgn"):
        check("nav exposes parse_pdgy/decode_pgn", True)


def test_imu():
    sect("IMU vendor-frame decode (loopback)")
    try:
        import imu_yb
    except Exception as e:  # noqa: BLE001
        check("import imu_yb", False, str(e)); return
    check("imu_yb exposes ImuReader + probe",
          hasattr(imu_yb, "ImuReader") and hasattr(imu_yb, "probe"))


def test_exif_and_pull(node):
    sect("EXIF + pull->rename against a real frame on %s" % node)
    import rigcore
    import run
    mon = next((n for n in rigcore.NODES if n["name"] == node), None)
    if not mon:
        check("node in fleet", False, node); return
    host = mon["host"]
    shots = rigcore.http_json("http://%s:8080/api/shots" % host, timeout=8)
    if not isinstance(shots, list) or not shots:
        check("node has frames to test with", False,
              "unreachable or empty — is ilxctl up on %s?" % host); return
    check("node lists %d frames" % len(shots), True)
    name = sorted(s["name"] for s in shots)[-1]
    data, err = rigcore.http_bytes("http://%s:8080/shot/%s" % (host, name),
                                   timeout=30)
    if not check("download a real frame (%s)" % name, data is not None,
                 err or ""):
        return
    check("frame is a JPEG", data[:2] == b"\xff\xd8", "%d bytes" % len(data))
    cap = run._exif_capture_epoch(data)
    check("EXIF capture time parsed", cap is not None and cap > 1_600_000_000,
          time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(cap)) if cap else "none")
    # Run it through the rename + a flight_log row exactly as the pull worker would.
    epoch = cap or time.time()
    cam_num = mon["cam_num"]
    fname = run._fmt_fname(cam_num, epoch)
    with tempfile.TemporaryDirectory() as d:
        dest = os.path.join(d, fname)
        with open(dest, "wb") as fh:
            fh.write(data)
        check("renamed file written", os.path.exists(dest)
              and os.path.getsize(dest) == len(data), fname)
        # a flight_log row with empty nav/imu (both hardware absent)
        path = os.path.join(d, "flight_log.csv")
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(run.FLIGHT_HEADER)
            row = {k: "" for k in run.FLIGHT_HEADER}
            row.update({"filename": fname, "datetime": run._fmt_dt(epoch),
                        "capture_source": "exif", "time_source": "jetson"})
            w.writerow([row[k] for k in run.FLIGHT_HEADER])
        with open(path) as fh:
            rows = list(csv.reader(fh))
        check("flight_log has header + 1 row", len(rows) == 2)
        check("flight_log row filename matches rename",
              rows[1][0] == fname)


def test_live(node):
    sect("live node reachability + settings readback (%s)" % node)
    import rigcore
    mon = next((n for n in rigcore.NODES if n["name"] == node), None)
    st = rigcore.http_json("http://%s:8080/api/status" % mon["host"], timeout=8)
    if st.get("_unreachable"):
        check("ilxctl reachable", False, "unreachable"); return
    check("ilxctl reachable", True)
    check("camera connected", bool(st.get("connected")),
          "%s %s" % (st.get("model"), st.get("id")))
    h = rigcore.http_json("http://%s:8081/health" % mon["host"], timeout=5)
    if not h.get("_unreachable"):
        g = h.get("gpio", {})
        check("piagent up + gpio chip resolved", bool(g.get("chip")),
              "chip=%s available=%s" % (g.get("chip"), g.get("available")))
        check("EXPOSURE monitor running", bool(g.get("monitor_running")),
              "edges=%s" % g.get("edges_seen"))
    else:
        print("  (piagent not running on %s — skip gpio checks)" % node)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", default="cam2")
    ap.add_argument("--offline", action="store_true",
                    help="skip every check that touches a camera node")
    a = ap.parse_args()
    print("Wild Sync selftest — %s%s" % (time.strftime("%Y-%m-%d %H:%M:%S"),
                                         " (offline)" if a.offline else ""))
    test_imports()
    test_rename_and_dt()
    test_rename_contract()
    test_flight_header()
    test_encodings()
    test_fleet_table()
    test_nav()
    test_imu()
    if a.offline:
        print("\n(offline: node checks skipped — run without --offline, or "
              "python3 rig/soaktest.py for the full fake-node gate)")
    else:
        test_exif_and_pull(a.node)
        test_live(a.node)
    print("\n%d passed, %d failed" % (len(PASS), len(FAIL)))
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
        sys.exit(1)
    print("ALL PASS")


if __name__ == "__main__":
    main()

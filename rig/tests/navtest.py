#!/usr/bin/env python3
"""Wild Sync nav integration tests — the ones nav.py's own --selftest cannot do.

`nav.py --selftest` and `navlog.py --selftest` cover the pure layers (parsers,
PGN decoders, UTM, TimeAuthority, log format).  This file covers the parts that
need a real serial device and a real reader thread:

  1. full stack over a PTY: bytes on a tty -> NavReader thread -> decode ->
     fix_at() -> the seven flight_log.csv columns
  2. malformed frames arriving mid-stream must not corrupt a good fix
  3. TimeAuthority must fall back to the Jetson clock the moment GPS goes stale
  4. gateway unplugged mid-run: clean drop, honest gateway_online, backoff retry
  5. gateway replugged on a DIFFERENT device node: by-id re-resolution
  6. raw log fidelity + replay reproducing the live fix exactly
  7. wrong baud (endless bytes, no newline) must not hoard memory or crash
  8. absent / unopenable device: explicit errors, no crash

    python3 rig/navtest.py           # PTY suite, no hardware needed
    python3 rig/navtest.py --hw      # also probe the real gateway on this box

A PTY is a real tty from the kernel's point of view — same termios path, same
pyserial code, same failure exceptions — so this exercises everything except
the FTDI driver and the gateway firmware itself.
"""

import base64
import os
import pty
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import nav          # noqa: E402
import navlog       # noqa: E402

# CN Tower; easting/northing below come from pyproj 3.7.2 (EPSG:4326->32617).
LAT, LON = 43.642567, -79.387139
UTM_E, UTM_N, UTM_Z = 630084.3008, 4833438.5857, "17T"

_failures = []


def ok(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name
          + ("" if cond else "   <- %s" % (detail,)))
    if not cond:
        _failures.append(name)


def sect(t):
    print("\n== %s" % t)


def mkline(pgn, payload):
    return ("!PDGY,%d,2,3,255,1.234,%s\r\n"
            % (pgn, base64.b64encode(payload).decode())).encode()


def pgn_129025(lat=LAT, lon=LON):
    return struct.pack("<ii", round(lat * 1e7), round(lon * 1e7))


def pgn_128267(depth_cm=1234, offset_mm=500):
    return struct.pack("<BIhB", 0, depth_cm, offset_mm, 0xFF)


def pgn_127250():
    # heading 1.5708 rad magnetic, deviation N/A, variation -0.1745 rad
    return struct.pack("<BHhhB", 0, 15708, 0x7FFF, -1745, 0xFD)


def pgn_129026():
    return struct.pack("<BBHHH", 0, 0xFC, 15708, 350, 0xFFFF)


def pgn_129029(gps_epoch, method=1, sats=12):
    days = int(gps_epoch // 86400)
    tod = gps_epoch - days * 86400
    d = struct.pack("<BHIqqq", 1, days, round(tod * 1e4),
                    round(LAT * 1e16), round(LON * 1e16), round(76.543 * 1e6))
    d += bytes([(method << 4) & 0xFF, 0xFC, sats])
    d += struct.pack("<hh", 90, 150) + struct.pack("<i", -3520) + bytes([0])
    return d


def new_pty():
    master, slave = pty.openpty()
    name = os.ttyname(slave)
    os.close(slave)          # NavReader opens it by name itself
    return master, name


# ---------------------------------------------------------------------------

def test_full_stack(tmpdir):
    sect("1-3. full stack over a PTY: bytes -> thread -> flight_log columns")
    master, port = new_pty()
    logpath = os.path.join(tmpdir, "nmea_raw.log")
    log = navlog.RawNmeaLog(logpath, run_id="navtest")
    nr = nav.NavReader(port=port, auto_reopen=True)
    nr.set_raw_hook(log)
    nr.open()
    nr.start()
    ok("opened the port", nr.port == port, nr.port)

    os.write(master, b"$PDGY,TEXT,iKonvert booting\r\n")
    os.write(master, b"$PDGY,000000,,,,,,,\r\n")      # powered but off-bus
    time.sleep(0.4)
    ok("gateway_online on a control line alone", nr.gateway_online)
    ok("off-bus gateway yields no fix", nr.fix_at()["valid"] is False)
    ok("no GPS yet -> jetson", nr.time_authority.source() == "jetson")

    os.write(master, mkline(129025, pgn_129025()))
    os.write(master, mkline(128267, pgn_128267()))
    os.write(master, mkline(127250, pgn_127250()))
    os.write(master, mkline(129026, pgn_129026()))
    t_cap = time.time()
    os.write(master, mkline(129029, pgn_129029(time.time() + 2.0)))
    time.sleep(0.3)
    os.write(master, mkline(129029, pgn_129029(time.time() + 2.0)))
    time.sleep(0.5)

    row = nr.flight_row(t_cap)
    print("   flight_row:", {k: row[k] for k in nav.FLIGHT_LOG_KEYS})
    ok("row valid", row["valid"], row)
    ok("lat", abs(row["lat"] - LAT) < 1e-7, row["lat"])
    ok("long", abs(row["long"] - LON) < 1e-7, row["long"])
    ok("xutm matches pyproj", abs(row["xutm"] - UTM_E) < 0.05, row["xutm"])
    ok("yutm matches pyproj", abs(row["yutm"] - UTM_N) < 0.05, row["yutm"])
    ok("utm_zone", row["utm_zone"] == UTM_Z, row["utm_zone"])
    ok("depth_from_xplore9", abs(row["depth_from_xplore9"] - 12.34) < 1e-9,
       row["depth_from_xplore9"])
    ok("heading_mag_xplore", abs(row["heading_mag_xplore"] - 90.00021) < 1e-3,
       row["heading_mag_xplore"])
    ok("age_s is small and real", 0 <= abs(row["age_s"]) < 2.0, row["age_s"])
    ok("two good GNSS frames -> time_source gps", row["time_source"] == "gps",
       nr.time_authority.status())
    ok("gps offset ~2.0 s", abs(nr.time_authority.offset_to() - 2.0) < 0.25,
       nr.time_authority.offset_to())

    sect("2. malformed frames must not disturb a good fix")
    before = nr.flight_row(t_cap)
    for junk in (b"!PDGY,129025,2,3,255,1.0,!!!not-base64!!!\r\n",
                 b"!PDGY,129025,2,3,255,1.0,AAA=\r\n",       # too short
                 b"\xff\xfe\x00 raw binary junk\r\n",
                 b"$GPRMC,123519,A,4807.038,N,01131.0,E,022.4,084.4*6A\r\n",
                 b"!PDGY,129025,2,3,255,1.0,"
                 + base64.b64encode(struct.pack("<iI", round(LAT * 1e7),
                                                0x7FFFFFFF)) + b"\r\n"):
        os.write(master, junk)
    time.sleep(0.5)
    after = nr.flight_row(t_cap)
    ok("still valid after garbage", after["valid"], after)
    ok("lat untouched by garbage", after["lat"] == before["lat"],
       (before["lat"], after["lat"]))
    ok("bad lines counted", nr._bad_line_count >= 2, nr._bad_line_count)

    sect("3. GPS going stale must hand authority back to the Jetson")
    nr.time_authority._fed_local = time.time() - (nav.TimeAuthority.STALE_S + 0.2)
    ok("stale -> jetson", nr.time_authority.source() == "jetson")
    ok("stale -> offset dropped", nr.time_authority.offset_to() == 0.0)
    ok("flight_row reports jetson", nr.flight_row()["time_source"] == "jetson")

    sect("4. gateway unplugged mid-run")
    seen = nr._line_count
    os.close(master)
    t0 = time.time()
    while time.time() - t0 < 6.0 and nr._ser is not None:
        time.sleep(0.1)
    ok("dead port dropped", nr._ser is None, nr._ser)
    ok("error recorded", bool(nr._last_error), nr._last_error)
    print("   last_error:", nr._last_error)
    time.sleep(nav.NavReader.ONLINE_S + 0.5)
    ok("gateway_online decayed to False", nr.gateway_online is False)
    snap = nr.snapshot()
    ok("snapshot goes invalid rather than serving stale values",
       snap["valid"] is False and snap["lat"] is None, snap["lat"])
    ok("reader thread still alive and retrying", nr._thread.is_alive())
    ok("retries back off instead of spinning",
       nr._backoff > nav.NavReader.REOPEN_MIN_S, nr._backoff)
    nr.stop()
    log.close()
    ok("stopped cleanly", nr._thread is None)

    sect("6. raw log fidelity and replay")
    recs = list(navlog.read_records(logpath))
    print("   %d records logged, %d lines seen by the reader" % (len(recs), seen))
    ok("every line logged", len(recs) >= seen, (len(recs), seen))
    ok("control line verbatim",
       any(r["line"] == "$PDGY,000000,,,,,,," for r in recs))
    ok("unparseable line kept verbatim",
       any("not-base64" in r["line"] for r in recs))
    ok("foreign NMEA0183 line kept verbatim",
       any(r["line"].startswith("$GPRMC") for r in recs))
    ok("monotonic column non-decreasing",
       all(recs[i]["mono"] <= recs[i + 1]["mono"] for i in range(len(recs) - 1)))
    ok("both clocks present and distinct",
       all(r["wall"] > 1.7e9 and r["mono"] < 1e7 for r in recs))

    rp = nav.NavReader(port="/dev/null", auto_reopen=False)
    n = navlog.replay(logpath, rp)
    rr = rp.flight_row(t_cap)
    ok("replayed %d lines" % n, n > 0)
    ok("replay reproduces lat", abs(rr["lat"] - LAT) < 1e-7, rr["lat"])
    ok("replay reproduces depth", abs(rr["depth_from_xplore9"] - 12.34) < 1e-9)
    ok("replay reproduces easting", abs(rr["xutm"] - UTM_E) < 0.05)


def test_replug():
    sect("5. unplug then replug on a different device node")
    master, p1 = new_pty()
    current = {"path": p1}
    saved = nav.find_ikonvert_port
    nav.find_ikonvert_port = lambda *a, **k: current["path"]
    try:
        nr = nav.NavReader(port=None, auto_reopen=True)   # None => re-resolve
        nr.REOPEN_MAX_S = 2.0
        nr.open()
        nr.start()
        ok("opened via the resolver", nr.port == p1, nr.port)
        line = mkline(129025, pgn_129025())
        os.write(master, line)
        time.sleep(0.4)
        ok("fix before unplug", nr.fix_at()["valid"])

        os.close(master)
        current["path"] = None                 # device node gone entirely
        t0 = time.time()
        while time.time() - t0 < 6 and nr._ser is not None:
            time.sleep(0.1)
        ok("port dropped", nr._ser is None)
        time.sleep(1.5)
        ok("says the device is absent, explicitly",
           "no iKonvert found" in (nr._last_error or ""), nr._last_error)

        master2, p2 = new_pty()
        current["path"] = p2
        t0 = time.time()
        while time.time() - t0 < 20 and nr._ser is None:
            time.sleep(0.2)
        ok("reattached to the new node", nr._ser is not None and nr.port == p2,
           (nr.port, p2))
        os.write(master2, line)
        time.sleep(0.6)
        ok("reopen counted", nr._reopen_count >= 1, nr._reopen_count)
        ok("gateway_online again", nr.gateway_online)
        ok("fix after replug", nr.fix_at()["valid"])
        ok("error cleared on success", nr._last_error is None, nr._last_error)
        nr.stop()
        os.close(master2)
    finally:
        nav.find_ikonvert_port = saved


def test_wrong_baud():
    sect("7. wrong baud: endless bytes, no line terminators")
    master, port = new_pty()
    nr = nav.NavReader(port=port, auto_reopen=False)
    got = []
    nr.set_raw_hook(lambda ep, ln: got.append(ln))
    nr.open()
    nr.start()
    for _ in range(3):
        os.write(master, os.urandom(4000).replace(b"\n", b"x"))
    time.sleep(1.2)
    ok("buffer capped, garbage emitted not hoarded", len(got) >= 1, len(got))
    ok("thread survived binary garbage", nr._thread.is_alive())
    ok("garbage produced no fix", nr.snapshot()["valid"] is False)
    ok("garbage produced no GPS authority",
       nr.time_authority.source() == "jetson")
    nr.stop()
    os.close(master)


def test_absent():
    sect("8. absent / unopenable device")
    nr = nav.NavReader(port="/dev/ttyUSB-does-not-exist", auto_reopen=False)
    try:
        nr.open()
        ok("absent device raises", False)
    except RuntimeError as exc:
        ok("absent device raises RuntimeError", True)
        print("   ", exc)
    dead = nav.NavReader(port="/dev/null", auto_reopen=False)
    row = dead.flight_row()
    ok("dead reader: every nav column None",
       all(row[k] is None for k in nav.FLIGHT_LOG_KEYS), row)
    ok("dead reader: not valid", row["valid"] is False)
    ok("dead reader: jetson time", row["time_source"] == "jetson")
    ok("dead reader: health is a dict", isinstance(dead.health(), dict))
    print("    resolver on this box:", nav.find_ikonvert_port())


def test_hardware():
    sect("HW. probe the real gateway on this machine")
    p = nav.probe_gateway(verbose=True)
    print("   port  :", p["port"])
    print("   state :", p["state"])
    print("   detail:", p["detail"])
    if p["hint"]:
        print("   hint  :", p["hint"])
    for ln in p["samples"][:10]:
        print("   sample:", ln)
    ok("probe returned a known state",
       p["state"] in (nav.ST_ABSENT, nav.ST_PORT_ERROR, nav.ST_SILENT,
                      nav.ST_ONLINE_NO_BUS, nav.ST_ONLINE, nav.ST_NMEA0183,
                      nav.ST_GARBLED), p["state"])
    if p["state"] == nav.ST_SILENT:
        print("   NOTE: silent is the expected result with the N2K connector "
              "unpowered — the gateway's processor is bus-powered.")


def main():
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="navtest-")
    try:
        test_full_stack(tmp)
        test_replug()
        test_wrong_baud()
        test_absent()
        if "--hw" in sys.argv:
            test_hardware()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print()
    if _failures:
        for f in _failures:
            print("failed:", f)
        print("NAV INTEGRATION: FAIL (%d)" % len(_failures))
        return 1
    print("NAV INTEGRATION: PASS (all assertions)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

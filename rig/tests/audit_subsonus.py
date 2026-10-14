"""Audit regression suite — nav_subsonus (the Subsonus over Ethernet).

rig/nav_subsonus.py reads the Subsonus's ANPP stream over TCP and feeds it
into the same nav.NavReader state the iKonvert path fills, so this suite
pins the seams that make that true:

S1  Framing: the CRC16-CCITT check value, LRC, resynchronisation across
    garbage, a frame split across two reads.
S2  System State decode: both layouts (116-byte Subsonus, 100-byte common),
    the GNSS fix -> N2K method mapping, the layout sanity check, and that a
    packet without a fix or without UTC produces NO position and NO time word.
S3  The consumer contract through feed_line(): snapshot()/fix_at()/
    flight_row() and the TimeAuthority promote exactly as they do for the
    gateway, and never off an unset clock.
S4  Transport: a loopback fake Subsonus streams at 20 Hz; the reader comes
    online, decodes, survives the unit dropping the connection (reconnects),
    and stop() joins its thread.
S5  Raw log + replay: a run recorded through the raw hook replays through a
    fresh NavReader with the same judgements (navlog.replay()).
S6  rigd source selection: WILDSYNC_NAV beats ~/rig/nav.json beats the
    iKonvert default, read from RIG_HOME (never the operator's ~/rig).

Everything is loopback or in-process; no serial device, no hardware.
"""

import json
import os
import socket
import tempfile
import threading
import time

from soaktest import check, sect, note

HERE = os.path.dirname(os.path.abspath(__file__))

import nav                      # noqa: E402
import navlog                   # noqa: E402
import nav_subsonus as ns       # noqa: E402

FLAGS_ALL = (ns.FILTER_ORIENTATION_INIT | ns.FILTER_NAVIGATION_INIT
             | ns.FILTER_HEADING_INIT)
LAT, LON = 41.4237, -71.4538


def _state(epoch, fix=7, lat=LAT, lon=LON, heading=90.0, flags=FLAGS_ALL,
           **kw):
    return ns.encode_system_state(lat, lon, 3.5, epoch=epoch, fix=fix,
                                  heading_deg=heading, filter_extra=flags,
                                  **kw)


# ---------------------------------------------------------------------------

def _s1_framing(opts):
    sect("audit S1: ANPP framing")
    check("CRC16-CCITT(0xFFFF) check value is 0x29B1",
          ns.crc16_ccitt(b"123456789") == 0x29B1,
          hex(ns.crc16_ccitt(b"123456789")))
    pkt = ns.encode_packet(20, bytes(range(116)))
    check("LRC closes the header",
          ns.header_lrc(pkt[1], pkt[2], pkt[3] | (pkt[4] << 8)) == pkt[0])
    dec = ns.AnppDecoder()
    got = dec.feed(b"\x00\xff\x13garbage" + pkt + b"\x55")
    check("decoder resyncs over leading garbage", got == [(20, bytes(range(116)))],
          str(got)[:60])
    check("skipped bytes are counted", dec.skipped >= 10, str(dec.skipped))
    a, b = pkt[:7], pkt[7:]
    first = dec.feed(a)
    second = dec.feed(b)
    check("a frame split across reads completes on the second read",
          first == [] and second == [(20, bytes(range(116)))])
    bad = bytearray(pkt)
    bad[10] ^= 0x01
    check("a corrupted payload fails the CRC and is dropped",
          dec.feed(bytes(bad)) == [])
    big = ns.AnppDecoder()
    big.feed(b"\x07" * (ns.AnppDecoder.MAX_BUFFER + 10))
    check("garbage never grows the buffer without bound",
          len(big._buf) <= 4096, str(len(big._buf)))


def _s2_decode(opts):
    sect("audit S2: System State decode")
    t = 1792000000.25
    d = _state(t, fix=7, vn=1.0, ve=1.0, roll_deg=1.0, pitch_deg=-2.0)
    check("Subsonus layout is 116 bytes", len(d) == 116, str(len(d)))
    st = ns.decode_system_state(d)
    check("RTK fixed decodes", st["fix_str"] == "rtk_fixed", str(st["fix_str"]))
    check("RTK fixed maps to the N2K rtk_fixed method (4)", st["method"] == 4)
    check("latitude/longitude in degrees",
          round(st["lat"], 6) == LAT and round(st["lon"], 6) == LON,
          "%s %s" % (st["lat"], st["lon"]))
    check("epoch from unix seconds + microseconds",
          abs(st["epoch"] - t) < 1e-5, str(st["epoch"]))
    check("heading in degrees, wrapped (f32 noise only)",
          round(st["heading_deg"], 3) == 90.0, str(st["heading_deg"]))
    check("layout passes the sanity check", st["suspect"] is False)
    for an_fix, method in ((0, 0), (1, 1), (2, 1), (3, 2), (4, 2), (5, 3),
                           (6, 5), (7, 4)):
        got = ns.decode_system_state(_state(t, fix=an_fix))["method"]
        check("AN fix %d -> N2K method %d" % (an_fix, method), got == method,
              str(got))
    pg = dict(ns.system_state_to_pgns(st))
    check("RTK fix emits 129025 + 129029 + 127250 + 127257 + 129026",
          set(pg) == {129025, 129029, 127250, 127257, 129026},
          str(sorted(pg)))
    check("129026 COG from vN/vE (45 deg) and SOG",
          round(pg[129026]["cog_deg"], 1) == 45.0
          and round(pg[129026]["sog_mps"], 3) == 1.414,
          str(pg[129026]))
    check("127250 is a TRUE heading with no variation",
          pg[127250]["reference"] == "true"
          and pg[127250]["variation_deg"] is None)

    nofix = ns.decode_system_state(_state(None, fix=0, flags=0))
    pg = dict(ns.system_state_to_pgns(nofix))
    check("no fix: no 129025 and 129029 lat/lon None",
          129025 not in pg and pg[129029]["lat"] is None, str(sorted(pg)))
    check("no UTC: 129029 carries no epoch (TimeAuthority cannot be fed)",
          "epoch" not in pg[129029])
    check("no fix: method_str says so", pg[129029]["method_str"] == "no_fix")
    check("uninitialised filters emit no heading/attitude/COG",
          set(pg) == {129029}, str(sorted(pg)))
    utc_nofix = ns.decode_system_state(_state(t, fix=0, flags=0))
    pg = dict(ns.system_state_to_pgns(utc_nofix))
    check("UTC set but no fix: time word present, method untrusted",
          pg[129029].get("epoch") == t and pg[129029]["method"] == 0)

    common = ns.decode_system_state(_state(t, fix=2, layout="common-100"))
    check("100-byte common layout decodes (3D -> gnss)",
          common["layout"] == "common-100" and common["method"] == 1
          and round(common["lat"], 6) == LAT)
    bad = bytearray(_state(t))
    import struct
    struct.pack_into("<d", bad, 16, 7.0)
    sus = ns.decode_system_state(bytes(bad))
    check("an out-of-range latitude marks the packet suspect",
          sus["suspect"] is True)
    check("a suspect packet yields no position",
          129025 not in dict(ns.system_state_to_pgns(sus)))
    check("an unknown length decodes to None",
          ns.decode_system_state(b"\x00" * 64) is None)


def _s3_consumer_contract(opts):
    sect("audit S3: the NavReader contract through feed_line()")
    nr = nav.NavReader(port="/dev/null", auto_reopen=False)
    base = 1792000100.0
    nr.set_clock(lambda: base + 2.5)
    for i in range(3):
        nr.feed_line(ns.raw_line(20, _state(base + i)), epoch=base + i)
    snap = nr.snapshot()
    check("snapshot has the Subsonus position",
          snap["valid"] and round(snap["lat"], 4) == LAT, str(snap["lat"]))
    check("fix_source names the RTK fix", snap["fix_source"] == "rtk_fixed",
          str(snap["fix_source"]))
    check("heading_true_deg set, heading_mag_deg honestly empty",
          round(snap["heading_true_deg"], 3) == 90.0
          and snap["heading_mag_deg"] is None,
          "%s %s" % (snap["heading_true_deg"], snap["heading_mag_deg"]))
    check("time_source promotes to gps after two agreeing packets",
          snap["time_source"] == "gps", str(nr.time_authority.status()))
    check("gps offset ~0 for a matching clock",
          abs(snap["gps_offset_s"]) < 0.01, str(snap["gps_offset_s"]))
    check("gateway_online from the packet stream", snap["gateway_online"])
    check("health() counts data lines and shows the PGNs",
          nr.health()["data_lines"] > 0 and 129029 in nr.health()["pgns_seen"])
    row = nr.flight_row(base + 1.0)
    check("flight_row: lat/long/UTM zone 19T, valid",
          row["valid"] and round(row["long"], 4) == LON
          and row["utm_zone"] == "19T", str(row))
    check("flight_row: depth and magnetic heading are empty, not fabricated",
          row["depth_from_xplore9"] is None
          and row["heading_mag_xplore"] is None)
    check("no bad lines for well-formed packets", nr.health()["bad_lines"] == 0,
          str(nr.health()["bad_lines"]))

    # a unit whose clock is not UTC-initialised must never become authority
    cold = nav.NavReader(port="/dev/null", auto_reopen=False)
    cold.set_clock(lambda: base + 2.5)
    for i in range(3):
        cold.feed_line(ns.raw_line(20, _state(None, fix=7)), epoch=base + i)
    s2 = cold.snapshot()
    check("position without UTC: lat present, time_source stays host",
          s2["valid"] and s2["time_source"] == "host",
          "%s %s" % (s2["valid"], s2["time_source"]))
    check("TimeAuthority rejected them as no_epoch",
          cold.time_authority.status()["rejected"].get("no_epoch", 0) >= 3,
          str(cold.time_authority.status()["rejected"]))

    # a unit with a fix but a clock 40 s off: offset is measured, not hidden
    off = nav.NavReader(port="/dev/null", auto_reopen=False)
    off.set_clock(lambda: base + 2.5)
    for i in range(3):
        off.feed_line(ns.raw_line(20, _state(base + i + 40.0)), epoch=base + i)
    check("a 40 s GPS-vs-host offset is measured",
          off.snapshot()["time_source"] == "gps"
          and abs(off.snapshot()["gps_offset_s"] - 40.0) < 0.01,
          str(off.snapshot()["gps_offset_s"]))

    other = nav.NavReader(port="/dev/null", auto_reopen=False)
    other.feed_line("!ANPP,28,00112233", epoch=base)
    check("a non-System-State packet is not a bad line",
          other.health()["bad_lines"] == 0)
    other.feed_line("!ANPP,20,0011", epoch=base)
    check("a System State of unknown length IS a bad line",
          other.health()["bad_lines"] == 1)
    other.feed_line("!ANPP,x", epoch=base)
    check("a malformed ANPP line is a bad line, never an exception",
          other.health()["bad_lines"] == 2)


class _FakeSubsonus:
    """A loopback TCP server that streams System State at `hz` to whoever
    connects; drop() closes the current client to simulate the unit
    rebooting or the cable being pulled."""

    def __init__(self, hz=20.0, junk_prefix=b"\x00\x01\x02hello", fix=7):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(2)
        self.port = self.srv.getsockname()[1]
        self.hz = hz
        self.junk = junk_prefix
        self.fix = fix
        self.accepts = 0
        self.sent = 0
        self._client = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        self.srv.settimeout(0.2)
        while not self._stop.is_set():
            try:
                c, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                self._client = c
            self.accepts += 1
            try:
                c.sendall(self.junk)
                while not self._stop.is_set():
                    now = time.time()
                    frame = ns.encode_packet(
                        20, _state(now, fix=self.fix))
                    # an unrelated packet rides along, as on the real unit
                    frame += ns.encode_packet(28, b"\x00" * 48)
                    c.sendall(frame)
                    self.sent += 1
                    time.sleep(1.0 / self.hz)
            except OSError:
                pass
            finally:
                try:
                    c.close()
                except OSError:
                    pass
                with self._lock:
                    if self._client is c:
                        self._client = None

    def drop(self):
        with self._lock:
            c, self._client = self._client, None
        if c is not None:
            try:
                c.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            c.close()

    def close(self):
        self._stop.set()
        self.drop()
        try:
            self.srv.close()
        except OSError:
            pass
        self._t.join(timeout=2.0)


def _wait(pred, timeout=4.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.05)
    return pred()


def _s4_transport(opts):
    sect("audit S4: TCP transport against a loopback Subsonus")
    fake = _FakeSubsonus()
    rd = ns.SubsonusReader.from_spec("anpp://127.0.0.1:%d" % fake.port)
    try:
        rd.open()
        rd.start()
        check("reader reports the ANPP transport in health()",
              rd.health()["transport"] == "anpp-tcp")
        check("comes online and decodes within 2 s",
              _wait(lambda: rd.gateway_online and rd.snapshot()["valid"], 2.0),
              json.dumps(rd.health(), default=str)[:300])
        snap = rd.snapshot()
        check("position and RTK fix from the stream",
              round(snap["lat"], 4) == LAT and snap["fix_source"] == "rtk_fixed",
              str(snap)[:200])
        check("the junk prefix was skipped, not fatal",
              rd.health()["bytes_skipped"] >= len(fake.junk)
              and rd.health()["packets"] > 0, str(rd.health()["bytes_skipped"]))
        check("both packet ids are counted, only 20 is nav data",
              set(rd.health()["packet_ids"]) == {20, 28}
              and rd.health()["bad_lines"] == 0,
              str(rd.health()["packet_ids"]))
        check("time_source gps from the unit's clock (matches ours)",
              _wait(lambda: rd.snapshot()["time_source"] == "gps", 2.0),
              str(rd.time_authority.status()))
        before = rd.health()["packets"]
        fake.drop()
        check("the unit dropping the connection reconnects (second accept)",
              _wait(lambda: fake.accepts >= 2, 4.0), str(fake.accepts))
        check("packets resume after the reconnect",
              _wait(lambda: rd.health()["packets"] > before + 5, 4.0),
              "%d -> %d" % (before, rd.health()["packets"]))
        check("reopen counted", rd.health()["reopens"] >= 1,
              str(rd.health()["reopens"]))
    finally:
        rd.stop()
        fake.close()
    check("stop() joins the reader thread", rd._thread is None)
    check("socket released on stop", rd._ser is None)

    dead = ns.SubsonusReader("127.0.0.1", 1)       # nothing listens here
    try:
        dead.open()
        check("open() against a closed port raises", False)
    except RuntimeError as e:
        check("open() against a closed port raises an actionable error",
              "cannot connect" in str(e), str(e))
    dead.start()
    time.sleep(0.3)
    check("a dead unit leaves the reader offline with last_error set",
          not dead.gateway_online and dead.health()["last_error"],
          str(dead.health()["last_error"]))
    dead.stop()


def _s5_rawlog_replay(opts):
    sect("audit S5: raw log and replay")
    tmp = tempfile.mkdtemp(prefix="wildsync-audit-subsonus-")
    fake = _FakeSubsonus()
    rd = ns.SubsonusReader.from_spec("anpp://127.0.0.1:%d" % fake.port)
    log = navlog.open_run_log(tmp, run_id="audit-subsonus")
    rd.set_raw_hook(log)
    try:
        rd.open()
        rd.start()
        _wait(lambda: rd.snapshot()["time_source"] == "gps", 3.0)
        time.sleep(0.5)
        live_at = time.time() - 0.2
        live = rd.fix_at(live_at)
    finally:
        rd.stop()
        fake.close()
        log.close()
    path = os.path.join(tmp, navlog.LOG_NAME)
    lines = [l for l in open(path) if not l.startswith("#")]
    check("every packet was logged as an !ANPP line",
          lines and all(" !ANPP," in l for l in lines), str(lines[:1]))
    check("packet 28 is in the log too (nothing hidden)",
          any(" !ANPP,28," in l for l in lines))
    fresh = nav.NavReader(port="/dev/null", auto_reopen=False)
    n = navlog.replay(path, fresh)
    rep = fresh.fix_at(live_at)
    check("replay fed every line", n == len(lines), "%d vs %d" % (n, len(lines)))
    check("replay reproduces the position at the live instant",
          rep["valid"] and rep["lat"] == live["lat"]
          and rep["long"] == live["long"], "%s vs %s" % (rep["lat"],
                                                          live["lat"]))
    check("replay reproduces time_source=gps at that instant",
          rep["time_source"] == live["time_source"] == "gps",
          "%s vs %s" % (rep["time_source"], live["time_source"]))
    check("replay reproduces the gps offset",
          abs((rep["gps_offset_s"] or 0) - (live["gps_offset_s"] or 0)) < 0.01)
    note("subsonus raw log: %d lines in %s" % (len(lines), path))


def _s6_rigd_source_selection(opts):
    sect("audit S6: rigd picks the nav source from WILDSYNC_NAV / nav.json")
    import rigcore
    import rigd
    saved_home = rigcore.RIG_HOME
    saved_env = os.environ.get("WILDSYNC_NAV")
    tmp = tempfile.mkdtemp(prefix="wildsync-audit-subsonus-rig-")
    rigcore.RIG_HOME = tmp

    class _Ev:
        def __init__(self):
            self.msgs = []

        def emit(self, sev, kind, msg, **kw):
            self.msgs.append((sev, kind, msg))

    class _Stub:
        events = _Ev()
    try:
        os.environ.pop("WILDSYNC_NAV", None)
        check("no config: None (the iKonvert by-id default)",
              rigd.Rig._nav_spec(_Stub()) is None)
        with open(os.path.join(tmp, "nav.json"), "w") as fh:
            json.dump({"source": "anpp://an-subsonus-1.local"}, fh)
        check("nav.json source is used",
              rigd.Rig._nav_spec(_Stub()) == "anpp://an-subsonus-1.local")
        os.environ["WILDSYNC_NAV"] = "anpp://192.168.1.60:16719"
        check("WILDSYNC_NAV beats nav.json",
              rigd.Rig._nav_spec(_Stub()) == "anpp://192.168.1.60:16719")
        os.environ.pop("WILDSYNC_NAV", None)
        with open(os.path.join(tmp, "nav.json"), "w") as fh:
            fh.write("{not json")
        stub = _Stub()
        stub.events = _Ev()
        check("a broken nav.json falls back to the iKonvert and says so",
              rigd.Rig._nav_spec(stub) is None
              and any("unreadable" in m[2] for m in stub.events.msgs),
              str(stub.events.msgs))
        check("the spec parser accepts the documented forms",
              ns.parse_spec("anpp://an-subsonus-1.local")
              == ("an-subsonus-1.local", ns.DEFAULT_PORT)
              and ns.parse_spec("subsonus://10.0.0.5:16720")
              == ("10.0.0.5", 16720))
        try:
            ns.parse_spec("/dev/ttyUSB0")
            check("a serial path is not an anpp spec", False)
        except ValueError:
            check("a serial path is not an anpp spec", True)
    finally:
        rigcore.RIG_HOME = saved_home
        if saved_env is None:
            os.environ.pop("WILDSYNC_NAV", None)
        else:
            os.environ["WILDSYNC_NAV"] = saved_env


def suite(opts):
    _s1_framing(opts)
    _s2_decode(opts)
    _s3_consumer_contract(opts)
    _s4_transport(opts)
    _s5_rawlog_replay(opts)
    _s6_rigd_source_selection(opts)


if __name__ == "__main__":
    import argparse
    import sys
    sys.path.insert(0, HERE)
    import soaktest
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    suite(ap.parse_args())
    print("\n%d passed, %d failed" % (len(soaktest.PASS), len(soaktest.FAIL)))
    sys.exit(1 if soaktest.FAIL else 0)

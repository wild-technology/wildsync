#!/usr/bin/env python3
"""nav_subsonus — the Subsonus over Ethernet as the rig's navigation source.

The nav path to date is NMEA 2000 -> iKonvert -> USB serial (rig/nav.py). The
Subsonus surface unit also speaks Advanced Navigation's binary packet protocol
(ANPP) over TCP, on ports 16718-16721, and out of the box streams its System
State packet (ID 20) at 20 Hz on them. This module reads that stream and feeds
it into the SAME NavReader state — the per-PGN history rings, TimeAuthority,
snapshot()/fix_at()/flight_row(), the raw log and navlog.replay() — so rigd,
run.py and the UI see one nav contract whichever cable is plugged in.

Wire format (docs.advancednavigation.com/subsonus/ANPP)
-------------------------------------------------------
  header   5 bytes: LRC, packet id, data length, CRC16 low, CRC16 high
           LRC = ((id + length + crc_lo + crc_hi) ^ 0xFF) + 1, low 8 bits
           CRC16-CCITT (poly 0x1021, init 0xFFFF) over the DATA bytes only
  data     little-endian, at most 255 bytes

  packet 20 System State, Subsonus layout, 116 bytes
     0 u32 system status            4 u32 filter status
     8 u32 unix time, seconds      12 u32 microseconds
    16 f64 latitude  rad           24 f64 longitude rad     32 f64 height m
    40 f32 velocity N E D m/s      52 f32 body accel x y z   64 f32 g force
    68 f32 roll pitch heading rad  80 f32 angular rate x y z rad/s
    92 f32 sd lat lon height m    104 f32 sd roll pitch heading rad

  The published field table puts latitude/longitude/height at 20/28/36,
  which cannot be right: an 8-byte height at 36 would overlap velocity north
  at 40, and no gap at 16 is the only reading that sums to the stated 116
  bytes. Until a live unit confirms it, every decoded packet is sanity-checked
  (finite latitude within +-90, longitude within +-180) and a packet that
  fails is counted in health()["suspect_layout"] and NOT turned into a
  position. The 100-byte layout the other AN units use (u16 statuses,
  latitude at 8) is decoded too, so a Certus/Spatial on the bench works.

  filter status bits: 0 orientation filter initialised, 1 navigation filter
  initialised, 2 heading initialised, 3 UTC time initialised, 4-6 GNSS fix
  (0 none, 1 2D, 2 3D, 3 SBAS, 4 differential, 5 PPP, 6 RTK float,
  7 RTK fixed).

Nothing is fabricated
---------------------
Each System State packet becomes the N2K-shaped samples the rest of nav.py
already understands, and a value is emitted only when the unit says it is
real:

  129025 lat/lon             GNSS fix >= 2D
  129029 GNSS position       always; lat/lon None without a fix, method_str
                             names the fix (no_fix, gnss, dgnss, precise,
                             rtk_float, rtk_fixed); the time word only while
                             UTC Time Initialised, so TimeAuthority can never
                             promote to 'gps' off an unset clock
  127250 heading (true)      heading initialised
  127257 attitude            orientation filter initialised
  129026 COG/SOG             navigation filter initialised

Depth and magnetic heading are not in this packet: depth_from_xplore9 and
heading_mag_xplore stay empty (heading_true_deg is in snapshot()).

Raw log and replay
------------------
Every packet is written to the run's nmea_raw.log through the ordinary raw
hook as one line, `!ANPP,<id>,<hex data>`, and NavReader.feed_line() routes
such a line back through ingest_packet(), so navlog.replay() of a Subsonus
run reproduces its judgements exactly as it does for an iKonvert run.

Configuration
-------------
rigd picks this source when WILDSYNC_NAV, or "source" in ~/rig/nav.json, is
`anpp://<host>[:<port>]` — e.g. anpp://an-subsonus-1.local:16718. Zeroconf
names resolve through the OS. With neither set, rigd keeps the iKonvert.

    python3 rig/nav_subsonus.py anpp://an-subsonus-1.local       # live tail
    python3 rig/nav_subsonus.py --selftest
"""

import collections
import math
import socket
import struct
import sys
import time

import nav

PACKET_ACK = 0
PACKET_SYSTEM_STATE = 20
PACKET_UNIX_TIME = 21
PACKET_PERIODS = 181

DEFAULT_PORT = 16718
SRC_ADDRESS = 254   # stands in for the N2K source address; not a bus address

# Advanced Navigation GNSS fix -> the N2K 129029 method nibble nav.py already
# ranks (nav.GNSS_METHODS_TRUSTED, nav._GNSS_METHOD). 2D and 3D both map to
# the plain "gnss" fix, as a 129029 sender would report them.
_AN_FIX = {0: "none", 1: "2d", 2: "3d", 3: "sbas", 4: "differential",
           5: "ppp", 6: "rtk_float", 7: "rtk_fixed"}
_AN_FIX_TO_METHOD = {0: 0, 1: 1, 2: 1, 3: 2, 4: 2, 5: 3, 6: 5, 7: 4}

FILTER_ORIENTATION_INIT = 1 << 0
FILTER_NAVIGATION_INIT = 1 << 1
FILTER_HEADING_INIT = 1 << 2
FILTER_UTC_INIT = 1 << 3


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------

def crc16_ccitt(data, crc=0xFFFF):
    """CRC16-CCITT, polynomial 0x1021, initial 0xFFFF, no reflection — the
    ANPP data CRC. Check value: crc16_ccitt(b"123456789") == 0x29B1."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def header_lrc(pid, length, crc):
    return ((((pid + length + (crc & 0xFF) + (crc >> 8)) & 0xFF) ^ 0xFF)
            + 1) & 0xFF


def encode_packet(pid, data):
    """Frame `data` as an ANPP packet of id `pid`."""
    data = bytes(data)
    if not 0 <= pid <= 255 or len(data) > 255:
        raise ValueError("ANPP id 0..255, data up to 255 bytes")
    crc = crc16_ccitt(data)
    return bytes([header_lrc(pid, len(data), crc), pid, len(data),
                  crc & 0xFF, crc >> 8]) + data


class AnppDecoder:
    """Byte stream -> (packet id, data) frames, resynchronising on the LRC.

    feed() returns the complete frames in the bytes so far; partial frames
    wait for more bytes. A byte that does not start a frame with a valid LRC
    and CRC is skipped and counted in `skipped`."""

    MAX_BUFFER = 64 * 1024

    def __init__(self):
        self._buf = bytearray()
        self.frames = 0
        self.skipped = 0

    def reset(self):
        self._buf.clear()

    def feed(self, chunk):
        buf = self._buf
        buf += chunk
        out = []
        i, n = 0, len(buf)
        while n - i >= 5:
            lrc, pid, length, c1, c2 = buf[i:i + 5]
            if lrc == header_lrc(pid, length, c1 | (c2 << 8)):
                if n - i < 5 + length:
                    break                       # frame not complete yet
                data = bytes(buf[i + 5:i + 5 + length])
                if crc16_ccitt(data) == (c1 | (c2 << 8)):
                    out.append((pid, data))
                    self.frames += 1
                    i += 5 + length
                    continue
            i += 1
            self.skipped += 1
        del buf[:i]
        if len(buf) > self.MAX_BUFFER:         # garbage with no frames: cap it
            self.skipped += len(buf) - 4096
            del buf[:-4096]
        return out


# ---------------------------------------------------------------------------
# System State packet
# ---------------------------------------------------------------------------

_FMT_SUBSONUS = struct.Struct("<IIIIddd19f")   # 116 bytes
_FMT_COMMON = struct.Struct("<HHIIddd16f")     # 100 bytes (Certus, Spatial, ...)


def _finite(x):
    return x is not None and math.isfinite(x)


def decode_system_state(data):
    """ANPP packet 20 -> dict, or None if the length is neither layout.

    Angles come back in degrees, position in degrees/metres, `epoch` as a
    UTC unix epoch (None until the unit reports UTC Time Initialised).
    `suspect` is True when latitude/longitude are not finite or out of range
    — the sign that the offsets above do not match this firmware."""
    if len(data) == _FMT_SUBSONUS.size:
        v = _FMT_SUBSONUS.unpack(data)
        layout = "subsonus-116"
        sd_rph = v[23:26]
    elif len(data) == _FMT_COMMON.size:
        v = _FMT_COMMON.unpack(data)
        layout = "common-100"
        sd_rph = (None, None, None)
    else:
        return None
    sysstat, filt, unix_s, micros = v[0], v[1], v[2], v[3]
    lat_r, lon_r, height = v[4], v[5], v[6]
    (vn, ve, vd, ax, ay, az, gforce, roll, pitch, heading,
     wx, wy, wz, sd_lat, sd_lon, sd_h) = v[7:23]
    fix = (filt >> 4) & 0x07
    lat = math.degrees(lat_r) if _finite(lat_r) else None
    lon = math.degrees(lon_r) if _finite(lon_r) else None
    suspect = (lat is None or lon is None
               or abs(lat) > 90.0 or abs(lon) > 180.0
               or not _finite(height))
    out = {
        "layout": layout,
        "system_status": sysstat,
        "filter_status": filt,
        "orientation_init": bool(filt & FILTER_ORIENTATION_INIT),
        "navigation_init": bool(filt & FILTER_NAVIGATION_INIT),
        "heading_init": bool(filt & FILTER_HEADING_INIT),
        "utc_init": bool(filt & FILTER_UTC_INIT),
        "fix": fix,
        "fix_str": _AN_FIX.get(fix, str(fix)),
        "method": _AN_FIX_TO_METHOD.get(fix, 0),
        "unix_s": unix_s,
        "micros": micros,
        "epoch": ((unix_s + micros / 1e6)
                  if (filt & FILTER_UTC_INIT) and micros < 1000000 else None),
        "lat": lat, "lon": lon,
        "height_m": height if _finite(height) else None,
        "vn": vn, "ve": ve, "vd": vd,
        "roll_deg": math.degrees(roll) if _finite(roll) else None,
        "pitch_deg": math.degrees(pitch) if _finite(pitch) else None,
        "heading_deg": (math.degrees(heading) % 360.0
                        if _finite(heading) else None),
        "sd_lat_m": sd_lat, "sd_lon_m": sd_lon, "sd_height_m": sd_h,
        "sd_roll_deg": (math.degrees(sd_rph[0])
                        if _finite(sd_rph[0]) else None),
        "sd_pitch_deg": (math.degrees(sd_rph[1])
                         if _finite(sd_rph[1]) else None),
        "sd_heading_deg": (math.degrees(sd_rph[2])
                           if _finite(sd_rph[2]) else None),
        "suspect": suspect,
    }
    return out


def encode_system_state(lat_deg, lon_deg, height_m=0.0, epoch=None, fix=7,
                        heading_deg=0.0, roll_deg=0.0, pitch_deg=0.0,
                        vn=0.0, ve=0.0, vd=0.0, filter_extra=0,
                        layout="subsonus-116"):
    """The inverse of decode_system_state, for tests and replays."""
    filt = filter_extra | ((fix & 0x07) << 4)
    if epoch is not None:
        filt |= FILTER_UTC_INIT
        unix_s = int(epoch)
        micros = int(round((epoch - unix_s) * 1e6))
    else:
        unix_s, micros = 0, 0
    body = [math.radians(lat_deg), math.radians(lon_deg), float(height_m),
            vn, ve, vd, 0.0, 0.0, 9.81, 1.0,
            math.radians(roll_deg), math.radians(pitch_deg),
            math.radians(heading_deg), 0.0, 0.0, 0.0,
            0.02, 0.02, 0.05]
    if layout == "subsonus-116":
        return _FMT_SUBSONUS.pack(0, filt, unix_s, micros, *(body + [0.001,
                                                                   0.001,
                                                                   0.002]))
    return _FMT_COMMON.pack(0, filt, unix_s, micros, *body)


def system_state_to_pgns(st):
    """One decoded System State -> [(pgn, decoded_dict), ...] in the shapes
    nav._decode_129025/129029/127250/127257/129026 produce."""
    out = []
    has_pos = (st["fix"] >= 1 and not st["suspect"])
    if has_pos:
        out.append((129025, {"lat": st["lat"], "lon": st["lon"]}))
    epoch = st["epoch"]
    gnss = {
        "sid": None,
        "date_days": (int(epoch // 86400) if epoch is not None else None),
        "time_s": (epoch % 86400.0 if epoch is not None else None),
        "lat": st["lat"] if has_pos else None,
        "lon": st["lon"] if has_pos else None,
        "alt_m": st["height_m"] if has_pos else None,
        "sats": None, "hdop": None, "pdop": None, "geoidal_sep_m": None,
        "gnss_type": 0,
        "method": st["method"],
        "method_str": nav._GNSS_METHOD.get(st["method"], str(st["method"])),
        "an_fix": st["fix"],
        "an_fix_str": st["fix_str"],
        "sd_lat_m": st["sd_lat_m"], "sd_lon_m": st["sd_lon_m"],
        "sd_height_m": st["sd_height_m"],
    }
    if epoch is not None:
        gnss["epoch"] = epoch
    out.append((129029, gnss))
    if st["heading_init"] and st["heading_deg"] is not None:
        out.append((127250, {
            "heading_rad": math.radians(st["heading_deg"]),
            "heading_deg": st["heading_deg"],
            "deviation_deg": None, "variation_deg": None,
            "reference": "true",
            "sd_heading_deg": st["sd_heading_deg"]}))
    if st["orientation_init"]:
        out.append((127257, {"yaw_deg": st["heading_deg"],
                             "pitch_deg": st["pitch_deg"],
                             "roll_deg": st["roll_deg"]}))
    if st["navigation_init"] and _finite(st["vn"]) and _finite(st["ve"]):
        sog = math.hypot(st["vn"], st["ve"])
        out.append((129026, {"cog_ref": "true",
                             "cog_deg": math.degrees(math.atan2(st["ve"],
                                                                st["vn"]))
                             % 360.0,
                             "sog_mps": sog}))
    return out


def ingest_packet(reader, pid, data, now):
    """Feed one ANPP packet into a nav.NavReader's state.

    True: a System State packet that decoded and was recorded. None: a valid
    packet of some other id (not nav data). False: a System State packet of
    a length this module does not know."""
    counts = getattr(reader, "_anpp_ids", None)
    if counts is not None:
        counts[pid] += 1
    if pid != PACKET_SYSTEM_STATE:
        return None
    st = decode_system_state(data)
    if st is None:
        return False
    if hasattr(reader, "_last_state"):
        reader._last_state = st
    if st["suspect"] and hasattr(reader, "_suspect"):
        reader._suspect += 1
    for pgn, dec in system_state_to_pgns(st):
        reader._ingest(pgn, b"", dec, now, SRC_ADDRESS)
    return True


def ingest_line(reader, line, now):
    """`!ANPP,<id>,<hex>` (the raw-log form) -> ingest_packet()."""
    try:
        _, pid, hexdata = line.split(",", 2)
        return ingest_packet(reader, int(pid), bytes.fromhex(hexdata.strip()),
                             now)
    except (ValueError, TypeError):
        return False


def raw_line(pid, data):
    return "!ANPP,%d,%s" % (pid, bytes(data).hex())


def parse_spec(spec):
    """'anpp://host[:port]' (or subsonus://) -> (host, port)."""
    s = str(spec).strip()
    for pfx in ("anpp://", "subsonus://"):
        if s.lower().startswith(pfx):
            s = s[len(pfx):]
            break
    else:
        raise ValueError("nav spec must start with anpp:// — got %r" % spec)
    s = s.rstrip("/")
    host, _, port = s.rpartition(":")
    if not host or not port.isdigit():
        host, port = s, DEFAULT_PORT
    if not host:
        raise ValueError("nav spec has no host: %r" % spec)
    return host, int(port)


def is_spec(spec):
    return isinstance(spec, str) and spec.lower().startswith(("anpp://",
                                                               "subsonus://"))


# ---------------------------------------------------------------------------
# The reader
# ---------------------------------------------------------------------------

class SubsonusReader(nav.NavReader):
    """nav.NavReader with the Subsonus ANPP TCP stream as its byte source.

    Everything a consumer touches — snapshot(), fix_at(), flight_row(),
    health(), bus_table(), the raw hook, set_clock(), feed_line() — is the
    base class's. Only the transport and the decode differ: no serial port,
    no $PDGY handshake, and binary frames instead of lines.

    The unit streams System State at 20 Hz by default on its ANPP ports;
    nothing is written to the unit. If the stream is silent (health()
    `online` False with `packets` 0 while the socket is up), enable packet
    20 on that port in the unit's web UI.
    """

    CONNECT_TIMEOUT_S = 3.0
    READ_TIMEOUT_S = 1.0

    def __init__(self, host, port=DEFAULT_PORT, raw_log_hook=None,
                 auto_reopen=True):
        super().__init__(port="anpp://%s:%d" % (host, port), baud=0,
                         raw_log_hook=raw_log_hook, auto_reopen=auto_reopen)
        self.host = host
        self.tcp_port = int(port)
        self._decoder = AnppDecoder()
        self._anpp_ids = collections.Counter()
        self._last_state = None
        self._suspect = 0
        self._connected_at = None

    @classmethod
    def from_spec(cls, spec, **kw):
        host, port = parse_spec(spec)
        return cls(host, port, **kw)

    # -- transport ----------------------------------------------------------

    def resolve_port(self):
        return self.port

    def open(self):
        try:
            s = socket.create_connection((self.host, self.tcp_port),
                                         timeout=self.CONNECT_TIMEOUT_S)
        except OSError as exc:
            self._last_error = str(exc)
            raise RuntimeError("cannot connect to the Subsonus at %s:%d: %s"
                               % (self.host, self.tcp_port, exc))
        s.settimeout(self.READ_TIMEOUT_S)
        try:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._decoder.reset()
        self._ser = s                      # the base lifecycle's "port" slot
        self._last_error = None
        self._opened_at = self._connected_at = time.time()
        return self

    def _send_init(self):
        # Nothing to hand-shake: the unit streams unprompted.
        self._last_init_tx = time.time()

    def _send_join(self):
        self._last_init_tx = time.time()

    def _tx(self, cmd):
        pass

    def _reader(self):
        while self._run.is_set():
            s = self._ser
            if s is None:
                if not self.auto_reopen:
                    break
                if self._stopping.wait(self._backoff):
                    break
                try:
                    self.open()
                    self._reopen_count += 1
                except RuntimeError as exc:
                    self._last_error = str(exc)
                self._backoff = min(self.REOPEN_MAX_S, self._backoff * 2)
                continue
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                continue                   # gateway_online ages out on its own
            except OSError as exc:
                self._last_error = "read failed: %s" % exc
                self._close_port()
                continue
            if not chunk:
                self._last_error = "connection closed by the unit"
                self._close_port()
                continue
            now = time.time()
            mono = time.monotonic()
            for pid, data in self._decoder.feed(chunk):
                self._handle_packet(pid, data, now, mono)

    def _handle_packet(self, pid, data, now, mono):
        self._line_count += 1
        self._last_rx = now
        self._backoff = self.REOPEN_MIN_S
        self._seen_pdgy = True             # "the source has spoken", for the UI
        self._emit_raw(now, mono, raw_line(pid, data))
        if ingest_packet(self, pid, data, now) is False:
            self._bad_line_count += 1

    # -- diagnostics --------------------------------------------------------

    def health(self):
        h = super().health()
        st = self._last_state
        h.update({
            "transport": "anpp-tcp",
            "host": self.host,
            "tcp_port": self.tcp_port,
            "packets": self._decoder.frames,
            "bytes_skipped": self._decoder.skipped,
            "packet_ids": dict(sorted(self._anpp_ids.items())),
            "suspect_layout": self._suspect,
            "fix": (st["fix_str"] if st else None),
            "utc_init": (st["utc_init"] if st else None),
            "heading_init": (st["heading_init"] if st else None),
            "layout": (st["layout"] if st else None),
        })
        return h


# ---------------------------------------------------------------------------
# Self-test and live tail
# ---------------------------------------------------------------------------

def _selftest():
    fails = []

    def check(name, got, want):
        ok = got == want
        print("  %s %s%s" % ("PASS" if ok else "FAIL", name,
                             "" if ok else " — got %r want %r" % (got, want)))
        if not ok:
            fails.append(name)

    def truthy(name, cond, detail=""):
        print("  %s %s%s" % ("PASS" if cond else "FAIL", name,
                             (" — " + detail) if (detail and not cond) else ""))
        if not cond:
            fails.append(name)

    check("CRC16-CCITT check value", crc16_ccitt(b"123456789"), 0x29B1)
    pkt = encode_packet(20, b"\x01\x02\x03")
    check("LRC closes the header", header_lrc(pkt[1], pkt[2],
                                             pkt[3] | (pkt[4] << 8)), pkt[0])
    dec = AnppDecoder()
    frames = dec.feed(b"\xff\x00garbage" + pkt + pkt[:3])
    check("decoder resyncs over garbage", frames, [(20, b"\x01\x02\x03")])
    frames = dec.feed(pkt[3:])
    check("split frame completes", frames, [(20, b"\x01\x02\x03")])

    t = 1792000000.25
    data = encode_system_state(41.4237, -71.4538, 3.5, epoch=t, fix=7,
                               heading_deg=270.0, roll_deg=1.0,
                               pitch_deg=-2.0, vn=1.0, ve=1.0,
                               filter_extra=(FILTER_ORIENTATION_INIT
                                             | FILTER_NAVIGATION_INIT
                                             | FILTER_HEADING_INIT))
    check("Subsonus layout is 116 bytes", len(data), 116)
    st = decode_system_state(data)
    check("fix decodes as RTK fixed", st["fix_str"], "rtk_fixed")
    check("method maps to N2K rtk_fixed", st["method"], 4)
    check("latitude round-trips", round(st["lat"], 6), 41.4237)
    check("longitude round-trips", round(st["lon"], 6), -71.4538)
    check("epoch round-trips", round(st["epoch"], 3), round(t, 3))
    check("heading in degrees", round(st["heading_deg"], 3), 270.0)
    check("not suspect", st["suspect"], False)
    pgns = dict(system_state_to_pgns(st))
    check("129025 emitted with a fix", pgns[129025]["lat"] is not None, True)
    check("129029 method_str", pgns[129029]["method_str"], "rtk_fixed")
    check("127250 is a true heading", pgns[127250]["reference"], "true")
    check("129026 COG from vN/vE", round(pgns[129026]["cog_deg"], 1), 45.0)

    nofix = decode_system_state(encode_system_state(0.0, 0.0, 0.0, epoch=None,
                                                    fix=0))
    pg = dict(system_state_to_pgns(nofix))
    check("no fix: no 129025", 129025 in pg, False)
    check("no fix: 129029 lat is None", pg[129029]["lat"], None)
    check("no UTC: 129029 has no epoch", "epoch" in pg[129029], False)
    check("no fix: method_str", pg[129029]["method_str"], "no_fix")

    common = decode_system_state(encode_system_state(10.0, 20.0, 1.0,
                                                     epoch=t, fix=2,
                                                     layout="common-100"))
    check("100-byte layout decodes", (common["layout"], round(common["lat"],
                                                              6)),
          ("common-100", 10.0))
    bad = bytearray(data)
    struct.pack_into("<d", bad, 16, 7.0)      # 7 rad "latitude" = 401 deg
    check("out-of-range latitude is suspect",
          decode_system_state(bytes(bad))["suspect"], True)
    check("unknown length is None", decode_system_state(b"\x00" * 50), None)

    # through a NavReader: the consumer contract, time authority and replay
    nr = nav.NavReader(port="/dev/null", auto_reopen=False)
    base = 1792000100.0
    nr.set_clock(lambda: base + 2.0)
    for i in range(3):
        d = encode_system_state(41.4237, -71.4538, 3.5, epoch=base + i,
                                fix=7, heading_deg=90.0,
                                filter_extra=FILTER_HEADING_INIT
                                | FILTER_ORIENTATION_INIT)
        nr.feed_line(raw_line(20, d), epoch=base + i)
    snap = nr.snapshot()
    check("snapshot lat via feed_line", round(snap["lat"] or 0, 4), 41.4237)
    check("snapshot fix_source", snap["fix_source"], "rtk_fixed")
    check("snapshot heading_true_deg", round(snap["heading_true_deg"], 3),
          90.0)
    check("heading_mag stays empty (unknown variation)",
          snap["heading_mag_deg"], None)
    check("time_source promotes to gps", snap["time_source"], "gps")
    check("gps offset is zero for a matching clock",
          round(snap["gps_offset_s"], 3), 0.0)
    row = nr.flight_row(base + 1.0)
    check("flight_row lat", round(row["lat"], 4), 41.4237)
    check("flight_row utm zone", row["utm_zone"], "19T")
    check("flight_row valid", row["valid"], True)
    check("bus table lists the stand-in source",
          any(SRC_ADDRESS in (r.get("src") or [])
              for r in nr.bus_table().get("pgns", [])), True)

    check("spec parses host and port",
          parse_spec("anpp://an-subsonus-1.local:16719"),
          ("an-subsonus-1.local", 16719))
    check("spec defaults the port", parse_spec("subsonus://192.168.1.60"),
          ("192.168.1.60", DEFAULT_PORT))
    check("is_spec", (is_spec("anpp://x"), is_spec("/dev/ttyUSB0")),
          (True, False))

    print("\nNAV-SUBSONUS SELF-TEST: %s"
          % ("PASS (all assertions)" if not fails
             else "FAIL (%d): %s" % (len(fails), ", ".join(fails))))
    return 0 if not fails else 1


def _live_tail(spec, seconds=10.0):
    rd = SubsonusReader.from_spec(spec)
    try:
        rd.open()
    except RuntimeError as exc:
        print(exc)
        return 2
    rd.start()
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            time.sleep(1.0)
            h = rd.health()
            s = rd.snapshot()
            print("%4.1fs packets=%d ids=%s fix=%s utc=%s lat=%s lon=%s "
                  "hdg=%s time_source=%s off=%s suspect=%d"
                  % (time.time() - t0, h["packets"], h["packet_ids"],
                     h["fix"], h["utc_init"], s["lat"], s["lon"],
                     s["heading_true_deg"], s["time_source"],
                     s["gps_offset_s"], h["suspect_layout"]))
    finally:
        rd.stop()
    return 0


if __name__ == "__main__":
    if "--selftest" in sys.argv[1:]:
        sys.exit(_selftest())
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        sys.exit(2)
    secs = next((float(a.split("=", 1)[1]) for a in sys.argv[1:]
                 if a.startswith("--seconds=")), 10.0)
    sys.exit(_live_tail(args[0], secs))

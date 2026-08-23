#!/usr/bin/env python3
"""Wild Sync nav agent — Digital Yacht iKonvert NMEA2000 gateway driver.

Talks to the iKonvert NMEA2000<->USB gateway (FTDI FT232R, /dev/ttyUSB0 on the
Jetson) using the ASCII "PDGY" RAW-mode protocol, decodes the navigation PGNs
the rig needs, and exposes:

    NavReader         background serial thread, per-PGN latest value AND a
                      time-indexed history ring, .snapshot() for rigd,
                      .fix_at(epoch) for run.py's per-image flight_log row,
                      .set_raw_hook() for nmea_raw.log
    TimeAuthority     GPS-time authority (PGN 126992 / 129029) with Jetson
                      fallback, per PROTOCOL.md's time model
    latlon_to_utm     pure WGS84 -> UTM forward transform (no external geo libs)
    parse_pdgy / decode_pgn    pure parser layer, unit-testable offline
    find_ikonvert_port         robust by-id device discovery
    probe_gateway              one-shot hardware diagnosis (baud/mode/silence)

No HTTP in this file.  Stdlib + pyserial only (apt: python3-serial).

==============================================================================
HARDWARE FACTS — measured on this Jetson 2026-08-16, and confirmed against the
Digital Yacht iKonvert developer wiki.  Read this before debugging "no data".
==============================================================================

* The unit enumerates as a **stock** FTDI FT232R (0403:6001, iManufacturer
  "FTDI", iProduct "FT232R USB UART").  Digital Yacht does not reprogram the
  EEPROM, so the USB *strings* cannot tell an iKonvert apart from any other
  FT232R cable.  The **iSerial** does: this rig's gateway is `B400BIHV`.
  Identify it by `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400BIHV-if00-port0`
  (or the `/dev/ikonvert` udev symlink installed by the deploy step) and NEVER
  by `/dev/ttyUSB<n>` — the Yahboom IMU is a CH340 (1a86:7523) that will race
  the FTDI for index 0 depending on plug order.

* **The gateway's processor is powered from the NMEA2000 bus, not from USB.**
  From the DY hardware page: "The NMEA2000 interface is fully isolated (power
  and data) and the board takes its power from the NMEA2000 network (LEN=1) …
  The USB interface is powered from the USB connection and will register as a
  virtual COM port on the computer it is connected to, *even if the gateway is
  not powered up from the NMEA2000 network*."

  Consequence, and this is the single most important thing in this file:
  **with the N2K connector unplugged the gateway is DEAD SILENT and cannot
  answer any command.**  There is no offline/loopback self-test that will get a
  `$PDGY` reply out of it.  A port that opens fine and returns zero bytes at
  every baud is the *expected* signature of "no bus power", not a driver bug.
  The field check is the unit's own POWER LED: dark = no N2K power.
  Applying 12 V to the N2K micro-C connector (NET-S/NET-C) is enough — a live
  backbone with other devices is NOT required to get the gateway talking; an
  unterminated, powered stub makes it emit its off-bus status sentence.

* RAW ("PDGY") mode runs at **230400 baud** 8N1.  The other documented rates
  (4800 / 38400 / 115200) belong to the NMEA0183 *conversion* modes, in which
  the unit emits ordinary `$GPRMC`/`$SDDBT`/`$HDG` sentences and NOT `$PDGY`.
  If probe_gateway() reports `nmea0183_mode`, the unit is in a conversion mode
  and this driver will decode nothing — that is a hardware-mode problem.

------------------------------------------------------------------------------
iKonvert serial protocol (Digital Yacht iKonvert wiki, "4. Serial Protocol";
canboat's ikonvert-serial is a good cross-reference)
------------------------------------------------------------------------------
Link: 230400 baud 8N1, CRLF-terminated ASCII lines.

Note the prefixes: **binary/PGN traffic uses `!PDGY`, control traffic uses
`$PDGY`**.  (Earlier revisions of this file had that backwards.)  The parser
accepts either prefix on either kind of line, because firmware in the field has
been observed to be inconsistent and being strict buys us nothing.

Received-PGN data lines (gateway -> host):

    !PDGY,<pgn>,<priority>,<src>,<dst>,<timer>,<base64-data>

  <pgn>     decimal PGN number (0..999999)
  <priority> 0..7 CAN priority
  <src>/<dst> NMEA2000 addresses (dst 255 = broadcast)
  <timer>   gateway time since power-up (seconds, decimal fraction)
  <base64-data> the PGN payload bytes, standard base64.  There is no NMEA
            checksum on these lines, so none is verified.
  The gateway reassembles NMEA2000 fast-packet frames, so 129029's 43+ byte
  payload arrives as ONE line.

Status / ack lines (gateway -> host):

    $PDGY,000000,<busload%>,<frame-errs>,<dev-count>,<uptime-s>,<addr>,<rej-tx>
        1 Hz network status once the gateway is online and on a live bus.
    $PDGY,000000,,,,,,,        the SAME sentence with every field empty: the
        gateway is powered and talking to us but is NOT on a bus.  This is the
        "present but no traffic" signal we expect during bench bring-up.
    $PDGY,ACK,<message>        command accepted
    $PDGY,NAK,<error text>     command rejected
    $PDGY,TEXT,<free text>     informational (boot banner, etc.)

Host -> gateway commands (CRLF-terminated):

    $PDGY,N2NET_OFFLINE        take the gateway off the N2K bus (clean state)
    $PDGY,N2NET_INIT,ALL       go online and forward ALL received PGNs
                               ("NORMAL" = the default filtered subset)
    $PDGY,SHOW_LISTS           dump the configured RX/TX PGN lists
    $PDGY,RX_LIST,<pgn>,...    restrict the RX list (must precede N2NET_INIT)
    $PDGY,N2NET_RESET          factory reset
    !PDGY,<pgn>,<dst>,<base64> transmit a PGN (unused here — we are RX-only)

Init handshake implemented by NavReader:
    1. open port, flush input
    2. send  $PDGY,N2NET_OFFLINE      (idempotent; clean re-init)
    3. wait ~0.3 s
    4. send  $PDGY,N2NET_INIT,ALL
    5. consider the gateway online on the first ACK, 000000 status, or data
       line; if nothing is heard, re-send N2NET_INIT,ALL every 3 s.
       (A powered gateway with no live N2K bus still ACKs and emits the empty
       000000 status; a gateway with no BUS POWER emits nothing at all.)

------------------------------------------------------------------------------
PGN payloads decoded (all little-endian; N2K "not available" (all ones) and
"out of range" (all ones - 1) sentinels honoured -> field is None):

  129025 Position, Rapid Update   lat/lon int32 * 1e-7 deg
  129029 GNSS Position Data       SID, date u16 (days since 1970-01-01),
                                  time u32 *1e-4 s since midnight UTC,
                                  lat/lon int64 *1e-16 deg, alt int64 *1e-6 m,
                                  type/method nibbles, integrity, nSats u8,
                                  HDOP/PDOP i16 *0.01, geoidal sep i32 *0.01
  126992 System Time              SID, source nibble, date u16, time u32 *1e-4
  127250 Vessel Heading           heading u16 *1e-4 rad, deviation i16 *1e-4,
                                  variation i16 *1e-4, ref 2 bits (0=true,1=mag)
  127257 Attitude                 yaw/pitch/roll i16 *1e-4 rad
  128267 Water Depth              depth u32 *0.01 m below transducer,
                                  offset i16 *0.001 m (offset >0 =
                                  transducer->waterline, <0 = ->keel),
                                  range u8 *10 m
  129026 COG & SOG, Rapid Update  ref 2 bits, COG u16 *1e-4 rad, SOG u16 *0.01
  130306 Wind Data                speed u16 *0.01 m/s, angle u16 *1e-4 rad,
                                  reference 3 bits
"""

import base64
import binascii
import collections
import glob
import math
import os
import struct
import sys
import threading
import time

try:
    import serial  # python3-serial
except ImportError:  # parser layer stays importable without pyserial
    serial = None

__all__ = [
    "NavReader", "TimeAuthority", "latlon_to_utm", "parse_pdgy",
    "parse_pdgy_full", "classify_status", "decode_pgn", "DECODED_PGNS",
    "find_ikonvert_port", "list_serial_candidates", "probe_gateway",
    "IKONVERT_BAUD", "IKONVERT_SERIAL_NO", "FLIGHT_LOG_KEYS",
]

# The RAW/PDGY-mode line rate.  See the hardware notes above: 4800/38400/115200
# are NMEA0183 conversion-mode rates, not this.
IKONVERT_BAUD = 230400

# FTDI iSerial of the gateway fitted to this rig.  Used to pick the right
# by-id path when several USB serial adapters are present.  None = accept any
# FT232R.
IKONVERT_SERIAL_NO = "B400BIHV"

# ---------------------------------------------------------------------------
# N2K sentinel handling
# ---------------------------------------------------------------------------

_NA_U = {1: 0xFF, 2: 0xFFFF, 4: 0xFFFFFFFF, 8: 0xFFFFFFFFFFFFFFFF}
_NA_S = {1: 0x7F, 2: 0x7FFF, 4: 0x7FFFFFFF, 8: 0x7FFFFFFFFFFFFFFF}


def _field(data, off, size, signed, scale=1.0):
    """Extract a little-endian integer field.

    Returns None when the field is truncated, or when the bus signalled
    "not available" (all ones / max positive) or "out of range" (one less).
    Both sentinels collapse to None: neither is a measurement.

    NOTE the signed path deliberately compares only against the *signed*
    sentinel.  The previous revision compared the unsigned reading against
    0xFFFF first, which threw away a perfectly legal -1 (e.g. a magnetic
    variation of -0.0001 rad) as "not available".
    """
    if off + size > len(data):
        return None
    if signed:
        raw = int.from_bytes(data[off:off + size], "little", signed=True)
        if raw >= _NA_S[size] - 1:
            return None
    else:
        raw = int.from_bytes(data[off:off + size], "little", signed=False)
        if raw >= _NA_U[size] - 1:
            return None
    return raw * scale if scale != 1.0 else raw


def _u(data, off, size, scale=1.0):
    return _field(data, off, size, False, scale)


def _s(data, off, size, scale=1.0):
    return _field(data, off, size, True, scale)


def _rad2deg(v):
    return None if v is None else math.degrees(v)


# ---------------------------------------------------------------------------
# Parser layer (pure, unit-testable)
# ---------------------------------------------------------------------------

MAX_PGN = 999999


def parse_pdgy(line):
    """Parse one iKonvert ASCII line.

    Returns (pgn:int, data:bytes) for a received-PGN data line,
    (None, None) for status/ack/heartbeat/garbage lines.
    """
    full = parse_pdgy_full(line)
    if full is None:
        return (None, None)
    return (full["pgn"], full["data"])


# Recognition names for the bus-sniffing table (/api/nav/all). Decoding is
# untouched — these only label what is SEEN so unknown traffic stands out.
# Proprietary ranges matter most: that is where a sonar's private data lives.
PGN_NAMES = {
    59392: "ISO Acknowledgement", 59904: "ISO Request",
    60160: "ISO TP Data", 60416: "ISO TP Connection",
    60928: "ISO Address Claim", 65240: "ISO Commanded Address",
    126208: "NMEA Group Function", 126464: "PGN List",
    126720: "Proprietary fast-packet (addressed)",
    126992: "System Time", 126993: "Heartbeat",
    126996: "Product Information", 126998: "Configuration Information",
    127245: "Rudder", 127250: "Vessel Heading", 127251: "Rate of Turn",
    127252: "Heave", 127257: "Attitude", 127258: "Magnetic Variation",
    127488: "Engine Rapid", 127489: "Engine Dynamic",
    127505: "Fluid Level", 127508: "Battery Status",
    128259: "Speed (water referenced)", 128267: "Water Depth",
    128275: "Distance Log",
    129025: "Position Rapid", 129026: "COG & SOG Rapid",
    129029: "GNSS Position", 129033: "Time & Date",
    129283: "Cross Track Error", 129284: "Navigation Data",
    129285: "Route/WP Information",
    129539: "GNSS DOPs", 129540: "GNSS Sats in View",
    130306: "Wind Data", 130310: "Environmental (obsolete)",
    130311: "Environmental Parameters", 130312: "Temperature",
    130313: "Humidity", 130314: "Actual Pressure",
    130316: "Temperature Extended", 130576: "Trim Tab Status",
    130577: "Direction Data",
}


def pgn_name(pgn):
    n = PGN_NAMES.get(pgn)
    if n:
        return n
    if 61184 <= pgn <= 61439 or 65280 <= pgn <= 65535:
        return "PROPRIETARY single-frame (manufacturer)"
    if 130816 <= pgn <= 131071:
        return "PROPRIETARY fast-packet (manufacturer)"
    return "unknown"


def parse_pdgy_full(line):
    """Like parse_pdgy but returns {'pgn','prio','src','dst','timer','data'}
    for data lines, None otherwise.  Never raises."""
    if isinstance(line, (bytes, bytearray)):
        try:
            line = bytes(line).decode("ascii", "replace")
        except Exception:  # noqa: BLE001
            return None
    if not isinstance(line, str):
        return None
    line = line.strip()
    if not (line.startswith("$PDGY,") or line.startswith("!PDGY,")):
        return None
    parts = line.split(",")
    if len(parts) < 7:
        return None  # command echo / ACK / NAK / TEXT / short status
    try:
        pgn = int(parts[1])
    except ValueError:
        return None  # ACK/NAK/TEXT lines have a word here
    if not (0 < pgn <= MAX_PGN):
        return None  # 0 = the $PDGY,000000 network-status sentence
    try:
        data = base64.b64decode(parts[6], validate=True)
    except (binascii.Error, ValueError):
        return None
    if not data:
        return None  # a PGN line with an empty payload is malformed
    out = {"pgn": pgn, "data": data}
    try:
        out["prio"] = int(parts[2])
        out["src"] = int(parts[3])
        out["dst"] = int(parts[4])
        out["timer"] = float(parts[5])
    except ValueError:
        out.setdefault("prio", None)
        out.setdefault("src", None)
        out.setdefault("dst", None)
        out.setdefault("timer", None)
    return out


def classify_status(line):
    """Classify a non-data iKonvert line.

    'ack' | 'nak' | 'text' | 'status' (on a live bus) |
    'status_offbus' ($PDGY,000000,,,,,,, — powered, not on a bus) | None.
    """
    if isinstance(line, (bytes, bytearray)):
        line = bytes(line).decode("ascii", "replace")
    if not isinstance(line, str):
        return None
    line = line.strip()
    if not (line.startswith("$PDGY,") or line.startswith("!PDGY,")):
        return None
    parts = line.split(",")
    if len(parts) < 2:
        return None
    tag = parts[1]
    if tag == "ACK":
        return "ack"
    if tag == "NAK":
        return "nak"
    if tag == "TEXT":
        return "text"
    if tag == "000000":
        # every remaining field empty => gateway powered but off-bus
        if all(p.strip() == "" for p in parts[2:]):
            return "status_offbus"
        return "status"
    return None


def looks_like_nmea0183(line):
    """True for an ordinary NMEA0183 sentence (i.e. the gateway is in a
    conversion mode, not RAW/PDGY mode).  Used only for diagnosis."""
    if isinstance(line, (bytes, bytearray)):
        line = bytes(line).decode("ascii", "replace")
    line = line.strip()
    if len(line) < 6 or line[0] not in "$!":
        return False
    if line.startswith("$PDGY") or line.startswith("!PDGY"):
        return False
    tag = line[1:6]
    return tag.isalnum() and tag.isupper() and "," in line


def _decode_129025(d):
    return {
        "lat": _s(d, 0, 4, 1e-7),
        "lon": _s(d, 4, 4, 1e-7),
    }


_GNSS_METHOD = {
    0: "no_fix", 1: "gnss", 2: "dgnss", 3: "precise",
    4: "rtk_fixed", 5: "rtk_float", 6: "estimated", 7: "manual", 8: "simulate",
}

# Methods whose time word may be believed.  Deliberately excludes 0 (no fix),
# 6 (dead-reckoning estimate), 7 (manual entry) and 8 (simulator).
GNSS_METHODS_TRUSTED = frozenset({1, 2, 3, 4, 5})

# PGN 126992 "source" nibble: 0 GPS, 1 GLONASS, 2 radio station, 3 local
# caesium, 4 local rubidium, 5 local crystal.  Only the satellite sources are
# actually GPS time; a plotter with no fix happily transmits 126992 off its
# local crystal (i.e. whatever the operator typed in), and believing it is the
# exact bug this class exists to prevent.
SYSTIME_SOURCES_TRUSTED = frozenset({0, 1})


def _decode_129029(d):
    date = _u(d, 1, 2)                 # days since 1970-01-01
    tod = _u(d, 3, 4, 1e-4)            # seconds since midnight UTC
    out = {
        "sid": _u(d, 0, 1),
        "date_days": date,
        "time_s": tod,
        "lat": _s(d, 7, 8, 1e-16),
        "lon": _s(d, 15, 8, 1e-16),
        "alt_m": _s(d, 23, 8, 1e-6),
        "sats": _u(d, 33, 1),
        "hdop": _s(d, 34, 2, 0.01),
        "pdop": _s(d, 36, 2, 0.01),
        "geoidal_sep_m": _s(d, 38, 4, 0.01),
    }
    if len(d) > 31:
        b = d[31]
        out["gnss_type"] = b & 0x0F
        out["method"] = (b >> 4) & 0x0F
        out["method_str"] = _GNSS_METHOD.get(out["method"], str(out["method"]))
    if len(d) > 32:
        out["integrity"] = d[32] & 0x03
    if date is not None and tod is not None:
        out["epoch"] = date * 86400.0 + tod   # UTC epoch (N2K date/time is UTC)
    return out


def _decode_126992(d):
    date = _u(d, 2, 2)
    tod = _u(d, 4, 4, 1e-4)
    out = {
        "sid": _u(d, 0, 1),
        "source": (d[1] & 0x0F) if len(d) > 1 else None,
        "date_days": date,
        "time_s": tod,
    }
    if date is not None and tod is not None:
        out["epoch"] = date * 86400.0 + tod
    return out


def _decode_127250(d):
    hdg = _u(d, 1, 2, 1e-4)
    dev = _s(d, 3, 2, 1e-4)
    var = _s(d, 5, 2, 1e-4)
    ref = (d[7] & 0x03) if len(d) > 7 else None
    return {
        "heading_rad": hdg,
        "heading_deg": _rad2deg(hdg),
        "deviation_deg": _rad2deg(dev),
        "variation_deg": _rad2deg(var),
        "reference": {0: "true", 1: "magnetic"}.get(ref, None),
    }


def _decode_127257(d):
    return {
        "yaw_deg": _rad2deg(_s(d, 1, 2, 1e-4)),
        "pitch_deg": _rad2deg(_s(d, 3, 2, 1e-4)),
        "roll_deg": _rad2deg(_s(d, 5, 2, 1e-4)),
    }


def _decode_128267(d):
    return {
        "depth_m": _u(d, 1, 4, 0.01),      # below transducer
        "offset_m": _s(d, 5, 2, 0.001),    # >0 waterline, <0 keel
        "range_m": _u(d, 7, 1, 10.0),
    }


def _decode_129026(d):
    ref = (d[1] & 0x03) if len(d) > 1 else None
    cog = _u(d, 2, 2, 1e-4)
    return {
        "cog_ref": {0: "true", 1: "magnetic"}.get(ref, None),
        "cog_deg": _rad2deg(cog),
        "sog_mps": _u(d, 4, 2, 0.01),
    }


def _decode_130306(d):
    ref = (d[5] & 0x07) if len(d) > 5 else None
    ang = _u(d, 3, 2, 1e-4)
    return {
        "wind_speed_mps": _u(d, 1, 2, 0.01),
        "wind_angle_deg": _rad2deg(ang),
        "wind_ref": {0: "true_north", 1: "magnetic", 2: "apparent",
                     3: "true_boat", 4: "true_water"}.get(ref, None),
    }


_DECODERS = {
    129025: _decode_129025,
    129029: _decode_129029,
    126992: _decode_126992,
    127250: _decode_127250,
    127257: _decode_127257,
    128267: _decode_128267,
    129026: _decode_129026,
    130306: _decode_130306,
}

DECODED_PGNS = tuple(sorted(_DECODERS))

# Minimum sane payload length per PGN.  Anything shorter is a truncated or
# corrupt frame and is rejected outright rather than silently half-decoded.
_MIN_LEN = {
    129025: 8, 129029: 43, 126992: 8, 127250: 8,
    127257: 7, 128267: 8, 129026: 8, 130306: 6,
}


def decode_pgn(pgn, data):
    """Decode a PGN payload -> dict of engineering-unit fields (None where the
    bus said 'not available').  Returns None for PGNs we don't decode, or for
    a payload too short to be that PGN."""
    fn = _DECODERS.get(pgn)
    if fn is None:
        return None
    if data is None or len(data) < _MIN_LEN.get(pgn, 1):
        return None
    try:
        return fn(data)
    except Exception:  # noqa: BLE001
        return None


# ---------------------------------------------------------------------------
# WGS84 -> UTM forward (Snyder/USGS Transverse Mercator series, k0=0.9996)
# ---------------------------------------------------------------------------

_UTM_BANDS = "CDEFGHJKLMNPQRSTUVWX"


def latlon_to_utm(lat, lon):
    """WGS84 lat/lon (degrees) -> (easting_m, northing_m, zone_str e.g. '17T').

    Pure-python Transverse Mercator forward series (Snyder 1987 eq. 8-9..8-15),
    accurate to well under 1 m anywhere in a zone.  Southern hemisphere gets
    the 10,000,000 m false northing.  Norway/Svalbard zone exceptions applied.
    Verified against pyproj EPSG:326xx/327xx — see _selftest().
    """
    if lat is None or lon is None:
        raise ValueError("lat/lon required")
    if not (-80.0 <= lat <= 84.0):
        raise ValueError("latitude outside UTM range [-80, 84]")
    lon = ((lon + 180.0) % 360.0) - 180.0

    zone = int((lon + 180.0) / 6.0) + 1
    # Norway
    if 56.0 <= lat < 64.0 and 3.0 <= lon < 12.0:
        zone = 32
    # Svalbard
    if 72.0 <= lat < 84.0:
        if 0.0 <= lon < 9.0:
            zone = 31
        elif 9.0 <= lon < 21.0:
            zone = 33
        elif 21.0 <= lon < 33.0:
            zone = 35
        elif 33.0 <= lon < 42.0:
            zone = 37
    band = _UTM_BANDS[min(len(_UTM_BANDS) - 1, int((lat + 80.0) / 8.0))]

    a = 6378137.0
    f = 1.0 / 298.257223563
    k0 = 0.9996
    e2 = f * (2.0 - f)
    ep2 = e2 / (1.0 - e2)

    phi = math.radians(lat)
    # wrap the offset from the central meridian into +/-180 so a point at
    # +/-180 longitude lands in the same place from either side
    dlam = math.radians(((lon - (zone * 6.0 - 183.0) + 180.0) % 360.0) - 180.0)

    sp, cp = math.sin(phi), math.cos(phi)
    N = a / math.sqrt(1.0 - e2 * sp * sp)
    T = (sp / cp) ** 2
    C = ep2 * cp * cp
    A = cp * dlam

    e4 = e2 * e2
    e6 = e4 * e2
    M = a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
             - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * phi)
             + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * phi)
             - (35 * e6 / 3072) * math.sin(6 * phi))

    easting = k0 * N * (A + (1 - T + C) * A ** 3 / 6
                        + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * A ** 5 / 120) \
        + 500000.0
    northing = k0 * (M + N * (sp / cp) * (A * A / 2
                                          + (5 - T + 9 * C + 4 * C * C) * A ** 4 / 24
                                          + (61 - 58 * T + T * T + 600 * C - 330 * ep2)
                                          * A ** 6 / 720))
    if lat < 0:
        northing += 10000000.0
    return easting, northing, "%d%s" % (zone, band)


# ---------------------------------------------------------------------------
# TimeAuthority — GPS time (126992/129029) with Jetson fallback
# ---------------------------------------------------------------------------

class TimeAuthority:
    """PROTOCOL.md time model: when GPS time is flowing, gps_offset = gps -
    jetson and all stamps are corrected by it; otherwise offset 0 and
    time_source = 'jetson'.

    source() returns "gps" ONLY when ALL of these hold:

      1. the time word came from a satellite source — PGN 129029 with a fix
         method in GNSS_METHODS_TRUSTED, or PGN 126992 with a source nibble in
         SYSTIME_SOURCES_TRUSTED.  "No fix", dead reckoning, manual entry,
         simulator and local-crystal clocks are all rejected;
      2. the decoded epoch is inside SANE_EPOCH (rejects a misparse that lands
         in 1970 or 2400);
      3. |gps - jetson| <= MAX_OFFSET_S (rejects a plausible-looking but wrong
         decode; a real rig is chrony-disciplined and within seconds);
      4. TWO successive accepted feeds agree on the offset to within
         CONFIRM_TOL_S — one lucky garbage frame cannot promote us to "gps";
      5. the last accepted feed is younger than STALE_S.

    STALE_S is the staleness bound.  126992/129029 are 1 Hz PGNs, so 5 s means
    "four consecutive messages missed".  The moment that budget is blown the
    Jetson clock is the authority again and offset_to() returns 0.0 — a stale
    GPS offset is never applied to a timestamp.

    Feed it decoded 126992/129029 dicts (must contain 'epoch') together with
    the local receive epoch.  Thread-safe.
    """

    STALE_S = 5.0            # a GPS time word older than this is not authority
    MAX_OFFSET_S = 86400.0   # refuse an implausible gps-vs-jetson difference
    CONFIRM_TOL_S = 0.5      # two feeds must agree within this to be believed
    SANE_EPOCH = (1735689600.0, 4102444800.0)   # 2025-01-01 .. 2100-01-01

    def __init__(self):
        self._lock = threading.Lock()
        self._offset = 0.0       # gps - jetson, seconds (confirmed)
        self._fed_local = None   # local epoch of last CONFIRMED feed
        self._fed_pgn = None
        self._cand_offset = None  # unconfirmed candidate
        self._cand_local = None
        self._accepted = 0
        self._rejected = collections.Counter()

    # -- validation ---------------------------------------------------------

    def _reject(self, why):
        self._rejected[why] += 1
        return False

    def _trustworthy(self, pgn, decoded):
        """Is this time word from a real satellite fix?  Caller holds no lock."""
        if not decoded:
            return self._reject("empty")
        epoch = decoded.get("epoch")
        if epoch is None:
            return self._reject("no_epoch")
        lo, hi = self.SANE_EPOCH
        if not (lo <= epoch <= hi):
            return self._reject("epoch_insane")
        if pgn == 129029:
            method = decoded.get("method")
            if method is None:
                return self._reject("method_unknown")
            if method not in GNSS_METHODS_TRUSTED:
                return self._reject("method_%s" % _GNSS_METHOD.get(method, method))
            sats = decoded.get("sats")
            if sats is not None and sats < 1:
                return self._reject("no_sats")
        elif pgn == 126992:
            src = decoded.get("source")
            if src is None:
                return self._reject("source_unknown")
            if src not in SYSTIME_SOURCES_TRUSTED:
                return self._reject("source_%s" % src)
        else:
            return self._reject("pgn_%s" % pgn)
        return True

    # -- feeding ------------------------------------------------------------

    def feed(self, pgn, decoded, local_epoch=None):
        """Offer a decoded 126992/129029 dict.  Returns True if it was
        accepted as a candidate/confirmation, False if rejected."""
        if local_epoch is None:
            local_epoch = time.time()
        with self._lock:
            if not self._trustworthy(pgn, decoded):
                return False
            off = decoded["epoch"] - local_epoch
            if abs(off) > self.MAX_OFFSET_S:
                return self._reject("offset_too_large")
            self._accepted += 1
            fresh_cand = (self._cand_local is not None
                          and local_epoch - self._cand_local <= self.STALE_S)
            if fresh_cand and abs(off - self._cand_offset) <= self.CONFIRM_TOL_S:
                # second agreeing observation -> confirmed authority
                self._offset = off
                self._fed_local = local_epoch
                self._fed_pgn = pgn
            self._cand_offset = off
            self._cand_local = local_epoch
            return True

    # -- reading ------------------------------------------------------------

    def _fresh(self, now=None):
        if self._fed_local is None:
            return False
        return ((now if now is not None else time.time()) - self._fed_local) \
            <= self.STALE_S

    def source(self):
        """'gps' when a fresh, confirmed, satellite-sourced time word is held,
        else 'jetson'."""
        with self._lock:
            return "gps" if self._fresh() else "jetson"

    def offset_to(self, local_epoch=None):
        """Seconds to ADD to a local (Jetson) epoch to express it in GPS time.
        0.0 when falling back to Jetson time.  local_epoch is accepted for
        symmetry; the offset is modeled as constant over a run."""
        with self._lock:
            return self._offset if self._fresh() else 0.0

    def correct(self, local_epoch):
        """local epoch -> log-stamp epoch (GPS-corrected when available)."""
        return local_epoch + self.offset_to(local_epoch)

    def gps_epoch(self):
        """Best current epoch: GPS-corrected now if fresh, else Jetson now."""
        return time.time() + self.offset_to()

    def age_s(self):
        """Seconds since the last confirmed GPS time word, or None."""
        with self._lock:
            if self._fed_local is None:
                return None
            return time.time() - self._fed_local

    def status(self):
        with self._lock:
            return {
                "source": "gps" if self._fresh() else "jetson",
                "offset_s": self._offset if self._fresh() else 0.0,
                "raw_offset_s": self._offset,
                "age_s": (None if self._fed_local is None
                          else time.time() - self._fed_local),
                "stale_bound_s": self.STALE_S,
                "from_pgn": self._fed_pgn,
                "accepted": self._accepted,
                "rejected": dict(self._rejected),
                "candidate_pending": (self._cand_local is not None
                                      and self._fed_local != self._cand_local),
            }


# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------

_BY_ID_DIR = "/dev/serial/by-id"


def list_serial_candidates():
    """Every USB serial device visible, with enough detail to tell the
    iKonvert (FTDI FT232R) apart from the Yahboom IMU (CH340)."""
    out = []
    if sys.platform == "darwin":
        # macOS has no /dev/serial/by-id. The Apple/FTDI VCP driver names the
        # port /dev/cu.usbserial-<iSerial> — this rig's gateway enumerates as
        # cu.usbserial-B400BIHV — and the CH340 IMU lands on cu.wchusbserial*.
        # Identify by those names; the iSerial match stays the authority.
        for path in sorted(glob.glob("/dev/cu.*")):
            name = os.path.basename(path)
            low = name.lower()
            if "bluetooth" in low or "debug" in low:
                continue
            serial_no = name.split("-", 1)[1] if "-" in name else None
            if "wchusbserial" in low:
                kind = "ch340 (IMU?)"
            elif serial_no and serial_no == IKONVERT_SERIAL_NO:
                kind = "ikonvert?"
            elif "usbserial" in low and not IKONVERT_SERIAL_NO:
                kind = "ikonvert?"
            else:
                kind = "other"
            out.append({"by_id": path, "dev": path, "kind": kind,
                        "serial_no": serial_no})
        return out
    for path in sorted(glob.glob(os.path.join(_BY_ID_DIR, "*"))):
        try:
            target = os.path.realpath(path)
        except OSError:
            target = None
        name = os.path.basename(path)
        low = name.lower()
        if "ftdi" in low and "ft232r" in low:
            kind = "ikonvert?"
        elif "1a86" in low or "ch340" in low or "ch341" in low:
            kind = "ch340 (IMU?)"
        else:
            kind = "other"
        out.append({"by_id": path, "dev": target, "kind": kind,
                    "serial_no": _serial_no_from_by_id(name)})
    return out


def _serial_no_from_by_id(name):
    # usb-FTDI_FT232R_USB_UART_B400BIHV-if00-port0 -> B400BIHV
    stem = name.split("-if")[0]
    return stem.rsplit("_", 1)[-1] if "_" in stem else None


def find_ikonvert_port(serial_no=IKONVERT_SERIAL_NO, extra_first=("/dev/ikonvert",)):
    """Resolve the iKonvert's device path, most-specific first.

    1. an explicit udev symlink (/dev/ikonvert) if present,
    2. the /dev/serial/by-id path whose FTDI iSerial matches `serial_no`,
    3. any FTDI FT232R by-id path,
    4. None.

    Never returns a bare /dev/ttyUSB<n>: the index depends on plug order and
    the CH340 IMU will steal index 0 given the chance.
    """
    for p in extra_first or ():
        if p and os.path.exists(p):
            return p
    cands = [c for c in list_serial_candidates() if c["kind"] == "ikonvert?"]
    if serial_no:
        for c in cands:
            if c["serial_no"] == serial_no:
                return c["by_id"]
    return cands[0]["by_id"] if cands else None


# ---------------------------------------------------------------------------
# probe_gateway — one-shot hardware diagnosis
# ---------------------------------------------------------------------------

# Diagnosis codes returned by probe_gateway()["state"].
ST_ABSENT = "absent"              # no FTDI serial device present at all
ST_PORT_ERROR = "port_error"      # present but unopenable (permissions/busy)
ST_SILENT = "silent"              # opens, zero bytes at every baud
ST_ONLINE_NO_BUS = "online_no_bus"    # PDGY control lines, no PGNs
ST_ONLINE = "online"              # PGNs flowing
ST_NMEA0183 = "nmea0183_mode"     # 0183 sentences -> wrong gateway mode
ST_GARBLED = "garbled"            # bytes, but not lines -> baud mismatch

_SILENT_HINT = (
    "The iKonvert takes its power from the NMEA2000 bus (LEN=1) and is "
    "galvanically isolated from USB, so it enumerates as a serial port even "
    "when it is completely unpowered. Zero bytes at every baud almost always "
    "means the N2K connector has no 12 V on it. Check the unit's POWER LED; "
    "applying bus power (a powered stub is enough) is what makes it talk."
)


def probe_gateway(port=None, bauds=(IKONVERT_BAUD, 115200, 38400, 4800),
                  listen_s=2.0, send_init=True, verbose=False):
    """Open the gateway and work out what it is doing.  Read-only and safe:
    the only bytes written are the documented OFFLINE/INIT commands.

    Returns {'state', 'port', 'baud', 'detail', 'hint', 'samples', 'per_baud'}.
    """
    result = {"state": ST_ABSENT, "port": port, "baud": None, "detail": "",
              "hint": "", "samples": [], "per_baud": {},
              "candidates": list_serial_candidates()}
    if port is None:
        port = find_ikonvert_port()
        result["port"] = port
    if port is None:
        result["detail"] = ("no FTDI FT232R found under %s" % _BY_ID_DIR)
        result["hint"] = "plug the iKonvert's USB lead in, then re-run"
        return result
    if serial is None:
        result["state"] = ST_PORT_ERROR
        result["detail"] = "pyserial not installed (apt install python3-serial)"
        return result

    best = None
    for baud in bauds:
        try:
            ser = serial.Serial(port, baud, timeout=0.2, exclusive=True)
        except Exception as exc:  # noqa: BLE001
            result["state"] = ST_PORT_ERROR
            result["detail"] = "cannot open %s at %d: %s" % (port, baud, exc)
            if "Permission" in str(exc):
                result["hint"] = ("add the service user to the 'dialout' group "
                                  "(usermod -aG dialout <user>) and re-login")
            elif "lock" in str(exc).lower() or "temporarily unavailable" in str(exc):
                result["hint"] = ("another process already holds the port — "
                                  "almost certainly rigd's own NavReader. Read "
                                  "its view with GET /api/nav instead, or stop "
                                  "rigd before probing.")
            return result
        try:
            ser.reset_input_buffer()
            raw = _probe_listen(ser, listen_s)
            if send_init and not raw:
                _probe_write(ser, "$PDGY,N2NET_OFFLINE")
                time.sleep(0.3)
                _probe_write(ser, "$PDGY,N2NET_INIT,ALL")
                raw += _probe_listen(ser, listen_s)
        finally:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
        info = _classify_probe(raw)
        info["bytes"] = len(raw)
        result["per_baud"][baud] = {k: info[k] for k in
                                    ("state", "bytes", "pdgy", "pgn", "n0183")}
        if verbose:
            print("  %6d baud: %d bytes -> %s" % (baud, len(raw), info["state"]))
        rank = {ST_ONLINE: 4, ST_ONLINE_NO_BUS: 3, ST_NMEA0183: 2,
                ST_GARBLED: 1, ST_SILENT: 0}
        if best is None or rank[info["state"]] > rank[best[1]["state"]]:
            best = (baud, info)
        if info["state"] in (ST_ONLINE, ST_ONLINE_NO_BUS):
            break

    baud, info = best
    result["baud"] = baud
    result["state"] = info["state"]
    result["samples"] = info["lines"][:20]
    if info["state"] == ST_SILENT:
        result["detail"] = ("port opens, zero bytes received at %s"
                            % ", ".join(str(b) for b in bauds))
        result["hint"] = _SILENT_HINT
    elif info["state"] == ST_NMEA0183:
        result["detail"] = ("NMEA0183 sentences at %d baud — the gateway is in "
                            "a conversion mode, not RAW/PDGY mode" % baud)
        result["hint"] = ("set the gateway to RAW mode (all internal DIP "
                          "switches ON) so it speaks $PDGY at %d baud"
                          % IKONVERT_BAUD)
    elif info["state"] == ST_GARBLED:
        result["detail"] = "bytes received but no complete ASCII lines"
        result["hint"] = "baud mismatch or a damaged lead"
    elif info["state"] == ST_ONLINE_NO_BUS:
        result["detail"] = ("gateway alive and answering PDGY at %d baud, but "
                            "no PGNs — nothing else on the N2K bus" % baud)
        result["hint"] = "expected on the bench; connect the backbone for data"
    else:
        result["detail"] = ("gateway online at %d baud, PGNs flowing: %s"
                            % (baud, sorted(info["pgns"])))
    return result


def _probe_write(ser, cmd):
    try:
        ser.write((cmd + "\r\n").encode("ascii"))
        _drain(ser)
    except Exception:  # noqa: BLE001
        pass


def _drain(ser):
    """flush() (tcdrain) — except on macOS, where tcdrain on a pty whose
    master is not being read blocks FOREVER in the kernel (Linux ptys return
    immediately). The offline suites drive NavReader over ptys, so a bare
    flush() deadlocks navtest on a Mac before its first assertion. The write
    itself has already been handed to the kernel; drain is only a nicety."""
    if sys.platform == "darwin":
        return
    ser.flush()


def _probe_listen(ser, secs):
    t_end = time.time() + secs
    buf = b""
    while time.time() < t_end:
        try:
            n = ser.in_waiting
            chunk = ser.read(n if n else 1)
        except Exception:  # noqa: BLE001
            break
        if chunk:
            buf += chunk
    return buf


def _classify_probe(raw):
    lines = [ln.decode("ascii", "backslashreplace")
             for ln in raw.replace(b"\r\n", b"\n").split(b"\n") if ln.strip()]
    pdgy = pgns = n0183 = 0
    seen = set()
    for ln in lines:
        full = parse_pdgy_full(ln)
        if full is not None:
            pdgy += 1
            pgns += 1
            seen.add(full["pgn"])
        elif classify_status(ln) is not None:
            pdgy += 1
        elif looks_like_nmea0183(ln):
            n0183 += 1
    if pgns:
        state = ST_ONLINE
    elif pdgy:
        state = ST_ONLINE_NO_BUS
    elif n0183:
        state = ST_NMEA0183
    elif raw:
        state = ST_GARBLED
    else:
        state = ST_SILENT
    return {"state": state, "lines": lines, "pdgy": pdgy, "pgn": pgns,
            "n0183": n0183, "pgns": seen}


# ---------------------------------------------------------------------------
# NavReader — serial thread + latest-value store + history ring
# ---------------------------------------------------------------------------

# The keys flight_log.csv needs from nav, in PROTOCOL.md's column order.
FLIGHT_LOG_KEYS = ("lat", "long", "xutm", "yutm", "utm_zone",
                   "depth_from_xplore9", "heading_mag_xplore")


class NavReader:
    """Background reader for the iKonvert on a serial port.

    Usage:
        nr = NavReader()                     # auto-resolves the by-id path
        nr.set_raw_hook(navlog.RawNmeaLog(path))
        nr.open(); nr.start()
        ... nr.snapshot() ...  nr.fix_at(capture_epoch) ...
        nr.stop()

    snapshot() values are None when the source PGN is absent or stale
    (rapid-update PGNs: >3 s old; others: >10 s old).

    The reader survives the gateway being unplugged mid-run: the port is
    closed, `gateway_online` goes False, and it re-resolves the by-id path and
    reopens with backoff.  It never raises out of the thread.
    """

    RAPID_STALE_S = 3.0     # 129025, 129026, 127250, 127257
    SLOW_STALE_S = 10.0     # 129029, 126992, 128267, 130306
    INIT_RETRY_S = 3.0
    OFFBUS_REINIT_S = 10.0  # alive-but-off-bus: re-send INIT,ALL this often
    ONLINE_S = 10.0         # no line for this long => gateway_online False
    HISTORY_S = 300.0       # per-PGN ring depth, seconds
    HISTORY_N = 4096        # per-PGN ring depth, samples
    REOPEN_MIN_S = 0.5
    REOPEN_MAX_S = 15.0
    MAX_LINE_BYTES = 8192   # emit and clear if no newline arrives within this

    _RAPID = {129025, 129026, 127250, 127257}

    def __init__(self, port=None, baud=IKONVERT_BAUD, raw_log_hook=None,
                 auto_reopen=True):
        # port=None => resolve by-id at open() time (and again on every reopen)
        self.port_hint = port
        self.port = port
        self.baud = baud
        self.auto_reopen = auto_reopen
        self.time_authority = TimeAuthority()
        self._hook_lock = threading.Lock()
        self._hook = raw_log_hook
        self._ser = None
        self._thread = None
        self._run = threading.Event()      # set while the reader should run
        self._stopping = threading.Event() # set by stop(), to interrupt sleeps
        self._lock = threading.Lock()
        self._latest = {}          # pgn -> (local_epoch, decoded_dict)
        self._hist = {}            # pgn -> deque[(local_epoch, decoded_dict)]
        # EVERY PGN seen on the bus, decoded or not — the sniffing surface.
        # A device this driver has never heard of (a sonar, a proprietary
        # sensor) still shows up here with its raw payload, source address,
        # rate and age, instead of being silently dropped at decode_pgn().
        self._bus = {}             # pgn -> {"n","first","last","src","dst",
        #                                    "prio","raw","times":deque}
        self._seen_pdgy = False    # have we EVER seen a PDGY line
        self._last_rx = None       # local epoch of the last line from the port
        self._last_init_tx = 0.0
        self._last_status_line = None
        # True while the gateway reports the empty 000000 status: powered and
        # talking to us but NOT joined to the N2K bus. Legitimate only for a
        # beat after our own N2NET_OFFLINE; persisting means the INIT,ALL that
        # follows it was lost, and the reader must re-send it or the gateway
        # sits commanded-off-bus forever while looking "online".
        self._offbus = False
        self._join_backoff = 10.0    # doubles to 60 s; reset on real traffic
        self._last_error = None
        self._line_count = 0
        self._data_count = 0
        self._bad_line_count = 0
        self._reopen_count = 0
        self._backoff = self.REOPEN_MIN_S
        self._opened_at = None

    # -- raw hook -----------------------------------------------------------

    def set_raw_hook(self, hook):
        """Install (or clear, with None) the per-line raw logger.

        `hook` may be either a plain callable taking (wall_epoch, line) — the
        legacy shape rigd/run.py already use — or an object exposing
        write_line(wall_epoch, monotonic, line), which gets the full-fidelity
        path (see rig/navlog.py).
        """
        with self._hook_lock:
            self._hook = hook

    # kept as a property so old code that assigned .raw_log_hook still works
    @property
    def raw_log_hook(self):
        return self._hook

    @raw_log_hook.setter
    def raw_log_hook(self, hook):
        self.set_raw_hook(hook)

    def _emit_raw(self, wall, mono, line):
        with self._hook_lock:
            hook = self._hook
        if hook is None:
            return
        try:
            wl = getattr(hook, "write_line", None)
            if callable(wl):
                wl(wall, mono, line)
            else:
                hook(wall, line)
        except Exception:  # noqa: BLE001
            pass   # a broken logger must never kill the serial thread

    # -- lifecycle ----------------------------------------------------------

    @property
    def gateway_online(self):
        """True while the gateway is actually sending us lines.  This is
        derived from the last receive time, not a sticky flag: an unplugged or
        unpowered gateway goes False on its own."""
        lr = self._last_rx
        return lr is not None and (time.time() - lr) <= self.ONLINE_S

    def resolve_port(self):
        """Pick the device path to use now.  Re-resolved on every reopen so a
        replug that lands on a different ttyUSB index still works."""
        if self.port_hint:
            return self.port_hint
        return find_ikonvert_port()

    def open(self):
        """Open the port and send the init handshake.

        Raises RuntimeError with an explicit, actionable message when the
        device is absent or unopenable — callers (rigd) log it and carry on
        with nav disabled.
        """
        if serial is None:
            raise RuntimeError("pyserial not installed (apt install python3-serial)")
        port = self.resolve_port()
        if port is None:
            raise RuntimeError(
                "no iKonvert found: no FTDI FT232R under %s (candidates: %s)"
                % (_BY_ID_DIR, [c["by_id"] for c in list_serial_candidates()]))
        try:
            self._ser = serial.Serial(port, self.baud, timeout=1.0,
                                      exclusive=True)
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)
            hint = ""
            if "Permission" in str(exc):
                hint = " (add the service user to the 'dialout' group)"
            raise RuntimeError("cannot open %s at %d: %s%s"
                               % (port, self.baud, exc, hint))
        self.port = port
        self._last_error = None
        self._opened_at = time.time()
        # NOTE: the backoff is deliberately NOT reset here.  A port that opens
        # cleanly and then immediately errors (a dead FTDI still enumerating,
        # a stale /dev/pts) would otherwise spin at full speed forever.  The
        # backoff resets in _handle_line(), i.e. only once real data arrives.
        try:
            self._ser.reset_input_buffer()
        except Exception:  # noqa: BLE001
            pass
        self._send_init()
        return self

    def start(self):
        if self._ser is None and not self.auto_reopen:
            self.open()
        elif self._ser is None:
            try:
                self.open()
            except RuntimeError as exc:
                # start() must not fail just because the gateway is missing;
                # the thread retries with backoff and gateway_online stays False
                self._last_error = str(exc)
        self._stopping.clear()
        self._run.set()
        self._thread = threading.Thread(target=self._reader, name="nav-serial",
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._run.clear()
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._close_port()

    def _close_port(self):
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    # -- gateway init handshake --------------------------------------------

    def _tx(self, cmd):
        ser = self._ser
        if ser is None:
            return
        ser.write((cmd + "\r\n").encode("ascii"))
        _drain(ser)      # NOT flush(): tcdrain deadlocks on macOS ptys

    def _send_init(self):
        """OFFLINE (clean state) -> INIT,ALL.  See module docstring.

        Full sequence — for port open and a genuinely silent gateway ONLY.
        Never use this to re-JOIN an alive-but-off-bus gateway: the leading
        OFFLINE knocks its bus interface down again on every retry, which
        strobes the unit's LEDs and can mute it long enough to trip the
        3 s silent-gateway retry into a permanent reset loop (observed live
        2026-08-20: all LEDs flashing once per ~3 s on a powered stub)."""
        try:
            self._tx("$PDGY,N2NET_OFFLINE")
            time.sleep(0.3)
            self._tx("$PDGY,N2NET_INIT,ALL")
        except Exception as exc:  # noqa: BLE001
            self._last_error = "init write failed: %s" % exc
        self._last_init_tx = time.time()

    def _send_join(self):
        """INIT,ALL alone — rejoin the bus WITHOUT the OFFLINE knock-down,
        with exponential backoff so a bare powered stub is asked gently
        (10 s, 20 s, 40 s, then every 60 s) instead of strobed forever."""
        try:
            self._tx("$PDGY,N2NET_INIT,ALL")
        except Exception as exc:  # noqa: BLE001
            self._last_error = "join write failed: %s" % exc
        self._last_init_tx = time.time()
        self._join_backoff = min(60.0, self._join_backoff * 2.0)

    # -- reader thread ------------------------------------------------------

    def _reader(self):
        buf = b""
        while self._run.is_set():
            if self._ser is None:
                if not self.auto_reopen:
                    break
                if buf:
                    self._flush_partial(buf)
                    buf = b""
                # _stopping (not _run) is the sleep interrupt: _run is SET
                # while we are running, so waiting on it would return at once.
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
                n = self._ser.in_waiting
                chunk = self._ser.read(n if n else 1)
            except Exception as exc:  # noqa: BLE001
                # unplugged mid-run, or the FTDI went away: drop the port and
                # let the reopen path re-resolve it
                self._last_error = "read failed: %s" % exc
                self._close_port()
                continue
            now = time.time()
            mono = time.monotonic()
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    self._handle_line(raw.rstrip(b"\r"), now, mono)
                if len(buf) > self.MAX_LINE_BYTES:
                    # a wrong baud rate produces endless bytes with no newline;
                    # never let the buffer grow without bound
                    self._handle_line(buf, now, mono)
                    buf = b""
            if not self.gateway_online \
                    and now - self._last_init_tx > self.INIT_RETRY_S:
                self._send_init()
            elif self._offbus \
                    and now - self._last_init_tx > self._join_backoff:
                # Alive but off-bus: the join was lost, or the backbone is
                # genuinely absent (a powered stub). Ask again with INIT,ALL
                # ONLY — no OFFLINE first — and back off, so a stub with no
                # backbone is nudged occasionally, never strobed.
                self._send_join()
        if buf:
            self._flush_partial(buf)

    def _flush_partial(self, buf):
        """Log whatever was received but never terminated by a newline."""
        try:
            self._emit_raw(time.time(), time.monotonic(),
                           buf.decode("ascii", "backslashreplace"))
        except Exception:  # noqa: BLE001
            pass

    def _handle_line(self, raw, now, mono=None):
        if not raw:
            return
        line = raw.decode("ascii", "backslashreplace")
        self._line_count += 1
        self._last_rx = now
        # real bytes arrived: the port is genuinely good, so clear the backoff
        self._backoff = self.REOPEN_MIN_S
        self._emit_raw(now, time.monotonic() if mono is None else mono, line)
        full = parse_pdgy_full(line)
        if full is None:
            kind = classify_status(line)
            if kind is not None:
                self._seen_pdgy = True
            if kind in ("status", "status_offbus"):
                self._last_status_line = line
                self._offbus = (kind == "status_offbus")
                if kind == "status":
                    self._join_backoff = 10.0    # on a real bus again
            elif kind == "nak":
                self._last_error = "gateway NAK: %s" % line
            elif kind is None:
                self._bad_line_count += 1
            return
        self._seen_pdgy = True
        # Record EVERY data line into the bus table before deciding whether we
        # know how to decode it: hidden traffic is only hidden until listed.
        with self._lock:
            ent = self._bus.get(full["pgn"])
            if ent is None:
                if len(self._bus) < 512:      # a real bus holds a few dozen
                    ent = self._bus[full["pgn"]] = {
                        "n": 0, "first": now,
                        "src": set(), "dst": full["dst"],
                        "prio": full["prio"], "raw": b"",
                        "times": collections.deque(maxlen=12)}
            if ent is not None:
                ent["n"] += 1
                ent["last"] = now
                if len(ent["src"]) < 8:
                    ent["src"].add(full["src"])
                ent["raw"] = full["data"]
                ent["times"].append(now)
        decoded = decode_pgn(full["pgn"], full["data"])
        if decoded is None:
            return
        self._data_count += 1
        with self._lock:
            self._latest[full["pgn"]] = (now, decoded)
            ring = self._hist.get(full["pgn"])
            if ring is None:
                ring = self._hist[full["pgn"]] = collections.deque(
                    maxlen=self.HISTORY_N)
            ring.append((now, decoded))
            while ring and now - ring[0][0] > self.HISTORY_S:
                ring.popleft()
        if full["pgn"] in (126992, 129029):
            self.time_authority.feed(full["pgn"], decoded, now)

    def feed_line(self, line, epoch=None):
        """Inject one gateway line as if it had just arrived on the port.

        The replay entry point: rig/navlog.py uses it to push a recorded
        nmea_raw.log back through the decode path, so a nav problem seen on
        the water can be reproduced at a desk with no hardware at all.
        Also how the self-tests drive the reader.
        """
        if isinstance(line, str):
            line = line.encode("ascii", "backslashreplace")
        self._handle_line(bytes(line).rstrip(b"\r\n"),
                          time.time() if epoch is None else float(epoch),
                          time.monotonic())

    # -- consumers ----------------------------------------------------------

    def _stale_limit(self, pgn):
        return self.RAPID_STALE_S if pgn in self._RAPID else self.SLOW_STALE_S

    def _get(self, pgn, now):
        """Latest sample for `pgn` if it is fresh relative to `now`."""
        return self._latest_pair(pgn, now)[1]

    def _latest_pair(self, pgn, now):
        """(local_epoch, decoded) for the newest sample of `pgn`, or
        (None, None) if there is none or it is past its staleness limit."""
        with self._lock:
            ent = self._latest.get(pgn)
        if ent is None or now - ent[0] > self._stale_limit(pgn):
            return (None, None)
        return ent

    def _nearest(self, pgn, at, max_age=None):
        """(local_epoch, decoded) nearest in time to `at`, or (None, None).

        Used by fix_at() so a frame captured 2 s ago is stamped with the nav
        sample from 2 s ago, not with 'now'.
        """
        if max_age is None:
            max_age = self._stale_limit(pgn)
        with self._lock:
            ring = self._hist.get(pgn)
            items = list(ring) if ring else []
        if not items:
            return (None, None)
        best_t, best_v, best_d = None, None, None
        for t, v in reversed(items):
            d = abs(t - at)
            if best_d is None or d < best_d:
                best_t, best_v, best_d = t, v, d
            elif t < at - best_d:
                break   # ring is time-ordered; nothing earlier can be closer
        if best_d is None or best_d > max_age:
            return (None, None)
        return (best_t, best_v)

    def _blank_view(self, epoch, source_epoch=None):
        return {
            "epoch": epoch,
            "time_source": None,
            "gps_offset_s": 0.0,
            "gateway_online": self.gateway_online,
            "lat": None, "lon": None, "alt_m": None,
            "xutm": None, "yutm": None, "utm_zone": None,
            "heading_true_deg": None, "heading_mag_deg": None,
            "heading_ref": None, "variation_deg": None, "deviation_deg": None,
            "depth_m": None, "depth_offset_m": None,
            "depth_below_surface_m": None,
            "sog_mps": None, "cog_deg": None, "cog_ref": None,
            "sats": None, "hdop": None, "fix_source": None,
            "yaw_deg": None, "pitch_deg": None, "roll_deg": None,
            "wind_speed_mps": None, "wind_angle_deg": None, "wind_ref": None,
            "age_s": None, "ages": {}, "valid": False, "stale": True,
        }

    def _fill(self, snap, at, getter):
        """Shared body of snapshot()/fix_at().  `getter(pgn) -> (t, decoded)`."""
        ages = snap["ages"]

        def take(pgn):
            t, v = getter(pgn)
            if v is not None and t is not None:
                ages[pgn] = round(at - t, 3)
            return t, v

        t_pos, pos = take(129025)
        t_gnss, gnss = take(129029)
        pos_t = None
        if pos is not None and pos.get("lat") is not None:
            snap["lat"], snap["lon"] = pos.get("lat"), pos.get("lon")
            pos_t = t_pos
        if gnss is not None:
            if snap["lat"] is None and gnss.get("lat") is not None:
                snap["lat"], snap["lon"] = gnss.get("lat"), gnss.get("lon")
                pos_t = t_gnss
            snap["alt_m"] = gnss.get("alt_m")
            snap["sats"] = gnss.get("sats")
            snap["hdop"] = gnss.get("hdop")
            snap["fix_source"] = gnss.get("method_str")

        _, hdg = take(127250)
        if hdg is not None and hdg.get("heading_deg") is not None:
            snap["heading_ref"] = hdg.get("reference")
            snap["variation_deg"] = hdg.get("variation_deg")
            snap["deviation_deg"] = hdg.get("deviation_deg")
            h, var = hdg["heading_deg"], hdg.get("variation_deg")
            if hdg.get("reference") == "true":
                snap["heading_true_deg"] = h % 360.0
                if var is not None:
                    snap["heading_mag_deg"] = (h - var) % 360.0
            elif hdg.get("reference") == "magnetic":
                snap["heading_mag_deg"] = h % 360.0
                if var is not None:
                    snap["heading_true_deg"] = (h + var) % 360.0

        _, depth = take(128267)
        if depth is not None:
            snap["depth_m"] = depth.get("depth_m")          # below transducer
            snap["depth_offset_m"] = depth.get("offset_m")
            if depth.get("depth_m") is not None \
                    and depth.get("offset_m") is not None:
                # offset >0 transducer->waterline, <0 transducer->keel;
                # adding it in both cases is the N2K convention
                snap["depth_below_surface_m"] = \
                    depth["depth_m"] + depth["offset_m"]

        _, cs = take(129026)
        if cs is not None:
            snap["sog_mps"] = cs.get("sog_mps")
            snap["cog_deg"] = cs.get("cog_deg")
            snap["cog_ref"] = cs.get("cog_ref")

        _, att = take(127257)
        if att is not None:
            snap["yaw_deg"] = att.get("yaw_deg")
            snap["pitch_deg"] = att.get("pitch_deg")
            snap["roll_deg"] = att.get("roll_deg")

        _, wind = take(130306)
        if wind is not None:
            snap["wind_speed_mps"] = wind.get("wind_speed_mps")
            snap["wind_angle_deg"] = wind.get("wind_angle_deg")
            snap["wind_ref"] = wind.get("wind_ref")

        if snap["lat"] is not None and snap["lon"] is not None:
            try:
                e, n, z = latlon_to_utm(snap["lat"], snap["lon"])
                snap["xutm"], snap["yutm"], snap["utm_zone"] = e, n, z
            except ValueError:
                pass
        if pos_t is not None:
            snap["age_s"] = round(at - pos_t, 3)
        snap["valid"] = snap["lat"] is not None and snap["lon"] is not None
        snap["stale"] = not snap["valid"]
        return snap

    def snapshot(self):
        """Latest engineering-unit view; None for stale/absent fields.

        Key set is a superset of the historical one — every previously present
        key is still present with the same meaning, so rigd's /api/nav and
        run.py keep working unchanged.
        """
        now = time.time()
        ta = self.time_authority
        snap = self._blank_view(ta.correct(now))
        snap["time_source"] = ta.source()
        snap["gps_offset_s"] = ta.offset_to(now)
        snap["local_epoch"] = now
        return self._fill(snap, now, lambda pgn: self._latest_pair(pgn, now))

    def fix_at(self, epoch=None, max_age_s=None):
        """Nav state at a given capture instant — the accessor run.py wants.

        `epoch` is a LOCAL (Jetson) epoch, e.g. the GPIO EXPOSURE edge time.
        Each field is taken from the sample nearest that instant in the history
        ring rather than from "now", so a frame that landed 3 s ago is stamped
        with the boat's position 3 s ago.

        Returns the full snapshot() key set plus flight_log-shaped aliases:

            lat, long, xutm, yutm, utm_zone,
            depth_from_xplore9, heading_mag_xplore     (the CSV columns)
            epoch          GPS-corrected stamp for the requested instant
            local_epoch    the requested (Jetson) instant, verbatim
            nav_epoch      local epoch of the position sample actually used
            age_s          |nav_epoch - local_epoch| — how far we reached
            valid          True only when lat AND lon are present and fresh
            stale          not valid
            ages           {pgn: seconds} per contributing PGN
            time_source    'gps' | 'jetson'
            gateway_online bool

        Never raises; a dead gateway yields valid=False with every value None,
        which run.py writes as empty CSV cells (PROTOCOL.md: never fabricate).
        """
        at = time.time() if epoch is None else float(epoch)
        ta = self.time_authority
        snap = self._blank_view(ta.correct(at))
        snap["time_source"] = ta.source()
        snap["gps_offset_s"] = ta.offset_to(at)
        snap["local_epoch"] = at
        self._fill(snap, at, lambda pgn: self._nearest(pgn, at, max_age_s))
        # nav_epoch = when the position we used was actually received
        snap["nav_epoch"] = (None if snap["age_s"] is None
                             else at - snap["age_s"])
        # flight_log.csv column aliases (PROTOCOL.md header)
        snap["long"] = snap["lon"]
        snap["depth_from_xplore9"] = snap["depth_m"]
        snap["heading_mag_xplore"] = snap["heading_mag_deg"]
        return snap

    def flight_row(self, epoch=None, max_age_s=None):
        """Just the seven flight_log.csv nav columns, plus validity metadata.

        {'lat','long','xutm','yutm','utm_zone','depth_from_xplore9',
         'heading_mag_xplore','epoch','local_epoch','age_s','valid','stale',
         'time_source'}  — values are None (never fabricated) when unavailable.
        """
        f = self.fix_at(epoch, max_age_s)
        row = {k: f.get(k) for k in FLIGHT_LOG_KEYS}
        row.update({k: f.get(k) for k in
                    ("epoch", "local_epoch", "nav_epoch", "age_s", "valid",
                     "stale", "time_source", "gateway_online")})
        return row

    def bus_table(self):
        """Every PGN observed on the bus — decoded or not — with source
        address, rate, AGE and the raw payload. The sniffing surface: a device
        this driver has never heard of still lists here, raw, instead of
        vanishing at decode_pgn()."""
        now = time.time()
        with self._lock:
            items = [(pgn, {"n": e["n"], "first": e["first"],
                            "last": e.get("last", e["first"]),
                            "src": sorted(e["src"]), "dst": e["dst"],
                            "prio": e["prio"], "raw": e["raw"],
                            "times": list(e["times"])})
                     for pgn, e in self._bus.items()]
            latest = {p: dict(d) for p, (t, d) in self._latest.items()}
        out = []
        for pgn, e in sorted(items):
            t = e["times"]
            hz = ((len(t) - 1) / (t[-1] - t[0])
                  if len(t) >= 2 and t[-1] > t[0] else None)
            raw = e["raw"]
            out.append({
                "pgn": pgn, "name": pgn_name(pgn),
                "decoded": pgn in latest,
                "fields": latest.get(pgn),
                "src": e["src"], "dst": e["dst"], "prio": e["prio"],
                "count": e["n"],
                "hz": round(hz, 2) if hz else None,
                "age_s": round(now - e["last"], 2),
                "bytes": len(raw) if raw else 0,
                "raw_hex": (raw.hex() if isinstance(raw, (bytes, bytearray))
                            else None),
            })
        return {"present": True, "now": now, "pgns": out,
                "health": self.health()}

    def health(self):
        """Everything rigd needs to explain nav's state in /api/diag."""
        with self._lock:
            pgns = sorted(self._latest.keys())
        return {
            "port": self.port,
            "port_hint": self.port_hint,
            "baud": self.baud,
            "online": self.gateway_online,
            "seen_pdgy": self._seen_pdgy,
            "last_rx_age_s": (None if self._last_rx is None
                              else round(time.time() - self._last_rx, 3)),
            "lines": self._line_count,
            "data_lines": self._data_count,
            "bad_lines": self._bad_line_count,
            "reopens": self._reopen_count,
            "last_error": self._last_error,
            "last_status": self._last_status_line,
            "pgns_seen": pgns,
            "time": self.time_authority.status(),
        }

    def stats(self):
        """Backwards-compatible short form of health()."""
        with self._lock:
            pgns = sorted(self._latest.keys())
        return {"lines": self._line_count, "data_lines": self._data_count,
                "online": self.gateway_online, "status": self._last_status_line,
                "pgns_seen": pgns, "last_error": self._last_error,
                "reopens": self._reopen_count}


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _mkline(pgn, payload, prio=2, src=3, dst=255, timer=12.345, pfx="!"):
    return "%sPDGY,%d,%d,%d,%d,%.3f,%s" % (
        pfx, pgn, prio, src, dst, timer, base64.b64encode(payload).decode())


def _utm_krueger(lat, lon, zone=None):
    """Independent 6th-order Krueger-series TM forward, written from a
    different formulation than latlon_to_utm's Snyder series purely so the two
    can be cross-checked.  Not used at runtime."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    k0 = 0.9996
    if zone is None:
        zone = int((((lon + 180.0) % 360.0)) / 6.0) + 1
    lam0 = math.radians(zone * 6.0 - 183.0)
    n = f / (2.0 - f)
    n2, n3, n4, n5, n6 = n**2, n**3, n**4, n**5, n**6
    A = a / (1 + n) * (1 + n2 / 4 + n4 / 64 + n6 / 256)
    al = [
        n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288,
        13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630,
        61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880,
        49561 * n4 / 161280 - 179 * n5 / 168,
        34729 * n5 / 80640,
    ]
    phi = math.radians(lat)
    dl = math.radians(((lon - math.degrees(lam0) + 180.0) % 360.0) - 180.0)
    e = math.sqrt(f * (2 - f))
    t = math.sinh(math.atanh(math.sin(phi))
                  - e * math.atanh(e * math.sin(phi)))
    xi_p = math.atan(t / math.cos(dl))
    eta_p = math.atanh(math.sin(dl) / math.sqrt(1 + t * t))
    xi = xi_p + sum(al[j] * math.sin(2 * (j + 1) * xi_p)
                    * math.cosh(2 * (j + 1) * eta_p) for j in range(5))
    eta = eta_p + sum(al[j] * math.cos(2 * (j + 1) * xi_p)
                      * math.sinh(2 * (j + 1) * eta_p) for j in range(5))
    E = 500000.0 + k0 * A * eta
    N = k0 * A * xi
    if lat < 0:
        N += 10000000.0
    return E, N, zone


def _selftest(verbose=True):
    failures = []

    def check(name, got, want, tol=0.0):
        ok = (got == want) if tol == 0.0 else (
            got is not None and want is not None and abs(got - want) <= tol)
        if not ok:
            failures.append("%s: got %r want %r (tol %g)" % (name, got, want, tol))

    def truthy(name, cond, detail=""):
        if not cond:
            failures.append("%s: %s" % (name, detail or "false"))

    lat, lon = 43.642567, -79.387139

    # ======================================================================
    # 1. Parser layer: hand-built frames, byte layouts taken from the PGN
    #    specs (canboat pgns.json), NOT from this module's own encoders.
    # ======================================================================

    # --- 129025 Position Rapid Update: CN Tower ---------------------------
    # Hand-computed: 43.642567 deg * 1e7 =  436425670 = 0x1A0353C6 (LE c6 53 03 1a)
    #               -79.387139 deg * 1e7 = -793871390 = 0xD0AE7BE2 (LE e2 7b ae d0)
    hand = bytes.fromhex("c653031a") + bytes.fromhex("e27baed0")
    check("129025 hand-built == struct", hand,
          struct.pack("<ii", round(lat * 1e7), round(lon * 1e7)))
    pgn, data = parse_pdgy(_mkline(129025, hand))
    check("129025 pgn", pgn, 129025)
    dec = decode_pgn(pgn, data)
    check("129025 lat", dec["lat"], lat, 1e-7)
    check("129025 lon", dec["lon"], lon, 1e-7)
    # the same line with the '$' prefix some firmware uses must also parse
    check("129025 via $PDGY", parse_pdgy(_mkline(129025, hand, pfx="$"))[0],
          129025)

    # sentinel: lon not available (0x7FFFFFFF); and lon == -1 must NOT be N/A
    d2 = struct.pack("<iI", round(lat * 1e7), 0x7FFFFFFF)
    check("129025 lon NA", decode_pgn(129025, d2)["lon"], None)
    d3 = struct.pack("<ii", round(lat * 1e7), -1)
    check("129025 lon -1 is a value, not NA",
          decode_pgn(129025, d3)["lon"], -1e-7, 1e-12)
    d4 = struct.pack("<iI", round(lat * 1e7), 0x7FFFFFFE)   # out-of-range
    check("129025 lon out-of-range", decode_pgn(129025, d4)["lon"], None)

    # --- 129029 GNSS Position Data (fast packet, 43 bytes) ----------------
    # 2026-08-15 = 20680 days since 1970-01-01; 12:34:56.7 UTC = 45296.7 s
    date_days, tod = 20680, 45296.7
    d = struct.pack("<BHIqqq", 1, date_days, round(tod * 1e4),
                    round(lat * 1e16), round(lon * 1e16), round(76.543 * 1e6))
    d += bytes([0x10])            # type=GPS(0), method=GNSS fix(1)
    d += bytes([0xFC])            # integrity 0 + reserved
    d += bytes([12])              # 12 sats
    d += struct.pack("<hh", 90, 150)          # HDOP 0.90, PDOP 1.50
    d += struct.pack("<i", -3520)             # geoidal sep -35.20 m
    d += bytes([0])               # 0 reference stations
    check("129029 payload length", len(d), 43)
    dec = decode_pgn(129029, d)
    check("129029 lat", dec["lat"], lat, 1e-9)
    check("129029 lon", dec["lon"], lon, 1e-9)
    check("129029 alt", dec["alt_m"], 76.543, 1e-6)
    check("129029 sats", dec["sats"], 12)
    check("129029 method", dec["method"], 1)
    check("129029 method_str", dec["method_str"], "gnss")
    check("129029 gnss_type", dec["gnss_type"], 0)
    check("129029 hdop", dec["hdop"], 0.90, 1e-9)
    check("129029 pdop", dec["pdop"], 1.50, 1e-9)
    check("129029 geoidal", dec["geoidal_sep_m"], -35.20, 1e-9)
    # hand-check: 20680*86400 = 1786752000 -> epoch 1786797296.7
    check("129029 epoch", dec["epoch"], 20680 * 86400 + 45296.7, 1e-3)
    check("129029 epoch abs", dec["epoch"], 1786797296.7, 1e-3)
    check("129029 truncated rejected", decode_pgn(129029, d[:40]), None)

    # --- 126992 System Time ----------------------------------------------
    d = struct.pack("<BBHI", 7, 0xF0, date_days, round(tod * 1e4))
    dec = decode_pgn(126992, d)
    check("126992 source", dec["source"], 0)   # 0 = GPS
    check("126992 epoch", dec["epoch"], 1786797296.7, 1e-3)
    dec_x = decode_pgn(126992, struct.pack("<BBHI", 7, 0xF5, date_days,
                                           round(tod * 1e4)))
    check("126992 crystal source", dec_x["source"], 5)

    # --- 127250 Vessel Heading (magnetic, variation -9.9977 deg) ----------
    d = struct.pack("<BHhhB", 0, 15708, 0x7FFF, -1745, 0xFD)  # ref bits = 1 (mag)
    dec = decode_pgn(127250, d)
    check("127250 hdg", dec["heading_deg"], math.degrees(1.5708), 1e-6)
    check("127250 ref", dec["reference"], "magnetic")
    check("127250 dev NA", dec["deviation_deg"], None)
    check("127250 var", dec["variation_deg"], math.degrees(-0.1745), 1e-6)
    # true-referenced heading must yield a magnetic heading via the variation
    d = struct.pack("<BHhhB", 0, 15708, 0x7FFF, -1745, 0xFC)  # ref bits = 0
    dec = decode_pgn(127250, d)
    check("127250 ref true", dec["reference"], "true")

    # --- 128267 Water Depth ----------------------------------------------
    d = struct.pack("<BIhB", 0, 1234, 500, 0xFF)
    dec = decode_pgn(128267, d)
    check("128267 depth", dec["depth_m"], 12.34, 1e-9)
    check("128267 offset", dec["offset_m"], 0.5, 1e-9)
    check("128267 range NA", dec["range_m"], None)
    d = struct.pack("<BIhB", 0, 4321, -300, 20)
    dec = decode_pgn(128267, d)
    check("128267 keel offset", dec["offset_m"], -0.3, 1e-9)
    check("128267 range", dec["range_m"], 200.0, 1e-9)

    # --- 129026 COG & SOG -------------------------------------------------
    d = struct.pack("<BBHHH", 0, 0xFC, 15708, 350, 0xFFFF)  # ref=true
    dec = decode_pgn(129026, d)
    check("129026 cog", dec["cog_deg"], math.degrees(1.5708), 1e-6)
    check("129026 sog", dec["sog_mps"], 3.50, 1e-9)
    check("129026 ref", dec["cog_ref"], "true")

    # --- 130306 Wind ------------------------------------------------------
    d = struct.pack("<BHHBH", 0, 1050, 7854, 0xFA, 0xFFFF)
    dec = decode_pgn(130306, d)
    check("130306 speed", dec["wind_speed_mps"], 10.50, 1e-9)
    check("130306 angle", dec["wind_angle_deg"], math.degrees(0.7854), 1e-6)
    check("130306 ref", dec["wind_ref"], "apparent")

    # --- status/ack lines and malformed input -----------------------------
    check("status heartbeat", parse_pdgy("$PDGY,000000,23,0,1542,120,34,0"),
          (None, None))
    check("status classify", classify_status("$PDGY,000000,23,0,1542,120,34,0"),
          "status")
    check("off-bus status classify", classify_status("$PDGY,000000,,,,,,,"),
          "status_offbus")
    check("off-bus status not data", parse_pdgy("$PDGY,000000,,,,,,,"),
          (None, None))
    check("ack", parse_pdgy("$PDGY,ACK,N2NET_INIT"), (None, None))
    check("ack classify", classify_status("$PDGY,ACK,N2NET_INIT"), "ack")
    check("nak classify", classify_status("$PDGY,NAK,bad command"), "nak")
    check("text classify", classify_status("$PDGY,TEXT,iKonvert v1.04"), "text")
    for junk in ("", "\x00\x00\xff", "$GPRMC,123519,A,4807.038,N*7F",
                 "$PDGY,129025,2,3,255,1.0,!!!!not-base64!!!!",
                 "$PDGY,129025,2,3,255,1.0,",
                 "$PDGY,129025,2,3,255", "$PDGY", "PDGY,1,2,3,4,5,6",
                 "$PDGY,notanumber,2,3,255,1.0,AAAA",
                 "$PDGY,-7,2,3,255,1.0,AAAA",
                 "$PDGY,129025,x,y,z,q," + base64.b64encode(hand).decode()):
        got = parse_pdgy(junk)
        if junk.startswith("$PDGY,129025,x"):
            check("malformed-but-decodable %r" % junk[:24], got[0], 129025)
        else:
            check("garbage rejected %r" % junk[:28], got, (None, None))
    check("bytes input accepted", parse_pdgy(_mkline(129025, hand).encode())[0],
          129025)
    check("nmea0183 detector", looks_like_nmea0183("$GPRMC,123519,A,4807.0,N"),
          True)
    check("nmea0183 detector rejects pdgy",
          looks_like_nmea0183("$PDGY,000000,,,,,,,"), False)

    # ======================================================================
    # 2. TimeAuthority — the "source()=='gps' only on a genuinely fresh fix"
    #    contract, exercised against every way it has been wrong before.
    # ======================================================================
    now = time.time()

    def gnss(method=1, dt=0.0, sats=9, at=None):
        """A decoded 129029 whose GPS epoch runs `dt` ahead of local time."""
        return {"epoch": (now if at is None else at) + dt, "method": method,
                "method_str": _GNSS_METHOD.get(method), "sats": sats}

    ta = TimeAuthority()
    check("TA starts on jetson", ta.source(), "jetson")
    check("TA offset 0", ta.offset_to(now), 0.0)
    check("TA age None", ta.age_s(), None)

    # a SINGLE good frame must not be enough
    truthy("TA accepts good 129029", ta.feed(129029, gnss(dt=2.5), now))
    check("TA one frame is still jetson", ta.source(), "jetson")
    # a second frame one second later, still 2.5 s ahead of local -> agrees
    truthy("TA accepts 2nd 129029",
           ta.feed(129029, gnss(dt=2.5, at=now + 1.0), now + 1.0))
    check("TA confirmed -> gps", ta.source(), "gps")
    check("TA offset", ta.offset_to(now), 2.5, 1e-6)
    check("TA correct", ta.correct(now), now + 2.5, 1e-6)

    # staleness: past STALE_S the Jetson takes over and the offset is dropped
    ta._fed_local = time.time() - (TimeAuthority.STALE_S + 0.5)
    check("TA stale -> jetson", ta.source(), "jetson")
    check("TA stale offset dropped", ta.offset_to(), 0.0)
    ta._fed_local = time.time() - (TimeAuthority.STALE_S - 0.5)
    check("TA just-fresh -> gps", ta.source(), "gps")

    # every untrustworthy provenance must be refused
    for name, pgn, dec in (
        ("no-fix", 129029, gnss(method=0)),
        ("dead-reckoning", 129029, gnss(method=6)),
        ("manual", 129029, gnss(method=7)),
        ("simulator", 129029, gnss(method=8)),
        ("unknown method", 129029, {"epoch": now}),
        ("zero sats", 129029, gnss(sats=0)),
        ("local crystal", 126992, {"epoch": now, "source": 5}),
        ("radio station", 126992, {"epoch": now, "source": 2}),
        ("unknown source", 126992, {"epoch": now}),
        ("epoch 1970", 129029, {"epoch": 0.0, "method": 1, "sats": 9}),
        ("epoch year 2400", 129029, {"epoch": 1.4e10, "method": 1, "sats": 9}),
        ("empty", 129029, None),
        ("wrong pgn", 129025, {"epoch": now, "method": 1}),
    ):
        t = TimeAuthority()
        truthy("TA refuses %s (feed)" % name, not t.feed(pgn, dec, now))
        t.feed(pgn, dec, now + 1.0)
        check("TA refuses %s (source)" % name, t.source(), "jetson")

    # an absurd offset is refused even from a nominally good fix
    t = TimeAuthority()
    truthy("TA refuses huge offset",
           not t.feed(129029, gnss(dt=2 * TimeAuthority.MAX_OFFSET_S), now))

    # two DISAGREEING frames must not confirm (one garbage frame can't promote)
    t = TimeAuthority()
    t.feed(129029, gnss(dt=2.0, at=now), now)
    t.feed(129029, gnss(dt=40.0, at=now + 1.0), now + 1.0)
    check("TA disagreeing pair stays jetson", t.source(), "jetson")
    t.feed(129029, gnss(dt=40.0, at=now + 2.0), now + 2.0)
    check("TA agreeing pair confirms", t.source(), "gps")
    check("TA settles on the confirmed offset", t.offset_to(), 40.0, 1e-6)

    st = t.status()
    truthy("TA status shape",
           set(("source", "offset_s", "age_s", "stale_bound_s", "accepted",
                "rejected")) <= set(st), str(sorted(st)))
    check("TA status stale bound", st["stale_bound_s"], TimeAuthority.STALE_S)

    # ======================================================================
    # 3. latlon_to_utm — reference coordinates + an independent series
    # ======================================================================
    # Reference points.  Every easting/northing below was produced by pyproj
    # 3.7.2 / PROJ (EPSG:4326 -> EPSG:326xx/327xx, always_xy) on 2026-08-16 and
    # pasted in verbatim, so this is a check against an independent,
    # authoritative implementation rather than against ourselves.
    for name, la, lo, ez, ee, en in (
        # CN Tower, Toronto — northern hemisphere, mid-zone
        ("CN Tower", 43.642567, -79.387139, "17T", 630084.3008, 4833438.5857),
        # Sydney Opera House — southern hemisphere false northing
        ("Sydney", -33.856784, 151.215297, "56H", 334900.2613, 6252290.5224),
        # equator on a central meridian — the degenerate case
        ("origin z31", 0.0, 3.0, "31N", 500000.0000, 0.0),
        # 40 N on the zone-13 central meridian
        # (40 N is the S/T band boundary; band T covers 40..48)
        ("Boulder CM", 40.0, -105.0, "13T", 500000.0000, 4427757.2187),
        # antimeridian, western edge of zone 1
        ("dateline", 0.0, -180.0, "1N", 166021.4431, 0.0),
        # Norway zone-32 widening exception
        ("Norway", 60.0, 5.0, "32V", 276979.9264, 6658157.2024),
        # Svalbard zone-33 exception
        ("Svalbard", 78.0, 15.0, "33X", 500000.0000, 8658369.5858),
    ):
        e, n, z = latlon_to_utm(la, lo)
        check("UTM %s zone" % name, z, ez)
        check("UTM %s easting" % name, e, ee, 0.05)
        check("UTM %s northing" % name, n, en, 0.05)

    # cross-check the Snyder series against an independently written
    # Krueger series over a grid spanning both hemispheres and zone edges
    worst = 0.0
    worst_at = None
    for la in (-79.0, -60.0, -33.9, -10.0, 0.0, 10.0, 43.6, 60.0, 71.0, 83.0):
        for lo in (-179.0, -105.0, -79.4, -3.1, 0.0, 2.9, 100.0, 151.2, 179.9):
            if 56.0 <= la < 64.0 and 3.0 <= lo < 12.0:
                continue    # Norway exception: zones differ by design
            if 72.0 <= la < 84.0 and 0.0 <= lo < 42.0:
                continue    # Svalbard exception
            e1, n1, z1 = latlon_to_utm(la, lo)
            e2, n2, _ = _utm_krueger(la, lo, zone=int(z1[:-1]))
            d = math.hypot(e1 - e2, n1 - n2)
            if d > worst:
                worst, worst_at = d, (la, lo)
    truthy("UTM Snyder vs Krueger agree < 0.5 m", worst < 0.5,
           "worst %.3f m at %s" % (worst, worst_at))
    if verbose:
        print("UTM cross-check: worst Snyder-vs-Krueger delta %.4f m at %s"
              % (worst, worst_at))

    # +180 and -180 are the same meridian and must land in the same place
    e1, n1, z1 = latlon_to_utm(0.0, -180.0)
    e2, n2, z2 = latlon_to_utm(0.0, 180.0)
    check("UTM +/-180 same zone", z1, z2)
    check("UTM +/-180 same easting", e1, e2, 1e-6)
    try:
        latlon_to_utm(85.0, 0.0)
        failures.append("UTM should reject lat 85")
    except ValueError:
        pass
    try:
        latlon_to_utm(None, 0.0)
        failures.append("UTM should reject None")
    except ValueError:
        pass

    # ======================================================================
    # 4. NavReader end-to-end over a fake serial port (no hardware)
    # ======================================================================
    nr = NavReader(port="/dev/null", auto_reopen=False)
    logged = []
    nr.set_raw_hook(lambda ep, ln: logged.append((ep, ln)))

    t0 = time.time()
    lines = [
        "$PDGY,TEXT,iKonvert booting",
        "$PDGY,000000,,,,,,,",                      # off-bus status
        _mkline(129025, hand),
        _mkline(129029, d if False else struct.pack("<BHIqqq", 1, date_days,
                round(tod * 1e4), round(lat * 1e16), round(lon * 1e16),
                round(76.543 * 1e6)) + bytes([0x10, 0xFC, 12])
                + struct.pack("<hh", 90, 150) + struct.pack("<i", -3520)
                + bytes([0])),
        _mkline(127250, struct.pack("<BHhhB", 0, 15708, 0x7FFF, -1745, 0xFD)),
        _mkline(128267, struct.pack("<BIhB", 0, 1234, 500, 0xFF)),
        _mkline(129026, struct.pack("<BBHHH", 0, 0xFC, 15708, 350, 0xFFFF)),
        "garbage that is not a sentence at all",
    ]
    for i, ln in enumerate(lines):
        nr._handle_line(ln.encode(), t0 - 0.5 + i * 0.01, time.monotonic())

    check("reader logged every line verbatim", len(logged), len(lines))
    check("reader kept the garbage line", logged[-1][1], lines[-1])
    truthy("reader saw pdgy", nr._seen_pdgy)
    truthy("reader reports online", nr.gateway_online)
    check("reader counted data lines", nr._data_count, 5)
    check("reader counted bad lines", nr._bad_line_count, 1)

    snap = nr.snapshot()
    check("snapshot lat", snap["lat"], lat, 1e-7)
    check("snapshot lon", snap["lon"], lon, 1e-7)
    check("snapshot depth", snap["depth_m"], 12.34, 1e-9)
    check("snapshot depth below surface", snap["depth_below_surface_m"],
          12.84, 1e-9)
    check("snapshot heading mag", snap["heading_mag_deg"],
          math.degrees(1.5708) % 360.0, 1e-6)
    truthy("snapshot heading true derived",
           snap["heading_true_deg"] is not None)
    check("snapshot sats", snap["sats"], 12)
    check("snapshot utm zone", snap["utm_zone"], "17T")
    check("snapshot xutm", snap["xutm"], 630084.0, 1.5)
    check("snapshot sog", snap["sog_mps"], 3.50, 1e-9)
    truthy("snapshot valid", snap["valid"])
    # 129029's time word came from a real fix, but only one of them arrived,
    # so the two-strike rule keeps us on the Jetson clock.
    check("snapshot time_source after 1 gnss frame", snap["time_source"],
          "jetson")

    # legacy key set must not have regressed
    for k in ("epoch", "time_source", "gps_offset_s", "gateway_online", "lat",
              "lon", "alt_m", "heading_true_deg", "heading_mag_deg",
              "heading_ref", "variation_deg", "depth_m", "depth_offset_m",
              "sog_mps", "cog_deg", "cog_ref", "sats", "hdop", "fix_source",
              "yaw_deg", "pitch_deg", "roll_deg", "wind_speed_mps",
              "wind_angle_deg", "wind_ref", "xutm", "yutm", "utm_zone"):
        truthy("snapshot key %s present" % k, k in snap)

    # fix_at() at the capture instant, and the flight_log column aliases
    fix = nr.fix_at(t0)
    truthy("fix_at valid", fix["valid"])
    check("fix_at long alias", fix["long"], fix["lon"])
    check("fix_at depth alias", fix["depth_from_xplore9"], fix["depth_m"])
    check("fix_at heading alias", fix["heading_mag_xplore"],
          fix["heading_mag_deg"])
    truthy("fix_at age_s small", abs(fix["age_s"]) < 1.0, str(fix["age_s"]))
    truthy("fix_at nav_epoch set", fix["nav_epoch"] is not None)
    row = nr.flight_row(t0)
    for k in FLIGHT_LOG_KEYS:
        truthy("flight_row has %s" % k, k in row)
    truthy("flight_row lat", abs(row["lat"] - lat) < 1e-7)
    truthy("flight_row valid", row["valid"])

    # reaching back further than the history allows must go invalid, not lie
    old = nr.fix_at(t0 - 3600.0)
    truthy("fix_at ancient epoch is invalid", not old["valid"])
    check("fix_at ancient lat is None", old["lat"], None)
    check("fix_at ancient depth is None", old["depth_from_xplore9"], None)
    check("fix_at ancient time_source", old["time_source"], "jetson")

    # a reader that never saw a byte must report cleanly, not crash
    dead = NavReader(port="/dev/null", auto_reopen=False)
    dsnap = dead.snapshot()
    truthy("dead reader not online", not dsnap["gateway_online"])
    truthy("dead reader not valid", not dsnap["valid"])
    check("dead reader lat", dsnap["lat"], None)
    drow = dead.flight_row()
    truthy("dead reader flight_row all None",
           all(drow[k] is None for k in FLIGHT_LOG_KEYS))
    truthy("dead reader health", isinstance(dead.health(), dict))
    check("dead reader time_source", drow["time_source"], "jetson")

    # gateway_online must decay: it is derived from the last receive time
    nr._last_rx = time.time() - (NavReader.ONLINE_S + 1.0)
    truthy("gateway_online decays when the gateway stops",
           not nr.gateway_online)

    # a hook that throws must not break the reader
    nr.set_raw_hook(lambda ep, ln: (_ for _ in ()).throw(RuntimeError("boom")))
    nr._handle_line(b"$PDGY,000000,,,,,,,", time.time(), time.monotonic())
    truthy("throwing hook survived", True)
    nr.set_raw_hook(None)

    # ---- discovery --------------------------------------------------------
    cands = list_serial_candidates()
    truthy("list_serial_candidates returns a list", isinstance(cands, list))
    if verbose:
        print("serial candidates:", cands or "(none)")
        print("resolved iKonvert port:", find_ikonvert_port())

    if verbose:
        print("decoded PGNs:", DECODED_PGNS)
    if failures:
        for fl in failures:
            print("FAIL", fl)
        print("SELF-TEST: FAIL (%d)" % len(failures))
        return False
    print("SELF-TEST: PASS (all assertions)")
    return True


def _live_tail(port=None, seconds=6.0, baud=IKONVERT_BAUD):
    port = port or find_ikonvert_port()
    print("--- live tail on %s (%.0f s @ %d) ---" % (port, seconds, baud))
    if port is None:
        print("no iKonvert present; candidates: %s" % (list_serial_candidates(),))
        return
    lines = []
    nr = NavReader(port, baud=baud,
                   raw_log_hook=lambda ep, ln: lines.append((ep, ln)))
    try:
        nr.open()
    except Exception as exc:  # noqa: BLE001
        print("port not usable (%s) — skipping live tail" % exc)
        return
    nr.start()
    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            time.sleep(2.0)
            snap = nr.snapshot()
            brief = {k: v for k, v in snap.items() if v is not None}
            print("snapshot:", brief)
    finally:
        nr.stop()
    print("health:", nr.health())
    for ep, ln in lines[:20]:
        print("raw %.3f %s" % (ep, ln))
    if not lines:
        print("no serial traffic. " + _SILENT_HINT)


def _print_probe(p):
    print("iKonvert probe")
    print("  port      : %s" % p["port"])
    print("  state     : %s" % p["state"])
    print("  baud      : %s" % p["baud"])
    print("  detail    : %s" % p["detail"])
    if p["hint"]:
        print("  hint      : %s" % p["hint"])
    for b, r in sorted(p["per_baud"].items()):
        print("  %8d  : %d bytes, %d pdgy, %d pgn, %d nmea0183 -> %s"
              % (b, r["bytes"], r["pdgy"], r["pgn"], r["n0183"], r["state"]))
    for ln in p["samples"][:10]:
        print("  sample    : %s" % ln)
    print("  candidates:")
    for c in p["candidates"]:
        print("    %-70s %-14s %s" % (c["by_id"], c["kind"], c["dev"]))


def _usage():
    print("usage: nav.py [--selftest] [--probe] [--tail[=SECONDS]] "
          "[--port=PATH] [--baud=N] [--no-live]")
    print("  --selftest  pure-layer assertions only (no hardware touched)")
    print("  --probe     open the gateway and diagnose baud / mode / silence")
    print("  --tail=N    stream snapshots for N seconds")
    print("  (no args)   --selftest followed by a 6 s live tail")


if __name__ == "__main__":
    args = sys.argv[1:]
    opt_port = next((a.split("=", 1)[1] for a in args
                     if a.startswith("--port=")), None)
    opt_baud = int(next((a.split("=", 1)[1] for a in args
                         if a.startswith("--baud=")), IKONVERT_BAUD))
    if "--help" in args or "-h" in args:
        _usage()
        sys.exit(0)
    if "--probe" in args:
        _print_probe(probe_gateway(opt_port, verbose=True))
        sys.exit(0)
    tail = next((a for a in args if a.startswith("--tail")), None)
    if tail:
        secs = float(tail.split("=", 1)[1]) if "=" in tail else 10.0
        _live_tail(opt_port, secs, opt_baud)
        sys.exit(0)
    ok = _selftest()
    if "--no-live" not in args and "--selftest" not in args:
        _live_tail(opt_port, 6.0, opt_baud)
    sys.exit(0 if ok else 1)

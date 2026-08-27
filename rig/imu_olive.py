#!/usr/bin/env python3
"""imu_olive — driver for the Olive Robotics olixVision X1 IMU (second unit).

WRITTEN BLIND (2026-08-27): the unit is mounted at cam2 but not yet cabled, so
its wire format could not be confirmed against hardware the way imu_yb's was.
Everything here is therefore built around one rule: when this driver cannot
decode what the device says, it must SHOW the operator the raw bytes rather
than silently publish nothing — probe() and checksum_state() both carry a hex
tail of the undecoded stream, so tomorrow's bring-up starts from "here is what
it speaks", not from a blank {present:false}.

The unit is ROS 2 native (USB-C, enumerating as CDC-ACM) and can be configured
for a raw streaming mode, so it will appear either as /dev/ttyACM* on cam2's
Pi or as UDP datagrams to the Jetson host. Three transports behind one codec:

    SerialTransport  /dev/ttyACM* via pyserial (lazy import). CDC-ACM ignores
                     baud, so 115200 is nominal. Opened EXCLUSIVE: imu_yb's
                     acquire loop dwell-probes /dev/ttyACM* every few seconds
                     looking for 0x7E frames, and two readers on one tty split
                     the byte stream between them — exclusive=True makes the
                     yb probe fail fast instead of siphoning our frames.
    UdpTransport     bound to a configurable port (default 9901, env
                     OLIVE_UDP_PORT); each datagram is one frame.
    SimTransport     synthetic NMEA-ish stream in SI units, for offline tests
                     and so the whole decode path can be audited with no
                     hardware at all.

The codec is deliberately tolerant. Candidate encodings, tried until one
"locks" (LOCK_N consecutive good records), rejected-with-a-count afterwards:

    json       one JSON object per line/datagram; keys mapped by name,
               ROS-message shapes included (orientation{x,y,z,w},
               angular_velocity{x,y,z}, linear_acceleration{x,y,z})
    nmea_csv   $TAG,f1,f2,...*hh — XOR checksum VERIFIED when present
    csv        bare comma-separated floats
    bin_u8 /   length-prefixed binary: [len:u8|u16le][len bytes of float32].
    bin_u16    Plausibility-gated (finite, |v|<1e7) so a garbage stream
               cannot lock a framing.
    udp_f32/64 a bare float vector per datagram (an optional 4-byte CDR
               encapsulation header is skipped) — the "raw streaming" case.

Numeric records use ONE documented field-order guess (see _map_numeric):
[device_ts?][qw qx qy qz | roll pitch yaw][ax ay az][gx gy gz][mx my mz][temp]
— a guess is unavoidable offline, so it is written down here, surfaced as the
locked codec name in /health, and cheap to correct tomorrow in one place.

Units: samples are published in the rig's units (degrees, g, dps) with the
SAME key names as imu_yb, so flight_log needs no changes. A ROS-native stream
is SI (rad, m/s², rad/s); which one the device speaks is INFERRED from
gravity: the median |accel| of the first 16 inertial records is ~1 in g and
~9.8 in m/s², a separation no boat motion can blur. Until the gate decides,
samples are buffered (bounded), never published in maybe-wrong units; the
verdict is published as unit_mode in every sample and in checksum_state(), so
"the numbers are in spec" is checkable, not assumed. Quaternions need no such
inference and are preferred for attitude whenever present.

Timestamping is ARRIVAL-BASED (the read/recvfrom instant): unlike imu_yb
there is no measured wire cadence to count byte-times against, and pretending
otherwise would fabricate precision. This is a bring-up-grade epoch — good to
a few ms on an idle Pi, worse under load — and refining it (device_ts
correlation, cadence model) is explicitly tomorrow's work once the real
stream is on a bench. Do not feed these epochs into anything that needs
better than ~10 ms until then.

Public interface — the imu_yb duck-type, exactly (piagent's Imu slot calls
these and only these):
    probe(spec=None) -> dict   (present, chip, port, baud, rates, notes,
                                raw_tail_hex when undecodable)
    ImuReader(port=..., baud=...).open()/read()/start()/stop()/close()
    .latest()/.window(t0,t1)/.rate_hz()/.frame_rate_hz()/.rate_measured()
    .rejected_frames()/.last_attitude_epoch()/.checksum_state()
    .orientation_frozen_s()
Sample keys match imu_yb where the quantity is the same (pitch roll yaw
heading ax..az gx..gz mx..mz temp pressure_pa qw..qz epoch fresh); extras:
src (transport), unit_mode, device_ts (when the stream carries one).

Nothing in read()/_ingest() ever raises: every decode failure increments a
counter that /health surfaces, because this process also owns the fire path.

    python3 imu_olive.py [spec]    # probe then stream 20 samples
    python3 imu_olive.py sim       # the same, no hardware
spec grammar (also the PIAGENT_IMU2 suffix): /dev/ttyACMn | udp[:port] |
<port-digits> | sim[:rate_hz]
"""

import glob
import json
import math
import os
import socket
import struct
import sys
import threading
import time

try:
    import serial
except Exception:  # noqa: BLE001
    serial = None

BAUD = 115200                    # nominal; CDC-ACM ignores it
UDP_PORT_DEFAULT = int(os.environ.get("OLIVE_UDP_PORT", "9901"))
CHIP = "olive_olixvision_x1"
RING_S = 65                      # PROTOCOL.md: the ring holds at least 60 s
G0 = 9.80665                     # standard gravity, for the SI->g conversion

# Only CDC-ACM devices are scanned in auto mode. NEVER /dev/ttyUSB*: that is
# the YB unit's CH340 on cam1, and a dwell there would steal bytes from the
# slot-1 reader mid-run.
SERIAL_PORTS = ("/dev/ttyACM*",)


def _fmt(v, n=1):
    return "--" if v is None else ("%.*f" % (n, v))


def _quat_to_euler(q):
    """(qw,qx,qy,qz) -> (roll,pitch,yaw) degrees. Same convention as imu_yb,
    duplicated rather than imported so this one file can be dropped onto a
    node alone during bring-up."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ---------------------------------------------------------------------------
# Record mapping — wire values to a neutral record dict
# {quat|euler, accel, gyro, mag, temp, device_ts}, units still as-received.
# ---------------------------------------------------------------------------
def _plaus(v):
    """Plausibility for a binary-decoded float. The subnormal floor matters:
    arbitrary garbage bytes decode overwhelmingly to denormals (~1e-38) and
    huge exponents, and without the floor a garbage stream can LOCK a binary
    framing (seen in the offline audit) — no real IMU quantity is a nonzero
    1e-6 in any admissible unit."""
    return v == 0.0 or (math.isfinite(v) and 1e-6 <= abs(v) < 1e7)


def _floats(fields):
    try:
        vals = [float(f) for f in fields]
    except (TypeError, ValueError):
        return None
    return vals if all(math.isfinite(v) for v in vals) else None


def _is_float(s):
    try:
        float(s)
        return True
    except (TypeError, ValueError):
        return False


def _map_numeric(vals):
    """The documented field-order guess for unlabeled numeric records.

    [device_ts?] [qw qx qy qz | roll pitch yaw] [ax ay az] [gx gy gz]
    [mx my mz] [temp]

    device_ts is claimed by a leading value > 1e5 (an epoch or a device tick
    counter; no attitude/rate/field quantity in any plausible unit reaches
    that). A leading 4-tuple of unit norm with every element <= 1 is a
    quaternion — self-checking, so it can never be confused with euler+ax.
    Everything else is positional. If the real device disagrees, this ONE
    function is where tomorrow's fix goes, and until then the per-field
    plausibility gates below keep a wrong guess from publishing garbage as
    attitude (the reject count and raw tail in /health show it instead)."""
    v = list(vals)
    rec = {}
    if v and v[0] > 1e5:
        rec["device_ts"] = v.pop(0)
    if len(v) >= 4:
        n = math.sqrt(sum(x * x for x in v[:4]))
        # TIGHT norm gate, unlike imu_yb's 0.9..1.1: there the frame TYPE said
        # quat and the norm only screened corruption; here the norm IS the
        # classifier, and a small-angle euler record with yaw near 1 rad plus
        # a small ax lands inside a loose band (seen in the offline audit). A
        # fusion output is normalized to float32 rounding, so 1e-3 is generous.
        if 0.999 < n < 1.001 and all(abs(x) <= 1.0005 for x in v[:4]):
            rec["quat"] = tuple(v[:4])
            v = v[4:]
    if "quat" not in rec and len(v) >= 3:
        e = tuple(v[:3])
        # rad or deg both admissible pre-inference; +/-720 rejects only what
        # cannot be an angle in EITHER unit.
        if all(abs(x) <= 720.0 for x in e):
            rec["euler"] = e
            v = v[3:]
        else:
            return None                       # first triple is not an attitude
    if len(v) >= 3 and all(abs(x) < 400.0 for x in v[:3]):   # <=40 g in m/s²
        rec["accel"] = tuple(v[:3])
        v = v[3:]
    if len(v) >= 3 and all(abs(x) < 4000.0 for x in v[:3]):  # dps or rad/s
        rec["gyro"] = tuple(v[:3])
        v = v[3:]
    if len(v) >= 3:
        rec["mag"] = tuple(v[:3])
        v = v[3:]
    if v and -60.0 < v[0] < 150.0:
        rec["temp"] = v[0]
    return rec if ("euler" in rec or "quat" in rec or "accel" in rec) else None


def _vec(v, n):
    """A 3- or 4-vector from JSON: {x,y,z[,w]} dict or a bare list."""
    if isinstance(v, dict):
        try:
            if n == 4:
                return (float(v["w"]), float(v["x"]),
                        float(v["y"]), float(v["z"]))
            return tuple(float(v[k]) for k in ("x", "y", "z"))
        except (KeyError, TypeError, ValueError):
            return None
    if isinstance(v, (list, tuple)) and len(v) == n:
        try:
            t = tuple(float(x) for x in v)
        except (TypeError, ValueError):
            return None
        if n == 4:
            # Bare-list quaternion order is a guess: ROS ships x,y,z,w (w
            # last, near ±1 when level); take w-first only when the first
            # element dominates. Documented, cheap to flip at bring-up.
            return t if abs(t[0]) >= abs(t[3]) else (t[3], t[0], t[1], t[2])
        return t
    return None


def _map_json(d):
    if not isinstance(d, dict):
        return None
    low = {str(k).lower(): v for k, v in d.items()}
    rec = {}
    for k in ("quaternion", "quat", "orientation", "q"):
        if k in low:
            q = _vec(low[k], 4)
            if q and all(map(math.isfinite, q)) \
                    and 0.9 < math.sqrt(sum(x * x for x in q)) < 1.1:
                rec["quat"] = q
            break
    if "quat" not in rec:
        try:
            e = tuple(float(low[k]) for k in ("roll", "pitch", "yaw"))
            if all(map(math.isfinite, e)):
                rec["euler"] = e
        except (KeyError, TypeError, ValueError):
            for k in ("euler", "rpy", "attitude"):
                if k in low:
                    e = _vec(low[k], 3)
                    if e and all(map(math.isfinite, e)):
                        rec["euler"] = e
                    break
    for names, key in (
            (("accel", "acc", "linear_acceleration", "linear_accel"), "accel"),
            (("gyro", "gyr", "angular_velocity", "angular_vel"), "gyro"),
            (("mag", "magnetic_field", "magnetometer"), "mag")):
        for k in names:
            if k in low:
                v = _vec(low[k], 3)
                if v and all(map(math.isfinite, v)):
                    rec[key] = v
                break
    for k in ("temp", "temperature"):
        if k in low and isinstance(low[k], (int, float)):
            rec["temp"] = float(low[k])
            break
    for k in ("ts", "timestamp", "stamp", "time", "t"):
        if k in low and isinstance(low[k], (int, float)):
            rec["device_ts"] = float(low[k])
            break
    return rec if ("euler" in rec or "quat" in rec or "accel" in rec) else None


# ---------------------------------------------------------------------------
# Codec — bytes in, neutral records out, with a lock-on so a decided stream
# stops auditioning and starts REJECTING (counted, surfaced).
# ---------------------------------------------------------------------------
class _Codec:
    LOCK_N = 4          # consecutive good records before a candidate is law
    TAIL_N = 64         # raw bytes kept for the bring-up report
    BUF_MAX = 8192

    def __init__(self):
        self.buf = bytearray()
        self.locked = None
        self.rejects = 0
        self.bytes_seen = 0
        self.records = 0
        self.tail = b""
        self._streak = {}

    def _won(self, name):
        self._streak[name] = self._streak.get(name, 0) + 1
        if self.locked is None and self._streak[name] >= self.LOCK_N:
            self.locked = name

    def feed(self, chunk, framed=False):
        """Decode `chunk`; return a list of record dicts (possibly empty)."""
        if not chunk:
            return []
        self.bytes_seen += len(chunk)
        self.tail = (self.tail + bytes(chunk))[-self.TAIL_N:]
        if framed:
            recs = self._feed_framed(bytes(chunk))
        else:
            self.buf.extend(chunk)
            recs = self._feed_stream()
            if len(self.buf) > self.BUF_MAX:
                # A stream we cannot frame: keep only a tail so a format we DO
                # eventually sync to still can, and COUNT the discard — silence
                # here would read upstream as "no data arriving".
                self.rejects += 1
                del self.buf[:-256]
        self.records += len(recs)
        return recs

    # -- byte-stream path (serial, sim) -----------------------------------
    def _feed_stream(self):
        ascii_locked = self.locked in ("json", "nmea_csv", "csv")
        bin_locked = self.locked is not None and not ascii_locked
        # The line path may only CONSUME when the buffer actually looks like
        # text: float32 payloads contain 0x0A one byte in 256, and splitting a
        # binary stream on it discards bytes and wrecks the frame alignment
        # the binary scanner needs (seen in the offline audit).
        if not bin_locked and b"\n" in self.buf \
                and (ascii_locked or self._ascii_ish()):
            recs = self._feed_lines()
            if recs or ascii_locked:
                return recs
        return self._feed_binary()

    def _ascii_ish(self):
        seg = bytes(self.buf[:128])
        if not seg:
            return False
        ok = sum(1 for b in seg if 32 <= b < 127 or b in (9, 10, 13))
        return ok >= 0.9 * len(seg)

    def _feed_lines(self):
        recs = []
        while True:
            i = self.buf.find(b"\n")
            if i < 0:
                break
            raw = bytes(self.buf[:i])
            del self.buf[:i + 1]
            line = raw.strip(b"\r\n\x00 \t")
            if not line:
                continue
            got = self._parse_line(line)
            if got is None:
                self.rejects += 1
            else:
                name, rec = got
                self._won(name)
                recs.append(rec)
        return recs

    def _parse_line(self, line):
        try:
            text = line.decode("ascii")
        except UnicodeDecodeError:
            return None
        if text.startswith("{"):
            try:
                rec = _map_json(json.loads(text))
            except ValueError:
                return None
            return ("json", rec) if rec else None
        if text.startswith("$"):
            body = text[1:]
            if "*" in body:
                body, ck = body.rsplit("*", 1)
                try:
                    want = int(ck.strip(), 16)
                except ValueError:
                    return None
                have = 0
                for ch in body:
                    have ^= ord(ch)
                if have != want:
                    return None            # corrupt on the wire — counted
            fields = body.split(",")
            if fields and not _is_float(fields[0]):
                fields = fields[1:]        # $TAG,...
            vals = _floats(fields)
            if not vals:
                return None
            rec = _map_numeric(vals)
            return ("nmea_csv", rec) if rec else None
        vals = _floats(text.split(","))
        if not vals or len(vals) < 4:
            return None
        rec = _map_numeric(vals)
        return ("csv", rec) if rec else None

    def _feed_binary(self):
        for name, psize in (("bin_u16", 2), ("bin_u8", 1)):
            if self.locked is not None and self.locked != name:
                continue
            recs, consumed = self._scan_prefixed(psize)
            if recs:
                del self.buf[:consumed]
                for _ in recs:
                    self._won(name)
                return recs
        if self.locked is None and len(self.buf) > 512:
            # Resync crawl: nothing frames from the front, so shed bytes and
            # count them. self.tail keeps the evidence for probe()/health.
            del self.buf[:256]
            self.rejects += 1
        return []

    def _scan_prefixed(self, psize):
        buf, i, n, recs = self.buf, 0, len(self.buf), []
        while i + psize <= n:
            length = buf[i] if psize == 1 else (buf[i] | (buf[i + 1] << 8))
            if length < 12 or length > 256 or length % 4:
                break                       # not this framing (or corrupt)
            if i + psize + length > n:
                break                       # incomplete — wait for the rest
            vals = struct.unpack("<%df" % (length // 4),
                                 bytes(buf[i + psize:i + psize + length]))
            if not all(_plaus(v) for v in vals):
                break                       # implausible floats: wrong framing
            rec = _map_numeric(list(vals))
            if rec is None:
                break
            recs.append(rec)
            i += psize + length
        return recs, i

    # -- datagram path (UDP): the datagram boundary IS the frame -----------
    def _feed_framed(self, pkt):
        line = pkt.strip(b"\r\n\x00 \t")
        if line and (line[:1] in (b"{", b"$")
                     or all(32 <= b < 127 for b in line)):
            got = self._parse_line(line)
            if got:
                self._won(got[0])
                return [got[1]]
        # CDR-ish: an optional 4-byte encapsulation header (0x00 0x00/0x01 +
        # options) is SKIPPED, not parsed, then the payload is tried as a bare
        # float32 then float64 vector — the raw streaming mode's likely shape.
        # Whole packet FIRST: a real encap header decodes to a subnormal float
        # (~1e-40) that _plaus refuses, so the stripped retry runs exactly for
        # genuine CDR packets; stripping first mis-shifted any packet whose
        # legitimate first float began 0x00 0x01/0x00.
        bodies = [pkt]
        if len(pkt) >= 8 and pkt[0] == 0 and pkt[1] in (0, 1):
            bodies.append(pkt[4:])
        for body in bodies:
            for name, fmt, size in (("udp_f32", "<%df", 4),
                                    ("udp_f64", "<%dd", 8)):
                if len(body) >= size * 4 and len(body) % size == 0:
                    vals = struct.unpack(fmt % (len(body) // size), body)
                    if all(_plaus(v) for v in vals):
                        rec = _map_numeric(list(vals))
                        if rec:
                            self._won(name)
                            return [rec]
        self.rejects += 1
        return []

    def state(self):
        # Superset of imu_yb's checksum_state keys so /health embeds either
        # driver's dict unchanged. raw_tail_hex only while undecoded: once
        # locked it would just be a rolling copy of a healthy stream.
        undecoded = self.locked is None and self.bytes_seen > 0
        return {"algo": self.locked,
                "dormant": undecoded and self.records == 0,
                "learning": self.locked is None and self.records > 0,
                "rejects": self.rejects,
                "bytes_seen": self.bytes_seen,
                "records": self.records,
                "raw_tail_hex": self.tail.hex() if undecoded else None}


# ---------------------------------------------------------------------------
# Unit inference — gravity separates SI from rig units beyond argument.
# ---------------------------------------------------------------------------
class _UnitGate:
    """Decide once whether the stream is SI (ROS: rad, m/s², rad/s) or already
    in rig units (deg, g, dps), from the median |accel| of the first N
    inertial records: ~1 in g, ~9.8 in m/s² — boat motion cannot blur a 10x
    separation. Samples are BUFFERED until the verdict (publishing first and
    inferring later would put maybe-radians into a flight_log). A stream with
    no usable accel decides "unknown" (pass-through) at the timeout, and the
    verdict is published everywhere so it can be checked, not trusted."""

    N = 16
    TIMEOUT_S = 2.0

    def __init__(self):
        self.mode = None                 # None=undecided, "rig"|"si"|"unknown"
        self._mags = []
        self._t0 = None

    def offer(self, rec, now):
        if self.mode is not None:
            return
        if self._t0 is None:
            self._t0 = now
        a = rec.get("accel")
        if a:
            self._mags.append(math.sqrt(sum(x * x for x in a)))
        if len(self._mags) >= self.N or now - self._t0 > self.TIMEOUT_S:
            self.force()

    def force(self):
        if self.mode is not None:
            return
        if self._mags:
            m = sorted(self._mags)[len(self._mags) // 2]
            if 0.5 < m < 2.0:
                self.mode = "rig"
            elif 5.0 < m < 20.0:
                self.mode = "si"
            else:
                self.mode = "unknown"    # free fall? bad mapping? — surfaced
        else:
            self.mode = "unknown"


def _ang(v, mode):
    return math.degrees(v) if mode == "si" else v


def _acc(v, mode):
    return v / G0 if mode == "si" else v


# ---------------------------------------------------------------------------
# Fold — successive records into one evolving sample (imu_yb._Decoder's role).
# ---------------------------------------------------------------------------
class _Fold:
    KEYS = ("pitch", "roll", "yaw", "heading", "ax", "ay", "az",
            "gx", "gy", "gz", "mx", "my", "mz", "temp", "pressure_pa",
            "qw", "qx", "qy", "qz")

    def __init__(self):
        self.s = {k: None for k in self.KEYS}
        self.fresh = set()
        self._prev = {}
        self._orient_change = None

    def feed(self, rec, epoch, mode):
        """-> "orient" (attitude carried), "aux" (only other fields), or None
        (nothing in the record survived its plausibility gate — a reject)."""
        s = self.s
        orient = applied = None
        if "quat" in rec:
            q = rec["quat"]
            r, p, y = _quat_to_euler(q)      # degrees; unit-inference-free
            if all(map(math.isfinite, (r, p, y))):
                s["qw"], s["qx"], s["qy"], s["qz"] = q
                s["roll"], s["pitch"], s["yaw"] = r, p, y
                s["heading"] = (y + 360.0) % 360.0
                self.fresh.add("quat")
                orient = ("quat", (r, p, y))
        elif "euler" in rec:
            r, p, y = (_ang(v, mode) for v in rec["euler"])
            # The same post-conversion gate imu_yb applies: a wrong field
            # order or unit verdict must land in the reject count, never in a
            # flight_log attitude column.
            if all(map(math.isfinite, (r, p, y))) and abs(r) <= 180.5 \
                    and abs(p) <= 90.5 and abs(y) <= 360.5:
                s["roll"], s["pitch"], s["yaw"] = r, p, y
                s["heading"] = (y + 360.0) % 360.0
                self.fresh.add("euler")
                orient = ("euler", (r, p, y))
        if "accel" in rec:
            a = tuple(_acc(v, mode) for v in rec["accel"])
            if all(map(math.isfinite, a)) and all(abs(x) < 64.0 for x in a):
                s["ax"], s["ay"], s["az"] = a
                self.fresh.add("inertial")
                applied = True
        if "gyro" in rec:
            g = tuple(_ang(v, mode) for v in rec["gyro"])
            if all(map(math.isfinite, g)) and all(abs(x) < 4001.0 for x in g):
                s["gx"], s["gy"], s["gz"] = g
                self.fresh.add("inertial")
                applied = True
        if "mag" in rec:
            # Units unknown (uT vs raw counts) and unused downstream today;
            # published as-received, calibrated only by the fusion layer.
            s["mx"], s["my"], s["mz"] = rec["mag"]
            self.fresh.add("mag")
            applied = True
        if "temp" in rec:
            s["temp"] = rec["temp"]
            self.fresh.add("temp")
            applied = True
        if orient:
            kind, vals = orient
            if vals != self._prev.get(kind):
                # Same trap as the YB: a locked-up fusion keeps streaming
                # identical numbers at full rate, so "records arriving" is not
                # "IMU working". Track when the attitude last MOVED.
                self._prev[kind] = vals
                self._orient_change = epoch
            return "orient"
        return "aux" if applied else None

    def orient_change_epoch(self):
        return self._orient_change


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------
class SerialTransport:
    framed = False

    def __init__(self, device, baud=BAUD):
        self.device, self.baud = device, baud or BAUD
        self.spec = device
        self._ser = None

    def open(self):
        if serial is None:
            raise RuntimeError("pyserial not installed")
        # exclusive: see module docstring — keeps imu_yb's ttyACM dwell-probe
        # from splitting this stream with us.
        self._ser = serial.Serial(self.device, self.baud, timeout=0.2,
                                  exclusive=True)
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        return self

    def read_chunk(self, max_wait=0.25):
        self._ser.timeout = max_wait
        want = self._ser.in_waiting or 1
        chunk = self._ser.read(min(want, 65536))
        return (chunk or b""), time.time()

    def close(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass


class UdpTransport:
    framed = True

    def __init__(self, port=UDP_PORT_DEFAULT, host="0.0.0.0"):
        self.port, self.host = int(port), host
        self.spec = "udp:%d" % int(port)
        self._sock = None

    def open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.settimeout(0.25)
        self._sock = s
        return self

    def read_chunk(self, max_wait=0.25):
        self._sock.settimeout(max_wait)
        try:
            data, _addr = self._sock.recvfrom(65535)
            return data, time.time()
        except socket.timeout:
            return b"", time.time()

    def close(self):
        try:
            if self._sock:
                self._sock.close()
        except Exception:  # noqa: BLE001
            pass


class SimTransport:
    """Synthetic olixVision: NMEA-ish CSV lines in SI units at `rate_hz`, with
    a real XOR checksum and a slow attitude oscillation — exercises the line
    codec, the checksum verify and the SI unit inference, i.e. the exact path
    a real ASCII-mode unit would take."""

    framed = False

    def __init__(self, rate_hz=50.0):
        self.rate = max(1.0, float(rate_hz or 50.0))
        self.spec = "sim"
        self._t0 = self._next = None

    def open(self):
        self._t0 = self._next = time.time()
        return self

    def _line(self, t):
        ph = 2 * math.pi * 0.1 * (t - self._t0)
        roll, pitch = 0.05 * math.sin(ph), 0.03 * math.cos(ph)
        yaw = (0.5 + 0.02 * (t - self._t0)) % (2 * math.pi)
        body = ("OLIX,%.6f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,"
                "%.5f,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f") % (
            t, roll, pitch, yaw,
            0.3 * math.sin(ph), 0.2 * math.cos(ph),
            G0 * math.cos(roll) * math.cos(pitch),
            0.01 * math.cos(ph), -0.01 * math.sin(ph), 0.002,
            21.0, 5.0, -43.0, 24.5)
        ck = 0
        for ch in body:
            ck ^= ord(ch)
        return ("$%s*%02X\r\n" % (body, ck)).encode()

    def read_chunk(self, max_wait=0.25):
        now = time.time()
        if self._next > now:
            time.sleep(min(max_wait, self._next - now))
            now = time.time()
        out = []
        while self._next <= now and len(out) < 512:
            out.append(self._line(self._next))
            self._next += 1.0 / self.rate
        return b"".join(out), now

    def close(self):
        pass


def _parse_spec(spec):
    """spec -> (kind, arg); (None, None) means autodetect."""
    if spec is None or str(spec).strip() in ("", "auto"):
        return None, None
    s = str(spec).strip()
    low = s.lower()
    if low == "sim" or low.startswith("sim:"):
        return "sim", (float(s.split(":", 1)[1]) if ":" in s else 50.0)
    if low == "udp":
        return "udp", UDP_PORT_DEFAULT
    if low.startswith("udp:"):
        return "udp", int(s.split(":", 1)[1])
    if s.isdigit():
        return "udp", int(s)
    return "serial", s


def _make_transport(kind, arg, baud=BAUD):
    if kind == "serial":
        return SerialTransport(arg, baud)
    if kind == "udp":
        return UdpTransport(arg)
    if kind == "sim":
        return SimTransport(arg)
    raise ValueError("unknown transport kind %r" % (kind,))


# ---------------------------------------------------------------------------
# Reader
# ---------------------------------------------------------------------------
class ImuReader:
    def __init__(self, port=None, baud=BAUD, **_):
        self.port = port                    # a spec string; see _parse_spec
        self.baud = baud or BAUD
        self._tr = None
        self._codec = _Codec()
        self._fold = _Fold()
        self._gate = _UnitGate()
        self._pend = []                     # records held until units decide
        self._latest = None
        self._ring = []                     # (epoch, sample)
        self._lock = threading.Lock()
        self._run = False
        self._thread = None
        self._rate = 0.0
        self._frame_rate = 0.0
        self._rate_measured = False         # "0.0 measured" != "not yet"
        self._rejected = 0
        self._errors = 0
        self._last_error = None
        self._last_ring_epoch = None
        self._ctr_frames = 0
        self._ctr_pubs = 0
        self._ctr_t0 = time.time()

    def open(self):
        kind, arg = _parse_spec(self.port)
        if kind is None:
            info = probe()
            if not info.get("present"):
                raise RuntimeError(
                    "no olixVision stream found (ttyACM scan + UDP :%d)"
                    % UDP_PORT_DEFAULT)
            kind, arg = _parse_spec(info["port"])
        self._tr = _make_transport(kind, arg, self.baud)
        self._tr.open()
        self.port = self._tr.spec
        return self

    # -- ingest (shared by read() and the sampler thread; never raises) ----
    def _ingest(self, chunk, t_read):
        try:
            self._ingest_inner(chunk, t_read)
        except Exception as e:  # noqa: BLE001 — this process owns the fire
            self._errors += 1   # path too; a decode bug must cost a counter,
            self._last_error = "%s: %s" % (type(e).__name__, e)  # not the node

    def _ingest_inner(self, chunk, t_read):
        recs = self._codec.feed(chunk, framed=self._tr.framed)
        for rec in recs:
            if self._gate.mode is None:
                self._gate.offer(rec, t_read)
            self._pend.append((rec, t_read))
            if len(self._pend) > 512:
                # The unit gate never resolved (no accel in the stream and the
                # timeout not yet hit at this cadence): shed oldest, counted.
                self._pend.pop(0)
                self._rejected += 1
        if self._gate.mode is not None and self._pend:
            for rec, ep in self._pend:
                self._apply(rec, ep)
            self._pend = []
        self._rate_tick(time.time())

    def _idle_flush(self, now):
        """A burst followed by silence must not leave samples parked behind an
        undecided unit gate forever."""
        if self._gate.mode is None and self._pend \
                and now - self._pend[0][1] > _UnitGate.TIMEOUT_S:
            self._gate.force()
            for rec, ep in self._pend:
                self._apply(rec, ep)
            self._pend = []
        self._rate_tick(now)

    def _apply(self, rec, epoch):
        verdict = self._fold.feed(rec, epoch, self._gate.mode or "unknown")
        if verdict is None:
            self._rejected += 1              # decoded but implausible
            return
        self._ctr_frames += 1
        self._publish(epoch, ring=(verdict == "orient"),
                      device_ts=rec.get("device_ts"))

    def _publish(self, epoch, ring, device_ts=None):
        s = dict(self._fold.s)
        s["epoch"] = epoch                   # ARRIVAL time; see module doc
        s["fresh"] = sorted(self._fold.fresh)
        self._fold.fresh.clear()
        s["src"] = self._tr.spec if self._tr else None
        s["unit_mode"] = self._gate.mode
        if device_ts is not None:
            s["device_ts"] = device_ts
        with self._lock:
            self._latest = s
            if not ring:
                return                       # aux refreshes latest() only —
            self._last_ring_epoch = epoch    # a ring entry asserts "attitude
            self._ring.append((epoch, s))    # measured at this epoch"
            cut = epoch - RING_S
            while self._ring and self._ring[0][0] < cut:
                self._ring.pop(0)
        self._ctr_pubs += 1

    def _rate_tick(self, now):
        if now - self._ctr_t0 >= 1.0:
            self._rate = self._ctr_pubs / (now - self._ctr_t0)
            self._frame_rate = self._ctr_frames / (now - self._ctr_t0)
            self._rate_measured = True
            self._ctr_frames, self._ctr_pubs, self._ctr_t0 = 0, 0, now

    # -- lifecycle ----------------------------------------------------------
    def read(self):
        """Block briefly for the next attitude sample; dict or None. Never
        raises past this frame (transport errors become counters)."""
        if self._tr is None:
            return None
        t0, seen = time.time(), self._last_ring_epoch
        while time.time() - t0 < 3.0:
            if self._thread:                 # sampler owns the transport
                if self._last_ring_epoch not in (None, seen):
                    return self.latest()
                time.sleep(0.05)
                continue
            try:
                chunk, t_read = self._tr.read_chunk(0.2)
            except Exception as e:  # noqa: BLE001
                self._errors += 1
                self._last_error = str(e)
                return None
            if chunk:
                self._ingest(chunk, t_read)
            else:
                self._idle_flush(time.time())
            if self._last_ring_epoch not in (None, seen):
                return self.latest()
        return None

    def _loop(self):
        while self._run:
            try:
                chunk, t_read = self._tr.read_chunk(0.25)
            except Exception as e:  # noqa: BLE001 — unplug/EIO: keep trying;
                self._errors += 1   # piagent's freshness check drops us and
                self._last_error = str(e)      # re-probes if this never heals
                time.sleep(0.05)
                continue
            if chunk:
                self._ingest(chunk, t_read)
            else:
                self._idle_flush(time.time())

    def start(self):
        if self._thread:
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._run = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def close(self):
        self.stop()
        if self._tr:
            self._tr.close()

    # -- the duck-typed query surface piagent's Imu slot getattr()s ---------
    def latest(self):
        with self._lock:
            return dict(self._latest) if self._latest else None

    def window(self, t0, t1):
        with self._lock:
            return [dict(s) for (e, s) in self._ring if t0 <= e <= t1]

    def rate_hz(self):
        """Ring publishes (real attitude updates) per second — NOT records."""
        return round(self._rate, 1)

    def frame_rate_hz(self):
        """All decoded records per second — link health, not sample cadence."""
        return round(self._frame_rate, 1)

    def rate_measured(self):
        return self._rate_measured

    def rejected_frames(self):
        """Records refused: checksum mismatch, unframeable bytes, implausible
        values after unit conversion. The bring-up signal for a wrong field-
        order or unit guess."""
        return self._rejected + self._codec.rejects

    def last_attitude_epoch(self):
        with self._lock:
            return self._last_ring_epoch

    def checksum_state(self):
        """Codec + unit-gate state, keys a superset of imu_yb's so /health
        embeds either dict unchanged. raw_tail_hex is the bring-up payload:
        the last bytes of a stream the codec could not decode."""
        st = self._codec.state()
        st["unit_mode"] = self._gate.mode
        st["errors"] = self._errors
        if self._last_error:
            st["last_error"] = self._last_error
        return st

    def orientation_frozen_s(self):
        t = self._fold.orient_change_epoch()
        return None if t is None else round(time.time() - t, 3)


# ---------------------------------------------------------------------------
# Probe
# ---------------------------------------------------------------------------
def _candidates(spec):
    kind, arg = _parse_spec(spec)
    if kind is not None:
        return [(kind, arg)]
    out = []
    for pat in SERIAL_PORTS:
        out.extend(("serial", p) for p in sorted(glob.glob(pat)))
    out.append(("udp", UDP_PORT_DEFAULT))
    return out


def _stream_stats(tr, dwell=0.7):
    """Open `tr`, listen for `dwell` s, report what ACTUALLY arrived — bytes,
    decoded records, orientation records, codec — never a datasheet claim."""
    try:
        tr.open()
    except Exception as e:  # noqa: BLE001 — busy (exclusive holder), absent
        return {"error": str(e)}
    codec = _Codec()
    orient = recs = 0
    t0 = time.time()
    try:
        while time.time() - t0 < dwell:
            chunk, _t = tr.read_chunk(0.2)
            for rec in codec.feed(chunk, framed=tr.framed):
                recs += 1
                if "euler" in rec or "quat" in rec:
                    orient += 1
    except Exception as e:  # noqa: BLE001
        return {"error": str(e)}
    finally:
        tr.close()
    return {"bytes": codec.bytes_seen, "records": recs, "orient": orient,
            "elapsed_s": max(1e-3, time.time() - t0),
            "codec": codec.locked, "tail_hex": codec.tail.hex(),
            "rejects": codec.rejects}


def probe(spec=None, verbose=False):
    """Detect the olixVision. Order: explicit spec > /dev/ttyACM* > UDP.

    present:true needs >= 2 DECODED orientation records — bytes alone are not
    an IMU. When a candidate produced bytes the codec could not decode, the
    note says so and raw_tail_hex carries the evidence: that is the designed
    bring-up outcome for a wire format this driver has not met yet."""
    out = {"present": False, "notes": []}
    best_tail = ""
    for kind, arg in _candidates(spec):
        tr = _make_transport(kind, arg)
        st = _stream_stats(tr)
        if "error" in st:
            out["notes"].append("%s: %s" % (tr.spec, st["error"]))
            continue
        if st["orient"] >= 2:
            out.update(
                present=True, chip=CHIP, port=tr.spec, baud=BAUD,
                transport=kind, codec=st["codec"],
                # Measured over the dwell, not quoted from a datasheet.
                sample_rate_hz=round(st["orient"] / st["elapsed_s"], 1),
                frame_rate_hz=round(st["records"] / st["elapsed_s"], 1))
            out["notes"].append("%s: %s records decoded (%s)"
                                % (tr.spec, st["records"], st["codec"]))
            if verbose:
                print(out)
            return out
        if st["bytes"]:
            out["notes"].append(
                "%s: %d bytes seen, %d records decoded, %d orientation — "
                "undecoded tail follows" % (tr.spec, st["bytes"],
                                            st["records"], st["orient"]))
            if len(st["tail_hex"]) > len(best_tail):
                best_tail = st["tail_hex"]
        else:
            out["notes"].append("%s: no data" % tr.spec)
    if best_tail:
        out["raw_tail_hex"] = best_tail
    if verbose:
        print(out)
    return out


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    info = probe(arg, verbose=False)
    print("probe:", json.dumps(info, indent=2))
    if not info.get("present"):
        raise SystemExit(1)
    r = ImuReader(port=info["port"], baud=info.get("baud") or BAUD).open()
    r.start()
    time.sleep(0.7)
    for _ in range(20):
        s = r.latest()
        if s:
            print("roll %7s pitch %7s yaw %7s | a %6s %6s %6s | t %5s | "
                  "%sHz att / %sHz rec | units %s | age %sms" % (
                      _fmt(s["roll"]), _fmt(s["pitch"]), _fmt(s["yaw"]),
                      _fmt(s["ax"], 2), _fmt(s["ay"], 2), _fmt(s["az"], 2),
                      _fmt(s["temp"], 1), r.rate_hz(), r.frame_rate_hz(),
                      s.get("unit_mode"),
                      _fmt((time.time() - s["epoch"]) * 1000)))
        time.sleep(0.2)
    print("state:", json.dumps(r.checksum_state(), indent=2))
    r.close()

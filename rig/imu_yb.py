#!/usr/bin/env python3
"""imu_yb — driver for the Yahboom YB-MRA02 10-axis IMU (USB-serial, CH340).

Reverse-engineered from the live device on 2026-08-16. The module streams a
0x7E-framed binary protocol at 115200 baud (NOT the WitMotion 0x55/921600
protocol an earlier draft assumed). Each frame is:

    0x7E 0x23 <type> <sub> <payload...> <checksum>

with length fixed per (type, sub). The four payload-bearing frames seen:

    (0x15,0x16)  16 B  quaternion        4 x float32 LE   (|q| == 1, self-checking)
    (0x11,0x26)  12 B  euler angles      3 x float32 LE   roll,pitch,yaw (radians)
    (0x17,0x04)  18 B  raw inertial      9 x int16  LE    ax ay az gx gy gz mx my mz
    (0x15,0x32)  16 B  baro / temp       4 x float32 LE   [_, tempC, pressPa, pressPa]

0x7E can occur inside a payload, so framing is length-based (sync on 0x7E 0x23,
then read the known payload length), never delimiter-split.

Public interface (unchanged, so piagent needs no changes):
    ImuReader(port=..., baud=...).open()/read()/start()/stop()/latest()/window()
    probe() -> dict describing what was detected

    python3 imu_yb.py            # probe then stream 20 samples
"""

import glob
import math
import struct
import threading
import time

try:
    import serial
except Exception:  # noqa: BLE001
    serial = None

BAUD = 115200
SOF, ADDR = 0x7E, 0x23
SERIAL_PORTS = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*")

# (type, sub) -> (payload_len_bytes, kind)
FRAMES = {
    (0x15, 0x16): (16, "quat"),
    (0x11, 0x26): (12, "euler"),
    (0x17, 0x04): (18, "inertial"),
    (0x15, 0x32): (16, "baro"),
}
_MAXLEN = max(n for n, _ in FRAMES.values())

# Full-scale ranges, confirmed empirically on the device: accel reads ~1 g total
# at rest with +/-16 g, and the gyro sits near zero at rest with +/-2000 dps.
ACC_G = 16.0 / 32768.0
GYR_DPS = 2000.0 / 32768.0


def _fmt(v, n=1):
    return "--" if v is None else ("%.*f" % (n, v))


def _f32(b, o):
    return struct.unpack_from("<f", b, o)[0]


def _quat_to_euler(q):
    """(qw,qx,qy,qz) -> (roll,pitch,yaw) degrees. Order picked to agree with the
    module's own euler frame."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class _Decoder:
    """Fold successive frames into one evolving sample dict."""

    def __init__(self):
        self.s = {k: None for k in (
            "pitch", "roll", "yaw", "heading", "ax", "ay", "az",
            "gx", "gy", "gz", "mx", "my", "mz", "temp", "pressure_pa",
            "qw", "qx", "qy", "qz")}

    def feed(self, kind, body):
        s = self.s
        if kind == "quat":
            q = (_f32(body, 0), _f32(body, 4), _f32(body, 8), _f32(body, 12))
            n = math.sqrt(sum(c * c for c in q))
            if not (0.9 < n < 1.1):
                return False                       # not a real quaternion frame
            s["qw"], s["qx"], s["qy"], s["qz"] = q
        elif kind == "euler":
            roll, pitch, yaw = (_f32(body, 0), _f32(body, 4), _f32(body, 8))
            s["roll"] = math.degrees(roll)
            s["pitch"] = math.degrees(pitch)
            s["yaw"] = math.degrees(yaw)
            s["heading"] = (s["yaw"] + 360.0) % 360.0
        elif kind == "inertial":
            v = struct.unpack_from("<9h", body, 0)
            s["ax"], s["ay"], s["az"] = (v[0] * ACC_G, v[1] * ACC_G, v[2] * ACC_G)
            s["gx"], s["gy"], s["gz"] = (v[3] * GYR_DPS, v[4] * GYR_DPS, v[5] * GYR_DPS)
            s["mx"], s["my"], s["mz"] = v[6], v[7], v[8]
        elif kind == "baro":
            s["temp"] = _f32(body, 4)
            s["pressure_pa"] = _f32(body, 8)
        return True

    def sample(self):
        s = dict(self.s)
        # If the module didn't send an euler frame, derive it from the quaternion.
        if s["roll"] is None and s["qw"] is not None:
            r, p, y = _quat_to_euler((s["qw"], s["qx"], s["qy"], s["qz"]))
            s["roll"], s["pitch"], s["yaw"] = r, p, y
            s["heading"] = (y + 360.0) % 360.0
        s["epoch"] = time.time()
        return s


def _iter_frames(buf):
    """Yield (kind, payload) from a byte buffer; return leftover bytes.

    Length-based framing so a 0x7E inside a payload never mis-frames."""
    i, n = 0, len(buf)
    while i < n:
        if buf[i] != SOF:
            i += 1
            continue
        if i + 4 > n:
            break                                   # need the header
        if buf[i + 1] != ADDR:
            i += 1
            continue
        key = (buf[i + 2], buf[i + 3])
        spec = FRAMES.get(key)
        if spec is None:
            i += 1                                   # unknown header, resync
            continue
        plen, kind = spec
        end = i + 4 + plen + 1                        # header + payload + checksum
        if end > n:
            break                                    # wait for the rest
        yield kind, buf[i + 4:i + 4 + plen]
        i = end
    return buf[i:]


class ImuReader:
    def __init__(self, port=None, baud=BAUD, **_):
        self.port = port
        self.baud = baud or BAUD
        self._ser = None
        self._dec = _Decoder()
        self._latest = None
        self._ring = []                              # (epoch, sample)
        self._lock = threading.Lock()
        self._run = False
        self._thread = None
        self._rate = 0.0

    def open(self):
        if serial is None:
            raise RuntimeError("pyserial not installed")
        if not self.port:
            self.port = _first_streaming_port()
            if not self.port:
                raise RuntimeError("no YB-MRA02 serial stream found")
        self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        return self

    def read(self):
        """Block briefly for the next complete sample; return the dict or None."""
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 1.0:
            chunk = self._ser.read(256)
            if chunk:
                buf.extend(chunk)
                got = False
                for kind, body in _iter_frames(bytes(buf)):
                    got = self._dec.feed(kind, body) or got
                buf = _tail_after_frames(bytes(buf))
                if got:
                    return self._dec.sample()
        return None

    # Frames that complete an orientation update. One ring sample is published
    # per such frame, so the ring runs at the device's real rate rather than at
    # whatever rate the OS happens to hand us serial chunks.
    _ORIENTATION = ("euler", "quat")

    def _loop(self):
        buf = bytearray()
        cnt, t0 = 0, time.time()
        while self._run:
            try:
                # Take whatever is buffered, but block for at least one byte, so
                # a chunk is returned the moment data exists. Reading a fixed 256
                # bytes instead waits for the buffer to fill (or for the 0.2 s
                # timeout), which throttled the ring to ~8 Hz off a 200 Hz device.
                want = self._ser.in_waiting or 1
                chunk = self._ser.read(min(want, 4096))
            except Exception:  # noqa: BLE001
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            published = False
            for kind, body in _iter_frames(bytes(buf)):
                if not self._dec.feed(kind, body):
                    continue
                cnt += 1
                if kind in self._ORIENTATION:
                    self._publish()
                    published = True
            buf = _tail_after_frames(bytes(buf))
            if len(buf) > 4096:
                buf = buf[-256:]
            if not published:
                # Inertial/baro-only chunk: still refresh `latest` so a caller
                # polling between orientation frames sees current accel data.
                self._publish()
            now = time.time()
            if now - t0 >= 1.0:
                self._rate = cnt / (now - t0)
                cnt, t0 = 0, now

    def _publish(self):
        samp = self._dec.sample()
        with self._lock:
            self._latest = samp
            self._ring.append((samp["epoch"], samp))
            cut = samp["epoch"] - 65
            while self._ring and self._ring[0][0] < cut:
                self._ring.pop(0)

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
        if self._ser:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass

    def latest(self):
        with self._lock:
            return dict(self._latest) if self._latest else None

    def window(self, t0, t1):
        with self._lock:
            return [dict(s) for (e, s) in self._ring if t0 <= e <= t1]

    def rate_hz(self):
        return round(self._rate, 1)


def _tail_after_frames(buf):
    """Return the bytes left after consuming every complete frame in buf."""
    i, n = 0, len(buf)
    while i < n:
        if buf[i] != SOF:
            i += 1
            continue
        if i + 4 > n:
            break
        if buf[i + 1] != ADDR:
            i += 1
            continue
        spec = FRAMES.get((buf[i + 2], buf[i + 3]))
        if spec is None:
            i += 1
            continue
        end = i + 4 + spec[0] + 1
        if end > n:
            break
        i = end
    return bytearray(buf[i:])


def _first_streaming_port():
    ports = []
    for pat in SERIAL_PORTS:
        ports.extend(sorted(glob.glob(pat)))
    for p in ports:
        if _stream_ok(p):
            return p
    return None


def _stream_ok(port):
    """True if the port yields at least a couple of valid YB frames at 115200."""
    if serial is None:
        return False
    try:
        s = serial.Serial(port, BAUD, timeout=0.3)
    except Exception:  # noqa: BLE001
        return False
    try:
        time.sleep(0.2)
        s.reset_input_buffer()
        time.sleep(0.4)
        data = s.read(1024)
    except Exception:  # noqa: BLE001
        return False
    finally:
        s.close()
    return sum(1 for _ in _iter_frames(bytes(data))) >= 2


def probe(verbose=False):
    out = {"present": False, "notes": []}
    ports = []
    for pat in SERIAL_PORTS:
        ports.extend(sorted(glob.glob(pat)))
    if not ports:
        out["notes"].append("no /dev/ttyUSB*|ttyACM* — is the IMU on a USB-A "
                             "(data) port? Pi 5 USB-C is power-only")
        return out
    for p in ports:
        ok = _stream_ok(p)
        out["notes"].append("%s: %s" % (p, "YB 0x7E frames present" if ok
                                        else "no valid frames at 115200"))
        if ok:
            out.update(present=True, chip="yahboom_yb-mra02 (0x7E protocol)",
                       port=p, baud=BAUD, sample_rate_hz=200)
            if verbose:
                print(out)
            return out
    return out


if __name__ == "__main__":
    info = probe(verbose=True)
    print("probe:", info)
    if not info.get("present"):
        raise SystemExit(1)
    r = ImuReader(port=info["port"]).open()
    r.start()
    time.sleep(0.5)
    for _ in range(20):
        s = r.latest()
        if s:
            print("roll %7s pitch %7s yaw %7s | a %6s %6s %6s | "
                  "t %5s | %sHz" % (
                      _fmt(s["roll"]), _fmt(s["pitch"]), _fmt(s["yaw"]),
                      _fmt(s["ax"], 2), _fmt(s["ay"], 2), _fmt(s["az"], 2),
                      _fmt(s["temp"], 1), r.rate_hz()))
        time.sleep(0.2)
    r.close()


def _fmt(v, n=1):
    return "--" if v is None else ("%.*f" % (n, v))

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

Timestamping (this is the whole point of the module for photogrammetry):
a sample's `epoch` is when the device SENT it, not when Python got round to
parsing it. Bytes leave the module at a fixed 10 bits / 115200 baud = 86.8 us
each, so a frame whose last byte sits N bytes before the end of the chunk we
just read left the device N byte-times before that chunk's last byte. Dating
every frame that way costs nothing and removes the parse-order error that made
"nearest sample to the capture instant" mean "nearest by arrival".

That ruler is only honest WITHIN one burst. The device sends ~2 kB/s on an
11.5 kB/s wire, so the line is idle ~83% of the time between device cycles: as
soon as one read returns more than one cycle (USB batching, a busy Pi), the
idle gaps between the cycles are invisible to the byte-count arithmetic and
every frame but the newest comes out several-to-tens of ms LATE - straight
into the attitude bound to a photo. So byte-time dating is now a ceiling, not
the answer: _date_by_cadence pulls each earlier same-kind frame of a multi-
cycle chunk back by ~one learned device period (see its docstring for the
residual, which is bounded by a fraction of one cycle rather than by the whole
idle gap). Left over on top of that: the USB/CH340 batching latency (~1-2 ms,
common to every frame in a chunk, a bias rather than jitter), and the
sensor->wire fusion lag, which is unknown and is NOT modelled here - do not
pretend either is zero anywhere downstream.

Every frame's trailing checksum byte is verified (_ChecksumGate learns which
of the common schemes this unit uses from its own stream), and euler/baro
payloads are range- and finiteness-checked, because a single dropped byte on
the CH340 link re-frames the stream and used to publish arbitrary float32
garbage - NaN included - as a fresh, authoritative attitude.

A sample is published to the ring ONLY when an orientation frame actually
arrived. Republishing the same attitude under a fresh epoch for every serial
chunk (which is what this used to do) manufactures a ring that looks like it
runs at ~190 Hz while ~84% of its entries are copies, so a consumer picking the
"nearest" sample picks a copy and believes its attitude is 6 ms old when it is
really up to 40 ms old - four times this rig's entire inter-camera sync budget.

Public interface (additive only - piagent getattr()s the newer members so a
node mid-rolling-deploy still answers /health):
    ImuReader(port=..., baud=...).open()/read()/start()/stop()/latest()/window()
    ImuReader.rate_hz()/frame_rate_hz()/orientation_frozen_s()
    ImuReader.rate_measured()      -> has a rate window closed? (0.0 measured
                                      is NOT the same as "not measured yet")
    ImuReader.last_attitude_epoch()-> last RING publish; latest() is refreshed
                                      by inertial frames and cannot answer
                                      "is the ATTITUDE still flowing?"
    ImuReader.rejected_frames()    -> frames refused (checksum / range / NaN)
    ImuReader.checksum_state()     -> which scheme is enforced, or dormant
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

# 8N1: one start bit + 8 data + one stop = 10 bits on the wire per byte. This is
# the ruler every sample epoch is measured with (see the module docstring).
BYTE_TIME_S = 10.0 / BAUD

# Frames that carry an attitude. Only these produce a ring entry: everything
# else refreshes accel/baro inside the SAME attitude and would be a duplicate.
ORIENTATION = ("euler", "quat")

RING_S = 65              # ring depth; PROTOCOL.md requires at least 60 s

# Smallest gap the cadence re-dating (see _date_by_cadence) will leave between
# two same-kind frames when its monotonicity floor bites. Well under one byte
# time, so it can never be mistaken for a measured interval - it exists only to
# keep the ring strictly ordered in a pathological batched read.
CADENCE_FLOOR_EPS_S = 0.0001

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
    """Fold successive frames into one evolving sample dict.

    Each frame is fed with the epoch it left the device on the wire, so a
    published sample can be dated by the frame that actually produced it and can
    say which of its fields are new rather than carried over."""

    def __init__(self):
        self.s = {k: None for k in (
            "pitch", "roll", "yaw", "heading", "ax", "ay", "az",
            "gx", "gy", "gz", "mx", "my", "mz", "temp", "pressure_pa",
            "qw", "qx", "qy", "qz")}
        self._fresh = set()        # frame kinds decoded since the last sample()
        self._prev = {}            # kind -> its last decoded value tuple
        self._orient_change = None # epoch the attitude numbers last MOVED

    def feed(self, kind, body, epoch=None):
        s = self.s
        vals = None
        if kind == "quat":
            q = (_f32(body, 0), _f32(body, 4), _f32(body, 8), _f32(body, 12))
            n = math.sqrt(sum(c * c for c in q))
            if not (0.9 < n < 1.1):
                return False                       # not a real quaternion frame
            s["qw"], s["qx"], s["qy"], s["qz"] = q
            vals = q
        elif kind == "euler":
            roll, pitch, yaw = (_f32(body, 0), _f32(body, 4), _f32(body, 8))
            r_d, p_d, y_d = (math.degrees(roll), math.degrees(pitch),
                             math.degrees(yaw))
            # Range/finiteness gate. A byte-shifted frame decodes to arbitrary
            # float32s - NaN and 1e38 included - and the quaternion norm check
            # guarded quat only, so a corrupt euler frame used to publish
            # straight into the ring (and json.dumps happily emits NaN into
            # flight_log). The device speaks radians: roll/yaw in +/-pi,
            # pitch +/-pi/2; a hair of slack for fusion overshoot.
            if not all(map(math.isfinite, (r_d, p_d, y_d))) \
                    or abs(r_d) > 180.5 or abs(p_d) > 90.5 or abs(y_d) > 360.5:
                return False                       # not a real euler frame
            s["roll"], s["pitch"], s["yaw"] = r_d, p_d, y_d
            s["heading"] = (s["yaw"] + 360.0) % 360.0
            vals = (s["roll"], s["pitch"], s["yaw"])
        elif kind == "inertial":
            v = struct.unpack_from("<9h", body, 0)
            s["ax"], s["ay"], s["az"] = (v[0] * ACC_G, v[1] * ACC_G, v[2] * ACC_G)
            s["gx"], s["gy"], s["gz"] = (v[3] * GYR_DPS, v[4] * GYR_DPS, v[5] * GYR_DPS)
            s["mx"], s["my"], s["mz"] = v[6], v[7], v[8]
        elif kind == "baro":
            t, p = _f32(body, 4), _f32(body, 8)
            # Finiteness only (the housing's pressure range is not worth
            # guessing at): baro fields are carried into every ring sample, so
            # one NaN here would poison every later attitude publish.
            if not (math.isfinite(t) and math.isfinite(p)):
                return False
            s["temp"] = t
            s["pressure_pa"] = p
        self._fresh.add(kind)
        if vals is not None and vals != self._prev.get(kind):
            # A fusion that has locked up keeps transmitting - identical numbers
            # at full rate - so "bytes are arriving" is NOT evidence the IMU
            # still works. Remember when the attitude last actually moved and
            # let health() report it, or a frozen unit reads perfectly healthy.
            self._prev[kind] = vals
            self._orient_change = epoch if epoch is not None else time.time()
        return True

    def orient_change_epoch(self):
        return self._orient_change

    def sample(self, epoch=None):
        s = dict(self.s)
        # If the module didn't send an euler frame, derive it from the quaternion.
        if s["roll"] is None and s["qw"] is not None:
            r, p, y = _quat_to_euler((s["qw"], s["qx"], s["qy"], s["qz"]))
            s["roll"], s["pitch"], s["yaw"] = r, p, y
            s["heading"] = (y + 360.0) % 360.0
        # Wire time of the frame that produced this sample - NOT time.time().
        # Stamping at parse time makes the epoch a measure of how busy the host
        # was, and the Jetson then binds an attitude to a photo on that basis.
        s["epoch"] = time.time() if epoch is None else epoch
        # Which sub-frames carry new numbers in this sample; everything else is
        # carried over from an earlier frame. A consumer binding attitude to an
        # image can tell a measurement from a copy.
        s["fresh"] = sorted(self._fresh)
        self._fresh.clear()
        return s


def _scan(buf):
    """Split `buf` into complete frames plus where the leftover tail starts.

    Returns ([(kind, payload, end, frame), ...], tail_index). `end` is the
    offset one past the frame's last byte, and it is what dates the frame: the
    caller counts back (len(buf) - end) byte-times from the read instant.
    `frame` is the whole frame including SOF and the trailing checksum byte, so
    the caller can verify the checksum (framing by length alone accepted any
    bit-shifted garbage that happened to start 0x7E 0x23). One scan does both
    jobs - the old pair of near-identical scanners could not agree on where a
    frame ended without walking the buffer twice.

    Length-based framing so a 0x7E inside a payload never mis-frames."""
    frames = []
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
        frames.append((kind, bytes(buf[i + 4:i + 4 + plen]), end,
                       bytes(buf[i:end])))
        i = end
    return frames, i


class _ChecksumGate:
    """Learns which checksum scheme the trailing byte follows, then enforces it.

    The checksum byte used to be carried and never compared, so a dropped byte
    on the CH340 link (or the forced resync) let a shifted frame publish
    arbitrary float garbage as a fresh attitude. The exact algorithm was never
    reverse-engineered, so hard-coding one risks rejecting EVERY frame from a
    live device that uses another - fatal in the field. Instead: candidate
    schemes (the common YB/HiPNUC additive/xor sums) audition against the
    first LEARN_N frames; whichever still matches all of them (first by name,
    deterministically, if more than one somehow survives 64 frames) is
    enforced from then on, and a mismatch is a corrupt frame. If no candidate
    survives (unknown algorithm, or a corrupt frame during the audition), the
    gate goes dormant and accepts everything - the decoder's range/finite
    checks remain the guard, and `state()` says so, which is why /health
    reports it rather than letting the operator assume verification is on."""

    LEARN_N = 64

    @staticmethod
    def _xor_after_sof(fb):
        x = 0
        for b in fb[1:-1]:
            x ^= b
        return x

    CANDIDATES = {
        "sum_after_sof": lambda fb: sum(fb[1:-1]) & 0xFF,
        "sum_all": lambda fb: sum(fb[:-1]) & 0xFF,
        "xor_after_sof": _xor_after_sof.__func__,
    }

    def __init__(self):
        self._alive = set(self.CANDIDATES)
        self._seen = 0
        self.algo = None                 # enforced algorithm name, or None
        self.dormant = False             # no candidate matched: verify nothing
        self.rejects = 0

    def check(self, frame):
        """True to accept `frame` (full bytes incl. trailing checksum)."""
        if self.algo is not None:
            ok = self.CANDIDATES[self.algo](frame) == frame[-1]
            if not ok:
                self.rejects += 1
            return ok
        if self.dormant:
            return True
        ck = frame[-1]
        self._alive = {n for n in self._alive
                       if self.CANDIDATES[n](frame) == ck}
        self._seen += 1
        if not self._alive:
            self.dormant = True
        elif self._seen >= self.LEARN_N:
            self.algo = sorted(self._alive)[0]
        return True                      # never reject while still learning

    def state(self):
        return {"algo": self.algo, "dormant": self.dormant,
                "rejects": self.rejects,
                "learning": self.algo is None and not self.dormant}


class ImuReader:
    def __init__(self, port=None, baud=BAUD, **_):
        self.port = port
        self.baud = baud or BAUD
        self._ser = None
        self._dec = _Decoder()
        self._gate = _ChecksumGate()
        self._latest = None
        self._ring = []                              # (epoch, sample)
        self._lock = threading.Lock()
        self._run = False
        self._thread = None
        self._rate = 0.0                             # ring publishes / s
        self._frame_rate = 0.0                       # decoded frames / s
        # "measured 0" and "not yet measured" must be distinguishable: the
        # health fallback to the probe figure keyed on a falsy rate, so a
        # stalled attitude read as the probe's healthy 49.8 Hz forever.
        self._rate_measured = False
        self._rejected = 0            # frames refused (checksum / range / NaN)
        self._last_ring_epoch = None  # epoch of the last ATTITUDE publish
        # _ingest state (was _loop locals; _ingest is per-chunk so tests can
        # drive it with synthetic bytes and synthetic read instants).
        self._buf = bytearray()
        self._ctr_frames = 0
        self._ctr_pubs = 0
        self._ctr_t0 = time.time()
        # Per-kind cadence model for dating multi-cycle chunks (see _ingest):
        # kind -> (last epoch, chunk#) and an EMA of the device's own period.
        self._kind_prev = {}
        self._kind_ema = {}
        self._chunk_no = 0

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
            t_read = time.time()
            if chunk:
                buf.extend(chunk)
                frames, tail = _scan(buf)
                blen = len(buf)
                got = None
                for kind, body, end, frame in frames:
                    if not self._gate.check(frame):
                        continue
                    t_frame = t_read - (blen - end) * BYTE_TIME_S
                    if self._dec.feed(kind, body, t_frame):
                        got = t_frame
                del buf[:tail]
                if got is not None:
                    return self._dec.sample(got)
        return None

    def _loop(self):
        while self._run:
            try:
                # Take whatever is buffered, but block for at least one byte, so
                # a chunk is returned the moment data exists. Reading a fixed 256
                # bytes instead waits for the buffer to fill (or for the 0.2 s
                # timeout), which throttled the ring to ~8 Hz off the device.
                # Drain the WHOLE backlog in one read: the epochs below are
                # counted backwards from the instant of the chunk's last byte, so
                # splitting a backlog across two reads would date its older half
                # as though it had just arrived.
                want = self._ser.in_waiting or 1
                chunk = self._ser.read(min(want, 65536))
                t_read = time.time()     # instant the last byte of the chunk landed
            except Exception:  # noqa: BLE001
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            self._ingest(chunk, t_read)

    def _ingest(self, chunk, t_read):
        """Scan, verify, date and publish one serial chunk read at `t_read`."""
        self._chunk_no += 1
        buf = self._buf
        buf.extend(chunk)
        frames, tail = _scan(buf)
        blen = len(buf)
        dated = []
        for kind, body, end, frame in frames:
            if not self._gate.check(frame):
                self._rejected += 1              # corrupt on the wire
                continue
            # Wire time: bytes leave the module at BYTE_TIME_S each, so a
            # frame ending (blen - end) bytes before the end of what we just
            # read left the device that many byte-times earlier. Any leftover
            # from the previous read holds no complete frame by construction,
            # so `end` always lands inside this chunk and t_read is always
            # the right reference.
            dated.append([kind, body, t_read - (blen - end) * BYTE_TIME_S])
        del buf[:tail]
        if len(buf) > 4096:
            del buf[:-256]
        self._date_by_cadence(dated)
        last, ringed = None, None
        for kind, body, t_frame in dated:
            if not self._dec.feed(kind, body, t_frame):
                self._rejected += 1              # NaN / out-of-range values
                continue
            self._ctr_frames += 1
            last = t_frame
            if kind in ORIENTATION:
                self._publish(t_frame, ring=True)
                self._ctr_pubs += 1
                ringed = t_frame
        if last is not None and last != ringed:
            # The chunk ended with inertial/baro frames: refresh `latest` so
            # a caller polling between orientation frames sees current accel
            # - but do NOT ring it. A ring entry asserts "this attitude was
            # measured at this epoch", and this one's attitude is a copy of
            # the last orientation frame's. Ringing copies is what made the
            # ring look like 190 Hz while the attitude in it was up to 40 ms
            # stale, and it kept latest() perpetually fresh so a stalled unit
            # stayed invisible to piagent's staleness check.
            self._publish(last, ring=False)
        now = time.time()
        if now - self._ctr_t0 >= 1.0:
            self._rate = self._ctr_pubs / (now - self._ctr_t0)
            self._frame_rate = self._ctr_frames / (now - self._ctr_t0)
            self._rate_measured = True
            self._ctr_frames, self._ctr_pubs, self._ctr_t0 = 0, 0, now

    def _date_by_cadence(self, dated):
        """Re-date the older same-kind frames of a multi-cycle chunk.

        Byte-time dating assumes the wire was continuously busy, but this
        device uses it ~17%: when one read returns two or more device cycles
        (USB batching, a busy Pi), the idle gaps between cycles vanish from
        the arithmetic and the OLDER frames come out several-to-tens of ms
        late. The device's own cadence is the better ruler there: walk each
        kind's frames newest-to-oldest and pull every predecessor back to at
        least ~one period before its successor, i.e. bound the dating error
        to a fraction of one device cycle instead of the whole idle gap.
        Residual (documented, not modelled): if the device itself skipped
        frames the predecessor may truly be older than one period - it is
        dated one period back regardless - and the EMA carries the device's
        cadence jitter. The period is learned only from chunks holding
        exactly ONE frame of the kind (those are reliably byte-dated), so the
        model never feeds on its own corrections."""
        by_kind = {}
        for idx, (kind, _b, _t) in enumerate(dated):
            by_kind.setdefault(kind, []).append(idx)
        for kind, idxs in by_kind.items():
            ema = self._kind_ema.get(kind)
            if ema and len(idxs) > 1:
                for j in range(len(idxs) - 2, -1, -1):
                    cur, nxt = dated[idxs[j]], dated[idxs[j + 1]]
                    # 0.75: enough to restore most of a swallowed idle gap
                    # while never dating a frame before its real predecessor.
                    cur[2] = min(cur[2], nxt[2] - 0.75 * ema)
                # Floor: never date a frame at or before the newest frame of
                # the same kind from an EARLIER chunk. That one is already in
                # the ring, so reaching back past it would append the ring out
                # of order and assert an attitude was measured before one the
                # host may already have bound to a photo. A long read stall
                # followed by one big buffered chunk is exactly the case that
                # would otherwise walk several cycles back. Oldest-to-newest so
                # the frames stay strictly ordered when the floor does bite.
                prv = self._kind_prev.get(kind)
                if prv is not None:
                    lo = prv[0]
                    for j in range(len(idxs)):
                        cur = dated[idxs[j]]
                        if cur[2] <= lo:
                            cur[2] = lo + CADENCE_FLOOR_EPS_S
                        lo = cur[2]
            prev = self._kind_prev.get(kind)
            if prev is not None and len(idxs) == 1 \
                    and prev[1] == self._chunk_no - 1:
                delta = dated[idxs[0]][2] - prev[0]
                if 0.002 < delta < 0.5:
                    ema = delta if ema is None else 0.9 * ema + 0.1 * delta
                    self._kind_ema[kind] = ema
            self._kind_prev[kind] = (dated[idxs[-1]][2], self._chunk_no)

    def _publish(self, epoch, ring=True):
        samp = self._dec.sample(epoch)
        with self._lock:
            self._latest = samp
            if not ring:
                return
            self._last_ring_epoch = epoch
            self._ring.append((epoch, samp))
            cut = epoch - RING_S
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
        """Ring publishes per second - i.e. real orientation updates.

        NOT the frame rate: the previous counter counted every decoded frame
        plus every duplicate publish and reported ~190 Hz for a device whose
        attitude output moves far slower. Downstream code sizes its "how stale
        may the attitude bound to this photo be" budget off this number, so it
        has to mean sample cadence."""
        return round(self._rate, 1)

    def frame_rate_hz(self):
        """All decoded frames per second — link health, not sample cadence."""
        return round(self._frame_rate, 1)

    def rate_measured(self):
        """True once a full rate window has closed - i.e. rate_hz() is a
        MEASUREMENT. Before this, 0.0 means "not measured yet" and a probe
        fallback is legitimate; after it, 0.0 means the attitude stream has
        genuinely stopped and must be reported as such."""
        return self._rate_measured

    def rejected_frames(self):
        """Frames refused (checksum mismatch, NaN, out-of-range values).
        Lets 'attitude frames present but discarded' be told apart from
        'device sends none'."""
        return self._rejected

    def last_attitude_epoch(self):
        """Epoch of the last RING publish (a real orientation frame).
        latest() is deliberately refreshed by trailing inertial/baro frames,
        so it cannot answer 'is the ATTITUDE still flowing?' - this can."""
        with self._lock:
            return self._last_ring_epoch

    def checksum_state(self):
        return self._gate.state()

    def orientation_frozen_s(self):
        """Seconds since the attitude numbers last CHANGED, or None if none seen.

        A locked-up fusion keeps streaming identical values, so freshness of
        bytes proves nothing; this is the signal that catches it."""
        t = self._dec.orient_change_epoch()
        return None if t is None else round(time.time() - t, 3)


def _first_streaming_port():
    ports = []
    for pat in SERIAL_PORTS:
        ports.extend(sorted(glob.glob(pat)))
    for p in ports:
        st = _stream_stats(p)
        if st and st["frames"] >= 2:
            return p
    return None


def _stream_stats(port, dwell=0.5):
    """Listen for `dwell` seconds and count what actually arrives.

    Returns {frames, orient, elapsed_s} or None if the port will not talk.
    The rates are measured rather than assumed: the datasheet's 200 Hz is the
    sensor's internal rate, not the rate its fusion output reaches the wire at,
    and every consumer that bounds "how old can this attitude be" needs the real
    figure. Quoting 200 Hz when the truth is a fraction of that is how a 40 ms
    stale attitude gets written into a flight_log as if it were simultaneous."""
    if serial is None:
        return None
    try:
        s = serial.Serial(port, BAUD, timeout=0.1)
    except Exception:  # noqa: BLE001
        return None
    buf = bytearray()
    try:
        time.sleep(0.2)
        s.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < dwell:
            buf.extend(s.read(4096) or b"")
        elapsed = max(1e-3, time.time() - t0)
    except Exception:  # noqa: BLE001
        return None
    finally:
        try:
            s.close()
        except Exception:  # noqa: BLE001
            pass
    frames, _ = _scan(buf)
    return {"frames": len(frames), "elapsed_s": elapsed,
            "orient": sum(1 for (k, _b, _e, _f) in frames
                          if k in ORIENTATION)}


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
        st = _stream_stats(p)
        ok = bool(st) and st["frames"] >= 2
        out["notes"].append("%s: %s" % (p, "YB 0x7E frames present" if ok
                                        else "no valid frames at 115200"))
        if ok:
            out.update(present=True, chip="yahboom_yb-mra02 (0x7E protocol)",
                       port=p, baud=BAUD,
                       # Measured over the probe window, not a datasheet claim.
                       sample_rate_hz=round(st["orient"] / st["elapsed_s"], 1),
                       frame_rate_hz=round(st["frames"] / st["elapsed_s"], 1))
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
                  "t %5s | %sHz samples / %sHz frames | age %sms" % (
                      _fmt(s["roll"]), _fmt(s["pitch"]), _fmt(s["yaw"]),
                      _fmt(s["ax"], 2), _fmt(s["ay"], 2), _fmt(s["az"], 2),
                      _fmt(s["temp"], 1), r.rate_hz(), r.frame_rate_hz(),
                      _fmt((time.time() - s["epoch"]) * 1000)))
        time.sleep(0.2)
    r.close()

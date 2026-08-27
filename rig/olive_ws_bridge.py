#!/usr/bin/env python3
"""Olive olixVision IMU -> piagent bridge.

The unit presents USB-C as a CDC-Ethernet gadget (host side configured by
deploy: iface `olive0`, Pi .2 / unit .100 on 192.168.7.0/24) and streams its
AHRS state as protobuf-framed binary WebSocket messages on port 5500 at
~100 Hz (discovered 2026-08-27; the advertised ROS 2/DDS path needs a DDS
stack the Pi image cannot build, and the REST paths are a Blazor SPA shell).

This bridge is deliberately dumb: connect, parse, re-emit each sample as one
flat JSON datagram to 127.0.0.1:9901, where piagent's imu2 slot
(PIAGENT_IMU2=olive:udp:9901, rig/imu_olive.py UdpTransport) ingests it with
its normal codec. No schema, no generated code: the wire fields are decoded
with a generic protobuf walker and mapped by the shapes observed on the real
unit:

    field 5  submsg of 4 floats   quaternion x,y,z,w   (x1-pro, AHRS mode)
    field 6  repeated fixed32 x9  magnetometer row + zeros (mG; only 3 nonzero)
    field 7  submsg of 3 floats   gyro rad/s
    field 9  submsg of 3 floats   accel m/s^2
    field 15/16/17 strings        "AHRS-", "x1-pro", "imu"

Units are converted here to the rig's IMU dialect (g, dps, degrees) so the
sample the driver publishes is directly comparable with imu_yb's. The unit's
own clock is ~2 years wrong (28-05-2024 seen live), so samples carry ONLY the
arrival epoch - the driver already refuses to trust device time.

Runs as a systemd unit (deploy/olive-bridge.service) on every node: where no
olive0 interface exists it idles in a slow retry loop, costing nothing.
"""

import json
import math
import os
import socket
import struct
import sys
import time

OLIVE = os.environ.get("OLIVE_WS", "192.168.7.100:5500")
DEST = ("127.0.0.1", int(os.environ.get("OLIVE_UDP_PORT", "9901")))
G = 9.80665


def log(msg):
    sys.stderr.write("olive-bridge: %s\n" % msg)
    sys.stderr.flush()


# ---------------------------------------------------------------------------
# minimal protobuf wire walker (schema-less)
# ---------------------------------------------------------------------------
def _varint(b, i):
    v = s = 0
    while True:
        x = b[i]
        i += 1
        v |= (x & 0x7F) << s
        if not x & 0x80:
            return v, i
        s += 7


def pb_fields(b):
    """[(field_no, wire_type, value)] - len-delimited values stay bytes."""
    out = []
    i = 0
    n = len(b)
    while i < n:
        tag, i = _varint(b, i)
        f, wt = tag >> 3, tag & 7
        if wt == 0:
            v, i = _varint(b, i)
        elif wt == 5:
            v = struct.unpack("<f", b[i:i + 4])[0]
            i += 4
        elif wt == 1:
            v = struct.unpack("<d", b[i:i + 8])[0]
            i += 8
        elif wt == 2:
            ln, i = _varint(b, i)
            v = b[i:i + ln]
            i += ln
        else:
            break
        out.append((f, wt, v))
    return out


def floats_of(sub):
    return [v for f, wt, v in pb_fields(sub) if wt == 5]


def quat_to_euler(x, y, z, w):
    """Same convention as imu_yb: aerospace ZYX, degrees."""
    roll = math.degrees(math.atan2(2 * (w * x + y * z),
                                   1 - 2 * (x * x + y * y)))
    s = 2 * (w * y - z * x)
    s = max(-1.0, min(1.0, s))
    pitch = math.degrees(math.asin(s))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                  1 - 2 * (y * y + z * z)))
    return pitch, roll, yaw


def parse_sample(payload):
    quat = gyro = accel = None
    mag9 = []
    for f, wt, v in pb_fields(payload):
        if f == 5 and wt == 2:
            q = floats_of(v)
            if len(q) == 4:
                quat = q
        elif f == 6 and wt == 5:
            mag9.append(v)
        elif f == 7 and wt == 2:
            g3 = floats_of(v)
            if len(g3) == 3:
                gyro = g3
        elif f == 9 and wt == 2:
            a3 = floats_of(v)
            if len(a3) == 3:
                accel = a3
    if quat is None or accel is None:
        return None
    x, y, z, w = quat
    pitch, roll, yaw = quat_to_euler(x, y, z, w)
    mag = [m for m in mag9 if m] or [0.0, 0.0, 0.0]
    # Shapes chosen for imu_olive's JSON codec: euler as roll/pitch/yaw
    # scalars (the quaternion is NOT forwarded - element order across
    # conventions is ambiguous and the euler here is already computed with
    # the rig's convention), inertial channels as 3-vectors, all in RIG units
    # (g, dps, degrees) so the driver's gravity gate reads |accel|~1 and
    # locks unit_mode="rig".
    doc = {
        "epoch": round(time.time(), 6),
        "pitch": round(pitch, 3), "roll": round(roll, 3),
        "yaw": round(yaw, 3),
        "accel": [round(accel[0] / G, 5), round(accel[1] / G, 5),
                  round(accel[2] / G, 5)],
        "gyro": ([round(math.degrees(v), 4) for v in gyro]
                 if gyro else [0.0, 0.0, 0.0]),
        "mag": [round(mag[0], 2), round(mag[1] if len(mag) > 1 else 0, 2),
                round(mag[2] if len(mag) > 2 else 0, 2)],
        "src": "olive-ws",
    }
    return doc


# ---------------------------------------------------------------------------
# tiny websocket client (stdlib)
# ---------------------------------------------------------------------------
def ws_connect(hostport, path="/", timeout=5.0):
    host, port = hostport.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=timeout)
    import base64
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall(("GET %s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n" % (path, hostport, key))
              .encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = s.recv(1024)
        if not chunk:
            raise ConnectionError("closed during handshake")
        buf += chunk
    head, rest = buf.split(b"\r\n\r\n", 1)
    if b" 101 " not in head.split(b"\r\n", 1)[0]:
        raise ConnectionError("no upgrade: %r" % head[:80])
    return s, rest


def ws_frames(s, rest):
    """Yield (opcode, payload). Server frames are unmasked per RFC 6455."""
    buf = rest
    s.settimeout(5.0)
    while True:
        while True:
            if len(buf) >= 2:
                b1, b2 = buf[0], buf[1]
                ln = b2 & 0x7F
                off = 2
                if ln == 126 and len(buf) >= 4:
                    ln = struct.unpack(">H", buf[2:4])[0]
                    off = 4
                elif ln == 127 and len(buf) >= 10:
                    ln = struct.unpack(">Q", buf[2:10])[0]
                    off = 10
                elif ln >= 126:
                    pass
                if ln < 126 or off > 2:
                    if len(buf) >= off + ln:
                        yield b1 & 0x0F, buf[off:off + ln]
                        buf = buf[off + ln:]
                        continue
            chunk = s.recv(65536)
            if not chunk:
                raise ConnectionError("stream closed")
            buf += chunk


def main():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    backoff = 2.0
    sent = 0
    last_note = 0.0
    while True:
        try:
            s, rest = ws_connect(OLIVE)
            log("connected to ws://%s (-> udp %s:%d)" % (OLIVE, *DEST))
            backoff = 2.0
            for op, payload in ws_frames(s, rest):
                if op == 8:                    # close
                    raise ConnectionError("server close")
                if op not in (1, 2):
                    continue
                doc = parse_sample(payload)
                if doc is None:
                    continue
                udp.sendto(json.dumps(doc, separators=(",", ":"))
                           .encode(), DEST)
                sent += 1
                now = time.time()
                if now - last_note > 60:
                    log("streaming: %d samples relayed" % sent)
                    last_note = now
        except (OSError, ConnectionError) as e:
            # No olive0 / unit unplugged / unit rebooting: quiet retry. This
            # unit runs on every node; on a Pi with no Olive this loop IS the
            # steady state and must cost nothing.
            now = time.time()
            if now - last_note > 300:
                log("not connected (%s) - retrying every %.0fs" % (e, backoff))
                last_note = now
            time.sleep(backoff)
            backoff = min(backoff * 1.5, 30.0)


if __name__ == "__main__":
    main()

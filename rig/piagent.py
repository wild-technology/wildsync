#!/usr/bin/env python3
"""piagent — the Pi-side hardware agent for a Wild Sync camera node.

One of these runs on every camera Pi alongside `ilxctl`. Where ilxctl owns the
USB/SDK path (settings, live view, image transfer), piagent owns everything
that is *not* the SDK:

    * the GPIO trigger harness  — FOCUS hold, TRIGGER firing on an absolute
      clock, and the EXPOSURE capture-edge monitor;
    * the IMU (cam3 only)       — via rig/imu_yb.py, sampled into a ring;
    * node health + a time probe the Jetson uses to align clocks.

It speaks plain JSON over HTTP on :8081 and depends on nothing outside the
standard library (imu_yb and pyserial are imported lazily and the agent runs
fine without them). Everything here is written to survive a field deployment:
a killed monitor is respawned, the FOCUS line is *always* released on the way
out, and every endpoint answers even when the hardware it fronts is absent.

Design notes that matter:

  * FOCUS is held by a long-lived `gpioset --mode=signal ... 17=0` process in
    open-drain. Releasing = killing it; the line then floats to the camera's
    own 2.8 V, which is the safe "not pressed" state. We never drive it high.
  * TRIGGER is pulsed with `gpioset --mode=time` in open-drain. To fire on a
    shared clock, we busy-wait to the requested epoch and stamp the instant the
    low actually starts, so the Jetson can measure real inter-camera skew.
  * The kernel edge timestamps from gpiomon are a different clock across
    libgpiod builds, so we stamp each EXPOSURE edge with wall time at read and
    keep the raw device ts only for interval math.
"""

import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------------------------------------------------------------------
# Configuration — the harness is fixed by rig/PROTOCOL.md.
# ---------------------------------------------------------------------------
BCM_FOCUS = 17      # header pin 11 -> harness pin 4
BCM_TRIGGER = 27    # header pin 13 -> harness pin 5
BCM_EXPOSURE = 22   # header pin 15 -> harness pin 6

# Guard window for EXPOSURE edge debouncing. Well under the >=1 ms Sony
# specifies for the assert (measured ~13 ms), and far above the ~60 us bursts a
# ringing harness produces.
EDGE_DEBOUNCE_S = 0.001

PORT = int(os.environ.get("PIAGENT_PORT", "8081"))
LOG_PATH = os.path.expanduser("~/rig/piagent.jsonl")
RING_SECONDS = 90        # EXPOSURE edges and IMU samples kept at least this long
CAM_SAVE_DIR = os.path.expanduser("~/Pictures/ILX-LR1")

NODE = socket.gethostname()
T_START = time.time()


# ---------------------------------------------------------------------------
# Structured logging — one JSON object per line, mirrored to stderr.
# ---------------------------------------------------------------------------
_log_lock = threading.Lock()


def log(sev, kind, msg, **ctx):
    rec = {"ts": round(time.time(), 3), "node": NODE, "sev": sev,
           "kind": kind, "msg": msg}
    if ctx:
        rec["ctx"] = ctx
    line = json.dumps(rec, separators=(",", ":"))
    with _log_lock:
        try:
            os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
            with open(LOG_PATH, "a") as fh:
                fh.write(line + "\n")
        except OSError:
            pass
        sys.stderr.write(line + "\n")
        sys.stderr.flush()


# ---------------------------------------------------------------------------
# GPIO chip discovery — Pi 4 is gpiochip0, Pi 5 moved the header to gpiochip4.
# We resolve it once by asking which chip actually carries the pinctrl driver,
# rather than trusting a hard-coded number that a kernel bump could shift.
# ---------------------------------------------------------------------------
def _run(cmd, timeout=5):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
        return 127, "", str(e)


def discover_gpiochip():
    """Return the gpiochip name whose driver is the Pi header controller."""
    rc, out, _ = _run(["gpiodetect"])
    if rc != 0:
        # gpiodetect often needs root until the udev rule lands; try with sudo.
        rc, out, _ = _run(["sudo", "-n", "gpiodetect"])
    best = None
    for line in out.splitlines():
        # e.g. "gpiochip0 [pinctrl-bcm2711] (58 lines)"
        m = re.match(r"(gpiochip\d+)\s+\[([^\]]+)\]\s+\((\d+)\s+lines\)", line)
        if not m:
            continue
        name, label, lines = m.group(1), m.group(2), int(m.group(3))
        if "pinctrl" in label and lines >= 40:
            best = name
            break
    return best or "gpiochip0"


GPIOCHIP = discover_gpiochip()
_HAVE_GPIO = shutil.which("gpioset") is not None
# On the Pi the udev rule may still be missing; fall back to sudo -n so the
# service works either way. Resolved once at startup.
_GPIO_SUDO = []
if _HAVE_GPIO:
    rc, _, _ = _run(["gpioget", "--bias=pull-up", GPIOCHIP, str(BCM_EXPOSURE)])
    if rc != 0:
        rc2, _, _ = _run(["sudo", "-n", "gpioget", "--bias=pull-up",
                          GPIOCHIP, str(BCM_EXPOSURE)])
        if rc2 == 0:
            _GPIO_SUDO = ["sudo", "-n"]


def _gpio(cmd):
    return _GPIO_SUDO + cmd


# ---------------------------------------------------------------------------
# In-process line driver — the low-jitter trigger path
# ---------------------------------------------------------------------------
class _LineDriver:
    """Persistent open-drain handles on FOCUS and TRIGGER via libgpiod.

    piagent used to shell out to `gpioset` for every shot. That spawn is not
    free and, worse, it is not the same cost on every node: measured 2.83 ms on
    a Pi 5 and 12.39 ms on a Pi 4. fire() stamps the fire time and *then*
    spawns, so the whole spawn cost lands after the recorded instant - a
    systematic ~9.6 ms skew between two cameras that is invisible in the logs
    and fatal to stereo photogrammetry.

    Holding the line open lets a shot be two register writes instead of a
    process launch. An open-drain output driven to 1 is not driven at all
    (high-Z), which is exactly the safe idle state for a camera input, so this
    handle also *is* the park - no separate parker process, and no window
    between releasing one and claiming the other."""

    IDLE, ASSERT = 1, 0                  # open-drain: 1 == high-Z, 0 == pulled low

    def __init__(self, chip_name, lines):
        self.ok = False
        self._lines = {}
        self._chip = None
        try:
            import gpiod
        except ImportError:
            return
        try:
            self._chip = gpiod.Chip(chip_name)
            flags = getattr(gpiod, "LINE_REQ_FLAG_OPEN_DRAIN", 0)
            # Bias is a newer kernel/libgpiod feature; if the build lacks it the
            # open-drain idle still leaves the line high-Z, which is safe.
            flags |= getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_UP", 0)
            for bcm in lines:
                ln = self._chip.get_line(bcm)
                ln.request(consumer="wildsync", type=gpiod.LINE_REQ_DIR_OUT,
                           flags=flags, default_vals=[self.IDLE])
                self._lines[bcm] = ln
            self.ok = True
        except Exception as e:  # noqa: BLE001
            log("warn", "gpio_driver",
                "libgpiod direct path unavailable, falling back to gpioset",
                err=str(e))
            self.close()

    def set(self, bcm, value):
        ln = self._lines.get(bcm)
        if ln is None:
            return False
        try:
            ln.set_value(value)
            return True
        except Exception:  # noqa: BLE001
            return False

    def pulse(self, bcm, hold_s):
        """Assert, hold, release. Returns the instant the line actually fell."""
        ln = self._lines.get(bcm)
        if ln is None:
            return None
        ln.set_value(self.ASSERT)
        t = time.time()                  # stamped AFTER the write, not before
        self._spin_until(t + hold_s)
        ln.set_value(self.IDLE)
        return t

    @staticmethod
    def _spin_until(end):
        while True:
            rem = end - time.time()
            if rem <= 0:
                return
            time.sleep(rem if rem < 0.002 else rem - 0.001)

    def shot(self, focus_bcm, trigger_bcm, lead_s, pulse_s):
        """One frame: FOCUS leads, TRIGGER fires, both release.

        The camera only accepts a TRIGGER while FOCUS is already Low, but
        holding FOCUS down for a whole transect half-presses the body for the
        duration, which AE-locks it - auto-ISO would freeze at whatever the
        light was when the run started. Asserting FOCUS a few tens of
        milliseconds ahead of each shot satisfies the camera and still lets it
        meter every frame. Both edges are register writes in this process, so
        the lead is precise regardless of how busy the host is."""
        f = self._lines.get(focus_bcm)
        t_ln = self._lines.get(trigger_bcm)
        if f is None or t_ln is None:
            return None
        f.set_value(self.ASSERT)
        try:
            self._spin_until(time.time() + lead_s)
            t_ln.set_value(self.ASSERT)
            t = time.time()
            self._spin_until(t + pulse_s)
            t_ln.set_value(self.IDLE)
        finally:
            # FOCUS must never be left asserted: that is a permanent half-press,
            # which locks the whole property table and looks like a dead camera.
            f.set_value(self.IDLE)
        return t

    def close(self):
        for ln in self._lines.values():
            try:
                ln.release()
            except Exception:  # noqa: BLE001
                pass
        self._lines = {}
        if self._chip is not None:
            try:
                self._chip.close()
            except Exception:  # noqa: BLE001
                pass
            self._chip = None
        self.ok = False


# ---------------------------------------------------------------------------
# GPIO manager
# ---------------------------------------------------------------------------
class Gpio:
    # FOCUS and TRIGGER are camera INPUTS, idle-high through the body's own
    # pull-up. A released libgpiod line does not idle high-Z: the pad falls back
    # to its power-on bias, and on this Pi 5 that leaves BCM17 (FOCUS) reading 0.
    # Wired up, that is a permanent half-press - which locks AE/AF, so ISO,
    # aperture and focus mode all report DisplayOnly, the body takes control
    # priority back from the PC, and the remote shutter is ignored. It looks
    # exactly like a dead camera and clears the instant the connector is pulled.
    #
    # So an unasserted input line is never merely released: it is parked under a
    # long-lived `gpiomon --bias=pull-up`, which holds it as an input with a
    # pull-up for as long as the process lives. Asserting stops the parker,
    # drives open-drain, and re-parks.
    PARKED_INPUTS = (BCM_FOCUS, BCM_TRIGGER)

    def __init__(self):
        self.chip = GPIOCHIP
        self._lock = threading.RLock()
        self._focus_proc = None          # long-lived gpioset holding FOCUS low
        self._parkers = {}               # bcm -> Popen holding the line idle-high
        self._focus_direct = False       # FOCUS state when using the gpiod path
        self._interval_stop = None
        self._interval_thread = None
        self._interval_state = {"running": False, "fired": 0, "target": 0,
                                "period_s": 0.0, "last_late_ms": None}
        self._edges = deque(maxlen=20000)   # (epoch, edge, raw_ts, seq)
        self._edge_seq = 0
        self._last_edge = {}                # edge -> last accepted timestamp
        self._bounced = 0                   # edges dropped as ring/bounce
        self._mon_proc = None
        self._mon_thread = None
        self._mon_run = False
        self.available = _HAVE_GPIO
        # Prefer the in-process driver: it removes the per-shot subprocess spawn
        # that made trigger latency platform-dependent, and its open-drain idle
        # doubles as the safe park for both camera inputs.
        self.driver = _LineDriver(self.chip, self.PARKED_INPUTS)
        if self.available:
            self._start_monitor()
            if not self.driver.ok:
                for bcm in self.PARKED_INPUTS:
                    self._park(bcm)

    # ---- safe parking of the camera-input lines ---------------------------
    def _park(self, bcm):
        """Hold `bcm` as an input with a pull-up so the camera sees it idle."""
        p = self._parkers.get(bcm)
        if p is not None and p.poll() is None:
            return True
        cmd = _gpio(["gpiomon", "--bias=pull-up", "--num-events=0",
                     self.chip, str(bcm)])
        try:
            self._parkers[bcm] = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as e:
            log("error", "gpio_error", "could not park line idle-high",
                bcm=bcm, err=str(e))
            self._parkers.pop(bcm, None)
            return False
        time.sleep(0.05)
        if self._parkers[bcm].poll() is not None:
            # Older gpiomon builds reject --num-events=0; fall back to a large
            # count, which parks the line just as effectively in practice.
            cmd = _gpio(["gpiomon", "--bias=pull-up", "--num-events=1000000000",
                         self.chip, str(bcm)])
            try:
                self._parkers[bcm] = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(0.05)
            except OSError:
                self._parkers.pop(bcm, None)
                return False
        alive = self._parkers.get(bcm) is not None and self._parkers[bcm].poll() is None
        if not alive:
            log("error", "gpio_error", "line parker would not stay up", bcm=bcm)
            self._parkers.pop(bcm, None)
        return alive

    def _unpark(self, bcm):
        p = self._parkers.pop(bcm, None)
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()

    def parked(self):
        return {bcm: (p is not None and p.poll() is None)
                for bcm, p in self._parkers.items()}

    # ---- FOCUS ------------------------------------------------------------
    def focus_held(self):
        with self._lock:
            if self.driver.ok:
                return self._focus_direct
            return self._focus_proc is not None and self._focus_proc.poll() is None

    def focus(self, hold):
        with self._lock:
            if self.driver.ok:
                ok = self.driver.set(
                    BCM_FOCUS,
                    _LineDriver.ASSERT if hold else _LineDriver.IDLE)
                if ok:
                    self._focus_direct = bool(hold)
                    log("info", "gpio_focus",
                        "FOCUS held low" if hold else "FOCUS released")
                return ok
            if hold:
                if self.focus_held():
                    return True
                self._unpark(BCM_FOCUS)
                cmd = _gpio(["gpioset", "--drive=open-drain", "--mode=signal",
                             self.chip, "%d=0" % BCM_FOCUS])
                try:
                    self._focus_proc = subprocess.Popen(
                        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError as e:
                    log("error", "gpio_error", "focus hold failed", err=str(e))
                    self._park(BCM_FOCUS)
                    return False
                # Give it a beat to actually claim the line, then confirm alive.
                time.sleep(0.05)
                if self._focus_proc.poll() is not None:
                    log("error", "gpio_error", "focus holder died immediately")
                    self._focus_proc = None
                    self._park(BCM_FOCUS)
                    return False
                log("info", "gpio_focus", "FOCUS held low")
                return True
            else:
                self._release_focus()
                return True

    def _release_focus(self):
        p = self._focus_proc
        self._focus_proc = None
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
            log("info", "gpio_focus", "FOCUS released")
        # Re-park unconditionally: dropping the holder is not enough, because a
        # merely-released line falls back to a pad bias that reads Low here.
        self._park(BCM_FOCUS)

    # ---- TRIGGER ----------------------------------------------------------
    def fire(self, at_epoch, pulse_ms, focus_lead_ms=0):
        if not self.available:
            return {"ok": False, "error": "no gpio on this node"}
        focus_lead_ms = max(0, min(int(focus_lead_ms or 0), 500))
        # A caller that asks for its own FOCUS lead does not need FOCUS to be
        # held already - that is the point: per-shot FOCUS keeps auto-exposure
        # live, where a continuous hold would AE-lock the body for the run.
        if not focus_lead_ms and not self.focus_held():
            return {"ok": False, "error": "FOCUS not held", "code": 409}
        pulse_ms = max(1, min(int(pulse_ms), 200))
        # Busy-wait the final approach so scheduling jitter does not smear the
        # fire time; sleep the coarse part to stay off the CPU.
        # Wake early enough to place the FOCUS lead before the target instant,
        # so it is the TRIGGER that lands on time, not the start of the sequence.
        lead_s = focus_lead_ms / 1000.0
        if at_epoch and at_epoch > 0:
            while True:
                dt = (at_epoch - lead_s) - time.time()
                if dt <= 0:
                    break
                if dt > 0.02:
                    time.sleep(dt - 0.015)
                # else spin
        requested = at_epoch if at_epoch else time.time()
        if self.driver.ok:
            # Two register writes. `actual` is stamped after the line is already
            # low, so it is the real assert instant rather than the moment we
            # decided to assert - the subprocess path could not tell the two
            # apart, and the gap between them differed by ~9.6 ms across Pi
            # models.
            if focus_lead_ms:
                actual = self.driver.shot(BCM_FOCUS, BCM_TRIGGER, lead_s,
                                          pulse_ms / 1000.0)
            else:
                actual = self.driver.pulse(BCM_TRIGGER, pulse_ms / 1000.0)
            if actual is None:
                return {"ok": False, "error": "trigger line unavailable"}
            late_ms = (actual - requested) * 1000.0
            if abs(late_ms) > 50:
                log("warn", "gpio_late", "trigger fired late",
                    late_ms=round(late_ms, 1))
            return {"ok": True, "requested_epoch": requested,
                    "actual_epoch": actual, "late_ms": round(late_ms, 2),
                    "pulse_ms": pulse_ms, "path": "gpiod"}
        actual = time.time()
        self._unpark(BCM_TRIGGER)
        cmd = _gpio(["gpioset", "--drive=open-drain", "--mode=time",
                     "--usec=%d" % (pulse_ms * 1000), self.chip,
                     "%d=0" % BCM_TRIGGER])
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,
                           timeout=pulse_ms / 1000.0 + 3)
        except (subprocess.TimeoutExpired, OSError) as e:
            log("error", "gpio_error", "trigger pulse failed", err=str(e))
            self._park(BCM_TRIGGER)
            return {"ok": False, "error": str(e)}
        finally_parked = self._park(BCM_TRIGGER)
        if not finally_parked:
            log("error", "gpio_error",
                "TRIGGER could not be re-parked idle-high - unplug the harness "
                "if the camera stops responding", bcm=BCM_TRIGGER)
        late_ms = (actual - requested) * 1000.0
        if abs(late_ms) > 50:
            log("warn", "gpio_late", "trigger fired late",
                late_ms=round(late_ms, 1))
        return {"ok": True, "requested_epoch": requested,
                "actual_epoch": actual, "late_ms": round(late_ms, 2),
                "pulse_ms": pulse_ms}

    # ---- interval ---------------------------------------------------------
    def interval_start(self, at_epoch, period_s, count):
        with self._lock:
            if self._interval_state["running"]:
                return {"ok": False, "error": "interval already running"}
            if not self.focus_held():
                return {"ok": False, "error": "FOCUS not held", "code": 409}
            period_s = max(0.05, float(period_s))
            self._interval_stop = threading.Event()
            self._interval_state.update(running=True, fired=0,
                                        target=int(count), period_s=period_s)
            t = threading.Thread(target=self._interval_loop,
                                 args=(at_epoch or time.time(), period_s,
                                       int(count)), daemon=True)
            self._interval_thread = t
            t.start()
            log("info", "interval", "interval started",
                period_s=period_s, count=count)
            return {"ok": True, "start_epoch": at_epoch or time.time(),
                    "period_s": period_s, "count": count}

    def _interval_loop(self, start, period, count):
        k = 0
        stop = self._interval_stop
        while not stop.is_set():
            target = start + k * period      # absolute schedule: no drift
            if count and k >= count:
                break
            res = self.fire(target, 5)
            if res.get("ok"):
                with self._lock:
                    self._interval_state["fired"] += 1
                    self._interval_state["last_late_ms"] = res.get("late_ms")
            else:
                log("warn", "interval", "interval frame failed",
                    reason=res.get("error"))
            k += 1
            # Wait for the next slot without holding the lock.
            nxt = start + k * period
            while not stop.is_set():
                dt = nxt - time.time()
                if dt <= 0:
                    break
                stop.wait(min(dt, 0.25))
        with self._lock:
            self._interval_state["running"] = False
        log("info", "interval", "interval finished",
            fired=self._interval_state["fired"])

    def interval_stop(self):
        with self._lock:
            if self._interval_stop:
                self._interval_stop.set()
            self._interval_state["running"] = False
        return {"ok": True}

    def interval_status(self):
        with self._lock:
            return dict(self._interval_state)

    # ---- EXPOSURE monitor -------------------------------------------------
    def _start_monitor(self):
        self._mon_run = True
        self._mon_thread = threading.Thread(target=self._monitor_loop,
                                            daemon=True)
        self._mon_thread.start()

    def _monitor_loop(self):
        """Long-lived gpiomon on EXPOSURE; respawn if it ever dies."""
        fmt = "%e %s.%n"
        while self._mon_run:
            # stdbuf forces gpiomon to line-buffer: it is a C program printing to
            # a pipe, which otherwise block-buffers, so infrequent EXPOSURE edges
            # would sit unflushed and never reach us (edges_seen stuck at 0).
            cmd = _gpio(["stdbuf", "-oL", "-eL", "gpiomon", "--bias=pull-up",
                         "--format=" + fmt, self.chip, str(BCM_EXPOSURE)])
            try:
                self._mon_proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, bufsize=1)
            except OSError as e:
                log("error", "gpio_error", "gpiomon spawn failed", err=str(e))
                time.sleep(2)
                continue
            log("debug", "gpio_monitor", "EXPOSURE monitor up", chip=self.chip)
            try:
                for line in self._mon_proc.stdout:
                    epoch = time.time()      # wall clock at read — see module doc
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    ev = parts[0] if parts else ""
                    raw = None
                    if len(parts) > 1:
                        try:
                            raw = float(parts[1])
                        except ValueError:
                            raw = None
                    # gpiomon %e encodes the edge, but the numbering varies by
                    # libgpiod build: v1.6.3 here emits "0" for falling / "1" for
                    # rising, while other builds use "2" for falling. "1" is
                    # always rising, so treat 0/2/FALLING as the fall. EXPOSURE
                    # idles high (pull-up) and falls when the curtain opens, so a
                    # falling edge is the capture instant.
                    edge = "rise" if ev in ("1", "RISING") else "fall"
                    # Debounce. On a long or unterminated harness the EXPOSURE
                    # line rings: cam2 produced 14 consecutive "fall" events
                    # inside 62 us, which no real signal can do. The FIRST edge
                    # of a burst is the true capture instant, so keep it and
                    # discard same-direction repeats inside the guard window.
                    # Sony specs EXPOSURE as asserted >=1 ms and it measures
                    # ~13 ms here, so 1 ms cannot swallow a genuine edge.
                    ref = raw if raw is not None else epoch
                    last = self._last_edge.get(edge)
                    if last is not None and (ref - last) < EDGE_DEBOUNCE_S:
                        # Do NOT advance the reference on a dropped edge. Doing
                        # so makes the dead time self-extending: a line that
                        # rings faster than the guard window keeps pushing the
                        # window forward and every genuine edge after the first
                        # is swallowed for as long as the ringing lasts. cam2's
                        # harness does exactly that, and it silently stopped
                        # reporting exposures altogether. The window must run
                        # from the last ACCEPTED edge only.
                        self._bounced += 1
                        continue
                    self._last_edge[edge] = ref
                    with self._lock:
                        self._edge_seq += 1
                        self._edges.append((epoch, edge, raw, self._edge_seq))
                    if edge == "fall":
                        log("debug", "exposure_edge", "capture edge",
                            epoch=round(epoch, 4))
            except Exception as e:  # noqa: BLE001 - monitor must never die silently
                log("warn", "gpio_monitor", "monitor read error", err=str(e))
            if self._mon_run:
                time.sleep(1)      # brief backoff before respawn

    def monitor_running(self):
        return self._mon_proc is not None and self._mon_proc.poll() is None

    def exposure_events(self, since):
        with self._lock:
            evs = [e for e in self._edges if e[3] > since]
            nxt = self._edge_seq
        return {"next": nxt,
                "events": [{"i": s, "edge": ed, "epoch": ep, "raw_ts": raw}
                           for (ep, ed, raw, s) in evs]}

    def state(self):
        parked = self.parked()
        return {"chip": self.chip, "available": self.available,
                "focus_held": self.focus_held(),
                "monitor_running": self.monitor_running(),
                "interval": self.interval_status(),
                "edges_seen": self._edge_seq,
                # An unparked camera-input line can hold the body in a permanent
                # half-press, so the fleet view needs to see this, not just the
                # edge count. False here is an operator-visible fault.
                "inputs_parked": parked,
                "trigger_path": "gpiod" if self.driver.ok else "gpioset",
                "edges_bounced": self._bounced,
                # With the gpiod driver the lines are held open-drain-idle for
                # the life of the process, which is the safe state by construction.
                "harness_safe": (self.driver.ok
                                 or all(parked.get(b) for b in self.PARKED_INPUTS))}

    def shutdown(self):
        self._mon_run = False
        self.interval_stop()
        if self._mon_proc and self._mon_proc.poll() is None:
            self._mon_proc.terminate()
        # Park before the parkers die with us. libgpiod v1 leaves the pad's bias
        # in place after the line is released, so a pull-up applied here still
        # protects the camera while piagent is stopped - which is the whole
        # window a restart or a crash opens up.
        if self.driver.ok:
            for bcm in self.PARKED_INPUTS:
                self.driver.set(bcm, _LineDriver.IDLE)
            self.driver.close()
        else:
            self._release_focus()
        # Leave a pull-up on the pads either way: once our handles are gone the
        # line reverts to pad bias, and a bare release lands on a pull-DOWN here
        # - which is a permanent half-press on the camera.
        for bcm in self.PARKED_INPUTS:
            self._park(bcm)
            self._unpark(bcm)


# ---------------------------------------------------------------------------
# IMU — optional, cam3 only. Imported lazily so nodes without it still run.
# ---------------------------------------------------------------------------
class Imu:
    """Owns the IMU reader and keeps trying to (re)acquire it.

    The IMU is a USB-serial device; it may be plugged in after piagent starts,
    moved between ports, or drop out. So detection runs in a background loop
    that probes until a device answers, samples it, and re-probes if the reader
    ever dies — no service restart needed when the user reseats it.
    """

    def __init__(self):
        self.reader = None
        self.info = {"present": False}
        self._lock = threading.Lock()
        self._run = True
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        threading.Thread(target=self._acquire_loop, daemon=True).start()

    def _acquire_loop(self):
        try:
            import imu_yb  # noqa: E402
        except Exception as e:  # noqa: BLE001
            log("info", "imu", "imu_yb not available", err=str(e))
            return
        announced_absent = False
        while self._run:
            if self.reader is not None:
                # Alive? If the sampler died or samples went stale, drop it and
                # re-acquire.
                try:
                    s = self.reader.latest()
                    fresh = s and s.get("epoch") and \
                        (time.time() - s["epoch"]) < 5
                except Exception:  # noqa: BLE001
                    fresh = False
                if not fresh:
                    log("warn", "imu", "IMU sampler stalled - re-acquiring")
                    self._drop()
                else:
                    time.sleep(2)
                    continue
            try:
                info = imu_yb.probe()
            except Exception:  # noqa: BLE001
                info = None
            if info and info.get("chip"):
                try:
                    reader = imu_yb.ImuReader(**{
                        k: info[k] for k in ("bus", "port", "addr", "baud")
                        if k in info and info[k] is not None})
                    reader.open()
                    reader.start()
                    with self._lock:
                        self.reader = reader
                        self.info = {"present": True, **info}
                    announced_absent = False
                    log("info", "imu", "IMU online", **{k: info.get(k) for k in
                        ("chip", "bus", "port", "addr", "baud")})
                except Exception as e:  # noqa: BLE001
                    log("warn", "imu", "IMU start failed", err=str(e))
                    self._drop()
            elif not announced_absent:
                announced_absent = True
                log("info", "imu", "no IMU yet - will keep probing "
                    "(USB-serial on /dev/ttyUSB*/ttyACM*)")
            time.sleep(4)

    def _drop(self):
        with self._lock:
            r, self.reader, self.info = self.reader, None, {"present": False}
        if r:
            try:
                r.stop(); r.close()
            except Exception:  # noqa: BLE001
                pass

    def latest(self):
        if not self.reader:
            return {"present": False}
        try:
            s = self.reader.latest()
            return s if s else {"present": True, "sample": None}
        except Exception as e:  # noqa: BLE001
            return {"present": True, "error": str(e)}

    def window(self, t0, t1):
        if not self.reader:
            return {"present": False, "samples": []}
        try:
            return {"present": True, "samples": self.reader.window(t0, t1)}
        except Exception as e:  # noqa: BLE001
            return {"present": True, "error": str(e), "samples": []}

    def health(self):
        h = {"present": bool(self.reader)}
        if self.reader:
            try:
                s = self.reader.latest()
                h["rate_hz"] = self.info.get("sample_rate_hz")
                h["age_s"] = round(time.time() - s["epoch"], 3) if s and \
                    s.get("epoch") else None
            except Exception:  # noqa: BLE001
                h["age_s"] = None
        return h

    def shutdown(self):
        self._run = False
        self._drop()


GPIO = Gpio()
IMU = Imu()


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
def _disk_free_mb(path):
    try:
        st = os.statvfs(path)
        return round(st.f_bavail * st.f_frsize / 1e6)
    except OSError:
        return None


def _load1():
    try:
        return round(os.getloadavg()[0], 2)
    except OSError:
        return None


def health():
    return {
        "node": NODE,
        "uptime_s": round(time.time() - T_START, 1),
        "gpio": GPIO.state(),
        "imu": IMU.health(),
        "disk_free_mb": _disk_free_mb(CAM_SAVE_DIR),
        "cam_frames": _count_frames(),
        "load1": _load1(),
        "time": {"epoch": time.time(), "source": "local"},
    }


def _count_frames():
    try:
        return sum(1 for f in os.listdir(CAM_SAVE_DIR)
                   if f.lower().endswith((".jpg", ".arw", ".jpeg")))
    except OSError:
        return None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):      # silence default logging
        pass

    def _send(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            n = 0
        if n <= 0 or n > 1 << 20:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode() or "{}")
        except (ValueError, OSError):
            return {}

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        q = {}
        if "?" in self.path:
            for kv in self.path.split("?", 1)[1].split("&"):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    q[k] = v
        try:
            if path == "/health":
                self._send(health())
            elif path == "/imu/latest":
                self._send(IMU.latest())
            elif path == "/imu/window":
                t0 = float(q.get("t0", 0) or 0)
                t1 = float(q.get("t1", time.time()) or time.time())
                self._send(IMU.window(t0, t1))
            elif path == "/gpio/exposure/events":
                since = int(q.get("since", 0) or 0)
                self._send(GPIO.exposure_events(since))
            elif path == "/gpio/state":
                self._send(GPIO.state())
            else:
                self._send({"ok": False, "error": "not found"}, 404)
        except Exception as e:  # noqa: BLE001
            log("error", "http", "GET handler error", path=path, err=str(e))
            self._send({"ok": False, "error": str(e)}, 500)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        b = self._body()
        try:
            if path == "/gpio/focus":
                ok = GPIO.focus(bool(b.get("hold")))
                self._send({"ok": ok, "focus_held": GPIO.focus_held()})
            elif path == "/gpio/fire":
                res = GPIO.fire(b.get("at_epoch", 0), b.get("pulse_ms", 5),
                                b.get("focus_lead_ms", 0))
                self._send(res, res.get("code", 200 if res.get("ok") else 400))
            elif path == "/gpio/interval/start":
                res = GPIO.interval_start(b.get("at_epoch", 0),
                                          b.get("period_s", 1.0),
                                          b.get("count", 0))
                self._send(res, res.get("code", 200 if res.get("ok") else 400))
            elif path == "/gpio/interval/stop":
                self._send(GPIO.interval_stop())
            elif path == "/timeprobe":
                t_rx = time.time()
                self._send({"t0": b.get("t0"), "t_rx": t_rx, "t_tx": time.time()})
            else:
                self._send({"ok": False, "error": "not found"}, 404)
        except Exception as e:  # noqa: BLE001
            log("error", "http", "POST handler error", path=path, err=str(e))
            self._send({"ok": False, "error": str(e)}, 500)


def main():
    def _bye(*_):
        log("info", "lifecycle", "piagent stopping")
        GPIO.shutdown()
        IMU.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _bye)
    signal.signal(signal.SIGTERM, _bye)

    log("info", "lifecycle", "piagent up", port=PORT, chip=GPIOCHIP,
        gpio=GPIO.available, imu=IMU.info.get("present", False))
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    try:
        srv.serve_forever()
    finally:
        GPIO.shutdown()
        IMU.shutdown()


if __name__ == "__main__":
    main()

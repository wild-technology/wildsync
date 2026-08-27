#!/usr/bin/env python3
"""piagent — the Pi-side hardware agent for a Wild Sync camera node.

One of these runs on every camera Pi alongside `ilxctl`. Where ilxctl owns the
USB/SDK path (settings, live view, image transfer), piagent owns everything
that is *not* the SDK:

    * the GPIO trigger harness  — FOCUS hold, TRIGGER firing on an absolute
      clock, and the EXPOSURE capture-edge monitor;
    * the IMU (whichever node    — via rig/imu_yb.py, sampled into a ring;
      it is plugged into)
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
  * A FOCUS hold is a LEASE, not a latch: /gpio/focus {hold:true} grants it for
    30 s (`ttl_s` overrides), repeating the call renews it, and a fire that
    relies on the hold renews it too. A watchdog releases a lapsed hold and
    logs `focus_hold_expired`. Before this, a host that died (or whose one
    unretried release call was dropped) left the body half-pressed for the
    rest of the session, AE-locked, with the per-shot FOCUS restore
    faithfully re-asserting it on every survey frame.
  * TRIGGER is pulsed with `gpioset --mode=time` in open-drain. To fire on a
    shared clock, we busy-wait to the requested epoch and stamp the instant the
    low actually starts, so the Jetson can measure real inter-camera skew.
  * The kernel edge timestamps from gpiomon are a different clock across
    libgpiod builds, so we stamp each EXPOSURE edge with wall time at read and
    keep the raw device ts only for interval math. Where the kernel stamp IS
    usable it is converted here and published as `epoch_hw`, with its own
    error bar (`hw_err_ms`) and the measured pipe-read latency (`hw_lag_ms`)
    beside it. When this node cannot produce one, `epoch_hw` is null and
    `hw_reject` says why: that edge's `epoch` is late by an unmeasured amount
    and the host must not write it as a hardware capture instant.
  * Shots are identified, not counted. Each /gpio/fire answers with a
    `fire_seq` and the `edge_seq` in force just before the TRIGGER, and each
    EXPOSURE edge carries its own index plus the `fire_seq` it belongs to, so
    the Jetson pairs a frame to the shot that produced it by identity. Pairing
    by queue position means one fire that never exposes shifts every later
    frame on that camera by a whole shot period, with nothing in the log to say
    so. All of these fields are additive: an older Jetson ignores them.
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
# Strobe sync (docs/strobe-trigger.md): Bolt VB-22 sync tip on header pin 37,
# shell on pin 39. Open-drain contact closure, NO pull-up flag (the flash
# supplies its own 2.9 V pull-up), never parked (it is our OUTPUT, not a camera
# input, and `gpio=26=ip,np` on the kernel cmdline makes unconfigured == off).
# 0 disables strobe support on this node.
BCM_STROBE = int(os.environ.get("WILDSYNC_STROBE_BCM", "26"))
# A strobe instant is scheduled relative to the shot's shared target T
# (δ ≈ 8–12 ms after T clears curtain travel plus skew). Anything more than
# this far from the trigger instant is a scheduling bug, not a plan.
STROBE_MAX_AFTER_S = 2.0

# Guard window for EXPOSURE edge debouncing. Well under the >=1 ms Sony
# specifies for the assert (measured ~13 ms), and far above the ~60 us bursts a
# ringing harness produces.
EDGE_DEBOUNCE_S = 0.001

# A held FOCUS is a lease, not a latch. The 2026-08 audit found the calibration
# hold could outlive its owner: the host's release is one HTTP call with no
# retry, so a host crash/sleep (or that one call being dropped) left the body
# half-pressed for the rest of the session - AE locked at calibration light,
# and the per-shot FOCUS restore faithfully re-asserted the stale hold on
# every survey frame. A hold now expires this long after the last
# /gpio/focus {hold:true} (repeating the call renews it; `ttl_s` in the body
# overrides, clamped to MIN..MAX) and a watchdog releases it with a
# `focus_hold_expired` log the host can grep. 30 s covers the longest
# legitimate hold today (trigger calibration, ~15-25 s per node).
FOCUS_HOLD_DEFAULT_TTL_S = 30.0
FOCUS_HOLD_MIN_TTL_S = 1.0
FOCUS_HOLD_MAX_TTL_S = 600.0

# How long a fall edge may keep its exposure "open" for rise attribution.
# Sony asserts EXPOSURE for ~13 ms at survey shutter speeds; anything still
# "open" 10 s later is a lost rise, and tagging some later spurious rise with
# the stale fire_seq made the host discard the genuine window (the 523 ms
# cam2 windows in the 2026-08 audit).
EDGE_FIRE_MAX_OPEN_S = 10.0

# What "the IMU is running slow" means, judged against what THIS device does.
# docs/HANDOFF.md's ">= 60 Hz" requirement is satisfied in that same document
# by "174 Hz measured" - which is the FRAME rate (quat + euler + inertial +
# baro). The YB-MRA02 publishes one quat and one euler per ~25 Hz device
# cycle, so its ATTITUDE cadence is ~50 Hz (probe() measures 49.8 Hz) and can
# never reach 60. The 2026-08 audit compared attitude_hz against the frame-rate
# figure, so /health returned imu_rate_low:true on a perfectly healthy unit
# from the first closed rate window onward - an alarm that is always on is an
# alarm nobody reads. Each number is now judged against its own spec: frame_hz
# against the HANDOFF figure, attitude_hz against a fraction of this device's
# own measured normal (the probe's sample_rate_hz, sanity-bounded).
IMU_MIN_FRAME_HZ = 60.0
IMU_ATTITUDE_HZ_NOMINAL = 50.0     # 2 orientation frames per 25 Hz cycle
IMU_ATTITUDE_LOW_FRAC = 0.6        # ~30 Hz here: a halved or stalled attitude
                                   # stream trips it, a healthy one never does

# Plausibility band for epoch_hw (the kernel edge stamp converted to wall
# time). `lag` = epoch - epoch_hw is NOT a stamp-quality metric: it IS the
# gpiomon pipe-read latency, measured on this hardware as a 0.09/0.32 ms
# median "with occasional excursions into the hundreds of ms under load"
# (see _monitor_loop). The 2026-08 audit capped it at 0.25 s, which cuts
# INSIDE that measured distribution: a loaded Pi threw the correct kernel
# stamp away precisely when `epoch` was worst, the host fell back to the late
# `epoch` and still wrote capture_source=gpio_edge with a clock-error-only
# bar - the exact error epoch_hw exists to remove. The corrupted-offset case
# this band was written for is handled where it can actually be MEASURED, in
# _wall_minus_mono(), whose bracket is published per edge as `hw_err_ms`.
# What is left for a bound to catch is a stamp from the WRONG CLOCK DOMAIN (a
# mis-scaled legacy gpiomon line is ~1e5 s or ~1.7e9 s out), so it sits far
# above any read latency this hardware can produce.
EDGE_HW_MAX_LAG_S = 5.0
EDGE_HW_SLOP_S = 0.005

# Floor under the per-edge `hw_err_ms`. The bracket can only resolve what the
# wall clock can: time.time() reports 1 us granularity on the dev Mac (1 ns on
# the Pis), so a bracket of exactly 0.0 means "below the tick", not "exact".
# Publishing 0.0 would hand the host a claim of a perfect conversion.
try:
    EDGE_HW_CLOCK_RES_S = float(time.get_clock_info("time").resolution)
except Exception:  # noqa: BLE001 - exotic platform; assume a 1 us tick
    EDGE_HW_CLOCK_RES_S = 1e-6

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

    def __init__(self, chip_name, lines, pull_up=True):
        """pull_up=False is for the strobe sync line: the flash supplies its
        own pull-up (measured 2.9 V open circuit), and adding ours would leak
        current into its trigger circuit for no benefit."""
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
            if pull_up:
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

    def shot(self, focus_bcm, trigger_bcm, lead_s, pulse_s, focus_after=None):
        """One frame: FOCUS leads, TRIGGER fires, both release.

        The camera only accepts a TRIGGER while FOCUS is already Low, but
        holding FOCUS down for a whole transect half-presses the body for the
        duration, which AE-locks it - auto-ISO would freeze at whatever the
        light was when the run started. Asserting FOCUS a few tens of
        milliseconds ahead of each shot satisfies the camera and still lets it
        meter every frame. Both edges are register writes in this process, so
        the lead is precise regardless of how busy the host is.

        `focus_after` is an optional callable invoked in the finally block that
        must leave FOCUS in the state its owner wants. This shot does not own
        FOCUS: a caller may legitimately be holding it (calibration does), and
        forcing it Idle here used to yank that hold away while the holder's
        bookkeeping still said "held" - after which every subsequent trigger
        pulsed with FOCUS idle, produced no exposure, and still answered ok.
        If it is absent or raises, FOCUS is forced Idle: never leave the line
        asserted on the way out, because that is a permanent half-press, which
        locks the whole property table and looks exactly like a dead camera."""
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
            restored = False
            if focus_after is not None:
                try:
                    focus_after()
                    restored = True
                except Exception as e:  # noqa: BLE001
                    log("warn", "gpio_error",
                        "FOCUS restore hook failed, forcing idle", err=str(e))
            if not restored:
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

    # An `at_epoch` is a promise about a shared clock, and fire() waits for it on
    # a CPU. Bound it in both directions so a bad promise cannot pin a core or
    # fire a frame nobody is waiting for any more:
    #  * the future bound used to be 10 s "because the host times out at ~10 s",
    #    but the host's fire timeout is 2 s now: a node clock 2-10 s behind
    #    accepted fires the host had already abandoned, held the fire lock for
    #    seconds, then fired an orphan frame onto the card with no command
    #    record - every shot - and the host saw only "stopped answering fires".
    #    The longest LEGITIMATE lead a node can see is SYNC_LEAD_S (0.30 s)
    #    plus the host's node-clock offset compensation, which is clamped to
    #    5 s on its side; 5.5 s adds margin for HTTP transit. Anything further
    #    out is a clock disagreement, not a schedule (contract: 0.3-5.5 s
    #    ahead must always be accepted).
    #  * an at_epoch already well in the past is a stale schedule (a queued
    #    request, or this clock running ahead). Firing it immediately puts an
    #    unplanned frame on the card and reports late_ms in the thousands.
    # Both are answered with an explicit error the Jetson can count, which is
    # also the only way a node clock this wrong ever becomes visible.
    FIRE_MAX_FUTURE_S = 5.5
    FIRE_MAX_PAST_S = 2.0

    def __init__(self):
        self.chip = GPIOCHIP
        self._lock = threading.RLock()
        self._focus_proc = None          # long-lived gpioset holding FOCUS low
        self._parkers = {}               # bcm -> Popen holding the line idle-high
        self._focus_direct = False       # FOCUS state when using the gpiod path
        # FOCUS hold lease (see FOCUS_HOLD_* above): a hold that outlives its
        # owner is a permanent half-press, so every hold carries an expiry the
        # watchdog below enforces. Monotonic clock: a chrony step while held
        # must not stretch or collapse the lease.
        self._focus_held_since = None    # monotonic instant the hold began
        self._focus_hold_expiry = None   # monotonic deadline; renewed per hold
        self._focus_hold_ttl = None      # lease length this hold was granted
        # fire() drives shared lines and must never interleave with another
        # fire; a dedicated lock keeps that serialisation off `_lock`, which the
        # edge monitor and /health take, so a fire waiting on its at_epoch can
        # never stall the health endpoint or delay an EXPOSURE edge.
        self._fire_lock = threading.Lock()
        self._fire_seq = 0               # identity of each fire, monotonic
        self._pending_fire = None        # (fire_seq, deadline) awaiting its edge
        self._edge_fire = None           # fire_seq of the exposure now open
        self._interval_stop = None
        self._interval_thread = None
        self._interval_state = {"running": False, "fired": 0, "target": 0,
                                "period_s": 0.0, "last_late_ms": None,
                                # Why a schedule ended early (FOCUS gone), so
                                # "running:false, fired:1 of 20" is readable.
                                "error": None}
        self._edges = deque(maxlen=20000)   # (epoch, edge, raw_ts, seq, hw, fire_seq)
        self._edge_seq = 0
        self._edge_fire_at = None           # wall epoch the open exposure fell
        self._last_edge = {}                # edge -> last accepted timestamp
        self._bounced = 0                   # edges dropped as ring/bounce
        self._hw_rejects = 0                # epoch_hw stamps outside the band
        self._hw_reject_why = {}            # reason -> count (see _edge_hw)
        self._hw_lag_ms_max = 0.0           # worst pipe-read latency seen
        self._hw_lag_ms_last = None         # None until one edge is stamped
        self._mon_proc = None
        self._mon_thread = None
        self._mon_run = False
        self.available = _HAVE_GPIO
        # Prefer the in-process driver: it removes the per-shot subprocess spawn
        # that made trigger latency platform-dependent, and its open-drain idle
        # doubles as the safe park for both camera inputs.
        self.driver = _LineDriver(self.chip, self.PARKED_INPUTS)
        # Strobe: claimed lazily on the first scheduled strobe, so a node with
        # nothing on pin 37 never touches the line at all. Open-drain with NO
        # pull-up (the flash supplies 2.9 V of its own) and never parked.
        self._strobe = None              # _LineDriver once claimed, or False
        self._strobe_fires = 0
        self._strobe_last = None         # epoch of the last pulse
        self._strobe_err = None
        # The FOCUS-hold watchdog runs regardless of `available`: any path
        # that can assert FOCUS (gpiod or gpioset) must be covered, and the
        # thread is idle unless a hold is actually live.
        self._watchdog_run = True
        threading.Thread(target=self._focus_watchdog_loop, daemon=True).start()
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

    def focus_held_s(self):
        """Seconds the current hold has been live, or None when not held.

        The host never had a way to SEE a stale hold (a failed release during
        calibration left the body half-pressed with nothing reporting it);
        this plus the lease watchdog is that visibility."""
        with self._lock:
            if self.focus_held() and self._focus_held_since is not None:
                return round(time.monotonic() - self._focus_held_since, 1)
            return None

    def _focus_lease(self, hold, ttl_s):
        """Set/renew (hold) or clear (release) the hold lease. Under _lock.

        `ttl_s` None on a RENEWAL means "the length this hold was granted", not
        the default: a keepalive must never silently stretch a lease its owner
        deliberately asked to be short."""
        if hold:
            try:
                ttl = float(ttl_s) if ttl_s is not None \
                    else (self._focus_hold_ttl or FOCUS_HOLD_DEFAULT_TTL_S)
            except (TypeError, ValueError):
                ttl = FOCUS_HOLD_DEFAULT_TTL_S
            ttl = max(FOCUS_HOLD_MIN_TTL_S, min(ttl, FOCUS_HOLD_MAX_TTL_S))
            now = time.monotonic()
            if self._focus_held_since is None:
                self._focus_held_since = now
            self._focus_hold_ttl = ttl
            self._focus_hold_expiry = now + ttl
        else:
            self._focus_held_since = None
            self._focus_hold_expiry = None
            self._focus_hold_ttl = None

    def _focus_watchdog_loop(self):
        while self._watchdog_run:
            try:
                self._focus_watchdog_tick()
            except Exception as e:  # noqa: BLE001 - the watchdog must survive
                log("warn", "gpio_focus", "focus watchdog error", err=str(e))
            time.sleep(0.5)

    def _focus_watchdog_tick(self):
        """Release a held FOCUS whose lease has lapsed; True if it acted.

        This is the recovery path for a dead host: without it a hold whose
        release call never arrived kept the body half-pressed (AE locked, the
        per-shot restore re-asserting it every frame) until someone pulled the
        connector. `_lock` is an RLock, so releasing from in here is safe."""
        with self._lock:
            if not self.focus_held():
                return False
            exp = self._focus_hold_expiry
            if exp is None or time.monotonic() < exp:
                return False
            held = self.focus_held_s()
            log("warn", "focus_hold_expired",
                "FOCUS hold lease lapsed without a renewal - releasing "
                "(host dead or its release call was lost?)",
                held_s=held)
            self.focus(False)
            if self.focus_held():
                # Release failed (line write refused). Retry in 5 s rather
                # than every tick, so a dead line does not flood the log.
                self._focus_hold_expiry = time.monotonic() + 5.0
            return True

    def focus(self, hold, ttl_s=None):
        with self._lock:
            if self.driver.ok:
                ok = self.driver.set(
                    BCM_FOCUS,
                    _LineDriver.ASSERT if hold else _LineDriver.IDLE)
                if ok:
                    self._focus_direct = bool(hold)
                    self._focus_lease(hold, ttl_s)
                    log("info", "gpio_focus",
                        "FOCUS held low" if hold else "FOCUS released")
                return ok
            if hold:
                if self.focus_held():
                    # Idempotent hold doubles as the keepalive: renew the lease
                    # rather than spawning a second holder.
                    self._focus_lease(True, ttl_s)
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
                self._focus_lease(True, ttl_s)
                log("info", "gpio_focus", "FOCUS held low")
                return True
            else:
                self._release_focus()
                return True

    def _release_focus(self):
        p = self._focus_proc
        self._focus_proc = None
        # The lease dies with the hold, whichever path releases it. This used
        # to clear the timestamps but leave _focus_hold_ttl set, so on the
        # gpioset path (no python3-libgpiod) the NEXT hold that omitted ttl_s
        # inherited the previous one's length: hold ttl_s=600, release, then a
        # plain hold, and the body sat half-pressed for ten minutes where the
        # caller and PROTOCOL.md both say thirty seconds. One definition of
        # "the lease dies", in _focus_lease.
        self._focus_lease(False, None)
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
    def _focus_restore(self):
        """Leave FOCUS where its owner wants it, after a shot borrowed the line.

        Read live under `_lock` rather than snapshotted before the shot, so a
        /gpio/focus release that lands mid-shot wins: re-asserting a hold the
        operator has just dropped would leave the body half-pressed with nobody
        holding it. Either way `_focus_direct` stays true to the wire, which is
        what focus_held() - and every guard built on it - depends on."""
        with self._lock:
            self.driver.set(BCM_FOCUS, _LineDriver.ASSERT if self._focus_direct
                            else _LineDriver.IDLE)

    @classmethod
    def _epoch_fault(cls, at_epoch):
        """Error dict if `at_epoch` is not a schedule this node can honour.

        See FIRE_MAX_* above: an out-of-range at_epoch is a clock disagreement,
        not a schedule, so say so instead of spinning to it. Returns (epoch,
        fault) with the epoch coerced to float; 0/absent means "now" as before."""
        try:
            at_epoch = float(at_epoch or 0)
        except (TypeError, ValueError):
            return 0.0, None
        if at_epoch <= 0:
            return 0.0, None
        skew = at_epoch - time.time()
        if -cls.FIRE_MAX_PAST_S <= skew <= cls.FIRE_MAX_FUTURE_S:
            return at_epoch, None
        log("error", "gpio_time",
            "refusing implausible at_epoch - check this node's clock",
            skew_s=round(skew, 3), at_epoch=round(at_epoch, 3))
        return at_epoch, {
            "ok": False, "code": 400,
            "error": ("at_epoch is %+.3f s from this node's clock (limits "
                      "+%.0f/-%.0f s) - node clock or schedule is wrong"
                      % (skew, cls.FIRE_MAX_FUTURE_S, cls.FIRE_MAX_PAST_S)),
            "clock_skew_s": round(skew, 3), "node_epoch": time.time()}

    def fire(self, at_epoch, pulse_ms, focus_lead_ms=0, strobe_at_epoch=0,
             strobe_pulse_ms=5):
        res = self._fire(at_epoch, pulse_ms, focus_lead_ms, strobe_at_epoch,
                         strobe_pulse_ms)
        # node_epoch on EVERY answer, refusals included: a fire refused for a
        # bad at_epoch (or abandoned by the host's 2 s timeout) is exactly the
        # moment the host needs to see this node's clock to diagnose the
        # disagreement, and only the success path used to carry it.
        if isinstance(res, dict):
            res.setdefault("node_epoch", time.time())
        return res

    def _fire(self, at_epoch, pulse_ms, focus_lead_ms=0, strobe_at_epoch=0,
              strobe_pulse_ms=5):
        if not self.available:
            return {"ok": False, "error": "no gpio on this node"}
        focus_lead_ms = max(0, min(int(focus_lead_ms or 0), 500))
        # A caller that asks for its own FOCUS lead does not need FOCUS to be
        # held already - that is the point: per-shot FOCUS keeps auto-exposure
        # live, where a continuous hold would AE-lock the body for the run.
        if not focus_lead_ms:
            if not self.focus_held():
                return {"ok": False, "error": "FOCUS not held", "code": 409}
            # A fire that RELIES on the held FOCUS is itself proof the hold's
            # owner is alive, so it renews the lease. Without this the 30 s
            # watchdog would cut a long trigger calibration short (the host
            # holds FOCUS for the whole sample loop and sends no keepalive,
            # and /api/calibrate does not clamp `samples`), and every later
            # sample would come back "FOCUS not held". A SURVEY fire brings
            # its own focus_lead_ms and therefore does NOT renew - which is
            # the case that must stay bounded, since the per-shot FOCUS
            # restore is exactly what kept a stale hold alive all run.
            with self._lock:
                if self.focus_held():
                    self._focus_lease(True, None)
        pulse_ms = max(1, min(int(pulse_ms), 200))
        at_epoch, fault = self._epoch_fault(at_epoch)
        if fault:
            return fault
        # Strobe schedule sanity: δ is measured in milliseconds after the shot's
        # target instant (docs/strobe-trigger.md §4.1). A strobe before the
        # trigger, or seconds after it, is a scheduling bug the Jetson needs to
        # hear about, not a plan to honour.
        try:
            strobe_at = float(strobe_at_epoch or 0)
        except (TypeError, ValueError):
            strobe_at = 0.0
        if strobe_at:
            base = at_epoch or time.time()
            delta = strobe_at - base
            if not (0.0 < delta <= STROBE_MAX_AFTER_S):
                return {"ok": False, "code": 400,
                        "error": "strobe_at_epoch sits %+.3f s from at_epoch - "
                                 "must be 0..%.1f s after it" %
                                 (delta, STROBE_MAX_AFTER_S)}
            # Claim the line now, while we are still waiting for the shot's
            # instant, so the first-ever strobe does not pay the chip-open
            # latency inside its timing window.
            self._strobe_driver()
        # One fire at a time. Two overlapping fires share FOCUS and TRIGGER, so
        # they interleave into a single malformed pulse train that drops frames,
        # and the second would steal the first's FOCUS. Queueing the second is no
        # better - by the time it ran its at_epoch would be gone - so refuse it
        # with an explicit busy error the Jetson can count against its own shot
        # log. Note this is NOT `_lock`: the edge monitor and /health take that,
        # and a fire may sit here for seconds waiting for its instant.
        if not self._fire_lock.acquire(blocking=False):
            log("warn", "gpio_busy", "fire rejected: another fire in flight")
            return {"ok": False, "code": 409, "busy": True,
                    "error": "another fire is in flight on this node"}
        try:
            return self._fire_locked(at_epoch, pulse_ms, focus_lead_ms,
                                     strobe_at, strobe_pulse_ms)
        finally:
            self._fire_lock.release()

    # ---- strobe -------------------------------------------------------------
    def _strobe_driver(self):
        """The strobe line, claimed lazily; False when unavailable.

        Lazy so a node with nothing on pin 37 never touches the pad at all —
        `gpio=26=ip,np` on the kernel cmdline keeps it high-Z until the first
        scheduled strobe actually claims it."""
        with self._lock:
            if self._strobe is not None:
                return self._strobe
            if not BCM_STROBE:
                self._strobe, self._strobe_err = False, "strobe disabled " \
                    "(WILDSYNC_STROBE_BCM=0)"
                return False
            if not self.available:
                self._strobe, self._strobe_err = False, "no gpio on this node"
                return False
            drv = _LineDriver(self.chip, (BCM_STROBE,), pull_up=False)
            if not drv.ok:
                self._strobe, self._strobe_err = False, \
                    "could not claim BCM%d (gpiod path required)" % BCM_STROBE
                log("warn", "strobe", self._strobe_err)
                return False
            self._strobe, self._strobe_err = drv, None
            log("info", "strobe", "strobe line claimed open-drain, no pull",
                bcm=BCM_STROBE)
            return drv

    def strobe_only(self, at_epoch, pulse_ms=5):
        """Pulse the strobe with NO camera fire — the survey's light must not
        depend on this node's camera being claimable. The 2026-08-16 card
        fault took cam1's camera out while its Pi (the strobe host) stayed
        perfectly healthy; riding the strobe exclusively on /gpio/fire would
        have darkened every cam2 frame for the rest of that survey."""
        if not self.available:
            return {"ok": False, "error": "no gpio on this node"}
        try:
            at = float(at_epoch or 0)
        except (TypeError, ValueError):
            at = 0.0
        if at <= 0:
            return {"ok": False, "code": 400,
                    "error": "at_epoch is required for a scheduled strobe"}
        skew = at - time.time()
        if not (-0.05 <= skew <= self.FIRE_MAX_FUTURE_S):
            return {"ok": False, "code": 400,
                    "error": "at_epoch is %+.3f s from this node's clock"
                             % skew}
        if self._strobe_driver() is False:
            return {"ok": False,
                    "error": self._strobe_err or "strobe unavailable"}
        r = self._strobe_pulse(at, pulse_ms)
        if "strobe_error" in r:
            return {"ok": False, "error": r["strobe_error"]}
        r["ok"] = True
        return r

    def _strobe_pulse(self, strobe_at, pulse_ms):
        """Busy-wait to the strobe instant and close the sync contact.

        Runs inside the fire lock, after the TRIGGER pulse: δ ≈ 8–12 ms after
        the shared target T, which is ~30 ms after this node's own trigger
        (fired at T − ~22 ms latency), so the wait is short and exclusive."""
        drv = self._strobe_driver()
        if drv is False:
            return {"strobe_error": self._strobe_err or "strobe unavailable"}
        pulse_ms = max(1, min(int(pulse_ms or 5), 100))
        _LineDriver._spin_until(strobe_at)
        t = drv.pulse(BCM_STROBE, pulse_ms / 1000.0)
        if t is None:
            return {"strobe_error": "strobe line write failed"}
        with self._lock:
            self._strobe_fires += 1
            self._strobe_last = t
        late = (t - strobe_at) * 1000.0
        if late > 5.0:
            log("warn", "strobe_late", "strobe pulsed late",
                late_ms=round(late, 2))
        return {"strobe_epoch": t, "strobe_late_ms": round(late, 2)}

    def _fire_locked(self, at_epoch, pulse_ms, focus_lead_ms,
                     strobe_at=0.0, strobe_pulse_ms=5):
        # Busy-wait the final approach so scheduling jitter does not smear the
        # fire time; sleep the coarse part to stay off the CPU.
        # Wake early enough to place the FOCUS lead before the target instant,
        # so it is the TRIGGER that lands on time, not the start of the sequence.
        lead_s = focus_lead_ms / 1000.0
        if at_epoch > 0:
            while True:
                dt = (at_epoch - lead_s) - time.time()
                if dt <= 0:
                    break
                if dt > 0.02:
                    time.sleep(dt - 0.015)
                # else spin
        requested = at_epoch if at_epoch else time.time()
        # Identity for this fire, and the edge counter as it stands immediately
        # before the TRIGGER. Every EXPOSURE edge carries a monotonic index, so
        # `edge_seq` lets the Jetson bound the answer by construction: this
        # fire's exposure is the first fall edge with i > edge_seq. Pairing by
        # queue position instead silently shifts every later frame by one shot
        # the first time a fire produces no edge.
        with self._lock:
            self._fire_seq += 1
            seq = self._fire_seq
            edge_seq = self._edge_seq
            # Claimed by the next fall edge (see _monitor_loop). The deadline
            # stops a fire that never exposed from adopting an unrelated edge
            # minutes later; trigger->exposure measures ~22 ms here.
            self._pending_fire = (seq, time.time() + 1.0)
        if self.driver.ok:
            # Two register writes. `actual` is stamped after the line is already
            # low, so it is the real assert instant rather than the moment we
            # decided to assert - the subprocess path could not tell the two
            # apart, and the gap between them differed by ~9.6 ms across Pi
            # models.
            if focus_lead_ms:
                actual = self.driver.shot(BCM_FOCUS, BCM_TRIGGER, lead_s,
                                          pulse_ms / 1000.0,
                                          focus_after=self._focus_restore)
            else:
                actual = self.driver.pulse(BCM_TRIGGER, pulse_ms / 1000.0)
            if actual is None:
                self._unclaim(seq)
                return {"ok": False, "error": "trigger line unavailable",
                        "fire_seq": seq, "edge_seq": edge_seq}
            extra = self._strobe_pulse(strobe_at, strobe_pulse_ms) \
                if strobe_at else {}
            res = self._fire_result(seq, edge_seq, requested, actual, pulse_ms,
                                    "gpiod")
            res.update(extra)
            return res
        # Fallback path: no python3-libgpiod, so every edge is a subprocess.
        held = self.focus_held()
        if focus_lead_ms and not held:
            # The gpiod path asserts FOCUS itself; this path used to ignore
            # focus_lead_ms entirely and pulse TRIGGER with FOCUS idle, which the
            # body simply ignores - a silent no-op frame reported as ok:true.
            # Spawning the holder costs ~50 ms, so the lead is approximate here;
            # a late frame beats a missing one.
            if not self.focus(True):
                self._unclaim(seq)
                return {"ok": False, "error": "could not assert FOCUS",
                        "fire_seq": seq, "edge_seq": edge_seq}
            # The holder spawn (~50 ms) has already eaten into the lead, so wait
            # only what is left of it - the TRIGGER should still land on
            # at_epoch, not 50 ms behind the other camera. Never leave the body
            # less than 40 ms of half-press: that is the shortest lead measured
            # to produce an exposure at all.
            rest = (at_epoch - time.time()) if at_epoch > 0 else lead_s
            time.sleep(max(0.040, min(lead_s, rest)))
        try:
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
                self._unclaim(seq)
                return {"ok": False, "error": str(e), "fire_seq": seq,
                        "edge_seq": edge_seq}
            finally_parked = self._park(BCM_TRIGGER)
            if not finally_parked:
                log("error", "gpio_error",
                    "TRIGGER could not be re-parked idle-high - unplug the "
                    "harness if the camera stops responding", bcm=BCM_TRIGGER)
        finally:
            if focus_lead_ms and not held:
                # Release only what this shot asserted, and never leave FOCUS
                # low on the way out: a permanent half-press locks the body.
                self.focus(False)
        extra = self._strobe_pulse(strobe_at, strobe_pulse_ms) \
            if strobe_at else {}
        res = self._fire_result(seq, edge_seq, requested, actual, pulse_ms,
                                "gpioset")
        res.update(extra)
        return res

    def _unclaim(self, seq):
        """A fire that never pulsed must not adopt the next edge on the line."""
        with self._lock:
            if self._pending_fire and self._pending_fire[0] == seq:
                self._pending_fire = None

    @staticmethod
    def _fire_result(seq, edge_seq, requested, actual, pulse_ms, path):
        late_ms = (actual - requested) * 1000.0
        if abs(late_ms) > 50:
            log("warn", "gpio_late", "trigger fired late",
                late_ms=round(late_ms, 1))
        return {"ok": True, "requested_epoch": requested,
                "actual_epoch": actual, "late_ms": round(late_ms, 2),
                "pulse_ms": pulse_ms, "path": path,
                # Identity, so a frame can be paired to the fire that made it
                # rather than to whatever is next in a queue.
                "fire_seq": seq, "edge_seq": edge_seq,
                # This node's clock, read as the answer is built. The Jetson can
                # difference it against its own to bound node clock offset from
                # every shot, without a separate /timeprobe round trip.
                "node_epoch": time.time()}

    # ---- interval ---------------------------------------------------------
    def interval_start(self, at_epoch, period_s, count):
        with self._lock:
            if self._interval_state["running"]:
                return {"ok": False, "error": "interval already running"}
            if not self.focus_held():
                return {"ok": False, "error": "FOCUS not held", "code": 409}
            # Same clock sanity as fire(): the loop's first frame targets
            # `at_epoch` directly, so an implausible start would be rejected
            # frame by frame instead of once, here, where it can be answered.
            at_epoch, fault = self._epoch_fault(at_epoch)
            if fault:
                return fault
            period_s = max(0.05, float(period_s))
            # The loop is the hold's owner for as long as it runs, and the
            # watchdog has to be told so. Its own fires renew the lease (they
            # carry focus_lead_ms=0), but only once per period: at period_s
            # above the 30 s default the lease lapsed in the gap, the watchdog
            # released FOCUS, and every frame after the first came back
            # "FOCUS not held" while /gpio/state still said running:true with
            # fired:1. Grant a lease that spans the gap; _interval_loop
            # re-arms it on every tick, so it also outlives a long period,
            # and it still lapses ~2.5 periods after the loop stops proving
            # it is alive.
            self._focus_lease(True, max(FOCUS_HOLD_DEFAULT_TTL_S,
                                        2.5 * period_s))
            self._interval_stop = threading.Event()
            self._interval_state.update(running=True, fired=0,
                                        target=int(count), period_s=period_s,
                                        error=None)
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
                # A lost FOCUS is not a bad frame, it is the end of the run:
                # every remaining frame fails identically, so the loop used to
                # log the same line `count` times and keep claiming
                # running:true. Stop and say why. (`busy` is a genuine
                # per-frame collision and does not end the schedule.)
                if res.get("code") == 409 and not res.get("busy") \
                        and not self.focus_held():
                    with self._lock:
                        self._interval_state["error"] = res.get("error")
                    log("warn", "interval", "interval stopped: FOCUS gone",
                        fired=self._interval_state["fired"])
                    break
            k += 1
            # Wait for the next slot without holding the lock.
            nxt = start + k * period
            while not stop.is_set():
                dt = nxt - time.time()
                if dt <= 0:
                    break
                stop.wait(min(dt, 0.25))
                # Keepalive. The loop, not the caller, is what keeps FOCUS
                # legitimate while a schedule runs; without this tick a period
                # longer than the lease expires it mid-gap and the watchdog
                # takes the line out from under the next frame. Renew with
                # ttl_s=None so it re-arms for the length interval_start
                # granted, never longer.
                with self._lock:
                    if self.focus_held():
                        self._focus_lease(True, None)
        with self._lock:
            self._interval_state["running"] = False
            # The long lease existed only to span this schedule's gaps. With
            # the loop gone nothing is proving the hold's owner is still
            # alive, so bring it back to the default bound - shortening only,
            # never extending a lease its owner asked to be shorter.
            if self._focus_hold_expiry is not None:
                self._focus_hold_ttl = min(self._focus_hold_ttl or
                                           FOCUS_HOLD_DEFAULT_TTL_S,
                                           FOCUS_HOLD_DEFAULT_TTL_S)
                self._focus_hold_expiry = min(
                    self._focus_hold_expiry,
                    time.monotonic() + FOCUS_HOLD_DEFAULT_TTL_S)
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
        # Seconds and nanoseconds are asked for as SEPARATE fields and joined
        # here. The obvious "%s.%n" is a trap: gpiomon prints %n as a plain
        # integer with no zero padding, so an edge 41,993,474 ns into its second
        # comes out as "2924.41993474" and float() reads it as 0.42 s instead of
        # 0.042 s. Every edge landing in the first 100 ms of a second is wrong,
        # by up to 0.9 s - measured 11% of edges on this rig. That corrupts the
        # ring's own debounce reference below, which is what decides whether a
        # ringing harness's repeats are swallowed or handed out as real
        # exposures.
        fmt = "%e %s %n"
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
                    if len(parts) > 2:
                        try:
                            # CLOCK_MONOTONIC on this kernel/libgpiod: seconds
                            # since boot plus a separate nanosecond field.
                            raw = int(parts[1]) + int(parts[2]) / 1e9
                        except ValueError:
                            raw = None
                    elif len(parts) > 1:
                        # Tolerate the older single-field form during a rolling
                        # deploy, accepting that it may be mis-scaled.
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
                    # The kernel edge timestamp is CLOCK_MONOTONIC (seconds since
                    # this node's boot), so it is meaningless to another machine
                    # on its own. Convert it to wall time HERE, where the offset
                    # can be sampled in the same breath as the edge was read, and
                    # publish that as `epoch_hw`. It is the same instant as
                    # `epoch` but measured by the kernel at the interrupt rather
                    # than by Python after the pipe read - measured median 0.09 ms
                    # of read latency on cam1 and 0.32 ms on cam2, with occasional
                    # excursions into the hundreds of ms under load. For a stereo
                    # pair that read latency is pure, uncorrelated skew error, so
                    # the Jetson should prefer epoch_hw when pairing frames.
                    # Note the direction: a BIG lag is exactly when epoch_hw is
                    # worth the most, so the stamp is kept and the measured lag
                    # is published beside it (`hw_lag_ms`) rather than used as
                    # grounds to drop it - see _edge_hw.
                    hw = self._edge_hw(raw, epoch)
                    # The reference carries its CLOCK DOMAIN. raw is kernel
                    # CLOCK_MONOTONIC (~1e5 s since boot); epoch is wall time
                    # (~1.7e9 s). One unparseable line used to store a wall
                    # reference, after which every genuine monotonic edge
                    # compared as (1e5 - 1.7e9) < guard — a bounce — forever:
                    # edges_seen froze and every frame silently fell back to
                    # EXIF. Comparing across domains is meaningless; only a
                    # same-domain reference may debounce an edge.
                    domain = "raw" if raw is not None else "epoch"
                    ref = raw if raw is not None else epoch
                    last = self._last_edge.get(edge)
                    if last is not None and last[0] == domain \
                            and (ref - last[1]) < EDGE_DEBOUNCE_S:
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
                    self._last_edge[edge] = (domain, ref)
                    self._record_edge(epoch, edge, raw, hw)
                    if edge == "fall":
                        log("debug", "exposure_edge", "capture edge",
                            epoch=round(epoch, 4))
            except Exception as e:  # noqa: BLE001 - monitor must never die silently
                log("warn", "gpio_monitor", "monitor read error", err=str(e))
            # A fresh gpiomon is a fresh timestamp stream: a reference carried
            # across the respawn could swallow the first genuine edge of the
            # new child (or poison the domain check above).
            self._last_edge = {}
            if self._mon_run:
                time.sleep(1)      # brief backoff before respawn

    @staticmethod
    def _wall_minus_mono():
        """(offset, bracket) — the wall-minus-monotonic offset and its bound.

        The old single `time.time() - time.monotonic()` pair silently shifted
        epoch_hw by the length of any preemption that landed between the two
        reads (a GIL handoff to the IMU parser, a /health JSON dump) - and
        epoch_hw is the preferred capture instant, so the shift went straight
        into the stereo pairing. Read the pair twice, bracketing the monotonic
        read with wall reads so the preemption is MEASURABLE, and keep the
        sample with the smallest bracket.

        The bracket is returned, not discarded: with midpoint pairing the
        offset's own error is at most bracket/2, and that - not the pipe-read
        latency - is the honest error bar on the converted stamp. Judging the
        stamp by the read latency instead is what made the 0.25 s band throw
        away good stamps under load."""
        best, best_gap = None, None
        for _ in range(2):
            w1 = time.time()
            m = time.monotonic()
            w2 = time.time()
            gap = w2 - w1
            if best_gap is None or gap < best_gap:
                # Midpoint pairing halves whatever preemption remains.
                best_gap, best = gap, (w1 + w2) / 2.0 - m
        return best, best_gap

    def _hw_reject(self, why, lag):
        """Count and log one edge published without a hardware instant."""
        with self._lock:
            self._hw_rejects += 1
            self._hw_reject_why[why] = self._hw_reject_why.get(why, 0) + 1
            n = self._hw_rejects
        if n == 1 or n % 100 == 0:
            log("warn", "gpio_monitor",
                "no usable kernel edge stamp (%s) - publishing the edge "
                "without epoch_hw; its `epoch` carries an unmeasured pipe-"
                "read latency, so the host must not write it as a hardware "
                "capture instant" % why, count=n,
                lag_ms=None if lag is None else round(lag * 1000, 2))
        return {"hw": None, "lag_ms": None, "err_ms": None, "reject": why}

    def _edge_hw(self, raw, epoch):
        """Kernel edge stamp -> {hw, lag_ms, err_ms, reject}, always a dict.

        `hw` is the kernel's interrupt instant in wall time - the value the
        host should use as the capture instant - or None when this node could
        not produce one. The rest is what the host needs to be HONEST about
        which it got:

          lag_ms  epoch - hw, i.e. the measured gpiomon pipe-read latency.
                  Diagnostic only when hw is published; it is also the load
                  excursion signal the fleet view can alarm on.
          err_ms  the node's own bound on hw (half the wall bracket used to
                  convert it, floored at half a clock tick). Add this to the
                  fleet clock error, do not ignore it.
          reject  None normally; a short reason when the stamp was refused.
                  hw is then None and `epoch` is late by an UNMEASURED amount
                  (hundreds of ms under load), so the host must not write that
                  edge as a hardware capture instant with a clock-error-only
                  bar - see docs/PROTOCOL.md.

        A large positive lag is a late READ, not a bad stamp: rejecting on it
        discards the good value and keeps the bad one. Only a wrong clock
        domain (EDGE_HW_MAX_LAG_S out) or a stamp that claims to postdate the
        read (negative beyond EDGE_HW_SLOP_S) is refused."""
        if raw is None:
            # gpiomon printed no parseable timestamp. Not a bad stamp - no
            # stamp at all - but the edge is just as hw-less downstream, so
            # it is counted with the rest: a build that never prints one
            # degrades EVERY edge to a software instant, silently.
            return self._hw_reject("no_stamp", None)
        off, gap = self._wall_minus_mono()
        hw = raw + off
        lag = epoch - hw
        if lag < -EDGE_HW_SLOP_S:
            return self._hw_reject("stamp_ahead", lag)
        if lag > EDGE_HW_MAX_LAG_S:
            return self._hw_reject("domain", lag)
        lag_ms = lag * 1000.0
        with self._lock:
            if lag_ms > self._hw_lag_ms_max:
                self._hw_lag_ms_max = lag_ms
            self._hw_lag_ms_last = lag_ms
        return {"hw": hw, "lag_ms": round(lag_ms, 3),
                "err_ms": round(max(gap, EDGE_HW_CLOCK_RES_S) * 500.0, 5),
                "reject": None}

    def _record_edge(self, epoch, edge, raw, hw=None):
        """Ring one accepted edge, attributed to the fire that caused it.

        `hw` is _edge_hw()'s dict (or None when the caller has no stamp at
        all); it is carried through to exposure_events verbatim so the host
        sees the same verdict this node reached.

        The identity matters as much as the instant: pairing a frame to a shot
        by queue position means one fire that produces no edge shifts every
        later frame on that camera by exactly one shot period, still labelled as
        a hardware capture. A fall claims the pending fire exactly once - the
        first fall after a TRIGGER is that trigger's exposure - and the matching
        rise carries the same id, since it closes the same exposure. An edge with
        no pending fire (a USB release, the body's own timer, a fire that already
        got its edge) reports null rather than borrowing another shot's
        identity. Best-effort by design; `edge_seq` from /gpio/fire is the exact
        bound."""
        with self._lock:
            fs = None
            if edge == "fall":
                pend = self._pending_fire
                if pend and time.time() <= pend[1]:
                    fs = self._edge_fire = pend[0]
                    self._edge_fire_at = epoch
                    self._pending_fire = None
                else:
                    self._edge_fire = None
                    self._edge_fire_at = None
            else:
                # An exposure has exactly ONE rise. The open fire_seq used to
                # persist until the next fall, so a spurious/mislabelled rise
                # minutes later still carried the stale id - and the host's
                # identity match then took THAT rise and discarded the genuine
                # window (the 523 ms cam2 windows). Hand the id to the first
                # rise only, bounded in time, and close the window either way.
                open_at = self._edge_fire_at
                if open_at is not None and \
                        (epoch - open_at) <= EDGE_FIRE_MAX_OPEN_S:
                    fs = self._edge_fire
                self._edge_fire = None
                self._edge_fire_at = None
            self._edge_seq += 1
            self._edges.append((epoch, edge, raw, self._edge_seq, hw, fs))
            return self._edge_seq

    def monitor_running(self):
        return self._mon_proc is not None and self._mon_proc.poll() is None

    def exposure_events(self, since):
        with self._lock:
            evs = [e for e in self._edges if e[3] > since]
            nxt = self._edge_seq
        # `fire_seq` is additive: an older Jetson ignores it and still pairs on
        # `i`/`epoch_hw` exactly as before. So are the hw_* fields, but the
        # host has to be able to tell "this node does not publish them" from
        # "this node published null" - the first means fall back to `epoch`
        # as before, the second means this edge has NO hardware instant and
        # must not be written as one. `hw_meta` is that version marker: an
        # older piagent omits it, and every edge from a node that sets it
        # carries all four hw_* keys.
        return {"next": nxt, "hw_meta": 1,
                "events": [{"i": s, "edge": ed, "epoch": ep, "raw_ts": raw,
                            "epoch_hw": (hw or {}).get("hw"),
                            "hw_lag_ms": (hw or {}).get("lag_ms"),
                            "hw_err_ms": (hw or {}).get("err_ms"),
                            "hw_reject": (hw or {}).get("reject"),
                            "fire_seq": fs}
                           for (ep, ed, raw, s, hw, fs) in evs]}

    def state(self):
        parked = self.parked()
        return {"chip": self.chip, "available": self.available,
                "focus_held": self.focus_held(),
                # How long the hold has been live: a hold that outlived its
                # owner used to be invisible to the host; now rigd can alarm on
                # a hold running outside calibration.
                "focus_held_s": self.focus_held_s(),
                "monitor_running": self.monitor_running(),
                "interval": self.interval_status(),
                "edges_seen": self._edge_seq,
                # Edges published WITHOUT epoch_hw - no parseable kernel
                # stamp, or one outside the domain band (see _edge_hw). A
                # bare count said
                # nothing about WHY, and nothing at all about the load that
                # made `epoch` late in the first place - so a node quietly
                # degrading to software stamps looked identical to a healthy
                # one. The reasons say which fault it is, and the lag figures
                # are the pipe-read excursion itself, in ms, for the fleet
                # view to alarm on before it reaches the capture instant.
                "edges_hw_rejected": self._hw_rejects,
                "edges_hw_reject_reasons": dict(self._hw_reject_why),
                "edge_hw_lag_ms_max": (round(self._hw_lag_ms_max, 3)
                                       if self._hw_lag_ms_last is not None
                                       else None),
                "edge_hw_lag_ms_last": (round(self._hw_lag_ms_last, 3)
                                        if self._hw_lag_ms_last is not None
                                        else None),
                # Fires dispatched vs edges seen: if these stop tracking each
                # other the harness is triggering without exposing (or the body
                # is exposing without us), which no other field reveals.
                "fires": self._fire_seq,
                # An unparked camera-input line can hold the body in a permanent
                # half-press, so the fleet view needs to see this, not just the
                # edge count. False here is an operator-visible fault.
                "inputs_parked": parked,
                "trigger_path": "gpiod" if self.driver.ok else "gpioset",
                "edges_bounced": self._bounced,
                # Strobe line (docs/strobe-trigger.md): claimed lazily on the
                # first scheduled strobe, so claimed=False on a node with no
                # flash is the normal, safe state, not a fault.
                "strobe": {"bcm": BCM_STROBE,
                           "claimed": bool(self._strobe),
                           "fires": self._strobe_fires,
                           "last_epoch": self._strobe_last,
                           "error": self._strobe_err},
                # With the gpiod driver the lines are held open-drain-idle for
                # the life of the process, which is the safe state by construction.
                "harness_safe": (self.driver.ok
                                 or all(parked.get(b) for b in self.PARKED_INPUTS))}

    def shutdown(self):
        self._mon_run = False
        self._watchdog_run = False
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
        # The strobe line releases to its cmdline-configured no-pull high-Z
        # (`gpio=26=ip,np`), which the flash's own pull-up holds idle — never
        # asserted, so a dying piagent cannot latch the tube on.
        if self._strobe:
            self._strobe.set(BCM_STROBE, _LineDriver.IDLE)
            self._strobe.close()
            self._strobe = None
        # Leave a pull-up on the pads either way: once our handles are gone the
        # line reverts to pad bias, and a bare release lands on a pull-DOWN here
        # - which is a permanent half-press on the camera.
        for bcm in self.PARKED_INPUTS:
            self._park(bcm)
            self._unpark(bcm)


# ---------------------------------------------------------------------------
# IMU — optional, on whichever node it happens to be plugged into (the Jetson
# discovers that from fleet health; nothing here assumes a node name). Imported
# lazily so nodes without it still run.
# ---------------------------------------------------------------------------
class Imu:
    """Owns one IMU reader slot and keeps trying to (re)acquire it.

    The IMU is a USB-serial device; it may be plugged in after piagent starts,
    moved between ports, or drop out. So detection runs in a background loop
    that probes until a device answers, samples it, and re-probes if the reader
    ever dies — no service restart needed when the user reseats it.

    This class is now a SLOT: everything below works for any driver module
    speaking the imu_yb duck-type (module probe(); ImuReader with read/latest/
    window/rate_hz/... — the getattr() fallbacks already assumed exactly
    that), so the second physical unit (Imu2, the Olive at cam2) is a second
    INSTANCE with different class attributes, not a second copy of this
    machinery. The attributes exist because the two devices' normals differ:
    a floor tuned to the YB's 50 Hz attitude cadence judged against a unit
    whose healthy cadence is unknown until bring-up would alarm always or
    never, and either way nobody reads it.
    """

    MODULE = "imu_yb"                    # driver module this slot loads
    TAG = "imu"                          # log tag; slot 2 logs as "imu2"
    ABSENT_MSG = ("no IMU yet - will keep probing "
                  "(USB-serial on /dev/ttyUSB*/ttyACM*)")
    NOMINAL_HZ = IMU_ATTITUDE_HZ_NOMINAL
    PROBE_BAND = (0.5, 4.0)              # sane band, multiples of NOMINAL_HZ
    FRAME_FLOOR_HZ = IMU_MIN_FRAME_HZ

    def __init__(self):
        self.reader = None
        self.info = {"present": False}
        self._lock = threading.Lock()
        self._run = True
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        threading.Thread(target=self._acquire_loop, daemon=True).start()

    def _acquire_loop(self):
        try:
            import importlib
            mod = importlib.import_module(self.MODULE)
        except Exception as e:  # noqa: BLE001
            log("info", self.TAG, "%s not available" % self.MODULE, err=str(e))
            return
        announced_absent = False
        while self._run:
            if self.reader is not None:
                # Alive? If the sampler died or samples went stale, drop it and
                # re-acquire.
                try:
                    fresh = self._reader_fresh(self.reader)
                except Exception:  # noqa: BLE001
                    fresh = False
                if not fresh:
                    log("warn", self.TAG, "IMU sampler stalled - re-acquiring")
                    self._drop()
                else:
                    time.sleep(2)
                    continue
            try:
                info = self._probe(mod)
            except Exception:  # noqa: BLE001
                info = None
            if info and info.get("chip"):
                try:
                    reader = mod.ImuReader(**{
                        k: info[k] for k in ("bus", "port", "addr", "baud")
                        if k in info and info[k] is not None})
                    reader.open()
                    reader.start()
                    with self._lock:
                        self.reader = reader
                        self.info = {"present": True, **info}
                    announced_absent = False
                    log("info", self.TAG, "IMU online", **{k: info.get(k)
                        for k in ("chip", "bus", "port", "addr", "baud")})
                except Exception as e:  # noqa: BLE001
                    log("warn", self.TAG, "IMU start failed", err=str(e))
                    self._drop()
            elif not announced_absent:
                announced_absent = True
                log("info", self.TAG, self.ABSENT_MSG)
            time.sleep(4)

    def _probe(self, mod):
        """One probe pass. A hook so Imu2 can pass its configured spec."""
        return mod.probe()

    @staticmethod
    def _reader_fresh(reader):
        """Is the reader still producing ATTITUDE, not merely bytes?

        The old test used latest(), which trailing inertial/baro frames
        deliberately refresh - so a fusion whose euler/quat output halted
        while raw inertial kept streaming looked fresh forever and the
        drop/re-acquire recovery never ran, while the host's imu_snapshot
        found an empty ring and blanked every IMU column. Freshness must
        mean the last RING publish (an orientation frame): imu_yb exposes
        its epoch; fall back to latest() only for an older imu_yb without
        the API (rolling deploy)."""
        fn = getattr(reader, "last_attitude_epoch", None)
        if callable(fn):
            e = fn()
            return bool(e) and (time.time() - e) < 5
        s = reader.latest()
        return bool(s and s.get("epoch") and (time.time() - s["epoch"]) < 5)

    def _attitude_floor_hz(self):
        """Below this attitude cadence, THIS device has departed from normal.

        Not a spec number: a fraction of what the unit itself measured at
        probe time (49.8 Hz on the YB-MRA02 here). A fixed 60 Hz was above
        the device's healthy rate, so imu_rate_low was permanently true; a
        fixed low number would miss a device whose fusion halved. The probe
        figure is only trusted inside a sane band around the nominal cadence,
        so a probe taken while the device was already sick cannot lower the
        bar on itself for the rest of the session."""
        base = self.NOMINAL_HZ
        try:
            probed = float(self.info.get("sample_rate_hz") or 0.0)
        except (TypeError, ValueError):
            probed = 0.0
        if self.PROBE_BAND[0] * self.NOMINAL_HZ <= probed \
                <= self.PROBE_BAND[1] * self.NOMINAL_HZ:
            base = probed
        return round(base * IMU_ATTITUDE_LOW_FRAC, 1)

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
                # The LIVE ring rate, not the probe's figure. `rate_hz` used
                # to substitute the probe's start-up number whenever the
                # measured rate was FALSY - so a fusion whose attitude output
                # stalled to a true 0.0 Hz read as a healthy 49.8 Hz forever.
                # Fall back to the probe value only while the first
                # measurement window is still open ("not yet measured", which
                # imu_yb now distinguishes from "measured zero"); once
                # measured, 0 means 0.
                # getattr, because a node mid-rolling-deploy may still be
                # running an older imu_yb without these; health must answer
                # regardless.
                rate = self.reader.rate_hz()
                mfn = getattr(self.reader, "rate_measured", None)
                measured = bool(mfn()) if callable(mfn) else bool(rate)
                att = rate if measured else \
                    (rate or self.info.get("sample_rate_hz"))
                # attitude vs frame cadence, NAMED (the old pair rate_hz /
                # frame_rate_hz kept getting read as one number): attitude_hz
                # is what the "how stale may this attitude be" budget sizes
                # against; frame_hz is link health. rate_hz/frame_rate_hz stay
                # for older readers of /health.
                h["rate_hz"] = att
                h["attitude_hz"] = att
                fr = getattr(self.reader, "frame_rate_hz", None)
                h["frame_rate_hz"] = fr() if fr else None
                h["frame_hz"] = h["frame_rate_hz"]
                # The alarm bit rigd's anomaly scan reads. It compared
                # attitude_hz against the 60 Hz figure, which is the FRAME
                # rate spec - and this device's healthy attitude cadence is
                # ~50 Hz, so /health reported imu_rate_low:true on a good
                # unit for the whole session. Judge each rate against its own
                # spec, and publish the thresholds so the number can be
                # checked rather than trusted. None until measured, so the
                # first second of streaming cannot false-alarm.
                h["attitude_floor_hz"] = self._attitude_floor_hz()
                h["frame_floor_hz"] = self.FRAME_FLOOR_HZ
                if not measured:
                    h["imu_rate_low"] = None
                else:
                    low = att is not None and att < h["attitude_floor_hz"]
                    if h["frame_hz"] is not None and \
                            h["frame_hz"] < self.FRAME_FLOOR_HZ:
                        low = True
                    h["imu_rate_low"] = low
                # Frames the decoder refused (checksum mismatch, out-of-range
                # or non-finite values): "quat frames present but discarded"
                # used to be indistinguishable from "device sends none".
                rj = getattr(self.reader, "rejected_frames", None)
                h["rejected_frames"] = rj() if callable(rj) else None
                # Whether frame-checksum verification is actually ON. The gate
                # learns the device's scheme from the live stream and goes
                # dormant if none of the candidates fits, so "corrupt frames
                # are being rejected" is a claim /health has to be able to
                # answer honestly rather than one the operator assumes.
                cs = getattr(self.reader, "checksum_state", None)
                h["checksum"] = cs() if callable(cs) else None
                # Seconds since the attitude last MOVED. A locked-up fusion
                # keeps streaming identical numbers at full rate, so age_s stays
                # small and nothing else here would ever notice.
                fz = getattr(self.reader, "orientation_frozen_s", None)
                h["orient_frozen_s"] = fz() if fz else None
                # Age of the last RING publish (attitude): latest() is
                # refreshed by inertial-only traffic, so age_s alone cannot
                # show an attitude stall.
                la = getattr(self.reader, "last_attitude_epoch", None)
                e = la() if callable(la) else None
                h["attitude_age_s"] = round(time.time() - e, 3) if e else None
                h["age_s"] = round(time.time() - s["epoch"], 3) if s and \
                    s.get("epoch") else None
            except Exception:  # noqa: BLE001
                h["age_s"] = None
        return h

    def shutdown(self):
        self._run = False
        self._drop()


# Second-IMU slot configuration. Values (case kept for device paths):
#   (unset) / "auto"        auto-probe: /dev/ttyACM* scan, then a UDP dwell
#   "off"|"0"|"none"        disabled — no thread, /health answers present:false
#   "olive"                 auto-probe, explicitly
#   "olive:/dev/ttyACM0"    that serial device
#   "olive:udp[:port]"      UDP listener (default port: imu_olive's 9901)
#   "olive:sim"             synthetic stream (offline tests)
IMU2_SPEC = os.environ.get("PIAGENT_IMU2", "")


class Imu2(Imu):
    """Second IMU slot — the Olive olixVision X1 at cam2 (rig/imu_olive.py).

    A second Imu INSTANCE, not new machinery: only the driver module, the
    config gate and the rate floors differ. Deliberately isolated from the
    fire path the same way slot 1 always was — its reader runs its own
    thread, every endpoint call is try/except'd in the base class, and the
    acquire loop shares nothing with Gpio — so a wedged or absent olive can
    stall neither a fire nor the YB sampler. When the unit is absent the slot
    behaves exactly like slot 1 before an IMU was ever plugged in: probe,
    find nothing, answer {present:false}. Zero change to any /imu/* payload.
    """

    MODULE = "imu_olive"
    TAG = "imu2"
    ABSENT_MSG = ("no second IMU yet - will keep probing "
                  "(olixVision on /dev/ttyACM* or UDP)")
    # The X1's healthy attitude cadence is UNKNOWN until bring-up (ROS-native
    # units commonly run 100-400 Hz). Wide probe band so the measured normal
    # can raise the bar honestly, and a frame floor low enough that a unit we
    # have never measured cannot ring imu_rate_low all day; tighten both from
    # the probe figures once the device is on the bench (docs/olive-imu.md).
    PROBE_BAND = (0.5, 20.0)
    FRAME_FLOOR_HZ = 10.0

    # Measured on the real unit 2026-08-27: the olive-bridge relay of the
    # port-5500 WebSocket stream delivers ~15 Hz. Floor below that so a
    # healthy stream never rings imu_rate_low while a stalled one still does;
    # revisit if the unit's Sensor Settings ever raises the stream rate.
    def _attitude_floor_hz(self):
        return 10.0

    def __init__(self, spec=None):
        raw = (IMU2_SPEC if spec is None else spec).strip()
        low = raw.lower()
        self.enabled = low not in ("off", "0", "none", "disabled", "no")
        self.spec = None                     # None = imu_olive autodetect
        if low.startswith("olive"):
            self.spec = raw[len("olive"):].lstrip(":") or None
        elif raw and low != "auto" and self.enabled:
            self.spec = raw                  # bare "/dev/ttyACM0" / "udp:9901"
        self._last_probe = None
        if not self.enabled:
            # No acquire thread at all: disabled must cost nothing, and the
            # query surface below still needs its fields to answer.
            self.reader = None
            self.info = {"present": False}
            self._lock = threading.Lock()
            self._run = False
            return
        super().__init__()

    def _probe(self, mod):
        info = mod.probe(self.spec)
        # Keep the last probe verdict readable from /health even while
        # absent: bring-up needs "the port exists but speaks 240 undecoded
        # bytes (hex follows)" without a shell on the node — that raw tail is
        # the whole point of the tolerant driver.
        if isinstance(info, dict) and not info.get("present"):
            self._last_probe = {k: info[k] for k in
                                ("notes", "raw_tail_hex") if k in info}
        else:
            self._last_probe = None
        return info

    def health(self):
        h = super().health()
        h["enabled"] = self.enabled
        lp = self._last_probe
        if not h.get("present") and lp:
            h["probe"] = lp
        return h


GPIO = Gpio()
IMU = Imu()
IMU2 = Imu2()


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


def _imu_health():
    """Slot-1 health with slot 2 nested INSIDE it as `imu2`.

    Nested rather than a sibling top-level key because rigd's fleet snapshot
    forwards h["imu"] verbatim ("imu": h.get("imu")), so the second unit
    reaches the fleet view and the anomaly scan through plumbing that already
    exists — a sibling would be silently dropped by every host until a host
    release. Additive only: every pre-imu2 key of the section is untouched,
    and a wedged slot-2 health can never take /health (and with it the
    host's clock-offset sample) down."""
    h = IMU.health()
    try:
        h["imu2"] = IMU2.health()
    except Exception as e:  # noqa: BLE001
        h["imu2"] = {"present": False, "error": str(e)}
    return h


def health():
    # Stamp the clock FIRST. The host derives this node's clock offset as
    # time.epoch minus the midpoint of its request round trip; the stamp used
    # to be the LAST field built, after GPIO subprocess polls, IMU state and a
    # listdir of the whole spool, so it was biased by that work - and on a Pi 4
    # with a few thousand spooled frames the inflated RTT pushed the node out
    # of the host's clock-skew gate entirely. Stamp before the work, and
    # report the work's duration so the host can see (and bound) what follows
    # the stamp.
    t0 = time.time()
    h = {
        "node": NODE,
        "time": {"epoch": t0, "source": "local"},
        "uptime_s": round(t0 - T_START, 1),
        # The Pi's uptime, not this process's: a service restart (every
        # deploy) must not read as a power loss on the host.
        "host_uptime_s": _host_uptime(),
        "gpio": GPIO.state(),
        "imu": _imu_health(),
        "disk_free_mb": _disk_free_mb(CAM_SAVE_DIR),
        "cam_frames": _count_frames(),
        "load1": _load1(),
        "power": _power(),
    }
    h["time"]["work_ms"] = round((time.time() - t0) * 1000.0, 2)
    return h


def _host_uptime():
    try:
        with open("/proc/uptime") as fh:
            return round(float(fh.read().split()[0]), 1)
    except (OSError, ValueError):
        return None


_THROTTLED_PATH = "/sys/devices/platform/soc/soc:firmware/get_throttled"


def _power():
    """The firmware's under-voltage / throttling word, straight from sysfs
    (no subprocess): bit 0 = under-voltage NOW, 16 = has occurred since boot,
    1/17 = frequency capped, 2/18 = throttled, 3/19 = soft temp limit. On
    this rig a node that browns out usually reboots outright (flags reset),
    but a sag that stops short of a reset shows here first."""
    try:
        with open(_THROTTLED_PATH) as fh:
            raw = int(fh.read().strip(), 0)
    except (OSError, ValueError):
        return None
    return {"throttled": "0x%x" % raw,
            "undervolt_now": bool(raw & 0x1),
            "undervolt_since_boot": bool(raw & 0x10000),
            "throttled_now": bool(raw & 0x4),
            "throttled_since_boot": bool(raw & 0x40000)}


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
        """POST body as a dict, or None for a body that FAILED to parse.

        The two used to be the same answer, {} - so a truncated JSON body on a
        state-changing endpoint proceeded on defaults: /gpio/fire with a
        mangled at_epoch fired NOW instead of at the scheduled instant,
        /gpio/focus dropped a hold it was asked to extend. A missing body is
        an intentional "all defaults" call (several endpoints are used that
        way); a PRESENT body that cannot be parsed is a client bug and must be
        a 400, never a guess (audit 2026-08-27)."""
        try:
            n = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            n = 0
        if n <= 0 or n > 1 << 20:
            return {}
        try:
            doc = json.loads(self.rfile.read(n).decode() or "{}")
            return doc if isinstance(doc, dict) else None
        except (ValueError, OSError):
            return None

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
            # The second IMU gets its OWN endpoints, not an "imu2" key inside
            # /imu/*: (a) /imu/latest's success payload IS the bare sample
            # dict — there is no envelope to hang a sub-object on without
            # polluting the key namespace flight_log reads by name; (b) rigd
            # elects the MASTER orientation source by probing /imu/latest for
            # an `epoch`, so the olive answering there would race the YB for
            # master. Same shapes as /imu/*; runbook: docs/olive-imu.md.
            elif path == "/imu2/latest":
                self._send(IMU2.latest())
            elif path == "/imu2/window":
                t0 = float(q.get("t0", 0) or 0)
                t1 = float(q.get("t1", time.time()) or time.time())
                self._send(IMU2.window(t0, t1))
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
        if b is None:
            self._send({"ok": False,
                        "error": "request body is not valid JSON"}, 400)
            return
        try:
            if path == "/gpio/focus":
                # Optional ttl_s: the hold lease length; absent means the
                # 30 s default. Repeating {hold:true} is the keepalive.
                ok = GPIO.focus(bool(b.get("hold")), ttl_s=b.get("ttl_s"))
                self._send({"ok": ok, "focus_held": GPIO.focus_held(),
                            "focus_held_s": GPIO.focus_held_s()})
            elif path == "/gpio/fire":
                res = GPIO.fire(b.get("at_epoch", 0), b.get("pulse_ms", 5),
                                b.get("focus_lead_ms", 0),
                                b.get("strobe_at_epoch", 0),
                                b.get("strobe_pulse_ms", 5))
                self._send(res, res.get("code", 200 if res.get("ok") else 400))
            elif path == "/gpio/strobe":
                res = GPIO.strobe_only(b.get("at_epoch", 0),
                                       b.get("pulse_ms", 5))
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
        IMU2.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _bye)
    signal.signal(signal.SIGTERM, _bye)

    log("info", "lifecycle", "piagent up", port=PORT, chip=GPIOCHIP,
        gpio=GPIO.available, imu=IMU.info.get("present", False),
        imu2=IMU2.info.get("present", False), imu2_enabled=IMU2.enabled)
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    try:
        srv.serve_forever()
    finally:
        GPIO.shutdown()
        IMU.shutdown()
        IMU2.shutdown()


if __name__ == "__main__":
    main()

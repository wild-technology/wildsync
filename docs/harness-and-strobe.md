# ILX-LR1 harness — leads, electrical behaviour, and the trigger chain

What each lead on the ILX-LR1 power/control harness does, how `FOCUS`, `TRIGGER`
and `EXPOSURE` behave electrically, and what the sync-speed and flash-timing
measurements say.

> **Scope.** This is the **camera harness** reference. The strobe sections that
> once lived here covered a Smart Vision Lights XR256, which is **out of scope** —
> the rig uses a Bolt VB-22 speedlight on a contact-closure sync port, and none of
> the XR256's 24 V / 20 A constraints carry over. Those sections have been removed.
> For the strobe as actually built, see **`docs/strobe-trigger.md`**.

**Sources**

| | |
|---|---|
| Sony *ILX-LR1 Help Guide*, `5-055-988-22(1)`, 437 pp | connector spec pp. 414–416, making your own cable p. 417, connection examples pp. 28–29, power p. 70, specifications p. 426. |
| Camera Remote SDK v2.02.00 support matrix | `CrSDK_API_Reference_v2.02.00/_static/device_property_list.csv` |

---

## 1. The headline

The ILX-LR1 has **no hot shoe, no multi-interface shoe, no PC sync terminal**.
Its only external connectors are USB Type-C, micro HDMI (type D), and the 6-pin
power/control connector.

The SDK is no help either — every flash property is unsupported on this body:

| Property | ILX-LR1 |
|---|---|
| `CrDeviceProperty_FlashMode` | — |
| `CrDeviceProperty_FlashCompensation` | — |
| `CrDeviceProperty_WirelessFlash` | — |
| `CrDeviceProperty_SynchroterminalForcedOutput` | — |

**One path exists: harness pin 6, `EXPOSURE`** — an open-drain, active-low output
that sinks while the shutter is open. Whether the strobe is driven from it, or
from a scheduled GPIO pulse that sees both cameras, is decided in
`docs/strobe-trigger.md`; the rig uses the scheduled GPIO pulse.

---

## 2. Camera side — pinout

**Camera-side connector: Molex Micro-Fit 3.0, 6-pin, `430450622`.**

| Pin | Dir | Name | Function |
|---|---|---|---|
| 1 | — | — | Non-functional. Connected to nothing. |
| 2 | GND | `DC 10-18V IN −` | Ground. **Also the signal return for pins 4/5/6.** |
| 3 | Power in | `DC 10-18V IN +` | Camera supply. |
| 4 | Input | `FOCUS` | Locks focus while the input is **Low**. |
| 5 | Input | `TRIGGER` | Takes the shot on **Low**, *provided `FOCUS` is already Low*. |
| 6 | **Output** | `EXPOSURE` | **Low** from the moment the front curtain is fully open until end of exposure. Asserted **1 ms or more**. |

**Power:** 10–18 V DC measured *at the camera connector* (size conductors for
the drop). **Momentary ~40 W** when taking a still — use a ≥40 W supply. Power
LED red = powered, switch off; green = powered and on. Red while switched ON
means SDK-issued power-off, power-save, <10 V, >18 V, or thermal shutdown.

---

## 3. The input leads — `FOCUS` and `TRIGGER`

```
Focus ON  : 0 V          Trigger ON  : 0 V
Focus OFF : open         Trigger OFF : open
```

Drive each from **a switch alternating between open (unconnected) and GND, or
from an open-drain / open-collector circuit.** Never drive them high — "off" is
*open*, not a logic 1.

> **Measured 2026-08-14: both inputs idle at 2.8 V** open-circuit, referenced to
> pin 2 with the camera powered. `EXPOSURE` reads 0 V on the same reference,
> which is what an open-drain output with no external pull-up should do.
>
> **This settles the interface question: a 3.3 V GPIO in open-drain mode drives
> `FOCUS` and `TRIGGER` directly.** No MOSFET, no level shifter. Asserting sinks
> only the camera's internal pull-up current; idling high-Z leaves the pin at
> 2.8 V, inside a 3.3 V pin's absolute maximum.

> ### Correction, measured 2026-08-15: a released GPIO is NOT high-Z
>
> An earlier revision of this document claimed an unconfigured GPIO boots
> high-Z and is therefore the safe idle state. **That is wrong, and it bricked
> the camera for most of a day.** A Pi pad carries a bias, and releasing a
> libgpiod line does not clear it — the pad keeps whatever the last requester
> left. `gpioset --drive=open-drain` leaves bias *disabled*, and BCM17 then
> measured a solid **0** with the harness disconnected and no bias applied:
>
> ```
> gpio17 (FOCUS)    as-is: 0   disable: 0    <- held Low
> gpio27 (TRIGGER)  as-is: 1   disable: 1
> gpio22 (EXPOSURE) as-is: 1   disable: 1
> ```
>
> Wired up, a Low `FOCUS` is a **permanent half-press**. The consequences look
> nothing like a wiring fault and cost a long night of misdiagnosis:
>
> * AE/AF lock, so `IsoSensitivity`, `FNumber` and `FocusMode` all report
>   `CrEnableValue_DisplayOnly` over the SDK — every write is refused;
> * the body takes control priority back, `PriorityKeySettings` reads
>   `CrPriorityKey_CameraPosition`, and the `-PC-` indicator shows white
>   instead of orange;
> * the remote shutter is silently ignored, over USB *and* over `TRIGGER`;
> * pulling the connector clears all of it instantly, which makes the whole
>   thing read as an SDK or firmware problem rather than one stuck line.
>
> **The safe idle state for a camera input is an input with a pull-up**, not a
> released line. `piagent` therefore *parks* `FOCUS` and `TRIGGER` under a
> long-lived `gpiomon --bias=pull-up` whenever they are not asserted, and
> re-parks on release, on error, and at shutdown. The pull-up bias persists in
> the pad after the parker exits, so the camera stays safe while piagent is
> stopped. `/gpio/state` exposes `harness_safe`; if it ever reads false, treat
> it as an operator-visible fault and unplug the harness.

**`FOCUS` must lead `TRIGGER`.** If `[AF-S Priority Setting]` / `[AF-C Priority
Setting]` is `AF` or `Balanced Emphasis` and the gap is short, the camera may not
focus in time and **will not fire**. Lengthen the gap, or set priority to
`Release` — the right choice for a fixed-focus survey rig.

**Achievable rates**, even with `[Drive Mode]` = `Single Shooting`:

| Pattern | Max rate |
|---|---|
| `TRIGGER` toggled, `FOCUS` held Low | **~5 fps** |
| `FOCUS` and `TRIGGER` both toggled | **~2.5 fps** |

(Body's own continuous ceiling: Hi+ ~8 fps, Hi ~6, Mid ~3, Lo ~2.5.)

**Why this matters for `ilxctl`.** Today the shutter fires over USB via
`SDK::SendCommand(Release)` — a few hundred ms round trip, which is why
`kMinHostIntervalSec` is pinned at 0.5 s (`src/camera.h:38`). The harness path is
a wire: GPIO-driven `FOCUS`/`TRIGGER` gives deterministic sub-ms release at up to
5 fps, independent of PTP session health. USB then keeps what it is good at —
settings, live view, status, image transfer.

---

## 4. The output lead — `EXPOSURE`

### Timing semantics

> Outputs "Low" from when the front curtain is **fully open** until the end of
> exposure (1 msec or more).

An *exposure-active* window referenced to the shutter itself, generated inside
the camera — none of the USB or host-scheduling jitter of the SDK release path.

**Sony's recommended trigger diagram (p. 416) puts real numbers on the chain:**

```
FOCUS    ──┐_______________┌──   held Low ≥1 ms, must lead TRIGGER
TRIGGER  ─────┐__________┌───    Low ≥1 ms
              │
              ├──── ~20 ms ────▶ exposure starts (first row uncovered)
              │                        ├── ~4 ms ──▶ front curtain FULLY open
EXPOSURE ─────────────────────────────────────────┐________┌──  Low ≥1 ms
```

| Interval | Value |
|---|---|
| `FOCUS` Low duration | **≥1 ms** |
| `TRIGGER` Low duration | **≥1 ms** |
| `TRIGGER` falling edge → exposure starts | **~20 ms** |
| exposure start → front curtain fully open | **~4 ms** (curtain travel) |
| `EXPOSURE` Low duration | **≥1 ms** |

Three things follow, and they drive every design decision below:

1. **Release lag is ~20 ms**, plus ~4 ms of curtain travel before `EXPOSURE`
   even asserts. Roughly **~24 ms from `TRIGGER` to a usable strobe edge.**
2. **`EXPOSURE` is not a faithful copy of the shutter-open interval.** It starts
   4 ms late relative to first light, and it is floored at 1 ms regardless of how
   short the exposure was.
3. The diagram draws the exposure as a **slanted parallelogram** — the explicit
   slit-scan of a focal-plane shutter. `EXPOSURE` marks the top-left corner, the
   moment the *last* row is finally uncovered.

### Electrical — open drain, needs a pull-up

```
Exposure ON  : open drain enabled   (pin pulled to 0 V)
Exposure OFF : open drain disabled  (pin floating)
```

Sony's sizing rule for the external pull-up:

> `C ≥ A × 1/(B+1)`
> A = pull-up supply (V) · B = pull-up resistance (kΩ) · C = V<sub>IL</sub> of the external circuit (V)

Read back, the implied model is ~1 kΩ of sink on-resistance, so the asserted low
sits at `A × 1/(B+1)` and must land below the receiver's V<sub>IL</sub>.

**Worked example, Pi GPIO input** (A = 3.3 V, V<sub>IL</sub> ≈ 0.8 V):

| Pull-up B | Low level `A/(B+1)` | OK? |
|---|---|---|
| 4.7 kΩ | 0.58 V | yes |
| 10 kΩ | 0.30 V | yes, comfortably |
| 1 kΩ | 1.65 V | **no** — too stiff |

**10 kΩ to 3.3 V** reads cleanly on a Jetson pin with no level shifting. Verify
on a scope; the 1 kΩ on-resistance is inferred from the formula, not stated.

### Grounding — the rule that bites

> When connecting `FOCUS`/`TRIGGER`/`EXPOSURE` to a device other than the power
> source, connect that device's GND (0 V) to the **`DC IN −` pin** on this
> product.

Signal return is pin 2, the *same* pin as power negative. Single-point tie there
— or sidestep the whole question with the opto in §5.

### The sync-speed caveat — now measured, see §4.1

`EXPOSURE` asserts once the front curtain is *fully open*. On a focal-plane
shutter that condition only truly holds at or below the X-sync speed; above it a
travelling slit scans the sensor and no instant exists when the whole frame is
uncovered. Sony publishes **no flash sync speed for this body** — unsurprising,
it supports no flash.

**Measured 2026-08-14: the ceiling is 1/200 s.** Banding is absent at 1/200 and
present at 1/250. That matches the ~4 ms curtain travel from Sony's own diagram
almost exactly — X-sync is the reciprocal of curtain transit, and 4 ms → 1/250.
The full measurement, including how the shutter was used to time the flash
itself, is in §4.1.

Shutter range (p. 426), focal-plane, vertically travelling, electronically
controlled:

- Mechanical: **1/4000 s – 30 s, BULB**
- Electronic: **1/8000 s – 30 s** (no BULB, no long-exposure NR, no anti-flicker)

`CrDeviceProperty_ShutterType` **is** SDK-settable here, so `ilxctl` can pick
mechanical vs electronic remotely — pin to mechanical for strobe work until the
electronic-shutter behaviour of `EXPOSURE` has been scoped.

---

## 4.1 Exposing faster than the shutter can open

*Bench measurements, 2026-08-14. Bolt VB-22 at 1/32 power on the 3.5 mm sync
port, f/11, dim room, a millisecond stopwatch in frame.*

The central result: **once the flash dominates, the camera's shutter speed stops
setting the exposure time.** The picture is exposed for as long as the flash
burns, which here is **~150 µs — an effective 1/6,600 s**, from a body whose
shutter cannot sync faster than 1/200. That is a factor of **33 beyond the
mechanical sync limit**, and it is the single most useful property of the rig.

### The flash is doing 97 % of the work

Sweeping shutter across 4.6 stops at fixed f/11 and ISO 3200:

| Shutter | Mean luma | vs 1/8 |
|---|---|---|
| 1/8 | 126.4 | — |
| 1/30 | 107.8 | −0.23 EV |
| 1/125 | 101.1 | −0.32 EV |
| 1/200 | 99.3 | **−0.35 EV** |

If ambient were exposing the frame, 1/8 → 1/200 would cost **4.6 EV**. It cost
**0.35**. Separating the two components (flash constant, ambient proportional to
time) puts **flash at ~97 %** and ambient at ~3 % at 1/200 — about five stops
of headroom.

The stopwatch confirms it directly rather than by inference. At **1/8 s** the
shutter is open 125 ms, during which the hundredths digit advances **12 counts**
— yet the display reads a crisp `3'46"98` with no smear. A 125 ms window
collapsed to a single instant. The 1/200 frame reads `3'56"98`, proving the
timer was running and the sharpness is not a frozen display.

### The sync limit is a *fast* limit only

Worth stating plainly because it is easy to get backwards: X-sync caps how
**short** the shutter may be, not how long. Below the limit the sensor is fully
uncovered and stays that way indefinitely.

Row-band luma across the frame, top band first:

| Shutter | Top band | Bands masked |
|---|---|---|
| 1/200 | 97 | 0 of 12 |
| **1/250** | **11** | 1 |
| 1/400 | 7 | 5 |
| 1/500 | 5 | **7 of 12** |

At 1/500 the curtain masks 60 % of the sensor while the flash fires. Going the
other way, **1/15, 0.8 s, 1 s and 2 s were all completely band-free.**

### The slow-side limit is ambient, not sync

| Shutter | Ambient share of exposure |
|---|---|
| 1/200 | **1.7 %** |
| 1/60 | 5.4 % |
| 1/15 | 18.6 % |
| 0.8 s | 73.3 % |
| 1 s | 77.4 % |
| 2 s | 87.3 % |

So the rule is **not** "use 1/200". It is: *use the slowest shutter whose ambient
contribution stays acceptable.* On this bench 1/60 holds ambient at 5 % while
giving a 16.7 ms window. In genuinely dark water ambient approaches zero and a
one-second exposure is legitimate — which matters enormously for the two-camera
rig, because the overlap window in which both shutters must be open becomes a
full second rather than a few milliseconds.

**Field procedure:** shoot one frame at the candidate shutter with the strobe
disconnected. If it comes back black, that shutter is free. Repeat as depth and
surface light change.

### Timing the flash with the shutter

Above sync the shutter is a travelling slit, so the **sharpness of the band edge
encodes how long the flash burned** while the slit moved. Ratioing each banded
frame against the fully-lit 1/200 frame cancels scene content and leaves pure
shutter transmission.

Three frames agree on the curtain transit, which is the calibration:

| Frame | Lit fraction | Implied transit | Edge width (10–90 %) |
|---|---|---|---|
| 1/250 | 90.3 % | 4.43 ms | 2.67 % of frame |
| 1/400 | 56.0 % | 4.46 ms | 3.67 % |
| 1/500 | 42.3 % | 4.72 ms | 3.33 % |

**4.43 / 4.46 / 4.72 ms** — mutually consistent, consistent with the observed
banding onset between 1/200 and 1/250, and consistent with Sony's published
~4 ms. Three independent routes to the same figure.

Applying transit to edge width gives the flash duration:

| | from 1/400 | from 1/500 |
|---|---|---|
| transit 4.0 ms | 147 µs | 133 µs |
| transit 5.0 ms | 183 µs | 167 µs |

**133–183 µs, centre ~150 µs.** Treat it as an *upper* bound — the measured edge
also carries lens vignetting, defocus and JPEG ringing, so the true flash is that
or shorter.

Note this is roughly **twice as fast as interpolating Bolt's published
1/300–1/10,000 range** would suggest for 1/32 power. Flash duration collapses
faster than power does at the low end, which is characteristic of IGBT-quenched
designs. Do not trust log-interpolation of a flash-duration range; measure it.

### What this buys

- **Motion blur at 1 m/s: 0.15 mm** — about **0.42 px** at 2 m range. Sub-pixel,
  and better than the 0.7 px projected from the datasheet interpolation.
  **1/32 power is already fast enough for photogrammetry at survey speed**; there
  is no need to drop to 1/64 or 1/128 and pay for it in light.
- **The shutter dial no longer controls sharpness.** Set it wherever ambient
  allows; the picture is still exposed at ~1/6,600 s. The dial governs how much
  ambient veiling you accept, not what is sharp.
- **The two-camera sync requirement largely dissolves.** Both shutters need only
  be *open* when the flash fires. In dark water that window is hundreds of
  milliseconds, against an inter-body lag mismatch of a few ms — so the 89 µs
  trigger-sync budget derived for continuous light does not apply to strobed
  work. See §5.5.
- **Frame rate is capped by strobe recycle, not the camera.** Bolt spec is
  0.1–5.2 s across its power range; at 1/32 expect roughly 0.5 s, so about
  2 fps. That, not the shutter, is the ceiling.

### Method notes, for repeating this

- Row-band profiles were computed on the delivered JPEGs at reduced decode scale;
  ratioing against a fully-lit reference from the **same sweep** is essential —
  an earlier attempt using a mismatched reference gave a curtain transit of
  8.2 ms, double the true value, and a correspondingly inflated flash duration.
- The stopwatch frames both landed on the same hundredths digit (`98`), which is
  either coincidence at the 1 % level or an artefact of the sweep cadence
  quantising near 10.00 s. It does not affect the conclusion — the brightness
  sweep is independent evidence — but a re-shoot at irregular intervals would
  make that line of evidence airtight.

---

## 5. Measured inter-camera skew — stopwatch ground truth

*2026-08-14, two ILX-LR1 bodies on separate Pi nodes, one running stopwatch in
both frames. This needs no clock discipline: the stopwatch is a shared reference,
so the difference between the two readings IS the skew.*

| Method | Trials | Skew observed | Mean |
|---|---|---|---|
| Host-commanded single release | 6 | 0, 0, 100, 100, 100, 200 ms | **+83 ms** |
| Continuous hold, both started together | 3 | 0, 0, 100 ms | **+33 ms** |

The display resolves 10 ms, so each reading is quantised — but the scale is
unambiguous, and it corroborates the EXIF-interval jitter measured separately
(sigma 68.8 ms over 100 frames). Two independent methods, same answer.

**The distinction that matters:** camera-timed capture fixes *interval* jitter,
not *start* alignment. Within a continuous burst the body free-runs at 15.3 ms
sigma; but the burst START is a host command carrying the full USB jitter, so
frame N from each camera is offset by whatever that start skew was.

| Path | Interval jitter | Start alignment |
|---|---|---|
| Host release per frame | 102.8 ms sigma | 0-200 ms |
| Host intervalometer | 87.0 ms sigma | 0-200 ms |
| Camera Interval REC | **4.7 ms sigma** | free-running, drifts |
| Continuous hold | **15.3 ms sigma** | 0-100 ms (host-started) |
| GPIO (projected) | sub-us | **sub-us** |

**Operational consequence.** Without GPIO, use a shutter long enough to absorb
~100-200 ms of start skew so both windows overlap and the flash can stamp both
frames with one instant. **1/8 s (125 ms) works; 1/30 s (33 ms) does not.** In
dark water a slow shutter costs nothing (see 4.1), so transects are possible
today. GPIO buys the shutter speed back, which matters only where ambient is
significant.

---

## 6. Triggering a second camera — do not daisy-chain

Sony does describe `EXPOSURE` as a shooting-trigger output ("send a shooting
trigger signal (EXPOSURE) to the drone"), so using camera 1 to trigger camera 2
looks natural. **The ~20 ms release lag makes it wrong.**

Chaining `EXPOSURE`(cam 1) → `TRIGGER`(cam 2) stacks the whole lag twice:

```
cam1 TRIGGER ──20 ms──▶ exposing ──4 ms──▶ EXPOSURE asserts
                                              └──▶ cam2 TRIGGER ──20 ms──▶ cam2 exposing
                                                                  ~44 ms after cam 1
```

Cam 2's exposure starts roughly **44 ms** after cam 1's — and cam 1's has often
already *ended*. At 1 m/s platform speed that is 44 mm of displacement between
frames that were supposed to be simultaneous. It also cannot work with one
strobe: the two exposure windows never overlap, so a single flash cannot land in
both.

**Instead, trigger both in parallel from one source.** Both bodies see the same
edge, so both carry the same ~20 ms lag and differ only by unit-to-unit
variation:

```
Pi GPIO ─┬─▶ cam1 FOCUS/TRIGGER
             └─▶ cam2 FOCUS/TRIGGER          same edge, same ~20 ms lag
```

**Then gate the strobe on both cameras being open at once.** `EXPOSURE` is
active-low, so "both exposing" is *both pins low* — which is precisely a
**2-input NOR gate**. One part does all three jobs:

- **AND-logic:** output goes high only when both cameras are mid-exposure.
- **Inversion:** active-low in, active-high out.
- **Sourcing drive:** a push-pull high output is what a sourcing, rising-edge
  strobe input wants.

> **Superseded in the rig as built.** The NOR gate existed to answer "fire only
> when BOTH cameras are exposing" in hardware. The scheduled-GPIO topology answers
> it in software instead — the Jetson picks one target instant, both nodes fire
> against it, and the strobe is scheduled at that same instant — so no NOR gate is
> needed. See `docs/strobe-trigger.md` §4. The reasoning below on **not** wiring
> the two `EXPOSURE` pins together still stands and is worth reading.

> **Do not wire the two `EXPOSURE` pins together.** They are open-drain, so
> tying them to a shared pull-up gives a wired-OR of the pull-downs — the line
> goes low when *either* camera is exposing. That is the opposite of what you
> want. It needs a real gate.

**Run a slow shutter so the windows overlap.** `EXPOSURE` is only guaranteed
≥1 ms; if the two bodies' lags differ by more than that, short windows may not
overlap at all and the NOR gate never fires. Give yourself margin — a 1/30 s
exposure holds `EXPOSURE` low for ~33 ms, so several ms of inter-camera skew
still leaves a wide overlap. In a dark environment the ambient contribution over
those 33 ms is negligible, and the **flash duration alone sets the effective
exposure and the motion blur.** This is the classic open-flash technique and it
is the right answer here.

**Unresolved:** unit-to-unit variation in the ~20 ms lag is not specified. Two
bodies could differ by a few ms, and that number decides how slow the shutter has
to be. Measure both `EXPOSURE` lines on a two-channel scope against a common
`TRIGGER` edge before trusting any sync budget.

---

## 7. Building the camera harness

| Ref | Part | Maker / number |
|---|---|---|
| A | Connector housing | Molex `430250600` |
| B | Crimp contact | Molex `462355001` — **gold-plated** |
| C | Conductor | your choice, sized for ≥10 V at the camera |
| D | DC plug on the supplied cable | SMK `LGP0038-0100F` |

Crimp B onto C with a Molex tool, seat into A, repeat per pin, terminate the far
end to suit. The supplied cable's DC plug is **IEC 60130-10 (JEITA RC-5320A)
TYPE4**.

Sony repeats one warning in both sections: get `power (+)` / `GND (−)` / `FOCUS`
/ `TRIGGER` / `EXPOSURE` wrong and the failure modes include **malfunction,
smoke, or fire**. Ring out every pin before applying power. Unlatch the Micro-Fit
before pulling, and pull the connector body, not the cable.

---

## 8. The rig, end to end

Sony's own documented topology splits the interfaces by role — power/control
connector for power, `FOCUS`, `TRIGGER` and `EXPOSURE`; micro HDMI for live
view; USB Type-C for SDK control.

> **Updated.** This section originally drove the harness from the Jetson. The rig
> as built puts a Raspberry Pi at each camera: the Jetson schedules one shared
> absolute instant and each Pi busy-waits to it on a chrony-disciplined clock
> (~85 µs RMS), so the GPIO lines are local to each node. See `docs/PROTOCOL.md`.

Per camera node:

```
  Pi GPIO BCM17 ──▶ FOCUS   (pin 4)   assert ~120 ms ahead, per shot, then release
  Pi GPIO BCM27 ──▶ TRIGGER (pin 5)   pulse Low to fire
                                          │
                                    camera exposes
                                          │
  EXPOSURE (pin 6) ──▶ BCM22 (bias pull-up, gpiomon)
                                    hardware capture timestamp

  PoE ──▶ Pi;  12 V ──▶ camera pins 3 / 2   camera grounds tie at pin 2 only
```

Strobe, cam1 only — a Bolt VB-22 on its own isolated battery, fired from a
scheduled GPIO pulse rather than off `EXPOSURE`, so it is placed against the
instant BOTH cameras expose at:

```
  Pi GPIO BCM26 (open-drain, header pin 37) ──▶ sync CENTRE
  Pi GND        (header pin 39)             ──▶ sync SHELL
```

Four wins over the current USB-only design:

1. **Deterministic release** — GPIO instead of a PTP round trip removes the 0.5 s
   floor and the dependency on a healthy USB session.
2. **A real capture timestamp** — `EXPOSURE` tapped into a Jetson input gives a
   hardware-timed exposure event to align against IMU / DVL / NMEA 2000, far
   better than timestamping `OnCompleteDownload`.
3. **Sync that is actually synchronous** — the strobe fires off the shutter, not
   off software.
4. **Blur decoupled from shutter speed** — a 20–250 µs flash sets the effective
   exposure, which is what makes imaging from a moving platform work at all.

None of this displaces the SDK: exposure settings, live view, storage
destination, focus/zoom on the PZ lens and image transfer all stay on USB. The
harness is added underneath as the timing-critical path.

---

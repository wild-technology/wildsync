# Strobe trigger — implementation brief

Working notes for wiring and firing the **Bolt VB-22** on the stereo rig.
`docs/harness-and-strobe.md` remains the electrical reference for the camera
harness; its XR256 sections do **not** apply — that light is out of scope, and
none of its 24 V / 20 A constraints carry over.

**Status 2026-08-16:** every open decision below is now **settled by measurement**.
Nothing is coded yet; the wiring is fully specified and needs no components.

---

## 0. The decisions — all settled

**The strobe is the Bolt VB-22, triggered through its sync port, running on its
own isolated battery pack. It is fired from a dedicated Pi GPIO output on cam1,
not spliced onto the camera's `EXPOSURE` lead.**

| Question | Answer | Basis |
|---|---|---|
| Topology (§4) | **A — scheduled GPIO pulse** | fires against the shared target instant, so the flash is placed relative to BOTH bodies; puts no load on the capture-timestamp net |
| Direct GPIO or MOSFET (§2.1) | **Direct GPIO — no components** | sync port **measured at 2.9 V DC** open circuit, safely under the 3.3 V pad limit |
| Which pin (§3) | **BCM26 = header pin 37**, GND on pin 39 | plain GPIO, unclaimed, measured pull-DOWN at boot on the live Pi 5 |
| Where the schedule lives | `piagent`, via an optional `strobe_at_epoch` on `/gpio/fire` | absolute instants on the shared chrony-disciplined clock, in-process, no second round trip |

**An open-drain GPIO *is* a contact closure** — the same mechanism already driving
`FOCUS` and `TRIGGER`. The MOSFET was only ever needed to survive a sync voltage
above 3.3 V, and this one measures 2.9 V. Do not reintroduce it.

Wiring, in full:

```
cam1 Pi header pin 37 (BCM26, open-drain) ──► flash sync CENTRE / tip
cam1 Pi header pin 39 (GND)               ──► flash sync SHELL / sleeve
```

Set `gpio=26=ip,np` in `/boot/firmware/cmdline.txt` on cam1 first, so the pad boots
high-Z with no pull and the flash's own pull-up holds the line idle.

### What the VB-22 simplifies away

The whole isolation argument in `harness-and-strobe.md` §5.1 existed because the
XR256 is a 24 V industrial light pulling 20 A for 15 ms off a shared supply. A
battery-powered speedlight on a contact-closure sync port has none of that:

| Concern | XR256 | VB-22 |
|---|---|---|
| Trigger current | sizing unknown, sourcing 4–24 V | **a few mA, contact closure** |
| Ground bounce | 0.66 V resistive + ~2 V `L·di/dt` | negligible |
| Supply domain | separate 24 V rail, must not tie | **isolated battery, nothing to tie** |
| Opto required | yes, non-negotiable | **no** |
| Duty cycle | 2 %, preprogrammed | n/a |
| Polarity | sourcing, rising edge — opposite to `EXPOSURE` | **sinking closure — compatible** |

So this is now a small-signal problem, and the remaining decisions are about
timing and pin safety rather than power electronics.

---

## 1. Why not just tap `EXPOSURE`

With the XR256 out, the electrical objection largely evaporates — a contact
closure is exactly what an open-drain sinking output does, so `EXPOSURE` → sync
centre, camera pin 2 → sync shell is directionally correct and would work. Three
reasons still argue against it, and they are weaker than the ones that killed
the XR256 wiring, so weigh them rather than treating them as settled:

1. **It is a shared net, and the other end is the capture timestamp.**

   ```
   camera pin 6 (open-drain, sinks during exposure)
         └── Pi header 15 / BCM22, INPUT with --bias=pull-up,
             permanently-respawning gpiomon (piagent.py:564)
   ```

   Those falling edges *are* `capture_source=gpio_edge`. Any added load lifts the
   asserted low level (Sony's `A/(B+1)` model, ~1 kΩ on-resistance) and adds
   capacitance to a line that already rings (§7). Break it and you lose the
   timestamps, quietly.
2. **Sony publishes no absolute-maximum voltage or sink current for the
   `EXPOSURE` pin.** It is the one pin on this rig with no replacement path.
3. **It only sees cam1.** Scheduling from GPIO fires against the common target
   instant, so the flash is placed relative to *both* bodies.

Point 3 is the one that actually matters operationally. Points 1–2 are
mitigable with a high-impedance buffer if topology B wins.

---

## 2. Grounding — the direct answer

**Pi GND to the sync shell.** Not the PoE daughter board specifically.

Two reasons this is easy now:

- **The PoE board's 0 V and the Pi's 0 V are the same net** — the HAT feeds the
  Pi through the header's 5 V/GND pins. It was never a choice of reference, only
  of where in the current path you land the wire. At a few mA of trigger
  current, that path does not matter; land it on any convenient header GND.
- **The VB-22's battery pack is isolated**, so its sync return is floating with
  respect to everything else on the rig. Tying it to Pi GND is not merely safe,
  it is what *establishes* the reference the switching element needs. There is no
  competing ground and nothing to create a loop with.

Nothing about the strobe touches the PoE domain, so the camera/Pi shared-feed
coupling is not a factor here. (Do not attempt to power the flash from PoE
anyway — it has a battery, use it.)

### 2.1 Direct GPIO — settled, no components

**Measured 2026-08-16: centre-to-shell open circuit = 2.9 V DC.** That is under the
3.3 V pad limit, so the ESD clamp never conducts and the GPIO can pull the centre
pin directly. **No MOSFET, no optocoupler, no resistor.**

The framing that made this look like it needed parts was treating the GPIO as an
active-HIGH driver into a FET gate. It does not need to be: **an open-drain GPIO
output *is* a contact closure**, which is exactly what a sync port wants, and it is
the same mechanism already driving `FOCUS` and `TRIGGER` on this rig
(`piagent.py` `_LineDriver`, `IDLE`=1=high-Z, `ASSERT`=0=pulled low).

Request the strobe line **open-drain WITHOUT the `BIAS_PULL_UP` flag** `_LineDriver`
ORs in for the camera inputs — the flash supplies its own pull-up — and keep it out
of `Gpio.PARKED_INPUTS`, which exists for camera *inputs* and has different rules.

If the flash is ever swapped, re-measure the open-circuit voltage before wiring it.
Above 3.3 V a logic-level N-channel FET (2N7002/BSS138) with a 100 kΩ gate-to-source
pulldown is the answer, but that case does not apply to the VB-22.

Short-circuit current was not measured (no suitable meter). It is inferable from a
voltmeter alone: with the Pi's ~50 kΩ boot pull-down connected,
`R_pullup = 50k x (2.9 / V_measured - 1)` and `I_sc = 2.9 / R_pullup`. It only matters
for the boot-state question, which `gpio=26=ip,np` eliminates outright.

---

## 3. Pin selection

Only BCM **17**, **22**, **27** are in use (`piagent.py:48-50`). The IMU is
USB-serial (`imu_yb.py`), so no I2C or SPI competes. The header is otherwise
open.

The one thing that matters is the **boot-time pull state**, because this rig has
already been bitten by assuming a released pad is high-Z:

| Range | Default bias | Verdict |
|---|---|---|
| BCM 0–8 | pull-**up** | **Avoid.** |
| BCM 9–27 | pull-**down** | Safe default state. |

**Measured on the live cam1 Pi 5 (2026-08-16), reading each pad directly:**

```
BCM  5 → 1    BCM 12 → 0    BCM 19 → 0
BCM  6 → 1    BCM 13 → 0    BCM 20 → 0
              BCM 16 → 0    BCM 21 → 0
                            BCM 26 → 0
```

> **Correction.** An earlier revision of this table listed **BCM 5 and 6 as
> candidates, claiming all candidates default pull-down. They do not** — both sit
> HIGH at boot, exactly as the 0–8 rule above predicts. Wiring the strobe to either
> would hold the line at the boot bias before `piagent` ever runs. They are struck.

**Candidates: BCM 12, 13, 16, 19, 20, 21, 26.** All verified pull-down, none reserved.
Also unavailable on cam1: 7, 8 (spi0 CS), 17/22/27 (the camera harness), 32/34/44/46
(system).

**Chosen: BCM26 = physical header pin 37**, with GND on **pin 39** — adjacent in the
odd row, so it is one 2-way takeoff. BCM26 has no competing bus alt-function, and it
sits at the opposite end of the header from the camera harness block (pins 9/11/13/15),
so a mis-plugged strobe lead cannot land on `FOCUS`.

Stakes are lower than the `FOCUS` incident — a stuck trigger wastes flashes and
drains the battery, it does not brick anything — but the principle holds: choose
the pin so that **"unconfigured" and "off" are the same state**, rather than
relying on software to park it. With the direct open-drain wiring of §2.1 that is
achieved by `gpio=26=ip,np` on the kernel command line, which costs nothing.

---

## 4. The two topologies

### A — scheduled GPIO pulse (the current preference)

`rigd` already picks one absolute target instant `T` and tells each node to fire
at `T − that node's own latency`. cam1's Pi knows `T`, so it can busy-wait to
`T + δ` on the same chrony-disciplined clock and pulse the strobe pin — the same
mechanism that already drives `FOCUS`/`TRIGGER`.

- **Sees both cameras.** The flash is placed against the common target, not
  against one body's curtain. This is the real win, and it makes the NOR gate of
  `harness-and-strobe.md` §5.5 unnecessary.
- **Does not touch the `EXPOSURE` net** — no added load, no risk to the capture
  timestamp path.
- **Costs jitter.** The scheduled path holds sub-ms (trigger sd 0.4–0.9 ms), so
  δ needs margin. §4.1.

### B — hardware switch off cam1's `EXPOSURE`

- **Effectively zero jitter** — the flash is timed off cam1's actual curtain, and
  a xenon tube fires within microseconds of its trigger.
- **Blind to cam2**, which then relies on the inter-camera skew holding —
  currently 0.59 ms mean / 1.82 ms worst, but unverified at sustained rate with
  real archive settings.
- **Loads the `EXPOSURE` net** — see §1. A FET gate is a nearly ideal load
  (picoamps, a few pF), so this is far gentler than the XR256 case ever was.
  Verify BCM22 still reads a valid low with it in circuit.

Honest read: B is now defensible on electrical grounds and B's jitter is
genuinely better. A wins on *knowing about cam2*, and that is the argument to
weigh, not the wiring.

### 4.1 Picking δ, if you take A

The flash must land after the **last** camera's front curtain is fully open and
before the **first** camera's shutter closes.

| Term | Value | Source |
|---|---|---|
| `TRIGGER` → exposure start | 22.1 ms, sd 0.4–0.9 ms | measured, README |
| exposure start → curtain fully open | ~4 ms | Sony, corroborated 4.43/4.46/4.72 ms three ways |
| inter-camera skew | **0.59 ms mean, 1.82 ms worst** | measured 2026-08-16 after `FOCUS_LEAD_MS` 40→120; sustained-rate figure NOT re-measured at real archive settings |
| `EXPOSURE` assert width | ~13 ms on this rig | `piagent.py:611` |
| sync closure → light | microseconds, xenon | assumed — item 3 in §8 |

So **δ ≈ 8–12 ms after `T`** clears curtain travel plus realistic skew. The far
end is set by shutter speed: at 1/60 (16.7 ms) a δ of 10 ms leaves ~6 ms of
margin, which is thin. **Run 1/30 (33 ms) or slower.** In dark water that costs
essentially nothing — `harness-and-strobe.md` §4.1 puts ambient at 5.4 % at
1/60 — and the ~150 µs flash sets motion blur regardless of shutter.

### 4.2 The acceptance test is free

Both `EXPOSURE` edges are already captured and served — `fall` is curtain fully
open, `rise` is end of exposure, both with `epoch_hw`, both from
`GET /gpio/exposure/events`. The shutter-open window is therefore **directly
measured per node**, and the check is:

> strobe instant ∈ ⋂ over nodes of [ `fall`ᵢ , `rise`ᵢ ]

Prefer `epoch_hw` over `epoch` when comparing across nodes — the pipe-read
latency (median 0.09 ms cam1, 0.32 ms cam2, with excursions into hundreds of ms
under load) is pure uncorrelated skew error (`piagent.py:612-622`).

Any frame failing that check should have come back unlit, so it is a
self-validating log line. Worth wiring into the flight log rather than
eyeballing frames — **but see §6, it only catches timing failures.**

---

## 5. Before connecting anything

1. ~~Sync open-circuit voltage~~ — **done: 2.9 V.** See §2.1.
2. Set `gpio=26=ip,np` in `/boot/firmware/cmdline.txt` on cam1 and reboot, so the
   pad boots high-Z with no pull.
3. **Confirm which contact is which.** Centre/tip is the trigger, shell/sleeve
   the return, but ring it out rather than trusting the convention.
4. With the interface connected and the rig firing, re-check that BCM22 still
   reports a valid low during exposure and that `edges_bounced` in
   `GET /gpio/state` is not climbing — relevant to topology B, and cheap
   insurance under A.
5. First fire with the flash **pointed away and at low power**, and confirm one
   pulse per shot rather than a latched-on tube.

---

## 6. What this does not solve

- **Recycle caps the frame rate.** Bolt at 1/32 is ~0.5 s, so ~2 fps. That is
  the same neighbourhood as the rig's current ~0.57 s/frame, so the flash and
  the USB delivery path are now roughly co-equal ceilings — chasing one without
  the other buys nothing.
- **There is no ready signal.** The VB-22 exposes no electrical feedback, so
  firing before recycle completes yields an unlit or half-lit frame and nothing
  tells the Jetson. The §4.2 acceptance test will **not** catch this: the timing
  is perfect, the frame is just dark. Set the rate conservatively below recycle,
  and treat frame luma as the only available proxy.
- **Battery depletion is invisible too**, and is a genuine survey-duration
  constraint rather than a bench annoyance. Worth a pre-dive charge check in the
  operating procedure.
- **No remote power control.** Level is set on the flash body. The only remote
  levers are camera-side (ISO, aperture, shutter).

---

## 7. Related, and worth folding in while you are in here

Ringing on cam2's `EXPOSURE` lead is currently logged as "shield or shorten"
(carried into `docs/AI-HANDOFF.md`). The measured signature argues for a different order — and
it matters more if topology B wins, since B hangs another load on that net.

cam2 produced 14 events inside 62 µs (`piagent.py:606`) — ~4.4 µs per
transition. Against the candidates:

| Mechanism | Timescale | Fits? |
|---|---|---|
| Transmission-line reflection, ~2 m cable | ~20 ns round trip | no — 200× too fast |
| Cable LC resonance (~2 µH, ~200 pF) | ~0.1 µs | no |
| RC ramp: Pi's ~50 kΩ internal pull-up × ~200 pF cable | **τ ≈ 10 µs** | **yes** |

That is a **slow, high-impedance rising edge**, not a reflection — so shielding,
which addresses coupling, is not the first fix. Suggested order:

1. **External pull-up, 4.7–10 kΩ from BCM22 to 3.3 V.** One resistor, 5–10×
   stiffer node. Already sized in `harness-and-strobe.md` §4: 10 kΩ → 0.30 V
   low, 4.7 kΩ → 0.58 V, both under V<sub>IL</sub>. Camera sinks 0.7 mA at
   4.7 kΩ. Leave `--bias=pull-up` alone; the internal one parallels harmlessly.
2. **Twisted pair** — `EXPOSURE` twisted with its own return to harness pin 2.
   Beats a shield against magnetic coupling, costs nothing.
3. **Shorten the lead.**
4. **Then shield**, if 1–3 do not finish it. Ground at **one end only** (the Pi),
   and never as the signal return.

Not urgent: the 1 ms debounce keeps the **first** edge of a burst, which is the
true capture instant, and the self-extending dead-time bug is already fixed
(`piagent.py:629`). Timestamps are not currently wrong. This is robustness.

---

## 8. Still unknown

1. ~~Sync port open-circuit voltage.~~ **Answered: 2.9 V DC.** Short-circuit current
   still unmeasured, but §2.1 explains why it no longer blocks anything.
2. **Recycle time at the working power setting.** Quoted 0.1–5.2 s across the
   range, ~0.5 s assumed at 1/32. Sets the real frame-rate ceiling and wants
   measuring at whatever power the survey actually uses.
3. **Sync closure → light latency and jitter.** Assumed negligible for xenon;
   unverified. Only matters if δ margin gets tight.
4. **Whether δ can be a constant**, or needs to track the per-node
   `calibrate_trigger` result the way `TRIGGER` already does. Depends on how
   stable the ~22 ms release lag is body-to-body over a session.
5. **Flashes per charge** at the working power, for survey planning.

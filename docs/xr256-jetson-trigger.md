# XR256 from the Jetson — hardware requirements and testing plan

Research brief for triggering the **Smart Vision Lights XR256** strobe from a
**Jetson Orin Nano** acting as the rig's command/control computer, so that one
flash exposes both ILX-LR1 bodies on sync.

> **Scope change.** `docs/harness-and-strobe.md` and `docs/strobe-trigger.md`
> declared the XR256 out of scope in favour of the Bolt VB-22. This brief
> reverses that: the XR256 is back in scope, and the trigger moves off cam1's Pi
> onto the Jetson — wiring anything to either Pi is difficult in the field. The
> VB-22 analysis is not wasted: the *topology* it settled carries over intact.
> Datasheet: `docs/XR256_Datasheet.pdf` (SVL, Rev 2020/06/15).

**Status 2026-08-26:** research only. Nothing wired, nothing coded. The two
mandatory hardware items are a **trigger interface board** (a bare Jetson GPIO
cannot trigger this light) and a **regulated 24 V supply** sized for the
light's recharge bursts.

---

## 0. What carries over, and what changes

The scheduled-pulse topology ("A") chosen in `strobe-trigger.md` §4 is exactly
what the Jetson wants: `rigd` already picks one absolute target instant `T`,
each Pi fires its camera at `T − that body's measured lag`, and the strobe is
scheduled at `T + δ` on the same disciplined clock. The software contract for
that exists and is tested:

| Piece | Where |
|---|---|
| Scheduled strobe in the fire path | `strobe_at_epoch`/`strobe_pulse_ms` on `POST /gpio/fire` |
| Standalone strobe (camera-fault tolerant) | `POST /gpio/strobe` (`rig/piagent.py:803`) |
| Host config + validation | `set_strobe`, `rig/run.py:1343-1440`, `~/rig/strobe.json` |
| Acceptance check | strobe ∈ ⋂ over nodes of [`fall`ᵢ, `rise`ᵢ] using `epoch_hw` (`rig/rigcore.py:2191-2261`), LIT / STROBE MISS chips in the UI |
| δ and shutter guidance | δ ≈ 8–12 ms after `T`, shutter **1/30 or slower** (`strobe-trigger.md` §4.1) — unchanged, the physics didn't move |

What changes:

| | VB-22 on cam1's Pi | XR256 on the Jetson |
|---|---|---|
| Trigger electrical | contact closure, 2.9 V, direct open-drain GPIO, zero components | **sourcing PNP, 4–24 V rising edge** — needs a driver stage, opto isolation |
| Power | isolated battery, nothing to tie | **24 VDC ±5 %, bursts to 20 A for 15 ms** — a real supply and a real grounding question |
| Recycle | ~0.5 s → ~2 fps ceiling | none in practice (5 000 strobes/s capable) — frame-rate ceiling moves back to the USB delivery path |
| Duration | set on the flash body, ~150 µs at 1/32 | 20–1000 µs by dial (pulse-initiated) or by trigger pulse width (pulse-following) |
| Failure feedback | none (dead battery invisible) | over-temp gating (SafeStrobe) is the residual silent failure; power/temp indicator LEDs exist but live inside a housing |
| Scheduling clock | cam1 Pi, chrony | Jetson — which, as host, is the natural chrony *reference* |

---

## 1. What the datasheet dictates

| Parameter | Value | Consequence |
|---|---|---|
| Electrical input | **24 VDC ±5 %** (22.8–25.2 V) | raw battery packs are out of tolerance across their charge curve — a regulated converter is required |
| Input current | **max 20 A for max 15 ms** | the recharge burst after a strobe; supply + bulk caps must ride it out inside the ±5 % window |
| Pulse energy | up to 2000 W while LEDs active (180 A die current) | max ~2 J per pulse (2000 W × 1 ms) |
| Duty cycle | **max 2 %**, SafeStrobe enforced | at 2 fps survey cadence with a 500 µs pulse, actual duty ≈ 0.1 % — no constraint |
| Pulse-initiated (PI) mode | fires on **rising edge of a sourcing PNP input, 4–24 V**; duration set by the 8-position dial, 20–1000 µs; trigger-to-light delay **1 µs** | 3.3 V GPIO is below the 4 V floor — cannot trigger it directly, ever |
| Pulse-following mode | NPN (sinking) or PNP (sourcing); light tracks the pulse width, max strobe 40 ms | duration becomes a software quantity — see §3 |
| Dial "Auto" position | light free-runs with no trigger | **never leave the dial on Auto** on this rig |
| Connection | 5-position screw terminal block (plug included): 24 V / NPN / PI / PNP / GND | |
| Ingress | **IP50** | not waterproof; underwater use means a pressure housing with an optical port |
| Thermal | LED die monitor, shuts down above 80 °C, cool-down mode | trivial at survey duty; matters in a sealed housing only under abuse |
| Eye safety | white/470/530 are IEC 62471 **Risk Group 1** | do not stare; bench-test pointed away, same rule as the VB-22 |

Unknowns the datasheet does not answer (bench items): PI input current draw,
actual recharge burst profile at the working dial setting, and which
wavelength/lens variant is on hand (part number on the light body).

---

## 2. Trigger interface — the mandatory board

Two facts make a bare Jetson pin a non-starter:

1. **Amplitude.** The PI input wants ≥4 V sourced; the header is 3.3 V logic.
2. **Drive.** The Orin Nano devkit's 40-pin header sits behind level shifters
   good for roughly **1 mA** — signal-only pins. Even an opto LED (5–10 mA) is
   too much load; buffer everything.

And one fact from the original XR256 analysis still stands
(`strobe-trigger.md` §"What the VB-22 simplifies away"): a light pulling 20 A
bursts on its own 24 V rail produces real ground bounce (estimated 0.66 V
resistive + ~2 V `L·di/dt`), so **galvanic isolation between the Jetson and
the 24 V domain is non-negotiable**. The opto that was "removed" for the VB-22
comes back.

### 2.1 Reference design

```
 Jetson side (3.3/5 V domain)          │           light side (24 V domain)
                                       │
 header GPIO ──R──▶ NPN/N-FET ──▶ opto LED (+R, ~10 mA from header 5 V)
                                       │
                                  optocoupler
                                       │
                        +24 V ──▶ opto collector
                                  opto emitter ──▶ PI terminal (pin 3)
                                  PI terminal ──R(4.7 kΩ)──▶ GND (pin 5)
```

- The opto in emitter-follower configuration **sources** the trigger: output
  high ≈ 23 V, far above the 4 V floor; the pull-down defines the low state and
  the falling edge.
- The 24 V return (terminal pin 5) is the light's own; **it never ties to
  Jetson ground**. The opto is the only crossing.
- Speed: the timing budget is millisecond-scale (δ margin, camera skew), so
  even a PC817-class opto (µs–tens of µs delays) is adequate **for edge-
  triggered PI mode**. A faster logic opto (H11L1/6N137 class, plus a small
  PNP high-side stage) removes the question entirely and is what pulse-
  following mode would require — see §3.
- Run the trigger as a **twisted pair** (signal + its return). A 24 V-swing
  signal over twisted pair is about as noise-immune as this gets; shielding
  per `strobe-trigger.md` §7 rules only if it proves necessary.

### 2.2 Jetson pin selection

Same discipline as `strobe-trigger.md` §3, new silicon:

- Pick a plain header GPIO with no competing alt function; set it as GPIO in
  the pinmux (`jetson-io` / pinmux spreadsheet for the Orin Nano devkit).
- **Measure the boot-time state of the candidate pin** across a power cycle
  before wiring — the FOCUS incident rule: *unconfigured and off must be the
  same state*. Stakes are lower here (a boot glitch through the opto fires one
  wasted flash; SafeStrobe prevents anything worse), but the principle holds.
- Optional but attractive: Tegra234 has a hardware timestamp engine (GTE on
  JetPack 5, upstream **HTE** on JetPack 6) that can hardware-timestamp GPIO
  edges — the AON-domain pins and LIC interrupt lines. If the chosen pin (or a
  loopback of the trigger into a second pin) is timestampable, the flight log
  gets a hardware-stamped strobe instant to drop into the existing
  `epoch_hw`-based acceptance check, closing the loop the way the Pis'
  `gpiomon` does for `EXPOSURE`. Verify domain coverage for the specific pin;
  otherwise a scope during bring-up is fine.

---

## 3. Mode choice: pulse-initiated vs pulse-following

**Recommendation: wire for PI mode (PNP terminal), set duration on the dial.**

| | Pulse-initiated | Pulse-following |
|---|---|---|
| Trigger needs | one clean rising edge; width of the GPIO pulse is irrelevant | the pulse **is** the flash — width must be accurate to tens of µs |
| Duration control | dial on the light body — inside the housing, not remotely adjustable | software — remotely adjustable per shot |
| Duration accuracy | hardware-exact, 8 fixed steps 20–1000 µs | at the mercy of a userspace busy-wait: 50–200 µs of preemption stretch on a 250 µs target is a 20–80 % exposure error, frame to frame |
| Software hazard | none — existing `strobe_pulse_ms: 5` default works as-is | the same 5 ms default becomes a 5 ms flash: ~14 px of motion blur and 10× the intended light |

Pulse-following's remote-duration appeal is real for a light sealed in a
housing, but doing it properly means a hardware one-shot (Tegra PWM or a
555-class monostable on the interface board), not a Python-timed pulse. Ship
PI mode first; treat pulse-following as a later upgrade with its own bench
pass. The §2.1 board wires to the PNP-capable terminals either way, so the
upgrade is a terminal-block and software change, not a new board.

### Dial setting — blur vs light

Measured rig scale: 0.15 mm of subject motion = 0.42 px at 2 m
(`harness-and-strobe.md` §4.1), i.e. **1 px ≈ 360 µs at 1 m/s**.

| Dial | Blur @ 1 m/s | Pulse energy (max) | Note |
|---|---|---|---|
| 100 µs | 0.28 px | 0.2 J | VB-22-like freeze, least light |
| **250 µs** | **0.7 px** | **0.5 J** | closest match to the VB-22's measured ~150 µs / 0.42 px working point |
| 500 µs | 1.4 px | 1.0 J | double the light, blur exceeds 1 px |
| 1000 µs | 2.8 px | 2.0 J | only if stationary or light-starved |

Whether 0.5 J at the survey geometry actually exposes the frame at acceptable
ISO is **the** open photometric question — §6 test 11 answers it before
anything is committed. The XR256's 2 J ceiling is speedlight-at-low-power
territory; if the water wants more light, the answer is aperture/ISO or a
second light, not a longer pulse.

---

## 4. Power — the 24 V rail

- **Regulated 24 V ±5 %.** The window (22.8–25.2 V) excludes raw 6S/7S Li-ion
  and 8S LiFePO4 across their charge curves. Use a regulated DC-DC from the
  rig bus, or a fixed 24 V supply on the bench (≥350 W class to shrug off the
  burst).
- **Ride out the recharge burst.** Datasheet worst case 20 A × 15 ms = 0.3 C.
  Put **bulk capacitance at the light end** of the cable so the cable carries
  average current, not the burst — at 20 A even 6 m of 16 AWG round trip drops
  ~1.6 V, which alone busts the ±5 % window. Sizing the caps/converter split
  is a bench measurement (§6 test 3), not a datasheet calculation: the 20 A
  figure is a max, and the real draw at a 250 µs dial setting will be far
  smaller.
- **Average power is trivial**: ~1 W at 2 fps × 0.5 J. This is a transient
  problem, not a heat problem.
- Fuse the 24 V feed; single-point the light-side ground at the terminal
  block; nothing from this domain touches the Jetson, the Pis, the PoE switch,
  or camera pin 2.

---

## 5. Software work list

From the code as it stands on `jetson-port` (which, despite the name, contains
no Jetson-specific code — the branch predates the Mac port):

1. **Strobe node concept.** `set_strobe` validates `node` against the camera
   fleet (`rig/run.py:1384-1390`). Register the Jetson as a camera-less node in
   `~/rig/nodes.json` (`rig/rigcore.py:44-74`) or add a `role: strobe` concept.
2. **A `/gpio/strobe`-compatible endpoint on the Jetson.** A minimal
   piagent subset satisfies the existing contract with zero host-side protocol
   changes. `BCM_STROBE` is already env-configurable (`rig/piagent.py:76`),
   but `_LineDriver` resolves its chip by "pinctrl chip with ≥40 lines" —
   Tegra's gpiochip enumeration differs and needs a port.
3. **Health gating for a camera-less node.** The standalone strobe path is
   suppressed when the strobe node is OFFLINE (`rig/run.py:2425-2438`), and
   `NodeMonitor` also polls `ilxctl` on :8080 (`rig/rigcore.py:431`). A node
   with no camera lands in `ILX_DOWN`; make sure that state doesn't suppress
   the flash.
4. **Clock topology.** `at_epoch` is a node-clock instant and the pulse is a
   local busy-wait (`rig/piagent.py:842`). Make the Jetson the **chrony
   reference** the Pis discipline to (as the retired Jetson was — ~85 µs RMS
   then); `host_offset` collapses toward zero and the strobe's schedule needs
   no cross-domain correction. An undisciplined third clock domain puts its
   full offset straight into δ — this is the failure mode the Mac host already
   demonstrated (187 ms, `late_ms ≈ 33 ms` on every fire).
5. **Scheduling quality on Tegra.** Busy-wait at SCHED_FIFO as piagent does;
   verify the Jetson's `late_ms` distribution under survey load matches the
   Pis' sub-ms behaviour (§6 test 7).
6. **Docs.** `harness-and-strobe.md` and `strobe-trigger.md` both declare the
   XR256 out of scope; revise once this brief is accepted. `FIELD-RUN.md`
   known-limits (§"No strobe yet", recycle ceiling, battery telemetry) change
   materially with this light.

The shutter conflict flagged in FIELD-RUN carries over unchanged: the fleet
default is 1/200, strobe work wants **1/30 or slower**; `set_strobe` already
warns.

---

## 6. Testing plan

Phased; each phase gates the next. Light pointed away or covered until test 9.

### Phase 0 — bench electrical (no cameras, no rig)

1. **First power.** Current-limited bench supply at 24.0 V, nothing on the
   trigger terminals. Green power LED; record the part number (wavelength /
   lens). Confirm terminal pinout against the datasheet before landing wires.
2. **First light.** Function generator (not the Jetson) into the §2.1 board:
   one flash per rising edge at each dial setting; no double-fires; dial
   never on Auto. Scope the PI pin: amplitude ≥4 V, clean monotonic edge.
3. **Burst draw.** Current probe or shunt on the 24 V feed while strobing at
   the working dial setting; capture the recharge profile. This measurement —
   not the 20 A datasheet max — sizes the converter and the light-end bulk
   caps. Verify the rail stays inside ±5 % during a 2 fps run.
4. **Latency and duration.** Photodiode + scope: trigger edge → light (spec
   1 µs; the opto adds its µs-scale delay — record it, it folds into δ), and
   actual optical pulse width per dial step.
5. **SafeStrobe behaviour.** Deliberately exceed 2 % duty; observe the gating
   and the yellow LED so the field signature of an over-temp shutdown is
   known before it happens in a housing underwater.

### Phase 1 — Jetson bring-up (light covered)

6. **Pin audit.** Boot-state measurement of the candidate GPIO across power
   cycles with the opto board connected: zero spurious flashes from cold boot,
   reboot, and agent restart. Pinmux set via jetson-io; document as
   `strobe-trigger.md` §3 did for BCM26.
7. **Scheduling accuracy.** Port the strobe-only piagent subset; fire against
   scheduled instants and measure the Jetson's `late_ms` distribution under
   realistic load (ingest running, UI polling). Target: the same sub-ms
   envelope as the Pis (trigger sd 0.4–0.9 ms). Scope or HTE loopback for
   ground truth.
8. **Clock discipline.** Jetson as chrony server, Pis re-pointed from orphan
   mode to it; `chronyc` offsets ≤1 ms sustained; `host_clock_offset` and
   `node_clock_skew` anomalies quiet over an hour.

### Phase 2 — integration (cameras, dark room)

9. **One live fire, pointed away** — the same single proof FIELD-RUN already
   demands for the VB-22, now from the Jetson path end to end
   (`/api/strobe` → schedule → opto → flash).
10. **Acceptance sweep.** Both cameras at 1/30, strobe enabled: verify the LIT
    chip and `strobe ∈ ⋂ [fallᵢ, riseᵢ]` per shot; sweep δ across 6–14 ms and
    chart the margin; confirm zero banding (the 1/200 X-sync ceiling and
    ~4.4 ms curtain transit apply to any short flash).
11. **Photometry.** Frames at survey geometry (2 m, survey aperture) across
    dial settings and ISO; A/B against saved VB-22 bench frames. Output: the
    chosen dial setting and ISO, and a verdict on whether 0.5 J is enough
    light — the go/no-go for this light as the sole illuminator.
12. **Rate ceiling.** With recycle gone, run the transect interval down from
    0.5 s until the delivery path, not the strobe, is the limiter; record the
    new ceiling and confirm duty stays ≪2 %.
13. **Soak.** 30–60 min at survey cadence: LIT rate, skew, `late_ms`, rail
    voltage, light body temperature. No SafeStrobe gating, no missed frames.

### Phase 3 — field

14. **Housing integration**: pressure housing with optical port, penetrators
    for the 24 V pair and trigger pair, bulk caps inside the housing at the
    light. Re-run test 10's acceptance sweep assembled.
15. **Procedure updates**: pre-dive check swaps the VB-22 battery check for a
    24 V rail check; the ambient-check frame (strobe disabled at candidate
    shutter) carries over; dial position (not Auto, chosen step) added to the
    checklist since it's unreachable once sealed.

---

## 7. Open questions

1. **Which XR256 variant is on hand** — wavelength and lens angle set the
   light budget and beam coverage vs the camera FOV at 2 m.
2. **Is ~0.5 J per pulse enough light** at survey geometry and acceptable ISO
   (§6 test 11). The single question that decides whether the XR256 works as
   the sole illuminator or needs a partner.
3. **PI input current** — unspecified; measured in §6 test 2, sizes the opto.
4. **Actual recharge burst** at the working dial setting (§6 test 3) — sizes
   the converter and caps.
5. **Rig 24 V source** — what DC bus exists on the platform to convert from.
6. **δ constant vs tracked** — carried over from `strobe-trigger.md` §8:
   whether δ should track `calibrate_trigger` per session. Unchanged by the
   light swap.
7. **HTE/GTE coverage** of the chosen header pin on JetPack 6 — nice-to-have
   for a hardware-stamped strobe instant, not a blocker.

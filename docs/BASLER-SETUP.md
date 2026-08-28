# Basler a2A4504-18umBAS on the cam3 Pi — setup and first flight

Build sheet and standalone test regime for the **Basler a2A4504-18umBAS**
(USB3 Vision, 20.2 MP mono, global shutter) running on **cam3's Raspberry
Pi 5** under **pypylon**. Everything here happens *outside* wildsync — no
rigd, no piagent involvement beyond wiring rehearsal — and gates the
integration plan in `docs/basler-cam3-node.md`.

Cross-references, not duplicates:

| For | Read |
|---|---|
| Bare SD card → running Pi node (flash, cloud-init, subnet trap, chrony slot 3, PoE) | `docs/PI-SETUP.md` |
| Making this camera a fleet member, and the strobe question | `docs/basler-cam3-node.md` |
| Full camera spec | `docs/a2A4504-18umBAS \| Basler Product Documentation.pdf` + the offline mirror `docs/basler-product-documentation-v133-en/` |

Assumes a Pi built through `PI-SETUP.md` §2–§5 (Ubuntu 24.04 Server arm64,
Python 3.12, on the rig switch as `pi-cam3` / 192.168.1.203). Skip
`PI-SETUP.md` §1's ILX items (body, harness to a Sony port, body SD card) —
this node has no ILX.

---

## 1. pylon vs pypylon — what the download is and what pip installs

Two packagings of the same runtime; you want **both**, for different jobs.

- **`pylon-26.08.1_linux-aarch64_setup.tar`** (the ~844 MB download) is the
  **pylon Software Suite**: the pylon runtime, `pylonviewer` (Qt GUI),
  `PylonFirmwareUpdater`, the C++/.NET SDKs and samples, the camera emulator,
  and — the part pip can never give you — **`setup-usb.sh`**, which installs
  the udev rules (`69-basler-cameras.rules`) that let a non-root user open a
  USB3 Vision camera. Installs to `/opt/pylon`.
- **pypylon** is Basler's official Python binding, installed with pip. The
  wheel **bundles its own copy of the pylon C++ runtime**, so Python code
  never touches `/opt/pylon`. aarch64 wheels exist for Python 3.9–3.14
  (manylinux_2_31 — Ubuntu 24.04's glibc qualifies).

Version pairing: current pypylon (26.7) bundles pylon 26.7.2; the suite
download is 26.08.1. **The mismatch is harmless** — the wheel uses its
bundled runtime, `/opt/pylon` serves the Viewer/udev/firmware side. Only a
from-source pypylon build would care about `PYLON_ROOT`.

So: the download was the right thing, it just isn't the *Python* thing.
Minimal viable node = pip wheel + the suite's `setup-usb.sh`. Install the
whole suite anyway; the Viewer and firmware updater earn their disk on the
bench.

---

## 2. What you need beyond the PI-SETUP parts list

| Item | Note |
|---|---|
| The camera | a2A4504-18umBAS: IMX541 (1.1", 2.74 µm), 4504×4504 default, mono, **global shutter**, 17.7 fps (19 with the throughput limit off). Needs pylon ≥ 6.0 — 26.x is fine. |
| **USB 3.0 Micro-B cable with screw lock** | Must be a true SuperSpeed cable into one of the Pi 5's **blue** USB3 ports. The camera is **bus-powered only** (~3.4 W ≈ 680 mA typical at 5 V; no aux power pin exists). |
| **M8 6-pin A-coded I/O cable, open-ended** | For the trigger/exposure harness (§5) and later the strobe work. Shielded twisted pair, ≤10 m. Note the ace 2 connector is **M8, not Hirose** — don't buy the 6-pin Hirose cable the older ace uses. |
| C-mount lens covering **1.1" format** | The image circle must reach 17.5 mm; a 2/3" lens vignettes hard on this sensor. |
| A metal mounting bracket | The bracket **is** the heatsink. Housing limit is 60 °C operating; the body is IP30 and will need an enclosure for the boat, which makes conducted cooling the only cooling. |

### 2.1 Power — the 680 mA problem

The camera's ~680 mA typical draw collides with the Pi 5's **default 600 mA
total downstream-USB budget**. A Pi that can't negotiate a 5 A USB-C supply
(which a PoE-HAT-powered Pi never can) stays at 600 mA and the camera will
brown out or flap off the bus.

```bash
# on the Pi: force the 1.6 A USB budget regardless of supply negotiation
echo 'usb_max_current_enable=1' | sudo tee -a /boot/firmware/config.txt
```

Do this **only** on a supply that can actually deliver it — the official
27 W PSU, or a PoE HAT whose 5 V rail is rated ≥5 A (an 802.3at/25 W-class
HAT is; check the specific HAT). If the camera still drops off the bus under
load, put it behind a **powered** USB3 hub and stop fighting the budget.
`vcgencmd get_throttled` must read `0x0` throughout first flight — the same
undervoltage discipline `PI-SETUP.md` §8 already imposes.

---

## 3. Install

### 3.1 pylon Suite (udev rules, Viewer, tools)

From the `INSTALL` file in the tar, verbatim procedure:

```bash
mkdir ~/pylon_setup && tar -C ~/pylon_setup -xf pylon-26.08.1_linux-aarch64_setup.tar
sudo mkdir /opt/pylon
sudo tar -C /opt/pylon -xzf ~/pylon_setup/pylon-*.tar.gz
sudo chmod 755 /opt/pylon
sudo /opt/pylon/share/pylon/setup-usb.sh
```

`setup-usb.sh` installs the udev rules, raises the open-file limit, and sets
`usbfs_memory_mb` — but its **persistence step edits GRUB, which the Pi does
not have**. Expect that step to fail or no-op; §3.2 does it the Pi way.
After the script: **unplug and replug the camera** — udev rules are not
retroactive.

### 3.2 usbfs memory — not optional on this camera

The kernel default of 16 MB of usbfs buffer memory cannot hold even one
20.3 MB Mono8 frame's worth of URBs; the failure signature is
`Failed to submit transfer status=0xe2100001`. Basler's own script uses
1000 MB:

```bash
# immediately:
echo 1000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
# persistently — append to the single kernel line (do not add a new line):
sudo sed -i '1 s/$/ usbcore.usbfs_memory_mb=1000/' /boot/firmware/cmdline.txt
```

Verify after a reboot: `cat /sys/module/usbcore/parameters/usbfs_memory_mb`.

### 3.3 Real-time priority for the USB transfer threads

```bash
echo '*  -  rtprio  99' | sudo tee /etc/security/limits.d/90-pylon-rtprio.conf
```

Re-login. This lets pylon's transfer loop elevate; without it grabs still
work but drop frames more readily under load.

### 3.4 pypylon

Ubuntu 24.04 is PEP-668 externally-managed; use the venv you'd need anyway:

```bash
sudo apt-get install -y python3-venv
python3 -m venv ~/pylon-venv
~/pylon-venv/bin/pip install pypylon numpy opencv-python-headless pillow
```

### 3.5 Verify enumeration

```python
from pypylon import pylon
devs = pylon.TlFactory.GetInstance().EnumerateDevices()
for d in devs:
    print(d.GetModelName(), d.GetSerialNumber())
```

One device, as the normal user, not root. Zero as user but one under `sudo`
means the udev rules didn't take — replug, then check `/dev/bus/usb`
permissions. Record the **serial number**: production code selects by serial,
never by "first found".

No-hardware rehearsal: `export PYLON_CAMEMU=2` conjures two emulated cameras
— useful for exercising scripts before the camera arrives or away from the
rig.

---

## 4. Baseline camera configuration

The survey configuration, set once and saved to a user set so it survives
power cycles:

| Parameter | Value | Why |
|---|---|---|
| `PixelFormat` | `Mono8` | The rig spools JPEG; 8-bit is what gets encoded. Mono10/12(p) exist for bench photometry. |
| Resolution | default 4504×4504 | Full frame. |
| `ExposureTime` | per shot plan (range 16 µs – 10 s) | Strobe work wants the window that covers `T+δ`, same logic as the ILX 1/30 rule. |
| `Gain` | 0–48 dB (analog to 24, digital above) | Start at 0, raise for photometry tests. |
| `DeviceLinkThroughputLimit` | leave **On** (17.7 fps cap) | A deliberate throttle that suits a small host; the rig cadence is ~2 fps anyway. |
| `TriggerMode` (FrameStart) | Off for Phases 1–3, On from Phase 4 | |

Save: `UserSetSelector=UserSet1`, `UserSetSave.Execute()`,
`UserSetDefault=UserSet1`. Confirm by power-cycling and reading back.

Two numbers to record from the live camera and keep with the node docs:
`BslExposureStartDelay` (the fixed trigger→exposure latency, ~112 µs at
10-bit sensor depth) and the device temperature node reading at idle on the
bench (baseline for the thermal soak).

---

## 5. Compression — mandatory, and it happens on the Pi

A raw Mono8 frame is **20.3 MB**. Left uncompressed, the spool→pull
pipeline drowns at exactly the survey cadence it exists to serve:

| | Raw Mono8 | JPEG mono, q90 (est. 2–4 MB, scene-dependent) |
|---|---|---|
| 2 fps write rate | 41 MB/s | ~4–8 MB/s |
| One hour of host outage | **146 GB** — a 64 GB card dies in ~16 min | 14–29 GB — hours of buffer |
| `spool_not_draining` alarm (400 frames) | 8.1 GB already on the card | ~1.2 GB |
| Pull over the rig switch at 2 fps | ~41 MB/s sustained, contending with the SD's own writes | trivial |

So frames are encoded to JPEG **before** they touch the SD card, in
`baslerctl`'s grab loop. JPEG is also what the rig contract wants: the host
validates SOI/EOI on `.jpg`, `FRAME_EXTS` gates frame serving, and rigcal
only detects on JPEG bytes — compressed-but-exotic formats (PNG, WebP,
JPEG XL) buy nothing and break the contract.

### 5.1 What the camera can't do for you

**In-camera compression (Compression Beyond) is not on this model.** It is
an ace 2 **Pro**-tier feature (FPGA entropy coding, lossless or fixed-ratio
lossy, decompressed host-side via pylon's `ImageDecompressor`); the
BAS(ic) tier omits it. Two consequences:

- If the body is not yet committed, the **a2A4504-18umPRO** adds it — but
  note what it actually buys here: less USB-link bandwidth and higher
  attainable fps. The output is Basler's own compressed payload, which the
  rig can't spool, so the JPEG encode step survives either way. At a 2 fps
  survey cadence the link isn't the bottleneck; the Pro upgrade is not
  justified by compression alone.
- **The Pi 5 has no hardware JPEG encoder.** The VideoCore JPEG block that
  existed through Pi 4 was dropped; every encode is CPU, via libjpeg-turbo
  (NEON). Don't plan around a hardware assist that isn't there.

### 5.2 Encoder and budget

- Use **libjpeg-turbo**: either OpenCV's `imencode(".jpg", …)` (the
  `opencv-python` wheel bundles it) or **PyTurboJPEG** with `TJSAMP_GRAY`
  (thinner Python overhead, explicit grayscale path — preferred). Skip
  Pillow-SIMD: x86-only.
- Ballpark for 20.2 MP grayscale on a Cortex-A76 core: **on the order of
  150–400 ms/frame single-threaded**. That envelope is wide because it must
  be — Phase 5 measures the real number on real scene content. At 2 fps a
  single worker may sit at 30–80 % of one core; run a **2-worker encode
  pool** feeding from the grab queue so encode never blocks
  `RetrieveResult`, and the Pi 5's four cores carry it with margin.
- Baseline **quality 90**, swept q85–95 in Phase 5 against file size and
  checkerboard detectability. Grayscale JPEG has no chroma to subsample —
  quality maps almost directly to luma quantization, so q85 is a real
  size lever if needed.
- JPEG is 8-bit: the spool path is Mono8 by design (§4). Photometry bench
  shots that want Mono10/12 are saved raw as deliberate one-offs outside
  the spool.

### 5.3 If that still isn't enough

The levers, in order: **binning 2×2** (sensor or FPGA, Sum/Average →
2252×2252 = 5.05 MP — 4× off every number above, plus SNR, if cam3's role
doesn't need 20 MP), then ROI, then cadence. All are camera-side and
shrink encode CPU, SD writes, and pull traffic together. The spool stays
on the SD card — it is the host-outage buffer, and RAM (tmpfs) forfeits
frames on power loss with no camera-card backup tier behind it.

---

## 6. The I/O harness — three wires, no components

The wiring deliberately mirrors the ILX harness so piagent's GPIO contract
carries over untouched (see `basler-cam3-node.md` §0). Camera side is the
M8 6-pin; Pi side is the same BCM pins `PROTOCOL.md` assigns every node.

| Pi BCM | Dir | M8 pin | Camera line | Config | Meaning |
|---|---|---|---|---|---|
| 27 (TRIGGER) | out, open-drain | 4 | Line 2 (GPIO) | `LineSelector=Line2, LineMode=Input`; `TriggerSelector=FrameStart, TriggerSource=Line2, TriggerActivation=FallingEdge` | Pi pulses LOW ≥1 ms → camera fires on the falling edge |
| 22 (EXPOSURE) | in, bias pull-up | 5 | Line 3 (GPIO) | `LineSelector=Line3, LineMode=Output, LineSource=ExposureActive` | Camera holds LOW while exposing → **fall = exposure start, rise = end**, same polarity as the ILX harness |
| GND | — | 6 | GPIO ground | | Common ground, Pi ↔ camera only |

Pins 1 (reserved), 2 (Line 1 opto input) and 3 (opto ground) stay
unconnected. BCM 17 (FOCUS) connects to nothing — the Basler has no
half-press.

Why this works with zero external parts: the camera's GPIO lines carry an
internal ≈650 Ω pull-up to 3.3 V, so the Pi's open-drain low is a clean
assertion (≈5 mA sink, well inside the Pi's limit) and release restores a
legal high (>2.0 V threshold). In the other direction the camera output is
an open-collector; `gpiomon --bias=pull-up` (how piagent already watches
BCM 22) supplies the 3.3 V and the camera sinks it during exposure. GPIO
propagation is <1 µs falling / <2.5 µs rising with <100 ns camera-inherent
jitter — an order of magnitude tighter than the opto input, which is why
Line 2 gets the trigger and Line 1 stays empty. If ExposureActive shows up
inverted on the bench, `LineInverter` on Line 3 is the fix — don't rewire.

Keep the harness a shielded twisted pair per signal; GPIO lines are the
EMI-susceptible flavor of Basler I/O, and this rig has a strobe in its
future.

---

## 7. First-flight test regime

Phased; each phase gates the next. All standalone — the only wildsync
artifact allowed in the room is this doc.

### Phase 0 — dry run, no camera

0. **Script rehearsal on the emulator.** `PYLON_CAMEMU=2`, run the Phase 1–3
   scripts end to end. Proves the venv, the API surface, and the test
   harness before hardware is at risk.

### Phase 1 — bring-up

1. **Enumerate and single-grab** (§3.5, then `StartGrabbingMax(1)` /
   `RetrieveResult(5000)`). Record model, serial, firmware version. Update
   firmware with `PylonFirmwareUpdater` *now* if one is pending — not mid-rig
   later.
2. **Viewer session** (optional, `ssh -X` from the Mac:
   `/opt/pylon/bin/pylonviewer`). Walk the feature tree once; sanity-check
   the §4 baseline; save UserSet1. Headless truth: on this node the Viewer
   is a bench tool only — every scripted test must stand without it.
3. **Power margin.** `vcgencmd get_throttled` → `0x0` while grabbing;
   deliberately wiggle/reseat nothing — a camera that re-enumerates under
   load is a power problem (§2.1), fix it before proceeding.

### Phase 2 — free-run soak

4. **Rate.** Free-run, no saving, count frames for 60 s: expect ≈17.7 fps
   (limit on) and ≈19 fps (limit off). CPU and RAM noted.
5. **Delivery-path counters at zero.** After a 10-minute free-run, read the
   stream grabber statistics: `Statistic_Buffer_Underrun_Count`,
   `Statistic_Failed_Buffer_Count`, `Statistic_Missed_Frame_Count`,
   `Statistic_Resynchronization_Count`. All zero. `Missed_Frame` counting up
   is the "host can't keep up" signature — revisit `MaxTransferSize` /
   `NumMaxQueuedUrbs` (their product must fit in usbfs memory) and
   `MaxNumBuffer` (each buffer ≈20.3 MB — 20 buffers is ~400 MB of the Pi's
   RAM; size deliberately).
6. **Thermal.** 30 min free-run mounted on the intended bracket: device
   temperature node stable and with margin against the 60 °C housing limit.
   Repeat later inside whatever enclosure the boat gets.

### Phase 3 — software trigger

7. **Triggered loop.** `TriggerMode=On, TriggerSource=Software`, grab via
   `ExecuteSoftwareTrigger()` at 2 fps for 5 min; every trigger yields
   exactly one frame.
8. **Latency distribution.** Enable chunks
   (`ChunkModeActive=True`, `BslChunkTimestampSelector=ExposureStart`);
   bracket `TimestampLatch` between two host clock reads to map the
   camera's 1 GHz tick clock to the Pi clock; log
   (host time of `ExecuteSoftwareTrigger()` → chunk ExposureStart) per shot.
   Expect milliseconds with USB-stack scatter — this number is *why* the rig
   path uses the hardware trigger; record it as the fallback path's error
   budget.

### Phase 4 — hardware trigger

9. **Harness continuity, camera unplugged from USB nothing — wiring check
   first.** Then §6 config; pulse BCM 27 by hand (`gpioset`) and confirm one
   frame per pulse, zero frames on release/boot/reboot. This is the same
   boot-state audit the FOCUS incident imposed on every rig pin: an
   unconfigured Pi pin must not fire the camera.
10. **Trigger latency, hardware path.** Fire N=100 pulses; measure Pi edge →
    ExposureActive fall on BCM 22 (gpiomon timestamps both — the Pi is its
    own scope here, and a real scope on Line 3 is the tiebreaker if numbers
    look odd). Expect `BslExposureStartDelay` (+ ~µs of line propagation)
    with sub-100 µs scatter. **This number is cam3's trigger lead** — the
    analog of what `calibrate_trigger` measures for the ILX bodies, and the
    first hard evidence the Basler can hold formation in a synced fire.
11. **Edge feedback integrity.** At 2 fps for 30 min: every fire produces
    exactly one fall/rise pair, width ≈ ExposureTime, no orphan edges, no
    double-fires. Debounce/hold-off via `BslInputFilterTime` /
    `BslInputHoldOffTime` only if the bench shows ringing — record any value
    used, it adds to the measured lead.

### Phase 5 — compression and spool rehearsal

12. **Encode benchmark** (§5.2). On 20–50 real scene frames (not test
    charts — entropy decides file size): ms/frame and MB/frame at q85, q90,
    q95, single-threaded PyTurboJPEG grayscale vs OpenCV `imencode`.
    Deliverables: the chosen quality, the measured encode time (replacing
    §5.2's 150–400 ms envelope), the per-frame size that feeds the spool
    math, and a rigcal checkerboard detected on a q-chosen frame as the
    quality floor check.
13. **Save path.** Triggered at 2 fps, encode Mono8 → JPEG at the chosen
    quality through the 2-worker pool, write via the rig's atomic pattern
    (`.part` → fsync → rename), EXIF `DateTimeOriginal`+subsec stamped from
    the chunk-timestamp mapping (Pillow/piexif — cv2 alone writes no EXIF).
    30 min: zero dropped frames, encode queue depth bounded, write latency
    stable, disk math consistent with §5's table. This rehearses exactly
    what the `baslerctl` service will do in production.
14. **Load coexistence.** Repeat 13 while a parallel `stress-ng --cpu 2` (or
    the real piagent idling) runs, plus a synthetic puller HTTP-reading and
    deleting frames as the host will: statistics counters still zero, edge
    feedback still clean, encode pool keeping up. The Pi 5 sits below
    Basler's "Jetson or better" recommendation — prove the margin rather
    than assuming it.

### Phase 6 — optional, feeds the strobe brief

15. **Timer output characterization.** Temporarily re-map Line 3:
    `LineSource=Timer1Active`, `TimerTriggerSource=ExposureStart`,
    `TimerDelay`/`TimerDuration` swept; scope the pulses. This measures the
    camera's hardware-timed strobe capability discussed in
    `basler-cam3-node.md` §7 — delay and width programmable to 0.01 µs
    resolution, rise <2.5 µs. Restore `LineSource=ExposureActive` and
    re-save UserSet1 afterwards — leaving this re-mapped breaks the §6
    contract silently.

**Exit criteria for "first flight complete":** Phases 1–5 pass; the recorded
artifacts are the serial/firmware, the §4 UserSet, `BslExposureStartDelay`,
the Phase 4 latency distribution, the Phase 5 encode numbers (quality,
ms/frame, MB/frame), the Phase 2/5 statistics printouts, and thermal
numbers on the real bracket. Only then does `basler-cam3-node.md`
integration work begin.

---

## 8. Troubleshooting quick table

| Symptom | Cause / fix |
|---|---|
| 0 devices as user, 1 as root | udev rules not applied — rerun `setup-usb.sh`, **replug**, check `/dev/bus/usb` perms |
| `Failed to submit transfer status=0xe2100001` / `Failed to probe and lock buffer=0xe2010130` | usbfs too small or `MaxTransferSize × NumMaxQueuedUrbs` exceeds it — §3.2, then lower `NumMaxQueuedUrbs` |
| Camera re-enumerates / vanishes under load | Power. §2.1: `usb_max_current_enable=1` + adequate supply, else powered hub |
| `Statistic_Missed_Frame_Count` climbing | Host/bandwidth — keep the throughput limit on, keep processing off the grab thread, check rtprio (§3.3) |
| Frames grab but no trigger response | `TriggerMode` still Off, or grab loop not armed (`StartGrabbing` must be waiting), or wrong `TriggerActivation` edge |
| ExposureActive polarity inverted | `LineInverter` on Line 3 — do not rewire |
| pylonviewer won't start on the Pi | It's a GUI on a headless node — `ssh -X`, and it needs the Mesa GL packages from the tar's `INSTALL` notes; scripted pypylon is the primary tool here |

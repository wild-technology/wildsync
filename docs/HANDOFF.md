# Handoff — Wild Sync stereo rig

Written 2026-08-16 for whoever picks this up next. **This file is the current truth
about what is done, what is half-done, and what will bite you.**

Read in this order:

1. **`README.md`** — what the system is, how to set up, build, deploy and test it,
   and the hardware rules that will otherwise cost you a day each. Start there.
2. **`docs/PROTOCOL.md`** — the interface contract: node HTTP APIs, the 23-column
   `flight_log.csv` header, run layout, the time model, Sony property encodings.
   Anything crossing a process boundary is defined there.
3. **This file** — current state, what is unfinished, and the measurements behind
   the decisions, so you do not re-derive them.
4. **`docs/future-tests.md`** — open questions that need bench time, each with the
   decision it unblocks and a decision rule.

The rig is on the bench and mostly working; §2 says exactly what state it is in,
including the one thing that currently needs a hand on the hardware.

---

## -1. 2026-08-23 state — macOS host, Jetson retired (read before §0)

The Jetson is gone; **this Mac runs rigd** (launchd agent
`org.wildtechnology.wildsync.rigd`, ProcessType Interactive, `~/rig/rigd-launchd.log`).
Both Pis were re-provisioned from their cards (`deploy/pi-resync/`): this Mac's SSH
key, chrony peering cam1<->cam2 in orphan mode (0.6 ms mutual lock, no upstream needed),
Pi 5 capped at 1.8 GHz with radios off. `deploy/deploy.sh node camN` works from macOS.

**Verified live this week:** sync 0.8 ms median pair spread; two real transects
(140 + 336 pairs, 0 missing, all GPIO-edge stamped); RAW+JPEG on card with Small JPEG
delivered; **white balance fixed at 5600 K and read back on both bodies**; strobe
support deployed (BCM26, unclaimed until used); card ingest tool (`rig/ingest.py`)
matched 476/476 frames and stamps RAWs with edge time + position + attitude (XMP);
stereo pairing verified by relative-pose consistency (`rig/stereo_check.py`).

**The brown-out is power, not software.** Synchronized fires on BOTH nodes drop one
node's PoE port (cam1 untrimmed; cam2 once cam1 was trimmed) — the Ubiquiti switch's
budget at the synchronized spike. Either node alone is clean. rigd now reports it
(`node_rebooted`, `node_undervoltage`, `capture_paused`). Fix is electrical: isolate
one node's power / bigger PoE input / body off the Pi's port. Check the UniFi PoE page.

**Open hardware items:** cam1's card stalls on L-size RAW+JPEG writes (§2.1 again) —
replace with V60/UHS-II; both body clocks ~2 days slow (menu-only; ingest corrects the
archive); iKonvert must be in RAW mode (all four DIP switches ON, 230400) — it shipped
in Mode 0 (NMEA-0183 at 4800); strobe continuity to the sync tip still unproven (one
live fire with the flash pointed away).

**Host-side behaviour added:** ILX_DOWN state (wedged daemon is not "camera not
claimed"); body_locked anomaly (card-stall property-table lock, hushes per-field
divergence); fire timeout 2 s + pause-on-dead-node + `unpaired_shots`; anomaly scan on a
2.5 s timer; stop-grace for the last shot; LiveTap throttle; cacheable run frames;
trigger-latency persistence (`~/rig/trigger_latency.json`, reused <24 h per body id);
static fix fallback (`~/rig/static_fix.json`); bus sniffer (`/nmea`).
**Pi-side:** ilxctl binds :8080 before connecting and answers status while a connect
is pending; stale SDK handles are released on reconnect; live-view backstop; spool
prune (`POST /api/spool/prune`); WB properties; piagent strobe, `host_uptime_s`,
`power` (under-voltage flags).

## 0. Read this first — the three things that will waste your day

1. **Everything is committed and pushed** — `ae6668d` on `jetson-port`. The working tree
   is clean and the full suite is green. You are starting from a known-good state.
2. **A fix workflow was stopped mid-flight.** Two of three lanes finished, one did not,
   and a fourth phase never started. It CANNOT be resumed — §5 has the work list instead.
3. **The rig cannot record right now.** cam1's SD card has a stuck write. This is
   hardware, not software; §2 has the state and the remedy. Do not spend hours
   debugging the software for it — that already happened, and this document is the
   result.

## 1. The goal, verbatim from the user

> Application that allows user to control two cameras for synchronised photogrammetry
> transects, with each image being taken <10ms from each other, ideally perfectly
> accurately. Collect external metadata from NMEA2000 and IMU sources. All computers
> cameras and sensors use gpstime as master clock, falling back to Jetson system time if
> gps master unavailable. Images from each camera renamed and matched to metadata
> available at time image was taken, saved as csv flight_log. UI is intuitive, easy to
> use. Settings are adjusted then applied to all cameras. Cam1 allows live preview of
> settings changes before deploying across fleet. Good logic to defensively handle and
> alert for missing sensors or cameras or missing data. Keep things organized by transect.

Additional operator constraints stated during the session:

- **Manual focus, always.** The rig never uses AF; focus is set in-app. Do not add AF to
  any capture path.
- **Focus/zoom must stay PER-CAMERA**, NOT fleet-applied, until lens encoder parity is
  proven (`docs/future-tests.md` §1). This *reverses* an instruction given to the
  stopped workflow — see §5.3.
- **Archive workflow:** RAW stored on the camera's card, JPEG delivered to the Pi, then
  **moved (with hash verification) to NVMe storage**, deleting the Pi's copy so the SD
  card does not fill. Not built yet.
- **Target archive resolution ~12 MP.** Currently delivering **1.7 MP**. See §4.2.
- **IMU must update ≥60 Hz** and bind to the instant the image was taken on the camera,
  not when the file lands on the Jetson.

## 2. HARDWARE STATE — as left

| Node | State |
|---|---|
| **cam1** (Pi 5, 192.168.1.201, camera `D516000F467B`) | Pi healthy. Camera connected but **CANNOT RECORD**. |
| **cam2** (Pi 4, 192.168.1.202, camera `D516000F46F7`) | **Fully working.** Delivered 6/6 frames on the last test. |
| cam3 (192.168.1.203) | No hardware, permanently OFFLINE by design. |
| Jetson | `rigd` running on :9090. iKonvert present at `/dev/ikonvert` but N2K bus **unpowered** — has never delivered a byte. Expected, not a fault. |

### 2.1 cam1's blocking fault

Operator observed on the body: SD activity LED solid red, a card-transfer animation with
**"1"** next to it, and format refused with *"writing to memory card. unable to operate."*

**One frame is stuck in the camera's write buffer and the card will not accept it.** That
single stuck write explains every downstream symptom, all of which look like software
bugs and are not:

- body goes busy → whole property table locks
- `writable.storeDest = 2` (DisplayOnly), `storeChoices = []`, `driveChoices = []`
- no PC delivery at all (`OnCompleteDownload` never fires, so no `Saved …` log line)
- `ilxctl` eventually wedges inside an SDK call
- **`slotStatus` still reads `OK`** — the card is *recognised* fine; it is the *write*
  that is hung. Do not trust `slotStatus` to catch this.

Compare against healthy cam2: `writable.storeDest = 1`, `storeChoices` populated,
`Saved /home/ubuntu/Pictures/ILX-LR1/ILX08224.JPG` after every fire.

**Remedy (operator, physical):** power cam1's camera fully OFF (PoE will reboot its Pi
too — fine, the usbfs fix now persists). Remove card, read it in a computer, copy off
anything wanted, full (not quick) format there, then format again in-camera. If it
mounts read-only or throws I/O errors, replace it.

**Likely root cause, and it matters for the survey plan:** the ILX-LR1 is 61 MP, so a RAW
is ~60–125 MB. At 1 fps that is 60–125 MB/s *sustained*; at 2 fps, double. UHS-I cannot
do that. If the card is UHS-I or worn it will stall exactly this way, and a stall takes
the whole body down. Check what card is in each body before committing to RAW-on-card.

### 2.2 Recovery recipe you WILL need: un-wedging `ilxctl`

`ilxctl` cannot recover from an unclean shutdown. Kill it mid-SDK-session and the
camera's PTP session stays stuck; the next start blocks forever in SDK connect and
**never binds :8080**, while the process sits `active` at 0% CPU. It looks exactly like a
dead camera. `systemctl restart` also hangs (systemd waits on the stuck stop).

```bash
DEV=$(for d in /sys/bus/usb/devices/*/; do [ -f "$d/idVendor" ] || continue;
  [ "$(cat $d/idVendor)" = 054c ] && [ "$(cat $d/idProduct)" = 0e90 ] && basename $d; done | head -1)
sudo pkill -9 -x ilxctl; sleep 1
echo -n "$DEV" | sudo tee /sys/bus/usb/drivers/usb/unbind; sleep 3
echo -n "$DEV" | sudo tee /sys/bus/usb/drivers/usb/bind;   sleep 4
sudo systemctl start ilxctl
```

This is a genuine robustness bug in `ilxctl` (blocks before binding its HTTP port).
Fixing it properly — bind first, connect in the background — is on the TODO (§7).

## 3. THE BIG DISCOVERIES — do not re-derive these

### 3.1 `FOCUS_LEAD_MS` was costing 5× the sync accuracy

Measured on the live pair, skew computed from **kernel** EXPOSURE-edge timestamps:

| FOCUS lead | per-body latency sd | mean skew | worst skew |
|---|---|---|---|
| **40 ms (was)** | 2.4 ms | 3.14 ms | **7.22 ms** |
| 80 ms | 0.5–0.7 ms | 0.59 ms | 2.10 ms |
| **120 ms (now)** | 0.6–0.8 ms | 0.59 ms | **1.82 ms** |
| 200 ms | 0.6–0.7 ms | 0.68 ms | 1.91 ms |
| 250 ms | 0.9 ms | 1.83 ms | 4.27 ms |

The 80–200 ms plateau is flat; below it the body has not settled when TRIGGER lands,
above ~250 ms it appears to re-meter. Host-side firing is already near-perfect (fire
lateness +0.1–0.3 ms mean, max 0.4 ms; chrony holds both Pis to the Jetson at **~85 µs
RMS**). **All remaining skew is inside the camera bodies.** Do not go looking for it in
the scheduler. Safe to lengthen only because the rig is always MF.

### 3.2 `CAUTION ON BODY` was a false alarm on every error, always

`CrCameraErrorCautionStatus` is 1-based: `NoError = 0x01`, `Error = 0x02`. `camera.cpp`
tested `!= 0`, so **`0x00000001` — literally NoError — was appended to every read-only
error message this rig has ever produced.** It was quoted in `README.md` and the old
`docs/HANDOFF.md` as though it were a real diagnosis. **Fixed in code, and both documents
corrected** (HANDOFF deleted). If you find a mention of a caution modal in any older note,
discount it.

### 3.3 `usbfs_memory_mb` silently reverted to 16 MB on every reboot

`deploy.sh` sets 150 MB (Sony's guidance; the 16 MB default drops the session mid-frame)
but persisted it via `/etc/modprobe.d/usbfs.conf`. **`usbcore` is built into the Pi
kernel, not a module**, so that file was never read. It only ever worked because the
runtime `echo` survived until the next reboot. **Fixed** — runtime set on both nodes,
persisted on the kernel cmdline (`usbcore.usbfs_memory_mb=150` in
`/boot/firmware/cmdline.txt`), and `deploy.sh` corrected.

### 3.4 `gpiomon "%s.%n"` corrupted ~11% of kernel timestamps

`%n` prints nanoseconds with **no zero padding**, so an edge 41,993,474 ns into its
second came out as `2924.41993474` and parsed as 0.42 s instead of 0.042 s. Every edge in
the first 100 ms of a second was wrong by up to 0.9 s. That value is the **debounce
reference**, which decides whether a ringing harness's repeats are swallowed or served as
real exposures. **Fixed** — format is now `"%e %s %n"` parsed as `sec + ns/1e9`.

Each edge now also publishes **`epoch_hw`** (kernel CLOCK_MONOTONIC converted to wall
time on the node). Prefer it over `epoch` for anything cross-node: userspace pipe-read
latency is median 0.09 ms on cam1 / 0.32 ms on cam2 with excursions into the hundreds of
ms, and it is uncorrelated between nodes, so it lands directly in apparent skew.

### 3.5 The IMU node was hardcoded to a node with no hardware

`RunManager.__init__` defaulted `imu_node="cam3"`, which `rigd` never overrode. cam3 is
an empty slot. So `imu_snapshot()` always found nothing and **every IMU column of every
flight_log was empty**, silently, while the IMU sat on cam1 sampling happily. **Fixed** —
the node is discovered from fleet health.

### 3.6 Ringing is a cam2-only problem

Across 851/887 edges: **cam1 `edges_bounced = 0`**, **cam2 `edges_bounced = 662`**. Any
argument about EXPOSURE-line ringing applies to cam2's harness only. The strobe goes on
cam1, whose lead is clean.

### 3.7 Live view is polled forever regardless of tab

`setInterval(pollFleet, 2000)` is not tab-gated and `renderFleet()` unconditionally emits
`<img src="/api/liveview?node=…&_=cachebuster">` per connected camera. Hidden tabs still
load images. **One browser tab left open pulls live view from both bodies every 2 s for
the entire survey.** (Corrected 2026-08-23: live view does NOT share `m_sdkMutex` with
frame transfer — the PC-save download runs on the SDK's own thread and takes no lock;
the measured starvation at ~53 fps happens inside the SDK/USB transfer path itself,
which ilxctl cannot observe. The mutex DOES serialise live view with `/api/status` and
lens drive. Both tiers are now rate-capped: rigd `LiveTap` 5 fps idle / 2 fps in-run,
ilxctl `/liveview.jpg` backstop ≤10 grabs/s.) Historical text: `liveViewJpeg()` takes `m_sdkMutex` — the same recursive mutex
shared by **all 24 SDK entry points**, including image transfer — so it contends directly
with frame delivery. (The UI lane may have partly addressed this; verify.)

Note: this was **not** the cause of the 2026-08-16 fault — no browser was connected.

## 4. VERIFIED MEASUREMENTS

### 4.1 Sync (post-fix, live)
- Inter-camera exposure skew: **0.59 ms mean / 1.82 ms worst** at 120 ms FOCUS lead.
- Per-camera TRIGGER→EXPOSURE latency: cam1 21.94 ms, cam2 21.83 ms (medians).
- Host fire lateness vs scheduled instant: +0.1 to +0.3 ms mean, 0.4 ms max.
- Chrony discipline Jetson↔Pis: ~85 µs RMS (`chronyc tracking`).
- **The <10 ms goal is met with ~5× margin for single shots.** Sustained rate is NOT
  re-measured at real archive settings — see §7.

### 4.2 Resolution — the archive is 35× too small
Delivered JPEGs measured at **1616 × 1080 = 1.7 MP, ~320 KB**. That is `transsize=1`
(Small), a review thumbnail. Every archived transect frame to date is this.

`transsize=0` (Original) is required for full-size delivery; `imagesize` (1=L, 2=M, 3=S)
then sets resolution. **L/M/S dimensions on this body were never measured** — the attempt
was cut short when both bodies faulted. Logged as `docs/future-tests.md` §2.

Current desired vector is `filetype=1` (**JPEG only — you are not recording RAW at all**),
`transsize=1`, `store_dest=3`. Target per `camera.cpp:1211-1218`: `filetype=3` (RAW+JPEG),
`pcsave=1` (JPEG only to host), `transsize=0` (Original).

### 4.3 IMU
**174 Hz measured** (exceeds the 60 Hz requirement). But sample epochs were stamped at
*parse* time, arriving in bursts 0.07 ms apart with gaps up to **35 ms** — so "nearest
sample" could be tens of ms off. The node lane rewrote this to back-date each frame by its
byte-time; **verify it on hardware.** The IMU also reports quaternions (`qw/qx/qy/qz`) and
`pressure_pa` that `flight_log.csv` currently throws away; quaternions are strictly better
for photogrammetry orientation.

### 4.4 `ilxctl` has no readback for the format properties
`imagesize`, `filetype`, `transsize`, `quality` can be **set** but appear nowhere in
`/api/status`. That is why they are "blind fields" in the convergence engine, and it means
**nothing can currently verify what the cameras are set to record.** Adding readback is
high value and unstarted.

## 5. IN-FLIGHT WORK — the stopped workflow

**The original run CANNOT be resumed.** Workflow resume (`resumeFromRunId`) is
same-session only, and that session has ended. The saved script has been deleted
rather than left as a trap. Start fresh from the work list instead:

**`docs/audit-2026-08-16-findings.json`** — 40 findings that survived adversarial
verification, each with `{title, file, line, severity, kind, dimension, goal_clause,
failure, fix}`. Filter by `file` to get a lane's work. 19 are in `rig/run.py`, 9 in
`rig/rig_ui.html`, 4 each in `rigcore.py`/`rigd.py`, 2 in `piagent.py`, 1 each in
`imu_yb.py`/`src/main.cpp`.

If you re-run this as a workflow, keep the file-disjoint lane structure — parallel
agents editing the same file corrupt each other — and **read §5.3 first.**

### 5.1 DONE (verified, tests green)
- **Lane `piagent.py` + `imu_yb.py`** — IMU byte-time epoch back-dating, ring only on
  genuine orientation frames, frozen-fusion detection, measured (not hardcoded) rate
  reporting; `fire()` serialisation and FOCUS state restore instead of forced release;
  `at_epoch` sanity bounds.
- **Lane `rig_ui.html`** — settings hydration + dirty-tracking so Apply can no longer
  clobber the fleet with page defaults; shutter/ISO/aperture as selects from the body's
  own choice lists (multi-second exposures and ISO AUTO now reachable); global anomaly
  pill polled independent of tab; honest per-node "Capture one" reporting; camera columns
  keyed on set identity; IMU hint names the discovered node.

### 5.2 NOT DONE
- **Lane `run.py`** — was still running when stopped. `run.py` shows ~1650 changed lines
  and **compiles and passes the full suite**, but it is *not* known to be complete. Treat
  it as unverified against its work list (the 19 `run.py` findings).
- **Phase "Cross-cutting"** — never started. This is where the two missing *goal*
  capabilities live: **cam1 preview-then-deploy** and the **transect browser**, plus 8
  `rigd.py`/`rigcore.py` findings.
- **Phase "Verify"** — never ran.

### 5.3 ⚠ ONE INSTRUCTION IN THAT SCRIPT IS NOW WRONG
The cross-cutting prompt tells the agent to add `focus_position` / `zoom_setting` /
`zoom_position` to the converged desired vector. **The operator has since forbidden
this.** Focus/zoom must remain per-camera until lens encoder parity is proven
(`docs/future-tests.md` §1 has the test and an explicit decision rule). Edit that section
out of the script before resuming, and make sure the UI never offers an "apply focus to
fleet" control.

## 6. WHAT CHANGED THIS SESSION (committed as `ae6668d`)

```
deploy/deploy.sh        +17    usbfs persistence via kernel cmdline
docs/PROTOCOL.md        +47    contract updates from the node lane
rig/imu_yb.py          +267    sample timestamping rewrite
rig/piagent.py         +339    gpiomon ns fix, epoch_hw, fire lock, FOCUS restore
rig/rig_ui.html        +542    settings hydration, selects, anomaly pill, column keying
rig/rigcore.py          +46    blind-field re-push, Content-Length truncation check
rig/run.py            +1649    FOCUS lead, IMU discovery, adoption, stop barrier, edges
rig/tests/soaktest.py   +59    path fixes + 3 wrong assertions corrected
src/camera.cpp          +76    caution fix, overheating/liveView/slotWriting readback
```

New files: `docs/future-tests.md`, `docs/HANDOFF.md` (this),
`docs/audit-2026-08-16-findings.json` (the remaining work list),
`docs/strobe-trigger.md` (author: operator, corrected this session).
Deleted: `docs/HANDOFF.md` (superseded by this file + README; its fleet and wiring
tables duplicated `docs/PROTOCOL.md`, and its diagnostics rested on the false-positive
caution bug of §3.2).

**Three of the "failures" I fixed were bugs in the TEST, not the code** — e.g. the
timestamp-rounding assertion sliced `hhmmss` at the wrong offset and could never pass.
Verify before "fixing" a failing assertion.

### Test baseline — hold this line
```
python3 rig/tests/soaktest.py        →  137 passed, 0 failed   (was 129/8)
python3 rig/tests/selftest.py --offline →  48 passed, 0 failed
python3 rig/nav.py --selftest        →  PASS
python3 rig/tests/navtest.py         →  PASS
python3 rig/navlog.py --selftest     →  PASS
make                                 →  builds clean, -Wall -Wextra
```

**Nothing is deployed to the nodes.** `src/camera.cpp` changes are built on the Jetson
only. `deploy.sh node camN` also ships `rig/piagent.py` and `rig/imu_yb.py`, so deploying
ships the node lane's work too — that is fine now that it is finished, but re-run the
suite first.

## 7. TODO, in priority order

1. **Get cam1 recording again** (operator; §2.1) and confirm both bodies deliver.
2. **Deploy** `deploy.sh node cam1 && deploy.sh node cam2`, then read the new
   `slotWriting` / `overheating` / `liveViewStatus` fields to confirm the fault signature
   is now visible.
3. **Verify the `run.py` lane's work.** ~1650 lines changed by an agent that was stopped
   before it finished. It compiles and the suite passes, but it is NOT verified against
   its 19-item work list in `docs/audit-2026-08-16-findings.json` (filter `file` ==
   `rig/run.py`). Check each finding is genuinely addressed before trusting it.
4. **Build the two missing goal capabilities** — neither exists: **cam1
   preview-then-deploy** and the **transect browser**. Plus the 8 `rigd.py`/`rigcore.py`
   findings. This was the phase that never started (§5.2). Mind §5.3.
5. **Fix the archive resolution** (§4.2): `transsize=0`, measure L/M/S, pick the setting
   nearest 12 MP, switch to `filetype=3` + `pcsave=1` so RAW lands on the card.
6. **Build the Pi→NVMe pipeline**: pull → verify hash → **delete the Pi's copy**. The
   SD cards are at 456/461 files and nothing prunes them; `/api/shots` re-lists the whole
   directory every 0.4 s.
7. **Implement the strobe** — fully specified in
   `docs/strobe-trigger.md`, whose open decisions are now all settled: 2.9 V
   sync (measured) means **no components needed**; open-drain BCM26 (header pin 37) + GND
   (pin 39); δ ≈ 8–12 ms after T; shutter 1/30 or slower; acceptance check
   `strobe ∈ ⋂[fallᵢ, riseᵢ]` using `epoch_hw`.
8. **Add format-property readback to `ilxctl`** (§4.4) so convergence stops flying blind.
9. **Fix `ilxctl` startup**: bind :8080 *before* connecting the camera, so a stuck SDK
   session cannot make the daemon look dead (§2.2).
10. **Tab-gate live view** and set `LiveView_Image_Quality` Low / expose an off switch
   (§3.7).
11. **Re-measure sustained frame rate at real archive settings** — every figure on record
    used 320 KB thumbnails (`docs/future-tests.md` §3).


## 7a. OPERATOR PUNCH LIST — 2026-08-16, after driving the GUI

Stated by the operator after the first real drive of the finished UI. These
supersede earlier UI decisions where they conflict, including one feature built
the same day (see the note on preview).

**Review tab — becomes the primary working screen**

1. Where "newest" is read from is undefined on initial launch. Define it
   explicitly: which source, which camera, what happens before any frame exists.
2. **Always present image PAIRS.** If a frame's partner does not exist - a
   mis-sync, a failed fire, a lost pull - render a **"missing"** panel in its
   place rather than silently showing one camera.
3. The **timestamp / filename must be the prominent title** of each frame.
4. Between the two rendered frames of a pair, show **`<#.#ms>`** - the measured
   inter-camera jitter for that pair. The EXPOSURE edges make this exact; use
   `epoch_hw`, not `epoch`.
5. Fold the **Fleet tab's content into Review** - either live fleet data, or the
   metadata belonging to the image currently on screen.
6. Move **Start transect / End transect** into Review, out of Controls.

**Controls tab — restructure**

7. **Remove the histogram.**
8. **Remove "Preview on Cam1" entirely.** Exposure changes are made **live to the
   respective camera**. The operator then either hits apply-to-fleet, or
   deliberately leaves the two cameras slightly different - which is a real
   workflow (balancing exposure between the two bodies), not a mistake to guard
   against. The staged preview/commit/discard model is the wrong shape for this.
9. **Perfectly columnar layout:** each camera's settings sit directly under that
   camera's preview. **Zoom and focus apply to that camera only** - never fleet.
10. **Exposure section gets its own "apply to fleet" button, top-right.** It must
    apply **ONLY the settings in that section**. No accidental global writes of
    anything else - not image type, not zoom, not focus.
11. Lens metadata belongs in the respective **zoom** or **focus** header.
12. Remove irrelevant debug text.

**Elsewhere**

13. **Drop all visibility of cam3** - not needed yet.
14. Transects tab wants an **"open folder"** button.

**Known-broken, needs fixing not just re-laying-out**

15. **Focus and zoom do not work reliably.** Zoom has huge lag and is
    inconsistent. This is behavioural, not cosmetic - the controls need to
    actually work, not merely be moved.

## 8. DOCUMENTS THAT ARE NOW WRONG

All corrected during the 2026-08-16 cleanup — `README.md` (sync figures, caution
guidance), `docs/strobe-trigger.md` (BCM 5/6 pull state, the MOSFET that is not needed),
`docs/harness-and-strobe.md` (XR256 sections removed, rig diagram was still Jetson-GPIO
and predated the Pi-node architecture). `docs/HANDOFF.md` was deleted outright.

The one document to re-read rather than trust: **`docs/PROTOCOL.md`**, updated by the
node lane this session. Verify it against the code before relying on it.

## 9. Working notes for whoever is next

- The rig's own diagnostics are good and mostly honest — `/api/diag`, `/api/anomalies`,
  `/gpio/state`, the event journal. Use them before shelling in.
- **Firing the cameras is authorised** by the operator for testing, but ~250 rapid fires
  preceded the card stall. Keep test batches modest and watch `slotWriting` once deployed.
- Prefer measuring on the live rig over reasoning from the code — most of the important
  findings this session came from measurement, and several plausible code-level theories
  (transsize breaking delivery, live view causing the fault) were **disproved** that way.
- The audit that produced `docs/audit-2026-08-16-findings.json` raised
  78 findings and **38 were refuted** under adversarial verification. Treat any single
  agent's claim as a hypothesis until you have read the code yourself.

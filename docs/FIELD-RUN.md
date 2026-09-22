# Field run — checklist and operating guide

The drive-it-on-the-boat sheet. The operator's guide is the field manual,
`docs/USER-GUIDE.html`; installing from scratch is `README.md` (Install) and
`docs/PI-SETUP.md`. rigd runs on the Jetson; each camera's Pi runs ilxctl +
piagent + olive-bridge; the Jetson also fires the XR256.

## Topology

| | |
|---|---|
| rigd host | Jetson Orin NX, 192.168.1.29, systemd `rigd`, UI at http://192.168.1.29:9090 (`localhost:9090` on the Jetson) |
| strobe | XR256, triggered from a Jetson GPIO through the opto board; systemd `piagent-strobe`; appears in the fleet as the camera-less node `jetson` |
| cam1 | Pi 5, 192.168.1.201, **on its own 802.3at Mode-A PoE injector** (NOT the switch) |
| cam2 | Pi 4, 192.168.1.202, on the Ubiquiti 2.5G PoE switch |
| IMUs | one Olive olixVision per camera, on that camera's Pi (USB-C, `olive0`); each camera's flight log carries its own IMU's attitude |
| clocks | the Jetson is the chrony master; the Pis follow it and peer each other as a fallback (~0.6 ms apart with no upstream) |
| nav | Subsonus RTK antenna on the NMEA 2000 backbone → iKonvert → Jetson USB (`/dev/serial/by-id/usb-FTDI_…`); **iKonvert DIP switches all ON for RAW mode** or nav stays silent |

**Why cam1 is on an injector:** two nodes firing in sync drew more than the
switch's PoE budget and it dropped a port every time (measured; not a Pi
brown-out — the PSE shed the port). Isolating cam1's power removed it: 60
synchronized fires, 0 loss. Keep cam1 on the injector. If you must put both on
the switch, in UniFi set both ports to **PoE+ (802.3at)** explicitly and watch
the port power-cycle events.

## Pre-dive checklist

1. **Power the injector and the switch; confirm both nodes.** UI header shows
   `2/2 cameras`. Or, on the Jetson:
   `curl -s localhost:9090/api/fleet | python3 -m json.tool`.
   **Open or create the project** on the first screen (one per campaign). Its
   transects, drained RAWs and exports live under
   `~/wildsync-projects/<name>/`; switching is refused during a run or drain.
2. **Camera clocks.** Both bodies are ~2 days slow and USB cannot set them
   (Sony refuses it). Set date/time in each body's menu. Not fatal if skipped —
   the rig timestamps every frame by its GPIO exposure edge, and ingest stamps
   the RAWs — but it keeps the card's own filenames sane.
3. **Cards.** Both should be fast (V60/UHS-II). cam1's old card stalled on
   L-size RAW writes; the new one passed. Check the UI has no `card_missing`
   or `body_locked` anomaly.
4. **Settings.** Fleet default is survey-ready and persists: F8 · 1/200 ·
   ISO 400 · MF · **5600 K** · RAW+JPEG · imagesize S · **transsize Small**
   (host gets a ~200 KB review JPEG) · **rawtype LossLessL** (full RAW on the
   card). Change exposure per-camera live in Controls, then Apply-to-fleet, or
   leave the two slightly different on purpose. Controls now shows a
   **per-camera format readback** under the Recording format card — confirm a
   push against that, not against delivered file size. "no readback" there means
   an older ilxctl on that node; "status unreadable" means the camera did not
   answer. **rawtype: LossLessL is the survey setting** — LossLessM/S are
   lossless at *reduced* pixel counts and change the ground sample distance of
   every frame. If `~/rig/desired.json` is unreadable the rig renames it aside
   to `desired.json.bad-<ts>`, raises an ERROR event and runs on the built-in
   survey vector **in memory** — it never overwrites your file, but nothing is
   persisted again until you save settings. The rig is always MF: any attempt to
   set autofocus is refused on every path. **Since 2026-08-24 the camera
   daemon enforces this itself**, not just rigd — the node answers 403 to
   AF-S/AF-C/AF-A/AF-D/DMF/PF on `/api/focus/mode`, and to `af:true` on
   `/api/shutter` and `/api/interval/start` (the half-press IS autofocus), and
   logs every attempt. So the AF controls on a node's own :8080 page will not
   work, by design; that is the rule holding, not a fault. The only way to lift
   it is to restart ilxctl with `--allow-autofocus` (bench only) — a node in
   that state advertises `afAllowed:true` to the whole fleet.
5. **Nav.** Subsonus powered with an RTK fix, iKonvert in RAW mode (all four
   DIP switches ON, power-cycle it after changing them). The header reads
   `nav fix`, and `/api/diag`'s `time` block shows the GPS time source.
   Without nav, the flight log uses the armed static fix in
   `~/rig/static_fix.json` — edit that to today's launch point. The preflight
   will say "STATIC fix in use"; that is a choice, hit Start anyway.
   **IMUs:** the IMU tab shows both Olives live (~15 Hz). If one is absent,
   check its USB-C cable and `systemctl status olive-bridge` on that Pi.
6. **Disk.** The project folder on the Jetson wants room: a Small JPEG is
   ~200 KB, a LossLessL RAW ~30-35 MB, and the drain now also pulls each shot's
   full-size **card JPEG** beside its RAW — budget roughly twice what a
   RAW-only drain used to need. `df -h ~`. The `runs_disk_low` anomaly fires
   under 5 GB. The Diag tab's cameras/cards table converts each body's
   remaining shots into GB: cam1's card is ~64 GB class and cam2's ~512 GB, so
   swap cam1 to a matching V60 512 GB for long lines.
7. **Host clock.** On the Jetson, `chronyc tracking` should show it
   synchronised and `chronyc clients` should list both Pis. The Jetson's clock
   is the reference every shot **and the strobe** are scheduled against. An
   earlier, undisciplined host measured 187 ms behind NTP and drifting
   ~60 ppm; the rig corrects the fire schedule and every frame timestamp for a
   host offset and says so once per run ("host clock is X ms off the nodes"),
   but the correction carries its own RTT/2 uncertainty into `time_err_ms`. A
   host >100 ms out raises `host_clock_offset`; the header shows an amber
   `clk ±N ms vs host` chip on any camera more than 50 ms from the host. That
   chip reads the **RTT-filtered median**, not the last raw `/health` sample;
   the raw sample is in the chip's tooltip, so the two differing by tens of ms
   on a busy link is expected. A link too slow to measure a clock over (best
   RTT ≥20 ms) reads `clk unmeasurable — link N ms` instead.
8. **Strobe (XR256).** Before its housing is sealed: dial on the working step
   (start at 250 µs; **never Auto**, which free-runs), and write it down — the
   dial is unreachable once sealed. 24 V rail regulated and inside 22.8–25.2 V,
   green power LED on. Trigger pair from the Jetson's opto board to the light's
   PI terminal. In the UI, Controls → Strobe: enabled, Node `jetson`,
   δ 10 ms; the header pill reads `strobe T+10ms`. **Shutter 1/30 or slower on
   both bodies** (the fleet default 1/200 is too fast — the preflight warns).
   Take one frame with the strobe disabled to check ambient, then Capture one
   with it enabled: the chip between the frames must read **LIT**. Point the
   light away from people on deck. `systemctl status piagent-strobe` on the
   Jetson if the node `jetson` is missing from the fleet.
9. **Calibration.** Calib tab, **in the water** (the WACP-C changes the focal
   length, so an in-air calibration is wrong underwater): checkerboard corner
   counts and square size, follow the amber guidance to green, Compute, Save.
   Stereo VSLAM picks it up automatically. For the IMUs, set the rig still and
   level and press **Calibrate both** (ten seconds, refused if anything moves).

## Running a transect

- Review tab → set label, interval, frame count (0 = until stop) → **Start
  transect**. The preflight lists any warnings; Start anyway commits. Interval
  is refused below 0.20 s or above 3600 s and **warned** below 0.50 s (the fire
  schedule loses its dispatch margin there). Start is disabled while a run is
  already recording — end that one with hold-Stop first.
- Live preview and small-JPEG delivery run together at 2 Hz with the RAW
  landing on the card — verified 30/30 pairs, 0.4 ms skew, 1 ms preview.
- Each delivered JPEG is **deleted from the Pi the instant it is verified on
  the Jetson**, so the Pi spool never fills. The RAW stays on the card. Shooting
  RAW+JPEG, the JPEG carries the flight_log row and the RAW is archived beside
  it under the same stem — frame counts count exposures, not files.
- **A camera that drops out pauses the grid within 3 shots** rather than letting
  the other one shoot the line alone. Resume needs a successful **probe fire**
  on the returning camera, not just a health answer; each probe costs one
  quarantined frame on that body (never in the transect) and backs off 2 s → 60 s
  while it keeps refusing. `answers its health poll but refused a probe fire`
  means that camera is reachable and cannot expose — GPIO harness or body.
- A camera adopted **mid-transect** is deliberately not EXIF-calibrated (the
  calibration shutter cannot be told apart from a survey fire and would cost a
  real frame). Its frames without an EXPOSURE edge fall back to the command
  epoch; stop and restart the line to calibrate it properly.
- **Stop (hold 1 s).** Can take ~20 s — the UI says "stopping…" and waits; read
  the pull tally in Transects. By default this also kicks off a card drain
  (below). `drain_started` in the reply is now the truth, so if it says false,
  the cards were **not** emptied and the reason is in the event journal.
- With the strobe armed, every pair shows **LIT**, **STROBE MISS** or
  **STROBE ?** beside the jitter chip. A miss means the flash landed outside an
  exposure window: check both shutters are 1/30 or slower, then move δ towards
  the middle of 8–12 ms. Transects counts strobe misses per line.
- `<run>/index.jsonl` is the authoritative frame index. **Do not delete it** —
  ingest and `stereo_check` read it in preference to `run.json`, whose index
  keeps only the last 2000 frames.

## Card drain — getting the RAWs off, safely

The full-res RAW never crosses USB during a run (it can't, at survey cadence).
It is pulled off the card **between runs**:

- **There is now a Card drain panel in the UI, on the Review tab, directly under
  the run controls** (that is where the drain belongs in the workflow: it is what
  happens *between* transects). It shows whether a drain is running and on which
  node, the queue, each node's last result (shots pulled / bytes / deleted /
  errors / cancelled), any node skipped for a wedged transfer subsystem, and it
  can start or cancel a drain. **Start transect is disabled while a drain holds a
  camera**, with a tooltip saying which — instead of the click bouncing off with
  an error. If the panel cannot read the drain status it says so and keeps the
  last state it saw, rather than drawing a stale state as "idle".
  Starting a drain from the panel is hold-to-confirm (it blocks the next
  transect for 10-15 min); cancelling is a single click, because cancelling
  cannot cost anything.
- Automatic after every run (the Stop does it), or manually: **`POST
  /api/drain`**, or `python3 rig/drain.py cam1` — which no longer needs rigd
  stopped: it detects a running daemon, hands the drain to it and prints
  progress. Use `--standalone` only with rigd actually stopped.
- The unit of work is the **shot**, not the file: a drain pulls **both** halves
  of each exposure (the RAW *and* its full-size card JPEG, which becomes
  `<base>.card.JPG` in the run and is what `stereo_check` reads). For each:
  pull to the project's `raw/<node>/` → the Pi's SHA-256 must match the bytes the host received
  → **only when every file of that shot is verified** is the shot deleted from
  the card. `/api/card/delete` is per-shot, so a partly-failed transfer now
  leaves the **whole** shot on the card — expect shots to survive a drain where
  they used to vanish; the error names the file and a re-drain finishes it. A
  host-disk floor (2 GB) stops the drain rather than overfilling.
- **A drain can be cancelled**: `POST /api/drain/cancel`. It stops between
  shots, never between a verify and a card delete, so cancelling can never cost
  a card original. Anything not reached is still on the card. Use it rather
  than restarting rigd, which kills the drain mid-pull.
- Duplicate card names (after a body File Number Reset) land as
  `<stem>-c<contentId>.ARW`. That is deliberate, not corruption: nothing already
  verified on the host is ever overwritten.
- The camera cannot shoot while draining (it is in the SDK's transfer mode);
  rigd refuses to start a run during a drain and vice-versa, and settings
  convergence is suspended for that body (its badge freezes rather than going
  divergent).
- After drain, **run `rig/ingest.py` per node** — `python3 rig/ingest.py
  ~/wildsync-projects/<name>/raw/cam1` (for the Legacy project,
  `~/rig-raw/cam1`) — which renames the RAWs to the rig scheme and writes XMP
  sidecars (true time, position, attitude). It now prints an `ingest totals:`
  line; read it. A **combined** directory holding both bodies' cards is
  REFUSED when rigd's journal has no `exif_offset` for the run
  (`ambiguous: DSC/_CA`, nothing renamed) — both bodies fire the same instants,
  so attribution would be a coin flip. Recovery: split into `cam1/` and `cam2/`,
  or restore `~/rig/rigd.jsonl` (the rotated `.1` is read too). A RAW-only tree
  now matches too — ingest reads the `.ARW`'s own EXIF when there is no JPEG.
  Two new artefacts: `<base>.wildsync.xmp` when a third-party sidecar already
  owns `<base>.xmp`, and `<base>.conflict-<sha8>.ARW` when the destination
  already holds different bytes (nothing overwritten, source kept).

**If a drain reports "card index not ready" / hangs:** the body's transfer
subsystem is wedged (happens after many mode switches in a session).
**Power-cycle that camera** to clear it, then drain again. rigd remembers the
wedge: the **automatic** drain after Stop will skip that node (with an event
saying so) until it sees the node power-cycle, while a manual `POST /api/drain`
overrides. The card keeps every frame throughout. The drain itself is
safe — it never deletes an unverified file and always restores shooting mode.
The card holds 500+ RAWs, so you do not need to drain every run; drain when
convenient or when a card is filling.

## What the UI will tell you (anomalies)

| anomaly | meaning / action |
|---|---|
| `node_rebooted` | a Pi lost power mid-run (PoE). Check the injector/switch. |
| `node_undervoltage` | a Pi's rail is sagging under load. Power. |
| `ilx_down` | camera daemon wedged. Power-cycle the body (HANDOFF §2.2). |
| `body_locked` | property table gone — the card-stall signature. Reformat/replace the card. Suppressed while the node reports `busy:true` or sits in transfer mode (a drain), where an absent property table is normal and means nothing. |
| `card_missing` | no card in that body. Insert one. |
| `capture_paused` | a node stopped answering fires, went OFFLINE, or answers `/health` but refuses a probe fire; the run is holding, not shooting the other camera alone. It resumes only when a probe fire on that node actually works. |
| `spool_not_draining` | delete-after-pull is failing; the Pi will fill. Check the spool is writable / run `/api/spool/prune`. |
| `camera_clock_wrong` | the body clock is days off (expected; ingest corrects the archive). |
| `nav_gateway_down` | no live NMEA. If a static fix is armed it is a note, not a fault. |
| `nav_no_fix` | **recording with no position.** The static fix is armed at run *start* only, so a bus that drops mid-line leaves those rows' lat/long EMPTY — they cannot be placed afterwards. Fix the bus and restart the line. |
| `host_clock_offset` | the Jetson's clock is off the nodes (checklist §7). Check `chronyc tracking` on the Jetson. |
| `imu_stall` | no IMU sample near a capture instant; that camera's attitude columns are left empty. Check its Olive (checklist §5). |
| `node_clock_unmeasurable` | that node's link is too slow (RTT ≥20 ms) to verify the 10 ms stereo budget. Check the PoE/switch path. |

Not an anomaly chip, but worth knowing when you are reading the event journal:
**`ilx_busy`** is a warn LINE (not a panel entry) meaning ilxctl answered with
no property block — the SDK mutex is held. The body is held out of the fleet
after 20 s of it; a short spell is an ordinary SDK stall and is ignored on
purpose, and a persistent one is the card-stall signature (see `body_locked`,
which is deliberately suppressed while the node says `busy`).

## Known limits (not blockers)

- The strobe's flash length is set on the XR256's dial and is fixed once its
  housing is sealed; change it only between dives.
- The strobe needs a shutter of 1/30 or slower; faster, and the curtain closes
  before the flash at T + δ.
- The card drain needs a clean transfer session; power-cycle if wedged (above).
- cam1's PoE must stay isolated on this switch (see topology).
- A GPS-corrected `datetime` is absolute to about **half a second** (the
  sender→bus→gateway→USB latency is baked in and nothing subtracts it, because
  nothing measures it). The pair's 10 ms co-exposure requirement is unaffected —
  that is a node-to-node comparison. If two devices on the backbone send time
  and disagree, the rig picks one, keeps surveying, and names it in `/api/diag`'s
  `time` block — a dock-side wiring item, not a reason to stop.
- Holding FOCUS by hand (`POST /api/node/focus {hold:true}`) now self-releases
  after ~30 s and logs `focus_hold_expired`; re-post to keep it. That is what
  stops a dead host leaving a body half-pressed for the rest of a dive.
- The nav pill reads amber **"nav STATIC"** when the position is the armed
  static fix. That is not a GPS fix and it is not a track; depth and heading
  stay empty.

## Recovery one-liners

```
# on the Jetson: fleet + anomalies
curl -s localhost:9090/api/fleet | python3 -m json.tool
curl -s localhost:9090/api/anomalies | python3 -m json.tool
# a node's health / power / IMU
curl -s 192.168.1.201:8081/health | python3 -m json.tool
# cancel a running card drain (stops between shots; never loses an original)
curl -s -X POST localhost:9090/api/drain/cancel
# restart rigd (finalises any active run first; the unit allows 60 s for it)
sudo systemctl restart rigd
# restart the strobe agent
sudo systemctl restart piagent-strobe
# an Olive IMU gone quiet (on that Pi)
sudo systemctl restart olive-bridge
# un-wedge a camera daemon (on the Pi) — see HANDOFF §2.2
# power-cycle the camera body to clear a stuck transfer/PTP session
```

## The UI tabs

- **Review** — run controls, card drain, the newest pair with jitter and strobe
  chips. Mid-run camera drops pause the grid and say so (header, Diag, a red
  CAPTURE PAUSED line); run stop lines report per-camera image counts.
- **Transects** — finished lines, complete pairs, missing shots, strobe misses.
- **Map** — every frame (cam1 dot, cam2 ring, one colour per run) and the
  boat's trail on an offline UTM canvas; wheel/drag to zoom/pan, Fit,
  follow-live, hover for the frame, click for its thumbnail. Bench runs on a
  static fix plot as one stacked point.
- **Controls** — per-camera exposure, focus and zoom; fleet recording format;
  the Strobe panel.
- **Nav / IMU** — live NMEA 2000 data and both Olives.
- **Calib** — stereo checkerboard and IMU calibration (checklist §9).
- **Diag** — status log, project + storage, cameras/cards table, drain
  ("Download images"), **Format card** (facts first, then type the camera's
  name), per-run ZIP downloads.
- **Alerts** — only what needs a human: active anomalies plus warn/error history.
- **Events** — the rig's diary.

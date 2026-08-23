# Field run — checklist and operating guide

Written 2026-08-23 for the first field survey after the macOS-host rebuild.
Read `docs/HANDOFF.md` §-1 for what changed; this is the drive-it-on-the-boat
sheet. rigd runs on the Mac; the two Pi camera nodes run ilxctl + piagent.

## Topology as left

| | |
|---|---|
| rigd host | this Mac, launchd agent `org.wildtechnology.wildsync.rigd`, UI at http://localhost:9090 |
| cam1 | Pi 5, 192.168.1.201, **on its own 802.3at Mode-A PoE injector** (NOT the switch) |
| cam2 | Pi 4, 192.168.1.202, on the Ubiquiti 2.5G PoE switch |
| clocks | the two Pis peer each other (chrony orphan mode); 0.6 ms apart, no upstream needed at sea |
| nav | iKonvert on /dev/cu.usbserial-B400BIHV; **set its DIP switches all-ON for RAW mode** or nav stays silent |

**Why cam1 is on an injector:** two nodes firing in sync drew more than the
switch's PoE budget and it dropped a port every time (measured; not a Pi
brown-out — the PSE shed the port). Isolating cam1's power removed it: 60
synchronized fires, 0 loss. Keep cam1 on the injector. If you must put both on
the switch, in UniFi set both ports to **PoE+ (802.3at)** explicitly and watch
the port power-cycle events.

## Pre-dive checklist

1. **Power the injector and the switch; confirm both nodes.** UI header shows
   `2/2 cameras`. Or: `curl -s localhost:9090/api/fleet | python3 -m json.tool`.
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
   leave the two slightly different on purpose.
5. **Nav.** If you want live GPS, put the iKonvert in RAW mode (all four DIP
   switches ON, power-cycle it). Otherwise the flight log uses the armed
   static fix in `~/rig/static_fix.json` — edit that to today's launch point.
   The preflight will say "STATIC fix in use"; that is a choice, hit Start
   anyway.
6. **Disk.** Host `~/rig-runs` and `~/rig-raw` want room: a Small JPEG is
   ~200 KB, a LossLessL RAW ~30-35 MB. `df -h ~`. The `runs_disk_low` anomaly
   fires under 5 GB.

## Running a transect

- Review tab → set label, interval, frame count (0 = until stop) → **Start
  transect**. The preflight lists any warnings; Start anyway commits.
- Live preview and small-JPEG delivery run together at 2 Hz with the RAW
  landing on the card — verified 30/30 pairs, 0.4 ms skew, 1 ms preview.
- Each delivered JPEG is **deleted from the Pi the instant it is verified on
  the host**, so the Pi spool never fills. The RAW stays on the card.
- **Stop (hold 1 s).** By default this also kicks off a card drain (below).

## Card drain — getting the RAWs off, safely

The full-res RAW never crosses USB during a run (it can't, at survey cadence).
It is pulled off the card **between runs**:

- Automatic after every run (the Stop does it), or manually: **`POST
  /api/drain`**, or `python3 rig/drain.py cam1`.
- For each file: pull to `~/rig-raw` → the Pi's SHA-256 must match the bytes
  the host received → **only then** delete that shot from the card. A failed
  or mismatched transfer leaves the file on the card. A host-disk floor
  (2 GB) stops the drain rather than overfilling.
- The camera cannot shoot while draining (it is in the SDK's transfer mode);
  rigd refuses to start a run during a drain and vice-versa.
- After drain, `rig/ingest.py` renames the RAWs to the rig scheme and writes
  XMP sidecars (true time, position, attitude).

**If a drain reports "card index not ready" / hangs:** the body's transfer
subsystem is wedged (happens after many mode switches in a session).
**Power-cycle that camera** to clear it, then drain again. The drain itself is
safe — it never deletes an unverified file and always restores shooting mode.
The card holds 500+ RAWs, so you do not need to drain every run; drain when
convenient or when a card is filling.

## What the UI will tell you (anomalies)

| anomaly | meaning / action |
|---|---|
| `node_rebooted` | a Pi lost power mid-run (PoE). Check the injector/switch. |
| `node_undervoltage` | a Pi's rail is sagging under load. Power. |
| `ilx_down` | camera daemon wedged. Power-cycle the body (HANDOFF §2.2). |
| `body_locked` | property table gone — the card-stall signature. Reformat/replace the card. |
| `card_missing` | no card in that body. Insert one. |
| `capture_paused` | a node stopped answering fires; the run is holding, not shooting the other camera alone. It resumes when the node is back. |
| `spool_not_draining` | delete-after-pull is failing; the Pi will fill. Check the spool is writable / run `/api/spool/prune`. |
| `camera_clock_wrong` | the body clock is days off (expected; ingest corrects the archive). |
| `nav_gateway_down` | no live NMEA. If a static fix is armed it is a note, not a fault. |

## Known limits (not blockers)

- No strobe yet (deferred). The hardware is wired and the software supports it;
  one live fire with the flash pointed away will prove the sync-tip wire.
- The card drain needs a clean transfer session; power-cycle if wedged (above).
- cam1's PoE must stay isolated on this switch (see topology).

## Recovery one-liners

```
# fleet + anomalies
curl -s localhost:9090/api/fleet | python3 -m json.tool
curl -s localhost:9090/api/anomalies | python3 -m json.tool
# a node's health / power
curl -s 192.168.1.201:8081/health | python3 -m json.tool
# restart rigd (finalises any active run first)
launchctl kickstart -k gui/$(id -u)/org.wildtechnology.wildsync.rigd
# un-wedge a camera daemon (on the Pi) — see HANDOFF §2.2
# power-cycle the camera body to clear a stuck transfer/PTP session
```

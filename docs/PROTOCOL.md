# Wild Sync rig — interface contract

The contract every component is built against. If code and this file disagree,
fix one of them in the same commit — never leave them apart.

## Fleet

| Node | Hardware | IP | User | GPIO chip | Camera | Extras |
|---|---|---|---|---|---|---|
| rigd host | **macOS (Apple Silicon)** — the Jetson (192.168.1.166) is retired 2026-08-23 | DHCP (.199 at last check) | wild | — | — | iKonvert on /dev/cu.usbserial-B400BIHV, rigd :9090 as a launchd agent. NOT a chrony master: the nodes peer each other (orphan mode) |
| pi-cam1 | **Pi 5 Model B Rev 1.0** | 192.168.1.201 | ubuntu | **gpiochip4** (pinctrl-rp1, 54 lines) | D516000F467B | Yahboom YB-MRA02 IMU on USB-A; primary node |
| pi-cam2 | **Pi 4 Model B Rev 1.5** | 192.168.1.202 | ubuntu | **gpiochip0** (pinctrl-bcm2711, 58 lines) | D516000F46F7 | — |

> **cam3 removed 2026-08-19.** The empty third slot fired a permanent
> `node_offline` anomaly and a forever-"2/3" header for hardware that does not
> exist. A third camera joins via `~/rig/nodes.json` (full node list or a
> one-key address override), never a code edit.

> **Renamed 2026-08-16.** The Pi 5 (formerly "cam3") is now **cam1**, the rig's
> primary: it hosts the IMU, previews global exposure changes, and is the
> intended stream source. The newly commissioned Pi 4 is **cam2**. Both hold a
> static address alongside a DHCP lease so a bad netplan cannot strand a node;
> `~/rig/nodes.json` on the Jetson overrides an address without a code change.
>
> **The GPIO chip differs by Pi generation and must never be hard-coded.** Pi 5
> puts the 40-pin header on `gpiochip4` (pinctrl-rp1) — its `gpiochip0` is an
> *internal* brcmstb controller and driving it does nothing at the header. Pi 4
> puts the header on `gpiochip0` (pinctrl-bcm2711). piagent resolves this at
> runtime by finding the pinctrl chip with ≥40 lines. The **header pinout is
> identical** across both, so the harness plugs into the same physical pins.
>
> **Harness status, measured 2026-08-16:** all four leads (GND, FOCUS, TRIGGER,
> EXPOSURE) are wired on both nodes and GPIO firing is verified end to end.
> A connected camera input reads 1 even with a Pi-side pull-down, because the
> body's own pull-up wins — that is the presence test. cam2's EXPOSURE lead
> rings (hundreds of debounced edges per run); the 1 ms guard absorbs it, but
> the wire wants shielding.

All Pis: Ubuntu 24.04, Python 3.12, libgpiod **v1.6.3** (v1 CLI syntax),
`ilxctl` systemd service on :8080, frames land in `/home/ubuntu/Pictures/ILX-LR1`.
`ubuntu` has passwordless sudo. `/dev/gpiochip*` is root-only until the deploy
step installs the udev rule (`GROUP="gpio", MODE="0660"`).

GPIO harness (wired and verified against Sony p.414):

| BCM | Header pin | Dir | Harness pin | Signal |
|---|---|---|---|---|
| 17 | 11 | out (open-drain) | 4 | FOCUS — hold LOW whole run |
| 27 | 13 | out (open-drain) | 5 | TRIGGER — pulse LOW ≥1 ms, FOCUS must already be LOW |
| 22 | 15 | in (bias pull-up) | 6 | EXPOSURE — LOW while front curtain fully open → end of exposure |

Never drive FOCUS/TRIGGER high: "off" is high-Z, not logic 1.

## Measured constants (do not re-derive)

- X-sync ceiling 1/200 s; banding at 1/250. Curtain transit ≈ 4.5 ms.
- Hardware release lag ≈ 20 ms command→exposure; flash ≈ 150 µs at 1/32.
- USB single-shot latency 0.44–0.83 s; USB soak 7200/7200 at 2 fps.
- Host-side USB trigger jitter σ 68.8 ms; camera Interval REC σ 4.7 ms.
- Inter-camera start skew over USB 0–200 ms → GPIO firing is the fix.
- Sony encodings: shutter `(num<<16)|den` (1/200 = 65736); aperture = f×100;
  ISO AUTO = 16777215; drive Single = 1, Continuous Lo = 65540;
  FileType None=0 JPEG=1 RAW=2 RAW+JPEG=3 RAW+HEIF=4 HEIF=5; ImageSize L/M/S=1/2/3;
  TransSize Original=0 Small=1; StoreDest PC=1 card=2 both=3.
- White balance (2026-08-21): `wb_mode` = CrWhiteBalance_* (AWB=0,
  Daylight=0x11, **ColorTemp=0x100**); `colortemp` = Kelvin, valid only while
  `wb_mode`=0x100. Both readable (`whiteBalance`/`colorTemp` in `/api/status`)
  and settable over USB; fleet-converged via the desired vector (default
  256/5600 — fixed WB is rig policy: AWB renders the two bodies of a pair
  differently). **Build-gated:** an ilxctl without the readback keys predates
  the field and the reconcile loop skips it silently — no pushes, no
  divergence — until the node is updated.
- **Body-menu-only properties.** `pcsave` (RAW_J_PC_Save_Image, "RAW+H Save Image")
  and `storeDest` report `enableFlag=DisplayOnly(2)` on these bodies and CANNOT be
  set over USB - they must be changed in the camera's own menu over HDMI, per body.
  `filetype`, `imagesize` and `transsize` ARE settable over USB. Measured 2026-08-16.
- `transsize=0` (Original) delivers the full frame: measured **9504x6336 = 60.2 MP,
  14.1 MB** per JPEG. `transsize=1` (Small) delivers 1616x1080 = 1.7 MP, ~320 KB.
- Camera DateTime **cannot** be set on ILX-LR1 (0x8402) — correct EXIF offsets
  host-side instead.

## ilxctl HTTP API (each Pi, :8080) — existing C++, do not change shape

- `GET  /api/status` → `{connected,model,id, iso,shutter,fnum,program,drive,
  battery,remainShots,slotStatus,storeDest, focus*, zoom*, interval{...},
  log[...]}` plus choice lists. Numeric fields use Sony encodings above.
- `POST /api/connect` → `{ok,model}` | `{ok:false,error}` (blocking, seconds)
- `POST /api/disconnect`
- `POST /api/shutter` `{af:bool}` — USB release
- `POST /api/shutter/hold` `{ms:int}`
- `POST /api/interval/start|stop` — host-timed intervalometer
- `POST /api/camera-interval/config|arm|run` — camera Interval REC
- `POST /api/exposure` `{which:"aperture"|"shutter"|"iso"|"drive"|"filetype"|
  "imagesize"|"quality"|"transsize"|"pcsave"|"rawtype"|"expcomp", value:int}`
  (`expcomp` added by the hardening pass; value = EV×1000 per Sony)
- `POST /api/focus/mode|drive|position`, `POST /api/zoom/drive|position|setting`
- `POST /api/store` `{dest}`
- `GET  /api/shots` → `[{name,size},...]` (PC-saved frames dir)
- `GET  /shot/<name>` → image bytes
- `GET  /liveview.jpg` → current live-view JPEG (503 when unavailable)

## piagent HTTP API (each Pi, :8081) — new, Python stdlib only

- `GET  /health` → `{node, uptime_s, gpio:{chip,ok,focus_held,monitor_running},
  imu:{present,rate_hz,age_s}, disk_free_mb, load1, time:{epoch, source}}`
- `POST /gpio/focus` `{hold:bool}` — hold = spawn open-drain LOW holder;
  release = kill it. Idempotent.
- `POST /gpio/fire` `{at_epoch:float|0, pulse_ms:int=5, focus_lead_ms:int=0,
  strobe_at_epoch:float=0, strobe_pulse_ms:int=5}` → busy-wait to
  `at_epoch` (0 = now), pulse TRIGGER LOW, return
  `{ok, requested_epoch, actual_epoch, late_ms, fire_seq, edge_seq,
  node_epoch}`. Requires FOCUS held (409 if not) unless `focus_lead_ms` asks
  the node to place its own per-shot FOCUS lead. A nonzero `strobe_at_epoch`
  (must sit 0..2 s after `at_epoch`) additionally pulses the strobe line
  (BCM26, open-drain, no pull — docs/strobe-trigger.md) at that instant and
  adds `strobe_epoch`/`strobe_late_ms` (or `strobe_error`) to the reply.
- `POST /gpio/strobe` `{at_epoch, pulse_ms:int=5}` → strobe pulse with NO
  camera fire, same schedule discipline. Exists so the survey's light never
  depends on this node's camera being claimable (the 2026-08-16 card fault
  took the camera out while the Pi stayed healthy).
- `POST /gpio/interval/start` `{at_epoch, period_s, count:int|0}` — absolute
  schedule `at+k·period`, immune to drift; 0 = until stop.
- `POST /gpio/interval/stop`
- `GET /gpio/state` includes `strobe: {bcm, claimed, fires, last_epoch,
  error}` — `claimed:false` is the normal state on a node with no flash; the
  line is claimed lazily on the first scheduled strobe.
- `GET  /gpio/exposure/events?since=<idx>` → `{next, events:[{i,edge:"fall"|"rise",epoch}]}`
  from a persistent gpiomon on BCM22. fall = exposure start.
- `GET  /imu/latest` → newest sample or `{present:false}`
- `GET  /imu/window?t0=&t1=` → samples in epoch range (ring holds ≥60 s)
- `POST /timeprobe` `{t0}` → `{t0,t_rx,t_tx}` NTP-lite offset check

IMU sample dict: `{epoch, pitch, roll, yaw, heading, ax, ay, az, gx, gy, gz,
mx, my, mz, temp}` — degrees, g, deg/s; missing fields null.

## Desired-state convergence (rigd holds one truth)

rigd keeps a single **desired settings vector**; non-exposure fields are
applied identically to every online camera:

```
desired = { aperture, shutter, iso, expcomp, drive,
            focus_mode, filetype, imagesize, transsize, store_dest }
```

**Exposure is per-camera between explicit applies (2026-08-20).** The
operator tunes one body live — `POST /api/exposure {node, which, value}`, a
node key is REQUIRED on that path — and either pushes that camera's exposure
to the fleet (`POST /api/settings`, the force path) or deliberately leaves
the two bodies different, e.g. balancing them against unequal strobes. The
continuous 3 s reconcile therefore only READS `aperture/shutter/iso/expcomp`
and reports a split as `convergence.exposure_split` (information, `synced`
stays true); they are still WRITTEN on an explicit apply, on a node
reconnect, and when the in-pass reboot tell (a NON-exposure readable field
reverted) fires — so a factory-reset body still rejoins the vector. The
staged preview-on-cam1 flow is superseded by this model; its endpoints still
answer for compatibility.

**What is deliberately NOT in it, and must not be added:** `focus_position`,
`zoom_setting`, `zoom_position`. Lens encoder parity between the two bodies is
unmeasured (`docs/future-tests.md` §1), so pushing one body's encoder count to
the other could silently change the stereo pair's interior orientation. They
stay per-camera, are exposed read-only per node in `/api/fleet`
(`focus_pos`, `zoom_pos`, `focus_mode_label`, `zoom_setting_label`) so a
mismatch is at least visible, and **the UI must never offer "apply focus to
fleet"**. Promotion requires the decision rule in future-tests §1 to pass first.

A reconcile loop (every 3 s, and once immediately after any reconnect or user
change) does, per online node: read `/api/status`, diff each field against
`desired`, re-push only the drifted fields via `/api/exposure` etc., then
re-read to confirm. This makes settings **independent of the node's last boot
or last command** — a camera that rebooted to defaults, or was nudged by hand,
is pulled back to `desired` automatically. A field that will not converge after
2 consecutive passes raises a `settings_divergent` alert naming node+field, and
the UI marks that camera; the alarm repeats only when the diverged field SET
changes, or after 300 s. `desired` persists to `~/rig/desired.json` so a rigd
restart keeps the fleet's agreed state. User "set" actions mutate `desired`,
never a single camera — with exactly one exception, the staged preview below.
Convergence status per node is exposed so the UI can show "synced ✓ /
converging… / divergent ✗ / PREVIEW".

Per-node convergence object:
```
{synced: true|false|null, diverged:[field…], unsettable:[field…],
 blind_errors:{field: "the body's own error text"}, last_check: epoch,
 preview: true, preview_fields:[field…]}      # last two only while pinned
```

**Fields with no readback.** `filetype`, `imagesize`, `transsize` and `expcomp`
are settable over USB but appear nowhere in ilxctl's `/api/status`, so
convergence tracks them against a per-node cache of what was last pushed. Three
rules make that cache honest: it is cleared on every `OFFLINE→CAM_CONNECTED`
transition, cleared whenever a *readable* field is found reverted (a body can
power-cycle between two polls), and **never** written for a push that failed or
that never reached the body. A refused blind field is reported in
`convergence.diverged` with the body's error in `blind_errors` — it is invisible
to the readback check by construction, and a body that silently rejects
`filetype` means the two cameras record in different formats.

**Body-menu-only properties.** `storeDest` and `pcsave` report
`enableFlag=DisplayOnly` on these bodies and cannot be set over USB. They appear
in `convergence.unsettable` (alarm kind `settings_unsettable`, raised once per
state change) and read-only in the UI.

**Validation.** `POST /api/settings` validates every value before it can enter
`desired`: type, range, the Sony encoding (shutter `num`/`den` both ≥ 1), and —
where the primary camera publishes a choice list — membership of that list.
Rejections are returned to the caller and journalled as `settings_rejected`;
nothing rejected is ever persisted. An unvalidated value (e.g. a cleared input
box sending ISO 0) used to persist to `~/rig/desired.json`, survive every
restart, and re-push to both bodies every 3 s forever.

### Staged preview — "Cam1 previews before the fleet deploys"

The one place a single camera is set on purpose. `POST /api/settings/preview`
applies pending EXPOSURE values (`aperture`, `shutter`, `iso`, `expcomp` — never
focus or zoom) to the **primary** camera only, leaves `desired` untouched, and
**pins** that node so the reconcile loop does not revert it. `commit` promotes
the staged values into `desired` and converges the fleet; `discard` drops them
and re-converges the previewed camera immediately.

While a pin is held the two bodies are **intentionally mismatched**, so every
pair shot in that window is unusable. The pin is therefore self-healing in four
independent ways:

| release | why |
|---|---|
| expires after **180 s** (`PREVIEW_TTL_S`) | a forgotten preview cannot cost more than one TTL |
| dropped when `desired` changes | the proposal is stale the moment the fleet moves |
| dropped when a run starts | a transect must never record a mismatched pair |
| never persisted | a rigd restart always comes up unpinned |

It is also loud while held: the node's convergence reports `preview:true` (never
`synced`), a `preview_pinned` anomaly fires, the UI outlines that camera's live
view with a PREVIEW badge and the pending-vs-live diff, and pre-flight **blocks**
a run start until it is deployed or discarded.

## Observability & AI live-correction hooks

Every service writes a structured event journal (JSONL), one object per line:
```
{ts, node, sev, kind, msg, ctx{...}}
```
`sev` ∈ debug|info|warn|error|critical. `kind` is a stable slug
(`node_transition`, `reconnect`, `settings_divergent`, `settings_unsettable`,
`settings_rejected`, `settings_preview`, `capture_fail`,
`pull_fail`, `pull_retry`, `gpio_late`, `sync_skew`, `sync_degraded`,
`orphan_fire`, `frames_missing`, `calibration_missing`, `timebase`,
`jitter_high`, `imu_stall`, `nav_stale`, `exif_offset`, `disk_low`,
`runs_disk_low`, `card_write_stuck`, `camera_overheating`, `pc_control_lost`,
`preview_pinned`, `run_recover`, `lifecycle`, `sdk_error`, …). Never log secrets.

`~/rig/rigd.jsonl` is capped at **16 MB** with exactly one previous generation
(`rigd.jsonl.1`). It is written from every thread in the process, so the writer
tolerates both `OSError` (disk full, read-only) and `ValueError` (a per-run
`events.log` closed underneath it at run stop) — an escaping exception there
kills the thread that was reporting the fault, and when that thread is a
NodeMonitor the fleet silently stops being watched.

Run-level alert kinds and what each one means:

| kind | raised when |
|---|---|
| `capture_fail` | a node's fire returned `ok:false` — that camera took no picture |
| `sync_skew` | the realised inter-camera exposure spread for one shot exceeded 10 ms |
| `gpio_late` | a node reported firing >2 ms from its scheduled instant |
| `sync_degraded` | a camera dropped to the USB path, or its `/health` went unreadable while it stays on the GPIO path it last had |
| `orphan_fire` | a fire produced no frame; its queued command was dropped so later frames keep their own capture instants |
| `frames_missing` | shots fired minus frames landed exceeded a small threshold on one camera |
| `calibration_missing` | a GPIO-armed camera has no trigger-latency measurement of its own and is firing on the fleet median |
| `timebase` | the master clock changed mid-run; the run keeps its start-of-run base |

Fleet/hardware alert kinds:

| kind | raised when |
|---|---|
| `runs_disk_low` | the **Jetson** volume holding `~/rig-runs` is below 5 GB. This is the volume every transect is written to; a node's `disk_free_mb` is that Pi's PC-save spool and says nothing about it. When it fills, each frame write raises OSError and that frame is **never retried** — no flight_log row, no nav/IMU correlation |
| `card_write_stuck` | `slotWriting` has read `WRITING` continuously for >30 s (critical at >120 s). One frame wedged in the body's write buffer locks the whole property table, stops PC delivery and wedges ilxctl **while `slotStatus` still reads OK** — do not trust slotStatus to catch it |
| `camera_overheating` | `overheatingLabel` is `pre-overheat` (warn) or `OVERHEATING` (critical); the body shuts itself down mid-transect |
| `pc_control_lost` | `priorityKeyLabel` reads `Camera position` — PC Remote priority was taken back by the body, so writes are refused and PC save stops delivering. Masquerades as an SDK bug |
| `preview_pinned` | a settings preview is being held on one camera, so the pair is not matched |

Every anomaly carries `sev: "bad"|"warn"` alongside `{kind, node, since,
evidence, suggested_action}`; the UI renders that field rather than pattern-
matching the kind's spelling.

rigd exposes, for both humans and an autonomous agent:
- `GET /api/events?since=<seq>&sev=<min>` → `{next, events:[...]}` — cursor-based,
  never blocks, bounded page. The AI watcher polls this.
- `GET /api/diag` → one self-describing snapshot: versions, every node's monitor
  state + convergence + last error, active run, disk, clock offsets, recent
  anomaly counts by kind, current desired vector. Enough to diagnose a field bug
  without shell access.
- `GET /api/anomalies` → currently-firing detectors with evidence:
  jitter σ over threshold, EXPOSURE edges missing while frames arrive, pull
  backlog growing, IMU/nav staleness, battery/disk trending down, a node
  flapping. Each has {kind, node, since, evidence, suggested_action}.
- Log files under the run dir (`events.log` human, `run.json` machine) and a
  rolling `~/rig/rigd.jsonl` across runs.

`/api/diag` also carries `storage:{runs_dir, jetson_free_mb,
low_threshold_mb}` and `/api/fleet` carries `jetson_free_mb` +
`jetson_disk_low_mb`, so the capacity of the volume the survey is written to is
visible before a run rather than after it fails.

Per-node fields in `/api/fleet` (beyond exposure/battery/convergence):
`slot_status`, `slot_writing` + `slot_writing_label`, `overheating` +
`overheating_label`, `live_view` + `live_view_label`, `priority_key_label`,
`store_dest` + `store_dest_label`, `writable{}`, `filetype`/`imagesize`/
`transsize` (null on real bodies — no readback), and the read-only per-camera
lens readings `focus_mode`, `focus_mode_label`, `focus_pos`, `zoom_pos`,
`zoom_setting_label`.

### rigd HTTP API (Jetson, :9090)

Settings:
- `GET  /api/settings` → the desired vector plus `_auto` and `_preview`
- `POST /api/settings` `{field:value,…}` → `{ok, applied{}, rejected{field:why}}`
- `GET  /api/settings/preview` → `{active}` | `{active, node, pending{},
  desired{}, since, expires_at, expires_in_s}`
- `POST /api/settings/preview` `{aperture?,shutter?,iso?,expcomp?,node?}` →
  `{ok, node, preview{}, failed{}, rejected{}}` — primary only, `desired`
  untouched, node pinned
- `POST /api/settings/commit` → promote the staged values into `desired`,
  release the pin, converge the fleet
- `POST /api/settings/discard` (alias `/api/settings/revert`) → drop them and
  re-converge the previewed camera
- `POST /api/settings/auto` `{on}`, `POST /api/ev` `{steps}`,
  `POST /api/reconcile`
- `POST /api/focus/mode` `{mode}` → routed through `desired` (it is fleet
  state), returns `{ok, applied, rejected}`. `POST /api/focus/drive|position`
  and `/api/zoom/drive|position|setting` fan out to the connected cameras and
  are **not** converged or persisted — per-camera by design.

Transect browser (read-only; the runs tree is never written by these):
- `GET /api/runs?limit=<n≤200>` → `{root, total, runs:[{run_id, label, started,
  nodes, cams, frames, final, interrupted, time_source, skew_ms_max,
  pulled{}, failed{}}…]}`, newest first
- `GET /api/run/detail?id=<run_id>` → the run.json header, `per_camera{cam:
  {rows, untimed_rows, files, first, last, sources{}, truncated}}` and
  **`pairs`** — the stereo-completeness view:
  `{cams, shots, complete, incomplete, tolerance_s, gaps:[{i, epoch, have[],
  missing[], files{}, spread_ms}…], gaps_truncated, strip, recent[]}`.
  Shots are formed by grouping every camera's flight_log rows within
  `tolerance_s` (≤0.75 s, or interval/2). `strip` is one character per shot,
  oldest first: `.` = every camera present, a digit = how many were missing.
- `GET /api/run/frame?id=&cam=&name=` → image bytes from the run directory
- `GET /api/run/flight_log?id=&cam=` → that camera's `flight_log.csv` verbatim

**Path safety.** `id`, `cam` and `name` come from the browser and are validated
twice: a strict character class (`[A-Za-z0-9][A-Za-z0-9._-]*`, so no separator,
no leading dot, no `..`, no absolute path) and a `realpath` containment check
against `~/rig-runs`, which also defeats a symlink planted inside a run folder.
Frames additionally require an image extension. Everything is bounded —
≤200 runs listed, ≤50 000 flight_log rows and 32 MB parsed per camera, ≤500 gaps
enumerated (the counts stay exact), ≤64 MB per frame — and run details are
cached on the mtimes of the files they were built from, because the UI polls
this during a live survey on the host that is writing the frames.

The intent: an AI agent (or the user) can poll `/api/diag` + `/api/anomalies`
in the field, see what's wrong with evidence, and correct — restart a node,
re-push settings, adjust exposure — through the same API, in the background,
without stopping the survey.

## Image pathway & naming

`camera card/PC → ilxctl PC-save dir → rigd pull → rename → organized`

Pulled frames are copied into the run's per-camera folder and **renamed**:
```
Cam{N}_YYYYMMDD_hhmmss.ss.jpg      (UTC, N = 1|2|3 from node)
```
e.g. `Cam3_20260815_193045.20.jpg`. The timestamp is the frame's best capture
instant (GPIO EXPOSURE edge > corrected EXIF > command epoch), same source
recorded in flight_log. The original camera name (`ILX00123.JPG`) is retained
in flight_log and run.json for traceability. RAW sidecars, if enabled, keep the
same stem with the raw extension. `flight_log.csv`'s `filename` column is the
**renamed** name.

## rigd (Jetson, :9090) — new

Node monitor state machine per camera node, 2 s poll:
`OFFLINE → REACHABLE → CAM_CONNECTED` with automatic `/api/connect` retry
(backoff 5→60 s), settings re-push + verify after every reconnect, alert on
every transition. Dropouts mid-run never stop the run for other nodes.

Each `/health` poll also samples the node's clock against rigd's
(RTT-bounded): `/api/fleet` nodes carry `clock_offset_ms`/`clock_rtt_ms`, and
a `node_clock_skew` anomaly fires when two nodes disagree beyond the noise
floor (warn >5 ms, bad >8 ms). Every scheduled fire and every `epoch_hw` edge
lives on the NODE's clock, so inter-node clock skew lands 1:1 in true
exposure skew **while the rig's own skew figures under-report it** — the
nodes' chrony must share one master (measured 16.8 ms apart free-running on
2026-08-20 after the Jetson master left the topology).

### 2026-08-20 additions — strobe, pairs, per-camera exposure

- `GET/POST /api/strobe` → `{ok, strobe:{enabled, node, delta_ms, pulse_ms},
  warnings:[…]}`. Config persists to `~/rig/strobe.json`. When enabled,
  `capture_once` schedules the pulse at `T + delta_ms` on the strobe node —
  inside its `/gpio/fire` when that node fires over GPIO, else via the
  standalone `/gpio/strobe` so a faulted camera cannot darken the survey.
  Warnings (not refusals): delta outside the measured-safe 8–12 ms, shutter
  faster than 1/30 (docs/strobe-trigger.md §4.1).
- `POST /api/exposure` `{node, which, value}` → per-camera LIVE exposure
  write, forwarded to that node's ilxctl; 400 without `node`, 409 when the
  camera is not connected. See "Desired-state convergence".
- `GET /api/run/shots?id&offset&limit` → `{run_id, total, offset, shots:[…]}`
  — the run's full grouped shot list (offset<0 counts from the end). Each
  shot: `{epoch, files:{cam:name}, missing:[…], spread_ms, spread_src:
  "index"|"flight_log", srcs:{cam:capture_source}, strobe?:{epoch, ok,
  margin_ms}}`. `spread_src:"index"` means µs-grade run.json-index epochs;
  flight_log datetimes are centisecond-quantised and must not be read as a
  jitter measurement. The strobe verdict is docs/strobe-trigger.md §4.2 —
  `ok` only when the pulse sits inside the INTERSECTION of every member's
  measured `[fall, rise]` window and every member is `gpio_edge`; `ok:null`
  when it fired but the windows are not all measured.
- `POST /api/run/open` `{id}` → opens the run directory in the rigd host's
  file manager (`open`/`xdg-open`; path-guarded; returns `{ok:false, path}`
  with no display). `/api/run/detail` gains `path` and `pairs.strobe_missed`.
- run.json index rows gain `rise` (end-of-exposure `epoch_hw`) and `strobe`
  (the shot's pulse instant, carried on the strobe node's frame); the run doc
  gains the `strobe` config in force.
- `POST /api/calibrate` refuses (409) while a run is active — calibration
  holds FOCUS (an AE-lock) and races the run's frame naming; the run
  calibrates itself at start. Measured latency persists to
  `~/rig/trigger_latency.json` keyed by node with the body's camera id and is
  reused for 24 h on the same body (no calibration frames fired); the API
  call always re-measures (`force`).

### 2026-08-23 additions — macOS host, node health, wedge-proofing

- Node states gain `ILX_DOWN`: piagent answers, ilxctl does not. rigd never
  POSTs `/api/connect` at it, marks the cached status `{connected:false,
  stale:true, ilx_down:true}`, and raises anomaly `ilx_down` (bad) with the
  §2.2 recipe. `REACHABLE` keeps its meaning: ilxctl answered
  `connected:false`.
- Anomalies: `node_rebooted` (host uptime went backwards — power loss),
  `node_undervoltage` (piagent `power` flags), `body_locked` (connected but
  no `writable` map / ISO / slotWriting — the card-stall table lock; per-field
  `settings_divergent` is suppressed while it lasts), `capture_paused` (the
  grid is holding for a node that failed 3 fires), `camera_clock_wrong`
  (|EXIF offset| > 60 s), `node_clock_skew`.
- piagent `/health` adds `host_uptime_s` (the Pi's `/proc/uptime`; `uptime_s`
  is the service's) and `power: {throttled, undervolt_now,
  undervolt_since_boot, throttled_now, throttled_since_boot}` from the
  firmware's `get_throttled` word via sysfs.
- Capture loop: per-node fire timeout is `max(2 s, lead + FOCUS lead + 1.5 s)`;
  three consecutive failed fires on a node pause the grid (no unpaired
  frames), `capture_paused`/`capture_resumed` events, `sync.unpaired_shots`
  and `sync.paused_for` in `/api/run` and run.json. Stop waits up to 6 s for
  frames of fires already committed before releasing the pull workers.
- rigd `/api/liveview` is a throttle (`LiveTap`): one upstream grab per
  camera shared by all clients, served from cache within 200 ms idle /
  500 ms while a run is active; headers `X-Frame-Age-ms`, `X-Live-Policy`.
  ilxctl `/liveview.jpg` has its own backstop (one SDK grab per 100 ms,
  `X-LiveView-Age-Ms`). `/api/run/frame` is cacheable (`ETag`, max-age 3600).
- ilxctl binds :8080 BEFORE the startup connect and `/api/status` answers
  `{connected:false, connecting:true, log:[…]}` without the SDK mutex while a
  connect is pending; a handle left by a dropped session is released on the
  next `/api/connect` instead of refusing "already connected".
- `POST /api/spool/prune` `{confirm:"prune", older_than_s=86400, keep=50}`
  moves old PC-save files into `<dir>-archive` (never deletes).
- Ingest: `rig/ingest.py <card_dir>` matches card files to flight-log rows
  (by file number when the card counter equals the PC-save counter, else by
  the measured clock offset with rigd's journaled `exif_offset` as the
  tie-break), places `<base>.ARW` / `<base>.card.JPG` / `<base>.xmp`
  (true instant, GPS, UTM, attitude, pair spread) beside the review JPEG,
  writes `ingest_manifest.csv`. `rig/stereo_check.py` verifies pairing by
  relative-pose consistency against a mis-paired control.

Transect run layout (Jetson, `~/rig-runs/`):

```
runs/260815_1930_transect-01/
  run.json            journal: config, nodes, events, per-frame index, alerts,
                      the timebase applied, and per-shot sync quality
  nmea_raw.log        every serial line, prefixed epoch
  events.log          human-readable event stream
  cam2/
    Cam2_20260815_193045.20.jpg …   (renamed; original name kept in flight_log)
    flight_log.csv
  cam3/ …
```

**Lifecycle.** `rigd` finalises an active run on SIGTERM (systemctl
stop/restart), on SIGINT, and on any clean exit: `Rig.stop()` drains the pull
workers and writes `run.json` with `final:true`. On **startup** it scans the 20
newest runs for a `run.json` still marked `final:false` — the signature of a
crash or a power cut — and repairs it: `frames` is recounted from the
`flight_log.csv` files actually on disk, `final` is set, and `interrupted:true`
plus `interrupted_note` and `recovered{at, flight_log_rows, frames_indexed}`
record that the manifest was rebuilt rather than written by the run itself. The
imagery, the flight_log rows and events.log survive an abnormal exit intact
(all are flushed as produced); it is only the manifest that needs rebuilding.

A run **never reuses an existing directory**. `run_id` is
`YYMMDD_hhmm_<label>`, which is not unique — two starts in the same minute with
the same label (a false start, or a rigd restart) would otherwise interleave two
surveys in one folder and destroy the first `run.json`. On collision the id gains
a `_b`, `_c`, … suffix and the API response returns the id actually used.

Calibration exposures (the EXIF-clock frame and the trigger-latency frames) are
**not survey data**: they are fired before the capture loop is armed, named by
the calibration that caused them, and excluded from the run folder, the
flight_log and the frame index. Run-start calibration and auto-capture never
overlap.

UI (`rig_ui.html`, served by rigd, **runs with no AI**) is tabbed:
- **Review** — the existing single-camera captures pane generalised to N cameras
  side-by-side, newest frame first per camera, per-camera ISO/f/shutter burned
  in, arrow-key stepping. Dynamically adds/removes camera columns as nodes come
  and go.
- **Fleet** — heartbeat strip per node (state, battery, disk, jitter, last
  frame age) with slick live sparklines, plus the camera-health fields (card
  write, thermal, live-view status, control priority). Live view here is **off
  by default** behind an explicit switch.
- **Transects** — the run browser: every run newest-first, and for the selected
  one its stereo-pair completeness (complete / incomplete counts, the per-shot
  strip, the gap list with the frame that *did* land), per-camera flight_log
  row counts and capture-source breakdown, a link to each `flight_log.csv`, and
  a side-by-side viewer for the last shots so a missing half of a pair is
  visible as a MISSING panel, not a number.
- **Nav** — location plot (track from lat/lon), heading rose, depth readout.
- **IMU** — live attitude (pitch/roll/yaw) horizon + heading.
- **Controls** — the one desired-settings panel that drives all cameras, with
  per-node convergence badges, the EV-bump control, the **Preview on Cam1 /
  Deploy to fleet** pair beside cam1's live view + histogram, the recording
  format selects (filetype / imagesize / transsize, marked "no readback"), and
  read-only panels for the body-menu-only properties and the per-camera lens
  positions.
- **Events** — the live event/anomaly stream.

**Live view is gated.** `liveViewJpeg()` takes the same recursive SDK mutex as
image transfer, so every live-view GET competes with the capture path on a body
that is mid-survey. No live-view or `/api/shots` request is issued unless its
tab is both selected AND `document.hidden === false`; the Fleet tab additionally
requires its switch to be on. One backgrounded browser tab used to pull frames
from both bodies every 2 s for an entire survey.

flight_log.csv header (exact):
```
filename,datetime,lat,long,xutm,yutm,utm_zone,depth_from_xplore9,pitch,roll,yaw,heading_mag_xplore,heading_imu,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_c,capture_source,time_source,time_err_ms
```
`datetime` = `YYMMDD_hhmmss.ss` **UTC**. Empty string for unavailable fields —
never fabricate. `capture_source` = `gpio_edge|exif|command`. `time_source` =
`gps|jetson`.

Time model: GPS time (PGN 126992/129029) when the N2K network is live →
`gps_offset = gps - jetson`; all log stamps corrected by it. No fix → offset 0,
`time_source=jetson`. Camera EXIF offset measured per node at run start
(calibration frame: fire, read EXIF DateTimeOriginal+SubSec, diff vs
GPIO exposure edge when available, else command epoch + 20 ms). Frame capture
instant preference: GPIO EXPOSURE fall edge > corrected EXIF > command time.

**Where the correction is applied, and where it must not be.** Every epoch
handled internally — fire commands, EXPOSURE edges, the nav history ring, the
IMU window — is a raw local (Jetson) epoch; those are all local-clock domains
and a corrected stamp handed to `fix_at()` or `/imu/window` silently blanks
those columns. `gps_offset` is applied exactly **once**, at the presentation
boundary: the `datetime` column and the `CamN_` filename. `time_source` names
the offset actually applied to that row, never a second, later reading.

The offset is **latched at run start** and held for the whole transect, so a fix
arriving or dropping mid-run cannot leave rows in one CSV sitting on two clocks.
The change is journalled (`timebase`) and `run.json` records both the applied
base and the live one, so a whole run can be re-based afterwards.

Exposure policy: **manual is the default** (aperture, shutter, ISO pushed
identically to every camera, verified by readback). Optional auto = ISO servo
(sRGB-linearised mean luma from pulled frames, one decision applied to all
bodies, ±1 choice-step per adjustment, bounds ISO 100–6400). EV bump: ±1/3-stop
steps applied live (manual → ISO step; auto → servo target shift).

## Deliverables & boundaries

- `rig/imu_yb.py` (IMU agent): `ImuReader` class + `probe()`; talks to the
  device only; no HTTP.
- `rig/nav.py` (nav agent): `NavReader` class (serial thread, ring buffers,
  `snapshot()`, `raw_log_hook`), `latlon_to_utm()` pure function,
  `TimeAuthority`. No HTTP.
- `src/*` (hardening agent): fixes only, no API shape changes beyond adding
  `expcomp` + noted fields.
- `rig/piagent.py`, `rig/rigd.py`, `rig/rig_ui.html`, `rig/deploy.sh`,
  systemd units (core, written by the session lead). Python stdlib only in
  services; pyserial allowed (apt `python3-serial`).

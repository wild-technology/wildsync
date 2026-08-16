# Wild Sync rig — interface contract

The contract every component is built against. If code and this file disagree,
fix one of them in the same commit — never leave them apart.

## Fleet

| Node | Hardware | IP | User | GPIO chip | Camera | Extras |
|---|---|---|---|---|---|---|
| jetson | Orin (aarch64) | 192.168.1.166 | wildtech | — | — | iKonvert on /dev/ttyUSB0 (FTDI FT232R B400BIHV), chrony master, rigd :9090 |
| pi-cam1 | **Pi 5 Model B Rev 1.0** | 192.168.1.201 | ubuntu | **gpiochip4** (pinctrl-rp1, 54 lines) | D516000F467B | Yahboom YB-MRA02 IMU on USB-A; primary node |
| pi-cam2 | **Pi 4 Model B Rev 1.5** | 192.168.1.202 | ubuntu | **gpiochip0** (pinctrl-bcm2711, 58 lines) | D516000F46F7 | — |
| pi-cam3 | not populated | 192.168.1.203 | — | — | — | slot reserved; reports OFFLINE |

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
  FileType JPEG=1 RAW=2 RAW+JPEG=3; ImageSize L/M/S=1/2/3;
  TransSize Original=0 Small=1; StoreDest PC=1 card=2 both=3.
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
- `POST /gpio/fire` `{at_epoch:float|0, pulse_ms:int=5}` → busy-wait to
  `at_epoch` (0 = now), pulse TRIGGER LOW, return
  `{ok, requested_epoch, actual_epoch, late_ms}`. Requires FOCUS held; 409 if not.
- `POST /gpio/interval/start` `{at_epoch, period_s, count:int|0}` — absolute
  schedule `at+k·period`, immune to drift; 0 = until stop.
- `POST /gpio/interval/stop`
- `GET  /gpio/exposure/events?since=<idx>` → `{next, events:[{i,edge:"fall"|"rise",epoch}]}`
  from a persistent gpiomon on BCM22. fall = exposure start.
- `GET  /imu/latest` → newest sample or `{present:false}`
- `GET  /imu/window?t0=&t1=` → samples in epoch range (ring holds ≥60 s)
- `POST /timeprobe` `{t0}` → `{t0,t_rx,t_tx}` NTP-lite offset check

IMU sample dict: `{epoch, pitch, roll, yaw, heading, ax, ay, az, gx, gy, gz,
mx, my, mz, temp}` — degrees, g, deg/s; missing fields null.

## Desired-state convergence (rigd holds one truth)

rigd keeps a single **desired settings vector** applied identically to every
online camera — the cameras are never controlled individually:

```
desired = { aperture, shutter, iso, expcomp, drive,
            focus_mode, focus_position, zoom_setting, zoom_position,
            filetype, imagesize, quality, transsize, store_dest }
```

A reconcile loop (every 3 s, and once immediately after any reconnect or user
change) does, per online node: read `/api/status`, diff each field against
`desired`, re-push only the drifted fields via `/api/exposure` etc., then
re-read to confirm. This makes settings **independent of the node's last boot
or last command** — a camera that rebooted to defaults, or was nudged by hand,
is pulled back to `desired` automatically. A field that will not converge after
N tries raises a `settings_divergent` alert naming node+field, and the UI marks
that camera. `desired` persists to `~/rig/desired.json` so a rigd restart keeps
the fleet's agreed state. User "set" actions mutate `desired`, never a single
camera. Convergence status per node is exposed so the UI can show "synced ✓ /
converging… / divergent ✗".

## Observability & AI live-correction hooks

Every service writes a structured event journal (JSONL), one object per line:
```
{ts, node, sev, kind, msg, ctx{...}}
```
`sev` ∈ debug|info|warn|error|critical. `kind` is a stable slug
(`node_transition`, `reconnect`, `settings_divergent`, `capture_fail`,
`pull_fail`, `pull_retry`, `gpio_late`, `sync_skew`, `sync_degraded`,
`orphan_fire`, `frames_missing`, `calibration_missing`, `timebase`,
`jitter_high`, `imu_stall`, `nav_stale`, `exif_offset`, `disk_low`,
`sdk_error`, …). Never log secrets.

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
  frame age) with slick live sparklines.
- **Nav** — location plot (track from lat/lon), heading rose, depth readout.
- **IMU** — live attitude (pitch/roll/yaw) horizon + heading.
- **Controls** — the one desired-settings panel that drives all cameras, with
  per-node convergence badges and the EV-bump control.
- **Events** — the live event/anomaly stream.

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

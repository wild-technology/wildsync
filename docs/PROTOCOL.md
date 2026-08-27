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
  Capture-format readback (2026-08-23): `filetypeValue`, `imagesizeValue`,
  `transsizeValue`, `rawtypeValue`, `qualityValue`, `pcsaveValue` — each with a
  matching `<name>Label` string — plus `expcompValue` (signed mEV, no label),
  and `writable.{filetype,imagesize,transsize,rawtype,quality,pcsave,expcomp}`
  enableFlags (2 = DisplayOnly). **A `<name>Value` key is OMITTED when the body
  did not answer that property read** (older firmware, property hidden in the
  current mode, table locked in transfer mode). A present key is always a real
  value, never a `0`/`-1` placeholder: readers take a missing key as *blind*,
  never as a divergence. Older ilxctl builds omit all of them.
- `GET /api/status` may also answer **degraded**: `{connected, busy:true,
  model:"", id:"", log:[…]}` when the SDK mutex is held past 4.5 s (a wedged
  SDK call, a long card-index build). It carries no property keys and no
  `writable` map. `busy:true` means "the node answered and told us nothing" —
  do not reconcile against it, do not count the missing keys as divergence, and
  do not read it as a disconnect.
  It also carries `sdkOp` (the SDK call holding the camera), `sdkHeldS` (how
  long) and `sdkOverdue` (held past the 10 s wedge threshold), read from
  atomics so they answer while the mutex is wedged. **State-machine rule
  (2026-08-24):** a degraded status can never promote a body to
  `CAM_CONNECTED`; it HOLDS an existing `CAM_CONNECTED` for up to 20 s
  (`NodeMonitor.BUSY_HOLD_S`) so an ordinary SDK stall does not flap a healthy
  body out of a live run's roster, and past that the node is **REACHABLE** with
  an `ilx_busy` warn — reachable, not disconnected. The host reconcile skips
  such a node entirely, exactly as it skips one in transfer mode or under a
  drain, and `body_locked` is suppressed while a node reports `busy:true` or
  sits in transfer mode (diagnosing "the card has stalled, reformat it" from an
  ABSENT property table is the reader-contract violation this paragraph
  forbids). Every status body — full, connecting and degraded — also carries
  `afAllowed`; `false` is the field state, and `true` means that node was
  started with `--allow-autofocus` and can be put into autofocus by anything
  that can reach :8080.
- `POST /api/connect` → `{ok,model}` | `{ok:false,error}` (blocking, seconds).
  While a connect is in flight (including the startup autoconnect) it answers
  **409 `{ok:false, pending:true, error:"connect in progress"}`** immediately —
  a single-flight gate, so a stuck `SDK::Connect` can no longer consume the
  worker pool and take `/api/status` down with it. `pending:true` means "not
  yet, poll again": no backoff escalation, no "dead SDK handle" diagnosis. A
  connect that finds the camera already up now answers `{ok:true,model}` (the
  SDK's own `CrReconnecting_ON` won the race) instead of
  `{ok:false,error:"already connected"}`. The same 409 comes from
  `/api/disconnect`, `/api/card/mode` and every camera-op endpoint.
- `POST /api/disconnect`
- **Any SDK-facing endpoint may answer 409 `{ok:false, busy:true,
  applied:false, error:"SDK busy: …"}`.** It means the request NEVER REACHED
  THE BODY and nothing was refused: it must not be recorded as a divergence, a
  rejection or a `blind_fail`, and it is safe to retry. Only the ACQUISITION of
  the SDK mutex is bounded (8 s, and not at all for a holder already past the
  10 s wedge threshold) — once the lock is held the SDK is waited out however
  long it takes, so a merely SLOW write still returns its real result and no
  divergence is invented.
- `POST /api/shutter` `{af:bool}` — USB release. **`af:true` is refused 403**
  (operator rule: always manual focus, never AF on any path). S1=Locked *is*
  the half-press, so it is a second autofocus path and is closed like the
  first. `POST /api/focus/mode` accepts only `1` (`CrFocus_MF`) — DMF and PF
  drive the lens too, so "manual-ish" is not manual — and
  `POST /api/interval/start {af:true}` is refused the same way. Every refusal
  is written to the node log tail rigd carries. The rule is enforced in the
  SDK layer as well as at the route, so a caller that never goes through
  main.cpp is covered. The only way to lift it is to restart ilxctl with
  `--allow-autofocus` (a start-up decision, so nothing reachable over :8080 can
  turn autofocus on), which prints a startup warning and shows as
  `afAllowed:true` to the whole fleet.
- `POST /api/shutter/hold` `{ms:int}`
- `POST /api/interval/start|stop` — host-timed intervalometer
- `POST /api/camera-interval/config|arm|run` — camera Interval REC
- `POST /api/exposure` `{which:"aperture"|"shutter"|"iso"|"drive"|"filetype"|
  "imagesize"|"quality"|"transsize"|"pcsave"|"rawtype"|"expcomp", value:int}`
  (`expcomp` added by the hardening pass; value = EV×1000 per Sony)
- `POST /api/focus/mode|drive|position`, `POST /api/zoom/drive|position|setting`
- `POST /api/store` `{dest}`
- **Write endpoints validate before they touch the SDK.** A present-but-
  non-numeric, fractional or non-finite `value`/`mode`/`step`/`speed`/`dest` is
  refused `400 {ok:false, error:"bad \"value\": not a number: \"abc\""}`; an
  *absent* optional field still takes its documented default.
  `POST /api/exposure` requires BOTH `which` and `value`, and additionally
  refuses a value the body does not offer when the SDK publishes a choice list
  for that property, naming the offered values. Range-typed properties
  (`colortemp`) and bodies that publish no list pass through unchecked. A typo
  used to reach the camera layer, stall the SDK ~6 s and flap the node.
- `GET  /api/shots` → `[{name,size},...]` (PC-saved frames dir). `rigcore.NodeMonitor.shots()` returns **`None`, not `[]`, when the call fails**: a failed listing and an empty spool are different facts, and conflating them let a worker adopt every pre-existing file on the node as survey data (and let a calibration's before/after diff delete real frames). Callers must treat `None` as "unknown, retry".
- `GET  /api/card/list` → one row **per file**: `{contentId, fileId, fileNumber,
  dirNumber, name, size, format, filesInContent, captured_utc}`. Files of one
  shot (RAW + card JPEG) share a `contentId`; `filesInContent` is how many.
  **`POST /api/card/delete` is per-CONTENT**, so a drain must pull and verify
  every file of a content before deleting it, or refuse the delete. A truncated
  listing can drop whole contents but never splits one.
- `GET  /shot/<name>` → image bytes
- `GET  /liveview.jpg` → current live-view JPEG (503 when unavailable)

## piagent HTTP API (each Pi, :8081) — new, Python stdlib only

- `GET  /health` → `{node, uptime_s, gpio:{chip,ok,focus_held,focus_held_s,
  monitor_running}, imu:{present,rate_hz,age_s,…}, disk_free_mb, load1,
  time:{epoch, source, work_ms}}`. **`time.epoch` is stamped FIRST**, before the
  spool listdir / GPIO / IMU work, and `time.work_ms` reports how long the rest
  of the body took — rigcore derives the node offset as `time.epoch` minus the
  RTT midpoint, and the stamp used to be the last field built, biasing it by
  ~half the round trip on a node with a big spool.
  `imu` additionally carries `attitude_hz` (ring publishes/s — the sample
  cadence the staleness budget is sized against), `frame_hz` (all decoded
  frames/s — link health), `imu_rate_low` (**changed 2026-08-24:** true when measured `attitude_hz` falls
  below THIS device's own floor, or `frame_hz` below the 60 Hz HANDOFF figure —
  60 Hz is the FRAME rate, and this unit emits one quat + one euler per ~25 Hz
  cycle, so its attitude cadence is ~50 Hz by construction and the old fixed
  60 Hz attitude threshold flagged a permanently-sick healthy IMU from the
  first closed rate window. `/health` now also publishes `attitude_floor_hz`
  and `frame_floor_hz` so the verdict can be checked rather than trusted;
  **null** while the first measurement window is still open — null is
  "not measured", not "clear"), `attitude_age_s` (age of the last RING publish;
  plain `age_s` is refreshed by inertial-only traffic and cannot show an
  attitude stall), `rejected_frames` and `checksum:{algo,dormant,learning,
  rejects}`. `rate_hz` keeps its meaning except that a *measured* 0.0 now reads
  as 0.0 instead of falling back to the probe figure.
- `POST /gpio/focus` `{hold:bool, ttl_s:float=30}` → `{…, focus_held_s}`. A hold
  is a **lease**, not a latch: it auto-releases after `ttl_s` (clamped 1–600 s)
  unless renewed, and logs `focus_hold_expired` with `held_s`. Renew by
  repeating `{hold:true}` (omitting `ttl_s` keeps the granted length) or with
  any `/gpio/fire` that relies on the hold (`focus_lead_ms == 0` — that is the
  calibration path, so a long calibration is safe). A fire carrying its own
  `focus_lead_ms` does NOT renew. A dead host can no longer leave a body
  half-pressed and AE-locked for the rest of a dive.
- `POST /gpio/fire` `{at_epoch:float|0, pulse_ms:int=5, focus_lead_ms:int=0,
  strobe_at_epoch:float=0, strobe_pulse_ms:int=5}` → busy-wait to
  `at_epoch` (0 = now), pulse TRIGGER LOW, return
  `{ok, requested_epoch, actual_epoch, late_ms, fire_seq, edge_seq,
  node_epoch}`. **`node_epoch` is on EVERY answer**, refusals included (no
  GPIO, FOCUS-not-held 409, busy 409, epoch fault) — a refusal is exactly when
  the host needs this node's clock; the epoch fault adds `clock_skew_s`.
  Accepted `at_epoch` band is **−2.0 s … +5.5 s** on the NODE clock
  (`SYNC_LEAD_S` 0.30 + the host's 5 s offset-compensation clamp + transit
  margin). The contract the host relies on: 0.3–5.5 s ahead is always accepted.
  It was +10 s — five times the host's own 2 s fire timeout, so a node seconds
  behind fired orphans the host had already abandoned.
  Requires FOCUS held (409 if not) unless `focus_lead_ms` asks
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
  line is claimed lazily on the first scheduled strobe — plus `focus_held_s`
  (float|null) and `edges_hw_rejected` (edges published WITHOUT `epoch_hw`).
  **Changed 2026-08-24:** the band was −5 ms … +250 ms *against the read
  instant*, which is not a stamp-quality metric at all — it IS the gpiomon
  pipe-read latency, whose measured median is 0.09 ms (cam1) / 0.32 ms (cam2)
  **with documented excursions into the hundreds of ms under load**. A 0.25 s
  ceiling therefore cut inside the measured distribution, and in the wrong
  direction: during an excursion the kernel stamp is the CORRECT value and
  `epoch` is the late one, so the band discarded the good value and left the
  host the bad one. The band is now −5 ms … +5 s and rejects only a wrong
  clock domain. Alongside it: `edges_hw_reject_reasons` (reason → count;
  reasons `no_stamp` / `stamp_ahead` / `domain`), `edge_hw_lag_ms_max` and
  `edge_hw_lag_ms_last` (the read-latency excursion itself, visible before it
  reaches a capture instant).
  A fall opens the `fire_seq` window and the FIRST rise closes it (an exposure
  has exactly one rise), with a 10 s cap — a spurious rise minutes later no
  longer inherits the open id and displaces the genuine one.
- `GET  /gpio/exposure/events?since=<idx>` →
  `{next, hw_meta:1, events:[{i, edge:"fall"|"rise", epoch, epoch_hw,
  hw_lag_ms, hw_err_ms, hw_reject, raw_ts, fire_seq}]}`
  from a persistent gpiomon on BCM22. fall = exposure start.
  - `hw_meta` (envelope) — present ⇔ this node publishes the four per-edge
    `hw_*` keys on EVERY event. **Absent ⇒ an older piagent**, where `epoch` is
    the only stamp there has ever been; keep the legacy behaviour. This marker
    exists because "field absent (old node)" and "field null (this node refused
    THIS stamp)" need opposite treatment and were otherwise indistinguishable.
  - `epoch_hw` (float|null) — the kernel interrupt instant in wall time. The
    capture instant the host should use.
  - `hw_lag_ms` (float|null) — the measured pipe-read latency, `epoch −
    epoch_hw`. Diagnostic and load signal, **not** an error bar. Non-null only
    when `epoch_hw` is.
  - `hw_err_ms` (float|null) — the node's own bound on `epoch_hw` (half the
    wall bracket it converted through, floored at half a clock tick).
    Single-digit microseconds on the Pis. Add it to the fleet clock error.
  - `hw_reject` (string|null) — null when `epoch_hw` is published; otherwise
    `no_stamp` (gpiomon printed no parseable timestamp), `stamp_ahead` (the
    stamp postdates the read — wrong clock domain) or `domain` (more than 5 s
    behind the read — a mis-scaled legacy single-field line).
  - **Host rule (required).** An edge with `epoch_hw` null on an `hw_meta` node
    must NEVER be written `capture_source=gpio_edge` with the fleet clock error
    as its whole bar: the `epoch` left over carries an UNMEASURED read latency.
    `run.py` marks such an edge *soft* and publishes it as
    `capture_source=gpio_edge_soft` with a 0.25 s bar — or falls through to
    EXIF when the body's MEASURED EXIF bar is genuinely tighter (SubSec, ~10 ms;
    without SubSec the EXIF tier is worth 1.5 s and falling through would make
    the row worse). A soft edge is also refused by `match_rise` (a ~13 ms
    exposure window cannot be measured with it), and is never folded into the
    trigger-latency median or the EXIF clock offset, which are per-node
    constants applied to the whole run.
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

**Fields with no readback (superseded 2026-08-23 for most of them).**
`filetype`, `imagesize`, `transsize`, `rawtype` and `expcomp` now DO have a
readback on current ilxctl builds, and convergence verifies them after every
push — see the `<name>Value` keys above. The blind path below remains, and is
selected **per field per node** by whether the key is present in that body's
`/api/status`: an un-upgraded node still has to shoot the survey's file type.
Blind convergence tracks a field against a per-node cache of what was last
pushed. Three rules make that cache honest: it is cleared on every
`OFFLINE→CAM_CONNECTED` transition, cleared whenever a *readable* field is found
reverted (a body can power-cycle between two polls), and **never** written for a
push that failed or that never reached the body. A refused blind field is
reported in `convergence.diverged` with the body's error in `blind_errors` — it
is invisible to the readback check by construction, and a body that silently
rejects `filetype` means the two cameras record in different formats.

**The reset tell is a CHANGE, not a standing mismatch.** The in-pass "a readable
non-exposure field reverted → the body rebooted → re-push the whole exposure
vector" rule excludes: a field the build does not report; a value at or below
ilxctl's `statusJson` "never read that property" sentinel (`drive` 0,
`store_dest` 0, `focus_mode` 0, `wb_mode` −1, `colortemp` 0 — **not** `transsize`,
whose 0 is a legal *Original*); a field whose `writable[]` flag is DisplayOnly;
and a field already in the previous pass's `diverged`/`unsettable`. Without
those exclusions every idle 3 s pass was a forced pass, overwriting the
deliberate per-camera exposure split continuously. If ilxctl's sentinel defaults
change, that table changes with them.

**Only an explicit apply writes exposure to the fleet.** `reconcile_all` takes
`exposure=` separately from `force=`: a format / white-balance / focus-mode
apply re-pushes only the fields the operator actually applied, an EV bump
touches `expcomp` alone, and releasing a preview pin re-converges only the
camera that was pinned. The reconnect tell and the reset tell above are the only
implicit exposure writes left.

**`focus_mode` is refused unless it is 1 (MF)** — on `POST /api/settings`, on
`POST /api/focus/mode`, and on `desired.json` load (which is corrected to MF and
reported). The rig is always manual focus on every path: AF silently changes the
stereo geometry between calibration and the frame. `POST /api/capture {af:true}`
is refused 400 for the same reason.

**A node being drained is skipped entirely.** A monitor with `suspend_control`
set, or whose cached status reports `controlMode == "transfer"` (a crashed drain
or a rigd restart mid-drain), is left alone by the reconcile — badge frozen as it
stood, no pushes into the RemoteTransfer session, no `settings_divergent` raised
off transfer-mode readbacks. On release the blind-field cache is dropped (the SDK
session was torn down and rebuilt, so the cache is no longer evidence) but the
exposure vector is NOT re-forced: a drain changes no capture setting, and
re-forcing is what used to wipe a deliberate split after every drain.

**`quality` and `pcsave` are opt-in.** They converge and verify like any other
field once pinned by `POST /api/settings`, and are absent from the default
desired vector, so the fleet does not start converging a property nobody chose.

**`~/rig/desired.json` is validated on load and never overwritten blind.** Every
loaded value goes through the same validator as the POST path (no choice list —
no camera may be connected yet), so an illegal ISO, an out-of-range drive or a
string `"1100"` falls back / coerces with a per-field reason, and unknown keys
are dropped; the refusals go out as one `settings` warn event. A file that
exists but will not parse is renamed aside to `desired.json.bad-<ts>`, reported
at `sev=error`, and the process runs on the built-in defaults **in memory only**
— the old code fell back and immediately saved defaults over the operator's only
copy of the survey vector. The built-in default is now the survey vector itself
(F8 · 1/200 · ISO 400 · MF · 5600 K · RAW+JPEG · imagesize S · transsize Small ·
rawtype LossLessL · store card+PC); it used to be JPEG-only/L, so a lost settings
file silently turned RAW off.

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
| `timebase` | the master clock changed mid-run; the run keeps its start-of-run base (once per RUN, not once per process) |
| `host_clock` | the rigd host's clock is >100 ms off the nodes; the fire schedule and every capture instant are being corrected for it (once per run) |
| `capture_paused` | also fires now for an OFFLINE run member, and for a node that answers `/health` but refuses a probe fire |
| `strobe_fail` | also fires now when the strobe node is OFFLINE and the shot is therefore unlit |

Fleet/hardware alert kinds:

| kind | raised when |
|---|---|
| `runs_disk_low` | the **Jetson** volume holding `~/rig-runs` is below 5 GB. This is the volume every transect is written to; a node's `disk_free_mb` is that Pi's PC-save spool and says nothing about it. When it fills, each frame write raises OSError and that frame is **never retried** — no flight_log row, no nav/IMU correlation |
| `card_write_stuck` | `slotWriting` has read `WRITING` continuously for >30 s (critical at >120 s). One frame wedged in the body's write buffer locks the whole property table, stops PC delivery and wedges ilxctl **while `slotStatus` still reads OK** — do not trust slotStatus to catch it |
| `camera_overheating` | `overheatingLabel` is `pre-overheat` (warn) or `OVERHEATING` (critical); the body shuts itself down mid-transect |
| `pc_control_lost` | `priorityKeyLabel` reads `Camera position` — PC Remote priority was taken back by the body, so writes are refused and PC save stops delivering. Masquerades as an SDK bug |
| `preview_pinned` | a settings preview is being held on one camera, so the pair is not matched |
| `host_clock_offset` | the **host** disagrees with the nodes: the median of the live nodes' filtered offsets is >0.1 s (warn) / >0.5 s (bad). `node_clock_skew` compares the nodes to *each other* and reads 0 for a common-mode host error, which is what hid the measured 187 ms. Action: enable network time on the host (System Settings → General → Date & Time) |
| `node_clock_unmeasurable` | a node's best `/health` RTT is ≥20 ms, so the 10 ms stereo budget cannot be verified on that link. Previously such a node was silently dropped from the comparison and the operator saw an all-clear on a check that was never made |

**Published but NOT yet detected.** These three faults are now *measurable* —
piagent and nav publish everything needed — but rigd's anomaly scan does not
raise them yet. They are named here so the fields do not drift before the
detectors land, and must not be described to an operator as working alarms:

| would-be kind | the field that already answers it |
|---|---|
| `nav_time_unconfirmed` | `nav.health()["time"]["unconfirmed_for_s"] > 10` with `candidate_pending` |
| `imu_rate_low` | `/health` `imu.imu_rate_low` is True (null = not yet measured, **not** a clear) |
| `focus_stale_hold` | `/health` `gpio.focus_held_s` running outside a calibration window — a half-pressed, AE-locked body |

`node_clock_skew` now reads the RTT-gated median (not the last raw `/health`
sample) and requires the disagreement to persist across **3 consecutive scans**;
the per-pair streak resets the moment one scan comes back inside budget, and the
evidence carries `scans`/`samples`. Single-sample "82.9 ms" alarms were being
raised while `chronyc` had the two Pis 20 µs apart. `nav_no_fix` is reachable
during a run (bad severity) — it used to sit behind `nav_gateway_down`'s `elif`
and never fire.

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
`transsize` (populated on current ilxctl builds, null on older ones), and the
read-only per-camera lens readings `focus_mode`, `focus_mode_label`,
`focus_pos`, `zoom_pos`, `zoom_setting_label`. Clock: the raw last-poll
`clock_offset_ms`/`clock_rtt_ms` **and** the filtered `clock_offset_s` +
`clock_offset_info{offset_s,n,rtt_ms_best,age_s}`. Both are published on
purpose — the raw one is one sample, the filtered one is what every decision is
made on. Sign convention is **node minus host** throughout.

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
  `POST /api/reconcile`. `{on}` and every other boolean body field
  (`/api/run/stop {drain}`, `/api/drain {keep}`, `/api/node/focus {hold}`,
  `/api/capture {af}`) must be a real JSON boolean or 0/1 — `bool("maybe")` is
  True, and a truthy string is not consent.
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

`/api/run/detail` also reports `index_source` (`"index.jsonl"` | `"run.json"`)
and recovers `frames_indexed` from the index when `run.json` is missing or
corrupt. A shot only ONE camera recorded reports `spread_ms: null` and
`spread_src: "single"` — `max()−min()` over a single epoch is 0.0, and
publishing that showed a measured green "0.00 ms" for exactly the shots where
the pair is broken. `/api/runs` entries may carry `frames: null` when `run.json`
is missing: null means *unknown, may still hold frames*, 0 means *definitely
empty*, and the UI picks which run to review on that distinction.

**Input validation (2026-08-23).** Every POST body must be a JSON **object**: a
malformed body, a bare array/string/number, or a literal `null` is a clean
`400 {ok:false,error}`, no longer silently treated as `{}`. A truncated
`POST /api/run/start` body used to be indistinguishable from the UI's empty one
and recorded a whole transect on the built-in defaults. A body over 1 MiB is
refused without being read. Bad query and field values are 400s naming the
parameter, not 500s carrying a Python exception (`?since=abc`, `?t0=nan`,
`?node=cam9`, `steps="x"`, `samples=0`). 400s are deliberately NOT journalled,
so a fuzzer cannot fill the event ring.

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
floor (warn >5 ms, bad >8 ms) — since 2026-08-23 off the RTT-gated median and
only after 3 consecutive scans. Every scheduled fire and every `epoch_hw` edge
lives on the NODE's clock, so inter-node clock skew lands 1:1 in true
exposure skew **while the rig's own skew figures under-report it** — the
nodes' chrony must share one master (measured 16.8 ms apart free-running on
2026-08-20 after the Jetson master left the topology). A *common-mode*
host-vs-node offset is invisible to this check by construction; that is
`host_clock_offset`, and the conversion applied to both domains is under
"CLOCK DOMAINS" below.

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
  next `/api/connect` instead of refusing "already connected". **Updated
  2026-08-23:** a concurrent `/api/connect` in that startup window now gets the
  409 `pending:true` above, not "already connected" — it used to run a SECOND
  `EnumCameraObjects` and `Release()` the info object the pending connect was
  still holding, then report a false dead handle.
- `POST /api/spool/prune` `{confirm:"prune", older_than_s=86400, keep=50}`
  moves old PC-save files into `<dir>-archive` (never deletes).
- Ingest: `rig/ingest.py <card_dir>` matches card files to flight-log rows
  (by file number when the card counter equals the PC-save counter, else by
  the measured clock offset with rigd's journaled `exif_offset` as the
  tie-break), places `<base>.ARW` / `<base>.card.JPG` / `<base>.xmp`
  (true instant, GPS, UTM, attitude, pair spread) beside the review JPEG,
  writes `ingest_manifest.csv`. `rig/stereo_check.py` verifies pairing by
  relative-pose consistency against a mis-paired control.

### 2026-08-23 audit pass — frame index, drain control, nav provenance

- **`<run_root>/index.jsonl` is the authoritative frame index.** One JSON object
  per indexed frame per line — the same fields as a `run.json` index entry
  (`cam, file, orig, epoch, src`, optional `path`/`rise`/`strobe`/`clk_off`)
  — `clk_off` is the fleet clock offset that frame's instant was actually
  converted with (seconds, 6 dp; absent when no conversion was applied, i.e.
  the card-review fallback), which is what makes a post-hoc re-base exact and a
  mid-run host-clock step visible by inspection — appended and
  flushed at index time. `run.json` is unchanged: `index` is still the LAST 2000
  entries, `frames` the full count, plus an `index_jsonl` pointer. **Readers
  prefer `index.jsonl` and fall back to `run.json`'s capped index**; a torn last
  line from a crash mid-append is expected and must be skipped, never guessed
  at. Reading the truncated tail instead cost a 2100-frame run its first 100
  shots and 50 stereo pairs, and dropped every earlier frame to the
  centisecond-quantised flight_log stamps. `rigcore` (run browser), `ingest` and
  `stereo_check` all go through it; bounded at 64 MB.
- **`POST /api/drain/cancel`** → `{ok, cancelling:<node>}` or
  `{ok:false, error:"no drain is running"}`. rigd creates a `threading.Event`
  and passes it to `Drainer.run(..., stop=ev)`; the drain finishes the content it
  is on and returns with `cancelled:true`, queued nodes are skipped with an
  event. **Cancellation is honoured at content boundaries only** — never between
  a pull-verify and a card delete, so a cancel can never cost a card original. A
  full card is 10–15 minutes during which a run start is refused, and the only
  escape was `launchctl kickstart -k`, which SIGTERMs the drain mid-pull.
- `GET /api/drain` → `{active, node, queue[], skipped{node:reason},
  last{node,at,pulled,bytes,deleted,verified,cancelled,errors},
  cancel_requested, wedged{node:{at,at_iso,rebooted_at}}}`. **The drain report's
  counts are per CONTENT** (one shot = its RAW + its full-size card JPEG), not
  per file: `rep["files"]` lists every file written to the host, `rep["bytes"]`
  their total. A content is deleted from the card only once **every** one of its
  files is sha256-verified on host disk; a content whose transfer partly failed
  now survives on the card instead of being deleted. Host names may carry a
  `-c<contentId>` (or `-c<contentId>-<sha8>`) suffix when a same-named file with
  different bytes was already there — nothing verified is ever overwritten.
  An auto-drain skips a node whose last report said "card index not ready" until
  the monitor SEES that node power-cycle; a manual `POST /api/drain` overrides.
- `POST /api/run/stop` → `drain_started` is now the TRUTH (it was hardcoded
  `true`) plus a `drain` object carrying `start_drain`'s full result including
  per-node skip reasons. Every refusal also emits a warn event naming the node
  whose card was NOT drained.
- `GET /api/nav` gains **`fix_kind`: `"live" | "static" | "none"`**, and a static
  row explicitly carries `valid:false` — it only omitted `valid` before, so
  anything defaulting on "lat is not None" wore the green fix pill and plotted a
  constant as a track. `fix_at()` rows carry `fix_kind` too, and `sog_mps` is the
  field name (there is no `sog`). **The static fix is an ARMED FALLBACK decided
  at run start** from whether the gateway was online at that instant: it never
  fills a mid-run gap. A transect that starts on good GPS and loses the bus for
  40 s writes EMPTY lat/long for those frames and raises `nav_no_fix` — a
  constant position is worse than empty, because the flight_log then looks
  complete.
- `nav.NavReader.health()["time"]` gains `from_src` (the N2K source address),
  `senders[]`, `disagreement{chosen,others,spread_s,since}`, `last_warning`,
  `unconfirmed_for_s`, `latency_bound_s` (0.5 s — the ASSUMED
  sender→bus→gateway→USB delay), `latency_subtracted_s` (**always 0.0**: nothing
  is subtracted until something measures it), `offset_scatter_s` (measured) and
  `abs_error_bound_s`. A GPS-corrected stamp is absolute to ~0.5 s, not to
  milliseconds; the pair's 10 ms co-exposure requirement is unaffected because
  it is a node-to-node comparison. Confirmation now requires two agreeing feeds
  from the **same** `(pgn, src)` — two disagreeing senders used to starve each
  other's single candidate slot and reject good GPS for a whole run, silently.
  `candidate_pending` changed meaning: it is real starvation now, not "the last
  feed was not the confirming one". `fix_at(epoch)` judges time source, offset
  and gateway state **at that instant**, so a frame pulled seconds after its
  exposure keeps the authority that held when it was exposed.
- Run ids: the character class is `^\w[\w.\-]{0,79}\Z` with `re.UNICODE`, so
  transects already on disk with accented labels are browsable, openable and
  recoverable. New ids should be ASCII — `rigcore.run_id_slug(label)` is the
  single sanitiser (NFKD-folds `Récif-Nord` → `Recif-Nord`). The guard is not
  weaker: `\w` never matches `/ \ . " < > &` or NUL, a leading dot still fails
  the first atom, and `\Z` closes the trailing-newline hole `$` left open. The
  realpath containment check is unchanged.
- `POST /api/run/start` gains `warnings[]` and `host_offset_s`, and can return
  `{ok:false, error}` for an invalid `interval_s`/`frames`/`label`: interval
  refused below 0.20 s (piagent's fire lock provably overlaps below it) or above
  3600 s and **warned** below 0.50 s (where the fire schedule loses its dispatch
  margin), frames 0…100000, labels stripped of control characters and capped at
  60. `sync.paused_for` carries `backoff_s`/`next_probe`.
- **A run member that is unreachable is a FAILED fire, not an absent one.** It
  was simply dropped from the roster, so `failed_fires` stayed 0 and the
  surviving camera shot the rest of the line alone. Resume is gated on a
  successful **probe fire** on the returning node — answering `/health` is
  necessary, not sufficient — backing off 2 s → 60 s while it refuses; the
  probe's own frame is quarantined as a calibration frame and never enters the
  transect.
- **RAW+JPEG is one shutter release: one claim, one edge, one flight_log row.**
  Claims are keyed by camera stem (`ILX01234` from `.JPG`/`.ARW`/`(1)`
  duplicates), the JPEG carries the row and defines the `CamN_<instant>` base
  name, and the RAW is archived beside it under the same stem with no second row
  and no second index entry. Frame counts still count exposures, not files. If
  only a RAW arrives it takes the row rather than the shot vanishing.
- `rig/ingest.py` returns (and logs) a summary — `{runs:[{run_id, summary}],
  leftover, totals{runs,matched,unmatched,raw,conflicts,cards,cards_timed,
  leftover,exif_mismatch,ambiguous}}` — so a drain that matched nothing is no
  longer invisible. It reads EXIF straight out of the `.ARW` when there is no
  JPEG, and **refuses** a combined two-body directory when the series cannot be
  attributed (`"ambiguous: DSC/_CA"`): both bodies fire the same instants, so
  attribution would be a coin flip. Recovery is per-node directories
  (`~/rig-raw/cam1`, which the drain always produces and which is authoritative)
  or rigd's journalled `exif_offset`. A destination holding different bytes is
  never overwritten — the new file is parked as `<base>.conflict-<sha8><ext>`
  and the source is kept even under `--move` — and a third-party `<base>.xmp`
  keeps its name while the rig's packet goes to `<base>.wildsync.xmp`.

Transect run layout (Jetson, `~/rig-runs/`):

```
runs/260815_1930_transect-01/
  run.json            journal: config, nodes, events, per-frame index (last
                      2000), alerts, the timebase and clock offsets applied,
                      and per-shot sync quality
  index.jsonl         the COMPLETE frame index, one JSON object per line,
                      appended as each frame is indexed — readers prefer this
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
overlap — and a camera **adopted mid-transect while a grid is running is
deliberately not EXIF-calibrated at all**: its `before` listing cannot tell its
own calibration frame from a scheduled fire landing in the same second, and the
puller would then delete a real survey frame. It raises `calibration_missing`
saying exactly what is unmeasured; trigger latency is still adopted from the
persisted same-body figure (which fires nothing). Calibrations are serialised
per node, and the quiet window runs until the frame is seen or `CAL_QUIET_S`
(3 s) — a flat 1.2 s covered a Small JPEG but not an Original-size frame on a
Pi 4, which then listed after the quiet ended and entered the transect as
survey data.

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
never fabricate. `capture_source` = `gpio_edge|gpio_edge_soft|exif|command`.
`gpio_edge_soft` is a real EXPOSURE edge on a node that could **not** stamp it
in the kernel (`hw_meta` present, `epoch_hw` null, `hw_reject` says why): the
instant is gpiomon's pipe-read stamp, so `time_err_ms` carries the documented
read excursion (0.25 s) instead of the clock error, and consumers that treat
`gpio_edge` as hardware-timed — the strobe verdict, the "measured" pair spread
— correctly decline it. `time_source` =
`gps|jetson`. The 23 columns are unchanged. **Behaviour change 2026-08-23:**
`time_err_ms` on a `gpio_edge` row is no longer `0.0` — it carries the
host↔node offset's own uncertainty (half the best RTT in the filter window,
typically well under 1 ms). `exif` and `command` rows add the same term.

Time model: GPS time (PGN 126992/129029) when the N2K network is live →
`gps_offset = gps - jetson`; all log stamps corrected by it. No fix → offset 0,
`time_source=jetson`. Camera EXIF offset measured per node at run start
(calibration frame: fire, read EXIF DateTimeOriginal+SubSec, diff vs
GPIO exposure edge when available, else command epoch + 20 ms). Frame capture
instant preference: GPIO EXPOSURE fall edge > corrected EXIF > command time.

**Where the correction is applied, and where it must not be.** `gps_offset` is
applied exactly **once**, at the presentation boundary: the `datetime` column
and the `CamN_` filename. `time_source` names the offset actually applied to
that row, never a second, later reading. A pull that outlives `stop()` keeps the
run's latched base — a worker latches `(gps_offset, source)` at construction, so
the last frame of a transect cannot be stamped from a base the rest of the log
never saw.

**CLOCK DOMAINS (2026-08-23 — this replaces "every epoch handled internally is a
raw local epoch").** The two Pis are chrony-locked to each other (~0.6 ms) but
the macOS host's clock is NOT disciplined — measured 187 ms behind NTP and
drifting ~60 ppm, which overran the fire schedule (both nodes reporting
`late_ms` ≈ 33 ms on every fire, skew degraded 0.59 → 1.78 ms) and put every nav
lookup ~190 ms out (19 cm at 1 m/s). There are now two named domains, and
`rigcore.NodeMonitor.clock_offset_s()` is the one bridge between them: the
**filtered** node-minus-host offset, the median of the last ≤8 samples whose
`rtt_ms` is within 2× the best RTT of that window and younger than 60 s, or
`None` when nothing is usable. `clock_offset_info()` gives
`{offset_s, n, rtt_ms_best, age_s}` for diagnostics.

| domain | what lives there |
|---|---|
| **NODE clock** | fire targets sent to piagent (`at_epoch`, `strobe_at_epoch`), GPIO edge `epoch`/`epoch_hw`, piagent `/health` `time.epoch`, the IMU ring |
| **HOST clock** | `time.time()` in rigd, nav's ring and `gps_offset`, everything written to `flight_log.csv`, `run.json` and `index.jsonl` |

- A fire target is `time.time() + SYNC_LEAD_S + offset`, where `offset` is the
  **median of the live members' filtered offsets**, clamped to −1.0…+5.0 s. ONE
  common value per shot, never per-node: the offsets are common-mode anyway, and
  a per-node estimate would inject its own noise straight into inter-camera skew.
- A capture instant is converted the other way — node epoch **minus the same one
  fleet offset**, latched **on the shot**: `capture_once` reads the offset and
  its error once per shot and carries both on each node's command record, and
  `_capture_instant` prefers those. (It used to be latched on WALL TIME —
  `fleet_clock_offset`, TTL 2 s — which is not shot identity: the two halves of
  one pair are pulled by independent worker threads, so a fire boundary falling
  between their conversions converted cam1 with L_k and cam2 with L_k+1 and put
  the difference straight into the displayed pair spread. The TTL latch remains
  only as the fallback for a frame with NO command — an unscheduled release, a
  card review, a pre-adoption backlog.) —
  before anything uses it: `nav.fix_at()`, the `CamN_` filename, the flight_log
  row, `run.json`, `index.jsonl`. It is emphatically **not** per-node: the
  displayed pair spread is computed from the indexed epochs
  (`RunBrowser._pairs`), so converting each camera with its own estimate would
  put the *difference of two estimates* (each worth ~RTT/2) straight into the
  one number this rig exists to keep under 10 ms. One shared value cancels out
  of the difference exactly. `match_rise` and piagent's `strobe_epoch` get that
  same offset, or the `[fall, rise]` window would be shifted and the strobe
  verdict would lie. The IMU ring is node-keyed, so `imu_snapshot` converts
  *back* to ask and forward again to record — with the identical number, so the
  round trip is an identity. `node_clock_offset()` still exists, but it is
  diagnostics only (`run.json`'s per-node block) and stamps nothing.
- **The one exception:** the card-review fallback (no command, no edge, no EXIF)
  is already a host instant and is not converted. The IMU query on that row
  still uses the real fleet offset — the ring is node-keyed, so converting
  *outward* is mandatory there even when the instant itself needs none. That
  offset travels as `None`, not `0.0`: `0.0` is right for the reverse
  conversions applied to values received FROM the node, but on the outward leg
  it asserts node == host and centres the ±100 ms window a whole fleet offset
  (187 ms on this rig) into the node's past.
- `run.json` records what was applied: `clock: {node_offsets_s:{node:seconds},
  host_offset_s, applied:true, applied_offset_s:{first,last,min,max,moved_ms}}`.
  A node with **no measurement** publishes `null`, not `0.000000` — the latter
  is indistinguishable from a measured zero beside a real +0.187.
  `applied_offset_s` is the SPAN of offsets actually applied to frames in the
  run, because the applied number moves across a transect and one live-read
  scalar could not describe it, and `sync` gains `host_offset_s`, so a transect
  recorded against a mis-set host clock can be re-based in post. The correction
  carries RTT/2 uncertainty, which lands in `time_err_ms`; disciplining the host
  removes the term.

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

---

## Additions 2026-08-27 — projects, diagnostics, second IMU, node recovery

### Project layer (host)

`rig/project.py` owns where data lands. A project is
`~/wildsync-projects/<slug>/{project.json, runs/, raw/, exports/}`; the active
slug lives in `~/wildsync-projects/active.json`. With no active.json the
implicit **legacy** project wraps the pre-project layout (`~/rig-runs`,
`~/rig-raw`) unchanged — nothing is migrated, ever. Switching projects
reassigns `rigcore.RUNS_DIR` (the only sanctioned writer) and is REFUSED while
a run or drain is active.

- `GET  /api/project` → `{active, explicit}` (`explicit` = an operator has
  ever chosen one; the UI shows the intro screen when false)
- `GET  /api/projects` → `{projects:[{slug,name,runs,last_run,active,…}]}`
- `POST /api/project/create {name, vessel?, site?, operator?, notes?}`
- `POST /api/project/open {slug}` · `POST /api/project/update {slug, fields}`

### Diagnostics / operations (host)

- `GET  /api/run/zip?id=<run_id>` — builds (or reuses)
  `<project>/exports/<run_id>.zip` (ZIP_STORED) and serves it with a real
  Content-Length. The export is a durable artifact, not a stream.
- `POST /api/node/format {node, confirm:"format", quick?}` — proxies the
  node's card format with rigd's guards: refused during a run, during a
  drain, and for a disconnected camera; journalled warn/info/error. The node
  side (`ilxctl /api/format`) itself completes only on the body's
  `CrWarning_Format_Complete` and re-reads RemainingNumber.
- `POST /api/power` (node, ilxctl) `{op:"off"|"on", confirm:"power"}` — SDK
  body power. **Over USB, PowerOff is one-way**: the standby body enumerates
  as a generic gadget the SDK cannot claim; wake it with a harness full-press
  (`/gpio/focus {hold:true}` then `/gpio/fire`).
- `GET  /api/props` (node, ilxctl) — the body's raw property table
  `[{code,value,enable,type,nvalues}]`; the diffing tool that found the
  movie-mode flip.
- `/api/fleet` node views now carry `remaining_shots` and `program`.

### Run integrity (contract additions)

- A mid-run camera drop **pauses** the grid; resume requires the camera
  CONNECTED again *and* a probe fire whose own frame arrives in the spool.
  A pulse alone is not proof (piagent cannot see exposures) — this is what
  ended the pause/resume oscillation that shot unpaired frames.
- Host frame writes are `.part` + fsync + rename, then the directory is
  fsynced, **before** the Pi spool copy is deleted; index.jsonl and
  flight_log.csv rows are fsynced per append. "Verified on host disk" is now
  literally true at the moment of spool deletion.
- Frames the run knowingly failed to keep are recorded in
  `<run>/unpulled.jsonl` (`{ts,cam,orig,reason,cmd_epoch?}`); ingest reports
  them and names each unmatched row's nearest staged file with its residual.
- Ingest time-matching tolerance is capped at 0.45 × the run's own
  `interval_s` (falls back to the global 0.75 s only for unknown intervals),
  so a hole in the staged card files yields honest unmatched rows instead of
  cascading every later RAW onto the previous frame's identity.
- The pull path rejects: size-0/unsized listings (retry), truncated reads,
  bad JPEG SOI/EOI, bad ARW TIFF magic, bad HEIF ftyp.

### Second IMU (piagent)

`PIAGENT_IMU2=off|olive[:device|udp[:port]|sim]` arms slot 2
(`rig/imu_olive.py`, Olive olixVision X1). Endpoints `GET /imu2/latest` and
`GET /imu2/window` mirror `/imu/*` shapes; `/health`'s `imu` section gains a
nested `imu2`. With the slot absent every payload is byte-identical to the
single-IMU protocol. See `docs/olive-imu.md` for bring-up and the fusion
(ESKF) roadmap.

### Known-physical facts recorded here so nobody re-debugs them

- cam1's card is ~64 GB class, cam2's ~512 GB class (measured: both bodies
  price a frame at ~38 MB — 33 MB LossLessL ARW + S-Fine JPEG — and report
  1325 vs 10873 shots after a format). Same settings, different cards. Swap
  cam1 to a matching V60 512 GB for long transects.
- A node service restart can leave a body in **Movie M** (still properties
  withdrawn, RemainingNumber=0, stills refused, formats still fine). The
  fleet vector now pins `program=1` and self-heals it within one reconcile.

## Additions 2026-08-27 (later) — Olive IMU live, rig calibration

### Olive olixVision X1 — how it actually connects (bring-up findings)

The unit's USB-C is a CDC-Ethernet gadget: on cam2's Pi it is netplan-pinned
as iface `olive0` (Pi 192.168.7.2/24, unit 192.168.7.100 — its own subnet, no
collision with the rig's 192.168.1.0/24). Services on the unit: 80 (olixOS
web UI, Blazor SPA — its /api/* paths are a router fallback, NOT a REST API),
22 (ssh), 8888 (JupyterLab), 4200, and WebSocket streams on 5400/5500/5530/
7070. **Port 5500 is the sensor stream**: protobuf-framed binary WS messages
(~15 Hz) carrying quaternion, accel m/s², gyro rad/s, mag mG, and identity
strings (AHRS, x1-pro). The advertised ROS 2/DDS path (RTPS on 239.255.0.1:
7400) needs a DDS stack the Pi image cannot build - not used.

`rig/olive_ws_bridge.py` (systemd `olive-bridge.service`, deployed on every
node, idles harmlessly where no Olive exists) parses that stream schema-less
and re-emits rig-unit JSON to udp://127.0.0.1:9901; piagent's imu2 slot
(`PIAGENT_IMU2=olive:udp:9901` in /etc/default/piagent, installed by deploy)
ingests it. The unit's own clock is ~2 years wrong - samples carry arrival
epochs only.

### Rig calibration (host, rig/rigcal.py, UI Calib tab)

The pair shoots through corrected underwater optics: focal length is UNKNOWN
until measured, so intrinsics come entirely from a checkerboard - no priors.

- `POST /api/rigcal/stereo/start {cols,rows,square_mm,baseline_mm?}` -
  inner-corner counts (non-square pattern enforced), measured square size,
  optional tape baseline for cross-check.
- `POST /api/rigcal/stereo/capture` - fires a genuine synchronized pair
  (capture_once), reads both frames from the node spools, detects corners.
  Detections are gated: full grid, not touching the frame edge, cell pitch
  >= 18 px (smaller boards demonstrably yield phantom half-cell lattices),
  180° corner-order canonicalized by row-direction dominance.
- `GET /api/rigcal` / `POST .../discard {index}` - live session state: 3x3
  coverage grids per camera, near/far/tilt diversity counts, and dynamic
  guidance text ("move the board to the top-right of cam2's view", "tilt
  ~30°...") until the set is sufficient.
- `POST /api/rigcal/stereo/compute` - per-camera calibrateCamera (no guess)
  + stereoCalibrate (FIX_INTRINSIC); reports fx/fy per cam, RMS, baseline,
  agreement vs tape. Synthetic validation: baseline to 0.2%, focal to ~2%
  with no prior, sub-pixel RMS (rig/tests/rigcal_selftest.py, 16 checks).
- `POST /api/rigcal/stereo/save` - writes ~/rig/stereo_calibration.json;
  vslam.load_stereo_calibration() freezes it into the stereo engine
  (baseline_calibrated=true) from the next run.
- `POST /api/rigcal/imu {target: cam1|cam2|both, seconds}` - still-window
  sampling (refused if the rig moved: gyro/accel variance gates) recording
  gyro bias, accel norm and the mounting attitude reference per camera IMU
  (cam1 YB, cam2 Olive); "both" samples simultaneously and records the
  cam1<->cam2 relative-orientation seed for the fusion filter. Saved to
  ~/rig/imu_calibration.json. First live calibration recorded 2026-08-27
  (cam1 gyro bias [-1.06,-1.27,0] dps; the Olive AHRS stream reports
  zeroed gyro at rest - on-device bias correction, noted for fusion).

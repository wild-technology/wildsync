# Changelog

All notable changes to Wild Sync are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Dates are commit
dates.

## [Unreleased]

### Added
- `rig/nav_subsonus.py`: the Advanced Navigation GNSS Compass (or Subsonus) as the navigation source over Ethernet
  (ANPP over TCP, System State packet), selected with `WILDSYNC_NAV` or
  `~/rig/nav.json` `{"source": "anpp://<host>"}`; the iKonvert path is
  unchanged and remains the default. Same `NavReader` contract, raw log and
  replay. Not yet used in the field (`docs/INSTALL.md` §7.1,
  `docs/PROTOCOL.md`, "Additions 2026-10-15").
- `rig/tests/audit_subsonus.py`: framing, decode, consumer contract, loopback
  transport, raw log replay and source selection; in the gate as
  `audit-subsonus`.

### Changed
- `nav.NavReader._handle_line` records a message through a shared `_ingest`
  and accepts `!ANPP,<id>,<hex>` lines (replay of a GNSS Compass / Subsonus run).
- `rigd` reads the nav source from `WILDSYNC_NAV` / `~/rig/nav.json`; the
  `nav_gateway_down` anomaly names the source in use.
- The navigation source over Ethernet is the Advanced Navigation GNSS Compass
  at 192.168.1.60 (static), and the ANPP reader decodes both the 100-byte
  (GNSS Compass) and the 116-byte (Subsonus) System State layouts.

### Fixed

Host (`rigd`, run management)
- Frames an adopted pull worker finds already on its node no longer claim the
  first fire command queued after adoption, which shifted every later frame of
  that camera by one shot period while still labelled `gpio_edge`. Frames in
  flight at the moment of adoption can still be affected.
- A pull worker whose `flight_log` could not be opened or written raised
  `AttributeError` instead of reporting it, killing the worker before its first
  pull or losing a frame after its spool copy was deleted. The stop path no
  longer raises when the `flight_log` has already been dropped.
- A standalone strobe (camera-less strobe node, the Jetson XR256) discarded its
  pulse instant, so no frame carried a strobe field and the per-pair LIT /
  STROBE MISS verdict never existed. The instant is now attached to every GPIO
  camera's frame record and a late flash is reported.
- `pulled` counted files rather than exposures, so a RAW+JPEG release counted
  twice, `stop()` ended its final-shot grace before the last shot was listed
  and the `frames_missing` alarm could not see a camera that had stopped
  delivering.
- `run.json` and `/api/runs` `sync.shots`, `skew_ms_max` and `skew_ms_p95`
  covered only the last 1000 shots of a run, hiding the worst skew of any
  longer transect.
- `start()` held the run lock across one blocking HTTP call per monitor (3 s
  for an unplugged slot) in `reset_edge_cursors()`, stalling status polls and
  the pull workers' bookkeeping.
- A camera switched off after three failed fires kept its stale `fail_streak`
  and held the whole capture grid paused, with no way out short of stop/start.
- A body that writes no SubSecTimeOriginal had its whole-second EXIF stamps
  published with the few-millisecond bar of the edge it was calibrated against,
  and that bar let them beat a soft edge. The calibrated bar is now 1.5 s plus
  clock error in that case.
- Frames shot between runs (bench Capture, bare calibrate) were never in
  `known_frames`, so a camera joining the next run late had them pulled into
  the transect as survey data. An idle census now records what is on each
  connected node while no run is open.
- A camera filename pulled and deleted, then reissued after the body's 9999 to
  0001 counter rollover, was skipped for the rest of the run (never pulled,
  never deleted, spool filling). Names now leave `seen` once the node confirms
  they are gone.
- A failed run-start EXIF calibration silently kept the previous run's
  camera-clock offset and error bar, so frames were stamped
  `capture_source=exif` from a stale offset while the events said it was
  unmeasured. Offsets are now cleared at run start.
- Frames still pending when a run stopped left no `unpulled.jsonl` record and
  were baselined away by the next run. They are now recorded with their claimed
  command and the stop names them.
- A pull worker that outlived `stop()` (join is 8 s, a download may take 30)
  indexed its late frame and counted its orphans into the next run. Workers now
  carry their run's id and a stale one is refused with a warning.
- A JSON string such as `"false"` armed auto-capture on `/api/run/start` and
  the strobe on `/api/strobe`, and `Infinity` escaped `int()` as
  `OverflowError`. Both fields now require a real boolean (or 0/1) and are
  refused with a clear error otherwise.
- `/api/camera/enabled` and `/api/node/format` coerced their booleans with
  `bool()`, so the string `"false"` switched a camera on (and persisted it) or
  asked for a quick format. Both now require a real boolean or 0/1 like every
  other boolean field.
- A fleet exposure apply that reached a camera held by a card drain, transfer
  mode or a busy status was reported as applied and never replayed, leaving the
  stereo pair split. The skipped fields are now owed and written, from the
  desired vector, on the first pass that can reach the body, and the apply
  reply and UI say which camera is deferred.
- A settings apply followed by a 3 s reconcile pass that still saw the
  pre-apply cached status read the lagging value as the body having reset and
  re-pushed the whole exposure vector, wiping a deliberate per-camera split.
  The reset tell now ignores a status older than our own last write.
- A verification re-read that landed in a busy spell (no property block)
  marked every readable field diverged and raised a false bad-severity
  `settings_divergent` on matched bodies. A degraded re-read now leaves the
  previous convergence verdict untouched.
- `colortemp`, `wb_mode`, `rawtype` and `program` had no bounds, so a cleared
  Kelvin box persisted colortemp 0 and was re-pushed every 3 s and a `Movie_*`
  program could enter the fleet vector. They are now range-checked like the
  other desired fields.
- A non-finite numeric token such as `{"iso":1e999}` escaped
  `_validate_field` as an `OverflowError`, giving a 500 from `/api/settings`
  and `/api/settings/preview` and, from a hand-edited `desired.json`, a rigd
  that would not start. It is now a normal rejection.
- State-changing POSTs are refused with 403 when the browser's `Origin` header
  does not match the `Host` they were sent to, so a page on another site can no
  longer start runs, stop drains or switch cameras on the rig network via a
  no-preflight `text/plain` request.
- The HTTP server had no socket timeout, so every client that vanished or
  stalled mid-body (a roaming phone UI) held a handler thread and a file
  descriptor until rigd restarted. Connections now time out after 120 s of
  silence and a timed-out body read closes the connection cleanly.
- A POST body sent with `Transfer-Encoding: chunked` or a negative
  `Content-Length` was treated as no body, so `POST /api/run/start` answered ok
  and started a default run. Unframed bodies are now refused with a 400 and the
  connection is closed.
- Pre-flight counted a connected camera the operator had switched off as a live
  camera, hiding the single-camera stereo warning and the 'no camera' block. It
  now judges only cameras that will shoot, while a switched-off node remains a
  valid strobe host.
- Concurrent `/api/run/zip` requests for the same run shared one `.tmp` file,
  giving the second request a 500 and the first a file still being rewritten.
  Exports are now serialised per run and the second request is handed the
  finished zip.
- The run zip export copied the whole run onto the survey volume with no space
  check, would export the run still recording, and leaked its partial `.tmp` on
  failure. It now refuses the live run, refuses an export that would leave less
  than the runs-volume low-water mark free, and removes the partial file on any
  error.
- Downloading the zip of a legacy run id containing non-latin-1 characters
  produced a malformed response (a 200 status line followed by a 500). The
  `Content-Disposition` header now carries an ASCII fallback and an RFC 5987
  UTF-8 name.
- A node that stopped answering on both daemons was served by `/api/fleet` as
  `connected:true` with its last battery and slot status, so the header count,
  health strip and Controls treated a dead camera as live. It is now
  `connected:false` (last-seen data kept).
- The Map tab's live breadcrumb and Follow read `nv.fix` and `nv.online`, which
  `/api/nav` never sends, so the live trail never moved. They now use the flat
  nav snapshot's live-fix test, still excluding an armed static position.

Nodes (`piagent`, `ilxctl`)
- A strobe-role piagent no longer runs `gpiomon --bias=pull-up` on offsets
  17/27 at shutdown or deploy restart. The Jetson's unrelated pads are never
  biased, matching the startup contract.
- Shutting down an active-high strobe line (`WILDSYNC_STROBE_ACTIVE=high`)
  drives it to its own idle level LOW instead of HIGH, so a piagent restart or
  deploy no longer sends a rising edge to the XR256.
- A single failed strobe-line claim is retried after a 5 s cooldown instead of
  being cached until piagent restarts.
- A `/gpio/fire` whose `strobe_pulse_ms` does not parse is answered 400 before
  TRIGGER is pulsed, instead of exposing the camera and then failing with a
  500.
- piagent answers 400 to chunked, non-numeric/negative or over-1-MiB POST
  bodies (and closes the connection) instead of treating them as empty and
  firing on defaults.
- A stopped interval's exiting thread no longer marks a restarted interval as
  not running or clamps its FOCUS lease, so the restarted run can still be
  stopped.
- Nav decoding drops impossible latitude/longitude, heading, COG and wind-angle
  values (as None) instead of reporting `valid=True` with a 150 deg latitude or
  folding a 372 deg heading to 12 deg.
- `depth_below_surface_m` is no longer derived from a keel (negative) sounder
  offset, where depth plus offset is the depth under the keel. It is None in
  that case.
- A GNSS position whose 129029 method is no-fix, estimated, manual or simulate
  is no longer reported `valid=True`, so rigd's `nav_no_fix` alarm and static
  fallback act on it instead of a frozen coordinate.
- State-changing requests to ilxctl carrying a cross-origin `Origin` header (a
  browser page on the rig LAN posting a no-preflight body) are refused with 403
  before any handler or confirm token runs. Requests with no `Origin` and the
  node's own page are unaffected.
- `/api/card/delete` and `/api/card/pull` no longer turn a missing, garbage or
  out-of-range `contentId`/`fileId` into content 0, and a `contentId` at or
  above 2^32 no longer wraps to content 0. They answer 400 naming the field,
  and an explicit 0 is still accepted.
- `/liveview.jpg` no longer re-serves an arbitrarily old cached frame as 200
  after live view stops. Frames older than about 1.5 s now give 503 (or 409
  when the SDK is busy).
- `/api/spool/prune` no longer overwrites a same-named frame already in the
  archive directory. A colliding name is kept under a numeric tag
  (`ILX00183~1.JPG`).
- `/api/shots` now answers 500 for a directory it cannot read instead of an
  empty `[]` listing, so the run-start baseline is retried rather than treating
  every spooled frame as new.
- `/api/card/mode` transfer reconnect now uses the same camera selection as
  connect (`--match`, `--camera`, ILX-LR1 preference) instead of the first
  listed body.
- `/api/shutter/hold`, `/api/interval/start` and `/api/camera-interval/config`
  now refuse a present but non-numeric field with 400 instead of silently
  substituting the default (a 1 s release hold, an unlimited 1 Hz run).
- `/api/format` no longer reads a non-numeric `quick` (such as `false`) as the
  default. It accepts true/false/1/0 and answers 400 for anything else, so a
  requested full format is not silently run as a quick one.
- A failed `SetSaveInfo` at connect is now retried, reported as
  `saveDirOk:false` in the connect reply and `/api/status`, and re-attempted on
  the next SDK reconnect instead of being logged once and forgotten.
- A timed-out connect whose abandon could not take the SDK no longer leaves
  every later connect, including rigd's backoff retries, answering 'connect in
  progress' for ever. The stale handle is released and the connect retried.
- The camera-interval configure log no longer claims a start delay the body
  refused, and says when the body's step snapped the requested interval.

Drain, ingest, calibration
- The card drain wrote each host file with a single `os.write` whose short
  count went unread, so a volume that filled or hit a quota mid-write blessed a
  truncated file, recorded its full-length hash and deleted the card original.
  The write now loops to the full length, the landed size is checked, and a
  failure keeps the content on the card and removes the `.part`.
- `drain.py` and `ingest.py` exited 0 after a failed drain or a refused,
  conflicting, hash-mismatched or unmatched ingest, so `drain && ingest` read a
  printed failure as success. `drain.py` now exits 1 on any per-shot error and
  `ingest.py` exits 1 on refused/ambiguous, conflicts, drain-hash mismatch, or
  a card that matched nothing (routine unmatched frames still exit 0).
- `ingest --move` unlinked the staged original after a cross-volume copy it
  never compared, so a bad copy could cost the only host copy of a RAW. The
  copy is now hashed against the source before the delete, and a mismatch keeps
  the source.
- Ingest's XMP `pair_spread_ms` keyed shots by `round(epoch)`, so at 2 Hz a
  lost frame paired the neighbouring shot (about 500 ms) and a shot straddling
  an x.5 boundary split in two. Frames now pair with the nearest frame of the
  other camera within half a shot period, blank when none, as the live review
  view does.
- Ingest time-matching tied the true clock offset with the one a shot period
  away when the first card file was missing (or an extra file preceded the run)
  and picked the shifted one by dict order, renaming every RAW one frame off.
  The journalled offset now chooses between near-equal alignments, and with no
  journal such a tie is refused as ambiguous.
- Ingest attributed a lone card series to cam1 whichever body it came from,
  silently renaming a single-body dump as `Cam1_*`. A lone series is now
  accepted only from a per-node directory, a matching journal offset, or
  elimination after the other camera resolved its own series, and is otherwise
  refused as ambiguous.
- Ingest wrote the uncorrected host epoch into the XMP CreateDate,
  DateTimeOriginal and DateCreated while the filename and flight log carried
  the GPS-corrected time. The sidecar dates now apply the run's
  `time.gps_offset_s` (`capture_epoch` stays the host epoch, and the applied
  offset is recorded as `wildsync:gps_offset_s`).
- The IMU calibration stillness gate looked only at the standard deviation of
  gyro and accel norm, so a steady rotation or a slow tilt passed and was
  averaged into the saved reference. It now also refuses a large mean rate and
  a pitch/roll drift between the first and last thirds of the window
  (wrap-aware near +/-180 roll).
- IMU calibration averaged a 0-360 heading arithmetically, so a rig pointing
  near north saved a mean near 180 and a wild relative yaw, and a missing
  heading was read as 0. `heading_mean` is now a circular mean and the relative
  yaw is left unset when either heading is unknown.
- The stereo VSLAM engine froze the saved calibration's R and T but
  triangulated with a guessed focal, so every depth and landmark carried an
  `f_guess/f_true` error while the status said 'measured'. It now takes its
  focal from the saved K1 (rescaled to the stream width) and the status states
  whether the intrinsics came from the calibration or are guessed.
- Saving a stereo calibration that failed its own verdict silently replaced the
  last good one, and stereo VSLAM then froze it as measured. `save()` now
  refuses a failed solve unless forced and keeps the overwritten file as
  `stereo_calibration.json.prev`. The Calib tab now asks for confirmation
  before saving a failed calibration, and `/api/rigcal/stereo/save` takes
  `force:true`.
- Stereo calibration captures were never checked for synchronisation, so a pair
  fired over USB (0-200 ms skew) or over the skew budget was solved and saved
  as if simultaneous. Such pairs are now marked unsynchronised, excluded from
  the guidance counts and the solve, and shown with the reason.

Deploy, documentation
- `deploy.sh provision` no longer dies on an unescaped `${Status}` in its
  unquoted remote heredoc under `set -u`; the node setup script was never sent,
  yet provision reported success.
- `deploy.sh provision` no longer runs `deploy.sh jetson` and `deploy.sh node`
  on the operator's host from backticks in a comment inside its unquoted
  heredoc.
- `deploy.sh all` deploys the Jetson before the nodes, so a first install
  writes the host chrony file before any node is pointed at it.
- `deploy.sh node`, `provision` and `all` stop and exit nonzero when a transfer
  or the remote script fails, instead of printing 'done' or 'provisioned' after
  the failure.
- `deploy/pi-resync/install.sh` no longer renames a named card to the image
  default. It writes the card's `hostname:` and `manage_etc_hosts:` back
  (`preserve_hostname: true` only for a card without one) and declares the
  gpio, i2c and spi groups.
- `deploy/pi-resync/install.sh` reports which of the passwd hash or the ssh key
  it could not find instead of exiting silently under `set -e`.
- `deploy/pi-power.sh` has a real `--strip` mode (the undo its docs described),
  rejects unknown options before touching the node, and does not reboot after a
  failed remote step.
- `sbom.spdx.json` no longer says vendor PDFs are kept in `docs/` and matches
  `THIRD_PARTY_NOTICES.md`: the vendor documentation is cited, not
  redistributed. The v1.1.0 tag keeps its published tree.
- The Mac host install uses a venv at `~/rig/venv` (Homebrew Python refuses a
  global `pip install`, PEP 668), and the launchd agent runs that interpreter.

## [1.1.0] - 2026-10-07

### Added
- `LICENSE`: MIT licence.
- `requirements.txt`: the Python packages imported beyond the standard library.
  Versions are not pinned; the file records which packages are needed.
- `sbom.spdx.json` (SPDX 2.3) and `THIRD_PARTY_NOTICES.md`.
- `docs/INSTALL.md`: installation guide, including which configurations have
  been used in the field and which are supported but untested in the field.
- `rig/buildinfo.py`: every run records the software that produced it
  (version, commit, dirty flag) in `run.json` under `software`. Runs recorded
  before 1.1.0 have no `software` entry. The rig UI shows the software
  description for a run.
- Simulated-run marker: a node that reports itself as simulated (the test fakes)
  causes `run.json` to carry `simulated` and `simulated_nodes`, and the rig UI
  labels the run SIMULATED. Runs without the key are real runs, including every
  run recorded before 1.1.0.
- SHA-256 hashes are recorded at drain time (`<dest>/SHA256SUMS`) and at ingest
  time (`<run>/SHA256SUMS`, with the frame hashes checked against the drain
  hashes). The files are in the format read by `shasum -a 256 -c`;
  `rig/ingest.py --verify <run>` checks them.
- `rig/tests/audit_provenance.py`: tests for the provenance additions above.

### Changed
- `time_source` is now written as `host` when stamps come from the rigd host
  clock (no GPS time). Runs recorded before 1.1.0 wrote `jetson` for the same
  condition whatever the host was; both values are still read.
- Sony Camera Remote SDK headers are no longer tracked. `.gitignore` excludes
  `include/CRSDK/`, and `make sdk` stages both the headers and the shared
  libraries from the user's own copy of the SDK. `README.md`, `Makefile`,
  `deploy/deploy.sh` and `docs/PI-SETUP.md` were updated to describe the
  staging.
- Host and node wording made consistent across the README, `docs/`, deploy
  files, code comments and tests (the rigd host is "the host", the Raspberry Pi
  camera machines are "nodes"; Jetson-specific wording removed where it applied
  to any host). `docs/vslam-jetson.md` renamed to `docs/vslam.md`.

### Removed
- Internal working notes: `docs/HANDOFF.md` and
  `docs/audit-2026-08-16-findings.json`.
- Setup notes, a capture script and vendor documentation for a camera that is
  not part of this system.

## [1.0.0] - 2026-09-21

Tag `v1.0.0`. Changes since the Task 6 report version, from the three commits
of 2026-09-21.

### Added
- Jetson Orin NX host installation (`deploy.sh jetson`; JetPack 6, Python 3.10)
  with a numbered install in the README (Jetson, nodes, strobe, first run).
- XR256 strobe role: `piagent` with `PIAGENT_ROLE=strobe` on the Jetson claims
  only the strobe GPIO line (chip, line and polarity configurable);
  `deploy.sh strobe` installs `piagent-strobe.service` and registers a strobe
  node with rigd. Covered by offline tests (`audit_piagent`). The strobe has not
  been fired with the camera array.
- Per-camera Olive IMUs: `piagent` can serve an Olive IMU on `/imu`
  (`PIAGENT_IMU=olive:...`); each camera's flight-log attitude comes from its
  own node's IMU, falling back to the fleet IMU node; `rigcal` resolves each
  camera's IMU endpoint from `/health`.
- Field manual (`docs/USER-GUIDE.html`) chapter 05, The strobe (XR256).
- `deploy.sh node`: Sony USB udev rule, Olive network link, chrony preferring
  the Jetson when deployed from it.

### Changed
- Field manual rewritten for the Jetson host, Subsonus via iKonvert (NMEA
  2000) and Olive IMUs, ESP32 node controllers removed; revised 2026-09-21,
  later chapters renumbered 06-12, "No strobe yet" limit removed.
  `docs/FIELD-RUN.md`, `docs/PI-SETUP.md`, `docs/PROTOCOL.md` and the XR256
  brief updated to match.
- Wording in the `audit_*.py` docstrings neutralised; `AUDIT_LANES` renamed
  `AUDIT_SUITES` in `soaktest` (no behaviour change).

## Task 6 report version - tag `task6-report-2026-08-31`

Commit dated 2026-08-28. This is the software as it stood at the Task 6 report
of 31 August 2026. From the commits between the macOS port and the tag:

- Host: `rigd` ported to macOS and the Jetson retired as host (2026-08-23); a
  Raspberry Pi per camera running `ilxctl` and `piagent`.
- Card drain (RAW pulled off the card, verified, then deleted), card ingest line
  and stereo-pairing verifier (2026-08-23).
- Audit passes: defect fixes and regression suites (2026-08-23, 2026-08-24;
  the build the 24 August field manual labels "audit-fixes").
- Field manual `docs/USER-GUIDE.html` and `rig/tests/live_validate.py`
  (2026-08-24).
- `ilxctl` format, power and property-dump endpoints (2026-08-26).
- Hardening, project layer, Diag/Alerts/Map tabs, VSLAM and Olive IMU
  (2026-08-27), including the Olive IMU live WebSocket bridge and rig
  calibration.
- Optional third node (`cam3`), `ilxctl` unit and two-predicate fleet
  (2026-08-28).

## Repository history

The repository history was rewritten on 2026-09-21 and again on 2026-10-05
(the second time to remove the Sony Camera Remote SDK headers from every
commit). Commit IDs quoted before 2026-10-05 no longer resolve. Known
mappings: `fe114e8` -> `43c91d6` and `f72b595` -> `4b57fc0`. Refer to the tags
(`task6-report-2026-08-31`, `v1.0.0`), not commit IDs.

On 2026-10-07 the history was rewritten a third time, when the repository was
re-created as a public repository: internal notes and the Sony SDK headers
were removed from every commit, so every tagged version also checks out on
Windows. No vendor PDF was ever committed (`THIRD_PARTY_NOTICES.md` says where
to get them). Commits before `9bf3ab7`, including the tags
`task6-report-2026-08-31` and `v1.0.0`, still carry the third-camera (cam3)
commissioning notes in `docs/PI-SETUP.md` and related files; `v1.1.0` and later
do not, and describe only the Sony two-camera rig and a generic third node.
Mappings:
`task6-report-2026-08-31` `43c91d6` -> `75113f6`, `v1.0.0` `4b57fc0` ->
`10dcc29`, `v1.1.0` `be8bfce` -> `9d89976`.

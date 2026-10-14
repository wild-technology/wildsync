# Changelog

All notable changes to Wild Sync are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Dates are commit
dates.

## [Unreleased]

### Added
- `rig/nav_subsonus.py`: the Subsonus as the navigation source over Ethernet
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
  and accepts `!ANPP,<id>,<hex>` lines (replay of a Subsonus run).
- `rigd` reads the nav source from `WILDSYNC_NAV` / `~/rig/nav.json`; the
  `nav_gateway_down` anomaly names the source in use.

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
re-created as a public repository: internal notes, the third-camera material
and the vendor PDFs (which are not redistributable; `THIRD_PARTY_NOTICES.md`
now says where to get them) were removed from every commit, so every
tagged version also checks out on Windows. Mappings:
`task6-report-2026-08-31` `43c91d6` -> `75113f6`, `v1.0.0` `4b57fc0` ->
`10dcc29`, `v1.1.0` `be8bfce` -> `9d89976`.

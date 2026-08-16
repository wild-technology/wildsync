# Future tests — things to verify before trusting them

Open questions that need hardware time, not code. Each one names the decision it
unblocks, so it is obvious what is being bought.

---

## 1. Lens encoder parity between cam1 and cam2

**Unblocks:** whether focus and zoom position may become fleet-applied settings.

Right now they must NOT be. Focus and zoom are controlled **per camera**, and
`focus_position` / `zoom_position` are deliberately kept OUT of the desired-state vector
that `SettingsManager` converges across the fleet. Pushing one body's encoder count to
the other assumes the two lenses report the same number for the same optical state, and
that has never been measured.

The rig is always manual focus (see the operating notes), so focus position is a survey
parameter — but only if the number means the same thing on both bodies.

**Test:**
1. Set both cameras to the same subject distance by eye, at the same focal length.
2. Read `focusPosCur` and `zoomPosCur` from each body's `/api/status`. Record both.
3. Repeat at several distances across the working range (near, mid, infinity).
4. Tabulate cam1 vs cam2 encoder counts at each. Are they equal? Linearly related?
   Unrelated?
5. Power-cycle both bodies. Re-read `focusPosCur` / `zoomPosCur` **without touching the
   lenses**. Does the encoder survive the reboot, or does it re-home to an arbitrary
   value?

**Decision rule:** only if the counts agree between bodies AND survive a power cycle may
focus/zoom be promoted into the converged desired vector. If they agree but do NOT
survive a reboot, they can be converged only after an explicit re-home step. If they do
not agree between bodies, they stay per-camera forever and the UI must never offer an
"apply focus to fleet" control.

**Why it matters:** a silent focus mismatch between the two bodies changes the interior
orientation of the stereo pair, which invalidates the photogrammetric solution in a way
that is very hard to see in the frames themselves.

---

## 2. ImageSize L/M/S actual pixel dimensions on the ILX-LR1

**Unblocks:** choosing the archive resolution (target ~12 MP).

Measured 2026-08-16: with `transsize=1` (Small) the body delivers **1616x1080 = 1.7 MP**
to the host — a review thumbnail, not survey data. `transsize=0` (Original) is required
for the delivered JPEG to be full size, after which `imagesize` (1=L, 2=M, 3=S) sets the
resolution.

The L/M/S dimensions on this body have **not** been measured — the attempt was cut short
when both bodies faulted. Sony's 61 MP full-frame family is nominally L 60 MP / M 26 MP /
S 15 MP, and ~12 MP may only be reachable in a crop mode, if at all.

**Test:** for each `imagesize` in 1/2/3 with `transsize=0`, fire one frame and read the
actual pixel dimensions of the delivered JPEG. Record the file size too — it sets the
transfer budget per frame, which is the real constraint on sustained frame rate.

**Note:** `ilxctl` currently exposes NO readback for `imagesize`, `filetype`, `transsize`
or `quality` — they are absent from `/api/status`. Until that is added, nothing can
verify what the cameras are actually set to record, and the convergence engine is
pushing these fields blind.

---

## 3. Sustained frame rate against the real archive settings

**Unblocks:** the survey's usable frame rate, and whether the strobe or the USB path is
the binding constraint.

All the sustained-rate figures on record (~0.57 s/frame, occasional drops) were taken
with `transsize=Small`, i.e. 320 KB thumbnails. At `transsize=Original` the per-frame
payload is one to two orders of magnitude larger and the delivery path is a completely
different problem.

**Test:** re-measure sustained capture at the real archive settings — full-size JPEG to
the host, RAW to the card — and find the interval at which frames stop being dropped.
Compare against the Bolt VB-22's recycle time at the working power setting, which is the
other ceiling. Whichever is slower sets the survey rate.

---

## 4. Strobe sync port characterisation

**Unblocks:** confidence in the direct-GPIO trigger, and re-checking it if the flash is
ever swapped.

Measured: 2.9 V DC open circuit, centre to shell. Short-circuit current NOT measured (no
suitable meter). It can be inferred with a voltmeter alone — see the strobe wiring notes.

**Test:** confirm one flash pulse per shot rather than a latched tube; confirm the
acceptance check (strobe instant inside every node's shutter-open window) passes; measure
recycle time at the working power setting and flashes per charge for survey planning.

---

## 5. What actually stalled cam1's card

**Unblocks:** knowing whether sustained RAW-to-card is safe at survey rate.

On 2026-08-16 cam1 stopped recording entirely: one frame stuck in the camera's write
buffer, SD LED solid red, a transfer icon showing "1", and format refused with
*"writing to memory card. unable to operate."* The body went busy, which locked its whole
property table (`storeDest` DisplayOnly, `storeChoices` and `driveChoices` empty), stopped
PC delivery, and eventually wedged `ilxctl`. cam2, on the same fleet and the same firing
sequence, was unaffected throughout.

> **Note on an earlier misdiagnosis.** This was first attributed to a body "caution" and
> then to thermal load. Both were wrong. The caution flag was a false positive in
> `ilxctl` (a 1-based enum tested as a bitmask, since fixed), and no browser was ever
> connected so live view was not running. Do not repeat either theory without evidence.

**Test:** identify what the two bodies' cards are — make, capacity, speed class, age —
and whether the ILX-LR1 slot is UHS-I or UHS-II. A 61 MP RAW is ~60-125 MB, so 1 fps is
60-125 MB/s *sustained* and 2 fps is double; UHS-I cannot do that. Then find the sustained
rate and duration that provokes a stall, on a known-good card, with the flight recorder
watching the new `slotWriting` property.

**Decision rule:** if a healthy card of known speed still stalls at the survey rate, RAW
must come off the card path entirely (JPEG-only to host, RAW disabled) rather than being
allowed to take a body down mid-transect. Also worth checking whether
`DeviceOverheatingState` moves at all during a long run, now that it is readable.

---

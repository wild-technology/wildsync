#!/usr/bin/env python3
"""Wild Sync ingest — give the RAW archive the rig's own timestamps.

The rig times every frame by its hardware EXPOSURE edge on a disciplined Pi
(sub-ms), and the delivered review JPEG is renamed to that instant:
Cam1_20260820_190952.64.jpg. The RAW stays on the camera's card under the
body's own name and the body's own clock — which on the ILX-LR1 cannot be set
over USB, sits days wrong and drifts ~0.3-0.9 s/day. This tool closes that gap
after a card download (or a remote pull): it matches every card file to its
flight-log row, renames the RAW (and the card JPEG) to the rig's scheme beside
the review JPEG, and writes an XMP sidecar carrying the true capture instant,
the position and the attitude — so photogrammetry software reads the survey's
real time and place straight off the archive.

    python3 rig/ingest.py ~/rig-raw/cam1          # what the drain staged
    python3 rig/ingest.py /Users/wild/Pictures/2026/2026-08-18
    python3 rig/ingest.py <card_dir> --run 260820_1925_transect-01 --move
    python3 rig/ingest.py <card_dir> --dry-run
    python3 rig/ingest.py --verify 260820_1925_transect-01   # check SHA256SUMS
    python3 rig/ingest.py --write-sums 260820_1925_transect-01
    python3 rig/ingest.py --selftest

Card files are timed from EXIF — the JPEG's when there is one, otherwise the
ARW's own TIFF EXIF, so a RAW-only staging tree still matches.

Matching, per run and camera, in order:
  1. by the body's file number when the card series shares the PC-save
     counter (cam2: ILX09187 <-> DSC09187), confirmed by a tight clock offset;
  2. else by time: the offset between the body's EXIF clock and the rig's
     epochs is found as the mode of all pairwise differences, then each row is
     matched to the nearest card file within TOL. No prior knowledge of the
     body's clock error is needed — it is measured from the data.
WHICH body a card series belongs to is a separate question: both bodies fire
the same instants, so a time match alone cannot tell them apart. A per-node
directory (~/rig-raw/cam1 — what the drain writes) settles it; failing that
rigd's journalled exif_offset does; with neither, ingest REFUSES that camera
rather than risk swapping the two bodies' RAWs.

Frames come from <run>/index.jsonl when it exists (the complete list) and from
run.json's last-2000 index only as a fallback. Originals are hardlinked into
the run (no extra space on the same volume) or copied; --move removes the
card-dir copy afterwards, and only ever after the destination is confirmed
byte-identical. Nothing is written unless every step of a run's match is
consistent; the manifest says what happened.

Hashes. ingest_manifest.csv carries the SHA-256 of each RAW and card JPEG as
placed in the run (raw_sha256, jpg_sha256, appended after the original eight
columns), and <run>/SHA256SUMS lists every frame in the run and, once run.json
says the run is final, the run's own records - in the format `shasum -a 256
-c` reads. A file already listed keeps the hash first recorded for it, so a
re-ingest can never re-bless a file that changed since; --verify re-hashes
everything listed. When the card files came from a drain, each placed file is
also checked against the hash the drain verified (<dest>/SHA256SUMS).
"""
import argparse
import csv
import glob
import json
import os
import re
import shutil
import statistics
import struct
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run import _exif_capture_epoch, FLIGHT_HEADER  # noqa: E402
import rigcore                                         # noqa: E402
# One hasher and one SHA256SUMS format for the drain and ingest alike.
from drain import (SUMS_NAME, file_sha256, read_sums,  # noqa: E402
                   verify_sums, write_sums)

TOL_S = 0.75          # max |card exif - rig epoch| after offset removal
BIN_S = 0.25          # histogram bin for offset discovery
MIN_FRAC = 0.7        # fraction of rows a candidate offset must explain
RUNS_DIR = os.path.expanduser("~/rig-runs")

# ingest_manifest.csv, in column order. The first eight are the original
# columns, unchanged; the two hashes were appended in v1.1.0, so a reader keyed
# on position or on name keeps working, and an older manifest simply lacks them
# (its rows read back with the hash columns empty).
MANIFEST_COLUMNS = ["run_id", "cam", "card", "new_base", "epoch",
                    "residual_ms", "raw", "jpg", "raw_sha256", "jpg_sha256"]

# What <run>/SHA256SUMS covers besides the frames, once run.json is final:
# the run's own records, at the top of the run and in each camera directory.
RUN_RECORD_FILES = ("run.json", "index.jsonl", "events.log", "nmea_raw.log",
                    "unpulled.jsonl", "ingest_manifest.csv")
CAM_RECORD_FILES = ("flight_log.csv",)


# ---------------------------------------------------------------------------
# card inventory
# ---------------------------------------------------------------------------
def _number(name):
    m = re.search(r"(\d{3,})", os.path.splitext(os.path.basename(name))[0])
    return int(m.group(1)) if m else None


def _series(name):
    stem = os.path.splitext(os.path.basename(name))[0]
    m = re.match(r"([A-Za-z_]+)\d", stem)
    return m.group(1) if m else "?"


def _arw_exif_epoch(path):
    """DateTimeOriginal(+SubSecTimeOriginal) from a Sony ARW, via a tiny
    bounded IFD walk: an ARW is a TIFF (b"II*" + NUL, or big-endian
    b"MM" + NUL + b"*"), so IFD0 -> ExifIFD (0x8769) -> 0x9003 / 0x9291.

    The drain stages RAW-only trees (transsize Small keeps the full JPEG on
    the card until it too is pulled), so ingest MUST be able to time a bare
    .ARW: reading EXIF only from JPEGs is why every post-drain ingest matched
    nothing (169 ARW, 0 XMP in the field — audit 2026-08-23, critical). Hand
    written rather than via PIL/exifread because the Pis and the Mac carry
    stdlib only, and because a torn download must cost nothing: every read is
    bounded and anything malformed returns None. Same local-time decode as
    run._exif_capture_epoch, so the two agree on a body's clock offset."""
    try:
        with open(path, "rb") as fh:
            head = fh.read(8)
            if len(head) < 8 or head[:4] not in (b"II*\x00", b"MM\x00*"):
                return None
            en = "<" if head[:2] == b"II" else ">"
            (off0,) = struct.unpack(en + "I", head[4:8])

            def ifd(off):
                if not 0 < off < 64 * 1024 * 1024:      # header region only
                    return {}
                fh.seek(off)
                b2 = fh.read(2)
                if len(b2) < 2:
                    return {}
                (n,) = struct.unpack(en + "H", b2)
                raw = fh.read(12 * min(n, 512))          # bounded read
                ents = {}
                for i in range(len(raw) // 12):
                    tag, typ, cnt = struct.unpack(en + "HHI", raw[i*12:i*12+8])
                    ents[tag] = (typ, cnt, raw[i*12+8:i*12+12])
                return ents

            def ascii_of(ent):
                if ent is None:
                    return None
                typ, cnt, val = ent
                if typ != 2 or cnt > 64:                 # ASCII, bounded
                    return None
                if cnt <= 4:
                    raw = val[:cnt]
                else:
                    (o,) = struct.unpack(en + "I", val)
                    fh.seek(o)
                    raw = fh.read(cnt)
                return raw.split(b"\x00")[0].decode("ascii", "replace").strip()

            def long_of(ent):
                if ent is None:
                    return None
                typ, cnt, val = ent
                if typ == 4 and cnt == 1:
                    return struct.unpack(en + "I", val)[0]
                if typ == 3 and cnt == 1:                # SHORT, left-justified
                    return struct.unpack(en + "H", val[:2])[0]
                return None

            exoff = long_of(ifd(off0).get(0x8769))
            if not exoff:
                return None
            ex = ifd(exoff)
            dt = ascii_of(ex.get(0x9003))
            if not dt:
                return None
            ss = ascii_of(ex.get(0x9291)) or "0"
            base = time.mktime(time.strptime(dt, "%Y:%m:%d %H:%M:%S"))
            return base + (float("0." + ss) if ss.isdigit() else 0.0)
    except Exception:  # noqa: BLE001 - a torn TIFF must not kill an ingest
        return None


def scan_card(card_dir):
    """Every still on the card: {stem, jpg, arw, exif, series, number}."""
    out = {}
    for p in sorted(glob.glob(os.path.join(card_dir, "*"))):
        ext = os.path.splitext(p)[1].lower()
        if ext not in (".jpg", ".jpeg", ".arw"):
            continue
        stem = os.path.splitext(os.path.basename(p))[0]
        e = out.setdefault(stem, {"stem": stem, "jpg": None, "arw": None,
                                  "exif": None, "series": _series(p),
                                  "number": _number(p)})
        if ext == ".arw":
            e["arw"] = p
        else:
            e["jpg"] = p
            try:
                with open(p, "rb") as fh:
                    e["exif"] = _exif_capture_epoch(fh.read())
            except OSError:
                pass
    # RAW-only staging must still be timeable: read the ARW's own EXIF. The
    # JPEG's is preferred when both exist (same shot, same stamps), but a
    # disagreement means the stem paired a foreign file — flag it, never
    # average it.
    for e in out.values():
        if e["arw"]:
            ae = _arw_exif_epoch(e["arw"])
            if e["exif"] is None:
                e["exif"] = ae
            elif ae is not None and abs(ae - e["exif"]) > 1.5:
                e["exif_mismatch"] = ae
    return list(out.values())


def _dir_camera(card_dir):
    """cam number when the directory IS a per-node staging dir: the drain
    writes ~/rig-raw/<node>/, and that path component is the one authoritative
    statement of which body the files came from — the two bodies produce
    colliding names and fire the same instants, so nothing inside the files
    can say (audit 2026-08-23, medium)."""
    m = re.fullmatch(r"cam(\d+)", os.path.basename(os.path.normpath(card_dir)))
    return int(m.group(1)) if m else None


# ---------------------------------------------------------------------------
# runs
# ---------------------------------------------------------------------------
def read_frame_index(root, doc=None):
    """The run's FULL per-frame index (contract C3). run.json keeps only the
    last 2000 entries — a long transect's head silently dropped out of every
    match and sidecar (audit 2026-08-23, high) — while run.py appends every
    frame to <run>/index.jsonl. Prefer that file, tolerating a torn last line
    from a crash mid-append; fall back to run.json's tail."""
    rows = []
    try:
        with open(os.path.join(root, "index.jsonl")) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    e = json.loads(line)
                except ValueError:
                    continue        # torn tail: the entry was mid-write
                if isinstance(e, dict) and "cam" in e and "epoch" in e:
                    rows.append(e)
    except OSError:
        rows = []
    if rows:
        return rows
    if doc is None:
        try:
            with open(os.path.join(root, "run.json")) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            return []
    return doc.get("index", [])


def load_run(root):
    with open(os.path.join(root, "run.json")) as fh:
        doc = json.load(fh)
    rows = {}
    for e in read_frame_index(root, doc):
        rows.setdefault(e["cam"], []).append(e)
    flight = {}
    for cam in rows:
        fl = os.path.join(root, "cam%d" % cam, "flight_log.csv")
        if os.path.exists(fl):
            with open(fl, newline="") as fh:
                for r in csv.DictReader(fh):
                    flight[r["filename"]] = r
    return doc, rows, flight


# ---------------------------------------------------------------------------
# rigd's own measurement of each body's clock error, per run start
# ---------------------------------------------------------------------------
JOURNAL = os.path.expanduser("~/rig/rigd.jsonl")


def load_journal_offsets(path=JOURNAL):
    """[(ts, node, offset_s)] from rigd's exif_offset events. rigd measures
    camera_epoch - true_epoch for every node at every run start; with a
    combined card directory it is the only thing that can say WHICH body a
    series came from.

    The rotated file is read too: rigcore.EventLog caps the journal at 16 MB
    and renames it to <path>.1, so a busy week could push a morning's
    exif_offset out of the current file — and losing it is what drops ingest
    into the ambiguous refusal (audit 2026-08-23, medium). Oldest first."""
    out = []
    for p in (path + ".1", path):
        try:
            with open(p) as fh:
                for line in fh:
                    try:
                        e = json.loads(line)
                    except ValueError:
                        continue
                    if e.get("kind") != "exif_offset" or not e.get("node"):
                        continue
                    m = re.search(r"offset (-?[\d.]+)s", e.get("msg", ""))
                    if m:
                        out.append((e["ts"], e["node"], float(m.group(1))))
        except OSError:
            continue
    return out


def expected_offset(offsets, node, at):
    """The journaled offset for `node` nearest to epoch `at` (within a day)."""
    best = None
    for ts, n, off in offsets:
        if n == node and abs(ts - at) < 86400:
            if best is None or abs(ts - at) < abs(best[0] - at):
                best = (ts, off)
    return best[1] if best else None


# ---------------------------------------------------------------------------
# matching
# ---------------------------------------------------------------------------
def _offset_by_number(rows, cards):
    """(series, offset) when the card numbering equals the PC-save counter."""
    by_num = {}
    for c in cards:
        if c["number"] is not None and c["exif"] is not None:
            by_num.setdefault((c["series"], c["number"]), []).append(c)
    best = None
    series = {c["series"] for c in cards}
    for s in series:
        diffs = []
        for r in rows:
            n = _number(r["orig"])
            cs = by_num.get((s, n))
            if cs and len(cs) == 1:
                diffs.append(cs[0]["exif"] - r["epoch"])
        if len(diffs) >= max(3, int(MIN_FRAC * len(rows))):
            sd = statistics.pstdev(diffs) if len(diffs) > 1 else 0.0
            if sd < 0.3:
                cand = (len(diffs), s, statistics.median(diffs))
                if best is None or cand > best:
                    best = cand
    return (best[1], best[2]) if best else (None, None)


def _offset_by_time(rows, cards, expect=None, authoritative=False):
    """The clock offset as the mode of all pairwise (exif - epoch) differences
    over every card series, then the series that carries it. Two bodies shoot
    the same instants, so every series aligns with every camera's rows at
    SOME offset: when rigd's journal says what this body's offset should be,
    the series whose offset agrees wins; with `authoritative` (a per-node
    staging dir — every file in it is this body's) the strongest mode does.
    With NEITHER, more than one qualifying series is a coin flip that would
    silently swap cam1's and cam2's RAWs (max() over tied counts returned the
    first-inserted key, i.e. glob order — audit 2026-08-23, medium): refuse.
    Returns (series, offset, why) where why is None or "ambiguous: ..."."""
    hist = {}
    for r in rows:
        for c in cards:
            if c["exif"] is None:
                continue
            d = c["exif"] - r["epoch"]
            if abs(d) > 30 * 86400:
                continue
            k = (c["series"], round(d / BIN_S))
            hist[k] = hist.get(k, 0) + 1
    if not hist:
        return None, None, None
    need = max(2, int(MIN_FRAC * len(rows)))
    cands = [(k, n) for k, n in hist.items() if n >= need]
    if not cands:
        return None, None, None
    if expect is not None:
        near = [(k, n) for k, n in cands if abs(k[1] * BIN_S - expect) <= 2.0]
        if near:
            cands = near
    series_set = sorted({k[0] for k, n in cands})
    if len(series_set) > 1 and not authoritative:
        return None, None, "ambiguous: " + "/".join(series_set)
    (series, kbin), n = max(cands, key=lambda kv: kv[1])
    # refine: median of diffs inside the winning bin (+-1 bin)
    diffs = [c["exif"] - r["epoch"] for r in rows for c in cards
             if c["series"] == series and c["exif"] is not None
             and abs(c["exif"] - r["epoch"] - kbin * BIN_S) <= 1.5 * BIN_S]
    return series, statistics.median(diffs), None


def match_run(rows, cards, used, expect=None, number_only=False,
              tol_s=None,
              authoritative=False):
    """rows: index entries for one camera. Returns (series, offset, how,
    matches, unmatched) where matches = [(row, card, residual_s)]; how is
    "ambiguous: <series list>" when attribution was refused (G5)."""
    avail = [c for c in cards if c["stem"] not in used and c["exif"] is not None]
    series, off = _offset_by_number(rows, avail)
    how = "number"
    if series is None:
        if number_only:
            return None, None, how, [], list(rows)
        series, off, why = _offset_by_time(rows, avail, expect, authoritative)
        how = why or "time"
    if series is None:
        return None, None, how, [], list(rows)
    pool = [c for c in avail if c["series"] == series]
    matches, unmatched = [], []
    for r in sorted(rows, key=lambda r: r["epoch"]):
        best = None
        for c in pool:
            if c["stem"] in used:
                continue
            res = c["exif"] - off - r["epoch"]
            # The global TOL_S (0.75 s) is WIDER than a 2 Hz survey's shot
            # period (0.5 s): with one hole in the staged files, row k then
            # claimed file k+1 at residual +period, and every later RAW
            # cascaded onto the previous frame's identity - wrong names, wrong
            # XMP instants, silently (audit 2026-08-27). The caller passes a
            # tolerance derived from the run's own interval; a hole now yields
            # an honest unmatched row instead of a shifted survey.
            lim = TOL_S if tol_s is None else tol_s
            if abs(res) <= lim and (best is None or abs(res) < abs(best[1])):
                best = (c, res)
        if best:
            used.add(best[0]["stem"])
            matches.append((r, best[0], best[1]))
        else:
            unmatched.append(r)
    return series, off, how, matches, unmatched


# ---------------------------------------------------------------------------
# output: rename/link + XMP sidecar + manifest
# ---------------------------------------------------------------------------
def _dms(v, pos, neg):
    """EXIF/XMP GPS 'DDD,MM.mmmmH' form from decimal degrees."""
    h = pos if v >= 0 else neg
    v = abs(v)
    d = int(v)
    return "%d,%.6f%s" % (d, (v - d) * 60.0, h)


def _iso(epoch):
    # Round ONCE at millisecond resolution and derive both fields from the
    # result: rounding the fraction on its own emitted ".1000Z" (invalid ISO,
    # and a stamp a full second early) for any epoch with fraction >= .9995 —
    # the same carry bug run._split_epoch documents (audit 2026-08-23, low).
    ms = int(round(epoch * 1000))
    whole, frac = divmod(ms, 1000)
    return (time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(whole))
            + ".%03dZ" % frac)


def xmp_sidecar(row, fl, run_id, cam, residual_s, pair_spread_ms):
    """An XMP packet carrying the rig's truth for this frame."""
    lat = fl.get("lat") if fl else None
    lon = fl.get("long") if fl else None
    gps = ""
    if lat not in (None, "") and lon not in (None, ""):
        gps = ("   <exif:GPSLatitude>%s</exif:GPSLatitude>\n"
               "   <exif:GPSLongitude>%s</exif:GPSLongitude>\n"
               "   <exif:GPSVersionID>2.3.0.0</exif:GPSVersionID>\n"
               % (_dms(float(lat), "N", "S"), _dms(float(lon), "E", "W")))

    def f(k):
        v = (fl or {}).get(k)
        return "" if v in (None, "") else v
    ws = "\n".join("   <wildsync:%s>%s</wildsync:%s>" % (k, v, k) for k, v in [
        ("run_id", run_id), ("camera", "cam%d" % cam),
        ("capture_epoch", "%.6f" % row["epoch"]),
        ("capture_source", row.get("src", "")),
        ("card_name", row.get("card_stem", "")),
        ("pc_save_name", row.get("orig", "")),
        ("exif_residual_ms", "%.1f" % (residual_s * 1000)),
        ("pair_spread_ms", "" if pair_spread_ms is None else "%.2f" % pair_spread_ms),
        ("xutm", f("xutm")), ("yutm", f("yutm")), ("utm_zone", f("utm_zone")),
        ("depth_m", f("depth_from_xplore9")),
        ("pitch_deg", f("pitch")), ("roll_deg", f("roll")),
        ("heading_imu_deg", f("heading_imu")),
        ("time_source", f("time_source")),
    ] if v != "")
    iso = _iso(row["epoch"])
    return ("""<?xpacket begin="﻿" id="W5M0MpCehiHzreSzNTczkc9d"?>
<x:xmpmeta xmlns:x="adobe:ns:meta/" x:xmptk="wildsync ingest">
 <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
  <rdf:Description rdf:about=""
    xmlns:xmp="http://ns.adobe.com/xap/1.0/"
    xmlns:exif="http://ns.adobe.com/exif/1.0/"
    xmlns:photoshop="http://ns.adobe.com/photoshop/1.0/"
    xmlns:wildsync="https://wildtechnology.org/ns/wildsync/1.0/">
   <xmp:CreateDate>%s</xmp:CreateDate>
   <exif:DateTimeOriginal>%s</exif:DateTimeOriginal>
   <photoshop:DateCreated>%s</photoshop:DateCreated>
%s%s
  </rdf:Description>
 </rdf:RDF>
</x:xmpmeta>
<?xpacket end="w"?>
""" % (iso, iso, iso, gps, ws))


def place(src, dst, move, dry):
    """Hardlink (same volume) or copy src to dst; --move removes src.

    An existing dst is "the same file" only when its CONTENT matches: with
    rawtype Uncompressed every frame has the same byte size, so the old
    size-only check let --move delete the only correct copy of a RAW while a
    wrong same-size image kept the run name (audit 2026-08-23, low). Same
    inode or same sha256 -> already placed (and only then may --move drop the
    source). Different content under the same run name is a genuine conflict —
    a re-timed run, or a card file placed by an earlier, wrong match — and
    neither copy has been verified against the other, so nothing is deleted or
    overwritten: the new file lands beside it as <base>.conflict-<sha8><ext>,
    the source is kept even under --move, and the manifest says "conflict"."""
    if dry:
        return "would %s" % ("move" if move else "link")
    if os.path.exists(dst):
        try:
            same = os.path.samefile(src, dst)
        except OSError:
            same = False
        if not same:
            same = (os.path.getsize(dst) == os.path.getsize(src)
                    and file_sha256(dst) == file_sha256(src))
        if same:
            if move and os.path.exists(src):
                os.unlink(src)
            return "exists"
        stem, ext = os.path.splitext(dst)
        alt = "%s.conflict-%s%s" % (stem, file_sha256(src)[:8], ext)
        if not os.path.exists(alt):
            tmp = alt + ".part"
            shutil.copy2(src, tmp)
            with open(tmp, "rb+") as fh:
                os.fsync(fh.fileno())
            os.replace(tmp, alt)
        return "conflict:" + os.path.basename(alt)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    try:
        os.link(src, dst)          # atomic by nature (same volume)
        how = "linked"
    except OSError:
        # Cross-volume: copy to a .part name, fsync, then rename. A crash or
        # ENOSPC mid-copy used to leave a truncated file under the final run
        # name - indistinguishable from the RAW it pretends to be, and the
        # re-run then filed the good copy as a "conflict" beside it instead
        # of healing (audit 2026-08-27).
        tmp = dst + ".part"
        try:
            shutil.copy2(src, tmp)
            with open(tmp, "rb+") as fh:
                os.fsync(fh.fileno())
            os.replace(tmp, dst)
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        how = "copied"
    if move:
        os.unlink(src)
        how = "moved"
    return how


# ---------------------------------------------------------------------------
# hashes: manifest columns and <run>/SHA256SUMS
# ---------------------------------------------------------------------------
_FRESH_PLACEMENTS = ("linked", "copied", "moved")


def _placed_path(how, dst):
    """Where place() left the card file's bytes, or None (dry run)."""
    how = str(how)
    if how.startswith("conflict:"):
        return os.path.join(os.path.dirname(dst), how.split(":", 1)[1])
    if how in _FRESH_PLACEMENTS or how == "exists":
        return dst
    return None


class RunHasher:
    """SHA-256 of files in one run, each hashed at most once per pass.

    A file <run>/SHA256SUMS already lists keeps the hash recorded for it
    unless the caller says its bytes were just written: re-hashing on every
    ingest would quietly re-record a frame that had changed on disk since,
    and the record exists to catch exactly that."""

    def __init__(self, root):
        self.root = root
        self.recorded = read_sums(os.path.join(root, SUMS_NAME))
        self.known = {}
        self.hashed = 0

    def rel(self, path):
        return os.path.relpath(path, self.root).replace(os.sep, "/")

    def sha(self, path, fresh=False):
        rel = self.rel(path)
        if rel in self.known and not fresh:
            return self.known[rel]
        if not fresh and rel in self.recorded:
            v = self.recorded[rel]
        else:
            v = file_sha256(path)
            self.hashed += 1
        self.known[rel] = v
        return v


def run_sum_files(root, doc=None):
    """The relative paths <run>/SHA256SUMS covers.

    Every frame (rigcore.FRAME_EXTS: review JPEG, RAW, card JPEG, a parked
    .conflict- file) in each camera directory - a subdirectory holding a
    flight_log.csv, the run browser's own definition - and, once run.json says
    final, the run's records (RUN_RECORD_FILES, CAM_RECORD_FILES). Not the
    .xmp sidecars: they are derived from those records, ingest rewrites its
    own, and photo editors write theirs. Symlinks are never followed."""
    if doc is None:
        try:
            with open(os.path.join(root, "run.json")) as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            doc = {}
    final = isinstance(doc, dict) and doc.get("final") is True
    out = []
    try:
        names = sorted(os.listdir(root))
    except OSError:
        return out, final
    for d in names:
        p = os.path.join(root, d)
        if os.path.islink(p):
            continue
        if os.path.isdir(p):
            if not os.path.isfile(os.path.join(p, "flight_log.csv")):
                continue
            for fn in sorted(os.listdir(p)):
                fp = os.path.join(p, fn)
                if os.path.islink(fp) or not os.path.isfile(fp):
                    continue
                if fn.lower().endswith(rigcore.FRAME_EXTS) \
                        or (final and fn in CAM_RECORD_FILES):
                    out.append(d + "/" + fn)
        elif final and d in RUN_RECORD_FILES and os.path.isfile(p):
            out.append(d)
    return out, final


def write_run_sums(root, doc=None, hasher=None, log=print):
    """(Re)write <run>/SHA256SUMS. Returns {files, hashed, final, missing,
    skipped}. Entries for files that are gone are KEPT, so --verify reports
    them missing instead of the record quietly forgetting them."""
    hasher = hasher or RunHasher(root)
    files, final = run_sum_files(root, doc)
    entries = {}
    for rel in files:
        # ingest_manifest.csv is rewritten by every ingest; nothing else
        # listed here is ever rewritten in place.
        entries[rel] = hasher.sha(os.path.join(root, *rel.split("/")),
                                  fresh=(rel == "ingest_manifest.csv"))
    missing = sorted(r for r in hasher.recorded if r not in entries
                     and not os.path.isfile(os.path.join(root, *r.split("/"))))
    for rel in hasher.recorded:
        entries.setdefault(rel, hasher.recorded[rel])
    skipped = write_sums(os.path.join(root, SUMS_NAME), entries)
    rid = os.path.basename(os.path.normpath(root))
    log("  %s: %s lists %d files (%d hashed now)%s%s%s"
        % (rid, SUMS_NAME, len(entries) - len(skipped), hasher.hashed,
           "" if final else "; run.json is not final, so only frames",
           "; WARNING %d listed file(s) are no longer present: %s"
           % (len(missing), ", ".join(missing[:5])) if missing else "",
           "; %d name(s) not representable, left out" % len(skipped)
           if skipped else ""))
    return {"files": len(entries) - len(skipped), "hashed": hasher.hashed,
            "final": final, "missing": missing, "skipped": skipped}


def resolve_run(arg, runs_dir=RUNS_DIR):
    """A run given as a directory path or as a run id under runs_dir."""
    if os.path.isdir(arg):
        return os.path.abspath(arg)
    return os.path.join(runs_dir, arg)


def verify_run(path, log=print):
    """Re-hash everything <path>/SHA256SUMS lists. Returns the verify_sums()
    report plus "unlisted": files a fresh SHA256SUMS would cover that this
    one does not (written after it, or a run that was never ingested)."""
    rep = verify_sums(path, log=log)
    if rep["sums"] is not None and os.path.isfile(os.path.join(path,
                                                               "run.json")):
        files, _ = run_sum_files(path)
        listed = read_sums(rep["sums"])
        rep["unlisted"] = [r for r in files if r not in listed]
    else:
        rep["unlisted"] = []
    return rep


def ingest(card_dir, runs_dir=RUNS_DIR, only=None, move=False, dry=False,
           log=print, offsets=None):
    """Returns {"runs": [...], "leftover": [...], "totals": {...}} — totals
    carries matched/unmatched/raw counts across every run so a caller that
    swallows the log (rigd's post-drain ingest) can still surface what
    happened; matched==0 with rows present is a failure worth an event."""
    cards = scan_card(card_dir)
    offsets = load_journal_offsets() if offsets is None else offsets
    dir_cam = _dir_camera(card_dir)
    log("card: %d stills (%d with RAW, %d with EXIF) in %s%s"
        % (len(cards), sum(1 for c in cards if c["arw"]),
           sum(1 for c in cards if c["exif"] is not None), card_dir,
           " [staging dir: cam%d only]" % dir_cam if dir_cam else ""))
    mism = sum(1 for c in cards if c.get("exif_mismatch") is not None)
    if mism:
        log("WARNING: %d stems where the ARW and JPEG EXIF stamps disagree "
            "- a stem collision paired foreign files; using the JPEG's" % mism)
    used = set()
    report = []
    # The drain's hash record for these files, when they came from a drain:
    # <dest>/SHA256SUMS lists them as "<node dir>/<name>".
    staging = os.path.basename(os.path.normpath(card_dir))
    drained = {rel.split("/", 1)[1]: sha for rel, sha in read_sums(
        os.path.join(os.path.dirname(os.path.normpath(card_dir)),
                     SUMS_NAME)).items()
        if rel.startswith(staging + "/")}
    hash_mismatch = []
    roots = sorted(glob.glob(os.path.join(runs_dir, "*", "run.json")))
    for rj in roots:
        root = os.path.dirname(rj)
        rid = os.path.basename(root)
        if only and rid not in only:
            continue
        doc, rows, flight = load_run(root)
        if not rows:
            continue
        nidx = sum(len(v) for v in rows.values())
        if doc.get("frames", 0) > nidx:
            log("  %s: WARNING frame index truncated - run.json says %d "
                "frames but only %d rows are readable (no index.jsonl?); "
                "the earlier frames cannot be matched"
                % (rid, doc["frames"], nidx))
        # The run's own shot period bounds how far a time-match may reach:
        # anything beyond ~45% of the period is closer to the NEIGHBOURING
        # frame than to its own. Manual/unknown-interval runs keep the wide
        # default.
        iv = ((doc.get("config") or {}).get("interval_s")
              if isinstance(doc.get("config"), dict) else None)
        run_tol = min(TOL_S, 0.45 * iv) if isinstance(iv, (int, float)) \
            and iv and iv > 0 else None
        # pair spreads by shot for the sidecar
        shots = {}
        for cam, rs in rows.items():
            for r in rs:
                shots.setdefault(round(r["epoch"]), {})[cam] = r["epoch"]
        manifest = []
        summary = {}
        hasher = None if dry else RunHasher(root)
        # Certain matches first: a camera whose card numbering equals its
        # PC-save counter is resolved by number before any camera is matched
        # by time, so the time pass only ever sees the series left over.
        results = {}
        pending = []
        for cam in sorted(rows):
            # A per-node staging dir is authoritative for WHICH body: never
            # let another camera's rows consume its files (sorted() used to
            # hand cam2's whole staging dir to cam1 by time-match).
            if dir_cam is not None and cam != dir_cam:
                results[cam] = (None, None, "other-node", [], [])
                continue
            r = match_run(rows[cam], cards, used, number_only=True,
                          tol_s=run_tol)
            if r[0] is not None:
                results[cam] = r
            else:
                pending.append(cam)
        started = doc.get("started") or min(r["epoch"] for rs in rows.values() for r in rs)
        for cam in pending:
            exp = expected_offset(offsets, "cam%d" % cam, started)
            results[cam] = match_run(rows[cam], cards, used, expect=exp,
                                     authoritative=(dir_cam == cam),
                                     tol_s=run_tol)
        # The run's own ledger of frames it knowingly failed to keep: their
        # card originals are exactly the "leftovers" this pass will refuse to
        # attribute by time alone, so say what they were.
        up_path = os.path.join(root, "unpulled.jsonl")
        if os.path.exists(up_path):
            try:
                with open(up_path) as fh:
                    n_up = sum(1 for ln in fh if ln.strip())
                if n_up:
                    log("  %s: %d frame(s) this run knowingly failed to keep "
                        "(unpulled.jsonl) - expect that many staged card "
                        "files to stay leftover for it" % (rid, n_up))
            except OSError:
                pass
        for cam in sorted(rows):
            series, off, how, matches, unmatched = results[cam]
            # Every unmatched row gets its nearest unclaimed staged file named
            # with the residual: a hole in the card files now refuses the
            # cascade (tolerance is capped by the shot period), and this line
            # is the operator's evidence for a manual attribution.
            if series is not None and off is not None and unmatched:
                pool = [c for c in cards
                        if c["series"] == series and c["stem"] not in used
                        and c["exif"] is not None]
                for r0 in unmatched[:8]:
                    best = min(pool,
                               key=lambda c: abs(c["exif"] - off - r0["epoch"]),
                               default=None)
                    if best is not None:
                        log("  %s cam%d: row %.2f unmatched; nearest staged "
                            "file %s at %+.2f s - outside tolerance, left in "
                            "staging" % (rid, cam, r0["epoch"], best["stem"],
                                         best["exif"] - off - r0["epoch"]))
            summary[cam] = {"series": series, "offset_s": off, "how": how,
                            "matched": len(matches), "unmatched": len(unmatched),
                            "raw": 0, "conflicts": 0, "residual_ms": []}
            if series is None:
                if how.startswith("ambiguous"):
                    log("  %s cam%d: REFUSED (%s) - several card series fit "
                        "and neither the journal offset nor a per-node "
                        "staging path says which body this is. Nothing was "
                        "renamed. Fix by putting each body's files in their "
                        "own directory (ingest ~/rig-raw/cam%d, which is what "
                        "the drain writes) or by restoring ~/rig/rigd.jsonl"
                        % (rid, cam, how, cam))
                elif how == "other-node":
                    log("  %s cam%d: skipped - staging dir belongs to cam%d"
                        % (rid, cam, dir_cam))
                else:
                    log("  %s cam%d: NO MATCH (%d rows) - no card series aligns"
                        % (rid, cam, len(rows[cam])))
                continue
            for r, c, res in matches:
                base = os.path.splitext(r["file"])[0]
                camdir = os.path.join(root, "cam%d" % cam)
                acts = {}
                if c["arw"]:
                    acts["arw"] = place(c["arw"], os.path.join(camdir, base + ".ARW"), move, dry)
                    summary[cam]["raw"] += 1
                if c["jpg"]:
                    acts["jpg"] = place(c["jpg"], os.path.join(camdir, base + ".card.JPG"), move, dry)
                # SHA-256 of what now sits in the run for each card file.
                # A file placed just now is hashed from disk; one placed by
                # an earlier ingest keeps the hash SHA256SUMS recorded.
                shas = {}
                for kind, src, ext in (("arw", c["arw"], ".ARW"),
                                       ("jpg", c["jpg"], ".card.JPG")):
                    held = (_placed_path(acts.get(kind),
                                         os.path.join(camdir, base + ext))
                            if hasher and kind in acts else None)
                    if not held or not os.path.isfile(held):
                        continue
                    shas[kind] = hasher.sha(
                        held, fresh=acts[kind] in _FRESH_PLACEMENTS)
                    want = drained.get(os.path.basename(src))
                    if want and want != shas[kind]:
                        hash_mismatch.append("%s cam%d %s" % (rid, cam,
                                                             hasher.rel(held)))
                        log("  %s cam%d: WARNING %s differs from the hash the "
                            "drain verified for %s (%s... vs %s...) - the "
                            "staged copy changed after the drain"
                            % (rid, cam, hasher.rel(held),
                               os.path.basename(src), shas[kind][:12],
                               want[:12]))
                # a run name already holding DIFFERENT bytes is never
                # overwritten (place() parks the new file beside it) - count
                # it so the operator is told rather than left to find it
                summary[cam]["conflicts"] += sum(
                    1 for v in acts.values() if str(v).startswith("conflict:"))
                sh = shots.get(round(r["epoch"]), {})
                spread = ((max(sh.values()) - min(sh.values())) * 1000.0
                          if len(sh) > 1 else None)
                r["card_stem"] = c["stem"]
                if not dry:
                    # Once a run has been opened in Lightroom/Capture One,
                    # <base>.xmp holds the operator's develop settings under
                    # exactly this name; rewriting it on every auto-drain lost
                    # those edits silently (audit 2026-08-23, low). Rewrite
                    # only our own packet (x:xmptk marker); a foreign sidecar
                    # keeps its name and ours becomes <base>.wildsync.xmp.
                    xp = os.path.join(camdir, base + ".xmp")
                    if os.path.exists(xp):
                        try:
                            with open(xp, errors="replace") as fh:
                                own = "wildsync ingest" in fh.read(2048)
                        except OSError:
                            own = False
                        if not own:
                            xp = os.path.join(camdir, base + ".wildsync.xmp")
                    with open(xp, "w") as fh:
                        fh.write(xmp_sidecar(r, flight.get(r["file"]), rid, cam, res, spread))
                summary[cam]["residual_ms"].append(res * 1000.0)
                manifest.append({"run_id": rid, "cam": cam, "card": c["stem"],
                                 "new_base": base, "epoch": "%.6f" % r["epoch"],
                                 "residual_ms": "%.1f" % (res * 1000.0),
                                 "raw": acts.get("arw", "none"),
                                 "jpg": acts.get("jpg", "none"),
                                 "raw_sha256": shas.get("arw", ""),
                                 "jpg_sha256": shas.get("jpg", "")})
            for r in unmatched:
                manifest.append({"run_id": rid, "cam": cam, "card": "",
                                 "new_base": os.path.splitext(r["file"])[0],
                                 "epoch": "%.6f" % r["epoch"], "residual_ms": "",
                                 "raw": "MISSING ON CARD", "jpg": "",
                                 "raw_sha256": "", "jpg_sha256": ""})
            rm = summary[cam]["residual_ms"]
            log("  %s cam%d: %d/%d matched by %s (series %s, clock offset %+.2f s)"
                " RAW %d | residual median %.1f ms max %.1f ms | unmatched %d%s"
                % (rid, cam, len(matches), len(rows[cam]), how, series, off,
                   summary[cam]["raw"],
                   statistics.median(rm) if rm else 0, max(map(abs, rm)) if rm else 0,
                   len(unmatched),
                   " | CONFLICTS %d (a run name already held different bytes;"
                   " the new file is parked as .conflict-<sha>)"
                   % summary[cam]["conflicts"] if summary[cam]["conflicts"] else ""))
        if manifest and not dry:
            mp = os.path.join(root, "ingest_manifest.csv")
            done_cams = {cam for cam in rows
                         if not (dir_cam is not None and cam != dir_cam)}
            old_rows = []
            if os.path.exists(mp):
                # per-node staging ingests arrive one camera at a time: keep
                # the other camera's rows instead of clobbering the manifest
                try:
                    with open(mp, newline="") as fh:
                        old_rows = [r for r in csv.DictReader(fh)
                                    if int(r.get("cam") or 0) not in done_cams]
                except (OSError, ValueError):
                    old_rows = []
            with open(mp, "w", newline="") as fh:
                # Rows kept from an older manifest have no hash columns;
                # restval leaves those cells empty.
                w = csv.DictWriter(fh, fieldnames=MANIFEST_COLUMNS,
                                   extrasaction="ignore", restval="")
                w.writeheader()
                w.writerows(old_rows + manifest)
            try:
                write_run_sums(root, doc=doc, hasher=hasher, log=log)
            except OSError as e:
                log("  %s: WARNING %s not written: %s" % (rid, SUMS_NAME, e))
        report.append({"run_id": rid, "summary": summary})
    # what the card holds that belongs to no run
    leftover = [c for c in cards if c["stem"] not in used]
    log("card files not attributed to any run: %d (calibration exposures, "
        "bench tests, the last fire of a stopped run) - left untouched"
        % len(leftover))
    totals = {"runs": len(report), "matched": 0, "unmatched": 0, "raw": 0,
              "conflicts": 0, "cards": len(cards),
              "cards_timed": sum(1 for c in cards if c["exif"] is not None),
              "leftover": len(leftover), "exif_mismatch": mism,
              "hash_mismatch": len(hash_mismatch),
              "ambiguous": []}
    for r in report:
        for cam, sm in r["summary"].items():
            totals["matched"] += sm.get("matched", 0)
            totals["unmatched"] += sm.get("unmatched", 0)
            totals["raw"] += sm.get("raw", 0)
            totals["conflicts"] += sm.get("conflicts", 0)
            if str(sm.get("how", "")).startswith("ambiguous"):
                totals["ambiguous"].append("%s cam%s" % (r["run_id"], cam))
    log("ingest totals: %d runs, %d matched, %d unmatched, %d RAW placed, "
        "%d leftover, %d of %d card files timed%s%s%s"
        % (totals["runs"], totals["matched"], totals["unmatched"],
           totals["raw"], totals["leftover"], totals["cards_timed"],
           totals["cards"],
           ", %d CONFLICTS" % totals["conflicts"] if totals["conflicts"] else "",
           ", AMBIGUOUS: " + "; ".join(totals["ambiguous"])
           if totals["ambiguous"] else "",
           ", %d HASH MISMATCH vs the drain record" % totals["hash_mismatch"]
           if totals["hash_mismatch"] else ""))
    return {"runs": report, "leftover": [c["stem"] for c in leftover],
            "totals": totals}


# ---------------------------------------------------------------------------
# self-test: synthetic card + run, both match strategies
# ---------------------------------------------------------------------------
def _selftest():
    sys.path.insert(0, os.path.join(HERE, "tests"))
    from fakenode import make_jpeg
    tmp = tempfile.mkdtemp(prefix="ingest-")
    card = os.path.join(tmp, "card"); os.makedirs(card)
    runs = os.path.join(tmp, "runs")
    rid = "260823_0000_selftest"
    root = os.path.join(runs, rid)
    t0 = 1787253940.0
    off1, off2 = -172769.57, -172748.18          # two bodies, two wrong clocks
    index, fl = [], {1: [], 2: []}
    for k in range(4):
        ep = t0 + k * 0.5
        for cam, off, ser in ((1, off1, "_CA"), (2, off2, "DSC")):
            e = ep + (0.001 if cam == 2 else 0.0)
            fn = "Cam%d_%s.%02d.jpg" % (cam, time.strftime("%Y%m%d_%H%M%S", time.gmtime(e)), int((e % 1) * 100))
            # cam2's card numbering equals its PC-save counter; cam1's does not
            num = 9000 + k if cam == 2 else 30000 + k
            orig = "ILX%05d.JPG" % (9000 + k if cam == 2 else 100 + k)
            index.append({"cam": cam, "file": fn, "orig": orig, "epoch": e,
                          "src": "gpio_edge", "path": "gpio"})
            stem = "%s%05d" % (ser, num)
            with open(os.path.join(card, stem + ".JPG"), "wb") as fh:
                fh.write(make_jpeg(e + off))          # body clock stamps it wrong
            with open(os.path.join(card, stem + ".ARW"), "wb") as fh:
                fh.write(b"RAW" * 100)
            os.makedirs(os.path.join(root, "cam%d" % cam), exist_ok=True)
            # "jetson": a run recorded before v1.1.0 carries the legacy
            # host-clock label, and ingest must keep reading it.
            fl[cam].append({"filename": fn, "lat": "41.4237752", "long": "-71.4537835",
                            "xutm": "294952.55", "yutm": "4588707.83", "utm_zone": "19T",
                            "pitch": "1.2", "roll": "-9.8", "heading_imu": "170.7",
                            "time_source": "jetson"})
    # a calibration frame on the card that belongs to no row
    with open(os.path.join(card, "DSC08999.JPG"), "wb") as fh:
        fh.write(make_jpeg(t0 - 20 + off2))
    for cam in (1, 2):
        with open(os.path.join(root, "cam%d" % cam, "flight_log.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FLIGHT_HEADER, extrasaction="ignore")
            w.writeheader()
            for r in fl[cam]:
                w.writerow({**{k: "" for k in FLIGHT_HEADER}, **r})
    with open(os.path.join(root, "run.json"), "w") as fh:
        json.dump({"run_id": rid, "index": index, "final": True}, fh)

    out = ingest(card, runs_dir=runs, log=lambda *a: None)
    s = out["runs"][0]["summary"]
    assert s[2]["how"] == "number" and s[2]["matched"] == 4, s[2]
    assert s[1]["how"] == "time" and s[1]["matched"] == 4, s[1]
    assert abs(s[1]["offset_s"] - off1) < 0.05 and abs(s[2]["offset_s"] - off2) < 0.05, (s[1], s[2])
    assert all(abs(x) < 20 for x in s[1]["residual_ms"] + s[2]["residual_ms"])
    assert out["leftover"] == ["DSC08999"], out["leftover"]
    arws = sorted(glob.glob(os.path.join(root, "cam1", "Cam1_*.ARW")))
    xmps = sorted(glob.glob(os.path.join(root, "cam2", "Cam2_*.xmp")))
    assert len(arws) == 4 and len(xmps) == 4, (arws, xmps)
    x = open(xmps[0]).read()
    assert "<exif:GPSLatitude>41,25.426512N" in x and "<wildsync:capture_epoch>" in x, x[:400]
    assert "<wildsync:time_source>jetson</wildsync:time_source>" in x, x
    assert os.path.exists(os.path.join(root, "ingest_manifest.csv"))
    # hashes: manifest columns appended, SHA256SUMS over frames + records
    with open(os.path.join(root, "ingest_manifest.csv"), newline="") as fh:
        mrows = list(csv.DictReader(fh))
        assert list(mrows[0].keys()) == MANIFEST_COLUMNS, list(mrows[0].keys())
    for mr in mrows:
        camdir = os.path.join(root, "cam%s" % mr["cam"])
        assert mr["raw_sha256"] == file_sha256(
            os.path.join(camdir, mr["new_base"] + ".ARW")), mr
        assert mr["jpg_sha256"] == file_sha256(
            os.path.join(camdir, mr["new_base"] + ".card.JPG")), mr
    v = verify_run(root, log=lambda *a: None)
    listed = read_sums(os.path.join(root, SUMS_NAME))
    assert v["ok"] == v["listed"] and not v["failed"] and not v["missing"], v
    assert "run.json" in listed and "cam1/flight_log.csv" in listed, listed
    assert not any(k.endswith(".xmp") for k in listed), listed
    assert len([k for k in listed if k.endswith(".ARW")]) == 8, listed
    # idempotent
    out2 = ingest(card, runs_dir=runs, log=lambda *a: None)
    assert out2["runs"][0]["summary"][2]["matched"] == 4
    # a changed frame is caught, and a later --write-sums does not re-bless it
    arw = os.path.join(root, "cam%s" % mrows[0]["cam"],
                       mrows[0]["new_base"] + ".ARW")
    os.unlink(arw)                      # break the hardlink, then rewrite
    with open(arw, "wb") as fh:
        fh.write(b"BITROT" * 50)
    write_run_sums(root, log=lambda *a: None)
    v2 = verify_run(root, log=lambda *a: None)
    assert v2["failed"] == [os.path.relpath(arw, root).replace(os.sep, "/")], v2
    shutil.rmtree(tmp)
    print("INGEST SELF-TEST: PASS (number + time matching, offsets recovered, "
          "RAW renamed, XMP with GPS, manifest, idempotent, hashes, "
          "SHA256SUMS verify)")


def _cli_verify(targets, runs_dir):
    """--verify: exit 0 when every listed file matches, 1 when any failed or
    is missing, 2 when a target has no SHA256SUMS at all."""
    worst = 0
    for t in targets:
        path = resolve_run(t, runs_dir)
        print("%s:" % path)
        rep = verify_run(path, log=lambda m: print("  " + m))
        if rep["sums"] is None:
            print("  no %s here - nothing was recorded (a run never ingested "
                  "or recorded before v1.1.0; --write-sums records one)"
                  % SUMS_NAME)
            worst = max(worst, 2)
            continue
        bad = len(rep["failed"]) + len(rep["missing"])
        print("  %d listed: %d OK, %d FAILED, %d MISSING%s"
              % (rep["listed"], rep["ok"], len(rep["failed"]),
                 len(rep["missing"]),
                 "; %d file(s) present but not listed (written after %s?)"
                 % (len(rep["unlisted"]), SUMS_NAME) if rep["unlisted"]
                 else ""))
        print("  %s" % ("VERIFIED" if not bad else "NOT VERIFIED"))
        if bad:
            worst = max(worst, 1)
    return worst


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("card_dir", nargs="?", help="directory of downloaded card files")
    ap.add_argument("--runs", default=RUNS_DIR)
    ap.add_argument("--run", action="append", help="only this run id (repeatable)")
    ap.add_argument("--move", action="store_true", help="remove card-dir copies after placing")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verify", action="append", metavar="RUN",
                    help="re-hash everything RUN's SHA256SUMS lists (a run id "
                         "under --runs, or any directory holding a SHA256SUMS, "
                         "e.g. the drain's ~/rig-raw); repeatable")
    ap.add_argument("--write-sums", action="append", metavar="RUN",
                    help="write or extend RUN's SHA256SUMS without ingesting "
                         "(files already listed keep their recorded hash); "
                         "repeatable")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest(); return
    if a.verify or a.write_sums:
        for t in a.write_sums or []:
            path = resolve_run(t, a.runs)
            if not os.path.isfile(os.path.join(path, "run.json")):
                ap.error("%s is not a run directory (no run.json)" % path)
            write_run_sums(path)
        if a.verify:
            sys.exit(_cli_verify(a.verify, a.runs))
        return
    if not a.card_dir:
        ap.error("card_dir is required")
    ingest(a.card_dir, runs_dir=a.runs, only=a.run, move=a.move, dry=a.dry_run)


if __name__ == "__main__":
    main()

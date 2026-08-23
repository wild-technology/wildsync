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

    python3 rig/ingest.py /Users/wild/Pictures/2026/2026-08-18
    python3 rig/ingest.py <card_dir> --run 260820_1925_transect-01 --move
    python3 rig/ingest.py <card_dir> --dry-run
    python3 rig/ingest.py --selftest

Matching, per run and camera, in order:
  1. by the body's file number when the card series shares the PC-save
     counter (cam2: ILX09187 <-> DSC09187), confirmed by a tight clock offset;
  2. else by time: the offset between the body's EXIF clock and the rig's
     epochs is found as the mode of all pairwise differences, then each row is
     matched to the nearest card file within TOL. No prior knowledge of the
     body's clock error is needed — it is measured from the data.
Originals are hardlinked into the run (no extra space on the same volume) or
copied; --move removes the card-dir copy afterwards. Nothing is written unless
every step of a run's match is consistent; the manifest says what happened.
"""
import argparse
import csv
import glob
import json
import os
import re
import shutil
import statistics
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run import _exif_capture_epoch, FLIGHT_HEADER  # noqa: E402

TOL_S = 0.75          # max |card exif - rig epoch| after offset removal
BIN_S = 0.25          # histogram bin for offset discovery
MIN_FRAC = 0.7        # fraction of rows a candidate offset must explain
RUNS_DIR = os.path.expanduser("~/rig-runs")


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
    return list(out.values())


# ---------------------------------------------------------------------------
# runs
# ---------------------------------------------------------------------------
def load_run(root):
    with open(os.path.join(root, "run.json")) as fh:
        doc = json.load(fh)
    rows = {}
    for e in doc.get("index", []):
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
    camera_epoch - true_epoch for every node at every run start; it is the
    authoritative way to tell two card series apart when both bodies shot
    the same instants."""
    out = []
    try:
        with open(path) as fh:
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
        pass
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


def _offset_by_time(rows, cards, expect=None):
    """The clock offset as the mode of all pairwise (exif - epoch) differences
    over every card series, then the series that carries it. Two bodies shoot
    the same instants, so every series aligns with every camera's rows at
    SOME offset: when rigd's journal says what this body's offset should be,
    the series whose offset agrees wins; otherwise the strongest mode does."""
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
        return None, None
    need = max(2, int(MIN_FRAC * len(rows)))
    cands = [(k, n) for k, n in hist.items() if n >= need]
    if not cands:
        return None, None
    if expect is not None:
        near = [(k, n) for k, n in cands if abs(k[1] * BIN_S - expect) <= 2.0]
        if near:
            cands = near
    (series, kbin), n = max(cands, key=lambda kv: kv[1])
    # refine: median of diffs inside the winning bin (+-1 bin)
    diffs = [c["exif"] - r["epoch"] for r in rows for c in cards
             if c["series"] == series and c["exif"] is not None
             and abs(c["exif"] - r["epoch"] - kbin * BIN_S) <= 1.5 * BIN_S]
    return series, statistics.median(diffs)


def match_run(rows, cards, used, expect=None, number_only=False):
    """rows: index entries for one camera. Returns (series, offset, how,
    matches, unmatched) where matches = [(row, card, residual_s)]."""
    avail = [c for c in cards if c["stem"] not in used and c["exif"] is not None]
    series, off = _offset_by_number(rows, avail)
    how = "number"
    if series is None:
        if number_only:
            return None, None, how, [], list(rows)
        series, off = _offset_by_time(rows, avail, expect)
        how = "time"
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
            if abs(res) <= TOL_S and (best is None or abs(res) < abs(best[1])):
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
    return (time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(epoch))
            + ".%03dZ" % int(round((epoch % 1) * 1000)))


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
    """Hardlink (same volume) or copy src to dst; --move removes src."""
    if dry:
        return "would %s" % ("move" if move else "link")
    if os.path.exists(dst) and os.path.getsize(dst) == os.path.getsize(src):
        if move and os.path.exists(src):
            os.unlink(src)
        return "exists"
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    try:
        os.link(src, dst)
        how = "linked"
    except OSError:
        shutil.copy2(src, dst)
        how = "copied"
    if move:
        os.unlink(src)
        how = "moved"
    return how


def ingest(card_dir, runs_dir=RUNS_DIR, only=None, move=False, dry=False,
           log=print, offsets=None):
    cards = scan_card(card_dir)
    offsets = load_journal_offsets() if offsets is None else offsets
    log("card: %d stills (%d with RAW, %d with EXIF) in %s"
        % (len(cards), sum(1 for c in cards if c["arw"]),
           sum(1 for c in cards if c["exif"] is not None), card_dir))
    used = set()
    report = []
    roots = sorted(glob.glob(os.path.join(runs_dir, "*", "run.json")))
    for rj in roots:
        root = os.path.dirname(rj)
        rid = os.path.basename(root)
        if only and rid not in only:
            continue
        doc, rows, flight = load_run(root)
        if not rows:
            continue
        # pair spreads by shot for the sidecar
        shots = {}
        for cam, rs in rows.items():
            for r in rs:
                shots.setdefault(round(r["epoch"]), {})[cam] = r["epoch"]
        manifest = []
        summary = {}
        # Certain matches first: a camera whose card numbering equals its
        # PC-save counter is resolved by number before any camera is matched
        # by time, so the time pass only ever sees the series left over.
        results = {}
        pending = []
        for cam in sorted(rows):
            r = match_run(rows[cam], cards, used, number_only=True)
            if r[0] is not None:
                results[cam] = r
            else:
                pending.append(cam)
        started = doc.get("started") or min(r["epoch"] for rs in rows.values() for r in rs)
        for cam in pending:
            exp = expected_offset(offsets, "cam%d" % cam, started)
            results[cam] = match_run(rows[cam], cards, used, expect=exp)
        for cam in sorted(rows):
            series, off, how, matches, unmatched = results[cam]
            summary[cam] = {"series": series, "offset_s": off, "how": how,
                            "matched": len(matches), "unmatched": len(unmatched),
                            "raw": 0, "residual_ms": []}
            if series is None:
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
                sh = shots.get(round(r["epoch"]), {})
                spread = ((max(sh.values()) - min(sh.values())) * 1000.0
                          if len(sh) > 1 else None)
                r["card_stem"] = c["stem"]
                if not dry:
                    with open(os.path.join(camdir, base + ".xmp"), "w") as fh:
                        fh.write(xmp_sidecar(r, flight.get(r["file"]), rid, cam, res, spread))
                summary[cam]["residual_ms"].append(res * 1000.0)
                manifest.append({"run_id": rid, "cam": cam, "card": c["stem"],
                                 "new_base": base, "epoch": "%.6f" % r["epoch"],
                                 "residual_ms": "%.1f" % (res * 1000.0),
                                 "raw": acts.get("arw", "none"),
                                 "jpg": acts.get("jpg", "none")})
            for r in unmatched:
                manifest.append({"run_id": rid, "cam": cam, "card": "",
                                 "new_base": os.path.splitext(r["file"])[0],
                                 "epoch": "%.6f" % r["epoch"], "residual_ms": "",
                                 "raw": "MISSING ON CARD", "jpg": ""})
            rm = summary[cam]["residual_ms"]
            log("  %s cam%d: %d/%d matched by %s (series %s, clock offset %+.2f s)"
                " RAW %d | residual median %.1f ms max %.1f ms | unmatched %d"
                % (rid, cam, len(matches), len(rows[cam]), how, series, off,
                   summary[cam]["raw"],
                   statistics.median(rm) if rm else 0, max(map(abs, rm)) if rm else 0,
                   len(unmatched)))
        if manifest and not dry:
            mp = os.path.join(root, "ingest_manifest.csv")
            with open(mp, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(manifest[0].keys()))
                w.writeheader()
                w.writerows(manifest)
        report.append({"run_id": rid, "summary": summary})
    # what the card holds that belongs to no run
    leftover = [c for c in cards if c["stem"] not in used]
    log("card files not attributed to any run: %d (calibration exposures, "
        "bench tests, the last fire of a stopped run) - left untouched"
        % len(leftover))
    return {"runs": report, "leftover": [c["stem"] for c in leftover]}


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
    assert os.path.exists(os.path.join(root, "ingest_manifest.csv"))
    # idempotent
    out2 = ingest(card, runs_dir=runs, log=lambda *a: None)
    assert out2["runs"][0]["summary"][2]["matched"] == 4
    shutil.rmtree(tmp)
    print("INGEST SELF-TEST: PASS (number + time matching, offsets recovered, "
          "RAW renamed, XMP with GPS, manifest, idempotent)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("card_dir", nargs="?", help="directory of downloaded card files")
    ap.add_argument("--runs", default=RUNS_DIR)
    ap.add_argument("--run", action="append", help="only this run id (repeatable)")
    ap.add_argument("--move", action="store_true", help="remove card-dir copies after placing")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest(); return
    if not a.card_dir:
        ap.error("card_dir is required")
    ingest(a.card_dir, runs_dir=a.runs, only=a.run, move=a.move, dry=a.dry_run)


if __name__ == "__main__":
    main()

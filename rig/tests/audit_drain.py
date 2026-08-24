#!/usr/bin/env python3
"""Audit regression suite — drain / ingest / stereo_check lane (G1..G7).

G1  The drain pulled only the RAW of each RAW+JPEG content and then deleted
    the CONTENT, destroying the never-pulled card JPEG of every shot; ingest
    read EXIF only from JPEGs, so after a drain (169 ARW, 0 XMP in the field)
    it matched nothing and said so to nobody. Now the drain's unit of work is
    the contentId — every file of a content is pulled and sha256-verified
    before the single card delete — ingest reads DateTimeOriginal/SubSec out
    of the ARW's own TIFF EXIF, and ingest() returns a totals dict its caller
    (rigd's post-drain ingest, which passes log=lambda *a: None) can emit.

G2  Two card files can carry the same name (file-number reset, counter wrap):
    the name-keyed host path replaced an earlier verified RAW whose card
    original was already deleted. A host file is never overwritten with
    different bytes now — the colliding content lands as <stem>-c<contentId>.

G3  run.json keeps only the last 2000 index entries, so a long transect's head
    silently dropped out of every match and every stereo check. Both readers
    now prefer the append-only <run>/index.jsonl (contract C3).

G4  A standalone `python3 rig/drain.py cam1` is defeated by rigd's stuck-
    transfer recovery; the CLI now hands the drain to a running rigd.

G5  With a combined card directory and no journal offset, attribution fell to
    dict order and could swap cam1's and cam2's RAWs. It now refuses
    ("ambiguous: ..."), and a per-node staging dir is authoritative.

G6  _iso() carry (".1000Z"), stereo_check tracebacking on a missing card JPEG,
    place(--move) deleting a source after a size-only comparison, and ingest
    clobbering a foreign <base>.xmp.

G7  Drainer.run(stop=Event) cancels at a content boundary (contract C4).

Hermetic: stub cameras and a loopback HTTP stub only — the live rigd on
localhost:9090 and the fleet on 192.168.1.x are never contacted (every rigd
call in here takes an explicit 127.0.0.1 base). Temp dirs only.

Run standalone:  python3 rig/tests/audit_drain.py
"""

import hashlib
import http.server
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

from soaktest import check, sect, note          # noqa: E402
import drain as draindrv                         # noqa: E402
import ingest as ing                             # noqa: E402
from fakenode import make_jpeg                   # noqa: E402

RAW = draindrv.RAW_FORMAT
JPEG = draindrv.JPEG_FORMAT

TMPS = []


def _tmp(prefix="audit-drain-"):
    d = tempfile.mkdtemp(prefix=prefix)
    TMPS.append(d)
    return d


def _cleanup():
    for d in TMPS:
        shutil.rmtree(d, ignore_errors=True)
    del TMPS[:]


# ---------------------------------------------------------------------------
# a synthetic Sony ARW: a bare little-endian TIFF whose ExifIFD carries only
# DateTimeOriginal + SubSecTimeOriginal, written by hand so the test never
# depends on a RAW library. Same LOCAL-time encoding fakenode.make_jpeg uses,
# because run._exif_capture_epoch decodes with time.mktime.
# ---------------------------------------------------------------------------
def make_arw(epoch, pad=0):
    dt = time.strftime("%Y:%m:%d %H:%M:%S", time.localtime(epoch)).encode() + b"\x00"
    ss = ("%02d" % int(round((epoch % 1) * 100))).encode() + b"\x00"
    ifd0, exif = 8, 8 + 18                 # header 8, IFD0 2+12+4, EXIF 2+24+4
    data = exif + 30
    out = bytearray()
    out += b"II*\x00" + struct.pack("<I", ifd0)
    out += struct.pack("<H", 1)
    out += struct.pack("<HHII", 0x8769, 4, 1, exif)          # ExifIFD pointer
    out += struct.pack("<I", 0)
    out += struct.pack("<H", 2)
    out += struct.pack("<HHII", 0x9003, 2, len(dt), data)    # DateTimeOriginal
    out += struct.pack("<HHI", 0x9291, 2, len(ss)) + ss.ljust(4, b"\x00")
    out += struct.pack("<I", 0)
    assert len(out) == data, (len(out), data)
    return bytes(out) + dt + ss + b"\x00" * pad


# ---------------------------------------------------------------------------
# stub camera — drain._selftest's Stub, made scriptable
# ---------------------------------------------------------------------------
class StubCam(draindrv.Drainer):
    """A card of CONTENTS: {contentId: [(name, bytes, format), ...]}.

    `fail_pull` names files whose /api/card/pull reports a failure (a stalled
    SDK transfer); `corrupt` names files whose fetched bytes do not match the
    hash the Pi reported.
    """

    def __init__(self, dest, card, node="camX", fail_pull=(), corrupt=()):
        super().__init__(node, "0.0.0.0", dest=dest, log=lambda *a: None)
        self.card = card
        self.fail_pull = set(fail_pull)
        self.corrupt = set(corrupt)
        self.mode = None
        self.deleted, self.pi_deleted, self.pulled = [], [], []
        self.on_delete = None

    def set_mode(self, mode):
        self.mode = mode
        return {"ok": True, "mode": mode}

    def card_ready(self):
        return {"ok": True, "ready": True, "mode": self.mode or "transfer"}

    def card_list(self, tries=30, delay=3.0):
        out = []
        for cid, files in sorted(self.card.items()):
            for fid, (nm, data, fmt) in enumerate(files, 1):
                out.append({"contentId": cid, "fileId": fid, "name": nm,
                            "size": len(data), "format": fmt,
                            "dirNumber": 100, "fileNumber": fid})
        return out

    def _bytes_of(self, name):
        for files in self.card.values():
            for (nm, data, _f) in files:
                if nm == name:
                    return data
        return None

    def pull(self, f):
        self.pulled.append(f["name"])
        if f["name"] in self.fail_pull:
            return {"ok": False, "error": "transfer failed"}
        d = self._bytes_of(f["name"])
        return {"ok": True, "name": f["name"], "bytes": len(d),
                "sha256": hashlib.sha256(d).hexdigest()}

    def fetch(self, name):
        if name in self.corrupt:
            return b"CORRUPT"
        return self._bytes_of(name)

    def card_delete(self, cid):
        self.card.pop(cid, None)
        self.deleted.append(cid)
        if self.on_delete:
            self.on_delete(cid)
        return {"ok": True}

    def pi_delete_raw(self, name):
        self.pi_deleted.append(name)
        return {"ok": True}

    def _host_free_mb(self):
        return 999999


def _host_files(dest):
    return sorted(os.listdir(dest)) if os.path.isdir(dest) else []


# ---------------------------------------------------------------------------
# G1(a) — whole-content drain
# ---------------------------------------------------------------------------
def _g1_whole_content(opts):
    sect("audit G1a: a shot is RAW+JPEG under one contentId; the card delete "
         "takes both, so both must be verified on the host first")
    d = _tmp()
    xr, xj = b"RAW-X" * 4000, make_jpeg(1787253940.0)
    yr, yj = b"RAW-Y" * 4000, make_jpeg(1787253941.0)
    s = StubCam(d, {10: [("X.JPG", xj, JPEG), ("X.ARW", xr, RAW)],
                    11: [("Y.JPG", yj, JPEG), ("Y.ARW", yr, RAW)]},
                node="cam1", fail_pull={"X.JPG"})
    rep = s.run()
    dest = os.path.join(d, "cam1")
    check("every file of a drained content is pulled, not just the RAW",
          "Y.JPG" in s.pulled and "Y.ARW" in s.pulled, str(s.pulled))
    check("the verified shot's card JPEG survives on the host",
          os.path.isfile(os.path.join(dest, "Y.JPG"))
          and os.path.isfile(os.path.join(dest, "Y.ARW")),
          str(_host_files(dest)))
    check("only the fully verified content was deleted from the card",
          s.deleted == [11] and 10 in s.card, str(s.deleted))
    check("the half-transferred content is reported, not silently dropped",
          any("X.JPG" in e for e in rep["errors"]), json.dumps(rep["errors"]))
    check("counts are per shot: 1 pulled, 1 verified, 1 deleted",
          (rep["pulled"], rep["verified"], rep["deleted"]) == (1, 1, 1),
          json.dumps({k: rep[k] for k in ("pulled", "verified", "deleted")}))
    check("no .part staging file is left behind",
          not any(".part" in n for n in _host_files(dest)), str(_host_files(dest)))

    # a corrupt half must keep the WHOLE content, including the good half's
    # card original
    d2 = _tmp()
    s2 = StubCam(d2, {20: [("Z.JPG", yj, JPEG), ("Z.ARW", yr, RAW)]},
                 node="cam1", corrupt={"Z.ARW"})
    rep2 = s2.run()
    check("a hash mismatch on one file keeps the whole content on the card",
          s2.deleted == [] and 20 in s2.card
          and any("HASH MISMATCH" in e for e in rep2["errors"]),
          json.dumps(rep2["errors"][:2]))
    check("the camera is returned to remote mode either way", s2.mode == "remote",
          str(s2.mode))


# ---------------------------------------------------------------------------
# G2 — duplicate card filenames
# ---------------------------------------------------------------------------
def _g2_duplicate_names(opts):
    sect("audit G2: the same card filename twice must never overwrite an "
         "already-verified host RAW whose card original is gone")
    d = _tmp()
    a, b = b"AAAA" * 3000, b"BBBB" * 3000
    s = StubCam(d, {30: [("DSC00042.ARW", a, RAW)],
                    31: [("DSC00042.ARW", b, RAW)]}, node="cam1")
    rep = s.run()
    dest = os.path.join(d, "cam1")
    blobs = {}
    for n in _host_files(dest):
        with open(os.path.join(dest, n), "rb") as fh:
            blobs[n] = fh.read()
    check("both same-named contents are on the host under distinct names",
          sorted(blobs.values()) == sorted([a, b]),
          "%s -> %s" % (_host_files(dest), [len(v) for v in blobs.values()]))
    check("the first verified file keeps the plain card name",
          blobs.get("DSC00042.ARW") == a, str(_host_files(dest)))
    check("the collision is disambiguated by contentId",
          any(re.match(r"DSC00042-c31\.ARW$", n) for n in blobs), str(list(blobs)))
    check("both card originals were deleted (nothing kept hostage)",
          sorted(s.deleted) == [30, 31] and rep["deleted"] == 2, str(s.deleted))

    # ... and across drains, once the first original is long gone
    s2 = StubCam(d, {32: [("DSC00042.ARW", b"CCCC" * 3000, RAW)]}, node="cam1")
    s2.run()
    files = _host_files(dest)
    check("a later drain with the same name again adds a third file",
          len(files) == 3 and "DSC00042.ARW" in files, str(files))
    with open(os.path.join(dest, "DSC00042.ARW"), "rb") as fh:
        check("the original verified bytes are still under the plain name",
              fh.read() == a)


# ---------------------------------------------------------------------------
# G7 — cancellable drain (contract C4)
# ---------------------------------------------------------------------------
def _g7_stop_event(opts):
    sect("audit G7/C4: a drain cancels at a content boundary, never between "
         "pull-verify and delete")
    d = _tmp()
    card = {}
    for i in range(4):
        card[40 + i] = [("S%d.JPG" % i, make_jpeg(1787253940.0 + i), JPEG),
                        ("S%d.ARW" % i, b"R%d" % i * 2000, RAW)]
    s = StubCam(d, card, node="cam1")
    stop = threading.Event()
    s.on_delete = lambda cid: stop.set() if len(s.deleted) >= 2 else None
    rep = s.run(stop=stop)
    dest = os.path.join(d, "cam1")
    check("the report says it was cancelled", rep["cancelled"] is True,
          json.dumps({k: rep[k] for k in ("cancelled", "pulled", "deleted")}))
    check("exactly the contents it finished were deleted",
          len(s.deleted) == 2 and rep["deleted"] == 2, str(s.deleted))
    check("the untouched contents are still whole on the card",
          len(s.card) == 2 and all(len(v) == 2 for v in s.card.values()),
          str({k: len(v) for k, v in s.card.items()}))
    on_host = set(_host_files(dest))
    check("no shot is half on the host and half on the card",
          all((("S%d.JPG" % i in on_host) == ("S%d.ARW" % i in on_host))
              for i in range(4)), str(sorted(on_host)))
    s3 = StubCam(_tmp(), {50: [("Q.ARW", b"Q" * 100, RAW)]}, node="cam1")
    pre = threading.Event()
    pre.set()
    rep3 = s3.run(stop=pre)
    check("stop set before the first content drains nothing at all",
          rep3["cancelled"] is True and rep3["pulled"] == 0
          and s3.deleted == [] and s3.mode == "remote",
          json.dumps({"cancelled": rep3["cancelled"], "pulled": rep3["pulled"]}))


# ---------------------------------------------------------------------------
# G1(b) — EXIF straight out of the ARW
# ---------------------------------------------------------------------------
def _g1_arw_exif(opts):
    sect("audit G1b: ingest must be able to time a bare .ARW (a RAW-only "
         "staging tree) from the file's own TIFF EXIF")
    d = _tmp()
    t = 1787253940.64
    with open(os.path.join(d, "DSC00001.ARW"), "wb") as fh:
        fh.write(make_arw(t))                       # RAW only
    with open(os.path.join(d, "DSC00002.ARW"), "wb") as fh:
        fh.write(make_arw(t + 1))
    with open(os.path.join(d, "DSC00002.JPG"), "wb") as fh:
        fh.write(make_jpeg(t + 1))                  # agreeing pair
    with open(os.path.join(d, "DSC00003.ARW"), "wb") as fh:
        fh.write(make_arw(t + 60))
    with open(os.path.join(d, "DSC00003.JPG"), "wb") as fh:
        fh.write(make_jpeg(t + 2))                  # a stem collision
    with open(os.path.join(d, "DSC00004.ARW"), "wb") as fh:
        fh.write(b"not a tiff at all, torn download")
    cards = {c["stem"]: c for c in ing.scan_card(d)}
    check("a RAW with no JPEG is timed from the ARW",
          cards["DSC00001"]["exif"] is not None
          and abs(cards["DSC00001"]["exif"] - t) < 0.02,
          str(cards["DSC00001"]["exif"]))
    check("with both present the JPEG's stamp is the one used",
          abs(cards["DSC00002"]["exif"] - (t + 1)) < 0.02
          and cards["DSC00002"].get("exif_mismatch") is None,
          str(cards["DSC00002"]["exif"]))
    check("an ARW/JPEG stamp disagreement is flagged, not averaged",
          cards["DSC00003"].get("exif_mismatch") is not None
          and abs(cards["DSC00003"]["exif"] - (t + 2)) < 0.02,
          str(cards["DSC00003"].get("exif_mismatch")))
    check("a torn ARW yields no time and no exception",
          cards["DSC00004"]["exif"] is None)
    check("the sub-second field survives (0.64 s, not 0)",
          abs((cards["DSC00001"]["exif"] % 1) - 0.64) < 0.02,
          str(cards["DSC00001"]["exif"] % 1))


# ---------------------------------------------------------------------------
# run fixtures
# ---------------------------------------------------------------------------
def _write_run(runs, rid, per_cam, cams=(1, 2), t0=1787253940.0, step=0.5,
               jsonl=True, truncate_json=None):
    """A run root with index.jsonl (full) and run.json (tail-truncated the way
    run.py does). Returns (root, {cam: [rows]})."""
    root = os.path.join(runs, rid)
    for cam in cams:
        os.makedirs(os.path.join(root, "cam%d" % cam), exist_ok=True)
    idx, rows = [], {c: [] for c in cams}
    for k in range(per_cam):
        ep = t0 + k * step
        for cam in cams:
            e = ep + (0.001 if cam == 2 else 0.0)
            r = {"cam": cam,
                 "file": "Cam%d_%s.%02d.jpg" % (cam,
                                                time.strftime("%Y%m%d_%H%M%S",
                                                              time.gmtime(e)),
                                                int((e % 1) * 100)),
                 "orig": "ILX%05d.JPG" % (1000 + k), "epoch": e,
                 "src": "gpio_edge"}
            idx.append(r)
            rows[cam].append(r)
    if jsonl:
        with open(os.path.join(root, "index.jsonl"), "w") as fh:
            for e in idx:
                fh.write(json.dumps(e) + "\n")
    keep = idx[-2000:] if truncate_json is None else idx[-truncate_json:]
    with open(os.path.join(root, "run.json"), "w") as fh:
        json.dump({"run_id": rid, "frames": len(idx), "index": keep,
                   "final": True, "started": t0}, fh)
    return root, rows


def _write_card(card_dir, rows, series, offset, numbers=True, jpg=False):
    os.makedirs(card_dir, exist_ok=True)
    for k, r in enumerate(rows):
        n = (1000 + k) if numbers else (70000 + k)
        stem = "%s%05d" % (series, n)
        with open(os.path.join(card_dir, stem + ".ARW"), "wb") as fh:
            fh.write(make_arw(r["epoch"] + offset))
        if jpg:
            with open(os.path.join(card_dir, stem + ".JPG"), "wb") as fh:
                fh.write(make_jpeg(r["epoch"] + offset))


# ---------------------------------------------------------------------------
# G1(c) — a RAW-only staging dir ingests, and says what it did
# ---------------------------------------------------------------------------
def _g1_raw_only_ingest(opts):
    sect("audit G1c: the post-drain ingest of a RAW-only staging dir matches, "
         "and returns totals for a caller that swallows the log")
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "260823_0000_rawonly", 6)
    card = os.path.join(d, "cam1")
    _write_card(card, rows[1], "_CA", -172769.57, numbers=False)
    logged = []
    out = ing.ingest(card, runs_dir=runs, log=logged.append, offsets=[])
    check("ingest returns per-run totals (rigd passes log=lambda *a: None and "
          "had nothing to emit)",
          isinstance(out, dict) and isinstance(out.get("totals"), dict),
          "returned %s, totals=%r" % (type(out).__name__,
                                      (out or {}).get("totals")))
    t = (out or {}).get("totals", {})
    check("every RAW-only card file was matched to its frame",
          t.get("matched") == 6 and t.get("raw") == 6 and t.get("leftover") == 0,
          json.dumps(t))
    check("the RAW landed under the rig's name with an XMP beside it",
          os.path.isfile(os.path.join(root, "cam1",
                                      os.path.splitext(rows[1][0]["file"])[0] + ".ARW"))
          and os.path.isfile(os.path.join(root, "cam1",
                                          os.path.splitext(rows[1][0]["file"])[0] + ".xmp")))
    check("the totals say how many card files could be timed",
          t.get("cards_timed") == 6 and t.get("cards") == 6, json.dumps(t))
    check("the log names the staging dir's camera",
          any("cam1 only" in m for m in logged), "; ".join(logged[:2]))


# ---------------------------------------------------------------------------
# G3 — the FULL frame index (contract C3)
# ---------------------------------------------------------------------------
def _g3_index_jsonl(opts):
    sect("audit G3/C3: readers take <run>/index.jsonl, not run.json's last "
         "2000 entries")
    d = _tmp()
    runs = os.path.join(d, "runs")
    per_cam = 1050                       # 2100 entries: run.json drops 100
    root, rows = _write_run(runs, "260823_0000_long", per_cam)
    idx = ing.read_frame_index(root)
    check("read_frame_index returns every frame, past 2000",
          len(idx) == 2 * per_cam, "%d rows" % len(idx))
    with open(os.path.join(root, "run.json")) as fh:
        check("run.json really is truncated (the defect is present)",
              len(json.load(fh)["index"]) == 2000)
    with open(os.path.join(root, "index.jsonl"), "a") as fh:
        fh.write('{"cam": 1, "file": "Cam1_tor')     # killed mid-append
    check("a torn last line is tolerated, not fatal",
          len(ing.read_frame_index(root)) == 2 * per_cam,
          "%d rows" % len(ing.read_frame_index(root)))

    for cam in (1, 2):
        _write_card(os.path.join(d, "cam%d" % cam), rows[cam],
                    "_CA" if cam == 1 else "DSC", -172769.57 + cam)
    tot = {"matched": 0, "leftover": 0}
    for cam in (1, 2):
        t = ing.ingest(os.path.join(d, "cam%d" % cam), runs_dir=runs,
                       log=lambda *a: None, offsets=[])["totals"]
        tot["matched"] += t["matched"]
        tot["leftover"] += t["leftover"]
    check("every frame of a >2000-frame transect is matched, head included",
          tot["matched"] == 2 * per_cam and tot["leftover"] == 0,
          json.dumps(tot))
    first = os.path.splitext(rows[1][0]["file"])[0]
    check("the FIRST shot - the one run.json's tail dropped - got its RAW",
          os.path.isfile(os.path.join(root, "cam1", first + ".ARW")), first)

    try:
        import stereo_check
    except Exception as e:                              # noqa: BLE001
        note("stereo_check not importable (%s) - pair-index check skipped" % e)
        return
    check("stereo_check pairs the whole run, not just run.json's tail",
          len(stereo_check.pairs_of(root)) == per_cam,
          "%d pairs" % len(stereo_check.pairs_of(root)))


# ---------------------------------------------------------------------------
# G5 — series attribution
# ---------------------------------------------------------------------------
def _g5_attribution(opts):
    sect("audit G5: with no journal offset a combined card dir must refuse to "
         "guess which body a series came from")
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "260823_0000_ambig", 6)
    both = os.path.join(d, "download")             # NOT a per-node staging dir
    _write_card(both, rows[1], "_CA", -172769.57, numbers=False)
    _write_card(both, rows[2], "DSC", -172748.18, numbers=False)
    out = ing.ingest(both, runs_dir=runs, log=lambda *a: None, offsets=[])
    sm = out["runs"][0]["summary"]
    check("both cameras are refused as ambiguous, not attributed by dict order",
          all(str(sm[c]["how"]).startswith("ambiguous") for c in (1, 2)),
          json.dumps({c: sm[c]["how"] for c in (1, 2)}))
    check("the refusal names the tied series so the operator can fix it",
          "_CA" in sm[1]["how"] and "DSC" in sm[1]["how"], sm[1]["how"])
    check("nothing was placed under a run name while ambiguous",
          out["totals"]["raw"] == 0 and out["totals"]["ambiguous"],
          json.dumps(out["totals"]))

    # the journal offset resolves it
    started = rows[1][0]["epoch"]
    offs = [(started, "cam1", -172769.57), (started, "cam2", -172748.18)]
    out2 = ing.ingest(both, runs_dir=runs, log=lambda *a: None, offsets=offs)
    s2 = out2["runs"][0]["summary"]
    check("rigd's journal offset resolves the same directory",
          s2[1]["series"] == "_CA" and s2[2]["series"] == "DSC",
          json.dumps({c: s2[c]["series"] for c in (1, 2)}))

    # the journal rotates at 16 MB; the offsets in the rotated half still count
    jd = _tmp()
    jp = os.path.join(jd, "rigd.jsonl")
    with open(jp + ".1", "w") as fh:
        fh.write(json.dumps({"ts": started, "kind": "exif_offset",
                             "node": "cam1",
                             "msg": "cam1 EXIF offset -172769.57s"}) + "\n")
    with open(jp, "w") as fh:
        fh.write("not json at all\n")
        fh.write(json.dumps({"ts": started + 1, "kind": "exif_offset",
                             "node": "cam2",
                             "msg": "cam2 EXIF offset -172748.18s"}) + "\n")
    got = ing.load_journal_offsets(jp)
    check("a rotated rigd.jsonl.1 is read as well as the current journal",
          sorted(n for _t, n, _o in got) == ["cam1", "cam2"], str(got))

    # a per-node staging dir is authoritative on its own
    d2 = _tmp()
    runs2 = os.path.join(d2, "runs")
    root2, rows2 = _write_run(runs2, "260823_0000_staged", 6)
    stage = os.path.join(d2, "cam2")
    _write_card(stage, rows2[2], "DSC", -172748.18, numbers=False)
    out3 = ing.ingest(stage, runs_dir=runs2, log=lambda *a: None, offsets=[])
    s3 = out3["runs"][0]["summary"]
    check("~/rig-raw/cam2 attributes to cam2 with no journal at all",
          s3[2]["matched"] == 6 and s3[2]["series"] == "DSC", json.dumps(s3[2]))
    check("and cam1's rows never consume cam2's staging dir",
          s3[1]["how"] == "other-node" and s3[1]["matched"] == 0,
          json.dumps(s3[1]))


# ---------------------------------------------------------------------------
# G6 — the small ones
# ---------------------------------------------------------------------------
def _g6_iso_carry(opts):
    sect("audit G6: _iso() must carry the millisecond rounding into the second")
    iso = ing._iso(1787253940.9996)
    check("no '.1000Z' - the carry lands in the seconds field",
          re.match(r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}Z$", iso) is not None
          and ".1000Z" not in iso, iso)
    check("the carried stamp is the next second, not a second early",
          iso.endswith(":41.000Z"), iso)
    check("an ordinary epoch is unchanged",
          ing._iso(1787253940.25).endswith(":40.250Z"), ing._iso(1787253940.25))


def _g6_place_move(opts):
    sect("audit G6: place(--move) must compare CONTENT before it removes a "
         "source (Uncompressed RAW makes every file the same size)")
    d = _tmp()
    src = os.path.join(d, "card.ARW")
    dst = os.path.join(d, "run", "Cam1_x.ARW")
    os.makedirs(os.path.dirname(dst))
    with open(src, "wb") as fh:
        fh.write(b"CORRECT-" * 512)
    with open(dst, "wb") as fh:
        fh.write(b"WRONGFIL" * 512)            # same size, different image
    how = ing.place(src, dst, move=True, dry=False)
    check("the source is NOT deleted when dst holds different bytes",
          os.path.isfile(src), how)
    check("the existing run file is not overwritten either",
          open(dst, "rb").read() == b"WRONGFIL" * 512, how)
    check("the conflict is reported and parked beside it",
          str(how).startswith("conflict:")
          and any(n.startswith("Cam1_x.conflict-")
                  for n in os.listdir(os.path.dirname(dst))),
          "%s -> %s" % (how, os.listdir(os.path.dirname(dst))))
    # identical content is still a no-op that may drop the source
    src2 = os.path.join(d, "same.ARW")
    dst2 = os.path.join(d, "run", "Cam1_y.ARW")
    with open(src2, "wb") as fh:
        fh.write(b"IDENTICAL" * 100)
    shutil.copy2(src2, dst2)
    how2 = ing.place(src2, dst2, move=True, dry=False)
    check("an identical file is 'exists' and the source may go",
          how2 == "exists" and not os.path.exists(src2), how2)


def _g6_foreign_xmp(opts):
    sect("audit G6: a foreign <base>.xmp (Lightroom, Capture One) is never "
         "clobbered by the post-drain ingest")
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "260823_0000_xmp", 4)
    card = os.path.join(d, "cam1")
    # card JPEGs too, so this case reproduces on the pre-fix code as well (the
    # clobber is independent of whether the ARW alone could be timed)
    _write_card(card, rows[1], "_CA", -172769.57, numbers=False, jpg=True)
    base = os.path.splitext(rows[1][0]["file"])[0]
    foreign = os.path.join(root, "cam1", base + ".xmp")
    body = ('<x:xmpmeta xmlns:x="adobe:ns:meta/" x:xmptk="Adobe XMP Core">'
            '<crs:Exposure2012>+1.25</crs:Exposure2012></x:xmpmeta>')
    with open(foreign, "w") as fh:
        fh.write(body)
    ing.ingest(card, runs_dir=runs, log=lambda *a: None, offsets=[])
    check("the operator's develop settings are still there",
          open(foreign).read() == body, open(foreign).read()[:60])
    side = os.path.join(root, "cam1", base + ".wildsync.xmp")
    check("the rig's packet went to <base>.wildsync.xmp instead",
          os.path.isfile(side) and "wildsync ingest" in open(side).read())
    other = os.path.splitext(rows[1][1]["file"])[0]
    check("a frame with no foreign sidecar still gets the plain <base>.xmp",
          os.path.isfile(os.path.join(root, "cam1", other + ".xmp")))
    ing.ingest(card, runs_dir=runs, log=lambda *a: None, offsets=[])
    check("re-ingesting rewrites our own packet, never the foreign one",
          open(foreign).read() == body)


def _g6_stereo_missing_jpg(opts):
    sect("audit G6: stereo_check must skip a pair whose .card.JPG is missing "
         "instead of tracebacking on the first Image.open")
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "260823_0000_stereo", 3)
    # nothing is ingested: no <base>.card.JPG exists at all
    p = subprocess.run([sys.executable, os.path.join(RIG, "stereo_check.py"),
                        root, "--every", "1"],
                       capture_output=True, text=True, timeout=180)
    check("the tool exits cleanly with every card JPEG missing",
          p.returncode == 0, "rc=%d %s" % (p.returncode, p.stderr.strip()[-200:]))
    check("it says how many pairs it skipped",
          "skipped 3 selected pairs" in p.stdout, p.stdout.strip()[-200:])
    check("and reports no usable estimates rather than a traceback",
          "no usable estimates" in p.stdout and "Traceback" not in p.stderr,
          p.stdout.strip()[-120:])


# ---------------------------------------------------------------------------
# G4 — the CLI hands a drain to a running rigd
# ---------------------------------------------------------------------------
class _RigdStub(http.server.BaseHTTPRequestHandler):
    posts = []
    polls = [0]

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path == "/api/diag":
            self._json({"ok": True, "pid": 1})
        elif self.path == "/api/drain":
            self.polls[0] += 1
            active = self.polls[0] < 3
            self._json({"active": active, "node": "cam1" if active else None,
                        "last": {"node": "cam1", "pulled": 7, "bytes": 700,
                                 "deleted": 7, "errors": 0}})
        else:
            self._json({"ok": False}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        self.posts.append((self.path, json.loads(self.rfile.read(n) or b"{}")))
        self._json({"ok": True, "draining": ["cam1"]})


def _g4_cli_routes_through_rigd(opts):
    sect("audit G4: a standalone drain while rigd is up must go through the "
         "daemon, whose stuck-transfer recovery would otherwise kill it")
    alive = getattr(draindrv, "_rigd_alive", None)
    via = getattr(draindrv, "_drain_via_rigd", None)
    if not (alive and via):
        check("drain.py exposes _rigd_alive/_drain_via_rigd", False,
              "the CLI still drives the body directly")
        return
    _RigdStub.posts, _RigdStub.polls = [], [0]
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _RigdStub)
    base = "http://127.0.0.1:%d" % srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        check("a running rigd is detected on its own base URL",
              alive(base, timeout=3) is True)
        rep = via("cam1", keep=False, base=base, log=lambda *a: None, poll_s=0.01)
        check("the drain is handed to POST /api/drain for that node",
              _RigdStub.posts and _RigdStub.posts[0][0] == "/api/drain"
              and _RigdStub.posts[0][1].get("nodes") == ["cam1"],
              json.dumps(_RigdStub.posts[:1]))
        check("the CLI tails GET /api/drain until it goes inactive",
              rep.get("ok") is True and rep.get("active") is False
              and _RigdStub.polls[0] >= 3, json.dumps(rep))
    finally:
        srv.shutdown()
        srv.server_close()
    dead = "http://127.0.0.1:%d" % _closed_port()
    check("no daemon on the port means no daemon (falls back to direct)",
          alive(dead, timeout=1) is False)
    import inspect
    src = inspect.getsource(draindrv.Drainer.run) + inspect.getsource(draindrv.drain_node)
    check("the probe lives ONLY on the CLI path, never in the library",
          "_rigd_alive" not in src and "_drain_via_rigd" not in src)
    check("the module docstring tells the operator what happens",
          "9090" in draindrv.__doc__ and "standalone" in draindrv.__doc__)


def _closed_port():
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# ---------------------------------------------------------------------------
def _run(fn, opts):
    """One broken area must not hide the other eleven: a missing attribute or
    signature (exactly what a half-applied fix looks like) is reported as a
    failed check, not as an exception that ends the suite."""
    try:
        fn(opts)
    except Exception as e:                              # noqa: BLE001
        import traceback
        traceback.print_exc()
        check("%s ran to completion" % fn.__name__, False, repr(e))


def suite(opts):
    try:
        for fn in (_g1_whole_content, _g2_duplicate_names, _g7_stop_event,
                   _g1_arw_exif, _g1_raw_only_ingest, _g3_index_jsonl,
                   _g5_attribution, _g6_iso_carry, _g6_place_move,
                   _g6_foreign_xmp, _g6_stereo_missing_jpg,
                   _g4_cli_routes_through_rigd):
            _run(fn, opts)
    finally:
        _cleanup()


if __name__ == "__main__":
    import argparse
    import soaktest
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    o = ap.parse_args()
    t0 = time.time()
    suite(o)
    print("\naudit_drain: %d passed, %d failed in %.0f s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - t0))
    sys.exit(1 if soaktest.FAIL else 0)

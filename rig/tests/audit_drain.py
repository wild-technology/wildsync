#!/usr/bin/env python3
"""Audit regression suite — drain / ingest / stereo_check (G1..G14).

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

G8  The host write was one bare os.write whose return count nobody read: a
    short write (volume full, quota, NFS/USB) blessed a truncated file and the
    card original was then deleted. The write now loops to the full length and
    the landed size is checked before the content is staged.

G9  The XMP pair_spread_ms was keyed by round(epoch): at 2 Hz a lost frame let
    the neighbouring shot share the bucket (about 500 ms printed), and a shot
    whose exposures straddled an x.5 boundary split in two. Frames now pair
    with the nearest frame of the other camera inside half a shot period.

G10 place(--move) deleted the staged original after a cross-volume copy it
    never compared; a copy that is not byte-identical now keeps the source.

G11 `drain.py` and `ingest.py` exit 1 when the drain had errors or the ingest
    was refused / conflicted / hash-mismatched / matched nothing.

G12 Time matching tied the true clock offset with the one a shot period away
    (a missing head frame, an extra pre-run card) and dict order picked the
    shifted one, renaming every RAW one frame off. The journal offset now
    arbitrates; without it a tie is refused.

G13 A lone card series was attributed to cam1 whichever body it came from; it
    is now taken only when the staging dir, the journal, or elimination (the
    other camera already resolved its own series) says whose it is.

G14 The XMP dates carried the raw host epoch while the filename carries the
    GPS-corrected time; they now agree (wildsync:capture_epoch stays host).

Hermetic: stub cameras and a loopback HTTP stub only — the live rigd on
localhost:9090 and the fleet on 192.168.1.x are never contacted (every rigd
call in here takes an explicit 127.0.0.1 base). Temp dirs only.

Run standalone:  python3 rig/tests/audit_drain.py
"""

import errno
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
# G8 — a short host write must not bless a truncated file
# ---------------------------------------------------------------------------
class _ShortWrites:
    """Patch os.write (and os.open, to learn which fd is a .part-c staging
    file) so only the drain's host write is disturbed. mode:
      "enospc"  - first call writes a third, any further call raises ENOSPC
      "short1"  - first call writes a third, later calls behave normally
      "lie"     - writes a third but reports the full count (a layer that
                  swallows the shortfall; only the size check can see it)"""

    def __init__(self, mode):
        self.mode, self.fds, self.calls = mode, set(), 0
        self._write, self._open = os.write, os.open

    def __enter__(self):
        def op(path, *a, **k):
            fd = self._open(path, *a, **k)
            if ".part-c" in str(path):
                self.fds.add(fd)
            return fd

        def wr(fd, data):
            if fd not in self.fds:
                return self._write(fd, data)
            self.calls += 1
            if self.calls == 1 or self.mode == "lie":
                n = self._write(fd, bytes(data)[:len(data) // 3])
                return len(data) if self.mode == "lie" else n
            if self.mode == "enospc":
                raise OSError(errno.ENOSPC, "No space left on device")
            return self._write(fd, data)
        os.open, os.write = op, wr
        return self

    def __exit__(self, *exc):
        os.open, os.write = self._open, self._write


def _g8_short_write(opts):
    sect("audit G8: a short os.write is not an exception - the drain must "
         "check the count, because the card is the only other copy")
    big = b"ARWDATA!" * 37500                      # 300000 B
    for mode in ("enospc", "lie"):
        d = _tmp()
        s = StubCam(d, {60: [("W.ARW", big, RAW)]}, node="cam1")
        with _ShortWrites(mode):
            rep = s.run()
        dest = os.path.join(d, "cam1")
        files = _host_files(dest)
        sums = os.path.join(d, "SHA256SUMS")
        recorded = open(sums).read() if os.path.isfile(sums) else ""
        check("%s: the failed host write is reported" % mode,
              any("host write" in e for e in rep["errors"]),
              json.dumps(rep["errors"]))
        check("%s: the card original is NOT deleted" % mode,
              s.deleted == [] and rep["deleted"] == 0 and 60 in s.card,
              "deleted=%s" % s.deleted)
        check("%s: no truncated final file and no .part is left" % mode,
              files == [], str(files))
        check("%s: nothing is recorded in SHA256SUMS" % mode,
              "W.ARW" not in recorded, recorded[:80])
    # a short count that the loop can finish is not an error
    d = _tmp()
    s = StubCam(d, {61: [("V.ARW", big, RAW)]}, node="cam1")
    with _ShortWrites("short1"):
        rep = s.run()
    dest = os.path.join(d, "cam1")
    got = open(os.path.join(dest, "V.ARW"), "rb").read() \
        if os.path.isfile(os.path.join(dest, "V.ARW")) else b""
    check("a single short write is completed by the loop: full-length file",
          got == big, "%d of %d B" % (len(got), len(big)))
    vs = draindrv.verify_sums(d)
    check("the recorded hash verifies and the card is then deleted",
          rep["errors"] == [] and s.deleted == [61] and rep["deleted"] == 1
          and vs["ok"] == 1 and not vs["failed"] and not vs["missing"],
          json.dumps({"errors": rep["errors"], "verify": vs}))


# ---------------------------------------------------------------------------
# G9 — pair_spread_ms pairs by nearest frame, not by integer second
# ---------------------------------------------------------------------------
def _spread_of(xmp_path):
    """<wildsync:pair_spread_ms> from a sidecar: float, or None when the
    field is absent (xmp_sidecar omits an empty value)."""
    m = re.search(r"<wildsync:pair_spread_ms>([^<]*)<", open(xmp_path).read())
    return float(m.group(1)) if m and m.group(1) != "" else None


def _ingest_pair(d, runs, rows, drop=None):
    """Ingest both cameras of a _write_run run from per-node staging dirs;
    rows[cam] minus `drop` (cam, k) is what lands on that card."""
    for cam in (1, 2):
        rs = [r for k, r in enumerate(rows[cam]) if drop != (cam, k)]
        _write_card(os.path.join(d, "cam%d" % cam), rs,
                    "_CA" if cam == 1 else "DSC", -172769.57 + cam,
                    numbers=False)
        ing.ingest(os.path.join(d, "cam%d" % cam), runs_dir=runs,
                   log=lambda *a: None, offsets=[])


def _drop_row(root, cam, k):
    """Remove one camera's frame from the run's index, as a lost frame would."""
    idx = ing.read_frame_index(root)
    gone = [r for r in idx if r["cam"] == cam][k]
    keep = [r for r in idx if r is not gone]
    with open(os.path.join(root, "index.jsonl"), "w") as fh:
        for r in keep:
            fh.write(json.dumps(r) + "\n")
    with open(os.path.join(root, "run.json")) as fh:
        doc = json.load(fh)
    doc["index"], doc["frames"] = keep, len(keep)
    with open(os.path.join(root, "run.json"), "w") as fh:
        json.dump(doc, fh)


def _g9_pair_spread(opts):
    sect("audit G9: pair_spread_ms pairs a frame with the NEAREST frame of the "
         "other camera, not whatever shares round(epoch)")
    # 2 Hz, cam2 fires 1 ms after cam1, phase chosen so every other shot's
    # exposures fall either side of an x.5 rounding boundary
    t0 = 1787253940.4995
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "261015_0000_spread", 8, t0=t0, interval=0.5)
    _ingest_pair(d, runs, rows)
    sp = [(_spread_of(os.path.join(root, "cam%d" % cam,
                                   os.path.splitext(r["file"])[0] + ".xmp")))
          for cam in (1, 2) for r in rows[cam]]
    check("a 2 Hz pair whose exposures straddle an x.5 boundary reads 1.00 ms "
          "on every frame (the second-bucket printed ~499 ms)",
          len(sp) == 16 and all(v is not None and abs(v - 1.0) < 0.05
                                for v in sp), str(sp))

    # a frame lost from one camera: its shot is a single, and the NEIGHBOURING
    # shots must not borrow its epoch
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "261015_0000_spreadlost", 8, t0=t0,
                            interval=0.5)
    _drop_row(root, 2, 4)
    kept = {1: rows[1], 2: [r for k, r in enumerate(rows[2]) if k != 4]}
    _ingest_pair(d, runs, rows, drop=(2, 4))
    got = {}
    for cam in (1, 2):
        for r in kept[cam]:
            got[(cam, r["orig"])] = _spread_of(os.path.join(
                root, "cam%d" % cam, os.path.splitext(r["file"])[0] + ".xmp"))
    lone = got[(1, rows[1][4]["orig"])]
    others = [v for (c, o), v in got.items() if o != rows[1][4]["orig"]]
    check("the shot cam2 lost is a single: blank spread, not a cross-shot figure",
          lone is None, str(lone))
    check("every other shot still reads 1.00 ms (the bucket printed ~500 ms "
          "for the neighbours)",
          len(others) == 14 and all(v is not None and abs(v - 1.0) < 0.05
                                    for v in others), str(others))

    # the default 2 s interval: the straddling shot used to land in two buckets
    # and print BLANK
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "261015_0000_spread2s", 5, t0=t0, step=2.0,
                            interval=2.0)
    _ingest_pair(d, runs, rows)
    sp = [_spread_of(os.path.join(root, "cam%d" % cam,
                                  os.path.splitext(r["file"])[0] + ".xmp"))
          for cam in (1, 2) for r in rows[cam]]
    check("at interval 2 s a straddling shot reads 1.00 ms instead of blank",
          all(v is not None and abs(v - 1.0) < 0.05 for v in sp), str(sp))


# ---------------------------------------------------------------------------
# G10 — --move keeps the source unless the copy is byte-identical
# ---------------------------------------------------------------------------
def _g10_move_verified(opts):
    sect("audit G10: place(--move) over a cross-volume copy must hash the copy "
         "before it unlinks the source")
    d = _tmp()
    src = os.path.join(d, "card.ARW")
    dst = os.path.join(d, "run", "Cam1_z.ARW")
    good = b"CORRECT-" * 512
    with open(src, "wb") as fh:
        fh.write(good)
    real_link, real_copy = ing.os.link, ing.shutil.copy2

    def no_link(*a, **k):
        raise OSError(errno.EXDEV, "cross-device")

    def bad_copy(s_, d_, *a, **k):
        with open(d_, "wb") as fh:
            fh.write(good[:len(good) // 2])       # a truncated copy
        return d_
    ing.os.link, ing.shutil.copy2 = no_link, bad_copy
    try:
        try:
            how = ing.place(src, dst, move=True, dry=False)
            err = None
        except OSError as e:
            how, err = None, e
        check("a corrupt cross-volume copy under --move raises",
              err is not None, str(how))
        check("the staged original is NOT deleted",
              os.path.isfile(src) and open(src, "rb").read() == good)
        check("nothing lands under the run name and no .part is left",
              not os.path.exists(dst) and not os.path.exists(dst + ".part"),
              str(os.listdir(os.path.dirname(dst))))
        # without --move the source is not at risk: behaviour is unchanged
        try:
            how2 = ing.place(src, dst, move=False, dry=False)
        except OSError as e:
            how2 = "raised %s" % e
        check("without --move a short copy is not second-guessed (source kept)",
              how2 == "copied" and os.path.isfile(src), str(how2))
        os.unlink(dst)
        ing.shutil.copy2 = real_copy
        how3 = ing.place(src, dst, move=True, dry=False)
        check("a faithful copy under --move is placed and the source removed",
              how3 == "moved" and not os.path.exists(src)
              and open(dst, "rb").read() == good, str(how3))
    finally:
        ing.os.link, ing.shutil.copy2 = real_link, real_copy


# ---------------------------------------------------------------------------
# G11 — drain CLI exit status
# ---------------------------------------------------------------------------
def _run_drain_main(argv):
    """drain.main() under argv; returns its exit code (0 when it returns)."""
    import contextlib
    import io
    old = sys.argv
    sys.argv = ["drain.py"] + argv
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                draindrv.main()
            except SystemExit as e:
                return 0 if e.code in (None, 0) else e.code
        return 0
    finally:
        sys.argv = old


def _g11_drain_exit_status(opts):
    sect("audit G11: the drain CLI must say failure with its exit status "
         "(the operator chains drain && ingest)")
    saved = (draindrv.drain_node, draindrv._rigd_alive, draindrv._drain_via_rigd)

    def rep(errors):
        return {"pulled": 3, "verified": 3, "deleted": 3, "skipped": 0,
                "files": ["a", "b"], "bytes": 10, "errors": errors,
                "cancelled": False}
    try:
        draindrv._rigd_alive = lambda *a, **k: False
        draindrv.drain_node = lambda *a, **k: rep(["X.ARW HASH MISMATCH"])
        check("a standalone drain with per-shot errors exits 1",
              _run_drain_main(["cam1", "--host", "127.0.0.1"]) == 1)
        draindrv.drain_node = lambda *a, **k: rep([])
        check("a clean standalone drain exits 0",
              _run_drain_main(["cam1", "--host", "127.0.0.1"]) == 0)
        draindrv._rigd_alive = lambda *a, **k: True
        draindrv._drain_via_rigd = lambda *a, **k: {
            "ok": True, "active": False,
            "last": {"node": "cam1", "pulled": 3, "deleted": 1, "errors": 2}}
        check("a rigd-routed drain whose last report has errors exits 1",
              _run_drain_main(["cam1"]) == 1)
        draindrv._drain_via_rigd = lambda *a, **k: {
            "ok": True, "active": False,
            "last": {"node": "cam1", "pulled": 3, "deleted": 3, "errors": 0}}
        check("a clean rigd-routed drain exits 0", _run_drain_main(["cam1"]) == 0)
        draindrv._drain_via_rigd = lambda *a, **k: {"ok": False,
                                                    "error": "lost rigd"}
        check("a lost rigd still exits 1", _run_drain_main(["cam1"]) == 1)
    finally:
        draindrv.drain_node, draindrv._rigd_alive, draindrv._drain_via_rigd = saved


# ---------------------------------------------------------------------------
# G12 — a tie between alignments is arbitrated by the journal, else refused
# ---------------------------------------------------------------------------
_T0, _OFF12 = 1787253940.0, -172769.57


def _g12_build(n=20, drop=(), extra_before=0, jit=0.0, seed=0):
    import random
    rnd = random.Random(seed)
    rows = [{"orig": "ILX%05d.JPG" % (100 + k), "epoch": _T0 + k * 0.5}
            for k in range(n)]
    cards = [{"stem": "_CA%05d" % (30000 + j), "series": "_CA",
              "number": 30000 + j, "arw": "x", "jpg": None,
              "exif": _T0 + j * 0.5 + _OFF12 + rnd.gauss(0, jit)}
             for j in range(-extra_before, n) if j not in drop]
    return rows, cards


def _g12_match(expect, **kw):
    rows, cards = _g12_build(**kw)
    s_, off, how, m, u = ing.match_run(rows, cards, set(), expect=expect,
                                       tol_s=0.225, authoritative=True)
    wrong = sum(1 for r, c, _res in m
                if int(c["stem"][3:]) - 30000 != int(r["orig"][3:8]) - 100)
    return s_, how, len(m), wrong


def _g12_offset_ties(opts):
    sect("audit G12: a missing head frame or an extra pre-run card must not "
         "shift the whole run by one frame")
    for name, kw, want in (("first card missing", dict(drop=(0,)), 19),
                           ("first two missing", dict(drop=(0, 1)), 18),
                           ("one extra card a period before row 0",
                            dict(extra_before=1), 20),
                           ("last card missing", dict(drop=(19,)), 19),
                           ("middle card missing", dict(drop=(7,)), 19),
                           ("no hole", {}, 20)):
        s_, how, nm, wrong = _g12_match(_OFF12, **kw)
        check("%s: journal offset picks the true alignment (%d matched, 0 wrong)"
              % (name, want), nm == want and wrong == 0,
              "matched %d wrong %d" % (nm, wrong))
    bad = 0
    for seed in range(50):
        for kw in (dict(drop=(0,)), {}):
            bad += _g12_match(_OFF12, jit=0.03, seed=seed, **kw)[3]
    check("30 ms of jitter over 50 seeds: no wrong assignment", bad == 0,
          "%d wrong" % bad)
    s_, how, nm, wrong = _g12_match(None, drop=(0,))
    check("a missing head frame with NO journal offset is refused, not shifted",
          s_ is None and str(how).startswith("ambiguous") and nm == 0,
          "%s %s %d" % (s_, how, nm))
    for name, kw in (("no hole", {}), ("a middle hole", dict(drop=(7,)))):
        s_, how, nm, wrong = _g12_match(None, **kw)
        check("%s with no journal offset still matches (no false refusal)"
              % name, s_ == "_CA" and wrong == 0 and nm >= 19,
              "%s %s %d wrong %d" % (s_, how, nm, wrong))

    # end to end: card dir missing the first frame, journal offset present
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "261015_0000_head", 8, interval=0.5)
    _write_card(os.path.join(d, "cam1"), rows[1][1:], "_CA", _OFF12,
                numbers=False)
    # card stems must still carry the ORIGINAL frame's number, so write them
    # for rows 1.. but with a counter that is not the PC-save one (done above)
    started = rows[1][0]["epoch"]
    ing.ingest(os.path.join(d, "cam1"), runs_dir=runs, log=lambda *a: None,
               offsets=[(started, "cam1", _OFF12)])
    okp, bad = 0, 0
    for k, r in enumerate(rows[1]):
        arw = os.path.join(root, "cam1", os.path.splitext(r["file"])[0] + ".ARW")
        if k == 0:
            bad += os.path.exists(arw)
        elif os.path.isfile(arw):
            ex = ing._arw_exif_epoch(arw)
            okp += ex is not None and abs(ex - _OFF12 - r["epoch"]) < 0.02
    check("through ingest(): rows 1..7 got their own RAW, row 0 stays unmatched",
          okp == 7 and bad == 0, "right %d, RAW on the lost row %d" % (okp, bad))


# ---------------------------------------------------------------------------
# G13 — a lone series is not attributed to whichever camera asks first
# ---------------------------------------------------------------------------
def _g13_lone_series(opts):
    sect("audit G13: one card series in a non-camN dir must not be handed to "
         "cam1 when nothing says it is cam1's")
    d = _tmp()
    runs = os.path.join(d, "runs")
    root, rows = _write_run(runs, "261015_0000_lone", 6)
    lone = os.path.join(d, "download")
    _write_card(lone, rows[2], "DSC", -172748.18, numbers=False)
    out = ing.ingest(lone, runs_dir=runs, log=lambda *a: None, offsets=[])
    sm = out["runs"][0]["summary"]
    check("both cameras are refused as ambiguous (cam1 used to claim all six)",
          all(str(sm[c]["how"]).startswith("ambiguous") for c in (1, 2)),
          json.dumps({c: sm[c]["how"] for c in (1, 2)}))
    check("nothing was placed and the totals say so",
          out["totals"]["raw"] == 0 and out["totals"]["ambiguous"]
          and not glob_any(os.path.join(root, "cam1"), ".ARW"),
          json.dumps(out["totals"]))

    started = rows[1][0]["epoch"]
    offs = [(started, "cam2", -172748.18), (started, "cam1", -172769.57)]
    out2 = ing.ingest(lone, runs_dir=runs, log=lambda *a: None, offsets=offs)
    s2 = out2["runs"][0]["summary"]
    check("the journal offset attributes the lone series to cam2",
          s2[2]["matched"] == 6 and s2[2]["series"] == "DSC", json.dumps(s2[2]))
    check("cam1 is still refused and receives no files",
          s2[1]["matched"] == 0 and not glob_any(os.path.join(root, "cam1"),
                                                 ".ARW"), json.dumps(s2[1]))

    # elimination still works: cam2 by number, cam1 takes the series left over
    d3 = _tmp()
    runs3 = os.path.join(d3, "runs")
    root3, rows3 = _write_run(runs3, "261015_0000_elim", 6)
    both = os.path.join(d3, "download")
    # cam2's PC-save counter (ILX09000+) is what its card numbering follows;
    # cam1's is not, so only cam2 resolves by number
    idx = ing.read_frame_index(root3)
    for r in idx:
        if r["cam"] == 2:
            r["orig"] = "ILX%05d.JPG" % (9000 + int(r["orig"][3:8]) - 1000)
    for fn in ("index.jsonl",):
        with open(os.path.join(root3, fn), "w") as fh:
            for r in idx:
                fh.write(json.dumps(r) + "\n")
    os.makedirs(both)
    for r in (x for x in idx if x["cam"] == 2):
        with open(os.path.join(both, "DSC%05d.ARW" % int(r["orig"][3:8])),
                  "wb") as fh:
            fh.write(make_arw(r["epoch"] - 172748.18))
    _write_card(both, rows3[1], "_CA", -172769.57, numbers=False)
    out3 = ing.ingest(both, runs_dir=runs3, log=lambda *a: None, offsets=[])
    s3 = out3["runs"][0]["summary"]
    check("cam2 matches by number and cam1 takes the remaining series by time",
          s3[2]["how"] == "number" and s3[2]["matched"] == 6
          and s3[1]["how"] == "time" and s3[1]["matched"] == 6
          and s3[1]["series"] == "_CA",
          json.dumps({c: (s3[c]["how"], s3[c]["matched"]) for c in (1, 2)}))

    rws, cds = _g12_build(n=6)
    a = ing._offset_by_time(rws, cds, None, False, False)
    b = ing._offset_by_time(rws, cds, None, False, True)
    c = ing._offset_by_time(rws, cds, None, True, False)
    check("_offset_by_time: a lone series needs authority or elimination",
          str(a[2]).startswith("ambiguous") and a[0] is None
          and b[0] == "_CA" and c[0] == "_CA", "%s | %s | %s" % (a, b, c))


def glob_any(folder, ext):
    return os.path.isdir(folder) and any(n.endswith(ext)
                                         for n in os.listdir(folder))


# ---------------------------------------------------------------------------
# G14 — the XMP dates are in the same timebase as the filename
# ---------------------------------------------------------------------------
def _g14_xmp_time(opts):
    sect("audit G14: XMP DateTimeOriginal must carry the GPS-corrected time "
         "the filename carries, not the raw host epoch")
    off = 259200.0 + 12.34                         # a host with no usable RTC
    for label, gps_off in (("gps run", off), ("host-time run", 0.0),
                           ("legacy run with no time block", None)):
        d = _tmp()
        runs = os.path.join(d, "runs")
        root, rows = _write_run(runs, "261015_0000_xmp", 3, gps_off=gps_off)
        _write_card(os.path.join(d, "cam1"), rows[1], "_CA", -172769.57,
                    numbers=False)
        ing.ingest(os.path.join(d, "cam1"), runs_dir=runs,
                   log=lambda *a: None, offsets=[])
        r = rows[1][1]
        base = os.path.splitext(r["file"])[0]
        x = open(os.path.join(root, "cam1", base + ".xmp")).read()
        want = ing._iso(r["epoch"] + (gps_off or 0.0))
        got = {k: re.search(r"<%s>([^<]*)<" % k, x).group(1)
               for k in ("exif:DateTimeOriginal", "xmp:CreateDate",
                         "photoshop:DateCreated")}
        check("%s: all three XMP dates are the presentation time" % label,
              all(v == want for v in got.values()), "%s vs %s" % (got, want))
        m = re.match(r"Cam1_(\d{8})_(\d{6})\.(\d\d)$", base)
        stem_t = time.mktime(time.strptime(m.group(1) + m.group(2),
                                           "%Y%m%d%H%M%S")) - time.timezone
        xt = time.mktime(time.strptime(want[:19], "%Y-%m-%dT%H:%M:%S")) \
            - time.timezone + float(want[19:-1])
        check("%s: the XMP time agrees with the filename to 10 ms" % label,
              abs(xt - (stem_t + int(m.group(3)) / 100.0)) < 0.011,
              "%s vs %s" % (want, base))
        check("%s: capture_epoch is still the raw host epoch" % label,
              ("<wildsync:capture_epoch>%.6f<" % r["epoch"]) in x)
        check("%s: gps_offset_s is recorded only when a correction was applied"
              % label, (("<wildsync:gps_offset_s>" in x) == bool(gps_off)))


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
               jsonl=True, truncate_json=None, interval=None, gps_off=None,
               skew2=0.001):
    """A run root with index.jsonl (full) and run.json (tail-truncated the way
    run.py does). Returns (root, {cam: [rows]})."""
    root = os.path.join(runs, rid)
    for cam in cams:
        os.makedirs(os.path.join(root, "cam%d" % cam), exist_ok=True)
    idx, rows = [], {c: [] for c in cams}
    for k in range(per_cam):
        ep = t0 + k * step
        for cam in cams:
            e = ep + (skew2 if cam == 2 else 0.0)
            # the filename is in the presentation timebase (host + GPS offset)
            ef = e + (gps_off or 0.0)
            r = {"cam": cam,
                 "file": "Cam%d_%s.%02d.jpg" % (cam,
                                                time.strftime("%Y%m%d_%H%M%S",
                                                              time.gmtime(ef)),
                                                int((ef % 1) * 100)),
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
        doc = {"run_id": rid, "frames": len(idx), "index": keep,
               "final": True, "started": t0}
        if interval is not None:
            doc["config"] = {"interval_s": interval}
        if gps_off is not None:
            doc["time"] = {"source": "gps" if gps_off else "host",
                           "gps_offset_s": gps_off}
        json.dump(doc, fh)
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
    try:
        import cv2  # noqa: F401 - stereo_check imports it at the top
    except ImportError as e:
        note("stereo_check needs OpenCV (%s) - G6 skipped on this host; "
             "deploy.sh jetson installs python3-opencv on the rig" % e)
        return
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
        for fn in (_g1_whole_content, _g2_duplicate_names, _g7_stop_event, _g8_short_write,
                   _g1_arw_exif, _g1_raw_only_ingest, _g3_index_jsonl,
                   _g5_attribution, _g6_iso_carry, _g6_place_move,
                   _g6_foreign_xmp, _g6_stereo_missing_jpg,
                   _g4_cli_routes_through_rigd, _g9_pair_spread,
                   _g10_move_verified, _g11_drain_exit_status,
                   _g12_offset_ties, _g13_lone_series, _g14_xmp_time):
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

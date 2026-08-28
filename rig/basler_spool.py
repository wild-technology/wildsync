#!/usr/bin/env python3
"""Basler a2A4504 -> JPEG -> spool. Phase 5 rehearsal for baslerctl's save path.

    python3 rig/basler_spool.py --seconds 120 --fps 2 --quality 90
    python3 rig/basler_spool.py --seconds 1800 --spool ~/Pictures/Basler

Runs ON cam3's Pi, inside ~/pylon-venv (pypylon + opencv). This is deliberately
NOT a service yet: it rehearses exactly what baslerctl will do in production
(docs/BASLER-SETUP.md Phase 5, step 13) so the numbers that size the rig are
measured rather than assumed.

Three things here are contracts with the rest of the rig, not preferences:

* NAMING - CamN_YYYYMMDD_hhmmss.ss.jpg, identical to run.py's _fmt_fname().
  ingest.py keys attribution off `cam%d` and the stem, so a divergent name is
  not a cosmetic difference: it silently unpairs frames.

* ATOMIC WRITE - .part -> fsync -> rename -> fsync(dir), identical to run.py's
  pull path. Two findings force it (audit 2026-08-27): a buffered write left
  bytes in the page cache while the source was deleted seconds later, and a
  failed write left a TRUNCATED file under the canonical name. A .part name can
  never be mistaken for a frame, and the fsyncs make "on disk" true.

* MONO8 -> JPEG BEFORE THE CARD - a raw frame is 20.3 MB. At 2 fps that is
  41 MB/s onto the SD card. Measured here: q90 is ~1.6 MB, 12.9x.

MEASURED ON CAM3, 2026-08-28 (4504x4504 Mono8, real scene, median of 20):
    cv2.imencode  q85 57.8 ms 1.073 MB | q90 62.0 ms 1.572 MB | q95 71.3 ms 2.887 MB
    PyTurboJPEG   q85 66.7 ms 1.078 MB | q90 69.6 ms 1.588 MB | q95 79.8 ms 3.000 MB
cv2 wins on speed at the same size - OpenCV 5.0 bundles a newer libjpeg-turbo
than the system libturbojpeg 2.x that PyTurboJPEG binds. It also replaces the
150-400 ms envelope BASLER-SETUP.md 5.2 had to guess at: ~12% of one core at
2 fps, so the encode pool has margin rather than being the bottleneck.

Pure-noise ceiling (q90, 16.0 MB, 1.3x) is the honest worst case: no real photo
compresses worse, so spool math between 1.6 and 16 MB/frame is bounded.
"""

import argparse
import io
import os
import queue
import sys
import threading
import time

try:
    import cv2
    import numpy as np
    from pypylon import pylon
except ImportError as e:  # pragma: no cover - node-only deps
    sys.exit("missing dep (%s). Run inside ~/pylon-venv on the node." % e)

try:
    import piexif
except ImportError:
    piexif = None


def _split_epoch(epoch):
    """(struct_time, '.ss') with the centisecond carry applied to BOTH halves.

    COPIED VERBATIM from run.py:_split_epoch and it must stay that way. It is
    not imported because deploy.sh does not ship run.py to the nodes, and this
    module has to run standalone on cam3.

    The first version of this file diverged from run.py in two ways while its
    docstring claimed to be identical, and neither shows up in a self-check
    because the EXIF stamp is derived from the same function:
      * time.localtime() instead of gmtime() - every Basler frame was named in
        LOCAL time while the rig names in UTC, so cam3's frames would not sort
        or pair against cam1/cam2 at all;
      * int((epoch % 1) * 100) rounds the fraction independently of the
        seconds. run.py's own comment: at ...000.999 the fraction rounds to
        1.00 and is pasted on to an unrounded seconds field, stamping the frame
        a full second early. That column is what nav is correlated against.
    Round once, derive both from the result."""
    cs = int(round(epoch * 100))          # centiseconds since the epoch
    whole, frac = divmod(cs, 100)
    return time.gmtime(whole), ".%02d" % frac


def _fmt_fname(cam_num, epoch, ext=".jpg"):
    """CamN_YYYYMMDD_hhmmss.ss.jpg - byte-identical to run.py:_fmt_fname."""
    lt, frac = _split_epoch(epoch)
    return "Cam%d_%s%s%s" % (cam_num, time.strftime("%Y%m%d_%H%M%S", lt), frac, ext)


def _stamp_exif(jpeg_bytes, epoch):
    """DateTimeOriginal + SubSecTimeOriginal from the frame's own timestamp.

    cv2 writes no EXIF at all, and rigd's EXIF fallback is what stamps a frame's
    capture instant when no GPIO edge is available - an unstamped frame silently
    inherits the command time instead (README 'Pillow is easy to miss').

    piexif.insert(exif, jpeg_bytes) RAISES when handed bytes with no third
    argument ("Give a 3rd argument to 'insert' to output file") - the bytes-in
    /bytes-out form needs an io.BytesIO sink. The first Phase 5 run shipped 240
    frames with NO EXIF at all because this sat behind a bare `except: return
    jpeg_bytes`, so a hard requirement failure read as a clean PASS. Raise
    instead: the caller counts it, and a frame with no capture instant is a
    defect worth failing the run over."""
    if piexif is None:
        raise RuntimeError("piexif not installed - frames would carry no capture instant")
    lt, frac = _split_epoch(epoch)
    dt = time.strftime("%Y:%m:%d %H:%M:%S", lt)
    exif = {"0th": {piexif.ImageIFD.Make: b"Basler",
                    piexif.ImageIFD.Model: b"a2A4504-18umBAS"},
            "Exif": {piexif.ExifIFD.DateTimeOriginal: dt.encode(),
                     piexif.ExifIFD.SubSecTimeOriginal: frac.lstrip(".").encode()},
            "1st": {}, "thumbnail": None, "GPS": {}, "Interop": {}}
    sink = io.BytesIO()
    piexif.insert(piexif.dump(exif), jpeg_bytes, sink)
    return sink.getvalue()


def _write_atomic(path, data):
    """.part -> fsync -> rename -> fsync(dir). Returns write seconds."""
    t0 = time.perf_counter()
    tmp = path + ".part"
    with open(tmp, "wb") as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)
    try:
        dfd = os.open(os.path.dirname(path), os.O_RDONLY)
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
    except OSError:
        pass
    return time.perf_counter() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--fps", type=float, default=2.0)
    ap.add_argument("--quality", type=int, default=90)
    ap.add_argument("--cam-num", type=int, default=3)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--spool", default=os.path.expanduser("~/Pictures/Basler"))
    a = ap.parse_args()

    if a.workers < 1:
        ap.error("--workers must be >= 1 (0 makes the queue UNBOUNDED)")
    if a.fps <= 0:
        ap.error("--fps must be > 0")
    if piexif is None:
        sys.exit("piexif not installed - every frame would carry no capture "
                 "instant. pip install piexif (docs/PI-SETUP.md 13).")
    os.makedirs(a.spool, exist_ok=True)
    # Bounded: the queue IS the backpressure signal. Unbounded, a slow card
    # turns into RAM growth and an OOM kill instead of a countable drop - which
    # is exactly how the first benchmark run died on this node (3 OOM kills,
    # 20 frames x 20.3 MB held live).
    q = queue.Queue(maxsize=a.workers * 3)
    stats = {"encoded": 0, "written": 0, "bytes": 0, "enc_s": [], "wr_s": [],
             "qdepth": [], "drop_full": 0, "exif_fail": 0, "exif_err": "",
             "write_fail": 0, "write_err": "", "encode_fail": 0}
    lock = threading.Lock()
    stop = threading.Event()

    def worker():
        while not (stop.is_set() and q.empty()):
            try:
                arr, epoch = q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                t0 = time.perf_counter()
                ok, buf = cv2.imencode(".jpg", arr,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
                enc = time.perf_counter() - t0
                if not ok:
                    with lock:
                        stats["encode_fail"] += 1
                    continue
                try:
                    data = _stamp_exif(buf.tobytes(), epoch)
                except Exception as exc:      # countable, never silent
                    with lock:
                        stats["exif_fail"] += 1
                        stats["exif_err"] = "%s: %s" % (type(exc).__name__, exc)
                    data = buf.tobytes()
                path = os.path.join(a.spool, _fmt_fname(a.cam_num, epoch))
                try:
                    wr = _write_atomic(path, data)
                except Exception as exc:      # ENOSPC, EACCES, EIO...
                    # WITHOUT this the exception escapes the while loop and
                    # kills the worker THREAD. With --workers 2, two bad frames
                    # kill both, the queue fills, every later frame is counted
                    # as a queue drop, and the run reports a plausible-looking
                    # failure that names the wrong cause. A full SD card is the
                    # likely trigger and is exactly when you need the truth.
                    with lock:
                        stats["write_fail"] += 1
                        stats["write_err"] = "%s: %s" % (type(exc).__name__, exc)
                    continue
                with lock:
                    stats["encoded"] += 1
                    stats["written"] += 1
                    stats["bytes"] += len(data)
                    stats["enc_s"].append(enc)
                    stats["wr_s"].append(wr)
            finally:
                q.task_done()

    threads = [threading.Thread(target=worker, daemon=True) for _ in range(a.workers)]
    for t in threads:
        t.start()

    cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    cam.Open()
    cam.MaxNumBuffer.Value = 6
    print("  camera   : %s  sn=%s" % (cam.GetDeviceInfo().GetModelName(),
                                      cam.GetDeviceInfo().GetSerialNumber()))
    print("  spool    : %s" % a.spool)
    print("  plan     : %.0fs @ %.1f fps, q%d, %d workers"
          % (a.seconds, a.fps, a.quality, a.workers))

    period = 1.0 / a.fps
    grabbed = failed = 0
    cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    t_start = time.time()
    nxt = t_start
    while time.time() - t_start < a.seconds:
        now = time.time()
        if now < nxt:
            time.sleep(min(0.02, nxt - now))
            continue
        nxt += period
        if nxt < now:
            # A RetrieveResult timeout leaves nxt seconds in the past, and an
            # uncorrected `nxt += period` then grabs back-to-back with no sleep
            # until it catches up - a burst at full sensor rate, not the 2 fps
            # the run claims to be testing. Re-base instead.
            nxt = now + period
        r = cam.RetrieveResult(5000, pylon.TimeoutHandling_Return)
        if not (r and r.IsValid()):
            failed += 1
            continue
        if r.GrabSucceeded():
            grabbed += 1
            epoch = time.time()
            try:
                q.put_nowait((r.Array.copy(), epoch))
                with lock:
                    stats["qdepth"].append(q.qsize())
            except queue.Full:
                with lock:
                    stats["drop_full"] += 1
        else:
            failed += 1
        r.Release()
    cam.StopGrabbing()
    cam.Close()

    q.join()
    stop.set()
    for t in threads:
        t.join(timeout=3)

    el = time.time() - t_start
    enc = np.array(stats["enc_s"]) * 1000
    wr = np.array(stats["wr_s"]) * 1000
    qd = np.array(stats["qdepth"]) if stats["qdepth"] else np.array([0])
    print("\n  === PHASE 5 SAVE PATH ===")
    print("  elapsed        : %.1f s" % el)
    print("  grabbed        : %d  (%.2f fps)" % (grabbed, grabbed / el))
    print("  grab failures  : %d" % failed)
    print("  written        : %d" % stats["written"])
    print("  dropped (q full): %d" % stats["drop_full"])
    print("  bytes          : %.1f MB  (%.3f MB/frame, %.2f MB/s)"
          % (stats["bytes"] / 1e6,
             stats["bytes"] / 1e6 / max(stats["written"], 1),
             stats["bytes"] / 1e6 / el))
    if len(enc):
        print("  encode ms      : p50 %.1f  p95 %.1f  max %.1f"
              % (np.percentile(enc, 50), np.percentile(enc, 95), enc.max()))
        print("  write ms       : p50 %.1f  p95 %.1f  max %.1f"
              % (np.percentile(wr, 50), np.percentile(wr, 95), wr.max()))
    print("  queue depth    : p50 %.0f  max %.0f (cap %d)"
          % (np.percentile(qd, 50), qd.max(), a.workers * 3))
    print("  EXIF failures  : %d %s" % (stats["exif_fail"], stats["exif_err"]))
    print("  encode failures: %d" % stats["encode_fail"])
    print("  write failures : %d %s" % (stats["write_fail"], stats["write_err"]))
    leftover = [f for f in os.listdir(a.spool) if f.endswith(".part")]
    print("  stray .part    : %d  %s" % (len(leftover), "OK" if not leftover else leftover[:3]))
    ok = (failed == 0 and stats["drop_full"] == 0 and not leftover
          and stats["written"] == grabbed and stats["exif_fail"] == 0
          and stats["write_fail"] == 0 and stats["encode_fail"] == 0)
    print("  VERDICT        : %s" % ("PASS" if ok else "SEE ABOVE"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Wild Sync raw NMEA/PDGY logger — run-scoped, verbatim, dual-clock.

PROTOCOL.md puts `nmea_raw.log` at the root of every run directory and calls it
"every serial line, prefixed epoch".  This module is that file's writer, and
its reader.

Why two clocks on every line
----------------------------
`time.time()` (wall) is what everything else in the rig is stamped with, but it
is not monotonic: chrony steps it, and — the whole point of nav.py's
TimeAuthority — a GPS fix arriving mid-run can shift it.  A raw log whose only
clock can jump backwards is useless for measuring inter-message intervals after
the fact.  `time.monotonic()` cannot jump but has no meaning across processes.
So every record carries both, and the pair also lets you *detect* a clock step
in post: if wall advances by 30 s while monotonic advances by 0.2 s, the
timebase moved, not the boat.

File format
-----------
Lines beginning with `#` are metadata written by this module.  Every other line
is one line received from the gateway, held verbatim:

    <wall_epoch:.6f> <monotonic:.6f> <line as received>

The gateway line is written exactly as it arrived, minus the CR/LF that
terminated it.  Bytes that are not ASCII are backslash-escaped by nav.py's
decoder (`errors="backslashreplace"`), so nothing is silently dropped and the
file stays greppable and line-oriented.

Usage
-----
    import navlog, nav
    log = navlog.open_run_log("/home/wildtech/rig-runs/260816_0100_transect-01")
    reader = nav.NavReader()
    reader.set_raw_hook(log)          # duck-typed: uses log.write_line()
    ...
    reader.set_raw_hook(None); log.close()

The object is also callable as `log(wall_epoch, line)` so it can be dropped in
wherever the older two-argument hook shape is expected; in that case the
monotonic column is sampled at write time instead of at receive time (a
sub-millisecond difference, and the record says so via a `~` marker).

Replay
------
    navlog.replay("run/nmea_raw.log", reader)

pushes a recorded log back through a NavReader, which is how the decode path
gets exercised against real bus traffic long after the boat is back on the
trailer — and how a nav bug found in the field can be reproduced on a desk.

Stdlib only.  No HTTP.  Nothing here ever raises into the serial thread.
"""

import io
import os
import sys
import threading
import time

__all__ = ["RawNmeaLog", "open_run_log", "parse_record", "read_records",
           "replay", "LOG_NAME"]

LOG_NAME = "nmea_raw.log"

_HEADER = (
    "# wildsync nmea_raw.log v1\n"
    "# format: <wall_epoch> <monotonic> <line received from gateway, verbatim>\n"
    "# lines starting with '#' are logger metadata, not gateway traffic\n"
)


class RawNmeaLog:
    """Append-only, thread-safe, dual-clock raw line log for one run.

    Designed to be handed straight to `nav.NavReader.set_raw_hook()`.  Every
    method swallows its own I/O errors and records them in `stats()["errors"]`:
    a full disk must degrade the log, never stall or kill the serial reader.
    """

    def __init__(self, path, run_id=None, flush_interval_s=1.0,
                 flush_every_lines=64, note_open=True):
        self.path = path
        self.run_id = run_id
        self.flush_interval_s = flush_interval_s
        self.flush_every_lines = flush_every_lines
        self._lock = threading.Lock()
        self._fh = None
        self._lines = 0
        self._bytes = 0
        self._errors = 0
        self._last_error = None
        self._since_flush = 0
        self._last_flush = 0.0
        self._opened_wall = None
        self._opened_mono = None
        self._closed = False
        self._open(note_open)

    # -- lifecycle ----------------------------------------------------------

    def _open(self, note_open):
        try:
            d = os.path.dirname(os.path.abspath(self.path))
            if d:
                os.makedirs(d, exist_ok=True)
            fresh = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
            self._fh = open(self.path, "a", buffering=io.DEFAULT_BUFFER_SIZE,
                            encoding="utf-8", errors="backslashreplace",
                            newline="\n")
            self._opened_wall = time.time()
            self._opened_mono = time.monotonic()
            self._last_flush = self._opened_mono
            if fresh:
                self._fh.write(_HEADER)
            if note_open:
                self._note_locked(
                    "open run_id=%s pid=%d wall=%.6f mono=%.6f"
                    % (self.run_id, os.getpid(), self._opened_wall,
                       self._opened_mono))
            self._fh.flush()
        except Exception as exc:  # noqa: BLE001
            self._fh = None
            self._errors += 1
            self._last_error = "open failed: %s" % exc

    def close(self):
        with self._lock:
            if self._fh is None:
                self._closed = True
                return
            try:
                self._note_locked("close lines=%d bytes=%d errors=%d "
                                  "duration_s=%.3f"
                                  % (self._lines, self._bytes, self._errors,
                                     time.monotonic() - (self._opened_mono or
                                                         time.monotonic())))
                self._fh.flush()
                os.fsync(self._fh.fileno())
            except Exception as exc:  # noqa: BLE001
                self._errors += 1
                self._last_error = "close failed: %s" % exc
            finally:
                try:
                    self._fh.close()
                except Exception:  # noqa: BLE001
                    pass
                self._fh = None
                self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -- writing ------------------------------------------------------------

    def write_line(self, wall_epoch, monotonic_s, line):
        """Full-fidelity path: nav.NavReader calls this with the clocks read
        at the moment the bytes came off the port."""
        self._write(wall_epoch, monotonic_s, line, exact=True)

    def __call__(self, wall_epoch, line):
        """Legacy two-argument hook shape (rigd/run.py).  The monotonic column
        is sampled here rather than at receive time; the record is marked `~`
        so nobody later mistakes it for a receive-instant measurement."""
        self._write(wall_epoch, time.monotonic(), line, exact=False)

    def _write(self, wall_epoch, monotonic_s, line, exact=True):
        if self._fh is None:
            return
        if isinstance(line, (bytes, bytearray)):
            line = bytes(line).decode("ascii", "backslashreplace")
        # never let an embedded newline break the one-record-per-line contract
        line = line.replace("\r", "").replace("\n", "\\n")
        rec = "%.6f %.6f %s%s\n" % (wall_epoch, monotonic_s,
                                    "" if exact else "~", line)
        with self._lock:
            fh = self._fh
            if fh is None:
                return
            try:
                fh.write(rec)
                self._lines += 1
                self._bytes += len(rec)
                self._since_flush += 1
                mono = monotonic_s
                if (self._since_flush >= self.flush_every_lines
                        or mono - self._last_flush >= self.flush_interval_s):
                    fh.flush()
                    self._since_flush = 0
                    self._last_flush = mono
            except Exception as exc:  # noqa: BLE001
                self._errors += 1
                self._last_error = str(exc)

    def note(self, text):
        """Write a `#` metadata line (port opened, gateway lost, run event…)."""
        with self._lock:
            self._note_locked(text)

    def _note_locked(self, text):
        if self._fh is None:
            return
        try:
            self._fh.write("# %.6f %.6f %s\n"
                           % (time.time(), time.monotonic(),
                              str(text).replace("\n", " ")))
            self._fh.flush()
            self._since_flush = 0
            self._last_flush = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self._errors += 1
            self._last_error = str(exc)

    def flush(self):
        with self._lock:
            if self._fh is None:
                return
            try:
                self._fh.flush()
                self._since_flush = 0
                self._last_flush = time.monotonic()
            except Exception as exc:  # noqa: BLE001
                self._errors += 1
                self._last_error = str(exc)

    # -- introspection ------------------------------------------------------

    def stats(self):
        with self._lock:
            return {
                "path": self.path,
                "run_id": self.run_id,
                "open": self._fh is not None,
                "closed": self._closed,
                "lines": self._lines,
                "bytes": self._bytes,
                "errors": self._errors,
                "last_error": self._last_error,
                "opened_wall": self._opened_wall,
                "age_s": (None if self._opened_mono is None
                          else time.monotonic() - self._opened_mono),
            }


def open_run_log(run_dir, run_id=None, name=LOG_NAME, **kw):
    """Open `<run_dir>/nmea_raw.log` — the path PROTOCOL.md specifies."""
    if run_id is None:
        run_id = os.path.basename(os.path.normpath(run_dir))
    return RawNmeaLog(os.path.join(run_dir, name), run_id=run_id, **kw)


# ---------------------------------------------------------------------------
# Reading back
# ---------------------------------------------------------------------------

def parse_record(text):
    """One log line -> {'wall','mono','line','exact'} or None for metadata."""
    if not text or text.startswith("#"):
        return None
    parts = text.rstrip("\n").split(" ", 2)
    if len(parts) < 3:
        return None
    try:
        wall = float(parts[0])
        mono = float(parts[1])
    except ValueError:
        return None
    line = parts[2]
    exact = not line.startswith("~")
    if not exact:
        line = line[1:]
    return {"wall": wall, "mono": mono, "line": line, "exact": exact}


def read_records(path):
    """Yield every gateway line recorded in `path`, in order."""
    with open(path, "r", encoding="utf-8", errors="backslashreplace") as fh:
        for text in fh:
            rec = parse_record(text)
            if rec is not None:
                yield rec


def replay(path, reader, use_recorded_time=True, limit=None,
           freeze_clock=True):
    """Push a recorded log back through a nav.NavReader.

    `use_recorded_time` stamps each line with the wall epoch it was received
    at, so staleness/`fix_at()` behave exactly as they did live.  Returns the
    number of lines fed.

    WHAT FAILED: this docstring promised the recorded run's behaviour, but
    every "now" judgement — TimeAuthority freshness, gateway_online — was made
    against TODAY's time.time().  Replaying yesterday's log therefore always
    reported time_source='jetson', gps_offset_s=0.0 and gateway_online=False,
    so a "wrong datetime" or "dropped to jetson mid-run" field report could
    not be reproduced at a desk at all.

    WHY THIS SHAPE: two complementary halves.  (1) nav.fix_at(t) now judges
    at `t` itself, so per-frame lookups — what run.py's flight_log rows
    actually use — come back as they were on the water with no extra
    ceremony.  (2) `freeze_clock` parks the reader's judgement clock at the
    last recorded line (nav.NavReader.set_clock), so the instant-less views —
    snapshot(), health(), gateway_online — describe the END of the log rather
    than this afternoon.  Freezing is skipped for a reader whose serial
    thread is running: a live reader with a frozen clock would call a dead
    gateway online forever.  Pass freeze_clock=False to keep the wall clock.
    """
    n = 0
    last_wall = None
    for rec in read_records(path):
        reader.feed_line(rec["line"], rec["wall"] if use_recorded_time else None)
        if rec["wall"] is not None:
            last_wall = rec["wall"]
        n += 1
        if limit and n >= limit:
            break
    if freeze_clock and use_recorded_time and last_wall is not None:
        run = getattr(reader, "_run", None)
        running = bool(run is not None and run.is_set())
        setter = getattr(reader, "set_clock", None)
        if callable(setter) and not running:
            setter(lambda t=last_wall: t)
    return n


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _selftest():
    import shutil
    import tempfile

    failures = []

    def truthy(name, cond, detail=""):
        if not cond:
            failures.append("%s: %s" % (name, detail or "false"))

    def check(name, got, want):
        if got != want:
            failures.append("%s: got %r want %r" % (name, got, want))

    tmp = tempfile.mkdtemp(prefix="navlog-test-")
    try:
        run_dir = os.path.join(tmp, "260816_0100_transect-01")
        log = open_run_log(run_dir)
        check("log path", os.path.basename(log.path), LOG_NAME)
        check("run_id from dir", log.run_id, "260816_0100_transect-01")

        samples = [
            "$PDGY,TEXT,iKonvert v1.04",
            "$PDGY,000000,,,,,,,",
            "!PDGY,129025,2,3,255,12.345,xlMDGuJ7rtA=",
            "line with a * and , and \"quotes\" kept verbatim",
            "\\xff\\xfe binary escaped by the reader",
        ]
        base_w, base_m = 1786800000.0, 1234.0
        for i, s in enumerate(samples):
            log.write_line(base_w + i * 0.1, base_m + i * 0.1, s)
        # the legacy two-argument hook shape must work too
        log(base_w + len(samples) * 0.1, "via the two-arg hook")
        log.note("gateway lost, reopening")
        log.flush()

        st = log.stats()
        check("stats lines", st["lines"], len(samples) + 1)
        truthy("stats no errors", st["errors"] == 0, str(st))

        log.close()
        truthy("closed", log.stats()["closed"])

        recs = list(read_records(log.path))
        check("records read back", len(recs), len(samples) + 1)
        for i, s in enumerate(samples):
            check("record %d verbatim" % i, recs[i]["line"], s)
            truthy("record %d wall" % i,
                   abs(recs[i]["wall"] - (base_w + i * 0.1)) < 1e-6)
            truthy("record %d mono" % i,
                   abs(recs[i]["mono"] - (base_m + i * 0.1)) < 1e-6)
            truthy("record %d exact" % i, recs[i]["exact"])
        truthy("legacy record marked inexact", not recs[-1]["exact"])

        raw = open(log.path, encoding="utf-8").read()
        truthy("header written", raw.startswith("# wildsync nmea_raw.log v1"))
        truthy("open note written", "open run_id=" in raw)
        truthy("close note written", "close lines=" in raw)
        truthy("custom note written", "gateway lost, reopening" in raw)

        # appending to an existing log must not duplicate the header
        log2 = open_run_log(run_dir)
        log2.write_line(base_w + 9, base_m + 9, "$PDGY,000000,,,,,,,")
        log2.close()
        check("header appears once",
              open(log.path, encoding="utf-8").read().count(
                  "# wildsync nmea_raw.log v1"), 1)
        check("records after append", len(list(read_records(log.path))),
              len(samples) + 2)

        # embedded newline must not break the one-record-per-line contract
        log3 = RawNmeaLog(os.path.join(tmp, "nl.log"))
        log3.write_line(1.0, 2.0, "a\nb\r\nc")
        log3.close()
        r = list(read_records(os.path.join(tmp, "nl.log")))
        check("newlines escaped", len(r), 1)
        check("newline escape content", r[0]["line"], "a\\nb\\nc")

        # a log whose directory vanishes must degrade, not raise
        bad = RawNmeaLog("/proc/definitely/not/writable/x.log")
        bad.write_line(1.0, 2.0, "should not raise")
        bad.note("nor should this")
        bad.flush()
        bad.close()
        truthy("unwritable log records an error", bad.stats()["errors"] > 0)

        # --- replay through a real NavReader -------------------------------
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            import nav
        except Exception as exc:  # noqa: BLE001
            failures.append("import nav: %s" % exc)
            nav = None
        if nav is not None:
            live = os.path.join(tmp, "live.log")
            lg = RawNmeaLog(live, note_open=False)
            t0 = time.time()
            import base64 as _b64
            import struct as _st
            pos = _st.pack("<ii", round(43.642567 * 1e7),
                           round(-79.387139 * 1e7))
            dep = _st.pack("<BIhB", 0, 1234, 500, 0xFF)
            lg.write_line(t0, 1.0, "$PDGY,000000,,,,,,,")
            lg.write_line(t0 + 0.1, 1.1, "!PDGY,129025,2,3,255,1.0,%s"
                          % _b64.b64encode(pos).decode())
            lg.write_line(t0 + 0.2, 1.2, "!PDGY,128267,3,4,255,1.0,%s"
                          % _b64.b64encode(dep).decode())
            lg.close()

            rd = nav.NavReader(port="/dev/null", auto_reopen=False)
            n = replay(live, rd)
            check("replay line count", n, 3)
            fix = rd.fix_at(t0 + 0.2)
            truthy("replay produced a valid fix", fix["valid"], str(fix))
            truthy("replay lat", abs(fix["lat"] - 43.642567) < 1e-7)
            truthy("replay depth", abs(fix["depth_from_xplore9"] - 12.34) < 1e-9)
            check("replay utm zone", fix["utm_zone"], "17T")
            truthy("replay easting",
                   abs(fix["xutm"] - 630084.3008) < 0.05, str(fix["xutm"]))

            # and the round trip: NavReader -> RawNmeaLog -> replay -> same fix
            rt = os.path.join(tmp, "roundtrip.log")
            rtlog = RawNmeaLog(rt, note_open=False)
            rd2 = nav.NavReader(port="/dev/null", auto_reopen=False)
            rd2.set_raw_hook(rtlog)
            rd2.feed_line("!PDGY,129025,2,3,255,1.0,%s"
                          % _b64.b64encode(pos).decode(), t0)
            rtlog.close()
            rd3 = nav.NavReader(port="/dev/null", auto_reopen=False)
            replay(rt, rd3)
            check("round-trip lat identical",
                  rd3.fix_at(t0)["lat"], rd2.fix_at(t0)["lat"])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        for f in failures:
            print("FAIL", f)
        print("NAVLOG SELF-TEST: FAIL (%d)" % len(failures))
        return False
    print("NAVLOG SELF-TEST: PASS (all assertions)")
    return True


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] not in ("--selftest", "-t"):
        # dump a log: navlog.py <path>
        for rec in read_records(sys.argv[1]):
            print("%.6f %8.3f %s" % (rec["wall"], rec["mono"], rec["line"]))
        sys.exit(0)
    sys.exit(0 if _selftest() else 1)

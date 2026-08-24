#!/usr/bin/env python3
"""Audit regression suite - ilxctl lane (findings X2/X3/X5, tripwires X1/X4/X6).

Unlike the other suites this one drives the REAL C++ daemon: it builds
build/ilxctl with make and talks to it over 127.0.0.1 with NO camera attached.
Every behavioural check here works camera-less because the failure modes under
audit live at the HTTP surface, before the SDK is reached:

X2  /api/connect during a connect already in flight walked into enumerate()
    and blocked on the SDK mutex behind it - for ever against the wedged
    SDK::Connect of HANDOFF 2.2, and httplib serves requests from a fixed
    thread pool, so eight of those and the daemon stopped answering
    /api/status too (the bind-before-connect fix defeated).  Camera-less the
    same serialization is measurable and the pool exhaustion is reproducible:
    enumerate(3) holds the SDK ~4 s, so a burst of connects used to pin every
    worker for 4 s EACH and /api/status went dark behind them.  Now one
    connect runs and the rest answer 409 {"ok":false,"pending":true} in
    microseconds.

X3  POST /api/exposure {"value":"abc"} parsed as 0 (jsonNum's catch) and went
    to SetDeviceProperty, stalling the SDK ~6 s on the live rig - long enough
    for the monitor's status poll to time out and flap cam1 to "no connected
    camera" over one bad POST.  The pre-fix tell, camera-less, is the error
    "not connected": proof the garbage got PAST the parser into the camera
    layer.  Now the daemon answers 400 naming the bad value without touching
    the SDK.  (The other half of X3 - validation against the body's own
    choice list - needs a body and is bench-tested, not tested here.)

X5  The startup autoconnect answers a concurrent /api/connect while its
    enumerate/connect is still in flight.  Pre-fix that was a slow blocking
    answer (and, with a body attached, the false "already connected" that
    rigcore diagnoses as a dead SDK handle: a "restart ilxctl on the node"
    alarm plus a 60 s backoff on every deploy).  Now: pending:true, fast.

X1/X6 (tripwires) contract C2's readback keys and the card-list
    filesInContent field need a camera and a card to exercise, so they are
    asserted twice instead: the key names in the source (the exact strings
    rigcore.CONVERGE_FIELDS / WRITABLE_KEY look for) and label literals in
    the SHIPPED binary, proving the built daemon carries the code.

X7  The X2 pass bounded only the READ side of the SDK mutex.  Every property
    WRITE (setProp/setPropForced/sendCmd), the choice-list read the X3
    validation added, cardList/cardDelete, live view and disconnect still took
    it with a plain lock_guard, so a body wedged inside an SDK call (HANDOFF
    2.1) stranded one httplib worker per request until the pool of 8 was gone
    and the node answered nothing at all - /api/status included, which was the
    one thing still diagnosing it correctly.  Now: the ACQUISITION is bounded
    (the call itself is not, so a slow write still returns its real result), a
    holder past the wedge threshold is refused instantly rather than waited
    out, and the refusal says "not sent" (409 busy:true) instead of "the body
    refused it" - which rigcore would record as a divergence.
    Camera-less this needs the --test-sdk-hold hook: with no camera every SDK
    entry point returns in microseconds, so the fault cannot otherwise be
    reproduced off the hardware.

X8  ilxctl accepted AF-S/AF-C from its own :8080 page and from any curl: the
    always-manual-focus operator rule was enforced only in rigd, which is not
    running during a drain, is not running when it is stopped, and is not the
    thing that talks to the SDK.  piagent asserts FOCUS (a half-press) before
    every TRIGGER, so one AF frame moves the lens - and focus POSITION is
    never converged, so rigd pushing the mode back does not undo the travel.
    Now every autofocus path (focus mode, af:true releases, af:true interval
    sequences) is refused in ilxctl itself with a 403 and a log line, and the
    only hatch is a start-up flag no field unit is given.

X4 (cardPull keyed handshake, cancel on timeout, disconnect wakes the CV) has
    no camera-less surface at all - it is code review plus the bench
    procedure - so only its presence in source and binary is tripwired.

Hermetic: builds in the worktree, binds only 127.0.0.1 on ephemeral ports,
temp save dirs, no camera, no fleet, no rigd.  soaktest's netguard (installed
on import) refuses anything not on 127/8.

Run standalone:  python3 rig/tests/audit_ilxctl.py
"""

import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
ROOT = os.path.dirname(RIG)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

from soaktest import check, sect, note, wait_for     # noqa: E402

BIN = os.path.join(ROOT, "build", "ilxctl")
SRC = os.path.join(ROOT, "src")


# ---------------------------------------------------------------------------
# Plumbing: build, spawn, talk.
# ---------------------------------------------------------------------------

def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _req(port, method, path, body=None, timeout=10.0):
    """Returns (status, dict, seconds). status 0 = transport error/timeout."""
    url = "http://127.0.0.1:%d%s" % (port, path)
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(url, data=data, method=method)
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            payload, code = resp.read(), resp.getcode()
    except urllib.error.HTTPError as e:
        payload, code = e.read(), e.code
    except Exception:
        return 0, {}, time.monotonic() - t0
    try:
        doc = json.loads(payload.decode())
    except Exception:
        doc = {}
    return code, doc, time.monotonic() - t0


class Daemon:
    def __init__(self, autoconnect, extra=()):
        self.port = _free_port()
        self.home = tempfile.mkdtemp(prefix="audit_ilxctl_")
        args = [BIN, "--host", "127.0.0.1", "--port", str(self.port),
                "--save-dir", os.path.join(self.home, "spool")]
        if not autoconnect:
            args.append("--no-autoconnect")
        args.extend(extra)
        self.log = open(os.path.join(self.home, "ilxctl.log"), "wb")
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=self.log)

    def wait_ready(self, timeout=25.0):
        # With autoconnect the first status can wait out statusJson's 4.5 s
        # SDK-lock bound while the startup enumerate holds it - allow for it.
        return wait_for(lambda: _req(self.port, "GET", "/api/status",
                                     timeout=8.0)[0] == 200,
                        timeout=timeout, interval=0.2)

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)
        self.log.close()
        shutil.rmtree(self.home, ignore_errors=True)


def _build():
    r = subprocess.run(["make"], cwd=ROOT, capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0:
        note("make failed:\n" + out[-2000:])
        return False, out
    return os.access(BIN, os.X_OK), out


# ---------------------------------------------------------------------------
# X3 - malformed write requests are refused at the door.
# ---------------------------------------------------------------------------

def _x3_reject_garbage(d):
    sect("audit-ilxctl X3: /api/exposure input validation")
    code, doc, secs = _req(d.port, "POST", "/api/exposure",
                           {"which": "iso", "value": "abc"})
    # The pre-fix tell: "abc" parsed as 0 and went to the camera layer, whose
    # answer (camera-less) is "not connected" - i.e. the garbage got past the
    # parser. Post-fix the 400 names the value and never touches the SDK.
    err = str(doc.get("error") or "")
    check("string value refused with 400 naming the value",
          code == 400 and doc.get("ok") is False and "value" in err
          and "abc" in err and "not connected" not in err,
          "code=%s err=%r" % (code, err))
    check("string value refused instantly (<2 s, no SDK stall)", secs < 2.0,
          "%.3fs" % secs)

    code, doc, _ = _req(d.port, "POST", "/api/exposure", {"which": "iso"})
    err = str(doc.get("error") or "")
    check("absent value refused with 400, not forwarded as 0",
          code == 400 and "value" in err and "not connected" not in err,
          "code=%s err=%r" % (code, err))

    code, doc, _ = _req(d.port, "POST", "/api/exposure", {"value": 400})
    check("absent which refused with 400",
          code == 400 and "which" in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))

    # A well-formed numeric request must still reach the camera layer - with
    # no camera that means exactly "not connected", nothing about the value.
    code, doc, _ = _req(d.port, "POST", "/api/exposure",
                        {"which": "iso", "value": 400})
    check("numeric value passes the gate (fails only on 'not connected')",
          code == 400 and "not connected" in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))

    # A fraction where an enum is expected is garbage too; it used to truncate.
    code, doc, _ = _req(d.port, "POST", "/api/exposure",
                        {"which": "filetype", "value": 2.5})
    check("fractional value refused rather than truncated",
          code == 400 and "not connected" not in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))

    # Focus is the one control the rig can never afford to guess at: the rig
    # is ALWAYS manual focus, so a garbage mode must not become a default.
    code, doc, _ = _req(d.port, "POST", "/api/focus/mode", {"mode": "AF"})
    check("focus/mode garbage refused with 400 naming the field",
          code == 400 and "mode" in str(doc.get("error") or "")
          and "not connected" not in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))

    code, doc, _ = _req(d.port, "POST", "/api/zoom/position",
                        {"value": "1e999"})
    check("non-finite value refused with 400",
          code == 400 and "not connected" not in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))

    # /api/store takes its value under either key, with a documented default:
    # an ABSENT key must still default, a present-and-garbage one must not.
    code, doc, _ = _req(d.port, "POST", "/api/store", {"dest": "card"})
    check("store garbage refused with 400",
          code == 400 and "not connected" not in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))
    code, doc, _ = _req(d.port, "POST", "/api/store", {})
    check("store with no field still reaches the camera layer (default kept)",
          code == 400 and "not connected" in str(doc.get("error") or ""),
          "code=%s err=%r" % (code, doc.get("error")))


# ---------------------------------------------------------------------------
# X2 - one connect at a time; everyone else answers immediately.
# ---------------------------------------------------------------------------

def _x2_single_flight(d):
    sect("audit-ilxctl X2: concurrent /api/connect answers pending, fast")
    first = {}

    def go():
        first["res"] = _req(d.port, "POST", "/api/connect", {}, timeout=30.0)

    t = threading.Thread(target=go)
    t.start()
    time.sleep(0.4)                      # the first is now inside enumerate()
    code, doc, secs = _req(d.port, "POST", "/api/connect", {}, timeout=10.0)
    check("second connect answers pending:true with 409",
          code == 409 and doc.get("pending") is True
          and doc.get("ok") is False,
          "code=%s doc=%r" % (code, doc))
    check("second connect answers in <1.5 s (worker not stranded "
          "behind the first's enumerate)", secs < 1.5, "%.3fs" % secs)

    # /api/card/mode disconnects + re-enumerates: it must refuse fast rather
    # than pile onto the SDK mutex behind the connect in flight.
    code, doc, secs = _req(d.port, "POST", "/api/card/mode",
                           {"mode": "transfer"}, timeout=10.0)
    check("card/mode during a connect answers pending:true, fast",
          code == 409 and doc.get("pending") is True and secs < 1.5,
          "code=%s secs=%.3f doc=%r" % (code, secs, doc))

    t.join(timeout=30)
    check("first connect completed on its own", not t.is_alive())
    code, doc, _ = first.get("res", (0, {}, 0))
    check("first connect reports the real outcome (no camera found)",
          code == 400 and "no camera" in str(doc.get("error") or ""),
          "code=%s doc=%r" % (code, doc))

    # With the gate released, a fresh connect must run the real path again
    # rather than answer pending for ever.
    code, doc, _ = _req(d.port, "POST", "/api/connect", {}, timeout=30.0)
    check("gate released after the attempt: next connect runs the real path",
          code == 400 and "no camera" in str(doc.get("error") or ""),
          "code=%s doc=%r" % (code, doc))


def _x2_pool_survives_burst(d):
    """The actual field failure: /api/status goes dark behind stuck connects.

    httplib 0.19 serves every request from a fixed pool of max(8, hw-1)
    workers. Pre-fix each /api/connect held a worker for the whole of the SDK
    call it was queued behind (camera-less: ~4 s per enumerate, serially; in
    the field, on a wedged SDK::Connect: for ever), so a burst from rigcore's
    2 s reconnect loop consumed the pool and the monitor's /api/status poll
    timed out - the node went ILX_DOWN with the daemon still running.
    Post-fix exactly one connect works and the rest answer 409, so status
    must stay responsive throughout.
    """
    sect("audit-ilxctl X2: /api/status survives a burst of connects")
    burst = 16                    # > any plausible worker pool
    done = []

    def hammer():
        done.append(_req(d.port, "POST", "/api/connect", {}, timeout=25.0))

    threads = [threading.Thread(target=hammer) for _ in range(burst)]
    for t in threads:
        t.start()
    time.sleep(0.6)               # the pool is now as busy as it will get
    worst, codes = 0.0, []
    for _ in range(4):
        code, _doc, secs = _req(d.port, "GET", "/api/status", timeout=8.0)
        codes.append(code)
        worst = max(worst, secs)
    check("status answered every poll during the burst",
          all(c == 200 for c in codes), "codes=%r" % (codes,))
    # 4.5 s is statusJson's own SDK-lock bound; past that the request was
    # queued behind a stranded worker, not merely behind the lock.
    check("status stayed within its 4.5 s lock bound during the burst",
          worst < 5.5, "worst=%.2fs" % worst)
    for t in threads:
        t.join(timeout=40)
    check("no connect worker was stranded",
          all(not t.is_alive() for t in threads))
    pend = sum(1 for c, doc, _ in done if c == 409 and doc.get("pending"))
    check("all but one of the burst answered pending:true",
          pend >= burst - 1, "%d/%d pending" % (pend, burst))


# ---------------------------------------------------------------------------
# X5 - the STARTUP connect claims the same gate.
# ---------------------------------------------------------------------------

def _x5_startup_window(opts):
    sect("audit-ilxctl X5: connect during the startup autoconnect window")
    d = Daemon(autoconnect=True)
    try:
        # rigcore POSTs /api/connect from the moment ilxctl binds - i.e. while
        # the startup thread is still inside enumerate(), a window
        # m_connecting alone cannot cover. Probe from t=0 with a SHORT client
        # timeout: an answer that does not come back inside 1 s is, by
        # definition, a worker held on the SDK mutex, which is the defect.
        # Post-fix the very first probe the port accepts answers 409 in
        # microseconds; pre-fix no probe ever answers inside the budget
        # (measured 8.1 s: the startup enumerate plus its own).
        got, doc, secs = None, {}, 0.0
        deadline = time.time() + 8.0
        while time.time() < deadline:
            code, doc, secs = _req(d.port, "POST", "/api/connect", {},
                                   timeout=1.0)
            if code == 409 and doc.get("pending") is True:
                got = secs
                break
            if code and "already connected" in str(doc.get("error") or ""):
                break                       # the dead-handle string: report it
            time.sleep(0.05)
        check("connect in the startup window answers pending:true with 409, "
              "fast", got is not None and got < 1.0,
              "%.3fs" % secs if got is None else "%.3fs" % got)
        # Whatever happened, ilxctl must never hand rigcore the string it
        # diagnoses as a dead SDK handle ("restart ilxctl on the node" plus a
        # 60 s backoff) while a connect is merely still in flight.
        check("never the dead-handle string during startup",
              "already connected" not in str(doc.get("error") or ""),
              repr(doc.get("error")))
        check("daemon answers /api/status through the startup window",
              d.wait_ready())
    finally:
        d.stop()


# ---------------------------------------------------------------------------
# X1/X4/X6 - contract keys and fix presence (tripwires).
# ---------------------------------------------------------------------------

def _x1_x6_tripwires():
    sect("audit-ilxctl X1/X4/X6: contract keys present (tripwire)")
    with open(os.path.join(SRC, "camera.cpp"), encoding="utf-8") as f:
        cam = f.read()
    with open(os.path.join(SRC, "main.cpp"), encoding="utf-8") as f:
        main = f.read()

    # C2: <name>Value / <name>Label are built as key + "Value" at run time, so
    # the discriminating token in source is the emitFmt/flagOf key itself.
    emitted = set(re.findall(r'emitFmt\("([a-z]+)"', cam))
    want = {"filetype", "imagesize", "transsize", "rawtype", "quality",
            "pcsave"}
    check("statusJson emits a Value+Label pair for every C2 format field",
          want <= emitted, "missing=%r" % sorted(want - emitted))
    check("statusJson emits expcompValue (signed mEV)",
          'emitNum("expcompValue"' in cam)
    check("expcompValue is emitted only when the body answered the read",
          "expCompSeen" in cam)
    # C2 writable flags: rigcore.WRITABLE_KEY falls back to the field name for
    # these, so the flagOf key has to be exactly the desired-vector name.
    flags = set(re.findall(r'flagOf\("([A-Za-z]+)"', cam))
    wantf = want | {"expcomp"}
    check("writable flags use rigcore's field names for the C2 fields",
          wantf <= flags, "missing=%r" % sorted(wantf - flags))
    # "omit rather than emit -1" is what lets rigcore tell "the body says
    # LossLessL" from "the body did not answer": an absent key is blind, a -1
    # is a real (wrong) value that can never converge.
    check("a format field the body did not answer is omitted, not sent as -1",
          re.search(r"emitFmt = .*?\n\s*if \(v < 0\) return;", cam,
                    re.S) is not None)

    # X6: one entry per file plus the per-content count the drain needs to
    # refuse deleting a content whose sibling was never pulled.
    for key in ("contentId", "fileId", "name", "size", "format",
                "captured_utc", "filesInContent"):
        check("card/list emits %s" % key, '\\"%s\\":' % key in main)

    # X4: the three parts of the keyed handshake.
    check("cardPull keys its wait on the file name (m_rtWant)",
          "m_rtWant" in cam and "Dropping a stale transfer result" in cam)
    check("cardPull cancels the SDK transfer on timeout",
          "ControlGetRemoteTransferContentsDataFile" in cam)
    check("OnDisconnected wakes a waiting cardPull",
          re.search(r"OnDisconnected.*?m_rtCv\.notify_all", cam,
                    re.S) is not None)

    # Finding 3: an explicit disconnect must claim the abandon flag, or the
    # SDK's own auto-reconnect can complete inside disconnect()'s join/lock
    # window and leave m_connected=true with m_handle=0 - a camera the fleet
    # calls connected that no operation can reach.
    check("disconnect() claims the abandon flag before dropping m_connected",
          re.search(r"Camera::disconnect\([^)]*\).*?m_attemptAbandoned = true;"
                    r".*?m_connected = false;", cam, re.S) is not None)
    check("OnConnected re-checks the abandon flag after setting m_connected",
          re.search(r"m_connected = true;\s*\n(\s*//.*\n)*\s*"
                    r"if \(m_attemptAbandoned\)", cam) is not None)

    # The SHIPPED binary must carry all of it - _build() ran make first, so a
    # stale build/ilxctl here means the build did not pick the source up (a
    # broken lib/ symlink, a read-only build dir).
    with open(BIN, "rb") as f:
        blob = f.read()
    for lit in (b"LossLessL", b"JPEG only", b"Uncompressed", b"ExFine",
                b"filesInContent", b"connect in progress"):
        check("built binary carries %r" % lit.decode(), lit in blob)



# ---------------------------------------------------------------------------
# X7 - a wedged SDK may not take the daemon down, and may not be reported as
#      a refusal by the camera.
# ---------------------------------------------------------------------------

def _hold(port, ms):
    """Wedge the SDK for ms. Needs --test-sdk-hold; 404 without it."""
    return _req(port, "POST", "/api/test/sdk-hold", {"ms": ms}, timeout=5.0)


def _x7_slow_sdk_still_tells_the_truth(d):
    """A busy-but-progressing SDK must NOT produce a busy refusal.

    This is the reason the audit pass gave for leaving writes unbounded: a
    bounded write that is merely slow comes back as a failure, and rigcore
    records a failed write as blind_fail -> settings_divergent, i.e. an
    invented divergence alarm on two bodies that are actually fine. So the
    bound has to be on the ACQUISITION only, generous, and skipped only for a
    holder that is past the wedge threshold. Camera-less the real answer is
    "not connected": anything else here means the write was refused by the
    daemon instead of being asked of the camera.
    """
    sect("audit-ilxctl X7: a merely slow SDK still returns the real answer")
    code, doc, _ = _hold(d.port, 3000)             # under both bounds
    if not check("test hook available (--test-sdk-hold)", code == 200,
                 "code=%s doc=%r" % (code, doc)):
        return
    time.sleep(0.3)
    code, doc, secs = _req(d.port, "POST", "/api/focus/mode", {"mode": 1},
                           timeout=15.0)
    err = str(doc.get("error") or "")
    check("write waits out a slow SDK instead of refusing",
          code == 400 and "not connected" in err and doc.get("busy") is None,
          "code=%s secs=%.2f err=%r" % (code, secs, err))
    check("it really waited (the hold was still up)", secs > 1.5,
          "%.2fs" % secs)
    wait_for(lambda: _req(d.port, "GET", "/api/status", timeout=8.0)[1]
             .get("busy") is None, timeout=8.0, interval=0.2)


def _x7_wedged_sdk(d):
    sect("audit-ilxctl X7: a wedged SDK is bounded, named, and not a refusal")
    code, doc, _ = _hold(d.port, 22000)
    if not check("test hook available (--test-sdk-hold)", code == 200,
                 "code=%s doc=%r" % (code, doc)):
        return

    # While the holder is young the status bound is the X2 one (4.5 s) and the
    # body already says what is in the way.
    code, doc, secs = _req(d.port, "GET", "/api/status", timeout=8.0)
    check("status answers during the wedge with busy:true",
          code == 200 and doc.get("busy") is True, "code=%s doc=%r" % (code, doc))
    check("status names the SDK call in the way and its age",
          doc.get("sdkOp") == "test-hold"
          and isinstance(doc.get("sdkHeldS"), (int, float)),
          "sdkOp=%r sdkHeldS=%r" % (doc.get("sdkOp"), doc.get("sdkHeldS")))
    check("status stayed inside its 4.5 s bound", secs < 5.5, "%.2fs" % secs)

    # Past the wedge threshold nobody waits any more.
    ok_overdue = wait_for(
        lambda: _req(d.port, "GET", "/api/status", timeout=8.0)[1]
        .get("sdkOverdue") is True, timeout=14.0, interval=0.3)
    check("status marks a holder past the wedge threshold overdue", ok_overdue)

    code, doc, secs = _req(d.port, "GET", "/api/status", timeout=8.0)
    check("status answers immediately once the SDK is overdue "
          "(no 4.5 s queue behind a wedge)", code == 200 and secs < 1.5,
          "%.2fs" % secs)

    # THE deploy-blocker: eight of these used to strand the whole worker pool.
    burst = 12
    out = []

    def hammer(body):
        out.append(_req(d.port, "POST", "/api/store", body, timeout=12.0))

    threads = [threading.Thread(target=hammer, args=({"dest": 3},))
               for _ in range(burst)]
    for t in threads:
        t.start()
    # ... and the poll that decides whether the node is alive at all.
    time.sleep(0.3)
    scode, _sdoc, ssecs = _req(d.port, "GET", "/api/status", timeout=6.0)
    check("status still answers while a burst of writes hits the wedge",
          scode == 200 and ssecs < 3.0, "code=%s %.2fs" % (scode, ssecs))
    for t in threads:
        t.join(timeout=20)
    check("no write worker was stranded", all(not t.is_alive() for t in threads))
    codes = [c for c, _d, _s in out]
    busy = [d_ for c, d_, _s in out if c == 409 and d_.get("busy") is True]
    check("every write answered", len(out) == burst and all(codes),
          "codes=%r" % (codes,))
    check("every write answered 409 busy:true", len(busy) == burst,
          "%d/%d busy" % (len(busy), burst))
    check("writes answered fast (nothing waited out the wedge)",
          all(s < 3.0 for _c, _d, s in out),
          "worst=%.2fs" % max([s for _c, _d, s in out] or [0]))

    # The distinction the whole fix turns on: this is NOT the body refusing a
    # value. rigcore keeps a refusal in blind_errors and alarms
    # settings_divergent on it; a "not sent" must not look the same.
    errs = [str(d_.get("error") or "") for _c, d_, _s in out]
    check("the answer says the write was NOT sent, not that it was refused",
          all(e.startswith("SDK busy") and "was NOT sent" in e for e in errs),
          repr(errs[:1]))
    check("the answer names the call holding the camera and for how long",
          all("test-hold" in e for e in errs), repr(errs[:1]))
    check("no write claims applied", all(d_.get("applied") is False
                                         for _c, d_, _s in out))

    # Every other SDK-facing route answers the same way rather than hanging.
    for path, body in (("/api/exposure", {"which": "iso", "value": 400}),
                       ("/api/focus/mode", {"mode": 1}),
                       ("/api/zoom/position", {"value": 100}),
                       ("/api/disconnect", {}),
                       ("/api/card/delete", {"confirm": "delete",
                                             "contentId": 1})):
        code, doc, secs = _req(d.port, "POST", path, body, timeout=12.0)
        check("%s answers busy fast during a wedge" % path,
              code == 409 and doc.get("busy") is True and secs < 3.0,
              "code=%s secs=%.2f doc=%r" % (code, secs, doc))
    code, doc, secs = _req(d.port, "GET", "/api/card/list", timeout=12.0)
    check("/api/card/list answers busy fast during a wedge",
          code == 409 and doc.get("busy") is True and secs < 3.0,
          "code=%s secs=%.2f" % (code, secs))

    # And it all comes back by itself when the body does.
    back = wait_for(lambda: _req(d.port, "GET", "/api/status", timeout=8.0)[1]
                    .get("busy") is None, timeout=25.0, interval=0.3)
    check("status returns to the full body once the SDK frees up", back)
    code, doc, _ = _req(d.port, "POST", "/api/focus/mode", {"mode": 1},
                        timeout=12.0)
    check("writes reach the camera layer again after the wedge clears",
          code == 400 and "not connected" in str(doc.get("error") or ""),
          "code=%s doc=%r" % (code, doc))


# ---------------------------------------------------------------------------
# X8 - always manual focus, enforced by the thing that talks to the SDK.
# ---------------------------------------------------------------------------

AF_MODES = {2: "AF-S", 3: "AF-C", 4: "AF-A", 5: "AF-D", 6: "DMF", 7: "PF"}


def _x8_manual_focus_rule(d):
    sect("audit-ilxctl X8: autofocus refused on the node itself")
    for mode, label in AF_MODES.items():
        code, doc, _ = _req(d.port, "POST", "/api/focus/mode", {"mode": mode})
        err = str(doc.get("error") or "")
        check("focus mode %s (%d) refused with 403" % (label, mode),
              code == 403 and doc.get("ok") is False,
              "code=%s err=%r" % (code, err))
        check("the %s refusal names the operator rule" % label,
              "manual focus" in err.lower(), repr(err))
        # The tell that it never reached the SDK: camera-less, anything that
        # gets past the policy answers "not connected".
        check("%s never reached the camera layer" % label,
              "not connected" not in err, repr(err))

    code, doc, _ = _req(d.port, "POST", "/api/focus/mode", {"mode": 1})
    check("MF (1) is not refused - it reaches the camera layer",
          code == 400 and "not connected" in str(doc.get("error") or ""),
          "code=%s doc=%r" % (code, doc))

    # S1=Locked is the half-press, so af:true is an autofocus path even with
    # the mode left at MF.
    code, doc, _ = _req(d.port, "POST", "/api/shutter", {"af": True})
    check("shutter af:true refused with 403",
          code == 403 and "manual focus" in str(doc.get("error") or "").lower(),
          "code=%s doc=%r" % (code, doc))
    code, doc, _ = _req(d.port, "POST", "/api/interval/start",
                        {"intervalSec": 2.0, "count": 1, "af": True})
    check("interval af:true refused with 403",
          code == 403 and "manual focus" in str(doc.get("error") or "").lower(),
          "code=%s doc=%r" % (code, doc))
    code, doc, _ = _req(d.port, "POST", "/api/shutter", {"af": False})
    check("a manual-focus release is not refused",
          code == 400 and "not connected" in str(doc.get("error") or ""),
          "code=%s doc=%r" % (code, doc))

    # An attempt on the rule must be visible to whoever reads the node: the
    # status log tail is what rigd shows.
    _, doc, _ = _req(d.port, "GET", "/api/status")
    log = " ".join(doc.get("log") or [])
    check("a refused autofocus attempt is logged where rigd can see it",
          "REFUSED" in log and "manual focus only" in log, log[-160:])
    check("status publishes afAllowed:false", doc.get("afAllowed") is False,
          repr(doc.get("afAllowed")))


def _x8_hatch_is_explicit():
    """The escape hatch exists, is out-of-band, and is visible to the fleet."""
    sect("audit-ilxctl X8: --allow-autofocus is the only way through")
    d = Daemon(autoconnect=False, extra=["--allow-autofocus"])
    try:
        if not check("ilxctl starts with --allow-autofocus", d.wait_ready()):
            return
        code, doc, _ = _req(d.port, "POST", "/api/focus/mode", {"mode": 2})
        check("with the hatch open AF-S reaches the camera layer",
              code == 400 and "not connected" in str(doc.get("error") or ""),
              "code=%s doc=%r" % (code, doc))
        _, doc, _ = _req(d.port, "GET", "/api/status")
        check("a node with the rule lifted says so in every status",
              doc.get("afAllowed") is True, repr(doc.get("afAllowed")))
    finally:
        d.stop()


# ---------------------------------------------------------------------------
# X7/X8 structural tripwires - the shape of the fix, in source and in binary.
# ---------------------------------------------------------------------------

def _x7_x8_tripwires():
    sect("audit-ilxctl X7/X8: no unbounded SDK acquisition is left (tripwire)")
    with open(os.path.join(SRC, "camera.cpp"), encoding="utf-8") as f:
        cam = f.read()
    with open(os.path.join(SRC, "main.cpp"), encoding="utf-8") as f:
        main = f.read()

    # Every take of m_sdkMutex must go through the bounded helper. A plain
    # lock_guard/unique_lock on it is the defect itself.
    stray = re.findall(r"(?:lock_guard|unique_lock)<std::recursive_timed_mutex>",
                       cam)
    check("no plain lock_guard/unique_lock on m_sdkMutex remains",
          not stray, "%d left" % len(stray))

    # ... and specifically in the entry points the review named.
    for fn in ("setProp", "setPropForced", "sendCmd", "getPropChoices",
               "cardList", "cardDelete", "liveViewJpeg", "setDateTime",
               "focusDrive", "zoomDrive", "setSaveDir", "modeSummary",
               "disconnect", "statusJson"):
        m = re.search(r"\n[A-Za-z_:<>, ]*Camera::%s\(.*?\n\}" % fn, cam, re.S)
        found = m is not None and "SdkHold" in m.group(0)
        check("%s acquires the SDK through the bounded helper" % fn, found,
              "" if found else
              ("no such function" if m is None
               else "takes m_sdkMutex without the bound"))

    # The bound is on the ACQUISITION only: once the lock is held the SDK is
    # waited out, which is what keeps a slow write from being called a failure.
    check("the wedge threshold is above every measured healthy SDK call",
          re.search(r"kSdkWedged\s*=\s*10000ms", cam) is not None)
    check("the polite wait sits under rigcore's 12 s POST timeout",
          re.search(r"kSdkWait\s*=\s*8000ms", cam) is not None)

    # "not sent" and "refused" must stay distinguishable on the wire.
    check("main.cpp answers a busy acquisition 409 busy:true, not a failure",
          "Camera::isBusyError(err)" in main and '\\"busy\\":true' in main)
    check("statusJson reports the SDK call in the way",
          '\\"sdkOp\\"' in cam and '\\"sdkHeldS\\"' in cam)

    # X8: the rule lives in the SDK layer, not only at the door.
    m = re.search(r"bool Camera::setFocusMode\(.*?\n\}", cam, re.S)
    check("setFocusMode refuses anything but CrFocus_MF",
          m is not None and "SDK::CrFocus_MF" in m.group(0)
          and "autofocusBlocked" in m.group(0))
    m = re.search(r"bool Camera::captureOnce\(.*?\n\}", cam, re.S)
    check("captureOnce refuses the autofocus half-press",
          m is not None and "autofocusBlocked" in m.group(0))
    check("the hatch is a start-up flag, not a request field",
          "--allow-autofocus" in main and "allowAutofocus(allowAf)" in main)
    check("the test hook is registered only behind --test-sdk-hold",
          re.search(r"if \(testHold\) \{\s*\n\s*g_srv\.Post\("
                    r"\"/api/test/sdk-hold\"", main) is not None)

    with open(BIN, "rb") as f:
        blob = f.read()
    # "SDK busy" alone is not discriminating - the pre-fix enumerate log line
    # ("SDK busy >5 s") already carried it. The wire wording is.
    for lit in (b"ALWAYS manual focus", b"was NOT sent to the body",
                b"--allow-autofocus"):
        check("built binary carries %r" % lit.decode(), lit in blob)


# ---------------------------------------------------------------------------

def suite(opts):
    built, out = _build()
    if not check("ilxctl builds (make)", built):
        return
    check("build is warning-free under -Wall -Wextra",
          "warning:" not in out, out[-400:] if "warning:" in out else "")
    d = Daemon(autoconnect=False)
    try:
        if not check("ilxctl starts camera-less and answers /api/status",
                     d.wait_ready()):
            return
        _, doc, _ = _req(d.port, "GET", "/api/status")
        check("camera-less status is connected:false",
              doc.get("connected") is False)
        _x3_reject_garbage(d)
        _x8_manual_focus_rule(d)
        _x2_single_flight(d)
        _x2_pool_survives_burst(d)
    finally:
        d.stop()
    # X7 needs the SDK wedged on purpose, which only --test-sdk-hold can do.
    d = Daemon(autoconnect=False, extra=["--test-sdk-hold"])
    try:
        if check("ilxctl starts with --test-sdk-hold", d.wait_ready()):
            _x7_slow_sdk_still_tells_the_truth(d)
            _x7_wedged_sdk(d)
    finally:
        d.stop()
    _x8_hatch_is_explicit()
    _x5_startup_window(opts)
    _x1_x6_tripwires()
    _x7_x8_tripwires()


if __name__ == "__main__":
    import argparse
    import soaktest
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    opts = ap.parse_args()
    t0 = time.time()
    suite(opts)
    print("\naudit_ilxctl: %d passed, %d failed in %d s"
          % (len(soaktest.PASS), len(soaktest.FAIL), time.time() - t0))
    sys.exit(1 if soaktest.FAIL else 0)

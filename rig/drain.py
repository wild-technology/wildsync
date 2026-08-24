#!/usr/bin/env python3
"""Card drain — pull each shot (its RAW and its full-size card JPEG) off
each camera's card to the host, verify it, delete it from the card, and hand
the files to ingest.

The survey shoots RAW+JPEG to the card with a Small JPEG delivered live; the
full-resolution files never cross USB during the run (they cannot, at survey
cadence). This drains them BETWEEN runs, over the SDK's Remote Transfer path:

    list the card  ->  group by contentId (one shot = RAW + card JPEG)  ->
    pull EVERY file of each content to the host  ->  every sha256 must match
    ->  delete that content from the card  ->  ingest the pulled files

A content is deleted from the card ONLY after every one of its files' host
copies' hashes match the Pi's — the card API deletes whole contents, so
verifying only the RAW and then deleting used to destroy the never-pulled
card JPEG of every shot — and a failed transfer never loses the original.
The camera is switched to RemoteTransfer control mode for the drain (it
cannot shoot in that mode) and back to remote afterwards, so a drain must
never overlap a run — rigd refuses to start one while a run is active, and
refuses a run while a drain is active.

Used from rigd (POST /api/drain, and auto after a run when armed) or stand
alone:

    python3 rig/drain.py cam1                     # drain cam1 now
    python3 rig/drain.py cam1 --dest ~/rig-raw --keep   # pull+verify, keep card
    python3 rig/drain.py --selftest

When rigd is running on this host the CLI must NOT drive the body directly:
rigd's stuck-transfer recovery flips the camera back to remote mode within
one ~2 s monitor poll and every pull dies mid-flight. The CLI therefore
probes for the daemon (GET localhost:9090/api/diag) and, when it answers,
routes the drain through POST /api/drain, tailing progress from GET
/api/drain until it finishes. Pass --standalone (with rigd stopped) to force
the direct path.
"""
import argparse
import hashlib
import json
import os
import sys
import time
import urllib.error
import urllib.request

RAW_FORMAT = 0xB101
JPEG_FORMAT = 0x3801
DEFAULT_DEST = os.path.expanduser("~/rig-raw")
NODES = {"cam1": "192.168.1.201", "cam2": "192.168.1.202"}


def _req(url, body=None, timeout=200):
    data = json.dumps(body).encode() if body is not None else None
    hdr = {"Content-Type": "application/json"} if data else {}
    try:
        with urllib.request.urlopen(urllib.request.Request(url, data=data, headers=hdr),
                                    timeout=timeout) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        # ilxctl signals a not-ready/refused card call as a 503 with a JSON
        # body; return the body so the caller can decide to retry, rather
        # than turning a transient "still indexing" into a hard failure.
        try:
            return json.load(e)
        except Exception:  # noqa: BLE001
            return {"ok": False, "error": "HTTP %d" % e.code}
    except Exception as e:  # noqa: BLE001 - URLError, socket.timeout
        # A transient network blip (PoE drop, link stall) must not escape as an
        # uncaught exception - during a mode switch that would leave the body
        # stuck in transfer mode. Return it so the caller retries or restores.
        return {"ok": False, "error": "request failed: %s" % e}


def _bytes(url, timeout=300):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


RIGD_BASE = "http://localhost:9090"


def _rigd_alive(base=RIGD_BASE, timeout=3):
    """True when a rigd answers /api/diag on this host. CLI-path only —
    tests and library callers must never probe the daemon port."""
    try:
        with urllib.request.urlopen(base + "/api/diag", timeout=timeout) as r:
            return r.status == 200
    except Exception:  # noqa: BLE001
        return False


def _drain_via_rigd(node, keep=False, base=RIGD_BASE, log=print, poll_s=2.0):
    """Hand the drain to rigd (POST /api/drain) and tail GET /api/drain until
    it finishes. rigd owns the monitor suspension, the run/drain mutual
    exclusion and the post-drain ingest — none of which a direct CLI drain
    gets while the daemon is up: its stuck-transfer recovery would restore
    remote mode under our feet (audit 2026-08-23, medium)."""
    r = _req(base + "/api/drain", {"nodes": [node], "keep": bool(keep)},
             timeout=30)
    if not r.get("ok"):
        return r
    log("drain handed to rigd (%s), draining %s" % (base, r.get("draining")))
    seen = None
    while True:
        st = _req(base + "/api/drain", timeout=10)
        if st.get("error"):
            return {"ok": False, "error": "lost rigd mid-drain: %s" % st["error"]}
        cur = (st.get("node"), json.dumps(st.get("last") or {}, sort_keys=True))
        if cur != seen:
            seen = cur
            if st.get("node"):
                log("draining %s ..." % st["node"])
            if st.get("last"):
                last = st["last"]
                log("%s: %d pulled (%.1f GB), %d deleted, %d errors"
                    % (last.get("node"), last.get("pulled", 0),
                       last.get("bytes", 0) / 1e9, last.get("deleted", 0),
                       last.get("errors", 0)))
        if not st.get("active"):
            return {"ok": True, **st}
        time.sleep(poll_s)


class Drainer:
    def __init__(self, node, host, dest=DEFAULT_DEST, log=print, disk_min_mb=2000):
        self.node = node
        self.base = "http://%s:8080" % host
        self.dest = dest
        self.log = log
        self.disk_min_mb = disk_min_mb

    # -- camera control -----------------------------------------------------
    def set_mode(self, mode):
        return _req(self.base + "/api/card/mode", {"mode": mode}, timeout=90)

    def card_list(self, tries=30, delay=3.0):
        # The body indexes the card asynchronously after the mode switch; the
        # list call answers "processing" (0x8D05) or InvalidCalled (0x8402)
        # until it is ready. Retry here too - ilxctl's own retry can time out
        # on a large card before the index lands.
        last = None
        for _ in range(tries):
            d = _req(self.base + "/api/card/list?n=8000", timeout=200)
            if d.get("ok"):
                return d["files"]
            last = d.get("error")
            if not any(code in str(last) for code in ("8d05", "8D05", "8402",
                                                      "Processing", "processing")):
                break
            time.sleep(delay)
        raise RuntimeError("card list failed: %s" % last)

    def card_ready(self):
        return _req(self.base + "/api/card/ready", timeout=10)

    def _wait_index(self, timeout=90):
        # Poll the cheap readiness endpoint instead of a blind sleep: the body
        # publishes "index ready" a beat after entering transfer mode, and on
        # a bad transfer session it never does - so this is bounded and gives
        # up cleanly rather than hanging a list for minutes.
        deadline = time.time() + timeout
        while time.time() < deadline:
            r = self.card_ready()
            if r.get("mode") == "remote":
                raise RuntimeError("camera fell back to remote mode before indexing")
            if r.get("ready"):
                return True
            time.sleep(2.0)
        raise RuntimeError("card index not ready within %ds - the body's "
                           "transfer subsystem is wedged (repeated mode "
                           "switches can do this); power-cycle the camera to "
                           "clear it. Card left untouched" % timeout)

    def pull(self, f):
        return _req(self.base + "/api/card/pull",
                    {"contentId": f["contentId"], "fileId": f["fileId"],
                     "name": f["name"]}, timeout=240)

    def fetch(self, name):
        return _bytes("%s/shot/%s?dir=raw" % (self.base, name))

    def card_delete(self, content_id):
        return _req(self.base + "/api/card/delete",
                    {"confirm": "delete", "contentId": content_id}, timeout=90)

    def pi_delete_raw(self, name):
        return _req(self.base + "/api/shots/delete",
                    {"confirm": "delete", "dir": "raw", "name": name}, timeout=30)

    # -- disk guard ---------------------------------------------------------
    def _host_free_mb(self):
        st = os.statvfs(self.dest if os.path.isdir(self.dest) else os.path.dirname(self.dest) or ".")
        return st.f_bavail * st.f_frsize / (1024 * 1024)

    # -- the drain ----------------------------------------------------------
    def run(self, keep_card=False, formats=(RAW_FORMAT,), limit=None,
            stop=None):
        """Returns a report dict. The unit of work is the CONTENT — one shot,
        i.e. the RAW plus the full-size card JPEG sharing one contentId —
        because /api/card/delete is per-content: pulling only the RAW and then
        deleting the content silently destroyed the never-pulled card JPEG of
        every RAW+JPEG shot, and left ingest with nothing it could read EXIF
        from (audit 2026-08-23, critical). `formats` selects which CONTENTS to
        drain (a content qualifies when ANY of its files matches; the default
        drains contents holding a RAW); EVERY file of a selected content is
        pulled, fsynced and sha256-verified on host disk before the content's
        single card delete, and any failure leaves the whole content on the
        card. `limit` bounds contents, not files.

        `stop` (threading.Event, contract C4): checked at each CONTENT
        boundary. A content already in flight is finished — pull, verify,
        write, delete — so a cancel can never land between pull-verify and
        delete, and never strands half a shot on the host while the other half
        is still on the card. The drain then returns with rep["cancelled"]=True
        and everything it did not reach still on the card.

        Counts are per CONTENT (one shot): pulled/verified/deleted count
        contents, rep["files"] lists every file actually written, and
        rep["bytes"] is their total.

        Per-node staging (~/rig-raw/<node>): the two Sony bodies reuse the
        same DSC/_CA counters and fire together, so their card filenames
        collide; a single shared directory let a same-named file from cam1
        stand in for cam2's and its card original be deleted unverified. Kept
        strictly separate here (audit 2026-08-23, critical). And the WHOLE
        drain - mode switch, indexing, listing, pulling - is inside one
        try/finally that always restores shooting mode, so a network blip
        during indexing can never leave the body stuck in transfer mode
        (audit 2026-08-23, critical)."""
        if os.path.basename(self.dest.rstrip(os.sep)) != self.node:
            # idempotent: a reused Drainer must not nest dest/<node>/<node>
            self.dest = os.path.join(self.dest, self.node)
        os.makedirs(self.dest, exist_ok=True)
        rep = {"node": self.node, "pulled": 0, "bytes": 0, "verified": 0,
               "deleted": 0, "skipped": 0, "errors": [], "files": [],
               "cancelled": False}
        try:
            files = None
            for attempt in range(2):
                self.log("[%s] entering transfer mode%s"
                         % (self.node, " (retry)" if attempt else ""))
                m = self.set_mode("transfer")
                if not m.get("ok"):
                    rep["errors"].append("mode switch failed: %s" % m.get("error"))
                    return rep
                try:
                    self._wait_index()
                    files = self.card_list()
                    break
                except Exception as e:  # noqa: BLE001 - timeout/URLError too
                    self.log("[%s] %s" % (self.node, e))
                    self.set_mode("remote")        # clean the session, then retry
                    time.sleep(2.0)
                    if attempt == 1:
                        rep["errors"].append(str(e))
                        return rep
            # Group the listing by contentId: the card API deletes whole
            # contents, so the drain must pull whole contents.
            contents = {}
            for f in files:
                contents.setdefault(f["contentId"], []).append(f)
            wanted = [fl for _, fl in sorted(contents.items())
                      if any(f["format"] in formats for f in fl)]
            rep["skipped"] = len(contents) - len(wanted)
            if limit:
                wanted = wanted[:limit]
            self.log("[%s] card holds %d contents (%d files), %d contents to drain"
                     % (self.node, len(contents), len(files), len(wanted)))
            for flist in wanted:
                if stop is not None and stop.is_set():
                    rep["cancelled"] = True
                    self.log("[%s] drain cancelled - remaining contents left "
                             "on the card" % self.node)
                    break
                if self._host_free_mb() < self.disk_min_mb:
                    rep["errors"].append("host disk below %d MB - stopping"
                                         % self.disk_min_mb)
                    self.log("[%s] STOP: host disk low" % self.node)
                    break
                self._drain_content(flist[0]["contentId"], flist, rep,
                                    keep_card)
            return rep
        finally:
            self.log("[%s] restoring remote mode" % self.node)
            try:
                self.set_mode("remote")
            except Exception as e:  # noqa: BLE001
                self.log("[%s] WARNING could not restore remote mode: %s"
                         % (self.node, e))

    def _drain_content(self, cid, flist, rep, keep_card):
        """Pull+verify every file of one content; delete the content only when
        ALL of them are safe on host disk. Once started, a content runs to the
        end (no cancel check in here): the card delete is per-content, so a
        content is the atomic unit — abandoning it midway would leave one half
        of the shot on the host and the whole shot still on the card. A
        partial FAILURE (one file's transfer died) does keep the files that
        verified — pure gain, the next drain re-verifies them in place — but
        never costs the card original."""
        staged = []          # (tmp, card_name, sha256, size)
        all_ok = True
        try:
            for f in flist:
                # ALWAYS pull and verify before deleting the card original -
                # no trust-by-size skip. A resumed drain re-pulls (a file is
                # <1 s); trusting a same-size local file was how an unverified
                # card original could be deleted (audit, critical). A verified
                # local copy is cheap to re-confirm and never lost.
                try:
                    pr = self.pull(f)
                except Exception as e:  # noqa: BLE001
                    rep["errors"].append("%s pull: %s" % (f["name"], e))
                    all_ok = False
                    continue
                if not pr.get("ok"):
                    rep["errors"].append("%s pull: %s" % (f["name"], pr.get("error")))
                    all_ok = False
                    continue
                try:
                    data = self.fetch(f["name"])
                except Exception as e:  # noqa: BLE001
                    rep["errors"].append("%s fetch: %s" % (f["name"], e))
                    all_ok = False
                    continue
                host_sha = hashlib.sha256(data).hexdigest()
                if host_sha != pr.get("sha256") or len(data) != f["size"]:
                    rep["errors"].append(
                        "%s HASH MISMATCH (pi %s host %s, %d/%d B) - kept on card"
                        % (f["name"], (pr.get("sha256") or "")[:12],
                           host_sha[:12], len(data), f["size"]))
                    all_ok = False
                    continue
                # Durability before deletion: the card is the only backstop
                # for the irreplaceable frame, so the host copy MUST survive a
                # crash before the card copy is erased. fsync now, into a
                # content-unique .part, and only os.replace to the final name
                # once the WHOLE content verified — the final-name choice
                # (dedup below) needs every sibling's hash known first.
                tmp = os.path.join(self.dest, "%s.part-c%s" % (f["name"], cid))
                try:
                    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
                    try:
                        os.write(fd, data)
                        os.fsync(fd)
                    finally:
                        os.close(fd)
                except OSError as e:
                    rep["errors"].append("%s host write: %s" % (f["name"], e))
                    all_ok = False
                    continue
                staged.append((tmp, f["name"], host_sha, len(data)))
            if not staged:
                return
            # The bodies reuse DSC/_CA counters (file-number reset, wrap past
            # 9999), so the same card NAME can recur with different bytes in
            # one drain or across drains — and the old name-keyed host path
            # silently replaced an earlier verified RAW whose card original
            # was already gone (audit 2026-08-23, high). Never overwrite a
            # verified host file: when any final name already holds DIFFERENT
            # content, every file of this content takes a -c<contentId>
            # suffix (all of them, so the RAW/JPEG stems keep pairing up for
            # ingest). Identical content is simply re-verified in place.
            suffix = ""
            for tmp, name, sha, _ in staged:
                dst = os.path.join(self.dest, name)
                if os.path.exists(dst) and _file_sha256(dst) != sha:
                    suffix = "-c%s" % cid
                    break
            if suffix:
                for tmp, name, sha, _ in staged:
                    stem, ext = os.path.splitext(name)
                    dst = os.path.join(self.dest, stem + suffix + ext)
                    if os.path.exists(dst) and _file_sha256(dst) != sha:
                        # same name AND same contentId yet different bytes: pin
                        # this copy by its own hash rather than lose either
                        suffix = "-c%s-%s" % (cid, sha[:8])
                        break
            wrote = 0
            for tmp, name, sha, size in staged:
                stem, ext = os.path.splitext(name)
                final = stem + suffix + ext
                try:
                    os.replace(tmp, os.path.join(self.dest, final))
                except OSError as e:
                    rep["errors"].append("%s host write: %s" % (name, e))
                    all_ok = False
                    continue
                wrote += 1
                rep["bytes"] += size
                rep["files"].append(final)
                self.pi_delete_raw(name)       # free the Pi's -raw staging
            if wrote:
                try:
                    dirfd = os.open(self.dest, os.O_RDONLY)
                    try:
                        os.fsync(dirfd)
                    finally:
                        os.close(dirfd)
                except OSError as e:
                    rep["errors"].append("dir fsync: %s" % e)
                    all_ok = False
            if not all_ok:
                return
            rep["pulled"] += 1
            rep["verified"] += 1
            if not keep_card:
                dr = self.card_delete(cid)
                if dr.get("ok"):
                    rep["deleted"] += 1
                else:
                    rep["errors"].append("%s card delete: %s"
                                         % (flist[0]["name"], dr.get("error")))
        finally:
            for tmp, _, _, _ in staged:
                try:
                    if os.path.exists(tmp):
                        os.unlink(tmp)
                except OSError:
                    pass


def drain_node(node, host=None, dest=DEFAULT_DEST, keep_card=False, log=print,
               limit=None):
    host = host or NODES.get(node)
    if not host:
        raise ValueError("unknown node %s" % node)
    return Drainer(node, host, dest=dest, log=log).run(keep_card=keep_card,
                                                        limit=limit)


# ---------------------------------------------------------------------------
def _selftest():
    # Verifies the hash-gates delete logic against a stub camera, with no
    # hardware: a mismatch must keep the file, a match must delete it. Content
    # A is a real RAW+JPEG shot (two files, one contentId) so the per-CONTENT
    # rule is exercised here too - the card delete takes both halves, so both
    # halves must be on host disk first.
    calls = {"deleted": [], "pi_deleted": []}
    good = b"RAW-BYTES" * 4096
    good_sha = hashlib.sha256(good).hexdigest()
    jpg = b"JPEG-BYTES" * 512
    jpg_sha = hashlib.sha256(jpg).hexdigest()
    bad = b"CORRUPT" * 100

    class Stub(Drainer):
        def __init__(self):
            super().__init__("camX", "0.0.0.0", dest="/tmp/drain-selftest", log=lambda *a: None)
            self.mode = None
            os.makedirs(self.dest, exist_ok=True)
        def set_mode(self, mode): self.mode = mode; return {"ok": True, "mode": mode}
        def card_ready(self): return {"ok": True, "ready": True, "mode": self.mode or "transfer"}
        def card_list(self):
            return [{"contentId": 1, "fileId": 1, "name": "A.JPG", "size": len(jpg), "format": JPEG_FORMAT},
                    {"contentId": 1, "fileId": 2, "name": "A.ARW", "size": len(good), "format": RAW_FORMAT},
                    {"contentId": 2, "fileId": 2, "name": "B.ARW", "size": len(good), "format": RAW_FORMAT}]
        def pull(self, f):
            # A's two files verify; B reports a hash that will not match what
            # fetch returns
            return {"ok": True, "sha256": jpg_sha if f["name"].endswith(".JPG")
                    else good_sha}
        def fetch(self, name):
            if name == "A.JPG":
                return jpg
            return good if name == "A.ARW" else bad     # B's bytes are corrupt
        def card_delete(self, cid): calls["deleted"].append(cid); return {"ok": True}
        def pi_delete_raw(self, name): calls["pi_deleted"].append(name); return {"ok": True}
        def _host_free_mb(self): return 999999

    s = Stub()
    rep = s.run()
    assert rep["pulled"] == 1, rep            # only content A survived
    assert rep["deleted"] == 1, rep
    assert calls["deleted"] == [1], calls     # only A's content deleted
    assert any("B.ARW HASH MISMATCH" in e for e in rep["errors"]), rep
    assert os.path.exists(os.path.join(s.dest, "A.ARW"))
    assert os.path.exists(os.path.join(s.dest, "A.JPG"))   # the card JPEG too
    assert not os.path.exists(os.path.join(s.dest, "B.ARW"))
    assert s.mode == "remote"                 # restored even though B failed
    import shutil
    shutil.rmtree(s.dest)
    print("DRAIN SELF-TEST: PASS (whole-content pull, hash-gated delete, "
          "corrupt file kept on card, mode restored, disk guard present)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("node", nargs="?")
    ap.add_argument("--host")
    ap.add_argument("--dest", default=DEFAULT_DEST)
    ap.add_argument("--keep", action="store_true", help="verify but do not delete from the card")
    ap.add_argument("--limit", type=int, help="drain at most this many contents")
    ap.add_argument("--standalone", action="store_true",
                    help="drive the body directly even when rigd is running "
                         "(stop rigd first, or its monitor fights the drain)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest(); return
    if not a.node:
        ap.error("node is required")
    # A direct drain while rigd is up is DOA: rigd's stuck-transfer recovery
    # restores remote mode within one ~2 s monitor poll, failing the index
    # wait or every pull thereafter (audit 2026-08-23, medium). Route through
    # the daemon when one answers. CLI-only probe — never taken by tests or
    # library callers.
    if not a.standalone and not a.host and _rigd_alive():
        if a.dest != DEFAULT_DEST or a.limit:
            print("rigd is running on this host: --dest/--limit need "
                  "--standalone (stop rigd first)")
            sys.exit(2)
        rep = _drain_via_rigd(a.node, keep=a.keep)
        print(json.dumps(rep, indent=1))
        sys.exit(0 if rep.get("ok") else 1)
    t0 = time.time()
    rep = drain_node(a.node, host=a.host, dest=a.dest, keep_card=a.keep, limit=a.limit)
    dt = time.time() - t0
    print(json.dumps(rep, indent=1))
    # counts are per CONTENT (a shot = its RAW + its card JPEG); files is the
    # per-file count actually written to the host.
    print("drained %d shots (%d files), %.1f GB, %d deleted from the card, "
          "%d errors in %.0f s (%.1f MB/s)"
          % (rep["pulled"], len(rep["files"]), rep["bytes"] / 1e9,
             rep["deleted"], len(rep["errors"]), dt,
             rep["bytes"] / 1e6 / dt if dt else 0))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Card drain — pull the RAW archive off each camera's card to the host,
verify it, delete it from the card, and hand the RAWs to ingest.

The survey shoots RAW+JPEG to the card with a Small JPEG delivered live; the
full-resolution RAW never crosses USB during the run (it cannot, at survey
cadence). This drains it BETWEEN runs, over the SDK's Remote Transfer path:

    list the card  ->  pull each RAW to the host  ->  sha256 must match  ->
    delete that shot from the card  ->  ingest the pulled RAWs into runs

Each file is deleted from the card ONLY after its host copy's hash matches the
Pi's — so a card never fills across a season, and a failed transfer never
loses the original. The camera is switched to RemoteTransfer control mode for
the drain (it cannot shoot in that mode) and back to remote afterwards, so a
drain must never overlap a run — rigd refuses to start one while a run is
active, and refuses a run while a drain is active.

Used from rigd (POST /api/drain, and auto after a run when armed) or stand
alone:

    python3 rig/drain.py cam1                     # drain cam1 now
    python3 rig/drain.py cam1 --dest ~/rig-raw --keep   # pull+verify, keep card
    python3 rig/drain.py --selftest
"""
import argparse
import hashlib
import json
import os
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
    def run(self, keep_card=False, formats=(RAW_FORMAT,), limit=None):
        """Returns a report dict. Deletes each card content only after its RAW
        copy is on host disk, fsynced, AND its sha256 matches the camera's; on
        any failure that file is left on the card.

        Per-node staging (~/rig-raw/<node>): the two Sony bodies reuse the
        same DSC/_CA counters and fire together, so their card filenames
        collide; a single shared directory let a same-named file from cam1
        stand in for cam2's and its card original be deleted unverified. Kept
        strictly separate here (audit 2026-08-23, critical). And the WHOLE
        drain - mode switch, indexing, listing, pulling - is inside one
        try/finally that always restores shooting mode, so a network blip
        during indexing can never leave the body stuck in transfer mode
        (audit 2026-08-23, critical)."""
        self.dest = os.path.join(self.dest, self.node)
        os.makedirs(self.dest, exist_ok=True)
        rep = {"node": self.node, "pulled": 0, "bytes": 0, "verified": 0,
               "deleted": 0, "skipped": 0, "errors": [], "files": []}
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
            wanted = [f for f in files if f["format"] in formats]
            if limit:
                wanted = wanted[:limit]
            self.log("[%s] card holds %d files, %d to drain"
                     % (self.node, len(files), len(wanted)))
            for f in wanted:
                if self._host_free_mb() < self.disk_min_mb:
                    rep["errors"].append("host disk below %d MB - stopping"
                                         % self.disk_min_mb)
                    self.log("[%s] STOP: host disk low" % self.node)
                    break
                dst = os.path.join(self.dest, f["name"])
                # ALWAYS pull and verify before deleting the card original - no
                # trust-by-size skip. A resumed drain re-pulls (a RAW is <1 s);
                # trusting a same-size local file was how an unverified card
                # original could be deleted (audit, critical). A verified local
                # copy is cheap to re-confirm and never lost.
                try:
                    pr = self.pull(f)
                except Exception as e:  # noqa: BLE001
                    rep["errors"].append("%s pull: %s" % (f["name"], e))
                    continue
                if not pr.get("ok"):
                    rep["errors"].append("%s pull: %s" % (f["name"], pr.get("error")))
                    continue
                try:
                    data = self.fetch(f["name"])
                except Exception as e:  # noqa: BLE001
                    rep["errors"].append("%s fetch: %s" % (f["name"], e))
                    continue
                host_sha = hashlib.sha256(data).hexdigest()
                if host_sha != pr.get("sha256") or len(data) != f["size"]:
                    rep["errors"].append(
                        "%s HASH MISMATCH (pi %s host %s, %d/%d B) - kept on card"
                        % (f["name"], (pr.get("sha256") or "")[:12],
                           host_sha[:12], len(data), f["size"]))
                    continue
                # Durability before deletion: the card is the only backstop for
                # the irreplaceable RAW, so the host copy MUST survive a crash
                # before the card copy is erased. fsync the file and its
                # directory (audit, critical).
                tmp = dst + ".part"
                try:
                    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
                    try:
                        os.write(fd, data)
                        os.fsync(fd)
                    finally:
                        os.close(fd)
                    os.replace(tmp, dst)
                    dirfd = os.open(self.dest, os.O_RDONLY)
                    try:
                        os.fsync(dirfd)
                    finally:
                        os.close(dirfd)
                except OSError as e:
                    rep["errors"].append("%s host write: %s" % (f["name"], e))
                    continue
                rep["pulled"] += 1
                rep["bytes"] += len(data)
                self.pi_delete_raw(f["name"])       # free the Pi's -raw staging
                rep["verified"] += 1
                rep["files"].append(f["name"])
                if not keep_card:
                    dr = self.card_delete(f["contentId"])
                    if dr.get("ok"):
                        rep["deleted"] += 1
                    else:
                        rep["errors"].append("%s card delete: %s"
                                             % (f["name"], dr.get("error")))
            return rep
        finally:
            self.log("[%s] restoring remote mode" % self.node)
            try:
                self.set_mode("remote")
            except Exception as e:  # noqa: BLE001
                self.log("[%s] WARNING could not restore remote mode: %s"
                         % (self.node, e))


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
    # hardware: a mismatch must keep the file, a match must delete it.
    import io
    calls = {"deleted": [], "pi_deleted": []}
    good = b"RAW-BYTES" * 4096
    good_sha = hashlib.sha256(good).hexdigest()
    bad = b"CORRUPT" * 100

    class Stub(Drainer):
        def __init__(self):
            super().__init__("camX", "0.0.0.0", dest="/tmp/drain-selftest", log=lambda *a: None)
            self.mode = None
            os.makedirs(self.dest, exist_ok=True)
        def set_mode(self, mode): self.mode = mode; return {"ok": True, "mode": mode}
        def card_ready(self): return {"ok": True, "ready": True, "mode": self.mode or "transfer"}
        def card_list(self):
            return [{"contentId": 1, "fileId": 2, "name": "A.ARW", "size": len(good), "format": RAW_FORMAT},
                    {"contentId": 2, "fileId": 2, "name": "B.ARW", "size": len(good), "format": RAW_FORMAT}]
        def pull(self, f):
            # A verifies; B reports a hash that will not match what fetch returns
            return {"ok": True, "sha256": good_sha if f["name"] == "A.ARW" else good_sha}
        def fetch(self, name):
            return good if name == "A.ARW" else bad     # B's bytes are corrupt
        def card_delete(self, cid): calls["deleted"].append(cid); return {"ok": True}
        def pi_delete_raw(self, name): calls["pi_deleted"].append(name); return {"ok": True}
        def _host_free_mb(self): return 999999

    s = Stub()
    rep = s.run()
    assert rep["pulled"] == 1, rep            # only A survived
    assert rep["deleted"] == 1, rep
    assert calls["deleted"] == [1], calls     # only A's content deleted
    assert any("B.ARW HASH MISMATCH" in e for e in rep["errors"]), rep
    assert os.path.exists(os.path.join(s.dest, "A.ARW"))
    assert not os.path.exists(os.path.join(s.dest, "B.ARW"))
    assert s.mode == "remote"                 # restored even though B failed
    import shutil
    shutil.rmtree(s.dest)
    print("DRAIN SELF-TEST: PASS (hash-gated delete, corrupt file kept on card, "
          "mode restored, disk guard present)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("node", nargs="?")
    ap.add_argument("--host")
    ap.add_argument("--dest", default=DEFAULT_DEST)
    ap.add_argument("--keep", action="store_true", help="verify but do not delete from the card")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest(); return
    if not a.node:
        ap.error("node is required")
    t0 = time.time()
    rep = drain_node(a.node, host=a.host, dest=a.dest, keep_card=a.keep, limit=a.limit)
    dt = time.time() - t0
    print(json.dumps(rep, indent=1))
    print("drained %d files, %.1f GB, %d deleted, %d errors in %.0f s (%.1f MB/s)"
          % (rep["pulled"], rep["bytes"] / 1e9, rep["deleted"], len(rep["errors"]),
             dt, rep["bytes"] / 1e6 / dt if dt else 0))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Post-deploy live validation of the Wild Sync rig. Read-only unless --fire/--run.

    python3 live_validate.py            # passive checks only
    python3 live_validate.py --fire 10  # + N scheduled captures, skew/late stats
    python3 live_validate.py --run 10   # + a 10-frame transect at 2 s (then NO auto-drain unless --drain)
"""
import argparse, json, statistics, sys, time, urllib.request

B = "http://localhost:9090"
OK, WARN, FAIL = [], [], []


def get(p, timeout=10):
    return json.load(urllib.request.urlopen(B + p, timeout=timeout))


def post(p, body, timeout=60):
    req = urllib.request.Request(B + p, data=json.dumps(body).encode(),
                                 headers={"content-type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=timeout))


def check(name, cond, detail=""):
    (OK if cond else FAIL).append(name)
    print("  %s %s%s" % ("PASS" if cond else "FAIL", name, (" — " + detail) if detail else ""))
    return cond


def warn(name, cond, detail=""):
    (OK if cond else WARN).append(name)
    print("  %s %s%s" % ("ok  " if cond else "WARN", name, (" — " + detail) if detail else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fire", type=int, default=0)
    ap.add_argument("--run", type=int, default=0)
    ap.add_argument("--drain", action="store_true")
    a = ap.parse_args()

    print("== fleet")
    f = get("/api/fleet")
    nodes = f["nodes"]
    check("2 cameras connected", sum(n["connected"] for n in nodes) == 2)
    for n in nodes:
        print("  --", n["name"], n["state"])
        check("%s convergence synced" % n["name"], n["convergence"]["synced"],
              str(n["convergence"]["diverged"]))
        check("%s MF" % n["name"], n.get("focus_mode") == 1)
        # format readback (new)
        st = get("/api/status?node=" + n["name"]).get("status", {})
        for k in ("filetypeValue", "imagesizeValue", "transsizeValue", "rawtypeValue"):
            check("%s has %s" % (n["name"], k), k in st, str(st.get(k)))
        print("     format: %s %s %s %s pcsave=%s quality=%s remain=%s" % (
            st.get("filetypeLabel"), st.get("imagesizeLabel"), st.get("transsizeLabel"),
            st.get("rawtypeLabel"), st.get("pcsaveLabel"), st.get("qualityLabel"),
            st.get("remainingShots")))
        warn("%s slot idle" % n["name"], st.get("slotWritingLabel") == "idle")
        warn("%s not overheating" % n["name"], st.get("overheatingLabel") == "ok")
    # format identical across bodies
    sts = [get("/api/status?node=" + n["name"]).get("status", {}) for n in nodes]
    for k in ("filetypeValue", "imagesizeValue", "transsizeValue", "rawtypeValue",
              "whiteBalance", "colorTemp", "isoValue", "shutterValue", "apertureValue"):
        vals = [s.get(k) for s in sts]
        check("bodies agree on %s" % k, len(set(map(str, vals))) == 1, str(vals))

    print("== clocks")
    for n in nodes:
        off = n.get("clock_offset_s", n.get("clock_offset_ms", 0) and n["clock_offset_ms"] / 1e3)
        print("  %s offset %.1f ms rtt %.1f ms" % (n["name"], (off or 0) * 1e3, n.get("clock_rtt_ms") or -1))
    an = get("/api/anomalies")["anomalies"]
    kinds = [x["kind"] for x in an]
    print("  anomalies:", kinds or "none")
    warn("no node_clock_skew", "node_clock_skew" not in kinds)

    if a.fire:
        print("== %d scheduled fires" % a.fire)
        skews, lates = [], []
        for i in range(a.fire):
            r = post("/api/capture", {})
            res = r.get("results", {})
            sk = r.get("skew_ms")
            lt = [v.get("late_ms") for v in res.values() if v.get("late_ms") is not None]
            if sk is not None:
                skews.append(sk)
            lates += lt
            print("  #%d skew=%s late=%s ok=%s" % (i, sk, [round(x, 1) for x in lt],
                                                   [v.get("ok") for v in res.values()]))
            time.sleep(1.2)
        if skews:
            check("skew mean < 2 ms", statistics.mean(skews) < 2.0,
                  "mean %.2f max %.2f" % (statistics.mean(skews), max(skews)))
            check("late mean < 5 ms", statistics.mean(lates) < 5.0,
                  "mean %.1f max %.1f (schedule honoured)" % (statistics.mean(lates), max(lates)))

    if a.run:
        print("== transect %d frames" % a.run)
        r = post("/api/run/start", {"label": "validate", "interval_s": 2.0,
                                     "frames": a.run, "confirm_warnings": True,
                                     "drain": bool(a.drain)})
        if not check("run started", r.get("ok"), str(r.get("error"))):
            sys.exit(1)
        rid = r["run_id"]
        t0 = time.time()
        while time.time() - t0 < a.run * 2 + 60:
            time.sleep(3)
            fr = get("/api/fleet")["run"]
            if not fr.get("active"):
                break
            print("  frames", fr.get("frames"), "sync", fr.get("sync", {}).get("shots"))
        st = post("/api/run/stop", {})
        print("  stop:", {k: (v.get("pulled"), v.get("failed")) for k, v in (st.get("summary") or {}).items()})
        d = get("/api/run/detail?id=" + rid)
        pc, pt = d.get("pairs_complete"), d.get("pairs_total")
        print("  pairs %s/%s skew_max %s" % (pc, pt, (d.get("sync") or {}).get("skew_ms_max")))
        check("all pairs complete", pc == pt and (pt or 0) >= a.run - 1, "%s/%s" % (pc, pt))

    print("\n%d pass, %d warn, %d fail" % (len(OK), len(WARN), len(FAIL)))
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()

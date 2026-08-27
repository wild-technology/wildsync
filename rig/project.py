"""Project layer — one named container per survey campaign.

A project owns the two data trees everything else writes into:

    runs_dir   transect run directories (frames, flight_log.csv, events.log)
    raw_dir    card-drain staging (raw/<node>/ ARWs until ingest files them)

plus project.json (operator metadata) and exports/. The layout:

    ~/wildsync-projects/<slug>/
        project.json
        runs/
        raw/
        exports/
    ~/wildsync-projects/active.json     -> {"slug": "..."}

WHY a "legacy" project exists: before this layer, runs lived in ~/rig-runs and
drained RAW in ~/rig-raw, and there are finished surveys in both. Moving user
data during an upgrade is how data gets lost, so the pre-project layout is not
migrated - it is WRAPPED: when no active.json exists (or it names "legacy"),
the active project is an implicit one whose runs_dir/raw_dir are the old
paths. rigd behaves byte-for-byte as before until the operator deliberately
creates a project. "legacy" is reserved and cannot be created or deleted.

WHY switching mutates rigcore.RUNS_DIR instead of everyone calling a getter:
the offline suites monkeypatch rigcore.RUNS_DIR to a temp dir (see
rigcore.RunsIndex), and every consumer except run.py already reads it as a
module attribute per call. Assigning the attribute on open() keeps both the
tests and the consumers working unchanged; run.py was switched to attribute
reads. The one rule: only open()/startup() assign it.

Host-portable by construction (Mac today, Jetson Orin tomorrow): every path
derives from $HOME, nothing platform-specific.
"""

import json
import os
import re
import threading
import time

import rigcore

PROJECTS_ROOT = os.path.expanduser("~/wildsync-projects")
ACTIVE_PATH = os.path.join(PROJECTS_ROOT, "active.json")
SCHEMA = 1

# The implicit wrapper for the pre-project layout. Reserved slug.
LEGACY_SLUG = "legacy"
LEGACY_RUNS = os.path.expanduser("~/rig-runs")
LEGACY_RAW = os.path.expanduser("~/rig-raw")

_lock = threading.RLock()
_active = None            # cached active project doc, or None before startup()


class ProjectError(Exception):
    """Operator-facing refusal (bad name, unknown slug, reserved word)."""


def _atomic_write(path, doc):
    # tmp + fsync + rename: a crash mid-write must never leave a half JSON
    # where the next startup expects project state (same discipline as the
    # run manifest).
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def _slugify(name):
    s = re.sub(r"[^a-zA-Z0-9._-]+", "-", name.strip()).strip("-._").lower()
    return s[:60]


def _legacy_doc():
    return {
        "slug": LEGACY_SLUG, "name": "Legacy (pre-project data)",
        "schema": SCHEMA, "created": None, "legacy": True,
        "runs_dir": LEGACY_RUNS, "raw_dir": LEGACY_RAW,
        "exports_dir": os.path.join(PROJECTS_ROOT, LEGACY_SLUG + "-exports"),
        "vessel": "", "site": "", "operator": "", "notes": "",
    }


def _doc_path(slug):
    return os.path.join(PROJECTS_ROOT, slug, "project.json")


def _load_doc(slug):
    if slug == LEGACY_SLUG:
        return _legacy_doc()
    p = _doc_path(slug)
    with open(p) as fh:
        doc = json.load(fh)
    if not isinstance(doc, dict) or doc.get("slug") != slug:
        raise ProjectError("project.json for %r is malformed" % slug)
    root = os.path.join(PROJECTS_ROOT, slug)
    # Paths are DERIVED from the slug, never trusted from the file: a stale or
    # hand-edited project.json must not silently point the rig at some other
    # directory tree. legacy is the one deliberate exception above.
    doc["runs_dir"] = os.path.join(root, "runs")
    doc["raw_dir"] = os.path.join(root, "raw")
    doc["exports_dir"] = os.path.join(root, "exports")
    return doc


def _apply(doc):
    """Point the rig's storage at this project. Caller holds _lock."""
    global _active
    os.makedirs(doc["runs_dir"], exist_ok=True)
    os.makedirs(doc["raw_dir"], exist_ok=True)
    rigcore.RUNS_DIR = doc["runs_dir"]
    _active = doc


def startup():
    """Resolve and apply the active project at rigd start.

    Any failure (unreadable active.json, project dir deleted underneath it)
    falls back to legacy rather than refusing to boot: on a boat, a rig that
    starts with the old layout beats one that will not start."""
    with _lock:
        slug = LEGACY_SLUG
        try:
            with open(ACTIVE_PATH) as fh:
                slug = (json.load(fh) or {}).get("slug") or LEGACY_SLUG
        except OSError:
            pass
        except Exception:
            slug = LEGACY_SLUG
        try:
            doc = _load_doc(slug)
        except Exception:
            doc = _legacy_doc()
        _apply(doc)
        return dict(_active)


def active():
    with _lock:
        if _active is None:
            startup()
        return dict(_active)


def explicit_active():
    """True once the operator has ever chosen a project (active.json exists).
    The UI uses this to decide whether to show the first-run intro screen."""
    return os.path.exists(ACTIVE_PATH)


def runs_dir():
    return active()["runs_dir"]


def raw_dir():
    return active()["raw_dir"]


def list_projects():
    """All projects, newest first, active flagged. Cheap by design: run
    counts come from a shallow listdir, never a recursive walk of imagery."""
    out = []
    with _lock:
        act = active()["slug"]
    slugs = []
    try:
        for d in os.listdir(PROJECTS_ROOT):
            if os.path.isfile(_doc_path(d)):
                slugs.append(d)
    except OSError:
        pass
    for slug in slugs:
        try:
            doc = _load_doc(slug)
        except Exception:
            continue
        out.append(_summarize(doc, act))
    out.sort(key=lambda p: p.get("created") or 0, reverse=True)
    # Legacy is listed exactly when it holds anything, so a fresh install is
    # not confused by an empty wrapper and an upgraded one never loses sight
    # of pre-project surveys.
    legacy = _legacy_doc()
    if _dir_count(legacy["runs_dir"]) or _dir_count(os.path.join(legacy["raw_dir"], "cam1")) \
            or _dir_count(os.path.join(legacy["raw_dir"], "cam2")) or act == LEGACY_SLUG:
        out.append(_summarize(legacy, act))
    return out


def _dir_count(path):
    try:
        return len(os.listdir(path))
    except OSError:
        return 0


def _summarize(doc, active_slug):
    runs = []
    try:
        runs = [d for d in os.listdir(doc["runs_dir"])
                if os.path.isdir(os.path.join(doc["runs_dir"], d))]
    except OSError:
        pass
    return {
        "slug": doc["slug"], "name": doc.get("name") or doc["slug"],
        "active": doc["slug"] == active_slug,
        "legacy": bool(doc.get("legacy")),
        "created": doc.get("created"),
        "vessel": doc.get("vessel", ""), "site": doc.get("site", ""),
        "operator": doc.get("operator", ""), "notes": doc.get("notes", ""),
        "runs": len(runs), "last_run": max(runs) if runs else None,
        "runs_dir": doc["runs_dir"], "raw_dir": doc["raw_dir"],
    }


def create(name, vessel="", site="", operator="", notes="", open_it=True):
    name = (name or "").strip()
    if not name:
        raise ProjectError("a project needs a name")
    slug = _slugify(name)
    if not slug:
        raise ProjectError("the name %r leaves nothing usable for a "
                           "directory name" % name)
    if slug == LEGACY_SLUG:
        raise ProjectError("'legacy' is reserved for the pre-project data")
    with _lock:
        if os.path.exists(_doc_path(slug)):
            raise ProjectError("a project named %r already exists" % slug)
        doc = {
            "slug": slug, "name": name, "schema": SCHEMA,
            "created": time.time(),
            "vessel": vessel, "site": site, "operator": operator,
            "notes": notes,
        }
        _atomic_write(_doc_path(slug), doc)
        for sub in ("runs", "raw", "exports"):
            os.makedirs(os.path.join(PROJECTS_ROOT, slug, sub), exist_ok=True)
        if open_it:
            return open_(slug)
        return _load_doc(slug)


def open_(slug):
    with _lock:
        doc = _load_doc(slug)       # raises on unknown/malformed
        _atomic_write(ACTIVE_PATH, {"slug": slug, "opened": time.time()})
        _apply(doc)
        return dict(doc)


def update(slug, fields):
    """Edit operator metadata. Identity and layout are immutable."""
    allowed = {"name", "vessel", "site", "operator", "notes"}
    bad = set(fields) - allowed
    if bad:
        raise ProjectError("cannot edit %s" % ", ".join(sorted(bad)))
    with _lock:
        if slug == LEGACY_SLUG:
            raise ProjectError("the legacy wrapper has no editable metadata")
        doc = _load_doc(slug)
        stored = {k: v for k, v in doc.items()
                  if k not in ("runs_dir", "raw_dir", "exports_dir")}
        for k, v in fields.items():
            stored[k] = str(v)
        _atomic_write(_doc_path(slug), stored)
        doc = _load_doc(slug)
        if _active and _active.get("slug") == slug:
            _apply(doc)
        return dict(doc)

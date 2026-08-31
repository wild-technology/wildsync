"""Populate an Unreal level with Cesium ion tilesets, snapped to an ion terrain.

Run from the Unreal editor's Python console:

    import cesium_populate; cesium_populate.run("config.json")

The level, its CesiumGeoreference and its terrain tileset must already exist —
this script never creates or moves the georeference, because doing so would
slide every tileset relative to terrain that was authored in Unreal
coordinates. It only adds model tilesets and corrects their height.

This must run in an editor that is ticking. Cesium's height sampler is
asynchronous: it loads the terrain tiles it needs on demand and calls back
several frames later, so the work is driven from a post-tick callback rather
than running straight through. A `-run=pythonscript` commandlet has no tick
loop and will time out at the sampling step.
"""

import json
import os
import statistics
import time

import unreal

import ion_locate

# The async action and its tick handle have to outlive run(); nothing else in
# the editor holds a reference and they would otherwise be collected mid-flight.
_PENDING = {}


# --------------------------------------------------------------------------
# level inspection
# --------------------------------------------------------------------------

def _actors():
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()


def require_georeference():
    """The level's existing origin. Never modified — see the module docstring."""
    found = [a for a in _actors() if isinstance(a, unreal.CesiumGeoreference)]
    if not found:
        raise RuntimeError(
            "no CesiumGeoreference in this level. Add one and set its origin to "
            "match the terrain before running this script."
        )
    if len(found) > 1:
        unreal.log_warning(
            "%d CesiumGeoreference actors found; using %s"
            % (len(found), found[0].get_actor_label())
        )
    return found[0]


def find_tileset(asset_id=None, label=None):
    for a in _actors():
        if not isinstance(a, unreal.Cesium3DTileset):
            continue
        if asset_id is not None and int(a.get_editor_property("ion_asset_id")) == int(asset_id):
            return a
        if label is not None and a.get_actor_label() == label:
            return a
    return None


def require_terrain(cfg):
    terrain = find_tileset(
        asset_id=cfg.get("terrain_ion_asset_id"), label=cfg.get("terrain_actor_label")
    )
    if terrain is None:
        raise RuntimeError(
            "terrain tileset not found. Set terrain_ion_asset_id or "
            "terrain_actor_label in the config to a Cesium3DTileset already in "
            "the level."
        )
    if not terrain.get_editor_property("create_physics_meshes"):
        # Height sampling does not need physics meshes, but anything else that
        # traces against the seafloor will, and it is cheap to notice here.
        unreal.log_warning(
            "terrain '%s' has CreatePhysicsMeshes off — height sampling still "
            "works, but line traces against it will not hit anything."
            % terrain.get_actor_label()
        )
    return terrain


# --------------------------------------------------------------------------
# authoring
# --------------------------------------------------------------------------

def ensure_tileset(entry, georef, template=None):
    """Spawn or rebind one model tileset. Idempotent on the ion asset ID.

    The actor is created at the origin: a tileset carries its own ECEF root
    transform, so the georeference alone decides where it lands. The only
    reason this script moves an actor afterwards is the height correction.
    """
    asset_id = int(entry["id"])
    tileset = find_tileset(asset_id=asset_id)
    if tileset is None:
        tileset = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).spawn_actor_from_class(
            template or unreal.Cesium3DTileset,
            unreal.Vector(0.0, 0.0, 0.0),
            unreal.Rotator(0.0, 0.0, 0.0),
        )

    tileset.set_actor_label(str(entry.get("name", asset_id)))
    tileset.set_editor_property("tileset_source", unreal.TilesetSource.FROM_CESIUM_ION)
    tileset.set_editor_property("ion_asset_id", asset_id)
    tileset.set_editor_property("georeference", georef)
    tileset.set_editor_property("create_physics_meshes", True)
    return tileset


def apply_height(tileset, georef, lon, lat, height_from, height_to):
    """Shift a tileset vertically from one ellipsoidal height to another.

    Done as the difference between two georeferenced points rather than a raw
    Z nudge: "up" is only the Unreal +Z axis at the georeference origin, and
    diverges from it with distance. Letting the georeference convert both
    endpoints keeps the offset along the local vertical wherever the site is.
    """
    a = georef.transform_longitude_latitude_height_position_to_unreal(
        unreal.Vector(lon, lat, height_from)
    )
    b = georef.transform_longitude_latitude_height_position_to_unreal(
        unreal.Vector(lon, lat, height_to)
    )
    offset = unreal.Vector(b.x - a.x, b.y - a.y, b.z - a.z)
    tileset.set_actor_location(offset, False, False)
    return offset


# --------------------------------------------------------------------------
# asynchronous height sampling
# --------------------------------------------------------------------------

def sample_heights(terrain, positions, on_complete, timeout_s=300.0):
    """Batch-sample terrain heights at cartographic positions, then call back.

    Cesium's sampler is exposed only as a Blueprint async node: its factory and
    Activate are flagged BlueprintInternalUseOnly, which means the Python
    binding generator gives them no snake_case method. They are still reachable
    through the generic reflection invoker, call_method().
    """
    cls = getattr(unreal, "CesiumSampleHeightMostDetailedAsyncAction", None)
    if cls is None:
        raise RuntimeError(
            "CesiumSampleHeightMostDetailedAsyncAction is not in the unreal "
            "module. Check the Cesium for Unreal plugin is enabled and is "
            "v2.21.0 or later."
        )

    action = unreal.get_default_object(cls).call_method(
        "SampleHeightMostDetailed", (terrain, positions)
    )

    started = time.time()

    def finish(results, warnings):
        if _PENDING.get("handle") is not None:
            unreal.unregister_slate_post_tick_callback(_PENDING.pop("handle"))
        _PENDING.pop("action", None)
        for w in warnings or []:
            unreal.log_warning("height sampling: %s" % w)
        on_complete(results)

    def watchdog(_delta_seconds):
        if time.time() - started > timeout_s:
            handle = _PENDING.pop("handle", None)
            if handle is not None:
                unreal.unregister_slate_post_tick_callback(handle)
            _PENDING.pop("action", None)
            unreal.log_error(
                "height sampling timed out after %.0f s with %d positions. The "
                "editor must stay focused and ticking while tiles load."
                % (timeout_s, len(positions))
            )

    action.on_heights_sampled.add_callable(finish)
    _PENDING["action"] = action
    _PENDING["handle"] = unreal.register_slate_post_tick_callback(watchdog)
    action.call_method("Activate")
    return action


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def _summarise(rows):
    """Report the correction applied, so a datum error stays visible.

    Snapping every model to the terrain hides a systematic height error by
    construction. A tight cluster of similar offsets is the signature of one —
    a geoid/ellipsoid mismatch shifts everything by nearly the same amount,
    whereas genuinely wrong placements scatter.
    """
    deltas = [r["delta_m"] for r in rows if r["sampled"]]
    if not deltas:
        return {"sampled": 0}
    summary = {
        "sampled": len(deltas),
        "mean_m": statistics.fmean(deltas),
        "median_m": statistics.median(deltas),
        "min_m": min(deltas),
        "max_m": max(deltas),
        "stdev_m": statistics.stdev(deltas) if len(deltas) > 1 else 0.0,
    }
    if summary["sampled"] > 2 and summary["stdev_m"] < 1.0 and abs(summary["mean_m"]) > 2.0:
        unreal.log_warning(
            "every asset moved by %.1f m +/- %.1f m. That is a datum offset, "
            "not scattered placement error — worth fixing in ion rather than "
            "re-snapping on every import."
            % (summary["mean_m"], summary["stdev_m"])
        )
    return summary


def run(config_path):
    with open(config_path) as fh:
        cfg = json.load(fh)

    if cfg.get("level"):
        unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(cfg["level"])

    georef = require_georeference()
    terrain = require_terrain(cfg)

    cached = cfg.get("manifest")
    if cached and os.path.exists(cached):
        with open(cached) as fh:
            manifest = json.load(fh)
    else:
        token = os.environ[cfg.get("token_env", "CESIUM_ION_TOKEN")]
        manifest = ion_locate.build_manifest(token, cfg.get("name_pattern"))
        if cached:
            with open(cached, "w") as fh:
                json.dump(manifest, fh, indent=2)

    sites = [m for m in manifest if "error" not in m]
    for bad in [m for m in manifest if "error" in m]:
        unreal.log_warning("could not locate ion asset %s: %s" % (bad["id"], bad["error"]))
    if not sites:
        raise RuntimeError("no locatable ion assets matched %r" % cfg.get("name_pattern"))

    terrain_id = terrain.get_editor_property("ion_asset_id")
    sites = [s for s in sites if int(s["id"]) != int(terrain_id)]

    placed = [(s, ensure_tileset(s, georef)) for s in sites]
    unreal.log("placed %d tilesets; sampling terrain heights" % len(placed))

    positions = [unreal.Vector(s["lon"], s["lat"], s["height"]) for s, _ in placed]

    def on_sampled(results):
        rows = []
        for (site, tileset), result in zip(placed, results):
            sampled = bool(result.get_editor_property("sample_success"))
            llh = result.get_editor_property("longitude_latitude_height")
            row = {
                "id": site["id"],
                "name": site["name"],
                "lon": site["lon"],
                "lat": site["lat"],
                "ion_height_m": site["height"],
                "terrain_height_m": llh.z if sampled else None,
                "delta_m": (llh.z - site["height"]) if sampled else None,
                "sampled": sampled,
            }
            if sampled:
                apply_height(tileset, georef, site["lon"], site["lat"],
                             site["height"], llh.z)
            else:
                unreal.log_warning(
                    "no terrain under '%s' (%.6f, %.6f) — left at its ion height"
                    % (site["name"], site["lat"], site["lon"])
                )
            rows.append(row)

        report = {"summary": _summarise(rows), "assets": rows}
        if cfg.get("report"):
            with open(cfg["report"], "w") as fh:
                json.dump(report, fh, indent=2)
        unreal.log("snapped %d/%d; %s" % (report["summary"].get("sampled", 0),
                                          len(rows), json.dumps(report["summary"])))

        unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
        unreal.log("done — level saved")

    sample_heights(terrain, positions, on_sampled,
                   timeout_s=float(cfg.get("timeout_s", 300)))
    unreal.log("sampling started; keep the editor focused until 'done' appears")

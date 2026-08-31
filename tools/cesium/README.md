# Cesium ion → Unreal 5.7

Populates an existing Unreal level with georeferenced Cesium ion tilesets and
snaps each one onto an ion terrain tileset (our multibeam bathymetry).

    ion  ──discover by name──▶  manifest.json  ──▶  Unreal level
                                     │                  │
                        lon/lat from tileset.json    snap to terrain
                                                     + snap-report.json

## Two halves, on purpose

`ion_locate.py` is stdlib-only and talks to nothing but the ion REST API. It
runs anywhere — CI, a laptop, Unreal's bundled Python. `cesium_populate.py`
needs the editor.

The split matters because ion's `/v1/assets/{id}` returns **no geographic
extent**. The location has to come from the tileset itself: ask ion for a
signed endpoint, fetch `tileset.json`, and reduce `root.transform ×
root.boundingVolume` to a cartographic centre. That is all `ion_locate.py` does,
and it is the part worth testing offline.

## Use

    export CESIUM_ION_TOKEN=...          # assets:list + assets:read, read-only
    python3 ion_locate.py 'Transect_*' > manifest.json

Then in the Unreal editor's Python console (Output Log → Cmd → Python):

    import sys; sys.path.append('/path/to/wildsync/tools/cesium')
    import cesium_populate; cesium_populate.run('config.json')

`name_pattern` is a shell glob, case-insensitive. Prefix with `re:` for a
regular expression.

## What it will not do

**It never creates or moves the `CesiumGeoreference`.** The georeference origin
decides where the whole globe sits relative to anything authored in Unreal
coordinates; moving it slides every tileset relative to the terrain. The level
must already have one, and the script errors out if it does not.

**It must run in a ticking editor.** Cesium's height sampler is asynchronous —
it loads the terrain tiles it needs on demand and calls back several frames
later. A `-run=pythonscript` commandlet has no tick loop and times out at the
sampling step. The discovery half is headless-safe; the snapping half is not.

## Height, and why the report exists

Every model is snapped to the terrain surface, which is what was asked for. It
is also, by construction, a way to hide a systematic error: Cesium heights are
metres above the **WGS 84 ellipsoid**, not depth and not mean sea level, so a
sounding of 32 m becomes roughly `h = N + tide − 32` where `N` is the geoid
separation — tens of metres in most of the world.

If that conversion is wrong, snapping makes it invisible. So
`cesium_populate.py` writes `snap-report.json` with the correction applied to
each asset, and warns when the corrections cluster:

    every asset moved by -37.2 m +/- 0.3 m. That is a datum offset, not
    scattered placement error — worth fixing in ion rather than re-snapping
    on every import.

Scattered deltas are genuine placement error. A tight cluster is a datum bug,
and the right fix is upstream in ion (`options.position` at upload, or the 3D
Tiles Location Editor) so CesiumJS and everything else downstream agree.

## The offset is not a Z nudge

Unreal's +Z is "up" only at the georeference origin. The script converts both
the old and new cartographic positions through the georeference and uses the
difference, so the shift follows the local vertical wherever the site is.
Measured against the reference geodesy in `tests/`, the horizontal component a
raw Z nudge would get wrong:

| distance from origin | error |
|---|---|
| 5 km | 1.6 cm |
| 50 km | 16 cm |
| 200 km | 63 cm |

## Tests

    python3 tests/test_populate.py

`tests/fake_unreal.py` stands in for the editor, with the real ECEF→ESU
geodesy rather than a stub, so the offset assertions mean something. It covers
placement, idempotency, the terrain being excluded from its own population,
unsampled sites being left alone, the datum warning, and both offset cases.

No Unreal required. What the tests **cannot** cover is whether the plugin's
Python bindings match: `SampleHeightMostDetailed` is a Blueprint async node
whose factory and `Activate` are `BlueprintInternalUseOnly`, so they get no
snake_case binding and are reached through `call_method()`. Verify once per
project:

    python3 -c "import unreal; print(hasattr(unreal, 'CesiumSampleHeightMostDetailedAsyncAction'))"

## Versions

Cesium for Unreal **v2.21.0+** for UE 5.7 (v2.29.0 requires 5.6+). Needs the
Python Editor Script Plugin enabled.

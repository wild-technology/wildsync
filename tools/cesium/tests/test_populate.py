"""Dry-run cesium_populate against a fake editor. No Unreal required.

What this actually proves: that the height correction lands along the local
vertical (not the origin's +Z), that re-running does not duplicate actors, that
the terrain tileset is not treated as a model, that an unsampled site is left
where ion put it, and that the reported deltas are the ones applied.
"""

import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import fake_unreal                                    # noqa: E402

sys.modules["unreal"] = fake_unreal

import cesium_populate                                # noqa: E402

ORIGIN = (-64.7500, 32.3000, 0.0)                     # lon, lat, height
TERRAIN_ID = 9000


def _write(obj):
    fh = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump(obj, fh)
    fh.close()
    return fh.name


class PopulateTest(unittest.TestCase):
    def setUp(self):
        fake_unreal.reset()
        self.georef = fake_unreal.CesiumGeoreference(*ORIGIN)
        self.georef.set_actor_label("CesiumGeoreference")
        self.terrain = fake_unreal.Cesium3DTileset()
        self.terrain.set_actor_label("Bathymetry")
        self.terrain.set_editor_property("ion_asset_id", TERRAIN_ID)
        self.terrain.set_editor_property("create_physics_meshes", True)
        fake_unreal._Level.actors = [self.georef, self.terrain]

        # Seafloor at a constant -1200 m, except one hole with no coverage.
        fake_unreal.TERRAIN_FN = lambda lon, lat: None if lat > 33.0 else -1200.0

        self.manifest = [
            # at the origin, so the offset must be purely +Z
            {"id": 101, "name": "Transect_01", "lon": ORIGIN[0], "lat": ORIGIN[1],
             "height": -1150.0},
            # ~50 km east, where local up has diverged from the origin's +Z
            {"id": 102, "name": "Transect_02", "lon": ORIGIN[0] + 0.53, "lat": ORIGIN[1],
             "height": -1180.0},
            # north of the terrain coverage: sampling fails here
            {"id": 103, "name": "Transect_03", "lon": ORIGIN[0], "lat": 33.5,
             "height": -900.0},
            # the terrain itself, which must not be placed as a model
            {"id": TERRAIN_ID, "name": "Bathymetry", "lon": ORIGIN[0], "lat": ORIGIN[1],
             "height": -1200.0},
        ]
        self.report_path = _write({})
        self.cfg_path = _write({
            "terrain_ion_asset_id": TERRAIN_ID,
            "manifest": _write(self.manifest),
            "report": self.report_path,
        })

    def report(self):
        with open(self.report_path) as fh:
            return json.load(fh)

    def models(self):
        return [a for a in fake_unreal._Level.actors
                if isinstance(a, fake_unreal.Cesium3DTileset) and a is not self.terrain]

    # -- placement ---------------------------------------------------------

    def test_spawns_one_actor_per_model_and_skips_the_terrain(self):
        cesium_populate.run(self.cfg_path)
        labels = sorted(a.get_actor_label() for a in self.models())
        self.assertEqual(labels, ["Transect_01", "Transect_02", "Transect_03"])
        self.assertEqual(int(self.terrain.get_editor_property("ion_asset_id")), TERRAIN_ID)

    def test_rerun_is_idempotent(self):
        cesium_populate.run(self.cfg_path)
        first = len(fake_unreal._Level.actors)
        cesium_populate.run(self.cfg_path)
        self.assertEqual(len(fake_unreal._Level.actors), first)

    def test_georeference_origin_is_never_touched(self):
        cesium_populate.run(self.cfg_path)
        self.assertEqual(self.georef.origin, ORIGIN)

    def test_missing_georeference_is_a_hard_error(self):
        fake_unreal._Level.actors = [self.terrain]
        with self.assertRaises(RuntimeError):
            cesium_populate.run(self.cfg_path)

    # -- the height correction --------------------------------------------

    def test_offset_at_origin_is_purely_vertical(self):
        cesium_populate.run(self.cfg_path)
        a = next(a for a in self.models() if a.get_actor_label() == "Transect_01")
        # -1150 m -> -1200 m is a 50 m drop, in centimetres.
        self.assertAlmostEqual(a.location.z, -5000.0, delta=1.0)
        self.assertAlmostEqual(a.location.x, 0.0, delta=1.0)
        self.assertAlmostEqual(a.location.y, 0.0, delta=1.0)

    def test_offset_far_from_origin_follows_local_up_not_world_z(self):
        cesium_populate.run(self.cfg_path)
        a = next(a for a in self.models() if a.get_actor_label() == "Transect_02")
        site = next(m for m in self.manifest if m["id"] == 102)
        delta_cm = (-1200.0 - site["height"]) * 100.0     # -2000 cm

        # magnitude is preserved: the model moved exactly the height difference
        length = math.sqrt(a.location.x ** 2 + a.location.y ** 2 + a.location.z ** 2)
        self.assertAlmostEqual(length, abs(delta_cm), delta=1.0)

        # direction is the site's local up expressed in the origin's frame,
        # which at 50 km has a horizontal component a raw Z nudge would miss
        east, south, up = fake_unreal.esu_basis(site["lon"], site["lat"])
        oe, os_, ou = fake_unreal.esu_basis(*ORIGIN[:2])
        dot = lambda p, q: p[0] * q[0] + p[1] * q[1] + p[2] * q[2]
        expected = (delta_cm * dot(up, oe), delta_cm * dot(up, os_), delta_cm * dot(up, ou))
        self.assertAlmostEqual(a.location.x, expected[0], delta=1.0)
        self.assertAlmostEqual(a.location.y, expected[1], delta=1.0)
        self.assertAlmostEqual(a.location.z, expected[2], delta=1.0)
        self.assertGreater(abs(a.location.x), 1.0, "local up should tilt at 50 km")

    def test_unsampled_site_is_left_at_its_ion_height(self):
        cesium_populate.run(self.cfg_path)
        a = next(a for a in self.models() if a.get_actor_label() == "Transect_03")
        self.assertEqual((a.location.x, a.location.y, a.location.z), (0.0, 0.0, 0.0))
        row = next(r for r in self.report()["assets"] if r["id"] == 103)
        self.assertFalse(row["sampled"])
        self.assertIsNone(row["delta_m"])

    # -- the report --------------------------------------------------------

    def test_report_records_the_deltas_actually_applied(self):
        cesium_populate.run(self.cfg_path)
        rows = {r["id"]: r for r in self.report()["assets"]}
        self.assertAlmostEqual(rows[101]["delta_m"], -50.0, places=6)
        self.assertAlmostEqual(rows[102]["delta_m"], -20.0, places=6)
        self.assertEqual(self.report()["summary"]["sampled"], 2)

    def test_uniform_offset_is_called_out_as_a_datum_error(self):
        for m in self.manifest:
            if m["id"] != TERRAIN_ID:
                m["lat"], m["lon"], m["height"] = ORIGIN[1], ORIGIN[0], -1163.0
        self.cfg_path = _write({
            "terrain_ion_asset_id": TERRAIN_ID,
            "manifest": _write(self.manifest),
            "report": self.report_path,
        })
        cesium_populate.run(self.cfg_path)
        warnings = [m for lvl, m in fake_unreal.LOG if lvl == "warn"]
        self.assertTrue(any("datum offset" in w for w in warnings),
                        "a uniform shift should be flagged, not silently applied")

    def test_level_is_saved(self):
        cesium_populate.run(self.cfg_path)
        self.assertTrue(fake_unreal._Level.saved)


if __name__ == "__main__":
    unittest.main(verbosity=2)

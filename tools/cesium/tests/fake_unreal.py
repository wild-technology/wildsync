"""A stand-in for the `unreal` module, enough to dry-run cesium_populate.

The georeference transform is the real geodesy, not a stub: ECEF difference
from the origin, rotated into Unreal's East/South/Up frame and scaled to
centimetres. That is what makes the height-offset assertions in
test_populate.py meaningful rather than circular.
"""

import math

_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

LOG = []


def log(msg):
    LOG.append(("log", str(msg)))


def log_warning(msg):
    LOG.append(("warn", str(msg)))


def log_error(msg):
    LOG.append(("error", str(msg)))


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def __repr__(self):
        return "Vector(%.4f, %.4f, %.4f)" % (self.x, self.y, self.z)


class Rotator:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


def _ecef(lon_deg, lat_deg, h):
    lo, la = math.radians(lon_deg), math.radians(lat_deg)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(la) ** 2)
    return (
        (n + h) * math.cos(la) * math.cos(lo),
        (n + h) * math.cos(la) * math.sin(lo),
        (n * (1.0 - _E2) + h) * math.sin(la),
    )


def esu_basis(lon_deg, lat_deg):
    """Unit East, South, Up vectors in ECEF at a cartographic position."""
    lo, la = math.radians(lon_deg), math.radians(lat_deg)
    east = (-math.sin(lo), math.cos(lo), 0.0)
    north = (-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la))
    up = (math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la))
    south = (-north[0], -north[1], -north[2])
    return east, south, up


class Actor:
    def __init__(self, cls):
        self._cls = cls
        self._label = ""
        self._props = {}
        self.location = Vector(0.0, 0.0, 0.0)

    def set_actor_label(self, label):
        self._label = label

    def get_actor_label(self):
        return self._label

    def set_editor_property(self, name, value):
        self._props[name] = value

    def get_editor_property(self, name):
        return self._props.get(name)

    def set_actor_location(self, loc, sweep, teleport):
        self.location = loc
        return True


class Cesium3DTileset(Actor):
    def __init__(self):
        Actor.__init__(self, "Cesium3DTileset")
        self._props["create_physics_meshes"] = False
        self._props["ion_asset_id"] = 0


class CesiumGeoreference(Actor):
    def __init__(self, lon=0.0, lat=0.0, height=0.0):
        Actor.__init__(self, "CesiumGeoreference")
        self.origin = (lon, lat, height)

    def transform_longitude_latitude_height_position_to_unreal(self, llh):
        ox, oy, oz = _ecef(*self.origin)
        px, py, pz = _ecef(llh.x, llh.y, llh.z)
        d = (px - ox, py - oy, pz - oz)
        east, south, up = esu_basis(self.origin[0], self.origin[1])
        dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
        return Vector(100.0 * dot(d, east), 100.0 * dot(d, south), 100.0 * dot(d, up))


class TilesetSource:
    FROM_CESIUM_ION = "FROM_CESIUM_ION"


class _Level:
    """The mutable world the subsystems act on."""
    actors = []
    saved = False
    loaded = None


class EditorActorSubsystem:
    def get_all_level_actors(self):
        return list(_Level.actors)

    def spawn_actor_from_class(self, cls, location, rotation):
        actor = cls()
        actor.location = location
        _Level.actors.append(actor)
        return actor


class LevelEditorSubsystem:
    def load_level(self, path):
        _Level.loaded = path
        return True


class EditorLoadingAndSavingUtils:
    @staticmethod
    def save_dirty_packages(maps, content):
        _Level.saved = True
        return True


def get_editor_subsystem(cls):
    return cls()


# --- the async height sampler -------------------------------------------------

TERRAIN_FN = None          # set by the test: (lon, lat) -> height or None


class SampleHeightResult:
    def __init__(self, llh, ok):
        self._llh, self._ok = llh, ok

    def get_editor_property(self, name):
        if name == "sample_success":
            return self._ok
        if name == "longitude_latitude_height":
            return self._llh
        raise KeyError(name)


class _Delegate:
    def __init__(self):
        self._callables = []

    def add_callable(self, fn):
        self._callables.append(fn)

    def broadcast(self, *args):
        for fn in list(self._callables):
            fn(*args)


class _Action:
    def __init__(self, tileset, positions):
        self.positions = positions
        self.on_heights_sampled = _Delegate()

    def call_method(self, name, args=()):
        if name != "Activate":
            raise KeyError(name)
        results, warnings = [], []
        for p in self.positions:
            h = TERRAIN_FN(p.x, p.y) if TERRAIN_FN else None
            if h is None:
                results.append(SampleHeightResult(Vector(p.x, p.y, p.z), False))
                warnings.append("no tile at %.4f, %.4f" % (p.y, p.x))
            else:
                results.append(SampleHeightResult(Vector(p.x, p.y, h), True))
        # Real sampling is asynchronous; firing inline is enough to exercise
        # every branch downstream of the callback.
        self.on_heights_sampled.broadcast(results, warnings)


class CesiumSampleHeightMostDetailedAsyncAction:
    pass


class _CDO:
    def call_method(self, name, args=()):
        if name != "SampleHeightMostDetailed":
            raise KeyError(name)
        return _Action(args[0], args[1])


def get_default_object(cls):
    return _CDO()


_HANDLES = []


def register_slate_post_tick_callback(fn):
    _HANDLES.append(fn)
    return fn


def unregister_slate_post_tick_callback(handle):
    if handle in _HANDLES:
        _HANDLES.remove(handle)


def reset():
    _Level.actors = []
    _Level.saved = False
    _Level.loaded = None
    _HANDLES.clear()
    LOG.clear()

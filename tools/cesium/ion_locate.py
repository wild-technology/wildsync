"""Discover Cesium ion 3D Tiles assets by name and resolve where each one sits.

Pure stdlib so it runs unchanged inside Unreal's bundled Python 3.11.
ion's /v1/assets metadata carries no geographic extent, so the location has to
come from the tileset itself: ask ion for a signed endpoint, fetch tileset.json,
and reduce root.transform x root.boundingVolume to a cartographic centre.
"""

import fnmatch
import json
import math
import os
import re
import urllib.parse
import urllib.request

API = "https://api.cesium.com"

# WGS 84
_A = 6378137.0
_F = 1.0 / 298.257223563
_B = _A * (1.0 - _F)
_E2 = _F * (2.0 - _F)
_EP2 = (_A * _A - _B * _B) / (_B * _B)


def _get(url, token=None):
    req = urllib.request.Request(url)
    if token:
        req.add_header("Authorization", "Bearer " + token)
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)


def list_assets(token, asset_type="3DTILES", status="COMPLETE"):
    """Every finished 3D Tiles asset on the account, newest first."""
    out, page = [], 1
    while True:
        q = urllib.parse.urlencode(
            {"type": asset_type, "status": status, "limit": 1000, "page": page,
             "sortBy": "DATE_ADDED", "sortOrder": "DESC"}
        )
        body = _get("%s/v1/assets?%s" % (API, q), token)
        items = body.get("items", [])
        out.extend(items)
        if len(items) < 1000:
            return out
        page += 1


def ecef_to_lon_lat_height(x, y, z):
    """Bowring's closed-form ECEF -> WGS84 lon/lat/ellipsoidal height (degrees, m)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    if p < 1e-9:                       # on the polar axis
        lat = math.copysign(math.pi / 2.0, z)
        return math.degrees(lon), math.degrees(lat), abs(z) - _B
    theta = math.atan2(z * _A, p * _B)
    lat = math.atan2(
        z + _EP2 * _B * math.sin(theta) ** 3,
        p - _E2 * _A * math.cos(theta) ** 3,
    )
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lon), math.degrees(lat), h


def _mat_apply(m, v):
    """m is a 3D Tiles 4x4 COLUMN-major transform, v a 3-vector point."""
    if not m:
        return list(v)
    return [
        m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12],
        m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13],
        m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14],
    ]


def tileset_origin(tileset_json):
    """Cartographic centre of a tileset's root tile: (lon_deg, lat_deg, height_m)."""
    root = tileset_json["root"]
    bv = root.get("boundingVolume", {})
    xform = root.get("transform")

    if "region" in bv:
        # [west, south, east, north, minHeight, maxHeight]; angles in RADIANS.
        # A region is already ECEF-aligned, so root.transform does not apply.
        w, s, e, n, hmin, hmax = bv["region"]
        if e < w:                      # region straddling the antimeridian
            e += 2.0 * math.pi
        lon = math.degrees((w + e) / 2.0)
        lon = (lon + 180.0) % 360.0 - 180.0
        return lon, math.degrees((s + n) / 2.0), (hmin + hmax) / 2.0

    if "box" in bv:
        # 12 numbers: centre xyz then three half-axis vectors.
        centre = bv["box"][0:3]
    elif "sphere" in bv:
        centre = bv["sphere"][0:3]
    else:
        raise ValueError("root tile has no usable boundingVolume")

    return ecef_to_lon_lat_height(*_mat_apply(xform, centre))


def locate_asset(asset_id, token):
    """(lon_deg, lat_deg, height_m) for an ion 3D Tiles asset."""
    ep = _get("%s/v1/assets/%d/endpoint" % (API, int(asset_id)), token)
    url = ep["url"]
    sep = "&" if urllib.parse.urlparse(url).query else "?"
    return tileset_origin(_get(url + sep + "access_token=" + ep["accessToken"]))


def match_names(assets, pattern):
    """Filter ion assets by name.

    A pattern starting with "re:" is a regular expression (searched, not
    anchored); anything else is a shell glob, matched case-insensitively so
    "Transect_*" catches "transect_04" too.
    """
    if not pattern:
        return list(assets)
    if pattern.startswith("re:"):
        rx = re.compile(pattern[3:], re.IGNORECASE)
        return [a for a in assets if rx.search(a.get("name", ""))]
    low = pattern.lower()
    return [a for a in assets if fnmatch.fnmatch(a.get("name", "").lower(), low)]


def build_manifest(token, pattern=None, asset_ids=None):
    """[{id, name, lon, lat, height}] — the input the Unreal script consumes.

    Assets that cannot be located carry an "error" key instead of coordinates,
    so one bad asset never silently drops out of the count.
    """
    if asset_ids:
        assets = [{"id": int(i), "name": str(i)} for i in asset_ids]
    else:
        assets = match_names(list_assets(token), pattern)

    manifest = []
    for a in assets:
        try:
            lon, lat, h = locate_asset(a["id"], token)
        except Exception as exc:                       # keep going; report at the end
            manifest.append({"id": a["id"], "name": a["name"], "error": str(exc)})
            continue
        manifest.append(
            {"id": a["id"], "name": a["name"], "lon": lon, "lat": lat, "height": h}
        )
    return manifest


if __name__ == "__main__":
    import sys

    tok = os.environ["CESIUM_ION_TOKEN"]
    pat = sys.argv[1] if len(sys.argv) > 1 else None
    print(json.dumps(build_manifest(tok, pat), indent=2))

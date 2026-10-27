#!/usr/bin/env python3
"""SPIKE-002 offline converter: verified sources -> bounded SFO runtime assets.

Outputs (all deterministic, under ``--output``; never committed):

* ``sfo-photoreal.gxvisual``  GXVISUAL 1.1: two ASTC 6x6 sRGB NAIP terrain
  textures (an airport detail layer and a whole-area context layer, 3DEP
  topobathymetric land/water mask and shore depth in alpha), a 3DEP terrain
  mesh in the pack's ENU frame, procedural runway/taxiway markings (decals)
  and either a caller-supplied AC3D preview aircraft or a generated triangle
  placeholder. The alpha pipeline replaces that preview with the GPL 787.
* ``sfo-real.world.txt``  the surveyed-SFO world source (oriented runway
  rectangles, runway markings, airport lights) for
  ``gx-app spike002-world`` -> ``sfo-real.gxworld`` (GXWORLD 2.0).
* ``manifest.json`` provenance, parameters, sizes and hashes; ``preview.jpg``.

The board never sees GeoTIFF, OSM XML or AC3D; this tool runs on the host
only and executes nothing from the downloaded data (astcenc is the pinned
Arm release from the asset cache, not from the data).
"""
from __future__ import annotations

import argparse, hashlib, json, math, pathlib, struct, subprocess, sys, tempfile, urllib.parse
import xml.etree.ElementTree as ET
try:
    import numpy as np
    from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageOps
except ImportError as exc:  # pragma: no cover
    raise SystemExit("install tools/spike002-requirements.txt: " + str(exc))

Image.MAX_IMAGE_PIXELS = 400_000_000

SCHEMA = "gx-visual-interchange/2"
CONVERTER_VERSION = 8
# ASCII "SFOP02" + 0x0001: the SPIKE-002 surveyed-SFO frame (not SFOSYN).
FRAME_ID = 0x5346_4F50_3032_0001
FRAME_GENERATION = 1
FIELD_ELEVATION_MSL_M = 4.0          # pack origin height (gx-world SFO fixture)
RUNWAY_Z = 0.2                       # GXWORLD runway surface (as the S4 pack)
MARKING_Z = 0.25
CENTERLINE_LIGHT_CLEARANCE_M = 0.005    # inset fixture just above the painted surface
WATER_LEVEL_M = 0.8                  # 3DEP (NAVD88) height treated as the Bay surface
EARTH_R = 6_372_800.0                # local Gaussian radius at SFO (WorldMesh uses the exact one)
KEEP_AEROWAYS = {"runway", "taxiway", "apron", "parking_position",
                 "holding_position", "runway_threshold"}
MAX_OUTPUT_BYTES = 256 * 1024 * 1024
TAXIWAY_LIGHTS = False                # daytime scene: green centreline dots read as clutter
WHITE_UV, YELLOW_UV = (0.25, 0.5), (0.75, 0.5)
# JSBSim global5000.xml (gx-render aircraft.rs): CG and main-gear contact, inches.
CG_X_IN, CG_Z_IN, MAIN_X_IN, GEAR_Z_IN = 790.82, -29.07, 822.43, -139.54


def canonical_json(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()


def sha256_file(path):
    digest = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def safe_path(root, relative):
    root, target = pathlib.Path(root).resolve(), (pathlib.Path(root) / relative).resolve()
    if root != target and root not in target.parents:
        raise ValueError(f"path escapes acquisition root: {relative}")
    return target


def verify_acquisition(root, manifest_path):
    root, manifest_path = pathlib.Path(root), pathlib.Path(manifest_path)
    source_bytes = manifest_path.read_bytes()
    source = json.loads(source_bytes)
    acquired = json.loads((root / "acquisition-manifest.json").read_text(encoding="utf-8"))
    if acquired.get("schema") != "gx-spike002-acquisition-manifest/1":
        raise ValueError("unsupported acquisition manifest")
    if hashlib.sha256(source_bytes).hexdigest() != acquired.get("source_manifest_sha256"):
        raise ValueError("source/acquisition manifest identity mismatch")
    for receipt in acquired["receipts"]:
        if receipt.get("missing"):
            continue
        path = safe_path(root, receipt["relative_path"])
        if path.stat().st_size != receipt["bytes"] or sha256_file(path) != receipt["sha256"]:
            raise ValueError(f"raw input failed receipt verification: {receipt['request_id']}")
    return source, acquired


def assert_exact_arcgis_grid(receipts, source_id):
    """Reject ArcGIS exports whose returned extent differs from the requested tile.

    ArcGIS otherwise adjusts each tile's aspect ratio independently.  Blindly
    concatenating those adjusted images creates repeated rows or columns at
    every join, so detail mosaics must prove that their server-returned WGS84
    extents are exact before pixels are decoded.
    """
    source_receipts = [r for r in receipts
                       if r.get("source_id") == source_id and not r.get("missing")]
    if not source_receipts:
        raise ValueError(f"{source_id}: no ArcGIS receipts")
    for receipt in source_receipts:
        request_id = receipt.get("request_id", source_id)
        query = urllib.parse.parse_qs(urllib.parse.urlparse(receipt["requested_url"]).query)
        requested = [float(value) for value in query["bbox"][0].split(",")]
        requested_size = [int(value) for value in query["size"][0].split(",")]
        result = receipt.get("arcgis_result") or {}
        extent = result.get("extent") or {}
        spatial_ref = extent.get("spatialReference") or {}
        wkids = {spatial_ref.get("wkid"), spatial_ref.get("latestWkid")}
        actual = [extent.get(name) for name in ("xmin", "ymin", "xmax", "ymax")]
        if 4326 not in wkids or any(value is None for value in actual):
            raise ValueError(f"{request_id}: returned extent is not WGS84")
        if any(abs(float(got) - want) > 1e-8 for got, want in zip(actual, requested)):
            raise ValueError(f"{request_id}: ArcGIS adjusted requested extent")
        if [result.get("width"), result.get("height")] != requested_size:
            raise ValueError(f"{request_id}: ArcGIS returned unexpected tile size")


def stitch_raster(root, acquired, source_id, grid, mode, *, exact_extents=False):
    """Native-resolution mosaic; tile row 0 is the southern row."""
    nx, ny = grid
    if exact_extents:
        assert_exact_arcgis_grid(acquired["receipts"], source_id)
    receipts = {r["request_id"]: r for r in acquired["receipts"] if not r.get("missing")}
    tiles = {}
    for row in range(ny):
        for col in range(nx):
            request_id = f"{source_id}-{row:02d}-{col:02d}"
            receipt = receipts.get(request_id)
            if receipt is None:
                raise ValueError(f"missing required raster tile {request_id}")
            with Image.open(safe_path(root, receipt["relative_path"])) as image:
                tiles[row, col] = image.convert(mode)
    tw, th = tiles[0, 0].size
    if any(t.size != (tw, th) for t in tiles.values()):
        raise ValueError(f"{source_id}: tiles differ in size")
    canvas = Image.new(mode, (tw * nx, th * ny))
    for (row, col), tile in tiles.items():
        canvas.paste(tile, (col * tw, (ny - 1 - row) * th))
    return canvas


def mip_chain(image):
    levels = [image]
    while levels[-1].size != (1, 1):
        w, h = levels[-1].size
        levels.append(levels[-1].resize((max(1, w // 2), max(1, h // 2)), Image.Resampling.LANCZOS))
    return levels


def geodetic_to_enu(lat, lon, height, origin):
    """WGS84 geodetic -> local ENU about ``origin`` (lat, lon, h); numpy-friendly."""
    a, inv_f = 6378137.0, 298.257223563
    e2 = (1 / inv_f) * (2 - 1 / inv_f)

    def ecef(phi_deg, lam_deg, h):
        phi, lam = np.radians(phi_deg), np.radians(lam_deg)
        n = a / np.sqrt(1 - e2 * np.sin(phi) ** 2)
        return ((n + h) * np.cos(phi) * np.cos(lam), (n + h) * np.cos(phi) * np.sin(lam),
                (n * (1 - e2) + h) * np.sin(phi))
    lat0, lon0, h0 = origin
    x, y, z = ecef(lat, lon, height)
    x0, y0, z0 = ecef(lat0, lon0, h0)
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = math.radians(lat0), math.radians(lon0)
    e = -math.sin(lam) * dx + math.cos(lam) * dy
    n = -math.sin(phi) * math.cos(lam) * dx - math.sin(phi) * math.sin(lam) * dy + math.cos(phi) * dz
    u = math.cos(phi) * math.cos(lam) * dx + math.cos(phi) * math.sin(lam) * dy + math.sin(phi) * dz
    if np.ndim(e) == 0:
        return float(e), float(n), float(u)
    return e, n, u


def sample(values, u, v):
    """Bilinear sample of a 2-D array at normalized (u right, v down) coordinates."""
    h, w = values.shape
    x = np.clip(np.asarray(u) * (w - 1), 0, w - 1)
    y = np.clip(np.asarray(v) * (h - 1), 0, h - 1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0 + 1, w - 1), np.minimum(y0 + 1, h - 1)
    tx, ty = x - x0, y - y0
    return (values[y0, x0] * (1 - tx) + values[y0, x1] * tx) * (1 - ty) + (values[y1, x0] * (1 - tx) + values[y1, x1] * tx) * ty


# ---------------------------------------------------------------- OSM ---

def parse_osm_aeroways(paths, origin):
    nodes, all_ways, ways, relations, standalone = {}, {}, {}, {}, {}
    for path in sorted(paths):
        root = ET.parse(path).getroot()
        for node in root.findall("node"):
            nid = int(node.attrib["id"]); nodes[nid] = (float(node.attrib["lat"]), float(node.attrib["lon"]))
            tags = {t.attrib["k"]: t.attrib["v"] for t in node.findall("tag")}
            if tags.get("aeroway") in KEEP_AEROWAYS: standalone[nid] = tags
        for way in root.findall("way"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in way.findall("tag")}
            refs = [int(n.attrib["ref"]) for n in way.findall("nd")]
            all_ways[int(way.attrib["id"])] = (refs, tags)
            if tags.get("aeroway") in KEEP_AEROWAYS:
                ways[int(way.attrib["id"])] = (refs, tags)
        for relation in root.findall("relation"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in relation.findall("tag")}
            if tags.get("aeroway") in KEEP_AEROWAYS:
                members = [(int(m.attrib["ref"]), m.attrib.get("role", "")) for m in relation.findall("member")
                           if m.attrib.get("type") == "way"]
                relations[int(relation.attrib["id"])] = (members, tags)
    result = []; origin_enu = (origin[1], origin[0], 0.)
    for wid, (refs, tags) in sorted(ways.items()):
        wgs84 = [[nodes[r][1], nodes[r][0]] for r in refs if r in nodes]
        points = [geodetic_to_enu(lat, lon, 0., origin_enu)[:2] for lon, lat in wgs84]
        if len(points) >= 2: result.append({"osm_type": "way", "osm_id": wid, "aeroway": tags["aeroway"], "tags": tags,
                                            "points_wgs84": wgs84, "points_enu_m": points})
    for nid, tags in sorted(standalone.items()):
        if nid in nodes:
            lat, lon = nodes[nid]
            result.append({"osm_type": "node", "osm_id": nid, "aeroway": tags["aeroway"], "tags": tags,
                           "points_wgs84": [[lon, lat]],
                           "points_enu_m": [geodetic_to_enu(lat, lon, 0., origin_enu)[:2]]})
    for rid, (members, tags) in sorted(relations.items()):
        member_ways = []
        for wid, role in members:
            refs, _ = all_ways.get(wid, ([], {}))
            wgs84 = [[nodes[r][1], nodes[r][0]] for r in refs if r in nodes]
            points = [geodetic_to_enu(lat, lon, 0., origin_enu)[:2] for lon, lat in wgs84]
            if len(points) >= 2: member_ways.append({"osm_way_id": wid, "role": role,
                                                     "points_wgs84": wgs84, "points_enu_m": points})
        if member_ways:
            result.append({"osm_type": "relation", "osm_id": rid, "aeroway": tags["aeroway"], "tags": tags,
                           "points_wgs84": member_ways[0]["points_wgs84"],
                           "points_enu_m": member_ways[0]["points_enu_m"], "member_ways": member_ways})
    return result


def parse_faa_geojson(path, layer, origin):
    """Read one bounded FAA Airport Diagram polygon layer in WGS84.

    The acquisition query already selects KSFO.  Keep both WGS84 rings (for
    texture masks) and local ENU rings (for runtime geometry) so no GIS runtime
    dependency is needed.
    """
    document = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    if document.get("type") != "FeatureCollection":
        raise ValueError(f"FAA {layer}: expected GeoJSON FeatureCollection")
    result = []
    origin_enu = (origin[1], origin[0], 0.0)
    for feature in document.get("features", []):
        geometry = feature.get("geometry") or {}
        kind, coordinates = geometry.get("type"), geometry.get("coordinates")
        if kind == "Polygon":
            polygons = [coordinates]
        elif kind == "MultiPolygon":
            polygons = coordinates
        else:
            raise ValueError(f"FAA {layer}: non-polygon geometry {kind!r}")
        for polygon_number, rings in enumerate(polygons):
            if not rings or len(rings[0]) < 4:
                raise ValueError(f"FAA {layer}: invalid polygon ring")
            wgs84 = [[[float(lon), float(lat)] for lon, lat, *_ in ring] for ring in rings]
            enu = [[list(geodetic_to_enu(lat, lon, 0.0, origin_enu)[:2])
                    for lon, lat in ring] for ring in wgs84]
            outer = np.asarray(enu[0], dtype=float)
            result.append({"layer": layer, "polygon": polygon_number,
                           "properties": feature.get("properties") or {},
                           "rings_wgs84": wgs84, "rings_enu_m": enu,
                           "bbox_enu_m": [float(outer[:, 0].min()), float(outer[:, 1].min()),
                                          float(outer[:, 0].max()), float(outer[:, 1].max())]})
    return result


def runways_from_osm(features):
    """Merge OSM runway ways by ref into one strip each: (name, low end, high end, width).

    The low end is the end whose designator is the lower number (10L of
    10L/28R); the strip runs from the low end towards the high end.
    """
    by_ref = {}
    for f in features:
        if f["aeroway"] == "runway" and f["osm_type"] == "way" and f["tags"].get("ref"):
            by_ref.setdefault(f["tags"]["ref"], []).append(f)
    out = []
    for ref, parts in sorted(by_ref.items()):
        pts = np.array([p for f in parts for p in f["points_enu_m"]], dtype=float)
        longest = max(parts, key=lambda f: math.dist(f["points_enu_m"][0], f["points_enu_m"][-1]))
        d = np.subtract(longest["points_enu_m"][-1], longest["points_enu_m"][0]).astype(float); d /= np.linalg.norm(d)
        low_designator = int("".join(ch for ch in ref.split("/")[0] if ch.isdigit()))
        want = math.radians(low_designator * 10.0)        # magnetic-ish; only the sense matters
        if d[0] * math.sin(want) + d[1] * math.cos(want) < 0:
            d = -d
        s = pts @ d
        c = pts.mean(axis=0); c = c + (s.min() + s.max()) / 2 * d - (c @ d) * d
        half = (s.max() - s.min()) / 2
        width = float(parts[0]["tags"].get("width", 45))
        out.append({"name": ref, "low": (c - half * d).tolist(), "high": (c + half * d).tolist(), "width": width,
                    "osm_ids": sorted(f["osm_id"] for f in parts),
                    "true_heading_deg": math.degrees(math.atan2(d[0], d[1])) % 360, "length_m": 2 * half})
    # The world pack's first runway is the start runway (10L/28R).
    out.sort(key=lambda r: (r["name"] != "10L/28R", r["name"]))
    return out


def runways_from_faa(features, published_widths=None):
    """Fit stable centreline strips to authoritative FAA runway polygons.

    ArcGIS polygon edges are irregularly densified, so treating vertices as
    equally weighted samples biases PCA toward whichever edge has more points.
    A minimum-area oriented bounding rectangle depends on the boundary shape,
    not its vertex spacing.
    """
    published_widths = published_widths or {}
    out = []
    for feature in features:
        name = feature["properties"].get("RWY_ID")
        if not name:
            raise ValueError("FAA runway polygon has no RWY_ID")
        points = np.asarray(feature["rings_enu_m"][0], dtype=float)
        if np.linalg.norm(points[0] - points[-1]) < 1e-6:
            points = points[:-1]
        if len(points) < 3:
            raise ValueError(f"FAA runway {name}: too few polygon points")
        best = None
        for start, end in zip(points, np.roll(points, -1, axis=0)):
            edge = end - start
            edge_length = np.linalg.norm(edge)
            if edge_length < 1e-6:
                continue
            direction = edge / edge_length
            normal = np.array([-direction[1], direction[0]])
            along, across = points @ direction, points @ normal
            along_min, along_max = float(along.min()), float(along.max())
            across_min, across_max = float(across.min()), float(across.max())
            length, width = along_max - along_min, across_max - across_min
            if length < width:
                direction, normal = normal, -direction
                along, across = points @ direction, points @ normal
                along_min, along_max = float(along.min()), float(along.max())
                across_min, across_max = float(across.min()), float(across.max())
                length, width = along_max - along_min, across_max - across_min
            candidate = (length * width, -length, direction, normal,
                         along_min, along_max, across_min, across_max)
            if best is None or candidate[:2] < best[:2]:
                best = candidate
        if best is None:
            raise ValueError(f"FAA runway {name}: degenerate polygon")
        _, _, direction, normal, along_min, along_max, across_min, across_max = best
        low_designator = int("".join(ch for ch in name.split("/")[0] if ch.isdigit()))
        wanted = math.radians(low_designator * 10.0)
        if direction[0] * math.sin(wanted) + direction[1] * math.cos(wanted) < 0:
            direction = -direction
            normal = -normal
            along_min, along_max = -along_max, -along_min
            across_min, across_max = -across_max, -across_min
        across_mid = 0.5 * (across_min + across_max)
        low = direction * along_min + normal * across_mid
        high = direction * along_max + normal * across_mid
        polygon_width = across_max - across_min
        out.append({"name": name, "low": low.tolist(), "high": high.tolist(),
                    # The diagram polygon can include paved shoulders. FAA's
                    # layer has no declared runway-width field, so preserve the
                    # published OSM width for markings and physics while using
                    # FAA for position, heading and length.
                    "width": float(published_widths.get(name, polygon_width)),
                    "faa_polygon_width_m": polygon_width,
                    "faa_objectid": feature["properties"].get("OBJECTID"),
                    "surface": feature["properties"].get("SURFACE"),
                    "true_heading_deg": math.degrees(math.atan2(direction[0], direction[1])) % 360,
                    "length_m": along_max - along_min})
    out.sort(key=lambda runway: (runway["name"] != "10L/28R", runway["name"]))
    return out


# -------------------------------------------------------- markings ---

class Quads:
    def __init__(self):
        self.quads = []          # (4 corners xy, uv)

    def rect(self, origin, u, n, a0, a1, b0, b1, uv):
        """Rectangle in a strip frame: along u in [a0,a1], across n in [b0,b1]."""
        o, u, n = np.asarray(origin), np.asarray(u), np.asarray(n)
        corners = [o + a0 * u + b0 * n, o + a1 * u + b0 * n, o + a1 * u + b1 * n, o + a0 * u + b1 * n]
        self.quads.append(([tuple(map(float, c)) for c in corners], uv))

    def segment(self, p, q, width, uv):
        p, q = np.asarray(p, float), np.asarray(q, float)
        length = float(np.linalg.norm(q - p))
        if length < 0.05:
            return
        u = (q - p) / length; n = np.array([-u[1], u[0]])
        self.rect(p, u, n, 0.0, length, -width / 2, width / 2, uv)


def runway_markings(rw, q: Quads):
    """FAA-style precision markings (AC 150/5340-1 proportions), both ends."""
    low, high, w = np.array(rw["low"]), np.array(rw["high"]), rw["width"]
    length = float(np.linalg.norm(high - low))
    for start, end in ((low, high), (high, low)):
        u = (end - start) / length; n = np.array([-u[1], u[0]])
        stripes = {18: 4, 23: 6, 30: 8, 45: 12, 60: 16}[min((18, 23, 30, 45, 60), key=lambda k: abs(k - w))]
        sw = 1.75
        per_side = stripes // 2
        for side in (-1, 1):
            for k in range(per_side):
                inner = 3.0 + k * 2 * sw
                q.rect(start, u, n, 6.0, 6.0 + 45.7, side * inner, side * (inner + sw), WHITE_UV)
        # Aiming point (1,020 ft) and touchdown zone (500..3,000 ft).
        for side in (-1, 1):
            q.rect(start, u, n, 311.0, 311.0 + 45.7, side * 11.0, side * 20.1, WHITE_UV)
            for at, bars in ((152.4, 3), (457.2, 2), (609.6, 2), (762.0, 1), (914.4, 1)):
                for b in range(bars):
                    inner = 11.0 + b * 3.4
                    q.rect(start, u, n, at, at + 22.9, side * inner, side * (inner + 1.8), WHITE_UV)
        # Side stripes.
        if start is low:
            for side in (-1, 1):
                q.rect(start, u, n, 0.0, length, side * (w / 2 - 1.4), side * (w / 2 - 0.5), WHITE_UV)
    # Centreline: 36.6 m stripes, 24.4 m gaps, 0.9 m wide, clear of the threshold areas.
    u = (high - low) / length; n = np.array([-u[1], u[0]])
    a = 75.0
    while a + 36.6 <= length - 75.0:
        q.rect(low, u, n, a, a + 36.6, -0.45, 0.45, WHITE_UV)
        a += 61.0


def inside_any_runway(p, runways, margin):
    for rw in runways:
        low, high = np.array(rw["low"]), np.array(rw["high"])
        d = high - low; L = np.linalg.norm(d); u = d / L
        r = np.asarray(p) - low; a = r @ u; b = abs(r @ np.array([-u[1], u[0]]))
        if -margin <= a <= L + margin and b <= rw["width"] / 2 + margin:
            return True
    return False


def point_in_faa_surfaces(point, features):
    """Even-odd point test with a bbox fast path for FAA polygon rings."""
    x, y = map(float, point)

    def in_ring(ring):
        inside = False
        for (x1, y1), (x2, y2) in zip(ring, ring[1:]):
            if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
                inside = not inside
        return inside

    for feature in features:
        x0, y0, x1, y1 = feature.get("bbox_enu_m") or (
            lambda p: [float(p[:, 0].min()), float(p[:, 1].min()),
                       float(p[:, 0].max()), float(p[:, 1].max())])(
                           np.asarray(feature["rings_enu_m"][0], dtype=float))
        if x0 <= x <= x1 and y0 <= y <= y1:
            rings = feature["rings_enu_m"]
            if in_ring(rings[0]) and not any(in_ring(hole) for hole in rings[1:]):
                return True
    return False


def surface_segment_intervals(p0, p1, features):
    """Return exact parameter intervals of a segment inside a polygon union."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    direction = p1 - p0
    cuts = [0.0, 1.0]

    def cross(a, b):
        return float(a[0] * b[1] - a[1] * b[0])

    for feature in features:
        x0, y0, x1, y1 = feature.get("bbox_enu_m") or (-math.inf,) * 2 + (math.inf,) * 2
        if (max(p0[0], p1[0]) < x0 or min(p0[0], p1[0]) > x1 or
                max(p0[1], p1[1]) < y0 or min(p0[1], p1[1]) > y1):
            continue
        for ring in feature["rings_enu_m"]:
            for q0, q1 in zip(ring, ring[1:]):
                q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
                edge = q1 - q0
                denominator = cross(direction, edge)
                if abs(denominator) < 1e-12:
                    continue
                delta = q0 - p0
                t, u = cross(delta, edge) / denominator, cross(delta, direction) / denominator
                if -1e-10 <= t <= 1.0 + 1e-10 and -1e-10 <= u <= 1.0 + 1e-10:
                    cuts.append(min(1.0, max(0.0, t)))
    cuts = sorted(set(round(value, 12) for value in cuts))
    inside = [(a, b) for a, b in zip(cuts, cuts[1:])
              if b - a > 1e-9 and point_in_faa_surfaces(
                  p0 + direction * ((a + b) / 2), features)]
    merged = []
    for a, b in inside:
        if merged and a - merged[-1][1] <= 1e-9:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def clip_marking_segment(p0, p1, width, features):
    """Clip a full-width marking quad to the union of FAA pavement polygons."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    direction = p1 - p0
    length = float(np.linalg.norm(direction))
    if length < 0.05:
        return []
    normal = np.array([-direction[1], direction[0]]) / length
    interval_sets = [surface_segment_intervals(
        p0 + side * width * 0.5 * normal, p1 + side * width * 0.5 * normal, features)
        for side in (-1.0, 1.0)]
    clipped = interval_sets[0]
    for other in interval_sets[1:]:
        clipped = [(max(a0, b0), min(a1, b1)) for a0, a1 in clipped for b0, b1 in other
                   if min(a1, b1) - max(a0, b0) > 1e-9]
        clipped.sort()
        merged = []
        for a, b in clipped:
            if merged and a - merged[-1][1] <= 1e-9:
                merged[-1] = (merged[-1][0], max(merged[-1][1], b))
            else:
                merged.append((a, b))
        clipped = merged
    # Move one centimetre inward at cut boundaries so floating-point boundary
    # classification cannot leave a quad corner just outside the pavement.
    inset = min(0.25, 0.01 / length)
    return [(p0 + direction * (a + inset), p0 + direction * (b - inset))
            for a, b in clipped if b - a > 2 * inset]


def taxiway_markings(features, runways, q: Quads, surface_features=None,
                     holding_surface_features=None):
    """Yellow taxiway centrelines, holding-position bars and lead-in lines."""
    counts = {"taxiway": 0, "holding_position": 0, "parking_position": 0}
    for f in features:
        kind = f["aeroway"]
        if f["osm_type"] != "way" or kind not in counts:
            continue
        pts = f["points_enu_m"]
        for p0, p1 in zip(pts, pts[1:]):
            p0, p1 = np.array(p0), np.array(p1)
            L = float(np.linalg.norm(p1 - p0))
            steps = max(1, int(math.ceil(L / 10.0)))
            for k in range(steps):
                a, b = p0 + (p1 - p0) * k / steps, p0 + (p1 - p0) * (k + 1) / steps
                allowed = (holding_surface_features if kind == "holding_position" and
                           holding_surface_features is not None else surface_features)
                if kind == "holding_position":
                    u = (b - a) / max(np.linalg.norm(b - a), 1e-9); n = np.array([-u[1], u[0]])
                    for off in (-0.9, -0.3, 0.3, 0.9):
                        candidates = ([(a + off * n, b + off * n)] if allowed is None else
                                      clip_marking_segment(a + off * n, b + off * n, 0.3, allowed))
                        for start, end in candidates:
                            if not inside_any_runway((start + end) / 2, runways, 3.0):
                                q.segment(start, end, 0.3, YELLOW_UV)
                else:
                    width = 0.5 if kind == "taxiway" else 0.35
                    candidates = [(a, b)] if allowed is None else clip_marking_segment(a, b, width, allowed)
                    for start, end in candidates:
                        if not inside_any_runway((start + end) / 2, runways, 3.0):
                            q.segment(start, end, width, YELLOW_UV)
            counts[kind] += 1
    return counts


def curvature_z(x, y, z):
    return z - (x * x + y * y) / (2 * EARTH_R)


def quads_mesh(q: Quads, z):
    verts, idx = bytearray(), []
    for k, (corners, uv) in enumerate(q.quads):
        for (x, y) in corners:
            verts += struct.pack("<8fI", x, y, curvature_z(x, y, z), 0, 0, 1, *uv, 0)
        b = 4 * k
        idx += [b, b + 1, b + 2, b, b + 2, b + 3]
    return bytes(verts), struct.pack(f"<{len(idx)}I", *idx), len(q.quads) * 2


# ----------------------------------------------------------- lights ---

def airport_lights(runways, features):
    lights = []
    def put(p, z, rgb, kind, cd):
        lights.append((float(p[0]), float(p[1]), z, *rgb, kind, cd))
    for rw in runways:
        low, high, w = np.array(rw["low"]), np.array(rw["high"]), rw["width"]
        L = float(np.linalg.norm(high - low)); u = (high - low) / L; n = np.array([-u[1], u[0]])
        for k in range(int(L // 60) + 1):
            for side in (-1, 1):
                put(low + u * (60 * k) + n * side * (w / 2 + 1.0), 0.5, (255, 250, 235), 1, 10000)
        caution_index = 0
        for k in range(int(L // 15)):
            a = 7.5 + 15 * k
            remaining = L - a
            if remaining <= 300.0:
                red = True
            elif remaining <= 900.0:
                # Active low -> high runway direction: alternate red/white
                # throughout the 900-to-300 m caution zone.
                red = caution_index % 2 == 0
                caution_index += 1
            else:
                red = False
            # Runway centreline lights belong on the surveyed runway axis.  A
            # former 0.6 m cross-track offset tried to keep the points visible
            # over the painted centreline, but made the lights visibly miss
            # both the OSM axis and the georegistered NAIP centreline. The
            # inset fixtures are flush with the painted runway surface;
            # retain only a tiny clearance to avoid depth fighting.
            put(low + u * a, MARKING_Z + CENTERLINE_LIGHT_CLEARANCE_M,
                (255, 40, 30) if red else (255, 250, 235), 2, 5000)
        for k in range(int(w // 3) + 1):
            off = -w / 2 + 3 * k
            put(low - u * 2 + n * off, 0.5, (40, 255, 90), 1, 10000)
            put(high + u * 2 + n * off, 0.5, (40, 255, 90), 1, 10000)
        for end, dirn in ((low, -u), (high, u)):
            for k in range(1, 31):
                for off in (-4, -2, 0, 2, 4):
                    put(end + dirn * (30 * k) + n * off, 1.0, (255, 255, 255), 5, 20000)
    for f in features if TAXIWAY_LIGHTS else ():
        if f["aeroway"] != "taxiway" or f["osm_type"] != "way":
            continue
        pts = f["points_enu_m"]; carry = 0.0
        for p0, p1 in zip(pts, pts[1:]):
            p0, p1 = np.array(p0), np.array(p1); L = float(np.linalg.norm(p1 - p0))
            a = carry
            while a < L:
                p = p0 + (p1 - p0) * (a / L)
                if not inside_any_runway(p, runways, 3.0):
                    put(p, 0.25, (30, 230, 90), 4, 200)
                a += 30.0
            carry = a - L
    return lights


# ------------------------------------------------------------ terrain ---

def terrain_meshes(dem, bbox, detail_bbox, grid, origin_llh, airport_bbox):
    """One grid in lon/lat; cells inside ``detail_bbox`` form the detail mesh."""
    cols, rows = grid
    west, south, east, north = bbox
    lon = west + (east - west) * np.arange(cols) / (cols - 1)
    lat = north + (south - north) * np.arange(rows) / (rows - 1)
    LON, LAT = np.meshgrid(lon, lat)
    U, V = (LON - west) / (east - west), (north - LAT) / (north - south)
    h = sample(dem, U, V)
    h = np.maximum(h, WATER_LEVEL_M)
    E, N, Z = geodetic_to_enu(LAT, LON, h, (origin_llh[1], origin_llh[0], FIELD_ELEVATION_MSL_M))
    # The airfield is flat at the runway height in the sim (JSBSim flat ground):
    # keep terrain at or below z = 0 there, ramping out 5 % beyond the box.
    aw, as_, ae, an = airport_bbox
    ae_e, an_n, _ = geodetic_to_enu(an, ae, FIELD_ELEVATION_MSL_M, (origin_llh[1], origin_llh[0], FIELD_ELEVATION_MSL_M))
    aw_e, as_n, _ = geodetic_to_enu(as_, aw, FIELD_ELEVATION_MSL_M, (origin_llh[1], origin_llh[0], FIELD_ELEVATION_MSL_M))
    dx = np.maximum(0, np.maximum(aw_e - E, E - ae_e)); dy = np.maximum(0, np.maximum(as_n - N, N - an_n))
    Z = np.minimum(Z, curvature_z(E, N, 0.0) + 0.05 * np.hypot(dx, dy))
    P = np.stack([E, N, Z], axis=-1)
    lr = np.roll(P, -1, axis=1) - np.roll(P, 1, axis=1); lr[:, 0] = P[:, 1] - P[:, 0]; lr[:, -1] = P[:, -1] - P[:, -2]
    ud = np.roll(P, 1, axis=0) - np.roll(P, -1, axis=0); ud[0] = P[0] - P[1]; ud[-1] = P[-2] - P[-1]
    Nrm = np.cross(lr, ud); Nrm /= np.maximum(np.linalg.norm(Nrm, axis=-1, keepdims=True), 1e-12)
    dw, ds, de, dn = detail_bbox
    # Detail box on even grid lines: the context mesh uses 2x2-cell quads.
    c0 = 2 * int(math.floor((dw - west) / (east - west) * (cols - 1) / 2))
    c1 = 2 * int(math.ceil((de - west) / (east - west) * (cols - 1) / 2))
    r0 = 2 * int(math.floor((north - dn) / (north - south) * (rows - 1) / 2))
    r1 = 2 * int(math.ceil((north - ds) / (north - south) * (rows - 1) / 2))
    # No cracks: detail-boundary vertices between two coarse vertices lie on
    # the coarse edge.
    for r in range(r0 + 1, r1, 2):
        for c in (c0, c1):
            P[r, c] = (P[r - 1, c] + P[r + 1, c]) / 2
    for c in range(c0 + 1, c1, 2):
        for r in (r0, r1):
            P[r, c] = (P[r, c - 1] + P[r, c + 1]) / 2

    def mesh(cells, uv_of, step=1):
        used = sorted({v for (r, c) in cells for v in ((r, c), (r + step, c), (r, c + step), (r + step, c + step))})
        index = {v: i for i, v in enumerate(used)}
        verts = bytearray()
        for (r, c) in used:
            uu, vv = uv_of(r, c)
            verts += struct.pack("<8fI", *P[r, c], *Nrm[r, c], uu, vv, 0)
        idx = []
        for (r, c) in cells:
            a, b, cc, d = index[r, c], index[r + step, c], index[r, c + step], index[r + step, c + step]
            idx += [a, b, cc, cc, b, d]
        return bytes(verts), struct.pack(f"<{len(idx)}I", *idx), len(used), len(idx) // 3

    inner = [(r, c) for r in range(r0, r1) for c in range(c0, c1)]
    if (cols - 1) % 2 or (rows - 1) % 2:
        raise ValueError("grid needs an even cell count for the 2x2 context quads")
    outer = [(r, c) for r in range(0, rows - 1, 2) for c in range(0, cols - 1, 2) if not (r0 <= r < r1 and c0 <= c < c1)]
    ctx = mesh(outer, lambda r, c: (float(U[r, c]), float(V[r, c])), step=2)
    det = mesh(inner, lambda r, c: (float((c - c0) / (c1 - c0)), float((r - r0) / (r1 - r0))))
    snapped = (west + (east - west) * c0 / (cols - 1), north + (south - north) * r1 / (rows - 1),
               west + (east - west) * c1 / (cols - 1), north + (south - north) * r0 / (rows - 1))
    stats = {"grid": [cols, rows], "cell_m": [float(E[0, 1] - E[0, 0]), float(N[0, 0] - N[1, 0])], "context_cell": "2x2 grid cells",
             "z_range_m": [float(Z.min()), float(Z.max())], "detail_cells": [c1 - c0, r1 - r0],
             "context": {"vertices": ctx[2], "triangles": ctx[3]}, "detail": {"vertices": det[2], "triangles": det[3]}}
    return ctx, det, snapped, stats


def connected_water(dem):
    """Water = below the water level AND connected to the Bay or the Pacific.

    Isolated lows (garages, pits, levee-ringed basins under the airfield) are
    raised to 2 m so they are neither water nor holes in the terrain.
    """
    below = Image.fromarray(np.where(dem < WATER_LEVEL_M, 255, 0).astype(np.uint8), "L").copy()
    h, w = dem.shape
    # Seeds: every deep (< -2 m) sample on a coarse grid; the Bay and the
    # Pacific are not connected inside the box (the Golden Gate is north of it).
    for r in range(0, h, 32):
        for c in range(0, w, 32):
            if dem[r, c] < -2.0 and below.getpixel((c, r)) == 255:
                ImageDraw.floodfill(below, (c, r), 128)
    m = np.asarray(below)
    lows = m == 255
    fixed = np.where(lows, np.maximum(dem, 2.0), dem).astype(np.float32)
    return fixed, int(lows.sum())


def ground_texture(naip, dem, bbox, sub_bbox, size, force_land=()):
    """NAIP crop of ``sub_bbox`` resized to ``size`` with the water mask in alpha."""
    west, south, east, north = bbox
    sw, ss, se, sn = sub_bbox
    W, H = naip.size
    top, bottom = (north - sn) / (north - south), (north - ss) / (north - south)
    box = ((sw - west) / (east - west) * W, top * H,
           (se - west) / (east - west) * W, bottom * H)
    rgb = naip.resize(size, Image.Resampling.LANCZOS, box=box)
    u = (sw + (se - sw) * (np.arange(size[0]) + 0.5) / size[0] - west) / (east - west)
    v = (north - (sn + (ss - sn) * (np.arange(size[1]) + 0.5) / size[1])) / (north - south)
    UU, VV = np.meshgrid(u, v)
    h = sample(dem, UU, VV)
    depth = np.clip((WATER_LEVEL_M - h) / 8.0, 0.0, 1.0)
    alpha = np.where(h >= WATER_LEVEL_M, 1.0, 0.6 * (1.0 - depth))
    # SFO is reclaimed low land: elevation alone cannot distinguish pavement
    # from the Bay. FAA airport polygons are authoritative forced-land masks.
    if force_land:
        mask = Image.new("L", size, 0)
        draw = ImageDraw.Draw(mask)
        for feature in force_land:
            pixel_rings = []
            for ring in feature["rings_wgs84"]:
                pixel_rings.append([((lon - sw) / (se - sw) * (size[0] - 1),
                                     (sn - lat) / (sn - ss) * (size[1] - 1))
                                    for lon, lat in ring])
            draw.polygon(pixel_rings[0], fill=255)
            for hole in pixel_rings[1:]:
                draw.polygon(hole, fill=0)
        alpha[np.asarray(mask) > 0] = 1.0
    a = Image.fromarray(np.round(alpha * 255).astype(np.uint8), "L").filter(ImageFilter.GaussianBlur(1.2))
    img = rgb.convert("RGBA"); img.putalpha(a)
    return img, float((alpha < 0.7).mean())


def _box_blur_axis(values, radius, axis):
    """Float32 moving average without a scipy dependency or window expansion."""
    radius = int(radius)
    if radius <= 0:
        return values.astype(np.float32, copy=True)
    pads = [(0, 0), (0, 0)]
    pads[axis] = (radius, radius)
    padded = np.pad(values.astype(np.float32, copy=False), pads, mode="edge")
    prefix_shape = list(padded.shape); prefix_shape[axis] = 1
    prefix = np.concatenate((np.zeros(prefix_shape, dtype=np.float32),
                             np.cumsum(padded, axis=axis, dtype=np.float32)), axis=axis)
    leading = [slice(None), slice(None)]; trailing = [slice(None), slice(None)]
    leading[axis] = slice(0, values.shape[axis])
    trailing[axis] = slice(2 * radius + 1, 2 * radius + 1 + values.shape[axis])
    return (prefix[tuple(trailing)] - prefix[tuple(leading)]) / float(2 * radius + 1)


def _smooth_float(values, radius):
    """Three box passes approximate a Gaussian while keeping memory O(pixels)."""
    radius = int(round(radius))
    if radius <= 0:
        return values.astype(np.float32, copy=True)
    out = values.astype(np.float32, copy=False)
    for _ in range(3):
        out = _box_blur_axis(out, radius, 1)
        out = _box_blur_axis(out, radius, 0)
    return out


def _normalised_smooth(values, weights, radius, fallback=0.0):
    denominator = _smooth_float(weights, radius)
    numerator = _smooth_float(values * weights, radius)
    return np.divide(numerator, denominator, out=np.full_like(numerator, fallback),
                     where=denominator > 1e-4), denominator


def _robust_rgb_reduce(image, factor, chunk_rows=64):
    """Block-median RGB reduction, chunked so the 106 MP source stays bounded."""
    width, height = image.size
    low_width = (width + factor - 1) // factor
    low_height = (height + factor - 1) // factor
    reduced = np.empty((low_height, low_width, 3), dtype=np.uint8)
    for low_top in range(0, low_height, chunk_rows):
        low_bottom = min(low_top + chunk_rows, low_height)
        top, bottom = low_top * factor, min(height, low_bottom * factor)
        source = np.asarray(image.crop((0, top, width, bottom)).convert("RGB"), dtype=np.uint8)
        pad_y = (low_bottom - low_top) * factor - source.shape[0]
        pad_x = low_width * factor - source.shape[1]
        if pad_y or pad_x:
            source = np.pad(source, ((0, pad_y), (0, pad_x), (0, 0)), mode="edge")
        blocks = source.reshape(low_bottom - low_top, factor, low_width, factor, 3)
        reduced[low_top:low_bottom] = np.median(blocks, axis=(1, 3)).astype(np.uint8)
    return reduced


def _reduced_classes(classes, size, class_ids=(1, 2, 4)):
    """Dominant material class per reduced cell; zero wins if coverage is weak."""
    best_class = np.zeros((size[1], size[0]), dtype=np.uint8)
    best_coverage = np.full(best_class.shape, 127, dtype=np.uint8)
    for class_id in class_ids:
        mask = classes.point(lambda value, wanted=class_id: 255 if value == wanted else 0)
        coverage = np.asarray(mask.resize(size, Image.Resampling.BOX), dtype=np.uint8)
        selected = coverage > best_coverage
        best_class[selected] = class_id
        best_coverage[selected] = coverage[selected]
    return best_class


def _dilate_bool(mask, radius):
    radius = int(radius)
    if radius <= 0:
        return mask
    size = 2 * radius + 1
    return np.asarray(Image.fromarray(mask.astype(np.uint8) * 255, "L").filter(
        ImageFilter.MaxFilter(size))) > 0


def _source_material_tone(image, classes, metres_per_pixel):
    """Sanitised, source-derived apron/taxiway tone at a coarse resolution.

    Narrow paint, coloured shoulders, aircraft and their shadows are rejected
    before interpolation.  Only spatial bands below 150 m survive, preventing
    the NAIP export/mosaic exposure field from becoming pavement weathering.
    """
    factor = max(1, int(round(4.8 / max(metres_per_pixel, 1e-6))))
    factor = min(factor, max(1, min(image.size)))
    rgb = _robust_rgb_reduce(image, factor)
    low_size = (rgb.shape[1], rgb.shape[0])
    material_class = _reduced_classes(classes, low_size)
    low_metres = metres_per_pixel * factor
    luma = ((rgb[..., 0].astype(np.float32) * 54.0 +
             rgb[..., 1].astype(np.float32) * 183.0 +
             rgb[..., 2].astype(np.float32) * 19.0) / 256.0)
    chroma = rgb.max(axis=2).astype(np.int16) - rgb.min(axis=2).astype(np.int16)
    tone = np.zeros(luma.shape, dtype=np.float32)
    stats = {}

    for class_id, name, gain, source_ids in (
            (1, "apron", 1.0, (1,)),
            (2, "taxiway", 0.85, (2,))):
        source_domain = np.isin(material_class, source_ids)
        output_domain = source_domain | ((material_class == 4) if class_id == 2 else False)
        preliminary = source_domain & (chroma <= 32) & (luma >= 45.0) & (luma <= 215.0)
        baseline, baseline_weight = _normalised_smooth(
            luma, preliminary.astype(np.float32), 24.0 / max(low_metres, 1e-6),
            float(np.median(luma[source_domain])) if source_domain.any() else 128.0)
        residual = np.abs(luma - baseline)
        rejected = source_domain & (~preliminary | (baseline_weight <= 1e-4) | (residual > 14.0))
        rejected = _dilate_bool(rejected, math.ceil(3.0 / max(low_metres, 1e-6)))
        rejected |= _dilate_bool(source_domain & (residual > 28.0),
                                 math.ceil(10.0 / max(low_metres, 1e-6)))
        valid = source_domain & ~rejected
        fallback = float(np.median(luma[valid])) if valid.any() else (
            float(np.median(luma[source_domain])) if source_domain.any() else 128.0)
        filled = luma.astype(np.float32, copy=True)
        unresolved = output_domain & ~valid
        for support_m in (12.0, 30.0, 70.0):
            estimate, weight = _normalised_smooth(
                luma, valid.astype(np.float32), support_m / max(low_metres, 1e-6), fallback)
            selected = unresolved & (weight > 1e-4)
            filled[selected] = estimate[selected]
            unresolved &= ~selected
        filled[unresolved] = fallback

        domain_weight = output_domain.astype(np.float32)
        b6, _ = _normalised_smooth(filled, domain_weight, 6.0 / max(low_metres, 1e-6), fallback)
        b24, _ = _normalised_smooth(filled, domain_weight, 24.0 / max(low_metres, 1e-6), fallback)
        b45, _ = _normalised_smooth(filled, domain_weight, 45.0 / max(low_metres, 1e-6), fallback)
        b80, _ = _normalised_smooth(filled, domain_weight, 80.0 / max(low_metres, 1e-6), fallback)
        b150, _ = _normalised_smooth(filled, domain_weight, 150.0 / max(low_metres, 1e-6), fallback)
        measured = (0.65 * np.clip(b6 - b24, -3.0, 3.0) +
                    0.90 * np.clip(b24 - b80, -5.0, 5.0) +
                    0.75 * np.clip(b80 - b150, -4.0, 4.0) +
                    0.50 * np.clip(b24 - b45, -3.0, 3.0))
        tone[output_domain] = measured[output_domain] * gain
        stats[name] = {"source_cells": int(source_domain.sum()),
                       "valid_cells": int(valid.sum()),
                       "valid_fraction": float(valid.sum() / max(1, source_domain.sum()))}
    return tone, factor, stats


def _sample_reduced(values, xs, ys, factor):
    """Bilinear sampling at global pixel centres; independent of output tiling."""
    fx = (xs.astype(np.float64) + 0.5) / factor - 0.5
    fy = (ys.astype(np.float64) + 0.5) / factor - 0.5
    x0 = np.clip(np.floor(fx).astype(np.int64), 0, values.shape[1] - 1)
    y0 = np.clip(np.floor(fy).astype(np.int64), 0, values.shape[0] - 1)
    x1, y1 = np.minimum(x0 + 1, values.shape[1] - 1), np.minimum(y0 + 1, values.shape[0] - 1)
    tx, ty = (fx - x0)[None, :], (fy - y0)[:, None]
    return ((values[y0[:, None], x0[None, :]] * (1.0 - tx) +
             values[y0[:, None], x1[None, :]] * tx) * (1.0 - ty) +
            (values[y1[:, None], x0[None, :]] * (1.0 - tx) +
             values[y1[:, None], x1[None, :]] * tx) * ty).astype(np.float32)


def _apron_taxiway_seam(classes, box):
    """One-pixel material joint at real apron/taxiway class transitions."""
    left, top, right, bottom = box
    halo_box = (max(0, left - 1), max(0, top - 1), min(classes.width, right + 1),
                min(classes.height, bottom + 1))
    halo = np.asarray(classes.crop(halo_box), dtype=np.uint8)
    padded = np.pad(halo, 1, mode="constant")
    neighbours_1 = np.zeros(halo.shape, dtype=bool)
    neighbours_2 = np.zeros(halo.shape, dtype=bool)
    for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        view = padded[1 + dy:1 + dy + halo.shape[0], 1 + dx:1 + dx + halo.shape[1]]
        neighbours_1 |= view == 1
        neighbours_2 |= view == 2
    seam = ((halo == 1) & neighbours_2) | ((halo == 2) & neighbours_1)
    x0, y0 = left - halo_box[0], top - halo_box[1]
    return seam[y0:y0 + bottom - top, x0:x0 + right - left]


def paint_faa_pavement(image, bbox, aprons, taxiways, runways, buildings=(), tile_size=512):
    """Rasterize clean FAA pavement into the airport-detail texture.

    The polygon source remains vector and auditable, but baking at the native
    ~0.6 m texture resolution removes photographed aircraft and old paint with
    no new runtime texture, material, mesh, or draw batch. Layer priority is
    apron < taxiway < runway. Alpha is retained from the forced-land mask.
    """
    width, height = image.size
    west, south, east, north = bbox

    def pixels(ring):
        return [((lon - west) / (east - west) * (width - 1),
                 (north - lat) / (north - south) * (height - 1))
                for lon, lat in ring]

    classes = Image.new("L", image.size, 0)
    layer_specs = ((1, "apron", aprons), (2, "taxiway", taxiways),
                   (3, "runway", runways))
    feature_counts = {}
    for class_id, name, features in layer_specs:
        layer = Image.new("L", image.size, 0)
        draw = ImageDraw.Draw(layer)
        for feature in sorted(features, key=lambda item: (
                item["properties"].get("OBJECTID", 0), item.get("polygon", 0))):
            rings = feature["rings_wgs84"]
            draw.polygon(pixels(rings[0]), fill=255)
            for hole in rings[1:]:
                draw.polygon(pixels(hole), fill=0)
        classes.paste(class_id, mask=layer)
        feature_counts[name] = len(features)
        if name == "taxiway":
            taxiway_mask = layer

    # Cover the discoloured photographic fringe around isolated FAA taxiway
    # polygons with an eight-metre paved shoulder and two-metre feather. Do
    # not overwrite aprons or runways; they retain higher-priority classes.
    mid_lat = math.radians(0.5 * (south + north))
    metres_x = abs(east - west) * 111_320.0 * math.cos(mid_lat)
    metres_y = abs(north - south) * 110_574.0
    metres_per_pixel = 0.5 * (metres_x / width + metres_y / height)
    shoulder_radius_px = max(1, int(math.ceil(8.0 / max(metres_per_pixel, 1e-6))))
    feather_radius_px = max(1, int(math.ceil(2.0 / max(metres_per_pixel, 1e-6))))
    shoulder_radius_px = min(shoulder_radius_px, 31)
    feather_radius_px = min(feather_radius_px, 15)
    # A full-resolution smooth field avoids the block-sized curved edges from
    # reduced-resolution morphology. The calibrated thresholds put the solid
    # edge at about 8 m and the zero-opacity edge at about 10 m.
    blur_radius_px = max(1.0, shoulder_radius_px * (10.0 / 14.0))
    distance_field = taxiway_mask.filter(ImageFilter.GaussianBlur(blur_radius_px))
    solid_threshold, outer_threshold = 20, 8
    outer = distance_field.point(lambda value: 255 if value >= outer_threshold else 0)
    opacity_lut = [max(0, min(255, round(
        (value - outer_threshold) * 255 / (solid_threshold - outer_threshold))))
                   for value in range(256)]
    shoulder_opacity = distance_field.point(opacity_lut)
    shoulder = ImageChops.subtract(outer, taxiway_mask)
    empty = classes.point(lambda value: 255 if value == 0 else 0)
    shoulder = ImageChops.multiply(shoulder, empty)
    shoulder_opacity = ImageChops.multiply(shoulder_opacity, empty)
    classes.paste(4, mask=shoulder)

    # Building footprints are not rendered in 3D in this slice. Preserve the
    # aerial roofs instead of painting pavement across them.
    building_mask = Image.new("L", image.size, 0)
    building_draw = ImageDraw.Draw(building_mask)
    for feature in sorted(buildings, key=lambda item: (
            item["properties"].get("OBJECTID", 0), item.get("polygon", 0))):
        rings = feature["rings_wgs84"]
        building_draw.polygon(pixels(rings[0]), fill=255)
        for hole in rings[1:]:
            building_draw.polygon(pixels(hole), fill=0)
    classes.paste(0, mask=building_mask)
    shoulder_opacity = ImageChops.multiply(
        shoulder_opacity, building_mask.point(lambda value: 255 - value))
    del distance_field, outer, shoulder, empty, building_mask, layer

    # Neutral physically plausible base colours. Apron and taxiway variation
    # now comes from sanitised NAIP structure rather than an invented periodic
    # or noise field; the accepted runway path below remains byte-for-byte v7.
    bases = np.asarray(((0, 0, 0), (162, 164, 160), (120, 123, 120),
                        (94, 97, 96), (103, 106, 103)), dtype=np.float32)
    histogram = classes.histogram()
    source_tone, source_factor, source_stats = _source_material_tone(
        image, classes, metres_per_pixel)
    for top in range(0, height, tile_size):
        bottom = min(top + tile_size, height)
        for left in range(0, width, tile_size):
            right = min(left + tile_size, width)
            box = (left, top, right, bottom)
            cls = np.asarray(classes.crop(box), dtype=np.uint8)
            if not cls.any():
                continue
            tile = np.asarray(image.crop(box), dtype=np.uint8).copy()
            x = np.arange(left, right, dtype=np.int64)[None, :]
            y = np.arange(top, bottom, dtype=np.int64)[:, None]
            hashed = ((x * 73856093) ^ (y * 19349663)) & 255
            measured_tone = _sample_reduced(
                source_tone, np.arange(left, right), np.arange(top, bottom), source_factor)
            # Preserve the accepted v6/v7 runway appearance byte-for-byte;
            # this slice changes only apron, taxiway and shoulder material.
            accepted_tone = ((np.sin(x / 71.0) + np.sin(y / 89.0) +
                              np.sin((x + y) / 149.0)) * 4.0 +
                             (np.sin((x + 2 * y) / 23.0) +
                              np.sin((3 * x - y) / 41.0)) * 2.0 +
                             (hashed.astype(np.float32) - 127.5) * 0.055)
            material_seam = _apron_taxiway_seam(classes, box)
            for class_id in (1, 2, 3, 4):
                selected = cls == class_id
                if selected.any():
                    tone = measured_tone if class_id in (1, 2, 4) else accepted_tone
                    material = np.clip(bases[class_id] + tone[..., None], 0, 255).astype(np.uint8)
                    if class_id in (1, 2):
                        material[material_seam] = np.maximum(
                            material[material_seam].astype(np.int16) - 5, 0).astype(np.uint8)
                    if class_id == 4:
                        opacity = np.asarray(shoulder_opacity.crop(box), dtype=np.float32) / 255.0
                        blend = (tile[..., :3] * (1.0 - opacity[..., None]) +
                                 material * opacity[..., None]).astype(np.uint8)
                        tile[..., :3][selected] = blend[selected]
                    else:
                        tile[..., :3][selected] = material[selected]
            image.paste(Image.fromarray(tile, "RGBA"), box)

    pixels_by_layer = {name: histogram[class_id] for class_id, name, _ in layer_specs}
    pixels_by_layer["taxiway_shoulder"] = histogram[4]
    total = sum(pixels_by_layer.values())
    return image, {"method": "FAA polygons baked into native-resolution detail texture",
                   "layer_order": ["apron", "taxiway_shoulder", "taxiway", "runway"],
                   "features": feature_counts, "pixels": pixels_by_layer,
                   "total_pixels": total, "detail_fraction": total / (width * height),
                   "building_cutouts": len(buildings),
                   "taxiway_shoulder_width_m": 8.0,
                   "taxiway_shoulder_feather_m": 2.0,
                   "taxiway_shoulder_radius_px": shoulder_radius_px,
                   "taxiway_shoulder_feather_px": feather_radius_px,
                   "taxiway_shoulder_edge_method": "full-resolution Gaussian distance field",
                   "taxiway_shoulder_blur_radius_px": blur_radius_px,
                   "material_tone": {"method": "sanitised source-derived luminance bands",
                                     "reduction_factor": source_factor,
                                     "band_scales_m": [6.0, 24.0, 45.0, 80.0, 150.0],
                                     "rejection": {"chroma_dn": 32, "luma_dn": [45, 215],
                                                   "local_delta_dn": 14, "strong_delta_dn": 28,
                                                   "dilation_m": [3.0, 10.0]},
                                     "apron_taxiway_seam": {"width_px_each_side": 1,
                                                            "darkening_dn": 5},
                                     "classes": source_stats},
                   "runtime_draw_batches_added": 0}


def _aeroway_width_m(tags, default):
    """Read the simple OSM width forms used at SFO, with a safe default."""
    value = tags.get("width", "")
    try:
        text = value.strip().lower()
        if text.endswith("ft"):
            return float(text[:-2].strip()) * 0.3048
        if text.endswith("m"):
            text = text[:-1].strip()
        width = float(text)
        return width if 2.0 <= width <= 150.0 else default
    except (AttributeError, TypeError, ValueError):
        return default


def clean_airport_pavement(image, detail_bbox, aeroways):
    """Replace transient NAIP objects on operational pavement, offline.

    OSM geometry defines only the replacement masks. A local low-frequency
    pavement estimate suppresses transient bright/dark objects while retaining
    a bounded amount of the original fine detail. Alpha is kept byte-for-byte
    so this pass cannot alter the terrain water mask.
    """
    rgba = np.asarray(image.convert("RGBA"), dtype=np.uint8).copy()
    source_rgb = rgba[:, :, :3].copy()
    height, width = rgba.shape[:2]
    west, south, east, north = detail_bbox
    mid_lat = (south + north) * 0.5
    metres_x = (east - west) * 111_320.0 * math.cos(math.radians(mid_lat))
    metres_y = (north - south) * 111_320.0
    pixels_per_metre = 0.5 * (width / metres_x + height / metres_y)

    def pixel_points(points):
        return [((lon - west) / (east - west) * (width - 1),
                 (north - lat) / (north - south) * (height - 1)) for lon, lat in points]

    masks = {name: Image.new("L", (width, height), 0) for name in ("apron", "taxiway", "runway")}
    counts = {name: 0 for name in masks}
    for feature in aeroways:
        kind = feature["aeroway"]
        if kind not in masks:
            continue
        draw = ImageDraw.Draw(masks[kind])
        paths = feature.get("member_ways", []) if feature["osm_type"] == "relation" else [feature]
        for path in paths:
            points = pixel_points(path.get("points_wgs84", []))
            if len(points) < 2:
                continue
            if kind == "apron" and len(points) >= 4 and math.dist(points[0], points[-1]) <= 1.5:
                draw.polygon(points, fill=255)
            elif kind != "apron":
                default = 23.0 if kind == "taxiway" else 45.0
                line_width = max(2, round(_aeroway_width_m(feature["tags"], default) * pixels_per_metre))
                draw.line(points, fill=255, width=line_width, joint="curve")
            else:
                continue
            counts[kind] += 1

    surface_pixels = {}
    for kind in ("apron", "taxiway", "runway"):
        selected = np.asarray(masks[kind]) > 0
        surface_pixels[kind] = int(selected.sum())
    apron_or_taxiway = (np.asarray(masks["apron"]) > 0) | (np.asarray(masks["taxiway"]) > 0)
    # Preserve the excellent source runway texture and baked rubber deposits;
    # the reported transient aircraft are on aprons/taxiways. Excluding runway
    # overlap also prevents legitimate white runway paint from becoming blur.
    surface = apron_or_taxiway & ~(np.asarray(masks["runway"]) > 0)

    # A roughly 28 m box reduction removes aircraft-sized detail while retaining
    # broad lighting/color changes across the airport. Only pixels that differ
    # strongly from that local pavement baseline are candidates for cleanup.
    source_image = Image.fromarray(source_rgb, "RGB")
    baseline_span = max(15, round(28.0 * pixels_per_metre))
    coarse_size = (max(2, width // baseline_span), max(2, height // baseline_span))
    baseline = np.asarray(source_image.resize(coarse_size, Image.Resampling.BOX).resize(
                          (width, height), Image.Resampling.BICUBIC), dtype=np.int16)
    source_i16 = source_rgb.astype(np.int16)
    # Accumulate one channel at a time in int32: the green term exceeds int16,
    # but full-frame int32 RGB copies waste hundreds of MiB on the host.
    def luma(rgb):
        out = rgb[:, :, 0].astype(np.int32) * 54
        out += rgb[:, :, 1].astype(np.int32) * 183
        out += rgb[:, :, 2].astype(np.int32) * 19
        return out // 256
    source_luma, base_luma = luma(source_i16), luma(baseline)
    luma_delta = source_luma - base_luma
    candidates = surface & ((luma_delta >= 45) | (luma_delta <= -55))
    # Grow around the detected fuselage/wing pixels to include soft shadows and
    # antialiased edges, but never extend beyond operational pavement.
    growth = max(3, round(8.0 * pixels_per_metre)) | 1
    candidates = (np.asarray(Image.fromarray(candidates.astype(np.uint8) * 255, "L").filter(
                  ImageFilter.MaxFilter(growth))) > 0) & surface
    rng = np.random.default_rng(0x53464F)
    selected_source, selected_base = source_i16[candidates], baseline[candidates]
    retained_detail = np.clip(selected_source - selected_base, -8, 8)
    replacement = np.clip(selected_base + retained_detail +
                          rng.normal(0.0, 0.6, (len(selected_base), 1)), 0, 255).astype(np.uint8)
    rgba[:, :, :3][candidates] = replacement
    changed = {kind: int((candidates & (np.asarray(mask) > 0)).sum()) for kind, mask in masks.items()}

    stats = {"method": "OSM-bounded local outlier replacement with seeded pavement grain",
             "features": counts, "surface_pixels": surface_pixels, "pixels": changed,
             "total_pixels": int(candidates.sum()), "detail_fraction": float(candidates.mean())}
    return Image.fromarray(rgba, "RGBA"), stats


def astc_mips(levels, executable, quality="-fast"):
    """Raw ASTC 6x6 sRGB blocks per mip (16-byte .astc header stripped)."""
    payloads = []
    with tempfile.TemporaryDirectory(prefix="gx-astc-") as td:
        td = pathlib.Path(td)
        for number, level in enumerate(levels):
            source, target = td / f"{number}.png", td / f"{number}.astc"
            level.save(source)
            subprocess.run([str(executable), "-cs", str(source), str(target), "6x6", quality],
                           check=True, stdout=subprocess.DEVNULL)
            container = target.read_bytes()
            expected = math.ceil(level.width / 6) * math.ceil(level.height / 6) * 16
            if container[:4] != b"\x13\xab\xa1\x5c" or len(container) - 16 != expected:
                raise ValueError(f"invalid ASTC output for mip {number}")
            payloads.append(container[16:])
    return payloads


# ----------------------------------------------------------- aircraft ---

def aircraft_animation_channel(name, x, y, z):
    """Conservative rigid channel fitted to the static bomb-5000 AI mesh."""
    if name != "body": return 0
    ay = abs(y)
    if 6.8 <= ay <= 13.8 and 0.35 <= z <= 1.65 and x < -0.55 * ay + 1.65:
        return 1 if y >= 0 else 2
    if ay >= 0.25 and z <= -3.55 and x < -0.75 * ay - 11.2:
        return 3 if y >= 0 else 4
    if ay <= 0.3 and z <= -0.7 and x < 0.75 * z - 9.5:
        return 5
    return 0

def assert_rigid_aircraft_triangles(vertices, indices):
    """Every triangle must be wholly static or owned by one rigid channel."""
    if len(vertices) % 36 or len(indices) % 12:
        raise ValueError("invalid animated aircraft buffer stride")
    channels = [struct.unpack_from("<I", vertices, offset + 32)[0]
                for offset in range(0, len(vertices), 36)]
    unpacked = struct.unpack(f"<{len(indices)//4}I", indices)
    for triangle in range(0, len(unpacked), 3):
        tri = unpacked[triangle:triangle+3]
        owned = {channels[index] for index in tri}
        if len(owned) != 1:
            raise ValueError(f"mixed animation channels in triangle {triangle//3}: {sorted(owned)}")

def aircraft_mesh(ac_path):
    """bomb-5000 AC3D -> body-FRD vertices about the JSBSim CG, main gear on the contacts."""
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    import spike002_aircraft as importer
    materials, objects, warnings = importer.parse_ac3d(ac_path)
    built = []
    for obj in objects:
        for m in sorted({s.material for s in obj.surfaces}):
            v, i = importer.build_mesh(obj, m)
            if i:
                built.append((obj.name, v, i))
    # Importer frame is (forward, left, up) (proper rotation of AC3D's y-up).
    gear = [v for name, vs, _ in built if name == "gear" for v in vs]
    mains = [v for v in gear if v[0] < 6.0]
    x_main = (min(v[0] for v in mains) + max(v[0] for v in mains)) / 2
    z_ground = min(v[2] for v in gear)
    dx = (CG_X_IN - MAIN_X_IN) * 0.0254 - x_main
    dz = (CG_Z_IN - GEAR_Z_IN) * 0.0254
    verts, idx, stats = bytearray(), [], {}
    channel_counts = {"static": 0, "right_aileron": 0, "left_aileron": 0,
                      "right_elevator": 0, "left_elevator": 0, "rudder": 0}
    names = ["static", "right_aileron", "left_aileron", "right_elevator", "left_elevator", "rudder"]
    for name, vs, ii in built:
        transformed = []
        for v in vs:
            x, y, z, nx, ny, nz, uu, vv = v
            p = (x + dx, -y, -(z - z_ground) + dz)
            transformed.append((*p, nx, -ny, -nz, uu, vv))
        # Channel ownership is decided per triangle. Reuse a vertex only
        # within the same channel; duplicate hinge-boundary vertices so a
        # triangle can never be sheared between two rigid transforms.
        remap = {}
        before = len(verts) // 36
        for triangle in range(0, len(ii), 3):
            source = ii[triangle:triangle+3]
            centroid = tuple(sum(transformed[k][axis] for k in source) / 3.0 for axis in range(3))
            animation = aircraft_animation_channel(name, *centroid)
            for k in source:
                key = (k, animation)
                out_index = remap.get(key)
                if out_index is None:
                    out_index = len(verts) // 36
                    remap[key] = out_index
                    verts += struct.pack("<8fI", *transformed[k], animation)
                    channel_counts[names[animation]] += 1
                idx.append(out_index)
        stats[name] = {"source_vertices": len(vs), "vertices": len(verts) // 36 - before,
                       "triangles": len(ii) // 3}
    xs = [v[0] for _, vs, _ in built for v in vs]; ys = [v[1] for _, vs, _ in built for v in vs]; zs = [v[2] for _, vs, _ in built for v in vs]
    stats["bounds_m"] = {"length": max(xs) - min(xs), "span": max(ys) - min(ys), "height": max(zs) - min(zs)}
    stats["placement"] = {"main_gear_model_x_m": x_main, "ground_model_z_m": z_ground, "dx_m": dx, "dz_m": dz}
    stats["animation_vertices"] = channel_counts
    stats["source_materials"] = len(materials); stats["warnings"] = warnings
    vertex_bytes = bytes(verts)
    index_bytes = struct.pack(f"<{len(idx)}I", *idx)
    assert_rigid_aircraft_triangles(vertex_bytes, index_bytes)
    return vertex_bytes, index_bytes, len(verts) // 36, stats


def placeholder_aircraft_mesh():
    """A deliberately tiny intermediate mesh, removed by b787_pack.py."""
    verts = bytearray()
    for x, y, z, u, v in ((-1.0, 0.0, 0.0, 0.0, 0.0),
                          (1.0, 0.0, 0.0, 1.0, 0.0),
                          (0.0, 0.0, 1.0, 0.5, 1.0)):
        verts += struct.pack("<8fI", x, y, z, 0.0, -1.0, 0.0, u, v, 0)
    return bytes(verts), struct.pack("<3I", 0, 1, 2), 3, {
        "source_vertices": 3, "triangles": 1, "vertices": 3,
        "warnings": ["generated intermediate placeholder; replace before runtime use"],
    }


# ------------------------------------------------------------- packing ---

def encode_gxvisual(textures, materials, meshes):
    """GXVISUAL 1.1 (animation channel per vertex), little-endian."""
    out = bytearray(struct.pack("<8sHHQQIII", b"GXVISUAL", 1, 1, FRAME_ID, FRAME_GENERATION,
                                len(textures), len(materials), len(meshes)))
    for tid, w, h, enc, levels in textures:
        out += struct.pack("<IIIB3xI", tid, w, h, enc, len(levels))
        for raw in levels:
            out += struct.pack("<Q", len(raw)) + raw
    for mid, tid, factor, rough, metal in materials:
        out += struct.pack("<II6f", mid, tid, *factor, rough, metal)
    for kind, mid, verts, idx in meshes:
        out += struct.pack("<B3xIII", kind, mid, len(verts) // 36, len(idx) // 4) + verts + idx
    return bytes(out)


def convert(acquisition, source_manifest, output, ac_path, ac_texture, astcenc, *, detail_size=(11520, 9216),
            context_size=(4096, 3072), grid=(513, 385)):
    acquisition, output = pathlib.Path(acquisition), pathlib.Path(output)
    source, acquired = verify_acquisition(acquisition, source_manifest); area = source["area"]
    bbox, airport, origin = area["bbox_wgs84"], area["airport_bbox_wgs84"], area["runtime_origin_wgs84"]
    output.mkdir(parents=True, exist_ok=True)
    naip_context = stitch_raster(acquisition, acquired, "usda_naip_2024", (4, 4), "RGB")
    naip_detail = stitch_raster(acquisition, acquired, "usda_naip_2024_airport_detail", (3, 3), "RGB",
                                 exact_extents=True)
    dem_img = stitch_raster(acquisition, acquired, "usgs_3dep_dem", (2, 2), "F")
    dem = np.asarray(dem_img, dtype=np.float32)
    if (dem < -1000).any():
        raise ValueError("3DEP mosaic contains nodata")
    dem, lows_filled = connected_water(dem)
    detail_req = (airport[0] - 0.006, airport[1] - 0.005, airport[2] + 0.006, airport[3] + 0.005)
    ctx_mesh, det_mesh, detail_bbox, terrain_stats = terrain_meshes(dem, bbox, detail_req, grid, origin, airport)
    aeroways = parse_osm_aeroways(list((acquisition / "raw/osm_ksfo_aeroways").glob("*.osm")), origin)
    faa_runway_features = parse_faa_geojson(
        acquisition / "raw/faa_ksfo_runways/faa_ksfo_runways.geojson", "runway", origin)
    faa_taxiway_features = parse_faa_geojson(
        acquisition / "raw/faa_ksfo_taxiways/faa_ksfo_taxiways.geojson", "taxiway", origin)
    faa_apron_features = parse_faa_geojson(
        acquisition / "raw/faa_ksfo_aprons/faa_ksfo_aprons.geojson", "apron", origin)
    faa_building_features = parse_faa_geojson(
        acquisition / "raw/faa_ksfo_buildings/faa_ksfo_buildings.geojson", "building", origin)
    faa_land = faa_runway_features + faa_taxiway_features + faa_apron_features + faa_building_features
    det_img, det_water = ground_texture(naip_detail, dem, area["airport_detail_bbox_wgs84"], detail_bbox,
                                        detail_size, faa_land)
    det_img.convert("RGB").resize((1536, 960), Image.Resampling.LANCZOS).save(
        output / "preview-detail-before-pavement.jpg", quality=88)
    det_img, pavement_stats = paint_faa_pavement(
        det_img, detail_bbox, faa_apron_features, faa_taxiway_features,
        faa_runway_features, faa_building_features)
    ctx_img, ctx_water = ground_texture(naip_context, dem, bbox, bbox, context_size)

    osm_runways = runways_from_osm(aeroways)
    runways = runways_from_faa(faa_runway_features,
                               {runway["name"]: runway["width"] for runway in osm_runways})
    white, yellow = Quads(), Quads()
    for rw in runways:
        runway_markings(rw, white)
    taxi_counts = taxiway_markings(
        aeroways, runways, yellow, faa_taxiway_features + faa_apron_features,
        faa_taxiway_features)
    decal = Quads(); decal.quads = white.quads + yellow.quads
    decal_mesh = quads_mesh(decal, MARKING_Z)
    lights = airport_lights(runways, aeroways)

    if ac_path is not None:
        ac_verts, ac_idx, ac_nv, ac_stats = aircraft_mesh(ac_path)
        with Image.open(ac_texture) as im:
            ac_img = ImageOps.flip(im.convert("RGBA"))   # GL t=0 is the image bottom (AC3D UVs)
        ac_source = {"source": str(ac_path), "ac_sha256": sha256_file(ac_path),
                     "texture_sha256": sha256_file(ac_texture),
                     "license": "caller-supplied source; retain its license and notices"}
    else:
        ac_verts, ac_idx, ac_nv, ac_stats = placeholder_aircraft_mesh()
        ac_img = Image.new("RGBA", (2, 2), (128, 128, 128, 255))
        ac_source = {"source": "generated intermediate placeholder",
                     "license": "GPL-2.0-or-later (project source)"}

    def tex(img, quality):
        levels = mip_chain(img)
        return [lvl for lvl in astc_mips(levels, astcenc, quality)], levels[0].size
    det_levels, det_dims = tex(det_img, "-fast")
    ctx_levels, ctx_dims = tex(ctx_img, "-fast")
    ac_levels, ac_dims = tex(ac_img, "-medium")
    palette = bytes([235, 235, 228, 255, 230, 178, 38, 255] * 2)
    textures = [(1, *ctx_dims, 1, ctx_levels), (2, *det_dims, 1, det_levels), (3, 2, 2, 2, [palette]), (4, *ac_dims, 1, ac_levels)]
    materials = [(1, 1, (1, 1, 1, 1), 0.9, 0.0), (2, 2, (1, 1, 1, 1), 0.9, 0.0),
                 (3, 3, (1, 1, 1, 1), 0.8, 0.0), (4, 4, (1, 1, 1, 1), 0.35, 0.0)]
    meshes = [(1, 1, ctx_mesh[0], ctx_mesh[1]), (1, 2, det_mesh[0], det_mesh[1]),
              (3, 3, decal_mesh[0], decal_mesh[1]), (2, 4, ac_verts, ac_idx)]
    pack = encode_gxvisual(textures, materials, meshes)
    if len(pack) > MAX_OUTPUT_BYTES: raise ValueError(f"GXVISUAL exceeds {MAX_OUTPUT_BYTES} byte ceiling")
    pack_path = output / "sfo-photoreal.gxvisual"; pack_path.write_bytes(pack)

    # World source for gx-app spike002-world (flat ENU; WorldMesh applies curvature).
    lines = ["gx-spike002-world/1", f"# converter v{CONVERTER_VERSION}; FAA runways {[r['name'] for r in runways]}",
             f"frame {FRAME_ID:016x} {FRAME_GENERATION}",
             f"origin {origin[1]:.7f} {origin[0]:.7f} {FIELD_ELEVATION_MSL_M}",
             "bounds -12000 -12000 -50 12000 12000 600"]
    for rw in runways:
        low, high, w = np.array(rw["low"]), np.array(rw["high"]), rw["width"]
        u = (high - low) / np.linalg.norm(high - low); n = np.array([-u[1], u[0]]) * w / 2
        corners = [low - n, high - n, high + n, low + n]
        lines.append(f"runway {rw['name']} " + " ".join(f"{c[0]:.3f} {c[1]:.3f} {RUNWAY_Z}" for c in corners))
    for corners, _ in white.quads:
        lines.append("marking " + " ".join(f"{x:.3f} {y:.3f} {MARKING_Z}" for x, y in corners))
    for l in lights:
        lines.append("light {:.3f} {:.3f} {} {} {} {} {} {}".format(*l))
    world_path = output / "sfo-real.world.txt"; world_path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")

    aeroway_path = output / "aeroways.json"; aeroway_path.write_bytes(canonical_json({"schema": "gx-osm-aeroways/1", "features": aeroways}))
    preview = det_img.convert("RGB").resize((1536, 960), Image.Resampling.LANCZOS)
    preview.save(output / "preview-detail.jpg", quality=88)
    def native_crop(name, lon, lat, extent=2048):
        west, south, east, north = detail_bbox
        x = round((lon - west) / (east - west) * det_img.width)
        y = round((north - lat) / (north - south) * det_img.height)
        half = extent // 2
        det_img.convert("RGB").crop((x - half, y - half, x + half, y + half)).save(
            output / name, quality=92, subsampling=0)
    native_crop("preview-terminal-native.jpg", -122.3820, 37.6150)
    native_crop("preview-runway-native.jpg", -122.3931, 37.6286)
    alpha_preview = det_img.getchannel("A").resize((768, 480)); alpha_preview.save(output / "preview-water-mask.png")
    ctx_img.convert("RGB").resize((1024, 768), Image.Resampling.LANCZOS).save(output / "preview-context.jpg", quality=88)

    def tex_info(name, dims, levels, enc, quality):
        return {"name": name, "width": dims[0], "height": dims[1], "mip_levels": len(levels), "encoding": enc,
                "bytes": sum(len(x) for x in levels), "astc_quality": quality}
    result = {
        "schema": SCHEMA,
        "converter": {"name": "spike002_convert.py", "version": CONVERTER_VERSION, "sha256": sha256_file(pathlib.Path(__file__)),
                      "parameters": {"detail_size": list(detail_size), "context_size": list(context_size), "grid": list(grid),
                                     "water_level_navd88_m": WATER_LEVEL_M, "field_elevation_msl_m": FIELD_ELEVATION_MSL_M}},
        "source_manifest_sha256": acquired["source_manifest_sha256"],
        "acquisition_manifest_sha256": sha256_file(acquisition / "acquisition-manifest.json"),
        "frame_id": FRAME_ID, "frame_generation": FRAME_GENERATION, "origin_wgs84": origin, "bbox_wgs84": bbox,
        "detail_bbox_wgs84": list(detail_bbox),
        "terrain": terrain_stats, "water_fraction": {"detail": det_water, "context": ctx_water},
        "pavement_cleanup": pavement_stats,
        "water_lows_filled_px": lows_filled,
        "water_mask": "3DEP topobathymetric DEM below the water level with FAA runway/taxiway/apron/building polygons forced to land; NOAA shoreline replacement remains pending",
        "textures": [tex_info("context NAIP", ctx_dims, ctx_levels, "astc-6x6-srgb", "-fast"),
                     tex_info("detail NAIP with native-resolution FAA vector pavement", det_dims, det_levels, "astc-6x6-srgb", "-fast"),
                     tex_info("marking palette", (2, 2), [palette], "rgba8-srgb", None),
                     tex_info("bomb-5000 livery (default)", ac_dims, ac_levels, "astc-6x6-srgb", "-medium")],
        "astc_encoder": {"tool": "Arm astcenc 5.7.0", "executable_sha256": sha256_file(astcenc),
                         "per_mip_command": "astcenc -cs IN.png OUT.astc 6x6 QUALITY; strip 16-byte header"},
        "faa_features": {"runways": len(faa_runway_features), "taxiways": len(faa_taxiway_features),
                         "aprons": len(faa_apron_features), "buildings": len(faa_building_features)},
        "runways": runways, "markings": {"runway_quads": len(white.quads), "yellow_quads": len(yellow.quads), "osm_ways": taxi_counts,
                                          "decal_triangles": decal_mesh[2]},
        "lights": len(lights),
        "aircraft": {**ac_source, "triangles": len(ac_idx) // 12, "vertices": ac_nv, **ac_stats},
        "provenance": [{k: item[k] for k in ("id", "authority", "metadata", "license") if k in item} for item in source["sources"]],
        "files": [{"path": p.name, "bytes": p.stat().st_size, "sha256": sha256_file(p)} for p in (pack_path, world_path, aeroway_path)],
    }
    (output / "manifest.json").write_bytes(canonical_json(result))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--acquisition", type=pathlib.Path, default=pathlib.Path("build/spike-002-assets"))
    p.add_argument("--manifest", type=pathlib.Path, default=pathlib.Path("assets/spike-002/sfo-sources.v1.json"))
    p.add_argument("--output", type=pathlib.Path,
                   default=pathlib.Path("build/spike-002-runtime-v6-material"))
    p.add_argument("--ac", type=pathlib.Path,
                   help="optional AC3D preview model; omitted by the alpha 787 pipeline")
    p.add_argument("--ac-texture", type=pathlib.Path,
                   help="texture for --ac (both options must be supplied together)")
    p.add_argument("--astcenc", type=pathlib.Path, required=True,
                   help="path to the Arm ASTC Encoder executable")
    a = p.parse_args()
    if (a.ac is None) != (a.ac_texture is None):
        p.error("--ac and --ac-texture must be supplied together")
    if not a.astcenc.is_file():
        raise SystemExit(f"astcenc not found: {a.astcenc}")
    r = convert(a.acquisition, a.manifest, a.output, a.ac, a.ac_texture, a.astcenc)
    print(json.dumps({"output": str(a.output), "gxvisual_bytes": r["files"][0]["bytes"],
                      "terrain": r["terrain"], "runways": [x["name"] for x in r["runways"]],
                      "aircraft_triangles": r["aircraft"]["triangles"], "lights": r["lights"]}, sort_keys=True))
    return 0


if __name__ == "__main__": raise SystemExit(main())

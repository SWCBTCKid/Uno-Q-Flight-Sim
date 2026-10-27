#!/usr/bin/env python3
"""3D buildings for the SFO pack: OSM footprints, NOAA lidar heights.

Audit P2.1 ("modest 3D KSFO massing"). Footprints are the OSM `building`
ways and multipolygons inside the pack's detail box (ODbL, the acquisition
already holds them); the 34 FAA footprints are added where OSM has none.
Heights come from the NOAA 2025 lidar (tools/spike002_lidar.py): roof = 85th
percentile of the building-class points inside the footprint, ground =
median of the ground-class points around it. Without enough lidar the OSM
`height` / `building:levels` tags, then a per-type default, are used.

Geometry: extruded walls plus a flat roof (ear-clipped). Roofs sample the
pack's own NAIP detail texture at their footprint, so they keep the real
roof imagery. Walls get a generated facade (glass curtain wall, hangar
cladding, punched or ribbon windows, tilt-up concrete, parking deck, tower
shaft and cab) chosen from the OSM/FAA tags, name, height and size. Each
facade is its own small tileable texture, and wall UVs are in tiles (metres
over the tile size), so the renderer repeats it at true scale on one quad
per wall edge (a Building-only texture samples with GL_REPEAT). Bases sit on the pack's terrain (the detail mesh, read back from the
pack) and are sunk 0.3 m. Output: the input GXVISUAL 1.2 plus one texture
per facade, a roof material and mesh, and a material and wall mesh per
facade in use.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).parent))

TOOL_VERSION = 2
MIN_AREA_M2 = 25.0
MIN_LIDAR_POINTS = 4
SINK_M = 0.3
TYPE_HEIGHT_M = {"hangar": 16.0, "terminal": 15.0, "airport": 15.0, "transportation": 12.0,
                 "industrial": 9.0, "warehouse": 9.0, "commercial": 10.0, "office": 15.0,
                 "retail": 7.0, "house": 7.0, "residential": 9.0, "apartments": 14.0,
                 "garage": 3.5, "garages": 3.5, "shed": 3.0, "roof": 5.0, "parking": 10.0}
DEFAULT_HEIGHT_M = 8.0
ROOF_MATERIAL, FACADE_MATERIAL0, FACADE_TEXTURE0 = 8, 9, 8   # facade cell k: material 9 + k, texture 8 + k
# GPU budget (board smoke, 2026-10-24): all 2,656 detail-box buildings cost
# +3.3 ms p50 / +10 ms p99 in the high wide view and dropped it to 16 fps.
# Houses are pixel-sized there and the roof imagery already shows them, so
# only airport structures and landmarks are modelled.
AIRPORT_BBOX = (-122.405, 37.598, -122.34, 37.636)   # sfo-sources.v1.json airport_bbox_wgs84
AIRPORT_MIN = (300.0, 12.0)        # area m2 or height m, inside the airport box
LANDMARK_MIN = (2000.0, 15.0)      # elsewhere in the detail box


class BuildError(ValueError):
    pass


# ------------------------------------------------------------------ OSM ---

def parse_height(text):
    if not text:
        return None
    m = re.match(r"\s*([0-9]+(?:\.[0-9]+)?)\s*(m|ft|')?\s*$", text)
    if not m:
        return None
    v = float(m.group(1))
    return v * 0.3048 if m.group(2) in ("ft", "'") else v


def assemble_rings(ways):
    """Join way node lists into closed rings (OSM multipolygon members)."""
    rings, open_ = [], [list(w) for w in ways if len(w) >= 2]
    while open_:
        ring = open_.pop(0)
        changed = True
        while ring[0] != ring[-1] and changed:
            changed = False
            for k, w in enumerate(open_):
                if w[0] == ring[-1]:
                    ring += w[1:]
                elif w[-1] == ring[-1]:
                    ring += w[::-1][1:]
                elif w[-1] == ring[0]:
                    ring = w[:-1] + ring
                elif w[0] == ring[0]:
                    ring = w[::-1][:-1] + ring
                else:
                    continue
                open_.pop(k)
                changed = True
                break
        if ring[0] == ring[-1] and len(ring) >= 4:
            rings.append(ring)
    return rings


def osm_buildings(paths):
    nodes, ways, rels = {}, {}, {}
    for path in sorted(paths):
        root = ET.parse(path).getroot()
        for n in root.findall("node"):
            nodes[int(n.attrib["id"])] = (float(n.attrib["lon"]), float(n.attrib["lat"]))
        for w in root.findall("way"):
            ways[int(w.attrib["id"])] = ([int(x.attrib["ref"]) for x in w.findall("nd")],
                                         {t.attrib["k"]: t.attrib["v"] for t in w.findall("tag")})
        for r in root.findall("relation"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in r.findall("tag")}
            if tags.get("type") == "multipolygon" and "building" in tags:
                rels[int(r.attrib["id"])] = ([(int(m.attrib["ref"]), m.attrib.get("role", "")) for m in r.findall("member")
                                              if m.attrib.get("type") == "way"], tags)
    out = []
    for wid, (refs, tags) in sorted(ways.items()):
        # Outlines, plus building parts that carry their own height (the
        # control tower is a 0-60 m shaft and a 60-67 m cab).
        wanted = "building" in tags or ("building:part" in tags and parse_height(tags.get("height")))
        if wanted and len(refs) >= 4 and refs[0] == refs[-1] and all(r in nodes for r in refs):
            out.append({"source": f"osm:way/{wid}", "tags": tags, "lonlat": [nodes[r] for r in refs[:-1]]})
    for rid, (members, tags) in sorted(rels.items()):
        outer = [ways[w][0] for w, role in members if role in ("outer", "") and w in ways]
        for k, ring in enumerate(assemble_rings(outer)):
            if all(r in nodes for r in ring):
                out.append({"source": f"osm:relation/{rid}#{k}", "tags": tags, "lonlat": [nodes[r] for r in ring[:-1]]})
    return out


def faa_buildings(path):
    g = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    out = []
    for f in g["features"]:
        geom = f["geometry"]
        polys = [geom["coordinates"]] if geom["type"] == "Polygon" else geom["coordinates"]
        for k, poly in enumerate(polys):
            ring = [tuple(p[:2]) for p in poly[0]]
            if ring[0] == ring[-1]:
                ring = ring[:-1]
            tags = {"building": "airport"}
            if f["properties"].get("DESIGNATOR"):
                tags["name"] = f["properties"]["DESIGNATOR"]
            out.append({"source": f"faa:{f['properties'].get('OBJECTID')}#{k}", "tags": tags,
                        "lonlat": ring})
    return out


# ------------------------------------------------------------- polygons ---

def signed_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def clean_ring(p):
    """Drop repeated and collinear vertices; CCW."""
    keep = [p[0]]
    for q in p[1:]:
        if np.hypot(*(q - keep[-1])) > 0.05:
            keep.append(q)
    if len(keep) > 1 and np.hypot(*(keep[0] - keep[-1])) <= 0.05:
        keep.pop()
    p = np.array(keep)
    changed = True
    while changed and len(p) > 3:
        changed = False
        for i in range(len(p)):
            a, b, c = p[i - 1], p[i], p[(i + 1) % len(p)]
            if abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 1e-3:
                p = np.delete(p, i, axis=0)
                changed = True
                break
    if signed_area(p) < 0:
        p = p[::-1]
    return p


def ear_clip(p):
    """Triangulate a simple CCW polygon; returns index triples."""
    idx = list(range(len(p)))
    tris = []

    def inside(pt, a, b, c):
        d1 = (pt[0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (pt[1] - b[1])
        d2 = (pt[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (pt[1] - c[1])
        d3 = (pt[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (pt[1] - a[1])
        neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
        pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
        return not (neg and pos)

    guard = 0
    while len(idx) > 3 and guard < 10 * len(p) * len(p):
        guard += 1
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = p[i0], p[i1], p[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-9:
                continue
            if any(inside(p[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break
    if len(idx) == 3:
        tris.append(tuple(idx))
    if len(tris) != len(p) - 2:
        raise BuildError("polygon is not simple")
    return tris


def points_in_polygon(px, py, poly):
    inside = np.zeros(len(px), dtype=bool)
    x, y = poly[:, 0], poly[:, 1]
    j = len(poly) - 1
    for i in range(len(poly)):
        cond = ((y[i] > py) != (y[j] > py)) & (px < (x[j] - x[i]) * (py - y[i]) / (y[j] - y[i] + 1e-30) + x[i])
        inside ^= cond
        j = i
    return inside


# ------------------------------------------------------------- the pack ---

def read_pack(data):
    magic, major, minor, fid, fgen, nt, nm, ng = struct.unpack_from("<8sHHQQIII", data, 0)
    if magic != b"GXVISUAL" or major != 1 or minor != 2:
        raise BuildError(f"input pack must be GXVISUAL 1.2, got {major}.{minor}")
    off = struct.calcsize("<8sHHQQIII")
    textures, materials, meshes = [], [], []
    for _ in range(nt):
        tid, w, h, enc, levels = struct.unpack_from("<IIIB3xI", data, off)
        start = off
        off += 20
        for _ in range(levels):
            (n,) = struct.unpack_from("<Q", data, off)
            off += 8 + n
        textures.append((tid, data[start:off]))
    for _ in range(nm):
        mid, tid = struct.unpack_from("<II", data, off)
        materials.append((mid, tid, data[off:off + 32]))
        off += 32
    for _ in range(ng):
        kind, mid, nv, ni = struct.unpack_from("<B3xIII", data, off)
        rec = data[off:off + 16 + nv * 36 + ni * 4]
        meshes.append((kind, mid, nv, ni, rec))
        off += len(rec)
    joints = data[off:]
    return {"frame_id": fid, "frame_generation": fgen, "textures": textures, "materials": materials,
            "meshes": meshes, "joints": joints}


def detail_grid(pack):
    """The terrain detail mesh (material 2) as a regular grid of ENU positions."""
    recs = [r for kind, mid, nv, ni, r in pack["meshes"] if kind == 1 and mid == 2]
    if len(recs) != 1:
        raise BuildError("pack has no single detail terrain mesh (material 2)")
    rec = recs[0]
    _, _, nv, _ = struct.unpack_from("<B3xIII", rec, 0)
    v = np.frombuffer(rec, dtype=np.float32, count=nv * 9, offset=16).reshape(nv, 9)
    u, w = v[:, 6].astype(np.float64), v[:, 7].astype(np.float64)
    du = np.diff(np.unique(np.round(u, 7)))
    dv = np.diff(np.unique(np.round(w, 7)))
    cols, rows = int(round(1 / du.min())), int(round(1 / dv.min()))
    grid = np.full((rows + 1, cols + 1, 3), np.nan)
    c = np.rint(u * cols).astype(int)
    r = np.rint(w * rows).astype(int)
    grid[r, c] = v[:, :3]
    if np.isnan(grid).any():
        raise BuildError("detail mesh is not a full grid")
    return grid


def bilinear(grid, u, v):
    rows, cols = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = np.clip(u * cols, 0, cols - 1e-9), np.clip(v * rows, 0, rows - 1e-9)
    c0, r0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = (x - c0)[..., None], (y - r0)[..., None]
    a, b = grid[r0, c0], grid[r0, c0 + 1]
    cc, d = grid[r0 + 1, c0], grid[r0 + 1, c0 + 1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (cc * (1 - fx) + d * fx) * fy


# ------------------------------------------------------------ buildings ---

def build(footprints, lidar, detail_bbox, origin, grid):
    from spike002_convert import geodetic_to_enu
    from scipy.spatial import cKDTree
    w, s, e, n = detail_bbox
    lat0, lon0 = origin[1], origin[0]
    enu = lambda lon, lat: np.stack(geodetic_to_enu(np.asarray(lat), np.asarray(lon), 0.0, (lat0, lon0, 0.0))[:2], axis=-1)
    le = enu(lidar["lon"], lidar["lat"])
    lz, lcls = lidar["z"].astype(np.float64), lidar["cls"]
    bpts, gpts = le[lcls == 6], le[lcls == 2]
    bz, gz = lz[lcls == 6], lz[lcls == 2]
    btree, gtree = cKDTree(bpts), cKDTree(gpts)
    roofs, walls = [], []
    stats = {"footprints_in_box": 0, "kept": 0, "skipped_small": 0, "skipped_invalid": 0,
             "height_source": {"lidar": 0, "osm_height": 0, "osm_levels": 0, "type_default": 0}, "heights_m": []}
    seen = []
    for k, fp in enumerate(footprints):
        ll = np.array(fp["lonlat"], dtype=np.float64)
        if not ((ll[:, 0] >= w).all() and (ll[:, 0] <= e).all() and (ll[:, 1] >= s).all() and (ll[:, 1] <= n).all()):
            continue
        stats["footprints_in_box"] += 1
        p = enu(ll[:, 0], ll[:, 1])
        try:
            order = clean_ring(p)
        except Exception:
            stats["skipped_invalid"] += 1
            continue
        if len(order) < 3 or abs(signed_area(order)) < MIN_AREA_M2:
            stats["skipped_small"] += 1
            continue
        centroid = order.mean(axis=0)
        if fp["source"].startswith("faa:") and any(points_in_polygon(np.array([centroid[0]]), np.array([centroid[1]]), q)[0] for q in seen):
            continue
        # Map ENU back to lon/lat for texture UVs and terrain height (the
        # footprint's own vertices, after cleaning, are a subset of p).
        lookup = {tuple(np.round(q, 6)): ll[i] for i, q in enumerate(p)}
        lonlat = np.array([lookup[tuple(np.round(q, 6))] for q in order])
        u = (lonlat[:, 0] - w) / (e - w)
        v = (n - lonlat[:, 1]) / (n - s)
        ground = bilinear(grid, u, v)[:, 2]
        try:
            tris = ear_clip(order)
        except BuildError:
            stats["skipped_invalid"] += 1
            continue
        radius = float(np.max(np.hypot(*(order - centroid).T)))
        cand = btree.query_ball_point(centroid, radius)
        height, source = None, None
        if cand:
            cand = np.array(cand)
            inside = points_in_polygon(bpts[cand, 0], bpts[cand, 1], order)
            roof_pts = bz[cand[inside]]
            gnear = gtree.query_ball_point(centroid, radius + 30.0)
            if len(roof_pts) >= MIN_LIDAR_POINTS and len(gnear) >= MIN_LIDAR_POINTS:
                gnear = np.array(gnear)
                outside = ~points_in_polygon(gpts[gnear, 0], gpts[gnear, 1], order)
                if outside.sum() >= MIN_LIDAR_POINTS:
                    height = float(np.percentile(roof_pts, 85) - np.median(gz[gnear[outside]]))
                    source = "lidar"
        tags = fp["tags"]
        part = "building:part" in tags and "building" not in tags
        if part:
            height, source = parse_height(tags.get("height")), "osm_height"
        if height is None or not (2.0 <= height <= 120.0):
            h = parse_height(tags.get("height"))
            if h:
                height, source = h, "osm_height"
            elif parse_height(tags.get("building:levels")):
                height, source = 3.3 * parse_height(tags.get("building:levels")), "osm_levels"
            else:
                height, source = TYPE_HEIGHT_M.get(tags.get("building"), DEFAULT_HEIGHT_M), "type_default"
        height = float(min(max(height, 2.5), 120.0))
        stats["height_source"][source] += 1
        stats["heights_m"].append(round(height, 1))
        stats["kept"] += 1
        if not part:
            seen.append(order)
        base = float(ground.min()) - SINK_M
        top = float(np.median(ground)) + height
        min_h = parse_height(tags.get("min_height")) if part else None
        if min_h and min_h < height:
            # A raised part: walls from min_height, and an underside.
            base = float(np.median(ground)) + min_h
            roofs.append((order, base, u, v, [(t[0], t[2], t[1]) for t in tris], -1.0))
        aw, as_, ae, an = AIRPORT_BBOX
        lon_c, lat_c = lonlat[:, 0].mean(), lonlat[:, 1].mean()
        min_area, min_height = AIRPORT_MIN if (aw <= lon_c <= ae and as_ <= lat_c <= an) else LANDMARK_MIN
        if not part and abs(signed_area(order)) < min_area and height < min_height:
            stats["kept"] -= 1
            stats["height_source"][source] -= 1
            stats["heights_m"].pop()
            stats["skipped_scope"] = stats.get("skipped_scope", 0) + 1
            if roofs and len(roofs[-1]) == 6 and roofs[-1][5] < 0 and roofs[-1][0] is order:
                roofs.pop()
            continue
        style = classify(tags, height, abs(signed_area(order)), min_area == AIRPORT_MIN[0], bool(part and min_h and min_h < height))
        variants = FACADES[style][3]
        cell = variants[hashlib.sha256(fp["source"].encode()).digest()[0] % len(variants)]
        stats.setdefault("facades", {}).setdefault(style, 0)
        stats["facades"][style] += 1
        roofs.append((order, top, u, v, tris, 1.0))
        walls.append((order, base, top, style, cell))
    return roofs, walls, stats


# -------------------------------------------------------------- facades ---

TILE_PX = 256
# style: (tile width m, tile height m, anchor, facade cells). "top" anchors
# the pattern at the roof line (parapets, clerestories); "bottom" at the
# ground (doors).
FACADES = {
    "glass":      (12.0, 9.0, "top", (0, 1)),
    "hangar":     (8.0, 16.0, "top", (2, 3)),
    "office":     (12.0, 7.2, "top", (4, 5)),
    "ribbon":     (12.0, 7.2, "top", (11,)),
    "industrial": (15.0, 10.0, "bottom", (6, 7)),
    "parking":    (9.0, 6.0, "top", (8,)),
    "shaft":      (6.0, 12.0, "top", (9,)),
    "cab":        (6.0, 7.0, "top", (10,)),
}
GLASS_NAMES = re.compile(r"terminal|boarding area|concourse|intl|international|customs", re.I)
HANGAR_NAMES = re.compile(r"hangar|superbay|maintenance|cargo", re.I)
PARKING_NAMES = re.compile(r"garage|parking", re.I)


def classify(tags, height, area, in_airport, raised):
    """Facade style from tags, name and massing (OSM is mostly building=yes)."""
    kind, name = tags.get("building") or tags.get("building:part", ""), tags.get("name", "")
    if height >= 40.0 and area < 1500.0:
        return "cab" if raised else "shaft"
    if kind in ("parking", "garage", "garages") or PARKING_NAMES.search(name):
        return "parking"
    if kind == "hangar" or HANGAR_NAMES.search(name):
        return "hangar"
    if kind == "terminal" or GLASS_NAMES.search(name):
        return "glass"
    if height >= 18.0:
        if kind in ("hotel", "apartments", "residential"):
            return "office"
        return "ribbon" if int(hashlib.sha256(name.encode()).digest()[0]) % 2 else "office"
    if in_airport and area >= 3000.0 and height >= 10.0:
        return "hangar"
    if kind in ("airport", "transportation", "commercial", "office", "retail") and height >= 9.0:
        return "office"
    return "industrial"


def _tile(style, variant):
    """One TILE_PX square tile, row 0 = bottom of the tile, sRGB 0..1."""
    w_m, h_m = FACADES[style][:2]
    n = TILE_PX
    x = (np.arange(n) + 0.5) / n * w_m           # metres across
    y = (np.arange(n) + 0.5) / n * h_m           # metres up
    X, Y = np.meshgrid(x, y)
    rng = np.random.default_rng(1000 + 17 * FACADES[style][3][0] + variant)
    img = np.ones((n, n, 3))

    def fill(mask, rgb):
        img[mask] = rgb

    if style in ("glass", "cab"):
        floor = 4.5 if style == "glass" else h_m
        glass = [(0.30, 0.42, 0.50), (0.36, 0.44, 0.42)][variant % 2] if style == "glass" else (0.16, 0.22, 0.27)
        fy = (Y % floor) / floor
        sky = 0.80 + 0.35 * fy                    # brighter up each pane: sky reflection
        img[:] = np.array(glass) * sky[..., None]
        pane = (X // 1.5).astype(int) + 7 * (Y // floor).astype(int)
        img *= (0.92 + 0.16 * rng.random(pane.max() + 1))[pane][..., None]
        fill((X % 1.5) < 0.08, (0.72, 0.74, 0.76))  # mullions
        if style == "glass":
            fill((Y % floor) < 0.9, (0.70, 0.72, 0.73))   # spandrel
            fill(np.abs((Y % floor) - 0.9) < 0.05, (0.55, 0.57, 0.58))
        else:
            fill(Y > h_m - 0.8, (0.85, 0.85, 0.83))       # cab roof band
            fill(Y < 0.5, (0.78, 0.78, 0.76))
    elif style == "hangar":
        clad = np.array([(0.86, 0.87, 0.87), (0.52, 0.60, 0.68)][variant % 2])
        rib = 0.90 + 0.10 * np.cos(2 * np.pi * X / 0.33)
        img[:] = clad * rib[..., None]
        fill((Y % 4.0) < 0.05, clad * 0.80)                # panel seams
        fill((Y > h_m - 1.8) & (Y < h_m - 0.4) & ((X % 2.0) > 0.12), (0.38, 0.45, 0.50))  # clerestory
        fill(Y > h_m - 0.4, clad * 0.75)                   # fascia
    elif style in ("office", "ribbon"):
        if style == "office":
            wall = np.array([(0.82, 0.76, 0.66), (0.88, 0.87, 0.84)][variant % 2])
        else:
            wall = np.array((0.70, 0.71, 0.72))
        img[:] = wall
        fy = Y % 3.6
        if style == "office":
            win = ((X % 3.0) > 0.6) & ((X % 3.0) < 2.4) & (fy > 0.9) & (fy < 2.6)
            fill(((X % 3.0) > 0.5) & ((X % 3.0) < 2.5) & (np.abs(fy - 0.85) < 0.06), wall * 0.8)  # sills
        else:
            win = (fy > 1.0) & (fy < 2.7)
        shade = 0.80 + 0.40 * ((fy - 0.9) / 1.8)
        img[win] = (np.array((0.24, 0.31, 0.37)) * shade[..., None])[win]
        fill(win & ((X % 1.5) < 0.07), (0.60, 0.62, 0.63))
        fill(Y > h_m - 0.35, wall * 0.85)                  # parapet cap
    elif style == "industrial":
        wall = np.array([(0.80, 0.75, 0.65), (0.70, 0.71, 0.70)][variant % 2])
        stain = 0.94 + 0.06 * np.sin(X * 1.7 + rng.random() * 6) * np.exp(-Y / 4.0)
        img[:] = wall * stain[..., None]
        fill((X % 7.5) < 0.05, wall * 0.78)                # panel joints
        door = (X > 2.0) & (X < 6.0) & (Y < 4.5)
        fill(door, (0.62, 0.63, 0.63))                      # roll-up door
        fill(door & ((Y % 0.3) < 0.03), (0.50, 0.51, 0.51))
        fill((X > 10.0) & (X < 11.0) & (Y < 2.2), (0.35, 0.33, 0.30))  # personnel door
        fill((Y > 6.5) & (Y < 7.3) & ((X % 3.75) > 0.6) & ((X % 3.75) < 3.1), (0.27, 0.32, 0.36))  # high windows
        fill(Y > h_m - 0.3, wall * 0.82)
    elif style == "parking":
        img[:] = (0.78, 0.77, 0.74)
        fy = Y % 3.0
        deck = (fy > 1.0) & (fy < 2.85)
        fill(deck, (0.13, 0.13, 0.14))                      # open deck
        fill(deck & ((X % 9.0) < 0.6), (0.70, 0.69, 0.66))  # columns
        fill((fy > 1.0) & (fy < 1.15), (0.55, 0.55, 0.53))  # barrier
    elif style == "shaft":
        img[:] = (0.86, 0.85, 0.82)
        img *= (0.93 + 0.07 * np.cos(2 * np.pi * X / 1.5))[..., None]   # fluting
        fill(((X % 6.0) > 2.7) & ((X % 6.0) < 3.3) & ((Y % 4.0) > 1.0) & ((Y % 4.0) < 3.2), (0.25, 0.30, 0.34))
    noise = 1.0 + 0.025 * rng.standard_normal((n, n))
    return np.clip(img * noise[..., None], 0, 1)


def facade_texture(cell):
    """The tileable texture of facade cell ``cell``; GL t = 0 is image row 0."""
    style, variant = next((st, cells.index(cell)) for st, (_, _, _, cells) in FACADES.items() if cell in cells)
    rgba = np.concatenate([np.round(_tile(style, variant) * 255), np.full((TILE_PX, TILE_PX, 1), 255.0)], axis=2)
    return Image.fromarray(rgba.astype(np.uint8), "RGBA")


def roof_mesh(roofs):
    verts, idx = bytearray(), []
    for order, top, u, v, tris, up in roofs:
        b = len(verts) // 36
        for (x, y), uu, vv in zip(order, u, v):
            verts += struct.pack("<8fI", x, y, top, 0.0, 0.0, up, uu, vv, 0)
        for t in tris:
            idx += [b + t[0], b + t[1], b + t[2]]
    return bytes(verts), idx


def wall_mesh(walls):
    """Walls per facade cell: {cell: (vertex bytes, indices)}. One quad per
    edge; UVs count tiles (u from the edge start, v from the anchor), so a
    repeating texture lands at true scale and is never stretched."""
    out = {}
    for order, base, top, style, cell in walls:
        tw, th, anchor = FACADES[style][:3]
        verts, idx = out.setdefault(cell, (bytearray(), []))
        v_of = (lambda z: 1.0 + (z - top) / th) if anchor == "top" else (lambda z: (z - base) / th)
        for i in range(len(order)):
            a, b = order[i], order[(i + 1) % len(order)]
            d = b - a
            ln = math.hypot(*d)
            if ln < 1e-6:
                continue
            nx, ny = d[1] / ln, -d[0] / ln          # outward for a CCW ring
            k = len(verts) // 36
            for (x, y), z, uu in ((a, base, 0.0), (b, base, ln / tw), (b, top, ln / tw), (a, top, 0.0)):
                verts += struct.pack("<8fI", x, y, z, nx, ny, 0.0, uu, v_of(z), 0)
            idx += [k, k + 1, k + 2, k, k + 2, k + 3]
    return {cell: (bytes(v), i) for cell, (v, i) in sorted(out.items())}


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--pack", type=pathlib.Path, required=True, help="directory with sfo-photoreal.gxvisual (1.2) and manifest.json")
    p.add_argument("--acquisition", type=pathlib.Path, required=True)
    p.add_argument("--lidar", type=pathlib.Path, required=True)
    p.add_argument("--output", type=pathlib.Path, required=True)
    a = p.parse_args(argv)
    from spike002_convert import mip_chain
    manifest = json.loads((a.pack / "manifest.json").read_text(encoding="utf-8"))
    pack = read_pack((a.pack / "sfo-photoreal.gxvisual").read_bytes())
    ours = range(ROOF_MATERIAL, FACADE_MATERIAL0 + sum(len(f[3]) for f in FACADES.values()))
    if any(kind == 4 for kind, *_ in pack["meshes"]) or any(mid in ours for mid, _, _ in pack["materials"]) \
            or any(tid >= FACADE_TEXTURE0 for tid, _ in pack["textures"]):
        raise BuildError("input pack already has buildings, or ids collide with the facade ids")
    grid = detail_grid(pack)
    fps = osm_buildings(list((a.acquisition / "raw/osm_ksfo_aeroways").glob("*.osm")))
    fps += faa_buildings(a.acquisition / "raw/faa_ksfo_buildings/faa_ksfo_buildings.geojson")
    lid = np.load(a.lidar)
    lidar = {k: lid[k] for k in ("lon", "lat", "z", "cls")}
    roofs, walls, stats = build(fps, lidar, manifest["detail_bbox_wgs84"], manifest["origin_wgs84"], grid)
    rv, ri = roof_mesh(roofs)
    walls_by_cell = wall_mesh(walls)
    textures = [rec for _, rec in pack["textures"]]
    materials = [rec for _, _, rec in pack["materials"]]
    meshes = [rec for *_, rec in pack["meshes"]]
    materials.append(struct.pack("<II6f", ROOF_MATERIAL, 2, 1.0, 1.0, 1.0, 1.0, 0.85, 0.0))
    building_meshes = [(ROOF_MATERIAL, rv, ri)]
    for cell, (wv, wi) in walls_by_cell.items():
        levels = mip_chain(facade_texture(cell))
        tex = bytearray(struct.pack("<IIIB3xI", FACADE_TEXTURE0 + cell, levels[0].width, levels[0].height, 2, len(levels)))
        for lvl in levels:
            raw = lvl.tobytes()
            tex += struct.pack("<Q", len(raw)) + raw
        textures.append(bytes(tex))
        materials.append(struct.pack("<II6f", FACADE_MATERIAL0 + cell, FACADE_TEXTURE0 + cell, 1.0, 1.0, 1.0, 1.0, 0.85, 0.0))
        building_meshes.append((FACADE_MATERIAL0 + cell, wv, wi))
    for mid, verts, idx in building_meshes:
        meshes.append(struct.pack("<B3xIII", 4, mid, len(verts) // 36, len(idx)) + verts + struct.pack(f"<{len(idx)}I", *idx))
    wi = [i for _, _, idx in building_meshes[1:] for i in idx]
    out = bytearray(struct.pack("<8sHHQQIII", b"GXVISUAL", 1, 2, pack["frame_id"], pack["frame_generation"],
                                len(textures), len(materials), len(meshes)))
    for rec in textures + materials + meshes:
        out += rec
    out += pack["joints"]
    a.output.mkdir(parents=True, exist_ok=True)
    path = a.output / "sfo-photoreal.gxvisual"
    path.write_bytes(bytes(out))
    hs = np.array(stats.pop("heights_m"))
    stats.update({"tool_version": TOOL_VERSION, "roof_triangles": len(ri) // 3, "wall_triangles": len(wi) // 3,
                  "height_m": {"min": float(hs.min()), "p50": float(np.median(hs)), "p95": float(np.percentile(hs, 95)), "max": float(hs.max())},
                  "lidar": json.loads(str(lid["provenance"])), "input_pack_sha256": sha256(a.pack / "sfo-photoreal.gxvisual"),
                  "output": {"bytes": len(out), "sha256": sha256(path)},
                  "sources": "OSM buildings (ODbL), FAA airport buildings, NOAA 2025 SF Bay lidar"})
    (a.output / "buildings-manifest.json").write_text(json.dumps(stats, indent=1), encoding="utf-8")
    print(json.dumps({k: stats.get(k) for k in ("footprints_in_box", "kept", "skipped_scope", "height_source", "roof_triangles", "wall_triangles", "height_m")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

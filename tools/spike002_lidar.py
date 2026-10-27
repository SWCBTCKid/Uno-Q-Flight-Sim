#!/usr/bin/env python3
"""Fetch NOAA 2025 SF Bay lidar (Entwine Point Tiles) over the SFO detail box.

Prior tools (ORG-015): PDAL's readers.ept is the reference reader
(`assets/spike-002/ksfo-lidar-pdal.v1.json`); PDAL is not installed on the
host, so this reads the same public EPT with laspy[lazrs] and keeps only
what the building heights need: ground (2), building (6) and bridge deck
(17) points, to a fixed octree depth, inside the box.

Requires: pip install "laspy[lazrs]==2.5.4" pyproj (a venv; the repo tools
use only numpy/PIL/scipy).

Output (npz): lon, lat (WGS84 degrees), z (NAVD88 metres, the EPT's
vertical datum), cls (uint8), plus the node keys and a provenance record.
Downloads are cached by node key so reruns are offline.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import pathlib
import urllib.request

import laspy
import numpy as np
from pyproj import Transformer

EPT = "https://noaa-nos-coastal-lidar-pds.s3.amazonaws.com/entwine/geoid18/14673"
KEEP_CLASSES = (2, 6, 17)


def get(url, cache: pathlib.Path):
    if cache.is_file():
        return cache.read_bytes()
    with urllib.request.urlopen(url, timeout=60) as r:
        data = r.read()
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(data)
    return data


def node_bounds(root, key):
    d, x, y, z = key
    size = (root[3] - root[0]) / (1 << d)
    return (root[0] + x * size, root[1] + y * size, root[2] + z * size,
            root[0] + (x + 1) * size, root[1] + (y + 1) * size, root[2] + (z + 1) * size)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--bbox", type=float, nargs=4, required=True, metavar=("W", "S", "E", "N"))
    p.add_argument("--depth", type=int, default=8)
    p.add_argument("--cache", type=pathlib.Path, default=pathlib.Path("build/spike-002-lidar/ept-cache"))
    p.add_argument("--output", type=pathlib.Path, default=pathlib.Path("build/spike-002-lidar/sfo-lidar.npz"))
    a = p.parse_args(argv)
    meta = json.loads(get(f"{EPT}/ept.json", a.cache / "ept.json"))
    root = meta["bounds"]
    horiz = meta["srs"]["horizontal"]
    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{horiz}", always_xy=True)
    to_ll = Transformer.from_crs(f"EPSG:{horiz}", "EPSG:4326", always_xy=True)
    w, s, e, n = a.bbox
    xs, ys = to_utm.transform([w, e, w, e], [s, s, n, n])
    box = (min(xs), min(ys), max(xs), max(ys))

    # Walk the hierarchy (sub-hierarchy files where a count is -1).
    counts = {}

    def load_hierarchy(key):
        name = "-".join(map(str, key))
        h = json.loads(get(f"{EPT}/ept-hierarchy/{name}.json", a.cache / "hierarchy" / f"{name}.json"))
        for k, v in h.items():
            kk = tuple(int(t) for t in k.split("-"))
            if kk[0] > a.depth:
                continue
            b = node_bounds(root, kk)
            if b[3] < box[0] or b[0] > box[2] or b[4] < box[1] or b[1] > box[3]:
                continue
            if v == -1:
                load_hierarchy(kk)
            else:
                counts[kk] = v

    load_hierarchy((0, 0, 0, 0))
    keys = sorted(counts)
    parts = {k: [] for k in ("x", "y", "z", "cls")}
    fetched = 0
    for key in keys:
        name = "-".join(map(str, key))
        data = get(f"{EPT}/ept-data/{name}.laz", a.cache / "data" / f"{name}.laz")
        fetched += len(data)
        las = laspy.read(io.BytesIO(data))
        cls = np.asarray(las.classification, dtype=np.uint8)
        x, y = np.asarray(las.x), np.asarray(las.y)
        keep = np.isin(cls, KEEP_CLASSES) & (x >= box[0]) & (x <= box[2]) & (y >= box[1]) & (y <= box[3])
        parts["x"].append(x[keep]); parts["y"].append(y[keep])
        parts["z"].append(np.asarray(las.z)[keep]); parts["cls"].append(cls[keep])
    x, y, z, cls = (np.concatenate(parts[k]) for k in ("x", "y", "z", "cls"))
    lon, lat = to_ll.transform(x, y)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    prov = {"ept": EPT, "ept_json_sha256": hashlib.sha256((a.cache / "ept.json").read_bytes()).hexdigest(),
            "horizontal_epsg": horiz, "vertical": meta["srs"].get("vertical"), "depth": a.depth, "bbox_wgs84": a.bbox,
            "nodes": len(keys), "bytes_fetched": fetched, "points": int(len(z)),
            "by_class": {int(c): int((cls == c).sum()) for c in KEEP_CLASSES}}
    np.savez_compressed(a.output, lon=lon, lat=lat, z=z.astype(np.float32), cls=cls, provenance=json.dumps(prov))
    print(json.dumps(prov))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

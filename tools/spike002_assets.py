#!/usr/bin/env python3
"""Plan and acquire the bounded SPIKE-002 SFO source set with provenance.

Only Python's standard library is required. The command never writes into the
source manifest directory and never downloads unless `acquire` is explicit.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import http.cookiejar
import json
import math
import os
import pathlib
import tempfile
import urllib.error
import urllib.parse
import urllib.request

ALLOWED_HOSTS = {
    "elevation.nationalmap.gov",
    "apps.geo.fpac.usda.gov",
    "api.openstreetmap.org",
    "nsde.ngs.noaa.gov",
    "noaa-nos-coastal-lidar-pds.s3.amazonaws.com",
    "services6.arcgis.com",
}


def canonical_json(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def load_manifest(path: pathlib.Path) -> dict:
    raw = path.read_bytes()
    value = json.loads(raw)
    if value.get("schema") != "gx-spike002-source-manifest/1":
        raise ValueError("unsupported source-manifest schema")
    west, south, east, north = value["area"]["bbox_wgs84"]
    if not (-180 <= west < east <= 180 and -90 <= south < north <= 90):
        raise ValueError("invalid WGS84 bounding box")
    ids = [source["id"] for source in value["sources"]]
    if len(ids) != len(set(ids)):
        raise ValueError("source ids must be unique")
    for source in value["sources"]:
        for required in ("authority", "service", "metadata", "license", "acquisition"):
            if not source.get(required):
                raise ValueError(f"{source['id']}: missing {required}")
        service = source["service"].replace("{z}", "0").replace("{x}", "0").replace("{y}", "0")
        parsed = urllib.parse.urlparse(service)
        if parsed.scheme != "https" or parsed.hostname not in ALLOWED_HOSTS:
            raise ValueError(f"{source['id']}: unapproved service host")
    for external in value.get("external_inputs", []):
        if not external.get("license") or not external.get("package_sha256"):
            raise ValueError(f"{external.get('id', '<unknown>')}: incomplete external provenance")
        if len(external["package_sha256"]) != 64:
            raise ValueError(f"{external['id']}: invalid package SHA-256")
    value["_manifest_sha256"] = sha256_bytes(raw)
    return value


def split_bbox(bbox: list[float], nx: int, ny: int):
    west, south, east, north = bbox
    for row in range(ny):
        y0 = south + (north - south) * row / ny
        y1 = south + (north - south) * (row + 1) / ny
        for col in range(nx):
            x0 = west + (east - west) * col / nx
            x1 = west + (east - west) * (col + 1) / nx
            yield col, row, [x0, y0, x1, y1]


def lon_to_tile_x(lon: float, zoom: int) -> int:
    return math.floor((lon + 180.0) / 360.0 * (1 << zoom))


def lat_to_tile_y(lat: float, zoom: int) -> int:
    lat = max(-85.05112878, min(85.05112878, lat))
    rad = math.radians(lat)
    return math.floor((1.0 - math.asinh(math.tan(rad)) / math.pi) / 2.0 * (1 << zoom))


def requests_for(manifest: dict) -> list[dict]:
    requests: list[dict] = []
    area = manifest["area"]
    for source in manifest["sources"]:
        acquisition = source["acquisition"]
        kind = acquisition["kind"]
        bbox = area[acquisition["bbox"]]
        if kind == "arcgis_export_grid":
            nx, ny = acquisition["grid"]
            width, height = acquisition["pixel_size"]
            for col, row, tile_bbox in split_bbox(bbox, nx, ny):
                params = {
                    # ArcGIS exportImage's direct `f=image` path is not
                    # reliable for the large TIFFs used here (the USDA NAIP
                    # service returned HTTP 500 while `f=json` completed the
                    # same export). Resolve the documented result href first,
                    # then stream that exact artifact with its own receipt.
                    "f": "json",
                    "bbox": ",".join(f"{number:.9f}" for number in tile_bbox),
                    "bboxSR": "4326",
                    "imageSR": "4326",
                    "size": f"{width},{height}",
                    **acquisition["parameters"],
                }
                requests.append({
                    "id": f"{source['id']}-{row:02d}-{col:02d}",
                    "source_id": source["id"],
                    "method": "GET",
                    "url": source["service"].rstrip("/") + "/exportImage?" + urllib.parse.urlencode(params),
                    "arcgis_result_href": True,
                    "path": f"raw/{source['id']}/{row:02d}-{col:02d}.{acquisition['extension']}",
                    "bbox_wgs84": tile_bbox,
                })
        elif kind == "osm_api_grid":
            nx, ny = acquisition["grid"]
            for col, row, tile_bbox in split_bbox(bbox, nx, ny):
                params = {"bbox": ",".join(f"{number:.9f}" for number in tile_bbox)}
                requests.append({
                    "id": f"{source['id']}-{row:02d}-{col:02d}",
                    "source_id": source["id"],
                    "method": "GET",
                    "url": source["service"] + "?" + urllib.parse.urlencode(params),
                    "path": f"raw/{source['id']}/{row:02d}-{col:02d}.{acquisition['extension']}",
                    "bbox_wgs84": tile_bbox,
                })
        elif kind == "web_mercator_tiles":
            west, south, east, north = bbox
            zoom = acquisition["zoom"]
            x0, x1 = lon_to_tile_x(west, zoom), lon_to_tile_x(east, zoom)
            y0, y1 = lat_to_tile_y(north, zoom), lat_to_tile_y(south, zoom)
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    requests.append({
                        "id": f"{source['id']}-{zoom}-{x}-{y}",
                        "source_id": source["id"],
                        "method": "GET",
                        "url": source["service"].format(z=zoom, x=x, y=y),
                        "path": f"raw/{source['id']}/{zoom}/{x}/{y}.{acquisition['extension']}",
                        "web_mercator_tile": [zoom, x, y],
                        # NOAA's S3 origin returns 403, not 404, for an empty
                        # sparse-vector tile. Both mean no shoreline here.
                        "optional_missing_statuses": [403, 404],
                    })
        elif kind == "arcgis_feature_query":
            # Keep vector acquisition just as reproducible and bounded as the
            # raster paths: one explicit layer query, one recorded response.
            # GeoJSON supplies WGS84 coordinates without an ArcGIS SDK/GDAL
            # dependency in the runtime-asset converter.
            params = {
                "where": acquisition["where"],
                "outFields": acquisition.get("out_fields", "*"),
                "orderByFields": acquisition.get("order_by", "OBJECTID ASC"),
                "resultRecordCount": str(acquisition.get("record_limit", 2000)),
                "returnGeometry": "true",
                "outSR": str(acquisition.get("out_sr", 4326)),
                "f": "geojson",
            }
            requests.append({
                "id": source["id"],
                "source_id": source["id"],
                "method": "GET",
                "url": source["service"].rstrip("/") + "/query?" + urllib.parse.urlencode(params),
                "path": f"raw/{source['id']}/{source['id']}.{acquisition.get('extension', 'geojson')}",
                "geojson_contract": {
                    "property": "ICAO_ID", "value": "KSFO",
                    "geometry_types": ["Polygon", "MultiPolygon"],
                    "record_limit": acquisition.get("record_limit", 2000),
                },
            })
        elif kind == "static_file":
            requests.append({
                "id": source["id"],
                "source_id": source["id"],
                "method": "GET",
                "url": source["service"],
                "path": f"raw/{source['id']}/{acquisition['filename']}",
            })
        else:
            raise ValueError(f"{source['id']}: unsupported acquisition kind {kind}")
    return requests


def write_json(path: pathlib.Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def plan(manifest: dict, output: pathlib.Path) -> list[dict]:
    request_list = requests_for(manifest)
    record = {
        "schema": "gx-spike002-acquisition-plan/1",
        "source_manifest_sha256": manifest["_manifest_sha256"],
        "area": manifest["area"],
        "source_records": [
            {key: source[key] for key in source if key != "acquisition"}
            for source in manifest["sources"]
        ],
        "external_inputs": manifest.get("external_inputs", []),
        "requests": request_list,
    }
    write_json(output / "acquisition-plan.json", record)
    return request_list


def safe_target(output: pathlib.Path, relative: str) -> pathlib.Path:
    root = output.resolve()
    target = (root / relative).resolve()
    if root != target and root not in target.parents:
        raise ValueError(f"output path escapes root: {relative}")
    return target


def reusable_receipt(request: dict, output: pathlib.Path) -> dict | None:
    receipt_path = output / "receipts" / f"{request['id']}.json"
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if (receipt.get("schema") != "gx-spike002-acquisition-receipt/1"
            or receipt.get("request_id") != request["id"]
            or receipt.get("source_id") != request["source_id"]
            or receipt.get("requested_url") != request["url"]):
        return None
    if receipt.get("missing"):
        return receipt if receipt.get("http_status") in request.get(
            "optional_missing_statuses", []) else None
    if receipt.get("relative_path") != request["path"]:
        return None
    target = safe_target(output, request["path"])
    try:
        size = target.stat().st_size
    except OSError:
        return None
    if size != receipt.get("bytes"):
        return None
    digest = hashlib.sha256()
    with target.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return receipt if digest.hexdigest() == receipt.get("sha256") else None


def acquire_one(request: dict, output: pathlib.Path, timeout: int, max_bytes: int) -> dict:
    target = safe_target(output, request["path"])
    target.parent.mkdir(parents=True, exist_ok=True)
    body = request.get("body")
    headers = {"User-Agent": "gx-spike002-assets/1 (bounded research acquisition)"}
    if request.get("content_type"):
        headers["Content-Type"] = request["content_type"]
    web_request = urllib.request.Request(
        request["url"], data=body.encode("utf-8") if body is not None else None,
        headers=headers, method=request["method"])
    # ArcGIS export artifacts can be local to one load-balanced backend. Keep
    # its affinity cookie between the JSON export request and artifact GET.
    opener = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    started = dt.datetime.now(dt.timezone.utc).isoformat()
    digest = hashlib.sha256()
    count = 0
    temp_name = None
    try:
        try:
            response = opener.open(web_request, timeout=timeout)
        except urllib.error.HTTPError as error:
            if error.code not in request.get("optional_missing_statuses", []):
                raise
            return {
                "schema": "gx-spike002-acquisition-receipt/1",
                "request_id": request["id"],
                "source_id": request["source_id"],
                "requested_url": request["url"],
                "method": request["method"],
                "retrieved_utc": started,
                "http_status": error.code,
                "missing": True,
                "bytes": 0,
                "sha256": None,
                "relative_path": None,
            }
        resolution_result = None
        if request.get("arcgis_result_href"):
            with response:
                resolution_body = response.read(1024 * 1024 + 1)
                if len(resolution_body) > 1024 * 1024:
                    raise RuntimeError("ArcGIS export response exceeds 1 MiB")
            result = json.loads(resolution_body)
            href = result.get("href")
            if not isinstance(href, str):
                detail = result.get("error", result)
                raise RuntimeError(f"ArcGIS export returned no artifact href: {detail}")
            parsed_href = urllib.parse.urlparse(href)
            if parsed_href.scheme != "https" or parsed_href.hostname not in ALLOWED_HOSTS:
                raise RuntimeError("ArcGIS export returned an unapproved artifact host")
            resolution_result = result
            response = opener.open(
                urllib.request.Request(href, headers=headers, method="GET"), timeout=timeout)
        with response:
            declared = response.headers.get("Content-Length")
            if declared is not None and int(declared) > max_bytes:
                raise RuntimeError(f"response exceeds per-file limit: {declared} > {max_bytes}")
            with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temp:
                temp_name = temp.name
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    count += len(chunk)
                    if count > max_bytes:
                        raise RuntimeError(f"response exceeds per-file limit: > {max_bytes}")
                    digest.update(chunk)
                    temp.write(chunk)
            if "geojson_contract" in request:
                validate_geojson(pathlib.Path(temp_name), request["geojson_contract"])
            os.replace(temp_name, target)
            temp_name = None
            return {
                "schema": "gx-spike002-acquisition-receipt/1",
                "request_id": request["id"],
                "source_id": request["source_id"],
                "requested_url": request["url"],
                "arcgis_result": resolution_result,
                "method": request["method"],
                "retrieved_utc": started,
                "final_url": response.geturl(),
                "http_status": response.status,
                "content_type": response.headers.get("Content-Type"),
                "etag": response.headers.get("ETag"),
                "last_modified": response.headers.get("Last-Modified"),
                "bytes": count,
                "sha256": digest.hexdigest(),
                "relative_path": request["path"],
                "missing": False,
            }
    finally:
        if temp_name is not None:
            pathlib.Path(temp_name).unlink(missing_ok=True)


def validate_geojson(path: pathlib.Path, contract: dict) -> None:
    """Reject signalled/limit-sized, cross-airport or non-polygon FAA data."""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        raise ValueError("invalid GeoJSON response") from error
    features = document.get("features") if document.get("type") == "FeatureCollection" else None
    if not isinstance(features, list):
        raise ValueError("GeoJSON response is not a FeatureCollection")
    if document.get("exceededTransferLimit"):
        raise ValueError("GeoJSON response exceeded the service transfer limit")
    if len(features) >= contract["record_limit"]:
        raise ValueError("GeoJSON response reached the record limit; pagination required")
    object_ids = set()
    for feature in features:
        properties, geometry = feature.get("properties") or {}, feature.get("geometry") or {}
        if properties.get(contract["property"]) != contract["value"]:
            raise ValueError("GeoJSON response contains an unexpected airport")
        object_id = properties.get("OBJECTID")
        if not isinstance(object_id, (int, float)) or object_id in object_ids:
            raise ValueError("GeoJSON response has invalid or duplicate OBJECTID")
        object_ids.add(object_id)
        if geometry.get("type") not in contract["geometry_types"]:
            raise ValueError("GeoJSON response contains unsupported geometry")


def acquire(manifest: dict, output: pathlib.Path, source_ids: set[str] | None) -> None:
    request_list = plan(manifest, output)
    if source_ids:
        known = {request["source_id"] for request in request_list}
        unknown = source_ids - known
        if unknown:
            raise ValueError("unknown source id(s): " + ", ".join(sorted(unknown)))
        request_list = [request for request in request_list if request["source_id"] in source_ids]
    limits = manifest["limits"]
    total = 0
    receipts = []
    for number, request in enumerate(request_list, 1):
        receipt = reusable_receipt(request, output)
        action = "reuse" if receipt is not None else "fetch"
        print(f"[{number}/{len(request_list)}] {action} {request['id']}", flush=True)
        if receipt is None:
            receipt = acquire_one(request, output, limits["request_timeout_seconds"], limits["max_response_bytes"])
        total += receipt["bytes"]
        if total > limits["max_total_bytes"]:
            raise RuntimeError("acquisition exceeded total-byte limit")
        receipt_path = output / "receipts" / f"{request['id']}.json"
        write_json(receipt_path, receipt)
        receipts.append(receipt)
    # A source-limited fetch is useful for refreshing one live provider but is
    # not a complete input set. Never let it overwrite the manifest consumed
    # by the converter.
    manifest_name = "acquisition-manifest.partial.json" if source_ids else "acquisition-manifest.json"
    write_json(output / manifest_name, {
        "schema": "gx-spike002-acquisition-manifest/1",
        "source_manifest_sha256": manifest["_manifest_sha256"],
        "completed_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "total_bytes": total,
        "receipts": receipts,
    })


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("validate", "plan", "acquire"))
    parser.add_argument("--manifest", type=pathlib.Path,
                        default=pathlib.Path("assets/spike-002/sfo-sources.v1.json"))
    parser.add_argument("--output", type=pathlib.Path,
                        default=pathlib.Path("build/spike-002-assets"))
    parser.add_argument("--source", action="append", help="limit acquire to a source id; repeatable")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest = load_manifest(args.manifest)
    request_list = requests_for(manifest)
    if args.command == "validate":
        print(f"valid: {len(manifest['sources'])} sources, {len(request_list)} bounded requests")
    elif args.command == "plan":
        plan(manifest, args.output)
        print(args.output / "acquisition-plan.json")
    else:
        acquire(manifest, args.output, set(args.source) if args.source else None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

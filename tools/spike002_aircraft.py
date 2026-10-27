#!/usr/bin/env python3
"""Deterministically convert a FlightGear AC3D aircraft into VisualAsset inputs.

The converter deliberately emits an intermediate directory rather than a Rust
binary format: ``visual-asset.json`` mirrors gx_world::VisualAsset and points at
little-endian vertex/index blobs plus bottom-up RGBA8 texture blobs.  Source
assets remain outside the repository; only hashes and transformation metadata
are recorded.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shlex
import struct
import subprocess
import sys
import zlib
from dataclasses import dataclass, field
from pathlib import Path

SCHEMA = "gx-visual-asset-intermediate/1"
PROVENANCE_SCHEMA = "gx-spike002-aircraft-provenance/1"
VERTEX_FORMAT = "<8f:position_xyz,normal_xyz,uv"
SOURCE_TO_LOCAL = ((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0))


class ConversionError(ValueError):
    pass


@dataclass
class Material:
    name: str
    rgb: tuple[float, float, float]
    spec: tuple[float, float, float]
    shininess: float
    transparency: float


@dataclass
class Surface:
    flags: int
    material: int
    refs: list[tuple[int, float, float]]


@dataclass
class Object3d:
    name: str = "unnamed"
    loc: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    texture: str | None = None
    vertices: list[tuple[float, float, float]] = field(default_factory=list)
    surfaces: list[Surface] = field(default_factory=list)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _numbers(tokens: list[str], count: int, what: str) -> tuple[float, ...]:
    if len(tokens) != count:
        raise ConversionError(f"{what}: expected {count} values")
    values = tuple(float(x) for x in tokens)
    if not all(math.isfinite(x) for x in values):
        raise ConversionError(f"{what}: non-finite value")
    return values


def parse_ac3d(path: Path) -> tuple[list[Material], list[Object3d], list[str]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0] not in ("AC3Db", "AC3Dc"):
        raise ConversionError("unsupported AC3D header")
    materials: list[Material] = []
    objects: list[Object3d] = []
    warnings: list[str] = []
    obj: Object3d | None = None
    i = 1
    root_declared_kids: int | None = None
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line:
            continue
        tokens = shlex.split(line)
        key = tokens[0]
        if key == "MATERIAL":
            fields: dict[str, list[str]] = {}
            j = 2
            widths = {"rgb": 3, "amb": 3, "emis": 3, "spec": 3, "shi": 1, "trans": 1}
            while j < len(tokens):
                width = widths.get(tokens[j])
                if width is None:
                    raise ConversionError(f"unknown MATERIAL field {tokens[j]}")
                fields[tokens[j]] = tokens[j + 1:j + 1 + width]
                j += 1 + width
            materials.append(Material(tokens[1], _numbers(fields["rgb"], 3, "rgb"),
                                      _numbers(fields["spec"], 3, "spec"),
                                      _numbers(fields["shi"], 1, "shi")[0],
                                      _numbers(fields["trans"], 1, "trans")[0]))
        elif key == "OBJECT":
            obj = Object3d()
            objects.append(obj)
        elif obj is None:
            raise ConversionError(f"property before OBJECT: {key}")
        elif key == "name":
            obj.name = tokens[1]
        elif key == "loc":
            obj.loc = _numbers(tokens[1:], 3, f"{obj.name} loc")
        elif key == "rot":
            obj.rot = _numbers(tokens[1:], 9, f"{obj.name} rot")
        elif key == "texture":
            obj.texture = tokens[1]
        elif key == "numvert":
            count = int(tokens[1])
            if count < 0 or i + count > len(lines):
                raise ConversionError(f"{obj.name}: invalid numvert")
            obj.vertices = [_numbers(lines[i + n].split(), 3, f"{obj.name} vertex") for n in range(count)]
            i += count
        elif key == "numsurf":
            count = int(tokens[1])
            for _ in range(count):
                if i + 3 > len(lines) or not lines[i].startswith("SURF "):
                    raise ConversionError(f"{obj.name}: malformed surface")
                flags = int(lines[i].split()[1], 0); i += 1
                if not lines[i].startswith("mat "):
                    raise ConversionError(f"{obj.name}: surface missing material")
                material = int(lines[i].split()[1]); i += 1
                if not lines[i].startswith("refs "):
                    raise ConversionError(f"{obj.name}: surface missing refs")
                refs_count = int(lines[i].split()[1]); i += 1
                refs: list[tuple[int, float, float]] = []
                for _ in range(refs_count):
                    parts = lines[i].split(); i += 1
                    if len(parts) != 3:
                        raise ConversionError(f"{obj.name}: malformed ref")
                    refs.append((int(parts[0]), float(parts[1]), float(parts[2])))
                obj.surfaces.append(Surface(flags, material, refs))
        elif key == "kids" and len(objects) == 1:
            root_declared_kids = int(tokens[1])
        elif key in {"data", "texrep", "texoff", "crease", "url", "kids"}:
            # data payload is one line in the source model; it carries an object label only.
            if key == "data":
                i += 1
        else:
            raise ConversionError(f"unsupported AC3D property {key}")
    geometry = [o for o in objects if o.vertices or o.surfaces]
    if root_declared_kids is not None and root_declared_kids != len(geometry):
        warnings.append(f"root declares {root_declared_kids} children; file contains {len(geometry)} geometry objects")
    if not materials or not geometry:
        raise ConversionError("AC3D contains no material or geometry")
    return materials, geometry, warnings


def _transform(obj: Object3d, p: tuple[float, float, float]) -> tuple[float, float, float]:
    r = obj.rot
    q = (r[0]*p[0] + r[1]*p[1] + r[2]*p[2] + obj.loc[0],
         r[3]*p[0] + r[4]*p[1] + r[5]*p[2] + obj.loc[1],
         r[6]*p[0] + r[7]*p[1] + r[8]*p[2] + obj.loc[2])
    return (-q[0], q[2], q[1])


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def build_mesh(obj: Object3d, material_index: int) -> tuple[list[tuple[float, ...]], list[int]]:
    positions = [_transform(obj, p) for p in obj.vertices]
    vertices: list[list[float]] = []
    indices: list[int] = []
    smooth_lookup: dict[tuple[int, float, float], int] = {}
    normal_sums: list[list[float]] = []
    for surface_number, surface in enumerate(obj.surfaces):
        if surface.material != material_index or (surface.flags & 0x0f) != 0:
            continue
        if len(surface.refs) < 3:
            continue
        for corner in range(1, len(surface.refs) - 1):
            tri = (surface.refs[0], surface.refs[corner], surface.refs[corner + 1])
            try:
                ps = [positions[ref[0]] for ref in tri]
            except IndexError as error:
                raise ConversionError(f"{obj.name}: vertex reference out of range") from error
            face = _cross(tuple(ps[1][n]-ps[0][n] for n in range(3)),
                          tuple(ps[2][n]-ps[0][n] for n in range(3)))
            length = math.sqrt(sum(x*x for x in face))
            if length <= 1e-12:
                continue
            smooth = bool(surface.flags & 0x10)
            for ref, p in zip(tri, ps):
                uv = (ref[1], ref[2])
                if not all(math.isfinite(v) and -1e-6 <= v <= 1.000001 for v in uv):
                    raise ConversionError(f"{obj.name}: UV outside VisualAsset [0,1]")
                uv = (min(1.0, max(0.0, uv[0])), min(1.0, max(0.0, uv[1])))
                key = (ref[0], uv[0], uv[1]) if smooth else (surface_number, len(indices), 0.0)
                vi = smooth_lookup.get(key)
                if vi is None:
                    vi = len(vertices)
                    smooth_lookup[key] = vi
                    vertices.append([*p, 0.0, 0.0, 0.0, *uv])
                    normal_sums.append([0.0, 0.0, 0.0])
                for n in range(3):
                    normal_sums[vi][n] += face[n]
                indices.append(vi)
    for vertex, normal in zip(vertices, normal_sums):
        length = math.sqrt(sum(x*x for x in normal))
        if length <= 1e-12:
            raise ConversionError(f"{obj.name}: zero output normal")
        vertex[3:6] = [x/length for x in normal]
    return [tuple(v) for v in vertices], indices


def decode_png(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ConversionError("texture is not PNG")
    chunks: list[bytes] = []
    width = height = color = depth = interlace = None
    pos = 8
    while pos < len(data):
        if pos + 12 > len(data):
            raise ConversionError("truncated PNG chunk")
        size = struct.unpack(">I", data[pos:pos+4])[0]
        kind = data[pos+4:pos+8]
        payload = data[pos+8:pos+8+size]
        crc = struct.unpack(">I", data[pos+8+size:pos+12+size])[0]
        if zlib.crc32(kind + payload) & 0xffffffff != crc:
            raise ConversionError("PNG CRC mismatch")
        pos += 12 + size
        if kind == b"IHDR":
            width, height, depth, color, compression, filtering, interlace = struct.unpack(">IIBBBBB", payload)
            if depth != 8 or color not in (2, 6) or compression or filtering or interlace:
                raise ConversionError("only non-interlaced 8-bit RGB/RGBA PNG is supported")
        elif kind == b"IDAT":
            chunks.append(payload)
        elif kind == b"IEND":
            break
    channels = 3 if color == 2 else 4
    stride = width * channels
    raw = zlib.decompress(b"".join(chunks))
    if len(raw) != height * (stride + 1):
        raise ConversionError("PNG inflated size mismatch")
    rows: list[bytearray] = []
    previous = bytearray(stride)
    offset = 0
    for _ in range(height):
        filter_type = raw[offset]; offset += 1
        scan = bytearray(raw[offset:offset+stride]); offset += stride
        for x in range(stride):
            a = scan[x-channels] if x >= channels else 0
            b = previous[x]
            c = previous[x-channels] if x >= channels else 0
            if filter_type == 1: scan[x] = (scan[x] + a) & 255
            elif filter_type == 2: scan[x] = (scan[x] + b) & 255
            elif filter_type == 3: scan[x] = (scan[x] + ((a+b)//2)) & 255
            elif filter_type == 4:
                p = a+b-c; pa=abs(p-a); pb=abs(p-b); pc=abs(p-c)
                scan[x] = (scan[x] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
            elif filter_type != 0: raise ConversionError("unsupported PNG filter")
        rows.append(scan); previous = scan
    rgba = bytearray()
    for row in reversed(rows):  # GLES row zero is the bottom row; preserve AC3D UV convention.
        if channels == 4: rgba.extend(row)
        else:
            for x in range(0, len(row), 3): rgba.extend(row[x:x+3] + b"\xff")
    return width, height, bytes(rgba)


def encode_astc(astcenc: Path, source_png: Path, output: Path, width: int, height: int) -> tuple[Path, dict]:
    container = output / "texture-0001.container.astc"
    command = [str(astcenc), "-cs", str(source_png), str(container), "6x6", "-medium", "-yflip"]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode:
        raise ConversionError(f"astcenc failed ({completed.returncode}): {completed.stderr.strip()}")
    data = container.read_bytes()
    if len(data) < 16 or data[:4] != b"\x13\xab\xa1\x5c":
        raise ConversionError("astcenc output has invalid ASTC header")
    block = tuple(data[4:7])
    dims = tuple(int.from_bytes(data[n:n+3], "little") for n in (7, 10, 13))
    expected = ((width + 5)//6) * ((height + 5)//6) * 16
    if block != (6, 6, 1) or dims != (width, height, 1) or len(data) - 16 != expected:
        raise ConversionError("astcenc output dimensions or payload length mismatch")
    payload = output / "texture-0001.astc6x6"
    payload.write_bytes(data[16:])
    container.unlink()
    version = subprocess.run([str(astcenc), "-version"], capture_output=True, text=True, check=False)
    archives = sorted(astcenc.parent.parent.parent.glob("astcenc-*-windows-x64.zip"))
    archive = ({"basename": archives[0].name, "sha256": sha256(archives[0])}
               if len(archives) == 1 else None)
    return payload, {"binary_sha256": sha256(astcenc), "distribution_archive": archive,
                     "version": (version.stdout or version.stderr).strip(),
                     "arguments": ["-cs", "<source-png>", "<output-astc>", "6x6", "-medium", "-yflip"],
                     "colour_space": "sRGB", "block": "6x6", "quality": "medium"}


def encode_gxvisual(manifest: dict, root: Path) -> bytes:
    """Encode the shared gx-world GXVISUAL 1.0 little-endian contract."""
    payload = bytearray(struct.pack("<8sHHQQIII", b"GXVISUAL", manifest["format_major"],
                                    manifest["format_minor"], manifest["frame_id"],
                                    manifest["frame_generation"], len(manifest["textures"]),
                                    len(manifest["materials"]), len(manifest["meshes"])))
    encoding_ids = {"Astc6x6Srgb": 1, "Rgba8Srgb": 2}
    for texture in sorted(manifest["textures"], key=lambda item: item["id"]):
        raw = (root / texture["payload"]["file"]).read_bytes()
        payload += struct.pack("<IIIB3xI", texture["id"], texture["width"], texture["height"],
                               encoding_ids[texture["encoding"]], 1)
        payload += struct.pack("<Q", len(raw)) + raw
    for material in sorted(manifest["materials"], key=lambda item: item["id"]):
        payload += struct.pack("<II6f", material["id"], material["base_color_texture"],
                               *material["base_color_factor"], material["roughness"], material["metallic"])
    for mesh in sorted(manifest["meshes"], key=lambda item: item["id"]):
        vertices = (root / mesh["vertices"]["file"]).read_bytes()
        indices = (root / mesh["indices"]["file"]).read_bytes()
        payload += struct.pack("<B3xIII", 2, mesh["material"], mesh["vertex_count"], mesh["index_count"])
        payload += vertices + indices
    return bytes(payload)


def convert(ac_path: Path, texture_path: Path, output: Path, frame_id: int, frame_generation: int,
            source_revision: str, source_package: str, astcenc: Path | None = None) -> dict:
    materials, objects, warnings = parse_ac3d(ac_path)
    width, height, rgba = decode_png(texture_path)
    output.mkdir(parents=True, exist_ok=True)
    if astcenc is None:
        texture_file = output / "texture-0001.rgba8"
        texture_file.write_bytes(rgba)
        texture_encoding = "Rgba8Srgb"
        encoder = None
    else:
        texture_file, encoder = encode_astc(astcenc, texture_path, output, width, height)
        texture_encoding = "Astc6x6Srgb"
    meshes = []
    mesh_id = 0
    used_materials: set[int] = set()
    for obj in objects:
        if obj.texture is not None and Path(obj.texture).name != texture_path.name:
            raise ConversionError(f"{obj.name}: expected texture {obj.texture}, got {texture_path.name}")
        for material_index in sorted({s.material for s in obj.surfaces}):
            if not 0 <= material_index < len(materials):
                raise ConversionError(f"{obj.name}: material reference out of range")
            vertices, indices = build_mesh(obj, material_index)
            if not indices:
                continue
            mesh_id += 1; used_materials.add(material_index)
            vb = output / f"mesh-{mesh_id:04d}.vertices.bin"
            ib = output / f"mesh-{mesh_id:04d}.indices.bin"
            vb.write_bytes(b"".join(struct.pack("<8f", *v) for v in vertices))
            ib.write_bytes(struct.pack(f"<{len(indices)}I", *indices))
            bounds = {"min": [min(v[n] for v in vertices) for n in range(3)],
                      "max": [max(v[n] for v in vertices) for n in range(3)]}
            meshes.append({"id": mesh_id, "name": obj.name, "kind": "Aircraft",
                           "material": material_index + 1, "vertex_format": VERTEX_FORMAT,
                           "vertex_count": len(vertices), "index_type": "u32-le",
                           "index_count": len(indices), "bounds_metres": bounds,
                           "vertices": {"file": vb.name, "sha256": sha256(vb)},
                           "indices": {"file": ib.name, "sha256": sha256(ib)}})
    out_materials = []
    for index in sorted(used_materials):
        mat = materials[index]
        if (not all(0.0 <= value <= 1.0 for value in (*mat.rgb, mat.transparency))
                or mat.shininess < 0.0):
            raise ConversionError(f"material {mat.name}: values outside VisualAsset range")
        out_materials.append({"id": index + 1, "source_name": mat.name,
                              "base_color_texture": 1,
                              "base_color_factor": [*mat.rgb, 1.0-mat.transparency],
                              "roughness": round(math.sqrt(2.0/(max(0.0, mat.shininess)+2.0)), 9),
                              # AC3D specular intensity is not a metalness signal; this livery is paint.
                              "metallic": 0.0})
    manifest = {"schema": SCHEMA, "format_major": 1, "format_minor": 0,
                "frame_id": frame_id, "frame_generation": frame_generation,
                "coordinate_system": {"name": "aircraft-local", "units": "metres",
                                      "axes": {"x": "forward", "y": "right", "z": "up"},
                                      "source_to_local": SOURCE_TO_LOCAL},
                "textures": [{"id": 1, "width": width, "height": height,
                              "encoding": texture_encoding, "mip_levels": 1,
                              "row_order": "block-compressed" if astcenc else "bottom-to-top",
                              "payload": {"file": texture_file.name, "sha256": sha256(texture_file)}}],
                "materials": out_materials, "meshes": meshes}
    manifest_path = output / "visual-asset.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pack_path = output / "aircraft.gxvisual"
    pack_path.write_bytes(encode_gxvisual(manifest, output))
    provenance = {"schema": PROVENANCE_SCHEMA,
                  "producer": "FlightGear project / Debian flightgear-data-ai",
                  "dataset": "FlightGear AI Bombardier Global 5000 (CCX)",
                  "source_package": source_package, "source_revision": source_revision,
                  "license": "GPL-2.0-or-later; redistributed derivatives must preserve the GPL and corresponding source",
                  "inputs": [{"role": "mesh", "basename": ac_path.name, "sha256": sha256(ac_path)},
                             {"role": "base_color", "basename": texture_path.name, "sha256": sha256(texture_path)}],
                  "transform": {"tool": "tools/spike002_aircraft.py", "axis_map": "(-source_x, source_z, source_y)",
                                "scale": 1.0, "triangulation": "deterministic polygon fan",
                                "normals": "area-weighted; AC3D shaded flag controls sharing",
                                "uv": "preserved; source image vertically flipped for GLES row convention",
                                "texture": "ASTC 6x6 sRGB" if astcenc else "lossless PNG decode to sRGB RGBA8",
                                "mip_generation": "none", "astc_encoder": encoder},
                  "warnings": warnings,
                  "outputs": [{"file": p.name, "sha256": sha256(p), "bytes": p.stat().st_size}
                              for p in sorted(output.glob("*.bin"))] +
                             [{"file": texture_file.name, "sha256": sha256(texture_file), "bytes": texture_file.stat().st_size},
                              {"file": manifest_path.name, "sha256": sha256(manifest_path), "bytes": manifest_path.stat().st_size}]}
    provenance["outputs"].append({"file": pack_path.name, "sha256": sha256(pack_path), "bytes": pack_path.stat().st_size})
    prov_path = output / "provenance.json"
    prov_path.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ac", type=Path, required=True)
    parser.add_argument("--texture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--frame-generation", type=lambda x: int(x, 0), default=1)
    parser.add_argument("--source-revision", required=True)
    parser.add_argument("--source-package", default="flightgear-data-ai")
    parser.add_argument("--astcenc", type=Path, help="official astcenc executable; emits ASTC 6x6 sRGB")
    args = parser.parse_args(argv)
    if args.frame_id <= 0 or args.frame_generation <= 0:
        parser.error("frame identity must be non-zero")
    try:
        result = convert(args.ac, args.texture, args.output, args.frame_id,
                         args.frame_generation, args.source_revision, args.source_package, args.astcenc)
    except (OSError, ConversionError, KeyError, ValueError, zlib.error) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"output": str(args.output), "meshes": len(result["meshes"]),
                      "vertices": sum(x["vertex_count"] for x in result["meshes"]),
                      "triangles": sum(x["index_count"] for x in result["meshes"]) // 3}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

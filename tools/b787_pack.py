#!/usr/bin/env python3
"""Splice the FlightGear 787-8 (BBJ livery, GXVISUAL 1.2 joints) into an SFO pack.

Implements the public alpha's FlightGear 787-8 conversion path.

Inputs:
  * a GXVISUAL 1.0/1.1 SFO pack from tools/spike002_convert.py (terrain,
    decals and markings are copied byte for byte; its aircraft texture,
    material and mesh are dropped);
  * the FGAddon `787-8` aircraft (GPL-2): Models/787-8.ac, Models/787-8.xml
    (animation hinges), BOE.png / Liveries/BOE2.png, wings.png,
    landing-gears.png.

Output: GXVISUAL 1.2 with the 787 as four aircraft batches (fuselage
livery, wings, landing gear, a colour palette for the untextured parts) and
one joint per FlightGear animation chain step that the FDM can drive (gear,
doors, flaps, slats, spoilers, ailerons, elevators, rudder, nose steering).

Frames. AC3D (x, y up, z) -> FlightGear model (x aft, y right, z up) =
(x, -z, y). The stock JSBSim 787-8 structural origin is the model origin
(`787-8.xml`: EYEPOINT/VRP = AC3D 0,0,0), so body FRD about the JSBSim CG is
(-(x - cg_x), y, -(z - cg_z)). That map is a proper rotation (det +1), so
FlightGear's right-handed hinge angles carry over unchanged; axes map as
vectors.

Animation semantics follow SimGear: an animation wraps every object (or
group) of that name; animations declared later sit *inside* earlier ones,
and a group's animations sit outside its children's. So a vertex's chain is
the ancestor groups' animations (root first), then its own, each in
declaration order, outermost first. Only animations whose property maps to
a GXVISUAL driver are kept; the rest (wing flex, trims, wheel spin and
compression, wipers, reversers, fan spin) are static here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

sys.path.insert(0, str(pathlib.Path(__file__).parent))

TOOL_VERSION = 1
IN_TO_M = 0.0254
# JSBSim 787-8 <location name="CG">, inches (structural = FlightGear model).
CG_IN = (-16.0, 0.0, 22.5972)
# JSBSim contacts (inches) used to check the model placement.
MAIN_CONTACT_IN = (77.1058, 193.59978, -153.63242)
NOSE_CONTACT_IN = (-809.50058, 0.0, -143.36872)

# Objects FlightGear hides by default (select animations: wing/ground
# lights, night windows, surge flashes, baggage-service cargo door) or that
# are not part of the airframe.
EXCLUDED = {
    "winglts", "l.surge", "r.surge", "l.surge.smoke", "r.surge.smoke", "ground-lights",
    "windows-light", "fuel_pipe", "fuel_pipe_disconnected", "cargo_door", "cargo_inside",
}

DRIVERS = {"aileron": 1, "elevator": 2, "rudder": 3, "flap": 4, "speedbrake": 5, "gear": 6}
# FlightGear property -> (driver, input range when only a factor is given).
PROPERTY_DRIVERS = [
    ("gear/gear[", "gear", (0.0, 1.0)),
    ("surface-positions/flap-pos-norm", "flap", (0.0, 1.0)),
    ("surface-positions/speedbrake-pos-norm", "speedbrake", (0.0, 1.0)),
    ("fdm/jsbsim/fcs/aileron-fbw-output", "aileron", (-1.0, 1.0)),
    ("fdm/jsbsim/fcs/elevator-fbw-output", "elevator", (-1.0, 1.0)),
    ("fdm/jsbsim/fcs/rudder-fbw-output", "rudder", (-1.0, 1.0)),
]
# Gear compression/rollspeed share the gear[ prefix but are not positions.
GEAR_POSITION_SUFFIX = "/position-norm"
# The model's FBW outputs carry the JSBSim surface signs (+aileron right
# roll, +elevator trailing edge down, +rudder trailing edge left): no flip.
# tests/test_b787_pack.py checks the trailing-edge directions.
#
# Strut compression is not an FDM output here. FlightGear tilts the main
# bogies 20 deg with the gear down and levels them with live compression; a
# fixed pose would leave the rear wheels 0.25 m below the runway. So the
# strut translations and scissor links are held at the static on-ground
# compression (unit -> value), which puts the wheels on the JSBSim contacts,
# and both bogie rotations (tilt and its compression counter-rotation) are
# dropped.
STATIC_COMPRESSION = {0: 0.166, 1: 0.2, 2: 0.2}
BOGIE_OBJECT = "WheelFront"

MAX_JOINTS = 64
MAX_JOINT_POINTS = 8

# Livery (W6): dark premium BBJ, fictional registration, no manufacturer marks.
NAVY = (20, 31, 56)
NAVY_LIGHT = (32, 46, 78)
GOLD = (201, 169, 110)
REGISTRATION = "N787GX"


class PackError(ValueError):
    pass


# ------------------------------------------------------------------ AC3D ---

class AcObject:
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.loc = (0.0, 0.0, 0.0)
        self.rot = None
        self.texture = None
        self.vertices = []      # AC local
        self.surfaces = []      # (flags, material, [(index, u, v)])
        self.kids = []

    def ancestors(self):
        out, o = [], self.parent
        while o is not None:
            out.append(o)
            o = o.parent
        return out[::-1]


def parse_ac(path):
    """AC3D -> (materials [(name, rgb)], root AcObject) with the hierarchy."""
    lines = pathlib.Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    if not lines or not lines[0].startswith("AC3D"):
        raise PackError("not an AC3D file")
    materials = []
    i = 1
    while i < len(lines) and lines[i].startswith("MATERIAL"):
        t = lines[i].split()
        name = lines[i].split('"')[1] if '"' in lines[i] else t[1]
        k = t.index("rgb")
        materials.append((name, tuple(float(x) for x in t[k + 1:k + 4])))
        i += 1

    def obj(i, parent):
        if not lines[i].startswith("OBJECT"):
            raise PackError(f"line {i + 1}: expected OBJECT")
        o = AcObject(None, parent)
        i += 1
        while i < len(lines):
            line = lines[i]
            t = line.split()
            i += 1
            if not t:
                continue
            k = t[0]
            if k == "name":
                o.name = line.split('"')[1] if '"' in line else t[1]
            elif k == "loc":
                o.loc = tuple(float(x) for x in t[1:4])
            elif k == "rot":
                o.rot = tuple(float(x) for x in t[1:10])
            elif k == "texture":
                o.texture = line.split('"')[1] if '"' in line else t[1]
            elif k == "numvert":
                n = int(t[1])
                o.vertices = [tuple(float(x) for x in lines[i + j].split()[:3]) for j in range(n)]
                i += n
            elif k == "numsurf":
                for _ in range(int(t[1])):
                    flags = int(lines[i].split()[1], 0)
                    mat = int(lines[i + 1].split()[1])
                    n = int(lines[i + 2].split()[1])
                    refs = []
                    for j in range(n):
                        p = lines[i + 3 + j].split()
                        refs.append((int(p[0]), float(p[1]), float(p[2])))
                    o.surfaces.append((flags, mat, refs))
                    i += 3 + n
            elif k == "data":
                i += 1
            elif k == "kids":
                for _ in range(int(t[1])):
                    kid, i = obj(i, o)
                    o.kids.append(kid)
                return o, i
        raise PackError("truncated AC3D object")

    first = next(n for n in range(i, len(lines)) if lines[n].startswith("OBJECT"))
    root, _ = obj(first, None)
    return materials, root


def walk(o):
    yield o
    for k in o.kids:
        yield from walk(k)


def world_matrix(o):
    """AC local -> AC model 4x4 (OSG ac3d reader: child = parent * (rot, loc))."""
    m = np.eye(4)
    for node in o.ancestors() + [o]:
        local = np.eye(4)
        if node.rot is not None:
            local[:3, :3] = np.array(node.rot).reshape(3, 3)
        local[:3, 3] = node.loc
        m = m @ local
    return m


def ac_to_model(p):
    """AC3D (x, y up, z) -> FlightGear model (x aft, y right, z up)."""
    return np.stack([p[..., 0], -p[..., 2], p[..., 1]], axis=-1)


CG_M = np.array(CG_IN) * IN_TO_M


def model_to_frd(p):
    """FlightGear model point -> body FRD metres about the JSBSim CG."""
    p = np.asarray(p, dtype=np.float64)
    return np.stack([-(p[..., 0] - CG_M[0]), p[..., 1] - CG_M[1], -(p[..., 2] - CG_M[2])], axis=-1)


def model_vec_to_frd(v):
    v = np.asarray(v, dtype=np.float64)
    return np.stack([-v[..., 0], v[..., 1], -v[..., 2]], axis=-1)


# ------------------------------------------------------- FG animations ---

def _gear_unit(prop):
    return int(prop.split("gear[", 1)[1].split("]", 1)[0])


def _driver(prop):
    for prefix, name, rng in PROPERTY_DRIVERS:
        if prop.startswith(prefix):
            if name == "gear" and not prop.endswith(GEAR_POSITION_SUFFIX):
                if prop.endswith("/compression-norm"):
                    return "compression", (0.0, 1.0)
                return None, None
            return name, rng
    return None, None


def parse_animations(xml_path):
    """FlightGear model XML -> ordered list of drivable rigid animations."""
    root = ET.parse(xml_path).getroot()
    out = []
    for a in root.findall("animation"):
        kind = a.findtext("type")
        if kind not in ("rotate", "translate"):
            continue
        prop = (a.findtext("property") or "").strip()
        driver, rng = _driver(prop)
        if driver is None:
            continue
        objects = [o.text.strip() for o in a.findall("object-name")]
        bogie = kind == "rotate" and any(BOGIE_OBJECT in o for o in objects)
        if bogie and driver in ("gear", "compression"):
            continue
        c = a.find("center")
        center = [float(c.findtext(k) or 0.0) for k in ("x-m", "y-m", "z-m")] if c is not None else [0.0, 0.0, 0.0]
        ax = a.find("axis")
        if ax is None:
            raise PackError(f"{kind} {prop}: no axis")
        if ax.find("x1-m") is not None:
            p1 = [float(ax.findtext(k) or 0.0) for k in ("x1-m", "y1-m", "z1-m")]
            p2 = [float(ax.findtext(k) or 0.0) for k in ("x2-m", "y2-m", "z2-m")]
            axis = [b - a_ for a_, b in zip(p1, p2)]
            center = p1
        else:
            axis = [float(ax.findtext(k) or 0.0) for k in ("x", "y", "z")]
        n = math.sqrt(sum(v * v for v in axis))
        if n <= 0:
            raise PackError(f"{kind} {prop}: zero axis")
        axis = [v / n for v in axis]
        interp = a.find("interpolation")
        if interp is not None:
            table = [(float(e.findtext("ind")), float(e.findtext("dep"))) for e in interp.findall("entry")]
        else:
            factor = float(a.findtext("factor") or 1.0)
            offset = float(a.findtext("offset-deg") or a.findtext("offset-m") or 0.0)
            lo, hi = rng
            table = [(lo, lo * factor + offset), (hi, hi * factor + offset)]
            vmin, vmax = a.findtext("min-deg") or a.findtext("min-m"), a.findtext("max-deg") or a.findtext("max-m")
            if vmin is not None or vmax is not None:
                lo_v = float(vmin) if vmin is not None else -1e9
                hi_v = float(vmax) if vmax is not None else 1e9
                table = [(x, min(max(y, lo_v), hi_v)) for x, y in table]
        table = sorted(table)
        # Merge duplicate inputs (keep the last), then drop collinear points.
        dedup = {}
        for x, y in table:
            dedup[x] = y
        table = sorted(dedup.items())
        if driver == "compression":
            # Held at the static on-ground compression: a constant joint.
            from b787_joints import evaluate
            value = evaluate(table, STATIC_COMPRESSION[_gear_unit(prop)])
            driver, table = "gear", [(0.0, value), (1.0, value)]
        if len(table) < 2:
            table = [(table[0][0], table[0][1]), (table[0][0] + 1.0, table[0][1])]
        if len(table) > MAX_JOINT_POINTS:
            raise PackError(f"{prop}: {len(table)} table points > {MAX_JOINT_POINTS}")
        out.append({"index": len(out), "kind": kind, "driver": driver, "property": prop, "objects": objects,
                    "center": center, "axis": axis, "table": table})
    return out


# ------------------------------------------------------------ geometry ---

def build(ac_path, xml_path):
    """Triangles grouped by texture, with joint ids, plus the joint table."""
    materials, root = parse_ac(ac_path)
    anims = parse_animations(xml_path)
    by_name = {}
    for a in anims:
        for name in a["objects"]:
            by_name.setdefault(name, []).append(a)

    joints = []         # dicts: parent, driver, kind, center, axis, table
    joint_ids = {}      # (parent id, anim index) -> id

    def joint_for(chain):
        parent = 0
        for a in chain:
            key = (parent, a["index"])
            if key not in joint_ids:
                center = model_to_frd(a["center"]).tolist()
                axis = model_vec_to_frd(a["axis"]).tolist()
                joints.append({"parent": parent, "driver": a["driver"], "kind": a["kind"], "center": center,
                               "axis": axis, "table": a["table"], "property": a["property"],
                               "objects": a["objects"]})
                joint_ids[key] = len(joints)
            parent = joint_ids[key]
        return parent

    groups = {}   # texture key -> {"pos": [], "nrm": [], "uv": [], "joint": [], "obj": [], "rgb": []}
    stats = {"objects": 0, "excluded": [], "triangles": 0, "lines_skipped": 0, "uv_clamped": 0}
    for o in walk(root):
        if o.name in EXCLUDED or any(a.name in EXCLUDED for a in o.ancestors()):
            if o.name in EXCLUDED and o.vertices:
                stats["excluded"].append(o.name)
            continue
        if not o.vertices or not o.surfaces:
            continue
        chain = [a for node in o.ancestors() + [o] for a in by_name.get(node.name, [])]
        joint = joint_for(chain)
        m = world_matrix(o)
        local = np.array(o.vertices, dtype=np.float64)
        ac = (np.c_[local, np.ones(len(local))] @ m.T)[:, :3]
        frd = model_to_frd(ac_to_model(ac))
        tex = pathlib.Path(o.texture).name if o.texture else None
        stats["objects"] += 1
        # Smooth surfaces share area-weighted normals per vertex index.
        tris = []
        for flags, mat, refs in o.surfaces:
            if flags & 0x0F:
                stats["lines_skipped"] += 1
                continue
            for k in range(1, len(refs) - 1):
                tris.append((flags, mat, (refs[0], refs[k], refs[k + 1])))
        vnorm = np.zeros_like(frd)
        face_n = []
        for flags, mat, tri in tris:
            p = frd[[r[0] for r in tri]]
            n = np.cross(p[1] - p[0], p[2] - p[0])
            face_n.append(n)
            if flags & 0x10:
                for r in tri:
                    vnorm[r[0]] += n
        for (flags, mat, tri), n in zip(tris, face_n):
            area = np.linalg.norm(n)
            if area <= 1e-12:
                continue
            key = tex if tex else "__palette__"
            g = groups.setdefault(key, {"pos": [], "nrm": [], "uv": [], "joint": [], "obj": [], "rgb": []})
            for r in tri:
                vn = vnorm[r[0]] if flags & 0x10 else n
                ln = np.linalg.norm(vn)
                vn = vn / ln if ln > 1e-12 else n / area
                u, v = r[1], r[2]
                if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
                    stats["uv_clamped"] += 1
                g["pos"].append(frd[r[0]])
                g["nrm"].append(vn)
                g["uv"].append((min(max(u, 0.0), 1.0), min(max(v, 0.0), 1.0)))
                g["joint"].append(joint)
                g["obj"].append(o.name)
                g["rgb"].append(materials[mat][1] if 0 <= mat < len(materials) else (0.7, 0.7, 0.7))
            stats["triangles"] += 1
    if len(joints) > MAX_JOINTS:
        raise PackError(f"{len(joints)} joints > {MAX_JOINTS}")
    for key, g in groups.items():
        for k in ("pos", "nrm", "uv", "rgb"):
            g[k] = np.array(g[k], dtype=np.float64)
        g["joint"] = np.array(g["joint"], dtype=np.uint32)
    stats["joints"] = len(joints)
    stats["textures"] = {k: len(g["joint"]) // 3 for k, g in groups.items()}
    return groups, joints, stats


def palette_texture(group, size=16):
    """Untextured parts: their AC material colours become texels of one
    small palette; each triangle's UVs point at its colour's texel centre."""
    colours = sorted({tuple(round(c, 4) for c in rgb) for rgb in group["rgb"]})
    if len(colours) > size * size:
        raise PackError(f"{len(colours)} palette colours > {size * size}")
    img = Image.new("RGBA", (size, size), (255, 255, 255, 255))
    index = {}
    for k, c in enumerate(colours):
        x, y = k % size, k // size
        img.putpixel((x, y), tuple(int(round(v * 255)) for v in c) + (255,))
        # AC/GL UV: v = 0 is the image bottom (the image is flipped on upload).
        index[c] = ((x + 0.5) / size, 1.0 - (y + 0.5) / size)
    group["uv"] = np.array([index[tuple(round(c, 4) for c in rgb)] for rgb in group["rgb"]])
    return img, len(colours)


# -------------------------------------------------------------- livery ---

FUSELAGE_PARTS = {"fuselage", "fuselage.002", "nwlightedfuse", "bodyfairing", "APU_Inlet", "lhfnosedoor",
                  "rhfnosedoor", "lhnosedoor", "rhnosedoor", "lhgearibdoor", "rhgearibdoor", "Mesh.328",
                  "Mesh.395", "antennae", "pilot_tubes", "lhkflap", "rhkflap"}
FIN_PARTS = {"fin", "rudder"}
NACELLE_PARTS = {"no1enge", "no2enge", "no1engl", "no2engl", "lhreverser", "rhreverser", "lhvortexdev",
                 "rhvortexdev", "lreverse", "rreverse"}


def rasterize_uv(group, size):
    """Per texel of the BOE atlas: owning part class and FRD position (static
    pose), by rasterizing every triangle in UV space."""
    # 0 none, 1 fuselage, 2 fin, 3 nacelle, 4 other, 5 fan face (kept as is).
    # Fan faces share UV islands with the inlet, so they win; other parts
    # never overwrite a painted class.
    cls = np.zeros((size, size), dtype=np.uint8)
    priority = np.array([0, 2, 2, 2, 1, 3], dtype=np.uint8)
    pos = np.zeros((size, size, 3), dtype=np.float32)
    uv = group["uv"].reshape(-1, 3, 2)
    p3 = group["pos"].reshape(-1, 3, 3)
    names = group["obj"][::3]
    for t in range(len(uv)):
        name = names[t]
        c = (1 if name in FUSELAGE_PARTS else 2 if name in FIN_PARTS else 3 if name in NACELLE_PARTS
             else 5 if name.startswith("eng") and "fan" in name else 4)
        px = uv[t, :, 0] * size
        py = (1.0 - uv[t, :, 1]) * size
        x0, x1 = int(max(math.floor(px.min()) - 1, 0)), int(min(math.ceil(px.max()) + 1, size))
        y0, y1 = int(max(math.floor(py.min()) - 1, 0)), int(min(math.ceil(py.max()) + 1, size))
        if x1 <= x0 or y1 <= y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
        (ax, ay), (bx, by), (cx, cy) = zip(px, py)
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-9:
            continue
        w0 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / den
        w1 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / den
        w2 = 1.0 - w0 - w1
        # Small dilation so seams between UV islands are painted too.
        inside = (w0 >= -0.02) & (w1 >= -0.02) & (w2 >= -0.02)
        if not inside.any():
            continue
        sub_cls = cls[y0:y1, x0:x1]
        sub_pos = pos[y0:y1, x0:x1]
        write = inside & (priority[c] >= priority[sub_cls])
        P = (w0[..., None] * p3[t, 0] + w1[..., None] * p3[t, 1] + w2[..., None] * p3[t, 2])
        sub_cls[write] = c
        sub_pos[write] = P[write]
    return cls, pos


def text_mask(text, width_px, height_px):
    """White-on-black text bitmap, tightly cropped then fitted."""
    try:
        font = ImageFont.truetype("arialbd.ttf", 200)
    except OSError:
        font = ImageFont.load_default()
    probe = Image.new("L", (2000, 400), 0)
    d = ImageDraw.Draw(probe)
    d.text((10, 10), text, fill=255, font=font)
    box = probe.getbbox()
    crop = probe.crop(box)
    return np.asarray(crop.resize((width_px, height_px), Image.Resampling.LANCZOS), dtype=np.float32) / 255.0


def sample_text(mask, u, v):
    h, w = mask.shape
    inside = (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
    out = np.zeros(u.shape, dtype=np.float32)
    ix = np.clip((u * w).astype(int), 0, w - 1)
    iy = np.clip((v * h).astype(int), 0, h - 1)
    out[inside] = mask[iy[inside], ix[inside]]
    return out


def paint_livery(base_png, windows_png, group):
    """Dark-premium BBJ livery on the BOE atlas (see module docstring).
    `base_png` (BOE2, near-white) gives the panel and door lines;
    `windows_png` (BOE, house colours) gives the cabin window dots."""
    base = Image.open(base_png).convert("RGB")
    size = base.width
    if base.size != (size, size):
        raise PackError("livery base must be square")
    src = np.asarray(base, dtype=np.float32)
    win_src = np.asarray(Image.open(windows_png).convert("RGB").resize(base.size), dtype=np.float32)
    cls, pos = rasterize_uv(group, size)
    x, y, z = pos[..., 0], pos[..., 1], pos[..., 2]
    lum = src.mean(axis=2)
    blue = (src[..., 2] - src[..., 0]) > 25          # manufacturer marks and lines
    win_lum = win_src.mean(axis=2)
    win_dark = (win_lum < 90) & ((win_src[..., 2] - win_src[..., 0]) <= 25)
    out = src.copy()

    fus = cls == 1
    # Window line: the dark, non-blue window dots of the house livery.
    windows_px = fus & win_dark & (np.abs(y) > 1.5)
    if windows_px.sum() < 100:
        raise PackError("no cabin windows found in the window source")
    win_z = float(np.median(z[windows_px]))
    # Paint detail: panel lines/doors keep their relative shading; manufacturer
    # marks (blue) and window glass are handled separately.
    detail = np.clip(np.where(blue, 1.0, lum / 235.0), 0.75, 1.05)[..., None]
    body = np.array(NAVY, dtype=np.float32) * detail
    out[fus] = body[fus]
    # Cabin windows: uniform smoked glass (the source dots carry reflection
    # detail that reads as noise on dark paint) with a soft grey frame.
    window_glass = fus & win_dark & (np.abs(z - win_z) < 0.35)
    # Close the mask (dilate, then erode, 3 texels) to fill the highlights.
    def shift_any(m, r, fn):
        acc = m.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                acc = fn(acc, np.roll(m, (dy, dx), axis=(0, 1)))
        return acc
    window_glass = shift_any(shift_any(window_glass, 3, np.logical_or), 3, np.logical_and) & fus
    out[window_glass] = (10, 12, 18)
    ring = window_glass.copy()
    for dy, dx in ((-2, 0), (2, 0), (0, -2), (0, 2)):
        ring |= np.roll(window_glass, (dy, dx), axis=(0, 1))
    ring &= fus & ~window_glass
    out[ring] = out[ring] * 0.55 + np.array((120, 128, 140), dtype=np.float32) * 0.45
    # Champagne cheatlines under the windows, nose to tail cone.
    along = (x < 22.0) & (x > -24.5)
    for dz, half in ((0.62, 0.075), (0.84, 0.025)):
        band = fus & along & (np.abs(z - (win_z + dz)) < half)
        out[band] = GOLD
    # Registration aft of the last cabin window, centred on the window line
    # (both sides, reading nose to tail on the left and tail to nose on the
    # right, as painted on real aircraft).
    reg_len, reg_h = 3.4, 0.5
    x_front = float(x[windows_px].min()) - 0.6
    z_top = win_z - reg_h / 2
    mask = text_mask(REGISTRATION, 680, 100)
    u_left = (x_front - x) / reg_len
    u_right = (x - (x_front - reg_len)) / reg_len
    v = (z - z_top) / reg_h
    t = np.where(y < 0, sample_text(mask, u_left, v), sample_text(mask, u_right, v))
    t = np.where(fus, t, 0.0)[..., None]
    out = out * (1 - t) + np.array(GOLD, dtype=np.float32) * t

    fin = cls == 2
    out[fin] = np.array(NAVY, dtype=np.float32)
    if fin.any():
        # Leading and trailing edges as straight lines x(height), fitted to
        # the most forward / aft fin texel per height band.
        height = -z
        h0, h1 = float(height[fin].min()), float(height[fin].max())
        bands = np.linspace(h0, h1, 25)
        hs, le, te = [], [], []
        for b0, b1 in zip(bands, bands[1:]):
            sel = fin & (height >= b0) & (height < b1)
            if sel.sum() > 20:
                hs.append(0.5 * (b0 + b1))
                le.append(np.percentile(x[sel], 99.5))
                te.append(np.percentile(x[sel], 0.5))
        le_fit = np.polyfit(hs, le, 1)
        te_fit = np.polyfit(hs, te, 1)
        x_le = np.polyval(le_fit, height)
        x_te = np.polyval(te_fit, height)
        behind = x_le - x
        hn = (height - h0) / max(h1 - h0, 1e-6)
        sweep = fin & (behind > 0.45) & (behind < 0.75) & (hn > 0.08)
        out[sweep] = GOLD
        # GX monogram centred in the fin at 45 % height.
        hc = h0 + 0.45 * (h1 - h0)
        xc = 0.5 * (np.polyval(le_fit, hc) + np.polyval(te_fit, hc)) - 0.6
        mono_len, mono_h = 3.0, 2.0
        mono = text_mask("GX", 300, 200)
        mv = (hc + mono_h / 2 - height) / mono_h
        mu = np.where(y < 0, (xc + mono_len / 2 - x) / mono_len, (x - (xc - mono_len / 2)) / mono_len)
        mt = np.where(fin, sample_text(mono, mu, mv), 0.0)[..., None]
        out = out * (1 - mt) + np.array(GOLD, dtype=np.float32) * mt

    nac = cls == 3
    out[nac] = np.array(NAVY_LIGHT, dtype=np.float32) * np.clip(lum / 235.0, 0.8, 1.05)[nac][:, None]
    if nac.any():
        lip = nac & (x > np.percentile(x[nac], 97))
        out[lip] = GOLD
    # Texture padding: grow painted parts a few texels into unused space so
    # filtering and mip levels never pull in the old white paint at seams.
    painted = (cls == 1) | (cls == 2) | (cls == 3)
    for _ in range(6):
        grow = painted.copy()
        acc = np.zeros_like(out)
        cnt = np.zeros(painted.shape, dtype=np.float32)
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            shifted = np.roll(painted, (dy, dx), axis=(0, 1))
            acc += np.where(shifted[..., None], np.roll(out, (dy, dx), axis=(0, 1)), 0)
            cnt += shifted
        fill = (cls == 0) & ~painted & (cnt > 0)
        out[fill] = acc[fill] / cnt[fill][:, None]
        grow |= fill
        painted = grow
    stats = {"window_z_frd_m": win_z, "texels": {"fuselage": int(fus.sum()), "fin": int(fin.sum()),
                                                  "nacelle": int(nac.sum())}}
    return Image.fromarray(np.clip(out, 0, 255).astype(np.uint8)), stats


# ------------------------------------------------------------ GXVISUAL ---

def read_pack(data):
    """Split a GXVISUAL 1.0/1.1 pack into raw records."""
    magic, major, minor, fid, fgen, nt, nm, ng = struct.unpack_from("<8sHHQQIII", data, 0)
    if magic != b"GXVISUAL" or major != 1 or minor > 1:
        raise PackError(f"base pack must be GXVISUAL 1.0/1.1, got {major}.{minor}")
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
    vstride = 36 if minor >= 1 else 32
    for _ in range(ng):
        kind, mid, nv, ni = struct.unpack_from("<B3xIII", data, off)
        rec = data[off:off + 16 + nv * vstride + ni * 4]
        off += len(rec)
        if minor == 0:
            # Re-encode 1.0 vertices with a zero joint id.
            verts = rec[16:16 + nv * 32]
            out = bytearray(rec[:16])
            for k in range(nv):
                out += verts[k * 32:(k + 1) * 32] + b"\0\0\0\0"
            out += rec[16 + nv * 32:]
            rec = bytes(out)
        meshes.append((kind, mid, rec))
    if off != len(data):
        raise PackError("trailing bytes after the base pack")
    return {"frame_id": fid, "frame_generation": fgen, "textures": textures, "materials": materials,
            "meshes": meshes}


def texture_record(tid, levels, encoding, dims):
    out = bytearray(struct.pack("<IIIB3xI", tid, dims[0], dims[1], encoding, len(levels)))
    for raw in levels:
        out += struct.pack("<Q", len(raw)) + raw
    return bytes(out)


def mesh_record(group):
    pos = group["pos"].astype(np.float32)
    nrm = group["nrm"].astype(np.float32)
    uv = group["uv"].astype(np.float32)
    joint = group["joint"].astype(np.uint32)
    n = len(pos)
    verts = bytearray()
    for k in range(n):
        verts += struct.pack("<8fI", *pos[k], *nrm[k], *uv[k], joint[k])
    idx = struct.pack(f"<{n}I", *range(n))
    return verts, idx, n


def joint_record(j):
    out = bytearray(struct.pack("<IBBBB", j["parent"], DRIVERS[j["driver"]], 1 if j["kind"] == "rotate" else 2,
                                len(j["table"]), 0))
    out += struct.pack("<6f", *j["center"], *j["axis"])
    for x, y in j["table"]:
        out += struct.pack("<2f", x, y)
    return bytes(out)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--base", type=pathlib.Path, required=True, help="GXVISUAL 1.1 SFO pack")
    p.add_argument("--aircraft", type=pathlib.Path, required=True, help="FGAddon 787-8 directory")
    p.add_argument("--output", type=pathlib.Path, required=True)
    p.add_argument("--astcenc", type=pathlib.Path)
    p.add_argument("--rgba", action="store_true", help="RGBA8 textures (host previews, no astcenc)")
    a = p.parse_args(argv)
    from spike002_convert import astc_mips, mip_chain
    models = a.aircraft / "Models"
    groups, joints, stats = build(models / "787-8.ac", models / "787-8.xml")
    expected = {"BOE.png", "wings.png", "landing-gears.png", "__palette__"}
    if set(groups) != expected:
        raise PackError(f"unexpected texture groups {sorted(groups)}")
    a.output.mkdir(parents=True, exist_ok=True)
    livery, livery_stats = paint_livery(models / "Liveries" / "BOE2.png", models / "BOE.png", groups["BOE.png"])
    livery.save(a.output / "livery-bbj.png")
    palette, colours = palette_texture(groups["__palette__"])
    images = [(4, "BOE.png", livery, 0.30), (5, "wings.png", Image.open(models / "wings.png"), 0.45),
              (6, "landing-gears.png", Image.open(models / "landing-gears.png"), 0.55),
              (7, "__palette__", palette, 0.40)]
    base = read_pack((a.base / "sfo-photoreal.gxvisual").read_bytes() if a.base.is_dir() else a.base.read_bytes())
    textures = [rec for tid, rec in base["textures"] if tid != 4]
    materials = [rec for mid, tid, rec in base["materials"] if mid != 4]
    meshes = [rec for kind, mid, rec in base["meshes"] if kind != 2]
    tex_info = []
    for tid, key, img, rough in images:
        img = ImageOps.flip(img.convert("RGBA"))      # GL t = 0 is the image bottom (AC UVs)
        levels = mip_chain(img)
        if a.rgba or key == "__palette__":
            raw = [lvl.tobytes() for lvl in levels]
            enc = 2
        else:
            if not a.astcenc or not a.astcenc.is_file():
                raise PackError("--astcenc is required unless --rgba")
            raw = astc_mips(levels, a.astcenc, "-medium")
            enc = 1
        textures.append(texture_record(tid, raw, enc, img.size))
        materials.append(struct.pack("<II6f", tid, tid, 1.0, 1.0, 1.0, 1.0, rough, 0.0))
        verts, idx, n = mesh_record(groups[key])
        meshes.append(struct.pack("<B3xIII", 2, tid, n, n) + verts + idx)
        tex_info.append({"id": tid, "source": key, "size": list(img.size), "encoding": "astc-6x6" if enc == 1 else "rgba8",
                         "bytes": sum(len(r) for r in raw), "triangles": n // 3, "roughness": rough})
    out = bytearray(struct.pack("<8sHHQQIII", b"GXVISUAL", 1, 2, base["frame_id"], base["frame_generation"],
                                len(textures), len(materials), len(meshes)))
    for rec in textures:
        out += rec
    for rec in materials:
        out += rec
    for rec in meshes:
        out += rec
    out += struct.pack("<I", len(joints))
    for j in joints:
        out += joint_record(j)
    pack = a.output / "sfo-photoreal.gxvisual"
    pack.write_bytes(bytes(out))
    manifest = {
        "schema": "gx-b787-pack/1", "tool_version": TOOL_VERSION,
        "base": {"path": str(a.base), "sha256": sha256(a.base / "sfo-photoreal.gxvisual" if a.base.is_dir() else a.base)},
        "aircraft": {"source": "FlightGear FGAddon 787-8 (Aircraft-2024 hangar)", "license": "GPL-2.0; redistributed derivatives must preserve the GPL and corresponding source",
                     "ac_sha256": sha256(models / "787-8.ac"), "xml_sha256": sha256(models / "787-8.xml"),
                     "livery_base_sha256": sha256(models / "Liveries" / "BOE2.png"),
                     "livery": {"style": "dark premium BBJ", "registration": REGISTRATION, **livery_stats},
                     "palette_colours": colours, **stats},
        "textures": tex_info,
        "joints": [{k: j[k] for k in ("parent", "driver", "kind", "center", "axis", "table", "property", "objects")} for j in joints],
        "gxvisual": {"bytes": len(out), "sha256": sha256(pack), "format": "1.2"},
    }
    (a.output / "b787-manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=True), encoding="utf-8")
    print(json.dumps({"output": str(pack), "bytes": len(out), "joints": len(joints), "triangles": stats["triangles"],
                      "textures": stats["textures"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

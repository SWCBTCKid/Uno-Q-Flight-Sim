"""Host evaluation of GXVISUAL 1.2 joints (mirror of gx-render photo.rs
joint_transforms), shared by tests and the preview renderer."""

from __future__ import annotations

import math

import numpy as np

DRIVER_INDEX = {"aileron": 0, "elevator": 1, "rudder": 2, "flap": 3, "speedbrake": 4, "gear": 5}


def evaluate(table, x):
    if not math.isfinite(x):
        x = 0.0
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (x - x0) / (x1 - x0) * (y1 - y0)
    return table[-1][1]


def local(j, value):
    out = evaluate(j["table"], value)
    a = np.asarray(j["axis"], dtype=np.float64)
    m = np.eye(4)
    if j["kind"] in ("translate", 2):
        m[:3, 3] = a * out
        return m
    th = math.radians(out)
    s, c = math.sin(th), math.cos(th)
    t = 1 - c
    r = np.array([
        [c + a[0] * a[0] * t, a[0] * a[1] * t - a[2] * s, a[0] * a[2] * t + a[1] * s],
        [a[1] * a[0] * t + a[2] * s, c + a[1] * a[1] * t, a[1] * a[2] * t - a[0] * s],
        [a[2] * a[0] * t - a[1] * s, a[2] * a[1] * t + a[0] * s, c + a[2] * a[2] * t],
    ])
    p = np.asarray(j["center"], dtype=np.float64)
    m[:3, :3] = r
    m[:3, 3] = p - r @ p
    return m


def transforms(joints, drivers):
    """drivers: dict name -> value. Returns list of 4x4, index 0 identity."""
    world = [np.eye(4)]
    for j in joints:
        world.append(world[j["parent"]] @ local(j, drivers.get(j["driver"], 0.0)))
    return world

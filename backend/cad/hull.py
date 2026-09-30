"""CadQuery builder for the displacement monohull and its hardware.

Build order: outer envelope loft -> minus cavity loft -> deck opening ->
coaming + gasket groove + magnet pads/bores -> rudder tube boss + bore ->
stuffing-tube boss + bore -> motor mount plate + holes -> servo thwart +
SG90 hole. Every printed fitting inside the hull is first intersected with the
cavity, so it bonds to the skin and can never poke through it. Every cut is
existence-checked afterwards by point classification (verify.py): a boolean
that "succeeds" but removes nothing is a failure, not a feature.

A layout check that failed in the physics skips its feature and the part list
says so by name.
"""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np

from ..physics.hullform import (HullParams, canoe_depth, deck_height,
                                inner_side_curve, side_curve)
from ..physics.layout import BOSS_WALL_MM

V = cq.Vector


def _station_xs(x0: float, x1: float, n: int) -> list[float]:
    # cosine clustering: sections change fastest at the stem and the transom
    return [x0 + (x1 - x0) * 0.5 * (1 - math.cos(math.pi * k / (n - 1))) for k in range(n)]


def _section_wire(x: float, curve: np.ndarray) -> cq.Wire:
    """Closed wire: port spline keel->deck edge, deck line, starboard spline back."""
    port = [V(x, float(y), float(z)) for y, z in curve]
    stbd = [V(x, -float(y), float(z)) for y, z in curve[::-1]]
    # no tangent is forced at the keel: the cosine-clustered points already run
    # nearly flat there, and a forced tangent scaled to the chord makes the tall,
    # 1.2 mm-wide stem sections self-intersect
    e1 = cq.Edge.makeSpline(port)
    e2 = cq.Edge.makeLine(port[-1], stbd[0])
    e3 = cq.Edge.makeSpline(stbd)
    return cq.Wire.assembleEdges([e1, e2, e3])


def envelope(p: HullParams, n: int = 29) -> cq.Solid:
    wires = [_section_wire(x, side_curve(p, x / p.L)) for x in _station_xs(0.0, p.L, n)]
    return cq.Solid.makeLoft(wires, False)


def cavity(p: HullParams, wall: float, x0: float, x1: float, n: int = 25) -> cq.Solid:
    wires = [_section_wire(x, inner_side_curve(p, x / p.L, wall)) for x in _station_xs(x0, x1, n)]
    return cq.Solid.makeLoft(wires, False)


def _box(x0, x1, y0, y1, z0, z1) -> cq.Solid:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def _cyl_z(x, y, r, z0, z1) -> cq.Solid:
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def _cyl_dir(p0: tuple, d: tuple, r: float, length: float) -> cq.Solid:
    return cq.Solid.makeCylinder(r, length, V(*p0), V(*d))


def _ok(lay: dict, name: str) -> bool:
    return all(c["ok"] for c in lay["checks"] if c["name"] == name)


def build_hull(p: HullParams, lay: dict) -> tuple[cq.Shape, dict]:
    """The printed hull (one watertight solid) and a report of features/refusals.

    Features are collected first and applied as ONE fuse and ONE cut: each
    boolean against the lofted hull costs seconds, so twenty of them in a row
    cost a minute. The order is still additive-then-subtractive, which is all
    the features need (every bore goes through a boss that already exists).
    """
    wall = lay["wall_mm"]
    report: dict = {"features": ["hollow hull, %.1f mm wall" % wall], "refused": []}
    env = envelope(p)
    cav = cavity(p, wall, *lay["cavity_x_mm"])
    adds: list[cq.Shape] = []
    cuts: list[cq.Shape] = []

    def feature(check: str, label: str, fn) -> None:
        if _ok(lay, check):
            fn()
            report["features"].append(label)
        else:
            report["refused"].append(check)

    # ---- deck opening, coaming, gasket groove, magnet pads -------------------
    def hatch():
        h = lay["hatch"]
        x0, x1, hw, cw = h["x0"], h["x1"], h["half_w"], h["coaming_wall"]
        ztop = h["coaming_top_z"]
        zdeck_lo = min(deck_height(p, x / p.L) for x in np.linspace(x0, x1, 9))
        zroot = zdeck_lo - wall - 2.0
        adds.append(_box(x0 - cw, x1 + cw, -hw - cw, hw + cw, zroot, ztop).cut(
            _box(x0, x1, -hw, hw, zroot - 1, ztop + 1)))
        # pads in the four inside corners, hanging from the coaming walls
        pad, pd = h["magnet_pad"], h["magnet_pad_depth"]
        pads = []
        for (px, sx) in ((x0, 1), (x1, -1)):
            for (py, sy) in ((-hw, 1), (hw, -1)):
                ax, bx = sorted((px, px + sx * pad))
                ay, by = sorted((py, py + sy * pad))
                pads.append(_box(ax, bx, ay, by, ztop - pd, ztop))
        adds.extend(pads)
        # deck opening inside the coaming, leaving the pads
        cuts.append(_box(x0, x1, -hw, hw, zdeck_lo - wall - 1.0, ztop + 5).cut(
            cq.Compound.makeCompound(pads)))
        # gasket groove centred on the coaming wall top
        gw, gd = h["groove_w"], h["groove_d"]
        mid = cw / 2
        g_out = _box(x0 - mid - gw / 2, x1 + mid + gw / 2, -hw - mid - gw / 2, hw + mid + gw / 2,
                     ztop - gd, ztop + 1)
        g_in = _box(x0 - mid + gw / 2, x1 + mid - gw / 2, -hw - mid + gw / 2, hw + mid - gw / 2,
                    ztop - gd - 1, ztop + 2)
        cuts.append(g_out.cut(g_in))
        # magnet bores in the pads, 0.2 mm below flush so the discs never stand proud
        mh = h["magnet_h"] + 0.2
        for (cx, cy) in magnet_centres(lay):
            cuts.append(_cyl_z(cx, cy, h["magnet_dia"] / 2, ztop - mh, ztop + 1))
    feature("hatch.size", "hatch opening, 10 mm coaming, gasket groove, 4 magnet bores", hatch)

    # ---- rudder tube -----------------------------------------------------------
    def rudder():
        r = lay["rudder"]
        zk = -canoe_depth(p, r["x"] / p.L)
        adds.append(_cyl_z(r["x"], 0, r["boss_od"] / 2, zk - 2, r["tube_top_z"]).intersect(cav))
        cuts.append(_cyl_z(r["x"], 0, r["tube_bore"] / 2, zk - 10, r["tube_top_z"] + 1))
    feature("rudder_tube.above_waterline", "rudder tube boss and bore", rudder)

    # ---- stuffing tube -----------------------------------------------------------
    s = lay["shaft"]
    th = math.radians(s["angle_deg"])
    d = (math.cos(th), 0.0, math.sin(th))

    def stuffing():
        start = (s["x_exit"] - 15 * d[0], 0.0, s["z_exit"] - 15 * d[2])
        length = 15 + s["tube_len"]
        adds.append(_cyl_dir(start, d, s["boss_od"] / 2, length).intersect(cav))
        # a web from the boss down to the floor along its whole length: the shaft
        # line climbs away from the floor, and an unsupported tube would hang
        # free in the cavity (and fall apart when the hull is split for the bed)
        web_w = 2 * BOSS_WALL_MM
        web = _box(0, length, -web_w / 2, web_w / 2, -3 * p.T, 0)
        web = web.rotate(V(0, 0, 0), V(0, 1, 0), -s["angle_deg"]).translate(V(*start))
        adds.append(web.intersect(cav))
        cuts.append(_cyl_dir(start, d, s["tube_bore"] / 2, length + 1))
    feature("stuffing_tube.inside", "stuffing tube boss and bore, %.0f deg" % s["angle_deg"], stuffing)

    # ---- motor mount plate ---------------------------------------------------------
    m = lay["motor"]

    def mount():
        half = m["can_r"] + 6.0
        plate = _box(0, m["plate_t"], -half, half, -(m["z_axis"] - (-p.T - 30)), half)
        plate = plate.rotate(V(0, 0, 0), V(0, 1, 0), -s["angle_deg"]).translate(
            V(m["x_face"] - m["plate_t"] * d[0], 0, m["z_axis"] - m["plate_t"] * d[2]))
        adds.append(plate.intersect(cav))
        o = (m["x_face"] - (m["plate_t"] + 2) * d[0], 0.0, m["z_axis"] - (m["plate_t"] + 2) * d[2])
        cut_len = m["plate_t"] + 4
        cuts.append(_cyl_dir(o, d, 5.0, cut_len))              # bearing boss + shaft
        for sy in (-1, 1):
            cuts.append(_cyl_dir((o[0], sy * m["screw_spacing"] / 2, o[2]), d, 1.45, cut_len))
    feature("motor_mount.fits", "motor mount plate, %s pattern" % m["name"], mount)

    # ---- servo thwart ----------------------------------------------------------------
    sv = lay["servo"]

    def servo():
        tz = sv["z_top"]
        adds.append(_box(sv["x"] - sv["tray_x"] / 2, sv["x"] + sv["tray_x"] / 2,
                         -p.B, p.B, tz - sv["thwart_t"], tz).intersect(cav))
        cuts.append(_box(sv["x"] - sv["pocket_x"] / 2, sv["x"] + sv["pocket_x"] / 2,
                         -sv["pocket_y"] / 2, sv["pocket_y"] / 2,
                         tz - sv["thwart_t"] - 1, tz + 1))
    feature("servo_tray.fits", "SG90 servo thwart", servo)

    hull = env.cut(cav)
    if adds:
        hull = hull.fuse(*adds)
    if cuts:
        hull = hull.cut(*cuts)
    return hull.clean(), report


def magnet_centres(lay: dict) -> list[tuple[float, float]]:
    h = lay["hatch"]
    x0, x1, hw, pad = h["x0"], h["x1"], h["half_w"], h["magnet_pad"]
    return [(x0 + pad / 2, -hw + pad / 2), (x0 + pad / 2, hw - pad / 2),
            (x1 - pad / 2, -hw + pad / 2), (x1 - pad / 2, hw - pad / 2)]


def build_lid(lay: dict) -> cq.Shape:
    h = lay["hatch"]
    x0, x1, hw, cw, c = h["x0"], h["x1"], h["half_w"], h["coaming_wall"], h["lid_clearance"]
    ztop, t, sk = h["coaming_top_z"], h["lid_plate"], h["lid_skirt"]
    skirt_t = 2.0
    ix0, ix1, iy = x0 - cw - c, x1 + cw + c, hw + cw + c        # skirt inner faces
    ox0, ox1, oy = ix0 - skirt_t, ix1 + skirt_t, iy + skirt_t
    lid = _box(ox0, ox1, -oy, oy, ztop, ztop + t)
    lid = lid.fuse(_box(ox0, ox1, -oy, oy, ztop - sk, ztop).cut(
        _box(ix0, ix1, -iy, iy, ztop - sk - 1, ztop + 0.01)))
    mh = h["magnet_h"] + 0.2
    for (cx, cy) in magnet_centres(lay):
        lid = lid.cut(_cyl_z(cx, cy, h["magnet_dia"] / 2, ztop - 1, ztop + mh))
    return lid.clean()


def naca00(tc: float, chord: float, n: int = 40) -> list[tuple[float, float]]:
    """Closed-trailing-edge NACA 00xx profile, LE at 0, TE at chord."""
    pts = []
    for k in range(n + 1):
        xc = 0.5 * (1 - math.cos(math.pi * k / n))
        yt = 5 * tc * (0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2
                       + 0.2843 * xc ** 3 - 0.1036 * xc ** 4)
        pts.append((xc * chord, yt * chord))
    upper = pts
    lower = [(x, -y) for x, y in pts[::-1]][1:-1]
    return upper + lower


def build_rudder(lay: dict) -> cq.Shape:
    """Balanced rudder: stock at 25 % chord, blade hanging below the hull."""
    r = lay["rudder"]
    c = r["chord"]
    prof = naca00(0.12, c)
    # profile in the x-y plane; LE forward (+x): x = x_r + 0.25 c - s
    pts = [(r["x"] + 0.25 * c - s, y) for s, y in prof]
    blade = (cq.Workplane("XY", origin=(0, 0, r["z_tip"]))
             .polyline(pts).close().extrude(r["span"]).val())
    # stock hole, 3 mm rod glued in, 80 % of the span
    hole = _cyl_z(r["x"], 0, r["stock_dia"] / 2 + 0.05, r["z_tip"] + 0.2 * r["span"], r["z_root"] + 1)
    return blade.cut(hole).clean()


def build_parts(design: dict) -> dict[str, cq.Shape]:
    p = HullParams(**design["params"])
    lay = design["layout"]
    hull, report = build_hull(p, lay)
    parts = {"hull": hull, "hatch_lid": build_lid(lay), "rudder": build_rudder(lay)}
    parts["_report"] = report
    return parts

"""Prove the exported geometry, not the builder's intentions.

1. Hydrostatics of the exported mesh: the envelope STL must displace the
   design volume at z = 0, float at z = 0 under the design mass, and put its
   LCB on the design LCG.
2. Every hardware cut exists: point classification of the finished solid at
   the centre of each bore, pocket, groove and the cavity.
3. The printed hull is one valid solid whose mesh is watertight.
"""
from __future__ import annotations

import io
import math

import cadquery as cq
import numpy as np
import trimesh
from scipy.optimize import brentq

from ..jsonsafe import plain
from ..physics.components import PLA_DENSITY
from ..physics.hullform import HullParams, canoe_depth
from ..physics.resistance import RHO_FRESH
from .hull import envelope, magnet_centres, well_bore

RHO_G_PER_MM3 = RHO_FRESH * 1e-6


def to_mesh(shape: cq.Shape, tol: float = 0.05, ang: float = 0.1) -> trimesh.Trimesh:
    verts, tris = shape.tessellate(tol, ang)
    v = np.array([(p.x, p.y, p.z) for p in verts])
    m = trimesh.Trimesh(v, np.array(tris), process=True)
    m.merge_vertices()
    return m


def below(mesh: trimesh.Trimesh, h: float) -> trimesh.Trimesh | None:
    return mesh.slice_plane([0, 0, h], [0, 0, -1], cap=True)


def mesh_hydrostatics(mesh: trimesh.Trimesh, mass_g: float, lcg: float) -> dict:
    sub0 = below(mesh, 0.0)
    vol0 = float(sub0.volume)
    target = mass_g / RHO_G_PER_MM3
    zmin, zmax = mesh.bounds[:, 2]
    h = brentq(lambda z: below(mesh, z).volume - target, zmin + 0.1, zmax - 0.1, xtol=1e-3)
    return {"volume_at_dwl_mm3": vol0, "lcb_mm": float(sub0.center_mass[0]),
            "kb_mm": float(sub0.center_mass[2]), "float_z_mm": float(h),
            "lcb_minus_lcg_mm": float(sub0.center_mass[0] - lcg)}


def _empty(shape: cq.Shape, pt) -> bool:
    return not shape.isInside(cq.Vector(*pt), 1e-3)


def _solid(shape: cq.Shape, pt) -> bool:
    return shape.isInside(cq.Vector(*pt), 1e-3)


def cut_checks(hull: cq.Shape, p: HullParams, lay: dict) -> list[dict]:
    out = []
    add = lambda n, ok: out.append({"name": n, "ok": bool(ok)})
    w = lay["wall_mm"]
    xm = p.x_max * p.L
    zk = -canoe_depth(p, p.x_max)
    add("cavity.empty", _empty(hull, (xm, 0, 0)))
    add("keel_skin.solid", _solid(hull, (xm, 0, zk + w / 2)))
    h = lay["hatch"]
    zt = h["coaming_top_z"]
    add("hatch_opening.empty", _empty(hull, ((h["x0"] + h["x1"]) / 2, 0, zt - 3)))
    add("coaming_wall.solid", _solid(hull, (h["x0"] - h["coaming_wall"] * 0.2, 0, zt - 3)))
    add("gasket_groove.empty",
        _empty(hull, (h["x0"] - h["coaming_wall"] / 2, 0, zt - h["groove_d"] / 2)))
    for i, (cx, cy) in enumerate(magnet_centres(lay)):
        add(f"magnet_bore_{i}.empty", _empty(hull, (cx, cy, zt - 1.5)))
        add(f"magnet_pad_{i}.solid", _solid(hull, (cx, cy, zt - h["magnet_h"] - 2)))
    r = lay["rudder"]
    zkr = -canoe_depth(p, r["x"] / p.L)
    add("rudder_bore.through_skin", _empty(hull, (r["x"], 0, zkr + w / 2)))
    add("rudder_bore.through_boss_top", _empty(hull, (r["x"], 0, r["tube_top_z"] - 1)))
    add("rudder_boss.solid", _solid(hull, (r["x"] + r["tube_bore"] / 2 + 1.0, 0, r["tube_top_z"] - 3)))
    s = lay["shaft"]
    th = math.radians(s["angle_deg"])
    for f in (0.1, 0.5, 0.95):
        x = s["x_exit"] + f * s["tube_len"] * math.cos(th)
        z = s["z_exit"] + f * s["tube_len"] * math.sin(th)
        add(f"stuffing_bore_{f}.empty", _empty(hull, (x, 0, z)))
    mid = (s["x_exit"] + 0.5 * s["tube_len"] * math.cos(th), 0,
           s["z_exit"] + 0.5 * s["tube_len"] * math.sin(th) + s["tube_bore"] / 2 + 1.0)
    add("stuffing_boss.solid", _solid(hull, mid))
    m = lay["motor"]
    back = (m["x_face"] - m["plate_t"] / 2 * math.cos(th), 0, m["z_axis"] - m["plate_t"] / 2 * math.sin(th))
    add("motor_shaft_hole.empty", _empty(hull, back))
    add("motor_plate.solid", _solid(hull, (back[0], m["can_r"] + 2, back[2])))
    for sy in (-1, 1):
        add(f"motor_screw_{sy}.empty", _empty(hull, (back[0], sy * m["screw_spacing"] / 2, back[2])))
    sv = lay["servo"]
    add("servo_pocket.empty", _empty(hull, (sv["x"], 0, sv["z_top"] - sv["thwart_t"] / 2)))
    add("servo_thwart.solid",
        _solid(hull, (sv["x"], sv["pocket_y"] / 2 + 3, sv["z_top"] - sv["thwart_t"] / 2)))
    for i, wl in enumerate(lay.get("wells", [])):
        zk = wl["z_keel"]
        add(f"well_{i}.open_to_water", _empty(hull, (wl["x"], 0, zk + w / 2)))
        add(f"well_{i}.bore_above_waterline", _empty(hull, (wl["x"], 0, wl["top_z"] - 1)))
        add(f"well_{i}.wall.solid", _solid(hull, (wl["x"] + wl["bore"] / 2 + 1.0, 0, wl["top_z"] - 3)))
    return out


def verify_design(design: dict, parts: dict) -> dict:
    p = HullParams(**design["params"])
    lay = design["layout"]
    hull = parts["hull"]
    env = envelope(p)
    wells = lay.get("wells", [])
    if wells:
        # the design volume is net of the flooded wells: water inside them is sea
        env = env.cut(*[well_bore(w) for w in wells])
    env_mesh = to_mesh(env)
    hy = mesh_hydrostatics(env_mesh, design["mass"]["total_g"], design["hydrostatics"]["lcg_mm"])
    hull_mesh = to_mesh(hull, 0.1, 0.2)
    printed = {k: volume(parts[k]) * PLA_DENSITY for k in ("hull", "hatch_lid", "rudder")}
    phys_printed = sum(i["mass_g"] for i in design["mass"]["items"]
                       if i["name"] in ("hull_shell", "coaming", "magnet_pads", "hatch_lid",
                                        "deck_opening", "rudder_boss", "rudder_blade",
                                        "stuffing_boss", "motor_mount", "servo_tray")
                       or i["name"].startswith("well boss"))
    return plain({
        "mesh_hydrostatics": hy,
        "design_volume_mm3": design["hydrostatics"]["volume_mm3"],
        "volume_error": hy["volume_at_dwl_mm3"] / design["hydrostatics"]["volume_mm3"] - 1,
        "float_error_frac_T": hy["float_z_mm"] / p.T,
        "trim_error_frac_L": hy["lcb_minus_lcg_mm"] / p.L,
        "hull_valid": hull.isValid(),
        "hull_solids": len(hull.Solids()),
        "hull_mesh_watertight": bool(hull_mesh.is_watertight),
        "printed_mass_g": printed,
        "printed_mass_physics_g": phys_printed,
        "cuts": cut_checks(hull, p, lay),
    })


def volume(shape: cq.Shape) -> float:
    """Volume with an explicit integration tolerance. CadQuery's default
    Volume() integrates B-spline faces coarsely and can be several percent off
    on a lofted hull; this agrees with the tessellated mesh."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props, 1e-6, True)
    return props.Mass()

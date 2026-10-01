"""Mass and centre of gravity: printed hull, hardware, and the ballast solve.

The printed hull is a thin shell: surface area (girth integrated along x plus
the transom) times the wall, at PLA density. Printed hardware (coaming, lid,
bosses, tray, mount) is estimated from its layout dimensions. The CAD test
weighs the real exported solid and holds this estimate to it.
"""
from __future__ import annotations

import math

import numpy as np

from . import components as C
from .hullform import HullParams, section_polygon
from .hydrostatics import poly_area_centroid


def _perimeter_centroid(poly: np.ndarray) -> tuple[float, float, float]:
    seg = np.roll(poly, -1, axis=0) - poly
    lens = np.hypot(seg[:, 0], seg[:, 1])
    mid = poly + 0.5 * seg
    per = lens.sum()
    return per, float((mid[:, 0] * lens).sum() / per), float((mid[:, 1] * lens).sum() / per)


def shell_mass(p: HullParams, wall: float, n: int = 121) -> dict:
    xs = np.linspace(0.0, p.L, n)
    per = np.zeros(n); zc = np.zeros(n)
    for i, x in enumerate(xs):
        per[i], _, zc[i] = _perimeter_centroid(section_polygon(p, x / p.L))
    area = float(np.trapezoid(per, xs))
    ax = float(np.trapezoid(per * xs, xs))
    az = float(np.trapezoid(per * zc, xs))
    # transom plate at x = 0
    ta, _, tz = poly_area_centroid(section_polygon(p, 0.0))
    area_t = area + ta
    vol = area_t * wall
    return {"area_mm2": area_t, "volume_mm3": vol, "mass_g": vol * C.PLA_DENSITY,
            "cg": ((ax + 0.0 * ta) / area_t, (az + tz * ta) / area_t)}


def printed_hardware(lay: dict) -> list[dict]:
    """Rough masses of the printed features, each with its CG (x, z)."""
    rho = C.PLA_DENSITY
    h = lay["hatch"]
    items = []
    lx = h["x1"] - h["x0"]; wy = 2 * h["half_w"]
    cw = h["coaming_wall"]
    coam_h = 10.0 + lay["wall_mm"] + 4.0   # height above deck + deck + root overlap
    per = 2 * (lx + wy) + 4 * cw
    items.append({"name": "coaming", "mass_g": per * cw * coam_h * rho,
                  "cg": ((h["x0"] + h["x1"]) / 2, h["coaming_top_z"] - coam_h / 2)})
    items.append({"name": "magnet_pads",
                  "mass_g": 4 * h["magnet_pad"] ** 2 * h["magnet_pad_depth"] * rho,
                  "cg": ((h["x0"] + h["x1"]) / 2, h["coaming_top_z"] - 5.0)})
    # lid plate over the coaming plus its skirt; minus the deck opening it replaces
    ox = lx + 2 * cw + 2 * h["lid_clearance"] + 4.0
    oy = wy + 2 * cw + 2 * h["lid_clearance"] + 4.0
    lid = (ox * oy * h["lid_plate"] + 2 * (ox + oy) * 2.0 * h["lid_skirt"]) * rho
    items.append({"name": "hatch_lid", "mass_g": lid,
                  "cg": ((h["x0"] + h["x1"]) / 2, h["coaming_top_z"] + 1.0)})
    items.append({"name": "deck_opening", "mass_g": -lx * wy * lay["wall_mm"] * rho,
                  "cg": ((h["x0"] + h["x1"]) / 2, h["coaming_top_z"] - 10.0)})
    r = lay["rudder"]
    tube_h = r["tube_top_z"] - r["z_root"]
    items.append({"name": "rudder_boss",
                  "mass_g": math.pi / 4 * (r["boss_od"] ** 2 - r["tube_bore"] ** 2) * tube_h * rho,
                  "cg": (r["x"], r["z_root"] + tube_h / 2)})
    # printed rudder blade, NACA 0012: section area = 0.685 * t * c = 0.082 c^2, solid
    items.append({"name": "rudder_blade",
                  "mass_g": r["chord"] * r["span"] * 0.082 * r["chord"] * rho,
                  "cg": (r["x"], r["z_root"] - r["span"] / 2)})
    s = lay["shaft"]
    web_h = max(s["z_tube_end"] - s["z_exit"], 0.0) / 2 + s["boss_od"] / 2   # mean web height
    items.append({"name": "stuffing_boss",
                  "mass_g": (math.pi / 4 * (s["boss_od"] ** 2 - s["tube_bore"] ** 2)
                             + 2 * 2.4 * web_h) * s["tube_len"] * rho,
                  "cg": ((s["x_exit"] + s["x_tube_end"]) / 2, (s["z_exit"] + s["z_tube_end"]) / 2)})
    m = lay["motor"]
    plate = (2 * m["can_r"] + 12) ** 2 * m["plate_t"] * rho
    items.append({"name": "motor_mount", "mass_g": plate, "cg": (m["x_face"], m["z_axis"])})
    sv = lay["servo"]
    thwart_w = 2 * lay["servo"].get("half_w", 50.0)
    tray = (sv["tray_x"] * thwart_w - sv["pocket_x"] * sv["pocket_y"]) * sv["thwart_t"] * rho
    items.append({"name": "servo_tray", "mass_g": tray, "cg": (sv["x"], sv["z_top"] - 2.0)})
    return items


def mass_budget(p: HullParams, lay: dict, motor: dict, battery: dict) -> dict:
    items = []
    sh = shell_mass(p, lay["wall_mm"])
    items.append({"name": "hull_shell", "mass_g": sh["mass_g"], "cg": sh["cg"]})
    items += printed_hardware(lay)
    items.append({"name": motor["name"], "mass_g": motor["mass_g"], "cg": lay["motor"]["cg"]})
    items.append({"name": battery["name"], "mass_g": battery["mass_g"], "cg": lay["battery"]["cg"]})
    items.append({"name": "servo_SG90", "mass_g": C.FIXED_PARTS["servo_SG90"], "cg": lay["servo"]["cg"]})
    e = lay["electronics"]["cg"]
    for k in ("receiver", "esc_brushed", "wiring"):
        items.append({"name": k, "mass_g": C.FIXED_PARTS[k], "cg": e})
    s = lay["shaft"]
    items.append({"name": "shaft_prop_coupler", "mass_g": C.FIXED_PARTS["shaft_prop_coupler"],
                  "cg": ((lay["prop"]["x"] + s["x_tube_end"]) / 2, (lay["prop"]["z"] + s["z_tube_end"]) / 2)})
    r = lay["rudder"]
    items.append({"name": "rudder_stock_tiller", "mass_g": C.FIXED_PARTS["rudder_stock_tiller"],
                  "cg": (r["x"], r["tube_top_z"])})
    h = lay["hatch"]
    items.append({"name": "magnets+gasket",
                  "mass_g": C.FIXED_PARTS["hatch_magnets"] + C.FIXED_PARTS["gasket"],
                  "cg": ((h["x0"] + h["x1"]) / 2, h["coaming_top_z"])})
    for it in lay.get("payload", []):
        items.append({"name": f"payload: {it['name']}", "mass_g": it["mass_g"], "cg": it["cg"],
                      "payload": True})
    for w in lay.get("wells", []):
        rho = C.PLA_DENSITY
        hgt = w["top_z"] - w["z_keel"]
        items.append({"name": f"well boss ({w['key']})",
                      "mass_g": math.pi / 4 * (w["od"] ** 2 - w["bore"] ** 2) * hgt * rho,
                      "cg": (w["x"], w["z_keel"] + hgt / 2)})
    m = sum(i["mass_g"] for i in items)
    xg = sum(i["mass_g"] * i["cg"][0] for i in items) / m
    zg = sum(i["mass_g"] * i["cg"][1] for i in items) / m
    return {"items": items, "mass_g": m, "cg": (xg, zg), "shell": sh}


def solve_ballast(budget: dict, displacement_g: float, lcb: float,
                  x_range: tuple[float, float], z_at: callable) -> dict:
    """Lead ballast that brings the boat to its design displacement AND puts
    LCG on LCB (level trim). Returns the ballast item and whether it reached."""
    m0 = budget["mass_g"]; xg0, zg0 = budget["cg"]
    mb = displacement_g - m0
    if mb <= 0:
        return {"mass_g": mb, "x": None, "reached": False, "cg": budget["cg"]}
    xb = (displacement_g * lcb - m0 * xg0) / mb
    reached = x_range[0] <= xb <= x_range[1]
    xb_c = min(max(xb, x_range[0]), x_range[1])
    zb = z_at(xb_c) + 4.0   # lead lump centre just above the floor
    xg = (m0 * xg0 + mb * xb_c) / displacement_g
    zg = (m0 * zg0 + mb * zb) / displacement_g
    return {"mass_g": mb, "x": xb_c, "x_wanted": xb, "z": zb, "reached": reached, "cg": (xg, zg)}

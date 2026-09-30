"""Where every piece of hardware goes, from the hull parameters alone.

The mass model (CG) and the CAD builder both read this, so a part is never in
one place for the physics and another for the geometry. Every placement that
can fail returns a named check instead of silently shipping without the part.
All lengths mm, frame as in hullform.py.
"""
from __future__ import annotations

import math

from . import components as C
from .hullform import (HullParams, canoe_depth, deck_height, flare,
                       half_breadth_wl, inner_half_breadth_at, inner_side_curve)

# SG90 envelope, measured off the AeroForge builder's own SG90 model
# (aeroforge backend/cad/servos.py SERVO_SG90): case 23.00 x 12.40, ear
# underside 17.55 above the case bottom, ears 32.50 tip to tip, spline top 32.55.
SG90 = {"case_len": 23.00, "case_wid": 12.40, "ear_z": 17.55,
        "ear_span": 32.50, "total_h": 32.55}
SERVO_CLEARANCE_MM = 0.25       # per face, the AeroForge servo doctrine

COAMING_WALL_MM = 4.0           # wide enough to carry the gasket groove
COAMING_HEIGHT_MM = 10.0        # above the highest deck point along the hatch
GASKET_GROOVE_W_MM = 1.6        # for 1.5 mm cord / 1.5 mm closed-cell foam
GASKET_GROOVE_D_MM = 1.2        # ~20 % squeeze on a 1.5 mm cord
LID_PLATE_MM = 2.4
LID_SKIRT_MM = 6.0
LID_CLEARANCE_MM = 0.35         # per side, AeroForge printed-fit band
MAGNET_DIA_MM = 8.15            # as-cut bore, AeroForge builder's spec
MAGNET_H_MM = 3.0
MAGNET_PAD_MM = 12.0
MAGNET_PAD_DEPTH_MM = 10.0
BOSS_WALL_MM = 2.4              # six perimeters around a brass tube

SHAFT_ANGLE_DEG = 8.0
RUDDER_AREA_FRAC = 0.025        # A_rudder / (L * T); see _rudder()
COUPLER_GAP_MM = 24.0          # tube end to mount plate: a 20 mm shaft coupler + play
MOUNT_PLATE_MM = 4.0


def _chk(name: str, ok: bool, detail: str) -> dict:
    return {"name": name, "ok": bool(ok), "detail": detail}


def cavity_range(p: HullParams, wall: float) -> tuple[float, float]:
    """x range the cavity is lofted over: from inside the transom to the last
    station whose cavity is still at least 6 mm wide at the waterline."""
    x0 = wall
    xi = 0.99
    while xi > 0.6:
        if half_breadth_wl(p, xi) - wall >= 6.0:
            break
        xi -= 0.005
    return x0, xi * p.L


def keel_z(p: HullParams, x: float) -> float:
    return -canoe_depth(p, min(max(x / p.L, 0.0), 1.0))


def inner_floor_z(p: HullParams, x: float, wall: float) -> float:
    return keel_z(p, x) + wall


def build_layout(p: HullParams, motor: dict, battery: dict, wall: float = C.WALL_MM) -> dict:
    L = p.L
    checks: list[dict] = []
    cx0, cx1 = cavity_range(p, wall)
    lay: dict = {"wall_mm": wall, "cavity_x_mm": [cx0, cx1]}

    # ---- hatch + coaming ------------------------------------------------
    hx0 = 0.30 * L
    hx1 = min(0.64 * L, cx1 - 30.0)
    xs = [hx0 + (hx1 - hx0) * k / 12 for k in range(13)]
    deck_inner_half = min(
        half_breadth_wl(p, x / L) * (1 + flare(p, x / L)) - wall * 2 for x in xs)
    h_half = deck_inner_half - COAMING_WALL_MM - 3.0
    deck_top = max(deck_height(p, x / L) for x in xs)
    coaming_top = deck_top + COAMING_HEIGHT_MM
    lay["hatch"] = {"x0": hx0, "x1": hx1, "half_w": h_half,
                    "coaming_wall": COAMING_WALL_MM, "coaming_top_z": coaming_top,
                    "groove_w": GASKET_GROOVE_W_MM, "groove_d": GASKET_GROOVE_D_MM,
                    "lid_plate": LID_PLATE_MM, "lid_skirt": LID_SKIRT_MM,
                    "lid_clearance": LID_CLEARANCE_MM,
                    "magnet_dia": MAGNET_DIA_MM, "magnet_h": MAGNET_H_MM,
                    "magnet_pad": MAGNET_PAD_MM, "magnet_pad_depth": MAGNET_PAD_DEPTH_MM}
    checks.append(_chk("hatch.size", (hx1 - hx0) >= 80 and 2 * h_half >= 40,
                       f"opening {hx1 - hx0:.0f} x {2 * h_half:.0f} mm (min 80 x 40)"))

    # ---- rudder -----------------------------------------------------------
    # Area 2.5 % of L*T: full-size single-screw practice is ~1.5-2 % (DNV
    # guidance); a model at low Reynolds number needs more for the same turn.
    area = RUDDER_AREA_FRAC * L * p.T
    span = max(25.0, 1.0 * p.T)
    chord = min(max(area / span, 20.0), 0.12 * L)
    x_r = max(0.75 * chord + 4.0, 0.06 * L)
    z_hull_r = keel_z(p, x_r)
    tube_bore = C.RUDDER_TUBE_OD_MM + C.TUBE_BORE_FIT_MM
    tube_top = min(deck_height(p, x_r / L) - wall - 2.0, max(12.0, 0.6 * p.F))
    lay["rudder"] = {"x": x_r, "chord": chord, "span": span, "area_mm2": chord * span,
                     "z_root": z_hull_r - 1.0, "z_tip": z_hull_r - 1.0 - span,
                     "stock_dia": C.RUDDER_STOCK_MM, "tube_bore": tube_bore,
                     "boss_od": tube_bore + 2 * BOSS_WALL_MM, "tube_top_z": tube_top}
    checks.append(_chk("rudder_tube.above_waterline", tube_top >= 8.0,
                       f"rudder tube top {tube_top:.1f} mm above the waterline (min 8), "
                       "so the tube cannot flood even without a seal"))

    # ---- propeller + shaft line --------------------------------------------
    dp = min(max(0.045 * L, 25.0), 45.0)
    x_p = x_r + 0.25 * chord + 10.0
    z_p = keel_z(p, x_p) - 0.15 * dp - dp / 2
    th = math.radians(SHAFT_ANGLE_DEG)
    zline = lambda x: z_p + (x - x_p) * math.tan(th)
    x_exit = x_p
    while x_exit < cx1 and zline(x_exit) < keel_z(p, x_exit):
        x_exit += 0.5
    tube_len = min(max(0.15 * L, 60.0), 150.0)
    x_tube_end = x_exit + tube_len * math.cos(th)
    boss_od = C.STUFFING_TUBE_OD_MM + C.TUBE_BORE_FIT_MM + 2 * BOSS_WALL_MM
    lay["prop"] = {"dia": dp, "x": x_p, "z": z_p, "z_bottom": z_p - dp / 2}
    lay["shaft"] = {"angle_deg": SHAFT_ANGLE_DEG, "x_exit": x_exit,
                    "z_exit": zline(x_exit), "x_tube_end": x_tube_end,
                    "z_tube_end": zline(x_tube_end),
                    "tube_bore": C.STUFFING_TUBE_OD_MM + C.TUBE_BORE_FIT_MM,
                    "boss_od": boss_od, "tube_len": tube_len}
    end_clear = zline(x_tube_end) - boss_od / 2 - inner_floor_z(p, x_tube_end, wall)
    checks.append(_chk("stuffing_tube.inside", x_exit < cx1 and end_clear >= -boss_od / 2,
                       f"tube exits the hull at x = {x_exit:.0f} mm and ends "
                       f"{zline(x_tube_end):.1f} mm (z) inside the cavity"))

    # ---- motor + mount -------------------------------------------------------
    x_face = x_tube_end + COUPLER_GAP_MM * math.cos(th)
    z_axis = zline(x_face)
    can_r = motor["can_dia_mm"] / 2
    can_len = {24.0: 30.0, 28.0: 38.0, 36.0: 50.0}.get(motor["can_dia_mm"], 40.0)
    floor_at_motor = max(inner_floor_z(p, x_face + k, wall) for k in (0, can_len / 2, can_len))
    motor_clear = (z_axis - can_r) - floor_at_motor
    half_w_motor = inner_half_breadth_at(p, x_face / L, z_axis, wall)
    lay["motor"] = {"name": motor["name"], "x_face": x_face, "z_axis": z_axis,
                    "can_r": can_r, "can_len": can_len,
                    "screw_spacing": motor["screw_spacing_mm"],
                    "plate_t": MOUNT_PLATE_MM,
                    "cg": (x_face + can_len / 2 * math.cos(th), z_axis + can_len / 2 * math.sin(th))}
    checks.append(_chk("motor_mount.fits",
                       motor_clear >= 1.0 and half_w_motor >= can_r + 2.0
                       and x_face + can_len < cx1,
                       f"motor can clears the cavity floor by {motor_clear:.1f} mm; "
                       f"cavity half-width at the axis {half_w_motor:.1f} mm vs can {can_r:.1f}"))

    # ---- servo tray ----------------------------------------------------------
    # A thwart: a 4 mm shelf bonded to both hull sides. The SG90 drops through a
    # rectangular hole and hangs from its ears; its case needs clear space below.
    x_s = x_r + 40.0
    z_top = deck_height(p, x_s / L) - wall - (SG90["total_h"] - SG90["ear_z"]) - 8.0
    pocket_x = SG90["case_wid"] + 2 * SERVO_CLEARANCE_MM
    pocket_y = SG90["case_len"] + 2 * SERVO_CLEARANCE_MM
    hang = SG90["ear_z"] + SERVO_CLEARANCE_MM          # case below the ear underside
    tray_x = pocket_x + 8.0
    floor = max(inner_floor_z(p, x_s + d, wall) for d in (-tray_x / 2, 0, tray_x / 2))
    half_w = inner_half_breadth_at(p, x_s / L, z_top, wall)
    shaft_z = zline(x_s) + boss_od / 2 if x_s > x_exit else -1e9
    lay["servo"] = {"x": x_s, "z_top": z_top, "thwart_t": 4.0, "half_w": half_w, "pocket_x": pocket_x,
                    "pocket_y": pocket_y, "hang": hang, "tray_x": tray_x,
                    "cg": (x_s, z_top - 8.0)}
    below = z_top - hang - max(floor, shaft_z)
    checks.append(_chk("servo_tray.fits",
                       below >= 1.0 and half_w >= SG90["ear_span"] / 2 + 3.0,
                       f"SG90 case clears the hull floor / shaft by {below:.1f} mm (min 1.0); "
                       f"cavity half-width at the thwart {half_w:.1f} mm vs ears {SG90['ear_span'] / 2:.1f}"))

    # ---- battery, electronics ------------------------------------------------
    xb = min(max(0.47 * L, hx0 + battery["len_mm"] / 2), hx1 - battery["len_mm"] / 2)
    need = battery["wid_mm"] / 2 + 1.0
    # lowest height where the cavity is wide enough for the pack (it sits on a cradle)
    zb = None
    ci = inner_side_curve(p, xb / L, wall)
    for (y, z) in ci:
        if y >= need:
            zb = z
            break
    fits = zb is not None and zb + battery["h_mm"] < deck_height(p, xb / L) - wall - 2
    zb = zb if zb is not None else 0.0
    lay["battery"] = {"name": battery["name"], "x": xb, "z_bottom": zb,
                      "cg": (xb, zb + battery["h_mm"] / 2), "len": battery["len_mm"],
                      "wid": battery["wid_mm"], "h": battery["h_mm"]}
    checks.append(_chk("battery.fits",
                       fits and battery["len_mm"] <= (hx1 - hx0),
                       f"{battery['name']} under the hatch, bottom at z = {zb:.1f} mm"))
    xe = min(xb + battery["len_mm"] / 2 + 25.0, hx1)
    lay["electronics"] = {"cg": (xe, inner_floor_z(p, xe, wall) + 12.0)}
    lay["checks"] = checks
    return lay

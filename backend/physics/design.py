"""Solve one displacement-monohull design from a request and a character.

The coupled problem: the hull's draft sets its displacement; the displacement
must equal the mass of the printed hull + hardware + battery + ballast; the
battery depends on the power, which depends on the resistance, which depends
on the hull. Iterate until the draft, the parts and the ballast agree, then
check stability, speed, endurance and fit, each as a named check.
"""
from __future__ import annotations

import math
import uuid

from . import components as C
from .hullform import HullParams, deck_height, flare, half_breadth_wl, section_polygon
from .hydrostatics import (clip_below, downflooding_angle, gz_curve, poly_area_centroid,
                           sample, submerged, waterplane)
from .layout import build_layout, inner_floor_z
from .resistance import RHO_FRESH, HullCoefficients, holtrop, hull_speed
from .weights import mass_budget, solve_ballast

RHO_G_PER_MM3 = RHO_FRESH * 1e-6   # g/mm^3

# Stability criteria for a free-running model. These are MariForge's own
# numbers, chosen to be conservative for a boat that will meet wakes and
# gusts at small scale; they are not a class rule.
GM_MIN_FRAC_B = 0.04          # GM >= 4 % of beam
RANGE_MIN_DEG = 70.0          # GZ positive to at least 70 deg
DOWNFLOOD_MIN_DEG = 35.0      # coaming top edge stays dry to 35 deg heel
BALLAST_MIN_FRAC = 0.05       # keep >= 5 % of displacement as trimming ballast
# Ballast above that is not waste: it is the payload the hull can carry
# (swap lead for cargo, cameras, sensors) and is reported as such.


def hull_coefficients(p: HullParams, n: int = 121) -> tuple[HullCoefficients, dict]:
    samples = sample(lambda x: section_polygon(p, x / p.L), 0.0, p.L, n)
    sb = submerged(samples, 0.0, p.L)
    wp = waterplane(samples, 0.0, p.L)
    mid = poly_area_centroid(clip_below(section_polygon(p, p.x_max), 0.0))[0]
    at = poly_area_centroid(clip_below(section_polygon(p, 0.0), 0.0))[0]
    hc = HullCoefficients(
        L=p.L / 1000, B=p.B / 1000, T=p.T / 1000, volume=sb.volume * 1e-9,
        S=sb.wetted * 1e-6, Cm=mid / (p.B * p.T), Cwp=wp["area"] / (p.L * p.B),
        lcb_pct=(sb.lcb - 0.5 * p.L) / p.L * 100, AT=at * 1e-6)
    return hc, {"sub": sb, "wp": wp}


def _pick(table: list[dict], key: str, need: float) -> dict | None:
    for row in table:
        if row[key] >= need:
            return row
    return None


def solve_design(req: dict, character: dict) -> dict:
    Lbox, Bbox, Hbox = req["length_mm"], req["beam_mm"], req["height_mm"]
    V = req["cruise_speed_mps"]
    endurance_min = req["endurance_min"]
    wall = C.WALL_MM

    L = Lbox
    shape = {k: character[k] for k in ("n", "x_max", "bow_exp", "stern_exp",
                                       "transom_b", "flare_mid", "flare_bow")}
    B = L / character["L_over_B"]
    # freeboard: ~L/13 for a scale cruiser, and never below 0.28 B so beamy
    # hulls keep their deck edge dry when heeled
    F = min(max(0.075 * L, 0.28 * L / character["L_over_B"], 25.0), 100.0)
    T = B / character["B_over_T"]

    motor, battery = C.MOTORS[0], C.BATTERIES[0]
    notes: list[str] = []
    for _ in range(30):
        # deck may not exceed the box beam anywhere (flare widens the bow deck)
        p = HullParams(L=L, B=B, T=T, F=F, **shape)
        deck_max = max(2 * half_breadth_wl(p, k / 40) * (1 + flare(p, k / 40)) for k in range(41))
        if deck_max > Bbox:
            B *= Bbox / deck_max * 0.999
            continue
        hc, hy = hull_coefficients(p, n=61)
        disp_g = hy["sub"].volume * RHO_G_PER_MM3
        lay = build_layout(p, motor, battery, wall)
        bud = mass_budget(p, lay, motor, battery)
        frac = (disp_g - bud["mass_g"]) / disp_g
        # power -> motor + battery
        r = holtrop(hc, V)
        p_elec = r["P_effective_W"] / (C.ETA_PROPULSIVE * motor["eta"])
        new_motor = _pick(C.MOTORS, "max_W", 3.0 * p_elec) or C.MOTORS[-1]
        e_need = p_elec * endurance_min / 60 / C.BATTERY_USABLE
        new_batt = _pick(C.BATTERIES, "Wh", e_need) or C.BATTERIES[-1]
        changed = False
        if new_motor is not motor or new_batt is not battery:
            motor, battery = new_motor, new_batt
            changed = True
        if frac < BALLAST_MIN_FRAC and T < 0.6 * B:
            T *= 1.04; changed = True
        if not changed:
            break

    p = HullParams(L=L, B=B, T=T, F=F, **shape)
    hc, hy = hull_coefficients(p)
    sb, wp = hy["sub"], hy["wp"]
    disp_g = sb.volume * RHO_G_PER_MM3
    lay = build_layout(p, motor, battery, wall)
    bud = mass_budget(p, lay, motor, battery)
    cx0, cx1 = lay["cavity_x_mm"]
    bal = solve_ballast(bud, disp_g, sb.lcb, (cx0 + 20, cx1 - 20),
                        lambda x: inner_floor_z(p, x, wall))
    kg = bal["cg"][1]
    bm = wp["I_T"] / sb.volume
    gm = sb.vcb + bm - kg

    sec = sample(lambda x: section_polygon(p, x / p.L), 0.0, p.L, 81)
    gz = gz_curve(sec, 0.0, p.L, sb.volume, kg, angles_deg=range(0, 91, 5),
                  z_bounds=(-3 * T, 3 * F))
    rng = 90.0
    for a, b in zip(gz, gz[1:]):
        if a["gz_mm"] > 0 and b["gz_mm"] <= 0:
            rng = a["heel_deg"] + 5 * a["gz_mm"] / (a["gz_mm"] - b["gz_mm"])
            break
    hz = lay["hatch"]
    corner = (hz["half_w"] + hz["coaming_wall"], hz["coaming_top_z"])
    dfa = downflooding_angle(corner, sec, 0.0, p.L, sb.volume)

    res = holtrop(hc, V)
    p_elec = res["P_effective_W"] / (C.ETA_PROPULSIVE * motor["eta"])
    endurance = battery["Wh"] * C.BATTERY_USABLE / max(p_elec, 1e-6) * 60
    v_hull = hull_speed(hc.L)
    curve = [holtrop(hc, v) for v in [v_hull * k / 12 for k in range(1, 15)]]

    overall_h = max(-lay["prop"]["z_bottom"], -lay["rudder"]["z_tip"], T) \
        + hz["coaming_top_z"] + hz["lid_plate"]
    deck_max = max(2 * half_breadth_wl(p, k / 40) * (1 + flare(p, k / 40)) for k in range(41))

    checks = list(lay["checks"])
    ck = lambda n, ok, d: checks.append({"name": n, "ok": bool(ok), "detail": d})
    ck("ballast.positive", bal["mass_g"] >= BALLAST_MIN_FRAC * disp_g,
       f"ballast {bal['mass_g']:.0f} g = {bal['mass_g'] / disp_g * 100:.0f} % of displacement "
       f"(min {BALLAST_MIN_FRAC * 100:.0f} %)")
    ck("trim.level", bal["reached"],
       "ballast can bring LCG onto LCB" if bal["reached"] else
       f"ballast would need x = {bal.get('x_wanted') or 0:.0f} mm, outside the cavity")
    ck("stability.gm", gm >= GM_MIN_FRAC_B * B,
       f"GM {gm:.1f} mm = {gm / B * 100:.1f} % of beam (min {GM_MIN_FRAC_B * 100:.0f} %)")
    ck("stability.range", rng >= RANGE_MIN_DEG,
       f"GZ positive to {rng:.0f} deg (min {RANGE_MIN_DEG:.0f}), fixed trim")
    ck("stability.downflooding", dfa >= DOWNFLOOD_MIN_DEG,
       f"coaming edge reaches the water at {dfa:.0f} deg heel (min {DOWNFLOOD_MIN_DEG:.0f})")
    ck("speed.froude", res["Fn"] <= 0.40,
       f"cruise Fn {res['Fn']:.2f}; displacement hull speed (Fn 0.40) is {v_hull:.2f} m/s")
    ck("resistance.method_range", res["in_range"], "Holtrop-Mennen valid to Fn 0.45")
    ck("endurance", endurance >= endurance_min,
       f"{endurance:.0f} min at cruise on a {battery['name']} (asked {endurance_min:.0f})")
    ck("motor.headroom", motor["max_W"] >= 2.0 * p_elec,
       f"{motor['name']} max {motor['max_W']:.0f} W vs {p_elec:.1f} W at cruise")
    ck("box.height", overall_h <= Hbox,
       f"keel/prop/rudder to hatch lid {overall_h:.0f} mm (box {Hbox:.0f})")
    ck("box.beam", deck_max <= Bbox + 0.5, f"max deck beam {deck_max:.0f} mm (box {Bbox:.0f})")

    return {
        "id": uuid.uuid4().hex[:12],
        "hull_type": "displacement_monohull",
        "character": character["key"], "label": character["label"],
        "blurb": character["blurb"],
        "request": req,
        "params": p.to_dict(),
        "layout": lay,
        "hydrostatics": {
            "displacement_g": disp_g, "volume_mm3": sb.volume,
            "lcb_mm": sb.lcb, "kb_mm": sb.vcb, "bm_mm": bm, "kg_mm": kg, "gm_mm": gm,
            "lcg_mm": bal["cg"][0], "wetted_mm2": sb.wetted,
            "waterplane_mm2": wp["area"], "Cb": hc.Cb, "Cp": hc.Cp, "Cm": hc.Cm,
            "Cwp": hc.Cwp, "lcb_pct": hc.lcb_pct, "gz": gz,
            "range_deg": rng, "downflooding_deg": dfa,
            "note": "Fixed-trim GZ. z = 0 is the design waterline; KG etc are z values.",
        },
        "mass": {"items": bud["items"], "without_ballast_g": bud["mass_g"],
                 "ballast": bal, "total_g": disp_g,
                 "printed_shell_g": bud["shell"]["mass_g"]},
        "resistance": {"cruise": res, "curve": curve, "hull_speed_mps": v_hull,
                       "assumptions": "ITTC-57 friction, Holtrop-Mennen 1982/84, C_A = 0, "
                                      "fresh water 15 C, no turbulence stimulation"},
        "propulsion": {"motor": motor, "battery": battery, "prop_dia_mm": lay["prop"]["dia"],
                       "P_electric_W": p_elec, "eta_propulsive": C.ETA_PROPULSIVE,
                       "endurance_min": endurance},
        "overall": {"length_mm": L, "beam_mm": deck_max, "height_mm": overall_h},
        "checks": checks,
        "feasible": all(c["ok"] for c in checks),
    }

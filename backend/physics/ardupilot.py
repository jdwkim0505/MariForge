"""ArduPilot Rover (boat frame) starting parameters from the design.

Mission Planner .param format (NAME,VALUE, '#' comments). These are starting
values computed from the design, not a tune: fly a Learn Cruise (RCx_OPTION =
50) and the speed/steering autotune steps in the ArduPilot Rover tuning guide
afterwards.

  CRUISE_SPEED      the design cruise speed
  CRUISE_THROTTLE   fixed-pitch prop: power ~ n^3 and boat speed ~ n, and a
                    brushed ESC sets n ~ throttle, so throttle ~ (P / P_full)^(1/3)
                    with P_full the motor's rated power (prop sized to load it)
  TURN_RADIUS       tactical diameter ~3.5 L for a single balanced rudder
                    displacement hull (full-size practice is 3-5 L)
  ATC_STR_RAT_MAX   V / R at cruise
  ATC_ACCEL_MAX     (T_full - R) / m at cruise, T ~ P^(2/3) at a fixed advance
  ATC_DECEL_MAX     coasting: R / m at cruise (no reverse thrust counted)
"""
from __future__ import annotations

import math

TURN_DIAMETER_PER_L = 3.5
MIN_ACCEL = 0.05   # m/s^2: below this the speed controller ramps uselessly slowly


def rover_params(design: dict) -> dict:
    pr = design["propulsion"]
    r = design["resistance"]["cruise"]
    V = r["V_mps"]
    m = design["mass"]["total_g"] / 1000
    L = design["params"]["L"] / 1000
    p_full = pr["motor"]["max_W"]
    p_c = pr["P_electric_W"]
    thr = 100 * min((p_c / p_full) ** (1 / 3), 1.0)
    R = r["R_total_N"]
    turn_r = TURN_DIAMETER_PER_L * L / 2
    accel = max(R * ((p_full / p_c) ** (2 / 3) - 1) / m, MIN_ACCEL)
    decel = max(R / m, MIN_ACCEL)
    cap_mah = pr["battery"]["mAh"]
    rows = [
        ("FRAME_CLASS", 2, "boat"),
        ("SERVO1_FUNCTION", 26, "ground steering: rudder servo on output 1"),
        ("SERVO3_FUNCTION", 70, "throttle: ESC on output 3"),
        ("CRUISE_SPEED", round(V, 2), "m/s, design cruise"),
        ("CRUISE_THROTTLE", round(thr), f"%, from {p_c:.1f} W of {p_full:.0f} W rated"),
        ("WP_SPEED", round(V, 2), "m/s"),
        ("WP_RADIUS", round(max(turn_r, 1.0), 1), "m, one turn radius"),
        ("TURN_RADIUS", round(turn_r, 2), f"m, {TURN_DIAMETER_PER_L} L tactical diameter / 2"),
        ("LOIT_RADIUS", round(max(2 * turn_r, 2.0), 1), "m"),
        ("ATC_STR_RAT_MAX", round(math.degrees(V / turn_r)), "deg/s at cruise"),
        ("ATC_ACCEL_MAX", round(accel, 2), "m/s/s"),
        ("ATC_DECEL_MAX", round(decel, 2), "m/s/s, coasting"),
        ("BATT_CAPACITY", cap_mah, f"mAh, {pr['battery']['name']}"),
    ]
    lines = [f"# MariForge {design['label']} ({design['id']}): ArduPilot Rover boat, starting values",
             f"# L {design['params']['L']:.0f} mm, {design['mass']['total_g']:.0f} g, "
             f"cruise {V:.2f} m/s at {R:.3f} N",
             "# Computed from the design, not tuned. Next: Learn Cruise (RCx_OPTION 50), then",
             "# the Rover speed and steering tuning steps. Check servo and ESC outputs before arming."]
    for name, val, note in rows:
        lines.append(f"# {note}")
        lines.append(f"{name},{val}")
    return {"params": [{"name": n, "value": v, "note": c} for n, v, c in rows],
            "text": "\n".join(lines) + "\n"}

"""Off-the-shelf parts a scale cruiser is built around, and print material.

Masses are typical catalogue values for the class of part, not a specific
SKU; each row says what it stands for. Change a row here and every design
re-solves with it.
"""
from __future__ import annotations

PLA_DENSITY = 1.24e-3   # g/mm^3 (1.24 g/cm^3, PLA datasheet typical)

# Hull wall: four 0.4 mm perimeters. Fewer than three perimeters is the usual
# cause of weeping FDM hulls; four leaves margin for a slightly wet layer seam.
WALL_MM = 1.6

# Brushed can motors are what scale cruisers actually run (quiet, cheap,
# gentle at low rpm). eta is a small-motor figure at its efficient load.
MOTORS = [
    {"name": "280-size brushed", "mass_g": 40.0, "max_W": 25.0, "eta": 0.55,
     "can_dia_mm": 24.0, "screw_spacing_mm": 16.0, "shaft_mm": 2.0},
    {"name": "380-size brushed", "mass_g": 60.0, "max_W": 50.0, "eta": 0.60,
     "can_dia_mm": 28.0, "screw_spacing_mm": 16.0, "shaft_mm": 2.3},
    {"name": "540-size brushed", "mass_g": 160.0, "max_W": 120.0, "eta": 0.65,
     "can_dia_mm": 36.0, "screw_spacing_mm": 25.0, "shaft_mm": 3.17},
]

# 2S LiPo packs, 7.4 V nominal. usable = fraction drawn before the 3.7 V/cell floor.
BATTERIES = [
    {"name": "2S 1000 mAh LiPo", "mass_g": 60.0, "Wh": 7.4, "len_mm": 70, "wid_mm": 34, "h_mm": 14},
    {"name": "2S 2200 mAh LiPo", "mass_g": 125.0, "Wh": 16.3, "len_mm": 105, "wid_mm": 34, "h_mm": 18},
    {"name": "2S 3300 mAh LiPo", "mass_g": 185.0, "Wh": 24.4, "len_mm": 138, "wid_mm": 44, "h_mm": 18},
    {"name": "2S 5000 mAh LiPo", "mass_g": 270.0, "Wh": 37.0, "len_mm": 138, "wid_mm": 46, "h_mm": 25},
]
BATTERY_USABLE = 0.80

# Propulsive efficiency eta_D = eta_0 * eta_H * eta_R for a small RC prop at
# low Reynolds number; full-size ships reach 0.6-0.7, model props are well
# below that. 0.45 is the assumption; it is shown in every design.
ETA_PROPULSIVE = 0.45

FIXED_PARTS = {
    "servo_SG90": 9.0,            # rudder servo
    "receiver": 8.0,
    "esc_brushed": 18.0,
    "shaft_prop_coupler": 22.0,   # 2 mm stainless shaft, brass tube, prop, coupler
    "rudder_stock_tiller": 6.0,   # 3 mm rod, tiller arm, pushrod
    "wiring": 12.0,
    "hatch_magnets": 4 * 1.5,     # four 8 x 3 mm N52 discs
    "gasket": 3.0,
}

# Rudder stock 3 mm rod in a 4 mm OD brass tube: the standard RC scale pairing.
RUDDER_TUBE_OD_MM = 4.0
RUDDER_STOCK_MM = 3.0
# Stuffing tube: 3/16" (4.76 mm) OD brass tube carrying a 2 mm shaft (bore 4.96).
STUFFING_TUBE_OD_MM = 4.76
# Press fit for brass tube in a printed bore: tube OD + 0.2 mm, bored after
# printing to size if needed. Epoxied in either way.
TUBE_BORE_FIT_MM = 0.20

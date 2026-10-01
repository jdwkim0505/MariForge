"""Bill of materials: what to buy, roughly what it costs, and how long it prints.

Prices are typical hobby-retail figures in USD for the class of part (one
row per catalogue row in components.py), not quotes. They are labelled as
estimates everywhere they are shown. Print time is a range between a stock
bed-slinger and a fast CoreXY printer at the 1.6 mm wall, from the printed
mass of the design.
"""
from __future__ import annotations

PRICE_USD = {
    # propulsion and control
    "280-size brushed": 6.0, "380-size brushed": 8.0, "540-size brushed": 12.0,
    "2S 1000 mAh LiPo": 12.0, "2S 2200 mAh LiPo": 20.0,
    "2S 3300 mAh LiPo": 28.0, "2S 5000 mAh LiPo": 40.0,
    "esc_brushed": 15.0, "receiver": 15.0, "servo_SG90": 3.0,
    "shaft_prop_coupler": 14.0, "rudder_stock_tiller": 5.0, "wiring": 6.0,
    "hatch_magnets": 2.0, "gasket": 3.0,
    # payload classes
    "autopilot": 150.0, "telemetry": 40.0, "fpv_camera": 45.0, "action_camera": 150.0,
    "companion_computer": 80.0, "echo_sounder": 350.0, "water_sampler": 250.0,
    "medical_kit": 50.0, "radio_relay": 60.0,
}
PLA_USD_PER_KG = 20.0
LEAD_USD_PER_KG = 10.0          # fishing weights / lead shot
EPOXY_USD = 10.0                 # tubes, split joints, ballast
PRINT_G_PER_H = (15.0, 40.0)    # slow and fast printer, effective throughput

NOTES = {
    "esc_brushed": "brushed ESC with reverse, 2S, BEC",
    "receiver": "any 2.4 GHz receiver (not needed if an autopilot drives the servos)",
    "shaft_prop_coupler": "2 mm stainless shaft in 3/16\" brass tube, prop, coupler",
    "rudder_stock_tiller": "3 mm rod, 4 mm OD brass tube, tiller arm, pushrod",
    "hatch_magnets": "four 8 x 3 mm N52 discs",
    "gasket": "1.5 mm foam or silicone cord",
}


def bill_of_materials(design: dict) -> dict:
    rows = []

    def add(item, qty, unit, note="", group="hardware", qty_unit="", priced=True):
        rows.append({"item": item, "qty": qty, "qty_unit": qty_unit, "unit_usd": unit,
                     "usd": qty * unit, "priced": priced, "note": note, "group": group})

    pr = design["propulsion"]
    add(pr["motor"]["name"] + " motor", 1, PRICE_USD[pr["motor"]["name"]], "", "propulsion")
    add(pr["battery"]["name"], 1, PRICE_USD[pr["battery"]["name"]], "", "propulsion")
    keys = {it.get("key") for it in design["payload"]["items"]}
    for k in ("esc_brushed", "servo_SG90", "receiver", "shaft_prop_coupler",
              "rudder_stock_tiller", "wiring", "hatch_magnets", "gasket"):
        if k == "receiver" and "autopilot" in keys:
            note = "still needed for manual control alongside the autopilot"
        else:
            note = NOTES.get(k, "")
        add(k.replace("_", " "), 1, PRICE_USD[k], note,
            "propulsion" if k in ("esc_brushed", "shaft_prop_coupler") else "hardware")
    for it in design["payload"]["items"]:
        add(it["name"], 1, PRICE_USD.get(it["key"], 0.0),
            "your own item, not priced" if it["key"] not in PRICE_USD else "", "payload",
            priced=it["key"] in PRICE_USD)
    printed = sum(i["mass_g"] for i in design["mass"]["items"]
                  if i["name"] in ("hull_shell", "coaming", "magnet_pads", "hatch_lid",
                                   "deck_opening", "rudder_boss", "rudder_blade",
                                   "stuffing_boss", "motor_mount", "servo_tray")
                  or i["name"].startswith("well boss"))
    # filament bought: printed mass plus ~10 % for purge, supports and failed layers
    add("PLA filament", round(printed * 1.10 / 1000, 2), PLA_USD_PER_KG, "incl. ~10 % waste", "materials", "kg")
    lead = max(design["mass"]["ballast"]["mass_g"], 0.0)
    add("lead ballast", round(lead / 1000, 2), LEAD_USD_PER_KG,
        f"at x = {design['mass']['ballast']['x'] or 0:.0f} mm", "materials", "kg")
    add("epoxy", 1, EPOXY_USD, "tubes, split joints, ballast", "materials")
    total = sum(r["usd"] for r in rows)
    return {"rows": rows, "total_usd": total,
            "by_group": {g: sum(r["usd"] for r in rows if r["group"] == g)
                         for g in ("propulsion", "hardware", "payload", "materials")},
            "printed_g": printed,
            "print_hours": (printed / PRINT_G_PER_H[1], printed / PRINT_G_PER_H[0]),
            "note": "Estimated prices, typical hobby retail in USD, not quotes. "
                    "Print time is a range from a fast CoreXY to a stock bed-slinger."}

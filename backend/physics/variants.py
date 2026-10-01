"""One request -> five genuinely different displacement hulls.

Each character is a row of hull-form coefficients in the range real small
craft use: L/B from 3 (harbour tug) to 5.2 (slender launch), section
fullness from a slack round bilge to a near-box tug section.
"""
from __future__ import annotations

from ..jsonsafe import plain
from .bom import bill_of_materials
from .design import solve_design

CHARACTERS = [
    {"key": "launch", "label": "Launch", "blurb": "Slender and easily driven; the longest run per charge.",
     "L_over_B": 5.2, "B_over_T": 3.6, "n": 2.2, "x_max": 0.55, "bow_exp": 1.6,
     "stern_exp": 2.0, "transom_b": 0.60, "flare_mid": 0.03, "flare_bow": 0.25},
    {"key": "cruiser", "label": "Cruiser", "blurb": "The balanced default: moderate beam, round bilge.",
     "L_over_B": 4.2, "B_over_T": 3.4, "n": 2.6, "x_max": 0.55, "bow_exp": 1.9,
     "stern_exp": 2.4, "transom_b": 0.72, "flare_mid": 0.04, "flare_bow": 0.30},
    {"key": "trawler", "label": "Trawler", "blurb": "Beamy and stiff, high flared bow for rough water.",
     "L_over_B": 3.5, "B_over_T": 3.2, "n": 3.0, "x_max": 0.52, "bow_exp": 2.1,
     "stern_exp": 2.6, "transom_b": 0.78, "flare_mid": 0.06, "flare_bow": 0.40},
    {"key": "tug", "label": "Tug", "blurb": "Full, deep and heavy; the most payload and pulling power.",
     "L_over_B": 3.0, "B_over_T": 2.8, "n": 3.4, "x_max": 0.50, "bow_exp": 2.4,
     "stern_exp": 2.8, "transom_b": 0.82, "flare_mid": 0.05, "flare_bow": 0.30},
    {"key": "patrol", "label": "Patrol", "blurb": "Long and fine-entried; highest hull speed for the box.",
     "L_over_B": 4.7, "B_over_T": 3.8, "n": 2.8, "x_max": 0.58, "bow_exp": 1.5,
     "stern_exp": 2.2, "transom_b": 0.80, "flare_mid": 0.05, "flare_bow": 0.35},
]

MISSIONS = {"scale_cruiser": {"label": "Scale cruiser", "default_speed_frac_hull": 0.8}}


def _traits(designs: list[dict]) -> None:
    """Normalised 0..1 traits across the set, for the gallery bars."""
    def col(f):
        v = [f(d) for d in designs]
        lo, hi = min(v), max(v)
        return [(x - lo) / (hi - lo) if hi > lo else 0.5 for x in v]
    stab = col(lambda d: d["hydrostatics"]["gm_mm"] / d["params"]["B"])
    econ = col(lambda d: d["propulsion"]["endurance_min"])
    spd = col(lambda d: d["resistance"]["hull_speed_mps"])
    load = col(lambda d: d["payload"]["spare_capacity_g"])
    sea = col(lambda d: d["seakeeping"]["max_hs_mm"])
    for i, d in enumerate(designs):
        d["traits"] = {"stability": stab[i], "endurance": econ[i], "hull_speed": spd[i],
                       "payload": load[i], "seaworthiness": sea[i]}


def generate_variants(req: dict) -> list[dict]:
    designs = [plain(solve_design(req, ch)) for ch in CHARACTERS]
    for d in designs:
        d["bom"] = plain(bill_of_materials(d))
    _traits(designs)
    return designs

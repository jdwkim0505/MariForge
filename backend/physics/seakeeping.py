"""Seakeeping: how much wave the hull can take before it stops being useful.

Head seas, heave and pitch coupled, strip theory in its simplest honest form:
  - added mass per section from Lewis forms (Lewis 1929; coefficient as in
    Journee & Massie, Offshore Hydromechanics, 2001, sec. 6.3),
  - restoring from the waterplane,
  - Froude-Krylov excitation with the Smith correction exp(-k d*), d* the
    section's mean draft (area / waterline breadth),
  - encounter frequency w_e = w + k V (deep water, k = w^2 / g),
  - linear damping at an assumed fraction of critical at the heave resonance,
    proportional to the restoring (no radiation solve).
Beam seas, roll: single-DOF, excited by the effective wave slope: the slope
averaged across the waterline beam (sin(kB/2)/(kB/2)) and attenuated to the
centre of buoyancy (exp(-k KB), Smith), radius of gyration 0.40 B plus 20 % added inertia, assumed damping.

The responses are combined with a two-parameter Bretschneider spectrum whose
peak period comes from a fixed significant steepness (young, fetch-limited
wind sea: the chop on a lake or a harbour, which is where models run).
Operability criteria are the NORDFORSK 1987 general-operability limits for
merchant ships, applied unchanged at model scale, plus a hatch criterion of
MariForge's own: green water must not reach the coaming top.

Everything here is a linear estimate. It ranks hulls and gives the right
order of magnitude for the wave height a model can work in; it is not a tank
test.
"""
from __future__ import annotations

import math

import numpy as np

from .hullform import HullParams, canoe_depth, deck_height, half_breadth_wl, section_polygon
from .hydrostatics import clip_below, poly_area_centroid
from .resistance import G, RHO_FRESH

ZETA_HEAVE = 0.20       # fraction of critical damping at heave resonance (assumed)
ZETA_ROLL = 0.08        # round bilge, no bilge keels (assumed)
K_XX_FRAC_B = 0.40      # roll radius of gyration / beam
K_YY_FRAC_L = 0.25      # pitch radius of gyration / length
ROLL_ADDED_INERTIA = 0.20
STEEPNESS = 1.0 / 20.0  # Hs / lambda_peak for a young wind sea
BOW_STATION = 0.90      # deck wetness and keel emergence are judged at 0.9 L

CRITERIA = {
    # NORDFORSK 1987, general operability, merchant ships
    "roll_rms_deg": 6.0,
    "deck_wetness_prob": 0.05,
    "keel_emergence_prob": 0.03,
    "bow_accel_rms_g": 0.275,
    # MariForge: water over the coaming top at the hatch's forward edge
    "hatch_wetness_prob": 0.01,
}
CRITERIA_LABELS = {
    "roll_rms_deg": "roll RMS in beam seas",
    "deck_wetness_prob": "green water on the foredeck",
    "keel_emergence_prob": "forefoot leaving the water (slamming)",
    "bow_accel_rms_g": "vertical acceleration at the bow",
    "hatch_wetness_prob": "water reaching the hatch coaming",
}


def lewis_coefficient(b: float, d: float, sigma: float) -> float:
    """Heave added-mass coefficient of a Lewis section: a33 = C rho pi b^2 / 2."""
    if b <= 0 or d <= 0:
        return 0.0
    H = b / d
    r = (H - 1) / (H + 1)
    c1 = (3 + 4 * sigma / math.pi) + (1 - 4 * sigma / math.pi) * r * r
    disc = 9 - 2 * c1
    if disc < 0:
        return 1.0
    a3 = (-c1 + 3 + math.sqrt(disc)) / c1
    a1 = (1 + a3) * r
    return ((1 + a1) ** 2 + 3 * a3 ** 2) / (1 + a1 + a3) ** 2


def bretschneider(w: np.ndarray, hs: float, tp: float) -> np.ndarray:
    wp = 2 * math.pi / tp
    return 5 / 16 * hs ** 2 * wp ** 4 / w ** 5 * np.exp(-1.25 * (wp / w) ** 4)


def peak_period(hs: float) -> float:
    """Tp from the significant steepness: lambda_p = Hs / s, lambda = g T^2 / 2 pi."""
    return math.sqrt(2 * math.pi * hs / STEEPNESS / G)


class Seakeeping:
    def __init__(self, p: HullParams, mass_kg: float, lcg_mm: float, gm_mm: float, kb_mm: float,
                 V: float, hatch_x_mm: float, hatch_top_mm: float, n: int = 61):
        self.p, self.V = p, V
        self.kb = max(-kb_mm, 0.0) / 1000      # depth of B below the waterline
        L = p.L / 1000
        xs = np.linspace(0.0, p.L, n)
        self.x = xs / 1000
        b = np.array([half_breadth_wl(p, x / p.L) for x in xs]) / 1000
        d = np.array([canoe_depth(p, x / p.L) for x in xs]) / 1000
        area = np.array([poly_area_centroid(clip_below(section_polygon(p, x / p.L), 0.0))[0]
                         for x in xs]) / 1e6
        sigma = np.clip(area / np.maximum(2 * b * d, 1e-12), 0.3, 1.0)
        c = np.array([lewis_coefficient(bi, di, si) for bi, di, si in zip(b, d, sigma)])
        self.b, self.dstar = b, area / np.maximum(2 * b, 1e-9)
        a33 = RHO_FRESH * math.pi * b ** 2 / 2 * c
        xi = self.x - lcg_mm / 1000
        self.xi = xi
        tr = np.trapezoid
        self.M = np.array([[mass_kg + tr(a33, self.x), tr(a33 * xi, self.x)],
                           [tr(a33 * xi, self.x), mass_kg * (K_YY_FRAC_L * L) ** 2 + tr(a33 * xi ** 2, self.x)]])
        rg2b = RHO_FRESH * G * 2 * b
        self.C = np.array([[tr(rg2b, self.x), tr(rg2b * xi, self.x)],
                           [tr(rg2b * xi, self.x), tr(rg2b * xi ** 2, self.x)]])
        self.T_heave = 2 * math.pi * math.sqrt(self.M[0, 0] / self.C[0, 0])
        # stiffness-proportional (Rayleigh) damping, set to ZETA_HEAVE at the
        # heave resonance: distributed along the hull like the restoring force,
        # so heave-pitch coupling stays consistent and long waves are followed
        self.Bd = ZETA_HEAVE * self.T_heave / math.pi * self.C
        self.T_pitch = 2 * math.pi * math.sqrt(self.M[1, 1] / self.C[1, 1])
        kxx2 = (1 + ROLL_ADDED_INERTIA) * (K_XX_FRAC_B * p.B / 1000) ** 2
        self.gm = gm_mm / 1000
        self.w_roll = math.sqrt(G * max(self.gm, 1e-6) / kxx2)
        self.T_roll = 2 * math.pi / self.w_roll
        xb = BOW_STATION * p.L
        self.bow = (xb / 1000, deck_height(p, BOW_STATION) / 1000, canoe_depth(p, BOW_STATION) / 1000)
        self.hatch = (hatch_x_mm / 1000, hatch_top_mm / 1000)
        self.w = np.linspace(0.8, 40.0, 500)   # rad/s: waves 4 cm to 96 m long
        self._raos()

    def _raos(self):
        tr = np.trapezoid
        w, x, xi = self.w, self.x, self.xi
        k = w ** 2 / G
        we = w + k * self.V
        z3 = np.zeros(len(w), complex); th = np.zeros(len(w), complex)
        for i in range(len(w)):
            # head seas: the wave runs from the stem (x = L) toward the transom
            f = RHO_FRESH * G * 2 * self.b * np.exp(-k[i] * self.dstar) * np.exp(1j * k[i] * x)
            F = np.array([tr(f, x), tr(f * xi, x)])
            A = -we[i] ** 2 * self.M + 1j * we[i] * self.Bd + self.C
            z3[i], th[i] = np.linalg.solve(A, F)
        self.k, self.we, self.z3, self.th = k, we, z3, th

        xg = float((x - xi)[0])

        def rel(xp):
            # wave elevation minus hull motion at station xp (per unit wave amplitude)
            return np.exp(1j * self.k * xp) - (self.z3 + (xp - xg) * self.th)

        self.rel_bow = np.abs(rel(self.bow[0]))
        self.rel_hatch = np.abs(rel(self.hatch[0]))
        self.acc_bow = self.we ** 2 * np.abs(self.z3 + (self.bow[0] - xg) * self.th)
        lam = w / self.w_roll
        kb2 = self.k * self.p.B / 2000
        gamma = np.abs(np.sinc(kb2 / math.pi)) * np.exp(-self.k * self.kb)
        self.roll_per_slope = gamma / np.sqrt((1 - lam ** 2) ** 2 + (2 * ZETA_ROLL * lam) ** 2)

    def responses(self, hs: float) -> dict:
        S = bretschneider(self.w, hs, peak_period(hs))
        tr = np.trapezoid
        m0 = lambda rao: float(tr(rao ** 2 * S, self.w))
        m_bow = m0(self.rel_bow)
        m_hatch = m0(self.rel_hatch)
        roll_rms = math.degrees(math.sqrt(m0(self.roll_per_slope * self.k)))
        acc = math.sqrt(m0(self.acc_bow)) / G
        _, f_bow, d_bow = self.bow
        f_hatch = self.hatch[1]
        p = lambda lim, m: math.exp(-lim ** 2 / (2 * m)) if m > 0 else 0.0
        return {"roll_rms_deg": roll_rms,
                "deck_wetness_prob": p(f_bow, m_bow),
                "keel_emergence_prob": p(d_bow, m_bow),
                "bow_accel_rms_g": acc,
                "hatch_wetness_prob": p(f_hatch, m_hatch)}

    def _ok(self, hs: float) -> tuple[bool, str | None]:
        r = self.responses(hs)
        for k, lim in CRITERIA.items():
            if r[k] > lim:
                return False, k
        return True, None

    def max_hs(self, lo: float = 0.002, hi: float = 1.0) -> tuple[float, str]:
        """Largest significant wave height meeting every criterion, and the
        criterion that sets it."""
        grid = np.geomspace(lo, hi, 50)
        prev = None
        for hs in grid:
            ok, why = self._ok(hs)
            if not ok:
                if prev is None:
                    return 0.0, why
                a, b = prev, hs
                for _ in range(30):
                    m = 0.5 * (a + b)
                    (a, b) = (m, b) if self._ok(m)[0] else (a, m)
                return a, self._ok(b)[1]
            prev = hs
        return hi, "none below 1 m"


def wmo_sea_state(hs: float) -> tuple[int, str]:
    """WMO sea state code (WMO-No. 306, code table 3700)."""
    for code, top, name in ((0, 0.0, "calm (glassy)"), (1, 0.1, "calm (rippled)"),
                            (2, 0.5, "smooth (wavelets)"), (3, 1.25, "slight"),
                            (4, 2.5, "moderate"), (5, 4.0, "rough")):
        if hs <= top:
            return code, name
    return 6, "very rough"


def seakeeping(p: HullParams, mass_g: float, lcg_mm: float, gm_mm: float, kb_mm: float,
               V: float, hatch: dict, wave_height_mm: float = 0.0) -> dict:
    sk = Seakeeping(p, mass_g / 1000, lcg_mm, gm_mm, kb_mm, V,
                    hatch_x_mm=hatch["x1"], hatch_top_mm=hatch["coaming_top_z"])
    hs, limit = sk.max_hs()
    code, name = wmo_sea_state(hs)
    lam_over_L = (2 * math.pi / sk.k) / (p.L / 1000)
    keep = (lam_over_L >= 0.3) & (lam_over_L <= 4.0)
    rao = [{"lambda_over_L": float(l), "heave": float(abs(z)), "pitch": float(abs(t) / k)}
           for l, z, t, k in zip(lam_over_L[keep][::-4], sk.z3[keep][::-4],
                                 sk.th[keep][::-4], sk.k[keep][::-4])][::-1]
    out = {
        "max_hs_mm": hs * 1000, "limited_by": limit,
        "limited_by_label": CRITERIA_LABELS.get(limit, limit),
        "wmo_code": code, "wmo_name": name,
        "tp_at_max_s": peak_period(hs) if hs > 0 else None,
        "natural_periods_s": {"heave": sk.T_heave, "pitch": sk.T_pitch, "roll": sk.T_roll},
        "at_max": sk.responses(hs) if hs > 0 else None,
        "criteria": CRITERIA,
        "rao_head_seas": rao,
        "speed_mps": V,
        "assumptions": (f"linear strip theory, head seas at cruise for heave/pitch, beam seas at rest "
                        f"for roll (effective slope); damping {ZETA_HEAVE:.2f}/{ZETA_ROLL:.2f} of critical assumed; "
                        f"Bretschneider spectrum at steepness 1/{1 / STEEPNESS:.0f}; "
                        "NORDFORSK 1987 limits applied at model scale"),
    }
    if wave_height_mm > 0:
        out["asked_hs_mm"] = wave_height_mm
        out["at_asked"] = sk.responses(wave_height_mm / 1000)
    return out

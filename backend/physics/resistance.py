"""Calm-water resistance of a displacement hull.

Friction: ITTC-1957 model-ship correlation line.
Form factor, wave and transom resistance: Holtrop & Mennen, "An approximate
power prediction method", International Shipbuilding Progress 29 (1982), with
the revised wave-resistance terms of Holtrop, ISP 31 (1984). The low-speed
branch (Fn <= 0.40) is implemented; above Fn 0.45 the method is outside its
regression range and the result is flagged, not extrapolated silently.

Model scale: an 0.8 m hull at 1 m/s runs at Re ~ 7e5, where part of the
boundary layer can stay laminar and the ITTC line over-predicts friction. No
turbulence stimulation is assumed; the correlation allowance C_A is set to 0
(Holtrop's C_A formula is fitted to full-scale hull roughness). Both are
stated in the output rather than hidden.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

RHO_FRESH = 999.1       # kg/m^3 at 15 C (ITTC 7.5-02-01-03)
NU_FRESH = 1.1386e-6    # m^2/s at 15 C (ITTC 7.5-02-01-03)
G = 9.80665

FN_VALID_MAX = 0.45


def cf_ittc57(re: float) -> float:
    return 0.075 / (math.log10(re) - 2.0) ** 2


@dataclass
class HullCoefficients:
    L: float        # waterline length, m
    B: float        # m
    T: float        # m, mean draft (canoe body at the max section)
    volume: float   # m^3
    S: float        # wetted surface, m^2
    Cm: float
    Cwp: float
    lcb_pct: float  # LCB forward of 0.5 L, percent of L
    AT: float       # immersed transom area at rest, m^2

    @property
    def Cb(self) -> float:
        return self.volume / (self.L * self.B * self.T)

    @property
    def Cp(self) -> float:
        return self.Cb / self.Cm


def holtrop(h: HullCoefficients, V: float, rho: float = RHO_FRESH,
            nu: float = NU_FRESH) -> dict:
    """Total calm-water resistance (N) and its components at speed V (m/s)."""
    L, B, T, vol, S = h.L, h.B, h.T, h.volume, h.S
    Cp, Cm, Cwp, lcb = h.Cp, h.Cm, h.Cwp, h.lcb_pct
    Fn = V / math.sqrt(G * L)
    q = 0.5 * rho * V * V

    re = V * L / nu
    cf = cf_ittc57(re)
    rf = q * S * cf

    # form factor 1 + k1 (c14 = 1: normal section shape aft)
    LR = L * (1 - Cp + 0.06 * Cp * lcb / (4 * Cp - 1))
    k1 = 0.93 + 0.487118 * (B / L) ** 1.06806 * (T / L) ** 0.46106 \
        * (L / LR) ** 0.121563 * (L ** 3 / vol) ** 0.36486 * (1 - Cp) ** -0.604247

    # wave resistance, Fn <= 0.40 branch (no bulb: c2 = 1)
    BL = B / L
    if BL < 0.11:
        c7 = 0.229577 * BL ** 0.33333
    elif BL < 0.25:
        c7 = BL
    else:
        c7 = 0.5 - 0.0625 * L / B
    iE = 1 + 89 * math.exp(-(L / B) ** 0.80856 * (1 - Cwp) ** 0.30484
                           * (1 - Cp - 0.0225 * lcb) ** 0.6367
                           * (LR / B) ** 0.34574 * (100 * vol / L ** 3) ** 0.16302)
    c1 = 2223105 * c7 ** 3.78613 * (T / B) ** 1.07961 * (90 - iE) ** -1.37565
    c5 = 1 - 0.8 * h.AT / (B * T * Cm)
    c16 = (8.07981 * Cp - 13.8673 * Cp ** 2 + 6.984388 * Cp ** 3) if Cp < 0.8 \
        else (1.73014 - 0.7067 * Cp)
    m1 = 0.0140407 * L / T - 1.75254 * vol ** (1 / 3) / L - 4.79323 * B / L - c16
    L3v = L ** 3 / vol
    if L3v < 512:
        c15 = -1.69385
    elif L3v > 1726.91:
        c15 = 0.0
    else:
        c15 = -1.69385 + (L / vol ** (1 / 3) - 8.0) / 2.36
    lam = 1.446 * Cp - 0.03 * L / B if L / B < 12 else 1.446 * Cp - 0.36
    if Fn > 0.01:
        m4 = c15 * 0.4 * math.exp(-0.034 * Fn ** -3.29)
        rw = c1 * c5 * vol * rho * G * math.exp(m1 * Fn ** -0.9 + m4 * math.cos(lam * Fn ** -2))
    else:
        rw = 0.0

    # immersed transom
    if h.AT > 0 and V > 0:
        fnt = V / math.sqrt(2 * G * h.AT / (B + B * Cwp))
        c6 = 0.2 * (1 - 0.2 * fnt) if fnt < 5 else 0.0
        rtr = q * h.AT * c6
    else:
        rtr = 0.0

    rt = rf * k1 + rw + rtr
    return {
        "V_mps": V, "Fn": Fn, "Re": re, "Cf": cf, "one_plus_k1": k1,
        "R_friction_N": rf * k1, "R_wave_N": rw, "R_transom_N": rtr,
        "R_total_N": rt, "P_effective_W": rt * V,
        "in_range": Fn <= FN_VALID_MAX,
    }


def hull_speed(L_m: float, fn: float = 0.40) -> float:
    return fn * math.sqrt(G * L_m)

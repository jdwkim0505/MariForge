"""Hydrostatics by numerical integration of hull sections.

Works on any section generator `section(x) -> (N, 2) polygon of (y, z)` so the
same code is tested against closed-form shapes (a box barge) and then run on
the parametric hull. Fixed trim throughout: heeling rotates the sections about
the x axis and re-solves the waterline height, it does not re-trim the hull.
That is the standard "fixed trim" GZ approximation and is stated in the
output; the CAD test checks trim separately (LCB vs LCG on the real mesh).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy.optimize import brentq

Section = Callable[[float], np.ndarray]


# ---- 2D polygon helpers --------------------------------------------------

def poly_area_centroid(poly: np.ndarray) -> tuple[float, float, float]:
    """Signed-area-safe (area, cy, cz) of a simple polygon."""
    if len(poly) < 3:
        return 0.0, 0.0, 0.0
    y, z = poly[:, 0], poly[:, 1]
    y1, z1 = np.roll(y, -1), np.roll(z, -1)
    cross = y * z1 - y1 * z
    a = 0.5 * cross.sum()
    if abs(a) < 1e-12:
        return 0.0, 0.0, 0.0
    cy = ((y + y1) * cross).sum() / (6 * a)
    cz = ((z + z1) * cross).sum() / (6 * a)
    return abs(a), cy, cz


def clip_below(poly: np.ndarray, h: float) -> np.ndarray:
    """Sutherland-Hodgman clip of a polygon to the half-plane z <= h."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        pin, qin = p[1] <= h, q[1] <= h
        if pin:
            out.append(p)
        if pin != qin:
            t = (h - p[1]) / (q[1] - p[1])
            out.append(p + t * (q - p))
    return np.asarray(out) if out else np.zeros((0, 2))


def rotate(poly: np.ndarray, phi: float) -> np.ndarray:
    """Rotate section points by heel angle phi (rad) about the x axis."""
    c, s = math.cos(phi), math.sin(phi)
    y, z = poly[:, 0], poly[:, 1]
    return np.column_stack([y * c - z * s, y * s + z * c])


def wetted_girth(poly: np.ndarray, h: float) -> float:
    """Length of the section outline lying below z = h."""
    sub = clip_below(poly, h)
    if len(sub) < 3:
        return 0.0
    # the clip adds a waterline chord; remove it (it is the only edge at z == h)
    seg = np.roll(sub, -1, axis=0) - sub
    lens = np.hypot(seg[:, 0], seg[:, 1])
    on_wl = (np.abs(sub[:, 1] - h) < 1e-9) & (np.abs(np.roll(sub, -1, axis=0)[:, 1] - h) < 1e-9)
    return float(lens[~on_wl].sum())


# ---- integration along x --------------------------------------------------

def _stations(x0: float, x1: float, n: int) -> np.ndarray:
    return np.linspace(x0, x1, n)


def _trapz(f: np.ndarray, x: np.ndarray) -> float:
    return float(np.trapezoid(f, x))


@dataclass
class Submerged:
    volume: float   # mm^3
    lcb: float      # x of centre of buoyancy, mm
    tcb: float      # y
    vcb: float      # z
    wetted: float   # mm^2 (hull girth integral; transom and deck excluded)


def sample(section: Section, x0: float, x1: float, n: int = 161) -> list:
    """Precompute section polygons once; every function below accepts either a
    section callable or this list, so a heel sweep does not rebuild sections."""
    return [(x, section(x)) for x in _stations(x0, x1, n)]


def _as_samples(section, x0, x1, n):
    return section if isinstance(section, list) else sample(section, x0, x1, n)


def submerged(section, x0: float, x1: float, h: float = 0.0,
              phi: float = 0.0, n: int = 161) -> Submerged:
    samples = _as_samples(section, x0, x1, n)
    n = len(samples)
    xs = np.array([x for x, _ in samples])
    a = np.zeros(n); my = np.zeros(n); mz = np.zeros(n); g = np.zeros(n)
    for i, (x, poly) in enumerate(samples):
        if phi:
            poly = rotate(poly, phi)
        sub = clip_below(poly, h)
        ai, cy, cz = poly_area_centroid(sub)
        a[i], my[i], mz[i] = ai, ai * cy, ai * cz
        g[i] = wetted_girth(poly, h)
    vol = _trapz(a, xs)
    if vol <= 0:
        return Submerged(0.0, 0.5 * (x0 + x1), 0.0, h, 0.0)
    return Submerged(
        volume=vol,
        lcb=_trapz(a * xs, xs) / vol,
        tcb=_trapz(my, xs) / vol,
        vcb=_trapz(mz, xs) / vol,
        wetted=_trapz(g, xs),
    )


def waterplane(section, x0: float, x1: float, h: float = 0.0,
               n: int = 161) -> dict:
    """Waterplane area, its centroid and transverse second moment, level hull."""
    samples = _as_samples(section, x0, x1, n)
    n = len(samples)
    xs = np.array([x for x, _ in samples])
    half = np.zeros(n)
    for i, (x, poly) in enumerate(samples):
        # waterline half-breadth: the widest crossing of z = h
        ys = []
        m = len(poly)
        for k in range(m):
            p, q = poly[k], poly[(k + 1) % m]
            if (p[1] - h) * (q[1] - h) <= 0 and p[1] != q[1]:
                t = (h - p[1]) / (q[1] - p[1])
                ys.append(p[0] + t * (q[0] - p[0]))
        half[i] = 0.5 * (max(ys) - min(ys)) if len(ys) >= 2 else 0.0
    awp = _trapz(2 * half, xs)
    lcf = _trapz(2 * half * xs, xs) / awp if awp > 0 else 0.5 * (x0 + x1)
    it = _trapz((2.0 / 3.0) * half ** 3, xs)
    return {"area": awp, "lcf": lcf, "I_T": it, "half_breadths": half, "x": xs}


def draft_for_volume(section, x0: float, x1: float, volume: float,
                     phi: float = 0.0, lo: float = -500.0, hi: float = 500.0) -> float:
    """Waterline height h at which the (heeled) hull displaces `volume`."""
    samples = _as_samples(section, x0, x1, 81)
    if phi:
        samples = [(x, rotate(poly, phi)) for x, poly in samples]
    xs = np.array([x for x, _ in samples])

    def f(h):
        a = [poly_area_centroid(clip_below(poly, h))[0] for _, poly in samples]
        return _trapz(np.asarray(a), xs) - volume
    return brentq(f, lo, hi, xtol=1e-3)


def gz_curve(section, x0: float, x1: float, volume: float,
             kg: float, angles_deg=range(0, 91, 5), z_bounds=(-500.0, 500.0)) -> list[dict]:
    """Righting arm GZ (mm) vs heel, fixed trim, G on the centreline at z = kg."""
    samples = _as_samples(section, x0, x1, 81)
    out = []
    for deg in angles_deg:
        phi = math.radians(deg)
        h = draft_for_volume(samples, x0, x1, volume, phi, *z_bounds)
        sb = submerged(samples, x0, x1, h, phi)
        y_g = -kg * math.sin(phi)   # G rotated with the hull
        out.append({"heel_deg": deg, "gz_mm": y_g - sb.tcb, "wl_mm": h})
    return out


def downflooding_angle(point_yz: tuple[float, float], section,
                       x0: float, x1: float, volume: float,
                       max_deg: float = 90.0) -> float:
    """Heel angle at which a hull opening (y, z) first reaches the waterline."""
    y, z = point_yz
    section = _as_samples(section, x0, x1, 81)

    def margin(deg: float) -> float:
        phi = math.radians(deg)
        h = draft_for_volume(section, x0, x1, volume, phi)
        # the opening edge on the low side (-y after rotation)
        z_rot = -abs(y) * math.sin(phi) + z * math.cos(phi)
        return z_rot - h

    if margin(max_deg) > 0:
        return max_deg
    return brentq(margin, 0.0, max_deg, xtol=0.05)

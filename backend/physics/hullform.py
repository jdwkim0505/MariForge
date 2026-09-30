"""Parametric displacement-monohull form.

One analytic surface is the single source of truth for the hull: the physics
integrates its sections numerically and the CAD lofts the same sections, so a
test can prove the exported STL displaces what the physics says it does.

Frame (all lengths in mm here; physics converts to SI):
    x  from the transom (x = 0) forward to the stem (x = L)
    y  positive to port, the hull is symmetric about y = 0
    z  up, with the DESIGN WATERLINE at z = 0 (keel at -T, deck at +F)

A section at station xi = x / L is:
    underwater (-t <= z <= 0): superellipse  y = b * (1 - (-z/t)^n)^(1/n)
    topside    (0 <= z <= f):  straight flare y = b * (1 + flare * z / f)
    deck       (z = f):        flat across
so the waterline half-breadth is b(xi), the canoe-body depth is t(xi), and
the deck height is f(xi) (a sheer that rises toward the bow).
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

# The stem is not a knife edge: FDM cannot print a zero-width section, and
# OpenCASCADE cannot loft one. 1.2 mm is three 0.4 mm extrusion widths.
STEM_HALF_BREADTH_MM = 1.2


@dataclass(frozen=True)
class HullParams:
    L: float            # waterline length = hull length (plumb stem, transom), mm
    B: float            # waterline beam at the max section, mm
    T: float            # canoe-body draft at the max section, mm
    F: float            # freeboard (deck above the design waterline) at midship, mm
    n: float = 2.6      # section fullness exponent: 2 = elliptic bilge, larger = boxier
    x_max: float = 0.55     # max-section station, fraction of L from the transom
    bow_exp: float = 1.9    # waterline entry exponent: larger = fuller bow
    stern_exp: float = 2.4  # run exponent toward the transom
    transom_b: float = 0.72  # transom waterline half-breadth / max half-breadth
    transom_t: float = 0.30  # transom immersion / T (rising buttocks)
    forefoot_t: float = 0.35  # canoe depth at the stem / T
    flare_mid: float = 0.04   # topside flare at the max section
    flare_bow: float = 0.30   # topside flare at the stem
    sheer_bow: float = 0.30   # deck rise at the stem, fraction of F

    def to_dict(self) -> dict:
        return asdict(self)


def _smooth01(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def half_breadth_wl(p: HullParams, xi: float) -> float:
    """Waterline half-breadth b(xi), mm."""
    b0 = p.B / 2.0
    if xi >= p.x_max:
        u = (xi - p.x_max) / (1.0 - p.x_max)
        f = 1.0 - u ** p.bow_exp
    else:
        u = (p.x_max - xi) / p.x_max
        f = 1.0 - (1.0 - p.transom_b) * u ** p.stern_exp
    return max(b0 * f, STEM_HALF_BREADTH_MM)


def canoe_depth(p: HullParams, xi: float) -> float:
    """Canoe-body depth t(xi) below the waterline, mm (positive)."""
    xs, xb = 0.40, 0.70
    if xi < xs:
        u = (xs - xi) / xs
        g = 1.0 - (1.0 - p.transom_t) * u ** 2
    elif xi > xb:
        u = (xi - xb) / (1.0 - xb)
        g = 1.0 - (1.0 - p.forefoot_t) * u ** 2
    else:
        g = 1.0
    return p.T * g


def deck_height(p: HullParams, xi: float) -> float:
    """Deck height f(xi) above the waterline, mm."""
    u = max(0.0, (xi - 0.5) / 0.5)
    return p.F * (1.0 + p.sheer_bow * u ** 2)


def flare(p: HullParams, xi: float) -> float:
    u = _smooth01((xi - p.x_max) / (1.0 - p.x_max)) if xi > p.x_max else 0.0
    return p.flare_mid + (p.flare_bow - p.flare_mid) * u


def side_curve(p: HullParams, xi: float, n_under: int = 18, n_top: int = 5) -> np.ndarray:
    """Port-side curve of one section, keel -> deck edge, as (y, z) rows, mm.

    Underwater points are cosine-clustered toward the keel, where the
    superellipse turns fastest.
    """
    b = half_breadth_wl(p, xi)
    t = canoe_depth(p, xi)
    f = deck_height(p, xi)
    fl = flare(p, xi)
    pts = []
    for k in range(n_under + 1):
        s = 0.5 * (1.0 - math.cos(math.pi * k / n_under))  # 0 at keel -> 1 at WL
        z = -t * (1.0 - s)
        r = (-z / t) if t > 0 else 0.0
        y = b * max(0.0, 1.0 - r ** p.n) ** (1.0 / p.n)
        pts.append((y, z))
    for k in range(1, n_top + 1):
        z = f * k / n_top
        pts.append((b * (1.0 + fl * z / f), z))
    return np.asarray(pts)


def section_polygon(p: HullParams, xi: float, **kw) -> np.ndarray:
    """Closed section polygon (y, z), counter-clockwise, first point not repeated."""
    port = side_curve(p, xi, **kw)
    stbd = port[::-1].copy()
    stbd[:, 0] *= -1.0
    # keel (port[0]) -> deck edge port -> deck edge starboard -> keel
    return np.vstack([port, stbd[:-1]])


def midship_coefficient(n: float) -> float:
    """Area of the underwater superellipse quadrant / (b * t)."""
    return math.gamma(1 + 1 / n) ** 2 / math.gamma(1 + 2 / n)


def inner_side_curve(p: HullParams, xi: float, wall: float, **kw) -> np.ndarray:
    """Port-side curve of the cavity: the outer side curve offset inward by
    `wall` along its 2D normals, stopped at the underside of the deck.

    A 2D per-station offset ignores the longitudinal slope of the surface; the
    cavity is only lofted where that slope is small (see layout.cavity_range)
    and the CAD test measures the real minimum wall.
    """
    c = side_curve(p, xi, **kw)
    d = np.gradient(c, axis=0)
    # the curve runs keel -> deck with the hull interior on its left, so the
    # inward normal is the tangent rotated +90 deg: (dy, dz) -> (-dz, dy)
    inward = np.column_stack([-d[:, 1], d[:, 0]]) / np.linalg.norm(d, axis=1)[:, None]
    off = c + wall * inward
    off[0] = (0.0, c[0, 1] + wall)          # keel point moves straight up
    z_top = deck_height(p, xi) - wall
    keep = [q for q in off if q[1] < z_top - 1e-6 and q[0] >= 0.0]
    keep = np.asarray(keep)
    # close at the deck underside by extending the last segment to z_top
    a, b = keep[-2], keep[-1]
    t = (z_top - a[1]) / (b[1] - a[1]) if b[1] != a[1] else 1.0
    top = a + t * (b - a)
    return np.vstack([keep, top])


def inner_half_breadth_at(p: HullParams, xi: float, z: float, wall: float) -> float:
    """Cavity half-breadth at height z (mm), 0 if z is outside the cavity."""
    c = inner_side_curve(p, xi, wall)
    if z < c[0, 1] or z > c[-1, 1]:
        return 0.0
    return float(np.interp(z, c[:, 1], c[:, 0]))


def inner_section_polygon(p: HullParams, xi: float, wall: float) -> np.ndarray:
    port = inner_side_curve(p, xi, wall)
    stbd = port[::-1].copy()
    stbd[:, 0] *= -1.0
    return np.vstack([port, stbd[:-1]])

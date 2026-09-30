"""The integrator against closed forms, before it is trusted on a hull."""
import math

import numpy as np
import pytest

from backend.physics.hydrostatics import (clip_below, gz_curve, poly_area_centroid,
                                          submerged, waterplane)

B, T, D, L = 200.0, 50.0, 120.0, 1000.0


def box(x):
    return np.array([[B / 2, -T], [B / 2, D - T], [-B / 2, D - T], [-B / 2, -T]])


def test_box_volume_centroid_wetted():
    s = submerged(box, 0, L)
    assert s.volume == pytest.approx(L * B * T, rel=1e-9)
    assert s.vcb == pytest.approx(-T / 2)
    assert s.wetted == pytest.approx(L * (B + 2 * T), rel=1e-9)


def test_box_metacentric_radius():
    s = submerged(box, 0, L)
    wp = waterplane(box, 0, L)
    assert wp["I_T"] / s.volume == pytest.approx(B * B / (12 * T), rel=1e-9)


@pytest.mark.parametrize("deg", [5, 10, 20])
def test_box_wall_sided_gz(deg):
    """Wall-sided formula GZ = sin(phi) (GM + BM/2 tan^2 phi), exact before deck edge immersion."""
    s = submerged(box, 0, L)
    bm = B * B / (12 * T)
    kg = 0.0
    gm = s.vcb + bm - kg
    phi = math.radians(deg)
    got = gz_curve(box, 0, L, s.volume, kg, [deg])[0]["gz_mm"]
    assert got == pytest.approx(math.sin(phi) * (gm + 0.5 * bm * math.tan(phi) ** 2), rel=1e-5)


def test_semicircle_area_centroid():
    r = 50.0
    th = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
    circ = np.column_stack([r * np.cos(th), r * np.sin(th)])
    a, cy, cz = poly_area_centroid(clip_below(circ, 0.0))
    assert a == pytest.approx(math.pi * r * r / 2, rel=1e-4)
    assert cz == pytest.approx(-4 * r / (3 * math.pi), rel=1e-3)

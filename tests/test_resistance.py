import pytest

from backend.physics.resistance import HullCoefficients, cf_ittc57, holtrop


def test_ittc57_reference_value():
    # 0.075 / (log10(1e6) - 2)^2 = 0.075 / 16
    assert cf_ittc57(1e6) == pytest.approx(0.075 / 16)


def _model():
    return HullCoefficients(L=0.8, B=0.19, T=0.056, volume=0.0051, S=0.19,
                            Cm=0.84, Cwp=0.78, lcb_pct=-2.0, AT=0.0015)


def test_components_positive_and_rising_with_speed():
    h = _model()
    prev = 0.0
    for v in (0.3, 0.5, 0.7, 0.9, 1.1):
        r = holtrop(h, v)
        assert r["R_friction_N"] > 0 and r["R_wave_N"] >= 0 and r["one_plus_k1"] > 1
        assert r["R_total_N"] > prev
        prev = r["R_total_N"]


def test_wave_share_grows_toward_hull_speed():
    h = _model()
    lo, hi = holtrop(h, 0.4), holtrop(h, 1.1)
    assert hi["R_wave_N"] / hi["R_total_N"] > lo["R_wave_N"] / lo["R_total_N"]


def test_out_of_range_is_flagged():
    assert not holtrop(_model(), 1.4)["in_range"]

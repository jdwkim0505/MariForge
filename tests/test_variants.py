"""Physics-only: every request returns five different, honest designs."""
import pytest

from backend.physics.variants import generate_variants
from tests.conftest import DEFAULT_REQ

REQS = [DEFAULT_REQ,
        {**DEFAULT_REQ, "length_mm": 500, "beam_mm": 160, "height_mm": 180, "cruise_speed_mps": 0.7},
        {**DEFAULT_REQ, "length_mm": 1200, "beam_mm": 400, "height_mm": 400, "cruise_speed_mps": 1.2}]


@pytest.fixture(scope="module", params=range(len(REQS)))
def designs(request):
    return REQS[request.param], generate_variants(REQS[request.param])


def test_five_distinct(designs):
    _, ds = designs
    assert len(ds) == 5
    assert len({round(d["params"]["B"]) for d in ds}) == 5
    assert len({round(d["hydrostatics"]["Cb"], 2) for d in ds}) >= 4


def test_mass_closes(designs):
    """Parts + ballast = displacement; LCG on LCB when the ballast can reach."""
    _, ds = designs
    for d in ds:
        m = d["mass"]
        assert m["without_ballast_g"] + m["ballast"]["mass_g"] == pytest.approx(m["total_g"], rel=1e-9)
        if m["ballast"]["reached"]:
            assert d["hydrostatics"]["lcg_mm"] == pytest.approx(d["hydrostatics"]["lcb_mm"], abs=0.01)


def test_fits_box_or_says_so(designs):
    req, ds = designs
    for d in ds:
        by = {c["name"]: c["ok"] for c in d["checks"]}
        assert by["box.height"] == (d["overall"]["height_mm"] <= req["height_mm"])
        assert d["overall"]["beam_mm"] <= req["beam_mm"] + 0.5
        assert d["overall"]["length_mm"] <= req["length_mm"]


def test_feasible_means_every_check(designs):
    _, ds = designs
    for d in ds:
        assert d["feasible"] == all(c["ok"] for c in d["checks"])


def test_gz_starts_at_zero_and_rises(designs):
    _, ds = designs
    for d in ds:
        gz = d["hydrostatics"]["gz"]
        assert gz[0]["gz_mm"] == pytest.approx(0.0, abs=1e-6)
        assert gz[1]["gz_mm"] > 0
        # small-angle slope matches GM: GZ(5 deg) ~ GM sin(5 deg) within 10 %
        import math
        assert gz[1]["gz_mm"] == pytest.approx(d["hydrostatics"]["gm_mm"] * math.sin(math.radians(5)), rel=0.10)


def test_too_fast_is_reported_not_hidden():
    ds = generate_variants({**DEFAULT_REQ, "cruise_speed_mps": 1.6})
    for d in ds:
        c = next(c for c in d["checks"] if c["name"] == "speed.froude")
        assert not c["ok"] and not d["feasible"]

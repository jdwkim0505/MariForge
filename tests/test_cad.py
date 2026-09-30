"""Full CAD builds: the exported geometry is the proof. Slow (~40 s per hull)."""
import pytest

from backend.cad.exporters import split_for_bed
from backend.cad.hull import build_parts
from backend.cad.verify import verify_design, volume
from backend.physics.variants import generate_variants
from tests.conftest import DEFAULT_REQ

_DESIGNS = generate_variants(DEFAULT_REQ)


@pytest.fixture(scope="module", params=[d["character"] for d in _DESIGNS])
def built(request):
    d = next(x for x in _DESIGNS if x["character"] == request.param)
    parts = build_parts(d)
    return d, parts, verify_design(d, parts)


def test_hull_is_one_valid_watertight_solid(built):
    _, parts, v = built
    assert v["hull_valid"] and v["hull_solids"] == 1 and v["hull_mesh_watertight"]
    for k in ("hatch_lid", "rudder"):
        assert parts[k].isValid()


def test_exported_envelope_displaces_the_design(built):
    _, _, v = built
    assert abs(v["volume_error"]) < 0.02


def test_floats_at_design_waterline_level(built):
    d, _, v = built
    assert abs(v["float_error_frac_T"]) < 0.02
    assert abs(v["trim_error_frac_L"]) < 0.005


def test_every_cut_exists(built):
    _, _, v = built
    bad = [c["name"] for c in v["cuts"] if not c["ok"]]
    assert not bad, bad


def test_printed_mass_estimate_holds(built):
    _, _, v = built
    real = sum(v["printed_mass_g"].values())
    assert real == pytest.approx(v["printed_mass_physics_g"], rel=0.10)


def test_no_feature_refused_on_a_feasible_design(built):
    d, parts, _ = built
    if d["feasible"]:
        assert parts["_report"]["refused"] == []


def test_split_segments_fit_the_bed(built):
    d, parts, _ = built
    segs = split_for_bed(d, parts["hull"], 250)
    total = 0.0
    for name, s in segs:
        bb = s.BoundingBox()
        assert bb.xlen <= 250
        assert len(s.Solids()) == 1, f"{name} falls apart into {len(s.Solids())} pieces"
        total += volume(s)
    assert total == pytest.approx(volume(parts["hull"]), rel=1e-3)

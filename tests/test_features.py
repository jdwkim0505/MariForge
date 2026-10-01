"""Payload-first design, seakeeping, ArduPilot export and the parts list."""
import math

import numpy as np
import pytest
from fastapi.testclient import TestClient

from backend.main import app
from backend.physics.ardupilot import rover_params
from backend.physics.hullform import HullParams
from backend.physics.seakeeping import Seakeeping, lewis_coefficient, seakeeping, wmo_sea_state
from backend.physics.variants import generate_variants
from tests.conftest import DEFAULT_REQ, PAYLOAD_REQ

_PLAIN = {d["character"]: d for d in generate_variants(DEFAULT_REQ)}
_LOADED = {d["character"]: d for d in generate_variants(PAYLOAD_REQ)}
client = TestClient(app)


# ---- payload --------------------------------------------------------------------
def test_every_payload_item_is_placed_and_named():
    for d in _LOADED.values():
        names = [c["name"] for c in d["checks"] if c["name"].startswith("payload.")]
        assert len(names) == 6
        assert len(d["payload"]["items"]) == 6, d["character"]


def test_payload_mass_and_power_are_carried():
    for k, d in _LOADED.items():
        assert d["payload"]["mass_g"] == pytest.approx(70 + 30 + 150 + 450 + 35 + 60)
        assert d["propulsion"]["P_payload_W"] == pytest.approx(2.5 + 1 + 1 + 3 + 5)
        # same displacement-class hull, more of the lead traded for cargo
        assert d["payload"]["spare_capacity_g"] < _PLAIN[k]["payload"]["spare_capacity_g"]
        assert d["propulsion"]["endurance_min"] >= DEFAULT_REQ["endurance_min"]


def test_flooded_well_is_taken_out_of_the_buoyancy():
    d = _LOADED["cruiser"]
    h = d["hydrostatics"]
    (w,) = d["layout"]["wells"]
    water = math.pi * (w["bore"] / 2) ** 2 * (-w["z_keel"])
    assert h["well_water_mm3"] == pytest.approx(water)
    assert h["volume_mm3"] == pytest.approx(h["hull_volume_mm3"] - water)
    assert h["free_surface_mm"] == pytest.approx(math.pi * (w["bore"] / 2) ** 4 / 4 / h["volume_mm3"])


def test_a_payload_that_cannot_fit_is_refused_by_name():
    req = {**DEFAULT_REQ, "payloads": [{"key": "custom", "name": "crate", "mass_g": 300,
                                        "dims_mm": [500, 400, 300], "mount": "bay"}]}
    for d in generate_variants(req):
        bad = [c for c in d["checks"] if c["name"] == "payload.custom1"]
        assert bad and not bad[0]["ok"] and not d["feasible"]


def test_api_validates_payload_rows():
    assert client.post("/api/generate", json={**DEFAULT_REQ, "payloads": [{"key": "nope"}]}).status_code == 422
    ok = client.post("/api/generate", json={**DEFAULT_REQ, "payloads": [
        {"key": "custom", "name": "LiDAR", "mass_g": 200, "dims_mm": [70, 60, 50], "mount": "deck"}]})
    assert ok.status_code == 200
    assert len(client.get("/api/options").json()["payloads"]) >= 9


# ---- seakeeping -------------------------------------------------------------------
def test_lewis_semicircle_is_unity():
    assert lewis_coefficient(1.0, 1.0, math.pi / 4) == pytest.approx(1.0, abs=1e-9)


def test_long_waves_are_followed():
    d = _PLAIN["cruiser"]; h = d["hydrostatics"]
    sk = Seakeeping(HullParams(**d["params"]), h["displacement_g"] / 1000, h["lcg_mm"],
                    h["gm_mm"], h["kb_mm"], 0.0, 300, 50)
    assert abs(sk.z3[0]) == pytest.approx(1.0, abs=0.01)            # heave = wave height
    assert abs(sk.th[0]) / sk.k[0] == pytest.approx(1.0, abs=0.01)  # pitch = wave slope
    assert sk.roll_per_slope[0] == pytest.approx(1.0, abs=0.01)     # roll = wave slope
    assert np.all(np.abs(sk.z3[-50:]) < 0.05)                       # short waves average out


def test_max_wave_height_is_the_edge_of_the_criteria():
    d = _PLAIN["cruiser"]; h = d["hydrostatics"]
    sk = Seakeeping(HullParams(**d["params"]), h["displacement_g"] / 1000, h["lcg_mm"],
                    h["gm_mm"], h["kb_mm"], 0.9, d["layout"]["hatch"]["x1"],
                    d["layout"]["hatch"]["coaming_top_z"])
    hs, why = sk.max_hs()
    assert 0.005 < hs < 0.5
    assert sk._ok(hs * 0.98)[0] and not sk._ok(hs * 1.05)[0]


def test_a_bigger_hull_takes_bigger_waves():
    big = generate_variants({**DEFAULT_REQ, "length_mm": 1500, "beam_mm": 470, "height_mm": 490})
    small = {d["character"]: d["seakeeping"]["max_hs_mm"] for d in _PLAIN.values()}
    for d in big:
        assert d["seakeeping"]["max_hs_mm"] > small[d["character"]]


def test_wave_requirement_is_a_check():
    req = {**DEFAULT_REQ, "wave_height_mm": 400}
    for d in generate_variants(req):
        c = next(c for c in d["checks"] if c["name"] == "seakeeping.wave_height")
        assert not c["ok"]


def test_wmo_codes():
    assert wmo_sea_state(0.05)[0] == 1 and wmo_sea_state(0.3)[0] == 2 and wmo_sea_state(1.0)[0] == 3


# ---- ArduPilot ----------------------------------------------------------------------
def test_rover_params_come_from_the_design():
    d = _PLAIN["cruiser"]
    p = {r["name"]: r["value"] for r in rover_params(d)["params"]}
    assert p["FRAME_CLASS"] == 2 and p["SERVO1_FUNCTION"] == 26 and p["SERVO3_FUNCTION"] == 70
    assert p["CRUISE_SPEED"] == pytest.approx(DEFAULT_REQ["cruise_speed_mps"])
    assert 0 < p["CRUISE_THROTTLE"] <= 100
    assert p["BATT_CAPACITY"] == d["propulsion"]["battery"]["mAh"]
    assert p["ATC_STR_RAT_MAX"] == pytest.approx(math.degrees(0.9 / p["TURN_RADIUS"]), abs=1)


def test_param_file_downloads():
    designs = client.post("/api/generate", json=DEFAULT_REQ).json()["designs"]
    r = client.get(f"/api/export/{designs[0]['id']}/ardupilot.param")
    assert r.status_code == 200 and "attachment" in r.headers["content-disposition"]
    lines = [l for l in r.text.splitlines() if l and not l.startswith("#")]
    assert all(len(l.split(",")) == 2 for l in lines)
    assert "FRAME_CLASS,2" in lines


# ---- parts list ----------------------------------------------------------------------
def test_bom_adds_up_and_lists_the_payload():
    d = _LOADED["cruiser"]
    b = d["bom"]
    assert b["total_usd"] == pytest.approx(sum(r["usd"] for r in b["rows"]))
    assert sum(b["by_group"].values()) == pytest.approx(b["total_usd"])
    assert sum(1 for r in b["rows"] if r["group"] == "payload") == 6
    lo, hi = b["print_hours"]
    assert 0 < lo < hi

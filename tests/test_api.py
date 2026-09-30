from fastapi.testclient import TestClient

from backend.main import app
from tests.conftest import DEFAULT_REQ

client = TestClient(app)


def test_options():
    r = client.get("/api/options")
    assert r.status_code == 200
    assert r.json()["hull_types"][0]["key"] == "displacement_monohull"


def test_generate_returns_five_json_designs():
    r = client.post("/api/generate", json=DEFAULT_REQ)
    assert r.status_code == 200
    ds = r.json()["designs"]
    assert len(ds) == 5 and all("checks" in d and "traits" in d for d in ds)
    assert client.get(f"/api/design/{ds[0]['id']}").status_code == 200


def test_rejects_out_of_range_box():
    assert client.post("/api/generate", json={**DEFAULT_REQ, "length_mm": 50}).status_code == 422

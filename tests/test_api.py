import time

import pytest
from conftest import FIXTURE
from fastapi.testclient import TestClient

from collision_checker.api import app, store


@pytest.fixture
def client():
    store.stop()
    store.session = store.job = None
    with TestClient(app) as client:
        yield client


def load(client):
    response = client.post("/api/load", json={"path": str(FIXTURE)})
    assert response.status_code == 200, response.text
    return response.json()


def wait_job(client):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        response = client.get("/api/state").json()
        if response["job"]["status"] not in ["running", "cancelling"]:
            return response
        time.sleep(0.05)
    pytest.fail("Worker did not finish")


def test_process_analysis_and_benchmark(client):
    load(client)
    assert (
        client.patch("/api/settings", json={"settings": {"samples": 100}}).status_code
        == 200
    )
    assert client.post("/api/jobs", json={"kind": "analysis"}).status_code == 200
    response = wait_job(client)
    assert response["job"]["status"] == "complete", response
    assert not response["session"]["stale"]
    assert client.post("/api/topk", json={"k": 2, "apply": True}).status_code == 200
    assert (
        client.post("/api/jobs", json={"kind": "benchmark", "samples": 5}).status_code
        == 200
    )
    response = wait_job(client)
    assert response["job"]["status"] == "complete", response
    assert response["session"]["benchmark"]["results"]["selected"]["link_pairs"] <= 2
    assert client.post("/api/undo", json={}).status_code == 200


def test_cancel_and_stale_result(client):
    load(client)
    client.patch("/api/settings", json={"settings": {"samples": 100000}})
    client.post("/api/jobs", json={"kind": "analysis"})
    assert client.post("/api/jobs", json={"kind": "benchmark"}).status_code == 400
    client.post("/api/jobs/cancel", json={})
    assert wait_job(client)["job"]["status"] == "cancelled"
    client.patch("/api/settings", json={"settings": {"samples": 3000}})
    client.post("/api/jobs", json={"kind": "analysis"})
    client.patch("/api/settings", json={"settings": {"seed": 10}})
    response = wait_job(client)
    assert response["job"]["status"] == "obsolete"
    assert response["session"]["stale"]


def test_project_roundtrip_and_fingerprint_invalidation(client, tmp_path):
    source = tmp_path / "robot.urdf"
    source.write_text(FIXTURE.read_text())
    session = client.post("/api/load", json={"path": str(source)}).json()
    key = session["pairs"][0]["id"]
    client.patch(
        "/api/pairs",
        json={"ids": [key], "patch": {"lock": "keep", "min_distance": 0.08}},
    )
    client.post("/api/pose", json={"pose": {"extension": 0.4}})
    project = tmp_path / "test.collision-project.json"
    assert (
        client.post("/api/project/save", json={"path": str(project)}).status_code == 200
    )
    restored = client.post("/api/project/open", json={"path": str(project)}).json()
    assert restored["warning"] is None
    assert restored["session"]["pairs"][0]["lock"] == "keep"
    assert restored["session"]["pairs"][0]["min_distance"] == 0.08
    assert restored["session"]["model"]["pose"]["finger_mimic"] == pytest.approx(0.02)
    source.write_text(source.read_text().replace('radius="0.25"', 'radius="0.26"'))
    restored = client.post("/api/project/open", json={"path": str(project)}).json()
    assert restored["warning"]
    assert restored["session"]["stale"]
    assert restored["session"]["pairs"][0]["lock"] is None


def test_atomic_bulk_validation_and_export_confirmation(client, tmp_path):
    session = load(client)
    key = session["pairs"][0]["id"]
    assert (
        client.patch(
            "/api/pairs", json={"ids": [key, "wrong"], "patch": {"manual": False}}
        ).status_code
        == 400
    )
    assert client.get("/api/state").json()["session"]["pairs"][0]["enabled"]
    assert (
        client.patch(
            "/api/pairs", json={"ids": [key], "patch": {"min_distance": -1}}
        ).status_code
        == 400
    )
    client.patch("/api/pairs", json={"ids": [key], "patch": {"min_distance": 0.2}})
    response = client.post("/api/export", json={"directory": str(tmp_path)})
    assert response.status_code == 409
    assert not list(tmp_path.iterdir())
    assert (
        client.post(
            "/api/export",
            json={"directory": str(tmp_path), "acknowledge_info_global_distance": True},
        ).status_code
        == 200
    )
    assert {p.name for p in tmp_path.iterdir()} == {
        "disabled_collisions.srdf",
        "self_collision.info",
        "self_collision.yaml",
    }


def test_bad_settings_and_local_api_boundary(client):
    load(client)
    for patch in [
        {"samples": -2},
        {"enabled": "false"},
        {"default_min_distance": None},
        {"always_threshold": 2},
        {"rules": {"bogus": True}},
    ]:
        assert (
            client.patch("/api/settings", json={"settings": patch}).status_code == 400
        )
    assert client.post("/api/pose", json={"pose": {"extension": -1}}).status_code == 400
    assert (
        client.post("/api/pose", json={"pose": {}, "revision": "old"}).status_code
        == 400
    )
    assert (
        client.post(
            "/api/load",
            json={"path": str(FIXTURE)},
            headers={"Origin": "https://example.com"},
        ).status_code
        == 403
    )
    assert (
        client.post(
            "/api/load", content="{}", headers={"Content-Type": "text/plain"}
        ).status_code
        == 415
    )

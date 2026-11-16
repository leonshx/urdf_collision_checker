import copy
import math

import pytest

from collision_checker.analysis import analyze, benchmark, validate_settings
from collision_checker.model import Robot


def key(robot, a, b):
    return robot.pair_key(a, b)


def test_geometry_indices_mimic_and_exact_distances(robot):
    assert len(robot.objects) == 7
    assert len(robot.by_link["base"]) == 2
    assert len(robot.pairs[key(robot, "base", "arm")]["objects"]) == 2
    assert robot.pairs[key(robot, "base", "arm")]["adjacent"]  # spacer has no geometry
    robot.place({"swing": 0.4, "extension": 0.6})
    assert robot.pose["finger_mimic"] == pytest.approx(0.03)
    assert robot.evaluate(robot.pairs[key(robot, "base", "far")])[
        "distance"
    ] == pytest.approx(3.0, abs=1e-5)
    robot.place({"swing": 0.0, "extension": 0.0})
    assert robot.evaluate(robot.pairs[key(robot, "arm", "obstacle")])["collision"]
    robot.place({"swing": math.pi, "extension": 0.0})
    assert not robot.evaluate(robot.pairs[key(robot, "arm", "obstacle")])["collision"]


def test_deterministic_classification_and_witnesses(session):
    robot = session.robot
    first = analyze(robot, session.settings, {})
    second = analyze(robot, session.settings, {})
    assert first["stats"] == second["stats"]
    assert "Never" in first["stats"][key(robot, "base", "far")]["classifications"]
    assert "Always" in first["stats"][key(robot, "tip", "finger")]["classifications"]
    varying = first["stats"][key(robot, "arm", "obstacle")]
    assert "Default" in varying["classifications"]
    assert 0 < varying["collision_rate"] < 1
    robot.place(varying["collision_pose"])
    assert robot.evaluate(robot.pairs[key(robot, "arm", "obstacle")])["collision"]
    robot.place(varying["closest_pose"])
    assert (
        robot.evaluate(robot.pairs[key(robot, "arm", "obstacle")])["distance"]
        == varying["min_distance"]
    )


def test_exclusions_and_near_override(session):
    robot = session.robot
    session.settings["excluded"] = ["far"]
    first = analyze(robot, session.settings, {})["stats"]
    assert all(
        "far" not in (robot.pairs[k]["link1"], robot.pairs[k]["link2"]) for k in first
    )
    override = analyze(robot, session.settings, {k: 100.0 for k in robot.pairs})[
        "stats"
    ]
    assert all(s["near_rate"] == 1 for s in override.values())
    assert first[key(robot, "base", "arm")]["near_rate"] == 0


def test_locks_topk_and_invalidation(session):
    session.accept_analysis(analyze(session.robot, session.settings, {}))
    # Explicitly enable two sample-never pairs and ensure Top-K still limits them.
    ids = [key(session.robot, "base", "far"), key(session.robot, "arm", "far")]
    for k in ids:
        session.update_pair(k, {"manual": True})
    before = copy.deepcopy(session.choices)
    session.topk(1, True)
    assert len(session.selected()) == 1
    session.choices = session.undo
    session.undo = None
    assert session.choices == before
    for k in ids:
        session.update_pair(k, {"lock": "keep"})
    assert session.topk(1)["exceeds_k"]
    session.update_settings({"excluded": ["far"]})
    assert all(k not in session.selected() for k in ids)
    with pytest.raises(ValueError, match="待更新"):
        session.topk(1)


def test_threshold_invalidates_statistics(session):
    session.accept_analysis(analyze(session.robot, session.settings, {}))
    session.update_pair(next(iter(session.choices)), {"min_distance": 0.1})
    assert session.stale
    with pytest.raises(ValueError):
        session.update_pair(next(iter(session.choices)), {"min_distance": float("nan")})


def test_sampling_limits_and_benchmark(session):
    settings = copy.deepcopy(session.settings)
    settings["ranges"] = {"extension": {"locked": True, "value": 0.3}}
    result = analyze(session.robot, settings, {})
    assert all(s["closest_pose"]["extension"] == 0.3 for s in result["stats"].values())
    settings["ranges"] = {"extension": {"lower": -1, "upper": 1}}
    with pytest.raises(ValueError):
        validate_settings(session.robot, settings)
    b = benchmark(session.robot, session.settings, session.selected()[:2], 5)
    assert b["results"]["selected"]["link_pairs"] == 2
    assert b["results"]["all"]["link_pairs"] == len(session.robot.pairs)
    assert all(b["results"]["all"][m]["p95_ms"] > 0 for m in ["collision", "distance"])
    assert b["seed"] != session.settings["seed"]


def test_cancel(session):
    assert analyze(session.robot, session.settings, {}, cancelled=lambda: True) is None


def test_invalid_resources(tmp_path):
    path = tmp_path / "bad.urdf"
    path.write_text(
        '<robot name="bad"><link name="base"><collision><geometry><mesh filename="missing.stl"/></geometry></collision></link></robot>'
    )
    with pytest.raises(ValueError, match="找不到"):
        Robot(path)

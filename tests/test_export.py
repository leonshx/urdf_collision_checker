import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pinocchio as pin
import pytest
import yaml

from collision_checker.analysis import analyze
from collision_checker.export import (
    InfoDistanceConfirmation,
    export_files,
    read_enabled_yaml,
)


@pytest.fixture(scope="module")
def ocs2_reader(tmp_path_factory):
    if not shutil.which("g++") or not Path("/usr/include/eigen3/Eigen/Core").exists():
        pytest.skip(
            "OCS2 reader compatibility test requires g++, Boost and Eigen3 headers"
        )
    root = Path(__file__).parent
    binary = tmp_path_factory.mktemp("ocs2") / "read_info"
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-I/usr/include/eigen3",
            f"-I{root / 'vendor/ocs2'}",
            str(root / "ocs2_info_reader.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
    )
    return binary


def test_export_complement_and_ocs2_reader(session, tmp_path, ocs2_reader):
    session.accept_analysis(analyze(session.robot, session.settings, {}))
    # Keep some default/never pairs, including multiple geometries on one link.
    for key in list(session.choices)[:4]:
        session.update_pair(key, {"manual": True})
    export_files(session, tmp_path)
    srdf = ET.parse(tmp_path / "disabled_collisions.srdf")
    disabled = {
        session.robot.pair_key(e.get("link1"), e.get("link2"))
        for e in srdf.findall("disable_collisions")
    }
    assert disabled.isdisjoint(session.selected())
    assert disabled | set(session.selected()) == set(session.choices)
    document = yaml.safe_load((tmp_path / "self_collision.yaml").read_text())
    output = subprocess.check_output(
        [str(ocs2_reader), str(tmp_path / "self_collision.info")], text=True
    ).splitlines()
    assert output[0].split() == ["1", "0.05", "0.01", "0.001"]
    expected_links = {
        (p["link1"], p["link2"])
        for p in document["self_collision"]["collisionLinkPairs"]
    }
    raw = read_enabled_yaml(tmp_path / "self_collision.yaml")
    expected_objects = {(o["object1"], o["object2"]) for o in raw}
    marker = output.index("OBJECTS")
    assert {tuple(line.split(",")) for line in output[2:marker]} == expected_links
    assert {
        tuple(map(int, line.split(","))) for line in output[marker + 1 :]
    } == expected_objects
    # OCS2 adds explicit object pairs first and then expands link pairs.
    geometry = session.robot.geometry.copy()
    geometry.removeAllCollisionPairs()
    for i, j in expected_objects:
        geometry.addCollisionPair(pin.CollisionPair(i, j))
    for a, b in expected_links:
        for i in session.robot.by_link[a]:
            for j in session.robot.by_link[b]:
                geometry.addCollisionPair(pin.CollisionPair(i, j))
    assert len(geometry.collisionPairs) == len(expected_objects)


def test_yaml_inheritance_switches_and_info_confirmation(session, tmp_path):
    key = next(iter(session.choices))
    session.update_pair(key, {"min_distance": 0.08})
    with pytest.raises(InfoDistanceConfirmation):
        export_files(session, tmp_path)
    assert not list(tmp_path.iterdir())
    export_files(session, tmp_path, True)
    path = tmp_path / "self_collision.yaml"
    document = yaml.safe_load(path.read_text())
    raw = read_enabled_yaml(path)
    assert any(o["min_distance"] == 0.08 for o in raw)
    assert any(o["min_distance"] == 0.05 for o in raw)
    document["self_collision"]["collisionLinkPairs"][0]["enabled"] = False
    path.write_text(yaml.safe_dump(document))
    assert len(read_enabled_yaml(path)) < len(raw)
    document["self_collision"]["enabled"] = False
    path.write_text(yaml.safe_dump(document))
    assert read_enabled_yaml(path) == []
    document["self_collision"]["default_min_distance"] = float("nan")
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        read_enabled_yaml(path)


def test_global_switch_does_not_change_srdf(session, tmp_path, ocs2_reader):
    before = session.selected()
    session.update_settings({"enabled": False})
    export_files(session, tmp_path)
    assert session.selected() == before
    assert not ET.parse(tmp_path / "disabled_collisions.srdf").findall(
        "disable_collisions"
    )
    assert read_enabled_yaml(tmp_path / "self_collision.yaml") == []
    output = subprocess.check_output(
        [str(ocs2_reader), str(tmp_path / "self_collision.info")], text=True
    )
    assert output.startswith("0 0.05")


def test_empty_pairs_are_ocs2_readable(session, tmp_path, ocs2_reader):
    session.update_settings({"excluded": session.robot.collision_links})
    export_files(session, tmp_path)
    output = subprocess.check_output(
        [str(ocs2_reader), str(tmp_path / "self_collision.info")], text=True
    ).splitlines()
    assert output[1] == "0 0"
    assert read_enabled_yaml(tmp_path / "self_collision.yaml") == []

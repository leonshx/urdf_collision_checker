import xml.etree.ElementTree as ET

import pytest

from collision_checker.analysis import analyze, validate_settings
from collision_checker.model import Robot
from collision_checker.state import Session

# Original tiny triangle surface; sufficient for path/loader regression tests.
STL = """solid triangle
facet normal 0 0 1
outer loop
vertex 0 0 0
vertex 0.1 0 0
vertex 0 0.1 0
endloop
endfacet
endsolid triangle
"""


def test_package_mesh_origin_and_fingerprint(tmp_path):
    mesh = tmp_path / "triangle.stl"
    mesh.write_text(STL)
    urdf = tmp_path / "mesh.urdf"
    urdf.write_text(
        """<robot name="mesh_fixture"><link name="base"><collision><origin xyz="1 2 3" rpy="0.1 0.2 0.3"/><geometry><mesh filename="package://shapes/triangle.stl" scale="2 2 2"/></geometry></collision></link></robot>"""
    )
    with pytest.raises(ValueError, match="package"):
        Robot(urdf)
    robot = Robot(urdf, {"shapes": str(tmp_path)})
    assert len(robot.objects) == 1
    assert list(robot.geom_data.oMg[0].translation) == [1.0, 2.0, 3.0]
    assert "/api/assets/revision/" in robot.browser_xml("revision")
    assert 'name="__cc_object_0"' in robot.browser_xml("revision")
    fingerprint = robot.fingerprint
    mesh.write_text(STL.replace("0.1", "0.2"))
    assert Robot(urdf, {"shapes": str(tmp_path)}).fingerprint != fingerprint


def test_xacro_expansion(tmp_path):
    pytest.importorskip("xacro")
    path = tmp_path / "robot.xacro"
    path.write_text(
        """<robot name="xacro_fixture" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="radius" default="0.1"/><link name="base"><collision><geometry><sphere radius="$(arg radius)"/></geometry></collision></link></robot>"""
    )
    first = Robot(path, xacro_args={"radius": "0.2"})
    second = Robot(path, xacro_args={"radius": "0.3"})
    assert first.fingerprint != second.fingerprint
    assert first.geometry.geometryObjects[0].geometry.radius == pytest.approx(0.2)


def test_missing_joint_limit_can_be_supplied(tmp_path):
    path = tmp_path / "unbounded.urdf"
    path.write_text(
        """<robot name="unbounded"><link name="a"/><link name="b"/><joint name="slide" type="prismatic"><parent link="a"/><child link="b"/><limit effort="1" velocity="1"/></joint></robot>"""
    )
    robot = Robot(path)
    session = Session(robot, "test")
    with pytest.raises(ValueError, match="范围"):
        validate_settings(robot, session.settings)
    session.update_settings(
        {"samples": 2, "ranges": {"slide": {"lower": -1, "upper": 1}}}
    )
    assert analyze(robot, session.settings, {})["stats"] == {}


def test_invalid_geometry_rejected_before_native_parser(tmp_path):
    path = tmp_path / "bad.urdf"
    path.write_text(
        '<robot name="invalid"><link name="base"><collision><geometry><sphere radius="-1"/></geometry></collision></link></robot>'
    )
    with pytest.raises(ValueError, match="几何尺寸"):
        Robot(path)


def test_negative_mimic_limits_constrain_default_and_sampling(tmp_path):
    from conftest import FIXTURE

    tree = ET.fromstring(FIXTURE.read_text())
    joint = tree.find("joint[@name='finger_mimic']")
    joint.find("limit").set("lower", "0.2")
    joint.find("limit").set("upper", "0.25")
    joint.find("mimic").set("multiplier", "-0.5")
    joint.find("mimic").set("offset", "0.3")
    path = tmp_path / "negative_mimic.urdf"
    path.write_text(ET.tostring(tree, encoding="unicode"))
    robot = Robot(path)
    assert robot.default_pose["extension"] == pytest.approx(0.1)
    session = Session(robot, "test")
    session.settings["samples"] = 10
    result = analyze(robot, session.settings, {})
    for stat in result["stats"].values():
        assert 0.1 - 1e-9 <= stat["closest_pose"]["extension"] <= 0.2 + 1e-9

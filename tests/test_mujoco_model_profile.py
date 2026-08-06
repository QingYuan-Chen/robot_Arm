from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_physics_profile_marks_mesh_geoms_as_visual_only():
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML, build_physics_profile_tree

    if not DEFAULT_GRIPPER_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    tree = build_physics_profile_tree(DEFAULT_GRIPPER_XML)
    mesh_geoms = [geom for geom in tree.getroot().findall(".//geom") if geom.get("type") == "mesh"]

    assert mesh_geoms
    assert all(geom.get("contype") == "0" for geom in mesh_geoms)
    assert all(geom.get("conaffinity") == "0" for geom in mesh_geoms)
    assert all(geom.get("group") == "1" for geom in mesh_geoms)


def test_physics_profile_adds_simplified_collision_geoms():
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML, build_physics_profile_tree

    if not DEFAULT_GRIPPER_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    root = build_physics_profile_tree(DEFAULT_GRIPPER_XML).getroot()
    names = {geom.get("name") for geom in root.findall(".//geom")}

    assert "link2_collision" in names
    assert "link3_collision" in names
    assert "left_finger_pad_collision" in names
    assert "right_finger_pad_collision" in names


def test_physics_profile_uses_urdf_aligned_motor_ranges_and_forces():
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML, build_physics_profile_tree

    if not DEFAULT_GRIPPER_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    root = build_physics_profile_tree(DEFAULT_GRIPPER_XML).getroot()
    actuators = {
        actuator.get("joint"): actuator
        for actuator in root.findall(".//actuator/position")
    }

    assert actuators["joint1"].get("ctrlrange") == "-2.8 2.8"
    assert actuators["joint1"].get("forcerange") == "-27 27"
    assert actuators["joint1"].get("kp") == "270"
    assert actuators["joint4"].get("forcerange") == "-7 7"
    assert actuators["left_finger"].get("ctrlrange") == "0 0.045"
    assert actuators["left_finger"].get("forcerange") == "-20 20"


def test_write_profiles_reference_generated_robot_from_scene(tmp_path):
    from rebotarm_simulation.mujoco_model_profile import (
        DEFAULT_GRASP_SCENE_XML,
        DEFAULT_GRIPPER_XML,
        write_grasp_scene_profile,
        write_physics_profile,
    )

    if not DEFAULT_GRIPPER_XML.exists() or not DEFAULT_GRASP_SCENE_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    robot_xml = write_physics_profile(tmp_path / "robot.xml", DEFAULT_GRIPPER_XML, include_keyframes=False)
    scene_xml = write_grasp_scene_profile(tmp_path / "scene.xml", robot_xml, DEFAULT_GRASP_SCENE_XML)

    text = scene_xml.read_text(encoding="utf-8")
    assert str(robot_xml.resolve()) in text


def test_model_profile_detects_missing_absolute_asset_references(tmp_path):
    from rebotarm_simulation.mujoco_model_profile import xml_asset_references_are_readable

    stale_xml = tmp_path / "stale.xml"
    stale_xml.write_text(
        """<?xml version="1.0"?>
<mujoco>
  <asset>
    <mesh name="base_link" file="/definitely/missing/base_link.STL"/>
  </asset>
</mujoco>
""",
        encoding="utf-8",
    )

    assert xml_asset_references_are_readable(stale_xml) is False

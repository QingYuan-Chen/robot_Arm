from __future__ import annotations

from importlib.util import find_spec
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


pytestmark = pytest.mark.skipif(find_spec("mujoco") is None, reason="mujoco is not installed")


def test_generated_physics_profile_loads_and_steps(tmp_path):
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML, write_physics_profile
    from rebotarm_simulation.mujoco_runner import run_smoke

    if not DEFAULT_GRIPPER_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    robot_xml = write_physics_profile(tmp_path / "robot.xml", DEFAULT_GRIPPER_XML)
    result = run_smoke(robot_xml, seconds=1.0)

    assert result.nq == 8
    assert result.nu == 7
    assert result.finite


def test_generated_step_response_is_force_limited(tmp_path):
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML, write_physics_profile
    from rebotarm_simulation.mujoco_runner import run_step_response

    if not DEFAULT_GRIPPER_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    robot_xml = write_physics_profile(tmp_path / "robot.xml", DEFAULT_GRIPPER_XML)
    result = run_step_response(robot_xml, joint="joint2", target=-0.6, seconds=1.0)

    assert result.max_abs_actuator_force <= 27.0 + 1e-6
    assert result.max_abs_error >= 0.0


def test_generated_grasp_scene_loads_and_steps(tmp_path):
    from rebotarm_simulation.mujoco_model_profile import (
        DEFAULT_GRASP_SCENE_XML,
        DEFAULT_GRIPPER_XML,
        write_grasp_scene_profile,
        write_physics_profile,
    )
    from rebotarm_simulation.mujoco_runner import run_grasp_benchmark

    if not DEFAULT_GRIPPER_XML.exists() or not DEFAULT_GRASP_SCENE_XML.exists():
        pytest.skip("reference MuJoCo checkout is not available")

    robot_xml = write_physics_profile(tmp_path / "robot.xml", DEFAULT_GRIPPER_XML, include_keyframes=False)
    scene_xml = write_grasp_scene_profile(tmp_path / "scene.xml", robot_xml, DEFAULT_GRASP_SCENE_XML)
    result = run_grasp_benchmark(scene_xml, seconds=1.0)

    assert result.finite
    assert result.max_contacts >= 0

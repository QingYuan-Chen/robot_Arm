from pathlib import Path

import yaml


PACKAGE = Path("src/rebotarm_simulation")


def test_launch_is_explicitly_mujoco_headless_and_contains_no_hardware_path():
    source = (PACKAGE / "launch/mujoco_sim.launch.py").read_text().lower()
    assert "rebotarm_mujoco_node" in source
    assert 'backend"' in source and '"mujoco"' in source
    assert "headless" in source and "true" in source
    for forbidden in ("rebotarmcontroller", "use_hardware", "can", "serial"):
        assert forbidden not in source


def test_config_has_bounded_safe_defaults_and_no_hardware_keys():
    path = PACKAGE / "config/mujoco_sim.yaml"
    config = yaml.safe_load(path.read_text())
    params = config["rebotarm_mujoco_node"]["ros__parameters"]
    assert params["backend"] == "mujoco"
    assert params["headless"] is True
    assert params["max_trajectory_points"] == 10000
    assert params["max_trajectory_duration_sec"] == 300.0
    assert len(params["initial_joint_positions"]) == 6
    assert params["goal_position_tolerance"] == 0.02
    assert params["goal_velocity_tolerance"] == 0.05
    assert params["goal_time_tolerance_sec"] == 5.0
    assert params["feedback_rate_hz"] == 20.0
    lowered = path.read_text().lower()
    for forbidden in ("rebotarmcontroller", "use_hardware", "can_interface", "serial_port"):
        assert forbidden not in lowered

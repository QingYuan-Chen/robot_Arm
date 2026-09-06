from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = (
    ROOT
    / "src/rebotarm_bringup/launch/visual_grasp_hardware.launch.py"
)
PROFILE_PATH = (
    ROOT
    / "src/rebotarm_bringup/config/visual_grasp_hardware.yaml"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "visual_grasp_hardware_launch", LAUNCH_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hardware_profile_pins_single_runner_execution_owner() -> None:
    payload = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    arguments = payload["launch_arguments"]

    assert payload["schema_version"] == 1
    assert "channel" not in arguments
    assert arguments["use_hardware"] is True
    assert arguments["execution_mode"] == "plan_only"
    assert arguments["vision_profile"] == "ubuntu_native"
    assert arguments["graspnet_source_mode"] == "in_process"
    assert arguments["start_sim_trajectory_controller"] is False
    assert arguments["start_visual_grasp_executor"] is False
    assert arguments["start_motion_execution"] is True
    assert arguments["candidate_pose_policy"] == "base_axis"
    assert arguments["fixed_grasp_orientation_xyzw"] == [
        0.0,
        0.0,
        -0.707106781,
        0.707106781,
    ]
    assert arguments["tcp_offset_xyz"] == [-0.04, 0.0, 0.0]
    assert arguments["trajectory_precheck_enabled"] is True
    assert arguments["auto_retry_enabled"] is False
    assert arguments["safe_home_after_grasp"] is True
    assert arguments["gripper_position_torque_cap_nm"] == 1.5
    assert arguments["gripper_grasp_close_force"] == 1.0
    assert arguments["grasp_hold_timeout_sec"] == 120.0


def test_hardware_launch_requires_explicit_channel_and_has_no_old_overlay() -> None:
    text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("channel")' in text
    assert "default_value" not in text.split('DeclareLaunchArgument("channel")', 1)[0][-80:]
    assert "visual_grasp_system.launch.py" in text
    assert '"channel": LaunchConfiguration("channel")' in text
    assert "gripper-unified-bus-fix" not in text
    assert "install-fix" not in text
    assert "PYTHONPATH" not in text


def test_hardware_profile_resolves_workspace_assets() -> None:
    module = _load_launch_module()

    arguments = module._load_hardware_profile(PROFILE_PATH, anchor=LAUNCH_PATH)

    for name in (
        "vision_python_executable",
        "graspnet_python_executable",
        "graspnet_model_root",
        "graspnet_checkpoint_path",
    ):
        assert Path(arguments[name]).is_absolute()
        assert Path(arguments[name]).exists()
    assert arguments["vision_python_executable"].endswith(
        "/.venv-vision/bin/python"
    )
    assert arguments["graspnet_python_executable"].endswith(
        "/.venv-graspnet/bin/python"
    )
    assert arguments["use_hardware"] == "true"
    assert arguments["fixed_grasp_orientation_xyzw"] == (
        "[0.0,0.0,-0.707106781,0.707106781]"
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda data: data["launch_arguments"].update(channel="/dev/ttyACM0"), "channel"),
        (lambda data: data["launch_arguments"].update(use_hardware=False), "use_hardware"),
        (lambda data: data["launch_arguments"].update(execution_mode="execute"), "execution_mode"),
        (
            lambda data: data["launch_arguments"].update(start_visual_grasp_executor=True),
            "start_visual_grasp_executor",
        ),
        (
            lambda data: data["launch_arguments"].pop("trajectory_precheck_enabled"),
            "字段",
        ),
    ],
)
def test_hardware_profile_rejects_unsafe_or_incomplete_contract(
    tmp_path: Path, mutation, message: str,
) -> None:
    module = _load_launch_module()
    payload = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    mutation(payload)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(RuntimeError, match=message):
        module._load_hardware_profile(path, anchor=LAUNCH_PATH)

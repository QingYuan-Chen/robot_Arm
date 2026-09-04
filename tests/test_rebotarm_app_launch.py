from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_rebotarm_app_launch_exposes_simple_modes_and_profiles() -> None:
    launch_text = _read("src/rebotarm_bringup/launch/rebotarm_app.launch.py")

    assert 'DeclareLaunchArgument("use_hardware", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("web_execute_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("profile"' not in launch_text
    assert 'DeclareLaunchArgument("name", default_value="teach_record")' in launch_text
    assert "moveit_hardware.launch.py" in launch_text
    assert 'executable="TeachRecorderNode"' not in launch_text
    assert 'executable="TeachReplayNode"' not in launch_text
    assert "_idle_recorder_node(" not in launch_text
    assert "Teach Trajectory card" in launch_text
    assert 'DeclareLaunchArgument("mode"' not in launch_text
    assert 'LaunchConfiguration("mode")' not in launch_text
    assert 'if mode ==' not in launch_text
    assert "mode must be one of" not in launch_text
    assert "teleop_system.launch.py" not in launch_text
    assert "def _as_bool" in launch_text
    assert '"use_hardware": _as_bool(use_hardware)' in launch_text
    assert '"web_execute_enabled": _as_bool(web_execute_enabled)' in launch_text
    assert '"panel_mode": panel_mode' in launch_text
    assert 'panel_mode="control"' in launch_text
    assert "full MoveIt + web teleop workbench" in launch_text
    assert 'DeclareLaunchArgument("channel", default_value="auto")' in launch_text
    assert 'DeclareLaunchArgument("execution_mode", default_value="execute")' in launch_text
    assert "def _resolve_channel" in launch_text
    assert 'for candidate in ("/dev/ttyACM0", "/dev/ttyACM1")' in launch_text
    assert '"use_rviz": "false"' in launch_text
    assert 'arguments=["-d", web_rviz_config]' in launch_text
    assert '"web_teleop_status.rviz"' in launch_text
    assert "_keyboard_node(" in launch_text
    assert 'safe_name = os.path.basename(safe_name.replace("\\\\", "/")) or "teach_record"' in launch_text
    assert 'return f"teleop_records/{safe_name}"' in launch_text


def test_web_teleop_rviz_is_robot_status_only() -> None:
    rviz_text = _read("src/rebotarm_bringup/rviz/web_teleop_status.rviz")

    assert "rviz_default_plugins/RobotModel" in rviz_text
    assert "rviz_default_plugins/TF" in rviz_text
    assert "rviz_default_plugins/Interact" in rviz_text
    assert "rviz_default_plugins/MoveCamera" not in rviz_text
    assert "rviz_default_plugins/Select" not in rviz_text
    assert "moveit_rviz_plugin/MotionPlanning" not in rviz_text
    assert "InteractiveMarkers" not in rviz_text
    assert "EndEffectorTarget" not in rviz_text


def test_replay_profiles_keep_safe_defaults_in_config() -> None:
    profiles = yaml.safe_load(
        _read("src/rebotarm_bringup/config/replay_profiles.yaml")
    )

    assert profiles["default_profile"] == "safe"
    assert {"safe", "normal", "large"}.issubset(profiles["profiles"])

    safe = profiles["profiles"]["safe"]
    assert safe["dry_run"] is False
    assert safe["speed"] <= 0.2
    assert safe["collision_check_enabled"] is True
    assert safe["use_moveit_start_align"] is True
    assert safe["max_replay_velocity_rad_s"] <= 3.0
    assert safe["max_replay_acceleration_rad_s2"] <= 5.0
    assert safe["max_replay_jerk_rad_s3"] <= 30.0

    large = profiles["profiles"]["large"]
    assert large["speed"] <= profiles["profiles"]["normal"]["speed"]
    assert large["large_motion_max_speed"] <= 1.0


def test_teach_recording_uses_higher_sampling_defaults() -> None:
    teleop_config = yaml.safe_load(
        _read("src/rebotarm_interactive_control/config/teleop_control.yaml")
    )
    params = teleop_config["/**"]["ros__parameters"]

    assert params["sample_rate_hz"] == 150.0
    assert params["filter_sample_rate_hz"] == 150.0
    assert params["resample_rate_hz"] == 150.0

    for launch_path in (
        "src/rebotarm_bringup/launch/moveit_hardware.launch.py",
        "src/rebotarm_bringup/launch/driver_only.launch.py",
        "src/rebotarm_bringup/launch/interactive_system.launch.py",
        "src/rebotarm_bringup/launch/bringup.launch.py",
        "src/rebotarm_bringup/launch/interactive_basic.launch.py",
        "src/rebotarm_bringup/launch/teleop_keyboard.launch.py",
    ):
        launch_text = _read(launch_path)
        assert 'DeclareLaunchArgument("joint_state_rate", default_value="100.0")' in launch_text

    driver_params = yaml.safe_load(_read("src/rebotarm_bringup/config/driver_params.yaml"))
    assert driver_params["reBotArmController"]["ros__parameters"]["joint_state_rate"] == 100.0


def test_common_commands_document_recommends_one_entrypoint() -> None:
    doc = _read("docs/rebotarm_common_commands.md")

    assert "rebotarm_app.launch.py" in doc
    assert "mode:=" not in doc
    assert "channel:=auto" in doc
    assert "reBotArm 遥操作使用文档" in doc
    assert "网页遥操作" in doc
    assert "键盘遥操作" in doc
    assert "重力补偿手拖示教录制" in doc
    assert "Ctrl+C" in doc
    assert "safe_home" in doc


def test_feature_commands_document_web_teleop_next_to_rviz_drag() -> None:
    doc = _read("docs/rebotarm_feature_commands.md")

    assert "## RViz MoveIt 末端拖动" in doc
    assert "## 网页遥操作" in doc
    assert "ros2 launch rebotarm_bringup rviz_ee_drag_real.launch.py" in doc
    assert "ros2 launch rebotarm_bringup rebotarm_app.launch.py" in doc
    assert "channel:=auto" in doc
    assert "网页关节 Preview / Execute / Stop" in doc

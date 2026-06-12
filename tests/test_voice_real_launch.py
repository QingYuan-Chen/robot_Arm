from __future__ import annotations

from pathlib import Path

ROS2_ROOT = Path(__file__).resolve().parents[1]
PKG = ROS2_ROOT / "src" / "rebotarm_voice_control"


def test_voice_real_launch_sets_execution_mode_to_real():
    launch_text = (PKG / "launch" / "voice_real.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("execution_mode", default_value="real")' in launch_text
    assert 'executable="rebotarm_voice_control_node"' in launch_text
    assert '"execution_mode": execution_mode' in launch_text


def test_voice_real_launch_does_not_enable_real_calls_by_itself():
    safety_text = (PKG / "config" / "safety_limits.yaml").read_text(encoding="utf-8")

    assert 'allow_real_ros_calls: false' in safety_text

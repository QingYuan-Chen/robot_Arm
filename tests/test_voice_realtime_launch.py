from __future__ import annotations

from pathlib import Path

ROS2_ROOT = Path(__file__).resolve().parents[1]
PKG = ROS2_ROOT / "src" / "rebotarm_voice_control"


def test_voice_realtime_launch_uses_gateway_entrypoint():
    launch_text = (PKG / "launch" / "voice_realtime.launch.py").read_text(encoding="utf-8")

    assert 'executable="rebotarm_realtime_gateway"' in launch_text
    assert 'DeclareLaunchArgument("execution_mode"' in launch_text
    assert 'DeclareLaunchArgument("event_jsonl"' in launch_text


def test_setup_registers_realtime_gateway_console_script():
    setup_text = (PKG / "setup.py").read_text(encoding="utf-8")

    assert "rebotarm_realtime_gateway = rebotarm_voice_control.realtime_voice_gateway_node:main" in setup_text

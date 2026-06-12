from __future__ import annotations

from pathlib import Path

ROS2_ROOT = Path(__file__).resolve().parents[1]
PKG = ROS2_ROOT / "src" / "rebotarm_voice_control"


def test_voice_sim_launch_sets_execution_mode_to_sim():
    launch_text = (PKG / "launch" / "voice_sim.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("execution_mode", default_value="sim")' in launch_text
    assert 'executable="rebotarm_voice_control_node"' in launch_text
    assert 'executable="rebotarm_sim_move_relative_action"' in launch_text
    assert '"execution_mode": execution_mode' in launch_text


def test_setup_registers_sim_executor_console_script():
    setup_text = (PKG / "setup.py").read_text(encoding="utf-8")

    assert "rebotarm_sim_executor = rebotarm_voice_control.sim_executor:main" in setup_text

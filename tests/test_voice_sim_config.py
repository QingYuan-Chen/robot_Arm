from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_sim_config


def test_load_sim_config_defaults_to_recorded_backend():
    config = load_sim_config(SRC / "config")

    assert config["backend"] == "recorded"
    assert config["moveit2"]["move_relative_action"] == "/rebotarm/sim/move_relative"
    assert config["moveit2"]["execute_trajectory_action"] == "/execute_trajectory"

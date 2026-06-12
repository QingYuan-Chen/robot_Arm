from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.models import SafetyViolationError
from rebotarm_voice_control.sim_move_relative_action_node import validate_move_relative_goal


class _Goal:
    def __init__(self, axis="z", distance_m=0.05, frame_id="base_link", speed_scale=0.2):
        self.axis = axis
        self.distance_m = distance_m
        self.frame_id = frame_id
        self.speed_scale = speed_scale


def test_validate_move_relative_goal_accepts_safe_goal():
    result = validate_move_relative_goal(_Goal())

    assert result == {
        "axis": "z",
        "distance_m": 0.05,
        "frame_id": "base_link",
        "speed_scale": 0.2,
    }


def test_validate_move_relative_goal_rejects_large_distance():
    with pytest.raises(SafetyViolationError, match="distance exceeds"):
        validate_move_relative_goal(_Goal(distance_m=0.08))


def test_validate_move_relative_goal_rejects_invalid_axis():
    with pytest.raises(SafetyViolationError, match="axis must be"):
        validate_move_relative_goal(_Goal(axis="yaw"))


def test_setup_registers_sim_move_relative_action_node():
    setup_text = (SRC / "setup.py").read_text(encoding="utf-8")

    assert "rebotarm_sim_move_relative_action = rebotarm_voice_control.sim_move_relative_action_node:main" in setup_text


def test_sim_move_relative_node_file_exists():
    assert (SRC / "rebotarm_voice_control" / "sim_move_relative_action_node.py").exists()

from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.sim_action_bindings import (
    build_move_relative_goal,
    build_sim_goal,
    resolve_sim_action_type,
)


class _MoveRelative:
    class Goal:
        def __init__(self):
            self.axis = ""
            self.distance_m = 0.0
            self.frame_id = ""
            self.speed_scale = 0.0


def test_move_relative_action_is_registered_in_rebotarm_msgs():
    cmake_text = (ROS2_ROOT / "src" / "rebotarm_msgs" / "CMakeLists.txt").read_text(encoding="utf-8")
    action_text = (
        ROS2_ROOT / "src" / "rebotarm_msgs" / "action" / "MoveRelative.action"
    ).read_text(encoding="utf-8")

    assert '"action/MoveRelative.action"' in cmake_text
    assert "string axis" in action_text
    assert "float64 distance_m" in action_text
    assert "string frame_id" in action_text
    assert "float64 speed_scale" in action_text


def test_build_move_relative_goal_uses_safe_defaults():
    goal = build_move_relative_goal(
        _MoveRelative,
        {"axis": "z", "distance_m": 0.05},
    )

    assert goal.axis == "z"
    assert goal.distance_m == 0.05
    assert goal.frame_id == "base_link"
    assert goal.speed_scale == 0.2


def test_move_relative_is_resolved_and_built_from_sim_action_name():
    action_type = resolve_sim_action_type(
        "/rebotarm/sim/move_relative",
        move_relative_type=_MoveRelative,
    )
    goal = build_sim_goal(
        "/rebotarm/sim/move_relative",
        {"axis": "x", "distance_m": 0.02, "frame_id": "tool0", "speed_scale": 0.1},
        move_relative_type=_MoveRelative,
    )

    assert action_type is _MoveRelative
    assert isinstance(goal, _MoveRelative.Goal)
    assert goal.axis == "x"
    assert goal.frame_id == "tool0"

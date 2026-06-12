from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.models import SafetyViolationError
from rebotarm_voice_control.sim_action_bindings import (
    build_execute_grasp_goal,
    build_move_to_pose_goal,
    build_sim_goal,
    resolve_sim_action_type,
)


class _Pose:
    def __init__(self):
        self.position = type("Position", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.orientation = type(
            "Orientation",
            (),
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        )()


class _MoveToPose:
    class Goal:
        def __init__(self):
            self.target_pose = None
            self.duration = 0.0


class _ExecuteGrasp:
    class Goal:
        def __init__(self):
            self.target_label = ""
            self.target_pose = None
            self.use_label = False
            self.use_pose = False


def _pose_factory():
    return _Pose()


def test_build_move_to_pose_goal_from_named_pose_dict():
    goal = build_move_to_pose_goal(
        _MoveToPose,
        _pose_factory,
        {
            "intent": "move_named_pose",
            "pose": {
                "position": {"x": 0.2, "y": 0.0, "z": 0.3},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "duration": 2.5,
        },
    )

    assert goal.duration == 2.5
    assert goal.target_pose.position.x == 0.2
    assert goal.target_pose.position.z == 0.3
    assert goal.target_pose.orientation.w == 1.0


def test_build_execute_grasp_goal_from_label():
    goal = build_execute_grasp_goal(
        _ExecuteGrasp,
        _pose_factory,
        {"intent": "pick_object", "label": "red_block"},
    )

    assert goal.target_label == "red_block"
    assert goal.use_label is True
    assert goal.use_pose is False


def test_build_execute_grasp_goal_from_pose():
    goal = build_execute_grasp_goal(
        _ExecuteGrasp,
        _pose_factory,
        {
            "intent": "place_object",
            "pose": {
                "position": {"x": 0.25, "y": 0.1, "z": 0.2},
                "orientation": {"w": 1.0},
            },
        },
    )

    assert goal.use_pose is True
    assert goal.target_pose.position.y == 0.1


def test_build_sim_goal_dispatches_by_action_name():
    goal = build_sim_goal(
        "/rebotarm/sim/move_to_pose",
        {"pose": {"position": {"x": 0.2, "y": 0.0, "z": 0.3}}},
        move_to_pose_type=_MoveToPose,
        execute_grasp_type=_ExecuteGrasp,
        pose_factory=_pose_factory,
    )

    assert isinstance(goal, _MoveToPose.Goal)


def test_resolve_sim_action_type_rejects_unknown_action():
    with pytest.raises(SafetyViolationError, match="unsupported sim action"):
        resolve_sim_action_type(
            "/rebotarm/sim/unknown",
            move_to_pose_type=_MoveToPose,
            execute_grasp_type=_ExecuteGrasp,
        )

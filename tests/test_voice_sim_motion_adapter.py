from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.sim_motion_adapter import (
    MoveRelativePlanResult,
    MoveRelativeSimMotionAdapter,
    build_relative_pose_target,
)


class _FakePlanner:
    def __init__(self, success=True):
        self.success = success
        self.calls = []

    def plan_preview(self, preview):
        self.calls.append((preview.pose_target, getattr(preview, "speed_scale", None)))
        pose_target = preview.pose_target
        return MoveRelativePlanResult(
            success=self.success,
            message="planned",
            trajectory={"joint_trajectory": {"points": [1, 2, 3]}},
            final_pose=pose_target,
        )


class _FakeTrajectoryClient:
    def __init__(self):
        self.calls = []

    def send_goal_async(self, goal_msg):
        self.calls.append(goal_msg)
        class _Future:
            def result(self_inner):
                return type("GoalHandle", (), {"accepted": True})()
        return _Future()


def _build_goal(trajectory, speed_scale):
    return {"trajectory": trajectory, "speed_scale": speed_scale}


def test_build_relative_pose_target_uses_axis_and_frame():
    pose = build_relative_pose_target(
        axis="z",
        distance_m=0.05,
        frame_id="base_link",
        current_pose={"x": 0.2, "y": 0.1, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    )

    assert pose.z == 0.35
    assert pose.x == 0.2
    assert pose.y == 0.1


def test_move_relative_adapter_plans_and_builds_trajectory_goal():
    planner = _FakePlanner()
    trajectory_client = _FakeTrajectoryClient()
    adapter = MoveRelativeSimMotionAdapter(
        planner=planner,
        trajectory_client=trajectory_client,
        current_pose_supplier=lambda: {"x": 0.2, "y": 0.1, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        goal_builder=_build_goal,
    )

    result = adapter.execute_move_relative(
        axis="z",
        distance_m=0.05,
        frame_id="base_link",
        speed_scale=0.2,
    )

    assert result.success is True
    assert result.message == "planned and dispatched to simulation controller"
    assert planner.calls[0][0].z == 0.35
    assert len(trajectory_client.calls) == 1
    assert trajectory_client.calls[0]["speed_scale"] == 0.2


def test_move_relative_adapter_rejects_failed_plan():
    planner = _FakePlanner(success=False)
    trajectory_client = _FakeTrajectoryClient()
    adapter = MoveRelativeSimMotionAdapter(
        planner=planner,
        trajectory_client=trajectory_client,
        current_pose_supplier=lambda: {"x": 0.2, "y": 0.1, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        goal_builder=_build_goal,
    )

    result = adapter.execute_move_relative(
        axis="z",
        distance_m=0.05,
        frame_id="base_link",
        speed_scale=0.2,
    )

    assert result.success is False
    assert result.message == "planned"
    assert len(trajectory_client.calls) == 0

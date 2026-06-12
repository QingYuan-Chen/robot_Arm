from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from rebotarm_motion.command_models import PoseTarget

from .models import SafetyViolationError


@dataclass(frozen=True)
class MoveRelativePlanResult:
    success: bool
    message: str
    trajectory: Any | None
    final_pose: PoseTarget | None


class TrajectoryGoalSender(Protocol):
    def send_goal_async(self, goal_msg: Any) -> Any:
        """Send a trajectory goal to the sim controller."""


def build_relative_pose_target(
    *,
    axis: str,
    distance_m: float,
    frame_id: str,
    current_pose: dict[str, float],
) -> PoseTarget:
    if axis not in {"x", "y", "z"}:
        raise SafetyViolationError("move_relative axis must be x, y, or z")
    if frame_id and frame_id not in {"base_link", "end_link", "tool0"}:
        raise SafetyViolationError("move_relative frame_id must be base_link, end_link, or tool0")

    base = PoseTarget(
        x=float(current_pose["x"]),
        y=float(current_pose["y"]),
        z=float(current_pose["z"]),
        roll=float(current_pose.get("roll", 0.0)),
        pitch=float(current_pose.get("pitch", 0.0)),
        yaw=float(current_pose.get("yaw", 0.0)),
    )
    dx, dy, dz = _axis_offset(axis, float(distance_m))
    return PoseTarget(
        x=base.x + dx,
        y=base.y + dy,
        z=base.z + dz,
        roll=base.roll,
        pitch=base.pitch,
        yaw=base.yaw,
    )


class MoveRelativeSimMotionAdapter:
    def __init__(
        self,
        *,
        planner: Any,
        trajectory_client: TrajectoryGoalSender,
        current_pose_supplier: Any,
        goal_builder: Any,
        future_waiter: Any | None = None,
    ) -> None:
        self._planner = planner
        self._trajectory_client = trajectory_client
        self._current_pose_supplier = current_pose_supplier
        self._goal_builder = goal_builder
        self._future_waiter = future_waiter

    def execute_move_relative(
        self,
        *,
        axis: str,
        distance_m: float,
        frame_id: str,
        speed_scale: float,
    ) -> MoveRelativePlanResult:
        current_pose = self._current_pose_supplier()
        pose_target = build_relative_pose_target(
            axis=axis,
            distance_m=distance_m,
            frame_id=frame_id,
            current_pose=current_pose,
        )
        preview = type(
            "Preview",
            (),
            {"pose_target": pose_target, "speed_scale": float(speed_scale)},
        )()
        plan_result = self._planner.plan_preview(preview)
        if not plan_result.success or plan_result.trajectory is None:
            return MoveRelativePlanResult(
                success=False,
                message=plan_result.message,
                trajectory=None,
                final_pose=pose_target,
            )

        goal_msg = self._goal_builder(plan_result.trajectory, float(speed_scale))
        future = self._trajectory_client.send_goal_async(goal_msg)
        if self._future_waiter is not None:
            self._future_waiter(future, timeout_sec=5.0)
        goal_handle = future.result()
        if not bool(getattr(goal_handle, "accepted", False)):
            return MoveRelativePlanResult(
                success=False,
                message="sim trajectory goal rejected",
                trajectory=plan_result.trajectory,
                final_pose=pose_target,
            )
        return MoveRelativePlanResult(
            success=True,
            message="planned and dispatched to simulation controller",
            trajectory=plan_result.trajectory,
            final_pose=pose_target,
        )


def _axis_offset(axis: str, distance_m: float) -> tuple[float, float, float]:
    if axis == "x":
        return distance_m, 0.0, 0.0
    if axis == "y":
        return 0.0, distance_m, 0.0
    return 0.0, 0.0, distance_m


def build_follow_joint_trajectory_goal(trajectory: Any, speed_scale: float) -> Any:
    try:
        from control_msgs.action import FollowJointTrajectory
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("control_msgs is required for sim trajectory dispatch") from exc

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance.sec = 0
    goal.goal_time_tolerance.nanosec = 0
    scale = 1.0 / max(float(speed_scale), 0.05)
    for point in getattr(goal.trajectory, "points", []):
        duration = getattr(point, "time_from_start", None)
        if duration is not None:
            total_ns = int((int(duration.sec) * 1_000_000_000 + int(duration.nanosec)) * scale)
            duration.sec = total_ns // 1_000_000_000
            duration.nanosec = total_ns % 1_000_000_000
    return goal

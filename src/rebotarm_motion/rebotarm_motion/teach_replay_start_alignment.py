from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


def set_duration(duration_msg: Any, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


@dataclass(frozen=True)
class MoveItStartAlignmentConfig:
    start_hold_sec: float
    first_hold_sec: float
    skip_threshold: float
    joint_goal_tolerance: float
    velocity_scaling: float
    acceleration_scaling: float


class MoveItStartAligner:
    """Append a MoveIt-planned start alignment segment to a trajectory."""

    def __init__(
        self,
        *,
        planner: Any,
        trajectory_point_factory: Callable[[], Any],
    ) -> None:
        self._planner = planner
        self._trajectory_point_factory = trajectory_point_factory

    def append(
        self,
        trajectory: Any,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
        config: MoveItStartAlignmentConfig,
    ) -> float:
        elapsed = max(float(config.start_hold_sec), 0.0)
        hold_point = self._trajectory_point_factory()
        hold_point.positions = [float(v) for v in current_positions]
        hold_point.velocities = [0.0 for _ in current_positions]
        set_duration(hold_point.time_from_start, elapsed)
        trajectory.points.append(hold_point)
        max_error = max(
            (abs(float(a) - float(b)) for a, b in zip(current_positions, first_positions)),
            default=0.0,
        )
        if max_error >= float(config.skip_threshold):
            elapsed = self._append_plan_points(
                trajectory,
                first_positions=first_positions,
                elapsed=elapsed,
                config=config,
            )
        first_hold = max(float(config.first_hold_sec), 0.0)
        if first_hold > 0.0:
            elapsed += first_hold
            first_point = self._trajectory_point_factory()
            first_point.positions = [float(v) for v in first_positions]
            first_point.velocities = [0.0 for _ in first_positions]
            set_duration(first_point.time_from_start, elapsed)
            trajectory.points.append(first_point)
        return elapsed

    def _append_plan_points(
        self,
        trajectory: Any,
        *,
        first_positions: tuple[float, ...],
        elapsed: float,
        config: MoveItStartAlignmentConfig,
    ) -> float:
        plan = self._planner.plan_joint_positions(
            joint_names=tuple(trajectory.joint_names),
            target_positions=first_positions,
            tolerance=float(config.joint_goal_tolerance),
            velocity_scaling=float(config.velocity_scaling),
            acceleration_scaling=float(config.acceleration_scaling),
        )
        if not plan.success or plan.trajectory is None:
            raise ValueError(f"moveit start alignment failed: {plan.message}")
        source_names = list(getattr(plan.trajectory, "joint_names", []))
        index_by_name = {name: index for index, name in enumerate(source_names)}
        missing = [name for name in trajectory.joint_names if name not in index_by_name]
        if missing:
            raise ValueError(f"moveit start alignment missing joints: {', '.join(missing)}")
        for source_point in getattr(plan.trajectory, "points", []):
            source_time = (
                float(source_point.time_from_start.sec)
                + float(source_point.time_from_start.nanosec) * 1e-9
            )
            point = self._trajectory_point_factory()
            point.positions = [
                float(source_point.positions[index_by_name[name]])
                for name in trajectory.joint_names
            ]
            if getattr(source_point, "velocities", None):
                point.velocities = [
                    float(source_point.velocities[index_by_name[name]])
                    for name in trajectory.joint_names
                ]
            set_duration(point.time_from_start, elapsed + source_time)
            trajectory.points.append(point)
        if trajectory.points:
            last = trajectory.points[-1].time_from_start
            elapsed = float(last.sec) + float(last.nanosec) * 1e-9
        return elapsed

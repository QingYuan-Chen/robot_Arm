from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .teach_recording import (
    build_replay_start_soft_points,
    retime_teach_samples,
)


def set_duration(duration_msg: Any, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


@dataclass(frozen=True)
class TeachReplayTrajectoryConfig:
    use_moveit_start_align: bool
    start_hold_sec: float
    soft_start_duration: float
    soft_start_steps: int
    first_hold_sec: float
    yellow_max_speed: float
    initial_replay_delay_sec: float
    max_velocity_rad_s: Any
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float


@dataclass(frozen=True)
class TeachReplayTrajectoryResult:
    trajectory: Any


class TeachReplayTrajectoryBuilder:
    """Build FollowJointTrajectory-compatible teach replay trajectories."""

    def __init__(
        self,
        *,
        trajectory_factory: Callable[[], Any],
        trajectory_point_factory: Callable[[], Any],
    ) -> None:
        self._trajectory_factory = trajectory_factory
        self._trajectory_point_factory = trajectory_point_factory

    def build(
        self,
        *,
        prepared: Any,
        current_positions: dict[str, float],
        start_band: str,
        settings: dict[str, float | int],
        config: TeachReplayTrajectoryConfig,
        moveit_start_alignment: Callable[..., float] | None = None,
    ) -> TeachReplayTrajectoryResult:
        replay_samples = list(prepared.samples)
        if not replay_samples:
            raise ValueError("record contains no samples")
        first = replay_samples[0]
        trajectory = self._trajectory_factory()
        trajectory.joint_names = list(first.joint_names)
        current = tuple(
            float(current_positions.get(name, start))
            for name, start in zip(first.joint_names, first.positions)
        )
        if config.use_moveit_start_align:
            if moveit_start_alignment is None:
                raise ValueError("MoveIt start alignment callback is required")
            elapsed = float(
                moveit_start_alignment(
                    trajectory,
                    current_positions=current,
                    first_positions=tuple(first.positions),
                )
            )
        else:
            elapsed = self._append_soft_start(
                trajectory,
                current_positions=current,
                first_positions=tuple(first.positions),
                start_band=start_band,
                settings=settings,
                config=config,
            )
        self._append_replay_points(trajectory, prepared=prepared, elapsed=elapsed, config=config)
        self.append_final_hold(trajectory, final_hold_sec=float(settings["final_hold_sec"]))
        return TeachReplayTrajectoryResult(trajectory=trajectory)

    def _append_soft_start(
        self,
        trajectory: Any,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
        start_band: str,
        settings: dict[str, float | int],
        config: TeachReplayTrajectoryConfig,
    ) -> float:
        start_points = build_replay_start_soft_points(
            current_positions=current_positions,
            first_positions=first_positions,
            start_band=start_band,
            start_hold_sec=float(config.start_hold_sec),
            soft_start_duration=float(config.soft_start_duration),
            soft_start_steps=int(config.soft_start_steps),
            align_duration=float(settings["align_duration"]),
            align_steps=int(settings["align_steps"]),
            first_hold_sec=float(config.first_hold_sec),
        )
        for start_point in start_points:
            point = self._trajectory_point_factory()
            point.positions = [float(v) for v in start_point.positions]
            point.velocities = [0.0 for _ in start_point.positions]
            set_duration(point.time_from_start, start_point.time_from_start)
            trajectory.points.append(point)
        return float(start_points[-1].time_from_start) if start_points else 0.0

    def _append_replay_points(
        self,
        trajectory: Any,
        *,
        prepared: Any,
        elapsed: float,
        config: TeachReplayTrajectoryConfig,
    ) -> None:
        speed = max(float(prepared.effective_replay_speed), 0.01)
        if str(prepared.after_quality.risk_level) == "yellow":
            speed = min(speed, float(config.yellow_max_speed))
        if prepared.retimed_points:
            initial_delay = max(float(config.initial_replay_delay_sec), 0.0)
            retimed_points = prepared.retimed_points
        else:
            initial_delay = 0.0
            retimed_points = retime_teach_samples(
                prepared.samples,
                replay_speed=speed,
                max_velocity_rad_s=config.max_velocity_rad_s,
                max_acceleration_rad_s2=float(config.max_acceleration_rad_s2),
                max_jerk_rad_s3=float(config.max_jerk_rad_s3),
                initial_delay_sec=float(config.initial_replay_delay_sec),
                boundary_zero_velocity=True,
            )
        for retimed in retimed_points:
            point = self._trajectory_point_factory()
            point.positions = [float(v) for v in retimed.positions]
            point.velocities = (
                [float(v) for v in retimed.velocities]
                if getattr(retimed, "velocities", None)
                else [0.0 for _ in point.positions]
            )
            set_duration(point.time_from_start, float(elapsed) + initial_delay + float(retimed.time_from_start))
            trajectory.points.append(point)

    def append_final_hold(self, trajectory: Any, *, final_hold_sec: float) -> None:
        final_hold = max(float(final_hold_sec), 0.0)
        if final_hold <= 0.0 or not trajectory.points:
            return
        last_point = trajectory.points[-1]
        last_time = float(last_point.time_from_start.sec) + float(last_point.time_from_start.nanosec) * 1e-9
        hold_point = self._trajectory_point_factory()
        hold_point.positions = [float(v) for v in last_point.positions]
        hold_point.velocities = [0.0 for _ in hold_point.positions]
        set_duration(hold_point.time_from_start, last_time + final_hold)
        trajectory.points.append(hold_point)

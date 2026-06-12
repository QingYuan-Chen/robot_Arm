from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class ReplayTrackingResult:
    ok: bool
    reason: str
    message: str
    worst_joint: str = ""
    max_tracking_error_rad: float = 0.0
    max_live_velocity_rad_s: float = 0.0


def _duration_to_sec(value) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    return float(getattr(value, "sec", 0)) + float(getattr(value, "nanosec", 0)) * 1e-9


def _point_time(point) -> float:
    if isinstance(point, dict):
        return _duration_to_sec(point.get("time_from_start", 0.0))
    return _duration_to_sec(getattr(point, "time_from_start", 0.0))


def _point_positions(point) -> tuple[float, ...]:
    if isinstance(point, dict):
        return tuple(float(v) for v in point.get("positions", ()))
    return tuple(float(v) for v in getattr(point, "positions", ()))


def _trajectory_joint_names(trajectory) -> tuple[str, ...]:
    if isinstance(trajectory, dict):
        return tuple(str(v) for v in trajectory.get("joint_names", ()))
    return tuple(str(v) for v in getattr(trajectory, "joint_names", ()))


def _trajectory_points(trajectory) -> list:
    if isinstance(trajectory, dict):
        return list(trajectory.get("points", ()))
    return list(getattr(trajectory, "points", ()))


def expected_positions_at(trajectory, elapsed_sec: float) -> tuple[float, ...]:
    points = _trajectory_points(trajectory)
    if not points:
        return ()
    elapsed = max(float(elapsed_sec), 0.0)
    first = points[0]
    first_time = _point_time(first)
    if elapsed <= first_time:
        return _point_positions(first)
    previous = first
    for current in points[1:]:
        current_time = _point_time(current)
        if elapsed <= current_time:
            previous_time = _point_time(previous)
            span = max(current_time - previous_time, 1e-9)
            ratio = min(max((elapsed - previous_time) / span, 0.0), 1.0)
            previous_positions = _point_positions(previous)
            current_positions = _point_positions(current)
            return tuple(
                float(a) + (float(b) - float(a)) * ratio
                for a, b in zip(previous_positions, current_positions)
            )
        previous = current
    return _point_positions(points[-1])


def evaluate_replay_tracking(
    trajectory,
    *,
    joint_names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float] = (),
    elapsed_sec: float,
    max_tracking_error_rad: float,
    max_live_velocity_rad_s: float,
) -> ReplayTrackingResult:
    trajectory_names = _trajectory_joint_names(trajectory)
    expected = expected_positions_at(trajectory, elapsed_sec)
    if not trajectory_names or not expected:
        return ReplayTrackingResult(False, "missing_trajectory", "active replay trajectory is empty")

    actual_by_name = {
        str(name): float(position)
        for name, position in zip(joint_names, positions)
    }
    missing = [name for name in trajectory_names if name not in actual_by_name]
    if missing:
        return ReplayTrackingResult(
            False,
            "missing_joint_state",
            f"joint_states missing replay joints: {', '.join(missing)}",
        )

    worst_joint = ""
    max_error = 0.0
    for name, target in zip(trajectory_names, expected):
        error = abs(float(actual_by_name[name]) - float(target))
        if error > max_error:
            max_error = error
            worst_joint = str(name)
    if max_error > float(max_tracking_error_rad):
        return ReplayTrackingResult(
            False,
            "tracking_error",
            (
                f"{worst_joint} tracking error {max_error:.4f} rad > "
                f"{float(max_tracking_error_rad):.4f} rad"
            ),
            worst_joint=worst_joint,
            max_tracking_error_rad=max_error,
        )

    if velocities:
        velocity_by_name = {
            str(name): abs(float(velocity))
            for name, velocity in zip(joint_names, velocities)
        }
        worst_velocity_joint = ""
        max_velocity = 0.0
        for name in trajectory_names:
            velocity = float(velocity_by_name.get(name, 0.0))
            if velocity > max_velocity:
                max_velocity = velocity
                worst_velocity_joint = str(name)
        if max_velocity > float(max_live_velocity_rad_s):
            return ReplayTrackingResult(
                False,
                "live_velocity",
                (
                    f"{worst_velocity_joint} live velocity {max_velocity:.4f} rad/s > "
                    f"{float(max_live_velocity_rad_s):.4f} rad/s"
                ),
                worst_joint=worst_velocity_joint,
                max_live_velocity_rad_s=max_velocity,
            )

    return ReplayTrackingResult(
        True,
        "ok",
        "replay tracking within limits",
        worst_joint=worst_joint,
        max_tracking_error_rad=max_error,
    )

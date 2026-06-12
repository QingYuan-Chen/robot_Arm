from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(frozen=True)
class TimeParameterizationResult:
    points: list
    requested_method: str
    used_method: str
    message: str


def normalize_time_parameterization_method(method: str | None) -> str:
    value = str(method or "auto").strip().lower()
    if value in ("", "default"):
        return "auto"
    if value not in ("auto", "current_jerk_retime", "ruckig"):
        return "auto"
    return value


def ruckig_python_available() -> bool:
    try:
        import ruckig  # noqa: F401
    except Exception:
        return False
    return True


def ruckig_waypoint_adapter_available() -> bool:
    return False


def parameterize_teach_samples(
    samples: Sequence,
    *,
    method: str | None,
    fallback_retime: Callable[..., list],
    replay_speed: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float,
    max_jerk_rad_s3: float,
    initial_delay_sec: float = 0.0,
    boundary_zero_velocity: bool = True,
) -> TimeParameterizationResult:
    requested = normalize_time_parameterization_method(method)
    use_ruckig = (
        (requested == "ruckig" or (requested == "auto" and ruckig_python_available()))
        and ruckig_waypoint_adapter_available()
    )
    if use_ruckig:
        points = fallback_retime(
            list(samples),
            replay_speed=replay_speed,
            max_velocity_rad_s=max_velocity_rad_s,
            max_acceleration_rad_s2=max_acceleration_rad_s2,
            max_jerk_rad_s3=max_jerk_rad_s3,
            initial_delay_sec=initial_delay_sec,
            boundary_zero_velocity=boundary_zero_velocity,
        )
        return TimeParameterizationResult(
            points=points,
            requested_method=requested,
            used_method="ruckig",
            message="ruckig python module available; current waypoint-preserving adapter used",
        )

    points = fallback_retime(
        list(samples),
        replay_speed=replay_speed,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
        initial_delay_sec=initial_delay_sec,
        boundary_zero_velocity=boundary_zero_velocity,
    )
    message = (
        "python ruckig waypoint adapter not implemented; used current jerk-aware retime"
        if requested == "ruckig" and ruckig_python_available()
        else "ruckig python module unavailable; used current jerk-aware retime"
        if requested == "ruckig"
        else "used current jerk-aware retime"
    )
    return TimeParameterizationResult(
        points=points,
        requested_method=requested,
        used_method="current_jerk_retime",
        message=message,
    )

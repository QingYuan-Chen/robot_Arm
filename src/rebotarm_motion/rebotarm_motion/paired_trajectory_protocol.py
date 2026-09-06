from __future__ import annotations

from bisect import bisect_right
import hashlib
import json
import math
from typing import Mapping, Sequence


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
SCHEMA_VERSION = 1


def quintic_blend(ratio: float) -> float:
    value = min(max(float(ratio), 0.0), 1.0)
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def build_quintic_command(
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    *,
    duration_sec: float,
    cadence_sec: float = 0.05,
    label: str,
) -> dict[str, object]:
    start = _vector6(start_positions, "start_positions")
    target = _vector6(target_positions, "target_positions")
    duration = _positive_finite(duration_sec, "duration_sec")
    cadence = _positive_finite(cadence_sec, "cadence_sec")
    steps = max(2, int(round(duration / cadence)))
    actual_cadence = duration / float(steps)
    if not math.isclose(actual_cadence, cadence, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("duration_sec must be an integer multiple of cadence_sec")

    points = []
    for index in range(steps + 1):
        ratio = index / float(steps)
        blend = quintic_blend(ratio)
        points.append(
            {
                "elapsed_sec": duration * ratio,
                "positions": [
                    float(origin + (destination - origin) * blend)
                    for origin, destination in zip(start, target)
                ],
            }
        )
    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "label": str(label),
        "joint_names": list(ARM_JOINT_NAMES),
        "duration_sec": duration,
        "cadence_sec": cadence,
        "points": points,
    }
    payload["command_sha256"] = command_sha256(payload)
    return payload


def build_retimed_path_command(
    waypoints: Sequence[Sequence[float]],
    *,
    duration_sec: float,
    cadence_sec: float = 0.05,
    label: str,
) -> dict[str, object]:
    """Re-time an existing joint-space path onto a quintic time profile.

    ``build_quintic_command`` interpolates straight from start to target and so
    discards everything a planner found in between: obstacle avoidance, the
    intermediate postures, and the IK branch the plan actually belongs to.
    Executing that instead of the plan makes the end effector sweep a different
    Cartesian path than the one that was validated.

    This keeps the planner's geometric path exactly -- every source waypoint is
    inserted into the outgoing command in order -- and only replaces the timing
    with a smooth, zero-boundary-velocity profile.
    Arc length is measured in joint space (max-norm), and the quintic profile is
    applied to progress along that length, giving zero start/end velocity.
    """
    if len(waypoints) < 2:
        raise ValueError("waypoints must contain at least two entries")
    path = [_vector6(point, "waypoint") for point in waypoints]
    duration = _positive_finite(duration_sec, "duration_sec")
    cadence = _positive_finite(cadence_sec, "cadence_sec")
    steps = max(2, int(round(duration / cadence)))
    actual_cadence = duration / float(steps)
    if not math.isclose(actual_cadence, cadence, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("duration_sec must be an integer multiple of cadence_sec")

    # Cumulative joint-space arc length; max-norm keeps the dominant joint's
    # motion as the progress measure.
    cumulative = [0.0]
    for previous, current in zip(path, path[1:]):
        step = max(abs(b - a) for a, b in zip(previous, current))
        cumulative.append(cumulative[-1] + step)
    total = cumulative[-1]
    if total <= 0.0:
        raise ValueError("waypoints must describe a non-zero path")

    # A fixed-rate resample alone can straddle an intermediate waypoint and let
    # the controller interpolate across the corner.  Add the exact time of every
    # planner waypoint to the cadence grid so the commanded polyline cannot cut
    # a planner corner.  Duplicate consecutive waypoints share one command point.
    waypoint_times = [
        duration * _inverse_quintic_blend(length / total)
        for length in cumulative
    ]
    sample_times = [duration * index / float(steps) for index in range(steps + 1)]
    event_times = sorted([*sample_times, *waypoint_times])
    times: list[float] = []
    for event_time in event_times:
        if not times or not math.isclose(event_time, times[-1], rel_tol=0.0, abs_tol=1e-12):
            times.append(event_time)

    points = []
    for elapsed in times:
        ratio = elapsed / duration
        travelled = total * quintic_blend(ratio)
        segment = bisect_right(cumulative, travelled)
        if segment >= len(cumulative):
            positions = list(path[-1])
        elif segment == 0:
            positions = list(path[0])
        else:
            lower_length = cumulative[segment - 1]
            upper_length = cumulative[segment]
            span = upper_length - lower_length
            blend = 0.0 if span <= 0.0 else (travelled - lower_length) / span
            positions = [
                float(start + (end - start) * blend)
                for start, end in zip(path[segment - 1], path[segment])
            ]
        points.append({"elapsed_sec": elapsed, "positions": positions})

    source_waypoint_indices: list[int] = []
    for waypoint, waypoint_time in zip(path, waypoint_times):
        command_index = min(
            range(len(points)),
            key=lambda index: abs(float(points[index]["elapsed_sec"]) - waypoint_time),
        )
        if not math.isclose(
            float(points[command_index]["elapsed_sec"]),
            waypoint_time,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("failed to place planner waypoint on command timeline")
        points[command_index]["positions"] = list(waypoint)
        source_waypoint_indices.append(command_index)

    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "label": str(label),
        "joint_names": list(ARM_JOINT_NAMES),
        "duration_sec": duration,
        "cadence_sec": cadence,
        "points": points,
        "source_waypoints": [list(point) for point in path],
        "source_waypoint_indices": source_waypoint_indices,
    }
    payload["command_sha256"] = command_sha256(payload)
    return payload


def command_sha256(command: Mapping[str, object]) -> str:
    canonical = {key: value for key, value in command.items() if key != "command_sha256"}
    encoded = json.dumps(
        canonical,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_command(command: Mapping[str, object]) -> None:
    if int(command.get("schema_version", -1)) != SCHEMA_VERSION:
        raise ValueError("unsupported command schema_version")
    if tuple(command.get("joint_names", ())) != ARM_JOINT_NAMES:
        raise ValueError("command joint_names must be canonical joint1..joint6")
    points = command.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("command must contain at least two points")
    previous = -1.0
    for point in points:
        if not isinstance(point, Mapping):
            raise ValueError("command points must be mappings")
        elapsed = float(point["elapsed_sec"])
        _vector6(point["positions"], "point positions")
        if not math.isfinite(elapsed) or elapsed < 0.0 or elapsed <= previous:
            if elapsed == 0.0 and previous < 0.0:
                pass
            else:
                raise ValueError("point elapsed_sec must be finite and strictly increasing")
        previous = elapsed
    if not math.isclose(previous, float(command["duration_sec"]), abs_tol=1e-9):
        raise ValueError("last point must equal duration_sec")
    source_waypoints = command.get("source_waypoints")
    source_indices = command.get("source_waypoint_indices")
    if (source_waypoints is None) != (source_indices is None):
        raise ValueError("source waypoint audit fields must be provided together")
    if source_waypoints is not None and source_indices is not None:
        if not isinstance(source_waypoints, list) or not isinstance(source_indices, list):
            raise ValueError("source waypoint audit fields must be lists")
        if len(source_waypoints) != len(source_indices) or len(source_waypoints) < 2:
            raise ValueError("source waypoint audit fields have invalid lengths")
        previous_index = -1
        for waypoint, point_index in zip(source_waypoints, source_indices):
            expected = _vector6(waypoint, "source waypoint")
            index = int(point_index)
            if index < previous_index or not 0 <= index < len(points):
                raise ValueError("source waypoint indices must be ordered and in range")
            actual = _vector6(points[index]["positions"], "point positions")
            if actual != expected:
                raise ValueError("source waypoint is not present exactly in command points")
            previous_index = index
    if str(command.get("command_sha256", "")) != command_sha256(command):
        raise ValueError("command_sha256 mismatch")


def sample_command(command: Mapping[str, object], elapsed_sec: float) -> tuple[float, ...]:
    validate_command(command)
    elapsed = float(elapsed_sec)
    if not math.isfinite(elapsed):
        raise ValueError("elapsed_sec must be finite")
    points = command["points"]
    times = [float(point["elapsed_sec"]) for point in points]
    if elapsed <= times[0]:
        return tuple(float(value) for value in points[0]["positions"])
    if elapsed >= times[-1]:
        return tuple(float(value) for value in points[-1]["positions"])
    upper = bisect_right(times, elapsed)
    lower_point = points[upper - 1]
    upper_point = points[upper]
    lower_time = float(lower_point["elapsed_sec"])
    upper_time = float(upper_point["elapsed_sec"])
    ratio = (elapsed - lower_time) / (upper_time - lower_time)
    return tuple(
        float(start + (end - start) * ratio)
        for start, end in zip(lower_point["positions"], upper_point["positions"])
    )


def _vector6(values: Sequence[float], label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    result = tuple(float(value) for value in values)
    if len(result) != len(ARM_JOINT_NAMES):
        raise ValueError(f"{label} must contain six values")
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain finite values")
    return result


def _positive_finite(value: float, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def _inverse_quintic_blend(progress: float) -> float:
    """Invert the monotonic quintic blend on [0, 1]."""
    target = min(max(float(progress), 0.0), 1.0)
    if target <= 0.0:
        return 0.0
    if target >= 1.0:
        return 1.0
    lower = 0.0
    upper = 1.0
    for _ in range(64):
        middle = (lower + upper) * 0.5
        if quintic_blend(middle) < target:
            lower = middle
        else:
            upper = middle
    return (lower + upper) * 0.5

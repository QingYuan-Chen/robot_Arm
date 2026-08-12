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

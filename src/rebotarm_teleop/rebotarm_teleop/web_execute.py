from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WebExecuteDecision:
    accepted: bool
    message: str
    joint_names: tuple[str, ...] = ()
    positions: tuple[float, ...] = ()
    max_delta: float = 0.0
    max_delta_limit: float = 0.0
    duration: float = 0.0


@dataclass(frozen=True)
class WebGripperDecision:
    accepted: bool
    message: str
    position: float = 0.0
    max_effort: float = 0.0


def smoothstep(ratio: float) -> float:
    value = min(max(float(ratio), 0.0), 1.0)
    return value * value * (3.0 - 2.0 * value)


def interpolate_joint_points(
    *,
    current: tuple[float, ...],
    target: tuple[float, ...],
    duration: float,
    step_period: float = 0.05,
) -> list[tuple[float, tuple[float, ...]]]:
    if len(current) != len(target):
        raise ValueError("current and target lengths must match")
    steps = max(2, int(float(duration) / max(float(step_period), 1e-3)))
    points: list[tuple[float, tuple[float, ...]]] = []
    for step in range(steps + 1):
        ratio = step / float(steps)
        blend = smoothstep(ratio)
        positions = tuple(
            float(start) + (float(end) - float(start)) * blend
            for start, end in zip(current, target)
        )
        points.append((float(duration) * ratio, positions))
    return points


def validate_web_execute_request(
    payload: dict[str, Any],
    *,
    joint_names: tuple[str, ...],
    current_positions: dict[str, float],
    joint_limits: dict[str, tuple[float, float]],
    max_delta_rad: float,
    min_duration: float,
    max_duration: float,
    joint_velocity_limits: dict[str, float] | None = None,
    max_joint_speed_rad_s: float | None = None,
) -> WebExecuteDecision:
    if str(payload.get("confirm", "")).strip().upper() != "EXECUTE":
        return WebExecuteDecision(False, "missing EXECUTE confirmation")

    raw_targets = payload.get("joint_positions")
    if not isinstance(raw_targets, dict):
        return WebExecuteDecision(False, "joint_positions must be an object")

    max_delta_limit = float(max_delta_rad)
    requested_max_delta = payload.get("max_delta_rad")
    if requested_max_delta is not None:
        try:
            requested_max_delta_value = float(requested_max_delta)
        except (TypeError, ValueError):
            return WebExecuteDecision(False, "invalid max_delta_rad")
        if not math.isfinite(requested_max_delta_value) or requested_max_delta_value <= 0.0:
            return WebExecuteDecision(False, "invalid max_delta_rad")
        max_delta_limit = min(requested_max_delta_value, max_delta_limit)

    positions: list[float] = []
    max_delta = 0.0
    missing_current: list[str] = []
    for name in joint_names:
        if name not in raw_targets:
            return WebExecuteDecision(False, f"missing target for {name}")
        if raw_targets[name] is None:
            return WebExecuteDecision(False, f"missing target for {name}")
        try:
            target = float(raw_targets[name])
        except (TypeError, ValueError):
            return WebExecuteDecision(False, f"invalid target for {name}")
        if not math.isfinite(target):
            return WebExecuteDecision(False, f"non-finite target for {name}")

        lower, upper = joint_limits.get(name, (-math.pi, math.pi))
        if upper < lower:
            lower, upper = upper, lower
        if target < lower or target > upper:
            return WebExecuteDecision(False, f"{name} target outside joint limit")

        if name not in current_positions:
            missing_current.append(name)
        else:
            max_delta = max(max_delta, abs(target - float(current_positions[name])))
        positions.append(target)

    if missing_current:
        return WebExecuteDecision(False, f"missing live joint state: {', '.join(missing_current)}")
    if max_delta > max_delta_limit:
        return WebExecuteDecision(
            False,
            f"target delta too large: {max_delta:.4f} rad > {max_delta_limit:.4f} rad",
        )

    try:
        duration = float(payload.get("duration", min_duration))
    except (TypeError, ValueError):
        return WebExecuteDecision(False, "invalid duration")
    if not math.isfinite(duration):
        return WebExecuteDecision(False, "invalid duration")
    duration = min(max(duration, float(min_duration)), float(max_duration))

    requested_speed_limit = payload.get("max_joint_speed_rad_s", max_joint_speed_rad_s)
    speed_limit = float(max_joint_speed_rad_s) if max_joint_speed_rad_s is not None else None
    if requested_speed_limit is not None:
        try:
            requested_speed_value = float(requested_speed_limit)
        except (TypeError, ValueError):
            return WebExecuteDecision(False, "invalid max_joint_speed_rad_s")
        if not math.isfinite(requested_speed_value) or requested_speed_value <= 0.0:
            return WebExecuteDecision(False, "invalid max_joint_speed_rad_s")
        speed_limit = requested_speed_value if speed_limit is None else min(speed_limit, requested_speed_value)

    if speed_limit is not None:
        for name, target in zip(joint_names, positions):
            current = current_positions[name]
            required_speed = abs(float(target) - float(current)) / duration
            configured_limit = None
            if joint_velocity_limits is not None and name in joint_velocity_limits:
                configured_limit = float(joint_velocity_limits[name])
            joint_limit = speed_limit if configured_limit is None else min(speed_limit, configured_limit)
            if required_speed > joint_limit:
                min_duration_needed = abs(float(target) - float(current)) / max(joint_limit, 1e-9)
                return WebExecuteDecision(
                    False,
                    (
                        f"{name} speed too high: {required_speed:.4f} rad/s > "
                        f"{joint_limit:.4f} rad/s; use duration >= {min_duration_needed:.2f}s"
                    ),
                )

    return WebExecuteDecision(
        True,
        f"web preview execution accepted: max_delta={max_delta:.4f} rad",
        joint_names=joint_names,
        positions=tuple(positions),
        max_delta=max_delta,
        max_delta_limit=max_delta_limit,
        duration=duration,
    )


def validate_web_gripper_request(
    payload: dict[str, Any],
    *,
    gripper_limits: tuple[float, float],
    default_max_effort: float,
    max_effort_limit: float,
) -> WebGripperDecision:
    if str(payload.get("confirm", "")).strip().upper() != "SET_GRIPPER":
        return WebGripperDecision(False, "missing SET_GRIPPER confirmation")
    try:
        position = float(payload.get("position"))
    except (TypeError, ValueError):
        return WebGripperDecision(False, "invalid gripper position")
    if not math.isfinite(position):
        return WebGripperDecision(False, "invalid gripper position")

    lower, upper = float(gripper_limits[0]), float(gripper_limits[1])
    if upper < lower:
        lower, upper = upper, lower
    if position < lower or position > upper:
        return WebGripperDecision(False, f"gripper target outside limit: {lower:.4f}..{upper:.4f} m")

    raw_effort = payload.get("max_effort", default_max_effort)
    try:
        max_effort = float(raw_effort)
    except (TypeError, ValueError):
        return WebGripperDecision(False, "invalid gripper max_effort")
    if not math.isfinite(max_effort) or max_effort <= 0.0:
        return WebGripperDecision(False, "invalid gripper max_effort")
    max_effort = min(max_effort, float(max_effort_limit))

    return WebGripperDecision(
        True,
        f"web gripper target accepted: position={position:.4f} m",
        position=position,
        max_effort=max_effort,
    )

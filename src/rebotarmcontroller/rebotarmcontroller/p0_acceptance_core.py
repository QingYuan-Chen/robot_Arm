from __future__ import annotations

import math
from typing import Any


EXPECTED_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
DAMIAO_VELOCITY_FEEDBACK_BITS = 12
DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT = {
    "joint1": 10.0,
    "joint2": 10.0,
    "joint3": 10.0,
    "joint4": 30.0,
    "joint5": 30.0,
    "joint6": 30.0,
}


def quantization_aware_raw_velocity_limits(
    physical_limit_rad_s: float,
    *,
    feedback_bits: int = DAMIAO_VELOCITY_FEEDBACK_BITS,
    vmax_by_joint: dict[str, float] | None = None,
) -> dict[str, float]:
    """Add only half a feedback LSB to a physical velocity limit.

    Damiao velocity feedback spans ``[-VMAX, +VMAX]``. A decoded sample is
    ambiguous within half of one quantization step, so a raw feedback gate at
    the physical limit itself can reject the bin that contains the limit.
    """

    physical_limit = float(physical_limit_rad_s)
    if not math.isfinite(physical_limit) or physical_limit <= 0.0:
        raise ValueError("physical velocity limit must be finite and positive")
    if int(feedback_bits) < 2:
        raise ValueError("feedback_bits must be at least 2")
    vmax_values = (
        DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT
        if vmax_by_joint is None
        else vmax_by_joint
    )
    missing = [name for name in EXPECTED_JOINTS if name not in vmax_values]
    if missing:
        raise ValueError(f"missing velocity VMAX for joints: {missing}")
    denominator = float((1 << int(feedback_bits)) - 1)
    limits: dict[str, float] = {}
    for name in EXPECTED_JOINTS:
        vmax = float(vmax_values[name])
        if not math.isfinite(vmax) or vmax <= 0.0:
            raise ValueError(f"{name} velocity VMAX must be finite and positive")
        half_lsb = vmax / denominator
        limits[name] = physical_limit + half_lsb
    return limits


def raw_velocity_limit_violations(
    velocities_by_name: dict[str, float],
    physical_limit_rad_s: float,
    *,
    raw_limit_floor_rad_s: float | None = None,
) -> dict[str, dict[str, float]]:
    limits = quantization_aware_raw_velocity_limits(physical_limit_rad_s)
    if raw_limit_floor_rad_s is not None:
        raw_limit_floor = float(raw_limit_floor_rad_s)
        if not math.isfinite(raw_limit_floor) or raw_limit_floor <= 0.0:
            raise ValueError("raw velocity limit floor must be finite and positive")
        limits = {
            name: max(limit, raw_limit_floor)
            for name, limit in limits.items()
        }
    missing = [name for name in EXPECTED_JOINTS if name not in velocities_by_name]
    if missing:
        raise ValueError(f"missing joint velocities: {missing}")
    violations: dict[str, dict[str, float]] = {}
    for name in EXPECTED_JOINTS:
        velocity = abs(float(velocities_by_name[name]))
        if not math.isfinite(velocity):
            raise ValueError(f"{name} velocity must be finite")
        if velocity > limits[name]:
            violations[name] = {
                "raw_velocity_rad_s": velocity,
                "raw_limit_rad_s": limits[name],
            }
    return violations


def joint_state_snapshot(msg: Any) -> dict[str, Any]:
    return {
        "names": list(msg.name),
        "positions": [float(value) for value in msg.position],
        "velocities": [float(value) for value in msg.velocity],
        "effort": [float(value) for value in msg.effort],
    }


def arm_status_snapshot(msg: Any) -> dict[str, Any]:
    return {
        "mode": str(msg.mode),
        "enabled": bool(msg.enabled),
        "control_loop_active": bool(msg.control_loop_active),
        "state_machine": str(msg.state_machine),
        "joint_names": list(msg.joint_names),
        "per_joint_status_code": [int(value) for value in msg.per_joint_status_code],
        "error_codes": list(msg.error_codes),
    }


def validate_preflight(status: Any, joint_state: Any) -> list[str]:
    errors: list[str] = []
    names = tuple(joint_state.name)
    if names != EXPECTED_JOINTS:
        errors.append(f"joint state names {names!r}, expected {EXPECTED_JOINTS!r}")
    if tuple(status.joint_names) != EXPECTED_JOINTS:
        errors.append("arm status joint names do not match joint1..joint6")
    if len(joint_state.position) != len(EXPECTED_JOINTS):
        errors.append("joint position vector must contain six values")
    if len(joint_state.velocity) != len(EXPECTED_JOINTS):
        errors.append("joint velocity vector must contain six values")
    numeric_values = list(joint_state.position) + list(joint_state.velocity)
    if not all(math.isfinite(float(value)) for value in numeric_values):
        errors.append("joint state contains non-finite position or velocity")
    if status.enabled:
        errors.append("hardware is already enabled")
    if status.control_loop_active:
        errors.append("control loop is already active")
    if status.state_machine != "IDLE":
        errors.append(f"state_machine={status.state_machine!r}, expected 'IDLE'")
    codes = [int(value) for value in status.per_joint_status_code]
    if codes != [0] * len(EXPECTED_JOINTS):
        errors.append(f"motor status codes are {codes!r}, expected all zero")
    if status.error_codes:
        errors.append(f"controller reports errors: {list(status.error_codes)!r}")
    return errors


def sample_motion_metrics(
    baseline_by_name: dict[str, float],
    joint_state: Any,
) -> tuple[float, float]:
    details = motion_sample_details(baseline_by_name, joint_state)
    return (
        float(details["max_position_jump_rad"]),
        float(details["max_abs_velocity_rad_s"]),
    )


def motion_sample_details(
    baseline_by_name: dict[str, float],
    joint_state: Any,
) -> dict[str, Any]:
    positions = {
        name: float(position)
        for name, position in zip(joint_state.name, joint_state.position)
    }
    missing = [name for name in baseline_by_name if name not in positions]
    if missing:
        raise ValueError(f"joint state missing baseline joints: {missing}")
    velocities = {
        name: float(velocity)
        for name, velocity in zip(joint_state.name, joint_state.velocity)
    }
    missing_velocity = [name for name in baseline_by_name if name not in velocities]
    if missing_velocity:
        raise ValueError(f"joint state missing joint velocities: {missing_velocity}")
    jump_by_name = {
        name: abs(positions[name] - baseline)
        for name, baseline in baseline_by_name.items()
    }
    jump_joint = max(jump_by_name, key=jump_by_name.get)
    velocity_joint = max(velocities, key=lambda name: abs(velocities[name]))
    max_jump = jump_by_name[jump_joint]
    max_velocity = abs(velocities[velocity_joint])
    if not math.isfinite(max_jump) or not math.isfinite(max_velocity):
        raise ValueError("motion metrics contain non-finite values")
    return {
        "max_position_jump_rad": max_jump,
        "position_jump_joint": jump_joint,
        "position_at_peak_rad": positions[jump_joint],
        "position_baseline_rad": baseline_by_name[jump_joint],
        "max_abs_velocity_rad_s": max_velocity,
        "velocity_joint": velocity_joint,
        "signed_velocity_at_peak_rad_s": velocities[velocity_joint],
        "abs_velocity_by_joint_rad_s": {
            name: abs(value) for name, value in velocities.items()
        },
    }

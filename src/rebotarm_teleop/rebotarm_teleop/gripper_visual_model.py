from __future__ import annotations


DEFAULT_GRIPPER_LIMITS_M = (0.0, 0.09)


def clamp_gripper_opening(position_m: float, limits_m: tuple[float, float]) -> float:
    lower, upper = float(limits_m[0]), float(limits_m[1])
    if upper < lower:
        lower, upper = upper, lower
    value = min(max(float(position_m), lower), upper)
    return value - lower


def gripper_opening_to_finger_joint_positions(
    position_m: float,
    limits_m: tuple[float, float] = DEFAULT_GRIPPER_LIMITS_M,
) -> tuple[float, float]:
    half_opening = 0.5 * clamp_gripper_opening(position_m, limits_m)
    return half_opening, -half_opening

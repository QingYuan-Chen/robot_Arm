from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

from .schemas import TrajectorySample


@dataclass(frozen=True)
class SafetyLimits:
    joint_position_limits: tuple[tuple[float, float], ...]
    actuator_torque_limits: tuple[float, ...]
    max_contact_force: float = math.inf
    max_contact_penetration: float = 0.01
    tolerance: float = 1e-6

    def __post_init__(self) -> None:
        joint_limits = tuple(
            (float(bounds[0]), float(bounds[1]))
            for bounds in self.joint_position_limits
        )
        torque_limits = tuple(float(value) for value in self.actuator_torque_limits)
        if len(joint_limits) != 6 or any(lower > upper for lower, upper in joint_limits):
            raise ValueError("joint_position_limits must contain six ordered bounds")
        if len(torque_limits) != 6 or any(value <= 0.0 for value in torque_limits):
            raise ValueError("actuator_torque_limits must contain six positive values")
        for field_name in ("max_contact_force", "max_contact_penetration", "tolerance"):
            value = float(getattr(self, field_name))
            if math.isnan(value) or value < 0.0:
                raise ValueError(f"{field_name} must be non-negative")
            object.__setattr__(self, field_name, value)
        object.__setattr__(self, "joint_position_limits", joint_limits)
        object.__setattr__(self, "actuator_torque_limits", torque_limits)


def safety_limits_from_env(
    env,
    *,
    max_contact_force: float = math.inf,
    max_contact_penetration: float = 0.01,
) -> SafetyLimits:
    return SafetyLimits(
        joint_position_limits=tuple(env.sim.arm_joint_limits),
        actuator_torque_limits=tuple(env.sim.arm_actuator_force_limits),
        max_contact_force=max_contact_force,
        max_contact_penetration=max_contact_penetration,
    )


def validate_trajectory(
    samples: Sequence[TrajectorySample],
    limits: SafetyLimits,
) -> dict[str, Any]:
    samples = tuple(samples)
    violations: list[dict[str, Any]] = []
    maxima = {
        "joint_limit_excess_rad": 0.0,
        "actuator_torque_excess": 0.0,
        "contact_force": 0.0,
        "contact_penetration": 0.0,
    }
    for sample in samples:
        for index, (position, bounds) in enumerate(
            zip(sample.joint_positions, limits.joint_position_limits)
        ):
            lower, upper = bounds
            excess = max(lower - position, position - upper, 0.0)
            maxima["joint_limit_excess_rad"] = max(
                maxima["joint_limit_excess_rad"], excess
            )
            if excess > limits.tolerance:
                violations.append(_violation(sample, "joint_limit", index, excess))
        for index, (torque, limit) in enumerate(
            zip(sample.actuator_torques, limits.actuator_torque_limits)
        ):
            excess = max(abs(torque) - limit, 0.0)
            maxima["actuator_torque_excess"] = max(
                maxima["actuator_torque_excess"], excess
            )
            if excess > limits.tolerance:
                violations.append(_violation(sample, "actuator_torque", index, excess))
        maxima["contact_force"] = max(maxima["contact_force"], sample.max_contact_force)
        maxima["contact_penetration"] = max(
            maxima["contact_penetration"], sample.max_contact_penetration
        )
        if sample.max_contact_force > limits.max_contact_force + limits.tolerance:
            violations.append(
                _violation(sample, "contact_force", None, sample.max_contact_force)
            )
        if sample.max_contact_penetration > limits.max_contact_penetration + limits.tolerance:
            violations.append(
                _violation(
                    sample,
                    "contact_penetration",
                    None,
                    sample.max_contact_penetration,
                )
            )
    return {
        "ok": bool(samples) and not violations,
        "sample_count": len(samples),
        "limits": {
            "joint_position_limits": [list(bounds) for bounds in limits.joint_position_limits],
            "actuator_torque_limits": list(limits.actuator_torque_limits),
            "max_contact_force": (
                limits.max_contact_force if math.isfinite(limits.max_contact_force) else None
            ),
            "max_contact_penetration": limits.max_contact_penetration,
            "tolerance": limits.tolerance,
        },
        "maxima": maxima,
        "violation_count": len(violations),
        "violations": violations,
    }


def _violation(
    sample: TrajectorySample,
    kind: str,
    joint_index: int | None,
    value: float,
) -> dict[str, Any]:
    result = {
        "kind": kind,
        "step_index": sample.step_index,
        "value": float(value),
    }
    if joint_index is not None:
        result["joint"] = f"joint{joint_index + 1}"
    return result

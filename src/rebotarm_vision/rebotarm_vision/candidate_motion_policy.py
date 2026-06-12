from __future__ import annotations

from dataclasses import dataclass
import math


def angle_delta(target_rad: float, current_rad: float) -> float:
    return math.atan2(math.sin(float(target_rad) - float(current_rad)), math.cos(float(target_rad) - float(current_rad)))


@dataclass(frozen=True)
class JointMotionPolicyConfig:
    joint_distance_weight: float = 0.15
    joint6_weight: float = 0.35
    max_joint6_delta_rad: float = 1.5708


@dataclass(frozen=True)
class JointMotionEvaluation:
    accepted: bool
    penalty: float
    reason: str
    joint_distance: float
    joint6_delta: float


def evaluate_joint_motion(
    *,
    current_positions: dict[str, float],
    target_positions: dict[str, float],
    config: JointMotionPolicyConfig = JointMotionPolicyConfig(),
) -> JointMotionEvaluation:
    common_names = [name for name in current_positions if name in target_positions and name.startswith("joint")]
    if not common_names:
        return JointMotionEvaluation(
            accepted=True,
            penalty=0.0,
            reason="joint_delta=unknown",
            joint_distance=0.0,
            joint6_delta=0.0,
        )

    deltas = {
        name: abs(angle_delta(float(target_positions[name]), float(current_positions[name])))
        for name in common_names
    }
    joint_distance = math.sqrt(sum(value * value for value in deltas.values()))
    joint6_delta = float(deltas.get("joint6", 0.0))
    max_joint6_delta = float(config.max_joint6_delta_rad)
    reason = f"joint_distance={joint_distance:.3f}, joint6_delta={joint6_delta:.3f}"
    if max_joint6_delta > 0.0 and joint6_delta > max_joint6_delta:
        return JointMotionEvaluation(
            accepted=False,
            penalty=0.0,
            reason=reason,
            joint_distance=joint_distance,
            joint6_delta=joint6_delta,
        )

    penalty = float(config.joint_distance_weight) * joint_distance + float(config.joint6_weight) * joint6_delta
    return JointMotionEvaluation(
        accepted=True,
        penalty=penalty,
        reason=reason,
        joint_distance=joint_distance,
        joint6_delta=joint6_delta,
    )

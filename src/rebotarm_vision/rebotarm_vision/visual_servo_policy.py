from __future__ import annotations

from dataclasses import dataclass
import math

from .visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class VisualServoApproachConfig:
    max_step_m: float = 0.02
    position_tolerance_m: float = 0.008


@dataclass(frozen=True)
class VisualServoStep:
    target: PoseTarget
    error_m: float
    reached: bool


def build_visual_servo_step(
    current: PoseTarget,
    desired: PoseTarget,
    config: VisualServoApproachConfig,
) -> VisualServoStep:
    dx = float(desired.position[0]) - float(current.position[0])
    dy = float(desired.position[1]) - float(current.position[1])
    dz = float(desired.position[2]) - float(current.position[2])
    error_m = math.sqrt(dx * dx + dy * dy + dz * dz)
    tolerance = max(0.0, float(config.position_tolerance_m))
    if error_m <= tolerance:
        return VisualServoStep(target=desired, error_m=error_m, reached=True)

    max_step = max(0.0, float(config.max_step_m))
    if max_step <= 0.0 or error_m <= max_step:
        return VisualServoStep(target=desired, error_m=error_m, reached=False)

    scale = max_step / error_m
    target = PoseTarget(
        position=(
            float(current.position[0]) + dx * scale,
            float(current.position[1]) + dy * scale,
            float(current.position[2]) + dz * scale,
        ),
        orientation=desired.orientation,
    )
    return VisualServoStep(target=target, error_m=error_m, reached=False)

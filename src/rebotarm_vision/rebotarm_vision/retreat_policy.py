from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class RetreatPolicyConfig:
    enabled: bool = False
    min_lift_z_m: float = 0.22
    retreat_distance_m: float = 0.06
    retreat_axis_xyz: tuple[float, float, float] = (-1.0, 0.0, 0.5)


def _normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    norm = (x * x + y * y + z * z) ** 0.5
    if norm <= 1e-9:
        raise ValueError("retreat_axis_xyz must be non-zero")
    return (x / norm, y / norm, z / norm)


def build_lift_pose(grasp: PoseTarget, *, lift_z_m: float, min_lift_z_m: float = 0.0) -> PoseTarget:
    from .visual_grasp_sequence import PoseTarget

    lift_z = max(float(grasp.position[2]) + float(lift_z_m), float(min_lift_z_m))
    return PoseTarget(
        position=(
            float(grasp.position[0]),
            float(grasp.position[1]),
            lift_z,
        ),
        orientation=grasp.orientation,
    )


def build_retreat_pose(lift: PoseTarget, config: RetreatPolicyConfig) -> PoseTarget:
    from .visual_grasp_sequence import PoseTarget

    axis = _normalize_vector(config.retreat_axis_xyz)
    distance = max(float(config.retreat_distance_m), 0.0)
    return PoseTarget(
        position=(
            float(lift.position[0]) + axis[0] * distance,
            float(lift.position[1]) + axis[1] * distance,
            float(lift.position[2]) + axis[2] * distance,
        ),
        orientation=lift.orientation,
    )

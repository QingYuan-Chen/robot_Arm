from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ApproachPolicyConfig:
    pregrasp_distance_m: float = 0.08
    pregrasp_z_offset_m: float = 0.0
    pregrasp_min_z_m: float = 0.0


def normalize_vector(vector_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = (float(vector_xyz[0]), float(vector_xyz[1]), float(vector_xyz[2]))
    norm = (x * x + y * y + z * z) ** 0.5
    if norm <= 1e-9:
        raise ValueError("approach_axis_xyz must be non-zero")
    return (x / norm, y / norm, z / norm)


def build_pregrasp_tcp(
    *,
    grasp_tcp_xyz: tuple[float, float, float],
    approach_axis_xyz: tuple[float, float, float],
    config: ApproachPolicyConfig = ApproachPolicyConfig(),
) -> tuple[float, float, float]:
    axis = normalize_vector(approach_axis_xyz)
    pregrasp = (
        float(grasp_tcp_xyz[0]) - axis[0] * float(config.pregrasp_distance_m),
        float(grasp_tcp_xyz[1]) - axis[1] * float(config.pregrasp_distance_m),
        float(grasp_tcp_xyz[2])
        - axis[2] * float(config.pregrasp_distance_m)
        + float(config.pregrasp_z_offset_m),
    )
    min_z = float(config.pregrasp_min_z_m)
    if min_z > 0.0:
        pregrasp = (pregrasp[0], pregrasp[1], max(pregrasp[2], min_z))
    return pregrasp

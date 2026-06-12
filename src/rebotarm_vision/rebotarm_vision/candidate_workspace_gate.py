from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CandidateWorkspaceGateConfig:
    enabled: bool = False
    min_xyz: tuple[float, float, float] = (0.18, -0.35, 0.0)
    max_xyz: tuple[float, float, float] = (0.64, 0.35, 0.45)
    max_grasp_to_object_center_m: float = 0.15


@dataclass(frozen=True)
class CandidateWorkspaceGateResult:
    accepted: bool
    reason: str = ""


def candidate_workspace_gate(
    *,
    grasp_position_xyz: tuple[float, float, float],
    object_center_xyz: tuple[float, float, float] | None,
    config: CandidateWorkspaceGateConfig,
) -> CandidateWorkspaceGateResult:
    if not config.enabled:
        return CandidateWorkspaceGateResult(True)

    x, y, z = (float(grasp_position_xyz[0]), float(grasp_position_xyz[1]), float(grasp_position_xyz[2]))
    min_x, min_y, min_z = (float(config.min_xyz[0]), float(config.min_xyz[1]), float(config.min_xyz[2]))
    max_x, max_y, max_z = (float(config.max_xyz[0]), float(config.max_xyz[1]), float(config.max_xyz[2]))
    if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
        return CandidateWorkspaceGateResult(
            False,
            f"grasp outside workspace ({x:.3f}, {y:.3f}, {z:.3f})",
        )

    if object_center_xyz is not None and float(config.max_grasp_to_object_center_m) > 0.0:
        ox, oy, oz = (
            float(object_center_xyz[0]),
            float(object_center_xyz[1]),
            float(object_center_xyz[2]),
        )
        distance = ((x - ox) ** 2 + (y - oy) ** 2 + (z - oz) ** 2) ** 0.5
        if distance > float(config.max_grasp_to_object_center_m):
            return CandidateWorkspaceGateResult(
                False,
                f"grasp too far from object center ({distance:.3f}m)",
            )

    return CandidateWorkspaceGateResult(True)

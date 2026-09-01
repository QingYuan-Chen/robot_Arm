from __future__ import annotations

from dataclasses import dataclass

from .candidate_workspace_gate import CandidateWorkspaceGateConfig, candidate_workspace_gate


@dataclass(frozen=True)
class CandidateGateConfig:
    min_jaw_width_m: float = 0.006
    max_jaw_width_m: float = 0.085
    min_grasp_z_m: float = 0.0
    workspace_gate_enabled: bool = False
    workspace_min_xyz: tuple[float, float, float] = (0.18, -0.35, 0.0)
    workspace_max_xyz: tuple[float, float, float] = (0.64, 0.35, 0.45)
    max_grasp_to_object_center_m: float = 0.15


@dataclass(frozen=True)
class CandidateGateResult:
    accepted: bool
    reason: str = ""


def evaluate_candidate_gate(
    *,
    jaw_width_m: float,
    grasp_position_xyz: tuple[float, float, float],
    object_center_xyz: tuple[float, float, float] | None,
    config: CandidateGateConfig = CandidateGateConfig(),
) -> CandidateGateResult:
    width = float(jaw_width_m)
    if width < float(config.min_jaw_width_m):
        return CandidateGateResult(False, f"jaw_width too small ({width:.3f}m)")
    if width > float(config.max_jaw_width_m):
        return CandidateGateResult(False, f"jaw_width too large ({width:.3f}m)")

    grasp_z = float(grasp_position_xyz[2])
    if grasp_z < float(config.min_grasp_z_m):
        return CandidateGateResult(False, f"grasp z too low ({grasp_z:.3f}m)")

    workspace = candidate_workspace_gate(
        grasp_position_xyz=grasp_position_xyz,
        object_center_xyz=object_center_xyz,
        config=CandidateWorkspaceGateConfig(
            enabled=bool(config.workspace_gate_enabled),
            min_xyz=config.workspace_min_xyz,
            max_xyz=config.workspace_max_xyz,
            max_grasp_to_object_center_m=float(config.max_grasp_to_object_center_m),
        ),
    )
    if not workspace.accepted:
        return CandidateGateResult(False, workspace.reason)

    return CandidateGateResult(True)

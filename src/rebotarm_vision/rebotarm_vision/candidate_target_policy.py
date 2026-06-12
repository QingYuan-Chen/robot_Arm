from __future__ import annotations

from dataclasses import dataclass
import math

from .pose_variant_policy import PoseVariantConfig, build_parallel_jaw_pose_variants, normalize_quaternion, quat_multiply
from .visual_grasp_pose_policy import (
    BaseAxisGraspPolicyConfig,
    OfficialGeometryGraspPolicyConfig,
    build_base_axis_grasp_targets,
    build_hybrid_geometry_grasp_targets,
    build_official_geometry_grasp_targets,
    build_preserve_candidate_grasp_targets,
)
from .visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class CandidateTargetVariant:
    pregrasp: PoseTarget
    grasp: PoseTarget
    label: str


@dataclass(frozen=True)
class CandidateTargetPolicyConfig:
    pose_policy: str = "hybrid_geometry_with_base_axis_fallback"
    fixed_grasp_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    base_approach_axis_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)
    base_pregrasp_distance_m: float = 0.08
    tcp_offset_xyz: tuple[float, float, float] = (-0.04, 0.0, 0.0)
    target_base_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pregrasp_base_z_offset_m: float = 0.05
    pregrasp_min_z_m: float = 0.0
    grasp_base_z_offset_m: float = 0.0
    orientation_yaw_offsets_rad: tuple[float, ...] = (0.0,)
    candidate_grasp_z_offsets_m: tuple[float, ...] = (0.0,)
    pose_variant_config: PoseVariantConfig = PoseVariantConfig()


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _nonempty_offsets(values: tuple[float, ...]) -> tuple[float, ...]:
    return values if values else (0.0,)


def _base_axis_config(
    config: CandidateTargetPolicyConfig,
    *,
    fixed_orientation_xyzw: tuple[float, float, float, float],
    grasp_z_offset_m: float,
) -> BaseAxisGraspPolicyConfig:
    return BaseAxisGraspPolicyConfig(
        fixed_orientation_xyzw=fixed_orientation_xyzw,
        approach_axis_xyz=config.base_approach_axis_xyz,
        pregrasp_distance_m=float(config.base_pregrasp_distance_m),
        tcp_offset_xyz=config.tcp_offset_xyz,
        target_base_offset_xyz=config.target_base_offset_xyz,
        pregrasp_z_offset_m=float(config.pregrasp_base_z_offset_m),
        pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        grasp_z_offset_m=float(grasp_z_offset_m),
    )


def _official_config(
    config: CandidateTargetPolicyConfig,
    *,
    grasp_z_offset_m: float,
) -> OfficialGeometryGraspPolicyConfig:
    return OfficialGeometryGraspPolicyConfig(
        pregrasp_distance_m=float(config.base_pregrasp_distance_m),
        tcp_offset_xyz=config.tcp_offset_xyz,
        target_base_offset_xyz=config.target_base_offset_xyz,
        pregrasp_z_offset_m=float(config.pregrasp_base_z_offset_m),
        pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        grasp_z_offset_m=float(grasp_z_offset_m),
    )


def _symmetric_orientations(
    *,
    base_label: str,
    orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
    include_original: bool = True,
) -> list[tuple[str, tuple[float, float, float, float]]]:
    return build_parallel_jaw_pose_variants(
        base_label=base_label,
        orientation_xyzw=orientation_xyzw,
        config=config.pose_variant_config,
        include_original=include_original,
    )


def _build_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    orientation_xyzw: tuple[float, float, float, float] | None,
    grasp_z_offset_extra_m: float,
    mode: str,
    config: CandidateTargetPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    orientation = orientation_xyzw or candidate_orientation_xyzw
    grasp_z_offset_m = float(config.grasp_base_z_offset_m) + float(grasp_z_offset_extra_m)
    base_config = _base_axis_config(
        config,
        fixed_orientation_xyzw=orientation,
        grasp_z_offset_m=grasp_z_offset_m,
    )
    if mode == "hybrid_geometry":
        return build_hybrid_geometry_grasp_targets(
            grasp_position_xyz=grasp_position_xyz,
            candidate_orientation_xyzw=orientation,
            config=base_config,
        )
    if mode == "preserve_candidate_pose":
        return build_preserve_candidate_grasp_targets(
            grasp_position_xyz=grasp_position_xyz,
            grasp_orientation_xyzw=orientation,
            config=_official_config(config, grasp_z_offset_m=grasp_z_offset_m),
        )
    if mode == "official_geometry":
        return build_official_geometry_grasp_targets(
            grasp_position_xyz=grasp_position_xyz,
            grasp_orientation_xyzw=orientation,
            config=_official_config(config, grasp_z_offset_m=grasp_z_offset_m),
        )
    return build_base_axis_grasp_targets(
        grasp_position_xyz=grasp_position_xyz,
        config=base_config,
    )


def _append_preserve_candidate_variants(
    variants: list[CandidateTargetVariant],
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
) -> None:
    for label, target_orientation in _symmetric_orientations(
        base_label="preserve_candidate_pose",
        orientation_xyzw=candidate_orientation_xyzw,
        config=config,
    ):
        pregrasp, grasp = _build_targets(
            grasp_position_xyz=grasp_position_xyz,
            candidate_orientation_xyzw=candidate_orientation_xyzw,
            orientation_xyzw=target_orientation,
            grasp_z_offset_extra_m=0.0,
            mode="preserve_candidate_pose",
            config=config,
        )
        variants.append(CandidateTargetVariant(pregrasp=pregrasp, grasp=grasp, label=label))


def _append_yaw_z_variants(
    variants: list[CandidateTargetVariant],
    *,
    grasp_position_xyz: tuple[float, float, float],
    base_orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
    mode: str,
    label_prefix: str,
) -> None:
    yaw_offsets = _nonempty_offsets(config.orientation_yaw_offsets_rad)
    z_offsets = _nonempty_offsets(config.candidate_grasp_z_offsets_m)
    for yaw_index, yaw_offset in enumerate(yaw_offsets):
        orientation = normalize_quaternion(quat_multiply(_yaw_quaternion(yaw_offset), base_orientation_xyzw))
        for z_index, z_offset in enumerate(z_offsets):
            label = f"{label_prefix}_yaw{yaw_index}_z{z_index}"
            pregrasp, grasp = _build_targets(
                grasp_position_xyz=grasp_position_xyz,
                candidate_orientation_xyzw=base_orientation_xyzw,
                orientation_xyzw=orientation,
                grasp_z_offset_extra_m=float(z_offset),
                mode=mode,
                config=config,
            )
            variants.append(CandidateTargetVariant(pregrasp=pregrasp, grasp=grasp, label=label))
            for symmetry_label, target_orientation in _symmetric_orientations(
                base_label=label,
                orientation_xyzw=orientation,
                config=config,
                include_original=False,
            ):
                pregrasp, grasp = _build_targets(
                    grasp_position_xyz=grasp_position_xyz,
                    candidate_orientation_xyzw=base_orientation_xyzw,
                    orientation_xyzw=target_orientation,
                    grasp_z_offset_extra_m=float(z_offset),
                    mode=mode,
                    config=config,
                )
                variants.append(CandidateTargetVariant(pregrasp=pregrasp, grasp=grasp, label=symmetry_label))


def build_candidate_target_variants(
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
) -> list[CandidateTargetVariant]:
    variants: list[CandidateTargetVariant] = []
    pose_policy = str(config.pose_policy).strip()
    candidate_orientation = normalize_quaternion(candidate_orientation_xyzw)
    if pose_policy == "preserve_candidate_pose":
        _append_preserve_candidate_variants(
            variants,
            grasp_position_xyz=grasp_position_xyz,
            candidate_orientation_xyzw=candidate_orientation,
            config=config,
        )
    if pose_policy in ("hybrid_geometry", "hybrid_geometry_with_base_axis_fallback"):
        _append_yaw_z_variants(
            variants,
            grasp_position_xyz=grasp_position_xyz,
            base_orientation_xyzw=candidate_orientation,
            config=config,
            mode="hybrid_geometry",
            label_prefix="hybrid_geometry",
        )
    if pose_policy in ("official_geometry", "official_geometry_with_base_axis_fallback"):
        _append_yaw_z_variants(
            variants,
            grasp_position_xyz=grasp_position_xyz,
            base_orientation_xyzw=candidate_orientation,
            config=config,
            mode="official_geometry",
            label_prefix="official_geometry",
        )
    if (
        pose_policy
        in (
            "base_axis",
            "official_geometry_with_base_axis_fallback",
            "hybrid_geometry_with_base_axis_fallback",
        )
        or not variants
    ):
        _append_yaw_z_variants(
            variants,
            grasp_position_xyz=grasp_position_xyz,
            base_orientation_xyzw=normalize_quaternion(config.fixed_grasp_orientation_xyzw),
            config=config,
            mode="base_axis",
            label_prefix="base_axis",
        )
    return variants

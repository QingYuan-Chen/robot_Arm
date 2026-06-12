from __future__ import annotations

from dataclasses import dataclass

from .approach_policy import ApproachPolicyConfig, build_pregrasp_tcp, normalize_vector
from .visual_grasp_sequence import PoseTarget
from .pose_variant_policy import (
    build_parallel_jaw_symmetric_orientation,
    quaternion_to_rotation_matrix,
)


@dataclass(frozen=True)
class BaseAxisGraspPolicyConfig:
    fixed_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    approach_axis_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)
    pregrasp_distance_m: float = 0.08
    tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_base_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pregrasp_z_offset_m: float = 0.05
    pregrasp_min_z_m: float = 0.0
    grasp_z_offset_m: float = 0.0


@dataclass(frozen=True)
class OfficialGeometryGraspPolicyConfig:
    pregrasp_distance_m: float = 0.08
    tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_base_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pregrasp_z_offset_m: float = 0.0
    pregrasp_min_z_m: float = 0.0
    grasp_z_offset_m: float = 0.0


def _normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return normalize_vector(vector)


def _normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = (float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3]))
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        raise ValueError("fixed_grasp_orientation_xyzw must be non-zero")
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_to_rotation_matrix(quaternion: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = _normalize_quaternion(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _subtract_tcp_offset(
    tcp_position: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    tcp_offset_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotation = _quat_to_rotation_matrix(orientation_xyzw)
    ox, oy, oz = (float(tcp_offset_xyz[0]), float(tcp_offset_xyz[1]), float(tcp_offset_xyz[2]))
    dx = rotation[0][0] * ox + rotation[0][1] * oy + rotation[0][2] * oz
    dy = rotation[1][0] * ox + rotation[1][1] * oy + rotation[1][2] * oz
    dz = rotation[2][0] * ox + rotation[2][1] * oy + rotation[2][2] * oz
    return (
        round(float(tcp_position[0]) - dx, 6),
        round(float(tcp_position[1]) - dy, 6),
        round(float(tcp_position[2]) - dz, 6),
    )


def _x_axis_from_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    rotation = _quat_to_rotation_matrix(quaternion)
    return (rotation[0][0], rotation[1][0], rotation[2][0])


def _yaw_from_quaternion(quaternion: tuple[float, float, float, float]) -> float:
    rotation = _quat_to_rotation_matrix(quaternion)
    return __import__("math").atan2(rotation[1][0], rotation[0][0])


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    math = __import__("math")
    half = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_base_axis_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    config: BaseAxisGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    axis = _normalize_vector(config.approach_axis_xyz)
    orientation = _normalize_quaternion(config.fixed_orientation_xyzw)
    base = (
        float(grasp_position_xyz[0]) + float(config.target_base_offset_xyz[0]),
        float(grasp_position_xyz[1]) + float(config.target_base_offset_xyz[1]),
        float(grasp_position_xyz[2]) + float(config.target_base_offset_xyz[2]),
    )
    grasp_tcp = (
        base[0],
        base[1],
        base[2] + float(config.grasp_z_offset_m),
    )
    pregrasp_tcp = build_pregrasp_tcp(
        grasp_tcp_xyz=base,
        approach_axis_xyz=axis,
        config=ApproachPolicyConfig(
            pregrasp_distance_m=float(config.pregrasp_distance_m),
            pregrasp_z_offset_m=float(config.pregrasp_z_offset_m),
            pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        ),
    )
    return (
        PoseTarget(
            position=_subtract_tcp_offset(pregrasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
        PoseTarget(
            position=_subtract_tcp_offset(grasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
    )


def build_hybrid_geometry_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    config: BaseAxisGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    yaw = _yaw_from_quaternion(candidate_orientation_xyzw)
    return build_base_axis_grasp_targets(
        grasp_position_xyz=grasp_position_xyz,
        config=BaseAxisGraspPolicyConfig(
            fixed_orientation_xyzw=_quaternion_from_yaw(yaw),
            approach_axis_xyz=config.approach_axis_xyz,
            pregrasp_distance_m=config.pregrasp_distance_m,
            tcp_offset_xyz=config.tcp_offset_xyz,
            target_base_offset_xyz=config.target_base_offset_xyz,
            pregrasp_z_offset_m=config.pregrasp_z_offset_m,
            pregrasp_min_z_m=config.pregrasp_min_z_m,
            grasp_z_offset_m=config.grasp_z_offset_m,
        ),
    )


def build_official_geometry_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    grasp_orientation_xyzw: tuple[float, float, float, float],
    config: OfficialGeometryGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    orientation = _normalize_quaternion(grasp_orientation_xyzw)
    approach_axis = _normalize_vector(_x_axis_from_quaternion(orientation))
    base = (
        float(grasp_position_xyz[0]) + float(config.target_base_offset_xyz[0]),
        float(grasp_position_xyz[1]) + float(config.target_base_offset_xyz[1]),
        float(grasp_position_xyz[2]) + float(config.target_base_offset_xyz[2]),
    )
    grasp_tcp = (
        base[0],
        base[1],
        base[2] + float(config.grasp_z_offset_m),
    )
    pregrasp_tcp = build_pregrasp_tcp(
        grasp_tcp_xyz=grasp_tcp,
        approach_axis_xyz=approach_axis,
        config=ApproachPolicyConfig(
            pregrasp_distance_m=float(config.pregrasp_distance_m),
            pregrasp_z_offset_m=float(config.pregrasp_z_offset_m),
            pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        ),
    )
    return (
        PoseTarget(
            position=_subtract_tcp_offset(pregrasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
        PoseTarget(
            position=_subtract_tcp_offset(grasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
    )


def build_preserve_candidate_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    grasp_orientation_xyzw: tuple[float, float, float, float],
    config: OfficialGeometryGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    return build_official_geometry_grasp_targets(
        grasp_position_xyz=grasp_position_xyz,
        grasp_orientation_xyzw=grasp_orientation_xyzw,
        config=config,
    )

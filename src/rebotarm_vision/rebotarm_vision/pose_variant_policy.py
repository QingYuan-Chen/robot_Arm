from __future__ import annotations

from dataclasses import dataclass
import math


def normalize_quaternion(quaternion_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = (float(v) for v in quaternion_xyzw)
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        raise ValueError("quaternion must be non-zero")
    return (x / norm, y / norm, z / norm, w / norm)


def quat_multiply(
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left_xyzw
    rx, ry, rz, rw = right_xyzw
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_to_rotation_matrix(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def build_parallel_jaw_symmetric_orientation(
    orientation_xyzw: tuple[float, float, float, float],
    *,
    angle_rad: float = math.pi,
) -> tuple[float, float, float, float]:
    half = float(angle_rad) * 0.5
    local_x_rotation = (math.sin(half), 0.0, 0.0, math.cos(half))
    return normalize_quaternion(quat_multiply(normalize_quaternion(orientation_xyzw), local_x_rotation))


@dataclass(frozen=True)
class PoseVariantConfig:
    joint6_symmetry_enabled: bool = True
    joint6_symmetry_angle_rad: float = math.pi


def build_parallel_jaw_pose_variants(
    *,
    base_label: str,
    orientation_xyzw: tuple[float, float, float, float],
    config: PoseVariantConfig = PoseVariantConfig(),
    include_original: bool = True,
) -> list[tuple[str, tuple[float, float, float, float]]]:
    variants: list[tuple[str, tuple[float, float, float, float]]] = []
    if include_original:
        variants.append((base_label, normalize_quaternion(orientation_xyzw)))
    if config.joint6_symmetry_enabled:
        variants.append(
            (
                f"{base_label}_parallel_jaw_symmetric",
                build_parallel_jaw_symmetric_orientation(
                    orientation_xyzw,
                    angle_rad=float(config.joint6_symmetry_angle_rad),
                ),
            )
        )
    return variants

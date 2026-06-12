from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Transform3D:
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


def quaternion_to_rotation_matrix(
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    qx, qy, qz, qw = rotation_xyzw
    return (
        (
            1.0 - 2.0 * qy * qy - 2.0 * qz * qz,
            2.0 * qx * qy - 2.0 * qz * qw,
            2.0 * qx * qz + 2.0 * qy * qw,
        ),
        (
            2.0 * qx * qy + 2.0 * qz * qw,
            1.0 - 2.0 * qx * qx - 2.0 * qz * qz,
            2.0 * qy * qz - 2.0 * qx * qw,
        ),
        (
            2.0 * qx * qz - 2.0 * qy * qw,
            2.0 * qy * qz + 2.0 * qx * qw,
            1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
        ),
    )


def transform_point(
    transform: Transform3D,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotation = quaternion_to_rotation_matrix(transform.rotation_xyzw)
    tx, ty, tz = transform.translation
    px, py, pz = point
    x = rotation[0][0] * px + rotation[0][1] * py + rotation[0][2] * pz + tx
    y = rotation[1][0] * px + rotation[1][1] * py + rotation[1][2] * pz + ty
    z = rotation[2][0] * px + rotation[2][1] * py + rotation[2][2] * pz + tz
    return (round(x, 6), round(y, 6), round(z, 6))


def normalize_quaternion(
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = rotation_xyzw
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def multiply_quaternions(
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = normalize_quaternion(left_xyzw)
    rx, ry, rz, rw = normalize_quaternion(right_xyzw)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def transform_pose_components(
    transform: Transform3D,
    position_xyz: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    position = transform_point(transform, position_xyz)
    orientation = multiply_quaternions(transform.rotation_xyzw, orientation_xyzw)
    return (position, orientation)

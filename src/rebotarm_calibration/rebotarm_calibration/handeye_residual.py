from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np


SCHEMA_VERSION = 1


def transform_matrix(transform: Mapping[str, object]) -> np.ndarray:
    translation = _vector(transform.get("translation"), 3, "translation")
    quaternion = _vector(transform.get("rotation_xyzw"), 4, "rotation_xyzw")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("rotation_xyzw norm must be positive")
    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def matrix_transform(matrix: np.ndarray) -> dict[str, list[float]]:
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError("matrix must be a finite 4x4 transform")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("matrix must have homogeneous bottom row [0, 0, 0, 1]")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, abs_tol=1e-8
    ):
        raise ValueError("matrix rotation must be orthonormal with determinant +1")
    return {
        "translation": [float(item) for item in value[:3, 3]],
        "rotation_xyzw": [float(item) for item in _matrix_quaternion(value[:3, :3])],
    }


def analyze_handeye_residual(
    payload: Mapping[str, object],
    *,
    min_samples: int = 5,
    min_end_translation_span_m: float = 0.05,
    min_end_rotation_span_deg: float = 20.0,
    max_position_rms_m: float = 0.005,
    max_position_residual_m: float = 0.010,
    max_rotation_rms_deg: float = 1.5,
    max_rotation_residual_deg: float = 3.0,
) -> dict[str, object]:
    if int(min_samples) < 2:
        raise ValueError("min_samples must be at least 2")
    positive_limits = {
        "min_end_translation_span_m": min_end_translation_span_m,
        "min_end_rotation_span_deg": min_end_rotation_span_deg,
        "max_position_rms_m": max_position_rms_m,
        "max_position_residual_m": max_position_residual_m,
        "max_rotation_rms_deg": max_rotation_rms_deg,
        "max_rotation_residual_deg": max_rotation_residual_deg,
    }
    if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in positive_limits.values()):
        raise ValueError("all residual and diversity limits must be finite and positive")
    if int(payload.get("schema_version", -1)) != SCHEMA_VERSION:
        raise ValueError("unsupported schema_version")
    samples = payload.get("samples")
    if not isinstance(samples, list) or len(samples) < min_samples:
        raise ValueError(f"at least {min_samples} samples are required")
    handeye = transform_matrix(_mapping(payload.get("end_to_camera"), "end_to_camera"))

    base_to_end = []
    base_to_marker = []
    normalized_samples = []
    for index, sample in enumerate(samples):
        source = _mapping(sample, f"samples[{index}]")
        base_end = transform_matrix(_mapping(source.get("base_to_end"), "base_to_end"))
        camera_marker = transform_matrix(
            _mapping(source.get("camera_to_marker"), "camera_to_marker")
        )
        marker = base_end @ handeye @ camera_marker
        base_to_end.append(base_end)
        base_to_marker.append(marker)
        normalized_samples.append(
            {
                "label": str(source.get("label", f"sample_{index + 1}")),
                "base_to_marker": matrix_transform(marker),
            }
        )

    positions = np.stack([value[:3, 3] for value in base_to_marker])
    rotations = np.stack([value[:3, :3] for value in base_to_marker])
    mean_position = np.mean(positions, axis=0)
    mean_rotation = _mean_rotation(rotations)
    position_residuals = np.linalg.norm(positions - mean_position, axis=1)
    rotation_residuals = np.array(
        [_rotation_angle_deg(mean_rotation.T @ rotation) for rotation in rotations]
    )
    end_translation_span = _max_pairwise_distance(
        np.stack([value[:3, 3] for value in base_to_end])
    )
    end_rotation_span = max(
        _rotation_angle_deg(left[:3, :3].T @ right[:3, :3])
        for index, left in enumerate(base_to_end)
        for right in base_to_end[index + 1 :]
    )
    diversity_pass = (
        end_translation_span >= float(min_end_translation_span_m)
        and end_rotation_span >= float(min_end_rotation_span_deg)
    )
    position_rms = float(math.sqrt(float(np.mean(position_residuals**2))))
    position_max = float(np.max(position_residuals))
    rotation_rms = float(math.sqrt(float(np.mean(rotation_residuals**2))))
    rotation_max = float(np.max(rotation_residuals))
    acceptance_pass = (
        diversity_pass
        and position_rms <= float(max_position_rms_m)
        and position_max <= float(max_position_residual_m)
        and rotation_rms <= float(max_rotation_rms_deg)
        and rotation_max <= float(max_rotation_residual_deg)
    )
    for item, position_error, rotation_error in zip(
        normalized_samples, position_residuals, rotation_residuals
    ):
        item["position_residual_m"] = float(position_error)
        item["rotation_residual_deg"] = float(rotation_error)

    return {
        "sample_count": len(samples),
        "mean_base_to_marker": matrix_transform(
            _matrix_from_rotation_translation(mean_rotation, mean_position)
        ),
        "position_residual": {
            "rms_m": position_rms,
            "max_m": position_max,
            "median_m": float(np.median(position_residuals)),
        },
        "rotation_residual": {
            "rms_deg": rotation_rms,
            "max_deg": rotation_max,
            "median_deg": float(np.median(rotation_residuals)),
        },
        "pose_diversity": {
            "end_translation_span_m": float(end_translation_span),
            "end_rotation_span_deg": float(end_rotation_span),
            "required_translation_span_m": float(min_end_translation_span_m),
            "required_rotation_span_deg": float(min_end_rotation_span_deg),
            "pass": bool(diversity_pass),
        },
        "acceptance": {
            "max_position_rms_m": float(max_position_rms_m),
            "max_position_residual_m": float(max_position_residual_m),
            "max_rotation_rms_deg": float(max_rotation_rms_deg),
            "max_rotation_residual_deg": float(max_rotation_residual_deg),
            "pass": bool(acceptance_pass),
        },
        "samples": normalized_samples,
    }


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _vector(value: object, size: int, label: str) -> np.ndarray:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{label} must contain {size} finite values")
    result = np.asarray([float(item) for item in value], dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def _matrix_from_rotation_translation(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _mean_rotation(rotations: np.ndarray) -> np.ndarray:
    left, _, right = np.linalg.svd(np.mean(rotations, axis=0))
    result = left @ right
    if np.linalg.det(result) < 0.0:
        left[:, -1] *= -1.0
        result = left @ right
    return result


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = min(1.0, max(-1.0, (float(np.trace(rotation)) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _max_pairwise_distance(points: np.ndarray) -> float:
    return max(
        float(np.linalg.norm(left - right))
        for index, left in enumerate(points)
        for right in points[index + 1 :]
    )


def _matrix_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [(rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale]
            )
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion

from __future__ import annotations

import math
from typing import Mapping, Sequence

import cv2
import numpy as np

from .handeye_residual import matrix_transform, transform_matrix


HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_eye_in_hand(
    samples: Sequence[Mapping[str, object]],
    *,
    method: str = "TSAI",
) -> np.ndarray:
    """Solve base->end * end->camera * camera->marker = base->marker."""
    if len(samples) < 3:
        raise ValueError("at least three samples are required for hand-eye solve")
    method_name = str(method).upper()
    if method_name not in HAND_EYE_METHODS:
        raise ValueError(f"unsupported hand-eye method: {method}")

    base_to_end = []
    camera_to_marker = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping):
            raise ValueError(f"samples[{index}] must be a mapping")
        base_to_end.append(
            transform_matrix(_mapping(sample.get("base_to_end"), "base_to_end"))
        )
        camera_to_marker.append(
            transform_matrix(
                _mapping(sample.get("camera_to_marker"), "camera_to_marker")
            )
        )

    rotation, translation = cv2.calibrateHandEye(
        [value[:3, :3] for value in base_to_end],
        [value[:3, 3] for value in base_to_end],
        [value[:3, :3] for value in camera_to_marker],
        [value[:3, 3] for value in camera_to_marker],
        method=HAND_EYE_METHODS[method_name],
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{method_name} hand-eye solve returned non-finite values")
    return result


def evaluate_eye_in_hand(
    samples: Sequence[Mapping[str, object]],
    end_to_camera: np.ndarray,
    *,
    reference_base_to_marker: np.ndarray | None = None,
) -> dict[str, object]:
    if not samples:
        raise ValueError("at least one evaluation sample is required")
    handeye = np.asarray(end_to_camera, dtype=np.float64)
    _validate_transform(handeye, "end_to_camera")

    marker_values = []
    labels = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping):
            raise ValueError(f"samples[{index}] must be a mapping")
        base_end = transform_matrix(_mapping(sample.get("base_to_end"), "base_to_end"))
        camera_marker = transform_matrix(
            _mapping(sample.get("camera_to_marker"), "camera_to_marker")
        )
        marker_values.append(base_end @ handeye @ camera_marker)
        labels.append(str(sample.get("label", f"sample_{index + 1}")))

    if reference_base_to_marker is None:
        reference = average_transforms(marker_values)
        reference_source = "evaluation_mean"
    else:
        reference = np.asarray(reference_base_to_marker, dtype=np.float64)
        _validate_transform(reference, "reference_base_to_marker")
        reference_source = "provided"

    position_errors = np.array(
        [np.linalg.norm(value[:3, 3] - reference[:3, 3]) for value in marker_values],
        dtype=np.float64,
    )
    rotation_errors = np.array(
        [rotation_angle_deg(reference[:3, :3].T @ value[:3, :3]) for value in marker_values],
        dtype=np.float64,
    )
    details = []
    for label, value, position_error, rotation_error in zip(
        labels, marker_values, position_errors, rotation_errors
    ):
        details.append(
            {
                "label": label,
                "base_to_marker": matrix_transform(value),
                "position_residual_m": float(position_error),
                "rotation_residual_deg": float(rotation_error),
            }
        )
    return {
        "sample_count": len(samples),
        "reference_source": reference_source,
        "reference_base_to_marker": matrix_transform(reference),
        "position_residual": _residual_summary(position_errors, "m"),
        "rotation_residual": _residual_summary(rotation_errors, "deg"),
        "samples": details,
    }


def average_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    if not transforms:
        raise ValueError("at least one transform is required")
    values = [np.asarray(value, dtype=np.float64) for value in transforms]
    for index, value in enumerate(values):
        _validate_transform(value, f"transforms[{index}]")
    left, _, right = np.linalg.svd(
        np.mean(np.stack([value[:3, :3] for value in values]), axis=0)
    )
    rotation = left @ right
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.mean(
        np.stack([value[:3, 3] for value in values]), axis=0
    )
    return result


def rotation_angle_deg(rotation: np.ndarray) -> float:
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    cosine = min(1.0, max(-1.0, (float(np.trace(value)) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _validate_transform(value: np.ndarray, label: str) -> None:
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(f"{label} must be a finite 4x4 transform")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{label} must be homogeneous")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
        raise ValueError(f"{label} rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-7):
        raise ValueError(f"{label} rotation determinant must be +1")


def _residual_summary(values: np.ndarray, unit: str) -> dict[str, float]:
    return {
        f"rms_{unit}": float(math.sqrt(float(np.mean(values**2)))),
        f"max_{unit}": float(np.max(values)),
        f"median_{unit}": float(np.median(values)),
    }

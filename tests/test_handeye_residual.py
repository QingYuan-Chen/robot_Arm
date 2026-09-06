from __future__ import annotations

import math

import numpy as np
import pytest


def _rotation_y(degrees: float) -> np.ndarray:
    value = math.radians(degrees)
    return np.array(
        [[math.cos(value), 0.0, math.sin(value)], [0.0, 1.0, 0.0], [-math.sin(value), 0.0, math.cos(value)]]
    )


def _matrix(rotation=None, translation=(0.0, 0.0, 0.0)) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.eye(3) if rotation is None else rotation
    result[:3, 3] = translation
    return result


def test_multi_pose_fixed_marker_has_near_zero_residual() -> None:
    from rebotarm_calibration.handeye_residual import (
        analyze_handeye_residual,
        matrix_transform,
    )

    end_to_camera = _matrix(_rotation_y(12.0), (-0.085, 0.01, 0.045))
    base_to_marker = _matrix(_rotation_y(-5.0), (0.08, -0.42, 0.18))
    samples = []
    for index, angle in enumerate((-30.0, -15.0, 0.0, 15.0, 30.0)):
        base_to_end = _matrix(
            _rotation_y(angle),
            (-0.02 + index * 0.03, -0.20 - index * 0.015, 0.25 + index * 0.01),
        )
        camera_to_marker = np.linalg.inv(base_to_end @ end_to_camera) @ base_to_marker
        samples.append(
            {
                "label": f"pose_{index}",
                "base_to_end": matrix_transform(base_to_end),
                "camera_to_marker": matrix_transform(camera_to_marker),
            }
        )
    report = analyze_handeye_residual(
        {
            "schema_version": 1,
            "end_to_camera": matrix_transform(end_to_camera),
            "samples": samples,
        }
    )
    assert report["position_residual"]["max_m"] < 1e-12
    assert report["rotation_residual"]["max_deg"] < 1e-6
    assert report["pose_diversity"]["pass"] is True
    assert report["acceptance"]["pass"] is True


def test_residual_reports_translation_error_and_insufficient_diversity() -> None:
    from rebotarm_calibration.handeye_residual import (
        analyze_handeye_residual,
        matrix_transform,
    )

    identity = _matrix()
    samples = []
    for index in range(5):
        marker = _matrix(translation=(0.001 * index, 0.0, 0.0))
        samples.append(
            {
                "base_to_end": matrix_transform(identity),
                "camera_to_marker": matrix_transform(marker),
            }
        )
    report = analyze_handeye_residual(
        {"schema_version": 1, "end_to_camera": matrix_transform(identity), "samples": samples}
    )
    assert report["position_residual"]["max_m"] == pytest.approx(0.002)
    assert report["pose_diversity"]["pass"] is False
    assert report["acceptance"]["pass"] is False


def test_residual_rejects_too_few_samples() -> None:
    from rebotarm_calibration.handeye_residual import analyze_handeye_residual

    with pytest.raises(ValueError, match="at least 5 samples"):
        analyze_handeye_residual({"schema_version": 1, "end_to_camera": {}, "samples": []})


def test_matrix_transform_rejects_non_rigid_matrix() -> None:
    from rebotarm_calibration.handeye_residual import matrix_transform

    invalid = np.eye(4)
    invalid[0, 0] = 2.0
    with pytest.raises(ValueError, match="rotation must be orthonormal"):
        matrix_transform(invalid)

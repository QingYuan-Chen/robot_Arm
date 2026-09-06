from __future__ import annotations

import cv2
import numpy as np
import pytest


def test_detect_generated_aruco_pose() -> None:
    from rebotarm_calibration.aruco_pose import detect_aruco_pose

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 300)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 0, 300)
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    image[90:390, 170:470] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    result = detect_aruco_pose(
        image,
        camera_matrix=np.array([[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]),
        distortion=[0.0] * 5,
        marker_length_m=0.1,
    )
    assert result["marker_id"] == 0
    assert result["corner_refinement"] == "CORNER_REFINE_SUBPIX"
    assert result["area_px2"] > 80_000
    assert result["reprojection_rmse_px"] < 1.0
    assert result["camera_to_marker"]["translation"][2] == pytest.approx(0.173, abs=0.01)


def test_detect_aruco_pose_rejects_missing_target() -> None:
    from rebotarm_calibration.aruco_pose import detect_aruco_pose

    with pytest.raises(ValueError, match="not detected"):
        detect_aruco_pose(
            np.full((100, 100, 3), 255, dtype=np.uint8),
            camera_matrix=np.eye(3),
            distortion=[0.0] * 5,
            marker_length_m=0.1,
        )


def test_detector_uses_subpixel_corner_refinement() -> None:
    import cv2

    from rebotarm_calibration.aruco_pose import detector_parameters

    parameters = detector_parameters()
    assert parameters.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX

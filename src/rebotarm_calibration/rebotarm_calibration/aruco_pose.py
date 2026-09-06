from __future__ import annotations

import math
from typing import Sequence

import cv2
import numpy as np

from .handeye_residual import matrix_transform


def detector_parameters():
    parameters = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector")
        else cv2.aruco.DetectorParameters_create()
    )
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return parameters


def detect_aruco_pose(
    color_bgr: np.ndarray,
    *,
    camera_matrix: np.ndarray,
    distortion: Sequence[float],
    marker_length_m: float,
    dictionary_name: str = "DICT_4X4_50",
    marker_id: int = 0,
) -> dict[str, object]:
    if color_bgr is None or color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
        raise ValueError("color_bgr must be a BGR image")
    length = float(marker_length_m)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("marker_length_m must be finite and positive")
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"unsupported ArUco dictionary: {dictionary_name}")
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("camera_matrix must be finite 3x3")
    coefficients = np.asarray(list(distortion), dtype=np.float64).reshape(-1, 1)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    parameters = detector_parameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(color_bgr)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            color_bgr,
            dictionary,
            parameters=parameters,
        )
    if ids is None:
        raise ValueError("target ArUco marker not detected")
    flat_ids = [int(value[0]) for value in ids]
    if int(marker_id) not in flat_ids:
        raise ValueError(f"ArUco marker id {marker_id} not detected")
    image_points = np.asarray(corners[flat_ids.index(int(marker_id))][0], dtype=np.float64)
    half = length * 0.5
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        matrix,
        coefficients,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        raise ValueError("ArUco IPPE pose solve failed")
    rotation, _ = cv2.Rodrigues(rvec)
    camera_to_marker = np.eye(4, dtype=np.float64)
    camera_to_marker[:3, :3] = rotation
    camera_to_marker[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, matrix, coefficients)
    projected = projected.reshape(-1, 2)
    reprojection_rmse = math.sqrt(float(np.mean(np.sum((projected - image_points) ** 2, axis=1))))
    return {
        "marker_id": int(marker_id),
        "corner_refinement": "CORNER_REFINE_SUBPIX",
        "corners_px": image_points.tolist(),
        "center_px": np.mean(image_points, axis=0).tolist(),
        "area_px2": abs(float(cv2.contourArea(image_points.astype(np.float32)))),
        "reprojection_rmse_px": reprojection_rmse,
        "camera_to_marker": matrix_transform(camera_to_marker),
    }

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

from .transform_points import Transform3D, transform_point


def resolve_aruco_dictionary_id(name: str | int, *, cv2_module=cv2) -> int:
    if isinstance(name, int):
        return int(name)

    normalized = str(name).strip().upper()
    if normalized.isdigit():
        return int(normalized)
    candidates = [normalized]
    if not normalized.startswith("DICT_"):
        candidates.append(f"DICT_{normalized}")

    for candidate in candidates:
        if hasattr(cv2_module.aruco, candidate):
            return int(getattr(cv2_module.aruco, candidate))
    raise ValueError(f"unsupported ArUco dictionary: {name}")


def build_camera_matrix(
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    return np.array(
        [
            [float(fx), 0.0, float(cx)],
            [0.0, float(fy), float(cy)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def detect_aruco_center_in_camera(
    color_bgr: np.ndarray,
    *,
    camera_matrix: np.ndarray,
    marker_length_m: float,
    dictionary_name: str | int = "DICT_4X4_50",
    marker_id: int = 0,
    dist_coeffs: Sequence[float] | None = None,
) -> tuple[float, float, float]:
    if color_bgr is None:
        raise ValueError("missing color image")
    if marker_length_m <= 0.0:
        raise ValueError("marker_length_m must be positive")

    dictionary_id = resolve_aruco_dictionary_id(dictionary_name)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(color_bgr)
    if ids is None or len(ids) == 0:
        raise ValueError("target ArUco marker not detected")

    flat_ids = [int(value[0]) for value in ids]
    if int(marker_id) not in flat_ids:
        raise ValueError(f"ArUco marker id {marker_id} not detected")
    index = flat_ids.index(int(marker_id))

    distortion = np.zeros((5, 1), dtype=np.float64)
    if dist_coeffs is not None:
        distortion = np.asarray(list(dist_coeffs), dtype=np.float64).reshape(-1, 1)

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[index]],
        float(marker_length_m),
        camera_matrix,
        distortion,
    )
    tvec = np.asarray(tvecs[0][0], dtype=np.float64)
    return (float(tvec[0]), float(tvec[1]), float(tvec[2]))


def transform_camera_point_to_base(
    transform_stamped,
    *,
    camera_point_xyz: Sequence[float],
) -> tuple[float, float, float]:
    transform = transform_stamped.transform
    translation = transform.translation
    rotation = transform.rotation
    return transform_point(
        Transform3D(
            translation=(float(translation.x), float(translation.y), float(translation.z)),
            rotation_xyzw=(float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)),
        ),
        (
            float(camera_point_xyz[0]),
            float(camera_point_xyz[1]),
            float(camera_point_xyz[2]),
        ),
    )

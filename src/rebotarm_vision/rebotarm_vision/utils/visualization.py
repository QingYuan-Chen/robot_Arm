from __future__ import annotations

import cv2
import numpy as np


def draw_detections(image_bgr, detection_msg):
    annotated = image_bgr.copy()
    for det in detection_msg.detections:
        cv2.rectangle(
            annotated,
            (det.x_min, det.y_min),
            (det.x_max, det.y_max),
            (0, 255, 0),
            2,
        )
        if det.has_obb and len(det.obb_points_xy) == 8:
            points = np.asarray(det.obb_points_xy, dtype=np.float32).reshape(4, 2)
            cv2.polylines(
                annotated,
                [np.round(points).astype(np.int32)],
                True,
                (255, 200, 0),
                2,
                cv2.LINE_AA,
            )
        elif getattr(det, "has_mask", False) and len(det.mask_polygon_xy) >= 6:
            points = np.asarray(det.mask_polygon_xy, dtype=np.float32).reshape(-1, 2)
            cv2.polylines(
                annotated,
                [np.round(points).astype(np.int32)],
                True,
                (255, 120, 0),
                2,
                cv2.LINE_AA,
            )
        label = f"{det.class_name} {det.confidence:.2f}"
        cv2.putText(
            annotated,
            label,
            (det.x_min, max(det.y_min - 8, 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return annotated

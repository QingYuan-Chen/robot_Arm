from __future__ import annotations

from typing import Any

import numpy as np
from rebotarm_msgs.msg import Detection2D, Detection2DArray


def _tensor_to_numpy(value: Any):
    if value is None:
        return None
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def _safe_attr_row(container: Any, attr: str, index: int):
    values = getattr(container, attr, None)
    if values is None:
        return None
    try:
        return _tensor_to_numpy(values[index])
    except Exception:
        return None


def _obb_metadata(result: Any, index: int):
    obb = getattr(result, "obb", None)
    if obb is None:
        return None
    xywhr = _safe_attr_row(obb, "xywhr", index)
    corners = _safe_attr_row(obb, "xyxyxyxy", index)
    if xywhr is None:
        return None
    xywhr = np.asarray(xywhr).reshape(-1)
    if xywhr.size < 5:
        return None
    points = []
    if corners is not None:
        corners = np.asarray(corners, dtype=np.float32)
        if corners.ndim == 3 and corners.shape[0] == 1:
            corners = corners[0]
        if corners.ndim == 1 and corners.size == 8:
            corners = corners.reshape(4, 2)
        if corners.shape == (4, 2):
            points = [float(v) for v in corners.reshape(-1)]
    return {
        "cx": float(xywhr[0]),
        "cy": float(xywhr[1]),
        "w": float(xywhr[2]),
        "h": float(xywhr[3]),
        "theta": float(xywhr[4]),
        "points": points,
    }


def result_to_detection_array_msg(results, stamp, frame_id: str) -> Detection2DArray:
    msg = Detection2DArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    for result in results:
        names = getattr(result, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue
        for index in range(len(boxes)):
            box = boxes[index]
            det = Detection2D()
            det.header.stamp = stamp
            det.header.frame_id = frame_id

            xyxy = np.asarray(_tensor_to_numpy(box.xyxy[0])).reshape(-1)
            cls_id = int(np.asarray(_tensor_to_numpy(box.cls[0])).reshape(-1)[0])
            conf = float(np.asarray(_tensor_to_numpy(box.conf[0])).reshape(-1)[0])

            x1, y1, x2, y2 = [int(v) for v in xyxy[:4]]
            det.class_name = names.get(cls_id, str(cls_id))
            det.confidence = conf
            det.center_u = int(round((x1 + x2) / 2.0))
            det.center_v = int(round((y1 + y2) / 2.0))
            det.x_min = x1
            det.y_min = y1
            det.x_max = x2
            det.y_max = y2

            obb_meta = _obb_metadata(result, index)
            if obb_meta is not None:
                det.has_obb = True
                det.obb_cx = obb_meta["cx"]
                det.obb_cy = obb_meta["cy"]
                det.obb_w = obb_meta["w"]
                det.obb_h = obb_meta["h"]
                det.obb_theta = obb_meta["theta"]
                det.obb_points_xy = obb_meta["points"]
            else:
                det.has_obb = False
                det.obb_points_xy = []

            msg.detections.append(det)

    return msg

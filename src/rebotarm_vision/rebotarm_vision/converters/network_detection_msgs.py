from __future__ import annotations

from typing import Any

from rebotarm_msgs.msg import Detection2D, Detection2DArray


def _float_list(values: Any) -> list[float]:
    if not isinstance(values, list):
        return []
    return [float(value) for value in values]


def detection_json_to_msg(payload: dict[str, Any], stamp, fallback_frame_id: str) -> Detection2DArray:
    msg = Detection2DArray()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = str(payload.get("frame_id") or fallback_frame_id)

    detections = payload.get("detections", [])
    if not isinstance(detections, list):
        return msg

    for item in detections:
        if not isinstance(item, dict):
            continue
        bbox = item.get("bbox_xyxy", [])
        if not isinstance(bbox, list) or len(bbox) < 4:
            continue
        x1, y1, x2, y2 = [int(round(float(value))) for value in bbox[:4]]

        det = Detection2D()
        if stamp is not None:
            det.header.stamp = stamp
        det.header.frame_id = msg.header.frame_id
        det.class_name = str(item.get("class_name", ""))
        det.confidence = float(item.get("confidence", 0.0))
        det.x_min = x1
        det.y_min = y1
        det.x_max = x2
        det.y_max = y2
        det.center_u = int(round((x1 + x2) / 2.0))
        det.center_v = int(round((y1 + y2) / 2.0))

        obb = item.get("obb")
        if isinstance(obb, dict):
            points = _float_list(obb.get("points_xy", []))
            det.has_obb = True
            det.obb_cx = float(obb.get("cx", 0.0))
            det.obb_cy = float(obb.get("cy", 0.0))
            det.obb_w = float(obb.get("w", 0.0))
            det.obb_h = float(obb.get("h", 0.0))
            det.obb_theta = float(obb.get("theta", 0.0))
            det.obb_points_xy = points
        else:
            det.has_obb = False
            det.obb_points_xy = []

        mask = item.get("mask")
        if isinstance(mask, dict):
            polygon = _float_list(mask.get("polygon_xy", []))
            det.has_mask = len(polygon) >= 6
            det.mask_polygon_xy = polygon
        else:
            det.has_mask = False
            det.mask_polygon_xy = []

        msg.detections.append(det)

    return msg

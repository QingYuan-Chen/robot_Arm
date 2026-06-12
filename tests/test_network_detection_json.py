from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import types

ROS2_ROOT = Path(__file__).resolve().parents[1]
path_str = str(ROS2_ROOT / "src" / "rebotarm_vision")
if path_str not in sys.path:
    sys.path.insert(0, path_str)


def _install_ros_message_stubs_if_needed():
    if "rebotarm_msgs" in sys.modules:
        return
    if importlib.util.find_spec("rebotarm_msgs") is not None:
        return

    class Header:
        def __init__(self):
            self.frame_id = ""
            self.stamp = None

    class Detection2D:
        def __init__(self):
            self.header = Header()
            self.class_name = ""
            self.confidence = 0.0
            self.center_u = 0
            self.center_v = 0
            self.x_min = 0
            self.y_min = 0
            self.x_max = 0
            self.y_max = 0
            self.has_obb = False
            self.obb_cx = 0.0
            self.obb_cy = 0.0
            self.obb_w = 0.0
            self.obb_h = 0.0
            self.obb_theta = 0.0
            self.obb_points_xy = []
            self.has_mask = False
            self.mask_polygon_xy = []

    class Detection2DArray:
        def __init__(self):
            self.header = Header()
            self.detections = []

    rebotarm_msgs = types.ModuleType("rebotarm_msgs")
    rebotarm_msgs_msg = types.ModuleType("rebotarm_msgs.msg")
    rebotarm_msgs_msg.Detection2D = Detection2D
    rebotarm_msgs_msg.Detection2DArray = Detection2DArray
    sys.modules["rebotarm_msgs"] = rebotarm_msgs
    sys.modules["rebotarm_msgs.msg"] = rebotarm_msgs_msg


def test_network_detection_json_converts_to_detection_array_msg():
    _install_ros_message_stubs_if_needed()

    from rebotarm_vision.converters.network_detection_msgs import (
        detection_json_to_msg,
    )

    payload = {
        "frame_id": "windows_gemini",
        "detections": [
            {
                "class_name": "water bottle",
                "confidence": 0.91,
                "bbox_xyxy": [10.2, 20.6, 110.8, 220.1],
                "obb": {
                    "cx": 60.0,
                    "cy": 120.0,
                    "w": 50.0,
                    "h": 160.0,
                    "theta": 0.25,
                    "points_xy": [40, 50, 80, 50, 80, 190, 40, 190],
                },
                "mask": {
                    "polygon_xy": [20, 30, 100, 30, 100, 210, 20, 210],
                },
            }
        ],
    }

    msg = detection_json_to_msg(payload, stamp=None, fallback_frame_id="camera_color_frame")

    assert msg.header.frame_id == "windows_gemini"
    assert len(msg.detections) == 1
    det = msg.detections[0]
    assert det.class_name == "water bottle"
    assert det.confidence == 0.91
    assert (det.x_min, det.y_min, det.x_max, det.y_max) == (10, 21, 111, 220)
    assert (det.center_u, det.center_v) == (60, 120)
    assert det.has_obb is True
    assert list(det.obb_points_xy) == [40.0, 50.0, 80.0, 50.0, 80.0, 190.0, 40.0, 190.0]
    assert det.has_mask is True
    assert list(det.mask_polygon_xy) == [20.0, 30.0, 100.0, 30.0, 100.0, 210.0, 20.0, 210.0]

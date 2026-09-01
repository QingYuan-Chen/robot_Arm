from __future__ import annotations

import sys
from types import SimpleNamespace
import types

import numpy as np


def _install_detection_message_stubs_if_needed() -> None:
    if "rebotarm_msgs.msg" in sys.modules:
        return

    class Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

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
    rebotarm_msgs.msg = rebotarm_msgs_msg
    sys.modules["rebotarm_msgs"] = rebotarm_msgs
    sys.modules["rebotarm_msgs.msg"] = rebotarm_msgs_msg


def test_ultralytics_segmentation_polygon_is_preserved_in_detection_message():
    _install_detection_message_stubs_if_needed()
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg

    box = SimpleNamespace(
        xyxy=np.asarray([[10.0, 20.0, 110.0, 220.0]], dtype=np.float32),
        cls=np.asarray([0.0], dtype=np.float32),
        conf=np.asarray([0.91], dtype=np.float32),
    )
    result = SimpleNamespace(
        names={0: "bottle"},
        boxes=[box],
        obb=None,
        masks=SimpleNamespace(
            xy=[
                np.asarray(
                    [[20.0, 30.0], [100.0, 30.0], [100.0, 210.0], [20.0, 210.0]],
                    dtype=np.float32,
                )
            ]
        ),
    )

    msg = result_to_detection_array_msg([result], stamp=None, frame_id="camera_color_frame")

    assert len(msg.detections) == 1
    detection = msg.detections[0]
    assert detection.class_name == "bottle"
    assert detection.has_mask is True
    assert list(detection.mask_polygon_xy) == [
        20.0,
        30.0,
        100.0,
        30.0,
        100.0,
        210.0,
        20.0,
        210.0,
    ]


def test_invalid_ultralytics_segmentation_polygon_fails_closed_to_no_mask():
    _install_detection_message_stubs_if_needed()
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg

    box = SimpleNamespace(
        xyxy=np.asarray([[10.0, 20.0, 110.0, 220.0]], dtype=np.float32),
        cls=np.asarray([0.0], dtype=np.float32),
        conf=np.asarray([0.91], dtype=np.float32),
    )
    result = SimpleNamespace(
        names={0: "bottle"},
        boxes=[box],
        obb=None,
        masks=SimpleNamespace(xy=[np.asarray([[20.0, 30.0], [100.0, 30.0]], dtype=np.float32)]),
    )

    msg = result_to_detection_array_msg([result], stamp=None, frame_id="camera_color_frame")

    detection = msg.detections[0]
    assert detection.has_mask is False
    assert list(detection.mask_polygon_xy) == []

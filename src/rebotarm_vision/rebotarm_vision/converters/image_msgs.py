from __future__ import annotations

import numpy as np
from sensor_msgs.msg import CameraInfo, Image


def color_to_msg(image_bgr, stamp, frame_id: str) -> Image:
    image_bgr = np.ascontiguousarray(image_bgr)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(image_bgr.shape[0])
    msg.width = int(image_bgr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(image_bgr.shape[1] * image_bgr.shape[2])
    msg.data = image_bgr.tobytes()
    return msg


def depth_to_msg(depth_mm, stamp, frame_id: str) -> Image:
    depth_mm = np.ascontiguousarray(depth_mm)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(depth_mm.shape[0])
    msg.width = int(depth_mm.shape[1])
    msg.encoding = "mono16"
    msg.is_bigendian = 0
    msg.step = int(depth_mm.shape[1] * 2)
    msg.data = depth_mm.tobytes()
    return msg


def camera_info_to_msg(camera_info: dict, stamp, frame_id: str) -> CameraInfo:
    fx = float(camera_info.get("fx", camera_info.get("k", [0.0, 0.0, 0.0])[0]))
    fy = float(camera_info.get("fy", camera_info.get("k", [0.0, 0.0, 0.0, 0.0, 0.0])[4]))
    cx = float(camera_info.get("cx", camera_info.get("k", [0.0, 0.0, 0.0])[2]))
    cy = float(camera_info.get("cy", camera_info.get("k", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])[5]))
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(camera_info.get("height", 0))
    msg.width = int(camera_info.get("width", 0))
    msg.distortion_model = str(camera_info.get("distortion_model", "plumb_bob"))
    msg.d = [float(value) for value in camera_info.get("d", camera_info.get("D", []))]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg

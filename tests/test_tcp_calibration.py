from __future__ import annotations

from pathlib import Path
import sys
import types

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_PATH = str(ROS2_ROOT / "src" / "rebotarm_vision")
if VISION_PATH not in sys.path:
    sys.path.insert(0, VISION_PATH)


class _Point:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:
    def __init__(self, x: float, y: float, z: float, w: float):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Transform:
    def __init__(self):
        self.translation = _Point(0.40, 0.10, 0.20)
        self.rotation = _Quaternion(0.0, 0.0, 0.0, 1.0)


class _TransformStamped:
    def __init__(self):
        self.transform = _Transform()


def _install_ros_stubs_if_needed():
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.duration = types.SimpleNamespace(Duration=lambda seconds=0.0: seconds)
    rclpy.time = types.SimpleNamespace(Time=lambda: None)
    sys.modules["rclpy"] = rclpy

    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.ExternalShutdownException = RuntimeError
    sys.modules["rclpy.executors"] = rclpy_executors

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy.node"] = rclpy_node

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.Buffer = object
    tf2_ros.TransformListener = object
    sys.modules["tf2_ros"] = tf2_ros


def test_sample_estimates_offset_in_end_link_frame_with_identity_rotation():
    from rebotarm_vision.tcp_calibration import estimate_sample_offset

    offset = estimate_sample_offset(
        end_link_position=(0.40, 0.10, 0.20),
        end_link_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        tcp_reference_position=(0.45, 0.08, 0.23),
    )

    assert offset == pytest.approx((0.05, -0.02, 0.03))


def test_sample_estimates_offset_in_rotated_end_link_frame():
    from rebotarm_vision.tcp_calibration import estimate_sample_offset

    offset = estimate_sample_offset(
        end_link_position=(1.0, 2.0, 3.0),
        end_link_orientation_xyzw=(0.0, 0.0, 0.7071067811865476, 0.7071067811865476),
        tcp_reference_position=(1.0, 2.1, 3.0),
    )

    assert offset == pytest.approx((0.1, 0.0, 0.0), abs=1e-6)


def test_average_offset_rejects_empty_samples():
    from rebotarm_vision.tcp_calibration import average_offsets

    with pytest.raises(ValueError, match="at least one"):
        average_offsets([])


def test_average_offset_formats_camera_yaml_snippet():
    from rebotarm_vision.tcp_calibration import average_offsets, format_tcp_offset_yaml

    offset = average_offsets(
        [
            (0.0501, -0.0202, 0.0303),
            (0.0499, -0.0198, 0.0297),
        ]
    )

    assert offset == pytest.approx((0.05, -0.02, 0.03))
    assert format_tcp_offset_yaml(offset) == "tcp_offset_xyz: [0.050000, -0.020000, 0.030000]"


def test_estimate_offset_from_transform_stamped_uses_tf_fields():
    _install_ros_stubs_if_needed()

    from rebotarm_vision.tcp_calibration_node import estimate_offset_from_transform

    offset = estimate_offset_from_transform(
        _TransformStamped(),
        tcp_reference_position=(0.45, 0.08, 0.23),
    )

    assert offset == pytest.approx((0.05, -0.02, 0.03))

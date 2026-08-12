from __future__ import annotations

from pathlib import Path
import sys
import types

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_PATH = str(ROS2_ROOT / "src" / "rebotarm_vision")
if VISION_PATH not in sys.path:
    sys.path.insert(0, VISION_PATH)
CALIBRATION_PATH = str(ROS2_ROOT / "src" / "rebotarm_calibration")
if CALIBRATION_PATH not in sys.path:
    sys.path.insert(0, CALIBRATION_PATH)


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
        if "rclpy.qos" not in sys.modules:
            rclpy_qos = types.ModuleType("rclpy.qos")
            rclpy_qos.qos_profile_sensor_data = object()
            sys.modules["rclpy.qos"] = rclpy_qos
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

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    sys.modules["rclpy.qos"] = rclpy_qos

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


def test_tcp_analysis_accepts_repeatable_diverse_samples():
    import math

    import numpy as np

    from rebotarm_calibration.tcp_calibration import analyze_tcp_samples

    expected = np.array((-0.04, 0.002, -0.001))

    def quaternion(axis, degrees):
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)
        half = math.radians(degrees) * 0.5
        xyz = axis * math.sin(half)
        return (*xyz, math.cos(half))

    samples = []
    for index, orientation in enumerate(
        (
            quaternion((1, 0, 0), 0),
            quaternion((1, 0, 0), 25),
            quaternion((0, 1, 0), -25),
            quaternion((0, 0, 1), 30),
            quaternion((1, 1, 0), 22),
        )
    ):
        from rebotarm_calibration.tcp_calibration import quaternion_to_rotation_matrix

        end = np.array((0.3 + index * 0.01, -0.2, 0.25))
        reference = end + quaternion_to_rotation_matrix(orientation) @ expected
        samples.append(
            {
                "end_link_position": end,
                "end_link_orientation_xyzw": orientation,
                "tcp_reference_position": reference,
            }
        )

    result = analyze_tcp_samples(samples)

    assert result["passed"] is True
    assert result["tcp_offset_xyz"] == pytest.approx(expected)
    assert result["rms_residual_m"] == pytest.approx(0.0, abs=1e-12)
    assert result["rotation_span_deg"] >= 20.0


def test_tcp_analysis_rejects_insufficient_orientation_diversity():
    from rebotarm_calibration.tcp_calibration import analyze_tcp_samples

    samples = [
        {
            "end_link_position": (0.4, 0.1, 0.2),
            "end_link_orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
            "tcp_reference_position": (0.36, 0.1, 0.2),
        }
        for _ in range(5)
    ]

    result = analyze_tcp_samples(samples)

    assert result["passed"] is False
    assert result["gates"]["rotation_span"] is False


def test_tcp_pivot_analysis_solves_unknown_reference_and_offset():
    import math

    import numpy as np

    from rebotarm_calibration.tcp_calibration import (
        analyze_tcp_pivot_samples,
        quaternion_to_rotation_matrix,
    )

    tcp = np.array((-0.041, 0.002, -0.003))
    pivot = np.array((0.25, -0.30, 0.18))

    def quaternion(axis, degrees):
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)
        half = math.radians(degrees) * 0.5
        return (*tuple(axis * math.sin(half)), math.cos(half))

    orientations = (
        quaternion((1, 0, 0), 0),
        quaternion((1, 0, 0), 35),
        quaternion((0, 1, 0), -40),
        quaternion((0, 0, 1), 45),
        quaternion((1, 1, 0), 50),
        quaternion((0, 1, 1), -45),
    )
    samples = []
    for orientation in orientations:
        rotation = quaternion_to_rotation_matrix(orientation)
        samples.append(
            {
                "end_link_position": pivot - rotation @ tcp,
                "end_link_orientation_xyzw": orientation,
            }
        )

    result = analyze_tcp_pivot_samples(samples)

    assert result["passed"] is True
    assert result["tcp_offset_xyz"] == pytest.approx(tcp, abs=1e-12)
    assert result["pivot_position_base_xyz"] == pytest.approx(pivot, abs=1e-12)
    assert result["matrix_rank"] == 6
    assert result["rms_residual_m"] == pytest.approx(0.0, abs=1e-12)


def test_tcp_pivot_analysis_rejects_repeated_orientation_rank_loss():
    from rebotarm_calibration.tcp_calibration import analyze_tcp_pivot_samples

    samples = [
        {
            "end_link_position": (0.30, -0.20, 0.20),
            "end_link_orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        }
        for _ in range(5)
    ]

    result = analyze_tcp_pivot_samples(samples)

    assert result["passed"] is False
    assert result["gates"]["full_rank"] is False

from __future__ import annotations

import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from .aruco_pose import detect_aruco_pose
from .tcp_calibration import (
    analyze_tcp_pivot_samples,
    analyze_tcp_samples,
    estimate_sample_offset,
    format_tcp_offset_yaml,
    quaternion_to_rotation_matrix,
)


def _tuple3(values, name: str) -> tuple[float, float, float]:
    items = list(values)
    if len(items) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return (float(items[0]), float(items[1]), float(items[2]))


def _transform_point(transform_stamped, point) -> tuple[float, float, float]:
    transform = transform_stamped.transform
    rotation = transform.rotation
    matrix = quaternion_to_rotation_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    translation = np.array(
        (transform.translation.x, transform.translation.y, transform.translation.z),
        dtype=np.float64,
    )
    result = translation + matrix @ np.asarray(point, dtype=np.float64)
    return tuple(float(value) for value in result)


def estimate_offset_from_transform(
    transform_stamped,
    *,
    tcp_reference_position: tuple[float, float, float],
) -> tuple[float, float, float]:
    transform = transform_stamped.transform
    translation = transform.translation
    rotation = transform.rotation
    return estimate_sample_offset(
        end_link_position=(translation.x, translation.y, translation.z),
        end_link_orientation_xyzw=(rotation.x, rotation.y, rotation.z, rotation.w),
        tcp_reference_position=tcp_reference_position,
    )


class AutoArucoReferenceProvider:
    """Compatibility provider used by older callers and focused tests."""

    def __init__(
        self,
        *,
        driver,
        tf_buffer,
        base_frame: str,
        camera_frame: str,
        lookup_timeout_sec: float,
        detector,
    ) -> None:
        self.driver = driver
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.lookup_timeout_sec = lookup_timeout_sec
        self.detector = detector

    def reference_position(self) -> tuple[float, float, float]:
        color_bgr, _ = self.driver.get_frame()
        if color_bgr is None:
            raise RuntimeError("failed to capture color image for ArUco reference")
        camera_point = self.detector(color_bgr)
        tf_msg = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )
        return _transform_point(tf_msg, camera_point)


class TcpCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_tcp_calibration")
        self.declare_parameter("reference_mode", "pivot")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_link_frame", "end_link")
        self.declare_parameter("tcp_reference_position", [0.0, 0.0, 0.0])
        self.declare_parameter("sample_count", 5)
        self.declare_parameter("preflight_only", False)
        self.declare_parameter("lookup_timeout_sec", 0.5)
        self.declare_parameter("output_path", "")
        self.declare_parameter("aruco.image_topic", "/camera/color/image_raw")
        self.declare_parameter("aruco.camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("aruco.camera_frame", "camera_depth_frame")
        self.declare_parameter("aruco.dictionary", "DICT_4X4_50")
        self.declare_parameter("aruco.marker_id", 0)
        self.declare_parameter("aruco.marker_length_m", 0.10)
        self.declare_parameter("aruco.reference_frames", 30)
        self.declare_parameter("aruco.maximum_frames", 45)
        self.declare_parameter("aruco.maximum_reference_std_m", 0.002)

        self.reference_mode = str(self.get_parameter("reference_mode").value).strip().lower()
        if self.reference_mode not in {"pivot", "manual", "aruco"}:
            raise ValueError("reference_mode must be 'pivot', 'manual', or 'aruco'")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.end_link_frame = str(self.get_parameter("end_link_frame").value)
        self.camera_frame = str(self.get_parameter("aruco.camera_frame").value)
        self.tcp_reference_position = _tuple3(
            self.get_parameter("tcp_reference_position").value,
            "tcp_reference_position",
        )
        self.sample_count = max(1, int(self.get_parameter("sample_count").value))
        self.preflight_only = bool(self.get_parameter("preflight_only").value)
        self.lookup_timeout_sec = max(0.05, float(self.get_parameter("lookup_timeout_sec").value))
        self.output_path = str(self.get_parameter("output_path").value).strip()
        self.reference_frames = max(1, int(self.get_parameter("aruco.reference_frames").value))
        self.maximum_frames = max(
            self.reference_frames,
            int(self.get_parameter("aruco.maximum_frames").value),
        )
        self.maximum_reference_std_m = max(
            0.0, float(self.get_parameter("aruco.maximum_reference_std_m").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.latest_image: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.samples: list[dict[str, object]] = []
        self.reference_capture: dict[str, object] | None = None
        if self.reference_mode == "aruco":
            self.create_subscription(
                Image,
                str(self.get_parameter("aruco.image_topic").value),
                self._image_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("aruco.camera_info_topic").value),
                self._camera_info_callback,
                qos_profile_sensor_data,
            )

    def _image_callback(self, message: Image) -> None:
        self.latest_image = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.latest_camera_info = message

    def _lookup(self, target: str, source: str):
        return self.tf_buffer.lookup_transform(
            target,
            source,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )

    def prepare_reference(self) -> tuple[float, float, float]:
        if self.reference_mode == "pivot":
            self.reference_capture = {
                "mode": "classic_tcp_pivot_unknown_reference",
                "instruction": "keep one physical pivot point fixed for every sample",
            }
            return self.tcp_reference_position
        if self.reference_mode == "manual":
            self.reference_capture = {
                "mode": "manual",
                "reference_position": list(self.tcp_reference_position),
            }
            return self.tcp_reference_position

        accepted: list[tuple[float, float, float]] = []
        normals: list[np.ndarray] = []
        attempts = 0
        last_stamp = None
        dictionary = str(self.get_parameter("aruco.dictionary").value)
        marker_id = int(self.get_parameter("aruco.marker_id").value)
        marker_length = float(self.get_parameter("aruco.marker_length_m").value)
        while attempts < self.maximum_frames and len(accepted) < self.reference_frames:
            rclpy.spin_once(self, timeout_sec=0.2)
            image = self.latest_image
            info = self.latest_camera_info
            if image is None or info is None:
                continue
            stamp = (int(image.header.stamp.sec), int(image.header.stamp.nanosec))
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            attempts += 1
            try:
                color = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
                marker = detect_aruco_pose(
                    color,
                    camera_matrix=matrix,
                    distortion=list(info.d),
                    marker_length_m=marker_length,
                    dictionary_name=dictionary,
                    marker_id=marker_id,
                )
                camera_point = marker["camera_to_marker"]["translation"]
                base_camera = self._lookup(self.base_frame, self.camera_frame)
                accepted.append(_transform_point(base_camera, camera_point))
                base_rotation = base_camera.transform.rotation
                base_to_camera_rotation = quaternion_to_rotation_matrix(
                    (base_rotation.x, base_rotation.y, base_rotation.z, base_rotation.w)
                )
                camera_to_marker_rotation = quaternion_to_rotation_matrix(
                    marker["camera_to_marker"]["rotation_xyzw"]
                )
                normals.append((base_to_camera_rotation @ camera_to_marker_rotation)[:, 2])
            except Exception as exc:
                self.get_logger().warn(f"ArUco reference frame rejected: {exc}")
        if len(accepted) < self.reference_frames:
            raise RuntimeError(
                f"ArUco reference gate failed: accepted={len(accepted)}/{attempts}, "
                f"required={self.reference_frames}"
            )
        values = np.asarray(accepted, dtype=np.float64)
        reference = np.mean(values, axis=0)
        axis_std = np.std(values, axis=0)
        if float(np.max(axis_std)) > self.maximum_reference_std_m:
            raise RuntimeError(
                f"ArUco reference unstable: max axis std={float(np.max(axis_std)):.6f} m"
            )
        self.tcp_reference_position = tuple(float(value) for value in reference)
        mean_normal = np.mean(np.stack(normals), axis=0)
        mean_normal /= np.linalg.norm(mean_normal)
        normal_errors_deg = [
            float(
                np.degrees(
                    np.arccos(np.clip(np.dot(normal / np.linalg.norm(normal), mean_normal), -1.0, 1.0))
                )
            )
            for normal in normals
        ]
        self.reference_capture = {
            "mode": "aruco_frozen_before_alignment",
            "reference_position": reference.tolist(),
            "axis_std_m": axis_std.tolist(),
            "plane_normal_base": mean_normal.tolist(),
            "plane_normal_rms_deg": float(np.sqrt(np.mean(np.square(normal_errors_deg)))),
            "plane_normal_max_deg": float(np.max(normal_errors_deg)),
            "accepted": len(accepted),
            "attempts": attempts,
            "camera_frame": self.camera_frame,
            "marker_length_m": marker_length,
            "marker_id": marker_id,
            "dictionary": dictionary,
        }
        return self.tcp_reference_position

    def capture_sample(self) -> dict[str, object]:
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        tf_msg = self._lookup(self.base_frame, self.end_link_frame)
        transform = tf_msg.transform
        sample: dict[str, object] = {
            "monotonic_ns": time.monotonic_ns(),
            "end_link_position": [
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.translation.z),
            ],
            "end_link_orientation_xyzw": [
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            ],
        }
        if self.reference_mode != "pivot":
            sample["tcp_reference_position"] = list(self.tcp_reference_position)
            sample["offset_xyz"] = list(
                estimate_sample_offset(
                    end_link_position=sample["end_link_position"],
                    end_link_orientation_xyzw=sample["end_link_orientation_xyzw"],
                    tcp_reference_position=self.tcp_reference_position,
                )
            )
        self.samples.append(sample)
        return sample

    def result(self) -> dict[str, object]:
        analysis = (
            analyze_tcp_pivot_samples(self.samples)
            if self.reference_mode == "pivot"
            else analyze_tcp_samples(self.samples)
        )
        return {
            "schema_version": 1,
            "reference_capture": self.reference_capture,
            "samples": self.samples,
            "analysis": analysis,
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpCalibrationNode()
    try:
        reference = node.prepare_reference()
        if node.reference_mode == "pivot":
            node.get_logger().info(
                "Classic TCP pivot mode ready. Keep one sharp physical pivot point fixed "
                "for every sample; the pivot base coordinate will be solved jointly."
            )
        else:
            node.get_logger().info(
                "TCP reference frozen before alignment at "
                f"({reference[0]:+.6f}, {reference[1]:+.6f}, {reference[2]:+.6f}) "
                f"in {node.base_frame}. Keep the fixture fixed."
            )
        if node.preflight_only:
            result = {
                "schema_version": 1,
                "preflight_only": True,
                "reference_capture": node.reference_capture,
            }
            print(json.dumps(result, ensure_ascii=False, indent=2))
            if node.output_path:
                path = Path(node.output_path).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            return
        for index in range(node.sample_count):
            input(
                f"Sample {index + 1}/{node.sample_count}: align the physical grasp TCP "
                "to the frozen reference, keep the actuator in controlled hold, then press Enter..."
            )
            sample = node.capture_sample()
            if "offset_xyz" in sample:
                offset = sample["offset_xyz"]
                node.get_logger().info(
                    f"sample {len(node.samples)} offset="
                    f"({offset[0]:+.6f}, {offset[1]:+.6f}, {offset[2]:+.6f})"
                )
            else:
                node.get_logger().info(f"pivot sample {len(node.samples)} captured")

        result = node.result()
        analysis = result["analysis"]
        print("")
        print("TCP calibration analysis:")
        print(json.dumps(analysis, ensure_ascii=False, indent=2))
        print(format_tcp_offset_yaml(analysis["tcp_offset_xyz"]))
        print("Candidate is not deployed automatically; deploy only when every gate passes.")
        if node.output_path:
            path = Path(node.output_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"raw result written to {path}")
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

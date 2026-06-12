from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .aruco_reference import (
    build_camera_matrix,
    detect_aruco_center_in_camera,
    transform_camera_point_to_base,
)
from .camera.network_mjpeg_driver import NetworkMjpegConfig, NetworkMjpegDriver
from .tcp_calibration import average_offsets, estimate_sample_offset, format_tcp_offset_yaml


def _tuple3(values, name: str) -> tuple[float, float, float]:
    items = list(values)
    if len(items) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return (float(items[0]), float(items[1]), float(items[2]))


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
        return transform_camera_point_to_base(tf_msg, camera_point_xyz=camera_point)


class TcpCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_tcp_calibration")
        self.declare_parameter("reference_mode", "manual")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_link_frame", "end_link")
        self.declare_parameter("tcp_reference_position", [0.0, 0.0, 0.0])
        self.declare_parameter("sample_count", 5)
        self.declare_parameter("lookup_timeout_sec", 0.5)
        self.declare_parameter("aruco.snapshot_url", "http://192.168.145.1:8081/snapshot.jpg")
        self.declare_parameter("aruco.camera_frame", "camera_depth_frame")
        self.declare_parameter("aruco.dictionary", "DICT_4X4_50")
        self.declare_parameter("aruco.marker_id", 0)
        self.declare_parameter("aruco.marker_length_m", 0.10)
        self.declare_parameter("aruco.fx", 692.562744140625)
        self.declare_parameter("aruco.fy", 692.2272338867188)
        self.declare_parameter("aruco.cx", 641.2417602539062)
        self.declare_parameter("aruco.cy", 361.8166198730469)
        self.declare_parameter("aruco.dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])

        self.reference_mode = str(self.get_parameter("reference_mode").value).strip().lower()
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.end_link_frame = str(self.get_parameter("end_link_frame").value)
        self.tcp_reference_position = _tuple3(
            self.get_parameter("tcp_reference_position").value,
            "tcp_reference_position",
        )
        self.sample_count = max(1, int(self.get_parameter("sample_count").value))
        self.lookup_timeout_sec = max(0.05, float(self.get_parameter("lookup_timeout_sec").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.offset_samples: list[tuple[float, float, float]] = []
        self.reference_provider = self._build_reference_provider()

    def _build_reference_provider(self):
        if self.reference_mode == "manual":
            return None
        if self.reference_mode != "aruco":
            raise ValueError("reference_mode must be 'manual' or 'aruco'")

        snapshot_url = str(self.get_parameter("aruco.snapshot_url").value)
        camera_frame = str(self.get_parameter("aruco.camera_frame").value)
        dictionary_name = str(self.get_parameter("aruco.dictionary").value)
        marker_id = int(self.get_parameter("aruco.marker_id").value)
        marker_length_m = float(self.get_parameter("aruco.marker_length_m").value)
        dist_coeffs = list(self.get_parameter("aruco.dist_coeffs").value)
        camera_matrix = build_camera_matrix(
            fx=float(self.get_parameter("aruco.fx").value),
            fy=float(self.get_parameter("aruco.fy").value),
            cx=float(self.get_parameter("aruco.cx").value),
            cy=float(self.get_parameter("aruco.cy").value),
        )
        driver = NetworkMjpegDriver(
            NetworkMjpegConfig(
                snapshot_url=snapshot_url,
                stream_url="",
                frame_timeout_ms=1000,
            )
        )
        driver.open()

        def detector(color_bgr):
            return detect_aruco_center_in_camera(
                color_bgr,
                camera_matrix=camera_matrix,
                marker_length_m=marker_length_m,
                dictionary_name=dictionary_name,
                marker_id=marker_id,
                dist_coeffs=dist_coeffs,
            )

        return AutoArucoReferenceProvider(
            driver=driver,
            tf_buffer=self.tf_buffer,
            base_frame=self.base_frame,
            camera_frame=camera_frame,
            lookup_timeout_sec=self.lookup_timeout_sec,
            detector=detector,
        )

    def capture_sample(self) -> tuple[float, float, float]:
        reference_position = self.tcp_reference_position
        if self.reference_provider is not None:
            reference_position = self.reference_provider.reference_position()
            self.get_logger().info(
                "aruco reference="
                f"({reference_position[0]:+.6f}, {reference_position[1]:+.6f}, "
                f"{reference_position[2]:+.6f}) in {self.base_frame}"
            )
        tf_msg = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.end_link_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )
        offset = estimate_offset_from_transform(
            tf_msg,
            tcp_reference_position=reference_position,
        )
        self.offset_samples.append(offset)
        return offset

    def average_result(self) -> tuple[float, float, float]:
        return average_offsets(self.offset_samples)


def _spin_until_tf_ready(node: TcpCalibrationNode) -> None:
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpCalibrationNode()
    try:
        node.get_logger().info(
            "TCP calibration ready. Align the real gripper center to "
            f"{node.reference_mode} reference in {node.base_frame}, then press Enter "
            f"for each of {node.sample_count} samples."
        )
        _spin_until_tf_ready(node)
        for index in range(node.sample_count):
            input(f"Sample {index + 1}/{node.sample_count}: press Enter after alignment...")
            try:
                offset = node.capture_sample()
            except Exception as exc:
                node.get_logger().warn(f"sample skipped: {exc}")
                continue
            node.get_logger().info(
                "sample "
                f"{len(node.offset_samples)} offset="
                f"({offset[0]:+.6f}, {offset[1]:+.6f}, {offset[2]:+.6f})"
            )
            rclpy.spin_once(node, timeout_sec=0.1)

        result = node.average_result()
        print("")
        print("Suggested camera.yaml value:")
        print(format_tcp_offset_yaml(result))
        print("")
        print("Apply the same value to both rebotarm_grasp_tcp_frame and rebotarm_grasp_preview_sender.")
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

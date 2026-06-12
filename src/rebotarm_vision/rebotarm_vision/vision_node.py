from __future__ import annotations

import os
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rebotarm_msgs.msg import Detection2DArray

from .camera.gemini2_driver import Gemini2Config, Gemini2Driver
from .camera.network_mjpeg_driver import NetworkMjpegConfig, NetworkMjpegDriver
from .converters.detection_msgs import result_to_detection_array_msg
from .converters.image_msgs import camera_info_to_msg, color_to_msg, depth_to_msg
from .converters.network_detection_msgs import detection_json_to_msg
from .detector.network_detection_client import NetworkDetectionClient, NetworkDetectionConfig
from .detector.yolo_detector import YoloDetector
from .utils.visualization import draw_detections


class RebotArmVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_vision_node")

        self._declare_parameters()
        self._load_parameters()
        self._preview_window_created = False

        self.color_pub = self.create_publisher(
            Image,
            "/camera/color/image_raw",
            qos_profile_sensor_data,
        )
        self.depth_pub = self.create_publisher(
            Image,
            "/camera/depth/image_raw",
            qos_profile_sensor_data,
        )
        self.depth_camera_info_pub = self.create_publisher(
            CameraInfo,
            "/camera/depth/camera_info",
            qos_profile_sensor_data,
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            "/grasp/detections",
            10,
        )
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                "/camera/color/annotated",
                qos_profile_sensor_data,
            )

        self.camera = self._create_camera_driver()

        self.detector = None
        self.network_detection_client = None
        if self.enable_network_detection:
            self.network_detection_client = NetworkDetectionClient(
                NetworkDetectionConfig(
                    detections_url=self.network_detections_url,
                    timeout_ms=self.network_detections_timeout_ms,
                )
            )
            self.get_logger().info("network YOLO detections enabled")
        elif self.enable_detection:
            self.detector = YoloDetector(
                model_path=self.model_path,
                device=self.device,
                conf_threshold=self.conf_threshold,
                iou_threshold=self.iou_threshold,
                use_world=self.use_world,
                custom_classes=self.custom_classes,
            )
            if self.use_world:
                if self.detector.open_vocab_enabled:
                    self.get_logger().info("YOLO open-vocabulary prompts enabled")
                elif self.detector.class_prompt_error:
                    self.get_logger().warn(
                        "YOLO open-vocabulary prompts unavailable, "
                        f"falling back to closed-set mode: {self.detector.class_prompt_error}"
                    )

        self.camera.open()
        ready = self.camera.warmup(self.warmup_frames)
        if not ready:
            self.get_logger().warn("camera warmup incomplete, continuing with frame retries")

        self.empty_frame_count = 0
        self._detection_log_countdown = 0
        if self.show_preview:
            display_env = os.environ.get("DISPLAY", "").strip()
            if not display_env:
                self.show_preview = False
                self.get_logger().warn(
                    "preview disabled because DISPLAY is not set; "
                    "launch from the Ubuntu desktop terminal to see the local window"
                )
            else:
                try:
                    cv2.namedWindow(self.preview_window_name, cv2.WINDOW_AUTOSIZE)
                    self._preview_window_created = True
                    self.get_logger().info(
                        f"preview window enabled on DISPLAY={display_env}"
                    )
                except Exception as exc:
                    self.show_preview = False
                    self.get_logger().warn(
                        f"preview disabled because OpenCV window creation failed: {exc}"
                    )
        period = 1.0 / max(self.loop_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info("rebotarm_vision_node initialized")

    def _create_camera_driver(self):
        if self.camera_type == "gemini2":
            return Gemini2Driver(
                Gemini2Config(
                    color_width=self.color_width,
                    color_height=self.color_height,
                    color_fps=self.color_fps,
                    enable_depth=self.enable_depth,
                    depth_width=self.depth_width,
                    depth_height=self.depth_height,
                    depth_fps=self.depth_fps,
                    frame_timeout_ms=self.frame_timeout_ms,
                    enable_align=self.enable_align,
                )
            )
        if self.camera_type == "network_mjpeg":
            return NetworkMjpegDriver(
                NetworkMjpegConfig(
                    snapshot_url=self.network_snapshot_url,
                    stream_url=self.network_stream_url,
                    frame_timeout_ms=self.frame_timeout_ms,
                    depth_url=self.network_depth_url,
                    camera_info_url=self.network_camera_info_url,
                )
            )
        raise RuntimeError(f"unsupported camera.type: {self.camera_type}")

    def _declare_parameters(self) -> None:
        self.declare_parameter("camera.type", "gemini2")
        self.declare_parameter("camera.color_width", 640)
        self.declare_parameter("camera.color_height", 480)
        self.declare_parameter("camera.color_fps", 30)
        self.declare_parameter("camera.color_format", "MJPG")
        self.declare_parameter("camera.enable_depth", True)
        self.declare_parameter("camera.depth_width", 0)
        self.declare_parameter("camera.depth_height", 0)
        self.declare_parameter("camera.depth_fps", 30)
        self.declare_parameter("camera.enable_align", False)
        self.declare_parameter("camera.warmup_frames", 15)
        self.declare_parameter("camera.frame_timeout_ms", 1000)
        self.declare_parameter("camera.max_empty_frames", 30)
        self.declare_parameter("camera.network_snapshot_url", "")
        self.declare_parameter("camera.network_stream_url", "")
        self.declare_parameter("camera.network_depth_url", "")
        self.declare_parameter("camera.network_camera_info_url", "")
        self.declare_parameter("camera.network_detections_url", "")
        self.declare_parameter("camera.network_detections_timeout_ms", 1000)
        self.declare_parameter("yolo.model_path", "")
        self.declare_parameter("yolo.device", "cpu")
        self.declare_parameter("yolo.conf_threshold", 0.5)
        self.declare_parameter("yolo.iou_threshold", 0.45)
        self.declare_parameter("yolo.use_world", True)
        self.declare_parameter("yolo.custom_classes", ["cup", "bottle", "banana"])
        self.declare_parameter("ros.frame_id_color", "camera_color_frame")
        self.declare_parameter("ros.frame_id_depth", "camera_depth_frame")
        self.declare_parameter("ros.publish_annotated", True)
        self.declare_parameter("ros.show_preview", False)
        self.declare_parameter("ros.preview_window_name", "RebotArm Vision Preview")
        self.declare_parameter("ros.loop_rate_hz", 10.0)
        self.declare_parameter("ros.enable_detection", False)
        self.declare_parameter("ros.enable_network_detection", False)

    def _load_parameters(self) -> None:
        self.camera_type = str(self.get_parameter("camera.type").value)
        self.color_width = int(self.get_parameter("camera.color_width").value)
        self.color_height = int(self.get_parameter("camera.color_height").value)
        self.color_fps = int(self.get_parameter("camera.color_fps").value)
        self.enable_depth = bool(self.get_parameter("camera.enable_depth").value)
        self.depth_width = int(self.get_parameter("camera.depth_width").value)
        self.depth_height = int(self.get_parameter("camera.depth_height").value)
        self.depth_fps = int(self.get_parameter("camera.depth_fps").value)
        self.enable_align = bool(self.get_parameter("camera.enable_align").value)
        self.warmup_frames = int(self.get_parameter("camera.warmup_frames").value)
        self.frame_timeout_ms = int(self.get_parameter("camera.frame_timeout_ms").value)
        self.max_empty_frames = int(self.get_parameter("camera.max_empty_frames").value)
        self.network_snapshot_url = str(self.get_parameter("camera.network_snapshot_url").value)
        self.network_stream_url = str(self.get_parameter("camera.network_stream_url").value)
        self.network_depth_url = str(self.get_parameter("camera.network_depth_url").value)
        self.network_camera_info_url = str(self.get_parameter("camera.network_camera_info_url").value)
        self.network_detections_url = str(self.get_parameter("camera.network_detections_url").value)
        self.network_detections_timeout_ms = int(
            self.get_parameter("camera.network_detections_timeout_ms").value
        )
        self.model_path = str(self.get_parameter("yolo.model_path").value)
        self.device = str(self.get_parameter("yolo.device").value)
        self.conf_threshold = float(self.get_parameter("yolo.conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("yolo.iou_threshold").value)
        self.use_world = bool(self.get_parameter("yolo.use_world").value)
        self.custom_classes = list(self.get_parameter("yolo.custom_classes").value)
        self.frame_id_color = str(self.get_parameter("ros.frame_id_color").value)
        self.frame_id_depth = str(self.get_parameter("ros.frame_id_depth").value)
        self.publish_annotated = bool(self.get_parameter("ros.publish_annotated").value)
        self.show_preview = bool(self.get_parameter("ros.show_preview").value)
        self.preview_window_name = str(self.get_parameter("ros.preview_window_name").value)
        self.loop_rate_hz = float(self.get_parameter("ros.loop_rate_hz").value)
        self.enable_detection = bool(self.get_parameter("ros.enable_detection").value)
        self.enable_network_detection = bool(self.get_parameter("ros.enable_network_detection").value)

        if self.enable_network_detection and not self.network_detections_url:
            raise RuntimeError("camera.network_detections_url must not be empty when network detection is enabled")

        if self.enable_detection and not self.enable_network_detection:
            if not self.model_path:
                raise RuntimeError("yolo.model_path must not be empty")
            if not Path(self.model_path).exists():
                raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

    def _on_timer(self) -> None:
        color_bgr, depth_mm = self.camera.get_frame()
        if color_bgr is None and depth_mm is None:
            self.empty_frame_count += 1
            if self.empty_frame_count % 10 == 0:
                self.get_logger().warn(
                    f"empty frames encountered: {self.empty_frame_count}, "
                    f"driver_state={self.camera.last_debug_message}"
                )
            return

        depth_expected = self.enable_depth
        if color_bgr is None or (depth_expected and depth_mm is None):
            self.empty_frame_count += 1
            if self.empty_frame_count % 10 == 0:
                self.get_logger().warn(
                    f"partial frames encountered: {self.empty_frame_count}, "
                    f"driver_state={self.camera.last_debug_message}"
                )
        else:
            self.empty_frame_count = 0
        stamp = self.get_clock().now().to_msg()
        preview_image = color_bgr

        if color_bgr is not None:
            self.color_pub.publish(color_to_msg(color_bgr, stamp, self.frame_id_color))
        if depth_mm is not None:
            self.depth_pub.publish(depth_to_msg(depth_mm, stamp, self.frame_id_depth))
            camera_info = self._camera_info_payload(depth_mm)
            if camera_info is not None:
                self.depth_camera_info_pub.publish(camera_info_to_msg(camera_info, stamp, self.frame_id_depth))

        if self.network_detection_client is not None and color_bgr is not None:
            payload = self.network_detection_client.fetch()
            detection_msg = detection_json_to_msg(payload, stamp, self.frame_id_color)
            self.detection_pub.publish(detection_msg)
            if self.annotated_pub is not None:
                annotated = draw_detections(color_bgr, detection_msg)
                self.annotated_pub.publish(color_to_msg(annotated, stamp, self.frame_id_color))
                preview_image = annotated
        elif self.detector is not None and color_bgr is not None:
            results = self.detector.infer(color_bgr)
            detection_msg = result_to_detection_array_msg(results, stamp, self.frame_id_color)
            self.detection_pub.publish(detection_msg)
            self._detection_log_countdown += 1
            if self._detection_log_countdown >= 20:
                self._detection_log_countdown = 0
                sample_names = [det.class_name for det in detection_msg.detections[:5]]
                self.get_logger().info(
                    f"detection_count={len(detection_msg.detections)} sample={sample_names}"
                )

            if self.annotated_pub is not None:
                annotated = draw_detections(color_bgr, detection_msg)
                self.annotated_pub.publish(color_to_msg(annotated, stamp, self.frame_id_color))
                preview_image = annotated
            elif self.show_preview:
                preview_image = draw_detections(color_bgr, detection_msg)

        if self.show_preview and self._preview_window_created and preview_image is not None:
            cv2.imshow(self.preview_window_name, preview_image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                self.show_preview = False
                cv2.destroyWindow(self.preview_window_name)
                self._preview_window_created = False
                self.get_logger().info("preview window closed by user")
            elif cv2.getWindowProperty(self.preview_window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.show_preview = False
                cv2.destroyWindow(self.preview_window_name)
                self._preview_window_created = False
                self.get_logger().info("preview window closed")

    def _camera_info_payload(self, depth_mm):
        getter = getattr(self.camera, "get_camera_info", None)
        if callable(getter):
            camera_info = getter()
            if isinstance(camera_info, dict):
                return camera_info
        height, width = depth_mm.shape[:2]
        return {
            "width": int(width),
            "height": int(height),
            "fx": float(max(width, 1)),
            "fy": float(max(height, 1)),
            "cx": float(width) * 0.5,
            "cy": float(height) * 0.5,
        }

    def destroy_node(self):
        try:
            self.camera.close()
            if self._preview_window_created:
                cv2.destroyWindow(self.preview_window_name)
                self._preview_window_created = False
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RebotArmVisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

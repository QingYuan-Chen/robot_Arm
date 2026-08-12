"""Run YOLO on a MuJoCo virtual RGB stream without touching hardware."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rebotarm_msgs.msg import Detection2DArray

from .converters.detection_msgs import result_to_detection_array_msg
from .converters.image_msgs import color_to_msg
from .offline_yolo import decode_image_to_bgr, filter_detection_array, parse_target_classes


class OfflineYoloNode(Node):
    """Software-only YOLO adapter with an explicit target-class gate.

    Raw model output is published separately for diagnosis.  Only detections
    in ``target_classes`` reach the filtered output; the canonical MuJoCo scene
    uses the ``bottle`` target, which is present in the stock COCO label set.
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_offline_yolo_node")
        self._declare_parameters()
        self._load_parameters()

        # Import lazily so pure image/filter tests do not require Ultralytics.
        from .detector.yolo_detector import YoloDetector

        self._detector = YoloDetector(
            model_path=self.model_path,
            device=self.device,
            conf_threshold=self.conf_threshold,
            iou_threshold=self.iou_threshold,
            use_world=self.use_world,
            custom_classes=self.target_classes,
        )
        self._raw_pub = self.create_publisher(
            Detection2DArray, self.raw_detection_topic, qos_profile_sensor_data
        )
        self._filtered_pub = self.create_publisher(
            Detection2DArray, self.detection_topic, qos_profile_sensor_data
        )
        self._annotated_pub = None
        if self.publish_annotated:
            self._annotated_pub = self.create_publisher(
                Image, self.annotated_topic, qos_profile_sensor_data
            )
        self._subscription = self.create_subscription(
            Image, self.input_topic, self._on_image, qos_profile_sensor_data
        )
        self._failure_latched = False
        self._frame_count = 0
        self.get_logger().info(
            "offline YOLO initialized: "
            f"input={self.input_topic}, model={self.model_path}, device={self.device}, "
            f"target_classes={list(self.target_classes) or '<none; reject all>'}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("offline_yolo.input_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "offline_yolo.raw_detection_topic", "/grasp/offline_yolo/raw_detections"
        )
        self.declare_parameter(
            "offline_yolo.detection_topic", "/grasp/offline_yolo/detections"
        )
        self.declare_parameter(
            "offline_yolo.annotated_topic", "/camera/color/offline_yolo_annotated"
        )
        self.declare_parameter("offline_yolo.model_path", "")
        self.declare_parameter("offline_yolo.device", "cpu")
        self.declare_parameter("offline_yolo.conf_threshold", 0.05)
        self.declare_parameter("offline_yolo.iou_threshold", 0.45)
        self.declare_parameter("offline_yolo.use_world", False)
        # Keep this as a string because launch arguments commonly arrive as a
        # YAML-looking string; parse_target_classes handles both strings and
        # native string arrays for direct node use.
        self.declare_parameter("offline_yolo.target_classes", "['cube']")
        self.declare_parameter("offline_yolo.publish_annotated", True)

    def _load_parameters(self) -> None:
        self.input_topic = str(self.get_parameter("offline_yolo.input_topic").value)
        self.raw_detection_topic = str(
            self.get_parameter("offline_yolo.raw_detection_topic").value
        )
        self.detection_topic = str(
            self.get_parameter("offline_yolo.detection_topic").value
        )
        self.annotated_topic = str(
            self.get_parameter("offline_yolo.annotated_topic").value
        )
        self.model_path = str(self.get_parameter("offline_yolo.model_path").value).strip()
        self.device = str(self.get_parameter("offline_yolo.device").value).strip()
        self.conf_threshold = float(
            self.get_parameter("offline_yolo.conf_threshold").value
        )
        self.iou_threshold = float(
            self.get_parameter("offline_yolo.iou_threshold").value
        )
        self.use_world = bool(self.get_parameter("offline_yolo.use_world").value)
        self.target_classes = parse_target_classes(
            self.get_parameter("offline_yolo.target_classes").value
        )
        self.publish_annotated = bool(
            self.get_parameter("offline_yolo.publish_annotated").value
        )
        if not self.input_topic.strip() or not self.detection_topic.strip():
            raise ValueError("offline YOLO input and detection topics must be non-empty")
        if not self.model_path:
            raise RuntimeError("offline_yolo.model_path must not be empty")
        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"offline YOLO model not found: {self.model_path}")
        if not self.device:
            raise ValueError("offline_yolo.device must not be empty")
        if not 0.0 <= self.conf_threshold <= 1.0:
            raise ValueError("offline_yolo.conf_threshold must be in [0, 1]")
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError("offline_yolo.iou_threshold must be in [0, 1]")

    def _on_image(self, image_msg: Image) -> None:
        stamp = image_msg.header.stamp
        frame_id = str(getattr(image_msg.header, "frame_id", ""))
        try:
            image_bgr = decode_image_to_bgr(image_msg)
            results = self._detector.infer(image_bgr)
            raw = result_to_detection_array_msg(results, stamp, frame_id)
            filtered = filter_detection_array(raw, self.target_classes)
        except Exception as exc:
            self._publish_empty(stamp, frame_id)
            if not self._failure_latched:
                self._failure_latched = True
                self.get_logger().error(
                    f"offline YOLO fail-closed: {type(exc).__name__}: {exc}"
                )
            return

        if self._failure_latched:
            self._failure_latched = False
            self.get_logger().info("offline YOLO stream recovered; fail-closed latch cleared")
        self._raw_pub.publish(raw)
        self._filtered_pub.publish(filtered)
        self._frame_count += 1
        if self._frame_count % 20 == 0:
            self.get_logger().info(
                f"offline YOLO frame={self._frame_count} raw={len(raw.detections)} "
                f"target={len(filtered.detections)}"
            )
        if self._annotated_pub is not None:
            try:
                from .utils.visualization import draw_detections

                annotated = draw_detections(image_bgr, raw)
                self._annotated_pub.publish(color_to_msg(annotated, stamp, frame_id))
            except Exception as exc:
                if not self._failure_latched:
                    self.get_logger().warning(
                        f"offline YOLO annotation disabled for this frame: {type(exc).__name__}: {exc}"
                    )

    def _publish_empty(self, stamp, frame_id: str) -> None:
        raw = Detection2DArray()
        raw.header.stamp = stamp
        raw.header.frame_id = frame_id
        filtered = Detection2DArray()
        filtered.header.stamp = stamp
        filtered.header.frame_id = frame_id
        self._raw_pub.publish(raw)
        self._filtered_pub.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfflineYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

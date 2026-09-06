from __future__ import annotations

from collections import deque
import json
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image

from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray

from .graspnet_baseline_adapter import (
    InProcessGraspNetBackend,
    closest_timestamped_frame,
    payload_to_candidate_array,
)
from .network_graspnet_client import NetworkGraspNetClient, NetworkGraspNetConfig
from .ordinary_grasp_node import depth_image_to_array


def color_image_to_array(msg: Image) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    channels = 3
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
    if msg.encoding == "rgb8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


class GraspNetBaselineNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_graspnet_baseline_node")
        self.declare_parameter("input_color_topic", "/camera/color/image_raw")
        self.declare_parameter("input_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("input_detections_topic", "/grasp/detections")
        self.declare_parameter("output_candidates_topic", "/grasp/graspnet_candidates")
        self.declare_parameter("output_frame_id", "camera_depth_frame")
        self.declare_parameter("source_mode", "network")
        self.declare_parameter("network_candidates_url", "http://127.0.0.1:8081/graspnet_candidates.json")
        self.declare_parameter("network_timeout_ms", 1000)
        self.declare_parameter("network_poll_hz", 5.0)
        self.declare_parameter("max_input_skew_ms", 100)
        self.declare_parameter("depth_scale_m_per_unit", 0.001)
        self.declare_parameter("model_root", os.environ.get("GRASPNET_MODEL_ROOT", ""))
        self.declare_parameter(
            "checkpoint_path", os.environ.get("GRASPNET_CHECKPOINT_PATH", "")
        )
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("backend_module", "graspnet_baseline_inference")
        self.declare_parameter(
            "backend_module_path", os.environ.get("GRASPNET_BACKEND_MODULE_PATH", "")
        )
        self.declare_parameter("max_grasps", 20)
        self.declare_parameter("max_jaw_width_m", 0.085)
        self.declare_parameter("max_points", 20000)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.20)
        self.declare_parameter("fx", 692.562744140625)
        self.declare_parameter("fy", 692.2272338867188)
        self.declare_parameter("cx", 641.2417602539062)
        self.declare_parameter("cy", 361.8166198730469)

        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.source_mode = str(self.get_parameter("source_mode").value).strip()
        if self.source_mode not in {"network", "in_process"}:
            raise ValueError(f"unsupported source_mode: {self.source_mode}")
        self.latest_color_bgr: np.ndarray | None = None
        self.latest_depth_mm: np.ndarray | None = None
        self.latest_color_timestamp_ns = 0
        self.latest_depth_timestamp_ns = 0
        self.latest_depth_frame_id = self.output_frame_id
        self.color_frame_cache = deque(maxlen=8)
        self.depth_frame_cache = deque(maxlen=8)
        self.latest_camera_info: dict[str, float] | None = None
        self.last_inprocess_detection_timestamp_ns = 0
        self.pending_inprocess_detections: Detection2DArray | None = None
        self._warned_backend = False
        self.backend = (
            self._create_inprocess_backend() if self.source_mode == "in_process" else None
        )
        self.network_client = self._create_network_client() if self.source_mode == "network" else None

        self.candidates_pub = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter("output_candidates_topic").value),
            10,
        )
        if self.source_mode == "network":
            period = 1.0 / max(float(self.get_parameter("network_poll_hz").value), 0.1)
            self.create_timer(period, self._on_network_timer)
        else:
            self.create_subscription(
                Image,
                str(self.get_parameter("input_color_topic").value),
                self._on_color,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("input_depth_topic").value),
                self._on_depth,
                qos_profile_sensor_data,
            )
            if self.source_mode == "in_process":
                self.create_subscription(
                    CameraInfo,
                    str(self.get_parameter("input_camera_info_topic").value),
                    self._on_camera_info,
                    qos_profile_sensor_data,
                )
            detection_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(
                Detection2DArray,
                str(self.get_parameter("input_detections_topic").value),
                self._on_detections,
                detection_qos,
            )
        self.get_logger().info(
            "GraspNet baseline candidate node ready: "
            f"output={str(self.get_parameter('output_candidates_topic').value)}, "
            f"source_mode={self.source_mode}, "
            f"backend_available={self.backend.available if self.backend is not None else 'n/a'}"
        )

    def _create_network_client(self) -> NetworkGraspNetClient:
        return NetworkGraspNetClient(
            NetworkGraspNetConfig(
                candidates_url=str(self.get_parameter("network_candidates_url").value),
                timeout_ms=int(self.get_parameter("network_timeout_ms").value),
            )
        )

    def _create_inprocess_backend(self) -> InProcessGraspNetBackend:
        backend = InProcessGraspNetBackend(
            model_root=str(self.get_parameter("model_root").value),
            checkpoint_path=str(self.get_parameter("checkpoint_path").value),
            device=str(self.get_parameter("device").value),
            module_name=str(self.get_parameter("backend_module").value),
            module_path=str(self.get_parameter("backend_module_path").value),
            num_point=max(1, int(self.get_parameter("max_points").value)),
        )
        if not backend.available:
            self.get_logger().error(
                "GraspNet in-process backend unavailable; candidates will fail closed: "
                f"{backend.backend_error}"
            )
        return backend

    def _on_color(self, msg: Image) -> None:
        try:
            self.latest_color_bgr = color_image_to_array(msg)
            self.latest_color_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
            self.color_frame_cache.append(
                (self.latest_color_timestamp_ns, self.latest_color_bgr)
            )
            if self.source_mode == "in_process":
                self._try_process_inprocess_detection()
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_depth(self, msg: Image) -> None:
        try:
            self.latest_depth_mm = depth_image_to_array(msg)
            self.latest_depth_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
            self.latest_depth_frame_id = str(msg.header.frame_id or self.output_frame_id)
            self.depth_frame_cache.append(
                (
                    self.latest_depth_timestamp_ns,
                    self.latest_depth_mm,
                    self.latest_depth_frame_id,
                )
            )
            if self.source_mode == "in_process":
                self._try_process_inprocess_detection()
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "cx": float(msg.k[2]),
            "cy": float(msg.k[5]),
        }
        self._try_process_inprocess_detection()

    def _on_network_timer(self) -> None:
        if self.network_client is None:
            return
        payload = self.network_client.fetch()
        candidates = payload_to_candidate_array(
            payload,
            fallback_frame_id=self.output_frame_id,
            max_candidates=int(self.get_parameter("max_grasps").value),
        )
        if not bool(payload.get("backend_configured", False)) and not self._warned_backend:
            self.get_logger().warn(
                "Windows GraspNet baseline backend is not configured; "
                f"network_status={self.network_client.last_debug_message}"
            )
            self._warned_backend = True
        self.candidates_pub.publish(candidates)

    def _on_detections(self, msg: Detection2DArray) -> None:
        if self.source_mode == "in_process":
            self._on_inprocess_detections(msg)
            return

    def _on_inprocess_detections(self, msg: Detection2DArray) -> None:
        self.pending_inprocess_detections = msg
        self._try_process_inprocess_detection()

    def _try_process_inprocess_detection(self) -> None:
        msg = self.pending_inprocess_detections
        if msg is None:
            return
        detection_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
        if detection_timestamp_ns <= self.last_inprocess_detection_timestamp_ns:
            self.pending_inprocess_detections = None
            return
        if not msg.detections:
            self.pending_inprocess_detections = None
            self.last_inprocess_detection_timestamp_ns = detection_timestamp_ns
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return
        if self.backend is None or self.latest_camera_info is None:
            return
        color_frame = closest_timestamped_frame(
            self.color_frame_cache, detection_timestamp_ns
        )
        depth_frame = closest_timestamped_frame(
            self.depth_frame_cache, detection_timestamp_ns
        )
        if color_frame is None or depth_frame is None:
            return
        color_timestamp_ns, color_bgr = color_frame
        depth_timestamp_ns, depth_mm, depth_frame_id = depth_frame
        max_skew_ns = int(self.get_parameter("max_input_skew_ms").value) * 1_000_000
        color_skew_ns = abs(color_timestamp_ns - detection_timestamp_ns)
        depth_skew_ns = abs(depth_timestamp_ns - detection_timestamp_ns)
        rgbd_skew_ns = abs(color_timestamp_ns - depth_timestamp_ns)
        if max(color_skew_ns, depth_skew_ns, rgbd_skew_ns) > max_skew_ns:
            # The matching image callback may not have been scheduled yet.
            # Keep only this newest detection pending and retry when RGB/depth arrives.
            return
        self.pending_inprocess_detections = None
        self.last_inprocess_detection_timestamp_ns = detection_timestamp_ns
        detection = max(msg.detections, key=lambda item: float(item.confidence))
        if not self.backend.available:
            self._publish_empty(timestamp_ns=depth_timestamp_ns, frame_id=depth_frame_id)
            return
        detection_payload = {
            "x_min": int(detection.x_min),
            "y_min": int(detection.y_min),
            "x_max": int(detection.x_max),
            "y_max": int(detection.y_max),
            "confidence": float(detection.confidence),
            "class_name": str(detection.class_name),
        }
        if bool(getattr(detection, "has_mask", False)) and len(
            getattr(detection, "mask_polygon_xy", [])
        ) >= 6:
            detection_payload["mask_polygon_xy"] = [
                float(value) for value in detection.mask_polygon_xy
            ]
        try:
            payload = self.backend.infer(
                timestamp_ns=depth_timestamp_ns,
                frame_id=depth_frame_id,
                color_bgr=color_bgr,
                depth_m=depth_mm.astype(np.float32)
                * float(self.get_parameter("depth_scale_m_per_unit").value),
                camera_info=self.latest_camera_info,
                detection=detection_payload,
                max_grasps=int(self.get_parameter("max_grasps").value),
                max_jaw_width_m=float(self.get_parameter("max_jaw_width_m").value),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"GraspNet in-process inference failed: {type(exc).__name__}: {exc}"
            )
            self._publish_empty(timestamp_ns=depth_timestamp_ns, frame_id=depth_frame_id)
            return
        self._log_stage_counts(
            timestamp_ns=depth_timestamp_ns,
            frame_id=depth_frame_id,
            class_name=str(detection.class_name),
            candidates=payload.get("candidates", []),
        )
        candidates = payload_to_candidate_array(
            payload,
            fallback_frame_id=depth_frame_id,
            max_candidates=int(self.get_parameter("max_grasps").value),
        )
        self.candidates_pub.publish(candidates)

    def _log_stage_counts(
        self,
        *,
        timestamp_ns: int,
        frame_id: str,
        class_name: str,
        candidates,
    ) -> None:
        if self.backend is None:
            return
        stage_counts = self.backend.last_stage_counts
        if not stage_counts:
            return
        top_score = None
        if candidates:
            top_score = float(candidates[0].get("score", 0.0))
        record = {
            "event": "graspnet_stage_counts",
            "transport": "in_process",
            "timestamp_ns": int(timestamp_ns),
            "frame_id": str(frame_id),
            "class_name": str(class_name),
            "max_grasps": int(self.get_parameter("max_grasps").value),
            "max_jaw_width_m": float(self.get_parameter("max_jaw_width_m").value),
            **stage_counts,
            "published_top_score": top_score,
        }
        self.get_logger().info(json.dumps(record, separators=(",", ":")))

    def _publish_empty(
        self,
        *,
        timestamp_ns: int = 0,
        frame_id: str | None = None,
    ) -> None:
        msg = GraspCandidateArray()
        msg.header.frame_id = str(frame_id or self.output_frame_id)
        if timestamp_ns > 0:
            msg.header.stamp.sec = timestamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        msg.best_index = -1
        self.candidates_pub.publish(msg)

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspNetBaselineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()

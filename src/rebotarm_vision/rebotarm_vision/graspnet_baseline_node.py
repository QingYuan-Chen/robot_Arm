from __future__ import annotations

from collections import deque

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
    CameraIntrinsics,
    GraspNetBaselineBackend,
    build_point_cloud_for_detection,
    payload_to_candidate_array,
    predictions_to_candidate_array,
)
from .network_graspnet_client import NetworkGraspNetClient, NetworkGraspNetConfig
from .local_graspnet_client import (
    LocalGraspNetClient,
    LocalGraspNetConfig,
    closest_timestamped_frame,
)
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
        self.declare_parameter("local_infer_url", "http://127.0.0.1:8081/infer")
        self.declare_parameter("local_timeout_ms", 2000)
        self.declare_parameter("max_input_skew_ms", 100)
        self.declare_parameter("depth_scale_m_per_unit", 0.001)
        self.declare_parameter("model_root", "")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("backend_module", "graspnet_baseline_inference")
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
        if self.source_mode not in {"network", "local_backend", "local_service"}:
            raise ValueError(f"unsupported source_mode: {self.source_mode}")
        self.intrinsics = CameraIntrinsics(
            fx=float(self.get_parameter("fx").value),
            fy=float(self.get_parameter("fy").value),
            cx=float(self.get_parameter("cx").value),
            cy=float(self.get_parameter("cy").value),
        )
        self.latest_color_bgr: np.ndarray | None = None
        self.latest_depth_mm: np.ndarray | None = None
        self.latest_color_timestamp_ns = 0
        self.latest_depth_timestamp_ns = 0
        self.latest_depth_frame_id = self.output_frame_id
        self.color_frame_cache = deque(maxlen=8)
        self.depth_frame_cache = deque(maxlen=8)
        self.latest_camera_info: dict[str, float] | None = None
        self.last_local_detection_timestamp_ns = 0
        self.pending_local_detections: Detection2DArray | None = None
        self._warned_backend = False
        self.backend = self._create_backend() if self.source_mode == "local_backend" else None
        self.network_client = self._create_network_client() if self.source_mode == "network" else None
        self.local_client = self._create_local_client() if self.source_mode == "local_service" else None

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
            if self.source_mode == "local_service":
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
            f"source_mode={self.source_mode}"
        )

    def _create_network_client(self) -> NetworkGraspNetClient:
        return NetworkGraspNetClient(
            NetworkGraspNetConfig(
                candidates_url=str(self.get_parameter("network_candidates_url").value),
                timeout_ms=int(self.get_parameter("network_timeout_ms").value),
            )
        )

    def _create_backend(self) -> GraspNetBaselineBackend:
        try:
            return GraspNetBaselineBackend(
                model_root=str(self.get_parameter("model_root").value),
                checkpoint_path=str(self.get_parameter("checkpoint_path").value),
                device=str(self.get_parameter("device").value),
                module_name=str(self.get_parameter("backend_module").value),
            )
        except Exception as exc:
            self.get_logger().warn(f"GraspNet baseline backend unavailable: {type(exc).__name__}: {exc}")
            return GraspNetBaselineBackend(model_root="")

    def _create_local_client(self) -> LocalGraspNetClient:
        return LocalGraspNetClient(
            LocalGraspNetConfig(
                infer_url=str(self.get_parameter("local_infer_url").value),
                timeout_ms=int(self.get_parameter("local_timeout_ms").value),
            )
        )

    def _on_color(self, msg: Image) -> None:
        try:
            self.latest_color_bgr = color_image_to_array(msg)
            self.latest_color_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
            self.color_frame_cache.append(
                (self.latest_color_timestamp_ns, self.latest_color_bgr)
            )
            if self.source_mode == "local_service":
                self._try_process_local_service_detection()
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
            if self.source_mode == "local_service":
                self._try_process_local_service_detection()
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "cx": float(msg.k[2]),
            "cy": float(msg.k[5]),
        }
        self._try_process_local_service_detection()

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
        detection_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
        if self.source_mode == "local_service":
            self._on_local_service_detections(msg)
            return
        if self.latest_depth_mm is None or self.latest_color_bgr is None:
            return
        if not msg.detections:
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return
        if self.backend is None or not self.backend.available:
            if not self._warned_backend:
                self.get_logger().warn(
                    "GraspNet baseline backend is not configured; "
                    "install/wrap graspnet-baseline before using V1.3 candidates"
                )
                self._warned_backend = True
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return

        detection = max(msg.detections, key=lambda item: float(item.confidence))
        crop = build_point_cloud_for_detection(
            self.latest_depth_mm,
            self.latest_color_bgr,
            detection,
            self.intrinsics,
            min_depth_m=float(self.get_parameter("min_depth_m").value),
            max_depth_m=float(self.get_parameter("max_depth_m").value),
        )
        if crop.points.size == 0:
            self.get_logger().warn("GraspNet baseline skipped frame: no valid depth in selected detection ROI")
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return
        points, colors = self._downsample(crop.points, crop.colors)
        try:
            predictions = self.backend.infer(
                points=points,
                colors=colors,
                max_grasps=int(self.get_parameter("max_grasps").value),
            )
        except Exception as exc:
            self.get_logger().warn(f"GraspNet baseline inference failed: {type(exc).__name__}: {exc}")
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return
        candidates = predictions_to_candidate_array(
            predictions,
            frame_id=self.output_frame_id,
            class_name=str(detection.class_name),
            max_candidates=int(self.get_parameter("max_grasps").value),
        )
        self.candidates_pub.publish(candidates)

    def _on_local_service_detections(self, msg: Detection2DArray) -> None:
        self.pending_local_detections = msg
        self._try_process_local_service_detection()

    def _try_process_local_service_detection(self) -> None:
        msg = self.pending_local_detections
        if msg is None:
            return
        detection_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
        if detection_timestamp_ns <= self.last_local_detection_timestamp_ns:
            self.pending_local_detections = None
            return
        if not msg.detections:
            self.pending_local_detections = None
            self.last_local_detection_timestamp_ns = detection_timestamp_ns
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return
        if self.local_client is None or self.latest_camera_info is None:
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
        self.pending_local_detections = None
        self.last_local_detection_timestamp_ns = detection_timestamp_ns
        detection = max(msg.detections, key=lambda item: float(item.confidence))
        payload = self.local_client.infer(
            timestamp_ns=depth_timestamp_ns,
            frame_id=depth_frame_id,
            color_bgr=color_bgr,
            depth_m=depth_mm.astype(np.float32)
            * float(self.get_parameter("depth_scale_m_per_unit").value),
            intrinsics=self.latest_camera_info,
            bbox={
                "x_min": int(detection.x_min),
                "y_min": int(detection.y_min),
                "x_max": int(detection.x_max),
                "y_max": int(detection.y_max),
                "confidence": float(detection.confidence),
                "class_name": str(detection.class_name),
            },
            max_grasps=int(self.get_parameter("max_grasps").value),
            max_jaw_width_m=float(self.get_parameter("max_jaw_width_m").value),
            mask_polygon_xy=(
                [float(value) for value in detection.mask_polygon_xy]
                if bool(getattr(detection, "has_mask", False))
                and len(getattr(detection, "mask_polygon_xy", [])) >= 6
                else None
            ),
        )
        candidates = payload_to_candidate_array(
            payload,
            fallback_frame_id=depth_frame_id,
            max_candidates=int(self.get_parameter("max_grasps").value),
        )
        self.candidates_pub.publish(candidates)

    def _downsample(self, points: np.ndarray, colors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        max_points = max(1, int(self.get_parameter("max_points").value))
        if len(points) <= max_points:
            return points, colors
        step = max(1, int(len(points) / max_points))
        return points[::step][:max_points], colors[::step][:max_points]

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

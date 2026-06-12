from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray, GraspPlan

from .converters.ordinary_grasp_adapter import CameraIntrinsics, plan_and_candidates_from_detections_and_depth
from .depth_quality import DepthQualityConfig


def depth_image_to_array(msg: Image) -> np.ndarray:
    if msg.encoding not in ("mono16", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    depth = np.frombuffer(msg.data, dtype=np.uint16)
    return depth.reshape((msg.height, msg.width))


class OrdinaryGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_ordinary_grasp_node")
        self.declare_parameter("ordinary_grasp.input_detections_topic", "/grasp/detections")
        self.declare_parameter("ordinary_grasp.input_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("ordinary_grasp.output_topic", "/grasp/plan")
        self.declare_parameter("ordinary_grasp.candidates_topic", "/grasp/candidates")
        self.declare_parameter("ordinary_grasp.pregrasp_pose_topic", "/grasp/pregrasp_pose")
        self.declare_parameter("ordinary_grasp.grasp_pose_topic", "/grasp/grasp_pose")
        self.declare_parameter("ordinary_grasp.root", "/home/u24/robotarm_ros2/../rebot_grasp")
        self.declare_parameter("ordinary_grasp.output_frame_id", "camera_depth_frame")
        self.declare_parameter("ordinary_grasp.depth_quantile", 0.75)
        self.declare_parameter("ordinary_grasp.pregrasp_offset_m", 0.08)
        self.declare_parameter("ordinary_grasp.fx", 500.0)
        self.declare_parameter("ordinary_grasp.fy", 500.0)
        self.declare_parameter("ordinary_grasp.cx", 640.0)
        self.declare_parameter("ordinary_grasp.cy", 360.0)
        self.declare_parameter("depth_quality.enabled", True)
        self.declare_parameter("depth_quality.min_valid_pixels", 80)
        self.declare_parameter("depth_quality.min_valid_ratio", 0.20)
        self.declare_parameter("depth_quality.min_depth_m", 0.15)
        self.declare_parameter("depth_quality.max_depth_m", 1.20)
        self.declare_parameter("depth_quality.max_depth_mad_m", 0.025)
        self.declare_parameter("depth_quality.max_depth_span_m", 0.080)
        self.declare_parameter("depth_quality.center_window_px", 9)
        self.declare_parameter("depth_quality.min_center_valid_ratio", 0.30)
        self.declare_parameter("depth_quality.override_enabled", False)
        self.declare_parameter("depth_quality.override_value", True)

        self.input_detections_topic = str(
            self.get_parameter("ordinary_grasp.input_detections_topic").value
        )
        self.input_depth_topic = str(self.get_parameter("ordinary_grasp.input_depth_topic").value)
        self.output_topic = str(self.get_parameter("ordinary_grasp.output_topic").value)
        self.candidates_topic = str(self.get_parameter("ordinary_grasp.candidates_topic").value)
        self.pregrasp_pose_topic = str(
            self.get_parameter("ordinary_grasp.pregrasp_pose_topic").value
        )
        self.grasp_pose_topic = str(self.get_parameter("ordinary_grasp.grasp_pose_topic").value)
        self.ordinary_grasp_root = Path(str(self.get_parameter("ordinary_grasp.root").value))
        self.output_frame_id = str(self.get_parameter("ordinary_grasp.output_frame_id").value)
        self.depth_quantile = float(self.get_parameter("ordinary_grasp.depth_quantile").value)
        self.pregrasp_offset_m = float(self.get_parameter("ordinary_grasp.pregrasp_offset_m").value)
        self.intrinsics = CameraIntrinsics(
            fx=float(self.get_parameter("ordinary_grasp.fx").value),
            fy=float(self.get_parameter("ordinary_grasp.fy").value),
            cx=float(self.get_parameter("ordinary_grasp.cx").value),
            cy=float(self.get_parameter("ordinary_grasp.cy").value),
        )
        depth_quality_enabled = bool(self.get_parameter("depth_quality.enabled").value)
        if bool(self.get_parameter("depth_quality.override_enabled").value):
            depth_quality_enabled = bool(self.get_parameter("depth_quality.override_value").value)
            self.set_parameters(
                [Parameter("depth_quality.enabled", Parameter.Type.BOOL, depth_quality_enabled)]
            )
        self.depth_quality_config = DepthQualityConfig(
            enabled=depth_quality_enabled,
            min_valid_pixels=int(self.get_parameter("depth_quality.min_valid_pixels").value),
            min_valid_ratio=float(self.get_parameter("depth_quality.min_valid_ratio").value),
            min_depth_m=float(self.get_parameter("depth_quality.min_depth_m").value),
            max_depth_m=float(self.get_parameter("depth_quality.max_depth_m").value),
            max_depth_mad_m=float(self.get_parameter("depth_quality.max_depth_mad_m").value),
            max_depth_span_m=float(self.get_parameter("depth_quality.max_depth_span_m").value),
            center_window_px=int(self.get_parameter("depth_quality.center_window_px").value),
            min_center_valid_ratio=float(self.get_parameter("depth_quality.min_center_valid_ratio").value),
        )
        self.latest_depth_mm = None

        self.plan_pub = self.create_publisher(GraspPlan, self.output_topic, 10)
        self.candidates_pub = self.create_publisher(GraspCandidateArray, self.candidates_topic, 10)
        self.pregrasp_pose_pub = self.create_publisher(PoseStamped, self.pregrasp_pose_topic, 10)
        self.grasp_pose_pub = self.create_publisher(PoseStamped, self.grasp_pose_topic, 10)
        self.depth_subscription = self.create_subscription(
            Image,
            self.input_depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.detections_subscription = self.create_subscription(
            Detection2DArray,
            self.input_detections_topic,
            self._on_detections,
            10,
        )
        self.get_logger().info(
            "ordinary grasp node ready: "
            f"detections={self.input_detections_topic}, depth={self.input_depth_topic}, "
            f"output={self.output_topic}, candidates={self.candidates_topic}, "
            f"depth_quality_enabled={self.depth_quality_config.enabled}, root={self.ordinary_grasp_root}"
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            self.latest_depth_mm = depth_image_to_array(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_detections(self, msg: Detection2DArray) -> None:
        if self.latest_depth_mm is None:
            return
        try:
            plan, candidates = plan_and_candidates_from_detections_and_depth(
                msg,
                self.latest_depth_mm,
                self.intrinsics,
                ordinary_grasp_root=self.ordinary_grasp_root,
                output_frame_id=self.output_frame_id,
                depth_quantile=self.depth_quantile,
                pregrasp_offset_m=self.pregrasp_offset_m,
                depth_quality_config=self.depth_quality_config,
            )
        except Exception as exc:
            self.get_logger().warn(f"ordinary grasp failed: {type(exc).__name__}: {exc}")
            return
        self.candidates_pub.publish(candidates)
        self.plan_pub.publish(plan)
        if plan.valid:
            self.pregrasp_pose_pub.publish(self._pose_stamped(plan, plan.pregrasp_pose))
            self.grasp_pose_pub.publish(self._pose_stamped(plan, plan.grasp_pose))

    @staticmethod
    def _pose_stamped(plan: GraspPlan, pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header = plan.header
        msg.pose = pose
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrdinaryGraspNode()
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

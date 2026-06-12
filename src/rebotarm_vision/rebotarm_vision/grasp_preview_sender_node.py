from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspPlan

from .transform_points import (
    Transform3D,
    quaternion_to_rotation_matrix,
    transform_pose_components,
)


def _transform_from_msg(tf_msg) -> Transform3D:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return Transform3D(
        translation=(float(t.x), float(t.y), float(t.z)),
        rotation_xyzw=(float(q.x), float(q.y), float(q.z), float(q.w)),
    )


def select_grasp_plan_pose(plan: GraspPlan, pose_mode: str) -> Pose:
    if not plan.valid:
        reason = plan.reason or "grasp plan is invalid"
        raise ValueError(reason)

    normalized = pose_mode.strip().lower()
    if normalized == "pregrasp":
        return deepcopy(plan.pregrasp_pose)
    if normalized == "grasp":
        return deepcopy(plan.grasp_pose)
    raise ValueError(f"unsupported pose mode: {pose_mode}")


def transform_pose_message(pose: Pose, transform: Transform3D) -> Pose:
    transformed = deepcopy(pose)
    position, orientation = transform_pose_components(
        transform,
        (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ),
        (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )
    transformed.position.x = position[0]
    transformed.position.y = position[1]
    transformed.position.z = position[2]
    transformed.orientation.x = orientation[0]
    transformed.orientation.y = orientation[1]
    transformed.orientation.z = orientation[2]
    transformed.orientation.w = orientation[3]
    return transformed


def apply_tcp_offset_to_pose(
    grasp_tcp_pose: Pose,
    tcp_offset_xyz: tuple[float, float, float],
) -> Pose:
    target = deepcopy(grasp_tcp_pose)
    rotation = quaternion_to_rotation_matrix(
        (
            float(grasp_tcp_pose.orientation.x),
            float(grasp_tcp_pose.orientation.y),
            float(grasp_tcp_pose.orientation.z),
            float(grasp_tcp_pose.orientation.w),
        )
    )
    ox, oy, oz = tcp_offset_xyz
    dx = rotation[0][0] * ox + rotation[0][1] * oy + rotation[0][2] * oz
    dy = rotation[1][0] * ox + rotation[1][1] * oy + rotation[1][2] * oz
    dz = rotation[2][0] * ox + rotation[2][1] * oy + rotation[2][2] * oz
    target.position.x = round(float(grasp_tcp_pose.position.x) - dx, 6)
    target.position.y = round(float(grasp_tcp_pose.position.y) - dy, 6)
    target.position.z = round(float(grasp_tcp_pose.position.z) - dz, 6)
    return target


class GraspPreviewSenderNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_preview_sender")
        self.declare_parameter("input_topic", "/grasp/plan")
        self.declare_parameter("output_topic", "/rebotarm/interactive_control/pose_target")
        self.declare_parameter("pose_mode", "pregrasp")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("base_z_offset_m", 0.0)
        self.declare_parameter("min_target_z_m", 0.0)
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("exit_after_publish", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.pose_mode = str(self.get_parameter("pose_mode").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self.target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self.base_z_offset_m = float(self.get_parameter("base_z_offset_m").value)
        self.min_target_z_m = float(self.get_parameter("min_target_z_m").value)
        self.publish_count = max(1, int(self.get_parameter("publish_count").value))
        self.exit_after_publish = bool(self.get_parameter("exit_after_publish").value)
        self.published = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Pose, self.output_topic, 10)
        self.subscription = self.create_subscription(
            GraspPlan,
            self.input_topic,
            self._on_plan,
            10,
        )
        self.get_logger().info(
            "grasp preview sender ready: "
            f"input={self.input_topic}, output={self.output_topic}, "
            f"pose_mode={self.pose_mode}, target_frame={self.target_frame}, "
            f"tcp_offset_xyz={self.tcp_offset_xyz}, "
            f"target_base_offset_xyz={self.target_base_offset_xyz}, "
            f"base_z_offset_m={self.base_z_offset_m:.3f}, "
            f"min_target_z_m={self.min_target_z_m:.3f}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))

    def _on_plan(self, plan: GraspPlan) -> None:
        if self.published and self.exit_after_publish:
            return

        try:
            pose = select_grasp_plan_pose(plan, self.pose_mode)
        except ValueError as exc:
            self.get_logger().warn(f"skip grasp plan: {exc}")
            if self.exit_after_publish:
                self.published = True
            return

        source_frame = str(plan.header.frame_id)
        target_frame = self.target_frame.strip()
        if target_frame and source_frame and source_frame != target_frame:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                pose = transform_pose_message(pose, _transform_from_msg(tf_msg))
            except Exception as exc:
                self.get_logger().warn(
                    f"skip grasp plan: waiting for TF {target_frame} <- {source_frame}: {exc}"
                )
                return

        pose = apply_tcp_offset_to_pose(pose, self.tcp_offset_xyz)
        pose.position.x = round(float(pose.position.x) + self.target_base_offset_xyz[0], 6)
        pose.position.y = round(float(pose.position.y) + self.target_base_offset_xyz[1], 6)
        pose.position.z = round(float(pose.position.z) + self.target_base_offset_xyz[2], 6)
        pose.position.z = round(float(pose.position.z) + self.base_z_offset_m, 6)
        if self.min_target_z_m > 0.0:
            pose.position.z = max(float(pose.position.z), self.min_target_z_m)
        for _ in range(self.publish_count):
            self.publisher.publish(pose)
        self.published = True
        self.get_logger().info(
            f"sent {self.pose_mode} pose to {self.output_topic}: "
            f"position=({pose.position.x:+.3f},{pose.position.y:+.3f},{pose.position.z:+.3f}), "
            f"orientation=({pose.orientation.x:+.4f},{pose.orientation.y:+.4f},"
            f"{pose.orientation.z:+.4f},{pose.orientation.w:+.4f})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspPreviewSenderNode()
    try:
        while rclpy.ok() and not (node.exit_after_publish and node.published):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

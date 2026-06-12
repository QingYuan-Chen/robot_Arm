from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def build_grasp_tcp_transform(
    *,
    parent_frame: str,
    child_frame: str,
    tcp_offset_xyz: tuple[float, float, float],
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = float(tcp_offset_xyz[0])
    msg.transform.translation.y = float(tcp_offset_xyz[1])
    msg.transform.translation.z = float(tcp_offset_xyz[2])
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = 0.0
    msg.transform.rotation.w = 1.0
    return msg


class GraspTcpFrameNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_tcp_frame")
        self.declare_parameter("parent_frame", "end_link")
        self.declare_parameter("child_frame", "grasp_tcp")
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.tcp_offset_xyz = self._tuple3("tcp_offset_xyz")

        self.broadcaster = StaticTransformBroadcaster(self)
        self.transform = build_grasp_tcp_transform(
            parent_frame=self.parent_frame,
            child_frame=self.child_frame,
            tcp_offset_xyz=self.tcp_offset_xyz,
        )
        self.broadcaster.sendTransform(self.transform)
        self.get_logger().info(
            "grasp tcp frame ready: "
            f"{self.parent_frame} -> {self.child_frame}, offset={self.tcp_offset_xyz}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspTcpFrameNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rebotarm_msgs.msg import JointMotorState
from sensor_msgs.msg import JointState

from .gripper_visual_model import DEFAULT_GRIPPER_LIMITS_M, gripper_opening_to_finger_joint_positions


class GripperVisualJointStateNode(Node):
    """Publish a visualization-only JointState stream for RViz finger links."""

    def __init__(self) -> None:
        super().__init__("gripper_visual_joint_state_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("gripper_lower_limit_m", DEFAULT_GRIPPER_LIMITS_M[0])
        self.declare_parameter("gripper_upper_limit_m", DEFAULT_GRIPPER_LIMITS_M[1])
        self.declare_parameter("left_finger_joint_name", "left_finger_joint")
        self.declare_parameter("right_finger_joint_name", "right_finger_joint")
        self.declare_parameter(
            "required_arm_joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._namespace = namespace
        lower = float(self.get_parameter("gripper_lower_limit_m").value)
        upper = float(self.get_parameter("gripper_upper_limit_m").value)
        if upper < lower:
            lower, upper = upper, lower
        self._gripper_limits = (lower, upper)
        self._left_joint = str(self.get_parameter("left_finger_joint_name").value)
        self._right_joint = str(self.get_parameter("right_finger_joint_name").value)
        self._required_arm_joints = {
            str(name) for name in list(self.get_parameter("required_arm_joint_names").value)
        }
        self._latest_arm_state: JointState | None = None
        self._latest_gripper_position = lower
        self._warned_invalid_arm_state = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        visual_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            JointState,
            f"/{namespace}/visual_joint_states",
            visual_qos,
        )
        self.create_subscription(
            JointState,
            f"/{namespace}/joint_states",
            self._on_arm_joint_state,
            sensor_qos,
        )
        self.create_subscription(
            JointMotorState,
            f"/{namespace}/gripper/state",
            self._on_gripper_state,
            sensor_qos,
        )
        self.get_logger().info(
            f"publishing RViz visual joint states on /{namespace}/visual_joint_states"
        )

    def _on_arm_joint_state(self, msg: JointState) -> None:
        if not self._valid_arm_joint_state(msg):
            if not self._warned_invalid_arm_state:
                self.get_logger().warn(
                    "ignored empty or incomplete arm joint state for RViz visual_joint_states"
                )
                self._warned_invalid_arm_state = True
            return
        self._warned_invalid_arm_state = False
        self._latest_arm_state = msg
        self._publish_visual_state(msg.header)

    def _on_gripper_state(self, msg: JointMotorState) -> None:
        if math.isfinite(float(msg.position)):
            self._latest_gripper_position = float(msg.position)
        header = msg.header
        if self._latest_arm_state is not None:
            header = self._latest_arm_state.header
        self._publish_visual_state(header)

    def _publish_visual_state(self, header) -> None:
        if self._latest_arm_state is None or not self._valid_arm_joint_state(self._latest_arm_state):
            return
        left, right = gripper_opening_to_finger_joint_positions(
            self._latest_gripper_position,
            self._gripper_limits,
        )
        msg = JointState()
        msg.header = header
        names, positions, velocities, efforts = self._filtered_state_parts(self._latest_arm_state)
        msg.name = names + [self._left_joint, self._right_joint]
        msg.position = positions + [left, right]
        msg.velocity = velocities + [0.0, 0.0] if velocities else []
        msg.effort = efforts + [0.0, 0.0] if efforts else []
        self._publisher.publish(msg)

    def _valid_arm_joint_state(self, state: JointState | None) -> bool:
        if state is None:
            return False
        if not state.name or not state.position:
            return False
        if len(state.position) < len(state.name):
            return False
        names = {str(name) for name in state.name}
        if self._required_arm_joints and not self._required_arm_joints.issubset(names):
            return False
        return all(math.isfinite(float(value)) for value in state.position[: len(state.name)])

    def _filtered_state_parts(self, state: JointState) -> tuple[list[str], list[float], list[float], list[float]]:
        skip = {self._left_joint, self._right_joint}
        names: list[str] = []
        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []
        has_velocity = len(state.velocity) == len(state.name)
        has_effort = len(state.effort) == len(state.name)
        for index, name in enumerate(state.name):
            if name in skip:
                continue
            names.append(str(name))
            positions.append(float(state.position[index]))
            if has_velocity:
                velocities.append(float(state.velocity[index]))
            if has_effort:
                efforts.append(float(state.effort[index]))
        return names, positions, velocities, efforts


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperVisualJointStateNode()
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

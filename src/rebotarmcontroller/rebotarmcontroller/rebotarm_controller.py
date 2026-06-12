from __future__ import annotations

import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .teach_recorder import InternalTeachRecorder


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("arm_config", "")
        self.declare_parameter("gripper_config", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("shutdown_safe_home", True)
        self.declare_parameter("teach_record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("teach_record_rate_hz", 150.0)
        self.declare_parameter("teach_record_require_gravity_comp", True)

        arm_config = self.get_parameter("arm_config").value or None
        gripper_config = self.get_parameter("gripper_config").value or None
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        teach_record_path = str(self.get_parameter("teach_record_path").value)
        teach_record_rate_hz = float(self.get_parameter("teach_record_rate_hz").value)
        teach_record_require_gravity_comp = bool(
            self.get_parameter("teach_record_require_gravity_comp").value
        )
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = None
        self.joint_state_publisher = None
        self.arm_services = None
        self.teach_recorder = None
        self.arm_actions = None
        self.motor_passthrough = None
        self.hardware = HardwareManager(
            arm_cfg=arm_config,
            gripper_cfg=gripper_config,
            channel=channel,
        )
        try:
            self.hardware.connect()
        except Exception as exc:
            self.get_logger().error(f"hardware connect failed; disabled before exit: {exc}")
            raise

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.teach_recorder = InternalTeachRecorder(
            self,
            self.hardware,
            self.arm_namespace,
            record_path=teach_record_path,
            rate_hz=teach_record_rate_hz,
            require_gravity_comp=teach_record_require_gravity_comp,
        )
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def publish_arm_status(self) -> None:
        self.joint_state_publisher.publish_status()

    def shutdown(self) -> None:
        if self.teach_recorder is not None:
            self.teach_recorder.shutdown()
        if self.hardware is None:
            return
        if bool(self.get_parameter("shutdown_safe_home").value):
            self._conditional_safe_home_before_shutdown()
        self.hardware.shutdown()

    def _conditional_safe_home_before_shutdown(self) -> None:
        if not (self.hardware is not None and self.hardware.connected and self.hardware.enabled):
            return
        initial_state = self.hardware.state_machine
        try:
            self.hardware.stop_active_motion()
        except Exception as exc:
            self.get_logger().error(f"shutdown trajectory stop/hold failed; disabling: {exc}")
            return

        allowed, reason = self._safe_home_shutdown_allowed(initial_state)
        if not allowed:
            self.get_logger().warn(f"shutdown safe_home skipped: {reason}; disabling")
            return

        try:
            self.get_logger().warn("shutdown requested: running conditional safe_home before disable")
            self.hardware.stop_gravity_compensation()
            self.hardware.ensure_pos_vel_control()
            self.hardware.endpos_ctrl.safe_home()
            self.get_logger().info("shutdown conditional safe_home complete")
        except Exception as exc:
            self.get_logger().error(f"shutdown safe_home failed; disabling anyway: {exc}")

    def _safe_home_shutdown_allowed(self, initial_state: str) -> tuple[bool, str]:
        if self.hardware is None:
            return False, "hardware unavailable"
        if initial_state in ("LOWLEVEL_STREAMING", "GRAVITY_COMP"):
            return False, f"unsafe controller state {initial_state}"
        if self.hardware.gripper_active:
            return False, f"gripper active in {self.hardware.gripper_mode}"
        try:
            positions, velocities, _effort = self.hardware.get_joint_state()
        except Exception as exc:
            return False, f"joint state unavailable: {exc}"
        expected = len(self.hardware.joint_names)
        if len(positions) != expected or len(velocities) != expected:
            return False, "joint state size mismatch"
        if not all(math.isfinite(float(value)) for value in positions):
            return False, "joint positions are not finite"
        if not all(math.isfinite(float(value)) for value in velocities):
            return False, "joint velocities are not finite"
        return True, "ok"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = reBotArmController()
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None:
            node.shutdown()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

from __future__ import annotations

import json
import select
import sys
import termios
import time
import tty
from contextlib import suppress

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .parameter_helpers import build_joint_limits, sensor_qos_kwargs
from .teleop_core import KeyboardCommandMapper, TeleopTargetPlanner


def _set_duration(duration_msg, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


class TeleopKeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_keyboard_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        self.declare_parameter("joint_step_rad", 0.02)
        self.declare_parameter("trajectory_duration", 0.2)
        self.declare_parameter("poll_period", 0.05)
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("deadman_required", False)
        self.declare_parameter("deadman_key", "z")

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        self._joint_limits = build_joint_limits(
            joint_names=self._joint_names,
            lower_limits=lower,
            upper_limits=upper,
        )
        self._trajectory_duration = float(self.get_parameter("trajectory_duration").value)
        self._input_timeout = float(self.get_parameter("input_timeout").value)
        self._deadman_required = bool(self.get_parameter("deadman_required").value)
        self._deadman_key = str(self.get_parameter("deadman_key").value)
        self._deadman_until = 0.0
        self._last_input_time = 0.0
        self._current_positions = {name: 0.0 for name in self._joint_names}
        self._terminal_settings = None
        self._mapper = KeyboardCommandMapper(joint_names=self._joint_names)
        self._planner = TeleopTargetPlanner(
            joint_names=self._joint_names,
            joint_limits=self._joint_limits,
            joint_step_rad=float(self.get_parameter("joint_step_rad").value),
        )
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/status",
            10,
        )
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        try:
            if sys.stdin.isatty():
                self._terminal_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
        except Exception as exc:
            self.get_logger().warn(f"keyboard raw mode unavailable: {exc}")

        self.create_timer(float(self.get_parameter("poll_period").value), self._poll_key)
        self._publish_status("idle", "keyboard teleop ready")

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self._current_positions:
                self._current_positions[str(name)] = float(position)

    def _poll_key(self) -> None:
        key = self._read_key()
        now = time.monotonic()
        if key is None:
            if self._last_input_time and now - self._last_input_time > self._input_timeout:
                self._publish_status("timeout", "keyboard input timeout")
                self._last_input_time = 0.0
            return
        self._last_input_time = now
        if key == " ":
            self._publish_status("stopped", "keyboard stop requested")
            return
        if key == self._deadman_key:
            self._deadman_until = now + self._input_timeout
            self._publish_status("deadman", "deadman refreshed")
            return
        if self._deadman_required and now > self._deadman_until:
            self._publish_status("blocked", "deadman key required")
            return
        command = self._mapper.command_for_key(key)
        if command is None:
            return
        target = self._planner.apply_delta(
            current_positions=self._current_positions,
            joint_name=command.joint_name,
            direction=command.direction,
        )
        if not target.accepted:
            self._publish_status("rejected", target.message)
            return
        self._send_target(target.joint_names, target.positions, target.message, key)

    def _read_key(self) -> str | None:
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                return sys.stdin.read(1)
        except Exception:
            return None
        return None

    def _send_target(
        self,
        joint_names: tuple[str, ...],
        positions: tuple[float, ...],
        message: str,
        key: str,
    ) -> None:
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            self._publish_status("unavailable", "follow_joint_trajectory action unavailable")
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        _set_duration(point.time_from_start, self._trajectory_duration)
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda _future: self._publish_status("active", message, key=key))

    def _publish_status(self, state: str, message: str, *, key: str | None = None) -> None:
        msg = String()
        payload = {
            "source": "keyboard",
            "state": state,
            "message": message,
            "last_key": key,
            "joint_positions": [self._current_positions[name] for name in self._joint_names],
        }
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            if self._terminal_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()

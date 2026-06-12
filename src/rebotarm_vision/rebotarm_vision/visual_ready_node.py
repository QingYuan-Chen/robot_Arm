from __future__ import annotations

import math
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _duration_msg(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def _smoothstep(ratio: float) -> float:
    ratio = max(0.0, min(1.0, float(ratio)))
    return 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5


def _finite_positions(values: list[float] | tuple[float, ...]) -> bool:
    return len(values) == len(ARM_JOINT_NAMES) and all(math.isfinite(float(v)) for v in values)


class VisualReadyNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_visual_ready")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("auto_move_on_start", True)
        self.declare_parameter("exit_after_startup_move", False)
        self.declare_parameter("startup_delay_sec", 0.0)
        self.declare_parameter("joint_positions", [0.0, -0.1, -0.2, 0.2, 0.0, 0.0])
        self.declare_parameter("duration_sec", 4.0)
        self.declare_parameter("wait_timeout_sec", 12.0)
        self.declare_parameter("max_start_delta_rad", 1.0)

        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_state_topic = f"/{namespace}/joint_states"
        self._action_name = f"/{namespace}/follow_joint_trajectory"
        self._latest_joint_state: JointState | None = None

        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, qos_profile_sensor_data)
        self._trajectory_client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self._move_service = self.create_service(
            Trigger,
            f"/{namespace}/visual_ready/move",
            self._handle_move_request,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _handle_move_request(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = self.move_to_visual_ready()
        response.message = "visual_ready reached" if response.success else "visual_ready move failed"
        return response

    def move_to_visual_ready(self) -> bool:
        target = [float(v) for v in self.get_parameter("joint_positions").value]
        if not _finite_positions(target):
            self.get_logger().error(f"joint_positions must contain 6 finite values, got {target}")
            return False

        current = self._wait_for_current_positions()
        if current is None:
            self.get_logger().error(f"no valid arm joint state received on {self._joint_state_topic}")
            return False

        max_start_delta = float(self.get_parameter("max_start_delta_rad").value)
        worst_delta = max(abs(t - c) for t, c in zip(target, current))
        if max_start_delta > 0.0 and worst_delta > max_start_delta:
            self.get_logger().error(
                "visual_ready startup move refused: "
                f"max joint delta {worst_delta:.3f} rad > limit {max_start_delta:.3f} rad"
            )
            return False

        if not self._trajectory_client.wait_for_server(timeout_sec=float(self.get_parameter("wait_timeout_sec").value)):
            self.get_logger().error(f"follow_joint_trajectory action unavailable: {self._action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_trajectory(current, target)
        self.get_logger().info(
            "moving to visual_ready: "
            + ", ".join(f"{name}={value:+.3f}" for name, value in zip(ARM_JOINT_NAMES, target))
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("visual_ready trajectory rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        success = int(result.error_code) == int(FollowJointTrajectory.Result.SUCCESSFUL)
        if not success:
            self.get_logger().error(
                f"visual_ready trajectory failed: error_code={result.error_code}, message={result.error_string}"
            )
        return success

    def _wait_for_current_positions(self) -> list[float] | None:
        deadline = time.monotonic() + float(self.get_parameter("wait_timeout_sec").value)
        while rclpy.ok() and time.monotonic() < deadline:
            msg = self._latest_joint_state
            if msg is not None:
                positions = self._extract_arm_positions(msg)
                if positions is not None:
                    return positions
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _extract_arm_positions(self, msg: JointState) -> list[float] | None:
        by_name = {name: index for index, name in enumerate(msg.name)}
        if not all(name in by_name for name in ARM_JOINT_NAMES):
            return None
        positions = [float(msg.position[by_name[name]]) for name in ARM_JOINT_NAMES]
        if not _finite_positions(positions):
            return None
        return positions

    def _build_trajectory(self, current: list[float], target: list[float]) -> JointTrajectory:
        duration = max(float(self.get_parameter("duration_sec").value), 0.2)
        steps = max(2, int(duration / 0.05))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        for step in range(steps + 1):
            ratio = step / float(steps)
            blend = _smoothstep(ratio)
            point = JointTrajectoryPoint()
            point.positions = [float(c + (t - c) * blend) for c, t in zip(current, target)]
            point.time_from_start = _duration_msg(duration * ratio)
            trajectory.points.append(point)
        return trajectory

    def wait_before_startup_move(self) -> None:
        delay = max(float(self.get_parameter("startup_delay_sec").value), 0.0)
        if delay <= 0.0:
            return
        self.get_logger().info(f"visual_ready startup move delayed {delay:.1f}s")
        deadline = time.monotonic() + delay
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(deadline - time.monotonic(), 0.0)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisualReadyNode()
    try:
        if bool(node.get_parameter("auto_move_on_start").value):
            node.wait_before_startup_move()
            if node.move_to_visual_ready():
                node.get_logger().info("visual_ready startup move complete")
            if bool(node.get_parameter("exit_after_startup_move").value):
                return
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

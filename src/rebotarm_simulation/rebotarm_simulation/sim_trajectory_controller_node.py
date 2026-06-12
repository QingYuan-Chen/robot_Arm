from __future__ import annotations

import bisect
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from rebotarm_msgs.msg import JointMotorState
from rebotarm_msgs.srv import SetGripper
from trajectory_msgs.msg import JointTrajectory

from .sim_gripper import gripper_joint_positions_for_width


def _duration_to_sec(duration: Duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class SimTrajectoryControllerNode(Node):
    """RViz-only FollowJointTrajectory server that animates complete joint states."""

    def __init__(self) -> None:
        super().__init__("rebotarm_sim_trajectory_controller")
        self.declare_parameter(
            "joint_names",
            [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "left_finger_joint",
                "right_finger_joint",
            ],
        )
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("default_duration_sec", 2.0)
        self.declare_parameter("gripper_max_width_m", 0.09)
        self.declare_parameter("gripper_min_width_m", 0.0)
        self.declare_parameter("initial_joint_positions", [0.0, -0.1, -0.2, 0.2, 0.0, 0.0])

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = [str(v) for v in list(self.get_parameter("joint_names").value)]
        self._publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._default_duration_sec = max(float(self.get_parameter("default_duration_sec").value), 0.1)
        self._gripper_max_width_m = max(float(self.get_parameter("gripper_max_width_m").value), 0.0)
        self._gripper_min_width_m = max(float(self.get_parameter("gripper_min_width_m").value), 0.0)
        self._positions_by_name = {name: 0.0 for name in self._joint_names}
        for name, position in zip(self._joint_names, list(self.get_parameter("initial_joint_positions").value)):
            self._positions_by_name[str(name)] = float(position)
        self._velocities_by_name = {name: 0.0 for name in self._joint_names}
        self._last_gripper_width_m = self._gripper_min_width_m
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()

        self._joint_state_pub = self.create_publisher(JointState, f"/{self._arm_namespace}/joint_states", 10)
        self._gripper_state_pub = self.create_publisher(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            10,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.create_service(Trigger, f"/{self._arm_namespace}/trajectory_stop", self._stop_service)
        self.create_service(SetGripper, f"/{self._arm_namespace}/gripper/set", self._set_gripper_service)
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_joint_state)
        self.get_logger().info(
            f"RViz sim trajectory controller ready: /{self._arm_namespace}/follow_joint_trajectory, "
            f"/{self._arm_namespace}/joint_states"
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not goal_request.trajectory.joint_names:
            self.get_logger().warn("rejecting empty trajectory goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._stop_requested.set()
        return CancelResponse.ACCEPT

    def _stop_service(self, _request, response):
        self._stop_requested.set()
        response.success = True
        response.message = "sim trajectory stop requested"
        return response

    def _set_gripper_service(self, request, response):
        left_position, right_position, width = gripper_joint_positions_for_width(
            float(request.position),
            min_width=self._gripper_min_width_m,
            max_width=self._gripper_max_width_m,
        )
        with self._lock:
            if "left_finger_joint" in self._positions_by_name:
                self._positions_by_name["left_finger_joint"] = left_position
            if "right_finger_joint" in self._positions_by_name:
                self._positions_by_name["right_finger_joint"] = right_position
            if "left_finger_joint" in self._velocities_by_name:
                self._velocities_by_name["left_finger_joint"] = 0.0
            if "right_finger_joint" in self._velocities_by_name:
                self._velocities_by_name["right_finger_joint"] = 0.0
        response.success = True
        response.reached_position = width
        self._last_gripper_width_m = width
        self._publish_gripper_state(width)
        self.get_logger().info(f"sim gripper set: width={width:.4f} m")
        return response

    def _publish_gripper_state(self, width_m: float) -> None:
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(width_m)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 0
        self._gripper_state_pub.publish(msg)

    def _execute_goal(self, goal_handle):
        self._stop_requested.clear()
        trajectory = self._normalize_trajectory(goal_handle.request.trajectory)
        result = FollowJointTrajectory.Result()
        if trajectory is None:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory contains no usable points"
            goal_handle.abort()
            return result

        names, points, times = trajectory
        start_time = time.monotonic()
        end_time = times[-1]
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = names

        while rclpy.ok():
            if goal_handle.is_cancel_requested or self._stop_requested.is_set():
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "sim trajectory canceled"
                return result
            elapsed = min(time.monotonic() - start_time, end_time)
            positions = self._interpolate(points, times, elapsed)
            self._set_positions(names, positions)
            feedback.actual.positions = positions
            feedback.desired.positions = positions
            goal_handle.publish_feedback(feedback)
            if elapsed >= end_time:
                break
            time.sleep(1.0 / self._publish_rate_hz)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "sim trajectory finished"
        return result

    def _set_positions(self, names: list[str], positions: list[float]) -> None:
        with self._lock:
            for name, position in zip(names, positions):
                self._positions_by_name[name] = float(position)
                self._velocities_by_name[name] = 0.0

    def _normalize_trajectory(self, trajectory: JointTrajectory):
        source_names = list(trajectory.joint_names)
        if not source_names or not trajectory.points:
            return None

        names = [name for name in source_names if name in self._positions_by_name]
        if not names:
            return None
        name_to_source_index = {name: index for index, name in enumerate(source_names)}

        points: list[list[float]] = []
        times: list[float] = []
        last_time = 0.0
        for index, point in enumerate(trajectory.points):
            positions = []
            with self._lock:
                current = dict(self._positions_by_name)
            for name in names:
                source_index = name_to_source_index[name]
                if source_index < len(point.positions):
                    positions.append(float(point.positions[source_index]))
                else:
                    positions.append(float(current.get(name, 0.0)))
            point_time = _duration_to_sec(point.time_from_start)
            if point_time <= 0.0:
                point_time = self._default_duration_sec * float(index + 1) / float(len(trajectory.points))
            point_time = max(point_time, last_time + 1e-3)
            last_time = point_time
            points.append(positions)
            times.append(point_time)
        return names, points, times

    def _interpolate(self, points: list[list[float]], times: list[float], elapsed: float) -> list[float]:
        if elapsed <= times[0]:
            return list(points[0])
        if elapsed >= times[-1]:
            return list(points[-1])
        right = bisect.bisect_left(times, elapsed)
        left = max(0, right - 1)
        t0 = times[left]
        t1 = times[right]
        ratio = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
        return [float(p0) + (float(p1) - float(p0)) * ratio for p0, p1 in zip(points[left], points[right])]

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        with self._lock:
            msg.position = [float(self._positions_by_name[name]) for name in self._joint_names]
            msg.velocity = [float(self._velocities_by_name[name]) for name in self._joint_names]
        msg.effort = [0.0 for _ in self._joint_names]
        self._joint_state_pub.publish(msg)
        self._publish_gripper_state(self._last_gripper_width_m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimTrajectoryControllerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

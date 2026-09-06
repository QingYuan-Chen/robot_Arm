"""ROS 2 client used by guarded real and simulated trajectory workflows.

This module owns no task policy.  It observes controller feedback, applies the
shared runtime guard, and sends already validated FollowJointTrajectory goals.
"""

from __future__ import annotations

from collections import deque
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from rebotarm_motion.paired_trajectory_protocol import (
    ARM_JOINT_NAMES,
    sample_command,
    validate_command,
)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _duration(seconds: float):
    total_nanoseconds = int(round(float(seconds) * 1_000_000_000))
    whole, nanoseconds = divmod(total_nanoseconds, 1_000_000_000)
    from builtin_interfaces.msg import Duration

    return Duration(sec=whole, nanosec=nanoseconds)


class GuardedTrajectoryNode(Node):
    def __init__(
        self,
        *,
        namespace: str,
        backend: str,
    ) -> None:
        super().__init__(f"rebotarm_guarded_trajectory_{backend}")
        self.namespace = namespace.strip("/")
        self.backend = backend
        self.latest_joint_state: JointState | None = None
        self.latest_status: ArmStatus | None = None
        self.latest_gripper: JointMotorState | None = None
        self.last_joint_monotonic: float | None = None
        self.active_command: dict[str, object] | None = None
        self.active_start_ns: int | None = None
        self.active_samples: list[dict[str, object]] = []
        self.guard_stop: dict[str, object] | None = None
        self.position_history: deque[tuple[float, tuple[float, ...]]] = deque()
        self.action_feedback: list[dict[str, object]] = []
        self.create_subscription(
            JointState,
            f"/{self.namespace}/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if backend == "real":
            self.create_subscription(
                ArmStatus,
                f"/{self.namespace}/arm_status",
                self._status_callback,
                latched,
            )
        self.create_subscription(
            JointMotorState,
            f"/{self.namespace}/gripper/state",
            self._gripper_callback,
            qos_profile_sensor_data,
        )
        self.action = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.namespace}/follow_joint_trajectory",
        )
        self.enable_client = self.create_client(Trigger, f"/{self.namespace}/enable")
        self.disable_client = self.create_client(Trigger, f"/{self.namespace}/disable")
        self.stop_client = self.create_client(Trigger, f"/{self.namespace}/trajectory_stop")

    def _status_callback(self, message: ArmStatus) -> None:
        self.latest_status = message
        if self.backend != "real" or self.active_command is None:
            return
        if not message.enabled or not message.control_loop_active:
            self.guard_stop = {"reason": "controller_left_enabled_control_state"}
        elif any(int(value) != 1 for value in message.per_joint_status_code):
            self.guard_stop = {
                "reason": "motor_status_changed",
                "per_joint_status_code": [int(value) for value in message.per_joint_status_code],
            }
        elif message.error_codes:
            self.guard_stop = {
                "reason": "controller_error",
                "error_codes": list(message.error_codes),
            }

    def _gripper_callback(self, message: JointMotorState) -> None:
        self.latest_gripper = message

    def _joint_state_callback(self, message: JointState) -> None:
        now = time.monotonic()
        now_ns = time.monotonic_ns()
        self.latest_joint_state = message
        self.last_joint_monotonic = now
        if self.active_command is None or self.active_start_ns is None:
            return
        by_name = {name: index for index, name in enumerate(message.name)}
        if any(name not in by_name for name in ARM_JOINT_NAMES):
            self.guard_stop = {"reason": "missing_joint", "names": list(message.name)}
            return
        if any(
            by_name[name] >= len(values)
            for values in (message.position, message.velocity, message.effort)
            for name in ARM_JOINT_NAMES
        ):
            self.guard_stop = {
                "reason": "incomplete_joint_state_arrays",
                "position_count": len(message.position),
                "velocity_count": len(message.velocity),
                "effort_count": len(message.effort),
            }
            return
        elapsed = max(0.0, (now_ns - self.active_start_ns) * 1e-9)
        desired = sample_command(self.active_command, elapsed)
        observed = tuple(float(message.position[by_name[name]]) for name in ARM_JOINT_NAMES)
        velocities = tuple(float(message.velocity[by_name[name]]) for name in ARM_JOINT_NAMES)
        efforts = tuple(float(message.effort[by_name[name]]) for name in ARM_JOINT_NAMES)
        if not all(math.isfinite(value) for values in (observed, velocities, efforts) for value in values):
            self.guard_stop = {"reason": "non_finite_joint_state"}
            return
        self.position_history.append((now, observed))
        while self.position_history and now - self.position_history[0][0] > 0.30:
            self.position_history.popleft()
        window_velocities = self._window_velocities(now, observed)
        tracking = tuple(target - actual for target, actual in zip(desired, observed))
        sample = {
            "monotonic_ns": now_ns,
            "ros_stamp_ns": _stamp_ns(message.header.stamp),
            "elapsed_sec": elapsed,
            "desired_positions": list(desired),
            "observed_positions": list(observed),
            "velocities": list(velocities),
            "window_velocities": list(window_velocities),
            "efforts": list(efforts),
            "tracking_errors": list(tracking),
            "action_state": "executing",
            "gripper": self._gripper_payload(),
            "contact": {"available": False, "reason": "no common real/sim contact topic"},
            "arm_status": self._status_payload(),
        }
        self.active_samples.append(sample)
    def _window_velocities(self, now: float, observed: tuple[float, ...]) -> tuple[float, ...]:
        candidate = None
        for timestamp, positions in self.position_history:
            if now - timestamp >= 0.20:
                candidate = (timestamp, positions)
            else:
                break
        if candidate is None:
            return (0.0,) * len(ARM_JOINT_NAMES)
        timestamp, positions = candidate
        delta = max(now - timestamp, 1e-9)
        return tuple((current - previous) / delta for current, previous in zip(observed, positions))

    def _gripper_payload(self) -> dict[str, object]:
        if self.latest_gripper is None:
            return {"available": False}
        message = self.latest_gripper
        return {
            "available": True,
            "position": float(message.position),
            "velocity": float(message.velocity),
            "torque": float(message.torque),
            "status_code": int(message.status_code),
        }

    def _status_payload(self) -> dict[str, object]:
        if self.latest_status is None:
            return {"available": False}
        message = self.latest_status
        return {
            "available": True,
            "enabled": bool(message.enabled),
            "control_loop_active": bool(message.control_loop_active),
            "state_machine": str(message.state_machine),
            "per_joint_status_code": [int(value) for value in message.per_joint_status_code],
            "error_codes": [str(value) for value in message.error_codes],
        }

    def wait_for_preflight(self, timeout_sec: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            status_ready = self.backend != "real" or self.latest_status is not None
            if self.latest_joint_state is not None and status_ready:
                break
        if self.latest_joint_state is None:
            raise RuntimeError("joint feedback unavailable")
        publishers = self.get_publishers_info_by_topic(f"/{self.namespace}/joint_states")
        if len(publishers) != 1:
            raise RuntimeError(f"expected one joint-state publisher, got {len(publishers)}")
        if self.backend == "real":
            assert self.latest_status is not None
            if self.latest_status.enabled or self.latest_status.control_loop_active:
                raise RuntimeError("real controller must start disabled")
            if any(int(value) != 0 for value in self.latest_status.per_joint_status_code):
                raise RuntimeError("real controller has non-zero motor status")
            if self.latest_status.error_codes:
                raise RuntimeError(f"real controller reports errors: {list(self.latest_status.error_codes)}")

    def canonical_positions(self) -> tuple[float, ...]:
        if self.latest_joint_state is None:
            raise RuntimeError("joint state unavailable")
        by_name = {name: index for index, name in enumerate(self.latest_joint_state.name)}
        return tuple(float(self.latest_joint_state.position[by_name[name]]) for name in ARM_JOINT_NAMES)

    def call_trigger(self, client, label: str, timeout_sec: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{label} service unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"{label} failed: {message}")
        return {"success": True, "message": str(response.message)}

    def _feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.action_feedback.append(
            {
                "monotonic_ns": time.monotonic_ns(),
                "joint_names": list(feedback.joint_names),
                "desired_positions": list(feedback.desired.positions),
                "actual_positions": list(feedback.actual.positions),
                "error_positions": list(feedback.error.positions),
            }
        )

    def execute_leg(self, command: dict[str, object]) -> dict[str, object]:
        validate_command(command)
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowJointTrajectory action unavailable")
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        for source in command["points"]:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in source["positions"]]
            point.time_from_start = _duration(float(source["elapsed_sec"]))
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.active_command = command
        self.active_samples = []
        self.action_feedback = []
        self.guard_stop = None
        self.position_history.clear()
        self.active_start_ns = time.monotonic_ns()
        send_future = self.action.send_goal_async(goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.active_command = None
            raise RuntimeError("trajectory goal rejected")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + float(command["duration_sec"]) + 12.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.guard_stop is not None:
                goal_handle.cancel_goal_async()
                if self.backend == "real":
                    self.call_trigger(self.stop_client, "trajectory_stop")
                break
            if self.last_joint_monotonic is None or time.monotonic() - self.last_joint_monotonic > 0.5:
                self.guard_stop = {"reason": "joint_state_stale"}
                goal_handle.cancel_goal_async()
                if self.backend == "real":
                    self.call_trigger(self.stop_client, "trajectory_stop")
                break
        if not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        wrapped = result_future.result()
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.02)
        self.active_command = None
        self.active_start_ns = None
        if wrapped is None:
            raise RuntimeError("trajectory result unavailable")
        result_payload = {
            "status": int(wrapped.status),
            "error_code": int(wrapped.result.error_code),
            "error_string": str(wrapped.result.error_string),
            "guard_stop": self.guard_stop,
        }
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            and self.guard_stop is None
        )
        return {
            "command": command,
            "samples": self.active_samples,
            "action_feedback": self.action_feedback,
            "result": result_payload,
            "success": succeeded,
        }

    def hold_and_collect(self, seconds: float) -> None:
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

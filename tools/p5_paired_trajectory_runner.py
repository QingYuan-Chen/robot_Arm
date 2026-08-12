#!/usr/bin/env python3
"""Collect one deterministic safe-posture round trip from ROS 2.

This task-level tool talks only to ROS topics, services, and the existing
FollowJointTrajectory action. It never imports the motor SDK directly.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
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


ROOT = Path(__file__).resolve().parents[1]
for package in ("rebotarm_motion", "rebotarm_simulation"):
    source = ROOT / "src" / package
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from rebotarm_motion.hardware_acceptance_guard import HardwareAcceptanceGuard
from rebotarm_motion.real_failure_recovery import recover_real_failure
from rebotarm_motion.paired_trajectory_protocol import (
    ARM_JOINT_NAMES,
    sample_command,
    validate_command,
)


EFFORT_LIMITS = {
    "joint1": 27.0,
    "joint2": 27.0,
    "joint3": 27.0,
    "joint4": 7.0,
    "joint5": 7.0,
    "joint6": 7.0,
}
REAL_CONFIRMATION = "REAL_PAIRED_SAFE_POSTURE"


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _duration(seconds: float):
    total_nanoseconds = int(round(float(seconds) * 1_000_000_000))
    whole, nanoseconds = divmod(total_nanoseconds, 1_000_000_000)
    from builtin_interfaces.msg import Duration

    return Duration(sec=whole, nanosec=nanoseconds)


class PairedTrajectoryNode(Node):
    def __init__(self, *, namespace: str, backend: str) -> None:
        super().__init__(f"p5_paired_trajectory_{backend}")
        self.namespace = namespace.strip("/")
        self.backend = backend
        self.latest_joint_state: JointState | None = None
        self.latest_status: ArmStatus | None = None
        self.latest_gripper: JointMotorState | None = None
        self.last_joint_monotonic: float | None = None
        self.active_command: dict[str, object] | None = None
        self.active_start_ns: int | None = None
        self.active_samples: list[dict[str, object]] = []
        self.active_guard = HardwareAcceptanceGuard(EFFORT_LIMITS) if backend == "real" else None
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
        if self.active_guard is not None:
            decision = self.active_guard.observe(
                now=now,
                raw_velocities_by_joint=dict(zip(ARM_JOINT_NAMES, velocities)),
                window_velocities_by_joint=dict(zip(ARM_JOINT_NAMES, window_velocities)),
                efforts_by_joint=dict(zip(ARM_JOINT_NAMES, efforts)),
                tracking_errors_by_joint=dict(zip(ARM_JOINT_NAMES, tracking)),
            )
            if decision.should_stop:
                self.guard_stop = {
                    "reason": decision.reason,
                    "joint": decision.joint,
                    "value": decision.value,
                    "limit": decision.limit,
                    "duration_sec": decision.duration_sec,
                }

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
        self.active_guard.reset() if self.active_guard is not None else None
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


def _load_command(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    validate_command(payload)
    return payload


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "real"), required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--outbound-command", type=Path)
    parser.add_argument("--return-command", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hold-sec", type=float, default=2.0)
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="record disabled feedback/status and exit without enabling or sending a goal",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not args.preflight_only and (args.outbound_command is None or args.return_command is None):
        raise SystemExit("trajectory run requires --outbound-command and --return-command")
    if args.backend == "real" and not args.preflight_only and args.confirm != REAL_CONFIRMATION:
        raise SystemExit(f"real run requires --confirm {REAL_CONFIRMATION}")
    outbound = None if args.preflight_only else _load_command(args.outbound_command)
    return_command = None if args.preflight_only else _load_command(args.return_command)
    rclpy.init()
    node = PairedTrajectoryNode(namespace=args.namespace, backend=args.backend)
    report: dict[str, object] = {
        "schema_version": 1,
        "backend": args.backend,
        "namespace": args.namespace.strip("/"),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "services": [],
        "legs": [],
        "success": False,
    }
    hardware_enabled = False
    allow_controlled_return = False
    captured_baseline: tuple[float, ...] | None = None
    try:
        node.wait_for_preflight()
        captured_baseline = node.canonical_positions()
        report["preflight_positions"] = list(captured_baseline)
        report["preflight_status"] = node._status_payload()
        if args.preflight_only:
            report["success"] = True
            return
        assert outbound is not None and return_command is not None
        command_start = tuple(float(value) for value in outbound["points"][0]["positions"])
        start_errors = [target - actual for target, actual in zip(command_start, node.canonical_positions())]
        report["command_start_errors"] = start_errors
        if max(abs(value) for value in start_errors) > 0.01:
            raise RuntimeError(f"live start differs from command by more than 0.01 rad: {start_errors}")
        if args.backend == "real":
            report["services"].append(node.call_trigger(node.enable_client, "enable"))
            hardware_enabled = True
            allow_controlled_return = True
            node.hold_and_collect(0.5)
        allow_controlled_return = False
        outbound_run = node.execute_leg(outbound)
        report["legs"].append(outbound_run)
        if not outbound_run["success"]:
            raise RuntimeError(f"outbound failed: {outbound_run['result']}")
        allow_controlled_return = True
        node.hold_and_collect(args.hold_sec)
        allow_controlled_return = False
        return_run = node.execute_leg(return_command)
        report["legs"].append(return_run)
        if not return_run["success"]:
            raise RuntimeError(f"return failed: {return_run['result']}")
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        final_positions = node.canonical_positions()
        baseline = tuple(float(value) for value in return_command["points"][-1]["positions"])
        final_errors = [target - actual for target, actual in zip(baseline, final_positions)]
        report["enabled_final_positions"] = list(final_positions)
        report["enabled_final_errors"] = final_errors
        if max(abs(value) for value in final_errors) > 0.02:
            raise RuntimeError(f"return baseline error exceeds 0.02 rad: {final_errors}")
        if args.backend == "real":
            report["services"].append(node.call_trigger(node.disable_client, "disable"))
            hardware_enabled = False
            node.hold_and_collect(0.5)
        report["final_status"] = node._status_payload()
        report["success"] = True
    except Exception as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        if args.backend == "real" and hardware_enabled:
            try:
                if captured_baseline is None:
                    raise RuntimeError("baseline unavailable for failure recovery")
                hardware_enabled = recover_real_failure(
                    node=node,
                    report=report,
                    baseline=captured_baseline,
                    allow_controlled_return=allow_controlled_return,
                    legs_key="legs",
                    command_label="paired_failure_return_baseline",
                )
            except Exception as recovery_exc:
                report["recovery_failure"] = f"{type(recovery_exc).__name__}: {recovery_exc}"
                report["recovery_final_status"] = node._status_payload()
        raise
    finally:
        report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        _write_json(args.output, report)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

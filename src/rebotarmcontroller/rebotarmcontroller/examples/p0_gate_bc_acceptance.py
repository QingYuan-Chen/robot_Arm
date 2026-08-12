#!/usr/bin/env python3
"""Run the P0 Gate B/C explicit-enable, hold, and disable acceptance test."""

from __future__ import annotations

import argparse
import json
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rebotarm_msgs.msg import ArmStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from ..p0_acceptance_core import (
    EXPECTED_JOINTS,
    arm_status_snapshot,
    joint_state_snapshot,
    motion_sample_details,
    validate_preflight,
)

CONFIRMATION_TOKEN = "ENABLE_HOLD_TEST"


class GateBCAcceptance(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("p0_gate_bc_acceptance")
        self._args = args
        namespace = args.namespace.strip("/")
        self._namespace = namespace
        self._latest_status: ArmStatus | None = None
        self._latest_joint_state: JointState | None = None
        self._last_joint_state_monotonic: float | None = None
        self._joint_sample_count = 0
        self._baseline_by_name: dict[str, float] | None = None
        self._monitor_motion = False
        self._max_position_jump_rad = 0.0
        self._max_abs_velocity_rad_s = 0.0
        self._peak_position_jump: dict[str, Any] | None = None
        self._peak_velocity: dict[str, Any] | None = None
        self._monitor_error: str | None = None
        self.stop_requested = False

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            ArmStatus,
            f"/{namespace}/arm_status",
            self._status_cb,
            status_qos,
        )
        self.create_subscription(
            JointState,
            f"/{namespace}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self._enable_client = self.create_client(Trigger, f"/{namespace}/enable")
        self._disable_client = self.create_client(Trigger, f"/{namespace}/disable")
        self._stop_client = self.create_client(
            Trigger,
            f"/{namespace}/trajectory_stop",
        )

    def _status_cb(self, msg: ArmStatus) -> None:
        self._latest_status = msg

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        self._last_joint_state_monotonic = time.monotonic()
        self._joint_sample_count += 1
        if not self._monitor_motion or self._baseline_by_name is None:
            return
        try:
            details = motion_sample_details(
                self._baseline_by_name,
                msg,
            )
        except ValueError as exc:
            self._monitor_error = str(exc)
            return
        max_jump = float(details["max_position_jump_rad"])
        max_velocity = float(details["max_abs_velocity_rad_s"])
        if max_jump > self._max_position_jump_rad:
            self._max_position_jump_rad = max_jump
            self._peak_position_jump = dict(details)
        if max_velocity > self._max_abs_velocity_rad_s:
            self._max_abs_velocity_rad_s = max_velocity
            self._peak_velocity = dict(details)
        if max_jump > self._args.max_position_jump_rad:
            self._monitor_error = (
                f"{details['position_jump_joint']} position jump "
                f"{max_jump:.6f} rad exceeds "
                f"{self._args.max_position_jump_rad:.6f} rad"
            )
        elif max_velocity > self._args.max_abs_velocity_rad_s:
            self._monitor_error = (
                f"{details['velocity_joint']} velocity "
                f"{max_velocity:.6f} rad/s exceeds "
                f"{self._args.max_abs_velocity_rad_s:.6f} rad/s"
            )

    def wait_until(
        self,
        predicate: Callable[[], bool],
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while rclpy.ok() and not self.stop_requested and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def call_trigger(
        self,
        client,
        label: str,
        *,
        timeout_sec: float | None = None,
    ) -> tuple[bool, str]:
        timeout = self._args.service_timeout_sec if timeout_sec is None else timeout_sec
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return False, f"{label} service unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False, f"{label} service timed out"
        response = future.result()
        if response is None:
            return False, f"{label} returned no response"
        return bool(response.success), str(response.message)

    def wait_for_initial_state(self) -> bool:
        return self.wait_until(
            lambda: self._latest_status is not None
            and self._latest_joint_state is not None
            and self._joint_sample_count >= 3,
            self._args.state_timeout_sec,
        )

    def start_motion_monitor(self) -> None:
        assert self._latest_joint_state is not None
        self._baseline_by_name = {
            name: float(position)
            for name, position in zip(
                self._latest_joint_state.name,
                self._latest_joint_state.position,
            )
        }
        self._max_position_jump_rad = 0.0
        self._max_abs_velocity_rad_s = 0.0
        self._peak_position_jump = None
        self._peak_velocity = None
        self._monitor_error = None
        self._monitor_motion = True

    def enabled_status_ready(self) -> bool:
        status = self._latest_status
        return bool(
            status is not None
            and status.enabled
            and status.control_loop_active
            and status.state_machine == "IDLE"
            and list(status.per_joint_status_code) == [1] * len(EXPECTED_JOINTS)
            and not status.error_codes
        )

    def disabled_status_ready(self) -> bool:
        status = self._latest_status
        return bool(
            status is not None
            and not status.enabled
            and not status.control_loop_active
            and status.state_machine == "IDLE"
            and list(status.per_joint_status_code) == [0] * len(EXPECTED_JOINTS)
        )

    def joint_state_is_fresh(self) -> bool:
        return bool(
            self._last_joint_state_monotonic is not None
            and time.monotonic() - self._last_joint_state_monotonic
            <= self._args.joint_state_stale_sec
        )

    def emergency_cleanup(self) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        stop_ok, stop_message = self.call_trigger(
            self._stop_client,
            "trajectory_stop",
            timeout_sec=min(self._args.service_timeout_sec, 3.0),
        )
        events.append(
            {"service": "trajectory_stop", "success": stop_ok, "message": stop_message}
        )
        disable_ok, disable_message = self.call_trigger(
            self._disable_client,
            "disable",
        )
        events.append(
            {"service": "disable", "success": disable_ok, "message": disable_message}
        )
        self.wait_until(self.disabled_status_ready, self._args.state_timeout_sec)
        return events


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--hold-seconds", type=float, default=10.0)
    parser.add_argument("--max-position-jump-rad", type=float, default=0.03)
    parser.add_argument("--max-abs-velocity-rad-s", type=float, default=0.05)
    parser.add_argument("--state-timeout-sec", type=float, default=5.0)
    parser.add_argument("--service-timeout-sec", type=float, default=12.0)
    parser.add_argument("--joint-state-stale-sec", type=float, default=0.5)
    parser.add_argument(
        "--report-dir",
        default="Agent/evidence/P0",
        help="directory for the JSON acceptance record",
    )
    return parser.parse_args()


def _write_report(report: dict[str, Any], report_dir: str) -> Path:
    output_dir = Path(report_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    path = output_dir / f"gate-bc-{stamp}.json"
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def main() -> None:
    args = _parse_args()
    if args.hold_seconds <= 0.0:
        raise SystemExit("--hold-seconds must be positive")
    if args.max_position_jump_rad <= 0.0:
        raise SystemExit("--max-position-jump-rad must be positive")
    if args.max_abs_velocity_rad_s <= 0.0:
        raise SystemExit("--max-abs-velocity-rad-s must be positive")
    if args.joint_state_stale_sec <= 0.0:
        raise SystemExit("--joint-state-stale-sec must be positive")

    report: dict[str, Any] = {
        "schema_version": 1,
        "gate": "P0-Gate-B-C",
        "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "namespace": args.namespace.strip("/"),
        "thresholds": {
            "hold_seconds": float(args.hold_seconds),
            "max_position_jump_rad": float(args.max_position_jump_rad),
            "max_abs_velocity_rad_s": float(args.max_abs_velocity_rad_s),
            "joint_state_stale_sec": float(args.joint_state_stale_sec),
        },
        "services": [],
        "outcome": "FAILED",
        "failure_reason": None,
    }

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GateBCAcceptance(args)
    enable_requested = False

    def request_stop(_signum, _frame) -> None:
        node.stop_requested = True
        node.get_logger().warning("stop requested; disabling hardware")

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        if not node.wait_for_initial_state():
            raise RuntimeError("arm_status/joint_states preflight data unavailable")
        assert node._latest_status is not None
        assert node._latest_joint_state is not None
        report["preflight_status"] = arm_status_snapshot(node._latest_status)
        report["preflight_joint_state"] = joint_state_snapshot(
            node._latest_joint_state
        )
        errors = validate_preflight(node._latest_status, node._latest_joint_state)
        if errors:
            raise RuntimeError("preflight failed: " + "; ".join(errors))
        baseline_speed = max(
            abs(float(value)) for value in node._latest_joint_state.velocity
        )
        if baseline_speed > args.max_abs_velocity_rad_s:
            raise RuntimeError(
                f"preflight velocity {baseline_speed:.6f} rad/s exceeds "
                f"{args.max_abs_velocity_rad_s:.6f} rad/s"
            )

        print("\nP0 Gate B/C preflight passed.")
        print("No trajectory, safe-home, or gripper command will be sent.")
        print("Baseline joint positions / rad:")
        for name, position in zip(
            node._latest_joint_state.name,
            node._latest_joint_state.position,
        ):
            print(f"  {name}: {float(position):+.6f}")
        print(
            f"Type {CONFIRMATION_TOKEN!r} only when the workcell is clear "
            "and the physical emergency stop is ready."
        )
        confirmation = input("> ").strip()
        if confirmation != CONFIRMATION_TOKEN:
            report["outcome"] = "ABORTED"
            raise RuntimeError("operator confirmation was not provided")
        if node.stop_requested:
            report["outcome"] = "ABORTED"
            raise RuntimeError("stop requested before enable")

        node.start_motion_monitor()
        enable_requested = True
        enable_ok, enable_message = node.call_trigger(
            node._enable_client,
            "enable",
        )
        report["services"].append(
            {"service": "enable", "success": enable_ok, "message": enable_message}
        )
        if not enable_ok:
            raise RuntimeError(f"enable failed: {enable_message}")
        if not node.wait_until(node.enabled_status_ready, args.state_timeout_sec):
            raise RuntimeError("enabled status verification failed")
        if not node.joint_state_is_fresh():
            raise RuntimeError("joint states became stale after enable")
        if node._monitor_error is not None:
            raise RuntimeError(node._monitor_error)

        hold_deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < hold_deadline:
            if node.stop_requested:
                report["outcome"] = "ABORTED"
                raise RuntimeError("operator requested stop during hold")
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.joint_state_is_fresh():
                raise RuntimeError("joint states became stale during hold")
            if node._monitor_error is not None:
                raise RuntimeError(node._monitor_error)

        node._monitor_motion = False
        samples_before_disable = node._joint_sample_count
        disable_ok, disable_message = node.call_trigger(
            node._disable_client,
            "disable",
        )
        report["services"].append(
            {"service": "disable", "success": disable_ok, "message": disable_message}
        )
        if not disable_ok:
            raise RuntimeError(f"disable failed: {disable_message}")
        if not node.wait_until(node.disabled_status_ready, args.state_timeout_sec):
            raise RuntimeError("disabled status verification failed")
        if not node.wait_until(
            lambda: node._joint_sample_count >= samples_before_disable + 2,
            args.state_timeout_sec,
        ):
            raise RuntimeError("joint states did not continue after disable")

        enable_requested = False
        report["outcome"] = "PASSED"
    except Exception as exc:
        report["failure_reason"] = str(exc)
        node.get_logger().error(str(exc))
    finally:
        unexpectedly_enabled = bool(
            node._latest_status is not None and node._latest_status.enabled
        )
        if enable_requested or unexpectedly_enabled:
            report["cleanup_services"] = node.emergency_cleanup()
        node._monitor_motion = False
        report["metrics"] = {
            "max_position_jump_rad": node._max_position_jump_rad,
            "max_abs_velocity_rad_s": node._max_abs_velocity_rad_s,
            "peak_position_jump": node._peak_position_jump,
            "peak_velocity": node._peak_velocity,
            "joint_samples": node._joint_sample_count,
        }
        if node._latest_status is not None:
            report["final_status"] = arm_status_snapshot(node._latest_status)
        if node._latest_joint_state is not None:
            report["final_joint_state"] = joint_state_snapshot(
                node._latest_joint_state
            )
        report["finished_at"] = datetime.now().astimezone().isoformat(
            timespec="seconds"
        )
        report_path = _write_report(report, args.report_dir)
        print(f"Acceptance report: {report_path}")
        print(f"Outcome: {report['outcome']}")
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if report["outcome"] == "PASSED" else 1)


if __name__ == "__main__":
    main()

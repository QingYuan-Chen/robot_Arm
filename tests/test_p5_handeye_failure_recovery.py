from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np
import pytest
from types import SimpleNamespace
import time


@dataclass
class _Status:
    enabled: bool = True
    control_loop_active: bool = True
    per_joint_status_code: list[int] = field(default_factory=lambda: [1] * 6)
    error_codes: list[str] = field(default_factory=list)


class _Node:
    stop_client = "stop"
    disable_client = "disable"

    def __init__(
        self,
        *,
        return_success: bool = True,
        critical: bool = False,
        return_exception: Exception | None = None,
        return_critical: bool = False,
        stop_exception: Exception | None = None,
        disable_exception: Exception | None = None,
        refresh_feedback: bool = True,
    ) -> None:
        self.latest_status = _Status()
        if critical:
            self.latest_status.error_codes = ["motor_fault"]
        self.return_success = return_success
        self.return_exception = return_exception
        self.return_critical = return_critical
        self.stop_exception = stop_exception
        self.disable_exception = disable_exception
        self.refresh_feedback = refresh_feedback
        self.calls: list[str] = []
        self.positions = np.zeros(6, dtype=np.float64)
        self.velocities = np.zeros(6, dtype=np.float64)
        self.last_joint_monotonic = time.monotonic()
        self.latest_joint_state = SimpleNamespace(
            name=[f"joint{index}" for index in range(1, 7)],
            position=self.positions,
            velocity=self.velocities,
        )

    def call_trigger(self, client, label: str):
        self.calls.append(label)
        if label == "trajectory_stop" and self.stop_exception is not None:
            raise self.stop_exception
        if label == "disable":
            if self.disable_exception is not None:
                raise self.disable_exception
            self.latest_status.enabled = False
            self.latest_status.control_loop_active = False
        return {"success": True, "message": label}

    def hold_and_collect(self, _seconds: float) -> None:
        if self.refresh_feedback:
            self.last_joint_monotonic = time.monotonic()
        self.latest_joint_state.position = self.positions
        self.latest_joint_state.velocity = self.velocities

    def _status_payload(self):
        return {
            "enabled": self.latest_status.enabled,
            "control_loop_active": self.latest_status.control_loop_active,
            "per_joint_status_code": list(self.latest_status.per_joint_status_code),
            "error_codes": list(self.latest_status.error_codes),
        }

    def canonical_positions(self):
        return tuple(float(value) for value in self.positions)

    def execute_leg(self, command):
        if self.return_exception is not None:
            self.latest_status.error_codes = ["ARM_FEEDBACK: arm feedback stale"]
            raise self.return_exception
        if self.return_critical:
            self.latest_status.error_codes = ["ARM_FEEDBACK: arm feedback stale"]
        if self.return_success:
            self.positions = np.asarray(command["points"][-1]["positions"], dtype=np.float64)
            self.latest_joint_state.position = self.positions
        return {
            "command": command,
            "samples": [],
            "action_feedback": [],
            "result": {"guard_stop": None},
            "success": self.return_success,
        }


def test_recoverable_capture_failure_returns_before_disable() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node()
    report = {"services": [], "motion_legs": []}
    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.full(6, 0.1),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
        verification_stable_sec=0.0,
    )
    assert still_enabled is False
    assert node.calls == ["trajectory_stop", "disable"]
    assert report["failure_recovery"]["outcome"] == "returned_to_baseline_then_disabled"
    assert report["motion_legs"][0]["recovery_leg"] is True


def test_failed_return_keeps_healthy_controller_enabled() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(return_success=False)
    report = {"services": [], "motion_legs": []}
    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.full(6, 0.1),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
    )
    assert still_enabled is True
    assert node.calls == ["trajectory_stop"]
    assert report["failure_recovery"]["outcome"] == "return_failed_healthy_enabled_hold"


def test_failed_stop_does_not_start_controlled_return() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(stop_exception=RuntimeError("arm feedback stale: age=0.163s"))
    report = {"services": [], "motion_legs": []}

    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.full(6, 0.1),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
    )

    assert still_enabled is True
    assert node.calls == ["trajectory_stop"]
    assert report["motion_legs"] == []
    assert report["failure_recovery"]["outcome"] == (
        "stop_failed_healthy_enabled_hold_requires_operator_recovery"
    )


def test_critical_status_is_only_automatic_disable_exception() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(critical=True)
    report = {"services": [], "motion_legs": []}
    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.zeros(6),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
    )
    assert still_enabled is False
    assert node.calls == ["trajectory_stop", "disable"]
    assert report["failure_recovery"]["outcome"] == "critical_status_protective_disable"


def test_stale_feedback_during_return_attempts_protective_disable() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(return_exception=RuntimeError("arm feedback stale: age=0.163s"))
    report = {"services": [], "motion_legs": []}

    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.full(6, 0.1),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
    )

    assert still_enabled is False
    assert node.calls == ["trajectory_stop", "disable"]
    assert report["failure_recovery"]["return_failure"] == (
        "RuntimeError: arm feedback stale: age=0.163s"
    )
    assert report["failure_recovery"]["outcome"] == (
        "return_exception_critical_status_protective_disable"
    )


def test_failed_return_with_critical_status_attempts_protective_disable() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(return_success=False, return_critical=True)
    report = {"services": [], "motion_legs": []}

    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.full(6, 0.1),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
    )

    assert still_enabled is False
    assert node.calls == ["trajectory_stop", "disable"]
    assert report["failure_recovery"]["outcome"] == (
        "return_failed_critical_status_protective_disable"
    )


def test_failed_protective_disable_is_recorded_without_false_success() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(
        critical=True,
        disable_exception=RuntimeError("disable verification unavailable"),
    )
    report = {"services": [], "motion_legs": []}

    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.zeros(6),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
        verification_stable_sec=0.0,
    )

    assert still_enabled is True
    assert node.calls == ["trajectory_stop", "disable"]
    assert report["failure_recovery"]["disable_failure"] == (
        "RuntimeError: disable verification unavailable"
    )
    assert report["failure_recovery"]["outcome"] == (
        "critical_status_protective_disable_failed"
    )


def test_paired_runner_recognizes_only_healthy_enabled_hold() -> None:
    from rebotarm_motion.real_failure_recovery import healthy_enabled_hold

    assert healthy_enabled_hold(_Status()) is True
    assert healthy_enabled_hold(_Status(enabled=False, control_loop_active=False)) is False
    assert healthy_enabled_hold(_Status(error_codes=["fault"])) is False
    assert healthy_enabled_hold(_Status(per_joint_status_code=[1, 1, 1, 1, 1, 2])) is False


def test_baseline_must_be_position_and_velocity_stable_before_disable() -> None:
    from rebotarm_motion.real_failure_recovery import verify_recovery_baseline

    node = _Node()
    node.positions[:] = 0.1
    node.velocities[4] = 0.06

    result = verify_recovery_baseline(
        node=node,
        baseline=np.full(6, 0.1),
        stable_sec=0.0,
        timeout_sec=0.0,
    )

    assert result["verified"] is False
    assert result["critical"] is False
    assert result["reason"] == "baseline_not_stable_before_timeout"
    assert result["worst_position_error_rad"] == pytest.approx(0.0)
    assert result["worst_velocity_rad_s"] == pytest.approx(0.06)


def test_event_driven_arm_status_age_is_not_treated_as_feedback_staleness() -> None:
    from rebotarm_motion.real_failure_recovery import verify_recovery_baseline

    node = _Node()
    node.last_status_monotonic = time.monotonic() - 10.0

    result = verify_recovery_baseline(
        node=node,
        baseline=np.zeros(6),
        stable_sec=0.0,
        timeout_sec=0.0,
    )

    assert result["verified"] is True
    assert result["reason"] == "baseline_stable"


def test_stale_feedback_is_critical_during_baseline_verification() -> None:
    from rebotarm_motion.real_failure_recovery import verify_recovery_baseline

    node = _Node(refresh_feedback=False)
    node.last_joint_monotonic = time.monotonic() - 1.0

    result = verify_recovery_baseline(
        node=node,
        baseline=np.zeros(6),
        stable_sec=0.0,
        timeout_sec=0.0,
    )

    assert result["verified"] is False
    assert result["critical"] is True
    assert result["reason"] == "joint_feedback_stale"


def test_recovery_keeps_enabled_hold_when_baseline_velocity_is_not_stable() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node()
    node.velocities[2] = 0.06
    report = {"services": [], "motion_legs": []}

    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.full(6, 0.1),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
        verification_stable_sec=0.0,
        recovery_timeout_sec=0.0,
    )

    assert still_enabled is True
    assert node.calls == ["trajectory_stop"]
    assert report["failure_recovery"]["outcome"] == (
        "return_not_stable_healthy_enabled_hold"
    )


def test_recovery_attempts_protective_disable_for_stale_feedback() -> None:
    from rebotarm_motion.real_failure_recovery import recover_real_failure

    node = _Node(refresh_feedback=False)
    node.last_joint_monotonic = time.monotonic() - 1.0
    report = {"services": [], "motion_legs": []}

    still_enabled = recover_real_failure(
        node=node,
        report=report,
        baseline=np.zeros(6),
        allow_controlled_return=True,
        legs_key="motion_legs",
        command_label="test_return",
        verification_stable_sec=0.0,
    )

    assert still_enabled is False
    assert node.calls == ["trajectory_stop", "disable"]
    assert report["failure_recovery"]["outcome"] == (
        "joint_feedback_stale_protective_disable"
    )

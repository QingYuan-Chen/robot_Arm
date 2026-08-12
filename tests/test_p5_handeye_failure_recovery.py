from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class _Status:
    enabled: bool = True
    control_loop_active: bool = True
    per_joint_status_code: list[int] = field(default_factory=lambda: [1] * 6)
    error_codes: list[str] = field(default_factory=list)


class _Node:
    stop_client = "stop"
    disable_client = "disable"

    def __init__(self, *, return_success: bool = True, critical: bool = False) -> None:
        self.latest_status = _Status()
        if critical:
            self.latest_status.error_codes = ["motor_fault"]
        self.return_success = return_success
        self.calls: list[str] = []
        self.positions = np.zeros(6, dtype=np.float64)

    def call_trigger(self, client, label: str):
        self.calls.append(label)
        if label == "disable":
            self.latest_status.enabled = False
            self.latest_status.control_loop_active = False
        return {"success": True, "message": label}

    def hold_and_collect(self, _seconds: float) -> None:
        return None

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
        if self.return_success:
            self.positions = np.asarray(command["points"][-1]["positions"], dtype=np.float64)
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


def test_paired_runner_recognizes_only_healthy_enabled_hold() -> None:
    from rebotarm_motion.real_failure_recovery import healthy_enabled_hold

    assert healthy_enabled_hold(_Status()) is True
    assert healthy_enabled_hold(_Status(enabled=False, control_loop_active=False)) is False
    assert healthy_enabled_hold(_Status(error_codes=["fault"])) is False
    assert healthy_enabled_hold(_Status(per_joint_status_code=[1, 1, 1, 1, 1, 2])) is False

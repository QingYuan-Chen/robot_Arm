from __future__ import annotations

import json
from pathlib import Path

import pytest

from rebot_b601_mapping.follower_controller import (
    EnableHoldResult,
    FollowerArmState,
    FollowerCommunicationError,
    FollowerLifecycleError,
    HoldResult,
)
from rebot_b601_mapping.live_follow import StopRequest, run_live_follow
from rebot_b601_mapping.models import LeaderSample
from rebot_b601_mapping.safety_supervisor import FollowState


ROOT = Path(__file__).parents[1]
MAPPING = ROOT / "mapping.example.json"
LIVE = ROOT / "live_follow.example.json"
SAFE_HOME = (
    -1.549363136291504,
    0.01659393310546875,
    -0.02002716064453125,
    -0.00858306884765625,
    0.10395240783691406,
    0.00133514404296875,
)
WEB_LIMITS = (
    (-2.8, 2.8),
    (-3.14, 0.02),
    (-3.14, 0.0),
    (-1.87, 1.57),
    (-1.57, 1.57),
    (-3.14, 3.14),
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0
        self.sleeps: list[float] = []

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        duration = max(0.0, float(seconds))
        self.sleeps.append(duration)
        self.now += duration


class FakeSampler:
    def __init__(
        self,
        clock: FakeClock,
        events: list[str],
        *,
        angles=(0.0,) * 7,
        stale=False,
        fail_after_baseline=False,
        fail_capture=False,
    ) -> None:
        self.clock = clock
        self.events = events
        self.angles = tuple(angles)
        self.stale = stale
        self.fail_after_baseline = fail_after_baseline
        self.fail_capture = fail_capture
        self.baseline_captured = False
        self.opened = False
        self.closed = False

    def open(self) -> None:
        self.opened = True
        self.events.append("leader:open")

    def latest(self) -> LeaderSample:
        if self.fail_after_baseline and self.baseline_captured:
            raise RuntimeError("leader sampler stopped")
        timestamp = (
            self.clock() - 1.0
            if self.stale and self.baseline_captured
            else self.clock()
        )
        angles = self.angles if self.baseline_captured else (0.0,) * 7
        return LeaderSample(timestamp_s=timestamp, angles_deg=angles)

    def capture_stable_baseline(self, sample_count: int) -> tuple[float, ...]:
        assert sample_count == 5
        self.events.append("leader:capture-qL0")
        if self.fail_capture:
            raise RuntimeError("baseline unavailable")
        self.baseline_captured = True
        return (0.0,) * 7

    def close(self) -> None:
        self.closed = True
        self.events.append("leader:close")


class FakeFollower:
    def __init__(
        self,
        clock: FakeClock,
        events: list[str],
        *,
        start=SAFE_HOME,
        fail_cycle=False,
        fail_disable=False,
        freeze_after_hold=False,
        fail_recovery_cycle=False,
        cycle_cost_s=0.0,
        feedback_after_enable=None,
        follow_feedback=None,
        feedback_after_first_hold=None,
        return_feedback=None,
        fail_configuration_check=False,
    ) -> None:
        self.clock = clock
        self.events = events
        self.positions = tuple(start)
        self.fail_cycle = fail_cycle
        self.fail_disable = fail_disable
        self.freeze_after_hold = freeze_after_hold
        self.fail_recovery_cycle = fail_recovery_cycle
        self.cycle_cost_s = float(cycle_cost_s)
        self.feedback_after_enable = feedback_after_enable
        self.follow_feedback = follow_feedback
        self.feedback_after_first_hold = feedback_after_first_hold
        self.return_feedback = return_feedback
        self.fail_configuration_check = fail_configuration_check
        self.holding = False
        self.in_hold_command = False
        self.hold_calls = 0
        self.sent_targets: list[tuple[float, ...]] = []
        self.enabled = False
        self.closed = False

    def _state(self, status: int | None = None) -> FollowerArmState:
        actual_status = (1 if self.enabled else 0) if status is None else status
        return FollowerArmState(
            timestamp_s=self.clock(),
            positions_rad=self.positions,
            velocities_rad_s=(0.0,) * 6,
            torques_nm=(0.0,) * 6,
            status_codes=(actual_status,) * 6,
            gripper_position_rad=-1.0,
            gripper_velocity_rad_s=0.0,
            gripper_torque_nm=0.0,
            gripper_status_code=0,
        )

    def open(self) -> None:
        self.events.append("follower:open")

    def read_state(self, expected_arm_status=None, expected_gripper_status=0):
        self.events.append("follower:read-disabled" if not self.enabled else "follower:read-enabled")
        state = self._state()
        if expected_arm_status is not None and state.status_codes != (expected_arm_status,) * 6:
            raise FollowerLifecycleError("status mismatch")
        return state

    def verify_pos_vel_configuration(self) -> None:
        self.events.append("follower:verify-pos-vel-read-only")
        if self.fail_configuration_check:
            raise FollowerLifecycleError("joint3 RID 10 模式不符")

    def enable_hold(self, speed_rad_s: float) -> EnableHoldResult:
        self.enabled = True
        self.events.append("follower:enable")
        self.events.append("follower:cycle-qf0")
        command = tuple(self.positions)
        self.sent_targets.append(command)
        if self.feedback_after_enable is not None:
            self.positions = tuple(self.feedback_after_enable)
        return EnableHoldResult(state=self._state(), command_rad=command)

    def cycle(self, target_rad, speed_rad_s: float) -> FollowerArmState:
        if self.fail_cycle or (self.fail_recovery_cycle and self.hold_calls >= 2):
            raise FollowerCommunicationError("bus unavailable")
        target = tuple(float(value) for value in target_rad)
        for index, (value, (lower, upper)) in enumerate(
            zip(target, WEB_LIMITS, strict=True),
            start=1,
        ):
            if value < lower or value > upper:
                raise ValueError(f"joint{index} 目标超出网页关节边界")
        self.sent_targets.append(target)
        if self.in_hold_command:
            self.events.append("hold-command")
        else:
            self.events.append("return" if self.holding else "cycle")
        if not self.holding and self.follow_feedback is not None:
            self.positions = tuple(self.follow_feedback)
        elif (
            self.holding
            and not self.in_hold_command
            and self.return_feedback is not None
        ):
            self.positions = tuple(self.return_feedback)
        elif not (self.holding and self.freeze_after_hold):
            self.positions = target
        self.clock.now += self.cycle_cost_s
        return self._state()

    def hold_current(
        self,
        speed_rad_s: float,
        *,
        fallback_target_rad=None,
    ) -> HoldResult:
        self.holding = True
        self.hold_calls += 1
        self.events.append("hold")
        if self.hold_calls == 1 and self.feedback_after_first_hold is not None:
            self.positions = tuple(self.feedback_after_first_hold)
        feedback = tuple(self.positions)
        used_fallback = any(
            value < lower or value > upper
            for value, (lower, upper) in zip(feedback, WEB_LIMITS, strict=True)
        )
        command = (
            tuple(float(value) for value in fallback_target_rad)
            if used_fallback and fallback_target_rad is not None
            else feedback
        )
        self.in_hold_command = True
        try:
            state = self.cycle(command, speed_rad_s)
        finally:
            self.in_hold_command = False
        if used_fallback:
            self.positions = feedback
            state = self._state()
        return HoldResult(
            state=state,
            command_rad=command,
            used_fallback=used_fallback,
        )

    def disable_verified(self) -> FollowerArmState:
        self.events.append("protective-disable" if self.holding else "normal-disable")
        if self.fail_disable:
            raise FollowerLifecycleError("disable unreachable")
        self.enabled = False
        return self._state(status=0)

    def close(self) -> None:
        self.closed = True
        self.events.append("close")


def run_fake(
    tmp_path: Path,
    *,
    confirmed=True,
    angles=(0.0,) * 7,
    stale=False,
    fail_leader_after_baseline=False,
    fail_leader_baseline=False,
    start=SAFE_HOME,
    fail_cycle=False,
    fail_disable=False,
    freeze_after_hold=False,
    fail_recovery_cycle=False,
    cycle_cost_s=0.0,
    feedback_after_enable=None,
    follow_feedback=None,
    feedback_after_first_hold=None,
    return_feedback=None,
    max_cycles=2,
    recovery_decisions=(),
):
    clock = FakeClock()
    events: list[str] = []
    sampler = FakeSampler(
        clock,
        events,
        angles=angles,
        stale=stale,
        fail_after_baseline=fail_leader_after_baseline,
        fail_capture=fail_leader_baseline,
    )
    follower = FakeFollower(
        clock,
        events,
        start=start,
        fail_cycle=fail_cycle,
        fail_disable=fail_disable,
        freeze_after_hold=freeze_after_hold,
        fail_recovery_cycle=fail_recovery_cycle,
        cycle_cost_s=cycle_cost_s,
        feedback_after_enable=feedback_after_enable,
        follow_feedback=follow_feedback,
        feedback_after_first_hold=feedback_after_first_hold,
        return_feedback=return_feedback,
    )
    decisions = iter(recovery_decisions)
    summary = run_live_follow(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyACM0",
        mapping_path=MAPPING,
        live_config_path=LIVE,
        log_path=tmp_path / "follow.jsonl",
        confirmed=confirmed,
        speed_rad_s=0.5,
        max_cycles=max_cycles,
        port_checker=lambda paths: events.append("ports:checked"),
        clock=clock,
        sleep=clock.sleep,
        sampler_factory=lambda port: sampler,
        follower_factory=lambda port, config, joint_limits: follower,
        recovery_decider=lambda timeout_s: next(decisions, None),
    )
    return summary, events, sampler, follower


def test_without_confirmation_only_reads_preflight_and_never_enables(tmp_path: Path) -> None:
    summary, events, sampler, follower = run_fake(tmp_path, confirmed=False)

    assert "follower:read-disabled" in events
    assert "follower:verify-pos-vel-read-only" not in events
    assert "follower:enable" not in events
    assert summary.final_state is FollowState.DISCONNECTED
    assert sampler.closed is True
    assert follower.closed is True


def test_startup_holds_qf0_then_captures_fresh_leader_baseline(tmp_path: Path) -> None:
    summary, events, _, follower = run_fake(tmp_path)

    assert events.index("follower:read-disabled") < events.index(
        "follower:verify-pos-vel-read-only"
    ) < events.index("follower:enable")
    assert events.index("follower:cycle-qf0") < events.index("leader:capture-qL0")
    assert follower.sent_targets[0] == pytest.approx(SAFE_HOME)
    assert summary.cycles == 2


def test_configuration_mismatch_stops_before_enable_or_position_command(
    tmp_path: Path,
) -> None:
    clock = FakeClock()
    events: list[str] = []
    sampler = FakeSampler(clock, events)
    follower = FakeFollower(clock, events, fail_configuration_check=True)

    with pytest.raises(
        FollowerLifecycleError,
        match=r"joint3 RID 10 模式不符",
    ):
        run_live_follow(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyACM0",
            mapping_path=MAPPING,
            live_config_path=LIVE,
            log_path=tmp_path / "configuration-mismatch.jsonl",
            confirmed=True,
            speed_rad_s=0.5,
            max_cycles=0,
            port_checker=lambda paths: events.append("ports:checked"),
            clock=clock,
            sleep=clock.sleep,
            sampler_factory=lambda port: sampler,
            follower_factory=lambda port, config, joint_limits: follower,
        )

    assert "follower:verify-pos-vel-read-only" in events
    assert "follower:enable" not in events
    assert follower.sent_targets == []
    assert follower.closed is True


def test_absolute_deadline_scheduler_keeps_successive_cycles_at_50_hz(tmp_path: Path) -> None:
    summary, _, _, _ = run_fake(tmp_path, max_cycles=3)
    rows = [
        json.loads(line)
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    periods = [row["period_s"] for row in rows if row["event"] == "cycle"]

    assert periods == pytest.approx([0.02, 0.02, 0.02])


def test_confirmed_mapping_signs_drive_all_six_axes(tmp_path: Path) -> None:
    start = (-1.5, -1.0, -1.0, 0.2, 0.0, 0.0)
    summary, _, _, follower = run_fake(
        tmp_path,
        angles=(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0),
        start=start,
        max_cycles=1,
    )

    command = follower.sent_targets[1]
    delta = 20.0 * 0.02 * 0.02
    assert command[0] < start[0]
    assert command[1] < start[1]
    assert command[2] > start[2]
    assert command[3] > start[3]
    assert command[4] > start[4]
    assert command[5] < start[5]
    assert max(abs(a - b) for a, b in zip(command, start)) <= delta + 1e-9
    assert summary.cycles == 1


def test_follow_accepts_small_targets_inside_web_joint_limits(tmp_path: Path) -> None:
    summary, _, _, follower = run_fake(
        tmp_path,
        angles=(0.0, -0.1, 0.1, 0.0, 0.0, 0.0, 0.0),
        start=SAFE_HOME,
        max_cycles=1,
    )

    assert summary.cycles == 1
    command = follower.sent_targets[1]
    assert command[1] > SAFE_HOME[1]
    assert command[2] > SAFE_HOME[2]


def test_normal_stop_holds_returns_home_verifies_then_disables(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(tmp_path)

    assert events.index("hold") < events.index("return") < events.index("protective-disable") < events.index("close")
    assert summary.safe_home_verified is True
    assert summary.disable_verified is True


def test_safe_home_return_starts_from_confirmed_hold_without_command_jump(
    tmp_path: Path,
) -> None:
    recorded_start = (
        -1.6096363067626953,
        -0.00667572021484375,
        -0.061226844787597656,
        0.11844825744628906,
        0.12378883361816406,
        0.10547828674316406,
    )
    _, _, _, follower = run_fake(
        tmp_path,
        start=recorded_start,
        max_cycles=0,
    )

    assert follower.sent_targets[0] == pytest.approx(recorded_start)
    assert follower.sent_targets[1] == pytest.approx(recorded_start)


def test_safe_home_return_matches_web_teleop_target_from_recorded_start(
    tmp_path: Path,
) -> None:
    recorded_start = (
        -1.567673683166504,
        -0.00667572021484375,
        -0.01735687255859375,
        0.00209808349609375,
        0.10967445373535156,
        -0.02346038818359375,
    )

    summary, _, _, follower = run_fake(
        tmp_path,
        start=recorded_start,
        max_cycles=0,
    )

    assert summary.safe_home_verified is True
    assert follower.positions == pytest.approx(SAFE_HOME)


def test_safe_home_return_rejects_out_of_limit_start_feedback(
    tmp_path: Path,
) -> None:
    drifted = list(SAFE_HOME)
    drifted[1] = 0.03

    summary, _, _, follower = run_fake(
        tmp_path,
        max_cycles=0,
        feedback_after_first_hold=tuple(drifted),
        recovery_decisions=("emergency_stop",),
    )

    rows = [
        json.loads(line)
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    events = [row["event"] for row in rows]
    assert "safe-home-start-out-of-limits" in events
    assert "safe-home-cycle" not in events
    assert summary.safe_home_verified is False
    assert all(
        lower <= value <= upper
        for target in follower.sent_targets
        for value, (lower, upper) in zip(target, WEB_LIMITS, strict=True)
    )


def test_out_of_limit_feedback_after_enable_uses_last_safe_command(
    tmp_path: Path,
) -> None:
    drifted = list(SAFE_HOME)
    drifted[1] = 0.03

    summary, events, _, follower = run_fake(
        tmp_path,
        max_cycles=None,
        feedback_after_enable=tuple(drifted),
        recovery_decisions=("emergency_stop",),
    )

    rows = [
        json.loads(line)
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    log_events = [row["event"] for row in rows]
    assert "enabled-feedback-out-of-limits" in log_events
    assert "hold-used-last-safe-command" in log_events
    assert "safe-home-start-out-of-limits" in log_events
    recoverable = next(row for row in rows if row["event"] == "recoverable-hold")
    assert "使能后反馈 joint2" in recoverable["reason"]
    assert "normal-disable" not in events
    assert summary.safe_home_verified is False
    assert all(
        lower <= value <= upper
        for target in follower.sent_targets
        for value, (lower, upper) in zip(target, WEB_LIMITS, strict=True)
    )


def test_follow_feedback_outside_web_limits_uses_last_safe_command(
    tmp_path: Path,
) -> None:
    drifted = list(SAFE_HOME)
    drifted[1] = 0.03

    summary, events, _, follower = run_fake(
        tmp_path,
        max_cycles=1,
        follow_feedback=tuple(drifted),
        recovery_decisions=("emergency_stop",),
    )

    rows = [
        json.loads(line)
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    log_events = [row["event"] for row in rows]
    assert "follow-feedback-out-of-limits" in log_events
    assert "hold-used-last-safe-command" in log_events
    assert "safe-home-start-out-of-limits" in log_events
    assert "normal-disable" not in events
    assert summary.cycles == 1
    assert summary.safe_home_verified is False
    assert all(
        lower <= value <= upper
        for target in follower.sent_targets
        for value, (lower, upper) in zip(target, WEB_LIMITS, strict=True)
    )


def test_safe_home_feedback_outside_web_limits_is_never_verified(
    tmp_path: Path,
) -> None:
    out_of_limit_feedback = list(SAFE_HOME)
    out_of_limit_feedback[1] = 0.03

    summary, events, _, follower = run_fake(
        tmp_path,
        max_cycles=0,
        return_feedback=tuple(out_of_limit_feedback),
        recovery_decisions=("emergency_stop",),
    )

    rows = [
        json.loads(line)
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    log_events = [row["event"] for row in rows]
    assert "safe-home-feedback-out-of-limits" in log_events
    assert "safe-home-verified" not in log_events
    assert "normal-disable" not in events
    assert summary.safe_home_verified is False
    assert all(
        lower <= value <= upper
        for target in follower.sent_targets
        for value, (lower, upper) in zip(target, WEB_LIMITS, strict=True)
    )


def test_safe_home_scheduler_does_not_add_cycle_cost_to_period(tmp_path: Path) -> None:
    start = (-1.5, -0.05, -0.1, 0.1, 0.05, 0.05)
    _, _, _, follower = run_fake(
        tmp_path,
        start=start,
        max_cycles=0,
        cycle_cost_s=0.005,
    )

    assert follower.clock.sleeps[0] == pytest.approx(0.015)


def test_leader_timeout_uses_same_guarded_return_path(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(tmp_path, stale=True, max_cycles=None)

    assert events.index("hold") < events.index("return") < events.index("protective-disable")
    assert summary.safe_home_verified is True


def test_leader_sampler_error_uses_guarded_return_instead_of_escaping(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(
        tmp_path,
        fail_leader_after_baseline=True,
        max_cycles=None,
    )

    assert events.index("hold") < events.index("return") < events.index("protective-disable")
    assert summary.safe_home_verified is True


def test_leader_baseline_failure_after_enable_uses_guarded_return(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(
        tmp_path,
        fail_leader_baseline=True,
        max_cycles=None,
    )

    assert events.index("follower:cycle-qf0") < events.index("hold")
    assert events.index("hold") < events.index("return") < events.index("protective-disable")
    assert summary.safe_home_verified is True


def test_return_failure_while_healthy_keeps_enabled_until_operator_emergency(tmp_path: Path) -> None:
    start = (-1.0, -1.0, -1.0, 0.0, 0.0, 0.0)
    summary, events, _, _ = run_fake(
        tmp_path,
        start=start,
        freeze_after_hold=True,
        recovery_decisions=("emergency_stop",),
    )

    log_events = [
        json.loads(line)["event"]
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    assert "operator-recovery-enabled-hold" in log_events
    assert log_events.index("operator-recovery-enabled-hold") < log_events.index("critical-stop")
    assert events.index("hold") < events.index("protective-disable")
    assert "normal-disable" not in events
    assert summary.safe_home_verified is False


def test_operator_recovery_feedback_failure_becomes_critical_stop(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(
        tmp_path,
        start=(-1.0, -1.0, -1.0, 0.0, 0.0, 0.0),
        freeze_after_hold=True,
        fail_recovery_cycle=True,
    )

    assert summary.final_state is FollowState.CRITICAL_STOP
    assert summary.disable_verified is True
    assert "protective-disable" in events
    assert "close" in events


def test_unreachable_follower_reports_unknown_disable_result(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(
        tmp_path,
        fail_cycle=True,
        fail_disable=True,
        max_cycles=None,
    )

    assert summary.final_state is FollowState.CRITICAL_STOP
    assert summary.disable_result_known is False
    assert summary.disable_verified is False
    assert "close" in events


def test_safe_home_disable_verification_failure_is_reported_unknown(tmp_path: Path) -> None:
    summary, events, _, _ = run_fake(tmp_path, fail_disable=True)

    assert summary.final_state is FollowState.CRITICAL_STOP
    assert summary.safe_home_verified is True
    assert summary.disable_verified is False
    assert summary.disable_result_known is False
    assert "close" in events


def test_jsonl_cycle_records_are_complete_and_parseable(tmp_path: Path) -> None:
    summary, _, _, _ = run_fake(tmp_path, max_cycles=1)

    rows = [
        json.loads(line)
        for line in summary.log_path.read_text(encoding="utf-8").splitlines()
    ]
    cycle = next(row for row in rows if row["event"] == "cycle")
    assert set(
        (
            "state",
            "period_s",
            "leader_angles_deg",
            "leader_delta_rad",
            "raw_target_rad",
            "command_rad",
            "feedback_rad",
            "feedback_velocity_rad_s",
            "tracking_error_rad",
            "leader_lag_rad",
            "status_codes",
            "follower_age_s",
        )
    ).issubset(cycle)
    assert rows[-1]["event"] == "run-summary"


def test_out_of_limit_preflight_never_enables_follower(tmp_path: Path) -> None:
    clock = FakeClock()
    events: list[str] = []
    sampler = FakeSampler(clock, events)
    follower = FakeFollower(
        clock,
        events,
        start=(0.0, 0.021, -1.0, 0.0, 0.0, 0.0),
    )

    with pytest.raises(ValueError, match="真实关节限位"):
        run_live_follow(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyACM0",
            mapping_path=MAPPING,
            live_config_path=LIVE,
            log_path=tmp_path / "preflight-invalid.jsonl",
            confirmed=True,
            port_checker=lambda paths: None,
            clock=clock,
            sleep=clock.sleep,
            sampler_factory=lambda port: sampler,
            follower_factory=lambda port, config, joint_limits: follower,
        )

    assert "follower:enable" not in events
    assert sampler.closed is True
    assert follower.closed is True

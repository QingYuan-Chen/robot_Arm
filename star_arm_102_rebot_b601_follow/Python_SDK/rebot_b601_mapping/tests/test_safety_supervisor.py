from __future__ import annotations

import pytest

from rebot_b601_mapping.safety_supervisor import (
    FaultClass,
    FollowState,
    RuntimeGuard,
    RuntimeObservation,
    SafetyAction,
    SafetyEvent,
    SafetySupervisor,
)


def observation(**overrides) -> RuntimeObservation:
    values = {
        "now_s": 10.0,
        "leader_age_s": 0.01,
        "follower_age_s": 0.01,
        "tracking_error_rad": (0.01,) * 6,
        "status_codes": (1,) * 6,
        "deadline_missed": False,
        "command_write_ok": True,
        "port_identity_ok": True,
    }
    values.update(overrides)
    return RuntimeObservation(**values)


def make_guard() -> RuntimeGuard:
    return RuntimeGuard(
        max_tracking_error_rad=0.25,
        tracking_error_grace_s=0.30,
        leader_stale_timeout_s=0.5,
        follower_stale_timeout_s=0.25,
        deadline_miss_limit=3,
    )


def test_recoverable_fault_never_disables_before_verified_safe_home() -> None:
    supervisor = SafetySupervisor(FollowState.FOLLOWING)

    first = supervisor.transition(SafetyEvent.RECOVERABLE_FAULT, "引导臂反馈超时")
    second = supervisor.transition(SafetyEvent.HOLD_CONFIRMED)

    assert first.state is FollowState.RECOVERABLE_HOLD
    assert first.actions == (SafetyAction.START_HOLD,)
    assert second.state is FollowState.RETURNING_SAFE_HOME
    assert second.actions == (SafetyAction.START_RETURN,)
    assert SafetyAction.REQUEST_PROTECTIVE_DISABLE not in first.actions + second.actions


def test_recoverable_fault_during_enabled_hold_still_starts_guarded_hold() -> None:
    supervisor = SafetySupervisor(FollowState.ENABLED_HOLD)

    transition = supervisor.transition(
        SafetyEvent.RECOVERABLE_FAULT,
        "引导臂基线采集失败",
    )

    assert transition.state is FollowState.RECOVERABLE_HOLD
    assert transition.actions == (SafetyAction.START_HOLD,)


def test_verified_safe_home_allows_disable_then_close() -> None:
    supervisor = SafetySupervisor(FollowState.RETURNING_SAFE_HOME)

    reached = supervisor.transition(SafetyEvent.SAFE_HOME_REACHED)
    disabled = supervisor.transition(SafetyEvent.DISABLE_OK)

    assert reached.state is FollowState.DISABLING
    assert reached.actions == (SafetyAction.REQUEST_PROTECTIVE_DISABLE,)
    assert disabled.state is FollowState.DISCONNECTED
    assert disabled.actions == (SafetyAction.CLOSE_HANDLES,)


def test_failed_return_while_healthy_requires_enabled_operator_recovery() -> None:
    supervisor = SafetySupervisor(FollowState.RETURNING_SAFE_HOME)

    transition = supervisor.transition(
        SafetyEvent.SAFE_HOME_FAILED_HEALTHY,
        "回安全位超时",
    )

    assert transition.state is FollowState.OPERATOR_RECOVERY
    assert transition.actions == (SafetyAction.KEEP_ENABLED_HOLD,)


def test_operator_retry_returns_to_safe_home_without_disabling() -> None:
    supervisor = SafetySupervisor(FollowState.OPERATOR_RECOVERY)

    transition = supervisor.transition(SafetyEvent.RETRY_RETURN)

    assert transition.state is FollowState.RETURNING_SAFE_HOME
    assert transition.actions == (SafetyAction.START_RETURN,)


def test_only_critical_fault_requests_disable_away_from_home() -> None:
    supervisor = SafetySupervisor(FollowState.FOLLOWING)

    transition = supervisor.transition(SafetyEvent.CRITICAL_FAULT, "从臂反馈超时")

    assert transition.state is FollowState.CRITICAL_STOP
    assert transition.actions == (
        SafetyAction.START_HOLD,
        SafetyAction.REQUEST_PROTECTIVE_DISABLE,
    )


def test_illegal_state_transition_is_never_silently_ignored() -> None:
    supervisor = SafetySupervisor(FollowState.DISCONNECTED)

    with pytest.raises(RuntimeError, match="非法安全状态转换"):
        supervisor.transition(SafetyEvent.FOLLOW_START)


def test_tracking_error_must_remain_high_for_full_grace_period() -> None:
    guard = make_guard()

    assert guard.observe(observation(now_s=1.0, tracking_error_rad=(0.3,) * 6)) is None
    assert guard.observe(observation(now_s=1.29, tracking_error_rad=(0.3,) * 6)) is None
    fault = guard.observe(observation(now_s=1.30, tracking_error_rad=(0.3,) * 6))

    assert fault is not None
    assert fault.fault_class is FaultClass.RECOVERABLE
    assert "跟踪误差" in fault.reason


def test_tracking_error_grace_resets_after_a_healthy_sample() -> None:
    guard = make_guard()

    assert guard.observe(observation(now_s=1.0, tracking_error_rad=(0.3,) * 6)) is None
    assert guard.observe(observation(now_s=1.2)) is None
    assert guard.observe(observation(now_s=1.4, tracking_error_rad=(0.3,) * 6)) is None
    assert guard.observe(observation(now_s=1.61, tracking_error_rad=(0.3,) * 6)) is None


def test_three_consecutive_deadline_misses_are_recoverable() -> None:
    guard = make_guard()

    assert guard.observe(observation(deadline_missed=True)) is None
    assert guard.observe(observation(deadline_missed=True)) is None
    fault = guard.observe(observation(deadline_missed=True))

    assert fault is not None
    assert fault.fault_class is FaultClass.RECOVERABLE
    assert "截止时间" in fault.reason


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"follower_age_s": 0.251}, "从臂反馈"),
        ({"status_codes": (1, 1, 4, 1, 1, 1)}, "status_code"),
        ({"command_write_ok": False}, "命令写入"),
        ({"port_identity_ok": False}, "设备身份"),
    ],
)
def test_follower_integrity_failures_are_immediately_critical(
    changes,
    message: str,
) -> None:
    fault = make_guard().observe(observation(**changes))

    assert fault is not None
    assert fault.fault_class is FaultClass.CRITICAL
    assert message in fault.reason


def test_leader_stale_is_recoverable() -> None:
    fault = make_guard().observe(observation(leader_age_s=0.501))

    assert fault is not None
    assert fault.fault_class is FaultClass.RECOVERABLE
    assert "引导臂反馈" in fault.reason

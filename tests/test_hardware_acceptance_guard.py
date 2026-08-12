from __future__ import annotations

import pytest

from rebotarm_motion.hardware_acceptance_guard import (
    HardwareAcceptanceGuard,
    HardwareAcceptanceGuardConfig,
)


JOINTS = tuple(f"joint{i}" for i in range(1, 7))
EFFORT_LIMITS = {
    **{name: 27.0 for name in JOINTS[:3]},
    **{name: 7.0 for name in JOINTS[3:]},
}


def sample(value: float = 0.0) -> dict[str, float]:
    return {name: float(value) for name in JOINTS}


def observe(
    guard: HardwareAcceptanceGuard,
    now: float,
    *,
    raw: dict[str, float] | None = None,
    window: dict[str, float] | None = None,
    effort: dict[str, float] | None = None,
    tracking: dict[str, float] | None = None,
):
    return guard.observe(
        now=now,
        raw_velocities_by_joint=raw or sample(),
        window_velocities_by_joint=window or sample(),
        efforts_by_joint=effort or sample(),
        tracking_errors_by_joint=tracking or sample(),
    )


def test_single_quantized_raw_spike_warns_without_stopping() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    raw = sample()
    raw["joint4"] = 0.15384674

    decision = observe(guard, 1.0, raw=raw)

    assert not decision.should_stop
    assert decision.warnings == (
        {
            "kind": "raw_velocity",
            "joint": "joint4",
            "value": pytest.approx(0.15384674),
            "limit": 0.07,
        },
    )


def test_raw_sustained_threshold_requires_full_duration() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    raw = sample()
    raw["joint4"] = 0.16

    assert not observe(guard, 1.00, raw=raw).should_stop
    assert not observe(guard, 1.19, raw=raw).should_stop
    decision = observe(guard, 1.20, raw=raw)

    assert decision.should_stop
    assert decision.reason == "raw_velocity_sustained"
    assert decision.joint == "joint4"
    assert decision.duration_sec == pytest.approx(0.20)


def test_raw_sustained_timer_resets_below_threshold() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    high = sample()
    high["joint4"] = 0.16

    assert not observe(guard, 1.0, raw=high).should_stop
    assert not observe(guard, 1.1, raw=sample()).should_stop
    assert not observe(guard, 1.3, raw=high).should_stop


def test_raw_fast_threshold_requires_window_corroboration() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    raw = sample()
    raw["joint4"] = 0.300001

    uncorroborated = observe(guard, 1.0, raw=raw)
    assert not uncorroborated.should_stop

    window = sample()
    window["joint4"] = 0.100001
    decision = observe(guard, 1.01, raw=raw, window=window)

    assert decision.should_stop
    assert decision.reason == "raw_velocity_corroborated_immediate"


def test_raw_absolute_threshold_stops_without_window_corroboration() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    raw = sample()
    raw["joint6"] = 0.400001

    decision = observe(guard, 1.0, raw=raw)

    assert decision.should_stop
    assert decision.reason == "raw_velocity_absolute_immediate"


def test_recorded_joint6_correction_burst_uses_sustained_gates() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    raw = sample()
    window = sample()
    raw["joint6"] = 0.34432220458984375
    window["joint6"] = 0.07771612963400154

    decision = observe(guard, 1.0, raw=raw, window=window)

    assert not decision.should_stop


def test_single_window_boundary_spike_warns_without_stopping() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    window = sample()
    window["joint6"] = 0.05744

    decision = observe(guard, 1.0, window=window)

    assert not decision.should_stop
    assert decision.warnings == (
        {
            "kind": "window_velocity",
            "joint": "joint6",
            "value": pytest.approx(0.05744),
            "limit": 0.05,
        },
    )


def test_window_velocity_has_sustained_and_immediate_gates() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    window = sample()
    window["joint6"] = 0.11
    assert not observe(guard, 1.0, window=window).should_stop
    sustained = observe(guard, 1.2, window=window)
    assert sustained.should_stop
    assert sustained.reason == "window_velocity_sustained"

    guard.reset()
    window["joint6"] = 0.200001
    immediate = observe(guard, 2.0, window=window)
    assert immediate.should_stop
    assert immediate.reason == "window_velocity_immediate"


def test_tracking_error_has_sustained_and_immediate_gates() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    tracking = sample()
    tracking["joint4"] = 0.06
    assert not observe(guard, 1.0, tracking=tracking).should_stop
    sustained = observe(guard, 1.5, tracking=tracking)
    assert sustained.should_stop
    assert sustained.reason == "tracking_error_sustained"

    guard.reset()
    tracking["joint4"] = 0.100001
    immediate = observe(guard, 2.0, tracking=tracking)
    assert immediate.should_stop
    assert immediate.reason == "tracking_error_immediate"


def test_joint4_effort_uses_urdf_limit_and_persistence() -> None:
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    effort = sample()
    effort["joint4"] = 6.0
    assert not observe(guard, 1.0, effort=effort).should_stop
    sustained = observe(guard, 1.5, effort=effort)
    assert sustained.should_stop
    assert sustained.reason == "effort_sustained"
    assert sustained.limit == pytest.approx(5.95)

    guard.reset()
    effort["joint4"] = 7.000001
    immediate = observe(guard, 2.0, effort=effort)
    assert immediate.should_stop
    assert immediate.reason == "effort_immediate"


def test_invalid_config_and_missing_feedback_fail_closed() -> None:
    with pytest.raises(ValueError, match="sustained threshold"):
        HardwareAcceptanceGuardConfig(
            raw_sustained_rad_s=0.31,
            raw_immediate_rad_s=0.30,
        )
    with pytest.raises(ValueError, match="absolute threshold"):
        HardwareAcceptanceGuardConfig(
            raw_immediate_rad_s=0.41,
            raw_absolute_immediate_rad_s=0.40,
        )
    guard = HardwareAcceptanceGuard(EFFORT_LIMITS)
    with pytest.raises(ValueError, match="missing joints"):
        guard.observe(
            now=1.0,
            raw_velocities_by_joint={"joint1": 0.0},
            window_velocities_by_joint=sample(),
            efforts_by_joint=sample(),
            tracking_errors_by_joint=sample(),
        )

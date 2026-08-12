from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarmcontroller.p0_acceptance_core import (
    DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT,
    EXPECTED_JOINTS,
    motion_sample_details,
    quantization_aware_raw_velocity_limits,
    raw_velocity_limit_violations,
    sample_motion_metrics,
    validate_preflight,
)


def make_status():
    return SimpleNamespace(
        mode="mit",
        enabled=False,
        control_loop_active=False,
        state_machine="IDLE",
        joint_names=list(EXPECTED_JOINTS),
        per_joint_status_code=[0] * len(EXPECTED_JOINTS),
        error_codes=[],
    )


def make_joint_state():
    return SimpleNamespace(
        name=list(EXPECTED_JOINTS),
        position=[-1.0, -0.5, -0.4, 0.1, 0.0, 0.2],
        velocity=[0.0] * len(EXPECTED_JOINTS),
        effort=[0.0] * len(EXPECTED_JOINTS),
    )


def test_gate_bc_preflight_accepts_connected_disabled_state() -> None:
    assert validate_preflight(make_status(), make_joint_state()) == []


def test_gate_bc_preflight_rejects_enabled_faulted_or_invalid_state() -> None:
    status = make_status()
    status.enabled = True
    status.control_loop_active = True
    status.per_joint_status_code[2] = 1
    status.error_codes = ["TEST_ERROR"]
    joint_state = make_joint_state()
    joint_state.position[4] = math.nan

    errors = validate_preflight(status, joint_state)

    assert any("already enabled" in error for error in errors)
    assert any("already active" in error for error in errors)
    assert any("expected all zero" in error for error in errors)
    assert any("non-finite" in error for error in errors)
    assert any("TEST_ERROR" in error for error in errors)


def test_gate_bc_motion_metrics_follow_joint_names() -> None:
    baseline = {name: float(index) for index, name in enumerate(EXPECTED_JOINTS)}
    msg = make_joint_state()
    msg.name = list(reversed(EXPECTED_JOINTS))
    msg.position = [baseline[name] for name in msg.name]
    msg.position[0] += 0.02
    msg.velocity = [0.01, -0.04, 0.02, 0.0, 0.0, 0.0]

    max_jump, max_velocity = sample_motion_metrics(baseline, msg)

    assert math.isclose(max_jump, 0.02)
    assert math.isclose(max_velocity, 0.04)
    details = motion_sample_details(baseline, msg)
    assert details["position_jump_joint"] == "joint6"
    assert details["velocity_joint"] == "joint5"
    assert details["abs_velocity_by_joint_rad_s"]["joint5"] == 0.04


def test_quantization_aware_velocity_limits_are_model_specific() -> None:
    limits = quantization_aware_raw_velocity_limits(0.05)

    assert limits["joint1"] == pytest.approx(0.05 + (10.0 / 4095.0))
    assert limits["joint3"] == limits["joint1"]
    assert limits["joint4"] == pytest.approx(0.05 + (30.0 / 4095.0))
    assert limits["joint6"] == limits["joint4"]
    assert DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT == {
        "joint1": 10.0,
        "joint2": 10.0,
        "joint3": 10.0,
        "joint4": 30.0,
        "joint5": 30.0,
        "joint6": 30.0,
    }


def test_quantization_aware_velocity_gate_accepts_boundary_bin_only() -> None:
    boundary_bin = {name: 0.051283 for name in EXPECTED_JOINTS}
    boundary_bin["joint1"] = 0.051282

    assert raw_velocity_limit_violations(boundary_bin, 0.05) == {}

    next_bins = dict(boundary_bin)
    next_bins["joint1"] = 0.056166
    next_bins["joint4"] = 0.065934
    violations = raw_velocity_limit_violations(next_bins, 0.05)

    assert set(violations) == {"joint1", "joint4"}


def test_operator_raw_velocity_floor_does_not_change_physical_window_limit() -> None:
    velocities = {name: 0.065934 for name in EXPECTED_JOINTS}

    assert raw_velocity_limit_violations(
        velocities,
        0.05,
        raw_limit_floor_rad_s=0.07,
    ) == {}

    velocities["joint4"] = 0.070001
    violations = raw_velocity_limit_violations(
        velocities,
        0.05,
        raw_limit_floor_rad_s=0.07,
    )

    assert set(violations) == {"joint4"}
    assert violations["joint4"]["raw_limit_rad_s"] == 0.07


def test_gate_bc_console_entrypoint_is_packaged() -> None:
    root = Path(__file__).resolve().parents[1]
    setup_text = (root / "src/rebotarmcontroller/setup.py").read_text(
        encoding="utf-8"
    )

    assert (
        "p0_gate_bc_acceptance = "
        "rebotarmcontroller.examples.p0_gate_bc_acceptance:main"
    ) in setup_text


def test_gate_bc_requires_confirmation_and_has_fail_safe_cleanup() -> None:
    root = Path(__file__).resolve().parents[1]
    source = (
        root
        / "src/rebotarmcontroller/rebotarmcontroller/examples"
        / "p0_gate_bc_acceptance.py"
    ).read_text(encoding="utf-8")

    assert 'CONFIRMATION_TOKEN = "ENABLE_HOLD_TEST"' in source
    assert source.index("confirmation = input") < source.index(
        "node._enable_client"
    )
    assert "def emergency_cleanup" in source
    assert 'self._disable_client,\n            "disable"' in source
    assert "if enable_requested or unexpectedly_enabled:" in source
    assert "trajectory_stop" in source
    assert "safe_home" not in source
    assert "joint states became stale after enable" in source
    assert "joint states became stale during hold" in source

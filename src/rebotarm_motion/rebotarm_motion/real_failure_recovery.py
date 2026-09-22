from __future__ import annotations

from collections.abc import MutableMapping, Sequence
import math
import time
from typing import Any

from .paired_trajectory_protocol import build_quintic_command


RECOVERY_POSITION_TOLERANCE_RAD = 0.02
RECOVERY_VELOCITY_TOLERANCE_RAD_S = 0.05
RECOVERY_STABLE_SEC = 1.0
RECOVERY_TIMEOUT_SEC = 30.0
RECOVERY_FEEDBACK_MAX_AGE_SEC = 0.5


def healthy_enabled_hold(status: object | None) -> bool:
    return bool(
        status is not None
        and bool(getattr(status, "enabled", False))
        and bool(getattr(status, "control_loop_active", False))
        and list(getattr(status, "per_joint_status_code", ())) == [1] * 6
        and not list(getattr(status, "error_codes", ()))
    )


def _recovery_health(node: Any) -> dict[str, Any]:
    status = getattr(node, "latest_status", None)
    if status is None:
        return {"healthy": False, "critical": True, "reason": "arm_status_unavailable"}
    enabled = bool(getattr(status, "enabled", False))
    active = bool(getattr(status, "control_loop_active", False))
    if not enabled and not active:
        return {
            "healthy": False,
            "critical": False,
            "reason": "controller_already_disabled",
        }
    if not healthy_enabled_hold(status):
        return {
            "healthy": False,
            "critical": True,
            "reason": "controller_not_in_healthy_enabled_hold",
        }
    return {"healthy": True, "critical": False, "reason": "ok"}


def _recovery_feedback(node: Any, *, now: float) -> dict[str, Any]:
    last_feedback = getattr(node, "last_joint_monotonic", None)
    if last_feedback is None:
        return {"valid": False, "critical": True, "reason": "joint_feedback_unavailable"}
    age_sec = max(0.0, now - float(last_feedback))
    if age_sec > RECOVERY_FEEDBACK_MAX_AGE_SEC:
        return {
            "valid": False,
            "critical": True,
            "reason": "joint_feedback_stale",
            "age_sec": age_sec,
        }

    message = getattr(node, "latest_joint_state", None)
    if message is None:
        return {"valid": False, "critical": True, "reason": "joint_feedback_unavailable"}
    names = list(getattr(message, "name", ()))
    positions = list(getattr(message, "position", ()))
    velocities = list(getattr(message, "velocity", ()))
    by_name = {name: index for index, name in enumerate(names)}
    expected_names = tuple(f"joint{index}" for index in range(1, 7))
    if any(name not in by_name for name in expected_names):
        return {"valid": False, "critical": True, "reason": "joint_feedback_incomplete"}
    if any(
        by_name[name] >= len(values)
        for name in expected_names
        for values in (positions, velocities)
    ):
        return {"valid": False, "critical": True, "reason": "joint_feedback_incomplete"}
    canonical_positions = tuple(float(positions[by_name[name]]) for name in expected_names)
    canonical_velocities = tuple(float(velocities[by_name[name]]) for name in expected_names)
    if not all(
        math.isfinite(value)
        for values in (canonical_positions, canonical_velocities)
        for value in values
    ):
        return {"valid": False, "critical": True, "reason": "joint_feedback_non_finite"}
    return {
        "valid": True,
        "critical": False,
        "reason": "ok",
        "age_sec": age_sec,
        "positions": canonical_positions,
        "velocities": canonical_velocities,
    }


def verify_recovery_baseline(
    *,
    node: Any,
    baseline: Sequence[float],
    position_tolerance_rad: float = RECOVERY_POSITION_TOLERANCE_RAD,
    velocity_tolerance_rad_s: float = RECOVERY_VELOCITY_TOLERANCE_RAD_S,
    stable_sec: float = RECOVERY_STABLE_SEC,
    timeout_sec: float = RECOVERY_TIMEOUT_SEC,
) -> dict[str, Any]:
    """Require fresh, healthy, stationary feedback at the recovery baseline."""
    baseline_values = tuple(float(value) for value in baseline)
    if len(baseline_values) != 6 or not all(math.isfinite(value) for value in baseline_values):
        raise ValueError("recovery baseline must contain six finite joint positions")
    position_limit = float(position_tolerance_rad)
    velocity_limit = float(velocity_tolerance_rad_s)
    stable_required = max(0.0, float(stable_sec))
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    stable_since: float | None = None
    latest_result: dict[str, Any] = {
        "verified": False,
        "critical": False,
        "reason": "baseline_not_stable_before_timeout",
    }

    while True:
        node.hold_and_collect(0.05)
        now = time.monotonic()
        health = _recovery_health(node)
        if not health["healthy"]:
            return {
                "verified": False,
                "critical": bool(health["critical"]),
                "reason": str(health["reason"]),
                "status_age_sec": health.get("age_sec"),
            }
        feedback = _recovery_feedback(node, now=now)
        if not feedback["valid"]:
            return {
                "verified": False,
                "critical": bool(feedback["critical"]),
                "reason": str(feedback["reason"]),
                "feedback_age_sec": feedback.get("age_sec"),
            }

        positions = feedback["positions"]
        velocities = feedback["velocities"]
        worst_position = max(
            abs(target - actual)
            for target, actual in zip(baseline_values, positions)
        )
        worst_velocity = max(abs(value) for value in velocities)
        within_limits = worst_position <= position_limit and worst_velocity <= velocity_limit
        if within_limits:
            if stable_since is None:
                stable_since = now
        else:
            stable_since = None
        stable_elapsed = 0.0 if stable_since is None else now - stable_since
        latest_result = {
            "verified": within_limits and stable_elapsed >= stable_required,
            "critical": False,
            "reason": (
                "baseline_stable"
                if within_limits and stable_elapsed >= stable_required
                else "baseline_not_stable_before_timeout"
            ),
            "worst_position_error_rad": worst_position,
            "worst_velocity_rad_s": worst_velocity,
            "stable_sec": stable_elapsed,
            "feedback_age_sec": feedback["age_sec"],
            "positions": list(positions),
            "velocities": list(velocities),
        }
        if latest_result["verified"]:
            return latest_result
        if now >= deadline:
            return latest_result


def _attempt_protective_disable(
    *,
    node: Any,
    recovery: MutableMapping[str, Any],
    services: list[Any],
    outcome: str,
) -> bool:
    """Attempt a critical-condition disable and record an unverifiable failure."""
    recovery["outcome"] = outcome
    try:
        services.append(node.call_trigger(node.disable_client, "disable"))
    except Exception as exc:
        recovery["disable_failure"] = f"{type(exc).__name__}: {exc}"
        recovery["outcome"] = f"{outcome}_failed"
        return True
    return False


def recover_real_failure(
    *,
    node: Any,
    report: MutableMapping[str, Any],
    baseline: Sequence[float],
    allow_controlled_return: bool,
    legs_key: str,
    command_label: str,
    duration_sec: float = 20.0,
    verification_position_tolerance_rad: float = RECOVERY_POSITION_TOLERANCE_RAD,
    verification_velocity_tolerance_rad_s: float = RECOVERY_VELOCITY_TOLERANCE_RAD_S,
    verification_stable_sec: float = RECOVERY_STABLE_SEC,
    recovery_timeout_sec: float = RECOVERY_TIMEOUT_SEC,
) -> bool:
    """Recover a real-arm task failure without dropping healthy holding torque.

    Returns whether the controller remains enabled. Automatic disable away from
    baseline is reserved for critical status where holding torque is not
    trustworthy.
    """
    recovery: dict[str, Any] = {
        "allow_controlled_return": bool(allow_controlled_return),
        "outcome": "started",
    }
    recovery_started = time.monotonic()
    report["failure_recovery"] = recovery
    services = report.setdefault("services", [])
    legs = report.setdefault(legs_key, [])
    stop_failed = False
    try:
        services.append(node.call_trigger(node.stop_client, "trajectory_stop"))
    except Exception as exc:
        stop_failed = True
        recovery["stop_failure"] = f"{type(exc).__name__}: {exc}"
    node.hold_and_collect(0.3)
    status = node.latest_status
    recovery["hold_status"] = node._status_payload()
    health = _recovery_health(node)
    recovery["hold_health"] = health
    if health["reason"] == "controller_already_disabled":
        recovery["outcome"] = "controller_already_disabled"
        return False
    if not health["healthy"]:
        recovery["critical_reason"] = health["reason"]
        outcome = (
            "stop_failure_critical_status_protective_disable"
            if stop_failed
            else "critical_status_protective_disable"
        )
        return _attempt_protective_disable(
            node=node,
            recovery=recovery,
            services=services,
            outcome=outcome,
        )
    feedback = _recovery_feedback(node, now=time.monotonic())
    recovery["hold_feedback"] = {
        key: value
        for key, value in feedback.items()
        if key not in {"positions", "velocities"}
    }
    if not feedback["valid"]:
        return _attempt_protective_disable(
            node=node,
            recovery=recovery,
            services=services,
            outcome=f"{feedback['reason']}_protective_disable",
        )
    if stop_failed:
        recovery["outcome"] = (
            "stop_failed_healthy_enabled_hold_requires_operator_recovery"
        )
        return True
    if not allow_controlled_return:
        recovery["outcome"] = "healthy_enabled_hold_requires_operator_recovery"
        return True

    baseline_values = tuple(float(value) for value in baseline)
    current = tuple(float(value) for value in node.canonical_positions())
    recovery["return_start_positions"] = list(current)
    command = build_quintic_command(
        current,
        baseline_values,
        duration_sec=float(duration_sec),
        cadence_sec=0.05,
        label=command_label,
    )
    try:
        return_leg = node.execute_leg(command)
    except Exception as exc:
        recovery["return_failure"] = f"{type(exc).__name__}: {exc}"
        try:
            node.hold_and_collect(0.3)
        except Exception as status_exc:
            recovery["return_status_collection_failure"] = (
                f"{type(status_exc).__name__}: {status_exc}"
            )
        status = node.latest_status
        recovery["return_status"] = node._status_payload()
        health = _recovery_health(node)
        recovery["return_health"] = health
        if health["reason"] == "controller_already_disabled":
            recovery["outcome"] = "return_exception_controller_already_disabled"
            return False
        if not health["healthy"]:
            recovery["critical_reason"] = health["reason"]
            return _attempt_protective_disable(
                node=node,
                recovery=recovery,
                services=services,
                outcome="return_exception_critical_status_protective_disable",
            )
        recovery["outcome"] = "return_exception_healthy_enabled_hold"
        return True
    return_leg["recovery_leg"] = True
    legs.append(return_leg)
    if not bool(return_leg["success"]):
        recovery["return_result"] = return_leg["result"]
        try:
            node.hold_and_collect(0.3)
        except Exception as status_exc:
            recovery["return_status_collection_failure"] = (
                f"{type(status_exc).__name__}: {status_exc}"
            )
        status = node.latest_status
        recovery["return_status"] = node._status_payload()
        health = _recovery_health(node)
        recovery["return_health"] = health
        if health["reason"] == "controller_already_disabled":
            recovery["outcome"] = "return_failed_controller_already_disabled"
            return False
        if not health["healthy"]:
            recovery["critical_reason"] = health["reason"]
            return _attempt_protective_disable(
                node=node,
                recovery=recovery,
                services=services,
                outcome="return_failed_critical_status_protective_disable",
            )
        recovery["outcome"] = "return_failed_healthy_enabled_hold"
        return True
    remaining_timeout = max(
        0.0,
        float(recovery_timeout_sec) - (time.monotonic() - recovery_started),
    )
    verification = verify_recovery_baseline(
        node=node,
        baseline=baseline_values,
        position_tolerance_rad=verification_position_tolerance_rad,
        velocity_tolerance_rad_s=verification_velocity_tolerance_rad_s,
        stable_sec=verification_stable_sec,
        timeout_sec=remaining_timeout,
    )
    recovery["baseline_verification"] = verification
    if "positions" in verification:
        recovery["enabled_final_positions"] = list(verification["positions"])
        recovery["enabled_final_errors"] = [
            target - actual
            for target, actual in zip(baseline_values, verification["positions"])
        ]
    if not verification["verified"]:
        if verification["critical"]:
            return _attempt_protective_disable(
                node=node,
                recovery=recovery,
                services=services,
                outcome=f"{verification['reason']}_protective_disable",
            )
        recovery["outcome"] = "return_not_stable_healthy_enabled_hold"
        return True
    services.append(node.call_trigger(node.disable_client, "disable"))
    node.hold_and_collect(0.5)
    recovery["final_status"] = node._status_payload()
    recovery["outcome"] = "returned_to_baseline_then_disabled"
    return False

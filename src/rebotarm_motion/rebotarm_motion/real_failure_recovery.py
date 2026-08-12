from __future__ import annotations

from collections.abc import MutableMapping, Sequence
from typing import Any

from .paired_trajectory_protocol import build_quintic_command


def healthy_enabled_hold(status: object | None) -> bool:
    return bool(
        status is not None
        and bool(getattr(status, "enabled", False))
        and bool(getattr(status, "control_loop_active", False))
        and list(getattr(status, "per_joint_status_code", ())) == [1] * 6
        and not list(getattr(status, "error_codes", ()))
    )


def recover_real_failure(
    *,
    node: Any,
    report: MutableMapping[str, Any],
    baseline: Sequence[float],
    allow_controlled_return: bool,
    legs_key: str,
    command_label: str,
    duration_sec: float = 20.0,
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
    report["failure_recovery"] = recovery
    services = report.setdefault("services", [])
    legs = report.setdefault(legs_key, [])
    try:
        services.append(node.call_trigger(node.stop_client, "trajectory_stop"))
    except Exception as exc:
        recovery["stop_failure"] = f"{type(exc).__name__}: {exc}"
    node.hold_and_collect(0.3)
    status = node.latest_status
    recovery["hold_status"] = node._status_payload()
    if status is None:
        recovery["outcome"] = "status_unavailable_leave_state_unchanged"
        return True
    if not bool(status.enabled) or not bool(status.control_loop_active):
        recovery["outcome"] = "controller_not_in_enabled_hold"
        return False
    if not healthy_enabled_hold(status):
        recovery["outcome"] = "critical_status_protective_disable"
        services.append(node.call_trigger(node.disable_client, "disable"))
        return False
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
    return_leg = node.execute_leg(command)
    return_leg["recovery_leg"] = True
    legs.append(return_leg)
    if not bool(return_leg["success"]):
        recovery["outcome"] = "return_failed_healthy_enabled_hold"
        recovery["return_result"] = return_leg["result"]
        return True
    node.hold_and_collect(0.5)
    final = tuple(float(value) for value in node.canonical_positions())
    errors = tuple(target - actual for target, actual in zip(baseline_values, final))
    recovery["enabled_final_positions"] = list(final)
    recovery["enabled_final_errors"] = list(errors)
    if max(abs(value) for value in errors) > 0.02:
        recovery["outcome"] = "return_error_healthy_enabled_hold"
        return True
    services.append(node.call_trigger(node.disable_client, "disable"))
    node.hold_and_collect(0.5)
    recovery["final_status"] = node._status_payload()
    recovery["outcome"] = "returned_to_baseline_then_disabled"
    return False

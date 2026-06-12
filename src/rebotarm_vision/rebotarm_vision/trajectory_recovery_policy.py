from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryConfig:
    auto_retry_enabled: bool = False
    safe_retreat_before_retry: bool = True


@dataclass(frozen=True)
class RecoveryDecision:
    retry: bool
    abort: bool
    request_stop: bool
    request_safe_retreat: bool
    reason: str


def recovery_decision_for_stage(
    stage_name: str,
    *,
    attempt_index: int,
    remaining_attempts: int,
    config: RecoveryConfig,
) -> RecoveryDecision:
    can_retry_stage = stage_name in {
        "move_to_pregrasp",
        "approach_grasp",
        "visual_servo_approach",
    }
    can_retry = bool(config.auto_retry_enabled) and can_retry_stage and int(remaining_attempts) > 0
    if can_retry:
        request_safe_retreat = bool(config.safe_retreat_before_retry) and stage_name != "move_to_pregrasp"
        return RecoveryDecision(
            retry=True,
            abort=False,
            request_stop=True,
            request_safe_retreat=request_safe_retreat,
            reason=f"{stage_name} failed on attempt {int(attempt_index) + 1}; retrying next candidate",
        )
    return RecoveryDecision(
        retry=False,
        abort=True,
        request_stop=True,
        request_safe_retreat=False,
        reason=f"{stage_name} failed; aborting visual grasp",
    )

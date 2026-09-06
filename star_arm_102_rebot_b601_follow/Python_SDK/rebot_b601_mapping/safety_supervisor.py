from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto


class FollowState(Enum):
    DISCONNECTED = auto()
    PRECHECK = auto()
    ENABLED_HOLD = auto()
    FOLLOWING = auto()
    RECOVERABLE_HOLD = auto()
    RETURNING_SAFE_HOME = auto()
    VERIFYING_SAFE_HOME = auto()
    DISABLING = auto()
    OPERATOR_RECOVERY = auto()
    CRITICAL_STOP = auto()


class SafetyEvent(Enum):
    PRECHECK_OK = auto()
    ENABLE_OK = auto()
    FOLLOW_START = auto()
    RECOVERABLE_FAULT = auto()
    HOLD_CONFIRMED = auto()
    SAFE_HOME_REACHED = auto()
    SAFE_HOME_FAILED_HEALTHY = auto()
    RETRY_RETURN = auto()
    CRITICAL_FAULT = auto()
    DISABLE_OK = auto()


class SafetyAction(Enum):
    START_HOLD = auto()
    START_RETURN = auto()
    KEEP_ENABLED_HOLD = auto()
    REQUEST_PROTECTIVE_DISABLE = auto()
    CLOSE_HANDLES = auto()


class FaultClass(Enum):
    RECOVERABLE = auto()
    CRITICAL = auto()


@dataclass(frozen=True)
class Transition:
    state: FollowState
    actions: tuple[SafetyAction, ...]
    reason: str


@dataclass(frozen=True)
class RuntimeObservation:
    now_s: float
    leader_age_s: float
    follower_age_s: float
    tracking_error_rad: tuple[float, ...]
    status_codes: tuple[int, ...]
    deadline_missed: bool
    command_write_ok: bool
    port_identity_ok: bool


@dataclass(frozen=True)
class GuardFault:
    fault_class: FaultClass
    reason: str


_TRANSITIONS = {
    (FollowState.DISCONNECTED, SafetyEvent.PRECHECK_OK): Transition(
        FollowState.PRECHECK, (), ""
    ),
    (FollowState.PRECHECK, SafetyEvent.ENABLE_OK): Transition(
        FollowState.ENABLED_HOLD, (SafetyAction.START_HOLD,), ""
    ),
    (FollowState.ENABLED_HOLD, SafetyEvent.FOLLOW_START): Transition(
        FollowState.FOLLOWING, (), ""
    ),
    (FollowState.ENABLED_HOLD, SafetyEvent.RECOVERABLE_FAULT): Transition(
        FollowState.RECOVERABLE_HOLD, (SafetyAction.START_HOLD,), ""
    ),
    (FollowState.FOLLOWING, SafetyEvent.RECOVERABLE_FAULT): Transition(
        FollowState.RECOVERABLE_HOLD, (SafetyAction.START_HOLD,), ""
    ),
    (FollowState.RECOVERABLE_HOLD, SafetyEvent.HOLD_CONFIRMED): Transition(
        FollowState.RETURNING_SAFE_HOME, (SafetyAction.START_RETURN,), ""
    ),
    (FollowState.RETURNING_SAFE_HOME, SafetyEvent.SAFE_HOME_REACHED): Transition(
        FollowState.DISABLING, (SafetyAction.REQUEST_PROTECTIVE_DISABLE,), ""
    ),
    (
        FollowState.RETURNING_SAFE_HOME,
        SafetyEvent.SAFE_HOME_FAILED_HEALTHY,
    ): Transition(
        FollowState.OPERATOR_RECOVERY,
        (SafetyAction.KEEP_ENABLED_HOLD,),
        "",
    ),
    (FollowState.OPERATOR_RECOVERY, SafetyEvent.RETRY_RETURN): Transition(
        FollowState.RETURNING_SAFE_HOME, (SafetyAction.START_RETURN,), ""
    ),
    (FollowState.DISABLING, SafetyEvent.DISABLE_OK): Transition(
        FollowState.DISCONNECTED, (SafetyAction.CLOSE_HANDLES,), ""
    ),
    (FollowState.CRITICAL_STOP, SafetyEvent.DISABLE_OK): Transition(
        FollowState.DISCONNECTED, (SafetyAction.CLOSE_HANDLES,), ""
    ),
}


class SafetySupervisor:
    def __init__(self, initial_state: FollowState = FollowState.DISCONNECTED) -> None:
        self._state = initial_state

    @property
    def state(self) -> FollowState:
        return self._state

    def transition(self, event: SafetyEvent, reason: str = "") -> Transition:
        if event is SafetyEvent.CRITICAL_FAULT and self._state not in {
            FollowState.DISCONNECTED,
            FollowState.DISABLING,
            FollowState.CRITICAL_STOP,
        }:
            result = Transition(
                FollowState.CRITICAL_STOP,
                (
                    SafetyAction.START_HOLD,
                    SafetyAction.REQUEST_PROTECTIVE_DISABLE,
                ),
                str(reason),
            )
        else:
            template = _TRANSITIONS.get((self._state, event))
            if template is None:
                raise RuntimeError(
                    f"非法安全状态转换：{self._state.name} + {event.name}"
                )
            result = Transition(template.state, template.actions, str(reason))
        self._state = result.state
        return result


class RuntimeGuard:
    def __init__(
        self,
        *,
        max_tracking_error_rad: float,
        tracking_error_grace_s: float,
        leader_stale_timeout_s: float,
        follower_stale_timeout_s: float,
        deadline_miss_limit: int,
    ) -> None:
        numeric = (
            max_tracking_error_rad,
            tracking_error_grace_s,
            leader_stale_timeout_s,
            follower_stale_timeout_s,
        )
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in numeric):
            raise ValueError("运行监视阈值必须大于零且为有限数值")
        if isinstance(deadline_miss_limit, bool) or deadline_miss_limit < 1:
            raise ValueError("deadline_miss_limit 必须是正整数")
        self._max_tracking_error = float(max_tracking_error_rad)
        self._tracking_grace = float(tracking_error_grace_s)
        self._leader_timeout = float(leader_stale_timeout_s)
        self._follower_timeout = float(follower_stale_timeout_s)
        self._deadline_miss_limit = int(deadline_miss_limit)
        self._tracking_violation_since: float | None = None
        self._deadline_misses = 0

    def observe(self, observation: RuntimeObservation) -> GuardFault | None:
        scalar_values = (
            observation.now_s,
            observation.leader_age_s,
            observation.follower_age_s,
        )
        if any(not math.isfinite(float(value)) or float(value) < 0.0 for value in scalar_values):
            return GuardFault(FaultClass.CRITICAL, "运行时间或反馈年龄无效")
        if len(observation.tracking_error_rad) != 6 or not all(
            math.isfinite(float(value)) and float(value) >= 0.0
            for value in observation.tracking_error_rad
        ):
            return GuardFault(FaultClass.CRITICAL, "从臂跟踪误差反馈无效")
        if len(observation.status_codes) != 6 or any(
            int(code) != 1 for code in observation.status_codes
        ):
            return GuardFault(
                FaultClass.CRITICAL,
                f"从臂 status_code 异常：{list(observation.status_codes)}",
            )
        if not observation.command_write_ok:
            return GuardFault(FaultClass.CRITICAL, "从臂命令写入失败")
        if not observation.port_identity_ok:
            return GuardFault(FaultClass.CRITICAL, "从臂串口设备身份变化")
        if observation.follower_age_s > self._follower_timeout:
            return GuardFault(
                FaultClass.CRITICAL,
                f"从臂反馈超时：{observation.follower_age_s:.3f}s",
            )
        if observation.leader_age_s > self._leader_timeout:
            return GuardFault(
                FaultClass.RECOVERABLE,
                f"引导臂反馈超时：{observation.leader_age_s:.3f}s",
            )

        if observation.deadline_missed:
            self._deadline_misses += 1
        else:
            self._deadline_misses = 0
        if self._deadline_misses >= self._deadline_miss_limit:
            return GuardFault(FaultClass.RECOVERABLE, "控制循环连续错过截止时间")

        worst_error = max(float(value) for value in observation.tracking_error_rad)
        if worst_error > self._max_tracking_error:
            if self._tracking_violation_since is None:
                self._tracking_violation_since = float(observation.now_s)
            elapsed = float(observation.now_s) - self._tracking_violation_since
            if elapsed + 1e-12 >= self._tracking_grace:
                return GuardFault(
                    FaultClass.RECOVERABLE,
                    f"从臂跟踪误差持续超限：{worst_error:.3f} rad",
                )
        else:
            self._tracking_violation_since = None
        return None

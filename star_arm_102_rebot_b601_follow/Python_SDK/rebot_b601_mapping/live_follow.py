from __future__ import annotations

import json
import math
import select
import statistics
import sys
import threading
import time
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .command_shaper import (
    CommandShaper,
    PointToPointTrajectory,
    TargetViolation,
)
from .follower_controller import (
    FollowerArmState,
    FollowerCommunicationError,
    FollowerController,
    FollowerLifecycleError,
)
from .leader_reader import LeaderReader
from .live_config import LiveFollowConfig, load_live_follow_config, validate_live_mapping
from .mapping import map_virtual_follower
from .models import Baseline, LeaderSample, MappingConfig, load_mapping_config
from .ports import assert_ports_unoccupied
from .safety_supervisor import (
    FaultClass,
    FollowState,
    RuntimeGuard,
    RuntimeObservation,
    SafetyEvent,
    SafetySupervisor,
)


@dataclass(frozen=True)
class LiveRunSummary:
    final_state: FollowState
    cycles: int
    safe_home_verified: bool
    disable_verified: bool
    disable_result_known: bool
    log_path: Path


class StopRequest:
    def __init__(self) -> None:
        self._normal = threading.Event()
        self._emergency = threading.Event()

    @property
    def normal_requested(self) -> bool:
        return self._normal.is_set()

    @property
    def emergency_requested(self) -> bool:
        return self._emergency.is_set()

    @property
    def requested(self) -> bool:
        return self.normal_requested or self.emergency_requested

    def request_normal(self) -> None:
        self._normal.set()

    def request_emergency(self) -> None:
        self._normal.set()
        self._emergency.set()


class LatestLeaderSampler:
    def __init__(
        self,
        port: str,
        *,
        reader_factory: Callable[[str], Any] = LeaderReader,
        sample_interval_s: float = 0.005,
    ) -> None:
        self._reader = reader_factory(port)
        self._sample_interval_s = float(sample_interval_s)
        self._condition = threading.Condition()
        self._history: deque[tuple[int, LeaderSample]] = deque(maxlen=256)
        self._sequence = 0
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def open(self) -> None:
        self._reader.open()
        first = self._reader.read_sample()
        with self._condition:
            self._sequence += 1
            self._history.append((self._sequence, first))
        self._thread = threading.Thread(
            target=self._run,
            name="star-arm-leader-sampler",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sample = self._reader.read_sample()
            except BaseException as exc:
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                return
            with self._condition:
                self._sequence += 1
                self._history.append((self._sequence, sample))
                self._condition.notify_all()
            self._stop.wait(self._sample_interval_s)

    def latest(self) -> LeaderSample:
        with self._condition:
            if not self._history:
                if self._error is not None:
                    raise RuntimeError(f"引导臂采样失败：{self._error}") from self._error
                raise RuntimeError("引导臂尚无有效样本")
            return self._history[-1][1]

    def capture_stable_baseline(self, sample_count: int) -> tuple[float, ...]:
        if sample_count < 1:
            raise ValueError("引导臂基线样本数必须大于零")
        deadline = time.monotonic() + 2.0
        with self._condition:
            start_sequence = self._sequence
            while True:
                samples = [
                    sample
                    for sequence, sample in self._history
                    if sequence > start_sequence
                ]
                if len(samples) >= sample_count:
                    selected = samples[-sample_count:]
                    break
                if self._error is not None:
                    raise RuntimeError(f"引导臂基线采样失败：{self._error}") from self._error
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("引导臂稳定基线采样超时")
                self._condition.wait(timeout=min(remaining, 0.05))
        if any(len(sample.angles_deg) != 7 for sample in selected):
            raise ValueError("引导臂基线样本必须包含七个角度")
        result = tuple(
            float(statistics.median(sample.angles_deg[index] for sample in selected))
            for index in range(7)
        )
        if not all(math.isfinite(value) for value in result):
            raise ValueError("引导臂基线包含非有限数值")
        return result

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._reader.close()


class _JsonlWriter:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", encoding="utf-8")

    def write(self, event: str, **values: Any) -> None:
        payload = {"event": event, **values}
        self._stream.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n")
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()


def poll_recovery_decision(timeout_s: float) -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], max(0.0, float(timeout_s)))
    if not readable:
        return None
    value = sys.stdin.readline().strip()
    return value if value in {"retry", "emergency_stop"} else None


def _joint_limits(mapping: MappingConfig) -> tuple[tuple[float, float], ...]:
    return tuple(
        (float(joint.lower_rad), float(joint.upper_rad))
        for joint in mapping.arm_joints
    )


def _joint_limit_violation(
    positions_rad: Sequence[float],
    joint_limits: Sequence[tuple[float, float]],
    *,
    label: str,
) -> str | None:
    positions = tuple(float(value) for value in positions_rad)
    limits = tuple((float(lower), float(upper)) for lower, upper in joint_limits)
    if len(positions) != 6 or len(limits) != 6:
        return f"{label}必须包含六个关节"
    if not all(math.isfinite(value) for value in positions):
        return f"{label}包含非有限数值"
    for index, (value, (lower, upper)) in enumerate(
        zip(positions, limits, strict=True),
        start=1,
    ):
        if value < lower or value > upper:
            return (
                f"{label} joint{index}={value:.12f} rad 超出网页关节边界 "
                f"[{lower:.12f}, {upper:.12f}] rad"
            )
    return None


def _make_shaper(
    mapping: MappingConfig,
    config: LiveFollowConfig,
    baseline: Sequence[float],
    speed: float,
) -> CommandShaper:
    return CommandShaper(
        joint_limits=_joint_limits(mapping),
        baseline_rad=baseline,
        max_speed_rad_s=speed,
        max_acceleration_rad_s2=config.max_acceleration_rad_s2,
        max_jerk_rad_s3=config.max_jerk_rad_s3,
    )


def _close_handles(follower: Any, sampler: Any, writer: _JsonlWriter) -> None:
    errors: list[str] = []
    for label, resource in (("从臂", follower), ("引导臂", sampler)):
        try:
            resource.close()
        except Exception as exc:
            errors.append(f"{label}句柄关闭失败：{exc}")
    writer.write("handles-closed", errors=errors)


def _summary(
    writer: _JsonlWriter,
    *,
    final_state: FollowState,
    cycles: int,
    safe_home_verified: bool,
    disable_verified: bool,
    disable_result_known: bool,
) -> LiveRunSummary:
    writer.write(
        "run-summary",
        final_state=final_state.name,
        cycles=cycles,
        safe_home_verified=safe_home_verified,
        disable_verified=disable_verified,
        disable_result_known=disable_result_known,
    )
    result = LiveRunSummary(
        final_state=final_state,
        cycles=cycles,
        safe_home_verified=safe_home_verified,
        disable_verified=disable_verified,
        disable_result_known=disable_result_known,
        log_path=writer.path,
    )
    writer.close()
    return result


def _return_safe_home(
    *,
    follower: Any,
    joint_limits: Sequence[tuple[float, float]],
    config: LiveFollowConfig,
    start_state: FollowerArmState,
    fallback_command: Sequence[float],
    speed: float,
    writer: _JsonlWriter,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> tuple[bool, FollowerArmState, tuple[float, ...]]:
    period = 1.0 / config.control_rate_hz
    last_safe_command = tuple(float(value) for value in fallback_command)
    fallback_violation = _joint_limit_violation(
        last_safe_command,
        joint_limits,
        label="安全回位备用保持命令",
    )
    if fallback_violation is not None:
        raise ValueError(fallback_violation)
    start_violation = _joint_limit_violation(
        start_state.positions_rad,
        joint_limits,
        label="安全回位起点反馈",
    )
    if start_violation is not None:
        writer.write("safe-home-start-out-of-limits", reason=start_violation)
        return False, start_state, last_safe_command
    trajectory = PointToPointTrajectory(
        start_rad=start_state.positions_rad,
        target_rad=config.safe_home_rad,
        max_speed_rad_s=speed,
        max_acceleration_rad_s2=config.max_acceleration_rad_s2,
        max_jerk_rad_s3=config.max_jerk_rad_s3,
    )
    started = float(clock())
    deadline = started + period
    stable_since: float | None = None
    tracking_violation_since: float | None = None
    state = start_state
    while float(clock()) - started <= config.safe_home_timeout_s:
        shaped = trajectory.sample(max(0.0, float(clock()) - started))
        state = follower.cycle(shaped.position_rad, speed)
        last_safe_command = shaped.position_rad
        error = tuple(
            abs(actual - target)
            for actual, target in zip(
                state.positions_rad,
                config.safe_home_rad,
                strict=True,
            )
        )
        tracking = tuple(
            abs(actual - target)
            for actual, target in zip(
                state.positions_rad,
                shaped.position_rad,
                strict=True,
            )
        )
        now = float(clock())
        writer.write(
            "safe-home-cycle",
            state=FollowState.RETURNING_SAFE_HOME.name,
            command_rad=list(shaped.position_rad),
            feedback_rad=list(state.positions_rad),
            feedback_velocity_rad_s=list(state.velocities_rad_s),
            planned_velocity_rad_s=list(shaped.velocity_rad_s),
            planned_acceleration_rad_s2=list(shaped.acceleration_rad_s2),
            safe_home_error_rad=list(error),
            tracking_error_rad=list(tracking),
        )
        feedback_violation = _joint_limit_violation(
            state.positions_rad,
            joint_limits,
            label="安全回位反馈",
        )
        if feedback_violation is not None:
            writer.write(
                "safe-home-feedback-out-of-limits",
                reason=feedback_violation,
            )
            return False, state, last_safe_command
        if max(tracking) > config.max_tracking_error_rad:
            if tracking_violation_since is None:
                tracking_violation_since = now
            elif now - tracking_violation_since >= config.tracking_error_grace_s:
                return False, state, last_safe_command
        else:
            tracking_violation_since = None
        arrived = max(error) < config.safe_home_tolerance_rad and max(
            abs(value) for value in state.velocities_rad_s
        ) < config.safe_home_velocity_tolerance_rad_s
        if arrived:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= config.safe_home_stable_s:
                return True, state, last_safe_command
        else:
            stable_since = None
        sleep(max(0.0, deadline - float(clock())))
        deadline += period
    return False, state, last_safe_command


def run_live_follow(
    *,
    leader_port: str,
    follower_port: str,
    mapping_path: Path,
    live_config_path: Path,
    log_path: Path,
    confirmed: bool,
    speed_rad_s: float | None = None,
    max_cycles: int | None = None,
    leader_factory: Callable[[str], Any] = LeaderReader,
    follower_factory: Callable[..., Any] = FollowerController,
    port_checker: Callable[[Sequence[str]], None] = assert_ports_unoccupied,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    stop_request: StopRequest | None = None,
    recovery_decider: Callable[[float], str | None] = poll_recovery_decision,
    sampler_factory: Callable[[str], Any] | None = None,
) -> LiveRunSummary:
    mapping = load_mapping_config(Path(mapping_path))
    config = load_live_follow_config(Path(live_config_path))
    validate_live_mapping(mapping, config)
    speed = config.default_speed_rad_s if speed_rad_s is None else float(speed_rad_s)
    if not math.isfinite(speed) or speed <= 0.0 or speed > config.max_speed_rad_s:
        raise ValueError(f"跟随速度必须位于 (0, {config.max_speed_rad_s}] rad/s")
    if max_cycles is not None and max_cycles < 0:
        raise ValueError("max_cycles 不得为负数")
    if not isinstance(confirmed, bool):
        raise TypeError("confirmed 必须是布尔值")

    port_checker((leader_port, follower_port))
    writer = _JsonlWriter(Path(log_path))
    sampler = (
        sampler_factory(leader_port)
        if sampler_factory is not None
        else LatestLeaderSampler(leader_port, reader_factory=leader_factory)
    )
    joint_limits = _joint_limits(mapping)
    follower = follower_factory(
        follower_port,
        config=config,
        joint_limits=joint_limits,
    )
    request = stop_request or StopRequest()
    cycles = 0
    sampler_open = False
    follower_open = False
    enabled = False
    last_command: tuple[float, ...] | None = None
    supervisor = SafetySupervisor(FollowState.PRECHECK)

    try:
        sampler.open()
        sampler_open = True
        follower.open()
        follower_open = True
        leader = sampler.latest()
        if len(leader.angles_deg) != 7 or not all(
            math.isfinite(float(value)) for value in leader.angles_deg
        ):
            raise ValueError("引导臂预检样本必须包含七个有限角度")
        leader_age = float(clock()) - float(leader.timestamp_s)
        if leader_age < 0.0 or leader_age > config.leader_stale_timeout_s:
            raise ValueError(f"引导臂预检反馈超时：{leader_age:.3f}s")
        preflight = follower.read_state(expected_arm_status=0, expected_gripper_status=0)
        preflight_shaper = _make_shaper(
            mapping,
            config,
            preflight.positions_rad,
            speed,
        )
        preflight_shaper.reset(preflight.positions_rad)
        writer.write(
            "preflight",
            mapping_acceptance=config.mapping_acceptance,
            leader_angles_deg=list(leader.angles_deg),
            follower_positions_rad=list(preflight.positions_rad),
            arm_status_codes=list(preflight.status_codes),
            gripper_status_code=preflight.gripper_status_code,
            speed_rad_s=speed,
            thresholds=asdict(config),
            motion_confirmed=confirmed,
        )
        if not confirmed:
            _close_handles(follower, sampler, writer)
            return _summary(
                writer,
                final_state=FollowState.DISCONNECTED,
                cycles=0,
                safe_home_verified=False,
                disable_verified=False,
                disable_result_known=True,
            )

        follower.verify_pos_vel_configuration()
        enable_result = follower.enable_hold(speed)
        enabled = True
        enabled_state = enable_result.state
        last_command = enable_result.command_rad
        supervisor.transition(SafetyEvent.ENABLE_OK, "六轴使能并保持启动位置")
        enabled_feedback_violation = _joint_limit_violation(
            enabled_state.positions_rad,
            joint_limits,
            label="使能后反馈",
        )
        startup_recoverable_reason = enabled_feedback_violation
        if enabled_feedback_violation is not None:
            writer.write(
                "enabled-feedback-out-of-limits",
                reason=enabled_feedback_violation,
            )
            leader_baseline = tuple(leader.angles_deg)
            request.request_normal()
            follower_baseline = last_command
        else:
            try:
                leader_baseline = sampler.capture_stable_baseline(5)
            except Exception as exc:
                leader_baseline = tuple(leader.angles_deg)
                startup_recoverable_reason = f"引导臂基线采集失败：{exc}"
                request.request_normal()
            follower_baseline = enabled_state.positions_rad
        baseline = Baseline(
            captured_at_s=float(clock()),
            leader_angles_deg=tuple(leader_baseline),
            follower_positions_rad=follower_baseline
            + (enabled_state.gripper_position_rad,),
        )
        if startup_recoverable_reason is None:
            supervisor.transition(SafetyEvent.FOLLOW_START, "进入实时跟随")
        shaper = _make_shaper(mapping, config, follower_baseline, speed)
        shaper.reset(follower_baseline)
        guard = RuntimeGuard(
            max_tracking_error_rad=config.max_tracking_error_rad,
            tracking_error_grace_s=config.tracking_error_grace_s,
            leader_stale_timeout_s=config.leader_stale_timeout_s,
            follower_stale_timeout_s=config.follower_stale_timeout_s,
            deadline_miss_limit=config.deadline_miss_limit,
        )
        period = 1.0 / config.control_rate_hz
        previous_tick = float(clock())
        deadline = previous_tick + period
        recoverable_reason = startup_recoverable_reason or "正常停止请求"
        critical_reason: str | None = None

        while True:
            if request.emergency_requested:
                critical_reason = "操作员紧急停止请求"
                break
            if request.normal_requested or (
                max_cycles is not None and cycles >= max_cycles
            ):
                break
            now = float(clock())
            dt = max(now - previous_tick, period)
            previous_tick = now
            try:
                leader = sampler.latest()
            except Exception as exc:
                recoverable_reason = f"引导臂采样失败：{exc}"
                break
            try:
                mapped = map_virtual_follower(leader, baseline, mapping)
                shaped = shaper.step(mapped.positions_rad, dt)
            except (ValueError, TargetViolation) as exc:
                recoverable_reason = str(exc)
                break
            try:
                follower_state = follower.cycle(shaped.position_rad, speed)
            except (FollowerCommunicationError, FollowerLifecycleError) as exc:
                critical_reason = str(exc)
                break
            last_command = shaped.position_rad
            finished_at = float(clock())
            tracking_error = tuple(
                abs(actual - commanded)
                for actual, commanded in zip(
                    follower_state.positions_rad,
                    shaped.position_rad,
                    strict=True,
                )
            )
            observation = RuntimeObservation(
                now_s=finished_at,
                leader_age_s=finished_at - leader.timestamp_s,
                follower_age_s=finished_at - follower_state.timestamp_s,
                tracking_error_rad=tracking_error,
                status_codes=follower_state.status_codes,
                deadline_missed=finished_at > deadline,
                command_write_ok=True,
                port_identity_ok=True,
            )
            fault = guard.observe(observation)
            writer.write(
                "cycle",
                state=supervisor.state.name,
                period_s=dt,
                leader_angles_deg=list(leader.angles_deg),
                leader_delta_rad=list(mapped.leader_deltas_rad),
                raw_target_rad=list(mapped.positions_rad),
                command_rad=list(shaped.position_rad),
                feedback_rad=list(follower_state.positions_rad),
                feedback_velocity_rad_s=list(follower_state.velocities_rad_s),
                tracking_error_rad=list(tracking_error),
                leader_lag_rad=[
                    raw - command
                    for raw, command in zip(
                        mapped.positions_rad,
                        shaped.position_rad,
                        strict=True,
                    )
                ],
                status_codes=list(follower_state.status_codes),
                follower_age_s=observation.follower_age_s,
            )
            cycles += 1
            feedback_violation = _joint_limit_violation(
                follower_state.positions_rad,
                joint_limits,
                label="跟随反馈",
            )
            if feedback_violation is not None:
                writer.write(
                    "follow-feedback-out-of-limits",
                    reason=feedback_violation,
                    command_rad=list(shaped.position_rad),
                    feedback_rad=list(follower_state.positions_rad),
                )
                recoverable_reason = feedback_violation
                break
            if fault is not None:
                if fault.fault_class is FaultClass.CRITICAL:
                    critical_reason = fault.reason
                else:
                    recoverable_reason = fault.reason
                break
            sleep(max(0.0, deadline - float(clock())))
            deadline += period

        if critical_reason is not None:
            supervisor.transition(SafetyEvent.CRITICAL_FAULT, critical_reason)
            writer.write("critical-stop", reason=critical_reason)
            if last_command is not None:
                try:
                    follower.cycle(last_command, speed)
                except Exception as exc:
                    writer.write("critical-hold-unavailable", reason=str(exc))
            disable_known = True
            disable_verified = False
            try:
                follower.disable_verified()
                disable_verified = True
            except Exception as exc:
                disable_known = False
                writer.write("protective-disable-unknown", reason=str(exc))
            _close_handles(follower, sampler, writer)
            return _summary(
                writer,
                final_state=FollowState.CRITICAL_STOP,
                cycles=cycles,
                safe_home_verified=False,
                disable_verified=disable_verified,
                disable_result_known=disable_known,
            )

        supervisor.transition(SafetyEvent.RECOVERABLE_FAULT, recoverable_reason)
        writer.write("recoverable-hold", reason=recoverable_reason)
        try:
            if last_command is None:
                raise FollowerLifecycleError("缺少可用于保持的最后安全命令")
            hold_result = follower.hold_current(
                speed,
                fallback_target_rad=last_command,
            )
            hold_state = hold_result.state
            last_command = hold_result.command_rad
            if hold_result.used_fallback:
                writer.write(
                    "hold-used-last-safe-command",
                    command_rad=list(last_command),
                )
        except Exception as exc:
            supervisor.transition(SafetyEvent.CRITICAL_FAULT, str(exc))
            writer.write("critical-stop", reason=f"无法确认保持：{exc}")
            try:
                if last_command is not None:
                    follower.cycle(last_command, speed)
                follower.disable_verified()
                disable_known, disable_verified = True, True
            except Exception as disable_exc:
                disable_known, disable_verified = False, False
                writer.write("protective-disable-unknown", reason=str(disable_exc))
            _close_handles(follower, sampler, writer)
            return _summary(
                writer,
                final_state=FollowState.CRITICAL_STOP,
                cycles=cycles,
                safe_home_verified=False,
                disable_verified=disable_verified,
                disable_result_known=disable_known,
            )
        supervisor.transition(SafetyEvent.HOLD_CONFIRMED, "保持目标已确认")

        while True:
            try:
                if last_command is None:
                    raise FollowerLifecycleError(
                        "缺少可用于安全回位的最后安全命令"
                    )
                home_ok, home_state, last_command = _return_safe_home(
                    follower=follower,
                    joint_limits=joint_limits,
                    config=config,
                    start_state=hold_state,
                    fallback_command=last_command,
                    speed=speed,
                    writer=writer,
                    clock=clock,
                    sleep=sleep,
                )
            except (FollowerCommunicationError, FollowerLifecycleError) as exc:
                supervisor.transition(SafetyEvent.CRITICAL_FAULT, str(exc))
                writer.write("critical-stop", reason=str(exc))
                try:
                    follower.disable_verified()
                    disable_known, disable_verified = True, True
                except Exception as disable_exc:
                    disable_known, disable_verified = False, False
                    writer.write("protective-disable-unknown", reason=str(disable_exc))
                _close_handles(follower, sampler, writer)
                return _summary(
                    writer,
                    final_state=FollowState.CRITICAL_STOP,
                    cycles=cycles,
                    safe_home_verified=False,
                    disable_verified=disable_verified,
                    disable_result_known=disable_known,
                )
            if home_ok:
                supervisor.transition(SafetyEvent.SAFE_HOME_REACHED, "安全位稳定到位")
                writer.write("safe-home-verified", positions_rad=list(home_state.positions_rad))
                try:
                    follower.disable_verified()
                except Exception as exc:
                    writer.write("protective-disable-unknown", reason=str(exc))
                    _close_handles(follower, sampler, writer)
                    return _summary(
                        writer,
                        final_state=FollowState.CRITICAL_STOP,
                        cycles=cycles,
                        safe_home_verified=True,
                        disable_verified=False,
                        disable_result_known=False,
                    )
                supervisor.transition(SafetyEvent.DISABLE_OK, "六轴失能已验证")
                writer.write("disable-verified", status_codes=[0] * 6)
                _close_handles(follower, sampler, writer)
                return _summary(
                    writer,
                    final_state=FollowState.DISCONNECTED,
                    cycles=cycles,
                    safe_home_verified=True,
                    disable_verified=True,
                    disable_result_known=True,
                )

            supervisor.transition(
                SafetyEvent.SAFE_HOME_FAILED_HEALTHY,
                "回安全位超时、跟踪误差持续超限或反馈越过网页关节边界",
            )
            try:
                if last_command is None:
                    raise FollowerLifecycleError("缺少可用于恢复保持的最后安全命令")
                hold_result = follower.hold_current(
                    speed,
                    fallback_target_rad=last_command,
                )
                hold_state = hold_result.state
                last_command = hold_result.command_rad
                if hold_result.used_fallback:
                    writer.write(
                        "hold-used-last-safe-command",
                        command_rad=list(last_command),
                    )
            except (FollowerCommunicationError, FollowerLifecycleError) as exc:
                supervisor.transition(SafetyEvent.CRITICAL_FAULT, str(exc))
                writer.write("critical-stop", reason=str(exc))
                try:
                    follower.disable_verified()
                    disable_known, disable_verified = True, True
                except Exception as disable_exc:
                    disable_known, disable_verified = False, False
                    writer.write("protective-disable-unknown", reason=str(disable_exc))
                _close_handles(follower, sampler, writer)
                return _summary(
                    writer,
                    final_state=FollowState.CRITICAL_STOP,
                    cycles=cycles,
                    safe_home_verified=False,
                    disable_verified=disable_verified,
                    disable_result_known=disable_known,
                )
            writer.write(
                "operator-recovery-enabled-hold",
                reason="回安全位失败，保持使能等待 retry 或 emergency_stop",
            )
            while True:
                try:
                    assert last_command is not None
                    follower.cycle(last_command, speed)
                except (FollowerCommunicationError, FollowerLifecycleError) as exc:
                    supervisor.transition(SafetyEvent.CRITICAL_FAULT, str(exc))
                    writer.write("critical-stop", reason=str(exc))
                    try:
                        follower.disable_verified()
                        disable_known, disable_verified = True, True
                    except Exception as disable_exc:
                        disable_known, disable_verified = False, False
                        writer.write(
                            "protective-disable-unknown",
                            reason=str(disable_exc),
                        )
                    _close_handles(follower, sampler, writer)
                    return _summary(
                        writer,
                        final_state=FollowState.CRITICAL_STOP,
                        cycles=cycles,
                        safe_home_verified=False,
                        disable_verified=disable_verified,
                        disable_result_known=disable_known,
                    )
                decision = recovery_decider(1.0 / config.control_rate_hz)
                if decision == "retry":
                    supervisor.transition(SafetyEvent.RETRY_RETURN, "操作员重试回位")
                    break
                if decision == "emergency_stop" or request.emergency_requested:
                    supervisor.transition(SafetyEvent.CRITICAL_FAULT, "操作员紧急停止")
                    writer.write("critical-stop", reason="操作员紧急停止")
                    try:
                        follower.disable_verified()
                        disable_known, disable_verified = True, True
                    except Exception as exc:
                        disable_known, disable_verified = False, False
                        writer.write("protective-disable-unknown", reason=str(exc))
                    _close_handles(follower, sampler, writer)
                    return _summary(
                        writer,
                        final_state=FollowState.CRITICAL_STOP,
                        cycles=cycles,
                        safe_home_verified=False,
                        disable_verified=disable_verified,
                        disable_result_known=disable_known,
                    )
                sleep(1.0 / config.control_rate_hz)
    except BaseException:
        if not enabled:
            if follower_open and sampler_open:
                _close_handles(follower, sampler, writer)
            elif follower_open:
                follower.close()
            elif sampler_open:
                sampler.close()
            writer.close()
        raise

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

from .hardware_specs import ARM_MOTOR_SPECS, GRIPPER_SPEC, MOTOR_SPECS
from .live_config import LiveFollowConfig
from .ports import PortIdentity, assert_same_port


FEEDBACK_RETRIES = 3
FEEDBACK_RETRY_INTERVAL_S = 0.005
POS_VEL_GAIN_REGISTERS = (25, 26, 27, 28)


class FollowerCommunicationError(RuntimeError):
    pass


class FollowerLifecycleError(RuntimeError):
    pass


@dataclass(frozen=True)
class FollowerArmState:
    timestamp_s: float
    positions_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...]
    torques_nm: tuple[float, ...]
    status_codes: tuple[int, ...]
    gripper_position_rad: float
    gripper_velocity_rad_s: float
    gripper_torque_nm: float
    gripper_status_code: int


@dataclass(frozen=True)
class HoldResult:
    state: FollowerArmState
    command_rad: tuple[float, ...]
    used_fallback: bool


@dataclass(frozen=True)
class EnableHoldResult:
    state: FollowerArmState
    command_rad: tuple[float, ...]


def _make_controller(port: str, baudrate: int):
    from motorbridge import Controller

    return Controller.from_dm_serial(port, baudrate)


class FollowerController:
    def __init__(
        self,
        port: str,
        *,
        config: LiveFollowConfig,
        joint_limits: Sequence[tuple[float, float]],
        controller_factory: Callable[[str, int], Any] = _make_controller,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        identity_factory: Callable[[str], PortIdentity] = PortIdentity.capture,
        identity_checker: Callable[[PortIdentity], None] = assert_same_port,
    ) -> None:
        self._port = str(port)
        self._config = config
        limits = tuple(
            (float(lower), float(upper)) for lower, upper in joint_limits
        )
        if len(limits) != 6 or any(
            not math.isfinite(lower)
            or not math.isfinite(upper)
            or lower >= upper
            for lower, upper in limits
        ):
            raise ValueError("joint_limits 必须是六组有效的网页关节边界")
        self._joint_limits = limits
        self._controller_factory = controller_factory
        self._clock = clock
        self._sleep = sleep
        self._identity_factory = identity_factory
        self._identity_checker = identity_checker
        self._identity: PortIdentity | None = None
        self._controller: Any | None = None
        self._motors: list[Any] = []

    @property
    def is_open(self) -> bool:
        return self._controller is not None

    def open(self) -> None:
        if self.is_open:
            raise FollowerLifecycleError("从臂串口已经打开")
        identity = self._identity_factory(self._port)
        controller = None
        motors: list[Any] = []
        try:
            controller = self._controller_factory(self._port, 921_600)
            self._identity_checker(identity)
            for spec in MOTOR_SPECS:
                motors.append(
                    controller.add_damiao_motor(
                        spec.motor_id,
                        spec.feedback_id,
                        spec.model,
                    )
                )
        except Exception as exc:
            for motor in motors:
                try:
                    motor.close()
                except Exception:
                    pass
            if controller is not None:
                try:
                    controller.close()
                except Exception:
                    pass
            raise FollowerCommunicationError(f"打开从臂串口失败：{exc}") from exc
        self._identity = identity
        self._controller = controller
        self._motors = motors

    def _require_open(self) -> tuple[Any, PortIdentity]:
        if self._controller is None or self._identity is None:
            raise FollowerLifecycleError("从臂串口尚未打开")
        return self._controller, self._identity

    def _refresh_all(self) -> tuple[Any, ...]:
        controller, identity = self._require_open()
        started = float(self._clock())
        states: list[Any] = []
        try:
            self._identity_checker(identity)
            for spec, motor in zip(MOTOR_SPECS, self._motors, strict=True):
                state = None
                for attempt in range(FEEDBACK_RETRIES):
                    motor.request_feedback()
                    controller.poll_feedback_once()
                    if (
                        float(self._clock()) - started
                        > self._config.follower_stale_timeout_s
                    ):
                        raise TimeoutError(
                            "全六轴反馈事务超过 "
                            f"{self._config.follower_stale_timeout_s:.2f}s"
                        )
                    state = motor.get_state()
                    if state is not None:
                        break
                    if attempt + 1 < FEEDBACK_RETRIES:
                        self._sleep(FEEDBACK_RETRY_INTERVAL_S)
                if state is None:
                    raise FollowerCommunicationError(
                        f"{spec.name} 反馈缺失，已重试 {FEEDBACK_RETRIES} 次"
                    )
                states.append(state)
        except Exception as exc:
            if isinstance(exc, FollowerCommunicationError):
                raise
            raise FollowerCommunicationError(f"刷新从臂反馈失败：{exc}") from exc
        return tuple(states)

    @staticmethod
    def _state_values(name: str, state: Any) -> tuple[float, float, float, int]:
        if state is None:
            raise FollowerCommunicationError(f"{name} 反馈缺失")
        try:
            values = (float(state.pos), float(state.vel), float(state.torq))
            status = int(state.status_code)
        except Exception as exc:
            raise FollowerCommunicationError(f"{name} 反馈结构无效：{exc}") from exc
        if not all(math.isfinite(value) for value in values):
            raise FollowerCommunicationError(f"{name} 反馈包含非有限数值")
        return values[0], values[1], values[2], status

    def read_state(
        self,
        expected_arm_status: int | None = None,
        expected_gripper_status: int | None = 0,
    ) -> FollowerArmState:
        states = self._refresh_all()
        arm_values: list[tuple[float, float, float, int]] = []
        for spec, state in zip(ARM_MOTOR_SPECS, states[:6], strict=True):
            values = self._state_values(spec.name, state)
            if expected_arm_status is not None and values[3] != expected_arm_status:
                raise FollowerLifecycleError(
                    f"{spec.name} status_code={values[3]}，期望 {expected_arm_status}"
                )
            arm_values.append(values)
        gripper_values = self._state_values(GRIPPER_SPEC.name, states[6])
        if (
            expected_gripper_status is not None
            and gripper_values[3] != expected_gripper_status
        ):
            raise FollowerLifecycleError(
                f"gripper status_code={gripper_values[3]}，期望 {expected_gripper_status}"
            )
        return FollowerArmState(
            timestamp_s=float(self._clock()),
            positions_rad=tuple(value[0] for value in arm_values),
            velocities_rad_s=tuple(value[1] for value in arm_values),
            torques_nm=tuple(value[2] for value in arm_values),
            status_codes=tuple(value[3] for value in arm_values),
            gripper_position_rad=gripper_values[0],
            gripper_velocity_rad_s=gripper_values[1],
            gripper_torque_nm=gripper_values[2],
            gripper_status_code=gripper_values[3],
        )

    def verify_pos_vel_configuration(self, timeout_ms: int = 250) -> None:
        from motorbridge import Mode

        timeout = int(timeout_ms)
        if timeout <= 0:
            raise ValueError("timeout_ms 必须为正整数")
        expected_mode = int(Mode.POS_VEL)
        for spec, motor in zip(
            ARM_MOTOR_SPECS,
            self._motors[:6],
            strict=True,
        ):
            try:
                actual_mode = int(motor.get_register_u32(10, timeout))
                if actual_mode != expected_mode:
                    raise FollowerLifecycleError(
                        f"{spec.name} RID 10 模式不符："
                        f"期望 {expected_mode}，实际 {actual_mode}"
                    )
                for register in POS_VEL_GAIN_REGISTERS:
                    actual = float(motor.get_register_f32(register, timeout))
                    if not math.isfinite(actual) or actual < 0.0:
                        raise FollowerLifecycleError(
                            f"{spec.name} RID {register} 参数无效："
                            f"实际 {actual:.9g}，要求有限且非负"
                        )
            except FollowerLifecycleError:
                raise
            except Exception as exc:
                raise FollowerLifecycleError(
                    f"{spec.name} POS_VEL 配置只读校验失败：{exc}"
                ) from exc

    def _wait_for_one_status(self, index: int, expected_status: int) -> int:
        controller, identity = self._require_open()
        spec = ARM_MOTOR_SPECS[index]
        motor = self._motors[index]
        deadline = float(self._clock()) + self._config.follower_stale_timeout_s
        last_status: int | None = None
        while True:
            try:
                self._identity_checker(identity)
                motor.request_feedback()
                controller.poll_feedback_once()
                state = motor.get_state()
                if state is not None:
                    last_status = self._state_values(spec.name, state)[3]
                    if last_status == expected_status:
                        return last_status
            except Exception as exc:
                if isinstance(exc, FollowerCommunicationError):
                    raise
                raise FollowerCommunicationError(
                    f"{spec.name} 状态反馈刷新失败：{exc}"
                ) from exc
            remaining = deadline - float(self._clock())
            if remaining <= 0.0:
                raise FollowerLifecycleError(
                    f"{spec.name} 等待 status_code={expected_status} 超时，"
                    f"最后状态为 {last_status}"
                )
            self._sleep(min(FEEDBACK_RETRY_INTERVAL_S, remaining))

    def _wait_for_all_arm_status(self, expected_status: int) -> FollowerArmState:
        deadline = float(self._clock()) + self._config.follower_stale_timeout_s
        last_statuses: tuple[int, ...] | None = None
        while True:
            state = self.read_state(
                expected_arm_status=None,
                expected_gripper_status=0,
            )
            last_statuses = state.status_codes
            if all(status == expected_status for status in last_statuses):
                return state
            remaining = deadline - float(self._clock())
            if remaining <= 0.0:
                raise FollowerLifecycleError(
                    f"等待六轴 status_code={expected_status} 超时，"
                    f"最后状态为 {list(last_statuses)}"
                )
            self._sleep(min(FEEDBACK_RETRY_INTERVAL_S, remaining))

    def _rollback_enable(self) -> str | None:
        errors: list[str] = []
        for spec, motor in zip(ARM_MOTOR_SPECS, self._motors[:6], strict=True):
            try:
                motor.disable()
            except Exception as exc:
                errors.append(f"{spec.name} 失能失败：{exc}")
        try:
            self._wait_for_all_arm_status(0)
        except Exception as exc:
            errors.append(f"失能验证失败：{exc}")
        return "；".join(errors) if errors else None

    def enable_hold(self, speed_rad_s: float) -> EnableHoldResult:
        speed = self._validate_speed(speed_rad_s)
        initial = self.read_state(expected_arm_status=0, expected_gripper_status=0)
        initial_target = self._target(initial.positions_rad)
        try:
            for index, (spec, motor) in enumerate(
                zip(ARM_MOTOR_SPECS, self._motors[:6], strict=True)
            ):
                try:
                    motor.enable()
                except Exception as exc:
                    raise FollowerLifecycleError(
                        f"{spec.name} 使能请求失败：{exc}"
                    ) from exc
                self._wait_for_one_status(index, 1)
                motor.send_pos_vel(initial_target[index], speed)
            state = self.read_state(
                expected_arm_status=1,
                expected_gripper_status=0,
            )
            return EnableHoldResult(state=state, command_rad=initial_target)
        except Exception as exc:
            rollback_error = self._rollback_enable()
            message = f"从臂使能保持失败：{exc}"
            if rollback_error:
                message += f"；回滚未验证：{rollback_error}"
            raise FollowerLifecycleError(message) from exc

    def _validate_speed(self, speed_rad_s: float) -> float:
        speed = float(speed_rad_s)
        if (
            not math.isfinite(speed)
            or speed <= 0.0
            or speed > self._config.max_speed_rad_s
        ):
            raise ValueError(
                f"速度必须位于 (0, {self._config.max_speed_rad_s}] rad/s"
            )
        return speed

    def _target(self, values: Sequence[float]) -> tuple[float, ...]:
        target = tuple(float(value) for value in values)
        if len(target) != 6:
            raise ValueError("从臂目标必须包含六个关节")
        if not all(math.isfinite(value) for value in target):
            raise ValueError("从臂目标必须为有限数值")
        for index, (value, (lower, upper)) in enumerate(
            zip(target, self._joint_limits, strict=True),
            start=1,
        ):
            if value < lower or value > upper:
                raise ValueError(
                    f"joint{index} 从臂目标超出网页关节边界 "
                    f"[{lower:.12f}, {upper:.12f}] rad"
                )
        return target

    def cycle(
        self,
        target_rad: Sequence[float],
        speed_rad_s: float,
    ) -> FollowerArmState:
        target = self._target(target_rad)
        speed = self._validate_speed(speed_rad_s)
        for index, (spec, motor) in enumerate(
            zip(ARM_MOTOR_SPECS, self._motors[:6], strict=True)
        ):
            try:
                motor.send_pos_vel(target[index], speed)
            except Exception as exc:
                raise FollowerCommunicationError(
                    f"{spec.name} POS_VEL 命令写入失败：{exc}"
                ) from exc
        return self.read_state(expected_arm_status=1, expected_gripper_status=0)

    def hold_current(
        self,
        speed_rad_s: float,
        *,
        fallback_target_rad: Sequence[float] | None = None,
    ) -> HoldResult:
        state = self.read_state(expected_arm_status=1, expected_gripper_status=0)
        used_fallback = False
        try:
            command = self._target(state.positions_rad)
        except ValueError:
            if fallback_target_rad is None:
                raise
            command = self._target(fallback_target_rad)
            used_fallback = True
        held_state = self.cycle(command, speed_rad_s)
        return HoldResult(
            state=held_state,
            command_rad=command,
            used_fallback=used_fallback,
        )

    def disable_verified(self) -> FollowerArmState:
        errors: list[str] = []
        for spec, motor in zip(ARM_MOTOR_SPECS, self._motors[:6], strict=True):
            try:
                motor.disable()
            except Exception as exc:
                errors.append(f"{spec.name} 失能请求失败：{exc}")
        try:
            state = self._wait_for_all_arm_status(0)
        except Exception as exc:
            errors.append(f"六轴失能验证失败：{exc}")
            state = None
        if errors:
            raise FollowerLifecycleError("；".join(errors))
        assert state is not None
        return state

    def close(self) -> None:
        motors = self._motors
        controller = self._controller
        self._motors = []
        self._controller = None
        self._identity = None
        errors: list[str] = []
        for spec, motor in zip(MOTOR_SPECS, motors):
            try:
                motor.close()
            except Exception as exc:
                errors.append(f"{spec.name} 句柄关闭失败：{exc}")
        if controller is not None:
            try:
                controller.close()
            except Exception as exc:
                errors.append(f"控制器句柄关闭失败：{exc}")
        if errors:
            raise FollowerCommunicationError("；".join(errors))

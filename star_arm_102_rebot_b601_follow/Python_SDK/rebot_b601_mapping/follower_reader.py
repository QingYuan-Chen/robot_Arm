from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from .hardware_specs import MOTOR_SPECS
from .models import FollowerSample, MotorFeedback
from .ports import PortIdentity, assert_same_port

FEEDBACK_RETRIES = 3
FEEDBACK_RETRY_INTERVAL_S = 0.005


def _make_controller(port: str, baudrate: int):
    from motorbridge import Controller

    return Controller.from_dm_serial(port, baudrate)


class FollowerReader:
    """直接使用 motorbridge 的 reBot B601-DM 只读反馈读取器。"""

    def __init__(
        self,
        port: str,
        *,
        controller_factory: Callable[[str, int], Any] = _make_controller,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        identity_factory: Callable[[str], PortIdentity] = PortIdentity.capture,
        identity_checker: Callable[[PortIdentity], None] = assert_same_port,
    ) -> None:
        self._port = str(port)
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
            raise RuntimeError("从臂串口已经打开")
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
        except BaseException:
            for motor in motors:
                try:
                    motor.close()
                except Exception:
                    pass
            if controller is not None:
                controller.close()
            raise
        self._identity = identity
        self._controller = controller
        self._motors = motors

    def read_sample(self) -> FollowerSample:
        if self._controller is None or self._identity is None:
            raise RuntimeError("从臂串口尚未打开")
        self._identity_checker(self._identity)
        feedback: list[MotorFeedback] = []
        for spec, motor in zip(MOTOR_SPECS, self._motors, strict=True):
            state = None
            last_error: Exception | None = None
            for attempt in range(FEEDBACK_RETRIES):
                try:
                    motor.request_feedback()
                    self._controller.poll_feedback_once()
                    state = motor.get_state()
                    if state is not None:
                        break
                except Exception as exc:
                    last_error = exc
                if attempt + 1 < FEEDBACK_RETRIES:
                    self._sleep(FEEDBACK_RETRY_INTERVAL_S)
            if state is None:
                detail = f"：{last_error}" if last_error is not None else ""
                raise RuntimeError(
                    f"{spec.name} 反馈缺失，已重试 {FEEDBACK_RETRIES} 次{detail}"
                )
            values = (float(state.pos), float(state.vel), float(state.torq))
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"{spec.name} 反馈包含非有限数值")
            status_code = int(state.status_code)
            if status_code != 0:
                raise RuntimeError(
                    f"{spec.name} status_code={status_code}，只读映射要求为 0"
                )
            feedback.append(
                MotorFeedback(
                    name=spec.name,
                    position_rad=values[0],
                    velocity_rad_s=values[1],
                    torque_nm=values[2],
                    status_code=status_code,
                )
            )
        return FollowerSample(
            timestamp_s=float(self._clock()),
            motors=tuple(feedback),
        )

    def close(self) -> None:
        motors = self._motors
        controller = self._controller
        self._motors = []
        self._controller = None
        self._identity = None
        errors: list[str] = []
        for motor in motors:
            try:
                motor.close()
            except Exception as exc:
                errors.append(f"电机句柄关闭失败：{exc}")
        if controller is not None:
            try:
                controller.close()
            except Exception as exc:
                errors.append(f"控制器句柄关闭失败：{exc}")
        if errors:
            raise RuntimeError("；".join(errors))

    def __enter__(self) -> "FollowerReader":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

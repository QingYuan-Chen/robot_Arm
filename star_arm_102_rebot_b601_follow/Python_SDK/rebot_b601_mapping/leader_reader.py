from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from .models import LeaderSample
from .ports import PortIdentity, assert_same_port


SERVO_IDS = (0, 1, 2, 3, 4, 5, 6)


def _open_serial(**kwargs):
    import serial

    return serial.Serial(**kwargs)


def _make_manager(serial_port):
    import fashionstar_uart_sdk as uservo

    return uservo.UartServoManager(serial_port)


class LeaderReader:
    """Star Arm 102-LD 仅监视读取器，不暴露任何控制接口。"""

    def __init__(
        self,
        port: str,
        *,
        serial_factory: Callable[..., Any] = _open_serial,
        manager_factory: Callable[[Any], Any] = _make_manager,
        clock: Callable[[], float] = time.monotonic,
        identity_factory: Callable[[str], PortIdentity] = PortIdentity.capture,
        identity_checker: Callable[[PortIdentity], None] = assert_same_port,
    ) -> None:
        self._port = str(port)
        self._serial_factory = serial_factory
        self._manager_factory = manager_factory
        self._clock = clock
        self._identity_factory = identity_factory
        self._identity_checker = identity_checker
        self._identity: PortIdentity | None = None
        self._serial: Any | None = None
        self._manager: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None

    def open(self) -> None:
        if self.is_open:
            raise RuntimeError("引导臂串口已经打开")
        identity = self._identity_factory(self._port)
        serial_port = None
        try:
            serial_port = self._serial_factory(
                port=self._port,
                baudrate=1_000_000,
                parity="N",
                stopbits=1,
                bytesize=8,
                timeout=0.05,
                exclusive=True,
            )
            self._identity_checker(identity)
            manager = self._manager_factory(serial_port)
        except BaseException:
            if serial_port is not None:
                serial_port.close()
            raise
        self._identity = identity
        self._serial = serial_port
        self._manager = manager

    def read_sample(self) -> LeaderSample:
        if self._manager is None or self._identity is None:
            raise RuntimeError("引导臂串口尚未打开")
        self._identity_checker(self._identity)
        states = self._manager.send_sync_servo_monitor(SERVO_IDS, realtime=True)
        angles: list[float] = []
        for servo_id in SERVO_IDS:
            if servo_id not in states or states[servo_id] is None:
                raise RuntimeError(f"引导臂 ID {servo_id} 反馈缺失")
            state = states[servo_id]
            angle = getattr(state, "angle_monitor", None)
            if angle is None:
                raise RuntimeError(f"引导臂 ID {servo_id} 无有效角度")
            angle_value = float(angle)
            if not math.isfinite(angle_value):
                raise RuntimeError(f"引导臂 ID {servo_id} 角度为非有限数值")
            angles.append(angle_value)
        return LeaderSample(timestamp_s=float(self._clock()), angles_deg=tuple(angles))

    def close(self) -> None:
        serial_port = self._serial
        self._serial = None
        self._manager = None
        self._identity = None
        if serial_port is not None:
            serial_port.close()

    def __enter__(self) -> "LeaderReader":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

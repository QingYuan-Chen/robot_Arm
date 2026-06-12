from __future__ import annotations

from .command_models import ControlMode


def parse_control_mode(value: str) -> ControlMode:
    normalized = value.strip().lower()
    if normalized == ControlMode.SIMULATION.value:
        return ControlMode.SIMULATION
    if normalized == ControlMode.REAL.value:
        return ControlMode.REAL
    raise ValueError(f"unsupported control mode: {value}")

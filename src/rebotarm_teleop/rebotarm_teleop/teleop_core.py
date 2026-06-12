from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


DEFAULT_KEY_BINDINGS: dict[str, tuple[str, float]] = {
    "1": ("joint1", 1.0),
    "q": ("joint1", -1.0),
    "2": ("joint2", 1.0),
    "w": ("joint2", -1.0),
    "3": ("joint3", 1.0),
    "e": ("joint3", -1.0),
    "4": ("joint4", 1.0),
    "r": ("joint4", -1.0),
    "5": ("joint5", 1.0),
    "t": ("joint5", -1.0),
    "6": ("joint6", 1.0),
    "y": ("joint6", -1.0),
}


@dataclass(frozen=True)
class KeyboardCommand:
    joint_name: str
    direction: float


class KeyboardCommandMapper:
    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        key_bindings: dict[str, tuple[str, float]] | None = None,
    ) -> None:
        self._joint_names = set(joint_names)
        self._bindings = key_bindings or DEFAULT_KEY_BINDINGS

    def command_for_key(self, key: str) -> KeyboardCommand | None:
        binding = self._bindings.get(key)
        if binding is None:
            return None
        joint_name, direction = binding
        if joint_name not in self._joint_names:
            return None
        return KeyboardCommand(joint_name=joint_name, direction=float(direction))


@dataclass(frozen=True)
class TeleopTarget:
    accepted: bool
    message: str
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]


@dataclass(frozen=True)
class WebKeyboardCommandDecision:
    accepted: bool
    message: str
    key: str = ""
    joint_name: str = ""
    joint_names: tuple[str, ...] = ()
    positions: tuple[float, ...] = ()
    step_rad: float = 0.0
    duration: float = 0.0
    max_joint_speed_rad_s: float = 0.0


class TeleopTargetPlanner:
    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        joint_step_rad: float,
    ) -> None:
        self._joint_names = joint_names
        self._joint_limits = joint_limits
        self._joint_step_rad = abs(float(joint_step_rad))

    def apply_delta(
        self,
        *,
        current_positions: dict[str, float],
        joint_name: str,
        direction: float,
    ) -> TeleopTarget:
        if joint_name not in self._joint_names:
            return TeleopTarget(
                accepted=False,
                message=f"unknown joint: {joint_name}",
                joint_names=self._joint_names,
                positions=self._ordered_positions(current_positions),
            )

        positions = list(self._ordered_positions(current_positions))
        index = self._joint_names.index(joint_name)
        lower, upper = self._joint_limits[joint_name]
        proposed = positions[index] + (float(direction) * self._joint_step_rad)
        clamped = min(max(proposed, lower), upper)
        positions[index] = clamped
        message = "teleop target ready"
        if clamped != proposed:
            message = f"teleop target clamped for {joint_name}"
        return TeleopTarget(
            accepted=True,
            message=message,
            joint_names=self._joint_names,
            positions=tuple(positions),
        )

    def _ordered_positions(self, current_positions: dict[str, float]) -> tuple[float, ...]:
        return tuple(float(current_positions.get(name, 0.0)) for name in self._joint_names)


def _float_from_payload(
    payload: dict[str, Any],
    key: str,
    default: float,
    *,
    minimum: float,
    maximum: float,
) -> float | None:
    raw = payload.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return min(max(value, float(minimum)), float(maximum))


def validate_web_keyboard_command(
    payload: dict[str, Any],
    *,
    enabled: bool,
    joint_names: tuple[str, ...],
    current_positions: dict[str, float],
    joint_limits: dict[str, tuple[float, float]],
    default_step_rad: float,
    min_step_rad: float,
    max_step_rad: float,
    default_duration: float,
    min_duration: float,
    max_duration: float,
    joint_velocity_limits: dict[str, float] | None = None,
    max_joint_speed_rad_s: float | None = None,
) -> WebKeyboardCommandDecision:
    if not enabled:
        return WebKeyboardCommandDecision(False, "web keyboard teleop disabled")
    if str(payload.get("confirm", "")).strip().upper() != "KEYBOARD_TELEOP":
        return WebKeyboardCommandDecision(False, "missing KEYBOARD_TELEOP confirmation")

    key = str(payload.get("key", ""))
    mapper = KeyboardCommandMapper(joint_names=joint_names)
    command = mapper.command_for_key(key)
    if command is None:
        return WebKeyboardCommandDecision(False, f"unmapped keyboard key: {key}")

    missing_current = [name for name in joint_names if name not in current_positions]
    if missing_current:
        return WebKeyboardCommandDecision(False, f"missing live joint state: {', '.join(missing_current)}")

    step_rad = _float_from_payload(
        payload,
        "step_rad",
        default_step_rad,
        minimum=min_step_rad,
        maximum=max_step_rad,
    )
    if step_rad is None or step_rad <= 0.0:
        return WebKeyboardCommandDecision(False, "invalid step_rad")
    duration = _float_from_payload(
        payload,
        "duration",
        default_duration,
        minimum=min_duration,
        maximum=max_duration,
    )
    if duration is None or duration <= 0.0:
        return WebKeyboardCommandDecision(False, "invalid duration")

    requested_speed_limit = payload.get("max_joint_speed_rad_s", max_joint_speed_rad_s)
    speed_limit = float(max_joint_speed_rad_s) if max_joint_speed_rad_s is not None else None
    if requested_speed_limit is not None:
        try:
            requested_speed_value = float(requested_speed_limit)
        except (TypeError, ValueError):
            return WebKeyboardCommandDecision(False, "invalid max_joint_speed_rad_s")
        if not math.isfinite(requested_speed_value) or requested_speed_value <= 0.0:
            return WebKeyboardCommandDecision(False, "invalid max_joint_speed_rad_s")
        speed_limit = requested_speed_value if speed_limit is None else min(speed_limit, requested_speed_value)

    planner = TeleopTargetPlanner(
        joint_names=joint_names,
        joint_limits=joint_limits,
        joint_step_rad=step_rad,
    )
    target = planner.apply_delta(
        current_positions=current_positions,
        joint_name=command.joint_name,
        direction=command.direction,
    )
    if not target.accepted:
        return WebKeyboardCommandDecision(False, target.message)

    index = joint_names.index(command.joint_name)
    actual_delta = abs(float(target.positions[index]) - float(current_positions[command.joint_name]))
    if speed_limit is not None:
        configured_limit = None
        if joint_velocity_limits is not None and command.joint_name in joint_velocity_limits:
            configured_limit = float(joint_velocity_limits[command.joint_name])
        joint_limit = speed_limit if configured_limit is None else min(speed_limit, configured_limit)
        required_speed = actual_delta / max(duration, 1e-9)
        if required_speed > joint_limit:
            min_duration_needed = actual_delta / max(joint_limit, 1e-9)
            return WebKeyboardCommandDecision(
                False,
                (
                    f"{command.joint_name} speed too high: {required_speed:.4f} rad/s > "
                    f"{joint_limit:.4f} rad/s; use duration >= {min_duration_needed:.2f}s"
                ),
            )

    return WebKeyboardCommandDecision(
        True,
        f"web keyboard target accepted: {key} -> {command.joint_name}",
        key=key,
        joint_name=command.joint_name,
        joint_names=target.joint_names,
        positions=target.positions,
        step_rad=step_rad,
        duration=duration,
        max_joint_speed_rad_s=float(speed_limit) if speed_limit is not None else 0.0,
    )

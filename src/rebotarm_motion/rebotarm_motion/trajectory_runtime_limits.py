from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class JointRuntimeLimit:
    """Runtime limits used by a trajectory execution guard.

    Values are absolute magnitudes.  ``None`` disables an individual check;
    this is useful for joints whose upstream contract does not define a
    planner-side acceleration or jerk limit.
    """

    max_effort: float | None = None
    max_velocity: float | None = None
    max_acceleration: float | None = None
    max_jerk: float | None = None

    def __post_init__(self) -> None:
        for field_name in ("max_effort", "max_velocity", "max_acceleration", "max_jerk"):
            value = getattr(self, field_name)
            if value is None:
                continue
            value = float(value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and non-negative")
            object.__setattr__(self, field_name, value)


@dataclass(frozen=True)
class RuntimeLimitViolation:
    kind: str
    joint: str
    value: float
    limit: float


class TrajectoryRuntimeLimitGuard:
    """Check measured effort/velocity and finite-difference acceleration/jerk."""

    def __init__(self, limits: Mapping[str, JointRuntimeLimit]) -> None:
        self._limits = {str(name): limit for name, limit in limits.items()}
        if not self._limits:
            raise ValueError("at least one joint runtime limit is required")
        if any(not isinstance(limit, JointRuntimeLimit) for limit in self._limits.values()):
            raise TypeError("limits must contain JointRuntimeLimit values")
        self.reset()

    def reset(self) -> None:
        self._previous_elapsed: float | None = None
        self._previous_velocities: dict[str, float] = {}
        self._previous_accelerations: dict[str, float] = {}

    def observe(
        self,
        *,
        joint_names: Sequence[str],
        velocities: Sequence[float],
        efforts: Sequence[float],
        elapsed: float,
    ) -> RuntimeLimitViolation | None:
        names = [str(name) for name in joint_names]
        if len(names) != len(velocities) or len(names) != len(efforts):
            raise ValueError("joint_names, velocities, and efforts must have equal lengths")
        unknown = [name for name in names if name not in self._limits]
        if unknown:
            raise ValueError(f"unknown joint in runtime limits: {unknown[0]}")
        current_time = float(elapsed)
        if not math.isfinite(current_time):
            raise ValueError("elapsed must be finite")
        if self._previous_elapsed is not None and current_time < self._previous_elapsed:
            raise ValueError("elapsed time must be monotonic")
        dt = None if self._previous_elapsed is None else current_time - self._previous_elapsed
        if dt is not None and dt <= 0.0:
            raise ValueError("elapsed time must advance between runtime samples")

        current_velocities = {name: _finite(value, "velocity") for name, value in zip(names, velocities)}
        current_efforts = {name: _finite(value, "effort") for name, value in zip(names, efforts)}

        for name in names:
            limit = self._limits[name]
            violation = _check_absolute("effort", name, current_efforts[name], limit.max_effort)
            if violation is not None:
                self._remember(current_time, current_velocities)
                return violation
            violation = _check_absolute("velocity", name, current_velocities[name], limit.max_velocity)
            if violation is not None:
                self._remember(current_time, current_velocities)
                return violation

        if dt is not None:
            for name in names:
                acceleration = (current_velocities[name] - self._previous_velocities[name]) / dt
                limit = self._limits[name]
                violation = _check_absolute("acceleration", name, acceleration, limit.max_acceleration)
                if violation is not None:
                    self._remember(current_time, current_velocities, {name: acceleration})
                    return violation
                if name in self._previous_accelerations:
                    jerk = (acceleration - self._previous_accelerations[name]) / dt
                    violation = _check_absolute("jerk", name, jerk, limit.max_jerk)
                    if violation is not None:
                        self._remember(current_time, current_velocities, {name: acceleration})
                        return violation
                self._previous_accelerations[name] = acceleration

        self._remember(current_time, current_velocities)
        return None

    def _remember(
        self,
        elapsed: float,
        velocities: Mapping[str, float],
        accelerations: Mapping[str, float] | None = None,
    ) -> None:
        self._previous_elapsed = elapsed
        self._previous_velocities = dict(velocities)
        if accelerations:
            self._previous_accelerations.update(accelerations)


def load_joint_runtime_limits(
    path: Path,
    joint_names: Sequence[str],
) -> dict[str, JointRuntimeLimit]:
    """Load planner-side velocity, acceleration, and jerk limits from MoveIt YAML."""
    import yaml

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    entries = payload.get("joint_limits", {})
    result: dict[str, JointRuntimeLimit] = {}
    for raw_name in joint_names:
        name = str(raw_name)
        if name not in entries:
            raise ValueError(f"joint limit missing from MoveIt YAML: {name}")
        entry = entries[name] or {}
        result[name] = JointRuntimeLimit(
            max_velocity=_enabled_limit(entry, "velocity"),
            max_acceleration=_enabled_limit(entry, "acceleration"),
            max_jerk=_enabled_limit(entry, "jerk"),
        )
    return result


def _finite(value: float, label: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be finite")
    return converted


def _enabled_limit(entry: Mapping[str, object], kind: str) -> float | None:
    if not bool(entry.get(f"has_{kind}_limits", False)):
        return None
    key = f"max_{kind}"
    if key not in entry:
        raise ValueError(f"{key} is required when has_{kind}_limits is true")
    value = float(entry[key])
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{key} must be finite and non-negative")
    return value


def _check_absolute(
    kind: str,
    joint: str,
    value: float,
    limit: float | None,
) -> RuntimeLimitViolation | None:
    if limit is None or abs(value) <= limit:
        return None
    return RuntimeLimitViolation(kind=kind, joint=joint, value=abs(value), limit=limit)

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping


@dataclass(frozen=True)
class HardwareAcceptanceGuardConfig:
    raw_warning_rad_s: float = 0.07
    raw_sustained_rad_s: float = 0.15
    raw_sustained_sec: float = 0.20
    raw_immediate_rad_s: float = 0.30
    raw_immediate_window_rad_s: float = 0.10
    raw_absolute_immediate_rad_s: float = 0.40
    window_warning_rad_s: float = 0.05
    window_sustained_rad_s: float = 0.10
    window_sustained_sec: float = 0.20
    window_immediate_rad_s: float = 0.20
    tracking_sustained_rad: float = 0.05
    tracking_sustained_sec: float = 0.50
    tracking_immediate_rad: float = 0.10
    effort_sustained_fraction: float = 0.85
    effort_sustained_sec: float = 0.50

    def __post_init__(self) -> None:
        positive = (
            "raw_warning_rad_s",
            "raw_sustained_rad_s",
            "raw_sustained_sec",
            "raw_immediate_rad_s",
            "raw_immediate_window_rad_s",
            "raw_absolute_immediate_rad_s",
            "window_warning_rad_s",
            "window_sustained_rad_s",
            "window_sustained_sec",
            "window_immediate_rad_s",
            "tracking_sustained_rad",
            "tracking_sustained_sec",
            "tracking_immediate_rad",
            "effort_sustained_fraction",
            "effort_sustained_sec",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, value)
        if self.raw_warning_rad_s > self.raw_sustained_rad_s:
            raise ValueError("raw warning threshold must not exceed sustained threshold")
        if self.raw_sustained_rad_s > self.raw_immediate_rad_s:
            raise ValueError("raw sustained threshold must not exceed immediate threshold")
        if self.raw_immediate_rad_s > self.raw_absolute_immediate_rad_s:
            raise ValueError("raw immediate threshold must not exceed absolute threshold")
        if self.window_warning_rad_s > self.window_sustained_rad_s:
            raise ValueError("window warning threshold must not exceed sustained threshold")
        if self.window_sustained_rad_s > self.window_immediate_rad_s:
            raise ValueError("window sustained threshold must not exceed immediate threshold")
        if self.raw_immediate_window_rad_s > self.window_immediate_rad_s:
            raise ValueError("raw corroboration window threshold must not exceed window immediate")
        if self.tracking_sustained_rad > self.tracking_immediate_rad:
            raise ValueError("tracking sustained threshold must not exceed immediate threshold")
        if self.effort_sustained_fraction > 1.0:
            raise ValueError("effort_sustained_fraction must not exceed 1.0")


@dataclass(frozen=True)
class HardwareAcceptanceDecision:
    should_stop: bool
    reason: str = "ok"
    joint: str = ""
    value: float = 0.0
    limit: float = 0.0
    duration_sec: float = 0.0
    warnings: tuple[dict[str, float | str], ...] = field(default_factory=tuple)


class HardwareAcceptanceGuard:
    """Corroborated real-hardware guard for a supervised trajectory trial.

    Quantized raw velocity is retained as an early warning signal. A raw sample
    above the fast threshold requires corroborating physical-window velocity;
    a separate higher absolute raw threshold still stops immediately. Sustained
    raw/window velocity, following error, and effort remain independent gates.
    """

    def __init__(
        self,
        effort_limits_by_joint: Mapping[str, float],
        config: HardwareAcceptanceGuardConfig | None = None,
    ) -> None:
        if not effort_limits_by_joint:
            raise ValueError("at least one joint effort limit is required")
        self.effort_limits = {
            str(name): self._positive_finite(value, f"{name} effort limit")
            for name, value in effort_limits_by_joint.items()
        }
        self.config = config or HardwareAcceptanceGuardConfig()
        self.reset()

    def reset(self) -> None:
        self._last_time: float | None = None
        self._raw_since: dict[str, float] = {}
        self._window_since: dict[str, float] = {}
        self._tracking_since: dict[str, float] = {}
        self._effort_since: dict[str, float] = {}

    def observe(
        self,
        *,
        now: float,
        raw_velocities_by_joint: Mapping[str, float],
        window_velocities_by_joint: Mapping[str, float],
        efforts_by_joint: Mapping[str, float],
        tracking_errors_by_joint: Mapping[str, float],
    ) -> HardwareAcceptanceDecision:
        timestamp = float(now)
        if not math.isfinite(timestamp):
            raise ValueError("now must be finite")
        if self._last_time is not None and timestamp < self._last_time:
            raise ValueError("now must be monotonic")
        self._last_time = timestamp
        self._require_joint_coverage(raw_velocities_by_joint, "raw velocity")
        self._require_joint_coverage(window_velocities_by_joint, "window velocity")
        self._require_joint_coverage(efforts_by_joint, "effort")
        self._require_joint_coverage(tracking_errors_by_joint, "tracking error")

        warnings: list[dict[str, float | str]] = []
        for joint in self.effort_limits:
            raw = abs(self._finite(raw_velocities_by_joint[joint], f"{joint} raw velocity"))
            window = abs(
                self._finite(window_velocities_by_joint[joint], f"{joint} window velocity")
            )
            effort = abs(self._finite(efforts_by_joint[joint], f"{joint} effort"))
            tracking = abs(
                self._finite(tracking_errors_by_joint[joint], f"{joint} tracking error")
            )
            effort_limit = self.effort_limits[joint]

            if raw > self.config.raw_warning_rad_s:
                warnings.append(
                    {
                        "kind": "raw_velocity",
                        "joint": joint,
                        "value": raw,
                        "limit": self.config.raw_warning_rad_s,
                    }
                )
            if window > self.config.window_warning_rad_s:
                warnings.append(
                    {
                        "kind": "window_velocity",
                        "joint": joint,
                        "value": window,
                        "limit": self.config.window_warning_rad_s,
                    }
                )
            immediate = self._immediate_decision(
                joint=joint,
                raw=raw,
                window=window,
                effort=effort,
                effort_limit=effort_limit,
                tracking=tracking,
                warnings=warnings,
            )
            if immediate is not None:
                return immediate

            sustained = self._sustained_decision(
                now=timestamp,
                joint=joint,
                raw=raw,
                window=window,
                effort=effort,
                effort_limit=effort_limit,
                tracking=tracking,
                warnings=warnings,
            )
            if sustained is not None:
                return sustained

        return HardwareAcceptanceDecision(False, warnings=tuple(warnings))

    def _immediate_decision(
        self,
        *,
        joint: str,
        raw: float,
        window: float,
        effort: float,
        effort_limit: float,
        tracking: float,
        warnings: list[dict[str, float | str]],
    ) -> HardwareAcceptanceDecision | None:
        if raw > self.config.raw_absolute_immediate_rad_s:
            return HardwareAcceptanceDecision(
                True,
                "raw_velocity_absolute_immediate",
                joint,
                raw,
                self.config.raw_absolute_immediate_rad_s,
                warnings=tuple(warnings),
            )
        if (
            raw > self.config.raw_immediate_rad_s
            and window > self.config.raw_immediate_window_rad_s
        ):
            return HardwareAcceptanceDecision(
                True,
                "raw_velocity_corroborated_immediate",
                joint,
                raw,
                self.config.raw_immediate_rad_s,
                warnings=tuple(warnings),
            )
        if window > self.config.window_immediate_rad_s:
            return HardwareAcceptanceDecision(
                True,
                "window_velocity_immediate",
                joint,
                window,
                self.config.window_immediate_rad_s,
                warnings=tuple(warnings),
            )
        if tracking > self.config.tracking_immediate_rad:
            return HardwareAcceptanceDecision(
                True,
                "tracking_error_immediate",
                joint,
                tracking,
                self.config.tracking_immediate_rad,
                warnings=tuple(warnings),
            )
        if effort > effort_limit:
            return HardwareAcceptanceDecision(
                True,
                "effort_immediate",
                joint,
                effort,
                effort_limit,
                warnings=tuple(warnings),
            )
        return None

    def _sustained_decision(
        self,
        *,
        now: float,
        joint: str,
        raw: float,
        window: float,
        effort: float,
        effort_limit: float,
        tracking: float,
        warnings: list[dict[str, float | str]],
    ) -> HardwareAcceptanceDecision | None:
        checks = (
            (
                self._raw_since,
                raw,
                self.config.raw_sustained_rad_s,
                self.config.raw_sustained_sec,
                "raw_velocity_sustained",
            ),
            (
                self._window_since,
                window,
                self.config.window_sustained_rad_s,
                self.config.window_sustained_sec,
                "window_velocity_sustained",
            ),
            (
                self._tracking_since,
                tracking,
                self.config.tracking_sustained_rad,
                self.config.tracking_sustained_sec,
                "tracking_error_sustained",
            ),
            (
                self._effort_since,
                effort,
                effort_limit * self.config.effort_sustained_fraction,
                self.config.effort_sustained_sec,
                "effort_sustained",
            ),
        )
        for active_since, value, limit, required_duration, reason in checks:
            if value > limit:
                started = active_since.setdefault(joint, now)
                duration = now - started
                if duration + 1e-9 >= required_duration:
                    return HardwareAcceptanceDecision(
                        True,
                        reason,
                        joint,
                        value,
                        limit,
                        duration,
                        tuple(warnings),
                    )
            else:
                active_since.pop(joint, None)
        return None

    def _require_joint_coverage(self, values: Mapping[str, float], label: str) -> None:
        missing = [joint for joint in self.effort_limits if joint not in values]
        if missing:
            raise ValueError(f"{label} missing joints: {missing}")

    @staticmethod
    def _finite(value: float, label: str) -> float:
        converted = float(value)
        if not math.isfinite(converted):
            raise ValueError(f"{label} must be finite")
        return converted

    @classmethod
    def _positive_finite(cls, value: float, label: str) -> float:
        converted = cls._finite(value, label)
        if converted <= 0.0:
            raise ValueError(f"{label} must be positive")
        return converted

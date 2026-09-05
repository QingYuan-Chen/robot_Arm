from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


class TargetViolation(ValueError):
    """原始跟随目标违反从臂网页位置边界。"""


@dataclass(frozen=True)
class ShapedCommand:
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]


def _six_finite(values: Sequence[float], label: str) -> np.ndarray:
    array = np.asarray(tuple(values), dtype=np.float64)
    if array.shape != (6,):
        raise ValueError(f"{label}必须包含六个关节")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label}必须全部为有限数值")
    return array


class PointToPointTrajectory:
    """从静止起点到静止终点的同步五次多项式轨迹。"""

    _MAX_NORMALIZED_SPEED = 1.875
    _MAX_NORMALIZED_ACCELERATION = 10.0 * math.sqrt(3.0) / 3.0
    _MAX_NORMALIZED_JERK = 60.0

    def __init__(
        self,
        *,
        start_rad: Sequence[float],
        target_rad: Sequence[float],
        max_speed_rad_s: float,
        max_acceleration_rad_s2: float,
        max_jerk_rad_s3: float,
    ) -> None:
        self._start = _six_finite(start_rad, "轨迹起点")
        self._target = _six_finite(target_rad, "轨迹终点")
        self._delta = self._target - self._start
        constraints = (
            (max_speed_rad_s, "max_speed_rad_s"),
            (max_acceleration_rad_s2, "max_acceleration_rad_s2"),
            (max_jerk_rad_s3, "max_jerk_rad_s3"),
        )
        for value, name in constraints:
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} 必须大于零且为有限数值")

        distance = float(np.max(np.abs(self._delta)))
        if distance == 0.0:
            self._duration = 0.0
        else:
            self._duration = max(
                self._MAX_NORMALIZED_SPEED * distance / float(max_speed_rad_s),
                math.sqrt(
                    self._MAX_NORMALIZED_ACCELERATION
                    * distance
                    / float(max_acceleration_rad_s2)
                ),
                (
                    self._MAX_NORMALIZED_JERK
                    * distance
                    / float(max_jerk_rad_s3)
                )
                ** (1.0 / 3.0),
            )

    @property
    def duration_s(self) -> float:
        return self._duration

    def sample(self, elapsed_s: float) -> ShapedCommand:
        elapsed = float(elapsed_s)
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError("elapsed_s 必须为非负有限数值")
        if self._duration == 0.0 or elapsed >= self._duration:
            zeros = (0.0,) * 6
            return ShapedCommand(
                position_rad=tuple(float(value) for value in self._target),
                velocity_rad_s=zeros,
                acceleration_rad_s2=zeros,
            )
        if elapsed == 0.0:
            zeros = (0.0,) * 6
            return ShapedCommand(
                position_rad=tuple(float(value) for value in self._start),
                velocity_rad_s=zeros,
                acceleration_rad_s2=zeros,
            )

        u = elapsed / self._duration
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        blend_velocity = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / self._duration
        blend_acceleration = (
            60.0 * u - 180.0 * u2 + 120.0 * u3
        ) / (self._duration * self._duration)
        position = self._start + self._delta * blend
        velocity = self._delta * blend_velocity
        acceleration = self._delta * blend_acceleration
        return ShapedCommand(
            position_rad=tuple(float(value) for value in position),
            velocity_rad_s=tuple(float(value) for value in velocity),
            acceleration_rad_s2=tuple(float(value) for value in acceleration),
        )


class CommandShaper:
    _RESPONSE_RATE_RAD_S = 6.0

    def __init__(
        self,
        *,
        joint_limits: Sequence[tuple[float, float]],
        baseline_rad: Sequence[float],
        max_speed_rad_s: float,
        max_acceleration_rad_s2: float,
        max_jerk_rad_s3: float,
    ) -> None:
        if len(tuple(joint_limits)) != 6:
            raise ValueError("joint_limits 必须包含六个关节")
        self._limits = np.asarray(tuple(joint_limits), dtype=np.float64)
        if self._limits.shape != (6, 2) or not np.all(np.isfinite(self._limits)):
            raise ValueError("joint_limits 必须是六组有限上下限")
        if np.any(self._limits[:, 0] >= self._limits[:, 1]):
            raise ValueError("关节下限必须小于上限")
        self._baseline = _six_finite(baseline_rad, "从臂基线")
        for value, name in (
            (max_speed_rad_s, "max_speed_rad_s"),
            (max_acceleration_rad_s2, "max_acceleration_rad_s2"),
            (max_jerk_rad_s3, "max_jerk_rad_s3"),
        ):
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} 必须大于零且为有限数值")
        self._max_speed = float(max_speed_rad_s)
        self._max_acceleration = float(max_acceleration_rad_s2)
        self._max_jerk = float(max_jerk_rad_s3)
        if np.any(self._baseline < self._limits[:, 0]) or np.any(
            self._baseline > self._limits[:, 1]
        ):
            raise ValueError("从臂基线超出真实关节限位")
        self._allowed_lower = self._limits[:, 0].copy()
        self._allowed_upper = self._limits[:, 1].copy()
        self._position = self._baseline.copy()
        self._velocity = np.zeros(6, dtype=np.float64)
        self._acceleration = np.zeros(6, dtype=np.float64)

    def reset(self, position_rad: Sequence[float]) -> None:
        position = _six_finite(position_rad, "重置位置")
        if np.any(position < self._limits[:, 0]) or np.any(position > self._limits[:, 1]):
            raise ValueError("重置位置超出真实关节限位")
        self._position = position.copy()
        self._velocity.fill(0.0)
        self._acceleration.fill(0.0)

    def _validate_target(self, raw_target_rad: Sequence[float]) -> np.ndarray:
        try:
            target = _six_finite(raw_target_rad, "原始目标")
        except ValueError as exc:
            raise TargetViolation(str(exc)) from exc
        for index, (value, lower, upper) in enumerate(
            zip(
                target,
                self._allowed_lower,
                self._allowed_upper,
                strict=True,
            ),
            start=1,
        ):
            if value < lower or value > upper:
                raise TargetViolation(f"joint{index} 原始目标超出安全位置边界")
        return target

    def point_to_point(
        self,
        raw_target_rad: Sequence[float],
    ) -> PointToPointTrajectory:
        target = self._validate_target(raw_target_rad)
        return PointToPointTrajectory(
            start_rad=self._position,
            target_rad=target,
            max_speed_rad_s=self._max_speed,
            max_acceleration_rad_s2=self._max_acceleration,
            max_jerk_rad_s3=self._max_jerk,
        )

    def step(self, raw_target_rad: Sequence[float], dt_s: float) -> ShapedCommand:
        dt = float(dt_s)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_s 必须大于零且为有限数值")
        target = self._validate_target(raw_target_rad)
        error = target - self._position
        response_rate = self._RESPONSE_RATE_RAD_S
        desired_acceleration = np.clip(
            response_rate * response_rate * error
            - 2.0 * response_rate * self._velocity,
            -self._max_acceleration,
            self._max_acceleration,
        )
        acceleration_delta = np.clip(
            desired_acceleration - self._acceleration,
            -self._max_jerk * dt,
            self._max_jerk * dt,
        )
        acceleration = np.clip(
            self._acceleration + acceleration_delta,
            -self._max_acceleration,
            self._max_acceleration,
        )
        velocity = np.clip(
            self._velocity + acceleration * dt,
            -self._max_speed,
            self._max_speed,
        )
        position = self._position + velocity * dt
        position = np.clip(position, self._allowed_lower, self._allowed_upper)
        blocked_at_lower = (position <= self._allowed_lower) & (
            (velocity < 0.0) | (acceleration < 0.0)
        )
        blocked_at_upper = (position >= self._allowed_upper) & (
            (velocity > 0.0) | (acceleration > 0.0)
        )
        blocked = blocked_at_lower | blocked_at_upper
        velocity = np.where(blocked, 0.0, velocity)
        acceleration = np.where(blocked, 0.0, acceleration)

        self._position = position
        self._velocity = velocity
        self._acceleration = acceleration
        return ShapedCommand(
            position_rad=tuple(float(value) for value in position),
            velocity_rad_s=tuple(float(value) for value in velocity),
            acceleration_rad_s2=tuple(float(value) for value in acceleration),
        )

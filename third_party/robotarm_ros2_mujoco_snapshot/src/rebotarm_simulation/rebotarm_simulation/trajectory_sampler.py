"""ROS-independent named joint trajectory normalization and sampling."""

from __future__ import annotations

from bisect import bisect_right
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


@dataclass(frozen=True)
class NamedTrajectoryPoint:
    """One named-trajectory point before canonical joint normalization."""

    time_from_start: float
    positions: tuple[float, ...]

    def __init__(self, time_from_start: float, positions: Sequence[float]) -> None:
        object.__setattr__(self, "time_from_start", time_from_start)
        object.__setattr__(self, "positions", tuple(positions))


class TrajectorySampler:
    """Normalize named arm targets and linearly sample them by elapsed sim time."""

    def __init__(
        self,
        joint_names: Sequence[str],
        points: Sequence[NamedTrajectoryPoint],
        *,
        initial_positions: Sequence[float] | Mapping[str, float] | None = None,
    ) -> None:
        names = tuple(joint_names)
        raw_points = tuple(points)
        self._validate_names(names)
        if not raw_points:
            raise ValueError("trajectory must contain at least one point")

        omitted = set(ARM_JOINT_NAMES).difference(names)
        if omitted and initial_positions is None:
            raise ValueError("partial trajectories require canonical initial positions")
        initial = self._normalize_initial(initial_positions)

        normalized_points: list[tuple[float, tuple[float, ...]]] = []
        previous_time = -math.inf
        indexes = tuple(ARM_JOINT_NAMES.index(name) for name in names)
        for point in raw_points:
            if not isinstance(point, NamedTrajectoryPoint):
                raise TypeError("trajectory points must be NamedTrajectoryPoint records")
            time = float(point.time_from_start)
            if not math.isfinite(time) or time < 0.0:
                raise ValueError("point times must be finite and non-negative")
            if time <= previous_time:
                raise ValueError("point times must be strictly increasing")
            if len(point.positions) != len(names):
                raise ValueError("point position count must match joint names")

            values = tuple(float(value) for value in point.positions)
            if any(not math.isfinite(value) for value in values):
                raise ValueError("point positions must be finite")
            canonical = list(initial)
            for index, value in zip(indexes, values):
                canonical[index] = value
            normalized_points.append((time, tuple(canonical)))
            previous_time = time

        self._initial = initial
        self._times = tuple(time for time, _ in normalized_points)
        self._positions = tuple(positions for _, positions in normalized_points)
        self._cancelled = False

    @staticmethod
    def _validate_names(names: tuple[str, ...]) -> None:
        if not names:
            raise ValueError("joint names must not be empty")
        if any(not isinstance(name, str) or not name for name in names):
            raise ValueError("joint names must be non-empty strings")
        if len(set(names)) != len(names):
            raise ValueError("duplicate joint names are not allowed")
        unknown = set(names).difference(ARM_JOINT_NAMES)
        if unknown:
            raise ValueError(f"unknown arm joint names: {sorted(unknown)}")

    @staticmethod
    def _normalize_initial(
        positions: Sequence[float] | Mapping[str, float] | None,
    ) -> tuple[float, ...]:
        if positions is None:
            values = (0.0,) * len(ARM_JOINT_NAMES)
        elif isinstance(positions, Mapping):
            unknown = set(positions).difference(ARM_JOINT_NAMES)
            missing = set(ARM_JOINT_NAMES).difference(positions)
            if unknown or missing:
                raise ValueError("initial positions must contain exactly the canonical arm joints")
            values = tuple(float(positions[name]) for name in ARM_JOINT_NAMES)
        else:
            if isinstance(positions, (str, bytes)) or len(positions) != len(ARM_JOINT_NAMES):
                raise ValueError("initial positions must contain six canonical arm values")
            values = tuple(float(value) for value in positions)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("initial positions must be finite")
        return values

    @property
    def joint_names(self) -> tuple[str, ...]:
        return ARM_JOINT_NAMES

    @property
    def duration(self) -> float:
        return self._times[-1]

    @property
    def is_cancelled(self) -> bool:
        return self._cancelled

    def is_complete(self, simulation_time: float) -> bool:
        time = self._validate_sample_time(simulation_time)
        return time >= self.duration

    def cancel(self) -> None:
        self._cancelled = True

    def clear_cancel(self) -> None:
        self._cancelled = False

    def reset(self) -> None:
        self.clear_cancel()

    def sample(self, simulation_time: float) -> tuple[float, ...]:
        time = self._validate_sample_time(simulation_time)
        if self._cancelled:
            raise RuntimeError("trajectory sampling was cancelled")
        if time >= self.duration:
            return self._positions[-1]

        upper = bisect_right(self._times, time)
        if upper == 0:
            lower_time = 0.0
            lower_positions = self._initial
        else:
            lower_time = self._times[upper - 1]
            lower_positions = self._positions[upper - 1]
            if lower_time == time:
                return lower_positions

        upper_time = self._times[upper]
        upper_positions = self._positions[upper]
        if upper_time == lower_time:
            return upper_positions
        fraction = (time - lower_time) / (upper_time - lower_time)
        return tuple(
            start + fraction * (end - start)
            for start, end in zip(lower_positions, upper_positions)
        )

    @staticmethod
    def _validate_sample_time(simulation_time: float) -> float:
        time = float(simulation_time)
        if not math.isfinite(time) or time < 0.0:
            raise ValueError("simulation time must be finite and non-negative")
        return time

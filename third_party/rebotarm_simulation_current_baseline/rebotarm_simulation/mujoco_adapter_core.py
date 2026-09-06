from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import bisect
import math
from typing import Iterable, Sequence


ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass(frozen=True)
class TrajectoryPoint:
    time_from_start: float
    positions: list[float]


@dataclass(frozen=True)
class JointStateSnapshot:
    names: list[str]
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


@dataclass(frozen=True)
class ToleranceViolation:
    joint: str
    error: float
    abs_error: float


def interpolate_trajectory(points: Sequence[TrajectoryPoint], elapsed: float) -> list[float]:
    if not points:
        raise ValueError("trajectory contains no points")
    if elapsed <= points[0].time_from_start:
        return list(points[0].positions)
    if elapsed >= points[-1].time_from_start:
        return list(points[-1].positions)

    times = [point.time_from_start for point in points]
    right = bisect.bisect_left(times, elapsed)
    left = max(0, right - 1)
    t0 = points[left].time_from_start
    t1 = points[right].time_from_start
    ratio = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
    return [
        float(p0) + (float(p1) - float(p0)) * ratio
        for p0, p1 in zip(points[left].positions, points[right].positions)
    ]


def normalize_trajectory_points(
    *,
    source_joint_names: Sequence[str],
    target_joint_names: Sequence[str],
    points: Iterable[object],
    current_positions: Sequence[float],
) -> list[TrajectoryPoint]:
    source_names = [str(name) for name in source_joint_names]
    target_names = [str(name) for name in target_joint_names]
    supported = [name for name in source_names if name in target_names]
    if not supported:
        raise ValueError("trajectory contains no supported joints")
    source_index = {name: index for index, name in enumerate(source_names)}
    target_current = {
        name: float(current_positions[index])
        for index, name in enumerate(target_names)
        if index < len(current_positions)
    }

    normalized: list[TrajectoryPoint] = []
    last_time = -1e-6
    for raw_point in points:
        positions = _positions_from_point(raw_point)
        target_positions: list[float] = []
        for name in target_names:
            if name in source_index and source_index[name] < len(positions):
                target_positions.append(float(positions[source_index[name]]))
            else:
                target_positions.append(float(target_current.get(name, 0.0)))
        time_from_start = max(_time_from_point(raw_point), last_time + 1e-3)
        last_time = time_from_start
        normalized.append(TrajectoryPoint(time_from_start=time_from_start, positions=target_positions))

    if not normalized:
        raise ValueError("trajectory contains no points")
    if normalized[0].time_from_start > 0.0:
        normalized.insert(
            0,
            TrajectoryPoint(
                time_from_start=0.0,
                positions=[float(target_current.get(name, 0.0)) for name in target_names],
            ),
        )
    return normalized


def gripper_width_to_ctrl(width: float, *, min_width: float = 0.0, max_width: float = 0.09) -> float:
    lower = min(float(min_width), float(max_width))
    upper = max(float(min_width), float(max_width))
    clamped = min(max(float(width), lower), upper)
    return clamped * 0.5


def consume_sim_steps(
    *,
    wall_delta: float,
    timestep: float,
    pending_sim_seconds: float,
) -> tuple[int, float]:
    if timestep <= 0.0:
        raise ValueError("MuJoCo timestep must be positive")
    pending = max(0.0, float(pending_sim_seconds)) + max(0.0, float(wall_delta))
    steps = int(pending / float(timestep))
    remainder = pending - float(steps) * float(timestep)
    return steps, remainder


def trajectory_error_code_for_stop_reason(stop_reason: str, *, result_type) -> int:
    if stop_reason == "finished":
        return int(result_type.SUCCESSFUL)
    if stop_reason == "goal_tolerance_violated":
        return int(result_type.GOAL_TOLERANCE_VIOLATED)
    return int(result_type.PATH_TOLERANCE_VIOLATED)


def execution_timeout_seconds(end_time: float, *, margin_sec: float = 2.0) -> float:
    """Return a positive wall-clock budget for a trajectory execution.

    The trajectory duration is measured in simulation seconds.  A separate
    margin leaves room for timer jitter and slower wall-clock execution while
    still bounding a stalled action callback.
    """
    duration = max(float(end_time), 0.0)
    margin = max(float(margin_sec), 0.0)
    return max(duration + margin, 1e-6)


def first_tolerance_violation(
    *,
    joint_names: Sequence[str],
    errors: Sequence[float],
    tolerance: float,
) -> ToleranceViolation | None:
    if float(tolerance) <= 0.0:
        return None
    for joint, error in zip(joint_names, errors):
        abs_error = abs(float(error))
        if abs_error > float(tolerance):
            return ToleranceViolation(joint=str(joint), error=float(error), abs_error=abs_error)
    return None


def default_step_response_targets(motor_profiles: Iterable[object]) -> dict[str, float]:
    targets: dict[str, float] = {}
    for profile in motor_profiles:
        joint = str(getattr(profile, "joint"))
        if joint == "left_finger":
            continue
        lower, upper = _parse_range(str(getattr(profile, "ctrlrange")))
        midpoint = (lower + upper) * 0.5
        target = midpoint + (upper - midpoint) * 0.5
        if upper <= 0.0:
            target = midpoint * 0.5
        targets[joint] = float(target)
    return targets


class MuJoCoArmAdapter:
    def __init__(
        self,
        model_xml: Path,
        *,
        arm_joint_names: Sequence[str] = ARM_JOINT_NAMES,
    ) -> None:
        import mujoco

        self._mujoco = mujoco
        self.model_xml = Path(model_xml).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_xml))
        self.data = mujoco.MjData(self.model)
        self.arm_joint_names = [str(name) for name in arm_joint_names]
        self._joint_ids = [self._require_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.arm_joint_names]
        self._qpos_indices = [int(self.model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids]
        self._dof_indices = [int(self.model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids]
        self._actuator_ids = [self._require_actuator_for_joint(joint_id, name) for joint_id, name in zip(self._joint_ids, self.arm_joint_names)]
        self._gripper_actuator_id = self._optional_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
        self._left_finger_joint_id = self._optional_id(mujoco.mjtObj.mjOBJ_JOINT, "left_finger")
        self._right_finger_joint_id = self._optional_id(mujoco.mjtObj.mjOBJ_JOINT, "right_finger")
        self._last_gripper_width = 0.0
        self.reset("zero")

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def sim_time(self) -> float:
        return float(self.data.time)

    def reset(self, keyframe: str | None = None) -> None:
        if keyframe:
            key_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_KEY, keyframe)
            if key_id >= 0:
                self._mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            else:
                self._mujoco.mj_resetData(self.model, self.data)
        else:
            self._mujoco.mj_resetData(self.model, self.data)
        self._mujoco.mj_forward(self.model, self.data)

    def step(self, steps: int = 1) -> None:
        for _ in range(max(1, int(steps))):
            self._mujoco.mj_step(self.model, self.data)

    def set_arm_targets(self, positions: Sequence[float]) -> None:
        for actuator_id, position in zip(self._actuator_ids, positions):
            self.data.ctrl[actuator_id] = float(position)

    def arm_positions(self) -> list[float]:
        return [float(self.data.qpos[index]) for index in self._qpos_indices]

    def arm_velocities(self) -> list[float]:
        return [float(self.data.qvel[index]) for index in self._dof_indices]

    def arm_actuator_forces(self) -> list[float]:
        return [float(self.data.actuator_force[index]) for index in self._actuator_ids]

    def arm_actuator_force_limits(self) -> list[float | None]:
        limits: list[float | None] = []
        for actuator_id in self._actuator_ids:
            if not int(self.model.actuator_forcelimited[actuator_id]):
                limits.append(None)
                continue
            lower, upper = (float(value) for value in self.model.actuator_forcerange[actuator_id])
            limits.append(max(abs(lower), abs(upper)))
        return limits

    def set_gripper_width(self, width: float, *, min_width: float = 0.0, max_width: float = 0.09) -> float:
        ctrl = gripper_width_to_ctrl(width, min_width=min_width, max_width=max_width)
        if self._gripper_actuator_id is not None:
            self.data.ctrl[self._gripper_actuator_id] = ctrl
        self._last_gripper_width = ctrl * 2.0
        return self._last_gripper_width

    def gripper_width(self) -> float:
        if self._left_finger_joint_id is None:
            return self._last_gripper_width
        qpos_index = int(self.model.jnt_qposadr[self._left_finger_joint_id])
        return max(0.0, float(self.data.qpos[qpos_index]) * 2.0)

    def joint_state_snapshot(self) -> JointStateSnapshot:
        positions = self.arm_positions()
        velocities = self.arm_velocities()
        efforts = self.arm_actuator_forces()
        return JointStateSnapshot(
            names=list(self.arm_joint_names),
            positions=positions,
            velocities=velocities,
            efforts=efforts,
        )

    def healthy(self) -> bool:
        values = list(self.data.qpos) + list(self.data.qvel)
        return all(math.isfinite(float(value)) for value in values)

    def _require_id(self, obj_type, name: str) -> int:
        obj_id = self._mujoco.mj_name2id(self.model, obj_type, name)
        if obj_id < 0:
            raise ValueError(f"MuJoCo object not found: {name}")
        return int(obj_id)

    def _optional_id(self, obj_type, name: str) -> int | None:
        obj_id = self._mujoco.mj_name2id(self.model, obj_type, name)
        return int(obj_id) if obj_id >= 0 else None

    def _require_actuator_for_joint(self, joint_id: int, name: str) -> int:
        for actuator_id in range(int(self.model.nu)):
            if (
                int(self.model.actuator_trntype[actuator_id]) == int(self._mujoco.mjtTrn.mjTRN_JOINT)
                and int(self.model.actuator_trnid[actuator_id, 0]) == int(joint_id)
            ):
                return actuator_id
        raise ValueError(f"MuJoCo actuator not found for joint: {name}")


def _positions_from_point(point: object) -> list[float]:
    if isinstance(point, dict):
        return [float(value) for value in point.get("positions", [])]
    return [float(value) for value in getattr(point, "positions", [])]


def _time_from_point(point: object) -> float:
    if isinstance(point, dict):
        return float(point.get("time_from_start", 0.0))
    duration = getattr(point, "time_from_start", None)
    if duration is None:
        return 0.0
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) * 1e-9


def _parse_range(value: str) -> tuple[float, float]:
    parts = [float(part) for part in str(value).split()]
    if len(parts) != 2:
        raise ValueError(f"expected two range values, got: {value}")
    return parts[0], parts[1]

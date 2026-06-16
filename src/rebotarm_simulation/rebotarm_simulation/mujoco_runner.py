from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time


@dataclass(frozen=True)
class SmokeResult:
    xml_path: Path
    nq: int
    nv: int
    nu: int
    finite: bool
    contacts: int
    sim_time: float


@dataclass(frozen=True)
class StepResponseResult:
    joint: str
    target: float
    final_position: float
    max_abs_error: float
    rms_error: float
    max_abs_velocity: float
    max_abs_actuator_force: float
    sim_time: float


@dataclass(frozen=True)
class GraspBenchmarkResult:
    xml_path: Path
    finite: bool
    box_height_m: float | None
    max_contacts: int
    final_contacts: int
    sim_time: float


def run_smoke(xml_path: Path, *, seconds: float = 3.0) -> SmokeResult:
    mujoco, np = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    steps = max(1, int(seconds / float(model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    return SmokeResult(
        xml_path=xml_path.resolve(),
        nq=int(model.nq),
        nv=int(model.nv),
        nu=int(model.nu),
        finite=finite,
        contacts=int(data.ncon),
        sim_time=float(data.time),
    )


def run_step_response(
    xml_path: Path,
    *,
    joint: str = "joint2",
    target: float = -0.6,
    seconds: float = 3.0,
) -> StepResponseResult:
    mujoco, np = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    _reset_keyframe_if_present(mujoco, model, data, "zero")

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    if joint_id < 0:
        raise ValueError(f"joint not found in MuJoCo model: {joint}")
    qpos_index = int(model.jnt_qposadr[joint_id])
    actuator_id = _actuator_id_for_joint(mujoco, model, joint_id)
    if actuator_id is None:
        raise ValueError(f"position actuator not found for joint: {joint}")

    data.ctrl[actuator_id] = float(target)
    steps = max(1, int(seconds / float(model.opt.timestep)))
    errors: list[float] = []
    max_abs_velocity = 0.0
    max_abs_force = 0.0
    for _ in range(steps):
        mujoco.mj_step(model, data)
        error = float(target) - float(data.qpos[qpos_index])
        errors.append(error)
        dof_index = int(model.jnt_dofadr[joint_id])
        max_abs_velocity = max(max_abs_velocity, abs(float(data.qvel[dof_index])))
        max_abs_force = max(max_abs_force, abs(float(data.actuator_force[actuator_id])))

    rms_error = math.sqrt(sum(error * error for error in errors) / float(len(errors)))
    return StepResponseResult(
        joint=joint,
        target=float(target),
        final_position=float(data.qpos[qpos_index]),
        max_abs_error=max(abs(error) for error in errors),
        rms_error=float(rms_error),
        max_abs_velocity=float(max_abs_velocity),
        max_abs_actuator_force=float(max_abs_force),
        sim_time=float(data.time),
    )


def run_grasp_benchmark(xml_path: Path, *, seconds: float = 5.0) -> GraspBenchmarkResult:
    mujoco, np = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    _reset_keyframe_if_present(mujoco, model, data, "0")

    max_contacts = 0
    steps = max(1, int(seconds / float(model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
        max_contacts = max(max_contacts, int(data.ncon))

    box_z = _body_z_position(mujoco, model, data, "box")
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    return GraspBenchmarkResult(
        xml_path=xml_path.resolve(),
        finite=finite,
        box_height_m=box_z,
        max_contacts=max_contacts,
        final_contacts=int(data.ncon),
        sim_time=float(data.time),
    )


def _load_mujoco():
    import mujoco
    import numpy as np

    return mujoco, np


def _reset_keyframe_if_present(mujoco, model, data, name: str) -> None:
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)


def _actuator_id_for_joint(mujoco, model, joint_id: int) -> int | None:
    for actuator_id in range(int(model.nu)):
        if (
            int(model.actuator_trntype[actuator_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[actuator_id, 0]) == int(joint_id)
        ):
            return actuator_id
    return None


def _body_z_position(mujoco, model, data, body_name: str) -> float | None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return None
    return float(data.xpos[body_id][2])


def benchmark_wall_time(command) -> tuple[float, object]:
    start = time.monotonic()
    result = command()
    return time.monotonic() - start, result

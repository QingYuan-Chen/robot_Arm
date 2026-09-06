from __future__ import annotations

import hashlib
import importlib
import math
import os
from pathlib import Path
import sys
from types import MappingProxyType
from typing import Mapping, Sequence

import numpy as np

from .motor_control import GripperMitController, PosVelController, load_motor_control_parameters
from .mujoco_types import ContactInfo, SavedSimulationState, SimulationState
from .sim_gripper import gripper_joint_positions_for_width
from .urdf_to_mjcf import actuator_name_for_joint


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
FINGER_JOINT_NAMES = ("left_finger_joint", "right_finger_joint")
JOINT_NAMES = ARM_JOINT_NAMES + FINGER_JOINT_NAMES
CONTROL_MODES = ("gravity_comp", "hold", "pos_vel")


def _default_scene_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    relative = Path("models/rebotarm/scene.xml")
    candidates = [package_root / relative, Path(sys.prefix) / "share/rebotarm_simulation" / relative]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share/rebotarm_simulation" / relative)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not locate reBotArm MuJoCo scene.xml; searched: {searched}")


def _finite_vector(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result


class RebotArmMujoco:
    joint_names = JOINT_NAMES

    def __init__(self, model_path: str | os.PathLike[str] | None = None) -> None:
        try:
            self._mj = importlib.import_module("mujoco")
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "MuJoCo is required. Install src/rebotarm_simulation/requirements-mujoco.txt "
                "in the active Python environment."
            ) from exc
        self.model_path = str(Path(model_path) if model_path is not None else _default_scene_path())
        self._model = self._mj.MjModel.from_xml_path(self.model_path)
        self._data = self._mj.MjData(self._model)
        self._closed = False
        self._rng = np.random.default_rng()
        motor_parameters = load_motor_control_parameters(Path(__file__).resolve().parents[3])
        self._motor_parameters = motor_parameters
        self._arm_controller = PosVelController(motor_parameters.arm)
        self._gripper_controller = GripperMitController(motor_parameters.gripper)
        self._control_steps_per_update = max(
            1, int(round((1.0 / motor_parameters.control_rate_hz) / float(self._model.opt.timestep)))
        )
        self._control_phase = 0
        self._position_targets = np.zeros(len(JOINT_NAMES), dtype=float)
        self._control_mode = "pos_vel"

        self._joint_ids = tuple(self._name_id(self._mj.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES)
        self._actuator_ids = tuple(
            self._name_id(self._mj.mjtObj.mjOBJ_ACTUATOR, actuator_name_for_joint(name))
            for name in JOINT_NAMES
        )
        self._ee_site_id = self._name_id(self._mj.mjtObj.mjOBJ_SITE, "ee_site")
        self._free_bodies = self._find_free_bodies()
        # State checkpoint scope is explicit even on MuJoCo releases where
        # mjSTATE_INTEGRATION already aggregates CTRL and USER. This preserves
        # commanded controls, applied forces, mocap/userdata/equality state,
        # plugin state, and the solver integration/warm-start state.
        self._state_spec = (
            int(self._mj.mjtState.mjSTATE_INTEGRATION)
            | int(self._mj.mjtState.mjSTATE_USER)
            | int(self._mj.mjtState.mjSTATE_CTRL)
        )
        self._model_dimensions = tuple(int(value) for value in (
            self._model.nq, self._model.nv, self._model.na, self._model.nu,
            self._model.nbody, self._model.njnt, self._model.ngeom,
            self._mj.mj_stateSize(self._model, self._state_spec),
        ))
        self._model_fingerprint = self._fingerprint_model()
        self.reset()

    @property
    def timestep(self) -> float:
        self._ensure_open()
        return float(self._model.opt.timestep)

    def _unsafe_viewer_handles(self):
        """Return mutable native handles for the package's viewer adapter.

        The returned MuJoCo objects are only valid while this simulation is
        open.  Callers must not retain them or mutate physics outside the
        simulation's owning thread.
        """
        self._ensure_open()
        return self._model, self._data

    @property
    def control_targets(self) -> tuple[float, ...]:
        self._ensure_open()
        return tuple(float(value) for value in self._position_targets)

    @property
    def control_mode(self) -> str:
        self._ensure_open()
        return self._control_mode

    def set_control_mode(self, mode: str) -> str:
        self._ensure_open()
        mode = str(mode)
        if mode not in CONTROL_MODES:
            raise ValueError(f"control mode must be one of {CONTROL_MODES}")
        if mode == "hold":
            self._sync_arm_targets_to_current_position()
        if mode in ("gravity_comp", "hold"):
            self._arm_controller.reset()
        self._control_mode = mode
        self._apply_motor_control()
        return self._control_mode

    def _name_id(self, object_type, name: str) -> int:
        identifier = int(self._mj.mj_name2id(self._model, object_type, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model is missing required {name!r}")
        return identifier

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("MuJoCo simulation is closed")

    def _find_free_bodies(self) -> dict[str, tuple[int, int]]:
        result: dict[str, tuple[int, int]] = {}
        free_type = int(self._mj.mjtJoint.mjJNT_FREE)
        for body_id in range(1, int(self._model.nbody)):
            joint_start = int(self._model.body_jntadr[body_id])
            joint_count = int(self._model.body_jntnum[body_id])
            for joint_id in range(joint_start, joint_start + joint_count):
                if int(self._model.jnt_type[joint_id]) == free_type:
                    name = self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, body_id)
                    result[str(name)] = (body_id, int(self._model.jnt_qposadr[joint_id]))
        return result

    def _fingerprint_model(self) -> str:
        digest = hashlib.sha256()
        digest.update(repr(self._model_dimensions).encode("ascii"))
        for values in (
            self._model.names,
            self._model.jnt_type,
            self._model.jnt_qposadr,
            self._model.jnt_dofadr,
            self._model.jnt_range,
            self._model.actuator_trnid,
            self._model.geom_bodyid,
        ):
            digest.update(np.asarray(values).tobytes())
        digest.update(np.asarray([self._model.opt.timestep], dtype=np.float64).tobytes())
        return digest.hexdigest()

    def reset(self, seed: int | None = None) -> SimulationState:
        self._ensure_open()
        self._rng = np.random.default_rng(seed)
        self._mj.mj_resetData(self._model, self._data)
        return self._finish_reset()

    def reset_home(self, seed: int | None = None) -> SimulationState:
        self._ensure_open()
        self._rng = np.random.default_rng(seed)
        home_key = self._mj.mj_name2id(
            self._model, self._mj.mjtObj.mjOBJ_KEY, "home"
        )
        if home_key >= 0:
            self._mj.mj_resetDataKeyframe(self._model, self._data, home_key)
        else:
            self._mj.mj_resetData(self._model, self._data)
        return self._finish_reset()

    def _finish_reset(self) -> SimulationState:
        for index, joint_id in enumerate(self._joint_ids):
            qpos_address = int(self._model.jnt_qposadr[joint_id])
            self._position_targets[index] = self._data.qpos[qpos_address]
        self._data.ctrl[:] = 0.0
        self._arm_controller.reset()
        self._control_phase = 0
        self._mj.mj_forward(self._model, self._data)
        self._seed_arm_torque_from_gravity()
        self._apply_motor_control()
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def set_joint_position_targets(
        self, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        self._ensure_open()
        current = list(self.control_targets[: len(ARM_JOINT_NAMES)])
        if isinstance(targets, Mapping):
            unknown = set(targets) - set(ARM_JOINT_NAMES)
            if unknown:
                raise ValueError(f"Unknown arm joint names: {sorted(unknown)}")
            updates = {name: float(value) for name, value in targets.items()}
            if not all(math.isfinite(value) for value in updates.values()):
                raise ValueError("Joint targets must be finite")
            for name, value in updates.items():
                current[ARM_JOINT_NAMES.index(name)] = value
        else:
            current = list(_finite_vector(targets, len(ARM_JOINT_NAMES), "joint targets"))

        reached = []
        for index, joint_id in enumerate(self._joint_ids[:6]):
            lower, upper = (float(value) for value in self._model.jnt_range[joint_id])
            value = min(max(current[index], lower), upper)
            self._position_targets[index] = value
            reached.append(value)
        self._control_mode = "pos_vel"
        return tuple(reached)

    def set_gripper_width(self, width: float) -> float:
        self._ensure_open()
        value = float(width)
        if not math.isfinite(value):
            raise ValueError("Gripper width must be finite")
        left, right, reached = gripper_joint_positions_for_width(value)
        self._position_targets[-2] = left
        self._position_targets[-1] = right
        return reached

    def step(self, n_steps: int = 1) -> SimulationState:
        self._ensure_open()
        if isinstance(n_steps, bool) or not isinstance(n_steps, int):
            raise TypeError("n_steps must be a positive integer")
        if n_steps <= 0:
            raise ValueError("n_steps must be a positive integer")
        for _ in range(n_steps):
            if self._control_phase == 0:
                self._apply_motor_control()
            self._mj.mj_step(self._model, self._data)
            self._control_phase = (self._control_phase + 1) % self._control_steps_per_update
        # mj_step integrates qpos after its position stage; refresh derived
        # kinematics so the returned pose describes the new qpos, not the
        # beginning of the final step.
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def _apply_motor_control(self) -> None:
        qpos_addresses = [int(self._model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids[:6]]
        qvel_addresses = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        position = np.asarray([self._data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([self._data.qvel[address] for address in qvel_addresses], dtype=float)
        gravity = self._gravity_compensation_torque(qvel_addresses)
        if self._control_mode == "gravity_comp":
            arm_torque = gravity
            self._arm_controller.applied_torque[:] = arm_torque
        elif self._control_mode == "hold":
            kp = np.asarray((12.0, 12.0, 12.0, 8.0, 8.0, 4.0), dtype=float)
            kd = np.asarray((1.2, 1.2, 1.2, 0.8, 0.8, 0.4), dtype=float)
            effort = np.asarray(self._motor_parameters.arm.effort_limit, dtype=float)
            arm_torque = np.clip(
                gravity + kp * (self._position_targets[:6] - position) - kd * velocity,
                -effort,
                effort,
            )
            self._arm_controller.applied_torque[:] = arm_torque
        else:
            arm_torque = self._arm_controller.compute(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
        for actuator_id, torque in zip(self._actuator_ids[:6], arm_torque):
            self._data.ctrl[actuator_id] = torque

        left_qpos = float(self._data.qpos[int(self._model.jnt_qposadr[self._joint_ids[-2]])])
        right_qpos = float(self._data.qpos[int(self._model.jnt_qposadr[self._joint_ids[-1]])])
        left_qvel = float(self._data.qvel[int(self._model.jnt_dofadr[self._joint_ids[-2]])])
        right_qvel = float(self._data.qvel[int(self._model.jnt_dofadr[self._joint_ids[-1]])])
        command = self._gripper_controller.compute(
            target=float(self._position_targets[-2] - self._position_targets[-1]),
            position=left_qpos - right_qpos,
            velocity=left_qvel - right_qvel,
            mode="move",
        )
        finger_force = self._stable_gripper_force(
            target_width=command.target_displacement_m,
            current_width=left_qpos - right_qpos,
            current_velocity=left_qvel - right_qvel,
        )
        left_force = self._clamp_actuator_control(self._actuator_ids[-2], finger_force)
        right_force = self._clamp_actuator_control(self._actuator_ids[-1], -finger_force)
        self._data.ctrl[self._actuator_ids[-2]] = left_force
        self._data.ctrl[self._actuator_ids[-1]] = right_force

    def _stable_gripper_force(
        self,
        *,
        target_width: float,
        current_width: float,
        current_velocity: float,
    ) -> float:
        gripper = self._motor_parameters.gripper
        error = float(target_width) - float(current_width)
        velocity = float(current_velocity)
        if (
            abs(error) < gripper.sim_force_deadband_m
            and abs(velocity) < gripper.sim_velocity_deadband_m_s
        ):
            return 0.0
        # MuJoCo force actuators act directly on the sliding finger joints in N.
        # The real MIT command still defines the motor-side torque limit, while
        # this linear-space PD keeps the simulated prismatic joints stable.
        return (
            gripper.sim_force_kp_n_per_m * error
            - gripper.sim_force_kd_n_s_per_m * velocity
        )

    def _clamp_actuator_control(self, actuator_id: int, value: float) -> float:
        if int(self._model.actuator_ctrllimited[actuator_id]):
            lower, upper = (float(v) for v in self._model.actuator_ctrlrange[actuator_id])
            return min(max(float(value), lower), upper)
        return float(value)

    def _seed_arm_torque_from_gravity(self) -> None:
        qvel_addresses = [int(self._model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        self._arm_controller.applied_torque[:] = self._gravity_compensation_torque(qvel_addresses)

    def _sync_arm_targets_to_current_position(self) -> None:
        for index, joint_id in enumerate(self._joint_ids[:6]):
            self._position_targets[index] = float(self._data.qpos[int(self._model.jnt_qposadr[joint_id])])

    def _gravity_compensation_torque(self, qvel_addresses: Sequence[int]) -> np.ndarray:
        scale = float(self._motor_parameters.arm.gravity_compensation_scale)
        if scale == 0.0:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return np.asarray([self._data.qfrc_bias[address] for address in qvel_addresses], dtype=float) * scale

    def get_state(self) -> SimulationState:
        self._ensure_open()
        positions = []
        velocities = []
        for joint_id in self._joint_ids:
            positions.append(float(self._data.qpos[int(self._model.jnt_qposadr[joint_id])]))
            velocities.append(float(self._data.qvel[int(self._model.jnt_dofadr[joint_id])]))
        object_poses = {
            name: (
                *(float(value) for value in self._data.qpos[address : address + 3]),
                *(float(value) for value in self._data.qpos[address + 4 : address + 7]),
                float(self._data.qpos[address + 3]),
            )
            for name, (_, address) in self._free_bodies.items()
        }
        ee_quaternion_wxyz = np.empty(4, dtype=float)
        self._mj.mju_mat2Quat(
            ee_quaternion_wxyz, self._data.site_xmat[self._ee_site_id]
        )
        return SimulationState(
            joint_names=JOINT_NAMES,
            joint_positions=tuple(positions),
            joint_velocities=tuple(velocities),
            actuator_forces=tuple(float(self._data.actuator_force[index]) for index in self._actuator_ids),
            end_effector_position=tuple(float(value) for value in self._data.site_xpos[self._ee_site_id]),
            end_effector_orientation=(
                *(float(value) for value in ee_quaternion_wxyz[1:]),
                float(ee_quaternion_wxyz[0]),
            ),
            gripper_width=max(0.0, min(0.09, positions[-2] - positions[-1])),
            object_poses=MappingProxyType(object_poses),
            simulation_time=float(self._data.time),
        )

    def get_contacts(self) -> tuple[ContactInfo, ...]:
        self._ensure_open()
        contacts = []
        force = np.zeros(6, dtype=float)
        for index in range(int(self._data.ncon)):
            contact = self._data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            body1, body2 = int(self._model.geom_bodyid[geom1]), int(self._model.geom_bodyid[geom2])
            force.fill(0.0)
            self._mj.mj_contactForce(self._model, self._data, index, force)
            contacts.append(ContactInfo(
                body1=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, body1) or "world"),
                body2=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, body2) or "world"),
                geom1=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_GEOM, geom1) or f"geom{geom1}"),
                geom2=str(self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_GEOM, geom2) or f"geom{geom2}"),
                position=tuple(float(value) for value in contact.pos),
                force=float(np.linalg.norm(force[:3])),
            ))
        return tuple(contacts)

    def save_state(self) -> SavedSimulationState:
        self._ensure_open()
        state = np.empty(self._model_dimensions[-1], dtype=float)
        self._mj.mj_getState(self._model, self._data, state, self._state_spec)
        return SavedSimulationState(
            model_identity=id(self._model),
            model_fingerprint=self._model_fingerprint,
            model_dimensions=self._model_dimensions,
            state_spec=self._state_spec,
            state=tuple(float(value) for value in state),
            control_targets=tuple(float(value) for value in self._position_targets),
            position_integral=tuple(float(value) for value in self._arm_controller.position_integral),
            velocity_integral=tuple(float(value) for value in self._arm_controller.velocity_integral),
            applied_torque=tuple(float(value) for value in self._arm_controller.applied_torque),
            control_phase=self._control_phase,
            control_mode=self._control_mode,
        )

    def restore_state(self, state: SavedSimulationState) -> SimulationState:
        self._ensure_open()
        if not isinstance(state, SavedSimulationState):
            raise TypeError("state must be returned by save_state()")
        compatible = (
            state.model_identity == id(self._model)
            and state.model_fingerprint == self._model_fingerprint
            and state.model_dimensions == self._model_dimensions
            and state.state_spec == self._state_spec
            and len(state.state) == self._model_dimensions[-1]
            and len(state.control_targets) == len(JOINT_NAMES)
            and len(state.position_integral) == len(ARM_JOINT_NAMES)
            and len(state.velocity_integral) == len(ARM_JOINT_NAMES)
            and len(state.applied_torque) == len(ARM_JOINT_NAMES)
            and state.control_mode in CONTROL_MODES
        )
        if not compatible:
            raise ValueError("saved state must belong to the same MuJoCo model instance")
        self._mj.mj_setState(
            self._model, self._data, np.asarray(state.state, dtype=float), self._state_spec
        )
        self._position_targets[:] = np.asarray(state.control_targets, dtype=float)
        self._arm_controller.position_integral[:] = np.asarray(state.position_integral, dtype=float)
        self._arm_controller.velocity_integral[:] = np.asarray(state.velocity_integral, dtype=float)
        self._arm_controller.applied_torque[:] = np.asarray(state.applied_torque, dtype=float)
        self._control_phase = int(state.control_phase) % self._control_steps_per_update
        self._control_mode = state.control_mode
        self._mj.mj_forward(self._model, self._data)
        return self.get_state()

    def set_object_pose(
        self,
        body_name: str,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        zero_velocity: bool = True,
    ) -> tuple[float, ...]:
        self._ensure_open()
        if body_name not in self._free_bodies:
            raise ValueError(f"Body {body_name!r} is not a free object")
        position_values = _finite_vector(position, 3, "position")
        quaternion = np.asarray(_finite_vector(orientation, 4, "orientation"), dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            raise ValueError("orientation quaternion must have non-zero norm")
        quaternion /= norm
        body_id, address = self._free_bodies[body_name]
        orientation_xyzw = tuple(float(value) for value in quaternion)
        internal_pose_wxyz = (
            *position_values,
            orientation_xyzw[3],
            *orientation_xyzw[:3],
        )
        self._data.qpos[address : address + 7] = internal_pose_wxyz
        if zero_velocity:
            joint_id = int(self._model.body_jntadr[body_id])
            dof_address = int(self._model.jnt_dofadr[joint_id])
            self._data.qvel[dof_address : dof_address + 6] = 0.0
        self._mj.mj_forward(self._model, self._data)
        return (*position_values, *orientation_xyzw)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._data = None
        self._model = None

    def __enter__(self) -> "RebotArmMujoco":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

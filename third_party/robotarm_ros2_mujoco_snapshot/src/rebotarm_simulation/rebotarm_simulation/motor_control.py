"""Pure motor-control math shared by MuJoCo front ends.

Firmware gains remain visible as raw parameters. Values that translate the
firmware velocity-loop output into physical torque live in the separate
simulation calibration file.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


@dataclass(frozen=True)
class MotorSpec:
    name: str
    rated_torque_nm: float
    peak_torque_nm: float
    reduction_ratio: float


@dataclass(frozen=True)
class ArmControlParameters:
    joint_names: tuple[str, ...]
    motor_models: tuple[str, ...]
    pos_kp: tuple[float, ...]
    pos_ki: tuple[float, ...]
    vel_kp: tuple[float, ...]
    vel_ki: tuple[float, ...]
    velocity_limit: tuple[float, ...]
    effort_limit: tuple[float, ...]
    rated_torque: tuple[float, ...]
    firmware_to_torque_scale: tuple[float, ...]
    torque_rate_limit_nm_s: tuple[float, ...]
    torque_lowpass_alpha: tuple[float, ...]
    gravity_compensation_scale: float
    position_integral_limit: float
    velocity_integral_limit: float


@dataclass(frozen=True)
class GripperControlParameters:
    firmware_default_kp: float
    firmware_default_kd: float
    motor_model: str
    move_kp: float
    move_kd: float
    closing_kp: float
    closing_kd: float
    hold_kp: float
    hold_kd: float
    motor_radians_per_opening_m: float
    transmission_efficiency: float
    motor_torque_limit_nm: float
    finger_force_limit_n: float
    sim_force_kp_n_per_m: float
    sim_force_kd_n_s_per_m: float
    sim_force_deadband_m: float
    sim_velocity_deadband_m_s: float
    displacement_min_m: float
    displacement_max_m: float


@dataclass(frozen=True)
class MotorControlParameters:
    control_rate_hz: float
    motor_specs: dict[str, MotorSpec]
    arm: ArmControlParameters
    gripper: GripperControlParameters


@dataclass(frozen=True)
class GripperCommand:
    mode: str
    target_displacement_m: float
    motor_target_rad: float
    kp: float
    kd: float
    motor_torque_nm: float
    finger_force_n: float


def load_motor_control_parameters(repo_root: str | Path) -> MotorControlParameters:
    root = Path(repo_root)
    arm_yaml = _read_yaml(root / "src/rebotarm_bringup/config/arm.yaml")
    gripper_yaml = _read_yaml(root / "src/rebotarm_bringup/config/gripper.yaml")
    calibration = _read_yaml(
        root / "src/rebotarm_simulation/config/motor_control_calibration.yaml"
    )
    urdf_efforts = _load_urdf_efforts(
        root / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    )

    motor_specs = {
        name: MotorSpec(
            name=name,
            rated_torque_nm=float(values["rated_torque_nm"]),
            peak_torque_nm=float(values["peak_torque_nm"]),
            reduction_ratio=float(values["reduction_ratio"]),
        )
        for name, values in calibration["motor_specs"].items()
    }

    arm_calibration = calibration["arm"]
    joint_names: list[str] = []
    motor_models: list[str] = []
    pos_kp: list[float] = []
    pos_ki: list[float] = []
    vel_kp: list[float] = []
    vel_ki: list[float] = []
    velocity_limit: list[float] = []
    effort_limit: list[float] = []
    rated_torque: list[float] = []
    firmware_to_torque_scale: list[float] = []
    torque_rate_limit_nm_s: list[float] = []
    torque_lowpass_alpha: list[float] = []
    for joint in arm_yaml["joints"]:
        name = joint["name"]
        calibration_entry = arm_calibration[name]
        motor_model = str(calibration_entry["motor_model"])
        motor_spec = motor_specs[motor_model]
        pos_vel = joint["POS_VEL"]
        joint_names.append(str(name))
        motor_models.append(motor_model)
        pos_kp.append(float(pos_vel["pos_kp"]))
        pos_ki.append(float(pos_vel["pos_ki"]))
        vel_kp.append(float(pos_vel["vel_kp"]))
        vel_ki.append(float(pos_vel["vel_ki"]))
        velocity_limit.append(float(pos_vel["vlim"]))
        effort_limit.append(float(urdf_efforts[name]))
        rated_torque.append(float(motor_spec.rated_torque_nm))
        firmware_to_torque_scale.append(float(calibration_entry["firmware_to_torque_scale"]))
        torque_rate_limit_nm_s.append(float(calibration_entry["torque_rate_limit_nm_s"]))
        alpha = float(calibration_entry["torque_lowpass_alpha"])
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"torque_lowpass_alpha for {name} must be in (0, 1]")
        torque_lowpass_alpha.append(alpha)

    source_gripper = gripper_yaml["gripper"][0]["MIT"]
    gripper_calibration = calibration["gripper"]
    modes = gripper_calibration["modes"]
    displacement_range = gripper_calibration["displacement_range_m"]
    return MotorControlParameters(
        control_rate_hz=float(arm_yaml["rate"]),
        motor_specs=motor_specs,
        arm=ArmControlParameters(
            joint_names=tuple(joint_names),
            motor_models=tuple(motor_models),
            pos_kp=tuple(pos_kp),
            pos_ki=tuple(pos_ki),
            vel_kp=tuple(vel_kp),
            vel_ki=tuple(vel_ki),
            velocity_limit=tuple(velocity_limit),
            effort_limit=tuple(effort_limit),
            rated_torque=tuple(rated_torque),
            firmware_to_torque_scale=tuple(firmware_to_torque_scale),
            torque_rate_limit_nm_s=tuple(torque_rate_limit_nm_s),
            torque_lowpass_alpha=tuple(torque_lowpass_alpha),
            gravity_compensation_scale=float(arm_calibration["gravity_compensation_scale"]),
            position_integral_limit=float(arm_calibration["position_integral_limit_rad_s"]),
            velocity_integral_limit=float(arm_calibration["velocity_integral_limit_rad"]),
        ),
        gripper=GripperControlParameters(
            firmware_default_kp=float(source_gripper["kp"]),
            firmware_default_kd=float(source_gripper["kd"]),
            motor_model=str(gripper_calibration["motor_model"]),
            move_kp=float(modes["move"]["kp"]),
            move_kd=float(modes["move"]["kd"]),
            closing_kp=float(modes["closing"]["kp"]),
            closing_kd=float(modes["closing"]["kd"]),
            hold_kp=float(modes["hold"]["kp"]),
            hold_kd=float(modes["hold"]["kd"]),
            motor_radians_per_opening_m=float(
                gripper_calibration["motor_radians_per_opening_m"]
            ),
            transmission_efficiency=float(gripper_calibration["transmission_efficiency"]),
            motor_torque_limit_nm=float(gripper_calibration["motor_torque_limit_nm"]),
            finger_force_limit_n=float(gripper_calibration["finger_force_limit_n"]),
            sim_force_kp_n_per_m=float(gripper_calibration["sim_force_kp_n_per_m"]),
            sim_force_kd_n_s_per_m=float(gripper_calibration["sim_force_kd_n_s_per_m"]),
            sim_force_deadband_m=float(gripper_calibration["sim_force_deadband_m"]),
            sim_velocity_deadband_m_s=float(gripper_calibration["sim_velocity_deadband_m_s"]),
            displacement_min_m=float(displacement_range[0]),
            displacement_max_m=float(displacement_range[1]),
        ),
    )


class PosVelController:
    """100 Hz cascaded position/velocity PI with conditional anti-windup."""

    def __init__(self, parameters: ArmControlParameters):
        self.parameters = parameters
        self.position_integral = np.zeros(6, dtype=float)
        self.velocity_integral = np.zeros(6, dtype=float)
        self.target_velocity = np.zeros(6, dtype=float)
        self.applied_torque = np.zeros(6, dtype=float)

    def reset(self) -> None:
        self.position_integral.fill(0.0)
        self.velocity_integral.fill(0.0)
        self.target_velocity.fill(0.0)
        self.applied_torque.fill(0.0)

    def compute(self, target, position, velocity, dt: float, feedforward=None) -> np.ndarray:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        target = _vector6(target, "target")
        position = _vector6(position, "position")
        velocity = _vector6(velocity, "velocity")
        feedforward_torque = (
            np.zeros(6, dtype=float)
            if feedforward is None
            else _vector6(feedforward, "feedforward")
        )

        pos_kp = np.asarray(self.parameters.pos_kp)
        pos_ki = np.asarray(self.parameters.pos_ki)
        velocity_limit = np.asarray(self.parameters.velocity_limit)
        position_error = target - position
        position_candidate = np.clip(
            self.position_integral + position_error * dt,
            -self.parameters.position_integral_limit,
            self.parameters.position_integral_limit,
        )
        raw_target_velocity = pos_kp * position_error + pos_ki * position_candidate
        self.target_velocity = np.clip(raw_target_velocity, -velocity_limit, velocity_limit)
        position_accept = (raw_target_velocity == self.target_velocity) | (
            np.sign(position_error) != np.sign(raw_target_velocity)
        )
        self.position_integral = np.where(
            position_accept, position_candidate, self.position_integral
        )

        velocity_error = self.target_velocity - velocity
        velocity_candidate = np.clip(
            self.velocity_integral + velocity_error * dt,
            -self.parameters.velocity_integral_limit,
            self.parameters.velocity_integral_limit,
        )
        raw_feedback_torque = (
            (
                np.asarray(self.parameters.vel_kp) * velocity_error
                + np.asarray(self.parameters.vel_ki) * velocity_candidate
            )
            * np.asarray(self.parameters.firmware_to_torque_scale)
        )
        raw_torque = raw_feedback_torque + feedforward_torque
        effort = np.asarray(self.parameters.effort_limit)
        clipped_torque = np.clip(raw_torque, -effort, effort)
        velocity_accept = (raw_torque == clipped_torque) | (
            np.sign(velocity_error) != np.sign(raw_feedback_torque)
        )
        self.velocity_integral = np.where(
            velocity_accept, velocity_candidate, self.velocity_integral
        )
        max_delta = np.asarray(self.parameters.torque_rate_limit_nm_s) * float(dt)
        rate_limited = self.applied_torque + np.clip(
            clipped_torque - self.applied_torque,
            -max_delta,
            max_delta,
        )
        alpha = np.asarray(self.parameters.torque_lowpass_alpha)
        smoothed = self.applied_torque + alpha * (rate_limited - self.applied_torque)
        self.applied_torque = np.clip(smoothed, -effort, effort)
        return self.applied_torque.copy()


class GripperMitController:
    def __init__(self, parameters: GripperControlParameters):
        self.parameters = parameters

    def compute(
        self,
        target: float,
        position: float,
        velocity: float,
        mode: str = "move",
        feedforward_torque: float = 0.0,
    ) -> GripperCommand:
        p = self.parameters
        gains = {
            "move": (p.move_kp, p.move_kd),
            "closing": (p.closing_kp, p.closing_kd),
            "hold": (p.hold_kp, p.hold_kd),
        }
        if mode not in gains:
            raise ValueError(f"unsupported gripper mode: {mode}")
        kp, kd = gains[mode]
        target_m = float(np.clip(target, p.displacement_min_m, p.displacement_max_m))
        position_m = float(np.clip(position, p.displacement_min_m, p.displacement_max_m))
        ratio = float(p.motor_radians_per_opening_m)
        motor_target = target_m * ratio
        motor_position = position_m * ratio
        motor_velocity = float(velocity) * ratio
        raw_torque = kp * (motor_target - motor_position) - kd * motor_velocity
        raw_torque += float(feedforward_torque)
        motor_torque = float(
            np.clip(raw_torque, -p.motor_torque_limit_nm, p.motor_torque_limit_nm)
        )
        return GripperCommand(
            mode=mode,
            target_displacement_m=target_m,
            motor_target_rad=motor_target,
            kp=kp,
            kd=kd,
            motor_torque_nm=motor_torque,
            finger_force_n=motor_torque * ratio * p.transmission_efficiency,
        )


def _read_yaml(path: Path) -> dict:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected YAML mapping in {path}")
    return payload


def _load_urdf_efforts(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    return {
        joint.attrib["name"]: float(limit.attrib["effort"])
        for joint in root.findall("joint")
        for limit in [joint.find("limit")]
        if limit is not None and "effort" in limit.attrib
    }


def _vector6(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {result.shape}")
    return result

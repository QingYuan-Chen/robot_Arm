from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotorSpec:
    name: str
    motor_id: int
    feedback_id: int
    model: str


@dataclass(frozen=True)
class PosVelGains:
    vel_kp: float
    vel_ki: float
    pos_kp: float
    pos_ki: float


MOTOR_SPECS = (
    MotorSpec("joint1", 0x01, 0x11, "4340P"),
    MotorSpec("joint2", 0x02, 0x12, "4340P"),
    MotorSpec("joint3", 0x03, 0x13, "4340P"),
    MotorSpec("joint4", 0x04, 0x14, "4310"),
    MotorSpec("joint5", 0x05, 0x15, "4310"),
    MotorSpec("joint6", 0x06, 0x16, "4310"),
    MotorSpec("gripper", 0x07, 0x17, "4310"),
)
ARM_MOTOR_SPECS = MOTOR_SPECS[:6]
GRIPPER_SPEC = MOTOR_SPECS[6]

POS_VEL_GAINS_BY_NAME = {
    **{
        name: PosVelGains(0.0125, 0.004, 150.0, 0.5)
        for name in ("joint1", "joint2", "joint3")
    },
    **{
        name: PosVelGains(0.0008, 0.002, 50.0, 1.0)
        for name in ("joint4", "joint5", "joint6")
    },
}

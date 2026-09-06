from __future__ import annotations

import math
from dataclasses import dataclass

import pytest

from rebot_b601_mapping.follower_reader import FollowerReader
from rebot_b601_mapping.hardware_specs import MOTOR_SPECS
from rebot_b601_mapping.ports import PortIdentity


@dataclass
class FakeState:
    can_id: int
    arbitration_id: int
    status_code: int = 0
    pos: float = 0.0
    vel: float = 0.0
    torq: float = 0.0
    t_mos: float = 25.0
    t_rotor: float = 25.0


class FakeMotor:
    def __init__(self, state: FakeState, events: list[str]) -> None:
        self.state = state
        self.events = events
        self.request_count = 0
        self.closed = False

    def request_feedback(self) -> None:
        self.request_count += 1

    def get_state(self) -> FakeState | None:
        return self.state

    def close(self) -> None:
        self.closed = True
        self.events.append(f"motor-close-{self.state.can_id}")

    def __getattr__(self, name: str):
        if name in {
            "enable",
            "disable",
            "clear_error",
            "ensure_mode",
            "set_zero_position",
            "send_mit",
            "send_pos_vel",
            "send_vel",
            "send_force_pos",
            "write_register_f32",
            "write_register_u32",
        }:
            raise AssertionError(f"禁止调用 {name}")
        raise AttributeError(name)


class FakeController:
    def __init__(self) -> None:
        self.add_calls: list[tuple[int, int, str]] = []
        self.motors: list[FakeMotor] = []
        self.poll_count = 0
        self.events: list[str] = []
        self.closed = False

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> FakeMotor:
        self.add_calls.append((motor_id, feedback_id, model))
        motor = FakeMotor(
            FakeState(
                can_id=motor_id,
                arbitration_id=feedback_id,
                pos=-0.1 * motor_id,
            ),
            self.events,
        )
        self.motors.append(motor)
        return motor

    def poll_feedback_once(self) -> None:
        self.poll_count += 1

    def close(self) -> None:
        self.closed = True
        self.events.append("controller-close")

    def shutdown(self) -> None:
        raise AssertionError("禁止调用 shutdown")

    def disable_all(self) -> None:
        raise AssertionError("禁止调用 disable_all")

    def enable_all(self) -> None:
        raise AssertionError("禁止调用 enable_all")


def make_reader(controller: FakeController) -> FollowerReader:
    return FollowerReader(
        "/dev/fake-follower",
        controller_factory=lambda port, baud: controller,
        clock=lambda: 20.0,
        sleep=lambda seconds: None,
        identity_factory=lambda path: PortIdentity(path, path, 166),
        identity_checker=lambda identity: None,
    )


def test_follower_registers_exact_b601_motors_and_reads_feedback() -> None:
    controller = FakeController()
    reader = make_reader(controller)
    reader.open()

    sample = reader.read_sample()

    assert controller.add_calls == [
        (0x01, 0x11, "4340P"),
        (0x02, 0x12, "4340P"),
        (0x03, 0x13, "4340P"),
        (0x04, 0x14, "4310"),
        (0x05, 0x15, "4310"),
        (0x06, 0x16, "4310"),
        (0x07, 0x17, "4310"),
    ]
    assert [spec.name for spec in MOTOR_SPECS] == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"
    ]
    assert [motor.request_count for motor in controller.motors] == [1] * 7
    assert controller.poll_count == 7
    assert sample.timestamp_s == 20.0
    assert [motor.name for motor in sample.motors] == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]
    assert [motor.position_rad for motor in sample.motors] == pytest.approx(
        (-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7)
    )


def test_close_releases_handles_without_lifecycle_commands() -> None:
    controller = FakeController()
    reader = make_reader(controller)
    reader.open()

    reader.close()

    assert controller.events == [
        "motor-close-1",
        "motor-close-2",
        "motor-close-3",
        "motor-close-4",
        "motor-close-5",
        "motor-close-6",
        "motor-close-7",
        "controller-close",
    ]
    assert controller.closed is True


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda controller: setattr(controller.motors[2], "state", None), "joint3.*缺失"),
        (
            lambda controller: setattr(controller.motors[4].state, "pos", math.nan),
            "joint5.*非有限",
        ),
        (
            lambda controller: setattr(controller.motors[5].state, "status_code", 1),
            "joint6 status_code=1",
        ),
    ],
)
def test_read_sample_rejects_invalid_feedback(mutate, message: str) -> None:
    controller = FakeController()
    reader = make_reader(controller)
    reader.open()
    mutate(controller)

    with pytest.raises(RuntimeError, match=message):
        reader.read_sample()


def test_context_exit_closes_handles_on_keyboard_interrupt() -> None:
    controller = FakeController()
    reader = make_reader(controller)

    with pytest.raises(KeyboardInterrupt):
        with reader:
            raise KeyboardInterrupt

    assert all(motor.closed for motor in controller.motors)
    assert controller.closed is True


def test_read_sample_retries_each_motor_until_initial_feedback_arrives() -> None:
    class DelayedMotor(FakeMotor):
        def get_state(self) -> FakeState | None:
            if self.request_count < 2:
                return None
            return self.state

    class DelayedController(FakeController):
        def add_damiao_motor(
            self,
            motor_id: int,
            feedback_id: int,
            model: str,
        ) -> FakeMotor:
            self.add_calls.append((motor_id, feedback_id, model))
            motor = DelayedMotor(
                FakeState(can_id=motor_id, arbitration_id=feedback_id, pos=-0.1 * motor_id),
                self.events,
            )
            self.motors.append(motor)
            return motor

    controller = DelayedController()
    reader = make_reader(controller)
    reader.open()

    sample = reader.read_sample()

    assert len(sample.motors) == 7
    assert [motor.request_count for motor in controller.motors] == [2] * 7
    assert controller.poll_count == 14

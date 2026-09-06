from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

from rebot_b601_mapping.follower_controller import (
    FollowerCommunicationError,
    FollowerController,
    FollowerLifecycleError,
)
from rebot_b601_mapping.hardware_specs import POS_VEL_GAINS_BY_NAME
from rebot_b601_mapping.live_config import load_live_follow_config
from rebot_b601_mapping.models import load_mapping_config
from rebot_b601_mapping.ports import PortIdentity


LIVE_CONFIG = Path(__file__).parents[1] / "live_follow.example.json"
MAPPING_CONFIG = Path(__file__).parents[1] / "mapping.example.json"
JOINT_LIMITS = tuple(
    (joint.lower_rad, joint.upper_rad)
    for joint in load_mapping_config(MAPPING_CONFIG).arm_joints
)
START = (0.1, -1.0, -1.1, 0.2, 0.0, 0.3)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, float(seconds))


@dataclass
class FakeState:
    pos: float
    vel: float = 0.0
    torq: float = 0.0
    status_code: int = 0


class FakeMotor:
    def __init__(self, name: str, position: float, controller) -> None:
        self.name = name
        self.state = FakeState(position)
        self.controller = controller
        self.register_writes: list[tuple[int, float]] = []
        gains = POS_VEL_GAINS_BY_NAME.get(name)
        self.register_u32 = {10: 2}
        self.register_f32 = (
            {}
            if gains is None
            else {
                25: gains.vel_kp,
                26: gains.vel_ki,
                27: gains.pos_kp,
                28: gains.pos_ki,
            }
        )
        self.register_reads: list[tuple[str, int, int]] = []
        self.ensure_modes: list[str] = []
        self.pos_vel_calls: list[tuple[float, float]] = []
        self.enable_calls = 0
        self.disable_calls = 0
        self.request_calls = 0
        self.closed = False
        self._pending_status: int | None = None
        self._status_transition_request = 0

    def write_register_f32(self, register: int, value: float) -> None:
        if self.controller.register_failure_at == self.name:
            raise RuntimeError("register failed")
        self.register_writes.append((register, value))

    def get_register_u32(self, register: int, timeout_ms: int = 1000) -> int:
        self.register_reads.append(("u32", register, timeout_ms))
        return self.register_u32[register]

    def get_register_f32(self, register: int, timeout_ms: int = 1000) -> float:
        self.register_reads.append(("f32", register, timeout_ms))
        return self.register_f32[register]

    def ensure_mode(self, mode, timeout_ms: int) -> None:
        if self.controller.mode_failure_at == self.name:
            raise RuntimeError("mode failed")
        self.ensure_modes.append(getattr(mode, "name", str(mode)))

    def enable(self) -> None:
        self.enable_calls += 1
        if self.controller.enable_failure_at == self.name:
            raise RuntimeError("enable failed")
        delay = self.controller.enable_status_delays.get(self.name, 0)
        if delay == 0:
            self.state.status_code = 1
        else:
            self._pending_status = 1
            self._status_transition_request = self.request_calls

    def disable(self) -> None:
        self.disable_calls += 1
        delay = self.controller.disable_status_delays.get(self.name, 0)
        if delay == 0:
            self.state.status_code = 0
            self._pending_status = None
        else:
            self._pending_status = 0
            self._status_transition_request = self.request_calls

    def send_pos_vel(self, position: float, speed: float) -> None:
        if self.controller.write_failure_at == self.name:
            raise RuntimeError("write failed")
        self.pos_vel_calls.append((position, speed))

    def request_feedback(self) -> None:
        self.request_calls += 1

    def get_state(self) -> FakeState:
        ready_after = self.controller.feedback_ready_after.get(self.name, 0)
        if self.request_calls < ready_after:
            return None
        if self._pending_status is not None:
            target = self._pending_status
            delays = (
                self.controller.enable_status_delays
                if target == 1
                else self.controller.disable_status_delays
            )
            if self.request_calls - self._status_transition_request >= delays.get(
                self.name, 0
            ):
                self.state.status_code = target
                self._pending_status = None
        return self.state

    def close(self) -> None:
        self.closed = True


class FakeController:
    def __init__(
        self,
        *,
        positions=START,
        enable_failure_at=None,
        register_failure_at=None,
        mode_failure_at=None,
        write_failure_at=None,
        poll_advance_s=0.0,
        feedback_ready_after=None,
        enable_status_delays=None,
        disable_status_delays=None,
        clock=None,
    ) -> None:
        self.positions = tuple(positions) + (-1.0,)
        self.enable_failure_at = enable_failure_at
        self.register_failure_at = register_failure_at
        self.mode_failure_at = mode_failure_at
        self.write_failure_at = write_failure_at
        self.poll_advance_s = poll_advance_s
        self.feedback_ready_after = dict(feedback_ready_after or {})
        self.enable_status_delays = dict(enable_status_delays or {})
        self.disable_status_delays = dict(disable_status_delays or {})
        self.clock = clock
        self.motors: list[FakeMotor] = []
        self.closed = False
        self.shutdown_calls = 0
        self.disable_all_calls = 0
        self.enable_all_calls = 0

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str):
        name = f"joint{motor_id}" if motor_id <= 6 else "gripper"
        motor = FakeMotor(name, self.positions[motor_id - 1], self)
        self.motors.append(motor)
        return motor

    def poll_feedback_once(self) -> None:
        if self.clock is not None:
            self.clock.now += self.poll_advance_s

    def close(self) -> None:
        self.closed = True

    def shutdown(self) -> None:
        self.shutdown_calls += 1

    def disable_all(self) -> None:
        self.disable_all_calls += 1

    def enable_all(self) -> None:
        self.enable_all_calls += 1


def make_follower(**controller_options):
    clock = FakeClock()
    controller = FakeController(clock=clock, **controller_options)
    follower = FollowerController(
        "/dev/fake-follower",
        config=load_live_follow_config(LIVE_CONFIG),
        joint_limits=JOINT_LIMITS,
        controller_factory=lambda port, baudrate: controller,
        clock=clock,
        sleep=clock.sleep,
        identity_factory=lambda path: PortIdentity(path, path, 166),
        identity_checker=lambda identity: None,
    )
    return controller, follower


def test_verify_pos_vel_configuration_reads_mode_and_gains_without_writes() -> None:
    controller, follower = make_follower()
    follower.open()

    follower.verify_pos_vel_configuration()

    for motor in controller.motors[:6]:
        assert motor.register_reads == [
            ("u32", 10, 250),
            ("f32", 25, 250),
            ("f32", 26, 250),
            ("f32", 27, 250),
            ("f32", 28, 250),
        ]
        assert motor.register_writes == []
        assert motor.ensure_modes == []
    assert controller.motors[6].register_reads == []


def test_verify_pos_vel_configuration_rejects_wrong_mode_before_enable() -> None:
    controller, follower = make_follower()
    follower.open()
    controller.motors[2].register_u32[10] = 1

    with pytest.raises(
        FollowerLifecycleError,
        match=r"joint3 RID 10 模式不符：期望 2，实际 1",
    ):
        follower.verify_pos_vel_configuration()

    assert all(motor.enable_calls == 0 for motor in controller.motors)
    assert all(not motor.pos_vel_calls for motor in controller.motors)


def test_verify_pos_vel_configuration_accepts_finite_nonnegative_persisted_gain() -> None:
    controller, follower = make_follower()
    follower.open()
    controller.motors[1].register_f32[25] = 0.0130000003

    follower.verify_pos_vel_configuration()

    assert all(motor.enable_calls == 0 for motor in controller.motors)
    assert all(not motor.pos_vel_calls for motor in controller.motors)
    assert all(motor.register_writes == [] for motor in controller.motors)


@pytest.mark.parametrize("invalid_gain", [-0.001, float("nan"), float("inf")])
def test_verify_pos_vel_configuration_rejects_invalid_gain_before_enable(
    invalid_gain: float,
) -> None:
    controller, follower = make_follower()
    follower.open()
    controller.motors[4].register_f32[27] = invalid_gain

    with pytest.raises(
        FollowerLifecycleError,
        match=r"joint5 RID 27 参数无效.*要求有限且非负",
    ):
        follower.verify_pos_vel_configuration()

    assert all(motor.enable_calls == 0 for motor in controller.motors)
    assert all(not motor.pos_vel_calls for motor in controller.motors)


def test_verify_pos_vel_configuration_surfaces_register_read_failure() -> None:
    controller, follower = make_follower()
    follower.open()

    def fail_read(_register: int, _timeout_ms: int = 1000) -> float:
        raise RuntimeError("register timeout")

    controller.motors[3].get_register_f32 = fail_read

    with pytest.raises(
        FollowerLifecycleError,
        match=r"joint4 POS_VEL 配置只读校验失败：register timeout",
    ):
        follower.verify_pos_vel_configuration()

    assert all(motor.enable_calls == 0 for motor in controller.motors)
    assert all(not motor.pos_vel_calls for motor in controller.motors)


def test_follower_controller_exposes_no_register_writing_prepare_path() -> None:
    _, follower = make_follower()

    assert not hasattr(follower, "prepare_pos_vel")


def test_enable_first_target_is_exact_current_feedback_and_gripper_stays_disabled() -> None:
    controller, follower = make_follower()
    follower.open()

    result = follower.enable_hold(0.5)

    assert result.state.positions_rad == pytest.approx(START)
    assert result.command_rad == pytest.approx(START)
    assert [motor.pos_vel_calls[0] for motor in controller.motors[:6]] == pytest.approx(
        [(position, 0.5) for position in START]
    )
    assert [motor.enable_calls for motor in controller.motors[:6]] == [1] * 6
    assert controller.motors[6].enable_calls == 0
    assert controller.motors[6].state.status_code == 0


def test_enable_rejects_out_of_limit_current_before_enabling() -> None:
    positions = list(START)
    positions[1] = 0.021
    controller, follower = make_follower(positions=positions)
    follower.open()

    with pytest.raises(ValueError, match="joint2.*网页关节边界"):
        follower.enable_hold(0.5)

    assert all(motor.enable_calls == 0 for motor in controller.motors[:6])
    assert all(not motor.pos_vel_calls for motor in controller.motors[:6])


def test_partial_enable_failure_rolls_back_and_verifies_all_six_disabled() -> None:
    controller, follower = make_follower(enable_failure_at="joint4")
    follower.open()

    with pytest.raises(FollowerLifecycleError, match="joint4"):
        follower.enable_hold(0.5)

    assert [motor.state.status_code for motor in controller.motors[:6]] == [0] * 6
    assert [motor.disable_calls for motor in controller.motors[:6]] == [1] * 6


def test_enable_waits_for_fresh_status_instead_of_reusing_cached_zero() -> None:
    controller, follower = make_follower(
        enable_status_delays={"joint1": 2, "joint4": 3}
    )
    follower.open()

    result = follower.enable_hold(0.5)

    assert result.state.status_codes == (1,) * 6
    assert controller.motors[0].request_calls >= 2
    assert controller.motors[3].request_calls >= 3


def test_disable_waits_for_fresh_status_instead_of_reusing_cached_one() -> None:
    controller, follower = make_follower(
        disable_status_delays={"joint1": 2, "joint5": 3}
    )
    follower.open()
    follower.enable_hold(0.5)

    state = follower.disable_verified()

    assert state.status_codes == (0,) * 6
    assert controller.motors[0].state.status_code == 0
    assert controller.motors[4].state.status_code == 0


def test_cycle_surfaces_joint_write_failure_without_swallowing_it() -> None:
    controller, follower = make_follower(write_failure_at="joint5")
    follower.open()
    for motor in controller.motors[:6]:
        motor.state.status_code = 1

    with pytest.raises(FollowerCommunicationError, match="joint5") as error:
        follower.cycle(START, 0.5)

    assert isinstance(error.value.__cause__, RuntimeError)


def test_cycle_rejects_out_of_limit_target_before_any_motor_write() -> None:
    controller, follower = make_follower()
    follower.open()
    for motor in controller.motors[:6]:
        motor.state.status_code = 1
    target = list(START)
    target[1] = 0.021

    with pytest.raises(ValueError, match="joint2.*网页关节边界"):
        follower.cycle(target, 0.5)

    assert all(not motor.pos_vel_calls for motor in controller.motors[:6])


def test_hold_uses_last_safe_command_when_feedback_is_out_of_limit() -> None:
    positions = list(START)
    positions[1] = 0.03
    controller, follower = make_follower(positions=positions)
    follower.open()
    for motor in controller.motors[:6]:
        motor.state.status_code = 1

    result = follower.hold_current(0.5, fallback_target_rad=START)

    assert result.used_fallback is True
    assert result.command_rad == pytest.approx(START)
    assert [motor.pos_vel_calls[-1] for motor in controller.motors[:6]] == pytest.approx(
        [(position, 0.5) for position in START]
    )


def test_feedback_transaction_rejects_stale_completion() -> None:
    controller, follower = make_follower(poll_advance_s=0.04)
    follower.open()

    with pytest.raises(FollowerCommunicationError, match="0.25"):
        follower.read_state(expected_arm_status=0)


def test_feedback_transaction_retries_a_delayed_first_frame() -> None:
    controller, follower = make_follower(
        feedback_ready_after={"joint3": 2, "joint4": 3}
    )
    follower.open()

    state = follower.read_state(expected_arm_status=0)

    assert state.positions_rad == pytest.approx(START)
    assert controller.motors[2].request_calls == 2
    assert controller.motors[3].request_calls == 3


def test_feedback_transaction_reports_axis_after_bounded_retries() -> None:
    controller, follower = make_follower(feedback_ready_after={"joint3": 4})
    follower.open()

    with pytest.raises(FollowerCommunicationError, match="joint3.*重试 3 次"):
        follower.read_state(expected_arm_status=0)

    assert controller.motors[2].request_calls == 3


def test_disable_verified_disables_each_arm_motor_but_not_gripper() -> None:
    controller, follower = make_follower()
    follower.open()
    for motor in controller.motors[:6]:
        motor.state.status_code = 1

    follower.disable_verified()

    assert [motor.disable_calls for motor in controller.motors[:6]] == [1] * 6
    assert controller.motors[6].disable_calls == 0
    assert [motor.state.status_code for motor in controller.motors[:6]] == [0] * 6


def test_close_only_releases_handles_without_hidden_lifecycle_calls() -> None:
    controller, follower = make_follower()
    follower.open()

    follower.close()

    assert all(motor.closed for motor in controller.motors)
    assert controller.closed is True
    assert controller.shutdown_calls == 0
    assert controller.disable_all_calls == 0
    assert controller.enable_all_calls == 0
    assert [motor.disable_calls for motor in controller.motors] == [0] * 7

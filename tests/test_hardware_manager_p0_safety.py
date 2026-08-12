from __future__ import annotations

from pathlib import Path
import threading
from types import SimpleNamespace
from xml.etree import ElementTree

import numpy as np
import pytest

from rebotarmcontroller.hardware_manager import (
    HardwareManager,
    _JOINT_POSITION_LIMITS_RAD,
    _G_ANGLE_OPEN,
    _G_MAX_DIST_M,
    _G_VERIFIED_OPEN_LIMIT_M,
)


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


class FakeMotor:
    def __init__(self, position: float = 0.0) -> None:
        self.request_feedback_calls = 0
        self.state = SimpleNamespace(
            pos=float(position),
            vel=0.0,
            torq=0.0,
            status_code=0,
        )
        self.set_zero_position_calls = 0

    def get_state(self):
        return self.state

    def request_feedback(self) -> None:
        self.request_feedback_calls += 1

    def set_zero_position(self) -> None:
        self.set_zero_position_calls += 1


class FakeArm:
    def __init__(self, *, fail_enable_joint: str | None = None) -> None:
        self.joint_names = list(JOINT_NAMES)
        self.num_joints = len(self.joint_names)
        self._joints = [SimpleNamespace(name=name) for name in self.joint_names]
        self._motor_map = {name: FakeMotor() for name in self.joint_names}
        self._ctrl_map = {}
        self.mode = "mit"
        self.control_loop_active = False
        self.connect_calls = 0
        self.enable_calls = 0
        self.disable_calls = 0
        self.disconnect_calls = 0
        self.mode_pos_vel_calls = 0
        self.start_control_loop_calls = 0
        self.fail_enable_joint = fail_enable_joint

    def connect(self) -> None:
        self.connect_calls += 1

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.control_loop_active = False

    def _request_and_poll(self) -> None:
        for motor in self._motor_map.values():
            motor.request_feedback()

    def get_state(self):
        states = [self._motor_map[name].state for name in self.joint_names]
        return (
            np.array([state.pos for state in states], dtype=np.float64),
            np.array([state.vel for state in states], dtype=np.float64),
            np.array([state.torq for state in states], dtype=np.float64),
        )

    def get_positions(self, request: bool = False):
        del request
        return self.get_state()[0]

    def enable(self) -> None:
        self.enable_calls += 1
        for name, motor in self._motor_map.items():
            motor.state.status_code = 0 if name == self.fail_enable_joint else 1

    def disable(self) -> None:
        self.disable_calls += 1
        for motor in self._motor_map.values():
            motor.state.status_code = 0
        self.control_loop_active = False

    def mode_pos_vel(self) -> bool:
        self.mode_pos_vel_calls += 1
        self.mode = "pos_vel"
        return True

    def start_control_loop(self, _callback) -> None:
        self.start_control_loop_calls += 1
        self.control_loop_active = True

    def stop_control_loop(self) -> None:
        self.control_loop_active = False


def make_manager(*, fail_enable_joint: str | None = None) -> HardwareManager:
    manager = HardwareManager.__new__(HardwareManager)
    manager._arm = FakeArm(fail_enable_joint=fail_enable_joint)
    manager._endpos_ctrl = SimpleNamespace(
        _q_target=np.full(6, 99.0, dtype=np.float64),
        _running=False,
        _moving=False,
        _stop_send=SimpleNamespace(set=lambda: None),
        _loop_cb=lambda _arm, _dt: None,
    )
    manager._connected = False
    manager._enabled = False
    manager._lifecycle_state = "DISCONNECTED"
    manager._state_machine = "IDLE"
    manager._error_codes = []
    manager._gripper_cfg_path = Path("unused-gripper.yaml")
    manager._gripper_cfg = None
    manager._gripper_mot = None
    manager._gripper_ctrl = None
    manager._gripper_active = False
    manager._gripper_mode = "idle"
    manager._gripper_lock = threading.Lock()
    manager._motor_lifecycle_lock = threading.RLock()
    manager._gravity_comp_active = False
    manager._gravity_comp_q_target = None
    manager._gravity_comp_integral = None
    manager._gravity_comp_lock_counter = 0
    manager._gravity_comp_q_last = None
    manager.init_gripper = lambda _path: None
    manager._stop_gripper_loop = lambda: None
    manager._start_gripper_loop = lambda: None
    return manager


def test_connect_keeps_all_motors_disabled_and_does_not_start_control() -> None:
    manager = make_manager()

    manager.connect()

    assert manager.connected is True
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.connect_calls == 1
    assert manager.arm.enable_calls == 0
    assert manager.arm.disable_calls == 0
    assert manager.arm.mode_pos_vel_calls == 0
    assert manager.arm.start_control_loop_calls == 0
    assert manager.get_joint_status_codes() == [0] * 6


def test_disabled_joint_state_reads_request_fresh_feedback() -> None:
    manager = make_manager()
    manager.connect()
    requests_before = manager.arm._motor_map["joint1"].request_feedback_calls

    manager.get_joint_state()

    assert manager.arm._motor_map["joint1"].request_feedback_calls == requests_before + 1


def test_enabled_joint_state_reads_still_request_fresh_feedback() -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    requests_before = manager.arm._motor_map["joint1"].request_feedback_calls

    manager.get_joint_state()

    assert manager.arm._motor_map["joint1"].request_feedback_calls == requests_before + 1


def test_connect_forces_unexpected_enabled_motors_back_to_disabled() -> None:
    manager = make_manager()
    manager.arm._motor_map["joint3"].state.status_code = 1

    manager.connect()

    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.disable_calls == 1
    assert manager.get_joint_status_codes() == [0] * 6


def test_enable_holds_current_position_before_starting_control_loop() -> None:
    manager = make_manager()
    current = np.array([0.2, -0.4, -0.3, 0.1, 0.0, 0.3])
    for name, position in zip(JOINT_NAMES, current):
        manager.arm._motor_map[name].state.pos = float(position)
    manager.connect()

    manager.enable()

    assert manager.enabled is True
    assert manager.lifecycle_state == "ENABLED_HOLD"
    assert manager.arm.enable_calls == 1
    assert manager.arm.mode_pos_vel_calls == 1
    assert manager.arm.start_control_loop_calls == 1
    assert np.allclose(manager.endpos_ctrl._q_target, current)
    assert manager.get_joint_status_codes() == [1] * 6
    assert manager.ready_for_motion is True


def test_enable_failure_disables_every_motor_and_returns_to_disabled_state() -> None:
    manager = make_manager(fail_enable_joint="joint4")
    manager.connect()
    disable_calls_before_enable = manager.arm.disable_calls

    with pytest.raises(RuntimeError, match="joint4"):
        manager.enable()

    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.disable_calls > disable_calls_before_enable
    assert manager.arm.control_loop_active is False
    assert manager.get_joint_status_codes() == [0] * 6


def test_motion_entrypoints_do_not_implicitly_enable() -> None:
    manager = make_manager()
    manager.connect()

    with pytest.raises(RuntimeError, match="explicit /rebotarm/enable"):
        manager.ensure_pos_vel_control()
    with pytest.raises(RuntimeError, match="explicit /rebotarm/enable"):
        manager.start_gravity_compensation()

    assert manager.arm.enable_calls == 0
    assert manager.ready_for_motion is False


def test_stop_active_motion_is_safe_during_concurrent_disable() -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    disable_started = threading.Event()
    allow_disable = threading.Event()
    original_disable_all_motors = manager._disable_all_motors
    errors: list[BaseException] = []

    def delayed_disable_all_motors() -> None:
        disable_started.set()
        assert allow_disable.wait(timeout=1.0)
        original_disable_all_motors()

    def record_errors(operation) -> None:
        try:
            operation()
        except BaseException as exc:  # pragma: no cover - assertion reports details
            errors.append(exc)

    manager._disable_all_motors = delayed_disable_all_motors
    disable_thread = threading.Thread(
        target=record_errors,
        args=(manager.disable,),
    )
    stop_thread = threading.Thread(
        target=record_errors,
        args=(manager.stop_active_motion,),
    )

    disable_thread.start()
    assert disable_started.wait(timeout=1.0)
    stop_thread.start()
    allow_disable.set()
    disable_thread.join(timeout=1.0)
    stop_thread.join(timeout=1.0)

    assert disable_thread.is_alive() is False
    assert stop_thread.is_alive() is False
    assert errors == []
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.state_machine == "IDLE"
    assert manager.get_joint_status_codes() == [0] * 6


def test_joint_state_read_waits_for_disable_transition() -> None:
    manager = make_manager()
    manager.connect()
    manager.enable()
    disable_started = threading.Event()
    allow_disable = threading.Event()
    original_disable_all_motors = manager._disable_all_motors
    errors: list[BaseException] = []
    reads: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []

    def delayed_disable_all_motors() -> None:
        disable_started.set()
        assert allow_disable.wait(timeout=1.0)
        original_disable_all_motors()

    def disable() -> None:
        try:
            manager.disable()
        except BaseException as exc:  # pragma: no cover - assertion reports details
            errors.append(exc)

    def read_joint_state() -> None:
        try:
            reads.append(manager.get_joint_state())
        except BaseException as exc:  # pragma: no cover - assertion reports details
            errors.append(exc)

    manager._disable_all_motors = delayed_disable_all_motors
    disable_thread = threading.Thread(target=disable)
    read_thread = threading.Thread(target=read_joint_state)

    disable_thread.start()
    assert disable_started.wait(timeout=1.0)
    read_thread.start()
    allow_disable.set()
    disable_thread.join(timeout=1.0)
    read_thread.join(timeout=1.0)

    assert disable_thread.is_alive() is False
    assert read_thread.is_alive() is False
    assert errors == []
    assert len(reads) == 1
    assert manager.enabled is False
    assert manager.get_joint_status_codes() == [0] * 6


def test_action_goal_callback_rejects_motion_until_explicit_enable() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "src/rebotarmcontroller/rebotarmcontroller/ros_actions.py"
    ).read_text(encoding="utf-8")

    assert "if not self._hardware.ready_for_motion:" in source
    assert "return GoalResponse.REJECT" in source


def test_connect_rejects_missing_or_out_of_limit_feedback_and_disconnects() -> None:
    manager = make_manager()
    manager.arm._motor_map["joint2"].state = None

    with pytest.raises(RuntimeError, match="joint2 feedback unavailable"):
        manager.connect()
    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"
    assert manager.arm.disconnect_calls == 1

    manager = make_manager()
    manager.arm._motor_map["joint5"].state.pos = 2.0
    with pytest.raises(RuntimeError, match="joint5 position"):
        manager.connect()
    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"


def test_joint2_zero_margin_accepts_quantization_but_rejects_larger_positive_feedback() -> None:
    manager = make_manager()
    manager.arm._motor_map["joint2"].state.pos = 0.019

    manager.connect()

    assert manager.connected is True
    manager.shutdown()

    manager = make_manager()
    manager.arm._motor_map["joint2"].state.pos = 0.021

    with pytest.raises(RuntimeError, match="joint2 position"):
        manager.connect()

    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"


def test_connect_does_not_swallow_active_feedback_refresh_failure() -> None:
    manager = make_manager()
    controller = SimpleNamespace(poll_feedback_once=lambda: None)
    manager.arm._ctrl_map = {"fake": controller}
    for joint in manager.arm._joints:
        joint.vendor = "fake"

    def fail_request() -> None:
        raise RuntimeError("feedback timeout")

    manager.arm._motor_map["joint3"].request_feedback = fail_request

    with pytest.raises(RuntimeError, match="arm feedback refresh failed"):
        manager.connect()

    assert manager.connected is False
    assert manager.lifecycle_state == "DISCONNECTED"
    assert manager.arm.disconnect_calls == 1


def test_feedback_refresh_requests_and_polls_each_motor_sequentially() -> None:
    manager = make_manager()
    events: list[str] = []
    controller = SimpleNamespace(
        poll_feedback_once=lambda: events.append("poll"),
    )
    manager.arm._ctrl_map = {"fake": controller}
    for joint in manager.arm._joints:
        joint.vendor = "fake"
        motor = manager.arm._motor_map[joint.name]
        motor.request_feedback = (
            lambda name=joint.name: events.append(f"request:{name}")
        )

    manager._refresh_arm_feedback()

    expected: list[str] = []
    for name in JOINT_NAMES:
        expected.extend((f"request:{name}", "poll"))
    assert events == expected


def test_feedback_refresh_holds_controller_lock_for_full_transaction() -> None:
    manager = make_manager()
    events: list[str] = []

    class RecordingLock:
        def __enter__(self):
            events.append("lock:enter")

        def __exit__(self, _exc_type, _exc, _traceback):
            events.append("lock:exit")

    controller = SimpleNamespace(
        _bus_lock=RecordingLock(),
        poll_feedback_once=lambda: events.append("poll"),
    )
    manager.arm._ctrl_map = {"fake": controller}
    for joint in manager.arm._joints:
        joint.vendor = "fake"
        motor = manager.arm._motor_map[joint.name]
        motor.request_feedback = (
            lambda name=joint.name: events.append(f"request:{name}")
        )

    manager._refresh_arm_feedback()

    assert events[0] == "lock:enter"
    assert events[-1] == "lock:exit"
    assert events.count("lock:enter") == 1
    assert events.count("lock:exit") == 1


def test_gripper_feedback_refresh_retries_until_state_arrives() -> None:
    manager = make_manager()
    calls = {"request": 0, "poll": 0}
    state_holder = {"state": None}

    def request_feedback() -> None:
        calls["request"] += 1

    def poll_feedback_once() -> None:
        calls["poll"] += 1
        if calls["poll"] == 2:
            state_holder["state"] = SimpleNamespace(status_code=0)

    manager._gripper_mot = SimpleNamespace(
        request_feedback=request_feedback,
        get_state=lambda: state_holder["state"],
    )
    manager._gripper_ctrl = SimpleNamespace(
        poll_feedback_once=poll_feedback_once,
    )

    manager._refresh_all_feedback()

    assert calls == {"request": 2, "poll": 2}


def test_gripper_mode_is_set_before_explicit_gripper_enable() -> None:
    manager = make_manager()
    events: list[str] = []
    state = SimpleNamespace(
        status_code=0,
        pos=-1.0,
        vel=0.0,
        torq=0.0,
    )

    def ensure_mode(_mode, _timeout_ms: int) -> None:
        events.append("gripper_mode")

    def gripper_enable() -> None:
        events.append("gripper_enable")
        state.status_code = 1

    def gripper_disable() -> None:
        state.status_code = 0

    manager._gripper_mot = SimpleNamespace(
        request_feedback=lambda: None,
        get_state=lambda: state,
        ensure_mode=ensure_mode,
        enable=gripper_enable,
        disable=gripper_disable,
    )
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    original_arm_enable = manager.arm.enable

    def arm_enable() -> None:
        events.append("arm_enable")
        original_arm_enable()

    manager.arm.enable = arm_enable
    manager._start_gripper_loop = lambda: events.append("gripper_loop")
    manager.connect()

    manager.enable()

    assert events == ["gripper_mode", "arm_enable", "gripper_enable"]
    assert manager.ready_for_motion is True
    manager.set_gripper_target(0.05)
    assert events[-1] == "gripper_loop"
    manager.disable()


def _configure_arrived_gripper_position_move(manager: HardwareManager) -> list[tuple[float, ...]]:
    commands: list[tuple[float, ...]] = []
    state = SimpleNamespace(pos=-4.72, vel=0.0, torq=0.0, status_code=1)

    def send_mit(pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        commands.append((pos, vel, kp, kd, tau))

    manager._gripper_mot = SimpleNamespace(
        get_state=lambda: state,
        send_mit=send_mit,
        request_feedback=lambda: None,
    )
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    manager._gripper_pos = state.pos
    manager._gripper_vel = state.vel
    manager._gripper_torque = state.torq
    manager._gripper_target_angle = state.pos
    manager._gripper_goal_angle = state.pos
    manager._gripper_target_effort = 0.15
    manager._gripper_close_force = 0.4
    manager._gripper_hold_force = 0.4
    manager._gripper_hold_angle = state.pos
    manager._gripper_mode = "position"
    manager._gripper_active = True
    return commands


def test_completed_gripper_position_move_sends_neutral_and_becomes_idle() -> None:
    manager = make_manager()
    commands = _configure_arrived_gripper_position_move(manager)

    assert manager.wait_gripper_target(timeout=0.01) is True
    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]


def test_gripper_tick_releases_completed_position_move_without_waiter() -> None:
    manager = make_manager()
    commands = _configure_arrived_gripper_position_move(manager)

    manager._gripper_tick()

    assert manager.gripper_active is False
    assert manager.gripper_mode == "idle"
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]
    manager._gripper_tick()
    assert commands == [(-4.72, 0.0, 0.0, 0.0, 0.0)]


def test_gripper_position_target_cannot_exceed_verified_85mm_limit() -> None:
    manager = make_manager()
    manager._connected = True
    manager._enabled = True
    manager._gripper_pos = 0.0
    manager._gripper_mot = SimpleNamespace(
        get_state=lambda: SimpleNamespace(pos=0.0),
    )
    manager._start_gripper_loop = lambda: None

    manager.set_gripper_target(0.09, max_effort=0.15)

    assert manager._gripper_goal_angle == pytest.approx(
        (_G_VERIFIED_OPEN_LIMIT_M / _G_MAX_DIST_M) * _G_ANGLE_OPEN
    )


def test_hardware_soft_limits_match_moveit_urdf() -> None:
    root = Path(__file__).resolve().parents[1]
    urdf_root = ElementTree.parse(
        root / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    ).getroot()
    urdf_limits = {
        joint.attrib["name"]: (
            float(joint.find("limit").attrib["lower"]),
            float(joint.find("limit").attrib["upper"]),
        )
        for joint in urdf_root.findall("joint")
        if joint.attrib.get("name") in _JOINT_POSITION_LIMITS_RAD
    }

    assert urdf_limits == _JOINT_POSITION_LIMITS_RAD


def test_gripper_initialization_contains_no_enable_or_control_loop_start() -> None:
    import inspect

    source = inspect.getsource(HardwareManager.init_gripper)

    assert "enable_all" not in source
    assert "ensure_mode" not in source
    assert "_start_gripper_loop" not in source


def test_set_zero_gripper_only_calibrates_disabled_gripper_motor() -> None:
    manager = make_manager()
    gripper = FakeMotor(position=0.0)
    manager._gripper_mot = gripper
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    manager.connect()

    assert manager.set_zero("gripper") is True
    assert gripper.set_zero_position_calls == 1
    assert manager.enabled is False
    assert manager.lifecycle_state == "CONNECTED_DISABLED"
    assert manager.arm.disable_calls == 0


def test_set_zero_gripper_rejects_enabled_arm() -> None:
    manager = make_manager()
    manager._gripper_mot = FakeMotor(position=0.0)
    manager._gripper_ctrl = SimpleNamespace(poll_feedback_once=lambda: None)
    manager.connect()
    manager._enabled = True

    with pytest.raises(RuntimeError, match="CONNECTED_DISABLED"):
        manager.set_zero("gripper")

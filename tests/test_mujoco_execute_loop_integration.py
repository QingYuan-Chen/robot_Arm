from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import threading

import pytest


ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


class _FakeAdapter:
    def __init__(
        self,
        *,
        advance_sim_time: bool = True,
        follow_targets: bool = True,
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> None:
        self._sim_time = -0.1 if advance_sim_time else 0.0
        self._advance_sim_time = advance_sim_time
        self._follow_targets = follow_targets
        self._positions = [0.0] * 6
        self._velocities = list(velocities or [0.0] * 6)
        self._efforts = list(efforts or [0.0] * 6)
        self.targets: list[list[float]] = []

    @property
    def sim_time(self) -> float:
        if self._advance_sim_time:
            self._sim_time += 0.1
        return self._sim_time

    def arm_positions(self) -> list[float]:
        return list(self._positions)

    def set_arm_targets(self, targets) -> None:
        values = [float(value) for value in targets]
        self.targets.append(values)
        if self._follow_targets:
            self._positions = list(values)

    def arm_velocities(self) -> list[float]:
        return list(self._velocities)

    def arm_actuator_forces(self) -> list[float]:
        return list(self._efforts)


class _FakeGoalHandle:
    def __init__(self, *, target: float = 0.2, canceled: bool = False) -> None:
        point = SimpleNamespace(
            positions=[target] * 6,
            time_from_start=SimpleNamespace(sec=0, nanosec=200_000_000),
        )
        self.request = SimpleNamespace(
            trajectory=SimpleNamespace(
                joint_names=[f"joint{index}" for index in range(1, 7)],
                points=[point],
            )
        )
        self.is_cancel_requested = canceled
        self.terminal_state: str | None = None
        self.feedback_count = 0

    def publish_feedback(self, _feedback) -> None:
        self.feedback_count += 1

    def succeed(self) -> None:
        self.terminal_state = "succeeded"

    def abort(self) -> None:
        self.terminal_state = "aborted"

    def canceled(self) -> None:
        self.terminal_state = "canceled"


def _build_node(tmp_path: Path, adapter: _FakeAdapter):
    module = _load_node_module()
    MuJoCoRosAdapterNode = module.MuJoCoRosAdapterNode

    node = MuJoCoRosAdapterNode.__new__(MuJoCoRosAdapterNode)
    node._adapter = adapter
    node._lock = threading.RLock()
    node._stop_requested = threading.Event()
    node._metrics_dir = tmp_path
    node._metrics_sample_stride = 1
    node._control_rate_hz = 1_000_000.0
    node._path_tolerance_rad = 0.0
    node._goal_tolerance_rad = 0.0
    node._execution_timeout_margin_sec = 1.0
    node._latest_targets = [0.0] * 6
    return node


def _load_node_module():
    pytest.importorskip("rclpy")
    pytest.importorskip("control_msgs")
    pytest.importorskip("rebotarm_msgs")
    pytest.importorskip("mujoco")
    from rebotarm_simulation import mujoco_ros_adapter_node

    return mujoco_ros_adapter_node


def test_execute_loop_success_reaches_succeeded_terminal_state(tmp_path: Path, monkeypatch) -> None:
    node_module = _load_node_module()

    adapter = _FakeAdapter()
    node = _build_node(tmp_path, adapter)
    goal = _FakeGoalHandle()
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: None)

    result = node._execute_goal(goal)

    assert goal.terminal_state == "succeeded"
    assert result.error_code == result.SUCCESSFUL
    assert result.error_string == "MuJoCo trajectory finished"
    assert goal.feedback_count >= 1


@pytest.mark.parametrize("stop_source", ["cancel", "stop"])
def test_execute_loop_cancel_and_stop_hold_current_position(
    tmp_path: Path, monkeypatch, stop_source: str
) -> None:
    node_module = _load_node_module()

    adapter = _FakeAdapter(follow_targets=False)
    node = _build_node(tmp_path, adapter)
    goal = _FakeGoalHandle(canceled=stop_source == "cancel")
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    if stop_source == "stop":
        monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: node._stop_requested.set())
    else:
        monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: None)

    result = node._execute_goal(goal)

    assert goal.terminal_state == "canceled"
    assert result.error_code != result.SUCCESSFUL
    assert result.error_string == f"MuJoCo trajectory {'canceled' if stop_source == 'cancel' else 'stopped'}"
    assert adapter.targets[-1] == pytest.approx(adapter.arm_positions())


def test_execute_loop_path_tolerance_failure_holds_current_position(tmp_path: Path, monkeypatch) -> None:
    node_module = _load_node_module()

    adapter = _FakeAdapter(follow_targets=False)
    node = _build_node(tmp_path, adapter)
    node._path_tolerance_rad = 0.05
    goal = _FakeGoalHandle(target=1.0)
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: None)

    result = node._execute_goal(goal)

    assert goal.terminal_state == "aborted"
    assert result.error_code == result.PATH_TOLERANCE_VIOLATED
    assert result.error_string == "MuJoCo trajectory path_tolerance_violated"
    assert adapter.targets[-1] == pytest.approx(adapter.arm_positions())


def test_execute_loop_goal_tolerance_failure_holds_current_position(tmp_path: Path, monkeypatch) -> None:
    node_module = _load_node_module()

    adapter = _FakeAdapter(follow_targets=False)
    node = _build_node(tmp_path, adapter)
    node._goal_tolerance_rad = 0.05
    goal = _FakeGoalHandle(target=1.0)
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: None)

    result = node._execute_goal(goal)

    assert goal.terminal_state == "aborted"
    assert result.error_code == result.GOAL_TOLERANCE_VIOLATED
    assert result.error_string == "MuJoCo trajectory goal_tolerance_violated"
    assert adapter.targets[-1] == pytest.approx(adapter.arm_positions())


def test_execute_loop_timeout_aborts_and_holds_current_position(tmp_path: Path, monkeypatch) -> None:
    node_module = _load_node_module()

    adapter = _FakeAdapter(advance_sim_time=False, follow_targets=False)
    node = _build_node(tmp_path, adapter)
    node._execution_timeout_margin_sec = 0.0
    goal = _FakeGoalHandle()
    clock = iter((0.0, 1.0))
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_module.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: None)

    result = node._execute_goal(goal)

    assert goal.terminal_state == "aborted"
    assert result.error_code == result.PATH_TOLERANCE_VIOLATED
    assert result.error_string == "MuJoCo trajectory timeout"
    assert adapter.targets[-1] == pytest.approx(adapter.arm_positions())


def test_execute_loop_runtime_limit_violation_aborts_and_holds(tmp_path: Path, monkeypatch) -> None:
    node_module = _load_node_module()
    from rebotarm_motion.trajectory_runtime_limits import (
        JointRuntimeLimit,
        TrajectoryRuntimeLimitGuard,
    )

    adapter = _FakeAdapter(follow_targets=False, velocities=[1.0] * 6)
    node = _build_node(tmp_path, adapter)
    node._runtime_limit_guard = TrajectoryRuntimeLimitGuard(
        {
            f"joint{index}": JointRuntimeLimit(max_velocity=0.5)
            for index in range(1, 7)
        }
    )
    goal = _FakeGoalHandle()
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_module.time, "sleep", lambda _seconds: None)

    result = node._execute_goal(goal)

    assert goal.terminal_state == "aborted"
    assert result.error_code == result.PATH_TOLERANCE_VIOLATED
    assert "runtime_limit_violated: velocity 1.000000 > 0.500000" in result.error_string
    assert adapter.targets[-1] == pytest.approx(adapter.arm_positions())

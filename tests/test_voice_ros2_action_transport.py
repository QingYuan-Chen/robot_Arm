from __future__ import annotations

from pathlib import Path
import sys
import types

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.models import SafetyViolationError
from rebotarm_voice_control.ros2_action_transport import Ros2ActionTransport
from rebotarm_voice_control.models import RouteResult
from rebotarm_voice_control.sim_executor import MoveIt2SimExecutor


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result


class _FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted


class _FakeActionClient:
    def __init__(self, node, action_type, action_name):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
        self.sent_goals = []
        self.server_ready = True
        self.goal_handle = _FakeGoalHandle(accepted=True)

    def wait_for_server(self, timeout_sec):
        self.timeout_sec = timeout_sec
        return self.server_ready

    def send_goal_async(self, goal_msg):
        self.sent_goals.append(goal_msg)
        return _FakeFuture(self.goal_handle)


class _ClientFactory:
    def __init__(self, client):
        self.client = client
        self.calls = []

    def __call__(self, node, action_type, action_name):
        self.calls.append((node, action_type, action_name))
        return self.client


def test_ros2_action_transport_sends_goal_with_builder():
    client = _FakeActionClient(None, None, "")
    factory = _ClientFactory(client)
    spun = []
    transport = Ros2ActionTransport(
        node="node",
        action_type_resolver=lambda action_name: f"type:{action_name}",
        action_client_factory=factory,
        goal_builder=lambda action_name, goal: {"action": action_name, "payload": goal},
        spin_until_future_complete=lambda node, future, timeout_sec: spun.append((node, future, timeout_sec)),
        wait_timeout_sec=1.5,
    )

    result = transport.send_action_goal("/rebotarm/sim/move_relative", {"axis": "z"})

    assert result == {
        "action_name": "/rebotarm/sim/move_relative",
        "goal_accepted": True,
        "status": "accepted",
    }
    assert factory.calls == [("node", "type:/rebotarm/sim/move_relative", "/rebotarm/sim/move_relative")]
    assert client.sent_goals == [{"action": "/rebotarm/sim/move_relative", "payload": {"axis": "z"}}]
    assert spun[0][0] == "node"


def test_ros2_action_transport_rejects_unavailable_server():
    client = _FakeActionClient(None, None, "")
    client.server_ready = False
    transport = Ros2ActionTransport(
        node="node",
        action_type_resolver=lambda action_name: object,
        action_client_factory=_ClientFactory(client),
        goal_builder=lambda action_name, goal: goal,
        spin_until_future_complete=lambda node, future, timeout_sec: None,
    )

    with pytest.raises(SafetyViolationError, match="action server unavailable"):
        transport.send_action_goal("/rebotarm/sim/move_relative", {"axis": "z"})


def test_ros2_action_transport_reports_rejected_goal():
    client = _FakeActionClient(None, None, "")
    client.goal_handle = _FakeGoalHandle(accepted=False)
    transport = Ros2ActionTransport(
        node="node",
        action_type_resolver=lambda action_name: object,
        action_client_factory=_ClientFactory(client),
        goal_builder=lambda action_name, goal: goal,
        spin_until_future_complete=lambda node, future, timeout_sec: None,
    )

    result = transport.send_action_goal("/rebotarm/sim/move_relative", {"axis": "z"})

    assert result["goal_accepted"] is False
    assert result["status"] == "rejected"


def test_moveit2_sim_executor_can_use_ros2_action_transport():
    client = _FakeActionClient(None, None, "")
    transport = Ros2ActionTransport(
        node="node",
        action_type_resolver=lambda action_name: f"type:{action_name}",
        action_client_factory=_ClientFactory(client),
        goal_builder=lambda action_name, goal: {"goal": goal},
        spin_until_future_complete=lambda node, future, timeout_sec: None,
    )
    executor = MoveIt2SimExecutor(transport=transport)

    result = executor.execute(
        RouteResult(
            "move_relative",
            "/rebotarm/sim/move_relative",
            "action",
            {"axis": "z", "distance_m": 0.05},
            dry_run=False,
        )
    )

    assert result.dispatched is True
    assert result.dispatch_result["goal_accepted"] is True
    assert client.sent_goals == [
        {"goal": {"intent": "move_relative", "axis": "z", "distance_m": 0.05}}
    ]


def test_default_spin_reports_incomplete_rclpy(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", types.SimpleNamespace())
    from rebotarm_voice_control import ros2_action_transport

    with pytest.raises(SafetyViolationError, match="spin_until_future_complete"):
        ros2_action_transport._default_spin_until_future_complete("node", _FakeFuture(None), 1.0)

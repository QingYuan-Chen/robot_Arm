from __future__ import annotations

from pathlib import Path
import json
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_voice_control_config
from rebotarm_voice_control.execution_modes import ExecutionModeRouter
from rebotarm_voice_control.models import IntentCommand, RouteResult, SafetyViolationError
from rebotarm_voice_control.sim_executor import (
    MoveIt2SimExecutor,
    RecordedSimExecutor,
    handle_sim_execution_json,
    main,
)


class _FakeMoveItTransport:
    def __init__(self):
        self.goals = []

    def send_action_goal(self, action_name, goal):
        self.goals.append((action_name, goal))
        return {"goal_id": "fake-goal-1", "status": "accepted"}


def test_recorded_sim_executor_accepts_sim_route():
    config = load_voice_control_config(SRC / "config")
    route = ExecutionModeRouter(config, execution_mode="sim").route(
        IntentCommand(
            intent="move_relative",
            command="move_relative",
            params={"axis": "z", "distance_m": 0.05},
        )
    )
    executor = RecordedSimExecutor()

    result = executor.execute(route.route)

    assert result.accepted is True
    assert result.dispatched is False
    assert result.backend == "recorded_sim"
    assert result.target == "/rebotarm/sim/move_relative"
    assert result.params["distance_m"] == 0.05


def test_recorded_sim_executor_rejects_dry_run_route():
    executor = RecordedSimExecutor()

    with pytest.raises(SafetyViolationError, match="dry-run route cannot be executed"):
        executor.execute(RouteResult("move_home", "/rebotarm/safe_home", "service", {}, dry_run=True))


def test_recorded_sim_executor_rejects_non_sim_namespace():
    executor = RecordedSimExecutor()

    with pytest.raises(SafetyViolationError, match="only accepts /rebotarm/sim routes"):
        executor.execute(RouteResult("move_home", "/rebotarm/safe_home", "service", {}, dry_run=False))


def test_handle_sim_execution_json_accepts_routed_payload():
    payload = {
        "route": {
            "intent": "move_relative",
            "target": "/rebotarm/sim/move_relative",
            "mode": "action",
            "params": {"axis": "z", "distance_m": 0.05},
            "dry_run": False,
        }
    }

    result = handle_sim_execution_json(json.dumps(payload))

    assert result["accepted"] is True
    assert result["backend"] == "recorded_sim"
    assert result["target"] == "/rebotarm/sim/move_relative"


def test_handle_sim_execution_json_accepts_backend_argument():
    payload = {
        "route": {
            "intent": "move_relative",
            "target": "/rebotarm/sim/move_relative",
            "mode": "action",
            "params": {"axis": "z", "distance_m": 0.05},
            "dry_run": False,
        }
    }

    result = handle_sim_execution_json(json.dumps(payload), backend="recorded")

    assert result["backend"] == "recorded_sim"


def test_moveit2_sim_executor_dispatches_relative_move_to_transport():
    transport = _FakeMoveItTransport()
    executor = MoveIt2SimExecutor(transport=transport)
    route = RouteResult(
        "move_relative",
        "/rebotarm/sim/move_relative",
        "action",
        {"axis": "z", "distance_m": 0.05},
        dry_run=False,
    )

    result = executor.execute(route)

    assert result.accepted is True
    assert result.dispatched is True
    assert result.backend == "moveit2_sim"
    assert result.dispatch_result == {"goal_id": "fake-goal-1", "status": "accepted"}
    assert transport.goals == [
        (
            "/rebotarm/sim/move_relative",
            {"intent": "move_relative", "axis": "z", "distance_m": 0.05},
        )
    ]


def test_moveit2_sim_executor_rejects_service_route_until_adapter_exists():
    executor = MoveIt2SimExecutor(transport=_FakeMoveItTransport())

    with pytest.raises(SafetyViolationError, match="only action routes are supported"):
        executor.execute(RouteResult("move_home", "/rebotarm/sim/safe_home", "service", {}, dry_run=False))


def test_sim_executor_cli_rejects_moveit2_backend_without_ros_node(tmp_path, capsys, monkeypatch):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps(
            {
                "route": {
                    "intent": "move_relative",
                    "target": "/rebotarm/sim/move_relative",
                    "mode": "action",
                    "params": {"axis": "z", "distance_m": 0.05},
                    "dry_run": False,
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["rebotarm_sim_executor", str(route_file), "--backend", "moveit2"],
    )

    with pytest.raises(SystemExit) as exc:
        main()

    assert exc.value.code == 1
    output = json.loads(capsys.readouterr().out)
    assert output["error"] == "SafetyViolationError"
    assert "ROS2 node is required" in output["message"]


def test_sim_executor_cli_uses_backend_from_config(tmp_path, capsys, monkeypatch):
    route_file = tmp_path / "route.json"
    config_root = tmp_path / "config"
    config_root.mkdir()
    (config_root / "sim_config.yaml").write_text(
        'backend: "moveit2"\nmoveit2: {}\n',
        encoding="utf-8",
    )
    route_file.write_text(
        json.dumps(
            {
                "route": {
                    "intent": "move_relative",
                    "target": "/rebotarm/sim/move_relative",
                    "mode": "action",
                    "params": {"axis": "z", "distance_m": 0.05},
                    "dry_run": False,
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["rebotarm_sim_executor", str(route_file), "--config-root", str(config_root)],
    )

    with pytest.raises(SystemExit) as exc:
        main()

    assert exc.value.code == 1
    output = json.loads(capsys.readouterr().out)
    assert output["error"] == "SafetyViolationError"
    assert "ROS2 node is required" in output["message"]

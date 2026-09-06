import ast
import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

from rebotarm_teach.teach_replay_workflow import TeachReplayWorkflow


ROOT = Path(__file__).resolve().parents[1]


def workflow_fixture(tmp_path):
    # Exercise the actual dashboard parameter contract without a ROS or HTTP server.
    source = ROOT / "src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py"
    parameters = {}
    for call in ast.walk(ast.parse(source.read_text())):
        if isinstance(call, ast.Call) and getattr(call.func, "attr", "") == "declare_parameter":
            try:
                parameters[ast.literal_eval(call.args[0])] = ast.literal_eval(call.args[1])
            except ValueError:
                pass
    parameters["record_path"] = str(tmp_path / "missing.jsonl")
    events = {}
    calls = []

    def call_async(request):
        calls.append(request)
        future = Future()
        future.set_result(SimpleNamespace(success=True, message="stopped"))
        return future

    client = SimpleNamespace(
        wait_for_service=lambda **kwargs: True, service_is_ready=lambda: True,
        call_async=call_async,
    )
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        create_client=lambda *args: client,
    )
    workflow = TeachReplayWorkflow(
        node=node, joint_names=tuple(parameters["joint_names"]),
        snapshot=lambda: SimpleNamespace(joints={}, teleop=events),
        publish_status=lambda channel, value: events.update({channel: value}),
        collision_defaults=lambda names: (), action_client=object(),
        trajectory_stop_client=client, request_stop=lambda **kwargs: True,
    )
    return workflow, events, calls


def test_workflow_owns_dry_run_failure_without_action_or_dashboard(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    result = workflow.dry_run({})
    assert not result["accepted"]
    assert workflow._last_teach_dry_run is None
    assert events["replay"] == result
    assert calls == []


def test_workflow_without_valid_dry_run_never_sends_goal(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    result = workflow.execute({})
    assert not result["accepted"]
    assert events["replay"] == result
    assert calls == []


def test_workflow_stop_uses_controller_fallback_without_active_goal(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    result = workflow.stop()
    assert result["accepted"]
    assert result["state"] == "stop_requested"
    assert len(calls) == 1
    assert events["replay"] == result


def test_workflow_prepares_dry_run_then_executes_through_injected_action(tmp_path):
    workflow, events, calls = workflow_fixture(tmp_path)
    names = tuple(f"joint{i}" for i in range(1, 7))
    path = tmp_path / "record.jsonl"
    path.write_text("\n".join(json.dumps({
        "stamp": i / 50., "joint_names": names,
        "positions": [i * .0001] * 6, "velocities": [0.] * 6, "efforts": [0.] * 6,
        "motor_status": dict.fromkeys(names, 1), "arm_state": "GRAVITY_COMP",
    }) for i in range(101)) + "\n")
    workflow._snapshot = lambda: SimpleNamespace(
        joints={name: {"position": 0.} for name in names}, teleop=events,
    )
    checked = []
    workflow._collision_prechecker = SimpleNamespace(
        check_positions=lambda **kwargs: checked.append(kwargs) or {"state": "pass", "message": "valid"},
    )
    pending = Future()
    goals = []
    workflow._action_client = SimpleNamespace(
        wait_for_server=lambda **kwargs: True,
        send_goal_async=lambda goal: goals.append(goal) or pending,
    )
    payload = {"record_path": str(path)}
    dry_run = workflow.dry_run(payload)
    assert dry_run["accepted"], dry_run
    assert goals == []
    result = workflow.execute(payload)
    assert result["accepted"], result
    assert len(goals) == 1
    assert goals[0].trajectory.points
    assert len(checked) == 2
    assert calls == []


def test_dashboard_no_longer_owns_replay_algorithms_or_planner():
    source = (ROOT / "src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py").read_text()
    for forbidden in ("MoveItMotionPlanner", "CollisionPrechecker", "ReplayRuntimeMonitor", "prepare_teach_replay_samples", "._client"):
        assert forbidden not in source
    assert "self._teach_workflow.execute(payload)" in source
    workflow_source = (ROOT / "src/rebotarm_teach/rebotarm_teach/teach_replay_workflow.py").read_text()
    assert "rebotarm_dashboard" not in workflow_source
    assert "_route_web_command" not in workflow_source

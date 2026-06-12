from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
import json
from pathlib import Path
import sys
from typing import Any, Protocol

from .config_loader import load_sim_config
from .models import RouteResult, SafetyViolationError
from .ros2_action_transport import Ros2ActionTransport
from .sim_action_bindings import build_sim_goal, resolve_sim_action_type


@dataclass(frozen=True)
class SimExecutionResult:
    accepted: bool
    dispatched: bool
    backend: str
    intent: str
    target: str
    mode: str
    params: dict[str, Any]
    message: str
    dispatch_result: dict[str, Any] | None = None


class SimActionTransport(Protocol):
    def send_action_goal(self, action_name: str, goal: dict[str, Any]) -> dict[str, Any]:
        """Send a sim action goal and return provider-specific dispatch metadata."""


class RecordedSimExecutor:
    """Validate and record sim commands without claiming MoveIt2 is connected yet."""

    backend = "recorded_sim"

    def execute(self, route: RouteResult) -> SimExecutionResult:
        _validate_sim_route(route)
        return SimExecutionResult(
            accepted=True,
            dispatched=False,
            backend=self.backend,
            intent=route.intent,
            target=route.target,
            mode=route.mode,
            params=dict(route.params),
            message="sim route accepted for recorded execution; MoveIt2 sim dispatch is not connected",
        )


class MoveIt2SimExecutor:
    """Adapter boundary for dispatching safe sim routes to a MoveIt2-style transport."""

    backend = "moveit2_sim"

    def __init__(self, transport: SimActionTransport):
        self._transport = transport

    def execute(self, route: RouteResult) -> SimExecutionResult:
        _validate_sim_route(route)
        if route.mode != "action":
            raise SafetyViolationError("MoveIt2 sim executor only action routes are supported")
        goal = self._build_goal(route)
        dispatch_result = self._transport.send_action_goal(route.target, goal)
        return SimExecutionResult(
            accepted=True,
            dispatched=True,
            backend=self.backend,
            intent=route.intent,
            target=route.target,
            mode=route.mode,
            params=dict(route.params),
            message="sim route dispatched through MoveIt2 transport adapter",
            dispatch_result=dict(dispatch_result),
        )

    def _build_goal(self, route: RouteResult) -> dict[str, Any]:
        goal = {"intent": route.intent}
        goal.update(dict(route.params))
        return goal


def _validate_sim_route(route: RouteResult) -> None:
    if route.dry_run:
        raise SafetyViolationError("dry-run route cannot be executed by sim executor")
    if not route.target.startswith("/rebotarm/sim"):
        raise SafetyViolationError("sim executor only accepts /rebotarm/sim routes")


def _route_from_payload(payload: dict[str, Any]) -> RouteResult:
    route = payload.get("route", payload)
    if not isinstance(route, dict):
        raise ValueError("sim execution payload must contain a route object")
    return RouteResult(
        intent=str(route["intent"]),
        target=str(route["target"]),
        mode=str(route["mode"]),
        params=dict(route.get("params", {})),
        dry_run=bool(route.get("dry_run", False)),
    )


def _create_sim_executor(
    backend: str,
    transport: SimActionTransport | None = None,
) -> RecordedSimExecutor | MoveIt2SimExecutor:
    if backend == "recorded":
        return RecordedSimExecutor()
    if backend == "moveit2":
        if transport is None:
            raise SafetyViolationError("MoveIt2 transport is not configured")
        return MoveIt2SimExecutor(transport=transport)
    raise SafetyViolationError(f"unsupported sim backend: {backend}")


def create_moveit2_sim_transport(ros_node: Any | None) -> Ros2ActionTransport:
    if ros_node is None:
        raise SafetyViolationError("ROS2 node is required for MoveIt2 sim transport")
    return Ros2ActionTransport(
        node=ros_node,
        action_type_resolver=resolve_sim_action_type,
        goal_builder=build_sim_goal,
    )


def handle_sim_execution_json(
    payload: str,
    backend: str = "recorded",
    transport: SimActionTransport | None = None,
) -> dict[str, Any]:
    loaded = json.loads(payload)
    if not isinstance(loaded, dict):
        raise ValueError("sim execution payload must be a JSON object")
    executor = _create_sim_executor(backend=backend, transport=transport)
    return asdict(executor.execute(_route_from_payload(loaded)))


def main() -> None:
    cli = argparse.ArgumentParser(description="Validate and record one /rebotarm/sim route.")
    cli.add_argument("json_file", help="Path to routed JSON, or '-' to read stdin.")
    cli.add_argument("--backend", choices=["recorded", "moveit2"])
    cli.add_argument("--config-root", default="")
    args = cli.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    config_root = Path(args.config_root) if args.config_root else package_root / "config"
    sim_config = load_sim_config(config_root)
    backend = args.backend or str(sim_config.get("backend", "recorded"))
    payload = (
        sys.stdin.read()
        if args.json_file == "-"
        else Path(args.json_file).read_text(encoding="utf-8-sig")
    )
    try:
        transport = create_moveit2_sim_transport(None) if backend == "moveit2" else None
        result = handle_sim_execution_json(payload, backend=backend, transport=transport)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

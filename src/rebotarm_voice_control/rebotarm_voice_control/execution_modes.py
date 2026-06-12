from __future__ import annotations

from .command_router import DryRunCommandRouter
from .models import ExecutionRouteResult, IntentCommand, RouteResult, SafetyViolationError, VoiceControlConfig


class ExecutionModeRouter:
    def __init__(self, config: VoiceControlConfig, execution_mode: str | None = None):
        self._config = config
        self.execution_mode = execution_mode or str(
            config.safety_limits.get("execution_mode", "dry_run")
        )
        self._dry_run_router = DryRunCommandRouter(config)

    def route(self, command: IntentCommand) -> ExecutionRouteResult:
        if self.execution_mode == "dry_run":
            return ExecutionRouteResult(
                execution_mode="dry_run",
                route=self._dry_run_router.route(command),
                simulated=False,
            )
        if self.execution_mode == "sim":
            return ExecutionRouteResult(
                execution_mode="sim",
                route=self._route_sim(command),
                simulated=True,
            )
        if self.execution_mode == "real":
            if not bool(self._config.safety_limits.get("allow_real_ros_calls", False)):
                raise SafetyViolationError("real ROS calls are disabled by safety_limits.yaml")
            dry_route = self._dry_run_router.route(command)
            return ExecutionRouteResult(
                execution_mode="real",
                route=RouteResult(
                    dry_route.intent,
                    dry_route.target,
                    dry_route.mode,
                    dry_route.params,
                    dry_run=False,
                ),
                simulated=False,
            )
        raise SafetyViolationError(f"unsupported execution mode: {self.execution_mode}")

    def _route_sim(self, command: IntentCommand) -> RouteResult:
        if command.command == "move_relative":
            return RouteResult(
                command.intent,
                "/rebotarm/sim/move_relative",
                "action",
                dict(command.params),
                dry_run=False,
            )
        dry_route = self._dry_run_router.route(command)
        return RouteResult(
            dry_route.intent,
            f"/rebotarm/sim{dry_route.target.removeprefix('/rebotarm')}",
            dry_route.mode,
            dry_route.params,
            dry_run=False,
        )

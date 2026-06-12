from __future__ import annotations

from .models import IntentCommand, RouteResult, SafetyViolationError, VoiceControlConfig


class DryRunCommandRouter:
    def __init__(self, config: VoiceControlConfig):
        self._config = config

    def route(self, command: IntentCommand) -> RouteResult:
        if command.command == "enable_arm":
            return RouteResult(command.intent, "/rebotarm/enable", "service", {})
        if command.command == "disable_arm":
            return RouteResult(command.intent, "/rebotarm/disable", "service", {})
        if command.command == "safe_home":
            return RouteResult(command.intent, "/rebotarm/safe_home", "service", {})
        if command.command == "set_gripper":
            return RouteResult(
                command.intent,
                "/rebotarm/gripper/set",
                "service",
                dict(command.params),
            )
        if command.command == "move_named_pose":
            name = command.params.get("name")
            pose = self._config.named_poses.get(str(name))
            if pose is None:
                raise SafetyViolationError(f"cannot route unknown named pose: {name}")
            return RouteResult(
                command.intent,
                "/rebotarm/move_to_pose",
                "action",
                {"name": name, "pose": pose},
            )
        if command.command == "move_relative":
            return RouteResult(
                command.intent,
                "/rebotarm/move_relative",
                "action",
                dict(command.params),
            )
        if command.command == "pick_object":
            return RouteResult(command.intent, "/rebotarm/pick_object", "action", dict(command.params))
        if command.command == "place_object":
            return RouteResult(command.intent, "/rebotarm/place_object", "action", dict(command.params))
        if command.command == "inspect_workspace":
            return RouteResult(command.intent, "/rebotarm/inspect_workspace", "service", dict(command.params))
        if command.command == "confirm_action":
            return RouteResult(command.intent, "/rebotarm/confirm_action", "service", dict(command.params))
        if command.command == "cancel_task":
            return RouteResult(command.intent, "/rebotarm/cancel_task", "service", dict(command.params))
        if command.command == "stop_motion":
            return RouteResult(command.intent, "/rebotarm/stop", "dry_run_stop", {})
        raise SafetyViolationError(f"cannot route command: {command.command}")

from __future__ import annotations

from typing import Any

from .models import IntentCommand, SafetyViolationError, VoiceControlConfig


class SafetyGuard:
    def __init__(self, config: VoiceControlConfig):
        self._config = config
        self._allowed_commands = {
            "enable_arm",
            "disable_arm",
            "safe_home",
            "set_gripper",
            "move_named_pose",
            "move_relative",
            "pick_object",
            "place_object",
            "inspect_workspace",
            "confirm_action",
            "cancel_task",
            "run_task_template",
            "stop_motion",
        }

    def validate(self, command: IntentCommand) -> IntentCommand:
        if command.command not in self._allowed_commands:
            raise SafetyViolationError(f"command is not whitelisted: {command.command}")

        if command.command == "move_named_pose":
            self._validate_named_pose(command)

        if command.command == "run_task_template":
            name = command.params.get("name")
            if name not in self._config.task_templates:
                raise SafetyViolationError(f"task template does not exist: {name}")

        if command.command == "move_relative":
            distance_m = abs(float(command.params.get("distance_m", 0.0)))
            if distance_m > 0.05:
                raise SafetyViolationError("move_relative distance exceeds 0.05 m")
            if command.params.get("axis") not in {"x", "y", "z"}:
                raise SafetyViolationError("move_relative axis must be x, y, or z")

        return command

    def _validate_named_pose(self, command: IntentCommand) -> None:
        name = command.params.get("name")
        allowed = set(self._config.safety_limits.get("allowed_named_poses", []))
        if name not in allowed:
            raise SafetyViolationError(f"named pose is not allowed: {name}")
        pose = self._config.named_poses.get(str(name))
        if pose is None:
            raise SafetyViolationError(f"named pose does not exist: {name}")
        self._validate_workspace(str(name), pose)

    def _validate_workspace(self, name: str, pose: dict[str, Any]) -> None:
        position = pose.get("position")
        if not isinstance(position, list) or len(position) != 3:
            raise SafetyViolationError(f"named pose has invalid position: {name}")

        workspace = self._config.safety_limits.get("workspace", {})
        for axis, value in zip(("x", "y", "z"), position):
            bounds = workspace.get(axis)
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise SafetyViolationError(f"workspace bounds missing for axis {axis}")
            low, high = float(bounds[0]), float(bounds[1])
            if float(value) < low or float(value) > high:
                raise SafetyViolationError(
                    f"named pose {name} is outside workspace on {axis}: {value}"
                )

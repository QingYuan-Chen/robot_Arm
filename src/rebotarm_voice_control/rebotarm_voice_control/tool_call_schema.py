from __future__ import annotations

import json
from typing import Any

from .models import IntentCommand, SafetyViolationError, ToolCall


_WHITELIST = {
    "move_home",
    "open_gripper",
    "close_gripper",
    "stop_robot",
    "move_relative",
    "pick_object",
    "place_object",
    "inspect_workspace",
    "confirm_action",
    "cancel_task",
}


def _distance_to_meters(value: Any, unit: str) -> float:
    amount = float(value)
    if unit == "m":
        return amount
    if unit == "cm":
        return amount / 100.0
    if unit == "mm":
        return amount / 1000.0
    raise SafetyViolationError(f"unsupported distance unit: {unit}")


class ToolCallParser:
    def parse_json(self, payload: str | dict[str, Any]) -> ToolCall:
        raw = json.loads(payload) if isinstance(payload, str) else dict(payload)
        tool = str(raw.get("tool") or raw.get("name") or "")
        if tool not in _WHITELIST:
            raise SafetyViolationError(f"tool is not whitelisted: {tool}")
        arguments = raw.get("arguments", {})
        if isinstance(arguments, str):
            arguments = json.loads(arguments or "{}")
        if not isinstance(arguments, dict):
            raise SafetyViolationError("tool arguments must be a JSON object")
        self._validate(tool, arguments)
        return ToolCall(tool=tool, arguments=arguments, call_id=str(raw.get("call_id", "")))

    def to_intent(self, call: ToolCall) -> IntentCommand:
        tool = call.tool
        args = dict(call.arguments)
        if tool == "move_home":
            return IntentCommand("move_home", "safe_home", need_confirm=True)
        if tool == "open_gripper":
            width = float(args.get("width", 0.09))
            return IntentCommand(
                "open_gripper",
                "set_gripper",
                {"position": width, "max_effort": float(args.get("max_effort", 0.5))},
            )
        if tool == "close_gripper":
            return IntentCommand(
                "close_gripper",
                "set_gripper",
                {"position": 0.0, "max_effort": float(args.get("max_effort", 0.5))},
            )
        if tool == "stop_robot":
            return IntentCommand("stop_motion", "stop_motion", {"level": args.get("level", "soft_stop")}, priority="highest")
        if tool == "move_relative":
            distance_m = _distance_to_meters(args["distance"], str(args["unit"]))
            return IntentCommand(
                "move_relative",
                "move_relative",
                {"axis": args["axis"], "distance_m": distance_m},
                need_confirm=True,
            )
        if tool == "pick_object":
            return IntentCommand("pick_object", "pick_object", args, need_confirm=True)
        if tool == "place_object":
            return IntentCommand("place_object", "place_object", args, need_confirm=True)
        if tool == "inspect_workspace":
            return IntentCommand("inspect_workspace", "inspect_workspace", args)
        if tool == "confirm_action":
            return IntentCommand("confirm_action", "confirm_action", args)
        if tool == "cancel_task":
            return IntentCommand("cancel_task", "cancel_task", args, priority="highest")
        raise SafetyViolationError(f"tool cannot be converted to intent: {tool}")

    def _validate(self, tool: str, arguments: dict[str, Any]) -> None:
        if tool == "move_relative":
            if arguments.get("axis") not in {"x", "y", "z"}:
                raise SafetyViolationError("move_relative axis must be x, y, or z")
            if "distance" not in arguments:
                raise SafetyViolationError("move_relative distance is required")
            if "unit" not in arguments:
                raise SafetyViolationError("move_relative unit is required")
            distance_m = abs(_distance_to_meters(arguments["distance"], str(arguments["unit"])))
            if distance_m > 0.05:
                raise SafetyViolationError("move_relative distance exceeds 0.05 m")
        if tool == "open_gripper" and "width" in arguments:
            width = float(arguments["width"])
            if width < 0.0 or width > 0.09:
                raise SafetyViolationError("open_gripper width must be between 0.0 and 0.09 m")

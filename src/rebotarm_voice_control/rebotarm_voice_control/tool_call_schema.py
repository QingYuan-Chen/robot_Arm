"""大模型工具调用（tool call）的白名单解析与安全校验。

大模型只被允许输出有限的"任务级工具"，本模块负责：

1. 解析 JSON（字符串或字典），取出 tool / arguments / call_id；
2. 拒绝白名单之外的工具名，避免模型直接下发关节角、电流或底层总线指令；
3. 做第一道参数硬校验（单次位移步长、夹爪开度等）；
4. 把工具调用转换成统一的 IntentCommand，再交给安全门与执行模式路由。

对外接口：``ToolCallParser.parse_json`` 与 ``ToolCallParser.to_intent``。
这里的安全上限与 safety_guard 中的校验有意重复，属于双保险。
"""

from __future__ import annotations

import json
from typing import Any

from .models import IntentCommand, SafetyViolationError, ToolCall


# 允许大模型调用的工具全集；不在此集合中的工具名一律拒绝。
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
    """把 m / cm / mm 单位的距离统一换算为米。

    未知单位直接判为安全违规而不做猜测，避免单位歧义导致超程运动。
    """
    amount = float(value)
    if unit == "m":
        return amount
    if unit == "cm":
        return amount / 100.0
    if unit == "mm":
        return amount / 1000.0
    raise SafetyViolationError(f"unsupported distance unit: {unit}")


class ToolCallParser:
    """工具调用解析器：JSON 解析、白名单过滤、参数校验与意图转换。

    无状态、可安全复用；每次解析独立校验，不缓存上一次的调用结果。
    """

    def parse_json(self, payload: str | dict[str, Any]) -> ToolCall:
        """解析大模型返回的工具调用，并完成白名单与参数校验。

        payload 既可以是 JSON 字符串，也可以是已解析的字典；arguments 若为
        字符串会被再解析一次（部分模型会把参数二次编码成 JSON）。
        校验失败抛 SafetyViolationError。
        """
        raw = json.loads(payload) if isinstance(payload, str) else dict(payload)
        # 兼容两种工具名字段：OpenAI 风格用 tool，个别模型用 name。
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
        """把已校验的工具调用转换成统一的意图命令。

        这里补齐各工具的默认参数（夹爪开度、最大力矩等），并按动作性质标注
        是否需要二次确认：会改变机械臂位姿或夹持状态的动作通常 need_confirm。
        """
        tool = call.tool
        args = dict(call.arguments)
        if tool == "move_home":
            return IntentCommand("move_home", "safe_home", need_confirm=True)
        if tool == "open_gripper":
            # 默认张开到 0.09 m，即夹爪的物理最大开口。
            width = float(args.get("width", 0.09))
            return IntentCommand(
                "open_gripper",
                "set_gripper",
                {"position": width, "max_effort": float(args.get("max_effort", 0.5))},
            )
        if tool == "close_gripper":
            # position 0.0 表示完全闭合；max_effort 为夹持力上限。
            return IntentCommand(
                "close_gripper",
                "set_gripper",
                {"position": 0.0, "max_effort": float(args.get("max_effort", 0.5))},
            )
        if tool == "stop_robot":
            # 停止类命令优先级最高，且默认只做软停（soft_stop）。
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
        """按工具类型校验参数硬约束，越界即抛 SafetyViolationError。

        这些上限是运动前的数值防线：单次相对位移不超过 0.05 m（与
        safety_guard 及仿真动作节点保持一致），夹爪开度限制在物理行程
        [0.0, 0.09] m 之内。
        """
        if tool == "move_relative":
            if arguments.get("axis") not in {"x", "y", "z"}:
                raise SafetyViolationError("move_relative axis must be x, y, or z")
            if "distance" not in arguments:
                raise SafetyViolationError("move_relative distance is required")
            if "unit" not in arguments:
                raise SafetyViolationError("move_relative unit is required")
            # 取绝对值判断，正负方向使用同一上限。
            distance_m = abs(_distance_to_meters(arguments["distance"], str(arguments["unit"])))
            if distance_m > 0.05:
                raise SafetyViolationError("move_relative distance exceeds 0.05 m")
        if tool == "open_gripper" and "width" in arguments:
            width = float(arguments["width"])
            if width < 0.0 or width > 0.09:
                raise SafetyViolationError("open_gripper width must be between 0.0 and 0.09 m")

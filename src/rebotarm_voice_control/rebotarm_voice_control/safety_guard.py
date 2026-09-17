"""命令级安全门：白名单、命名位姿与工作空间范围校验。

语音链路有两个入口（本地关键词解析与大模型工具调用），无论走哪个入口，
最终都必须经过本模块的 ``SafetyGuard.validate`` 才能进入路由与执行。这里
集中回答"能不能做"的问题：

- 命令名必须在内置白名单内，避免上层拼接出任意接口调用；
- 命名位姿必须同时出现在 safety_limits.yaml 的 allowed_named_poses 与
  named_poses.yaml 中；
- 命名位姿的位置必须落在 safety_limits.yaml 的 workspace 盒内（单位米，
  坐标系由位姿自带的 frame_id 决定，当前配置为 base_link）；
- 相对运动单次位移不超过 0.05 m；
- 任务模板必须真实存在于 task_templates.yaml。

校验通过后原样返回 IntentCommand（不做拷贝），失败抛
SafetyViolationError。
"""

from __future__ import annotations

from typing import Any

from .models import IntentCommand, SafetyViolationError, VoiceControlConfig


class SafetyGuard:
    """基于白名单的命令校验器，构造时绑定一份 VoiceControlConfig。"""

    def __init__(self, config: VoiceControlConfig):
        self._config = config
        # 允许路由的命令全集，与 command_router 实际支持的分支保持一致。
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
        """校验单条命令；通过则原样返回，违规抛 SafetyViolationError。

        校验顺序为：先过命令白名单，再按命令类型做专项校验；任意一步失败
        立即中断，不再执行后续检查。
        """
        if command.command not in self._allowed_commands:
            raise SafetyViolationError(f"command is not whitelisted: {command.command}")

        if command.command == "move_named_pose":
            self._validate_named_pose(command)

        if command.command == "run_task_template":
            # 模板名必须已在 task_templates.yaml 中定义，防止注入任意模板。
            name = command.params.get("name")
            if name not in self._config.task_templates:
                raise SafetyViolationError(f"task template does not exist: {name}")

        if command.command == "move_relative":
            # 单次相对位移上限 0.05 m，与 tool_call_schema、仿真动作节点一致；
            # 取绝对值以便正负方向共用同一限制。
            distance_m = abs(float(command.params.get("distance_m", 0.0)))
            if distance_m > 0.05:
                raise SafetyViolationError("move_relative distance exceeds 0.05 m")
            if command.params.get("axis") not in {"x", "y", "z"}:
                raise SafetyViolationError("move_relative axis must be x, y, or z")

        return command

    def _validate_named_pose(self, command: IntentCommand) -> None:
        """校验命名位姿：在允许清单内、有定义、且位于工作空间内。"""
        name = command.params.get("name")
        allowed = set(self._config.safety_limits.get("allowed_named_poses", []))
        if name not in allowed:
            raise SafetyViolationError(f"named pose is not allowed: {name}")
        pose = self._config.named_poses.get(str(name))
        if pose is None:
            raise SafetyViolationError(f"named pose does not exist: {name}")
        self._validate_workspace(str(name), pose)

    def _validate_workspace(self, name: str, pose: dict[str, Any]) -> None:
        """校验位姿位置是否落在 safety_limits.yaml 的 workspace 轴对齐盒内。

        position 必须是长度为 3 的 [x, y, z] 列表（单位米），每个轴都要在
        配置里给出 [下限, 上限]。边界按闭区间处理，恰好等于边界视为通过；
        缺少任一轴的范围同样判为违规（宁拒绝、不放行）。
        """
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

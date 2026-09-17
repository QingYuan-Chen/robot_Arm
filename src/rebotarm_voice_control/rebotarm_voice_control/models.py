"""语音控制包的共享数据模型与安全异常定义。

本模块是语音控制流水线的数据契约层，只定义不可变数据结构与异常类型，不
包含任何解析、路由或执行逻辑。一次典型的处理链路为：

    中文文本 → ToolCall（大模型输出的工具调用，须过白名单）
             → IntentCommand（归一化后的意图命令）
             → RouteResult（目标接口 + 参数）
             → ExecutionRouteResult（叠加执行模式：dry_run / sim / real）

所有 dataclass 都声明为 frozen，保证一旦通过安全校验，命令参数不会再被
后续环节就地改写。安全违规统一抛 ``SafetyViolationError``，由上层终止处理。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


class VoiceControlError(Exception):
    """语音控制功能的基类异常，本包所有自定义异常都继承它。"""


class UnknownCommandError(VoiceControlError):
    """文本无法匹配 intents.yaml 中任何 pattern 时抛出。"""


class SafetyViolationError(VoiceControlError):
    """命令未通过安全校验（白名单、工作空间、参数上限等）时抛出。

    这是流水线的硬中断信号：调用方必须放弃执行，不得降级为"尽力尝试"。
    """


@dataclass(frozen=True)
class IntentCommand:
    """归一化后的意图命令，是安全校验与路由的共同输入。

    可由本地关键词解析（intents.yaml）或大模型工具调用转换而来，构造后
    字段全部只读。
    """

    intent: str  # 意图名（intents.yaml 顶层键），如 "safe_home"
    command: str  # 归一化命令名，用于路由与安全白名单，如 "move_named_pose"
    params: dict[str, Any] = field(default_factory=dict)  # 命令参数，如 {"position": 0.09}
    need_confirm: bool = False  # 为 True 时执行前需操作员二次确认
    source_text: str = ""  # 触发本次命令的原始文本，供审计与回显
    priority: str = "normal"  # 优先级；"highest" 保留给急停/取消类命令


@dataclass(frozen=True)
class VoiceControlConfig:
    """config/ 目录下四份 YAML 配置的聚合。

    字段分别对应 intents.yaml、named_poses.yaml、task_templates.yaml 与
    safety_limits.yaml 的原始映射内容，由 config_loader 统一加载后传入。
    """

    intents: dict[str, dict[str, Any]]  # 意图名 → {patterns, command, params, need_confirm, priority}
    named_poses: dict[str, dict[str, Any]]  # 命名位姿名 → {frame_id, position, orientation}
    task_templates: dict[str, dict[str, Any]]  # 模板名 → {description, need_confirm, steps}
    safety_limits: dict[str, Any]  # 执行模式、真实调用开关、允许位姿与工作空间盒


@dataclass(frozen=True)
class RouteResult:
    """一次命令的路由结果：发往哪里、走哪类接口、带什么参数。

    ``dry_run=True`` 表示只是演练路由、尚未经执行器确认；仿真执行器会
    拒绝执行 dry_run 的路由。
    """

    intent: str  # 意图名，透传自 IntentCommand
    target: str  # 目标 ROS 服务或动作名，如 "/rebotarm/safe_home"
    mode: str  # 目标接口类型："service" / "action" / "dry_run_stop"
    params: dict[str, Any]  # 下发给目标的参数
    dry_run: bool = True  # 是否为纯演练路由


@dataclass(frozen=True)
class ToolCall:
    """大模型返回、并经白名单与参数校验的工具调用。"""

    tool: str  # 工具名，必须属于 tool_call_schema 的白名单
    arguments: dict[str, Any] = field(default_factory=dict)  # 工具参数（保留原始单位）
    call_id: str = ""  # 调用标识，用于日志与结果回填


@dataclass(frozen=True)
class ExecutionRouteResult:
    """在 RouteResult 之上标注执行模式与是否指向仿真后端。"""

    execution_mode: str  # 执行模式："dry_run" / "sim" / "real"
    route: RouteResult  # 具体路由结果
    simulated: bool = False  # 为 True 时目标为仿真动作或服务

"""实时事件 → 工具调用 → 安全校验 → 执行模式路由的桥接层。

职责：把实时语音会话产生的原始事件筛选、还原为工具调用，并复用与文本链路
相同的白名单与安全守卫，保证"语音说出来的动作"和"文本输入的动作"走同一套
安全策略，不会因为入口不同而放宽限制。

事件过滤：只有 ``response.function_call_arguments.done`` 类型才代表一次参数
已完整生成、可以执行的函数调用；其余事件（增量参数、音频、状态变更等）一律
忽略并返回 None。
"""

from __future__ import annotations

from dataclasses import asdict
import json
from typing import Any

from .execution_modes import ExecutionModeRouter
from .models import VoiceControlConfig
from .safety_guard import SafetyGuard
from .tool_call_schema import ToolCallParser


class RealtimeToolBridge:
    """无状态桥接器：持有解析器、安全守卫与执行模式路由三者。

    构造参数：
        config：聚合后的语音控制配置。
        execution_mode：执行模式 "dry_run" / "sim" / "real"；为 None 时回落到
        safety_limits.yaml 中的 execution_mode（缺省 dry_run）。
    """

    def __init__(self, config: VoiceControlConfig, execution_mode: str | None = None):
        self._config = config
        self._parser = ToolCallParser()
        self._safety_guard = SafetyGuard(config)
        self._mode_router = ExecutionModeRouter(config, execution_mode=execution_mode)

    def handle_event(self, event: dict[str, Any]) -> dict[str, Any] | None:
        """处理一条实时事件；非函数调用完成事件返回 None。

        返回：含义与命令行工具调用入口一致的字典（call_id / tool / intent /
        execution_mode / route），可直接序列化回填给会话。

        异常：工具名不在白名单、参数不合法或安全校验失败时向上抛出，
        由调用方决定是否中断整条会话。
        """
        if event.get("type") != "response.function_call_arguments.done":
            return None

        # 实时事件的参数是 JSON 字符串，这里统一还原成解析器期望的字典结构。
        call = self._parser.parse_json(
            {
                "tool": event.get("name"),
                "arguments": event.get("arguments", "{}"),
                "call_id": event.get("call_id", ""),
            }
        )
        command = self._safety_guard.validate(self._parser.to_intent(call))
        execution = self._mode_router.route(command)
        return {
            "call_id": call.call_id,
            "tool": call.tool,
            "intent": command.intent,
            "execution_mode": execution.execution_mode,
            "route": asdict(execution.route),
        }

    def handle_event_json(self, event_json: str) -> dict[str, Any] | None:
        """``handle_event`` 的字符串入口：接收一条事件 JSON 文本。"""
        return self.handle_event(json.loads(event_json))

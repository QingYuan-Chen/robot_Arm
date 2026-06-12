from __future__ import annotations

from dataclasses import asdict
import json
from typing import Any

from .execution_modes import ExecutionModeRouter
from .models import VoiceControlConfig
from .safety_guard import SafetyGuard
from .tool_call_schema import ToolCallParser


class RealtimeToolBridge:
    def __init__(self, config: VoiceControlConfig, execution_mode: str | None = None):
        self._config = config
        self._parser = ToolCallParser()
        self._safety_guard = SafetyGuard(config)
        self._mode_router = ExecutionModeRouter(config, execution_mode=execution_mode)

    def handle_event(self, event: dict[str, Any]) -> dict[str, Any] | None:
        if event.get("type") != "response.function_call_arguments.done":
            return None

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
        return self.handle_event(json.loads(event_json))

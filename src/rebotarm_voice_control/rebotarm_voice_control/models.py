from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


class VoiceControlError(Exception):
    """Base error for voice-control MVP failures."""


class UnknownCommandError(VoiceControlError):
    """Raised when text cannot be mapped to a configured intent."""


class SafetyViolationError(VoiceControlError):
    """Raised when a command fails safety validation."""


@dataclass(frozen=True)
class IntentCommand:
    intent: str
    command: str
    params: dict[str, Any] = field(default_factory=dict)
    need_confirm: bool = False
    source_text: str = ""
    priority: str = "normal"


@dataclass(frozen=True)
class VoiceControlConfig:
    intents: dict[str, dict[str, Any]]
    named_poses: dict[str, dict[str, Any]]
    task_templates: dict[str, dict[str, Any]]
    safety_limits: dict[str, Any]


@dataclass(frozen=True)
class RouteResult:
    intent: str
    target: str
    mode: str
    params: dict[str, Any]
    dry_run: bool = True


@dataclass(frozen=True)
class ToolCall:
    tool: str
    arguments: dict[str, Any] = field(default_factory=dict)
    call_id: str = ""


@dataclass(frozen=True)
class ExecutionRouteResult:
    execution_mode: str
    route: RouteResult
    simulated: bool = False

from __future__ import annotations

from copy import deepcopy

from .models import IntentCommand, SafetyViolationError, VoiceControlConfig
from .safety_guard import SafetyGuard


class TaskPlanner:
    def __init__(self, config: VoiceControlConfig, safety_guard: SafetyGuard):
        self._config = config
        self._safety_guard = safety_guard

    def expand(self, command: IntentCommand) -> list[IntentCommand]:
        command = self._safety_guard.validate(command)
        if command.command != "run_task_template":
            return [command]

        template_name = command.params.get("name")
        template = self._config.task_templates.get(str(template_name))
        if template is None:
            raise SafetyViolationError(f"task template does not exist: {template_name}")

        steps = []
        for raw_step in template.get("steps", []):
            step = IntentCommand(
                intent=str(raw_step["intent"]),
                command=str(raw_step["command"]),
                params=deepcopy(raw_step.get("params", {})),
                need_confirm=bool(raw_step.get("need_confirm", False)),
                source_text=command.source_text,
                priority=str(raw_step.get("priority", "normal")),
            )
            steps.append(self._safety_guard.validate(step))
        return steps

from __future__ import annotations

from copy import deepcopy
from typing import Any

from .models import IntentCommand, UnknownCommandError


def _normalize_text(text: str) -> str:
    return "".join(str(text).strip().split())


class IntentParser:
    def __init__(self, intents: dict[str, dict[str, Any]]):
        self._intents = intents

    def parse(self, text: str) -> IntentCommand:
        normalized = _normalize_text(text)
        if not normalized:
            raise UnknownCommandError("empty command text")

        for intent_name, spec in self._intents.items():
            for pattern in spec.get("patterns", []):
                if _normalize_text(pattern) in normalized:
                    return IntentCommand(
                        intent=intent_name,
                        command=str(spec.get("command", intent_name)),
                        params=deepcopy(spec.get("params", {})),
                        need_confirm=bool(spec.get("need_confirm", False)),
                        source_text=text,
                        priority=str(spec.get("priority", "normal")),
                    )

        raise UnknownCommandError(f"unknown command text: {text}")

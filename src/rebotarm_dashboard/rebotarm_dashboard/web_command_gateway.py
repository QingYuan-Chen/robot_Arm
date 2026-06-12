from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WebCommandRequest:
    intent: str
    execution_mode: str
    payload: dict[str, Any]
    request_id: str = ""

    @classmethod
    def from_payload(cls, payload: dict[str, Any], *, execution_mode: str) -> "WebCommandRequest":
        data = dict(payload)
        return cls(
            intent=str(data.get("intent", "") or ""),
            execution_mode=str(execution_mode or "dry_run").strip().lower(),
            payload=data,
            request_id=str(data.get("request_id", "") or ""),
        )


class WebCommandGateway:
    def route(self, request: WebCommandRequest) -> dict[str, Any]:
        if request.execution_mode != "execute":
            return {
                "accepted": False,
                "state": "dry_run",
                "intent": request.intent,
                "request_id": request.request_id,
                "message": "web command dry-run; launch with execution_mode:=execute to run hardware commands",
            }
        return {
            "accepted": True,
            "state": "accepted",
            "intent": request.intent,
            "request_id": request.request_id,
        }

    def blocked_legacy_response(
        self,
        *,
        intent: str,
        message: str,
        execution_mode: str,
        request_id: str = "",
        blocked_legacy_execution: bool = False,
    ) -> dict[str, Any]:
        return {
            "accepted": False,
            "state": "blocked",
            "intent": intent,
            "execution_mode": str(execution_mode or "dry_run"),
            "request_id": request_id,
            "blocked_legacy_execution": bool(blocked_legacy_execution),
            "message": message,
        }

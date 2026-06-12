from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any
from urllib.request import Request, urlopen


@dataclass
class NetworkDetectionConfig:
    detections_url: str
    timeout_ms: int


class NetworkDetectionClient:
    def __init__(self, config: NetworkDetectionConfig) -> None:
        self._config = config
        self._last_payload: dict[str, Any] = {"detections": []}
        self._last_debug_message = "client_not_used"

    def fetch(self) -> dict[str, Any]:
        timeout = max(self._config.timeout_ms, 1) / 1000.0
        try:
            request = Request(self._config.detections_url, headers={"User-Agent": "rebotarm_vision/0.1"})
            with urlopen(request, timeout=timeout) as response:
                payload = json.loads(response.read().decode("utf-8"))
            if not isinstance(payload, dict):
                self._last_debug_message = "invalid_json_root"
                return self._last_payload
            self._last_payload = payload
            self._last_debug_message = f"ok detections={len(payload.get('detections', []))}"
            return payload
        except Exception as exc:
            self._last_debug_message = f"exception:{type(exc).__name__}:{exc}"
            return self._last_payload

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message

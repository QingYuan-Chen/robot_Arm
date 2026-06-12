from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
import json
from typing import Any, TextIO


class RealtimeSessionClient(ABC):
    """Provider-neutral source of Realtime API events."""

    @abstractmethod
    def connect(self) -> None:
        """Open the event source."""

    @abstractmethod
    def recv_event(self) -> dict[str, Any] | None:
        """Return the next event, or None when the stream is exhausted."""

    @abstractmethod
    def close(self) -> None:
        """Close the event source."""


class JsonlRealtimeSessionClient(RealtimeSessionClient):
    """Replay a realtime event stream from one JSON object per line."""

    def __init__(self, jsonl_path: str | Path):
        self._path = Path(jsonl_path)
        self._stream: TextIO | None = None

    def connect(self) -> None:
        self._stream = self._path.open("r", encoding="utf-8-sig")

    def recv_event(self) -> dict[str, Any] | None:
        if self._stream is None:
            raise RuntimeError("realtime session is not connected")
        for line in self._stream:
            stripped = line.strip()
            if stripped:
                loaded = json.loads(stripped)
                if not isinstance(loaded, dict):
                    raise ValueError("realtime JSONL events must be JSON objects")
                return loaded
        return None

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None

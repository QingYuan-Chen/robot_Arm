from __future__ import annotations

from pathlib import Path
import os
from typing import Any


class AsrConfigurationError(RuntimeError):
    """Raised when online ASR is not configured."""


class OnlineAsrClient:
    def __init__(
        self,
        api_key: str | None = None,
        model: str = "gpt-4o-transcribe",
        openai_client: Any | None = None,
    ):
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        self.model = model
        self._client = openai_client

    def transcribe_file(self, audio_path: str | Path) -> str:
        path = Path(audio_path)
        if not self.api_key and self._client is None:
            raise AsrConfigurationError(
                "OPENAI_API_KEY is required for online ASR; use text input or provide a configured client"
            )
        if not path.exists():
            raise FileNotFoundError(path)

        client = self._client or self._build_openai_client()
        with path.open("rb") as audio_file:
            response = client.audio.transcriptions.create(
                model=self.model,
                file=audio_file,
                response_format="text",
            )
        if isinstance(response, str):
            return response.strip()
        text = getattr(response, "text", "")
        return str(text).strip()

    def _build_openai_client(self):
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise AsrConfigurationError(
                "The openai Python package is required for online ASR"
            ) from exc
        return OpenAI(api_key=self.api_key)

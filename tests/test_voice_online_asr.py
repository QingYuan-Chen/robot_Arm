from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.asr_client import AsrConfigurationError, OnlineAsrClient


class _FakeTranscriptions:
    def __init__(self):
        self.calls = []

    def create(self, **kwargs):
        self.calls.append(kwargs)
        return "打开夹爪"


class _FakeAudio:
    def __init__(self):
        self.transcriptions = _FakeTranscriptions()


class _FakeOpenAIClient:
    def __init__(self):
        self.audio = _FakeAudio()


def test_online_asr_requires_api_key_without_fake_client(tmp_path, monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    audio_path = tmp_path / "cmd.wav"
    audio_path.write_bytes(b"fake wav bytes")

    client = OnlineAsrClient()

    with pytest.raises(AsrConfigurationError, match="OPENAI_API_KEY"):
        client.transcribe_file(audio_path)


def test_online_asr_uses_configured_model_and_returns_text(tmp_path):
    fake_client = _FakeOpenAIClient()
    audio_path = tmp_path / "cmd.wav"
    audio_path.write_bytes(b"fake wav bytes")

    client = OnlineAsrClient(
        api_key="test-key",
        model="gpt-4o-transcribe",
        openai_client=fake_client,
    )

    text = client.transcribe_file(audio_path)

    assert text == "打开夹爪"
    call = fake_client.audio.transcriptions.calls[0]
    assert call["model"] == "gpt-4o-transcribe"
    assert call["response_format"] == "text"
    assert call["file"].name == str(audio_path)

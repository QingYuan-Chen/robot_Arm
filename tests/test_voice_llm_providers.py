from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.llm_providers import (
    DoubaoLLMProvider,
    LLMConfigurationError,
    MockLLMProvider,
    OpenAICompatibleLLMProvider,
    create_llm_provider,
)


class _FakeMessage:
    def __init__(self, content: str):
        self.content = content


class _FakeChoice:
    def __init__(self, content: str):
        self.message = _FakeMessage(content)


class _FakeCompletions:
    def __init__(self):
        self.calls = []

    def create(self, **kwargs):
        self.calls.append(kwargs)
        return type(
            "Completion",
            (),
            {
                "choices": [
                    _FakeChoice('{"tool":"move_home","arguments":{},"call_id":"llm_1"}')
                ]
            },
        )()


class _FakeChat:
    def __init__(self):
        self.completions = _FakeCompletions()


class _FakeClient:
    def __init__(self):
        self.chat = _FakeChat()


def test_mock_provider_returns_whitelisted_tool_call():
    provider = MockLLMProvider()

    payload = provider.parse_tool_call("向上移动 5 厘米")

    assert payload["tool"] == "move_relative"
    assert payload["arguments"] == {"axis": "z", "distance": 5, "unit": "cm"}


def test_openai_compatible_provider_uses_chat_completion_json():
    fake_client = _FakeClient()
    provider = OpenAICompatibleLLMProvider(
        provider_name="test",
        api_key="test-key",
        base_url="https://example.test/v1",
        model="test-model",
        openai_client=fake_client,
        temperature=0.2,
        response_format="json_object",
    )

    payload = provider.parse_tool_call("回到初始位置")

    assert payload["tool"] == "move_home"
    call = fake_client.chat.completions.calls[0]
    assert call["model"] == "test-model"
    assert call["response_format"] == {"type": "json_object"}
    assert call["temperature"] == 0.2
    assert "回到初始位置" in call["messages"][-1]["content"]


def test_doubao_provider_defaults_to_ark_api_key_and_base_url(monkeypatch):
    monkeypatch.setenv("ARK_API_KEY", "ark-test-key")

    provider = DoubaoLLMProvider(model="doubao-test", openai_client=_FakeClient())

    assert provider.api_key == "ark-test-key"
    assert provider.base_url == "https://ark.cn-beijing.volces.com/api/v3"


def test_provider_without_api_key_is_rejected(monkeypatch):
    monkeypatch.delenv("ARK_API_KEY", raising=False)
    provider = DoubaoLLMProvider(model="doubao-test")

    with pytest.raises(LLMConfigurationError, match="ARK_API_KEY"):
        provider.parse_tool_call("回到初始位置")


def test_create_provider_from_config_supports_mock_and_doubao(monkeypatch):
    monkeypatch.setenv("ARK_API_KEY", "ark-test-key")

    assert isinstance(create_llm_provider({"provider": "mock"}), MockLLMProvider)
    assert isinstance(
        create_llm_provider({"provider": "doubao", "model": "doubao-test"}),
        DoubaoLLMProvider,
    )


def test_create_doubao_provider_respects_configured_api_key_env(monkeypatch):
    monkeypatch.delenv("ARK_API_KEY", raising=False)
    monkeypatch.setenv("CUSTOM_ARK_KEY", "custom-key")

    provider = create_llm_provider(
        {
            "provider": "doubao",
            "model": "doubao-test",
            "api_key_env": "CUSTOM_ARK_KEY",
            "temperature": 0,
            "response_format": "json_object",
        }
    )

    assert provider.api_key == "custom-key"
    assert provider.api_key_env == "CUSTOM_ARK_KEY"
    assert provider.temperature == 0
    assert provider.response_format == "json_object"

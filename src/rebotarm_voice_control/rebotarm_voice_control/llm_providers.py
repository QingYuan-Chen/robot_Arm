from __future__ import annotations

from abc import ABC, abstractmethod
import json
import os
import re
from typing import Any


class LLMConfigurationError(RuntimeError):
    """Raised when a configured LLM provider cannot run."""


class BaseLLMProvider(ABC):
    provider_name = "base"

    @abstractmethod
    def parse_tool_call(self, text: str) -> dict[str, Any]:
        """Convert natural language text into a whitelisted tool-call JSON object."""


class MockLLMProvider(BaseLLMProvider):
    provider_name = "mock"

    def parse_tool_call(self, text: str) -> dict[str, Any]:
        normalized = "".join(str(text).split())
        if "向上" in normalized and ("厘米" in normalized or "cm" in normalized):
            match = re.search(r"(\d+(?:\.\d+)?)", normalized)
            distance = float(match.group(1)) if match else 5.0
            if distance.is_integer():
                distance = int(distance)
            return {
                "tool": "move_relative",
                "arguments": {"axis": "z", "distance": distance, "unit": "cm"},
                "call_id": "mock_move_relative",
            }
        if any(key in normalized for key in ("回到初始位置", "回零", "复位", "回家")):
            return {"tool": "move_home", "arguments": {}, "call_id": "mock_move_home"}
        if "打开夹爪" in normalized or "张开夹爪" in normalized:
            return {"tool": "open_gripper", "arguments": {"width": 0.09}, "call_id": "mock_open"}
        if "关闭夹爪" in normalized or "夹紧" in normalized:
            return {"tool": "close_gripper", "arguments": {"max_effort": 0.5}, "call_id": "mock_close"}
        if "停止" in normalized or "急停" in normalized:
            return {"tool": "stop_robot", "arguments": {"level": "soft_stop"}, "call_id": "mock_stop"}
        return {
            "tool": "inspect_workspace",
            "arguments": {"query": "unknown_intent"},
            "call_id": "mock_unknown",
        }


class OpenAICompatibleLLMProvider(BaseLLMProvider):
    provider_name = "openai_compatible"

    def __init__(
        self,
        provider_name: str,
        api_key: str | None,
        base_url: str,
        model: str,
        openai_client: Any | None = None,
        api_key_env: str = "OPENAI_API_KEY",
        temperature: float = 0,
        response_format: str = "json_object",
    ):
        self.provider_name = provider_name
        self.api_key = api_key or os.environ.get(api_key_env)
        self.api_key_env = api_key_env
        self.base_url = base_url
        self.model = model
        self.temperature = temperature
        self.response_format = response_format
        self._client = openai_client

    def parse_tool_call(self, text: str) -> dict[str, Any]:
        if not self.api_key and self._client is None:
            raise LLMConfigurationError(f"{self.api_key_env} is required for {self.provider_name}")
        client = self._client or self._build_client()
        completion = client.chat.completions.create(
            model=self.model,
            response_format={"type": self.response_format},
            temperature=self.temperature,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "你是机械臂语音控制意图解析器。只能输出 JSON，格式为 "
                        '{"tool": "...", "arguments": {...}, "call_id": "..."}。'
                        "tool 必须来自白名单：move_home, open_gripper, close_gripper, "
                        "stop_robot, move_relative, pick_object, place_object, "
                        "inspect_workspace, confirm_action, cancel_task。"
                        "不能输出关节角、电机电流、底层轨迹或 CAN 指令。"
                    ),
                },
                {"role": "user", "content": text},
            ],
        )
        content = completion.choices[0].message.content
        return json.loads(content)

    def _build_client(self):
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise LLMConfigurationError("openai Python package is required") from exc
        return OpenAI(api_key=self.api_key, base_url=self.base_url)


class DoubaoLLMProvider(OpenAICompatibleLLMProvider):
    provider_name = "doubao"

    def __init__(
        self,
        api_key: str | None = None,
        model: str = "doubao-seed-1.6",
        base_url: str = "https://ark.cn-beijing.volces.com/api/v3",
        openai_client: Any | None = None,
        api_key_env: str = "ARK_API_KEY",
        temperature: float = 0,
        response_format: str = "json_object",
    ):
        super().__init__(
            provider_name="doubao",
            api_key=api_key,
            base_url=base_url,
            model=model,
            openai_client=openai_client,
            api_key_env=api_key_env,
            temperature=temperature,
            response_format=response_format,
        )


class OpenAILLMProvider(OpenAICompatibleLLMProvider):
    provider_name = "openai"

    def __init__(
        self,
        api_key: str | None = None,
        model: str = "gpt-4.1-mini",
        base_url: str = "https://api.openai.com/v1",
        openai_client: Any | None = None,
        api_key_env: str = "OPENAI_API_KEY",
        temperature: float = 0,
        response_format: str = "json_object",
    ):
        super().__init__(
            provider_name="openai",
            api_key=api_key,
            base_url=base_url,
            model=model,
            openai_client=openai_client,
            api_key_env=api_key_env,
            temperature=temperature,
            response_format=response_format,
        )


def create_llm_provider(config: dict[str, Any]) -> BaseLLMProvider:
    provider = str(config.get("provider", "mock")).lower()
    if provider == "mock":
        return MockLLMProvider()
    if provider == "doubao":
        return DoubaoLLMProvider(
            model=str(config.get("model", "doubao-seed-1.6")),
            base_url=str(config.get("base_url", "https://ark.cn-beijing.volces.com/api/v3")),
            api_key_env=str(config.get("api_key_env", "ARK_API_KEY")),
            temperature=float(config.get("temperature", 0)),
            response_format=str(config.get("response_format", "json_object")),
        )
    if provider == "openai":
        return OpenAILLMProvider(
            model=str(config.get("model", "gpt-4.1-mini")),
            base_url=str(config.get("base_url", "https://api.openai.com/v1")),
            api_key_env=str(config.get("api_key_env", "OPENAI_API_KEY")),
            temperature=float(config.get("temperature", 0)),
            response_format=str(config.get("response_format", "json_object")),
        )
    raise LLMConfigurationError(f"unsupported llm provider: {provider}")

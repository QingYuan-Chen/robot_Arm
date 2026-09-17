# 大模型意图解析提供方适配层。
#
# 本模块位于语音控制链路的最前端：把用户的一句话（中文自然语言）转换成
# 白名单内的工具调用 JSON，供上层任务规划与执行组件消费。之所以必须经过
# 白名单，是因为大模型输出不可信，绝不能让模型直接产出关节角、电机电流、
# 底层轨迹或 CAN 指令这类会绕过安全校验的内容。
#
# 对外接口（纯函数式，不依赖运行中的节点）：
# - BaseLLMProvider.parse_tool_call(text) -> dict
# - create_llm_provider(config) -> BaseLLMProvider
#
# 支持的提供方由 provider 取值决定：
# - "mock"：本地规则匹配，离线可用，用于测试与无网络环境（默认）；
# - "doubao"：火山方舟（Ark）兼容接口，密钥默认读环境变量 ARK_API_KEY；
# - "openai"：官方接口，密钥默认读环境变量 OPENAI_API_KEY。
#
# 密钥只从环境变量或显式入参读取，不写入任何配置文件；缺失密钥时立即抛错，
# 避免带着空密钥发起请求后拿到难以定位的失败。

from __future__ import annotations

from abc import ABC, abstractmethod
import json
import os
import re
from typing import Any


class LLMConfigurationError(RuntimeError):
    """提供方配置不完整或不可用（缺密钥、缺依赖包、提供方名不认识）时抛出。"""


class BaseLLMProvider(ABC):
    """所有意图解析提供方的抽象基类，只约定一个方法。"""

    provider_name = "base"

    @abstractmethod
    def parse_tool_call(self, text: str) -> dict[str, Any]:
        """把一段自然语言文本转换成白名单内的工具调用 JSON 对象。

        返回值形如 ``{"tool": ..., "arguments": {...}, "call_id": ...}``；
        ``tool`` 必须落在系统提示词声明的白名单内，否则由下游继续拦截。
        """


class MockLLMProvider(BaseLLMProvider):
    """本地规则版提供方：不联网、不消耗额度，用于测试与演示。

    只覆盖演示用的少量中文说法，语义范围远小于真实大模型；它同时可作为
    "链路是否打通"的基准实现，因此解析结果中的 call_id 带 ``mock_`` 前缀。
    """

    provider_name = "mock"

    def parse_tool_call(self, text: str) -> dict[str, Any]:
        # 先去掉所有空白字符，让「向上 移动 5 厘米」这类带空格的口语也能命中关键字
        normalized = "".join(str(text).split())
        # 「向上 + 厘米/cm」判定为相对 Z 轴抬升；距离缺省 5.0 厘米，属于保守小位移
        if "向上" in normalized and ("厘米" in normalized or "cm" in normalized):
            # 取句中第一个数字作为距离；正则同时接受整数与小数
            match = re.search(r"(\d+(?:\.\d+)?)", normalized)
            distance = float(match.group(1)) if match else 5.0
            # 整数就回收成 int，保证返回的 JSON 形状稳定（测试依赖该形状）
            if distance.is_integer():
                distance = int(distance)
            return {
                "tool": "move_relative",
                "arguments": {"axis": "z", "distance": distance, "unit": "cm"},
                "call_id": "mock_move_relative",
            }
        # 回零类说法统一映射到 move_home
        if any(key in normalized for key in ("回到初始位置", "回零", "复位", "回家")):
            return {"tool": "move_home", "arguments": {}, "call_id": "mock_move_home"}
        # 夹爪张开：0.09 m 为夹爪可用的中等开口宽度
        if "打开夹爪" in normalized or "张开夹爪" in normalized:
            return {"tool": "open_gripper", "arguments": {"width": 0.09}, "call_id": "mock_open"}
        # 夹爪闭合：max_effort 0.5 是限力上限，避免夹坏物体
        if "关闭夹爪" in normalized or "夹紧" in normalized:
            return {"tool": "close_gripper", "arguments": {"max_effort": 0.5}, "call_id": "mock_close"}
        # 停止类说法一律降级为软停（soft_stop），不触发硬急停
        if "停止" in normalized or "急停" in normalized:
            return {"tool": "stop_robot", "arguments": {"level": "soft_stop"}, "call_id": "mock_stop"}
        # 其余无法识别：返回只读的观察工具，绝不猜测成动作指令
        return {
            "tool": "inspect_workspace",
            "arguments": {"query": "unknown_intent"},
            "call_id": "mock_unknown",
        }


class OpenAICompatibleLLMProvider(BaseLLMProvider):
    """对接兼容 OpenAI Chat Completions 协议的 HTTP 服务。

    豆包（方舟）与 OpenAI 官方提供方都继承本类，差别只在默认模型名、
    默认 base_url 与默认密钥环境变量。
    """

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
        # 显式传入的密钥优先；否则回退到环境变量（None 表示环境里也没有）
        self.provider_name = provider_name
        self.api_key = api_key or os.environ.get(api_key_env)
        self.api_key_env = api_key_env
        self.base_url = base_url
        self.model = model
        # temperature=0 表示贪心解码，意图解析要求结果可复现，不要随机性
        self.temperature = temperature
        # 要求服务端直接返回 JSON 对象，省去从自由文本里抠 JSON 的脆弱解析
        self.response_format = response_format
        # 允许注入已构造好的客户端，便于测试时替换成假客户端而不发真实请求
        self._client = openai_client

    def parse_tool_call(self, text: str) -> dict[str, Any]:
        """调用一次对话补全，返回解析后的工具调用 JSON 字典。

        既没有密钥也没有注入客户端时抛 ``LLMConfigurationError``（提前失败，
        而不是发出必然 401 的请求）。
        """
        if not self.api_key and self._client is None:
            raise LLMConfigurationError(f"{self.api_key_env} is required for {self.provider_name}")
        client = self._client or self._build_client()
        completion = client.chat.completions.create(
            model=self.model,
            response_format={"type": self.response_format},
            temperature=self.temperature,
            messages=[
                {
                    # 系统提示词是意图解析的**安全白名单**：模型只被允许在这些
                    # 任务级工具里选择，且被明确禁止输出关节角/电流/轨迹/CAN 指令。
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
        """按需 import 并构造真实 HTTP 客户端。

        延迟到调用时才 import：缺少该第三方包的环境仍可加载本模块并使用 mock
        提供方，不会在 import 阶段就失败。
        """
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise LLMConfigurationError("openai Python package is required") from exc
        return OpenAI(api_key=self.api_key, base_url=self.base_url)


class DoubaoLLMProvider(OpenAICompatibleLLMProvider):
    """火山方舟（豆包）提供方，默认走北京区 Ark 端点与 ARK_API_KEY。"""

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
    """OpenAI 官方提供方，默认模型 gpt-4.1-mini、默认密钥环境变量 OPENAI_API_KEY。"""

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
    """按配置字典构造提供方实例（工厂函数）。

    配置键与含义（通常来自 ``llm_config.yaml``）：
    - ``provider``：提供方标识，``mock`` / ``doubao`` / ``openai``，缺省 ``mock``；
    - ``model``：模型名，随提供方选择，缺省见各分支；
    - ``base_url``：服务端点，指向代理或私有部署时改这里；
    - ``api_key_env``：存放密钥的环境变量名，便于复用已有凭据而不落盘；
    - ``temperature``：采样温度，意图解析建议保持 0；
    - ``response_format``：响应格式，``json_object`` 表示要求纯 JSON 输出。

    提供方名不在上述集合内时抛 ``LLMConfigurationError``；默认回落到 mock，
    保证没配置好时是离线可用而不是误连外部服务。
    """
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

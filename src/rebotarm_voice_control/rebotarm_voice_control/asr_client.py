"""在线语音识别（ASR）客户端。

职责：把本地音频文件送去在线转写接口，取回文本，交给文本指令流水线继续处理
（见 voice_file_node）。本模块只负责"音频 → 中文文本"这一段，不做意图解析。

配置来源与安全约束：
    * API Key 优先取构造参数，其次取环境变量 ``OPENAI_API_KEY``；
    * 两者都没有、且未注入现成客户端时，抛 ``AsrConfigurationError``，
      而不是静默返回空串——避免上层把空文本当成"没听清"继续执行；
    * 依赖的第三方客户端库采用惰性导入，未安装时只在实际调用时报错，
      这样纯文本链路（text_input_node）在没有该库的环境里仍可运行。
"""

from __future__ import annotations

from pathlib import Path
import os
from typing import Any


class AsrConfigurationError(RuntimeError):
    """在线 ASR 未完成配置（缺少 API Key 或缺少客户端库）时抛出。"""


class OnlineAsrClient:
    """在线转写客户端的最小封装，便于测试时注入假客户端。

    构造参数：
        api_key：接口密钥；为 None 时回退到环境变量 ``OPENAI_API_KEY``。
        model：转写模型名，默认 "gpt-4o-transcribe"。
        openai_client：已构造好的客户端实例（测试注入用）；为 None 时按需自建。
    """

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
        """转写单个音频文件，返回去除首尾空白后的文本。

        参数 ``audio_path``：本地音频文件路径（常见为 wav/mp3，格式由服务端决定）。

        异常：
            AsrConfigurationError：既无密钥也无注入客户端，或缺少客户端库；
            FileNotFoundError：音频文件不存在（提前失败，避免无谓的网络调用）。
        """
        path = Path(audio_path)
        # 密钥校验放在读取文件之前：配置缺失是部署问题，不应被文件系统错误掩盖。
        if not self.api_key and self._client is None:
            raise AsrConfigurationError(
                "OPENAI_API_KEY is required for online ASR; use text input or provide a configured client"
            )
        if not path.exists():
            raise FileNotFoundError(path)

        client = self._client or self._build_openai_client()
        # 以二进制方式打开句柄交给 SDK 上传，避免文本模式换行转换破坏音频字节。
        with path.open("rb") as audio_file:
            response = client.audio.transcriptions.create(
                model=self.model,
                file=audio_file,
                response_format="text",
            )
        # 兼容两种返回形态：直接返回字符串，或返回带 .text 属性的对象。
        if isinstance(response, str):
            return response.strip()
        text = getattr(response, "text", "")
        return str(text).strip()

    def _build_openai_client(self):
        """惰性构造真实客户端；缺少第三方库时转换为本模块的配置异常。"""
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise AsrConfigurationError(
                "The openai Python package is required for online ASR"
            ) from exc
        return OpenAI(api_key=self.api_key)

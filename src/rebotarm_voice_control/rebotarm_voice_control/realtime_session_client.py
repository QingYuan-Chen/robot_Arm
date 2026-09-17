"""实时事件源的抽象接口与本地 JSONL 实现。

职责：把"事件从哪里来"与"事件怎么被路由"解耦。网关只依赖这里的抽象接口，
因此换成真实网络会话（WebSocket 等）时无需改动路由与安全校验代码；当前的
JSONL 实现用于离线回放与自动化测试：文件每行一个 JSON 对象（一个事件）。

事件本身保持提供方原始结构（不在此层做字段裁剪），字段语义由上层
``RealtimeToolBridge`` 按需读取。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
import json
from typing import Any, TextIO


class RealtimeSessionClient(ABC):
    """与具体提供方无关的实时事件源接口。"""

    @abstractmethod
    def connect(self) -> None:
        """打开事件源（建立连接或打开文件），为后续接收事件做准备。"""

    @abstractmethod
    def recv_event(self) -> dict[str, Any] | None:
        """返回下一个事件；流已耗尽时返回 None（约定为正常结束，而非异常）。"""

    @abstractmethod
    def close(self) -> None:
        """关闭事件源并释放资源；可重复调用，不应对已关闭的连接报错。"""


class JsonlRealtimeSessionClient(RealtimeSessionClient):
    """从"每行一个 JSON 对象"的 JSONL 文件回放实时事件流。"""

    def __init__(self, jsonl_path: str | Path):
        self._path = Path(jsonl_path)
        self._stream: TextIO | None = None

    def connect(self) -> None:
        # utf-8-sig：容忍文件带 UTF-8 BOM，避免首行 JSON 解析失败。
        self._stream = self._path.open("r", encoding="utf-8-sig")

    def recv_event(self) -> dict[str, Any] | None:
        """顺序读取下一条非空行并解析为事件字典。

        空行（含仅含空白字符的行）被跳过，便于人工编辑的事件文件留出分隔。
        顶层不是 JSON 对象时抛 ValueError——实时事件契约要求对象结构。
        """
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

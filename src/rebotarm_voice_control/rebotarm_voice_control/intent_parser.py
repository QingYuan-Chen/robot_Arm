"""基于关键词的本地意图解析器（无大模型依赖）。

职责：把一句中文文本映射为 ``IntentCommand``。规则来自 intents.yaml：

    intent_name:
      patterns: [...]      # 触发说法（子串匹配）
      command: "..."       # 归一化命令名（供安全白名单与路由使用）
      params: {...}        # 命中后携带的参数
      need_confirm: bool   # 是否需要操作员二次确认
      priority: "..."      # 优先级

匹配语义（重要）：
    * 匹配前会把文本与模式都做"去空白"归一化（含中文全角空格、换行），因此
      "回到 初始 位置" 也能命中 "回到初始位置"；
    * 采用**子串包含**判断（pattern in normalized），不是分词或精确相等；
    * 按 intents.yaml 的书写顺序遍历，命中即返回，所以模式之间存在包含关系时
      （如 "停止" 与 "紧急停止"）先写的优先——调整 YAML 顺序会改变行为。
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any

from .models import IntentCommand, UnknownCommandError


def _normalize_text(text: str) -> str:
    """去除首尾空白并删除所有内部空白字符，用于宽松匹配中文口语说法。"""
    return "".join(str(text).strip().split())


class IntentParser:
    """关键词意图解析器；构造时注入 intents.yaml 的映射内容。"""

    def __init__(self, intents: dict[str, dict[str, Any]]):
        self._intents = intents

    def parse(self, text: str) -> IntentCommand:
        """解析文本，返回第一个命中的意图命令。

        参数 ``text``：原始用户文本（保留原文写入 ``source_text`` 以便审计）。

        异常：
            UnknownCommandError：文本为空或去空白后为空；
            UnknownCommandError：所有意图的所有模式都未命中。
        """
        normalized = _normalize_text(text)
        if not normalized:
            raise UnknownCommandError("empty command text")

        for intent_name, spec in self._intents.items():
            for pattern in spec.get("patterns", []):
                if _normalize_text(pattern) in normalized:
                    # params 深拷贝：命令构造后不再与配置共享同一字典对象。
                    return IntentCommand(
                        intent=intent_name,
                        command=str(spec.get("command", intent_name)),
                        params=deepcopy(spec.get("params", {})),
                        need_confirm=bool(spec.get("need_confirm", False)),
                        source_text=text,
                        priority=str(spec.get("priority", "normal")),
                    )

        raise UnknownCommandError(f"unknown command text: {text}")

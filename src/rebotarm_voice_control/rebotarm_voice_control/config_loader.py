"""语音控制配置文件的统一加载入口。

职责：把 config/ 目录下的 YAML 读成 Python 映射，并对结构做最小校验。所有
配置都从**运行时包目录**（而非当前工作目录）解析，保证从任意路径启动结果一致。

涉及的文件：
    intents.yaml        中文说法 → 意图名/命令/参数/是否需要确认
    named_poses.yaml    命名位姿 → 坐标系与位姿数值
    task_templates.yaml 任务模板 → 有序步骤列表
    safety_limits.yaml  执行模式、真实调用开关、允许位姿与工作空间盒
    llm_config.yaml     大模型提供方配置（可被命令行覆盖）
    sim_config.yaml     仿真后端与动作名映射

安全相关：``load_voice_control_config`` 一次性把安全限值与意图表一起加载，
避免出现"意图有了但限值没加载"的中间态。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .models import VoiceControlConfig


def _load_yaml(path: Path) -> dict[str, Any]:
    """读取单个 YAML 文件并要求顶层是映射（mapping）。

    空文件视为空映射（``or {}``），便于渐进式补配置；顶层是列表/标量时抛
    ValueError——这类结构错误必须尽早暴露，而不是留给下游按 None 处理。
    """
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return loaded


def load_voice_control_config(config_root: str | Path) -> VoiceControlConfig:
    """加载四份核心配置并聚合为 ``VoiceControlConfig``。

    参数 ``config_root``：包含上述 YAML 的目录（通常为 <包根>/config）。
    缺少任一文件都会直接抛 FileNotFoundError，不提供默认值兜底。
    """
    root = Path(config_root)
    return VoiceControlConfig(
        intents=_load_yaml(root / "intents.yaml"),
        named_poses=_load_yaml(root / "named_poses.yaml"),
        task_templates=_load_yaml(root / "task_templates.yaml"),
        safety_limits=_load_yaml(root / "safety_limits.yaml"),
    )


def load_llm_provider_config(
    config_root: str | Path,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """加载大模型提供方配置，并叠加命令行等外部覆盖项。

    覆盖规则：``overrides`` 中值为 None 或空串的键视为"未指定"而跳过，因此
    命令行未传的选项不会把 YAML 里的有效配置清空。

    返回：合并后的配置字典（provider / model / base_url / api_key_env 等）。
    """
    root = Path(config_root)
    config = _load_yaml(root / "llm_config.yaml")
    for key, value in (overrides or {}).items():
        if value not in (None, ""):
            config[key] = value
    return config


def load_sim_config(config_root: str | Path) -> dict[str, Any]:
    """加载仿真后端配置：backend 取值与各动作名到仿真命名空间的映射。"""
    return _load_yaml(Path(config_root) / "sim_config.yaml")

from __future__ import annotations

from pathlib import Path
import shutil
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.llm_tool_node import handle_llm_text
from rebotarm_voice_control.llm_providers import LLMConfigurationError


def test_handle_llm_text_uses_provider_and_safety_pipeline():
    result = handle_llm_text(
        "向上移动 5 厘米",
        SRC / "config",
        provider_config={"provider": "mock"},
        execution_mode="sim",
    )

    assert result["provider"] == "mock"
    assert result["tool_call"]["tool"] == "move_relative"
    assert result["route"]["target"] == "/rebotarm/sim/move_relative"
    assert result["route"]["params"]["distance_m"] == 0.05


def test_handle_llm_text_loads_provider_config_by_default(tmp_path):
    config_root = tmp_path / "config"
    shutil.copytree(SRC / "config", config_root)
    (config_root / "llm_config.yaml").write_text(
        'provider: "unsupported_provider"\n',
        encoding="utf-8",
    )

    with pytest.raises(LLMConfigurationError, match="unsupported_provider"):
        handle_llm_text("回到初始位置", config_root)

from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import (
    load_llm_provider_config,
    load_voice_control_config,
)


def test_load_default_config_contains_v1_named_poses():
    config_root = SRC / "config"

    config = load_voice_control_config(config_root)

    assert set(config.named_poses) == {"home", "camera_pose", "place_left"}
    assert config.safety_limits["execution_mode"] == "dry_run"


def test_intents_include_stop_highest_priority():
    config_root = SRC / "config"

    config = load_voice_control_config(config_root)

    stop = config.intents["stop_motion"]
    assert stop["priority"] == "highest"
    assert "急停" in stop["patterns"]


def test_load_llm_provider_config_defaults_to_doubao():
    config_root = SRC / "config"

    config = load_llm_provider_config(config_root)

    assert config["provider"] == "doubao"
    assert config["api_key_env"] == "ARK_API_KEY"
    assert config["base_url"] == "https://ark.cn-beijing.volces.com/api/v3"
    assert config["response_format"] == "json_object"
    assert config["temperature"] == 0


def test_load_llm_provider_config_allows_cli_overrides():
    config_root = SRC / "config"

    config = load_llm_provider_config(
        config_root,
        overrides={"provider": "mock", "model": "ignored-for-mock", "empty": ""},
    )

    assert config["provider"] == "mock"
    assert config["model"] == "ignored-for-mock"
    assert "empty" not in config

from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.command_router import DryRunCommandRouter
from rebotarm_voice_control.config_loader import load_voice_control_config
from rebotarm_voice_control.intent_parser import IntentParser


def test_open_gripper_routes_to_gripper_set_service():
    config = load_voice_control_config(SRC / "config")
    command = IntentParser(config.intents).parse("打开夹爪")

    result = DryRunCommandRouter(config).route(command)

    assert result.target == "/rebotarm/gripper/set"
    assert result.mode == "service"
    assert result.dry_run is True
    assert result.params["position"] == 0.09


def test_camera_pose_routes_to_move_to_pose_action():
    config = load_voice_control_config(SRC / "config")
    command = IntentParser(config.intents).parse("移动到拍照位")

    result = DryRunCommandRouter(config).route(command)

    assert result.target == "/rebotarm/move_to_pose"
    assert result.mode == "action"
    assert result.params["pose"] == config.named_poses["camera_pose"]


def test_safe_home_routes_to_existing_service():
    config = load_voice_control_config(SRC / "config")
    command = IntentParser(config.intents).parse("回零")

    result = DryRunCommandRouter(config).route(command)

    assert result.target == "/rebotarm/safe_home"
    assert result.mode == "service"

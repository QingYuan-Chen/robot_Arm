from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_voice_control_config
from rebotarm_voice_control.intent_parser import IntentParser
from rebotarm_voice_control.models import UnknownCommandError


def _parser() -> IntentParser:
    config = load_voice_control_config(SRC / "config")
    return IntentParser(config.intents)


def test_parse_open_gripper_synonym():
    command = _parser().parse("请打开夹爪")

    assert command.intent == "open_gripper"
    assert command.command == "set_gripper"
    assert command.params["position"] == 0.09
    assert command.need_confirm is False
    assert command.source_text == "请打开夹爪"


def test_parse_camera_pose_requires_confirmation():
    command = _parser().parse("移动到拍照位")

    assert command.intent == "move_camera_pose"
    assert command.command == "move_named_pose"
    assert command.params == {"name": "camera_pose"}
    assert command.need_confirm is True


def test_parse_stop_highest_priority():
    command = _parser().parse("急停")

    assert command.intent == "stop_motion"
    assert command.priority == "highest"


def test_unknown_command_is_rejected():
    with pytest.raises(UnknownCommandError):
        _parser().parse("随便转一下")

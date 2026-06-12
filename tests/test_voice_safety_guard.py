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
from rebotarm_voice_control.models import IntentCommand, SafetyViolationError
from rebotarm_voice_control.safety_guard import SafetyGuard


def _guard_and_parser():
    config = load_voice_control_config(SRC / "config")
    return SafetyGuard(config), IntentParser(config.intents)


def test_open_gripper_passes_safety():
    guard, parser = _guard_and_parser()

    command = guard.validate(parser.parse("打开夹爪"))

    assert command.intent == "open_gripper"


def test_unknown_named_pose_is_rejected():
    config = load_voice_control_config(SRC / "config")
    guard = SafetyGuard(config)

    command = IntentCommand(
        intent="move_named_pose",
        command="move_named_pose",
        params={"name": "pick_down"},
        need_confirm=True,
        source_text="移动到取物位",
    )

    with pytest.raises(SafetyViolationError, match="named pose is not allowed"):
        guard.validate(command)


def test_workspace_violation_is_rejected():
    config = load_voice_control_config(SRC / "config")
    config.named_poses["bad_pose"] = {
        "frame_id": "base_link",
        "position": [2.0, 0.0, 0.2],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    config.safety_limits["allowed_named_poses"].append("bad_pose")
    guard = SafetyGuard(config)

    command = IntentCommand(
        intent="move_named_pose",
        command="move_named_pose",
        params={"name": "bad_pose"},
        need_confirm=True,
        source_text="移动到危险点",
    )

    with pytest.raises(SafetyViolationError, match="outside workspace"):
        guard.validate(command)


def test_raw_joint_command_is_rejected():
    config = load_voice_control_config(SRC / "config")
    guard = SafetyGuard(config)

    command = IntentCommand(
        intent="raw_joint_move",
        command="raw_joint_move",
        params={"joint1": 1.0},
        source_text="一号关节转一下",
    )

    with pytest.raises(SafetyViolationError, match="not whitelisted"):
        guard.validate(command)

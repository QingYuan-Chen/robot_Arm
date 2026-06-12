from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_voice_control_config
from rebotarm_voice_control.intent_parser import IntentParser
from rebotarm_voice_control.safety_guard import SafetyGuard
from rebotarm_voice_control.task_planner import TaskPlanner


def test_non_template_command_returns_single_step():
    config = load_voice_control_config(SRC / "config")
    planner = TaskPlanner(config, SafetyGuard(config))
    command = IntentParser(config.intents).parse("打开夹爪")

    steps = planner.expand(command)

    assert len(steps) == 1
    assert steps[0].intent == "open_gripper"


def test_pick_place_demo_expands_to_safe_dry_run_steps():
    config = load_voice_control_config(SRC / "config")
    planner = TaskPlanner(config, SafetyGuard(config))
    command = IntentParser(config.intents).parse("执行搬运演示")

    steps = planner.expand(command)

    assert [step.command for step in steps] == [
        "move_named_pose",
        "set_gripper",
        "move_named_pose",
        "set_gripper",
        "safe_home",
    ]
    assert steps[0].params["name"] == "camera_pose"
    assert steps[2].params["name"] == "place_left"

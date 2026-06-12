from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.text_input_node import handle_text_command


def test_handle_text_command_returns_dry_run_routes():
    result = handle_text_command("移动到拍照位", SRC / "config")

    assert result["source_text"] == "移动到拍照位"
    assert result["steps"][0]["target"] == "/rebotarm/move_to_pose"
    assert result["steps"][0]["dry_run"] is True


def test_handle_task_template_returns_multiple_routes():
    result = handle_text_command("执行搬运演示", SRC / "config")

    assert len(result["steps"]) == 5
    assert result["steps"][0]["target"] == "/rebotarm/move_to_pose"
    assert result["steps"][-1]["target"] == "/rebotarm/safe_home"

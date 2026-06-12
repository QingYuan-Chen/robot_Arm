from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.models import SafetyViolationError
from rebotarm_voice_control.tool_call_schema import ToolCallParser


def test_tool_call_json_move_relative_requires_units():
    call = ToolCallParser().parse_json(
        """
        {
          "tool": "move_relative",
          "arguments": {
            "axis": "z",
            "distance": 5,
            "unit": "cm"
          },
          "call_id": "call_1"
        }
        """
    )

    assert call.tool == "move_relative"
    assert call.arguments == {"axis": "z", "distance": 5, "unit": "cm"}
    assert call.call_id == "call_1"


def test_tool_call_without_unit_is_rejected():
    with pytest.raises(SafetyViolationError, match="unit"):
        ToolCallParser().parse_json(
            '{"tool":"move_relative","arguments":{"axis":"z","distance":5}}'
        )


def test_unknown_tool_is_rejected():
    with pytest.raises(SafetyViolationError, match="not whitelisted"):
        ToolCallParser().parse_json('{"tool":"set_joint_positions","arguments":{}}')

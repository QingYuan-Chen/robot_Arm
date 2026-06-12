from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.tool_call_node import handle_tool_call_json
from rebotarm_voice_control.realtime_event_node import handle_realtime_event_json


def test_handle_tool_call_json_supports_sim_mode():
    result = handle_tool_call_json(
        '{"tool":"move_relative","arguments":{"axis":"z","distance":5,"unit":"cm"}}',
        SRC / "config",
        execution_mode="sim",
    )

    assert result["tool"] == "move_relative"
    assert result["intent"] == "move_relative"
    assert result["execution_mode"] == "sim"
    assert result["route"]["target"] == "/rebotarm/sim/move_relative"
    assert result["route"]["dry_run"] is False


def test_handle_realtime_event_json_supports_dry_run_mode():
    result = handle_realtime_event_json(
        """
        {
          "type": "response.function_call_arguments.done",
          "call_id": "call_stop",
          "name": "stop_robot",
          "arguments": "{\\"level\\":\\"soft_stop\\"}"
        }
        """,
        SRC / "config",
        execution_mode="dry_run",
    )

    assert result["call_id"] == "call_stop"
    assert result["tool"] == "stop_robot"
    assert result["route"]["target"] == "/rebotarm/stop"
    assert result["route"]["dry_run"] is True

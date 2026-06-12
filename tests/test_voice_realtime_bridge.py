from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_voice_control_config
from rebotarm_voice_control.realtime_bridge import RealtimeToolBridge


def test_realtime_function_call_event_routes_to_dry_run_result():
    config = load_voice_control_config(SRC / "config")
    bridge = RealtimeToolBridge(config)

    result = bridge.handle_event(
        {
            "type": "response.function_call_arguments.done",
            "call_id": "call_home",
            "name": "move_home",
            "arguments": "{}",
        }
    )

    assert result["call_id"] == "call_home"
    assert result["tool"] == "move_home"
    assert result["route"]["target"] == "/rebotarm/safe_home"
    assert result["route"]["dry_run"] is True


def test_realtime_non_tool_event_is_ignored():
    config = load_voice_control_config(SRC / "config")
    bridge = RealtimeToolBridge(config)

    assert bridge.handle_event({"type": "response.audio.delta", "delta": "abc"}) is None

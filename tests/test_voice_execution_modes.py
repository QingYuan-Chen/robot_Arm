from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.config_loader import load_voice_control_config
from rebotarm_voice_control.execution_modes import ExecutionModeRouter
from rebotarm_voice_control.models import IntentCommand, SafetyViolationError


def test_dry_run_mode_does_not_execute_motion():
    config = load_voice_control_config(SRC / "config")
    router = ExecutionModeRouter(config, execution_mode="dry_run")

    result = router.route(
        IntentCommand(intent="move_home", command="safe_home", need_confirm=True)
    )

    assert result.execution_mode == "dry_run"
    assert result.route.dry_run is True
    assert result.simulated is False


def test_sim_mode_marks_motion_as_simulated_without_real_hardware():
    config = load_voice_control_config(SRC / "config")
    router = ExecutionModeRouter(config, execution_mode="sim")

    result = router.route(
        IntentCommand(
            intent="move_relative",
            command="move_relative",
            params={"axis": "z", "distance_m": 0.05},
            need_confirm=True,
        )
    )

    assert result.execution_mode == "sim"
    assert result.simulated is True
    assert result.route.target == "/rebotarm/sim/move_relative"
    assert result.route.dry_run is False


def test_real_mode_is_rejected_until_explicitly_allowed():
    config = load_voice_control_config(SRC / "config")
    router = ExecutionModeRouter(config, execution_mode="real")

    with pytest.raises(SafetyViolationError, match="real ROS calls are disabled"):
        router.route(IntentCommand(intent="move_home", command="safe_home"))

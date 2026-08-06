from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_mujoco_motor_ctrlranges_match_urdf_position_limits():
    from rebotarm_simulation.mujoco_limit_checks import motor_profile_position_limit_mismatches
    from rebotarm_simulation.mujoco_model_profile import MOTOR_PROFILES

    mismatches = motor_profile_position_limit_mismatches(
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf",
        MOTOR_PROFILES,
    )

    assert mismatches == []


def test_limit_check_reports_drifted_motor_ctrlrange():
    from rebotarm_simulation.mujoco_limit_checks import motor_profile_position_limit_mismatches
    from rebotarm_simulation.mujoco_model_profile import MotorProfile

    profiles = [
        MotorProfile("joint1", "-2.7 2.8", "-27 27", "270", "24"),
    ]

    mismatches = motor_profile_position_limit_mismatches(
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf",
        profiles,
    )

    assert len(mismatches) == 1
    assert mismatches[0].joint == "joint1"
    assert mismatches[0].urdf_lower == pytest.approx(-2.8)
    assert mismatches[0].mujoco_lower == pytest.approx(-2.7)

from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_health_check_reports_finite_model_and_bounded_counts() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.mujoco_health import check_model_health
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML

    report = check_model_health(DEFAULT_GRIPPER_XML, steps=2)

    assert report.ok is True
    assert report.model_loaded is True
    assert report.physics_step_finite is True
    assert report.joint_count == 8
    assert report.actuator_count == 7
    assert report.simulation_time == pytest.approx(0.005)


def test_health_check_rejects_invalid_step_count() -> None:
    from rebotarm_simulation.mujoco_health import check_model_health
    from rebotarm_simulation.mujoco_model_profile import DEFAULT_GRIPPER_XML

    with pytest.raises(ValueError, match="steps"):
        check_model_health(DEFAULT_GRIPPER_XML, steps=0)

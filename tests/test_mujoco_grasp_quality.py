from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_grasp_quality_detects_contact_and_lift_success():
    from rebotarm_simulation.mujoco_grasp_quality import evaluate_grasp_quality

    quality = evaluate_grasp_quality(
        contact_count=4,
        initial_box_height_m=0.05,
        final_box_height_m=0.12,
        min_lift_m=0.03,
    )

    assert quality.contact_detected is True
    assert quality.lift_detected is True
    assert quality.success is True
    assert quality.status == "grasp_lift_success"


def test_grasp_quality_rejects_contact_without_lift():
    from rebotarm_simulation.mujoco_grasp_quality import evaluate_grasp_quality

    quality = evaluate_grasp_quality(
        contact_count=3,
        initial_box_height_m=0.05,
        final_box_height_m=0.052,
        min_lift_m=0.03,
    )

    assert quality.contact_detected is True
    assert quality.lift_detected is False
    assert quality.success is False
    assert quality.status == "contact_without_lift"

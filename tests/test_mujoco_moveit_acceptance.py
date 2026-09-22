from __future__ import annotations

import io
from pathlib import Path

from rebotarm_simulation import mujoco_moveit_acceptance


def test_moveit_acceptance_entrypoint_reports_runtime_errors_without_hardware(monkeypatch):
    output = io.StringIO()
    errors = io.StringIO()

    def fail_acceptance(**_kwargs):
        raise RuntimeError("moveit acceptance setup failed")

    monkeypatch.setattr(mujoco_moveit_acceptance, "run_acceptance", fail_acceptance)
    code = mujoco_moveit_acceptance.main(["--timeout", "0.1"], stdout=output, stderr=errors)

    assert code == 1
    assert output.getvalue() == ""
    assert "moveit acceptance setup failed" in errors.getvalue()
    assert "use_hardware" not in errors.getvalue()


def test_moveit_acceptance_source_plans_with_moveit_then_executes_mujoco_action():
    source = Path(
        "src/rebotarm_simulation/rebotarm_simulation/mujoco_moveit_acceptance.py"
    ).read_text(encoding="utf-8")

    required = (
        "/plan_kinematic_path",
        "/rebotarm/follow_joint_trajectory",
        "/rebotarm/joint_states",
        "/clock",
        "GetMotionPlan",
        "ActionClient",
        "FollowJointTrajectory",
        "MOVEIT_ACCEPTANCE_TARGET",
        "moveit_plan_success",
        "trajectory_action_success",
        "final_max_joint_error_rad",
    )
    for value in required:
        assert value in source
    assert "rebotarmcontroller" not in source.lower()
    assert "use_hardware" not in source

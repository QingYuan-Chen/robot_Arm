from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_interpolates_follow_joint_trajectory_targets():
    from rebotarm_simulation.mujoco_adapter_core import (
        TrajectoryPoint,
        interpolate_trajectory,
    )

    points = [
        TrajectoryPoint(time_from_start=0.0, positions=[0.0, -0.2]),
        TrajectoryPoint(time_from_start=2.0, positions=[1.0, -0.6]),
    ]

    assert interpolate_trajectory(points, 1.0) == pytest.approx([0.5, -0.4])
    assert interpolate_trajectory(points, -1.0) == pytest.approx([0.0, -0.2])
    assert interpolate_trajectory(points, 3.0) == pytest.approx([1.0, -0.6])


def test_normalizes_trajectory_goal_to_adapter_joint_order():
    from rebotarm_simulation.mujoco_adapter_core import normalize_trajectory_points

    points = [
        {"positions": [0.2, -0.5], "time_from_start": 1.0},
        {"positions": [0.4, -0.7], "time_from_start": 2.0},
    ]

    normalized = normalize_trajectory_points(
        source_joint_names=["joint2", "joint1"],
        target_joint_names=["joint1", "joint2"],
        points=points,
        current_positions=[0.0, 0.0],
    )

    assert normalized[0].positions == pytest.approx([0.0, 0.0])
    assert normalized[1].positions == pytest.approx([-0.5, 0.2])
    assert normalized[2].positions == pytest.approx([-0.7, 0.4])
    assert [point.time_from_start for point in normalized] == pytest.approx([0.0, 1.0, 2.0])


def test_normalization_prepends_current_state_for_delayed_first_goal_point():
    from rebotarm_simulation.mujoco_adapter_core import interpolate_trajectory, normalize_trajectory_points

    normalized = normalize_trajectory_points(
        source_joint_names=["joint1", "joint2"],
        target_joint_names=["joint1", "joint2"],
        points=[{"positions": [1.0, -0.6], "time_from_start": 2.0}],
        current_positions=[0.2, -0.4],
    )

    assert normalized[0].time_from_start == pytest.approx(0.0)
    assert normalized[0].positions == pytest.approx([0.2, -0.4])
    assert normalized[1].time_from_start == pytest.approx(2.0)
    assert normalized[1].positions == pytest.approx([1.0, -0.6])
    assert interpolate_trajectory(normalized, 1.0) == pytest.approx([0.6, -0.5])


def test_normalization_rejects_trajectory_without_arm_joints():
    from rebotarm_simulation.mujoco_adapter_core import normalize_trajectory_points

    with pytest.raises(ValueError, match="no supported joints"):
        normalize_trajectory_points(
            source_joint_names=["unknown_joint"],
            target_joint_names=["joint1", "joint2"],
            points=[{"positions": [1.0], "time_from_start": 1.0}],
            current_positions=[0.0, 0.0],
        )


def test_gripper_width_maps_to_mujoco_control_range():
    from rebotarm_simulation.mujoco_adapter_core import gripper_width_to_ctrl

    assert gripper_width_to_ctrl(0.03, min_width=0.0, max_width=0.09) == pytest.approx(0.015)
    assert gripper_width_to_ctrl(0.20, min_width=0.0, max_width=0.09) == pytest.approx(0.045)
    assert gripper_width_to_ctrl(-1.0, min_width=0.0, max_width=0.09) == pytest.approx(0.0)


def test_wall_time_is_accumulated_into_mujoco_step_count():
    from rebotarm_simulation.mujoco_adapter_core import consume_sim_steps

    steps, remainder = consume_sim_steps(
        wall_delta=0.005,
        timestep=0.0025,
        pending_sim_seconds=0.0,
    )

    assert steps == 2
    assert remainder == pytest.approx(0.0)


def test_fractional_wall_time_is_preserved_for_later_mujoco_steps():
    from rebotarm_simulation.mujoco_adapter_core import consume_sim_steps

    steps, remainder = consume_sim_steps(
        wall_delta=0.00125,
        timestep=0.0025,
        pending_sim_seconds=0.0,
    )
    assert steps == 0
    assert remainder == pytest.approx(0.00125)

    steps, remainder = consume_sim_steps(
        wall_delta=0.00125,
        timestep=0.0025,
        pending_sim_seconds=remainder,
    )
    assert steps == 1
    assert remainder == pytest.approx(0.0)


def test_follow_joint_trajectory_stop_reasons_map_to_non_success_codes():
    from types import SimpleNamespace

    from rebotarm_simulation.mujoco_adapter_core import trajectory_error_code_for_stop_reason

    result_type = SimpleNamespace(
        SUCCESSFUL=0,
        PATH_TOLERANCE_VIOLATED=-4,
        GOAL_TOLERANCE_VIOLATED=-5,
    )

    assert trajectory_error_code_for_stop_reason("finished", result_type=result_type) == 0
    assert trajectory_error_code_for_stop_reason("stopped", result_type=result_type) == -4
    assert trajectory_error_code_for_stop_reason("canceled", result_type=result_type) == -4
    assert trajectory_error_code_for_stop_reason("goal_tolerance_violated", result_type=result_type) == -5


def test_timeout_is_a_non_success_trajectory_stop_reason():
    from types import SimpleNamespace

    from rebotarm_simulation.mujoco_adapter_core import trajectory_error_code_for_stop_reason

    result_type = SimpleNamespace(
        SUCCESSFUL=0,
        PATH_TOLERANCE_VIOLATED=-4,
        GOAL_TOLERANCE_VIOLATED=-5,
    )

    assert trajectory_error_code_for_stop_reason("timeout", result_type=result_type) == -4


def test_execution_timeout_budget_includes_trajectory_duration_and_margin():
    from rebotarm_simulation.mujoco_adapter_core import execution_timeout_seconds

    assert execution_timeout_seconds(2.0, margin_sec=1.5) == pytest.approx(3.5)
    assert execution_timeout_seconds(-1.0, margin_sec=-1.0) > 0.0


def test_first_tolerance_violation_reports_joint_and_error():
    from rebotarm_simulation.mujoco_adapter_core import first_tolerance_violation

    violation = first_tolerance_violation(
        joint_names=["joint1", "joint2"],
        errors=[0.02, -0.08],
        tolerance=0.05,
    )

    assert violation is not None
    assert violation.joint == "joint2"
    assert violation.error == -0.08
    assert violation.abs_error == 0.08


def test_zero_tolerance_disables_violation_check():
    from rebotarm_simulation.mujoco_adapter_core import first_tolerance_violation

    assert first_tolerance_violation(
        joint_names=["joint1"],
        errors=[100.0],
        tolerance=0.0,
    ) is None


def test_default_step_response_targets_use_ctrlrange_midpoints():
    from rebotarm_simulation.mujoco_adapter_core import default_step_response_targets
    from rebotarm_simulation.mujoco_model_profile import MotorProfile

    targets = default_step_response_targets(
        [
            MotorProfile("joint1", "-2.8 2.8", "-27 27", "270", "24"),
            MotorProfile("joint2", "-3.14 0", "-27 27", "270", "24"),
        ]
    )

    assert targets == {"joint1": 1.4, "joint2": -0.785}

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

    assert normalized[0].positions == pytest.approx([-0.5, 0.2])
    assert normalized[1].positions == pytest.approx([-0.7, 0.4])
    assert [point.time_from_start for point in normalized] == pytest.approx([1.0, 2.0])


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

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from rebotarm_simulation.sim2real.schemas import (
    ComparisonReport,
    TrajectoryMetrics,
    TrajectorySample,
)


def _sample(**overrides):
    values = dict(
        schema_version=1,
        episode_id="episode-1",
        step_index=0,
        simulation_time=0.01,
        joint_positions=(0.0,) * 6,
        joint_velocities=(0.1,) * 6,
        joint_targets=(0.2,) * 6,
        actuator_torques=(1.0,) * 6,
        gripper_width=0.05,
        gripper_target_width=0.04,
        end_effector_position=(0.2, 0.0, 0.2),
        end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        action=(0.0,) * 7,
        max_contact_force=0.0,
        contact_count=0,
        source="sim",
    )
    values.update(overrides)
    return TrajectorySample(**values)


def test_trajectory_sample_normalizes_sequences_and_serializes_to_json_values():
    sample = _sample(joint_positions=[0.1] * 6)

    assert isinstance(sample.joint_positions, tuple)
    payload = sample.to_dict()
    assert payload["schema_version"] == 1
    assert payload["joint_positions"] == [0.1] * 6
    assert payload["source"] == "sim"


def test_trajectory_sample_is_immutable():
    sample = _sample()

    with pytest.raises(FrozenInstanceError):
        sample.step_index = 1


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("joint_positions", (0.0,) * 5, "6 values"),
        ("joint_velocities", (0.0,) * 7, "6 values"),
        ("end_effector_position", (0.0,) * 2, "3 values"),
        ("end_effector_orientation_xyzw", (0.0,) * 3, "4 values"),
        ("action", (0.0,) * 8, "6 or 7 values"),
        ("simulation_time", float("nan"), "finite"),
        ("max_contact_force", -1.0, "non-negative"),
        ("contact_count", -1, "non-negative"),
        ("source", "hardware", "source"),
    ],
)
def test_trajectory_sample_rejects_invalid_values(field, value, message):
    with pytest.raises(ValueError, match=message):
        _sample(**{field: value})


def test_trajectory_metrics_and_comparison_report_are_json_serializable():
    metrics = TrajectoryMetrics(
        joint_position_rmse=0.01,
        joint_position_max=0.02,
        joint_velocity_rmse=0.03,
        joint_velocity_max=0.04,
        end_effector_position_rmse=0.005,
        end_effector_position_max=0.006,
        gripper_width_rmse=0.001,
        gripper_width_max=0.002,
        actuator_torque_rmse=0.1,
        actuator_torque_max=0.2,
        contact_force_rmse=0.3,
        contact_force_max=0.4,
    )
    report = ComparisonReport(
        ok=True,
        metrics=metrics,
        validation_errors=(),
        sample_count=2,
    )

    assert report.to_dict() == {
        "ok": True,
        "metrics": metrics.to_dict(),
        "validation_errors": [],
        "sample_count": 2,
    }

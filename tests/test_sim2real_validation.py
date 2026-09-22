from __future__ import annotations

from dataclasses import replace

from rebotarm_simulation.sim2real.schemas import TrajectorySample
from rebotarm_simulation.sim2real.validation import SafetyLimits, validate_trajectory


def _sample(**changes):
    sample = TrajectorySample(
        schema_version=1,
        episode_id="episode-1",
        step_index=0,
        simulation_time=0.01,
        joint_positions=(0.0,) * 6,
        joint_velocities=(0.0,) * 6,
        joint_targets=(0.0,) * 6,
        actuator_torques=(0.0,) * 6,
        gripper_width=0.05,
        gripper_target_width=0.05,
        end_effector_position=(0.2, 0.0, 0.2),
        end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        action=(0.0,) * 7,
        max_contact_force=1.0,
        contact_count=1,
        source="sim",
        max_contact_penetration=0.001,
    )
    return replace(sample, **changes)


def _limits():
    return SafetyLimits(
        joint_position_limits=((-1.0, 1.0),) * 6,
        actuator_torque_limits=(10.0,) * 6,
        max_contact_force=20.0,
        max_contact_penetration=0.005,
    )


def test_validate_trajectory_accepts_bounded_samples():
    report = validate_trajectory([_sample()], _limits())

    assert report["ok"] is True
    assert report["sample_count"] == 1
    assert report["violation_count"] == 0
    assert report["limits"]["max_contact_penetration"] == 0.005


def test_validate_trajectory_reports_joint_torque_force_and_penetration():
    report = validate_trajectory(
        [
            _sample(
                joint_positions=(1.2, 0.0, 0.0, 0.0, 0.0, 0.0),
                actuator_torques=(0.0, 12.0, 0.0, 0.0, 0.0, 0.0),
                max_contact_force=25.0,
                max_contact_penetration=0.01,
            )
        ],
        _limits(),
    )

    assert report["ok"] is False
    assert {item["kind"] for item in report["violations"]} == {
        "joint_limit",
        "actuator_torque",
        "contact_force",
        "contact_penetration",
    }


def test_validate_trajectory_rejects_an_empty_log():
    report = validate_trajectory([], _limits())

    assert report["ok"] is False
    assert report["sample_count"] == 0

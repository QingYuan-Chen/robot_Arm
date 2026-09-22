from __future__ import annotations

import json

import pytest

from rebotarm_simulation.sim2real.schemas import TrajectorySample
from rebotarm_simulation.sim2real.trajectory_log import TrajectoryRecorder


def _sample(step_index=0, simulation_time=0.01, **overrides):
    values = dict(
        schema_version=1,
        episode_id="episode-1",
        step_index=step_index,
        simulation_time=simulation_time,
        joint_positions=(0.0,) * 6,
        joint_velocities=(0.1,) * 6,
        joint_targets=(0.2,) * 6,
        actuator_torques=(1.0,) * 6,
        gripper_width=0.05,
        gripper_target_width=0.04,
        end_effector_position=(0.2, 0.0, 0.2),
        end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        action=(0.0,) * 7,
        max_contact_force=2.0,
        contact_count=1,
        source="sim",
    )
    values.update(overrides)
    return TrajectorySample(**values)


def test_recorder_requires_matching_episode_and_source_and_monotonic_time():
    recorder = TrajectoryRecorder(episode_id="episode-1", source="sim")
    recorder.append(_sample())
    recorder.append(_sample(step_index=1, simulation_time=0.02))

    assert len(recorder) == 2
    with pytest.raises(ValueError, match="strictly increasing"):
        recorder.append(_sample(step_index=2, simulation_time=0.02))
    with pytest.raises(ValueError, match="episode_id"):
        recorder.append(_sample(step_index=2, simulation_time=0.03, episode_id="other"))
    with pytest.raises(ValueError, match="source"):
        recorder.append(_sample(step_index=2, simulation_time=0.03, source="real"))


def test_recorder_summary_reports_trajectory_bounds():
    recorder = TrajectoryRecorder(episode_id="episode-1", source="sim")
    recorder.append(_sample())
    recorder.append(
        _sample(
            step_index=1,
            simulation_time=0.02,
            joint_velocities=(0.4,) * 6,
            actuator_torques=(2.0,) * 6,
            max_contact_force=3.0,
        )
    )

    summary = recorder.summary()
    assert summary == {
        "episode_id": "episode-1",
        "source": "sim",
        "sample_count": 2,
        "start_time": 0.01,
        "end_time": 0.02,
        "max_joint_velocity": 0.4,
        "max_actuator_torque": 2.0,
        "max_contact_force": 3.0,
    }


def test_recorder_jsonl_round_trip(tmp_path):
    recorder = TrajectoryRecorder(episode_id="episode-1", source="sim")
    recorder.append(_sample())
    recorder.append(_sample(step_index=1, simulation_time=0.02))
    path = tmp_path / "trajectory.jsonl"

    recorder.to_jsonl(path)
    loaded = TrajectoryRecorder.from_jsonl(path)

    assert loaded.episode_id == recorder.episode_id
    assert loaded.source == recorder.source
    assert loaded.samples == recorder.samples
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["step_index"] == 0

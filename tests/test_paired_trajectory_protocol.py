from __future__ import annotations

import copy
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
for package in ("rebotarm_motion", "rebotarm_simulation"):
    source = ROOT / "src" / package
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))


def test_quintic_command_is_deterministic_and_hash_checked() -> None:
    from rebotarm_motion.paired_trajectory_protocol import (
        build_quintic_command,
        sample_command,
        validate_command,
    )

    command = build_quintic_command(
        [-1.6, 0.0, -0.04, 0.07, 0.03, 0.03],
        [-1.5707963268, -0.1, -0.2, 0.2, 0.0, 0.0],
        duration_sec=20.0,
        cadence_sec=0.05,
        label="to_safe_posture",
    )
    validate_command(command)
    assert len(command["points"]) == 401
    assert sample_command(command, 0.0) == pytest.approx(command["points"][0]["positions"])
    assert sample_command(command, 20.0) == pytest.approx(command["points"][-1]["positions"])
    assert sample_command(command, 10.0) == pytest.approx(
        [(a + b) * 0.5 for a, b in zip(command["points"][0]["positions"], command["points"][-1]["positions"])]
    )

    tampered = copy.deepcopy(command)
    tampered["points"][-1]["positions"][0] += 0.1
    with pytest.raises(ValueError, match="command_sha256 mismatch"):
        validate_command(tampered)


def test_analysis_reports_per_joint_and_paired_metrics() -> None:
    from rebotarm_motion.paired_trajectory_protocol import build_quintic_command, sample_command
    from rebotarm_simulation.paired_trajectory_analysis import analyze_run, compare_runs

    command = build_quintic_command(
        [0.0] * 6,
        [0.2, -0.2, -0.1, 0.1, 0.05, -0.05],
        duration_sec=1.0,
        cadence_sec=0.05,
        label="test",
    )

    def make_run(offset: float):
        samples = []
        for index in range(26):
            elapsed = index * 0.05
            desired = sample_command(command, elapsed)
            observed = [value - offset for value in desired]
            samples.append(
                {
                    "elapsed_sec": elapsed,
                    "desired_positions": desired,
                    "observed_positions": observed,
                    "velocities": [0.0] * 6,
                    "efforts": [float(index)] * 6,
                }
            )
        return {"command": command, "samples": samples}

    sim = make_run(0.0)
    real = make_run(0.01)
    summary = analyze_run(real)
    paired = compare_runs(sim, real)
    assert summary["sample_count"] == 26
    assert summary["per_joint"]["joint4"]["rms_tracking_error_rad"] == pytest.approx(0.01)
    assert paired["paired"]["joint4"]["rms_real_minus_sim_position_rad"] == pytest.approx(0.01)


def test_mujoco_can_reset_to_exact_paired_start_state() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.mujoco_sim import RebotArmMujoco

    start = [-1.6, -0.01, -0.04, 0.07, 0.03, 0.03]
    with RebotArmMujoco() as simulation:
        state = simulation.reset_joint_positions(start)
        assert state.joint_positions[:6] == pytest.approx(start, abs=1e-12)
        assert state.joint_velocities[:6] == pytest.approx([0.0] * 6, abs=1e-12)

        with pytest.raises(ValueError, match="joint2 position"):
            simulation.reset_joint_positions([-1.6, 0.5, -0.04, 0.07, 0.03, 0.03])

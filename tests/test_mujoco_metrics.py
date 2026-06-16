from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_metrics_recorder_writes_csv_and_summary(tmp_path):
    from rebotarm_simulation.mujoco_metrics import TrajectoryMetricsRecorder

    recorder = TrajectoryMetricsRecorder(tmp_path, joint_names=["joint1", "joint2"])
    recorder.record(
        elapsed=0.0,
        targets=[0.0, -0.5],
        actual=[0.1, -0.6],
        velocities=[0.2, 0.3],
        actuator_forces=[1.0, 2.0],
    )
    recorder.record(
        elapsed=1.0,
        targets=[0.2, -0.7],
        actual=[0.25, -0.72],
        velocities=[0.1, 0.4],
        actuator_forces=[3.0, 4.0],
    )

    summary = recorder.finish(success=True, stop_reason="finished")

    assert (tmp_path / "trajectory_metrics.csv").exists()
    assert (tmp_path / "summary.json").exists()
    assert summary["success"] is True
    assert summary["stop_reason"] == "finished"
    assert summary["joint_count"] == 2
    assert summary["max_abs_error"] == 0.1
    assert summary["max_abs_velocity"] == 0.4
    assert summary["max_abs_actuator_force"] == 4.0

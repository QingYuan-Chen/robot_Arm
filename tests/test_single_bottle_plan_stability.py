from __future__ import annotations

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "src/rebotarm_vision/rebotarm_vision/single_bottle_grasp.py"


def _run_runner_script(script: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            "-lc",
            'source install/setup.bash && export PYTHONPATH="$PWD/src/rebotarm_vision:$PWD/src/rebotarm_motion${PYTHONPATH:+:$PYTHONPATH}" && exec "$@"',
            "bash",
            sys.executable,
            "-c",
            script,
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_single_bottle_plan_requires_height_and_three_stable_sensor_frames() -> None:
    script = f"""
import importlib.util
from rebotarm_msgs.msg import GraspPlan

spec = importlib.util.spec_from_file_location('single_bottle_stability_test', {str(RUNNER_PATH)!r})
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

def plan(stamp_ns, x=0.20, y=-0.40, z=0.20, width=0.06):
    msg = GraspPlan()
    msg.valid = True
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.candidate.class_name = 'bottle'
    msg.candidate.confidence = 0.8
    msg.jaw_width = width
    msg.grasp_pose.position.x = x
    msg.grasp_pose.position.y = y
    msg.grasp_pose.position.z = z
    return msg

tracker = runner.PlanStabilityTracker(
    required_frames=3,
    xy_tolerance_m=0.015,
    z_tolerance_m=0.010,
    jaw_width_tolerance_m=0.010,
    min_grasp_z_m=0.05,
)
assert not tracker.update(plan(1_000_000_000, z=0.049))
assert tracker.sample_count == 0
assert not tracker.update(plan(2_000_000_000))
assert not tracker.update(plan(2_100_000_000, x=0.21, z=0.205, width=0.065))
assert tracker.update(plan(2_200_000_000, x=0.205, z=0.202, width=0.061))
assert tracker.accepted_plan is not None

tracker.reset()
assert not tracker.update(plan(3_000_000_000))
assert not tracker.update(plan(3_100_000_000, z=0.04))
assert tracker.sample_count == 0
"""
    result = _run_runner_script(script)
    assert result.returncode == 0, result.stderr


def test_single_bottle_plan_age_uses_sensor_stamp() -> None:
    script = f"""
import importlib.util
from rebotarm_msgs.msg import GraspPlan

spec = importlib.util.spec_from_file_location('single_bottle_age_test', {str(RUNNER_PATH)!r})
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
msg = GraspPlan()
msg.header.stamp.sec = 10
msg.header.stamp.nanosec = 250_000_000
assert runner._plan_sensor_age_sec(msg, now_ns=11_000_000_000) == 0.75
assert runner._plan_sensor_age_sec(msg, now_ns=12_000_000_000) == 1.75
"""
    result = _run_runner_script(script)
    assert result.returncode == 0, result.stderr

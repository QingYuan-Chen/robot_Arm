from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts" / "p1_upstream_mujoco"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def test_snapshot_environment_isolated_to_vendored_package() -> None:
    from run_upstream import snapshot_env, snapshot_python

    python = snapshot_python(ROOT)
    env = snapshot_env(ROOT)

    assert python == ROOT / "third_party/rebotarm_mujoco_venv/bin/python"
    assert env["PYTHONPATH"].split(":")[0] == str(
        ROOT / "third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_simulation"
    )
    assert str(ROOT / "src/rebotarm_simulation") not in env["PYTHONPATH"].split(":")


@pytest.mark.parametrize(
    "command",
    [
        ["ros2", "launch", "rebotarmcontroller", "driver.launch.py"],
        ["python3", "tool.py", "--channel", "/dev/ttyACM0"],
        ["python3", "tool.py", "--use-hardware"],
    ],
)
def test_runner_rejects_hardware_commands(command: list[str]) -> None:
    from run_upstream import validate_safe_command

    with pytest.raises(ValueError, match="hardware"):
        validate_safe_command(command)


def test_runner_accepts_headless_health_command() -> None:
    from run_upstream import validate_safe_command

    validate_safe_command(
        [
            "-m",
            "rebotarm_simulation.mujoco_health",
            "--model",
            "scene.xml",
            "--skip-renderer",
        ]
    )

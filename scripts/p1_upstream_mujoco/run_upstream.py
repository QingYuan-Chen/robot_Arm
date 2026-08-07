from __future__ import annotations

from pathlib import Path
import os
import subprocess
import sys
from subprocess import CompletedProcess
from typing import Sequence


SNAPSHOT_RELATIVE = Path("third_party/robotarm_ros2_mujoco_snapshot")
VENV_PYTHON_RELATIVE = Path("third_party/rebotarm_mujoco_venv/bin/python")


def snapshot_root(repo_root: Path) -> Path:
    return Path(repo_root).resolve() / SNAPSHOT_RELATIVE


def snapshot_python(repo_root: Path) -> Path:
    python = Path(repo_root).resolve() / VENV_PYTHON_RELATIVE
    if not python.is_file():
        raise FileNotFoundError(
            f"pinned MuJoCo Python environment is missing: {python}; "
            "install requirements-mujoco.txt first"
        )
    return python


def snapshot_env(repo_root: Path) -> dict[str, str]:
    root = snapshot_root(repo_root)
    package_src = root / "src" / "rebotarm_simulation"
    if not package_src.is_dir():
        raise FileNotFoundError(f"upstream snapshot package is missing: {package_src}")
    env = dict(os.environ)
    env["PYTHONPATH"] = str(package_src)
    env.pop("AMENT_PREFIX_PATH", None)
    env.pop("COLCON_PREFIX_PATH", None)
    return env


def validate_safe_command(command: Sequence[str]) -> None:
    text = " ".join(str(part) for part in command).lower()
    forbidden = (
        "/dev/tty",
        "rebotarmcontroller",
        "use_hardware",
        "use-hardware",
        "driver_only",
        "driver-only",
        "safe_home",
        "safe-home",
        "trajectory_stop",
        "trajectory-stop",
    )
    if any(token in text for token in forbidden):
        raise ValueError(f"hardware command rejected by upstream runner: {text}")


def run_upstream(
    repo_root: Path,
    command: Sequence[str],
    *,
    cwd: Path | None = None,
) -> CompletedProcess[str]:
    validate_safe_command(command)
    python = snapshot_python(repo_root)
    env = snapshot_env(repo_root)
    process = subprocess.run(
        [str(python), *[str(part) for part in command]],
        cwd=str(cwd or snapshot_root(repo_root)),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    return process


def main(argv: Sequence[str] | None = None) -> int:
    command = list(sys.argv[1:] if argv is None else argv)
    if not command:
        print("usage: run_upstream.py <python module/script arguments>", file=sys.stderr)
        return 2
    try:
        result = run_upstream(Path(__file__).resolve().parents[2], command)
    except (FileNotFoundError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    return int(result.returncode)


if __name__ == "__main__":
    raise SystemExit(main())

"""Launch the pinned upstream MuJoCo ROS adapter in an isolated process.

The upstream package remains byte-preserved under ``third_party``.  This
module is only a small process boundary: it supplies the snapshot path first
on ``PYTHONPATH`` and then replaces itself with the pinned MuJoCo interpreter.
The current adapter entrypoint remains available as the explicit fallback.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys
from typing import Mapping, Sequence


SNAPSHOT_RELATIVE = Path("third_party/robotarm_ros2_mujoco_snapshot")
PYTHON_RELATIVE = Path("third_party/rebotarm_mujoco_venv/bin/python")
PACKAGE_RELATIVE = Path("src/rebotarm_simulation")


def _layout(repo_root: Path) -> tuple[Path, Path, Path]:
    root = Path(repo_root).resolve()
    snapshot = root / SNAPSHOT_RELATIVE
    package_src = snapshot / PACKAGE_RELATIVE
    python = root / PYTHON_RELATIVE
    if not package_src.is_dir():
        raise FileNotFoundError(f"upstream MuJoCo package is missing: {package_src}")
    if not python.is_file():
        raise FileNotFoundError(f"pinned MuJoCo Python is missing: {python}")
    return snapshot, package_src, python


def _candidate_roots(start: Path) -> tuple[Path, ...]:
    resolved = Path(start).resolve()
    candidates = (resolved,) + tuple(resolved.parents)
    return tuple(dict.fromkeys(candidates))


def find_repo_root(start: Path | None = None) -> Path:
    configured = os.environ.get("REBOTARM_REPO_ROOT", "").strip()
    if configured:
        root = Path(configured).expanduser().resolve()
        _layout(root)
        return root
    roots: list[Path] = []
    if start is not None:
        roots.extend(_candidate_roots(Path(start)))
    roots.extend(_candidate_roots(Path.cwd()))
    roots.extend(_candidate_roots(Path(__file__).resolve()))
    for root in dict.fromkeys(roots):
        try:
            _layout(root)
        except FileNotFoundError:
            continue
        return root
    raise FileNotFoundError(
        "cannot locate the repository root containing the pinned upstream MuJoCo snapshot"
    )


def build_upstream_environment(
    repo_root: Path,
    *,
    base_environment: Mapping[str, str] | None = None,
) -> dict[str, str]:
    snapshot, package_src, _python = _layout(repo_root)
    environment = dict(os.environ if base_environment is None else base_environment)
    existing_pythonpath = environment.get("PYTHONPATH", "")
    entries = [str(package_src)]
    if existing_pythonpath:
        entries.append(existing_pythonpath)
    environment["PYTHONPATH"] = os.pathsep.join(entries)
    environment["REBOTARM_UPSTREAM_SNAPSHOT"] = str(snapshot)
    return environment


def build_upstream_command(
    repo_root: Path,
    args: Sequence[str] = (),
    *,
    python_executable: str | Path | None = None,
) -> list[str]:
    _snapshot, _package_src, pinned_python = _layout(repo_root)
    configured_python = str(python_executable or os.environ.get("REBOTARM_MUJOCO_PYTHON", ""))
    python = Path(configured_python).expanduser()
    if not configured_python:
        python = pinned_python
    elif not python.is_absolute():
        python = Path(repo_root).resolve() / python
    if not python.is_file():
        raise FileNotFoundError(f"MuJoCo Python executable is missing: {python}")
    return [
        str(python),
        "-m",
        "rebotarm_simulation.mujoco_ros_node",
        *[str(arg) for arg in args],
    ]


def main(args: Sequence[str] | None = None) -> None:
    repo_root = find_repo_root()
    forwarded = list(sys.argv[1:] if args is None else args)
    command = build_upstream_command(repo_root, forwarded)
    environment = build_upstream_environment(repo_root)
    os.execvpe(command[0], command, environment)


if __name__ == "__main__":
    main()

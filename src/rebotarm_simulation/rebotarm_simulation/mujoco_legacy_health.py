from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

from .mujoco_model_profile import DEFAULT_GRIPPER_XML, build_physics_profile_tree


@dataclass(frozen=True)
class ModelHealthReport:
    model_path: str
    model_loaded: bool
    physics_step_finite: bool
    joint_count: int
    actuator_count: int
    simulation_time: float
    ok: bool


def _load_mujoco() -> tuple[Any, Any]:
    try:
        import mujoco
        import numpy as np
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "MuJoCo health check requires the pinned MuJoCo environment; "
            "install requirements-mujoco.txt first"
        ) from exc
    return mujoco, np


def check_model_health(model_path: Path = DEFAULT_GRIPPER_XML, *, steps: int = 1) -> ModelHealthReport:
    if isinstance(steps, bool) or int(steps) < 1:
        raise ValueError("steps must be a positive integer")

    mujoco, np = _load_mujoco()
    path = Path(model_path).expanduser().resolve()
    if path == DEFAULT_GRIPPER_XML.resolve():
        # The package-owned source XML uses relative mesh names. Build the same
        # absolute-asset profile used by the current CLI before loading it.
        tree = build_physics_profile_tree(path)
        model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding="unicode"))
    else:
        model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    for _ in range(int(steps)):
        mujoco.mj_step(model, data)
    finite = bool(
        np.isfinite(data.qpos).all()
        and np.isfinite(data.qvel).all()
        and np.isfinite(data.actuator_force).all()
        and np.isfinite(data.time)
    )
    return ModelHealthReport(
        model_path=str(path),
        model_loaded=True,
        physics_step_finite=finite,
        joint_count=int(model.njnt),
        actuator_count=int(model.nu),
        simulation_time=float(data.time),
        ok=finite,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a headless MuJoCo model health check")
    parser.add_argument("--model", type=Path, default=DEFAULT_GRIPPER_XML)
    parser.add_argument("--steps", type=int, default=1)
    args = parser.parse_args(argv)
    report = check_model_health(args.model, steps=args.steps)
    print(json.dumps(asdict(report), ensure_ascii=False))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

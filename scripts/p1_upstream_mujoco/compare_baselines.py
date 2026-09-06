from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Mapping

from run_upstream import run_upstream


ROOT = Path(__file__).resolve().parents[2]
CURRENT_SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(CURRENT_SIM_SRC) not in sys.path:
    sys.path.insert(0, str(CURRENT_SIM_SRC))

from rebotarm_simulation.mujoco_health import check_model_health
from rebotarm_simulation.mujoco_model_profile import (
    MOTOR_PROFILES,
    build_physics_profile_tree,
    write_grasp_scene_profile,
    write_physics_profile,
)
from rebotarm_simulation.mujoco_runner import (
    run_grasp_benchmark,
    run_smoke,
    run_step_response_suite,
)
from rebotarm_simulation.mujoco_adapter_core import (
    ARM_JOINT_NAMES,
    MuJoCoArmAdapter,
    default_step_response_targets,
)


def common_command_contract(
    targets: Mapping[str, float], *, gripper_width: float = 0.04
) -> dict[str, Any]:
    """Return the normalized command sent to both simulation backends.

    This is intentionally a command-level contract. It does not claim that
    the two backends use the same actuator count or controller gains; those
    differences remain visible in the measured response fields.
    """
    expected = set(ARM_JOINT_NAMES)
    actual = {str(name) for name in targets}
    unknown = sorted(actual - expected)
    if unknown:
        raise ValueError(f"unknown arm joints: {unknown}")
    if actual != expected:
        raise ValueError("targets must contain exactly the arm joints")
    values = [float(targets[name]) for name in ARM_JOINT_NAMES]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("arm targets must be finite numbers")
    width = float(gripper_width)
    if not math.isfinite(width):
        raise ValueError("gripper width must be finite")
    return {
        "joint_names": list(ARM_JOINT_NAMES),
        "targets": values,
        "gripper_width": width,
    }


def normalize_model(model_path: Path) -> dict[str, Any]:
    import mujoco

    path = Path(model_path).resolve()
    model = mujoco.MjModel.from_xml_path(str(path))
    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(int(model.njnt))
    ]
    actuator_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(int(model.nu))
    ]
    return {
        "path": str(path),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "njnt": int(model.njnt),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nsensor": int(model.nsensor),
        "joint_names": joint_names,
        "actuator_names": actuator_names,
    }


def _flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    if isinstance(value, Mapping):
        output: dict[str, Any] = {}
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            output.update(_flatten(child, child_prefix))
        return output
    if isinstance(value, (list, tuple)):
        return {prefix: list(value)}
    return {prefix: value}


def compare_records(current: Mapping[str, Any], upstream: Mapping[str, Any]) -> dict[str, Any]:
    current_flat = _flatten(current)
    upstream_flat = _flatten(upstream)
    fields: dict[str, dict[str, Any]] = {}
    for key in sorted(set(current_flat) | set(upstream_flat)):
        fields[key] = {"current": current_flat.get(key), "upstream": upstream_flat.get(key)}
    return {
        "backends": [str(current.get("backend", "current")), str(upstream.get("backend", "upstream"))],
        "differences": fields,
    }


def _as_json(value: Any) -> Any:
    if is_dataclass(value):
        return _as_json(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _as_json(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_as_json(child) for child in value]
    return value


def _last_json_line(text: str) -> dict[str, Any]:
    for line in reversed(text.splitlines()):
        candidate = line.strip()
        if not candidate:
            continue
        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
    raise ValueError(f"no JSON object found in command output: {text[-500:]}")


def _test_summary(stdout: str, returncode: int) -> dict[str, Any]:
    summary: dict[str, Any] = {"returncode": int(returncode), "status": "passed" if returncode == 0 else "failed"}
    counts = {name: 0 for name in ("passed", "skipped", "failed", "deselected")}
    for value, name in re.findall(r"(\d+) (passed|skipped|failed|deselected)", stdout):
        counts[name] = max(counts[name], int(value))
    summary.update(counts)
    defect = "test_saved_integration_state_replays_deterministically" in stdout
    summary["known_test_defect"] = defect
    if defect:
        summary["defect_classification"] = "test_defect"
    return summary


def _prepare_current_models(output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    robot = write_physics_profile(output_dir / "current_robot.xml")
    scene_robot = output_dir / "current_scene_robot.xml"
    write_physics_profile(scene_robot, include_keyframes=False)
    scene = write_grasp_scene_profile(output_dir / "current_scene.xml", scene_robot)
    return robot, scene


def _run_upstream_json(command: list[str]) -> dict[str, Any]:
    process = run_upstream(ROOT, command)
    payload = _last_json_line(process.stdout)
    payload["returncode"] = int(process.returncode)
    if process.stderr:
        payload["stderr"] = process.stderr
    return payload


def _upstream_step_response(robot_path: Path, duration: float) -> dict[str, Any]:
    targets = default_step_response_targets(MOTOR_PROFILES)
    target_payload = json.dumps(targets, separators=(",", ":"))
    script = f"""
import json
from rebotarm_simulation.mujoco_sim import RebotArmMujoco

targets = json.loads({target_payload!r})
joint_names = tuple(f'joint{{index}}' for index in range(1, 7))
results = []
for joint_name, target in targets.items():
    with RebotArmMujoco({str(robot_path.resolve())!r}) as sim:
        command = [0.0] * 6
        command[joint_names.index(joint_name)] = float(target)
        sim.set_joint_position_targets(command)
        errors = []
        max_velocity = 0.0
        max_force = 0.0
        steps = max(1, int(float({duration}) / sim.timestep))
        for _ in range(steps):
            state = sim.step(1)
            position_index = joint_names.index(joint_name)
            errors.append(float(target) - state.joint_positions[position_index])
            max_velocity = max(max_velocity, abs(state.joint_velocities[position_index]))
            max_force = max(max_force, max(abs(value) for value in state.actuator_forces))
        results.append({{
            'joint': joint_name,
            'target': float(target),
            'final_abs_error': abs(errors[-1]),
            'max_abs_error': max(abs(value) for value in errors),
            'max_abs_velocity': max_velocity,
            'max_abs_actuator_force': max_force,
            'sim_time': state.simulation_time,
        }})
print(json.dumps({{'results': results, 'max_final_abs_error': max(item['final_abs_error'] for item in results), 'max_abs_error': max(item['max_abs_error'] for item in results), 'max_abs_velocity': max(item['max_abs_velocity'] for item in results), 'max_abs_actuator_force': max(item['max_abs_actuator_force'] for item in results)}}, separators=(',', ':')))
"""
    return _run_upstream_json(["-c", script])


def _current_common_command(
    robot_path: Path, contract: Mapping[str, Any], duration: float
) -> dict[str, Any]:
    targets = [float(value) for value in contract["targets"]]
    target_by_joint = dict(zip(ARM_JOINT_NAMES, targets))
    sim = MuJoCoArmAdapter(robot_path)
    sim.set_arm_targets(targets)
    sim.set_gripper_width(float(contract["gripper_width"]))
    errors: list[float] = []
    max_velocity = 0.0
    max_force = 0.0
    steps = max(1, int(float(duration) / sim.timestep))
    for _ in range(steps):
        sim.step(1)
        positions = sim.arm_positions()
        velocities = sim.arm_velocities()
        forces = sim.arm_actuator_forces()
        errors.extend(
            float(target_by_joint[name]) - position
            for name, position in zip(ARM_JOINT_NAMES, positions)
        )
        max_velocity = max(max_velocity, *(abs(value) for value in velocities))
        max_force = max(max_force, *(abs(value) for value in forces))
    final_positions = sim.arm_positions()
    final_errors = [
        float(target) - position for target, position in zip(targets, final_positions)
    ]
    return {
        "contract": dict(contract),
        "final_positions": final_positions,
        "final_abs_error": max(abs(value) for value in final_errors),
        "max_abs_error": max(abs(value) for value in errors),
        "max_abs_velocity": max_velocity,
        "max_abs_actuator_force": max_force,
        "final_gripper_width": sim.gripper_width(),
        "simulation_time": sim.sim_time,
    }


def _upstream_common_command(
    robot_path: Path, contract: Mapping[str, Any], duration: float
) -> dict[str, Any]:
    target_payload = json.dumps(dict(contract), separators=(",", ":"))
    script = f"""
import json
from rebotarm_simulation.mujoco_sim import RebotArmMujoco

contract = json.loads({target_payload!r})
with RebotArmMujoco({str(robot_path.resolve())!r}) as sim:
    sim.set_joint_position_targets(contract['targets'])
    sim.set_gripper_width(contract['gripper_width'])
    errors = []
    max_velocity = 0.0
    max_force = 0.0
    steps = max(1, int(float({duration}) / sim.timestep))
    for _ in range(steps):
        state = sim.step(1)
        positions = state.joint_positions[:6]
        velocities = state.joint_velocities[:6]
        forces = state.actuator_forces[:6]
        errors.extend(target - position for target, position in zip(contract['targets'], positions))
        max_velocity = max(max_velocity, *(abs(value) for value in velocities))
        max_force = max(max_force, *(abs(value) for value in forces))
    final_positions = list(state.joint_positions[:6])
    final_errors = [target - position for target, position in zip(contract['targets'], final_positions)]
    print(json.dumps({{
        'contract': contract,
        'final_positions': final_positions,
        'final_abs_error': max(abs(value) for value in final_errors),
        'max_abs_error': max(abs(value) for value in errors),
        'max_abs_velocity': max_velocity,
        'max_abs_actuator_force': max_force,
        'final_gripper_width': state.gripper_width,
        'simulation_time': state.simulation_time,
    }}, separators=(',', ':')))
"""
    return _run_upstream_json(["-c", script])


def _upstream_grasp_smoke(scene_path: Path, duration: float) -> dict[str, Any]:
    script = f"""
import json
from rebotarm_simulation.mujoco_sim import RebotArmMujoco

with RebotArmMujoco({str(scene_path.resolve())!r}) as sim:
    initial = sim.reset_home()
    sim.set_gripper_width(0.04)
    initial_z = initial.object_poses.get('test_cube', (None, None, None))[2]
    max_contacts = 0
    steps = max(1, int(float({duration}) / sim.timestep))
    for _ in range(steps):
        state = sim.step(1)
        max_contacts = max(max_contacts, len(sim.get_contacts()))
    final_z = state.object_poses.get('test_cube', (None, None, None))[2]
    print(json.dumps({{
        'finite': all(abs(value) < float('inf') for value in state.joint_positions),
        'initial_box_height_m': initial_z,
        'box_height_m': final_z,
        'max_contacts': max_contacts,
        'final_contacts': len(sim.get_contacts()),
        'gripper_width': state.gripper_width,
        'simulation_time': state.simulation_time,
    }}, separators=(',', ':')))
"""
    return _run_upstream_json(["-c", script])


def build_comparison(duration: float) -> dict[str, Any]:
    current_robot, current_scene = _prepare_current_models(ROOT / "build/upstream_comparison")
    contract = common_command_contract(default_step_response_targets(MOTOR_PROFILES))
    current_step = run_step_response_suite(
        current_robot,
        targets=default_step_response_targets(MOTOR_PROFILES),
        seconds=duration,
    )
    current = {
        "backend": "current",
        "model": normalize_model(current_robot),
        "health": _as_json(check_model_health(current_robot, steps=2)),
        "smoke": _as_json(run_smoke(current_robot, seconds=duration)),
        "step_response": _as_json(current_step),
        "common_command": _as_json(_current_common_command(current_robot, contract, duration)),
        "grasp": _as_json(run_grasp_benchmark(current_scene, seconds=duration)),
        "tests": {"status": "not_run_by_harness", "command": "python3 -m pytest tests -q"},
    }

    snapshot_root = ROOT / "third_party/robotarm_ros2_mujoco_snapshot"
    upstream_scene = snapshot_root / "src/rebotarm_simulation/models/rebotarm/scene.xml"
    upstream_robot = snapshot_root / "src/rebotarm_simulation/models/rebotarm/robot.xml"
    health_process = run_upstream(
        ROOT,
        [
            "-m",
            "rebotarm_simulation.mujoco_health",
            "--model",
            str(upstream_scene.resolve()),
            "--skip-renderer",
        ],
    )
    headless_process = run_upstream(
        ROOT,
        ["-m", "rebotarm_simulation.mujoco_cli", "--headless", "--duration", str(duration)],
    )
    test_process = run_upstream(
        ROOT,
        ["-m", "pytest", *sorted(str(path.relative_to(snapshot_root)) for path in (snapshot_root / "tests").glob("test_mujoco_*.py")), "tests/test_urdf_to_mjcf.py", "-q"],
    )
    upstream = {
        "backend": "upstream",
        "model": normalize_model(upstream_robot),
        "scene_model": normalize_model(upstream_scene),
        "health": {
            "result": _last_json_line(health_process.stdout),
            "returncode": health_process.returncode,
            "stderr": health_process.stderr,
        },
        "headless": {
            "result": _last_json_line(headless_process.stdout),
            "returncode": headless_process.returncode,
            "stderr": headless_process.stderr,
        },
        "step_response": _upstream_step_response(upstream_robot, duration),
        "common_command": _upstream_common_command(upstream_robot, contract, duration),
        "grasp": _upstream_grasp_smoke(upstream_scene, duration),
        "tests": {
            **_test_summary(test_process.stdout + test_process.stderr, test_process.returncode),
            "stdout_tail": (test_process.stdout + test_process.stderr)[-4000:],
        },
    }
    return {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": float(duration),
        "current": current,
        "upstream": upstream,
        "comparison": compare_records(current, upstream),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare current and vendored upstream MuJoCo baselines")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    report = build_comparison(args.duration)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"comparison_report={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

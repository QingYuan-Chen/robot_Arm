from __future__ import annotations

import argparse
import json
from pathlib import Path

from .mujoco_adapter_core import default_step_response_targets
from .mujoco_model_profile import (
    DEFAULT_GRIPPER_XML,
    DEFAULT_GRASP_SCENE_XML,
    MOTOR_PROFILES,
    write_grasp_scene_profile,
    write_physics_profile,
)
from .mujoco_runner import run_grasp_benchmark, run_smoke, run_step_response, run_step_response_suite


DEFAULT_OUTPUT_DIR = Path("build") / "mujoco_models"
DEFAULT_ROBOT_OUTPUT = DEFAULT_OUTPUT_DIR / "reBot-DevArm_gripper_physics.xml"
DEFAULT_SCENE_OUTPUT = DEFAULT_OUTPUT_DIR / "sim_reBot_grasp_physics.xml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="reBotArm MuJoCo simulation utilities")
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate_parser = subparsers.add_parser("generate", help="Generate optimized MuJoCo XML profiles")
    generate_parser.add_argument("--source", type=Path, default=DEFAULT_GRIPPER_XML)
    generate_parser.add_argument("--scene-source", type=Path, default=DEFAULT_GRASP_SCENE_XML)
    generate_parser.add_argument("--robot-output", type=Path, default=DEFAULT_ROBOT_OUTPUT)
    generate_parser.add_argument("--scene-output", type=Path, default=DEFAULT_SCENE_OUTPUT)

    smoke_parser = subparsers.add_parser("smoke", help="Run a headless stability smoke test")
    smoke_parser.add_argument("--xml", type=Path, default=DEFAULT_ROBOT_OUTPUT)
    smoke_parser.add_argument("--seconds", type=float, default=3.0)

    step_parser = subparsers.add_parser("step-response", help="Run a single-joint actuator response test")
    step_parser.add_argument("--xml", type=Path, default=DEFAULT_ROBOT_OUTPUT)
    step_parser.add_argument("--joint", default="joint2")
    step_parser.add_argument("--target", type=float, default=-0.6)
    step_parser.add_argument("--seconds", type=float, default=3.0)

    step_suite_parser = subparsers.add_parser("step-response-suite", help="Run actuator response tests for all arm joints")
    step_suite_parser.add_argument("--xml", type=Path, default=DEFAULT_ROBOT_OUTPUT)
    step_suite_parser.add_argument("--seconds", type=float, default=3.0)
    step_suite_parser.add_argument("--json-output", type=Path)

    grasp_parser = subparsers.add_parser("grasp-benchmark", help="Run the basic grasp scene benchmark")
    grasp_parser.add_argument("--xml", type=Path, default=DEFAULT_SCENE_OUTPUT)
    grasp_parser.add_argument("--seconds", type=float, default=5.0)

    args = parser.parse_args(argv)
    if args.command == "generate":
        robot_xml = write_physics_profile(args.robot_output, args.source)
        scene_robot_xml = args.scene_output.with_name(f"{args.scene_output.stem}_robot_include.xml")
        write_physics_profile(scene_robot_xml, args.source, include_keyframes=False)
        scene_xml = write_grasp_scene_profile(args.scene_output, scene_robot_xml, args.scene_source)
        print(f"robot_xml={robot_xml}")
        print(f"scene_robot_xml={scene_robot_xml.resolve()}")
        print(f"scene_xml={scene_xml}")
        return 0
    if args.command == "smoke":
        result = run_smoke(args.xml, seconds=args.seconds)
        print(
            f"xml={result.xml_path} nq={result.nq} nv={result.nv} nu={result.nu} "
            f"finite={result.finite} contacts={result.contacts} sim_time={result.sim_time:.3f}"
        )
        return 0 if result.finite else 1
    if args.command == "step-response":
        result = run_step_response(args.xml, joint=args.joint, target=args.target, seconds=args.seconds)
        print(
            f"joint={result.joint} target={result.target:.4f} final={result.final_position:.4f} "
            f"final_abs_error={result.final_abs_error:.4f} "
            f"max_abs_error={result.max_abs_error:.4f} rms_error={result.rms_error:.4f} "
            f"max_abs_velocity={result.max_abs_velocity:.4f} "
            f"max_abs_actuator_force={result.max_abs_actuator_force:.4f} "
            f"sim_time={result.sim_time:.3f}"
        )
        return 0
    if args.command == "step-response-suite":
        targets = default_step_response_targets(MOTOR_PROFILES)
        result = run_step_response_suite(args.xml, targets=targets, seconds=args.seconds)
        payload = {
            "xml_path": str(result.xml_path.resolve()),
            "joint_count": len(result.results),
            "max_final_abs_error": result.max_final_abs_error,
            "max_abs_error": result.max_abs_error,
            "max_rms_error": result.max_rms_error,
            "max_abs_velocity": result.max_abs_velocity,
            "max_abs_actuator_force": result.max_abs_actuator_force,
            "results": [item.__dict__ for item in result.results],
        }
        if args.json_output is not None:
            args.json_output.parent.mkdir(parents=True, exist_ok=True)
            args.json_output.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        print(
            f"xml={result.xml_path.resolve()} joint_count={len(result.results)} "
            f"max_final_abs_error={result.max_final_abs_error:.4f} "
            f"max_abs_error={result.max_abs_error:.4f} max_rms_error={result.max_rms_error:.4f} "
            f"max_abs_velocity={result.max_abs_velocity:.4f} "
            f"max_abs_actuator_force={result.max_abs_actuator_force:.4f}"
        )
        return 0
    if args.command == "grasp-benchmark":
        result = run_grasp_benchmark(args.xml, seconds=args.seconds)
        print(
            f"xml={result.xml_path} finite={result.finite} box_height_m={result.box_height_m} "
            f"max_contacts={result.max_contacts} final_contacts={result.final_contacts} "
            f"grasp_success={result.grasp_success} grasp_status={result.grasp_status} "
            f"sim_time={result.sim_time:.3f}"
        )
        return 0 if result.finite else 1
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

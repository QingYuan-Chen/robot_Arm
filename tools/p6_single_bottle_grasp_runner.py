#!/usr/bin/env python3
"""Run guarded, repeatable real bottle grasp-return tests through ROS 2.

Each trial requires a newly received filtered bottle plan, plans both poses
against live feedback, and executes only:
pregrasp -> approach -> close -> hold -> return-to-baseline.

It intentionally excludes lift, retreat, safe-home, and grasp-outcome
classification because the installed gripper has no force-feedback evidence.
The script calls ROS services/actions only; it never imports the motor SDK.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rebotarm_msgs.msg import GraspPlan
from rebotarm_msgs.srv import ExecutePose, SetGripper


ROOT = Path(__file__).resolve().parents[1]
for source in (ROOT / "tools", ROOT / "src" / "rebotarm_motion"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from p5_paired_trajectory_runner import PairedTrajectoryNode
from rebotarm_motion.paired_trajectory_protocol import build_quintic_command
from rebotarm_motion.real_failure_recovery import recover_real_failure


REAL_CONFIRMATION = "REAL_SINGLE_BOTTLE_GRASP"
ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
MIN_CANDIDATE_CONFIDENCE = 0.4


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--plan-timeout-sec", type=float, default=45.0)
    parser.add_argument("--max-plan-age-sec", type=float, default=1.5)
    parser.add_argument("--pregrasp-sec", type=float, default=45.0)
    parser.add_argument("--approach-sec", type=float, default=45.0)
    parser.add_argument("--hold-sec", type=float, default=20.0)
    parser.add_argument("--return-sec", type=float, default=45.0)
    parser.add_argument("--max-pregrasp-delta-rad", type=float, default=2.5)
    parser.add_argument("--max-approach-delta-rad", type=float, default=1.2)
    parser.add_argument(
        "--gripper-open-m",
        type=float,
        required=True,
        help="mechanically verified safe opening in metres; no implicit full-open target",
    )
    parser.add_argument(
        "--gripper-open-max-effort",
        type=float,
        default=0.15,
        help="motor-side torque cap for the opening move (must be 0.05..0.15 N.m)",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm", default="")
    return parser.parse_args()


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _pose_payload(pose) -> dict[str, object]:
    return {
        "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
        "orientation_xyzw": [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ],
    }


def _valid_bottle_plan(message: GraspPlan) -> bool:
    candidate = message.candidate
    return bool(
        message.valid
        and str(candidate.class_name) == "bottle"
        and str(message.header.frame_id) == "base_link"
        and float(candidate.confidence) >= MIN_CANDIDATE_CONFIDENCE
        and 0.0 < float(message.jaw_width) <= 0.085
    )


def _request_plan(
    node: PairedTrajectoryNode,
    client,
    pose,
    label: str,
) -> tuple[dict[str, object], tuple[float, ...]]:
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"{label} planning service unavailable")
    request = ExecutePose.Request()
    request.target_pose = PoseStamped()
    request.target_pose.header.frame_id = "base_link"
    request.target_pose.pose = pose
    request.velocity_scaling = 0.10
    request.acceleration_scaling = 0.10
    request.timeout_sec = 25.0
    request.execute = False
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result() if future.done() else None
    if response is None or not response.success or len(response.planned_trajectory.points) < 2:
        message = "no response" if response is None else f"{response.stage}: {response.message}"
        raise RuntimeError(f"{label} plan failed: {message}")
    names = list(response.planned_trajectory.joint_names)
    by_name = {name: index for index, name in enumerate(names)}
    if any(name not in by_name for name in ARM_JOINT_NAMES):
        raise RuntimeError(f"{label} plan does not contain canonical arm joints")
    final = response.planned_trajectory.points[-1]
    target = tuple(float(final.positions[by_name[name]]) for name in ARM_JOINT_NAMES)
    return {
        "points": len(response.planned_trajectory.points),
        "stage": str(response.stage),
        "message": str(response.message),
        "target": list(target),
    }, target


def _call_gripper(
    node: PairedTrajectoryNode,
    client,
    position: float,
    label: str,
    *,
    max_effort: float = 0.15,
) -> dict[str, object]:
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"{label} gripper service unavailable")
    request = SetGripper.Request()
    request.position = float(position)
    request.max_effort = float(max_effort)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result() if future.done() else None
    if response is None:
        raise RuntimeError(f"{label} gripper service timed out")
    result = {
        "requested_position_m": float(position),
        "reached_position_m": float(response.reached_position),
        "max_effort": float(max_effort),
        "success": bool(response.success),
    }
    if not result["success"]:
        raise RuntimeError(f"{label} gripper command failed: {result}")
    return result


def _wait_for_fresh_bottle_plan(
    node: PairedTrajectoryNode,
    latest: dict[str, object],
    *,
    timeout_sec: float,
    max_age_sec: float,
) -> GraspPlan:
    latest["message"] = None
    latest["received_monotonic"] = None
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        message = latest.get("message")
        received = latest.get("received_monotonic")
        if isinstance(message, GraspPlan) and isinstance(received, float):
            age = time.monotonic() - received
            if age <= max_age_sec:
                return message
    raise RuntimeError("fresh valid bottle plan unavailable")


def _healthy_enabled(node: PairedTrajectoryNode) -> bool:
    status = node.latest_status
    return bool(
        status is not None
        and status.enabled
        and status.control_loop_active
        and list(status.per_joint_status_code) == [1] * 6
        and not list(status.error_codes)
        and node.last_joint_monotonic is not None
        and time.monotonic() - node.last_joint_monotonic <= 0.5
    )


def _run_one(
    node: PairedTrajectoryNode,
    latest_plan: dict[str, object],
    pose_client,
    gripper_client,
    args: argparse.Namespace,
    index: int,
) -> dict[str, object]:
    trial: dict[str, object] = {"trial": index, "legs": [], "services": [], "success": False}
    baseline: tuple[float, ...] | None = None
    enabled = False
    allow_controlled_return = False
    try:
        node.wait_for_preflight()
        baseline = node.canonical_positions()
        trial["baseline_positions"] = list(baseline)
        trial["preflight_status"] = node._status_payload()
        plan = _wait_for_fresh_bottle_plan(
            node,
            latest_plan,
            timeout_sec=args.plan_timeout_sec,
            max_age_sec=args.max_plan_age_sec,
        )
        received = float(latest_plan["received_monotonic"])
        trial["plan"] = {
            "age_sec": time.monotonic() - received,
            "class_name": str(plan.candidate.class_name),
            "confidence": float(plan.candidate.confidence),
            "jaw_width_m": float(plan.jaw_width),
            "source": str(plan.source),
            "pregrasp": _pose_payload(plan.pregrasp_pose),
            "grasp": _pose_payload(plan.grasp_pose),
        }
        pre_plan, pre_target = _request_plan(node, pose_client, plan.pregrasp_pose, "pregrasp")
        grasp_plan, grasp_target = _request_plan(node, pose_client, plan.grasp_pose, "approach")
        pre_delta = max(abs(target - actual) for target, actual in zip(pre_target, baseline))
        approach_delta = max(abs(target - actual) for target, actual in zip(grasp_target, pre_target))
        trial["plan_only"] = {
            "pregrasp": pre_plan,
            "approach": grasp_plan,
            "pregrasp_max_delta_rad": pre_delta,
            "approach_max_delta_rad": approach_delta,
        }
        if pre_delta > args.max_pregrasp_delta_rad or approach_delta > args.max_approach_delta_rad:
            raise RuntimeError(
                "joint-delta gate rejected route: "
                f"pregrasp={pre_delta:.6f}, approach={approach_delta:.6f}"
            )

        trial["services"].append(node.call_trigger(node.enable_client, "enable"))
        enabled = True
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        if not _healthy_enabled(node):
            raise RuntimeError("controller unhealthy after enable")

        opened = _call_gripper(
            node,
            gripper_client,
            args.gripper_open_m,
            "open",
            max_effort=args.gripper_open_max_effort,
        )
        trial["gripper_open"] = opened
        if float(opened["reached_position_m"]) < args.gripper_open_m - 0.003:
            raise RuntimeError(f"safe gripper open failed: {opened}")

        pre_command = build_quintic_command(
            baseline, pre_target, duration_sec=args.pregrasp_sec, cadence_sec=0.05,
            label=f"single_bottle_trial_{index}_pregrasp_{args.pregrasp_sec:g}s",
        )
        pre_leg = node.execute_leg(pre_command)
        trial["legs"].append(pre_leg)
        if not pre_leg["success"]:
            raise RuntimeError(f"pregrasp failed: {pre_leg['result']}")
        node.hold_and_collect(0.5)
        pre_actual = node.canonical_positions()
        if max(abs(actual - target) for actual, target in zip(pre_actual, pre_target)) > 0.02:
            raise RuntimeError("pregrasp endpoint error exceeds 0.02 rad")

        approach_command = build_quintic_command(
            pre_actual, grasp_target, duration_sec=args.approach_sec, cadence_sec=0.05,
            label=f"single_bottle_trial_{index}_approach_{args.approach_sec:g}s",
        )
        approach_leg = node.execute_leg(approach_command)
        trial["legs"].append(approach_leg)
        if not approach_leg["success"]:
            raise RuntimeError(f"approach failed: {approach_leg['result']}")
        node.hold_and_collect(0.5)
        grasp_actual = node.canonical_positions()
        if max(abs(actual - target) for actual, target in zip(grasp_actual, grasp_target)) > 0.02:
            raise RuntimeError("approach endpoint error exceeds 0.02 rad")

        close_position = min(0.08, max(0.006, float(plan.jaw_width) - 0.012))
        trial["gripper_close"] = _call_gripper(node, gripper_client, close_position, "close")
        hold_deadline = time.monotonic() + args.hold_sec
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not _healthy_enabled(node):
                raise RuntimeError("controller or joint feedback unhealthy during hold")

        return_start = node.canonical_positions()
        return_command = build_quintic_command(
            return_start, baseline, duration_sec=args.return_sec, cadence_sec=0.05,
            label=f"single_bottle_trial_{index}_return_baseline_{args.return_sec:g}s",
        )
        return_leg = node.execute_leg(return_command)
        trial["legs"].append(return_leg)
        if not return_leg["success"]:
            raise RuntimeError(f"return failed: {return_leg['result']}")
        node.hold_and_collect(0.5)
        final = node.canonical_positions()
        errors = tuple(target - actual for target, actual in zip(baseline, final))
        trial["enabled_final_errors_rad"] = list(errors)
        if max(abs(value) for value in errors) > 0.02:
            raise RuntimeError(f"baseline return error exceeds 0.02 rad: {errors}")
        trial["services"].append(node.call_trigger(node.disable_client, "disable"))
        enabled = False
        node.hold_and_collect(0.5)
        trial["final_status"] = node._status_payload()
        trial["success"] = True
    except Exception as exc:
        trial["failure"] = f"{type(exc).__name__}: {exc}"
        if enabled and baseline is not None:
            try:
                enabled = recover_real_failure(
                    node=node,
                    report=trial,
                    baseline=baseline,
                    allow_controlled_return=allow_controlled_return,
                    legs_key="legs",
                    command_label=f"single_bottle_trial_{index}_failure_return_baseline",
                    duration_sec=args.return_sec,
                )
            except Exception as recovery_exc:
                trial["recovery_failure"] = f"{type(recovery_exc).__name__}: {recovery_exc}"
        trial["final_status"] = node._status_payload()
    return trial


def main() -> None:
    args = _parse_args()
    if args.confirm != REAL_CONFIRMATION:
        raise SystemExit(f"real run requires --confirm {REAL_CONFIRMATION}")
    if args.runs < 1:
        raise SystemExit("--runs must be at least 1")
    if min(args.pregrasp_sec, args.approach_sec, args.hold_sec, args.return_sec) <= 0.0:
        raise SystemExit("all durations must be positive")
    if not 0.0 < args.gripper_open_m <= 0.085:
        raise SystemExit("--gripper-open-m must be within (0, 0.085] metres")
    if not 0.05 <= args.gripper_open_max_effort <= 0.15:
        raise SystemExit("--gripper-open-max-effort must be within [0.05, 0.15] N.m")
    report: dict[str, object] = {
        "schema_version": 1,
        "backend": "real",
        "authorized_scope": "fresh bottle plan; 45s pregrasp, 45s approach, close, 20s hold, 45s return; no lift or retreat",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "trials": [],
    }
    node: PairedTrajectoryNode | None = None
    try:
        rclpy.init()
        node = PairedTrajectoryNode(namespace=args.namespace, backend="real")
        latest_plan: dict[str, object] = {}

        def plan_callback(message: GraspPlan) -> None:
            if _valid_bottle_plan(message):
                latest_plan["message"] = message
                latest_plan["received_monotonic"] = time.monotonic()

        node.create_subscription(GraspPlan, "/grasp/filtered_plan", plan_callback, 10)
        pose_client = node.create_client(ExecutePose, f"/{node.namespace}/motion_execution/execute_pose")
        gripper_client = node.create_client(SetGripper, f"/{node.namespace}/gripper/set")
        for index in range(1, args.runs + 1):
            trial = _run_one(node, latest_plan, pose_client, gripper_client, args, index)
            report["trials"].append(trial)
            if not trial["success"]:
                break
        report["success"] = bool(report["trials"]) and all(trial["success"] for trial in report["trials"])
    finally:
        report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        _write_json(args.output, report)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

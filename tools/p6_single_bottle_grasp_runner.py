#!/usr/bin/env python3
"""Run repeatable real bottle grasp-return tests through ROS 2.

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
from rebotarm_msgs.srv import ExecutePose, GraspGripper, SetGripper


ROOT = Path(__file__).resolve().parents[1]
for source in (ROOT / "tools", ROOT / "src" / "rebotarm_motion"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from p5_paired_trajectory_runner import PairedTrajectoryNode
from rebotarm_motion.paired_trajectory_protocol import (
    build_quintic_command,
    build_retimed_path_command,
)
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
    parser.add_argument(
        "--gripper-open-m",
        type=float,
        required=True,
        help="mechanically verified safe opening in metres; no implicit full-open target",
    )
    parser.add_argument(
        "--gripper-open-max-effort",
        type=float,
        default=1.0,
        help="motor-side torque cap for the opening move (must be 0.05..1.5 N.m)",
    )
    # Closing onto the object uses force control (GraspGripper), not the
    # position service: a position move neutralizes on arrival and therefore
    # produces no sustained grip.  These are tiered so the operator can start
    # low and step up on real hardware.
    parser.add_argument(
        "--grasp-close-force",
        type=float,
        default=0.4,
        help="motor-side closing torque for the force-controlled close (0.05..1.0 N.m)",
    )
    parser.add_argument(
        "--grasp-hold-force",
        type=float,
        default=0.4,
        # NOT the main grip-force knob.  grasp_holding commands
        # kp=5.0 * (stall_angle - current) + kd*... + hold_force, so the position
        # term dominates and grip force is set by how deep close_force drove the
        # jaws before stalling.  2026-08-14: identical hold_force=0.4 produced
        # 0.271-0.286 N.m at a 2.02 mm bite and 0.423-0.437 N.m at a 15.21 mm
        # bite -- a 1.5x spread from close_force alone.
        help=(
            "feed-forward holding torque after stall (0.05..1.5 N.m); NOT the "
            "primary grip-force control -- grip is dominated by the stall depth "
            "that --grasp-close-force produces"
        ),
    )
    parser.add_argument(
        "--grasp-close-timeout-sec",
        type=float,
        default=4.0,
        # 2026-08-14 real hardware: closing runs at ~9.51 mm/s under 0.4 N.m, so
        # the ~30 mm from an 80 mm open position to a 50 mm contact point needs
        # ~3.15 s before a stall can even begin.  The former 2.0 s default
        # expired mid-travel and was misread as insufficient closing torque.
        help=(
            "give up closing if no velocity stall is detected within this time; "
            "must cover the full free travel to the object, not just contact"
        ),
    )
    parser.add_argument(
        "--controller-grasp-hold-timeout-sec",
        type=float,
        default=30.0,
        help=(
            "the controller's grasp_hold_timeout_sec; must exceed --hold-sec or "
            "the grip would be released mid-hold"
        ),
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
    # Keep the whole planned path, not just its endpoint: the intermediate
    # waypoints carry MoveIt's obstacle avoidance and IK branch.  Executing only
    # the endpoint made the end effector sweep an unvalidated Cartesian path.
    waypoints = [
        tuple(float(point.positions[by_name[name]]) for name in ARM_JOINT_NAMES)
        for point in response.planned_trajectory.points
    ]
    target = waypoints[-1]
    return {
        "points": len(response.planned_trajectory.points),
        "stage": str(response.stage),
        "message": str(response.message),
        "target": list(target),
        "waypoints": len(waypoints),
    }, waypoints


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


def _call_grasp(
    node: PairedTrajectoryNode,
    client,
    args,
) -> dict[str, object]:
    """Force-controlled close.

    There is no force sensor on this gripper, so ``contact_detected`` means
    "closure travel reached and velocity stalled", i.e. stall detection.  A
    stall cannot distinguish the object from a mechanical stop or a jam, so
    every field here is recorded as an observation and never used to classify
    the grasp as successful.
    """
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("grasp gripper service unavailable")
    request = GraspGripper.Request()
    request.close_force = float(args.grasp_close_force)
    request.hold_force = float(args.grasp_hold_force)
    request.close_timeout_sec = float(args.grasp_close_timeout_sec)
    request.min_close_time_sec = 0.08
    request.velocity_threshold = 0.04
    request.min_closure_distance_m = 0.006
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result() if future.done() else None
    if response is None:
        raise RuntimeError("grasp gripper service timed out")
    return {
        "close_force": float(request.close_force),
        "hold_force": float(response.hold_force),
        "close_timeout_sec": float(request.close_timeout_sec),
        "success": bool(response.success),
        "stall_detected": bool(response.contact_detected),
        "stall_position_m": float(response.contact_position),
        "reached_position_m": float(response.reached_position),
        "message": str(response.message),
        "interpretation": (
            "stall detection only; no force sensor, so this does not classify "
            "grip, slip or pickup"
        ),
    }


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
    grasp_client,
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
            "min_candidate_confidence": MIN_CANDIDATE_CONFIDENCE,
            "jaw_width_m": float(plan.jaw_width),
            "source": str(plan.source),
            "pregrasp": _pose_payload(plan.pregrasp_pose),
            "grasp": _pose_payload(plan.grasp_pose),
        }
        pre_plan, pre_path = _request_plan(node, pose_client, plan.pregrasp_pose, "pregrasp")
        pre_target = pre_path[-1]
        pre_delta = max(abs(target - actual) for target, actual in zip(pre_target, baseline))
        # The approach path is planned later, from the arm's actual pregrasp
        # position -- see the note at that call site.  Pre-screen the reachable
        # grasp joint target here so an out-of-range route is rejected before
        # anything moves; this preview plan is not executed.
        grasp_preview, grasp_preview_path = _request_plan(
            node, pose_client, plan.grasp_pose, "approach-preview"
        )
        approach_delta = max(
            abs(target - actual)
            for target, actual in zip(grasp_preview_path[-1], pre_target)
        )
        trial["plan_only"] = {
            "pregrasp": pre_plan,
            "approach_preview": grasp_preview,
            "pregrasp_observed_max_delta_rad": pre_delta,
            "approach_preview_observed_max_delta_rad": approach_delta,
        }
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

        # Follow MoveIt's planned path, re-timed onto a slow quintic profile.
        # Prepend the live baseline so the path starts from where the arm is.
        pre_command = build_retimed_path_command(
            [baseline, *pre_path], duration_sec=args.pregrasp_sec, cadence_sec=0.05,
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

        # Plan the approach only now that the arm is physically at pregrasp.
        # MoveIt plans from current feedback, so a plan requested before the
        # pregrasp leg starts at the baseline instead -- executing that path
        # walked pregrasp -> baseline -> grasp (4.5950 rad of arc where the real
        # move is 0.4438 rad), so execution now starts from true feedback.
        # This re-plan does NOT need a fresh perception frame: the Cartesian
        # grasp pose still comes from the same accepted plan, only the joint-space
        # route to it is recomputed from the true start state.
        grasp_plan, grasp_path = _request_plan(
            node, pose_client, plan.grasp_pose, "approach"
        )
        trial["plan_only"]["approach"] = grasp_plan
        executed_approach_delta = max(
            abs(target - actual) for target, actual in zip(grasp_path[-1], pre_actual)
        )
        trial["plan_only"]["approach_executed_observed_max_delta_rad"] = executed_approach_delta
        grasp_target = grasp_path[-1]
        approach_command = build_retimed_path_command(
            [pre_actual, *grasp_path], duration_sec=args.approach_sec, cadence_sec=0.05,
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

        # Force-controlled close.  A SetGripper position move neutralizes on
        # arrival and so cannot hold the object; only this path sustains torque.
        closed = _call_grasp(node, grasp_client, args)
        trial["gripper_close"] = closed
        # Fail closed rather than holding a 20 s "grip" that never engaged.
        # stall_detected is not proof the bottle is held, but its absence IS
        # proof the close did not complete, so continuing is meaningless.
        if not closed["success"] or not closed["stall_detected"]:
            raise RuntimeError(f"force-controlled close did not stall: {closed}")
        hold_deadline = time.monotonic() + args.hold_sec
        hold_samples: list[dict[str, float]] = []
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not _healthy_enabled(node):
                raise RuntimeError("controller or joint feedback unhealthy during hold")
            gripper = node._gripper_payload()
            if gripper.get("available"):
                hold_samples.append(gripper)
        # Observations only: jaw drift over the hold and the reported torque
        # magnitude. Neither is a calibrated grip measurement.
        trial["hold_observations"] = {
            "samples": len(hold_samples),
            "position_first_m": (
                float(hold_samples[0]["position"]) if hold_samples else None
            ),
            "position_last_m": (
                float(hold_samples[-1]["position"]) if hold_samples else None
            ),
            "torque_min_nm": min(
                (float(s["torque"]) for s in hold_samples), default=None
            ),
            "torque_max_nm": max(
                (float(s["torque"]) for s in hold_samples), default=None
            ),
        }

        # Explicitly stop gripping before moving the arm.  Relying on the
        # controller's hold timeout would either keep loading the motor through
        # the whole 45 s return or expire at an unpredictable moment mid-travel.
        # Opening to the verified safe width both releases the object and leaves
        # the gripper neutral+idle on arrival.
        released = _call_gripper(
            node,
            gripper_client,
            args.gripper_open_m,
            "release",
            max_effort=args.gripper_open_max_effort,
        )
        trial["gripper_release"] = released
        # Do not start a 45 s return while still gripping the bottle.
        if float(released["reached_position_m"]) < args.gripper_open_m - 0.006:
            raise RuntimeError(f"release before return did not open: {released}")

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
    if not 0.05 <= args.gripper_open_max_effort <= 1.5:
        raise SystemExit("--gripper-open-max-effort must be within [0.05, 1.5] N.m")
    if not 0.05 <= args.grasp_close_force <= 1.0:
        raise SystemExit("--grasp-close-force must be within [0.05, 1.0] N.m")
    if not 0.05 <= args.grasp_hold_force <= 1.5:
        raise SystemExit("--grasp-hold-force must be within [0.05, 1.5] N.m")
    if not 0.1 <= args.grasp_close_timeout_sec <= 10.0:
        raise SystemExit("--grasp-close-timeout-sec must be within [0.1, 10.0] s")
    if not 0.1 <= args.controller_grasp_hold_timeout_sec <= 120.0:
        raise SystemExit(
            "--controller-grasp-hold-timeout-sec must be within [0.1, 120.0] s"
        )
    # The controller bounds grasp_holding independently.  If this runner's hold
    # window outlived that bound the grip would release mid-hold and the JSON
    # would still look like a clean hold, so refuse the combination up front.
    if args.hold_sec >= args.controller_grasp_hold_timeout_sec:
        raise SystemExit(
            f"--hold-sec ({args.hold_sec:g} s) must be shorter than the controller's "
            f"grasp_hold_timeout_sec ({args.controller_grasp_hold_timeout_sec:g} s); "
            "raise the controller parameter or shorten the hold"
        )
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
        node = PairedTrajectoryNode(
            namespace=args.namespace,
            backend="real",
        )
        latest_plan: dict[str, object] = {}

        def plan_callback(message: GraspPlan) -> None:
            if _valid_bottle_plan(message):
                latest_plan["message"] = message
                latest_plan["received_monotonic"] = time.monotonic()

        node.create_subscription(GraspPlan, "/grasp/filtered_plan", plan_callback, 10)
        pose_client = node.create_client(ExecutePose, f"/{node.namespace}/motion_execution/execute_pose")
        gripper_client = node.create_client(SetGripper, f"/{node.namespace}/gripper/set")
        grasp_client = node.create_client(
            GraspGripper, f"/{node.namespace}/gripper/grasp"
        )
        for index in range(1, args.runs + 1):
            trial = _run_one(
                node, latest_plan, pose_client, gripper_client, grasp_client, args, index
            )
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

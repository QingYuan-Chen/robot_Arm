#!/usr/bin/env python3
"""Collect one replacement pose and complete the interrupted P5 hand-eye residual."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy


ROOT = Path(__file__).resolve().parents[1]
for source in (
    ROOT / "src" / "rebotarm_calibration",
    ROOT / "src" / "rebotarm_motion",
    ROOT / "src" / "rebotarm_simulation",
    ROOT / "src" / "rebotarm_vision",
    ROOT / "third_party" / "reBotArm_control_py",
    ROOT / "tools",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from p5_handeye_residual_runner import (
    REFERENCE_BASELINE,
    _camera,
    _capture_pose,
    _load_handeye,
    _sdk_extrinsic,
    _write_json,
)
from p5_paired_trajectory_runner import PairedTrajectoryNode
from rebotarm_calibration.handeye_residual import (
    analyze_handeye_residual,
    matrix_transform,
)
from rebotarm_motion.paired_trajectory_protocol import build_quintic_command
from rebotarm_motion.real_failure_recovery import recover_real_failure
from reBotArm_control_py.kinematics import load_robot_model


CONFIRMATION = "REAL_HAND_EYE_REPLACEMENT_POSE"
CANDIDATE = np.array([-1.50, -0.12, -0.22, 0.12, -0.08, 0.28], dtype=np.float64)
VALIDATED_DISABLED_START = np.array(
    [-1.556229591369629, 0.00476837158203125, -0.01239776611328125,
     0.01392364501953125, 0.058938026428222656, 0.00743865966796875],
    dtype=np.float64,
)
REQUIRED_PRIOR_LABELS = ("baseline", "pose_1", "pose_2", "safe", "pose_4")


def _prior_aggregates(path: Path) -> list[dict[str, object]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    captures = payload.get("pose_captures")
    if not isinstance(captures, list):
        raise ValueError("prior report has no pose_captures list")
    by_label = {str(item.get("label")): item for item in captures}
    if tuple(label for label in REQUIRED_PRIOR_LABELS if label not in by_label):
        missing = [label for label in REQUIRED_PRIOR_LABELS if label not in by_label]
        raise ValueError(f"prior report is missing captures: {missing}")
    result = []
    for label in REQUIRED_PRIOR_LABELS:
        capture = by_label[label]
        if int(capture.get("accepted", 0)) < 30 or float(capture.get("detection_rate", 0.0)) < 0.90:
            raise ValueError(f"prior capture {label} does not satisfy the detection gate")
        aggregate = capture.get("aggregate")
        if not isinstance(aggregate, dict) or str(aggregate.get("label")) != label:
            raise ValueError(f"prior capture {label} has no valid aggregate")
        result.append(aggregate)
    return result


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--prior-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--handeye-config",
        type=Path,
        default=ROOT / "src" / "rebotarm_vision" / "config" / "handeye.yaml",
    )
    parser.add_argument("--confirm", default="")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.confirm != CONFIRMATION:
        raise SystemExit(f"real run requires --confirm {CONFIRMATION}")
    prior = _prior_aggregates(args.prior_report)
    rclpy.init()
    node = PairedTrajectoryNode(namespace=args.namespace, backend="real")
    camera = _camera()
    model = load_robot_model()
    report: dict[str, object] = {
        "schema_version": 1,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "authorized_scope": "one replacement hand-eye pose and controlled reference-baseline return",
        "candidate": CANDIDATE.tolist(),
        "reference_baseline": REFERENCE_BASELINE.tolist(),
        "prior_report": str(args.prior_report),
        "services": [],
        "motion_legs": [],
        "success": False,
    }
    hardware_enabled = False
    allow_controlled_return = False
    try:
        node.wait_for_preflight()
        live_start = np.asarray(node.canonical_positions(), dtype=np.float64)
        report["live_start"] = live_start.tolist()
        report["preflight_status"] = node._status_payload()
        start_drift = float(np.max(np.abs(live_start - VALIDATED_DISABLED_START)))
        report["validated_start_max_drift_rad"] = start_drift
        if start_drift > 0.02:
            raise RuntimeError(
                f"live disabled start differs from MoveIt-validated start by {start_drift:.6f} rad"
            )
        camera.open()
        camera.warmup(15)
        report["camera_device"] = camera.get_device_info()
        report["camera_info"] = camera.get_camera_info("color")
        report["camera_calibration"] = camera.get_calibration_info()
        report["disabled_marker_preflight"] = _capture_pose(
            node=node,
            camera=camera,
            model=model,
            label="replacement_disabled_preflight",
            target=live_start,
            enabled=False,
        )

        report["services"].append(node.call_trigger(node.enable_client, "enable"))
        hardware_enabled = True
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        outbound = build_quintic_command(
            live_start,
            CANDIDATE,
            duration_sec=20.0,
            cadence_sec=0.05,
            label="handeye_replacement_pose",
        )
        allow_controlled_return = False
        outbound_leg = node.execute_leg(outbound)
        report["motion_legs"].append(outbound_leg)
        if not outbound_leg["success"]:
            raise RuntimeError(f"replacement pose motion failed: {outbound_leg['result']}")
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        candidate_capture = _capture_pose(
            node=node,
            camera=camera,
            model=model,
            label="pose_6_replacement",
            target=CANDIDATE,
            enabled=True,
        )
        report["candidate_capture"] = candidate_capture

        return_command = build_quintic_command(
            CANDIDATE,
            REFERENCE_BASELINE,
            duration_sec=20.0,
            cadence_sec=0.05,
            label="handeye_replacement_return_reference_baseline",
        )
        allow_controlled_return = False
        return_leg = node.execute_leg(return_command)
        report["motion_legs"].append(return_leg)
        if not return_leg["success"]:
            raise RuntimeError(f"reference baseline return failed: {return_leg['result']}")
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        final = np.asarray(node.canonical_positions(), dtype=np.float64)
        errors = REFERENCE_BASELINE - final
        report["enabled_final_positions"] = final.tolist()
        report["enabled_final_errors"] = errors.tolist()
        if float(np.max(np.abs(errors))) > 0.02:
            raise RuntimeError("controlled reference-baseline return error exceeds 0.02 rad")
        report["services"].append(node.call_trigger(node.disable_client, "disable"))
        hardware_enabled = False
        node.hold_and_collect(0.5)
        report["final_status"] = node._status_payload()

        samples = prior + [candidate_capture["aggregate"]]
        end_to_depth = _load_handeye(args.handeye_config)
        depth_to_color = _sdk_extrinsic(report["camera_calibration"], "depth_to_color")
        hypotheses = {
            "legacy_numeric_as_color": end_to_depth,
            "named_depth_plus_factory_depth_to_color": end_to_depth @ depth_to_color,
        }
        report["residual_hypotheses"] = {}
        for name, end_to_detection_camera in hypotheses.items():
            payload = {
                "schema_version": 1,
                "end_to_camera": matrix_transform(end_to_detection_camera),
                "samples": samples,
            }
            report["residual_hypotheses"][name] = {
                "input": payload,
                "analysis": analyze_handeye_residual(payload),
            }
        report["success"] = True
    except Exception as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        if hardware_enabled:
            try:
                hardware_enabled = recover_real_failure(
                    node=node,
                    report=report,
                    baseline=REFERENCE_BASELINE,
                    allow_controlled_return=allow_controlled_return,
                    legs_key="motion_legs",
                    command_label="handeye_replacement_failure_return_reference_baseline",
                )
            except Exception as recovery_exc:
                report["recovery_failure"] = f"{type(recovery_exc).__name__}: {recovery_exc}"
                report["recovery_final_status"] = node._status_payload()
        raise
    finally:
        camera.close()
        report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        _write_json(args.output, report)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

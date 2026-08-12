#!/usr/bin/env python3
"""Collect the reviewed 15-pose P5 hand-eye recalibration dataset."""

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
    _average_transforms,
    _camera,
    _capture_pose,
    _write_json,
)
from p5_paired_trajectory_runner import PairedTrajectoryNode
from rebotarm_calibration.handeye_residual import matrix_transform, transform_matrix
from rebotarm_motion.paired_trajectory_protocol import build_quintic_command
from rebotarm_motion.real_failure_recovery import recover_real_failure
from reBotArm_control_py.kinematics import load_robot_model


CONFIRMATION = "REAL_HAND_EYE_RECALIBRATION_15_POSE"


def _load_plan(path: Path) -> dict[str, object]:
    plan = json.loads(path.read_text(encoding="utf-8"))
    kind = plan.get("kind")
    if int(plan.get("schema_version", -1)) != 1 or kind not in {
        "p5_handeye_recalibration_plan",
        "p5_handeye_recalibration_resume_plan",
        "p5_handeye_recalibration_batch_plan",
    }:
        raise ValueError("unsupported recalibration plan")
    poses = plan.get("poses")
    if not isinstance(poses, list) or not poses:
        raise ValueError("recalibration plan must contain poses")
    roles = [str(item.get("role")) for item in poses]
    if kind == "p5_handeye_recalibration_plan":
        if len(poses) != 15 or (roles.count("training"), roles.count("holdout")) != (12, 3):
            raise ValueError("full recalibration plan must contain 12 training and 3 holdout poses")
    elif kind == "p5_handeye_recalibration_resume_plan":
        accepted_prior = plan.get("accepted_prior_labels")
        prior_roles = plan.get("accepted_prior_role_counts")
        if not isinstance(accepted_prior, list) or not accepted_prior:
            raise ValueError("resume plan must reference accepted prior captures")
        if len(accepted_prior) + len(poses) != 15:
            raise ValueError("resume plan must merge to exactly 15 accepted poses")
        if not isinstance(prior_roles, dict):
            raise ValueError("resume plan must declare accepted prior role counts")
        merged_roles = (
            int(prior_roles.get("training", -1)) + roles.count("training"),
            int(prior_roles.get("holdout", -1)) + roles.count("holdout"),
        )
        if merged_roles != (12, 3):
            raise ValueError("resume plan must merge to 12 training and 3 holdout poses")
    else:
        cohort_labels = plan.get("cohort_expected_labels")
        cohort_roles = plan.get("cohort_expected_role_counts")
        if (
            not isinstance(cohort_labels, list)
            or len(cohort_labels) != 15
            or len(set(map(str, cohort_labels))) != 15
        ):
            raise ValueError("batch plan must declare 15 unique cohort labels")
        if not isinstance(cohort_roles, dict) or (
            int(cohort_roles.get("training", -1)),
            int(cohort_roles.get("holdout", -1)),
        ) != (12, 3):
            raise ValueError("batch plan cohort must contain 12 training and 3 holdout poses")
        if not set(str(item.get("label")) for item in poses).issubset(
            set(map(str, cohort_labels))
        ):
            raise ValueError("batch pose labels must belong to the declared cohort")
    labels = [str(item.get("label")) for item in poses]
    if len(labels) != len(set(labels)):
        raise ValueError("recalibration pose labels must be unique")
    for item in poses:
        positions = np.asarray(item.get("positions"), dtype=np.float64)
        if positions.shape != (6,) or not np.all(np.isfinite(positions)):
            raise ValueError(f"invalid positions for {item.get('label')}")
    baseline = np.asarray(plan.get("reference_baseline"), dtype=np.float64)
    if baseline.shape != (6,) or not np.all(np.isfinite(baseline)):
        raise ValueError("invalid reference_baseline")
    max_start_drift = float(plan.get("max_disabled_start_drift_rad", 0.02))
    if not np.isfinite(max_start_drift) or not 0.0 < max_start_drift <= 0.05:
        raise ValueError("max_disabled_start_drift_rad must be in (0, 0.05]")
    return plan


def _capture_stability(capture: dict[str, object]) -> dict[str, object]:
    frames = capture["raw_frames"]
    centers = np.asarray([frame["marker"]["center_px"] for frame in frames], dtype=np.float64)
    joints = np.asarray([frame["joint_positions"] for frame in frames], dtype=np.float64)
    marker_translations = np.asarray(
        [transform_matrix(frame["marker"]["camera_to_marker"])[:3, 3] for frame in frames],
        dtype=np.float64,
    )
    center_std = np.std(centers, axis=0)
    joint_span = np.ptp(joints, axis=0)
    marker_translation_std = np.std(marker_translations, axis=0)
    passed = bool(
        float(np.max(center_std)) <= 1.0
        and float(np.max(joint_span)) <= 0.004
        and float(np.max(marker_translation_std)) <= 0.002
    )
    return {
        "center_std_px": center_std.tolist(),
        "joint_span_rad": joint_span.tolist(),
        "marker_translation_std_m": marker_translation_std.tolist(),
        "limits": {
            "center_std_px": 1.0,
            "joint_span_rad": 0.004,
            "marker_translation_std_m": 0.002,
        },
        "pass": passed,
    }


def _select_stable_window(
    capture: dict[str, object], *, window_frames: int
) -> dict[str, object] | None:
    frames = capture["raw_frames"]
    if window_frames <= 0 or window_frames > len(frames):
        raise ValueError("stability window must fit inside captured frames")
    for start in range(len(frames) - window_frames + 1):
        selected = frames[start : start + window_frames]
        candidate = dict(capture)
        candidate["raw_frames"] = selected
        stability = _capture_stability(candidate)
        if not stability["pass"]:
            continue
        base_to_end = [
            transform_matrix(frame["base_to_end"]) for frame in selected
        ]
        camera_to_marker = [
            transform_matrix(frame["marker"]["camera_to_marker"]) for frame in selected
        ]
        candidate["aggregate"] = {
            "label": capture["label"],
            "base_to_end": matrix_transform(_average_transforms(base_to_end)),
            "camera_to_marker": matrix_transform(_average_transforms(camera_to_marker)),
        }
        candidate["accepted"] = window_frames
        candidate["stability_window"] = {
            "source_frame_count": len(frames),
            "start_index": start,
            "frame_count": window_frames,
        }
        candidate["stability"] = stability
        return candidate
    return None


def _stable_capture(
    *,
    node: PairedTrajectoryNode,
    camera,
    model,
    label: str,
    target: np.ndarray,
    settle_sec: float,
    candidate_frames: int,
    stability_window_frames: int,
    rejected_sink: list[dict[str, object]],
) -> dict[str, object]:
    failures: list[str] = []
    for attempt in range(1, 4):
        node.hold_and_collect(settle_sec)
        try:
            capture = _capture_pose(
                node=node,
                camera=camera,
                model=model,
                label=label,
                target=target,
                enabled=True,
                required=candidate_frames,
                maximum_frames=max(candidate_frames, int(np.ceil(candidate_frames / 0.90))),
            )
        except RuntimeError as exc:
            failures.append(f"attempt {attempt}: {exc}")
            continue
        selected = _select_stable_window(
            capture, window_frames=stability_window_frames
        )
        stability = _capture_stability(capture)
        capture["stability"] = stability
        capture["attempt_number"] = attempt
        if selected is not None:
            selected["attempt_number"] = attempt
            return selected
        rejected_sink.append(capture)
        failures.append(f"attempt {attempt}: stability gate failed: {stability}")
    raise RuntimeError(f"{label} failed three stable-capture attempts: {failures}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument(
        "--plan",
        type=Path,
        default=ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-recalibration-plan.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm", default="")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.confirm != CONFIRMATION:
        raise SystemExit(f"real run requires --confirm {CONFIRMATION}")
    plan = _load_plan(args.plan)
    reference = np.asarray(plan["reference_baseline"], dtype=np.float64)
    duration_sec = float(plan["duration_sec_per_leg"])
    settle_sec = float(plan["settle_sec"])
    candidate_frames = int(plan.get("capture_candidate_frames", 30))
    stability_window_frames = int(plan.get("capture_stability_window_frames", 30))
    if not 30 <= stability_window_frames <= candidate_frames <= 120:
        raise ValueError("capture frame settings must satisfy 30 <= window <= candidate <= 120")
    rclpy.init()
    node = PairedTrajectoryNode(namespace=args.namespace, backend="real")
    camera = _camera()
    model = load_robot_model()
    report: dict[str, object] = {
        "schema_version": 1,
        "kind": "p5_handeye_recalibration_raw",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "authorized_scope": (
            f"{len(plan['poses'])} reviewed hand-eye poses and controlled "
            "reference-baseline return"
        ),
        "plan": plan,
        "prior_capture_report": plan.get("prior_capture_report"),
        "prior_capture_reports": plan.get("prior_capture_reports", []),
        "services": [],
        "motion_legs": [],
        "captures": [],
        "rejected_captures": [],
        "capture_failures": [],
        "success": False,
    }
    hardware_enabled = False
    allow_controlled_return = False
    try:
        node.wait_for_preflight()
        live_start = np.asarray(node.canonical_positions(), dtype=np.float64)
        report["live_start"] = live_start.tolist()
        report["preflight_status"] = node._status_payload()
        start_drift = float(np.max(np.abs(live_start - reference)))
        report["reference_start_max_drift_rad"] = start_drift
        max_start_drift = float(plan.get("max_disabled_start_drift_rad", 0.02))
        report["max_disabled_start_drift_rad"] = max_start_drift
        if start_drift > max_start_drift:
            raise RuntimeError(
                "live disabled start differs from reference baseline by "
                f"{start_drift:.6f} rad > {max_start_drift:.6f} rad"
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
            label="recalibration_disabled_preflight",
            target=live_start,
            enabled=False,
        )

        report["services"].append(node.call_trigger(node.enable_client, "enable"))
        hardware_enabled = True
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        current = live_start
        if start_drift > 0.002:
            align = build_quintic_command(
                current,
                reference,
                duration_sec=10.0,
                cadence_sec=0.05,
                label="handeye_recalibration_align_reference",
            )
            allow_controlled_return = False
            align_leg = node.execute_leg(align)
            report["motion_legs"].append(align_leg)
            if not align_leg["success"]:
                raise RuntimeError(f"reference alignment failed: {align_leg['result']}")
            allow_controlled_return = True
            current = reference

        for source in plan["poses"]:
            label = str(source["label"])
            target = np.asarray(source["positions"], dtype=np.float64)
            command = build_quintic_command(
                current,
                target,
                duration_sec=duration_sec,
                cadence_sec=0.05,
                label=f"handeye_recalibration_{label}",
            )
            allow_controlled_return = False
            leg = node.execute_leg(command)
            report["motion_legs"].append(leg)
            if not leg["success"]:
                raise RuntimeError(f"{label} motion failed: {leg['result']}")
            allow_controlled_return = True
            try:
                capture = _stable_capture(
                    node=node,
                    camera=camera,
                    model=model,
                    label=label,
                    target=target,
                    settle_sec=settle_sec,
                    candidate_frames=candidate_frames,
                    stability_window_frames=stability_window_frames,
                    rejected_sink=report["rejected_captures"],
                )
            except RuntimeError as exc:
                report["capture_failures"].append(
                    {"label": label, "role": source["role"], "failure": str(exc)}
                )
            else:
                capture["role"] = source["role"]
                report["captures"].append(capture)
            current = target

        return_command = build_quintic_command(
            current,
            reference,
            duration_sec=duration_sec,
            cadence_sec=0.05,
            label="handeye_recalibration_return_reference",
        )
        allow_controlled_return = False
        return_leg = node.execute_leg(return_command)
        report["motion_legs"].append(return_leg)
        if not return_leg["success"]:
            raise RuntimeError(f"reference return failed: {return_leg['result']}")
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        final = np.asarray(node.canonical_positions(), dtype=np.float64)
        errors = reference - final
        report["enabled_final_positions"] = final.tolist()
        report["enabled_final_errors"] = errors.tolist()
        if float(np.max(np.abs(errors))) > 0.02:
            raise RuntimeError("controlled reference-baseline return error exceeds 0.02 rad")
        report["services"].append(node.call_trigger(node.disable_client, "disable"))
        hardware_enabled = False
        node.hold_and_collect(0.5)
        report["final_status"] = node._status_payload()
        report["motion_completed"] = True
        report["success"] = not report["capture_failures"]
    except Exception as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        if hardware_enabled:
            try:
                hardware_enabled = recover_real_failure(
                    node=node,
                    report=report,
                    baseline=reference,
                    allow_controlled_return=allow_controlled_return,
                    legs_key="motion_legs",
                    command_label="handeye_recalibration_failure_return_reference",
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

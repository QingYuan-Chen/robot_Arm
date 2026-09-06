#!/usr/bin/env python3
"""Run the authorized low-speed six-pose hand-eye residual collection."""

from __future__ import annotations

import argparse
import json
import math
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

from p5_paired_trajectory_runner import PairedTrajectoryNode
from rebotarm_calibration.aruco_pose import detect_aruco_pose
from rebotarm_calibration.handeye_residual import (
    analyze_handeye_residual,
    matrix_transform,
    transform_matrix,
)
from rebotarm_motion.paired_trajectory_protocol import build_quintic_command
from rebotarm_motion.real_failure_recovery import recover_real_failure
from rebotarm_vision.camera.gemini2_driver import Gemini2Config, Gemini2Driver
from reBotArm_control_py.kinematics import compute_fk, load_robot_model


CONFIRMATION = "REAL_HAND_EYE_SIX_POSE"
REFERENCE_BASELINE = np.array(
    [-1.5558481216430664, 0.00553131103515625, -0.01163482666015625,
     0.00972747802734375, 0.08144474029541016, 0.00667572021484375],
    dtype=np.float64,
)
POSES = (
    ("pose_1", [-1.52, -0.04, -0.08, 0.08, 0.08, 0.00]),
    ("pose_2", [-1.61, -0.08, -0.14, 0.16, 0.02, -0.05]),
    ("safe", [-math.pi / 2.0, -0.10, -0.20, 0.20, 0.0, 0.0]),
    ("pose_4", [-1.50, -0.12, -0.22, 0.12, -0.08, 0.08]),
    ("pose_5", [-1.64, -0.16, -0.24, 0.28, 0.10, -0.10]),
)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    left, _, right = np.linalg.svd(np.mean(np.stack(rotations), axis=0))
    result = left @ right
    if np.linalg.det(result) < 0.0:
        left[:, -1] *= -1.0
        result = left @ right
    return result


def _average_transforms(transforms: list[np.ndarray]) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _average_rotation([value[:3, :3] for value in transforms])
    result[:3, 3] = np.mean(np.stack([value[:3, 3] for value in transforms]), axis=0)
    return result


def _camera() -> Gemini2Driver:
    return Gemini2Driver(
        Gemini2Config(640, 480, 30, True, 640, 400, 30, 1000, True)
    )


def _camera_matrix(info: dict[str, object]) -> np.ndarray:
    return np.array(
        [[info["fx"], 0.0, info["cx"]], [0.0, info["fy"], info["cy"]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _joint_vectors(node: PairedTrajectoryNode) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    message = node.latest_joint_state
    if message is None:
        raise RuntimeError("joint state unavailable during capture")
    by_name = {name: index for index, name in enumerate(message.name)}
    names = tuple(f"joint{index}" for index in range(1, 7))
    return tuple(
        np.array([float(values[by_name[name]]) for name in names], dtype=np.float64)
        for values in (message.position, message.velocity, message.effort)
    )


def _check_enabled_hold(node: PairedTrajectoryNode, target: np.ndarray) -> None:
    if node.latest_status is None:
        raise RuntimeError("ArmStatus unavailable during enabled hold")
    status = node.latest_status
    if not status.enabled or not status.control_loop_active:
        raise RuntimeError("controller left enabled hold state")
    if list(status.per_joint_status_code) != [1] * 6 or status.error_codes:
        raise RuntimeError(
            f"motor status/error during hold: {list(status.per_joint_status_code)} {list(status.error_codes)}"
        )
    if node.last_joint_monotonic is None or time.monotonic() - node.last_joint_monotonic > 0.5:
        raise RuntimeError("joint state stale during hold")
def _capture_pose(
    *,
    node: PairedTrajectoryNode,
    camera: Gemini2Driver,
    model,
    label: str,
    target: np.ndarray,
    enabled: bool,
    required: int = 30,
    maximum_frames: int = 45,
) -> dict[str, object]:
    info = camera.get_camera_info("color")
    if info is None:
        raise RuntimeError("color CameraInfo unavailable")
    matrix = _camera_matrix(info)
    distortion = list(info["d"])
    raw = []
    attempts = 0
    while attempts < maximum_frames and len(raw) < required:
        color, _ = camera.get_frame(allow_partial=True)
        attempts += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        if enabled:
            _check_enabled_hold(node, target)
        if color is None:
            continue
        try:
            marker = detect_aruco_pose(
                color,
                camera_matrix=matrix,
                distortion=distortion,
                marker_length_m=0.100,
            )
        except ValueError:
            continue
        if marker["area_px2"] < 10_000 or marker["reprojection_rmse_px"] > 2.0:
            continue
        positions, velocities, efforts = _joint_vectors(node)
        _, _, base_to_end = compute_fk(model, positions, "end_link")
        raw.append(
            {
                "monotonic_ns": time.monotonic_ns(),
                "joint_positions": positions.tolist(),
                "joint_velocities": velocities.tolist(),
                "joint_efforts": efforts.tolist(),
                "base_to_end": matrix_transform(base_to_end),
                "marker": marker,
            }
        )
    rate = len(raw) / float(max(attempts, 1))
    if len(raw) < required or rate < 0.90:
        raise RuntimeError(
            f"{label} marker capture gate failed: accepted={len(raw)}/{attempts}, rate={rate:.3f}"
        )
    base_to_end_values = [transform_matrix(item["base_to_end"]) for item in raw]
    camera_marker_values = [
        transform_matrix(item["marker"]["camera_to_marker"]) for item in raw
    ]
    aggregate = {
        "label": label,
        "base_to_end": matrix_transform(_average_transforms(base_to_end_values)),
        "camera_to_marker": matrix_transform(_average_transforms(camera_marker_values)),
    }
    return {
        "label": label,
        "attempts": attempts,
        "accepted": len(raw),
        "detection_rate": rate,
        "aggregate": aggregate,
        "raw_frames": raw,
    }


def _load_handeye(path: Path) -> np.ndarray:
    import yaml

    data = yaml.safe_load(path.read_text(encoding="utf-8"))["handeye"]
    return transform_matrix(
        {
            "translation": [data["translation"][key] for key in "xyz"],
            "rotation_xyzw": [data["rotation"][key] for key in "xyzw"],
        }
    )


def _sdk_extrinsic(calibration: dict[str, object], key: str) -> np.ndarray:
    source = calibration[key]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(source["rotation"], dtype=np.float64)
    result[:3, 3] = np.asarray(source["translation_mm"], dtype=np.float64) * 1e-3
    return result


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--handeye-config",
        type=Path,
        default=ROOT / "src" / "rebotarm_vision" / "config" / "handeye.yaml",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="capture the disabled baseline marker sample and exit without enabling",
    )
    parser.add_argument("--confirm", default="")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not args.preflight_only and args.confirm != CONFIRMATION:
        raise SystemExit(f"real run requires --confirm {CONFIRMATION}")
    rclpy.init()
    node = PairedTrajectoryNode(namespace=args.namespace, backend="real")
    camera = _camera()
    model = load_robot_model()
    report: dict[str, object] = {
        "schema_version": 1,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "authorized_scope": "six-pose hand-eye residual; no approach/gripper/lift/retreat",
        "success": False,
        "services": [],
        "motion_legs": [],
        "pose_captures": [],
    }
    hardware_enabled = False
    baseline: np.ndarray | None = None
    allow_controlled_return = False
    try:
        node.wait_for_preflight()
        baseline = np.asarray(node.canonical_positions(), dtype=np.float64)
        report["baseline"] = baseline.tolist()
        report["preflight_status"] = node._status_payload()
        drift = float(np.max(np.abs(baseline - REFERENCE_BASELINE)))
        report["reference_baseline_max_drift_rad"] = drift
        if drift > 0.05:
            raise RuntimeError(f"live baseline drift exceeds 0.05 rad: {drift}")
        camera.open()
        camera.warmup(15)
        report["camera_device"] = camera.get_device_info()
        report["camera_info"] = camera.get_camera_info("color")
        report["camera_calibration"] = camera.get_calibration_info()
        report["pose_captures"].append(
            _capture_pose(
                node=node,
                camera=camera,
                model=model,
                label="baseline",
                target=baseline,
                enabled=False,
            )
        )
        if args.preflight_only:
            report["preflight_only"] = True
            report["final_status"] = node._status_payload()
            report["success"] = True
            return
        report["services"].append(node.call_trigger(node.enable_client, "enable"))
        hardware_enabled = True
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        current = baseline
        for label, values in POSES:
            target = np.asarray(values, dtype=np.float64)
            command = build_quintic_command(
                current,
                target,
                duration_sec=15.0,
                cadence_sec=0.05,
                label=f"handeye_{label}",
            )
            allow_controlled_return = False
            leg = node.execute_leg(command)
            report["motion_legs"].append(leg)
            if not leg["success"]:
                raise RuntimeError(f"{label} motion failed: {leg['result']}")
            allow_controlled_return = True
            node.hold_and_collect(0.5)
            report["pose_captures"].append(
                _capture_pose(
                    node=node,
                    camera=camera,
                    model=model,
                    label=label,
                    target=target,
                    enabled=True,
                )
            )
            current = target
        return_command = build_quintic_command(
            current,
            baseline,
            duration_sec=20.0,
            cadence_sec=0.05,
            label="handeye_return_baseline",
        )
        allow_controlled_return = False
        return_leg = node.execute_leg(return_command)
        report["motion_legs"].append(return_leg)
        if not return_leg["success"]:
            raise RuntimeError(f"return motion failed: {return_leg['result']}")
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        final = np.asarray(node.canonical_positions(), dtype=np.float64)
        report["enabled_final_positions"] = final.tolist()
        report["enabled_final_errors"] = (baseline - final).tolist()
        if float(np.max(np.abs(baseline - final))) > 0.02:
            raise RuntimeError("controlled baseline return error exceeds 0.02 rad")
        report["services"].append(node.call_trigger(node.disable_client, "disable"))
        hardware_enabled = False
        node.hold_and_collect(0.5)
        report["final_status"] = node._status_payload()

        samples = [capture["aggregate"] for capture in report["pose_captures"]]
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
                if baseline is None:
                    raise RuntimeError("baseline unavailable for failure recovery")
                hardware_enabled = recover_real_failure(
                    node=node,
                    report=report,
                    baseline=baseline,
                    allow_controlled_return=allow_controlled_return,
                    legs_key="motion_legs",
                    command_label="handeye_failure_return_baseline",
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

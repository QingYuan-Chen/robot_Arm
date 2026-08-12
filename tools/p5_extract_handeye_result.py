#!/usr/bin/env python3
"""Extract a compact durable summary from a raw replacement-pose report."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics


def _capture_summary(capture: dict[str, object]) -> dict[str, object]:
    frames = capture.get("raw_frames") or []
    centers = [frame["marker"]["center_px"] for frame in frames]
    return {
        "label": capture.get("label"),
        "attempts": capture.get("attempts"),
        "accepted": capture.get("accepted"),
        "detection_rate": capture.get("detection_rate"),
        "center_mean_px": [statistics.fmean(value[index] for value in centers) for index in (0, 1)],
        "center_std_px": [statistics.pstdev(value[index] for value in centers) for index in (0, 1)],
        "area_min_px2": min(frame["marker"]["area_px2"] for frame in frames),
        "area_mean_px2": statistics.fmean(frame["marker"]["area_px2"] for frame in frames),
        "reprojection_rmse_max_px": max(
            frame["marker"]["reprojection_rmse_px"] for frame in frames
        ),
        "aggregate": capture.get("aggregate"),
    }


def _leg_summary(leg: dict[str, object]) -> dict[str, object]:
    samples = leg.get("samples") or []
    command = leg.get("command") or {}
    return {
        "label": command.get("label"),
        "command_sha256": command.get("command_sha256"),
        "duration_sec": command.get("duration_sec"),
        "point_count": len(command.get("points") or []),
        "success": leg.get("success"),
        "result": leg.get("result"),
        "sample_count": len(samples),
        "max_tracking_error_rad": max(
            (max(abs(value) for value in sample["tracking_errors"]) for sample in samples),
            default=None,
        ),
        "max_raw_velocity_rad_s": max(
            (max(abs(value) for value in sample["velocities"]) for sample in samples),
            default=None,
        ),
        "max_window_velocity_rad_s": max(
            (max(abs(value) for value in sample["window_velocities"]) for sample in samples),
            default=None,
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raw_bytes = args.input.read_bytes()
    source = json.loads(raw_bytes)
    output = {
        "schema_version": 1,
        "kind": "p5_handeye_replacement_result_summary",
        "source_sha256": hashlib.sha256(raw_bytes).hexdigest(),
        "started_at": source.get("started_at"),
        "finished_at": source.get("finished_at"),
        "success": source.get("success"),
        "failure": source.get("failure"),
        "authorized_scope": source.get("authorized_scope"),
        "candidate": source.get("candidate"),
        "reference_baseline": source.get("reference_baseline"),
        "live_start": source.get("live_start"),
        "validated_start_max_drift_rad": source.get("validated_start_max_drift_rad"),
        "preflight_status": source.get("preflight_status"),
        "camera_device": source.get("camera_device"),
        "camera_info": source.get("camera_info"),
        "camera_calibration": source.get("camera_calibration"),
        "disabled_marker_preflight": _capture_summary(source["disabled_marker_preflight"]),
        "candidate_capture": _capture_summary(source["candidate_capture"]),
        "motion_legs": [_leg_summary(leg) for leg in source.get("motion_legs") or []],
        "services": source.get("services"),
        "enabled_final_positions": source.get("enabled_final_positions"),
        "enabled_final_errors": source.get("enabled_final_errors"),
        "final_status": source.get("final_status"),
        "residual_hypotheses": source.get("residual_hypotheses"),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(args.output), "success": output["success"]}))


if __name__ == "__main__":
    main()

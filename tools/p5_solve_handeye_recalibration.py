#!/usr/bin/env python3
"""Solve and diagnose the split P5 eye-in-hand recalibration dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_SOURCE = ROOT / "src" / "rebotarm_calibration"
if str(CALIBRATION_SOURCE) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_SOURCE))

from rebotarm_calibration.handeye_residual import matrix_transform, transform_matrix
from rebotarm_calibration.handeye_solver import (
    HAND_EYE_METHODS,
    average_transforms,
    evaluate_eye_in_hand,
    rotation_angle_deg,
    solve_eye_in_hand,
)


DEFAULT_REPORTS = [
    ROOT / "Agent/evidence/P5/2026-08-09-handeye-recalibration-raw.json",
    ROOT / "Agent/evidence/P5/2026-08-09-handeye-recalibration-resume-raw.json",
    ROOT / "Agent/evidence/P5/2026-08-09-handeye-recalibration-resume2-raw.json",
    ROOT / "Agent/evidence/P5/2026-08-09-handeye-recalibration-remaining9-raw.json",
]


def _sample(capture: dict[str, object]) -> dict[str, object]:
    aggregate = dict(capture["aggregate"])
    aggregate["role"] = str(capture["role"])
    return aggregate


def _capture_diagnostic(capture: dict[str, object]) -> dict[str, object]:
    frames = capture["raw_frames"]
    marker_values = [
        transform_matrix(frame["marker"]["camera_to_marker"]) for frame in frames
    ]
    mean_marker = average_transforms(marker_values)
    rotation_deviations = np.array(
        [
            rotation_angle_deg(mean_marker[:3, :3].T @ value[:3, :3])
            for value in marker_values
        ],
        dtype=np.float64,
    )
    reprojection = np.array(
        [float(frame["marker"]["reprojection_rmse_px"]) for frame in frames],
        dtype=np.float64,
    )
    return {
        "label": capture["label"],
        "role": capture["role"],
        "accepted_frames": len(frames),
        "attempt_number": capture.get("attempt_number"),
        "rotation_jitter_rms_deg": float(
            math.sqrt(float(np.mean(rotation_deviations**2)))
        ),
        "rotation_jitter_max_deg": float(np.max(rotation_deviations)),
        "reprojection_rmse_mean_px": float(np.mean(reprojection)),
        "reprojection_rmse_max_px": float(np.max(reprojection)),
        "stability": capture.get("stability"),
    }


def _pose_diversity(samples: list[dict[str, object]]) -> dict[str, float | None]:
    if len(samples) < 2:
        return {
            "end_translation_span_m": None,
            "end_rotation_span_deg": None,
        }
    base_to_end = [transform_matrix(sample["base_to_end"]) for sample in samples]
    translation_span = max(
        float(np.linalg.norm(left[:3, 3] - right[:3, 3]))
        for index, left in enumerate(base_to_end)
        for right in base_to_end[index + 1 :]
    )
    rotation_span = max(
        rotation_angle_deg(left[:3, :3].T @ right[:3, :3])
        for index, left in enumerate(base_to_end)
        for right in base_to_end[index + 1 :]
    )
    return {
        "end_translation_span_m": translation_span,
        "end_rotation_span_deg": rotation_span,
    }


def _method_reports(
    training: list[dict[str, object]],
    holdout: list[dict[str, object]],
) -> tuple[list[dict[str, object]], str]:
    reports = []
    best_score = math.inf
    selected = ""
    for method in HAND_EYE_METHODS:
        handeye = solve_eye_in_hand(training, method=method)
        train_report = evaluate_eye_in_hand(training, handeye)
        marker_reference = transform_matrix(train_report["reference_base_to_marker"])
        holdout_report = (
            evaluate_eye_in_hand(
                holdout,
                handeye,
                reference_base_to_marker=marker_reference,
            )
            if holdout
            else None
        )
        score = (
            float(train_report["position_residual"]["rms_m"]) / 0.005
            + float(train_report["rotation_residual"]["rms_deg"]) / 1.5
        )
        reports.append(
            {
                "method": method,
                "training_gate_normalized_score": score,
                "end_to_camera": matrix_transform(handeye),
                "training": train_report,
                "holdout_against_training_reference": holdout_report,
            }
        )
        if score < best_score:
            best_score = score
            selected = method
    return reports, selected


def _cohort_report(
    samples: list[dict[str, object]],
) -> dict[str, object]:
    training = [sample for sample in samples if sample["role"] == "training"]
    holdout = [sample for sample in samples if sample["role"] == "holdout"]
    methods, selected = _method_reports(training, holdout)
    return {
        "sample_count": len(samples),
        "training_count": len(training),
        "holdout_count": len(holdout),
        "training_diversity": _pose_diversity(training),
        "holdout_diversity": _pose_diversity(holdout),
        "selection_rule": "minimum training-only normalized gate score; holdout not used",
        "selected_method": selected,
        "methods": methods,
    }


def _find_method(report: dict[str, object], method: str) -> dict[str, object]:
    return next(item for item in report["methods"] if item["method"] == method)


def _evaluate_cross_cohort(
    solved_cohort: dict[str, object],
    evaluation_samples: list[dict[str, object]],
) -> dict[str, object]:
    selected = str(solved_cohort["selected_method"])
    method = _find_method(solved_cohort, selected)
    handeye = transform_matrix(method["end_to_camera"])
    marker_reference = transform_matrix(
        method["training"]["reference_base_to_marker"]
    )
    return {
        "source_method": selected,
        "evaluation": evaluate_eye_in_hand(
            evaluation_samples,
            handeye,
            reference_base_to_marker=marker_reference,
        ),
    }


def _scale_sensitivity(
    training: list[dict[str, object]], method: str
) -> dict[str, object]:
    rows = []
    best = None
    for marker_length_mm in np.linspace(80.0, 120.0, 401):
        factor = float(marker_length_mm / 100.0)
        scaled = []
        for sample in training:
            item = dict(sample)
            camera_marker = transform_matrix(sample["camera_to_marker"])
            camera_marker[:3, 3] *= factor
            item["camera_to_marker"] = matrix_transform(camera_marker)
            scaled.append(item)
        handeye = solve_eye_in_hand(scaled, method=method)
        report = evaluate_eye_in_hand(scaled, handeye)
        value = float(report["position_residual"]["rms_m"])
        candidate = {
            "marker_length_mm": float(marker_length_mm),
            "position_rms_m": value,
            "position_max_m": float(report["position_residual"]["max_m"]),
            "rotation_rms_deg": float(report["rotation_residual"]["rms_deg"]),
            "rotation_max_deg": float(report["rotation_residual"]["max_deg"]),
        }
        if best is None or value < best[0]:
            best = (value, candidate)
        if marker_length_mm in {95.0, 97.5, 100.0, 102.5, 105.0}:
            rows.append(candidate)
    assert best is not None
    return {
        "assumption": "PnP translation scales linearly with physical marker side; rotation is unchanged",
        "search_range_mm": [80.0, 120.0],
        "selected_method": method,
        "best_training_position_rms": best[1],
        "reference_points": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, action="append", default=[])
    parser.add_argument("--fixed-report", type=Path)
    parser.add_argument(
        "--same-fixed-cohort",
        action="store_true",
        help="Treat every input report as captured without moving the marker",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    paths = args.input or DEFAULT_REPORTS
    fixed_path = (args.fixed_report or paths[-1]).resolve()

    reports = []
    captures = []
    labels = set()
    for path in paths:
        raw_bytes = path.read_bytes()
        source = json.loads(raw_bytes)
        report_entry = {
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(raw_bytes).hexdigest(),
            "started_at": source.get("started_at"),
            "finished_at": source.get("finished_at"),
            "success": source.get("success"),
            "motion_completed": source.get("motion_completed"),
            "capture_count": len(source.get("captures") or []),
            "capture_failure_count": len(source.get("capture_failures") or []),
        }
        reports.append(report_entry)
        for capture in source.get("captures") or []:
            label = str(capture["label"])
            if label in labels:
                raise ValueError(f"duplicate accepted capture label: {label}")
            labels.add(label)
            captures.append(
                {
                    "source_path": str(path.resolve()),
                    "sample": _sample(capture),
                    "diagnostic": _capture_diagnostic(capture),
                }
            )

    all_samples = [item["sample"] for item in captures]
    if args.same_fixed_cohort:
        fixed_samples = list(all_samples)
        earlier_samples = []
    else:
        fixed_samples = [
            item["sample"]
            for item in captures
            if Path(item["source_path"]).resolve() == fixed_path
        ]
        earlier_samples = [
            item["sample"]
            for item in captures
            if Path(item["source_path"]).resolve() != fixed_path
        ]
        if not fixed_samples or not earlier_samples:
            raise ValueError("fixed and earlier cohorts must both contain accepted captures")

    combined = _cohort_report(all_samples)
    fixed = _cohort_report(fixed_samples)
    cross = (
        _evaluate_cross_cohort(fixed, earlier_samples) if earlier_samples else None
    )
    fixed_training = [sample for sample in fixed_samples if sample["role"] == "training"]
    scale = _scale_sensitivity(fixed_training, str(fixed["selected_method"]))

    selected_method = str(combined["selected_method"])
    selected_result = _find_method(combined, selected_method)
    training_result = selected_result["training"]
    holdout_result = selected_result["holdout_against_training_reference"]
    holdout_diversity = combined["holdout_diversity"]
    complete_split = len(all_samples) == 15 and (
        sum(sample["role"] == "training" for sample in all_samples),
        sum(sample["role"] == "holdout" for sample in all_samples),
    ) == (12, 3)
    residual_pass = bool(
        float(training_result["position_residual"]["rms_m"]) <= 0.005
        and float(training_result["position_residual"]["max_m"]) <= 0.010
        and float(training_result["rotation_residual"]["rms_deg"]) <= 1.5
        and float(training_result["rotation_residual"]["max_deg"]) <= 3.0
        and holdout_result is not None
        and float(holdout_result["position_residual"]["rms_m"]) <= 0.005
        and float(holdout_result["position_residual"]["max_m"]) <= 0.010
        and float(holdout_result["rotation_residual"]["rms_deg"]) <= 1.5
        and float(holdout_result["rotation_residual"]["max_deg"]) <= 3.0
    )
    diversity_pass = bool(
        holdout_diversity["end_rotation_span_deg"] is not None
        and float(holdout_diversity["end_rotation_span_deg"]) >= 20.0
    )
    deploy = bool(
        args.same_fixed_cohort and complete_split and residual_pass and diversity_pass
    )
    if args.same_fixed_cohort:
        if not complete_split:
            decision_reason = "fixed cohort is incomplete"
            required_next = "complete the fixed-marker training/holdout cohort"
        elif residual_pass and diversity_pass:
            decision_reason = (
                "fixed cohort is complete and all training, independent holdout, and "
                "holdout rotation-diversity gates are satisfied"
            )
            required_next = (
                "review the candidate transform and perform an explicitly authorized "
                "configuration deployment plus post-deployment TCP residual validation"
            )
        else:
            decision_reason = (
                "fixed cohort is complete, but the training/holdout residual and holdout "
                "rotation-diversity gates are not all satisfied"
            )
            required_next = (
                "diagnose rotation accuracy and holdout residual before any handeye deployment"
            )
    else:
        decision_reason = (
            "marker was re-positioned/fixed between cohorts; the fixed-cohort solution "
            "shows a systematic earlier-cohort rotation disagreement, and the fixed "
            "cohort has only one holdout pose"
        )
        required_next = (
            "replace the four earlier training and two earlier holdout captures while "
            "the marker remains rigidly fixed, then solve on 12 training and evaluate "
            "three independent holdout captures"
        )

    output = {
        "schema_version": 1,
        "kind": "p5_handeye_recalibration_solve_diagnostic",
        "reported_marker_length_m": 0.100,
        "acceptance_limits": {
            "position_rms_m": 0.005,
            "position_max_m": 0.010,
            "rotation_rms_deg": 1.5,
            "rotation_max_deg": 3.0,
            "holdout_min_samples": 3,
            "holdout_min_rotation_span_deg": 20.0,
        },
        "source_reports": reports,
        "accepted_capture_count": len(captures),
        "role_counts": {
            "training": sum(sample["role"] == "training" for sample in all_samples),
            "holdout": sum(sample["role"] == "holdout" for sample in all_samples),
        },
        "capture_diagnostics": [item["diagnostic"] for item in captures],
        "combined_dataset": combined,
        "fixed_marker_cohort": fixed,
        "fixed_solution_applied_to_earlier_cohort": cross,
        "fixed_cohort_marker_scale_sensitivity": scale,
        "decision": {
            "deploy_handeye": deploy,
            "combined_dataset_valid": bool(args.same_fixed_cohort and complete_split),
            "complete_training_holdout_split": complete_split,
            "residual_gates_pass": residual_pass,
            "holdout_diversity_pass": diversity_pass,
            "reason": decision_reason,
            "required_next_dataset": required_next,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(output["decision"], ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-recalibration-plan.json"
RESUME_PLAN = (
    ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-recalibration-resume-plan.json"
)
RESUME2_PLAN = (
    ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-recalibration-resume2-plan.json"
)
REMAINING9_PLAN = (
    ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-recalibration-remaining9-plan.json"
)
REPLACE6_PLAN = (
    ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-recalibration-replace6-plan.json"
)
SUBPIX_BATCH_PLANS = [
    ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-subpix-batch1-plan.json",
    ROOT / "Agent" / "evidence" / "P5" / "2026-08-09-handeye-subpix-batch2-plan.json",
]


def test_recalibration_plan_has_training_holdout_split_and_validated_path() -> None:
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    assert plan["kind"] == "p5_handeye_recalibration_plan"
    assert len(plan["reference_baseline"]) == 6
    assert len(plan["poses"]) == 15
    assert sum(pose["role"] == "training" for pose in plan["poses"]) == 12
    assert sum(pose["role"] == "holdout" for pose in plan["poses"]) == 3
    assert len({pose["label"] for pose in plan["poses"]}) == 15
    assert all(len(pose["positions"]) == 6 for pose in plan["poses"])
    assert all(np.all(np.isfinite(pose["positions"])) for pose in plan["poses"])
    assert min(pose["robust_fov_margin_px"] for pose in plan["poses"]) >= 50.0
    assert plan["rotation_vector_condition_number"] < 5.0
    assert plan["moveit_validation"] == {
        "leg_count": 16,
        "samples_per_leg": 101,
        "valid": 1616,
        "total": 1616,
        "invalid": 0,
    }


def test_recalibration_plan_keeps_slow_motion_and_stability_capture() -> None:
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    assert plan["duration_sec_per_leg"] >= 20.0
    assert plan["cadence_sec"] == 0.05
    assert plan["settle_sec"] >= 2.0
    assert plan["capture_frames_per_pose"] >= 30


def test_recalibration_resume_plan_reuses_only_accepted_prior_captures() -> None:
    plan = json.loads(RESUME_PLAN.read_text(encoding="utf-8"))
    assert plan["kind"] == "p5_handeye_recalibration_resume_plan"
    assert plan["accepted_prior_labels"] == ["holdout_01", "holdout_02", "train_01"]
    assert plan["accepted_prior_role_counts"] == {"training": 1, "holdout": 2}
    assert plan["rejected_prior_labels"] == ["train_02"]
    assert len(plan["poses"]) == 12
    assert sum(pose["role"] == "training" for pose in plan["poses"]) == 11
    assert sum(pose["role"] == "holdout" for pose in plan["poses"]) == 1
    assert all(pose["positions"][4] >= 0.0 for pose in plan["poses"])
    assert min(pose["robust_fov_margin_px"] for pose in plan["poses"]) >= 50.0
    assert plan["rotation_vector_condition_number_after_merge"] < 5.0
    assert plan["moveit_validation"] == {
        "leg_count": 13,
        "samples_per_leg": 101,
        "valid": 1313,
        "total": 1313,
        "invalid": 0,
    }


def test_recalibration_second_resume_plan_merges_to_same_15_pose_dataset() -> None:
    plan = json.loads(RESUME2_PLAN.read_text(encoding="utf-8"))
    assert len(plan["accepted_prior_labels"]) == 5
    assert plan["accepted_prior_role_counts"] == {"training": 3, "holdout": 2}
    assert len(plan["poses"]) == 10
    assert sum(pose["role"] == "training" for pose in plan["poses"]) == 9
    assert sum(pose["role"] == "holdout" for pose in plan["poses"]) == 1
    assert len(plan["accepted_prior_labels"]) + len(plan["poses"]) == 15
    assert all(pose["positions"][4] >= 0.0 for pose in plan["poses"])
    assert plan["moveit_validation"] == {
        "leg_count": 11,
        "samples_per_leg": 101,
        "valid": 1111,
        "total": 1111,
        "invalid": 0,
    }


def test_recalibration_remaining9_plan_reduces_hold_instability_targets() -> None:
    plan = json.loads(REMAINING9_PLAN.read_text(encoding="utf-8"))
    assert len(plan["accepted_prior_labels"]) == 6
    assert plan["accepted_prior_role_counts"] == {"training": 4, "holdout": 2}
    assert len(plan["poses"]) == 9
    assert sum(pose["role"] == "training" for pose in plan["poses"]) == 8
    assert sum(pose["role"] == "holdout" for pose in plan["poses"]) == 1
    assert max(pose["positions"][4] for pose in plan["poses"]) <= 0.14
    assert min(pose["robust_fov_margin_px"] for pose in plan["poses"]) >= 50.0
    assert plan["rotation_vector_condition_number_after_merge"] < 5.0
    assert plan["moveit_validation"] == {
        "leg_count": 10,
        "samples_per_leg": 101,
        "valid": 1010,
        "total": 1010,
        "invalid": 0,
    }


def test_recalibration_replace6_uses_only_fixed_cohort_as_prior() -> None:
    plan = json.loads(REPLACE6_PLAN.read_text(encoding="utf-8"))

    assert plan["kind"] == "p5_handeye_recalibration_resume_plan"
    assert plan["accepted_prior_role_counts"] == {"training": 8, "holdout": 1}
    assert len(plan["accepted_prior_labels"]) == 9
    assert {pose["role"] for pose in plan["poses"]} == {"training", "holdout"}
    assert sum(pose["role"] == "training" for pose in plan["poses"]) == 4
    assert sum(pose["role"] == "holdout" for pose in plan["poses"]) == 2
    assert set(plan["invalidated_cohort_labels"]) == {
        "holdout_01",
        "holdout_02",
        "train_01",
        "train_02r",
        "train_03r",
        "train_04",
    }
    assert plan["moveit_validation"] == {
        "leg_count": 7,
        "samples_per_leg": 101,
        "valid": 707,
        "total": 707,
        "invalid": 0,
    }
    assert plan["requires_new_real_motion_authorization"] is True
    assert plan["max_disabled_start_drift_rad"] == 0.04
    assert plan["live_start_alignment_validation_required"] is True


def test_subpix_batch_plans_form_one_complete_fixed_marker_cohort() -> None:
    plans = [json.loads(path.read_text(encoding="utf-8")) for path in SUBPIX_BATCH_PLANS]
    assert {plan["kind"] for plan in plans} == {"p5_handeye_recalibration_batch_plan"}
    assert {plan["cohort_id"] for plan in plans} == {"2026-08-09-subpix-fixed-marker"}
    assert {plan["batch_index"] for plan in plans} == {1, 2}
    assert all(plan["corner_refinement"] == "CORNER_REFINE_SUBPIX" for plan in plans)
    assert all(plan["settle_sec"] == 8.0 for plan in plans)
    poses = [pose for plan in plans for pose in plan["poses"]]
    assert len(poses) == 15
    assert len({pose["label"] for pose in poses}) == 15
    assert sum(pose["role"] == "training" for pose in poses) == 12
    assert sum(pose["role"] == "holdout" for pose in poses) == 3

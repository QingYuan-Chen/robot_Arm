from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspVerificationConfig:
    enabled: bool = True
    min_closure_distance_m: float = 0.006
    require_gripper_contact: bool = True
    visual_lift_check_enabled: bool = False
    min_visual_lift_delta_m: float = 0.030


@dataclass(frozen=True)
class GraspVerificationInput:
    gripper_contact_detected: bool
    closure_distance_m: float
    visual_lift_delta_m: float = 0.0
    visual_lift_evidence_available: bool = False


@dataclass(frozen=True)
class GraspVerificationResult:
    success: bool
    reason: str


def verify_grasp_after_lift(
    evidence: GraspVerificationInput,
    config: GraspVerificationConfig,
) -> GraspVerificationResult:
    if not config.enabled:
        return GraspVerificationResult(True, "verification disabled")
    if config.require_gripper_contact and not evidence.gripper_contact_detected:
        return GraspVerificationResult(False, "gripper contact not detected")
    if float(evidence.closure_distance_m) < float(config.min_closure_distance_m):
        return GraspVerificationResult(False, "gripper closure distance too small")
    if config.visual_lift_check_enabled:
        if not evidence.visual_lift_evidence_available:
            return GraspVerificationResult(False, "visual lift evidence unavailable")
        if float(evidence.visual_lift_delta_m) < float(config.min_visual_lift_delta_m):
            return GraspVerificationResult(False, "visual lift delta too small")
    return GraspVerificationResult(True, "grasp verified")

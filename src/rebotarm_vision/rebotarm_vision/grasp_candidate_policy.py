from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspCandidateScoringConfig:
    max_allowed_width_m: float = 0.20
    min_confidence: float = 0.05
    width_penalty_weight: float = 0.15


def score_grasp_candidate(
    *,
    confidence: float,
    jaw_width_m: float,
    valid: bool,
    config: GraspCandidateScoringConfig | None = None,
) -> float:
    cfg = config or GraspCandidateScoringConfig()
    if not valid:
        return -1.0
    confidence_value = float(confidence)
    width = max(float(jaw_width_m or 0.0), 0.0)
    if confidence_value < float(cfg.min_confidence):
        return -1.0
    if width <= 0.0 or width > float(cfg.max_allowed_width_m):
        return -1.0
    width_ratio = width / max(float(cfg.max_allowed_width_m), 1e-9)
    return confidence_value - float(cfg.width_penalty_weight) * width_ratio

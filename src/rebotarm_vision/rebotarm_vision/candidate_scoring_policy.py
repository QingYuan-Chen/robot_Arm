from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CandidateScoringInput:
    original_index: int
    variant_label: str
    motion_penalty: float = 0.0


@dataclass(frozen=True)
class CandidateScoringResult:
    score: float
    reason: str


def z_variant_penalty(variant_label: str) -> float:
    if "_z" not in variant_label:
        return 0.0
    try:
        return float(str(variant_label).rsplit("_z", 1)[1]) * 0.001
    except ValueError:
        return 0.0


def score_candidate(scoring_input: CandidateScoringInput) -> CandidateScoringResult:
    rank_score = -float(scoring_input.original_index)
    variant_penalty = z_variant_penalty(scoring_input.variant_label)
    motion_penalty = float(scoring_input.motion_penalty)
    score = rank_score - variant_penalty - motion_penalty
    return CandidateScoringResult(
        score=score,
        reason=(
            f"rank_score={rank_score:.2f}, "
            f"variant_penalty={variant_penalty:.3f}, "
            f"motion_penalty={motion_penalty:.3f}"
        ),
    )

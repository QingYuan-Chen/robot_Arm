from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CandidatePrecheckConfig:
    min_confidence: float = 0.0
    min_jaw_width_m: float = 0.006
    max_jaw_width_m: float = 0.085


@dataclass(frozen=True)
class CandidatePrecheckResult:
    accepted: bool
    reason: str = ""


def evaluate_candidate_precheck(
    *,
    confidence: float,
    jaw_width_m: float,
    config: CandidatePrecheckConfig = CandidatePrecheckConfig(),
) -> CandidatePrecheckResult:
    score = float(confidence)
    width = float(jaw_width_m)
    if not math.isfinite(score):
        return CandidatePrecheckResult(False, "confidence is not finite")
    if score < float(config.min_confidence):
        return CandidatePrecheckResult(
            False,
            f"confidence below minimum ({score:.4f} < {float(config.min_confidence):.4f})",
        )
    if not math.isfinite(width):
        return CandidatePrecheckResult(False, "jaw_width is not finite")
    if width < float(config.min_jaw_width_m):
        return CandidatePrecheckResult(False, f"jaw_width too small ({width:.3f}m)")
    if width > float(config.max_jaw_width_m):
        return CandidatePrecheckResult(False, f"jaw_width too large ({width:.3f}m)")
    return CandidatePrecheckResult(True)

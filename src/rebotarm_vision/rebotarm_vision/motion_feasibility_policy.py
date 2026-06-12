from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .visual_grasp_sequence import PoseTarget


CheckTarget = Callable[[PoseTarget, str], object | None]
MotionPenalty = Callable[[object], tuple[float | None, str]]


@dataclass(frozen=True)
class MotionFeasibilityResult:
    accepted: bool
    reason: str
    motion_penalty: float | None = None
    pregrasp_solution: object | None = None
    grasp_solution: object | None = None


def evaluate_motion_feasibility(
    *,
    pregrasp: PoseTarget,
    grasp: PoseTarget,
    variant_label: str,
    check_target: CheckTarget,
    motion_penalty: MotionPenalty,
) -> MotionFeasibilityResult:
    pregrasp_solution = check_target(pregrasp, f"{variant_label}/pregrasp")
    if pregrasp_solution is None:
        return MotionFeasibilityResult(False, "pregrasp infeasible")

    grasp_solution = check_target(grasp, f"{variant_label}/grasp")
    if grasp_solution is None:
        return MotionFeasibilityResult(False, "grasp infeasible", pregrasp_solution=pregrasp_solution)

    penalty, reason = motion_penalty(grasp_solution)
    if penalty is None:
        return MotionFeasibilityResult(
            False,
            reason,
            pregrasp_solution=pregrasp_solution,
            grasp_solution=grasp_solution,
        )

    return MotionFeasibilityResult(
        True,
        reason,
        motion_penalty=float(penalty),
        pregrasp_solution=pregrasp_solution,
        grasp_solution=grasp_solution,
    )

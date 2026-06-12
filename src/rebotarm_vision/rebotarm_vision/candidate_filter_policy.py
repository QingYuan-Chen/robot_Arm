from __future__ import annotations

from copy import deepcopy
from collections.abc import Sequence

from rebotarm_msgs.msg import GraspCandidateArray


def filter_candidate_array_by_reachability(
    candidates: GraspCandidateArray,
    reachable: Sequence[bool],
) -> GraspCandidateArray:
    filtered = GraspCandidateArray()
    filtered.header = candidates.header
    filtered.best_index = -1
    filtered.candidates = []
    original_to_filtered: dict[int, int] = {}
    for index, (candidate, ok) in enumerate(zip(candidates.candidates, reachable)):
        if bool(ok):
            original_to_filtered[index] = len(filtered.candidates)
            filtered.candidates.append(deepcopy(candidate))
    if filtered.candidates:
        original_best = int(getattr(candidates, "best_index", -1))
        filtered.best_index = original_to_filtered.get(original_best, 0)
    return filtered

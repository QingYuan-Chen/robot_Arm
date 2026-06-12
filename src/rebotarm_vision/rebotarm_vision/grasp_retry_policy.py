from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterable


@dataclass(frozen=True)
class RetryPolicyConfig:
    enabled: bool = False
    max_attempts: int = 1


def ordered_candidate_indices(
    *,
    candidate_count: int,
    best_index: int,
    failed_indices: Iterable[int],
    config: RetryPolicyConfig,
) -> list[int]:
    if candidate_count <= 0:
        return []
    failed = {int(index) for index in failed_indices}
    best = int(best_index)
    if best < 0 or best >= candidate_count:
        best = 0

    if not config.enabled:
        return [] if best in failed else [best]

    ordered = [best] + [index for index in range(candidate_count) if index != best]
    remaining = [index for index in ordered if index not in failed]
    remaining_attempts = max(0, int(config.max_attempts) - len(failed))
    return remaining[:remaining_attempts]

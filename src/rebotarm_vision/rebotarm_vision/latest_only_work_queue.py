from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Generic, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class LatestWork(Generic[T]):
    item: T
    pending_age_ms: float


@dataclass(frozen=True)
class LatestOnlyWorkQueueStats:
    received: int
    started: int
    completed: int
    coalesced: int
    busy: bool
    pending: bool


class LatestOnlyWorkQueue(Generic[T]):
    """Serialize expensive work while retaining only the newest pending item."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._busy = False
        self._pending: tuple[T, float] | None = None
        self._received = 0
        self._started = 0
        self._completed = 0
        self._coalesced = 0

    def submit(self, item: T, *, received_at: float) -> LatestWork[T] | None:
        with self._lock:
            self._received += 1
            if not self._busy:
                self._busy = True
                self._started += 1
                return LatestWork(item=item, pending_age_ms=0.0)
            if self._pending is not None:
                self._coalesced += 1
            self._pending = (item, float(received_at))
            return None

    def complete(self, *, completed_at: float) -> LatestWork[T] | None:
        with self._lock:
            if not self._busy:
                raise RuntimeError("cannot complete latest-only work while idle")
            self._completed += 1
            if self._pending is None:
                self._busy = False
                return None
            item, received_at = self._pending
            self._pending = None
            self._started += 1
            age_ms = max(0.0, (float(completed_at) - received_at) * 1000.0)
            return LatestWork(item=item, pending_age_ms=age_ms)

    def snapshot(self) -> LatestOnlyWorkQueueStats:
        with self._lock:
            return LatestOnlyWorkQueueStats(
                received=self._received,
                started=self._started,
                completed=self._completed,
                coalesced=self._coalesced,
                busy=self._busy,
                pending=self._pending is not None,
            )

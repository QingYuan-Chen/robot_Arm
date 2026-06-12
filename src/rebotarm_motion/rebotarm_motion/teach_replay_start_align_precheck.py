from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MoveItStartAlignPrecheckConfig:
    enabled: bool
    service: str
    skip_threshold: float
    joint_goal_tolerance: float
    velocity_scaling: float
    acceleration_scaling: float


class MoveItStartAlignPrechecker:
    """Summarize whether MoveIt start alignment is ready for teach replay."""

    def __init__(self, *, planner: Any, service_client: Any) -> None:
        self._planner = planner
        self._service_client = service_client

    def summary(
        self,
        info_payload: dict,
        *,
        config: MoveItStartAlignPrecheckConfig,
        samples=None,
        plan: bool = False,
    ) -> dict:
        max_error = info_payload.get("max_error")
        threshold = float(config.skip_threshold)
        service = str(config.service)
        if not bool(config.enabled):
            return {"state": "disabled", "message": "MoveIt start alignment disabled"}
        if not self._is_number_like(max_error):
            return {"state": "unknown", "message": "current start error unavailable"}
        if float(max_error) < threshold:
            return {
                "state": "skipped",
                "message": "already near teach start; MoveIt alignment not required",
                "max_error": float(max_error),
                "skip_threshold": threshold,
            }
        if not self._service_available():
            return {
                "state": "unavailable",
                "message": "MoveIt planning service unavailable",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": service,
            }
        if not plan:
            return {
                "state": "ready",
                "message": "MoveIt planning service ready",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": service,
            }
        if not samples:
            return {
                "state": "unknown",
                "message": "no teach samples for MoveIt start alignment precheck",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": service,
            }
        first = samples[0]
        result = self._planner.plan_joint_positions(
            joint_names=tuple(first.joint_names),
            target_positions=tuple(first.positions),
            tolerance=float(config.joint_goal_tolerance),
            velocity_scaling=float(config.velocity_scaling),
            acceleration_scaling=float(config.acceleration_scaling),
        )
        points = len(getattr(result.trajectory, "points", [])) if result.trajectory is not None else 0
        return {
            "state": "planned" if result.success else "failed",
            "message": result.message,
            "max_error": float(max_error),
            "skip_threshold": threshold,
            "service": service,
            "points": points,
        }

    def _service_available(self) -> bool:
        try:
            available = bool(self._service_client.service_is_ready())
            if not available:
                available = bool(self._service_client.wait_for_service(timeout_sec=0.0))
            return available
        except Exception:
            return False

    @staticmethod
    def _is_number_like(value) -> bool:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return False
        return math.isfinite(number)

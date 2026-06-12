from __future__ import annotations

from dataclasses import dataclass, field

from .trajectory_safety_monitor import evaluate_replay_tracking


@dataclass(frozen=True)
class ReplayRuntimeMonitorConfig:
    enabled: bool
    start_grace_sec: float
    violation_grace_sec: float
    max_tracking_error_rad: float
    max_live_velocity_rad_s: float


@dataclass(frozen=True)
class ReplayRuntimeMonitorDecision:
    should_stop: bool
    status: dict = field(default_factory=dict)


class ReplayRuntimeMonitor:
    """Stateful runtime guard for active teach replay tracking."""

    def __init__(self) -> None:
        self.violation_since: float | None = None
        self.stop_requested = False

    def reset(self) -> None:
        self.violation_since = None
        self.stop_requested = False

    def check(
        self,
        *,
        trajectory,
        started_at: float | None,
        joints: dict,
        now: float,
        config: ReplayRuntimeMonitorConfig,
    ) -> ReplayRuntimeMonitorDecision:
        if not bool(config.enabled) or trajectory is None or started_at is None or self.stop_requested:
            return ReplayRuntimeMonitorDecision(False)
        elapsed = float(now) - float(started_at)
        if elapsed < float(config.start_grace_sec):
            return ReplayRuntimeMonitorDecision(False)
        result = evaluate_replay_tracking(
            trajectory,
            joint_names=tuple(joints.keys()),
            positions=tuple(float(item.get("position", 0.0)) for item in joints.values()),
            velocities=tuple(float(item.get("velocity", 0.0)) for item in joints.values()),
            elapsed_sec=elapsed,
            max_tracking_error_rad=float(config.max_tracking_error_rad),
            max_live_velocity_rad_s=float(config.max_live_velocity_rad_s),
        )
        if result.ok:
            self.violation_since = None
            return ReplayRuntimeMonitorDecision(False)
        if self.violation_since is None:
            self.violation_since = float(now)
            return ReplayRuntimeMonitorDecision(False)
        if float(now) - float(self.violation_since) < float(config.violation_grace_sec):
            return ReplayRuntimeMonitorDecision(False)
        self.stop_requested = True
        return ReplayRuntimeMonitorDecision(
            True,
            {
                "state": "safety_stop",
                "message": f"runtime replay monitor stopped trajectory: {result.message}",
                "runtime_monitor": {
                    "reason": result.reason,
                    "worst_joint": result.worst_joint,
                    "max_tracking_error_rad": result.max_tracking_error_rad,
                    "max_live_velocity_rad_s": result.max_live_velocity_rad_s,
                    "tracking_error": result.reason == "tracking_error",
                    "live_velocity": result.reason == "live_velocity",
                },
                "dry_run": False,
            },
        )

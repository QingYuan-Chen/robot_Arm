from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .teach_recording import estimate_teach_replay, validate_teach_replay_execute_request


@dataclass(frozen=True)
class TeachReplayLimits:
    max_prepared_jump_rad: float
    max_replay_acceleration_rad_s2: float
    max_replay_jerk_rad_s3: float


class TeachReplayCoordinator:
    """Build teach-replay API payloads and keep replay gates in one place."""

    def evaluate_execute_request(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        prepared_quality: dict,
        dry_run_token: dict,
        limits: TeachReplayLimits,
        yellow_max_speed: float,
    ):
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        dry_run_passed = (
            bool(dry_run_token.get("accepted"))
            and str(dry_run_token.get("record_path", "")) == str(info_payload.get("path", ""))
            and str(dry_run_token.get("prepared_risk_level", "")) == str(prepared_quality.get("risk_level", ""))
            and dry_run_token.get("settings") == settings
        )
        return validate_teach_replay_execute_request(
            str(info_payload.get("start_band", "")),
            dry_run_passed=dry_run_passed,
            risk_level=str(quality.get("risk_level", "unknown")),
            prepared_risk_level=str(prepared_quality.get("risk_level", "")) or None,
            prepared_max_jump_rad=prepared_quality.get("max_jump_rad"),
            max_prepared_jump_rad=float(limits.max_prepared_jump_rad),
            retimed_max_acceleration_rad_s2=prepared_quality.get("max_acceleration_rad_s2"),
            max_replay_acceleration_rad_s2=float(limits.max_replay_acceleration_rad_s2),
            retimed_max_jerk_rad_s3=prepared_quality.get("max_jerk_rad_s3"),
            max_replay_jerk_rad_s3=float(limits.max_replay_jerk_rad_s3),
            replay_speed=float(settings["replay_speed"]),
            yellow_max_speed=float(yellow_max_speed),
        )

    def build_dry_run_result(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        decision: Any,
        prepared_payload: dict,
        prepared_record_path: str,
        moveit_align: dict,
        collision_precheck: dict,
        trajectory_points: int,
        limits: TeachReplayLimits,
        target_runtime: str,
        compact_payload: Callable[[dict], dict] | None = None,
    ) -> dict:
        gate_blocked = (
            str(moveit_align.get("state", "")).lower() in ("failed", "unavailable")
            or str(collision_precheck.get("state", "")).lower() in ("collision", "unknown")
        )
        return self._build_replay_result(
            info_payload=info_payload,
            settings=settings,
            accepted=bool(decision.accepted) and not gate_blocked,
            state="blocked" if decision.accepted and gate_blocked else decision.state,
            message=(
                f"{decision.message}; MoveIt/collision precheck blocked real replay"
                if decision.accepted and gate_blocked
                else decision.message
            ),
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=trajectory_points,
            limits=limits,
            target_runtime=target_runtime,
            dry_run=True,
            compact_payload=compact_payload,
        )

    def build_execute_result(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        decision: Any,
        prepared_payload: dict,
        prepared_record_path: str,
        moveit_align: dict,
        collision_precheck: dict,
        trajectory_points: int,
        limits: TeachReplayLimits,
        target_runtime: str,
        compact_payload: Callable[[dict], dict] | None = None,
    ) -> dict:
        return self._build_replay_result(
            info_payload=info_payload,
            settings=settings,
            accepted=bool(decision.accepted),
            state=str(decision.state),
            message=str(decision.message),
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=trajectory_points,
            limits=limits,
            target_runtime=target_runtime,
            dry_run=False,
            compact_payload=compact_payload,
        )

    def _build_replay_result(
        self,
        *,
        info_payload: dict,
        settings: dict[str, float | int],
        accepted: bool,
        state: str,
        message: str,
        prepared_payload: dict,
        prepared_record_path: str,
        moveit_align: dict,
        collision_precheck: dict,
        trajectory_points: int,
        limits: TeachReplayLimits,
        target_runtime: str,
        dry_run: bool,
        compact_payload: Callable[[dict], dict] | None,
    ) -> dict:
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        prepared_quality = (
            prepared_payload.get("after_quality")
            if isinstance(prepared_payload.get("after_quality"), dict)
            else {}
        )
        estimate = estimate_teach_replay(
            samples=int(info_payload.get("samples") or 0),
            record_duration_sec=float(info_payload.get("duration_sec") or 0.0),
            start_band=str(info_payload.get("start_band", "")),
            replay_speed=float(settings["replay_speed"]),
            align_duration=float(settings["align_duration"]),
            align_steps=int(settings["align_steps"]),
            final_hold_sec=float(settings["final_hold_sec"]),
        )
        result = {
            "accepted": bool(accepted),
            "state": str(state),
            "message": str(message),
            "record_path": str(info_payload.get("path", "")),
            "prepared_record_path": prepared_record_path,
            "start_band": str(info_payload.get("start_band", "")),
            "max_error": info_payload.get("max_error"),
            "worst_joint": str(info_payload.get("worst_joint", "")),
            "samples": int(info_payload.get("samples") or 0),
            "trajectory_points": int(trajectory_points or prepared_payload.get("prepared_samples") or estimate["trajectory_points"]),
            "estimated_duration_sec": float(estimate["estimated_duration_sec"]),
            "settings": settings,
            "quality": quality,
            "risk_level": str(quality.get("risk_level", "unknown")),
            "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
            "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
            "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
            "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
            "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
            "max_prepared_jump_rad": float(limits.max_prepared_jump_rad),
            "max_replay_acceleration_rad_s2": float(limits.max_replay_acceleration_rad_s2),
            "max_replay_jerk_rad_s3": float(limits.max_replay_jerk_rad_s3),
            "prepared_replay": prepared_payload,
            "moveit_start_align": moveit_align,
            "collision_precheck": collision_precheck,
            "target_runtime": target_runtime,
            "dry_run": bool(dry_run),
        }
        return compact_payload(result) if compact_payload is not None else result

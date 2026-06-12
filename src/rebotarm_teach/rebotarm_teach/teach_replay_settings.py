from __future__ import annotations

from dataclasses import dataclass

from .teach_recording import compute_auto_align_duration, normalize_teach_replay_settings


@dataclass(frozen=True)
class TeachReplaySettingsProvider:
    replay_speed: float
    align_duration: float
    align_duration_auto: bool
    align_target_speed_rad_s: float
    align_min_duration: float
    align_max_duration: float
    align_steps: int

    def from_payload(self, payload: dict, *, max_error: float | None = None) -> dict[str, float | int]:
        values = payload.get("settings") if isinstance(payload.get("settings"), dict) else {}
        align_duration = self.auto_align_duration(max_error)
        if not self.align_duration_auto:
            align_duration = float(values.get("align_duration", align_duration))
        return normalize_teach_replay_settings(
            replay_speed=float(values.get("replay_speed", self.replay_speed)),
            align_duration=align_duration,
            align_steps=int(values.get("align_steps", self.align_steps)),
            final_hold_sec=1.0,
        )

    def auto_align_duration(self, max_error: float | None) -> float:
        if self.align_duration_auto:
            return compute_auto_align_duration(
                max_error,
                target_speed_rad_s=float(self.align_target_speed_rad_s),
                min_duration_sec=float(self.align_min_duration),
                max_duration_sec=float(self.align_max_duration),
            )
        return float(self.align_duration)

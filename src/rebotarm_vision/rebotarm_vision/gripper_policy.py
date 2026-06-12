from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperPolicyConfig:
    auto_width: bool = True
    auto_effort: bool = True
    default_open_width_m: float = 0.09
    default_close_width_m: float = 0.025
    default_max_effort: float = 0.4
    open_clearance_m: float = 0.0
    close_margin_m: float = 0.012
    min_open_width_m: float = 0.035
    max_open_width_m: float = 0.09
    min_close_width_m: float = 0.006
    max_close_width_m: float = 0.08
    min_effort: float = 0.22
    max_effort: float = 0.60
    max_allowed_width_m: float = 0.082


@dataclass(frozen=True)
class GripperCommand:
    open_width_m: float
    close_width_m: float
    max_effort: float
    allowed: bool = True
    reason: str = "ok"


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def resolve_gripper_command(
    *,
    jaw_width_m: float,
    object_length_m: float = 0.0,
    class_name: str = "",
    config: GripperPolicyConfig | None = None,
) -> GripperCommand:
    cfg = config or GripperPolicyConfig()
    width = max(float(jaw_width_m or 0.0), 0.0)

    if cfg.auto_width and width > float(cfg.max_allowed_width_m):
        return GripperCommand(
            open_width_m=float(cfg.max_open_width_m),
            close_width_m=float(cfg.max_close_width_m),
            max_effort=_clamp(float(cfg.default_max_effort), float(cfg.min_effort), float(cfg.max_effort)),
            allowed=False,
            reason=f"object too wide for gripper: {width:.3f}m > {cfg.max_allowed_width_m:.3f}m",
        )

    if cfg.auto_width and width > 0.0:
        open_width = _clamp(
            width + float(cfg.open_clearance_m),
            float(cfg.min_open_width_m),
            float(cfg.max_open_width_m),
        )
        close_width = _clamp(
            width - float(cfg.close_margin_m),
            float(cfg.min_close_width_m),
            float(cfg.max_close_width_m),
        )
        close_width = min(close_width, open_width)
    else:
        open_width = _clamp(
            float(cfg.default_open_width_m),
            float(cfg.min_open_width_m),
            float(cfg.max_open_width_m),
        )
        close_width = _clamp(
            float(cfg.default_close_width_m),
            float(cfg.min_close_width_m),
            float(cfg.max_close_width_m),
        )

    max_effort = _clamp(float(cfg.default_max_effort), float(cfg.min_effort), float(cfg.max_effort))

    return GripperCommand(
        open_width_m=open_width,
        close_width_m=close_width,
        max_effort=max_effort,
        allowed=True,
        reason="ok",
    )

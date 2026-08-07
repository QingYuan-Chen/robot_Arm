from __future__ import annotations


def clamp_width(width: float, *, min_width: float, max_width: float) -> float:
    lower = min(float(min_width), float(max_width))
    upper = max(float(min_width), float(max_width))
    return min(max(float(width), lower), upper)


def gripper_joint_positions_for_width(
    width: float,
    *,
    min_width: float = 0.0,
    max_width: float = 0.09,
) -> tuple[float, float, float]:
    reached_width = clamp_width(width, min_width=min_width, max_width=max_width)
    return reached_width * 0.5, -reached_width * 0.5, reached_width

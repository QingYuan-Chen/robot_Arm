from __future__ import annotations


def build_joint_limits(
    *,
    joint_names: tuple[str, ...],
    lower_limits: tuple[float, ...],
    upper_limits: tuple[float, ...],
) -> dict[str, tuple[float, float]]:
    if len(joint_names) != len(lower_limits) or len(joint_names) != len(upper_limits):
        raise ValueError(
            "joint_names, lower_limits, and upper_limits must have the same length"
        )
    return {
        name: (float(lower), float(upper))
        for name, lower, upper in zip(joint_names, lower_limits, upper_limits)
    }


def sensor_qos_kwargs() -> dict[str, object]:
    return {
        "depth": 10,
        "reliability": "best_effort",
    }

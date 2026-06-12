from __future__ import annotations


def sensor_qos_kwargs() -> dict[str, object]:
    return {
        "depth": 10,
        "reliability": "best_effort",
    }

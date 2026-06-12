from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class StatusSnapshot:
    joints: dict[str, dict[str, float | int]]
    arm: dict[str, Any]
    teleop: dict[str, Any]


class TeleopStatusStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._joints: dict[str, dict[str, float | int]] = {}
        self._arm: dict[str, Any] = {
            "mode": "",
            "enabled": False,
            "state_machine": "",
            "error_codes": [],
        }
        self._teleop: dict[str, Any] = {
            "status": "idle",
            "recording": "idle",
            "replay": "idle",
        }

    def update_joint_state(
        self,
        *,
        names: tuple[str, ...],
        positions: tuple[float, ...],
        velocities: tuple[float, ...],
        efforts: tuple[float, ...],
    ) -> None:
        with self._lock:
            for index, name in enumerate(names):
                joint = dict(self._joints.get(name, {}))
                joint["position"] = float(positions[index])
                if index < len(velocities):
                    joint["velocity"] = float(velocities[index])
                if index < len(efforts):
                    joint["torque"] = float(efforts[index])
                self._joints[name] = joint

    def update_motor_state(
        self,
        *,
        joint_name: str,
        position: float,
        velocity: float,
        torque: float,
        status_code: int,
    ) -> None:
        with self._lock:
            self._joints[joint_name] = {
                "position": float(position),
                "velocity": float(velocity),
                "torque": float(torque),
                "status_code": int(status_code),
            }

    def update_arm_status(
        self,
        *,
        mode: str,
        enabled: bool,
        state_machine: str,
        error_codes: tuple[str, ...],
    ) -> None:
        with self._lock:
            self._arm = {
                "mode": mode,
                "enabled": bool(enabled),
                "state_machine": state_machine,
                "error_codes": list(error_codes),
            }

    def update_teleop_status(self, key: str, value: Any) -> None:
        with self._lock:
            self._teleop[key] = value

    def snapshot(self) -> StatusSnapshot:
        with self._lock:
            return StatusSnapshot(
                joints={name: dict(data) for name, data in self._joints.items()},
                arm=dict(self._arm),
                teleop=dict(self._teleop),
            )

    def snapshot_dict(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        return {
            "joints": snapshot.joints,
            "arm": snapshot.arm,
            "teleop": snapshot.teleop,
        }


def encode_sse_event(payload: dict[str, Any], *, event: str = "status") -> str:
    data = json.dumps(payload, separators=(",", ":"))
    return f"event: {event}\ndata: {data}\n\n"


def format_angle_readout(radians: float) -> dict[str, str]:
    value = float(radians)
    return {
        "rad": f"{value:.4f}",
        "deg": f"{math.degrees(value):.1f}",
    }


def clamp_preview_value(value: float, lower: float, upper: float) -> float:
    low = float(lower)
    high = float(upper)
    if high < low:
        low, high = high, low
    return min(max(float(value), low), high)

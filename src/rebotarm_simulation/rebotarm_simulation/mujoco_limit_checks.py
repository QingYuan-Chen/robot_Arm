from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

from .mujoco_model_profile import MotorProfile


@dataclass(frozen=True)
class PositionLimitMismatch:
    joint: str
    urdf_lower: float
    urdf_upper: float
    mujoco_lower: float
    mujoco_upper: float


def motor_profile_position_limit_mismatches(
    urdf_path: Path,
    motor_profiles: Iterable[MotorProfile],
    *,
    tolerance: float = 1e-9,
) -> list[PositionLimitMismatch]:
    urdf_limits = load_urdf_position_limits(urdf_path)
    mismatches: list[PositionLimitMismatch] = []
    for profile in motor_profiles:
        if profile.joint not in urdf_limits:
            continue
        urdf_lower, urdf_upper = urdf_limits[profile.joint]
        mujoco_lower, mujoco_upper = _parse_ctrlrange(profile.ctrlrange)
        if abs(urdf_lower - mujoco_lower) > tolerance or abs(urdf_upper - mujoco_upper) > tolerance:
            mismatches.append(
                PositionLimitMismatch(
                    joint=profile.joint,
                    urdf_lower=urdf_lower,
                    urdf_upper=urdf_upper,
                    mujoco_lower=mujoco_lower,
                    mujoco_upper=mujoco_upper,
                )
            )
    return mismatches


def load_urdf_position_limits(urdf_path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(urdf_path).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        lower = limit.get("lower")
        upper = limit.get("upper")
        if lower is None or upper is None:
            continue
        limits[name] = (float(lower), float(upper))
    return limits


def _parse_ctrlrange(value: str) -> tuple[float, float]:
    parts = [float(part) for part in str(value).split()]
    if len(parts) != 2:
        raise ValueError(f"expected two ctrlrange values, got: {value}")
    return parts[0], parts[1]

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


DEFAULT_GRIPPER_LIMITS_M = (0.0, 0.09)


def clamp_gripper_opening(position_m: float, limits_m: tuple[float, float]) -> float:
    lower, upper = float(limits_m[0]), float(limits_m[1])
    if upper < lower:
        lower, upper = upper, lower
    value = min(max(float(position_m), lower), upper)
    return value - lower


def gripper_opening_to_finger_joint_positions(
    position_m: float,
    limits_m: tuple[float, float] = DEFAULT_GRIPPER_LIMITS_M,
) -> tuple[float, float]:
    half_opening = 0.5 * clamp_gripper_opening(position_m, limits_m)
    return half_opening, -half_opening


def rewrite_package_mesh_uris(urdf_text: str, *, mesh_route: str = "meshes") -> str:
    prefix = "package://rebotarm_bringup/description/meshes/"
    route = mesh_route.rstrip("/")
    return str(urdf_text).replace(prefix, f"{route}/")


def safe_mesh_path(mesh_dir: Path, requested_name: str) -> Path | None:
    raw_name = str(requested_name)
    if "/" in raw_name or "\\" in raw_name:
        return None
    name = Path(raw_name).name
    if not name or name in {".", ".."}:
        return None
    path = mesh_dir / name
    if not path.exists() or not path.is_file():
        return None
    return path


def load_urdf_joint_limits(urdf_path: Path, joint_names: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    wanted = set(joint_names)
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = str(joint.attrib.get("name", ""))
        if name not in wanted:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        if "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        lower = float(limit.attrib["lower"])
        upper = float(limit.attrib["upper"])
        if upper < lower:
            lower, upper = upper, lower
        limits[name] = (lower, upper)
    return limits


def merge_joint_limits(
    *,
    joint_names: tuple[str, ...],
    fallback_limits: dict[str, tuple[float, float]],
    preferred_limits: dict[str, tuple[float, float]],
) -> dict[str, tuple[float, float]]:
    merged: dict[str, tuple[float, float]] = {}
    for name in joint_names:
        merged[name] = preferred_limits.get(name, fallback_limits[name])
    return merged


def load_gripper_limits(config: dict[str, Any] | None) -> tuple[float, float]:
    if not isinstance(config, dict):
        return DEFAULT_GRIPPER_LIMITS_M

    candidates = (
        ("min_position_m", "max_position_m"),
        ("min_opening_m", "max_opening_m"),
        ("closed_position_m", "open_position_m"),
    )
    for lower_key, upper_key in candidates:
        if lower_key in config and upper_key in config:
            lower = float(config[lower_key])
            upper = float(config[upper_key])
            if upper < lower:
                lower, upper = upper, lower
            return lower, upper

    gripper_items = config.get("gripper")
    if isinstance(gripper_items, list) and gripper_items:
        first = gripper_items[0]
        if isinstance(first, dict):
            nested = load_gripper_limits(first)
            if nested != DEFAULT_GRIPPER_LIMITS_M:
                return nested

    return DEFAULT_GRIPPER_LIMITS_M


def load_moveit_velocity_limits(path: Path, joint_names: tuple[str, ...]) -> dict[str, float]:
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    joint_limits = data.get("joint_limits", {})
    limits: dict[str, float] = {}
    for name in joint_names:
        entry = joint_limits.get(name, {})
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("has_velocity_limits", False)):
            continue
        try:
            value = float(entry["max_velocity"])
        except (KeyError, TypeError, ValueError):
            continue
        if value > 0.0:
            limits[name] = value
    return limits


def merge_velocity_limits(
    *,
    joint_names: tuple[str, ...],
    default_limit: float,
    preferred_limits: dict[str, float],
) -> dict[str, float]:
    return {
        name: float(preferred_limits.get(name, default_limit))
        for name in joint_names
    }

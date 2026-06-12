from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class HandeyeConfig:
    parent_frame: str
    child_frame: str
    translation_x: float
    translation_y: float
    translation_z: float
    rotation_x: float
    rotation_y: float
    rotation_z: float
    rotation_w: float

    def as_static_transform_arguments(self) -> list[str]:
        return [
            str(self.translation_x),
            str(self.translation_y),
            str(self.translation_z),
            str(self.rotation_x),
            str(self.rotation_y),
            str(self.rotation_z),
            str(self.rotation_w),
            self.parent_frame,
            self.child_frame,
        ]


def _required_mapping(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"handeye config missing mapping: {key}")
    return value


def _required_float(data: dict[str, Any], key: str) -> float:
    if key not in data:
        raise ValueError(f"handeye config missing value: {key}")
    return float(data[key])


def load_handeye_config(path: str | Path) -> HandeyeConfig:
    config_path = Path(path)
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError("handeye config root must be a mapping")

    handeye = _required_mapping(data, "handeye")
    translation = _required_mapping(handeye, "translation")
    rotation = _required_mapping(handeye, "rotation")

    parent_frame = str(handeye.get("parent_frame", "")).strip()
    child_frame = str(handeye.get("child_frame", "")).strip()
    if not parent_frame or not child_frame:
        raise ValueError("handeye config requires parent_frame and child_frame")

    return HandeyeConfig(
        parent_frame=parent_frame,
        child_frame=child_frame,
        translation_x=_required_float(translation, "x"),
        translation_y=_required_float(translation, "y"),
        translation_z=_required_float(translation, "z"),
        rotation_x=_required_float(rotation, "x"),
        rotation_y=_required_float(rotation, "y"),
        rotation_z=_required_float(rotation, "z"),
        rotation_w=_required_float(rotation, "w"),
    )

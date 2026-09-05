from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .models import EXPECTED_ARM_SIGNS, MappingConfig


@dataclass(frozen=True)
class LiveFollowConfig:
    mapping_acceptance: str
    control_rate_hz: float
    default_speed_rad_s: float
    max_speed_rad_s: float
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float
    max_tracking_error_rad: float
    tracking_error_grace_s: float
    leader_stale_timeout_s: float
    follower_stale_timeout_s: float
    safe_home_rad: tuple[float, ...]
    safe_home_tolerance_rad: float
    safe_home_velocity_tolerance_rad_s: float
    safe_home_stable_s: float
    safe_home_timeout_s: float
    deadline_miss_limit: int

    @property
    def follow_ready_rad(self) -> tuple[float, ...]:
        """跟随就绪姿态与网页停放安全位共用同一配置值。"""

        return self.safe_home_rad


_FLOAT_FIELDS = (
    "control_rate_hz",
    "default_speed_rad_s",
    "max_speed_rad_s",
    "max_acceleration_rad_s2",
    "max_jerk_rad_s3",
    "max_tracking_error_rad",
    "tracking_error_grace_s",
    "leader_stale_timeout_s",
    "follower_stale_timeout_s",
    "safe_home_tolerance_rad",
    "safe_home_velocity_tolerance_rad_s",
    "safe_home_stable_s",
    "safe_home_timeout_s",
)


def _positive_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是大于零的有限数值")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} 必须大于零且为有限数值")
    return result


def load_live_follow_config(path: Path) -> LiveFollowConfig:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("实时跟随配置 schema_version 必须为 1")
    if data.get("mapping_acceptance") != "rviz_visual_accepted":
        raise ValueError("映射验收级别必须是 RViz 目视确认")
    if "joint_margin_rad" in data:
        raise ValueError(
            "joint_margin_rad 已移除；J1～J6 直接使用网页遥操作关节边界"
        )
    if "max_relative_delta_rad" in data:
        raise ValueError(
            "max_relative_delta_rad 已移除；J1～J6 只使用网页遥操作关节边界"
        )

    values: dict[str, float] = {}
    for name in _FLOAT_FIELDS:
        if name not in data:
            raise ValueError(f"实时跟随配置缺少 {name}")
        values[name] = _positive_float(data[name], name)
    if values["max_speed_rad_s"] != 1.5:
        raise ValueError("速度硬上限必须为 1.5 rad/s")
    if values["default_speed_rad_s"] > values["max_speed_rad_s"]:
        raise ValueError("默认速度不得超过速度硬上限")

    safe_home_raw = data.get("safe_home_rad")
    if not isinstance(safe_home_raw, list) or len(safe_home_raw) != 6:
        raise ValueError("safe_home_rad 必须包含六个关节")
    safe_home = tuple(float(item) for item in safe_home_raw)
    if not all(math.isfinite(item) for item in safe_home):
        raise ValueError("safe_home_rad 必须是有限数值")

    miss_limit = data.get("deadline_miss_limit")
    if isinstance(miss_limit, bool) or not isinstance(miss_limit, int) or miss_limit < 1:
        raise ValueError("deadline_miss_limit 必须是正整数")

    return LiveFollowConfig(
        mapping_acceptance=str(data["mapping_acceptance"]),
        safe_home_rad=safe_home,
        deadline_miss_limit=miss_limit,
        **values,
    )


def validate_live_mapping(mapping: MappingConfig, live: LiveFollowConfig) -> None:
    signs = tuple(joint.sign for joint in mapping.arm_joints)
    if signs != EXPECTED_ARM_SIGNS:
        raise ValueError("实时跟随方向符号与已确认映射不一致")
    scales = tuple(joint.scale for joint in mapping.arm_joints)
    if scales != (1.0,) * 6:
        raise ValueError("实时跟随比例必须全部为 1.0")
    for index, (joint, home) in enumerate(
        zip(mapping.arm_joints, live.safe_home_rad, strict=True),
        start=1,
    ):
        if not float(joint.lower_rad) <= home <= float(joint.upper_rad):
            raise ValueError(f"joint{index} safe_home 超出真实关节限位")

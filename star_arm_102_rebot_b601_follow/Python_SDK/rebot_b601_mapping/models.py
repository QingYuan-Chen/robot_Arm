from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


EXPECTED_LEADER_IDS = (0, 1, 2, 3, 4, 5, 6)
EXPECTED_FOLLOWER_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)
EXPECTED_ARM_SIGNS = (-1, -1, 1, 1, 1, -1)


@dataclass(frozen=True)
class JointSpec:
    leader_id: int
    follower_name: str
    sign: int | None
    scale: float | None
    lower_rad: float | None
    upper_rad: float | None
    verified: bool


@dataclass(frozen=True)
class Thresholds:
    max_sample_age_s: float
    baseline_max_velocity_rad_s: float
    min_direction_delta_rad: float
    other_joint_max_delta_rad: float
    sign_window_size: int


@dataclass(frozen=True)
class MappingConfig:
    leader_ids: tuple[int, ...]
    arm_joints: tuple[JointSpec, ...]
    gripper: JointSpec
    thresholds: Thresholds


@dataclass(frozen=True)
class LeaderSample:
    timestamp_s: float
    angles_deg: tuple[float, ...]


@dataclass(frozen=True)
class MotorFeedback:
    name: str
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    status_code: int


@dataclass(frozen=True)
class FollowerSample:
    timestamp_s: float
    motors: tuple[MotorFeedback, ...]


@dataclass(frozen=True)
class Baseline:
    captured_at_s: float
    leader_angles_deg: tuple[float, ...]
    follower_positions_rad: tuple[float, ...]


@dataclass(frozen=True)
class MappingResult:
    positions_rad: tuple[float, ...]
    leader_deltas_rad: tuple[float, ...]


@dataclass(frozen=True)
class DirectionEvidence:
    follower_name: str
    observed_at_s: float
    leader_delta_rad: float
    follower_delta_rad: float
    inferred_sign: int
    candidate_sign: int
    consistent: bool
    confirmed: bool
    verified: bool


def _require_dict(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} 必须是 JSON 对象")
    return value


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须是有限数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} 阈值必须是有限数值")
    return result


def _parse_joint(raw: Any, index: int) -> JointSpec:
    item = _require_dict(raw, f"mapping[{index}]")
    try:
        leader_id = int(item["leader_id"])
        follower_name = str(item["follower_name"])
        sign_raw = item["sign"]
        scale_raw = item["scale"]
        verified = item["verified"]
    except KeyError as exc:
        raise ValueError(f"mapping[{index}] 缺少字段 {exc.args[0]}") from exc

    if not isinstance(verified, bool):
        raise ValueError(f"mapping[{index}].verified 必须是布尔值")

    sign = None if sign_raw is None else int(sign_raw)
    scale = None if scale_raw is None else _finite_number(scale_raw, "比例系数")
    lower = (
        None
        if item.get("lower_rad") is None
        else _finite_number(item["lower_rad"], "关节下限")
    )
    upper = (
        None
        if item.get("upper_rad") is None
        else _finite_number(item["upper_rad"], "关节上限")
    )
    if lower is not None and upper is not None and lower >= upper:
        raise ValueError(f"{follower_name} 关节下限必须小于上限")
    return JointSpec(
        leader_id=leader_id,
        follower_name=follower_name,
        sign=sign,
        scale=scale,
        lower_rad=lower,
        upper_rad=upper,
        verified=verified,
    )


def load_mapping_config(path: Path) -> MappingConfig:
    """加载并严格验证从臂坐标方向映射配置。"""

    data = _require_dict(json.loads(Path(path).read_text(encoding="utf-8")), "配置根节点")
    if data.get("schema_version") != 1:
        raise ValueError("schema_version 必须为 1")

    leader_ids_raw = data.get("leader_ids")
    if not isinstance(leader_ids_raw, list):
        raise ValueError("leader_ids 必须是数组")
    leader_ids = tuple(int(value) for value in leader_ids_raw)
    if leader_ids != EXPECTED_LEADER_IDS:
        raise ValueError("引导臂 ID 必须严格为 0..6 且不得重复")

    mapping_raw = data.get("mapping")
    if not isinstance(mapping_raw, list) or len(mapping_raw) != 7:
        raise ValueError("mapping 必须包含六个机械臂关节和一个夹爪")
    joints = tuple(_parse_joint(item, index) for index, item in enumerate(mapping_raw))
    if tuple(joint.leader_id for joint in joints) != EXPECTED_LEADER_IDS:
        raise ValueError("引导臂 ID 顺序错误或存在重复")
    if tuple(joint.follower_name for joint in joints) != EXPECTED_FOLLOWER_NAMES:
        raise ValueError("从臂关节顺序必须严格为 joint1..joint6、gripper")

    arm_joints = joints[:6]
    if tuple(joint.sign for joint in arm_joints) != EXPECTED_ARM_SIGNS:
        raise ValueError("J1～J6 候选符号必须为 [-1,-1,+1,+1,+1,-1]")
    for joint in arm_joints:
        if joint.scale is None or joint.scale <= 0.0:
            raise ValueError(f"{joint.follower_name} 比例系数必须为正有限数值")
        if joint.lower_rad is None or joint.upper_rad is None:
            raise ValueError(f"{joint.follower_name} 必须配置从臂软限位")

    gripper = joints[6]
    if gripper.sign is not None or gripper.scale is not None or gripper.verified:
        raise ValueError("夹爪必须保持未验证，sign 和 scale 必须为 null")

    thresholds_raw = _require_dict(data.get("thresholds"), "thresholds")
    try:
        thresholds = Thresholds(
            max_sample_age_s=_finite_number(
                thresholds_raw["max_sample_age_s"], "max_sample_age_s"
            ),
            baseline_max_velocity_rad_s=_finite_number(
                thresholds_raw["baseline_max_velocity_rad_s"],
                "baseline_max_velocity_rad_s",
            ),
            min_direction_delta_rad=_finite_number(
                thresholds_raw["min_direction_delta_rad"],
                "min_direction_delta_rad",
            ),
            other_joint_max_delta_rad=_finite_number(
                thresholds_raw["other_joint_max_delta_rad"],
                "other_joint_max_delta_rad",
            ),
            sign_window_size=int(thresholds_raw["sign_window_size"]),
        )
    except KeyError as exc:
        raise ValueError(f"thresholds 缺少字段 {exc.args[0]}") from exc
    numeric_thresholds = (
        thresholds.max_sample_age_s,
        thresholds.baseline_max_velocity_rad_s,
        thresholds.min_direction_delta_rad,
        thresholds.other_joint_max_delta_rad,
    )
    if any(value <= 0.0 for value in numeric_thresholds):
        raise ValueError("所有数值阈值必须大于零")
    if thresholds.sign_window_size < 1:
        raise ValueError("sign_window_size 必须是正整数")

    return MappingConfig(
        leader_ids=leader_ids,
        arm_joints=arm_joints,
        gripper=gripper,
        thresholds=thresholds,
    )

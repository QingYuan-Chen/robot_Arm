from __future__ import annotations

import json
from pathlib import Path

import pytest

from rebot_b601_mapping.hardware_specs import (
    ARM_MOTOR_SPECS,
    GRIPPER_SPEC,
    POS_VEL_GAINS_BY_NAME,
)
from rebot_b601_mapping.live_config import (
    load_live_follow_config,
    validate_live_mapping,
)
from rebot_b601_mapping.models import load_mapping_config


ROOT = Path(__file__).parents[1]
LIVE_CONFIG = ROOT / "live_follow.example.json"
MAPPING_CONFIG = ROOT / "mapping.example.json"


def _copy_with_change(tmp_path: Path, key: str, value) -> Path:
    data = json.loads(LIVE_CONFIG.read_text(encoding="utf-8"))
    data[key] = value
    path = tmp_path / "live.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def test_shared_hardware_specs_keep_arm_and_gripper_separate() -> None:
    assert [(item.name, item.motor_id, item.feedback_id, item.model) for item in ARM_MOTOR_SPECS] == [
        ("joint1", 0x01, 0x11, "4340P"),
        ("joint2", 0x02, 0x12, "4340P"),
        ("joint3", 0x03, 0x13, "4340P"),
        ("joint4", 0x04, 0x14, "4310"),
        ("joint5", 0x05, 0x15, "4310"),
        ("joint6", 0x06, 0x16, "4310"),
    ]
    assert (GRIPPER_SPEC.name, GRIPPER_SPEC.motor_id, GRIPPER_SPEC.feedback_id) == (
        "gripper",
        0x07,
        0x17,
    )
    assert POS_VEL_GAINS_BY_NAME["joint1"].pos_kp == 150.0
    assert POS_VEL_GAINS_BY_NAME["joint6"].pos_kp == 50.0


def test_example_config_matches_approved_live_limits() -> None:
    config = load_live_follow_config(LIVE_CONFIG)

    assert config.control_rate_hz == 50.0
    assert config.default_speed_rad_s == 0.5
    assert config.max_speed_rad_s == 1.5
    assert config.max_acceleration_rad_s2 == 5.0
    assert config.max_jerk_rad_s3 == 20.0
    assert config.deadline_miss_limit == 3
    assert config.safe_home_rad == pytest.approx(
        (
            -1.549363136291504,
            0.01659393310546875,
            -0.02002716064453125,
            -0.00858306884765625,
            0.10395240783691406,
            0.00133514404296875,
        )
    )
    validate_live_mapping(load_mapping_config(MAPPING_CONFIG), config)


def test_live_config_loads_without_independent_joint_margin(tmp_path: Path) -> None:
    data = json.loads(LIVE_CONFIG.read_text(encoding="utf-8"))
    data.pop("joint_margin_rad", None)
    path = tmp_path / "live-with-web-limits.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    config = load_live_follow_config(path)

    assert config.safe_home_rad == pytest.approx(
        (
            -1.549363136291504,
            0.01659393310546875,
            -0.02002716064453125,
            -0.00858306884765625,
            0.10395240783691406,
            0.00133514404296875,
        )
    )


def test_live_config_rejects_removed_joint_margin(tmp_path: Path) -> None:
    data = json.loads(LIVE_CONFIG.read_text(encoding="utf-8"))
    data["joint_margin_rad"] = 0.02
    path = tmp_path / "live-with-removed-margin.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="joint_margin_rad 已移除"):
        load_live_follow_config(path)


def test_live_config_loads_without_relative_baseline_limit(tmp_path: Path) -> None:
    data = json.loads(LIVE_CONFIG.read_text(encoding="utf-8"))
    data.pop("max_relative_delta_rad", None)
    path = tmp_path / "live-without-relative-baseline-limit.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    config = load_live_follow_config(path)

    assert config.max_speed_rad_s == 1.5


def test_live_config_rejects_removed_relative_baseline_limit(tmp_path: Path) -> None:
    data = json.loads(LIVE_CONFIG.read_text(encoding="utf-8"))
    data["max_relative_delta_rad"] = 1.5
    path = tmp_path / "live-with-removed-relative-baseline-limit.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="max_relative_delta_rad 已移除"):
        load_live_follow_config(path)


def test_follow_ready_and_safe_home_share_one_web_pose() -> None:
    config = load_live_follow_config(LIVE_CONFIG)

    assert config.follow_ready_rad is config.safe_home_rad


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("default_speed_rad_s", 1.6, "默认速度"),
        ("max_speed_rad_s", 1.6, "硬上限"),
        ("leader_stale_timeout_s", 0.0, "大于零"),
        ("mapping_acceptance", "hardware_verified", "RViz"),
        ("safe_home_rad", [0.0] * 5, "六个"),
        ("deadline_miss_limit", 0, "正整数"),
    ],
)
def test_live_config_rejects_unapproved_values(
    tmp_path: Path,
    field: str,
    value,
    message: str,
) -> None:
    path = _copy_with_change(tmp_path, field, value)

    with pytest.raises(ValueError, match=message):
        load_live_follow_config(path)


def test_live_mapping_rejects_wrong_sign_or_scale(tmp_path: Path) -> None:
    data = json.loads(MAPPING_CONFIG.read_text(encoding="utf-8"))
    data["mapping"][3]["scale"] = 0.5
    path = tmp_path / "mapping.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="比例"):
        validate_live_mapping(
            load_mapping_config(path),
            load_live_follow_config(LIVE_CONFIG),
        )


def test_live_mapping_rejects_safe_home_outside_true_joint_limits(
    tmp_path: Path,
) -> None:
    path = _copy_with_change(
        tmp_path,
        "safe_home_rad",
        [
            -1.549363136291504,
            0.021,
            -0.02002716064453125,
            -0.00858306884765625,
            0.10395240783691406,
            0.00133514404296875,
        ],
    )

    with pytest.raises(ValueError, match="joint2 safe_home.*真实关节限位"):
        validate_live_mapping(
            load_mapping_config(MAPPING_CONFIG),
            load_live_follow_config(path),
        )

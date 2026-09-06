from __future__ import annotations

import json
from pathlib import Path

import pytest

from rebot_b601_mapping.models import load_mapping_config


EXAMPLE_PATH = Path(__file__).parents[1] / "mapping.example.json"


def _write_config(tmp_path: Path, mutate=None) -> Path:
    data = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))
    if mutate is not None:
        mutate(data)
    path = tmp_path / "mapping.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def test_load_mapping_config_preserves_follower_coordinate_authority() -> None:
    config = load_mapping_config(EXAMPLE_PATH)

    assert config.leader_ids == (0, 1, 2, 3, 4, 5, 6)
    assert [joint.follower_name for joint in config.arm_joints] == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]
    assert [joint.sign for joint in config.arm_joints] == [-1, -1, 1, 1, 1, -1]
    assert all(joint.scale == 1.0 for joint in config.arm_joints)
    assert config.gripper.leader_id == 6
    assert config.gripper.follower_name == "gripper"
    assert config.gripper.sign is None
    assert config.gripper.scale is None
    assert config.gripper.verified is False


def test_load_mapping_config_rejects_verified_gripper(tmp_path: Path) -> None:
    path = _write_config(
        tmp_path,
        lambda data: data["mapping"][6].update({"verified": True}),
    )

    with pytest.raises(ValueError, match="夹爪必须保持未验证"):
        load_mapping_config(path)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: data["mapping"][1].update({"follower_name": "joint1"}),
            "从臂关节顺序",
        ),
        (
            lambda data: data["mapping"][1].update({"leader_id": 0}),
            "引导臂 ID",
        ),
        (
            lambda data: data["mapping"][0].update({"sign": 0}),
            "候选符号",
        ),
        (
            lambda data: data["mapping"][0].update({"scale": 0.0}),
            "比例系数",
        ),
        (
            lambda data: data["thresholds"].update({"max_sample_age_s": float("inf")}),
            "阈值",
        ),
    ],
)
def test_load_mapping_config_rejects_unsafe_structure(
    tmp_path: Path,
    mutate,
    message: str,
) -> None:
    path = _write_config(tmp_path, mutate)

    with pytest.raises(ValueError, match=message):
        load_mapping_config(path)

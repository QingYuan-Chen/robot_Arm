from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "scripts" / "p1_upstream_mujoco"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def test_normalize_model_reports_dimensions_and_actuator_names(tmp_path: Path) -> None:
    pytest.importorskip("mujoco")

    from compare_baselines import normalize_model

    xml = tmp_path / "model.xml"
    xml.write_text(
        "<mujoco><worldbody><body><inertial pos='0 0 0' mass='1' diaginertia='1 1 1'/><joint name='joint1'/></body></worldbody>"
        "<actuator><position name='joint1' joint='joint1'/></actuator></mujoco>",
        encoding="utf-8",
    )
    report = normalize_model(xml)

    assert report["nq"] == 1
    assert report["nv"] == 1
    assert report["nu"] == 1
    assert report["njnt"] == 1
    assert report["actuator_names"] == ["joint1"]


def test_compare_records_preserves_backend_and_mismatch_details() -> None:
    from compare_baselines import compare_records

    current = {"backend": "current", "model": {"nu": 7}, "health": {"ok": True}}
    upstream = {"backend": "upstream", "model": {"nu": 8}, "health": {"ok": True}}

    result = compare_records(current, upstream)

    assert result["backends"] == ["current", "upstream"]
    assert result["differences"]["model.nu"] == {"current": 7, "upstream": 8}
    assert result["differences"]["health.ok"] == {"current": True, "upstream": True}


def test_common_command_contract_orders_arm_targets_and_gripper() -> None:
    from compare_baselines import common_command_contract

    contract = common_command_contract(
        {"joint3": -0.3, "joint1": 0.7, "joint6": 1.1, "joint2": -0.5, "joint5": 0.2, "joint4": 0.4},
        gripper_width=0.04,
    )

    assert contract == {
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        "targets": [0.7, -0.5, -0.3, 0.4, 0.2, 1.1],
        "gripper_width": 0.04,
    }


def test_common_command_contract_rejects_unknown_or_missing_joints() -> None:
    from compare_baselines import common_command_contract

    with pytest.raises(ValueError, match="exactly the arm joints"):
        common_command_contract({"joint1": 0.1})
    with pytest.raises(ValueError, match="unknown arm joints"):
        common_command_contract(
            {
                "joint1": 0.1,
                "joint2": 0.1,
                "joint3": 0.1,
                "joint4": 0.1,
                "joint5": 0.1,
                "joint6": 0.1,
                "joint7": 0.1,
            }
        )

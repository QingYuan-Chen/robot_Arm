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

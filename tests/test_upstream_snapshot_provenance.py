from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT / "third_party" / "robotarm_ros2_mujoco_snapshot"


def test_upstream_snapshot_is_pinned_and_outside_default_src() -> None:
    manifest = json.loads((SNAPSHOT / "UPSTREAM_PROVENANCE.json").read_text(encoding="utf-8"))

    assert manifest["remote"] == "https://github.com/huangbinai/robotarm_ros2.git"
    assert manifest["branch"] == "main"
    assert manifest["commit"] == "fb28dcdd358b45de79eb47adfb333e2e94e9d5b4"
    assert (SNAPSHOT / "src/rebotarm_simulation/package.xml").is_file()
    assert not (ROOT / "src/rebotarm_simulation_upstream").exists()


def test_upstream_snapshot_manifest_records_license_and_file_hashes() -> None:
    manifest = json.loads((SNAPSHOT / "UPSTREAM_PROVENANCE.json").read_text(encoding="utf-8"))

    assert manifest["license_status"] == "AUTHORIZED_BY_UPSTREAM_OWNER"
    assert manifest["license_evidence"]["package_xml"] == "Apache-2.0"
    assert manifest["license_evidence"]["root_license_file"] is False
    assert manifest["sha256"]
    assert "src/rebotarm_simulation/rebotarm_simulation/mujoco_health.py" in manifest["sha256"]

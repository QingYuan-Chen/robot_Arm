from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ACTIVE = ROOT / "src" / "rebotarm_simulation"
ARCHIVE = ROOT / "third_party" / "rebotarm_simulation_current_baseline"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_current_simulation_source_archive_manifest_is_present_and_self_consistent():
    manifest_path = ARCHIVE / "ARCHIVE_MANIFEST.json"
    assert manifest_path.is_file()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["source"] == "src/rebotarm_simulation before upstream swap"
    assert manifest["files"]

    for entry in manifest["files"]:
        path = ARCHIVE / entry["path"]
        assert path.is_file(), entry["path"]
        assert path.stat().st_size == entry["size"], entry["path"]
        assert _sha256(path) == entry["sha256"], entry["path"]


def test_active_source_contains_upstream_core_without_current_runtime_entrypoint():
    setup_text = (ACTIVE / "setup.py").read_text(encoding="utf-8")
    package_text = (ACTIVE / "package.xml").read_text(encoding="utf-8")

    assert (ACTIVE / "models/rebotarm/robot.xml").is_file()
    assert (ACTIVE / "models/rebotarm/scene.xml").is_file()
    assert (ACTIVE / "rebotarm_simulation/mujoco_sim.py").is_file()
    assert (ACTIVE / "rebotarm_simulation/mujoco_ros_node.py").is_file()
    assert "rebotarm_mujoco_node = rebotarm_simulation.mujoco_ros_node:main" in setup_text
    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" not in setup_text
    assert "rebotarm_upstream_mujoco_node = rebotarm_simulation.upstream_backend:main" not in setup_text
    assert "<exec_depend>ament_index_python</exec_depend>" in package_text


def test_moveit_sim_wrapper_is_upstream_only():
    launch_text = (ACTIVE / "launch/mujoco_moveit_sim.launch.py").read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_node"' in launch_text
    assert "simulation_backend" not in launch_text
    assert "rebotarm_mujoco_adapter" not in launch_text
    assert "current_condition" not in launch_text
    assert '"use_fake_joint_states": "false"' in launch_text


def test_archived_current_source_remains_available_but_inactive():
    archive_setup = (ARCHIVE / "setup.py").read_text(encoding="utf-8")
    active_setup = (ACTIVE / "setup.py").read_text(encoding="utf-8")

    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" in archive_setup
    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" not in active_setup

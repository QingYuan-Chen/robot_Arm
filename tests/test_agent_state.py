from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "Agent/update_state.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("rebotarm_agent_update_state", MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_agent_phase_weights_and_checklists_are_valid() -> None:
    module = _load_module()
    phases = module.parse_phases()

    assert [phase["id"] for phase in phases] == [
        "P0",
        "P1",
        "P2",
        "P3",
        "P4",
        "P5",
        "P6",
    ]
    assert sum(phase["weight"] for phase in phases) == 100
    assert all(phase["total_items"] > 0 for phase in phases)
    assert all(
        0 <= phase["completed_items"] <= phase["total_items"]
        for phase in phases
    )
    assert all(0.0 <= phase["completion_percent"] <= 100.0 for phase in phases)


def test_generated_agent_state_has_required_live_fields() -> None:
    state = json.loads((ROOT / "Agent/STATE.json").read_text(encoding="utf-8"))

    assert state["schema_version"] == 1
    assert state["plan_source"] == "新项目规划.md"
    assert 0.0 <= state["overall_completion_percent"] <= 100.0
    assert len(state["phases"]) == 7
    first_incomplete = next(
        (phase for phase in state["phases"] if phase["completion_percent"] < 100.0),
        None,
    )
    if first_incomplete is not None:
        assert state["active_phase"].startswith(first_incomplete["id"])
    else:
        assert state["overall_completion_percent"] == 100.0
    module = _load_module()
    assert state["active_phase"] == module.parse_active_phase()
    assert state["blockers"] == module.parse_section_bullets(module.MEMORY, "## 当前阻塞")
    assert state["next_actions"]
    assert state["git"]["branch"]
    assert state["git"]["head"]

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "docs" / "mujoco_gap_matrix.md"


def test_mujoco_gap_matrix_has_auditable_p1_capability_rows():
    text = MATRIX.read_text(encoding="utf-8")

    required_headers = (
        "| Capability | Current evidence | Upstream observation | Decision | Owner | Next action |",
        "|---|---|---|---|---|---|",
    )
    for header in required_headers:
        assert header in text

    required_capabilities = (
        "model generation and packaging",
        "headless smoke and metrics",
        "ROS 2 trajectory adapter",
        "grasp-scene keyframe",
        "timeout and cancellation",
        "trajectory limit consistency",
        "tracking, collision, contact, and calibration",
        "source authorization and environment",
    )
    for capability in required_capabilities:
        assert f"| {capability} |" in text

    assert "defer" in text.lower()
    assert "missing" in text.lower()
    assert "reBotArm_develop_hjx" in text
    assert "copy" in text.lower()

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools/setup_motorbridge_fresh_feedback.py"
SPEC = importlib.util.spec_from_file_location(
    "setup_motorbridge_fresh_feedback",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
SETUP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SETUP)


def _module_with_contract(
    *,
    method_present: bool,
    feedback_sequence: bool | None,
    version: str = "0.4.6+rebotarm.1",
) -> SimpleNamespace:
    motor = type("Motor", (), {})
    if method_present:
        motor.get_state_with_sequence = lambda self: (None, 0)

    features = {}
    if feedback_sequence is not None:
        features["feedback_sequence"] = feedback_sequence
    return SimpleNamespace(
        __version__=version,
        Motor=motor,
        abi_capabilities=lambda: {"features": features},
    )


def test_runtime_contract_rejects_missing_sequence_method() -> None:
    module = _module_with_contract(
        method_present=False,
        feedback_sequence=True,
    )

    with pytest.raises(RuntimeError, match="Motor.get_state_with_sequence"):
        SETUP.validate_runtime_contract(module)


def test_runtime_contract_rejects_missing_sequence_capability() -> None:
    module = _module_with_contract(
        method_present=True,
        feedback_sequence=None,
    )

    with pytest.raises(RuntimeError, match="features.feedback_sequence"):
        SETUP.validate_runtime_contract(module)


def test_runtime_contract_accepts_patched_api_and_capability() -> None:
    module = _module_with_contract(
        method_present=True,
        feedback_sequence=True,
    )

    assert SETUP.validate_runtime_contract(module) is None


def test_runtime_contract_rejects_unexpected_package_version() -> None:
    module = _module_with_contract(
        method_present=True,
        feedback_sequence=True,
        version="0.4.6",
    )

    with pytest.raises(RuntimeError, match="0.4.6\\+rebotarm.1"):
        SETUP.validate_runtime_contract(module)

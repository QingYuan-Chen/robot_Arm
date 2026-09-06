from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "src/rebotarm_vision/rebotarm_vision/single_bottle_grasp.py"
PROFILE_PATH = ROOT / "src/rebotarm_vision/config/single_bottle_grasp.yaml"


def _load_runner_module():
    spec = importlib.util.spec_from_file_location(
        "single_bottle_grasp_profile_test", RUNNER_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_single_bottle_profile_pins_accepted_run_parameters() -> None:
    payload = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))

    assert payload == {
        "schema_version": 1,
        "feature_arguments": {
            "namespace": "rebotarm",
            "runs": 1,
            "plan_timeout_sec": 45.0,
            "max_plan_age_sec": 1.5,
            "pregrasp_sec": 10.0,
            "approach_sec": 3.0,
            "hold_sec": 3.0,
            "return_sec": 10.0,
            "gripper_open_m": 0.080,
            "gripper_open_max_effort": 1.5,
            "grasp_close_force": 1.0,
            "grasp_hold_force": 0.4,
            "grasp_close_timeout_sec": 4.0,
            "controller_grasp_hold_timeout_sec": 30.0,
        },
    }


def test_runner_loads_profile_and_allows_explicit_cli_override(tmp_path: Path) -> None:
    runner = _load_runner_module()
    output = tmp_path / "trial.json"

    args = runner._parse_args(
        [
            "--config",
            str(PROFILE_PATH),
            "--output",
            str(output),
            "--confirm",
            runner.REAL_CONFIRMATION,
            "--hold-sec",
            "4.0",
        ]
    )

    assert args.namespace == "rebotarm"
    assert args.runs == 1
    assert args.pregrasp_sec == 10.0
    assert args.approach_sec == 3.0
    assert args.hold_sec == 4.0
    assert args.return_sec == 10.0
    assert args.gripper_open_m == 0.080
    assert args.gripper_open_max_effort == 1.5
    assert args.grasp_close_force == 1.0
    assert args.output == output
    assert args.confirm == runner.REAL_CONFIRMATION
    assert runner._authorized_scope(args) == (
        "fresh bottle plan; 10s pregrasp, 3s approach, close, 4s hold, "
        "10s return; no lift or retreat"
    )


def test_runner_uses_installed_feature_profile_by_default(tmp_path: Path) -> None:
    runner = _load_runner_module()

    args = runner._parse_args(
        [
            "--output",
            str(tmp_path / "trial.json"),
            "--confirm",
            runner.REAL_CONFIRMATION,
        ]
    )

    assert args.config.name == "single_bottle_grasp.yaml"
    assert args.pregrasp_sec == 10.0
    assert args.approach_sec == 3.0
    assert args.hold_sec == 3.0
    assert args.return_sec == 10.0
    assert args.gripper_open_max_effort == 1.5


@pytest.mark.parametrize("forbidden", ["confirm", "output", "config"])
def test_runner_profile_rejects_per_run_fields(
    tmp_path: Path, forbidden: str
) -> None:
    runner = _load_runner_module()
    payload = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["feature_arguments"][forbidden] = "must-not-be-persisted"
    profile = tmp_path / "invalid.yaml"
    profile.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(RuntimeError, match=forbidden):
        runner._load_feature_profile(profile)


def test_runner_profile_rejects_missing_or_unknown_fields(tmp_path: Path) -> None:
    runner = _load_runner_module()
    payload = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    payload["feature_arguments"].pop("return_sec")
    payload["feature_arguments"]["return_duration_sec"] = 10.0
    profile = tmp_path / "invalid.yaml"
    profile.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(RuntimeError, match="return_sec|return_duration_sec"):
        runner._load_feature_profile(profile)


def test_failure_report_converts_non_finite_device_values_to_null(
    tmp_path: Path,
) -> None:
    runner = _load_runner_module()
    output = tmp_path / "failed-trial.json"
    payload = {
        "success": False,
        "gripper_close": {
            "success": False,
            "reached_position_m": math.nan,
            "contact_position_m": math.inf,
        },
    }

    runner._write_json(output, payload)

    assert json.loads(output.read_text(encoding="utf-8")) == {
        "success": False,
        "gripper_close": {
            "success": False,
            "reached_position_m": None,
            "contact_position_m": None,
        },
    }

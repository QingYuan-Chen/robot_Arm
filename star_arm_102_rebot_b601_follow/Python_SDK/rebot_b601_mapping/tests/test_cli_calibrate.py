from __future__ import annotations

import json
from pathlib import Path

import pytest

from rebot_b601_mapping.cli import run_calibration
from rebot_b601_mapping.models import FollowerSample, LeaderSample, MotorFeedback


EXAMPLE_CONFIG = Path(__file__).parents[1] / "mapping.example.json"
NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
BASE_POSITIONS = (0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0)


def leader_sample(angles=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) -> LeaderSample:
    return LeaderSample(timestamp_s=10.0, angles_deg=tuple(float(v) for v in angles))


def follower_sample(positions=BASE_POSITIONS) -> FollowerSample:
    return FollowerSample(
        timestamp_s=10.0,
        motors=tuple(
            MotorFeedback(
                name=name,
                position_rad=float(position),
                velocity_rad_s=0.0,
                torque_nm=0.0,
                status_code=0,
            )
            for name, position in zip(NAMES, positions, strict=True)
        ),
    )


class FakeReader:
    def __init__(self, samples) -> None:
        self.samples = iter(samples)
        self.closed = False

    def open(self) -> None:
        pass

    def read_sample(self):
        return next(self.samples)

    def close(self) -> None:
        self.closed = True


def copy_config(tmp_path: Path) -> Path:
    path = tmp_path / "mapping.local.json"
    path.write_bytes(EXAMPLE_CONFIG.read_bytes())
    return path


def run_joint1(
    tmp_path: Path,
    *,
    leader_after=(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    follower_after=(-0.2, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0),
    confirmation="确认",
):
    config_path = copy_config(tmp_path)
    output_path = tmp_path / "direction.json"
    leader = FakeReader([leader_sample()] * 5 + [leader_sample(leader_after)] * 5)
    follower = FakeReader(
        [follower_sample()] * 5 + [follower_sample(follower_after)] * 5
    )
    answers = iter(("", confirmation))
    result = run_calibration(
        selected_joint="joint1",
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyACM0",
        config_path=config_path,
        output_path=output_path,
        baseline_samples=5,
        window_samples=5,
        interval_s=0.0,
        leader_factory=lambda port: leader,
        follower_factory=lambda port: follower,
        port_checker=lambda paths: None,
        clock=lambda: 10.0,
        sleep=lambda seconds: None,
        input_fn=lambda prompt: next(answers),
        print_fn=lambda text: None,
    )
    return result, config_path, output_path, leader, follower


def test_calibration_persists_only_selected_joint_after_confirmation(
    tmp_path: Path,
) -> None:
    result, config_path, output_path, leader, follower = run_joint1(tmp_path)

    updated = json.loads(config_path.read_text(encoding="utf-8"))
    evidence = json.loads(output_path.read_text(encoding="utf-8"))
    assert result == evidence
    assert result["persisted"] is True
    assert result["direction"]["inferred_sign"] == -1
    assert updated["mapping"][0]["sign"] == -1
    assert updated["mapping"][0]["verified"] is True
    assert [item["verified"] for item in updated["mapping"][1:]] == [False] * 6
    assert updated["mapping"][0]["evidence"]["leader_delta_rad"] != 0.0
    assert updated["mapping"][0]["evidence"]["follower_delta_rad"] != 0.0
    assert leader.closed is True
    assert follower.closed is True


@pytest.mark.parametrize(
    ("leader_after", "follower_after", "confirmation", "expected_exception"),
    [
        (
            (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (-0.2, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0),
            "不确认",
            None,
        ),
        (
            (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.2, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0),
            "确认",
            None,
        ),
        (
            (10.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (-0.2, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0),
            "确认",
            ValueError,
        ),
    ],
)
def test_calibration_rejection_paths_leave_config_bytes_unchanged(
    tmp_path: Path,
    leader_after,
    follower_after,
    confirmation: str,
    expected_exception,
) -> None:
    config_path = copy_config(tmp_path)
    original = config_path.read_bytes()
    config_path.unlink()
    if expected_exception is None:
        result, actual_path, _, _, _ = run_joint1(
            tmp_path,
            leader_after=leader_after,
            follower_after=follower_after,
            confirmation=confirmation,
        )
        assert result["persisted"] is False
    else:
        with pytest.raises(expected_exception, match="非选定关节"):
            run_joint1(
                tmp_path,
                leader_after=leader_after,
                follower_after=follower_after,
                confirmation=confirmation,
            )
        actual_path = tmp_path / "mapping.local.json"
    assert actual_path.read_bytes() == original


def test_calibration_rejects_gripper_before_opening_hardware(tmp_path: Path) -> None:
    config_path = copy_config(tmp_path)
    factory_called = False

    def factory(port):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("不应打开硬件")

    with pytest.raises(ValueError, match="joint1..joint6"):
        run_calibration(
            selected_joint="gripper",
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyACM0",
            config_path=config_path,
            output_path=tmp_path / "direction.json",
            leader_factory=factory,
            follower_factory=factory,
        )

    assert factory_called is False

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rebot_b601_mapping.cli import run_snapshot
from rebot_b601_mapping.models import FollowerSample, LeaderSample, MotorFeedback


CONFIG_PATH = Path(__file__).parents[1] / "mapping.example.json"
NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")


def leader_sample(index: int) -> LeaderSample:
    return LeaderSample(
        timestamp_s=10.0,
        angles_deg=(float(index), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )


def follower_sample(index: int) -> FollowerSample:
    positions = (0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0)
    return FollowerSample(
        timestamp_s=10.0,
        motors=tuple(
            MotorFeedback(
                name=name,
                position_rad=position,
                velocity_rad_s=0.0,
                torque_nm=0.01 * index,
                status_code=0,
            )
            for name, position in zip(NAMES, positions, strict=True)
        ),
    )


class FakeReader:
    def __init__(self, samples) -> None:
        self.samples = iter(samples)
        self.opened = False
        self.closed = False

    def open(self) -> None:
        self.opened = True

    def read_sample(self):
        return next(self.samples)

    def close(self) -> None:
        self.closed = True


def test_snapshot_writes_twenty_read_only_evidence_samples(tmp_path: Path) -> None:
    leader = FakeReader([leader_sample(0)] * 5 + [leader_sample(i) for i in range(20)])
    follower = FakeReader([follower_sample(i) for i in range(25)])
    output = tmp_path / "evidence" / "snapshot.json"
    checked_ports = []

    evidence = run_snapshot(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyACM0",
        config_path=CONFIG_PATH,
        output_path=output,
        baseline_samples=5,
        sample_count=20,
        interval_s=0.0,
        leader_factory=lambda port: leader,
        follower_factory=lambda port: follower,
        port_checker=lambda paths: checked_ports.extend(paths),
        clock=lambda: 10.0,
        sleep=lambda seconds: None,
    )

    persisted = json.loads(output.read_text(encoding="utf-8"))
    assert evidence == persisted
    assert evidence["mode"] == "snapshot"
    assert evidence["sample_count"] == 20
    assert len(evidence["samples"]) == 20
    assert len(evidence["samples"][-1]["virtual_follower_rad"]) == 6
    assert all(item["verified"] is False for item in evidence["mapping"])
    assert checked_ports == ["/dev/ttyUSB0", "/dev/ttyACM0"]
    assert leader.closed is True
    assert follower.closed is True


@pytest.mark.parametrize("failure", [RuntimeError("读取失败"), KeyboardInterrupt()])
def test_snapshot_closes_both_readers_when_acquisition_stops(
    tmp_path: Path,
    failure: BaseException,
) -> None:
    leader = FakeReader([leader_sample(0)] * 6)

    class FailingReader(FakeReader):
        def read_sample(self):
            raise failure

    follower = FailingReader([])

    with pytest.raises(type(failure), match="读取失败" if isinstance(failure, RuntimeError) else None):
        run_snapshot(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyACM0",
            config_path=CONFIG_PATH,
            output_path=tmp_path / "snapshot.json",
            baseline_samples=5,
            sample_count=20,
            interval_s=0.0,
            leader_factory=lambda port: leader,
            follower_factory=lambda port: follower,
            port_checker=lambda paths: None,
            clock=lambda: 10.0,
            sleep=lambda seconds: None,
        )

    assert leader.closed is True
    assert follower.closed is True

from __future__ import annotations

import math
from pathlib import Path

import pytest

from rebot_b601_mapping.models import LeaderSample


CONFIG_PATH = Path(__file__).parents[1] / "mapping.example.json"


class FakeLeader:
    def __init__(self, samples: list[LeaderSample]) -> None:
        self._samples = iter(samples)
        self.opened = False
        self.closed = False

    def open(self) -> None:
        self.opened = True

    def read_sample(self) -> LeaderSample:
        return next(self._samples)

    def close(self) -> None:
        self.closed = True


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[tuple[tuple[str, ...], tuple[float, ...]]] = []
        self.closed = False

    def publish(self, names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        self.messages.append((names, positions))

    def close(self) -> None:
        self.closed = True


def sample(angles: tuple[float, ...], timestamp: float = 10.0) -> LeaderSample:
    return LeaderSample(timestamp_s=timestamp, angles_deg=angles)


def test_preview_reads_only_leader_and_publishes_candidate_mapping() -> None:
    from rebot_b601_mapping.rviz_preview import run_rviz_preview

    zeros = (0.0,) * 7
    moved = (10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0)
    leader = FakeLeader([sample(zeros)] * 5 + [sample(moved)])
    publisher = FakePublisher()
    checked_ports: list[str] = []

    result = run_rviz_preview(
        leader_port="/dev/ttyUSB0",
        config_path=CONFIG_PATH,
        baseline_samples=5,
        rate_hz=20.0,
        max_samples=1,
        leader_factory=lambda port: leader,
        publisher_factory=lambda topic: publisher,
        port_checker=lambda paths: checked_ports.extend(paths),
        clock=lambda: 10.0,
        sleep=lambda seconds: None,
        print_fn=lambda message: None,
    )

    names, positions = publisher.messages[0]
    delta = math.radians(10.0)
    assert checked_ports == ["/dev/ttyUSB0"]
    assert names == ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    assert positions == pytest.approx(
        (
            -delta,
            -1.56 - delta,
            -1.57 + delta,
            -0.15 + delta,
            delta,
            -delta,
        ),
        abs=0.01,
    )
    assert result["published_samples"] == 1
    assert leader.closed is True
    assert publisher.closed is True


def test_preview_rejects_invalid_rate_before_opening_port() -> None:
    from rebot_b601_mapping.rviz_preview import run_rviz_preview

    leader = FakeLeader([])

    with pytest.raises(ValueError, match="rate_hz"):
        run_rviz_preview(
            leader_port="/dev/ttyUSB0",
            config_path=CONFIG_PATH,
            rate_hz=0.0,
            leader_factory=lambda port: leader,
            publisher_factory=lambda topic: FakePublisher(),
        )

    assert leader.opened is False


def test_cli_exposes_rviz_preview_without_follower_port(monkeypatch, capsys) -> None:
    from rebot_b601_mapping import cli

    received: dict[str, object] = {}

    def fake_run(**kwargs):
        received.update(kwargs)
        return {"published_samples": 3}

    monkeypatch.setattr(cli, "run_rviz_preview", fake_run, raising=False)

    exit_code = cli.main(
        [
            "rviz-preview",
            "--leader-port",
            "/dev/ttyUSB9",
            "--config",
            str(CONFIG_PATH),
            "--rate-hz",
            "15",
            "--max-samples",
            "3",
        ]
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert received["leader_port"] == "/dev/ttyUSB9"
    assert "follower_port" not in received
    assert "RViz 预览结束" in output

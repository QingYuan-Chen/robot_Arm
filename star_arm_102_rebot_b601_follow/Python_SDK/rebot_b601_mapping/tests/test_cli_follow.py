from __future__ import annotations

from pathlib import Path

from rebot_b601_mapping.live_follow import LiveRunSummary, StopRequest
from rebot_b601_mapping.safety_supervisor import FollowState


ROOT = Path(__file__).parents[1]
MAPPING = ROOT / "mapping.example.json"
LIVE = ROOT / "live_follow.example.json"


def fake_summary(log_path, state=FollowState.DISCONNECTED):
    return LiveRunSummary(
        final_state=state,
        cycles=3,
        safe_home_verified=state is FollowState.DISCONNECTED,
        disable_verified=state is FollowState.DISCONNECTED,
        disable_result_known=True,
        log_path=Path(log_path),
    )


def test_follow_cli_passes_explicit_motion_confirmation(monkeypatch, tmp_path) -> None:
    from rebot_b601_mapping import cli

    received = {}
    monkeypatch.setattr(
        cli,
        "run_live_follow",
        lambda **kwargs: received.update(kwargs) or fake_summary(kwargs["log_path"]),
        raising=False,
    )
    monkeypatch.setattr(cli.signal, "signal", lambda *args: None)

    code = cli.main(
        [
            "follow",
            "--leader-port",
            "/dev/ttyUSB9",
            "--follower-port",
            "/dev/ttyACM9",
            "--mapping-config",
            str(MAPPING),
            "--live-config",
            str(LIVE),
            "--log",
            str(tmp_path / "follow.jsonl"),
            "--speed-rad-s",
            "0.5",
            "--confirm-live-motion",
        ]
    )

    assert code == 0
    assert received["leader_port"] == "/dev/ttyUSB9"
    assert received["follower_port"] == "/dev/ttyACM9"
    assert received["confirmed"] is True
    assert received["speed_rad_s"] == 0.5
    assert isinstance(received["stop_request"], StopRequest)


def test_follow_without_confirmation_requests_static_preflight_only(
    monkeypatch,
    tmp_path,
    capsys,
) -> None:
    from rebot_b601_mapping import cli

    received = {}
    monkeypatch.setattr(
        cli,
        "run_live_follow",
        lambda **kwargs: received.update(kwargs) or fake_summary(kwargs["log_path"]),
        raising=False,
    )
    monkeypatch.setattr(cli.signal, "signal", lambda *args: None)

    code = cli.main(
        [
            "follow",
            "--mapping-config",
            str(MAPPING),
            "--live-config",
            str(LIVE),
            "--log",
            str(tmp_path / "preflight.jsonl"),
        ]
    )

    assert code == 0
    assert received["confirmed"] is False
    assert "未使能、未发送运动命令" in capsys.readouterr().out


def test_second_sigint_changes_normal_stop_to_emergency_stop(capsys) -> None:
    from rebot_b601_mapping import cli

    request = StopRequest()
    handler = cli._make_sigint_handler(request)

    handler(2, None)
    assert request.normal_requested is True
    assert request.emergency_requested is False

    handler(2, None)
    assert request.normal_requested is True
    assert request.emergency_requested is True
    output = capsys.readouterr().err
    assert "受控返回安全位" in output
    assert "紧急停机" in output


def test_follow_critical_stop_returns_nonzero(monkeypatch, tmp_path) -> None:
    from rebot_b601_mapping import cli

    monkeypatch.setattr(
        cli,
        "run_live_follow",
        lambda **kwargs: fake_summary(kwargs["log_path"], FollowState.CRITICAL_STOP),
        raising=False,
    )
    monkeypatch.setattr(cli.signal, "signal", lambda *args: None)

    code = cli.main(
        [
            "follow",
            "--mapping-config",
            str(MAPPING),
            "--live-config",
            str(LIVE),
            "--log",
            str(tmp_path / "critical.jsonl"),
            "--confirm-live-motion",
        ]
    )

    assert code == 2

from __future__ import annotations

from pathlib import Path

from rebot_b601_mapping.tests import hardware_snapshot_smoke


def test_hardware_smoke_entry_forwards_to_read_only_snapshot(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    calls = []

    def fake_snapshot(**kwargs):
        calls.append(kwargs)
        return {"sample_count": kwargs["sample_count"]}

    monkeypatch.setattr(hardware_snapshot_smoke, "run_snapshot", fake_snapshot)
    output = tmp_path / "snapshot.json"

    exit_code = hardware_snapshot_smoke.main(
        [
            "--leader-port",
            "/dev/ttyUSB0",
            "--follower-port",
            "/dev/ttyACM0",
            "--samples",
            "20",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    assert len(calls) == 1
    assert calls[0]["leader_port"] == "/dev/ttyUSB0"
    assert calls[0]["follower_port"] == "/dev/ttyACM0"
    assert calls[0]["sample_count"] == 20
    assert calls[0]["output_path"] == output
    stdout = capsys.readouterr().out
    assert "严格只读" in stdout
    assert "不会使能、失能、置零或控制机械臂" in stdout

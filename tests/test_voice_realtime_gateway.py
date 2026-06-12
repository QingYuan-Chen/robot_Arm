from __future__ import annotations

from pathlib import Path
import json
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.realtime_session_client import JsonlRealtimeSessionClient
from rebotarm_voice_control.realtime_voice_gateway_node import main, run_realtime_gateway


def test_jsonl_realtime_gateway_routes_only_function_calls(tmp_path):
    event_file = tmp_path / "events.jsonl"
    event_file.write_text(
        "\n".join(
            [
                '{"type":"response.audio.delta","delta":"abc"}',
                (
                    '{"type":"response.function_call_arguments.done",'
                    '"call_id":"call_up","name":"move_relative",'
                    '"arguments":"{\\"axis\\":\\"z\\",\\"distance\\":5,\\"unit\\":\\"cm\\"}"}'
                ),
                '{"type":"response.done"}',
            ]
        ),
        encoding="utf-8",
    )
    client = JsonlRealtimeSessionClient(event_file)

    results = run_realtime_gateway(client, SRC / "config", execution_mode="sim")

    assert len(results) == 1
    assert results[0]["call_id"] == "call_up"
    assert results[0]["tool"] == "move_relative"
    assert results[0]["route"]["target"] == "/rebotarm/sim/move_relative"
    assert results[0]["route"]["params"]["distance_m"] == 0.05


def test_jsonl_realtime_client_skips_blank_lines(tmp_path):
    event_file = tmp_path / "events.jsonl"
    event_file.write_text('\n{"type":"response.done"}\n\n', encoding="utf-8")
    client = JsonlRealtimeSessionClient(event_file)

    client.connect()
    try:
        assert client.recv_event() == {"type": "response.done"}
        assert client.recv_event() is None
    finally:
        client.close()


def test_jsonl_realtime_client_accepts_utf8_bom(tmp_path):
    event_file = tmp_path / "events.jsonl"
    event_file.write_text('{"type":"response.done"}\n', encoding="utf-8-sig")
    client = JsonlRealtimeSessionClient(event_file)

    client.connect()
    try:
        assert client.recv_event() == {"type": "response.done"}
    finally:
        client.close()


def test_realtime_gateway_cli_prints_routed_results(tmp_path, capsys, monkeypatch):
    event_file = tmp_path / "events.jsonl"
    event_file.write_text(
        (
            '{"type":"response.function_call_arguments.done",'
            '"call_id":"call_home","name":"move_home","arguments":"{}"}\n'
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["rebotarm_realtime_gateway", "--event-jsonl", str(event_file), "--mode", "dry_run"],
    )

    main()

    output = json.loads(capsys.readouterr().out)
    assert output[0]["call_id"] == "call_home"
    assert output[0]["route"]["target"] == "/rebotarm/safe_home"

from __future__ import annotations

from pathlib import Path
import argparse
import json
from typing import Any

from .config_loader import load_voice_control_config
from .realtime_bridge import RealtimeToolBridge
from .realtime_session_client import JsonlRealtimeSessionClient, RealtimeSessionClient


def run_realtime_gateway(
    session_client: RealtimeSessionClient,
    config_root: str | Path,
    execution_mode: str = "dry_run",
) -> list[dict[str, Any]]:
    config = load_voice_control_config(config_root)
    bridge = RealtimeToolBridge(config, execution_mode=execution_mode)
    results: list[dict[str, Any]] = []

    session_client.connect()
    try:
        while True:
            event = session_client.recv_event()
            if event is None:
                break
            result = bridge.handle_event(event)
            if result is not None:
                results.append(result)
    finally:
        session_client.close()

    return results


def main() -> None:
    cli = argparse.ArgumentParser(description="Route a provider-neutral Realtime event stream safely.")
    cli.add_argument("--event-jsonl", required=True, help="JSONL file with one Realtime event per line.")
    cli.add_argument("--mode", default="dry_run", choices=["dry_run", "sim", "real"])
    args = cli.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    try:
        results = run_realtime_gateway(
            JsonlRealtimeSessionClient(args.event_jsonl),
            package_root / "config",
            execution_mode=args.mode,
        )
        print(json.dumps(results, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

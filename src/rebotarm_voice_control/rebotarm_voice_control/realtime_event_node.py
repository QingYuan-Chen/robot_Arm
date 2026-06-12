from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys

from .config_loader import load_voice_control_config
from .realtime_bridge import RealtimeToolBridge


def handle_realtime_event_json(
    payload: str,
    config_root: str | Path,
    execution_mode: str = "dry_run",
) -> dict | None:
    config = load_voice_control_config(config_root)
    return RealtimeToolBridge(config, execution_mode=execution_mode).handle_event_json(payload)


def main() -> None:
    cli = argparse.ArgumentParser(description="Route one OpenAI Realtime event JSON.")
    cli.add_argument("json_file", help="Path to JSON file, or '-' to read stdin.")
    cli.add_argument("--mode", default="dry_run", choices=["dry_run", "sim", "real"])
    args = cli.parse_args()

    payload = sys.stdin.read() if args.json_file == "-" else Path(args.json_file).read_text(encoding="utf-8")
    package_root = Path(__file__).resolve().parents[1]
    try:
        result = handle_realtime_event_json(payload, package_root / "config", execution_mode=args.mode)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

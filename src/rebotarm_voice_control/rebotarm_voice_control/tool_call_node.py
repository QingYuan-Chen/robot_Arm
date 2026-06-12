from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import argparse
import json
import sys

from .config_loader import load_voice_control_config
from .execution_modes import ExecutionModeRouter
from .safety_guard import SafetyGuard
from .tool_call_schema import ToolCallParser


def handle_tool_call_json(
    payload: str,
    config_root: str | Path,
    execution_mode: str = "dry_run",
) -> dict:
    config = load_voice_control_config(config_root)
    parser = ToolCallParser()
    call = parser.parse_json(payload)
    command = SafetyGuard(config).validate(parser.to_intent(call))
    execution = ExecutionModeRouter(config, execution_mode=execution_mode).route(command)
    return {
        "call_id": call.call_id,
        "tool": call.tool,
        "intent": command.intent,
        "execution_mode": execution.execution_mode,
        "route": asdict(execution.route),
    }


def main() -> None:
    cli = argparse.ArgumentParser(description="Route one whitelisted LLM tool-call JSON.")
    cli.add_argument("json_file", help="Path to JSON file, or '-' to read stdin.")
    cli.add_argument("--mode", default="dry_run", choices=["dry_run", "sim", "real"])
    args = cli.parse_args()

    payload = sys.stdin.read() if args.json_file == "-" else Path(args.json_file).read_text(encoding="utf-8")
    package_root = Path(__file__).resolve().parents[1]
    try:
        result = handle_tool_call_json(payload, package_root / "config", execution_mode=args.mode)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

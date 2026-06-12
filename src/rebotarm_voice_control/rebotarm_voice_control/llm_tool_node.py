from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import argparse
import json

from .config_loader import load_llm_provider_config, load_voice_control_config
from .execution_modes import ExecutionModeRouter
from .llm_providers import BaseLLMProvider, create_llm_provider
from .safety_guard import SafetyGuard
from .tool_call_schema import ToolCallParser


def handle_llm_text(
    text: str,
    config_root: str | Path,
    provider_config: dict | None = None,
    execution_mode: str = "dry_run",
    provider: BaseLLMProvider | None = None,
) -> dict:
    config = load_voice_control_config(config_root)
    resolved_provider_config = provider_config or load_llm_provider_config(config_root)
    llm_provider = provider or create_llm_provider(resolved_provider_config)
    tool_payload = llm_provider.parse_tool_call(text)
    parser = ToolCallParser()
    tool_call = parser.parse_json(tool_payload)
    command = SafetyGuard(config).validate(parser.to_intent(tool_call))
    execution = ExecutionModeRouter(config, execution_mode=execution_mode).route(command)
    return {
        "provider": llm_provider.provider_name,
        "source_text": text,
        "tool_call": {
            "tool": tool_call.tool,
            "arguments": tool_call.arguments,
            "call_id": tool_call.call_id,
        },
        "intent": command.intent,
        "execution_mode": execution.execution_mode,
        "route": asdict(execution.route),
    }


def main() -> None:
    cli = argparse.ArgumentParser(description="Parse text with an LLM provider and route safely.")
    cli.add_argument("text", help="Chinese command text")
    cli.add_argument("--provider", choices=["mock", "doubao", "openai"])
    cli.add_argument("--model", default="")
    cli.add_argument("--mode", default="dry_run", choices=["dry_run", "sim", "real"])
    args = cli.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    provider_config = load_llm_provider_config(
        package_root / "config",
        overrides={"provider": args.provider, "model": args.model},
    )
    try:
        result = handle_llm_text(
            args.text,
            package_root / "config",
            provider_config=provider_config,
            execution_mode=args.mode,
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

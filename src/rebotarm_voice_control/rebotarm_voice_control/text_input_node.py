from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import json

from .command_router import DryRunCommandRouter
from .config_loader import load_voice_control_config
from .intent_parser import IntentParser
from .safety_guard import SafetyGuard
from .task_planner import TaskPlanner


def handle_text_command(text: str, config_root: str | Path) -> dict:
    config = load_voice_control_config(config_root)
    parser = IntentParser(config.intents)
    guard = SafetyGuard(config)
    planner = TaskPlanner(config, guard)
    router = DryRunCommandRouter(config)

    command = parser.parse(text)
    steps = planner.expand(command)
    routes = [asdict(router.route(step)) for step in steps]
    return {
        "source_text": text,
        "intent": command.intent,
        "need_confirm": command.need_confirm,
        "steps": routes,
    }


def main() -> None:
    package_root = Path(__file__).resolve().parents[1]
    config_root = package_root / "config"
    print("reBotArm text command MVP. Type '退出' to quit.")
    while True:
        text = input("> ").strip()
        if text in {"退出", "quit", "exit"}:
            return
        try:
            result = handle_text_command(text, config_root)
            print(json.dumps(result, ensure_ascii=False, indent=2))
        except Exception as exc:
            print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))

"""中文文本指令的交互式命令行入口（MVP）。

职责：不接麦克风，直接在终端读入中文文本，走与语音链路完全相同的解析与安全
流程，便于在没有音频设备/ASR 密钥时验证意图表与安全策略：

    文本 → IntentParser（匹配 intents.yaml 的 patterns）
         → TaskPlanner（若命中任务模板则展开为多条步骤）
         → SafetyGuard（逐条校验）
         → DryRunCommandRouter（给出目标接口与参数）

安全语义：本入口固定使用演练路由，只打印"将会调用哪个接口"，不产生任何真实
运动指令；退出词为中文"退出"或 quit / exit。
"""

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
    """解析一条中文文本指令并返回演练路由结果。

    参数：
        text：原始中文指令文本，如 "回到初始位置"。
        config_root：语音控制配置目录。

    返回：包含 source_text、intent、need_confirm 与逐步路由列表 steps 的字典。
    任务模板会被展开为多条步骤；非模板命令则退化为单步列表。

    异常：文本无法匹配任何意图（UnknownCommandError）、安全校验不通过
    （SafetyViolationError）或模板不存在时向上抛出。
    """
    config = load_voice_control_config(config_root)
    parser = IntentParser(config.intents)
    guard = SafetyGuard(config)
    planner = TaskPlanner(config, guard)
    router = DryRunCommandRouter(config)

    command = parser.parse(text)
    # 模板展开内部会再次做安全校验，故此处不必重复校验最终步骤。
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
        # "退出" 为中文退出词，与 quit/exit 等价；空行会继续循环而不退出。
        if text in {"退出", "quit", "exit"}:
            return
        try:
            result = handle_text_command(text, config_root)
            print(json.dumps(result, ensure_ascii=False, indent=2))
        except Exception as exc:
            # 单条指令失败不结束会话，只打印错误，方便连续调试意图表。
            print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))

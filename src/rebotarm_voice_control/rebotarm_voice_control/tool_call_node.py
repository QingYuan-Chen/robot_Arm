"""单条大模型工具调用的命令行入口。

处理链路（每一步都是必需的，缺一不可）：

    工具调用 JSON → 白名单与参数解析（ToolCallParser）
                  → 安全校验（SafetyGuard）
                  → 执行模式路由（ExecutionModeRouter，dry_run/sim/real）
                  → 结构化 JSON 结果

安全语义：工具名必须先过大模型工具白名单，再经安全守卫（工作空间、位移上限等）
校验；任何一步失败都直接抛异常，绝不降级执行。``--mode`` 默认 dry_run。
"""

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
    """解析并路由一条工具调用 JSON，返回可序列化的结果字典。

    参数：
        payload：工具调用 JSON 文本，字段为 tool / arguments / call_id。
        config_root：语音控制配置目录。
        execution_mode：执行模式，默认 "dry_run"。

    返回：包含 call_id、tool、intent、execution_mode 与 route 明细的字典，
    供命令行打印或上层回填给大模型。

    异常：JSON 结构或工具名非法、安全校验不通过、执行模式不支持时向上抛出。
    """
    config = load_voice_control_config(config_root)
    parser = ToolCallParser()
    call = parser.parse_json(payload)
    # 先校验再路由：确保任何路由结果都已经过白名单与安全限值检查。
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
    # json_file 为 "-" 时从标准输入读取，便于管道对接大模型输出。
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

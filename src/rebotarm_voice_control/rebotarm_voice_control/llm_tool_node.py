"""大模型文本 → 工具调用 → 安全校验 → 执行模式路由的串联入口。

``handle_llm_text`` 把一句话（中文）跑完整条流水线：

    1. 加载语音控制配置与 LLM provider 配置，创建 provider；
    2. provider 把文本解析为工具调用 JSON；
    3. ToolCallParser 做白名单与参数校验并转换成意图命令；
    4. SafetyGuard 做命令级安全校验；
    5. ExecutionModeRouter 按执行模式给出路由结果。

默认执行模式是 dry_run：即使模型配置有误或模型乱答，也不会产生真实运动
指令。main() 是命令行入口，异常统一以 JSON 打印并以退出码 1 结束。
"""

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
    """把一段自然语言文本解析为"已校验意图 + 路由结果"的字典。

    参数：
        text：待解析的中文指令文本。
        config_root：config/ 目录所在路径（内含 intent、命名位姿、任务模板、
            安全限制与大模型配置等多份 YAML）。
        provider_config：可直接注入的大模型配置；为 None 时从 config_root
            加载，便于测试替换。
        execution_mode：执行模式，默认 dry_run（最安全的演练模式）。
        provider：可直接注入的 provider 实例；为 None 时按配置创建。

    返回：包含 provider 名、原始文本、工具调用、意图名、执行模式与路由明细
    的字典。任何安全违规都以异常抛出，不返回部分结果。
    """
    config = load_voice_control_config(config_root)
    resolved_provider_config = provider_config or load_llm_provider_config(config_root)
    llm_provider = provider or create_llm_provider(resolved_provider_config)
    tool_payload = llm_provider.parse_tool_call(text)
    parser = ToolCallParser()
    tool_call = parser.parse_json(tool_payload)
    # 先转换意图再做安全校验：校验对象是归一化后的命令而非模型的原始输出。
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
    """命令行入口：解析参数、构建大模型配置并打印 JSON 结果。"""
    cli = argparse.ArgumentParser(description="Parse text with an LLM provider and route safely.")
    cli.add_argument("text", help="Chinese command text")
    cli.add_argument("--provider", choices=["mock", "doubao", "openai"])
    cli.add_argument("--model", default="")
    # 默认 dry_run：命令行不显式指定模式时不会碰真实硬件。
    cli.add_argument("--mode", default="dry_run", choices=["dry_run", "sim", "real"])
    args = cli.parse_args()

    # config/ 与本文件同属一个包，按文件位置定位以便从任意工作目录运行。
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

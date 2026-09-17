"""单条实时语音事件（JSON）的命令行入口。

职责：读取一条实时事件 JSON，经 ``RealtimeToolBridge`` 过滤、解析、安全校验与
执行模式路由后打印结果。适合把线上录到的一条事件单独拿来做回归验证。

安全语义：``--mode`` 默认 "dry_run"；非函数调用完成事件返回 null，属正常结果
而非错误。
"""

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
    """处理一条实时事件 JSON。

    参数：
        payload：单条事件的 JSON 文本（字段 type / name / arguments / call_id）。
        config_root：语音控制配置目录。
        execution_mode：执行模式，默认 "dry_run"。

    返回：被识别为工具调用的结果字典；事件类型不是函数调用完成时返回 None。

    异常：JSON 非法、工具名不在白名单或安全校验失败时向上抛出。
    """
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

"""与提供方无关的实时语音事件网关。

职责：从实现了 ``RealtimeSessionClient`` 接口的事件源（当前为 JSONL 回放文件）
逐条读取实时事件，交给 ``RealtimeToolBridge`` 做工具白名单校验与执行模式路由，
并把每条成功路由的结果收集成列表输出（JSON）。本模块不直接访问麦克风或网络，
事件源的具体实现由调用方注入，便于离线回放与测试。

安全默认值：``execution_mode`` 缺省为 "dry_run"（只路由、不下发）；选择 "real"
时仍会被 safety_limits.yaml 中的真实调用开关二次拦截。
"""

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
    """消费整个事件流并返回被成功路由的事件结果列表。

    参数：
        session_client：事件源，本函数负责 connect/close，调用方不必管理生命周期。
        config_root：语音控制配置目录（含 intents/named_poses/task_templates/safety_limits）。
        execution_mode：执行模式 "dry_run" / "sim" / "real"，默认 dry_run。

    返回：每条被识别为工具调用的事件对应一个结果字典；非工具调用事件返回 None，
    因此不会进入结果列表。

    副作用：会打开并关闭事件源；``close`` 放在 finally 中，保证异常退出也不泄漏
    文件句柄。
    """
    config = load_voice_control_config(config_root)
    bridge = RealtimeToolBridge(config, execution_mode=execution_mode)
    results: list[dict[str, Any]] = []

    session_client.connect()
    try:
        while True:
            event = session_client.recv_event()
            if event is None:
                # None 是"事件流结束"的约定信号，不是错误。
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
    # 默认 dry_run：不指定模式时绝不产生真实运动指令。
    cli.add_argument("--mode", default="dry_run", choices=["dry_run", "sim", "real"])
    args = cli.parse_args()

    # 包根目录 = 本文件所在包的上一级，运行时配置固定取 <包根>/config。
    package_root = Path(__file__).resolve().parents[1]
    try:
        results = run_realtime_gateway(
            JsonlRealtimeSessionClient(args.event_jsonl),
            package_root / "config",
            execution_mode=args.mode,
        )
        print(json.dumps(results, ensure_ascii=False, indent=2))
    except Exception as exc:
        # 失败时以结构化 JSON 输出错误类型与消息，并用非零退出码告知调用方。
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

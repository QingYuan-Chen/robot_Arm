# 仿真执行后端：接收上层路由结果（RouteResult），校验后交给仿真侧执行。
#
# 提供两种后端，由 sim_config.yaml 的 backend 字段或命令行 --backend 选择：
# - "recorded"（默认，RecordedSimExecutor）：只做校验并记录，明确不派发，
#   因此 dispatch_result 为空；用于在没有仿真控制器时也能跑通链路。
# - "moveit2"（MoveIt2SimExecutor）：通过动作传输层把目标真正发出去，
#   需要注入 transport，否则直接拒绝启动。
#
# 安全边界（两道，缺一不可）：
# 1. 只接受动作名以 "/rebotarm/sim" 开头的目标，物理机械臂的动作名前缀不同，
#    这样即使上层路由被污染也无法误打到真机；
# 2. dry_run 的路由禁止进入执行器——dry-run 只允许预演，不得产生副作用。
#
# 本模块也可作为命令行工具运行，读取一个路由 JSON 并打印执行结果。

from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
import json
from pathlib import Path
import sys
from typing import Any, Protocol

from .config_loader import load_sim_config
from .models import RouteResult, SafetyViolationError
from .ros2_action_transport import Ros2ActionTransport
from .sim_action_bindings import build_sim_goal, resolve_sim_action_type


@dataclass(frozen=True)
class SimExecutionResult:
    """一次仿真执行的结果快照（不可变，便于直接序列化成 JSON）。

    - ``accepted``：路由是否通过校验；
    - ``dispatched``：是否真的把目标发出了（recorded 后端恒为 False）；
    - ``backend``：实际使用的后端标识；
    - ``intent`` / ``target`` / ``mode`` / ``params``：回显原始路由信息；
    - ``message``：面向操作者的结果说明；
    - ``dispatch_result``：传输层返回的派发元数据，未派发时为 None。
    """

    accepted: bool
    dispatched: bool
    backend: str
    intent: str
    target: str
    mode: str
    params: dict[str, Any]
    message: str
    dispatch_result: dict[str, Any] | None = None


class SimActionTransport(Protocol):
    """动作传输层的结构化协议：只要求实现一个发送目标的方法。"""

    def send_action_goal(self, action_name: str, goal: dict[str, Any]) -> dict[str, Any]:
        """发送一个仿真动作目标，返回提供方自定义的派发元数据。"""


class RecordedSimExecutor:
    """只校验并记录仿真命令，不声称已连接运动规划执行侧。"""

    backend = "recorded_sim"

    def execute(self, route: RouteResult) -> SimExecutionResult:
        _validate_sim_route(route)
        return SimExecutionResult(
            accepted=True,
            dispatched=False,
            backend=self.backend,
            intent=route.intent,
            target=route.target,
            mode=route.mode,
            params=dict(route.params),
            message="sim route accepted for recorded execution; MoveIt2 sim dispatch is not connected",
        )


class MoveIt2SimExecutor:
    """把安全的仿真路由派发到运动规划风格传输层的适配边界。"""

    backend = "moveit2_sim"

    def __init__(self, transport: SimActionTransport):
        self._transport = transport

    def execute(self, route: RouteResult) -> SimExecutionResult:
        _validate_sim_route(route)
        # 只支持动作模式：服务/话题模式的语义与动作不同，混用会导致结果无反馈
        if route.mode != "action":
            raise SafetyViolationError("MoveIt2 sim executor only action routes are supported")
        goal = self._build_goal(route)
        dispatch_result = self._transport.send_action_goal(route.target, goal)
        return SimExecutionResult(
            accepted=True,
            dispatched=True,
            backend=self.backend,
            intent=route.intent,
            target=route.target,
            mode=route.mode,
            params=dict(route.params),
            message="sim route dispatched through MoveIt2 transport adapter",
            dispatch_result=dict(dispatch_result),
        )

    def _build_goal(self, route: RouteResult) -> dict[str, Any]:
        """把意图与参数摊平成传给传输层的目标字典。"""
        goal = {"intent": route.intent}
        goal.update(dict(route.params))
        return goal


def _validate_sim_route(route: RouteResult) -> None:
    """仿真路由的强制安全门：dry-run 不执行，且只接受仿真命名空间。"""
    if route.dry_run:
        raise SafetyViolationError("dry-run route cannot be executed by sim executor")
    if not route.target.startswith("/rebotarm/sim"):
        raise SafetyViolationError("sim executor only accepts /rebotarm/sim routes")


def _route_from_payload(payload: dict[str, Any]) -> RouteResult:
    """从 JSON 载荷中取出路由对象；兼容外层包一层 route 的写法。

    兼容处理是为了让上层既能直接传路由本身，也能传带上下文字段的包裹结构；
    路由对象缺失或类型不对时抛 ValueError。
    """
    route = payload.get("route", payload)
    if not isinstance(route, dict):
        raise ValueError("sim execution payload must contain a route object")
    return RouteResult(
        intent=str(route["intent"]),
        target=str(route["target"]),
        mode=str(route["mode"]),
        params=dict(route.get("params", {})),
        dry_run=bool(route.get("dry_run", False)),
    )


def _create_sim_executor(
    backend: str,
    transport: SimActionTransport | None = None,
) -> RecordedSimExecutor | MoveIt2SimExecutor:
    """按后端名构造执行器；后端名未知或缺少传输层时拒绝，不做静默降级。"""
    if backend == "recorded":
        return RecordedSimExecutor()
    if backend == "moveit2":
        if transport is None:
            raise SafetyViolationError("MoveIt2 transport is not configured")
        return MoveIt2SimExecutor(transport=transport)
    raise SafetyViolationError(f"unsupported sim backend: {backend}")


def create_moveit2_sim_transport(ros_node: Any | None) -> Ros2ActionTransport:
    """创建基于 ROS 2 动作的仿真传输层。

    传输层需要两个回调：动作名到消息类型的解析器，以及目标构造函数——两者都由
    绑定层提供，保证校验规则只有一处定义。
    """
    if ros_node is None:
        raise SafetyViolationError("ROS2 node is required for MoveIt2 sim transport")
    return Ros2ActionTransport(
        node=ros_node,
        action_type_resolver=resolve_sim_action_type,
        goal_builder=build_sim_goal,
    )


def handle_sim_execution_json(
    payload: str,
    backend: str = "recorded",
    transport: SimActionTransport | None = None,
) -> dict[str, Any]:
    """JSON 字符串入口：解析 → 构造执行器 → 执行 → 返回字典结果。

    默认使用 recorded 后端，即只记录不执行，确保未显式指定后端时不会产生
    意外运动。输入不是 JSON 对象时抛 ValueError。
    """
    loaded = json.loads(payload)
    if not isinstance(loaded, dict):
        raise ValueError("sim execution payload must be a JSON object")
    executor = _create_sim_executor(backend=backend, transport=transport)
    return asdict(executor.execute(_route_from_payload(loaded)))


def main() -> None:
    """命令行入口：校验并记录（或派发）一条仿真路由，结果以 JSON 打印。"""
    cli = argparse.ArgumentParser(description="Validate and record one /rebotarm/sim route.")
    cli.add_argument("json_file", help="Path to routed JSON, or '-' to read stdin.")
    # 不传 --backend 时回落到配置文件的 backend 值
    cli.add_argument("--backend", choices=["recorded", "moveit2"])
    # 允许指定配置目录，便于从源码树之外运行；为空表示用包内 config/
    cli.add_argument("--config-root", default="")
    args = cli.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    config_root = Path(args.config_root) if args.config_root else package_root / "config"
    sim_config = load_sim_config(config_root)
    backend = args.backend or str(sim_config.get("backend", "recorded"))
    # 文件读取用 utf-8-sig：兼容带 BOM 的输入文件（stdin 无此问题）
    payload = (
        sys.stdin.read()
        if args.json_file == "-"
        else Path(args.json_file).read_text(encoding="utf-8-sig")
    )
    try:
        # 命令行没有 ROS 节点上下文，moveit2 后端必然拿不到传输层而失败；
        # 这是有意为之：真派发必须由节点内调用，而不是离线脚本。
        transport = create_moveit2_sim_transport(None) if backend == "moveit2" else None
        result = handle_sim_execution_json(payload, backend=backend, transport=transport)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        # 失败时也输出结构化 JSON 便于脚本消费，并以非零码退出
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)

"""可注入依赖的 ROS 2 动作（action）传输适配层。

仿真执行器通过本模块把安全路由下发为 ROS 2 动作目标。为了让纯逻辑测试
无需真实 ROS 2 环境即可运行，动作类型解析、目标构造、客户端工厂与等待
future 的自旋函数全部通过构造参数注入；只有生产路径才回落到函数内部延迟
导入的默认实现（因此缺少运行环境时抛的是 SafetyViolationError，而不是
ImportError）。

超时单位均为秒：等待动作服务器就绪用 wait_timeout_sec（默认 2.0 秒），
等待目标响应 future 用 goal_response_timeout_sec（默认 2.0 秒）。服务器
不可用时判为安全违规，调用方必须视为下发失败。
"""

from __future__ import annotations

from typing import Any, Callable

from .models import SafetyViolationError


ActionTypeResolver = Callable[[str], Any]  # 动作名 → 动作类型（如 MoveRelative）
GoalBuilder = Callable[[str, dict[str, Any]], Any]  # (动作名, 参数字典) → 目标消息
ActionClientFactory = Callable[[Any, Any, str], Any]  # (节点, 动作类型, 动作名) → 动作客户端
SpinUntilFutureComplete = Callable[[Any, Any, float | None], None]  # (节点, future, 超时秒) → None


def _default_action_client_factory(node: Any, action_type: Any, action_name: str) -> Any:
    """默认动作客户端工厂：延迟导入 rclpy，缺失时转成安全违规。"""
    try:
        from rclpy.action import ActionClient
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rclpy is required for ROS2 action transport") from exc
    return ActionClient(node, action_type, action_name)


def _default_spin_until_future_complete(
    node: Any,
    future: Any,
    timeout_sec: float | None,
) -> None:
    """默认自旋等待实现：要求 rclpy 提供 spin_until_future_complete。"""
    try:
        import rclpy
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rclpy is required for ROS2 action transport") from exc
    if not hasattr(rclpy, "spin_until_future_complete"):
        raise SafetyViolationError("rclpy.spin_until_future_complete is required for ROS2 action transport")
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)


class Ros2ActionTransport:
    """薄封装、依赖可注入的 ROS 2 动作传输层，供仿真执行器使用。"""

    def __init__(
        self,
        node: Any,
        action_type_resolver: ActionTypeResolver,
        goal_builder: GoalBuilder,
        action_client_factory: ActionClientFactory | None = None,
        spin_until_future_complete: SpinUntilFutureComplete | None = None,
        wait_timeout_sec: float = 2.0,
        goal_response_timeout_sec: float = 2.0,
    ):
        """保存注入的依赖与超时配置。

        参数：
            node：ROS 2 节点，用于创建动作客户端与自旋等待。
            action_type_resolver：动作名 → 动作类型的解析函数。
            goal_builder：把参数字典构造成动作目标消息的函数。
            action_client_factory：动作客户端工厂，缺省用真实实现。
            spin_until_future_complete：等待 future 的自旋函数，缺省用真实实现。
            wait_timeout_sec：等待动作服务器就绪的秒数，默认 2.0。
            goal_response_timeout_sec：等待目标响应 future 的秒数，默认 2.0。
        """
        self._node = node
        self._action_type_resolver = action_type_resolver
        self._goal_builder = goal_builder
        self._action_client_factory = action_client_factory or _default_action_client_factory
        self._spin_until_future_complete = (
            spin_until_future_complete or _default_spin_until_future_complete
        )
        self._wait_timeout_sec = wait_timeout_sec
        self._goal_response_timeout_sec = goal_response_timeout_sec

    def send_action_goal(self, action_name: str, goal: dict[str, Any]) -> dict[str, Any]:
        """同步下发一个动作目标，返回目标是否被服务器接受。

        参数：action_name 为动作名（须能被 resolver 识别），goal 为参数字典。
        返回：包含 action_name、goal_accepted、status 三个键的字典，status
        取 "accepted" 或 "rejected"。
        副作用：等待动作服务器、创建客户端、发送目标并自旋至 future 完成。
        服务器不可用时抛 SafetyViolationError；本方法只保证目标被接受，
        并不代表动作已执行完成。
        """
        action_type = self._action_type_resolver(action_name)
        client = self._action_client_factory(self._node, action_type, action_name)
        # 服务器未在超时内就绪即判失败，不做重试，避免命令悬挂。
        if not client.wait_for_server(timeout_sec=self._wait_timeout_sec):
            raise SafetyViolationError(f"action server unavailable: {action_name}")

        goal_msg = self._goal_builder(action_name, goal)
        future = client.send_goal_async(goal_msg)
        self._spin_until_future_complete(
            self._node,
            future,
            timeout_sec=self._goal_response_timeout_sec,
        )
        goal_handle = future.result()
        accepted = bool(getattr(goal_handle, "accepted", False))
        return {
            "action_name": action_name,
            "goal_accepted": accepted,
            "status": "accepted" if accepted else "rejected",
        }

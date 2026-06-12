from __future__ import annotations

from typing import Any, Callable

from .models import SafetyViolationError


ActionTypeResolver = Callable[[str], Any]
GoalBuilder = Callable[[str, dict[str, Any]], Any]
ActionClientFactory = Callable[[Any, Any, str], Any]
SpinUntilFutureComplete = Callable[[Any, Any, float | None], None]


def _default_action_client_factory(node: Any, action_type: Any, action_name: str) -> Any:
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
    try:
        import rclpy
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rclpy is required for ROS2 action transport") from exc
    if not hasattr(rclpy, "spin_until_future_complete"):
        raise SafetyViolationError("rclpy.spin_until_future_complete is required for ROS2 action transport")
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)


class Ros2ActionTransport:
    """Thin, injectable ROS2 action transport used by sim executors."""

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
        action_type = self._action_type_resolver(action_name)
        client = self._action_client_factory(self._node, action_type, action_name)
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

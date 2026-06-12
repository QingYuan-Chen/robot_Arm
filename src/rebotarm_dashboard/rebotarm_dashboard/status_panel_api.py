from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


class StatusPanelApiError(ValueError):
    pass


@dataclass(frozen=True)
class PostRoute:
    handler_name: str
    payload: bool = False
    argument: str | None = None


POST_ROUTES: dict[str, PostRoute] = {
    "/api/execute_preview": PostRoute("_handle_execute_preview", payload=True),
    "/api/stop_execute": PostRoute("_handle_stop_execute"),
    "/api/set_gripper": PostRoute("_handle_set_gripper", payload=True),
    "/api/keyboard_enable": PostRoute("_handle_keyboard_enable", payload=True),
    "/api/keyboard_disable": PostRoute("_handle_keyboard_disable"),
    "/api/keyboard_key": PostRoute("_handle_keyboard_key", payload=True),
    "/api/teach_record_start": PostRoute("_handle_teach_record_start", payload=True),
    "/api/teach_record_stop": PostRoute("_handle_teach_record_stop"),
    "/api/teach_dry_run": PostRoute("_handle_teach_dry_run", payload=True),
    "/api/teach_replay_execute": PostRoute("_handle_teach_replay_execute", payload=True),
    "/api/teach_replay_stop": PostRoute("_handle_teach_replay_stop"),
    "/api/arm_safe_home": PostRoute("_handle_arm_service_command", argument="safe_home"),
    "/api/arm_enable": PostRoute("_handle_arm_service_command", argument="enable"),
    "/api/arm_disable": PostRoute("_handle_arm_service_command", argument="disable"),
}


def post_paths() -> frozenset[str]:
    return frozenset(POST_ROUTES)


def is_allowed_post_path(path: str) -> bool:
    return path in POST_ROUTES


def dispatch_post_request(node: object, path: str, payload_reader: Callable[[], dict]) -> dict:
    route = POST_ROUTES.get(path)
    if route is None:
        raise StatusPanelApiError(f"unknown POST path: {path}")
    handler = getattr(node, route.handler_name)
    if route.payload:
        result = handler(payload_reader())
    elif route.argument is not None:
        result = handler(route.argument)
    else:
        result = handler()
    if not isinstance(result, dict):
        raise StatusPanelApiError(f"{route.handler_name} must return a dict")
    return result

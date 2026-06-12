from __future__ import annotations

import time
from typing import Any

from std_srvs.srv import Trigger

from .arm_command_api import (
    arm_command_timeout_sec,
    normalize_arm_command,
    should_stop_trajectory_before_arm_command,
)


class ArmControlClient:
    """Thin service-client facade for whole-arm operator commands."""

    def __init__(
        self,
        *,
        enable_client: Any,
        disable_client: Any,
        safe_home_client: Any,
        trajectory_stop_client: Any,
    ) -> None:
        self._clients = {
            "safe_home": safe_home_client,
            "enable": enable_client,
            "disable": disable_client,
        }
        self._trajectory_stop_client = trajectory_stop_client

    def execute(self, command: object) -> dict:
        normalized = normalize_arm_command(command)
        if normalized is None:
            return {
                "accepted": False,
                "state": "rejected",
                "command": str(command or ""),
                "message": "unknown arm command",
            }
        stop_requested = False
        stop_message = ""
        if should_stop_trajectory_before_arm_command(normalized):
            stop_requested, stop_message = self.call_trigger_service(
                self._trajectory_stop_client,
                timeout_sec=2.0,
            )
        ok, message = self.call_trigger_service(
            self._clients[normalized],
            timeout_sec=arm_command_timeout_sec(normalized),
        )
        if stop_message:
            message = f"{message}; trajectory_stop: {stop_message}" if message else f"trajectory_stop: {stop_message}"
        return {
            "accepted": ok,
            "state": "done" if ok else "failed",
            "command": normalized,
            "message": message or ("done" if ok else "failed"),
            "trajectory_stop_requested": stop_requested,
        }

    @staticmethod
    def call_trigger_service(client: Any, *, timeout_sec: float) -> tuple[bool, str]:
        try:
            if not client.wait_for_service(timeout_sec=min(timeout_sec, 0.5)):
                return False, "service unavailable"
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + max(float(timeout_sec), 0.1)
            while time.monotonic() < deadline:
                if future.done():
                    response = future.result()
                    return bool(getattr(response, "success", False)), str(getattr(response, "message", ""))
                time.sleep(0.02)
            return False, "service timeout"
        except Exception as exc:
            return False, str(exc)

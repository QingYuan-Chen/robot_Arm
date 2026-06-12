from __future__ import annotations

from typing import Any

from .service_call_helpers import call_trigger_service
from .teach_recording import validate_teach_replay_stop_request


class TeachReplayClient:
    """Small action/control facade for teach replay dashboard commands."""

    def stop(self, goal_handle: Any | None, *, trajectory_stop_client: Any) -> dict:
        decision = validate_teach_replay_stop_request(goal_handle is not None)
        if not decision.accepted:
            stop_requested, _message = call_trigger_service(
                trajectory_stop_client,
                timeout_sec=0.8,
            )
            if stop_requested:
                return {
                    "accepted": True,
                    "state": "stop_requested",
                    "message": "controller trajectory_stop requested",
                    "cancel_future": None,
                }
            return {
                "accepted": False,
                "state": decision.state,
                "message": decision.message,
                "cancel_future": None,
            }
        try:
            future = goal_handle.cancel_goal_async()
            call_trigger_service(trajectory_stop_client, timeout_sec=0.2)
        except Exception as exc:
            return {
                "accepted": False,
                "state": "failed",
                "message": f"failed to request teach replay cancel: {exc}",
                "cancel_future": None,
            }
        return {
            "accepted": True,
            "state": decision.state,
            "message": decision.message,
            "cancel_future": future,
        }

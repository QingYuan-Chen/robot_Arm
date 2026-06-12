from __future__ import annotations

import time
from typing import Any, Callable

from .service_call_helpers import call_trigger_service


class TeachRecordClient:
    """Coordinates teach-record ROS services for the dashboard layer."""

    def __init__(
        self,
        *,
        set_path_client: Any,
        start_client: Any,
        stop_client: Any,
        gravity_start_client: Any,
        gravity_stop_client: Any,
        record_path_request_factory: Callable[[], Any],
    ) -> None:
        self._set_path_client = set_path_client
        self._start_client = start_client
        self._stop_client = stop_client
        self._gravity_start_client = gravity_start_client
        self._gravity_stop_client = gravity_stop_client
        self._record_path_request_factory = record_path_request_factory

    def start(self, payload: dict | None = None) -> dict:
        payload = payload or {}
        requested_path = str(payload.get("record_path", "")).strip()
        normalized_path = ""
        if requested_path:
            set_path_ok, set_path_message, normalized_path = self._set_record_path(requested_path)
            if not set_path_ok:
                return {
                    "accepted": False,
                    "state": "blocked",
                    "message": f"record path: {set_path_message}",
                    "record_path": normalized_path or requested_path,
                }
        gravity_ok, gravity_message = call_trigger_service(
            self._gravity_start_client,
            timeout_sec=2.0,
        )
        record_ok, record_message = call_trigger_service(
            self._start_client,
            timeout_sec=2.0,
        )
        accepted = record_ok and (gravity_ok or "already" in gravity_message.lower())
        return {
            "accepted": accepted,
            "state": "starting" if accepted else "blocked",
            "message": f"gravity: {gravity_message or gravity_ok}; record: {record_message or record_ok}",
            "gravity_started": gravity_ok,
            "record_started": record_ok,
            "record_path": normalized_path or requested_path,
        }

    def stop(self) -> dict:
        record_ok, record_message = call_trigger_service(
            self._stop_client,
            timeout_sec=2.0,
        )
        gravity_ok, gravity_message = call_trigger_service(
            self._gravity_stop_client,
            timeout_sec=2.0,
        )
        return {
            "accepted": record_ok,
            "state": "stopped" if record_ok else "failed",
            "message": f"record: {record_message or record_ok}; gravity: {gravity_message or gravity_ok}",
            "record_stopped": record_ok,
            "gravity_stopped": gravity_ok,
        }

    def _set_record_path(self, requested_path: str) -> tuple[bool, str, str]:
        try:
            if not self._set_path_client.wait_for_service(timeout_sec=0.5):
                return False, "record path service unavailable", ""
            request = self._record_path_request_factory()
            request.record_path = requested_path
            future = self._set_path_client.call_async(request)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not future.done():
                time.sleep(0.02)
            if not future.done():
                return False, "record path service timeout", ""
            response = future.result()
            return (
                bool(getattr(response, "success", False)),
                str(getattr(response, "message", "")),
                str(getattr(response, "normalized_path", "")),
            )
        except Exception as exc:
            return False, str(exc), ""

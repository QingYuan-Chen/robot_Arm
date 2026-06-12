from __future__ import annotations

import time

from std_srvs.srv import Trigger


def call_trigger_service(client, *, timeout_sec: float) -> tuple[bool, str]:
    try:
        if not client.wait_for_service(timeout_sec=min(float(timeout_sec), 0.5)):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, "service timeout"
        response = future.result()
        return bool(getattr(response, "success", False)), str(getattr(response, "message", ""))
    except Exception as exc:
        return False, str(exc)

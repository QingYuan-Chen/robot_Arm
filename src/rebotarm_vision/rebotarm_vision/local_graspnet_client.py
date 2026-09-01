from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

from .graspnet_service_contract import encode_inference_request


def closest_timestamped_frame(frames, target_timestamp_ns: int):
    """Return the cached frame nearest to a non-zero target timestamp."""
    if target_timestamp_ns <= 0 or not frames:
        return None
    return min(frames, key=lambda item: abs(int(item[0]) - target_timestamp_ns))


@dataclass(frozen=True)
class LocalGraspNetConfig:
    infer_url: str
    timeout_ms: int = 2000


class LocalGraspNetClient:
    def __init__(self, config: LocalGraspNetConfig) -> None:
        self._config = config
        self._last_debug_message = "client_not_used"

    def infer(
        self,
        *,
        timestamp_ns: int,
        frame_id: str,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: dict[str, float],
        bbox: dict[str, Any],
        max_grasps: int,
        max_jaw_width_m: float | None = None,
        mask_polygon_xy: list[float] | None = None,
    ) -> dict[str, Any]:
        payload = encode_inference_request(
            timestamp_ns=timestamp_ns,
            frame_id=frame_id,
            color_bgr=color_bgr,
            depth_m=depth_m,
            intrinsics=intrinsics,
            bbox=bbox,
            max_grasps=max_grasps,
            max_jaw_width_m=max_jaw_width_m,
            mask_polygon_xy=mask_polygon_xy,
        )
        request = Request(
            self._config.infer_url,
            data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            headers={"Content-Type": "application/json", "User-Agent": "rebotarm_vision/0.1"},
            method="POST",
        )
        timeout = max(int(self._config.timeout_ms), 1) / 1000.0
        try:
            with urlopen(request, timeout=timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
            if not isinstance(result, dict):
                raise ValueError("invalid JSON root")
            self._last_debug_message = f"ok candidates={len(result.get('candidates', []))}"
            return result
        except HTTPError as exc:
            try:
                result = json.loads(exc.read().decode("utf-8"))
            except Exception:
                result = {}
            self._last_debug_message = f"http:{exc.code}:{result.get('error', 'unknown')}"
            return self._empty(timestamp_ns, frame_id, stale=bool(result.get("stale", True)))
        except Exception as exc:
            self._last_debug_message = f"exception:{type(exc).__name__}:{exc}"
            return self._empty(timestamp_ns, frame_id, stale=True)

    @staticmethod
    def _empty(timestamp_ns: int, frame_id: str, *, stale: bool) -> dict[str, Any]:
        return {
            "source": "ubuntu_local_graspnet",
            "backend_configured": False,
            "stale": bool(stale),
            "timestamp_ns": int(timestamp_ns),
            "frame_id": str(frame_id),
            "candidates": [],
        }

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message

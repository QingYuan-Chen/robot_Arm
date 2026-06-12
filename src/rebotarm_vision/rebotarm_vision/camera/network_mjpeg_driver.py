from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Optional
from urllib.request import Request, urlopen

import cv2
import numpy as np


@dataclass
class NetworkMjpegConfig:
    snapshot_url: str
    stream_url: str
    frame_timeout_ms: int
    depth_url: str = ""
    camera_info_url: str = ""


class NetworkMjpegDriver:
    def __init__(self, config: NetworkMjpegConfig) -> None:
        self._config = config
        self._capture = None
        self._last_color_bgr: Optional[np.ndarray] = None
        self._last_depth_mm: Optional[np.ndarray] = None
        self._last_camera_info: Optional[dict] = None
        self._last_debug_message = "driver_not_opened"

    def open(self) -> None:
        if self._config.snapshot_url:
            self._last_debug_message = "snapshot_ready"
            return
        if not self._config.stream_url and not self._config.depth_url:
            raise RuntimeError("network_mjpeg requires camera.network_snapshot_url or camera.network_stream_url")
        self._capture = cv2.VideoCapture(self._config.stream_url)
        if not self._capture.isOpened():
            self._capture.release()
            self._capture = None
            raise RuntimeError(f"failed to open network stream: {self._config.stream_url}")
        self._last_debug_message = "stream_ready"

    def warmup(self, frames: int) -> bool:
        attempts = max(frames, 1)
        for _ in range(attempts):
            color_bgr, depth_mm = self.get_frame()
            if color_bgr is not None or depth_mm is not None:
                return True
        return False

    def get_frame(self, allow_partial: bool = False) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        depth_mm = self._get_depth_frame() if self._config.depth_url else None
        if self._config.snapshot_url:
            return self._get_snapshot_frame(), depth_mm
        if self._config.stream_url:
            return self._get_stream_frame(), depth_mm
        return None, depth_mm

    def _get_snapshot_frame(self) -> Optional[np.ndarray]:
        timeout = max(self._config.frame_timeout_ms, 1) / 1000.0
        try:
            request = Request(self._config.snapshot_url, headers={"User-Agent": "rebotarm_vision/0.1"})
            with urlopen(request, timeout=timeout) as response:
                payload = response.read()
            encoded = np.frombuffer(payload, dtype=np.uint8)
            color_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if color_bgr is None:
                self._last_debug_message = "snapshot_decode_failed"
                return self._last_color_bgr
            color_bgr = np.ascontiguousarray(color_bgr)
            self._last_color_bgr = color_bgr
            self._last_debug_message = (
                f"snapshot_ok color={color_bgr.shape[1]}x{color_bgr.shape[0]} "
                f"std={float(color_bgr.std()):.1f}"
            )
            return color_bgr
        except Exception as exc:
            self._last_debug_message = f"snapshot_exception:{type(exc).__name__}:{exc}"
            return self._last_color_bgr

    def _get_depth_frame(self) -> Optional[np.ndarray]:
        timeout = max(self._config.frame_timeout_ms, 1) / 1000.0
        try:
            request = Request(self._config.depth_url, headers={"User-Agent": "rebotarm_vision/0.1"})
            with urlopen(request, timeout=timeout) as response:
                payload = response.read()
            encoded = np.frombuffer(payload, dtype=np.uint8)
            depth_mm = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
            if depth_mm is None:
                self._last_debug_message = "depth_decode_failed"
                return self._last_depth_mm
            if depth_mm.dtype != np.uint16:
                depth_mm = depth_mm.astype(np.uint16)
            depth_mm = np.ascontiguousarray(depth_mm)
            self._last_depth_mm = depth_mm
            self._last_debug_message = (
                f"depth_ok depth={depth_mm.shape[1]}x{depth_mm.shape[0]} "
                f"valid={int(np.count_nonzero(depth_mm))}"
            )
            return depth_mm
        except Exception as exc:
            self._last_debug_message = f"depth_exception:{type(exc).__name__}:{exc}"
            return self._last_depth_mm

    def get_camera_info(self) -> Optional[dict]:
        if not self._config.camera_info_url:
            return self._last_camera_info
        timeout = max(self._config.frame_timeout_ms, 1) / 1000.0
        try:
            request = Request(self._config.camera_info_url, headers={"User-Agent": "rebotarm_vision/0.1"})
            with urlopen(request, timeout=timeout) as response:
                payload = response.read().decode("utf-8")
            parsed = json.loads(payload)
            if isinstance(parsed, dict):
                self._last_camera_info = parsed
            return self._last_camera_info
        except Exception as exc:
            self._last_debug_message = f"camera_info_exception:{type(exc).__name__}:{exc}"
            return self._last_camera_info

    def _get_stream_frame(self) -> Optional[np.ndarray]:
        if self._capture is None:
            self._last_debug_message = "stream_capture_none"
            return self._last_color_bgr
        ok, frame = self._capture.read()
        if not ok or frame is None:
            self._last_debug_message = "stream_read_failed"
            return self._last_color_bgr
        color_bgr = np.ascontiguousarray(frame)
        self._last_color_bgr = color_bgr
        self._last_debug_message = (
            f"stream_ok color={color_bgr.shape[1]}x{color_bgr.shape[0]} "
            f"std={float(color_bgr.std()):.1f}"
        )
        return color_bgr

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message

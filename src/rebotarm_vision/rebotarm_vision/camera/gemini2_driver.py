from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class Gemini2Config:
    color_width: int
    color_height: int
    color_fps: int
    enable_depth: bool
    depth_width: int
    depth_height: int
    depth_fps: int
    frame_timeout_ms: int
    enable_align: bool


class Gemini2Driver:
    def __init__(self, config: Gemini2Config) -> None:
        self._config = config
        self._pipeline = None
        self._format_convert_filter = None
        self._last_color_bgr: Optional[np.ndarray] = None
        self._last_depth_mm: Optional[np.ndarray] = None
        self._last_debug_message = "driver_not_opened"

    @staticmethod
    def _ensure_contiguous(array: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if array is None:
            return None
        return np.ascontiguousarray(array)

    def open(self) -> None:
        from pyorbbecsdk import Config, FormatConvertFilter, OBAlignMode, OBFormat, OBSensorType, Pipeline

        self._pipeline = Pipeline()
        self._format_convert_filter = FormatConvertFilter()
        config = Config()

        color_profiles = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = None
        for fmt in (OBFormat.MJPG, OBFormat.RGB):
            try:
                color_profile = color_profiles.get_video_stream_profile(
                    self._config.color_width,
                    self._config.color_height,
                    fmt,
                    self._config.color_fps,
                )
                break
            except Exception:
                pass
        if color_profile is None:
            color_profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)

        if self._config.enable_depth:
            depth_profiles = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = None
            if self._config.depth_width > 0 and self._config.depth_height > 0:
                try:
                    depth_profile = depth_profiles.get_video_stream_profile(
                        self._config.depth_width,
                        self._config.depth_height,
                        OBFormat.Y16,
                        self._config.depth_fps,
                    )
                except Exception:
                    depth_profile = None
            if depth_profile is None:
                depth_profile = depth_profiles.get_default_video_stream_profile()
            config.enable_stream(depth_profile)

        if self._config.enable_align:
            config.set_align_mode(OBAlignMode.HW_MODE)

        self._pipeline.start(config)

    def warmup(self, frames: int) -> bool:
        got_color = False
        got_depth = not self._config.enable_depth
        for _ in range(max(frames, 0)):
            color_bgr, depth_mm = self.get_frame(allow_partial=True)
            got_color = got_color or color_bgr is not None
            got_depth = got_depth or depth_mm is not None
            if got_color and got_depth:
                return True
        return got_color and got_depth

    def get_frame(self, allow_partial: bool = False) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._pipeline is None:
            self._last_debug_message = "pipeline_none"
            return None, None

        try:
            from pyorbbecsdk import OBFormat

            frames = self._pipeline.wait_for_frames(self._config.frame_timeout_ms)
            if frames is None:
                self._last_debug_message = "wait_for_frames_none"
                return None, None

            color_bgr = None
            color_frame = frames.get_color_frame()
            if color_frame is not None:
                width = color_frame.get_width()
                height = color_frame.get_height()
                fmt = color_frame.get_format()
                if fmt == OBFormat.MJPG and self._format_convert_filter is not None:
                    from pyorbbecsdk import OBConvertFormat

                    self._format_convert_filter.set_format_convert_format(
                        OBConvertFormat.MJPG_TO_BGR888
                    )
                    converted = self._format_convert_filter.process(color_frame)
                    if converted is not None:
                        converted_data = np.asanyarray(converted.get_data(), dtype=np.uint8)
                        color_bgr = np.resize(converted_data, (height, width, 3))
                    else:
                        self._last_debug_message = "mjpg_convert_failed"
                else:
                    raw = np.asanyarray(color_frame.get_data(), dtype=np.uint8)
                    if fmt == OBFormat.RGB:
                        rgb = np.ascontiguousarray(np.resize(raw, (height, width, 3)))
                        color_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    else:
                        color_bgr = np.resize(raw, (height, width, 3))
                color_bgr = self._ensure_contiguous(color_bgr)
                self._last_color_bgr = color_bgr
            else:
                self._last_debug_message = "color_frame_none"

            depth_mm = None
            depth_frame = frames.get_depth_frame()
            if depth_frame is not None:
                width = depth_frame.get_width()
                height = depth_frame.get_height()
                depth_bytes = bytes(depth_frame.get_data())
                depth_mm = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(height, width)
                depth_mm = self._ensure_contiguous(depth_mm)
                self._last_depth_mm = depth_mm
            else:
                self._last_debug_message = "depth_frame_none"

            if allow_partial:
                if color_bgr is not None and depth_mm is not None:
                    self._last_debug_message = (
                        f"partial_ok color={color_bgr.shape[1]}x{color_bgr.shape[0]} "
                        f"depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                    )
                elif color_bgr is not None:
                    self._last_debug_message = (
                        f"partial_color_only color={color_bgr.shape[1]}x{color_bgr.shape[0]}"
                    )
                elif depth_mm is not None:
                    self._last_debug_message = (
                        f"partial_depth_only depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                    )
                return color_bgr, depth_mm

            if color_bgr is None:
                color_bgr = self._last_color_bgr
            if depth_mm is None:
                depth_mm = self._last_depth_mm

            if color_bgr is not None and depth_mm is not None:
                self._last_debug_message = (
                    f"frame_ok color={color_bgr.shape[1]}x{color_bgr.shape[0]} "
                    f"depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                )
            elif color_bgr is not None:
                self._last_debug_message = (
                    f"frame_color_only color={color_bgr.shape[1]}x{color_bgr.shape[0]}"
                )
            elif depth_mm is not None:
                self._last_debug_message = (
                    f"frame_depth_only depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                )
            else:
                self._last_debug_message = "frame_none_after_fallback"

            return color_bgr, depth_mm
        except Exception as exc:
            self._last_debug_message = f"exception:{type(exc).__name__}:{exc}"
            return None, None

    def close(self) -> None:
        if self._pipeline is None:
            return
        try:
            self._pipeline.stop()
        except Exception:
            pass
        self._pipeline = None
        self._format_convert_filter = None

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message

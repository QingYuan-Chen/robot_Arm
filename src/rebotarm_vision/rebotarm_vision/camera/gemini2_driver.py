from __future__ import annotations

from copy import deepcopy
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
        self._camera_info_by_stream: dict[str, dict] = {}
        self._device_info: dict = {}
        self._calibration_info: dict = {}
        self._stream_profiles: dict[str, dict] = {}
        self._frame_metadata: dict[str, dict] = {}
        self._depth_scale_mm: Optional[float] = None
        self._last_debug_message = "driver_not_opened"

    @staticmethod
    def _ensure_contiguous(array: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if array is None:
            return None
        return np.ascontiguousarray(array)

    @staticmethod
    def _select_video_profile(
        profiles,
        *,
        stream_name: str,
        width: int,
        height: int,
        formats: tuple,
        fps: int,
    ):
        if width <= 0 or height <= 0:
            return profiles.get_default_video_stream_profile()

        errors = []
        for fmt in formats:
            try:
                return profiles.get_video_stream_profile(width, height, fmt, fps)
            except Exception as exc:
                errors.append(f"{fmt}: {type(exc).__name__}: {exc}")
        requested = f"{width}x{height}@{fps} formats={list(map(str, formats))}"
        raise RuntimeError(
            f"requested {stream_name} profile unavailable: {requested}; "
            + "; ".join(errors)
        )

    @staticmethod
    def _camera_info_from_profile(profile) -> dict:
        intrinsic = profile.get_intrinsic()
        distortion = profile.get_distortion()
        return {
            "width": int(intrinsic.width),
            "height": int(intrinsic.height),
            "fx": float(intrinsic.fx),
            "fy": float(intrinsic.fy),
            "cx": float(intrinsic.cx),
            "cy": float(intrinsic.cy),
            "distortion_model": "rational_polynomial",
            "d": [
                float(distortion.k1),
                float(distortion.k2),
                float(distortion.p1),
                float(distortion.p2),
                float(distortion.k3),
                float(distortion.k4),
                float(distortion.k5),
                float(distortion.k6),
            ],
        }

    @staticmethod
    def _profile_payload(profile) -> dict:
        return {
            "width": int(profile.get_width()),
            "height": int(profile.get_height()),
            "fps": int(profile.get_fps()),
            "format": str(profile.get_format()),
        }

    @classmethod
    def _profile_list_payload(cls, profiles) -> list[dict]:
        result = []
        for index in range(profiles.get_count()):
            profile = profiles.get_stream_profile_by_index(index)
            if not profile.is_video_stream_profile():
                continue
            result.append(cls._profile_payload(profile.as_video_stream_profile()))
        return result

    @staticmethod
    def _extrinsic_payload(extrinsic) -> dict:
        rotation = np.asarray(extrinsic.rot, dtype=float).reshape(3, 3)
        translation = np.asarray(extrinsic.transform, dtype=float).reshape(3)
        return {
            "rotation": rotation.tolist(),
            "translation_mm": translation.tolist(),
        }

    @staticmethod
    def _optional_call(obj, method_name: str, default=None):
        method = getattr(obj, method_name, None)
        if not callable(method):
            return default
        try:
            return method()
        except Exception:
            return default

    def open(self) -> None:
        from pyorbbecsdk import Config, FormatConvertFilter, OBAlignMode, OBFormat, OBSensorType, Pipeline

        self._pipeline = Pipeline()
        self._format_convert_filter = FormatConvertFilter()
        config = Config()

        color_profiles = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = self._select_video_profile(
            color_profiles,
            stream_name="color",
            width=self._config.color_width,
            height=self._config.color_height,
            formats=(OBFormat.MJPG, OBFormat.RGB),
            fps=self._config.color_fps,
        )
        config.enable_stream(color_profile)
        self._stream_profiles["color"] = {
            "selected": self._profile_payload(color_profile),
            "supported": self._profile_list_payload(color_profiles),
        }

        depth_profile = None
        if self._config.enable_depth:
            depth_profiles = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = self._select_video_profile(
                depth_profiles,
                stream_name="depth",
                width=self._config.depth_width,
                height=self._config.depth_height,
                formats=(OBFormat.Y16,),
                fps=self._config.depth_fps,
            )
            config.enable_stream(depth_profile)
            self._stream_profiles["depth"] = {
                "selected": self._profile_payload(depth_profile),
                "supported": self._profile_list_payload(depth_profiles),
            }

        if self._config.enable_align:
            config.set_align_mode(OBAlignMode.HW_MODE)
        set_depth_scale_require = getattr(config, "set_depth_scale_require", None)
        if self._config.enable_depth and callable(set_depth_scale_require):
            set_depth_scale_require(True)

        self._pipeline.start(config)
        device = self._pipeline.get_device()
        device_info = device.get_device_info()
        self._device_info = {
            "name": self._optional_call(device_info, "get_name", ""),
            "serial_number": self._optional_call(device_info, "get_serial_number", ""),
            "firmware_version": self._optional_call(device_info, "get_firmware_version", ""),
            "hardware_version": self._optional_call(device_info, "get_hardware_version", ""),
            "connection_type": self._optional_call(device_info, "get_connection_type", ""),
            "uid": self._optional_call(device_info, "get_uid", ""),
            "vid": self._optional_call(device_info, "get_vid", 0),
            "pid": self._optional_call(device_info, "get_pid", 0),
            "supported_min_sdk_version": self._optional_call(
                device_info, "get_supported_min_sdk_version", ""
            ),
        }
        self._camera_info_by_stream["color"] = self._camera_info_from_profile(color_profile)
        if depth_profile is not None:
            native_depth_info = self._camera_info_from_profile(depth_profile)
            self._camera_info_by_stream["depth_native"] = native_depth_info
            self._camera_info_by_stream["depth"] = deepcopy(
                self._camera_info_by_stream["color"]
                if self._config.enable_align
                else native_depth_info
            )
            self._calibration_info = {
                "color_to_depth": self._extrinsic_payload(
                    color_profile.get_extrinsic_to(depth_profile)
                ),
                "depth_to_color": self._extrinsic_payload(
                    depth_profile.get_extrinsic_to(color_profile)
                ),
                "aligned_depth_uses_color_coordinates": bool(self._config.enable_align),
            }

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
                self._frame_metadata["color"] = self._frame_metadata_payload(color_frame)
            else:
                self._last_debug_message = "color_frame_none"

            depth_mm = None
            depth_frame = frames.get_depth_frame()
            if depth_frame is not None:
                width = depth_frame.get_width()
                height = depth_frame.get_height()
                depth_bytes = bytes(depth_frame.get_data())
                depth_raw = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(height, width)
                depth_scale_mm = float(self._optional_call(depth_frame, "get_depth_scale", 1.0))
                self._depth_scale_mm = depth_scale_mm
                depth_mm = np.rint(depth_raw.astype(np.float32) * depth_scale_mm)
                depth_mm = np.clip(depth_mm, 0, np.iinfo(np.uint16).max).astype(np.uint16)
                depth_mm = self._ensure_contiguous(depth_mm)
                self._last_depth_mm = depth_mm
                self._frame_metadata["depth"] = self._frame_metadata_payload(
                    depth_frame,
                    depth_scale_mm=depth_scale_mm,
                )
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

    @classmethod
    def _frame_metadata_payload(cls, frame, *, depth_scale_mm=None) -> dict:
        payload = {
            "width": int(cls._optional_call(frame, "get_width", 0)),
            "height": int(cls._optional_call(frame, "get_height", 0)),
            "format": str(cls._optional_call(frame, "get_format", "")),
            "timestamp_us": int(cls._optional_call(frame, "get_timestamp_us", 0)),
            "system_timestamp_us": int(
                cls._optional_call(frame, "get_system_timestamp_us", 0)
            ),
            "frame_index": int(cls._optional_call(frame, "get_index", 0)),
        }
        if depth_scale_mm is not None:
            payload["depth_scale_mm"] = float(depth_scale_mm)
        return payload

    def get_camera_info(self, stream: str = "depth") -> Optional[dict]:
        payload = self._camera_info_by_stream.get(stream)
        return deepcopy(payload) if payload is not None else None

    def get_device_info(self) -> dict:
        return deepcopy(self._device_info)

    def get_calibration_info(self) -> dict:
        payload = deepcopy(self._calibration_info)
        payload["depth_scale_mm"] = self._depth_scale_mm
        return payload

    def get_stream_profiles(self) -> dict:
        return deepcopy(self._stream_profiles)

    def get_frame_metadata(self) -> dict:
        return deepcopy(self._frame_metadata)

    def get_frame_timestamp_ns(self, stream: str) -> Optional[int]:
        timestamp_us = int(
            self._frame_metadata.get(stream, {}).get("system_timestamp_us", 0) or 0
        )
        return timestamp_us * 1000 if timestamp_us > 0 else None

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message

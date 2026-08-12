from __future__ import annotations

import base64
from dataclasses import dataclass
import time
from typing import Any

import numpy as np


CONTRACT_VERSION = "1.0"


class ContractError(ValueError):
    pass


@dataclass(frozen=True)
class GraspNetInferenceInput:
    timestamp_ns: int
    sent_at_unix_ns: int
    frame_id: str
    color_bgr: np.ndarray
    depth_m: np.ndarray
    camera_info: dict[str, float]
    detection: dict[str, Any]
    max_grasps: int


def decode_inference_request(payload: Any) -> GraspNetInferenceInput:
    if not isinstance(payload, dict):
        raise ContractError("request root must be an object")
    if payload.get("contract_version") != CONTRACT_VERSION:
        raise ContractError(f"contract_version must be {CONTRACT_VERSION}")

    timestamp_ns = _positive_int(payload.get("timestamp_ns"), "timestamp_ns")
    sent_at_unix_ns = _positive_int(
        payload.get("sent_at_unix_ns", timestamp_ns),
        "sent_at_unix_ns",
    )
    frame_id = str(payload.get("frame_id", "")).strip()
    if not frame_id:
        raise ContractError("frame_id must be non-empty")

    color_bgr = _decode_image(payload.get("rgb"), name="rgb", encoding="bgr8", dtype=np.uint8, channels=3)
    depth_m = _decode_image(payload.get("depth"), name="depth", encoding="32FC1", dtype=np.float32, channels=1)
    if payload.get("depth", {}).get("unit") != "m":
        raise ContractError("depth.unit must be m")
    if color_bgr.shape[:2] != depth_m.shape:
        raise ContractError("rgb and depth dimensions must match")
    if not np.isfinite(depth_m).all() or np.any(depth_m < 0.0):
        raise ContractError("depth must contain finite, non-negative metric values")

    intrinsics = payload.get("intrinsics")
    if not isinstance(intrinsics, dict):
        raise ContractError("intrinsics must be an object")
    camera_info = {key: _finite_float(intrinsics.get(key), f"intrinsics.{key}") for key in ("fx", "fy", "cx", "cy")}
    if camera_info["fx"] <= 0.0 or camera_info["fy"] <= 0.0:
        raise ContractError("intrinsics fx/fy must be positive")
    camera_info["depth_scale_m"] = 1.0

    bbox = payload.get("bbox")
    if not isinstance(bbox, dict):
        raise ContractError("bbox must be an object")
    detection = {
        "x_min": _finite_float(bbox.get("x_min"), "bbox.x_min"),
        "y_min": _finite_float(bbox.get("y_min"), "bbox.y_min"),
        "x_max": _finite_float(bbox.get("x_max"), "bbox.x_max"),
        "y_max": _finite_float(bbox.get("y_max"), "bbox.y_max"),
        "confidence": _finite_float(bbox.get("confidence", 0.0), "bbox.confidence"),
        "class_name": str(bbox.get("class_name", "")),
    }
    height, width = depth_m.shape
    if not (0.0 <= detection["x_min"] < detection["x_max"] <= width):
        raise ContractError("bbox x range is outside image bounds")
    if not (0.0 <= detection["y_min"] < detection["y_max"] <= height):
        raise ContractError("bbox y range is outside image bounds")

    max_grasps = int(payload.get("max_grasps", 10))
    if not 1 <= max_grasps <= 100:
        raise ContractError("max_grasps must be in [1, 100]")
    return GraspNetInferenceInput(
        timestamp_ns=timestamp_ns,
        sent_at_unix_ns=sent_at_unix_ns,
        frame_id=frame_id,
        color_bgr=color_bgr,
        depth_m=depth_m,
        camera_info=camera_info,
        detection=detection,
        max_grasps=max_grasps,
    )


def build_inference_response(request: GraspNetInferenceInput, candidates: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "contract_version": CONTRACT_VERSION,
        "source": "ubuntu_local_graspnet",
        "backend_configured": True,
        "stale": False,
        "timestamp_ns": request.timestamp_ns,
        "frame_id": request.frame_id,
        "candidates": candidates,
    }


def encode_inference_request(
    *,
    timestamp_ns: int,
    frame_id: str,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: dict[str, float],
    bbox: dict[str, Any],
    max_grasps: int,
    sent_at_unix_ns: int | None = None,
) -> dict[str, Any]:
    color = np.ascontiguousarray(color_bgr, dtype=np.uint8)
    depth = np.ascontiguousarray(depth_m, dtype=np.float32)
    if color.ndim != 3 or color.shape[2] != 3:
        raise ContractError("color_bgr must have shape HxWx3")
    if depth.ndim != 2 or color.shape[:2] != depth.shape:
        raise ContractError("depth_m must match color image dimensions")
    height, width = depth.shape
    payload = {
        "contract_version": CONTRACT_VERSION,
        "timestamp_ns": int(timestamp_ns),
        # Transport freshness must remain wall-clock based even when
        # timestamp_ns belongs to a ROS simulation clock.
        "sent_at_unix_ns": int(sent_at_unix_ns or time.time_ns()),
        "frame_id": str(frame_id),
        "rgb": {
            "encoding": "bgr8",
            "width": width,
            "height": height,
            "data_base64": base64.b64encode(color.tobytes()).decode("ascii"),
        },
        "depth": {
            "encoding": "32FC1",
            "unit": "m",
            "width": width,
            "height": height,
            "data_base64": base64.b64encode(depth.tobytes()).decode("ascii"),
        },
        "intrinsics": dict(intrinsics),
        "bbox": dict(bbox),
        "max_grasps": int(max_grasps),
    }
    decode_inference_request(payload)
    return payload


def _decode_image(value: Any, *, name: str, encoding: str, dtype, channels: int) -> np.ndarray:
    if not isinstance(value, dict):
        raise ContractError(f"{name} must be an object")
    if value.get("encoding") != encoding:
        raise ContractError(f"{name}.encoding must be {encoding}")
    width = _positive_int(value.get("width"), f"{name}.width")
    height = _positive_int(value.get("height"), f"{name}.height")
    raw = value.get("data_base64")
    if not isinstance(raw, str):
        raise ContractError(f"{name}.data_base64 must be a string")
    try:
        decoded = base64.b64decode(raw, validate=True)
    except Exception as exc:
        raise ContractError(f"{name}.data_base64 is invalid") from exc
    expected = width * height * channels * np.dtype(dtype).itemsize
    if len(decoded) != expected:
        raise ContractError(f"{name} byte length mismatch: expected {expected}, got {len(decoded)}")
    shape = (height, width, channels) if channels > 1 else (height, width)
    return np.frombuffer(decoded, dtype=dtype).reshape(shape).copy()


def _positive_int(value: Any, name: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"{name} must be an integer") from exc
    if result <= 0:
        raise ContractError(f"{name} must be positive")
    return result


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"{name} must be numeric") from exc
    if not np.isfinite(result):
        raise ContractError(f"{name} must be finite")
    return result

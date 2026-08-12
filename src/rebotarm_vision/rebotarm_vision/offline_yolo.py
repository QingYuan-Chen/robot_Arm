"""Pure helpers for the software-only RGB YOLO validation path.

The MuJoCo virtual camera publishes ``sensor_msgs/Image`` messages instead of
using a camera driver.  These helpers deliberately keep image decoding and
target-class filtering independent from ROS node lifecycle code so the safety
contract can be tested without starting a simulator or a real robot.
"""

from __future__ import annotations

import ast
import json
from typing import Iterable

import numpy as np

_ENCODING_CHANNELS = {
    "rgb8": (3, "rgb"),
    "bgr8": (3, "bgr"),
    "rgba8": (4, "rgba"),
    "bgra8": (4, "bgra"),
}


def parse_target_classes(value) -> tuple[str, ...]:
    """Parse a ROS parameter into normalized class names.

    Launch files often pass a list as a YAML-looking string.  Empty input is
    valid and intentionally means *reject every detection* (fail-closed).
    """

    if value is None:
        return ()
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return ()
        parsed = None
        for parser in (json.loads, ast.literal_eval):
            try:
                parsed = parser(text)
            except (ValueError, SyntaxError, TypeError, json.JSONDecodeError):
                continue
            if isinstance(parsed, (list, tuple, set)):
                value = parsed
                break
        else:
            value = text.split(",")
    if isinstance(value, (str, bytes)):
        value = [value]
    try:
        names = tuple(_normalize_class_name(item) for item in value)
    except TypeError as exc:
        raise ValueError("target_classes must be a sequence or comma-separated string") from exc
    return tuple(name for name in names if name)


def _normalize_class_name(value) -> str:
    return " ".join(str(value).strip().casefold().split())


def decode_image_to_bgr(image_msg) -> np.ndarray:
    """Decode a ROS 8-bit color image into a contiguous BGR NumPy array."""

    encoding = str(getattr(image_msg, "encoding", "")).strip().casefold()
    try:
        channels, layout = _ENCODING_CHANNELS[encoding]
    except KeyError as exc:
        raise ValueError(
            f"offline YOLO supports rgb8/bgr8/rgba8/bgra8, got {encoding!r}"
        ) from exc

    height = int(getattr(image_msg, "height", 0))
    width = int(getattr(image_msg, "width", 0))
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    row_bytes = width * channels
    step = int(getattr(image_msg, "step", 0) or row_bytes)
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than row width {row_bytes}")
    payload = memoryview(getattr(image_msg, "data", b""))
    required = step * height
    if len(payload) < required:
        raise ValueError(f"image payload has {len(payload)} bytes, needs at least {required}")

    rows = np.frombuffer(payload[:required], dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :row_bytes].reshape(height, width, channels)
    if layout == "rgb":
        bgr = pixels[..., ::-1]
    elif layout == "bgr":
        bgr = pixels
    elif layout == "rgba":
        bgr = pixels[..., 2::-1]
    else:  # bgra
        bgr = pixels[..., :3]
    return np.ascontiguousarray(bgr)


def filter_detection_array(
    detection_msg,
    target_classes: Iterable[str],
):
    """Return only detections whose class is explicitly allow-listed."""

    allowed = {_normalize_class_name(name) for name in target_classes if str(name).strip()}
    filtered = type(detection_msg)()
    filtered.header = detection_msg.header
    if not hasattr(filtered, "detections"):
        filtered.detections = []
    for detection in detection_msg.detections:
        if _normalize_class_name(getattr(detection, "class_name", "")) in allowed:
            filtered.detections.append(detection)
    return filtered

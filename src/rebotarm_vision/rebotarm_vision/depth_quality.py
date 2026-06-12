from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthQualityConfig:
    enabled: bool = True
    min_valid_pixels: int = 80
    min_valid_ratio: float = 0.20
    min_depth_m: float = 0.15
    max_depth_m: float = 1.20
    max_depth_mad_m: float = 0.025
    max_depth_span_m: float = 0.080
    center_window_px: int = 9
    min_center_valid_ratio: float = 0.30


@dataclass(frozen=True)
class DepthQualityResult:
    accepted: bool
    reason: str
    valid_depth_pixels: int = 0
    valid_depth_ratio: float = 0.0
    z_median_m: float = 0.0
    z_mad_m: float = 0.0
    z_span_m: float = 0.0
    center_valid_ratio: float = 0.0


def evaluate_detection_depth_quality(det, depth_mm: np.ndarray, config: DepthQualityConfig) -> DepthQualityResult:
    if not config.enabled:
        return DepthQualityResult(accepted=True, reason="disabled")
    if depth_mm is None or depth_mm.size == 0:
        return DepthQualityResult(accepted=False, reason="missing_depth")

    mask = _detection_mask(det, depth_mm.shape[:2])
    pixel_count = int(np.count_nonzero(mask))
    if pixel_count <= 0:
        return DepthQualityResult(accepted=False, reason="empty_roi")

    values_mm = np.asarray(depth_mm[mask > 0], dtype=np.float32)
    valid_mm = values_mm[values_mm > 0.0]
    valid_pixels = int(valid_mm.size)
    valid_ratio = float(valid_pixels) / float(pixel_count)
    if valid_pixels < int(config.min_valid_pixels):
        return DepthQualityResult(
            accepted=False,
            reason="too_few_valid_depth_pixels",
            valid_depth_pixels=valid_pixels,
            valid_depth_ratio=valid_ratio,
        )
    if valid_ratio < float(config.min_valid_ratio):
        return DepthQualityResult(
            accepted=False,
            reason="valid_depth_ratio_too_low",
            valid_depth_pixels=valid_pixels,
            valid_depth_ratio=valid_ratio,
        )

    valid_m = valid_mm / 1000.0
    z_median = float(np.median(valid_m))
    z_mad = float(np.median(np.abs(valid_m - z_median)))
    z_span = float(np.percentile(valid_m, 95.0) - np.percentile(valid_m, 5.0))
    center_ratio = _center_valid_ratio(det, depth_mm, int(config.center_window_px))
    result_kwargs = {
        "valid_depth_pixels": valid_pixels,
        "valid_depth_ratio": valid_ratio,
        "z_median_m": z_median,
        "z_mad_m": z_mad,
        "z_span_m": z_span,
        "center_valid_ratio": center_ratio,
    }
    if z_median < float(config.min_depth_m) or z_median > float(config.max_depth_m):
        return DepthQualityResult(accepted=False, reason="depth_out_of_range", **result_kwargs)
    if z_mad > float(config.max_depth_mad_m):
        return DepthQualityResult(accepted=False, reason="depth_mad_too_high", **result_kwargs)
    if z_span > float(config.max_depth_span_m):
        return DepthQualityResult(accepted=False, reason="depth_span_too_high", **result_kwargs)
    if center_ratio < float(config.min_center_valid_ratio):
        return DepthQualityResult(accepted=False, reason="center_depth_ratio_too_low", **result_kwargs)
    return DepthQualityResult(accepted=True, reason="ok", **result_kwargs)


def _detection_mask(det, image_shape: tuple[int, int]) -> np.ndarray:
    height, width = int(image_shape[0]), int(image_shape[1])
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = _detection_polygon(det)
    if polygon is not None:
        cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 1)
        return mask

    x_min = max(0, min(width, int(round(float(getattr(det, "x_min", 0))))))
    x_max = max(0, min(width, int(round(float(getattr(det, "x_max", 0))))))
    y_min = max(0, min(height, int(round(float(getattr(det, "y_min", 0))))))
    y_max = max(0, min(height, int(round(float(getattr(det, "y_max", 0))))))
    if x_max > x_min and y_max > y_min:
        mask[y_min:y_max, x_min:x_max] = 1
    return mask


def _detection_polygon(det) -> np.ndarray | None:
    if bool(getattr(det, "has_mask", False)) and len(getattr(det, "mask_polygon_xy", [])) >= 6:
        return np.asarray(getattr(det, "mask_polygon_xy"), dtype=np.float32).reshape(-1, 2)
    if bool(getattr(det, "has_obb", False)) and len(getattr(det, "obb_points_xy", [])) == 8:
        return np.asarray(getattr(det, "obb_points_xy"), dtype=np.float32).reshape(4, 2)
    return None


def _center_valid_ratio(det, depth_mm: np.ndarray, window_px: int) -> float:
    height, width = depth_mm.shape[:2]
    window_px = max(1, int(window_px))
    half = window_px // 2
    u = int(round(float(getattr(det, "center_u", 0))))
    v = int(round(float(getattr(det, "center_v", 0))))
    x_min = max(0, u - half)
    x_max = min(width, u + half + 1)
    y_min = max(0, v - half)
    y_max = min(height, v + half + 1)
    if x_max <= x_min or y_max <= y_min:
        return 0.0
    roi = np.asarray(depth_mm[y_min:y_max, x_min:x_max])
    return float(np.count_nonzero(roi > 0)) / float(roi.size)

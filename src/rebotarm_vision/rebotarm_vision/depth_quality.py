"""单目标深度质量门：判定某个检测区域的深度是否可信到可以做三维反投影。

职责与位置
----------
视觉抓取链路的"深度可用性"前置检查。深度缺失、大面积空洞或深度面离散度太大时，
反投影出的抓取点会明显偏离真实目标，宁可拒绝该检测也不要生成错误候选，所以这里
采用 fail-closed 策略：任何一项检查不过都返回 ``accepted=False`` 并给出原因字符串，
由调用方决定丢弃、重试还是降级。

检查内容（按顺序短路返回）
--------------------------
1. 有效深度像素数与其占检测区域面积的比例；
2. 检测区域深度的中位数是否落在工作距离区间内；
3. 深度离散度：中位绝对偏差与 5%~95% 分位跨度；
4. 检测框中心小窗口的有效深度比例（抓取点主要取在中心附近）。

坐标与单位
----------
输入深度图单位为毫米，无效像素约定为 0（也按 <=0 处理）；所有输出与阈值统一换算成
米。检测区域的像素坐标与深度图同分辨率、同帧。掩膜语义与检测消息一致：有效掩膜
至少 3 个点，旋转框角点必须正好 4 个。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthQualityConfig:
    """深度质量门阈值。

    所有长度单位 m，比例无量纲 [0, 1]，像素数为整数。参数入口是上层节点的
    ``depth_quality.*`` 参数，可在线调整：

    - ``enabled``：总开关；关闭时直接放行（返回原因 "disabled"）。
    - ``min_valid_pixels``：检测区域内有效深度像素数下限；太小说明该区域基本没有
      有效测量，统计量不可用。
    - ``min_valid_ratio``：有效像素占检测区域面积的比例下限；用于识别"目标被遮挡或
      反光导致大片空洞"的情况。
    - ``min_depth_m`` / ``max_depth_m``：中位深度允许区间；太近超出相机近端可靠
      范围，太远精度与点云密度都不足。
    - ``max_depth_mad_m``：中位绝对偏差上限（对离群点鲁棒的离散度）；调大更宽松，
      但会放过斜面和混入背景的深度。
    - ``max_depth_span_m``：5%~95% 分位跨度上限，捕获"区域里混进了明显更远/更近的
      物体"这类整体分布过宽的问题。
    - ``center_window_px``：中心窗口边长（像素），至少 1；过小易受单像素噪声影响，
      过大则失去"抓取点附近"的约束意义。
    - ``min_center_valid_ratio``：中心窗口内有效深度比例下限，保证夹爪真正落点的
      位置有直接测量，而不是靠推测。
    """

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
    """深度质量门的判定结果。

    ``accepted`` 为是否放行；``reason`` 是固定的英文枚举字符串（"ok"、"disabled"、
    "missing_depth"、"empty_roi"、"too_few_valid_depth_pixels"、
    "valid_depth_ratio_too_low"、"depth_out_of_range"、"depth_mad_too_high"、
    "depth_span_too_high"、"center_depth_ratio_too_low"），会进入日志与诊断，不可
    翻译。其余字段是当次实测值，即使被拒绝也尽量带上，便于判断卡在哪一项：
    ``valid_depth_pixels`` 有效像素数；``valid_depth_ratio`` 有效像素占比；
    ``z_median_m`` 深度中位数；``z_mad_m`` 中位绝对偏差；``z_span_m`` 5%~95%
    分位跨度；``center_valid_ratio`` 中心窗口有效比例。
    """

    accepted: bool
    reason: str
    valid_depth_pixels: int = 0
    valid_depth_ratio: float = 0.0
    z_median_m: float = 0.0
    z_mad_m: float = 0.0
    z_span_m: float = 0.0
    center_valid_ratio: float = 0.0


def evaluate_detection_depth_quality(det, depth_mm: np.ndarray, config: DepthQualityConfig) -> DepthQualityResult:
    """判定单个检测的深度质量。

    参数：
    - ``det``：单个检测项（本包检测消息形式），读取 ``has_mask``/``mask_polygon_xy``、
      ``has_obb``/``obb_points_xy`` 以及 ``x_min``/``x_max``/``y_min``/``y_max``、
      ``center_u``/``center_v``。
    - ``depth_mm``：与检测同帧同分辨率的深度图，单位 mm，0 表示无效测量。
    - ``config``：阈值配置。

    返回 :class:`DepthQualityResult`。函数无副作用、不修改输入数组。深度统计量都是
    鲁棒统计（中位数/分位/中位绝对偏差），个别飞点不会直接把整帧判死，但混入背景
    导致整体分布变宽时会被离散度检查拦下。
    """
    if not config.enabled:
        return DepthQualityResult(accepted=True, reason="disabled")
    if depth_mm is None or depth_mm.size == 0:
        return DepthQualityResult(accepted=False, reason="missing_depth")

    mask = _detection_mask(det, depth_mm.shape[:2])
    pixel_count = int(np.count_nonzero(mask))
    if pixel_count <= 0:
        return DepthQualityResult(accepted=False, reason="empty_roi")

    values_mm = np.asarray(depth_mm[mask > 0], dtype=np.float32)
    # 0 及负值都视为"无测量"：深度相机在空洞/超量程处会填 0，必须在换算成米之前剔除。
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
    # MAD = 各点到中位数距离的中位数，对离群点不敏感，衡量"目标表面是否平整"。
    z_mad = float(np.median(np.abs(valid_m - z_median)))
    # 5%~95% 跨度忽略两端各 5% 的极值，衡量区域深度分布整体宽度。
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
    """构造检测区域的二值掩膜（1 表示参与统计）。

    优先使用掩膜多边形（最贴合目标形状）；没有掩膜但有旋转框时用四角点；两者都没有
    才退回轴对齐检测框。检测框角点一律先钳到图像范围内再切片（``x_max``/``y_max``
    是开区间边界），因此退化或越界的框会得到空掩膜而不是越界异常。返回值形状与深度
    图前两维一致，dtype 为 uint8。
    """
    height, width = int(image_shape[0]), int(image_shape[1])
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = _detection_polygon(det)
    if polygon is not None:
        # fillPoly 需要整数顶点，用四舍五入而非截断，减少半像素级面积偏差。
        cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 1)
        return mask

    x_min = max(0, min(width, int(round(float(getattr(det, "x_min", 0))))))
    x_max = max(0, min(width, int(round(float(getattr(det, "x_max", 0))))))
    y_min = max(0, min(height, int(round(float(getattr(det, "y_min", 0))))))
    y_max = max(0, min(height, int(round(float(getattr(det, "y_max", 0))))))
    # 仅在框确实有面积时填充；退化的空框保持全 0，由调用方按 empty_roi 拒绝。
    if x_max > x_min and y_max > y_min:
        mask[y_min:y_max, x_min:x_max] = 1
    return mask


def _detection_polygon(det) -> np.ndarray | None:
    """取出检测的轮廓多边形（Nx2 浮点像素坐标），没有可用轮廓时返回 None。

    掩膜需至少 3 个点（6 个浮点数）；旋转框角点必须正好 8 个浮点数（4 个点）。字段
    缺失或长度不符时返回 None，让调用方退回检测框。
    """
    if bool(getattr(det, "has_mask", False)) and len(getattr(det, "mask_polygon_xy", [])) >= 6:
        return np.asarray(getattr(det, "mask_polygon_xy"), dtype=np.float32).reshape(-1, 2)
    if bool(getattr(det, "has_obb", False)) and len(getattr(det, "obb_points_xy", [])) == 8:
        return np.asarray(getattr(det, "obb_points_xy"), dtype=np.float32).reshape(4, 2)
    return None


def _center_valid_ratio(det, depth_mm: np.ndarray, window_px: int) -> float:
    """计算检测框中心窗口内的有效深度像素比例（0~1）。

    窗口边长取 ``window_px``（至少 1），以 ``center_u``/``center_v`` 为中心，
    半宽为 ``window_px // 2``，因此偶数边长时窗口会偏中心右下各半像素，这里按像素
    切片取整处理。窗口被图像边界裁剪；完全落在图像外（或裁剪后为空）时返回 0.0，
    由上层按中心比例过低拒绝。
    """
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

"""抓取候选的置信度-开口打分（用于候选排序与剔除）。

分数越高越优先；返回负分表示该候选被剔除，调用方按 ``score < 0.0`` 过滤。
打分刻意保持简单可解释：置信度减去开口宽度带来的小罚分，鼓励在同等置信度下
优先选择开口更小（更贴合物体、夹持更稳）的候选。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspCandidateScoringConfig:
    """打分参数；默认值与夹爪开口范围、检测置信度分布相匹配。"""

    # 允许的最大开口，单位米。它同时充当宽度罚分的归一化基准：默认 0.20 明显
    # 大于夹爪的真实最大开口（约 0.085），因此宽度项带来的罚分较小。
    max_allowed_width_m: float = 0.20
    # 置信度下限 [0, 1]；低于它直接判为无效（返回 -1.0）。
    min_confidence: float = 0.05
    # 开口罚分权重（无量纲）：分数 = 置信度 - 权重 × (开口 / 最大开口)。
    # 调大更偏好窄开口的候选。
    width_penalty_weight: float = 0.15


def score_grasp_candidate(
    *,
    confidence: float,
    jaw_width_m: float,
    valid: bool,
    config: GraspCandidateScoringConfig | None = None,
) -> float:
    """给单个抓取候选打分；返回值 < 0 表示剔除。

    参数：
        confidence: 候选置信度，期望 [0, 1]。
        jaw_width_m: 候选所需的夹爪开口，单位米；负值/None 按 0 处理。
        valid: 上游给出的候选有效性标志；False 直接剔除。
        config: 打分参数；None 时使用默认配置。

    返回：
        分数（越高越好）。以下情况返回 -1.0：候选无效、置信度低于下限、
        开口 <= 0（退化或缺少信息）或超过最大允许开口。
    """

    cfg = config or GraspCandidateScoringConfig()
    if not valid:
        return -1.0
    confidence_value = float(confidence)
    width = max(float(jaw_width_m or 0.0), 0.0)
    # 置信度不足说明检测本身不可靠，直接剔除而不是给低分（低分仍可能被选中）。
    if confidence_value < float(cfg.min_confidence):
        return -1.0
    if width <= 0.0 or width > float(cfg.max_allowed_width_m):
        return -1.0
    # 1e-9 用于防止 max_allowed_width_m 被配成 0 时除零。
    width_ratio = width / max(float(cfg.max_allowed_width_m), 1e-9)
    return confidence_value - float(cfg.width_penalty_weight) * width_ratio

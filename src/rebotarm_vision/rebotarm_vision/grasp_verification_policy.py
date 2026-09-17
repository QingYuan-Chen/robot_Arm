"""抬升后判定「是否真的抓住」的证据融合策略。

该夹爪没有力传感器，「接触」是由闭合行程加电机速度堵转推断出来的，因此单靠
一项证据容易误判。本模块把夹爪接触标志、闭合行程和（可选的）视觉抬升位移组合
成一次成功/失败判定，并给出英文原因用于日志与服务响应。

判定是纯函数：不读参数、不查坐标变换、不访问硬件；所有阈值由调用方通过配置
传入。默认只启用夹爪侧证据，视觉校验需要显式打开。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspVerificationConfig:
    """抓取校验参数。"""

    # 总开关。默认 True：校验默认开启；置 False 时直接判成功（跳过所有检查）。
    enabled: bool = True
    # 最小闭合行程，单位米。闭合行程 = 指令张开位置 - 实际停止位置；小于该值
    # 说明夹爪几乎没有真正合拢（行程不足 = 没夹到东西），判失败。
    min_closure_distance_m: float = 0.006
    # 是否要求夹爪反馈接触标志（由堵转推断）。默认 True，双重证据更可靠。
    require_gripper_contact: bool = True
    # 是否启用视觉抬升位移校验。默认 False：缺少视觉证据时不至于把成功判成失败。
    visual_lift_check_enabled: bool = False
    # 视觉抬升位移下限，单位米。物体被带起的竖直位移小于它就认为没夹牢；
    # 调大更严格，但会受深度噪声影响。
    min_visual_lift_delta_m: float = 0.030


@dataclass(frozen=True)
class GraspVerificationInput:
    """一次校验所需的证据快照。"""

    # 夹爪是否报出接触（由堵转推断，不是实测接触力）。
    gripper_contact_detected: bool
    # 本次闭合的实际行程，单位米（非负）。
    closure_distance_m: float
    # 视觉测得的物体抬升位移，单位米；无证据时保持 0.0。
    visual_lift_delta_m: float = 0.0
    # 视觉抬升证据是否可用（例如深度/检测是否拿到有效数据）。
    visual_lift_evidence_available: bool = False


@dataclass(frozen=True)
class GraspVerificationResult:
    """校验结果；``reason`` 为英文诊断文本。"""

    success: bool
    reason: str


def verify_grasp_after_lift(
    evidence: GraspVerificationInput,
    config: GraspVerificationConfig,
) -> GraspVerificationResult:
    """按固定优先级短路判定抓取是否成功。

    判定顺序（任一失败立即返回，避免后续检查掩盖真实原因）：
        1. 校验未启用 → 直接成功；
        2. 要求接触但未检测到接触 → 失败；
        3. 闭合行程小于下限 → 失败（夹空/未夹到）；
        4. 若启用视觉校验：先要求证据可用，再要求抬升位移达标。

    返回：
        GraspVerificationResult；``reason`` 会写入执行结果与日志，勿改文案。
    """

    if not config.enabled:
        return GraspVerificationResult(True, "verification disabled")
    if config.require_gripper_contact and not evidence.gripper_contact_detected:
        return GraspVerificationResult(False, "gripper contact not detected")
    if float(evidence.closure_distance_m) < float(config.min_closure_distance_m):
        return GraspVerificationResult(False, "gripper closure distance too small")
    if config.visual_lift_check_enabled:
        # 证据缺失按失败处理：宁可让上层重试，也不能把"没看到"当成"抬起来了"。
        if not evidence.visual_lift_evidence_available:
            return GraspVerificationResult(False, "visual lift evidence unavailable")
        if float(evidence.visual_lift_delta_m) < float(config.min_visual_lift_delta_m):
            return GraspVerificationResult(False, "visual lift delta too small")
    return GraspVerificationResult(True, "grasp verified")

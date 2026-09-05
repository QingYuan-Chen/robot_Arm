from __future__ import annotations

import math
import statistics
from dataclasses import replace
from typing import Sequence

from .models import (
    Baseline,
    DirectionEvidence,
    FollowerSample,
    LeaderSample,
    MappingConfig,
    MappingResult,
)


def _require_finite(values: Sequence[float], label: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{label}包含非有限数值")


def _validate_age(timestamp_s: float, now_s: float, max_age_s: float, label: str) -> None:
    age = float(now_s) - float(timestamp_s)
    if not math.isfinite(age):
        raise ValueError(f"{label}时间戳包含非有限数值")
    if age < 0.0:
        raise ValueError(f"{label}时间戳晚于当前时间")
    if age > max_age_s:
        raise ValueError(f"{label}样本已过期：{age:.3f}s")


def validate_paired_sample(
    leader: LeaderSample,
    follower: FollowerSample,
    config: MappingConfig,
    *,
    now_s: float,
) -> None:
    """验证一组只读反馈的完整性、新鲜度、状态和软限位。"""

    expected_names = tuple(joint.follower_name for joint in config.arm_joints) + (
        config.gripper.follower_name,
    )
    if len(leader.angles_deg) != len(config.leader_ids):
        raise ValueError("引导臂样本必须包含 ID 0..6 的七个角度")
    if tuple(motor.name for motor in follower.motors) != expected_names:
        raise ValueError("从臂反馈顺序必须严格为 joint1..joint6、gripper")

    _validate_age(
        leader.timestamp_s,
        now_s,
        config.thresholds.max_sample_age_s,
        "引导臂",
    )
    _validate_age(
        follower.timestamp_s,
        now_s,
        config.thresholds.max_sample_age_s,
        "从臂",
    )
    _require_finite(leader.angles_deg, "引导臂角度")

    specs = config.arm_joints + (config.gripper,)
    for motor, spec in zip(follower.motors, specs, strict=True):
        _require_finite(
            (motor.position_rad, motor.velocity_rad_s, motor.torque_nm),
            f"{motor.name} 反馈",
        )
        if motor.status_code != 0:
            raise ValueError(f"{motor.name} status_code={motor.status_code}，要求为 0")
        if spec.lower_rad is not None and motor.position_rad < spec.lower_rad:
            raise ValueError(
                f"{motor.name} 位置 {motor.position_rad:.6f} 超出软限位"
            )
        if spec.upper_rad is not None and motor.position_rad > spec.upper_rad:
            raise ValueError(
                f"{motor.name} 位置 {motor.position_rad:.6f} 超出软限位"
            )


def capture_baseline(
    paired_samples: Sequence[tuple[LeaderSample, FollowerSample]],
    config: MappingConfig,
    *,
    now_s: float,
) -> Baseline:
    """从稳定反馈窗口逐轴取中位数，建立相对映射基线。"""

    if len(paired_samples) < config.thresholds.sign_window_size:
        raise ValueError(
            "稳定基线样本不足："
            f"需要 {config.thresholds.sign_window_size}，实际 {len(paired_samples)}"
        )
    for leader, follower in paired_samples:
        validate_paired_sample(leader, follower, config, now_s=now_s)
        for motor in follower.motors:
            if abs(motor.velocity_rad_s) > config.thresholds.baseline_max_velocity_rad_s:
                raise ValueError(
                    f"{motor.name} 基线速度 {motor.velocity_rad_s:.6f} rad/s 超过阈值"
                )

    leader_medians = tuple(
        float(statistics.median(sample.angles_deg[index] for sample, _ in paired_samples))
        for index in range(len(config.leader_ids))
    )
    follower_medians = tuple(
        float(
            statistics.median(
                sample.motors[index].position_rad for _, sample in paired_samples
            )
        )
        for index in range(len(config.arm_joints) + 1)
    )
    return Baseline(
        captured_at_s=float(now_s),
        leader_angles_deg=leader_medians,
        follower_positions_rad=follower_medians,
    )


def map_virtual_follower(
    leader: LeaderSample,
    baseline: Baseline,
    config: MappingConfig,
) -> MappingResult:
    """按从臂坐标和候选符号计算六关节虚拟目标，不发送控制指令。"""

    if len(leader.angles_deg) != 7 or len(baseline.leader_angles_deg) != 7:
        raise ValueError("引导臂与基线必须各包含七个角度")
    if len(baseline.follower_positions_rad) != 7:
        raise ValueError("从臂基线必须包含六个关节和夹爪")
    _require_finite(leader.angles_deg, "引导臂角度")
    _require_finite(baseline.leader_angles_deg, "引导臂基线")
    _require_finite(baseline.follower_positions_rad, "从臂基线")

    deltas: list[float] = []
    positions: list[float] = []
    for index, spec in enumerate(config.arm_joints):
        delta = math.radians(
            leader.angles_deg[spec.leader_id]
            - baseline.leader_angles_deg[spec.leader_id]
        )
        position = (
            baseline.follower_positions_rad[index]
            + int(spec.sign) * float(spec.scale) * delta
        )
        if position < float(spec.lower_rad) or position > float(spec.upper_rad):
            raise ValueError(
                f"{spec.follower_name} 虚拟目标 {position:.6f} 超出软限位"
            )
        deltas.append(delta)
        positions.append(position)
    return MappingResult(tuple(positions), tuple(deltas))


def infer_direction(
    baseline: Baseline,
    paired_window: Sequence[tuple[LeaderSample, FollowerSample]],
    selected_joint: str,
    config: MappingConfig,
    *,
    now_s: float,
) -> DirectionEvidence:
    """根据人工移动后的稳定窗口推断一个机械臂关节的方向符号。"""

    names = tuple(joint.follower_name for joint in config.arm_joints)
    if selected_joint not in names:
        raise ValueError("方向标定只接受 joint1..joint6，不接受夹爪")
    window_size = config.thresholds.sign_window_size
    if len(paired_window) < window_size:
        raise ValueError(f"方向窗口样本不足：需要 {window_size}")
    samples = paired_window[-window_size:]
    selected_index = names.index(selected_joint)

    observations: list[tuple[float, float, int]] = []
    for leader, follower in samples:
        validate_paired_sample(leader, follower, config, now_s=now_s)
        leader_deltas = tuple(
            math.radians(current - initial)
            for current, initial in zip(
                leader.angles_deg[:6], baseline.leader_angles_deg[:6], strict=True
            )
        )
        follower_deltas = tuple(
            motor.position_rad - initial
            for motor, initial in zip(
                follower.motors[:6],
                baseline.follower_positions_rad[:6],
                strict=True,
            )
        )
        for index, name in enumerate(names):
            if index == selected_index:
                continue
            if (
                abs(leader_deltas[index])
                > config.thresholds.other_joint_max_delta_rad
                or abs(follower_deltas[index])
                > config.thresholds.other_joint_max_delta_rad
            ):
                raise ValueError(f"{name} 作为非选定关节发生了明显运动")

        leader_delta = leader_deltas[selected_index]
        follower_delta = follower_deltas[selected_index]
        if (
            abs(leader_delta) < config.thresholds.min_direction_delta_rad
            or abs(follower_delta) < config.thresholds.min_direction_delta_rad
        ):
            raise ValueError(f"{selected_joint} 运动幅度过小，无法推断方向")
        inferred_sign = 1 if follower_delta / leader_delta > 0.0 else -1
        observations.append((leader_delta, follower_delta, inferred_sign))

    signs = {observation[2] for observation in observations}
    if len(signs) != 1:
        raise ValueError(f"{selected_joint} 窗口内推断符号不一致")
    spec = config.arm_joints[selected_index]
    return DirectionEvidence(
        follower_name=selected_joint,
        observed_at_s=float(now_s),
        leader_delta_rad=float(statistics.median(item[0] for item in observations)),
        follower_delta_rad=float(statistics.median(item[1] for item in observations)),
        inferred_sign=signs.pop(),
        candidate_sign=int(spec.sign),
        consistent=True,
        confirmed=False,
        verified=False,
    )


def apply_confirmation(
    evidence: DirectionEvidence,
    confirmed: bool,
) -> DirectionEvidence:
    """仅在人工确认且推断符号匹配候选配置时标记通过。"""

    if not isinstance(confirmed, bool):
        raise TypeError("confirmed 必须是布尔值")
    verified = bool(
        confirmed
        and evidence.consistent
        and evidence.inferred_sign == evidence.candidate_sign
    )
    return replace(evidence, confirmed=confirmed, verified=verified)

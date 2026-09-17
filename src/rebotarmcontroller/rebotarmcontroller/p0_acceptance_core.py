"""P0 验收（Gate B/C）的纯判据核心。

本模块汇总"显式使能 —— 保持 —— 失能"验收流程所需的判据计算，被示例入口
``p0_gate_bc_acceptance`` 与单元测试共同复用。这里刻意不依赖 ROS 运行时对象，
只接收消息对象（鸭子类型）并返回普通容器，因此可以脱离硬件离线单测。

主要职责：

- 预检校验：只有"已连接但未使能、控制环未激活、状态机 IDLE、电机状态码全 0、
  反馈有限且恰好 6 轴"的现场才允许进入使能验收；
- 运动指标采样：以某帧关节位置为基线，统计位置跳变峰值与绝对速度峰值；
- 原始反馈速度门限：把 12 位速度反馈的半个量化步长折算进门限，避免在物理上限
  附近把正常码值误判为超限。

本模块只做判据计算，不下发任何指令，也不接触串口/总线。
"""

from __future__ import annotations

import math
from typing import Any


# 期望的关节名与顺序：反馈数组按此顺序对齐，顺序或成员不符即预检失败。
EXPECTED_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
# 达妙电机速度反馈的量化位宽：解码速度落在 [-VMAX, +VMAX] 上，共 2**12 个码值。
DAMIAO_VELOCITY_FEEDBACK_BITS = 12
# 各关节速度反馈的满量程 VMAX（单位 rad/s）：joint1~joint3 为 10，joint4~joint6 为 30。
# 该值只用于把"半个量化步长"换算成 rad/s，与运动学上的物理速度上限无关。
DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT = {
    "joint1": 10.0,
    "joint2": 10.0,
    "joint3": 10.0,
    "joint4": 30.0,
    "joint5": 30.0,
    "joint6": 30.0,
}


def quantization_aware_raw_velocity_limits(
    physical_limit_rad_s: float,
    *,
    feedback_bits: int = DAMIAO_VELOCITY_FEEDBACK_BITS,
    vmax_by_joint: dict[str, float] | None = None,
) -> dict[str, float]:
    """在物理速度上限之上只补半个反馈 LSB，得到原始反馈速度门限。

    达妙速度反馈的取值范围是 ``[-VMAX, +VMAX]``（见
    ``DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT``）：满量程 2*VMAX 被
    ``2**feedback_bits - 1`` 个步长划分，解码样本在半个步长内存在不确定性。
    若直接拿物理上限当原始门限，恰好包含该上限的那个量化桶会被误判为超限，
    因此每个关节补上 ``VMAX / (2**feedback_bits - 1)``（即半个量化步长）。

    参数：
        physical_limit_rad_s: 物理速度上限，单位 rad/s，必须有限且为正。
        feedback_bits: 速度反馈量化位宽，默认 12 位；小于 2 位无物理意义。
        vmax_by_joint: 可选的"关节名 -> VMAX"覆盖表，缺省用达妙实测表。

    返回：
        "关节名 -> 原始反馈门限（rad/s）"字典，键顺序与 ``EXPECTED_JOINTS`` 一致。

    异常：
        ValueError: 物理上限非有限或非正、位宽小于 2、VMAX 表缺少关节、
            或某个 VMAX 非有限/非正。
    """

    physical_limit = float(physical_limit_rad_s)
    if not math.isfinite(physical_limit) or physical_limit <= 0.0:
        raise ValueError("physical velocity limit must be finite and positive")
    if int(feedback_bits) < 2:
        raise ValueError("feedback_bits must be at least 2")
    vmax_values = (
        DAMIAO_VELOCITY_VMAX_RAD_S_BY_JOINT
        if vmax_by_joint is None
        else vmax_by_joint
    )
    missing = [name for name in EXPECTED_JOINTS if name not in vmax_values]
    if missing:
        raise ValueError(f"missing velocity VMAX for joints: {missing}")
    # 12 位时 denominator=4095，即满量程 2*VMAX 被划分成的步长数。
    denominator = float((1 << int(feedback_bits)) - 1)
    limits: dict[str, float] = {}
    for name in EXPECTED_JOINTS:
        vmax = float(vmax_values[name])
        if not math.isfinite(vmax) or vmax <= 0.0:
            raise ValueError(f"{name} velocity VMAX must be finite and positive")
        # 半个量化步长（rad/s）：denominator 个步长覆盖 2*VMAX，故半步长为 VMAX/denominator。
        half_lsb = vmax / denominator
        limits[name] = physical_limit + half_lsb
    return limits


def raw_velocity_limit_violations(
    velocities_by_name: dict[str, float],
    physical_limit_rad_s: float,
    *,
    raw_limit_floor_rad_s: float | None = None,
) -> dict[str, dict[str, float]]:
    """挑出原始反馈速度超过量化感知门限的关节。

    该判定是验收期的一次性检查，不会改变硬件里生效的物理速度窗口：
    ``raw_limit_floor_rad_s`` 只是操作者给原始门限设的下限（例如担心门限过紧
    导致误停），抬高门限后仍按同一个物理上限解释结果。

    参数：
        velocities_by_name: "关节名 -> 原始反馈速度（rad/s）"，比较时取绝对值。
        physical_limit_rad_s: 物理速度上限（rad/s），透传给
            ``quantization_aware_raw_velocity_limits``。
        raw_limit_floor_rad_s: 可选的原始门限下限（rad/s），必须有限且为正；
            实际门限取 ``max(量化感知门限, 该下限)``。

    返回：
        "超限关节 -> {'raw_velocity_rad_s': 实测值, 'raw_limit_rad_s': 生效门限}"；
        全部合格时返回空字典。

    异常：
        ValueError: 缺少任一 ``EXPECTED_JOINTS`` 关节、速度非有限、或下限非法。
    """
    limits = quantization_aware_raw_velocity_limits(physical_limit_rad_s)
    if raw_limit_floor_rad_s is not None:
        raw_limit_floor = float(raw_limit_floor_rad_s)
        if not math.isfinite(raw_limit_floor) or raw_limit_floor <= 0.0:
            raise ValueError("raw velocity limit floor must be finite and positive")
        limits = {
            name: max(limit, raw_limit_floor)
            for name, limit in limits.items()
        }
    missing = [name for name in EXPECTED_JOINTS if name not in velocities_by_name]
    if missing:
        raise ValueError(f"missing joint velocities: {missing}")
    violations: dict[str, dict[str, float]] = {}
    for name in EXPECTED_JOINTS:
        velocity = abs(float(velocities_by_name[name]))
        if not math.isfinite(velocity):
            raise ValueError(f"{name} velocity must be finite")
        if velocity > limits[name]:
            violations[name] = {
                "raw_velocity_rad_s": velocity,
                "raw_limit_rad_s": limits[name],
            }
    return violations


def joint_state_snapshot(msg: Any) -> dict[str, Any]:
    """把一帧关节状态消息摊平成可 JSON 序列化的字典（用于验收报告落盘）。

    列表保持消息中的原始顺序，不重排、不补默认值。
    """
    return {
        "names": list(msg.name),
        "positions": [float(value) for value in msg.position],
        "velocities": [float(value) for value in msg.velocity],
        "effort": [float(value) for value in msg.effort],
    }


def arm_status_snapshot(msg: Any) -> dict[str, Any]:
    """把一帧机械臂状态消息摊平成可 JSON 序列化的字典（用于验收报告落盘）。

    含模式、使能位、控制环占用、状态机、关节名、每关节状态码与错误码列表；
    这些字段是判断"使能后是否真的进入保持"以及"失能后是否彻底停住"的依据。
    """
    return {
        "mode": str(msg.mode),
        "enabled": bool(msg.enabled),
        "control_loop_active": bool(msg.control_loop_active),
        "state_machine": str(msg.state_machine),
        "joint_names": list(msg.joint_names),
        "per_joint_status_code": [int(value) for value in msg.per_joint_status_code],
        "error_codes": list(msg.error_codes),
    }


def validate_preflight(status: Any, joint_state: Any) -> list[str]:
    """使能验收前的静态预检，返回错误描述列表（空列表表示可以继续）。

    逐项检查：
        - 关节状态的名字与顺序必须恰好等于 ``EXPECTED_JOINTS``（joint1..joint6）；
        - 机械臂状态里的关节名也必须完全一致；
        - 位置、速度向量长度必须为 6；
        - 位置与速度必须全为有限值（NaN/Inf 说明反馈不可信）；
        - 硬件必须尚未使能、控制环未激活（否则现场可能已有运动指令在生效）；
        - 状态机必须处于 "IDLE"（其它状态说明有未结束的运动流程）；
        - 每个电机的状态码必须全为 0（失能状态下驱动应回报 0）；
        - 控制器不得上报任何错误码。

    返回全部问题而不是首个问题，方便一次性看清现场状态；本函数只做判定。
    """
    errors: list[str] = []
    names = tuple(joint_state.name)
    if names != EXPECTED_JOINTS:
        errors.append(f"joint state names {names!r}, expected {EXPECTED_JOINTS!r}")
    if tuple(status.joint_names) != EXPECTED_JOINTS:
        errors.append("arm status joint names do not match joint1..joint6")
    if len(joint_state.position) != len(EXPECTED_JOINTS):
        errors.append("joint position vector must contain six values")
    if len(joint_state.velocity) != len(EXPECTED_JOINTS):
        errors.append("joint velocity vector must contain six values")
    numeric_values = list(joint_state.position) + list(joint_state.velocity)
    if not all(math.isfinite(float(value)) for value in numeric_values):
        errors.append("joint state contains non-finite position or velocity")
    if status.enabled:
        errors.append("hardware is already enabled")
    if status.control_loop_active:
        errors.append("control loop is already active")
    if status.state_machine != "IDLE":
        errors.append(f"state_machine={status.state_machine!r}, expected 'IDLE'")
    codes = [int(value) for value in status.per_joint_status_code]
    if codes != [0] * len(EXPECTED_JOINTS):
        errors.append(f"motor status codes are {codes!r}, expected all zero")
    if status.error_codes:
        errors.append(f"controller reports errors: {list(status.error_codes)!r}")
    return errors


def sample_motion_metrics(
    baseline_by_name: dict[str, float],
    joint_state: Any,
) -> tuple[float, float]:
    """返回 (最大位置跳变 rad, 最大绝对速度 rad/s) 两个峰值标量。

    语义与 ``motion_sample_details`` 完全一致，只是把两个最常用的峰值单独取出，
    方便调用方直接与阈值比较。
    """
    details = motion_sample_details(baseline_by_name, joint_state)
    return (
        float(details["max_position_jump_rad"]),
        float(details["max_abs_velocity_rad_s"]),
    )


def motion_sample_details(
    baseline_by_name: dict[str, float],
    joint_state: Any,
) -> dict[str, Any]:
    """按关节名对齐基线，算出这一帧反馈相对基线的运动细节。

    关节名取自消息本身，因此消息里的关节顺序与基线字典顺序不同也能正确对齐
    （``zip`` 只负责同一帧内"名字 -> 数值"的配对，不假设顺序）。

    参数：
        baseline_by_name: "关节名 -> 位置（rad）"，通常是使能前的稳定姿态。
        joint_state: 含 name/position/velocity 的关节状态消息。

    返回字典字段：
        - ``max_position_jump_rad`` / ``position_jump_joint``：最大绝对位置跳变
          （rad）及其关节名；
        - ``position_at_peak_rad`` / ``position_baseline_rad``：该关节的实测值
          与基线值（rad），便于报告还原现场；
        - ``max_abs_velocity_rad_s`` / ``velocity_joint`` /
          ``signed_velocity_at_peak_rad_s``：最大速度幅值、对应关节名，以及该关节
          的带符号速度（rad/s，正负号保留，用于判断方向）；
        - ``abs_velocity_by_joint_rad_s``：所有关节的绝对速度（rad/s）。

    异常：
        ValueError: 基线中的关节在消息里缺位置或缺速度，或峰值指标非有限值。
    """
    positions = {
        name: float(position)
        for name, position in zip(joint_state.name, joint_state.position)
    }
    missing = [name for name in baseline_by_name if name not in positions]
    if missing:
        raise ValueError(f"joint state missing baseline joints: {missing}")
    velocities = {
        name: float(velocity)
        for name, velocity in zip(joint_state.name, joint_state.velocity)
    }
    missing_velocity = [name for name in baseline_by_name if name not in velocities]
    if missing_velocity:
        raise ValueError(f"joint state missing joint velocities: {missing_velocity}")
    # 位置跳变逐关节取绝对值：使能与保持期间，任何关节相对基线的位移都应接近 0。
    jump_by_name = {
        name: abs(positions[name] - baseline)
        for name, baseline in baseline_by_name.items()
    }
    jump_joint = max(jump_by_name, key=jump_by_name.get)
    # 速度峰值按绝对值挑选（方向不参与比较），后面再把带符号速度一并返回。
    velocity_joint = max(velocities, key=lambda name: abs(velocities[name]))
    max_jump = jump_by_name[jump_joint]
    max_velocity = abs(velocities[velocity_joint])
    if not math.isfinite(max_jump) or not math.isfinite(max_velocity):
        raise ValueError("motion metrics contain non-finite values")
    return {
        "max_position_jump_rad": max_jump,
        "position_jump_joint": jump_joint,
        "position_at_peak_rad": positions[jump_joint],
        "position_baseline_rad": baseline_by_name[jump_joint],
        "max_abs_velocity_rad_s": max_velocity,
        "velocity_joint": velocity_joint,
        "signed_velocity_at_peak_rad_s": velocities[velocity_joint],
        "abs_velocity_by_joint_rad_s": {
            name: abs(value) for name, value in velocities.items()
        },
    }

"""轨迹运行时限位守卫：按采样检查实测力矩/速度，并用差分校验加速度/加加速度。

用途与位置
    规划期通过的轨迹不代表运行期安全（负载变化、跟踪误差、外部扰动都会让实际速度与力矩
    超出预期）。本模块提供执行阶段的软件限位：设计用途是由执行/回放监控按控制周期把关节名、
    实测速度、实测力矩与运行时间喂给 :class:`TrajectoryRuntimeLimitGuard`，一旦越限就拿到一条
    :class:`RuntimeLimitViolation`，据此停机或降级。

数值方法
    速度与力矩来自反馈，此处不做估计；加速度与加加速度没有直接测量值，用相邻采样的一阶差分
    （后向差分，即当前采样减去上一次采样）估计：``a_k = (v_k - v_{k-1}) / dt``、
    ``j_k = (a_k - a_{k-1}) / dt``。因此结果对采样周期与反馈噪声敏感：``dt`` 由调用方给出，
    必须单调递增且严格大于 0（零或负间隔会放大噪声甚至除零，按调用错误处理）；加加速度从第三个
    采样点起才有值。

状态与安全语义
    守卫持有上一次的时间、速度和加速度历史，是有状态对象：每次回放/执行开始前必须调用
    :meth:`TrajectoryRuntimeLimitGuard.reset`，否则会拿上一轮的残值做差分。
    越限时返回违规描述，``None`` 表示本次采样全部通过；输入不合法（未知关节、时间倒退、
    采样间隔不为正、非有限值）一律抛异常，绝不静默放过——安全路径上"没报错"必须等于"查过了"。

限位来源
    规划侧的速度/加速度/加加速度上限从 MoveIt 的关节限位 YAML 读取
    （:func:`load_joint_runtime_limits`）；力矩上限不在这份 YAML 里，需要调用方另行给出。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class JointRuntimeLimit:
    """执行守卫使用的一组关节运行限位。

    所有数值都是"绝对值上限"（比较时对实测值取绝对值），单位随关节量纲：旋转关节为
    N·m、rad/s、rad/s²、rad/s³；移动关节（例如夹爪指）为 N、m/s、m/s²、m/s³。
    某个字段为 ``None`` 表示关闭该项检查；对于上游契约里没有规划侧加速度/加加速度限位的
    关节，这样可以保留其它检查而不误报。
    """

    max_effort: float | None = None
    max_velocity: float | None = None
    max_acceleration: float | None = None
    max_jerk: float | None = None

    def __post_init__(self) -> None:
        # 校验并归一化：None 表示关闭该项检查；NaN/Inf 或负数属于配置错误，直接拒绝。
        for field_name in ("max_effort", "max_velocity", "max_acceleration", "max_jerk"):
            value = getattr(self, field_name)
            if value is None:
                continue
            value = float(value)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and non-negative")
            # 冻结 dataclass 不能直接赋值，用 object.__setattr__ 写入规范化后的 float。
            object.__setattr__(self, field_name, value)


@dataclass(frozen=True)
class RuntimeLimitViolation:
    """一次越限的具体描述（供日志、状态 payload 与停机决策使用）。

    ``kind``  越限类型，取值为 ``effort`` / ``velocity`` / ``acceleration`` / ``jerk``；
              这些字符串是上层判断依据，不要改动；
    ``joint`` 越限关节名；
    ``value`` 实测幅值（已取绝对值）；
    ``limit`` 触发该违规的限位值（绝对值）。
    """

    kind: str
    joint: str
    value: float
    limit: float


class TrajectoryRuntimeLimitGuard:
    """按采样检查实测力矩/速度，并用差分估计的加速度/加加速度做越限判定。

    每次 :meth:`observe` 只报告第一条违规（同一采样的多项越限不聚合），因为调用方的动作都一样
    ——立即停车或降级；这样也避免在安全路径上做多余计算。
    检查顺序：先遍历所有关节查力矩，再查速度（这两项是直接测量值，最可信），最后才用差分结果
    查加速度与加加速度。
    """

    def __init__(self, limits: Mapping[str, JointRuntimeLimit]) -> None:
        """构造守卫。

        ``limits`` 是 ``{关节名: 关节限位}`` 映射，至少要有一个关节；守卫只认识映射里出现过的
        关节名，后续采样出现未知关节会直接报错，避免有关节漏配限位却无人察觉。
        """
        # 复制一份映射：守卫运行期间不希望外部再改动限位配置。
        self._limits = {str(name): limit for name, limit in limits.items()}
        if not self._limits:
            raise ValueError("at least one joint runtime limit is required")
        if any(not isinstance(limit, JointRuntimeLimit) for limit in self._limits.values()):
            raise TypeError("limits must contain JointRuntimeLimit values")
        self.reset()

    def reset(self) -> None:
        """清空差分历史，开始一段新的执行/回放。

        必须在每次执行开始前调用：残留的上一次速度与加速度会被当成"上一采样"，使第一个采样点
        算出错误的加速度/加加速度。
        """
        self._previous_elapsed: float | None = None
        self._previous_velocities: dict[str, float] = {}
        self._previous_accelerations: dict[str, float] = {}

    def observe(
        self,
        *,
        joint_names: Sequence[str],
        velocities: Sequence[float],
        efforts: Sequence[float],
        elapsed: float,
    ) -> RuntimeLimitViolation | None:
        """喂入一个采样周期的测量值，返回第一条越限记录，全部通过则返回 ``None``。

        ``joint_names`` 与 ``velocities``、``efforts`` 必须等长且顺序一一对应；``elapsed`` 是调用方
        提供的时间基准（秒，通常取节点单调时钟），要求相对上一次调用单调递增，两次调用的差值
        即差分用的 ``dt``。

        异常：长度不一致、出现未配置限位的关节、时间非有限值或倒退、相邻采样间隔不为正、
        速度/力矩非有限值都会抛 ``ValueError``（调用错误不能被误判为"安全"）。

        副作用：内部历史（时间、速度以及算得出的加速度）会被更新；即使本次越限也会先记录当前
        状态再返回，因此调用方选择继续运行时下一次差分仍然连续。
        """
        names = [str(name) for name in joint_names]
        if len(names) != len(velocities) or len(names) != len(efforts):
            raise ValueError("joint_names, velocities, and efforts must have equal lengths")
        unknown = [name for name in names if name not in self._limits]
        if unknown:
            raise ValueError(f"unknown joint in runtime limits: {unknown[0]}")
        current_time = float(elapsed)
        if not math.isfinite(current_time):
            raise ValueError("elapsed must be finite")
        if self._previous_elapsed is not None and current_time < self._previous_elapsed:
            raise ValueError("elapsed time must be monotonic")
        # dt 为 None 表示这是第一个采样点，无法差分；dt <= 0 属于调用错误（除零或时间未推进）。
        dt = None if self._previous_elapsed is None else current_time - self._previous_elapsed
        if dt is not None and dt <= 0.0:
            raise ValueError("elapsed time must advance between runtime samples")

        # 先转成 float 并校验有限性：NaN 参与比较恒为假，会让限位判断失去意义，必须在这里拦下。
        current_velocities = {name: _finite(value, "velocity") for name, value in zip(names, velocities)}
        current_efforts = {name: _finite(value, "effort") for name, value in zip(names, efforts)}

        for name in names:
            limit = self._limits[name]
            violation = _check_absolute("effort", name, current_efforts[name], limit.max_effort)
            if violation is not None:
                self._remember(current_time, current_velocities)
                return violation
            violation = _check_absolute("velocity", name, current_velocities[name], limit.max_velocity)
            if violation is not None:
                self._remember(current_time, current_velocities)
                return violation

        if dt is not None:
            for name in names:
                # 加速度由相邻速度差商估计；dt 已在上面保证为正。
                acceleration = (current_velocities[name] - self._previous_velocities[name]) / dt
                limit = self._limits[name]
                violation = _check_absolute("acceleration", name, acceleration, limit.max_acceleration)
                if violation is not None:
                    self._remember(current_time, current_velocities, {name: acceleration})
                    return violation
                if name in self._previous_accelerations:
                    # 加加速度需要连续两次加速度，因此从第三个采样点起才检查。
                    jerk = (acceleration - self._previous_accelerations[name]) / dt
                    violation = _check_absolute("jerk", name, jerk, limit.max_jerk)
                    if violation is not None:
                        self._remember(current_time, current_velocities, {name: acceleration})
                        return violation
                # 只在成功算出该关节加速度时更新它的历史。
                self._previous_accelerations[name] = acceleration

        self._remember(current_time, current_velocities)
        return None

    def _remember(
        self,
        elapsed: float,
        velocities: Mapping[str, float],
        accelerations: Mapping[str, float] | None = None,
    ) -> None:
        """记录本次采样，供下一次差分使用。

        ``accelerations`` 为空或 ``None`` 时保留上一次的加速度历史：越限返回时只有越限关节带
        新加速度，其余关节的历史不能被清掉（此时调用方通常会停机，未补算的值也不再使用）。
        """
        self._previous_elapsed = elapsed
        self._previous_velocities = dict(velocities)
        if accelerations:
            self._previous_accelerations.update(accelerations)


def load_joint_runtime_limits(
    path: Path,
    joint_names: Sequence[str],
) -> dict[str, JointRuntimeLimit]:
    """从 MoveIt 的关节限位 YAML 读取速度、加速度与加加速度上限。

    ``path``        关节限位文件路径：顶层键 ``joint_limits``，每个关节下用 ``has_<kind>_limits``
                    加 ``max_<kind>`` 描述限位；
    ``joint_names`` 需要加载的关节名序列，缺任何一个都直接抛 ``ValueError``——宁可启动失败，
                    也不要用"没有限位"的关节去跑执行守卫。

    返回 ``{关节名: JointRuntimeLimit}``；其中 ``max_effort`` 恒为 ``None``，因为 MoveIt 的关节限位
    文件不含力矩字段，需要力矩保护时由调用方另行构造限位。
    """
    # 延迟导入 yaml：本模块的纯逻辑（守卫本身）不依赖 YAML 解析，避免导入期强耦合。
    import yaml

    # 空文件时 safe_load 返回 None，这里按空映射处理，让后面按键缺失正常报错。
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    entries = payload.get("joint_limits", {})
    result: dict[str, JointRuntimeLimit] = {}
    for raw_name in joint_names:
        name = str(raw_name)
        if name not in entries:
            # 缺限位的关节不允许静默跳过，否则该关节将完全不受执行守卫保护。
            raise ValueError(f"joint limit missing from MoveIt YAML: {name}")
        # 关节节点为空（YAML 里只写了关节名）时按"无限位"处理，由 _enabled_limit 决定各项开关。
        entry = entries[name] or {}
        result[name] = JointRuntimeLimit(
            max_velocity=_enabled_limit(entry, "velocity"),
            max_acceleration=_enabled_limit(entry, "acceleration"),
            max_jerk=_enabled_limit(entry, "jerk"),
        )
    return result


def _finite(value: float, label: str) -> float:
    """把数值转成 float 并校验有限性；非有限值视为调用错误。"""
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be finite")
    return converted


def _enabled_limit(entry: Mapping[str, object], kind: str) -> float | None:
    """按 MoveIt 约定读取单项限位。

    ``has_<kind>_limits`` 为假或缺省时返回 ``None``（关闭该项检查）；为真时 ``max_<kind>`` 必填，
    且必须是非负有限值。
    """
    # 未声明 has_*_limits 视为该关节不检查这一项（例如夹爪指没有加加速度限位）。
    if not bool(entry.get(f"has_{kind}_limits", False)):
        return None
    key = f"max_{kind}"
    if key not in entry:
        # 声明了有限位却没给数值：配置自相矛盾，直接报错。
        raise ValueError(f"{key} is required when has_{kind}_limits is true")
    value = float(entry[key])
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{key} must be finite and non-negative")
    return value


def _check_absolute(
    kind: str,
    joint: str,
    value: float,
    limit: float | None,
) -> RuntimeLimitViolation | None:
    """绝对值限位比较：``|value| > limit`` 才算越限（等于限位视为通过）。

    ``limit`` 为 ``None`` 表示该项未配置，一律通过；返回的违规里记录幅值而不是带符号的原始值，
    因为方向对停机决策没有意义。
    """
    if limit is None or abs(value) <= limit:
        return None
    return RuntimeLimitViolation(kind=kind, joint=joint, value=abs(value), limit=limit)

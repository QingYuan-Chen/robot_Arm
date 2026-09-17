"""仿真后端的纯逻辑核心：轨迹归一化、插值、夹爪映射与推进步数换算。

本模块刻意不依赖 ROS 消息与图形界面，只有 ``MuJoCoArmAdapter`` 在实例化时才导入
MuJoCo 运行库，因此其余函数都能脱离仿真环境做单元测试。仿真执行器与无头检查
都复用这里的实现，保证"轨迹怎么采样、夹爪宽度怎么映射、一次回调推进多少物理步"
在两条路径上完全一致。

安全相关约定：
  * 上层轨迹里的关节顺序与仿真模型不一致是常态，必须经 ``normalize_trajectory_points``
    重排并对缺失关节取当前值兜底，避免关节名错配导致的意外运动；
  * 本模块绝不接触真实电机，仿真与硬件共用的是"轨迹执行"的上层契约，不是硬件接口。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import bisect
import math
from typing import Iterable, Sequence


# 仿真模型中受控的六个转动关节；顺序即模型里 qpos/执行器的排列顺序，
# 也是对外反馈数组的元素顺序，改动本列表等同于改变接口约定。
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass(frozen=True)
class TrajectoryPoint:
    """轨迹上的一个采样点。

    time_from_start: 相对轨迹起点的时刻，单位秒；必须单调不减。
    positions: 与目标关节顺序一一对应的关节角，单位弧度。
    """

    time_from_start: float
    positions: list[float]


@dataclass(frozen=True)
class JointStateSnapshot:
    """一次关节状态快照，可直接映射为反馈消息。

    names/positions/velocities/efforts 四个数组等长且同序：
    位置单位弧度，速度单位弧度/秒，力矩单位牛·米（取自执行器实际输出力，
    不是估计值）。
    """

    names: list[str]
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


@dataclass(frozen=True)
class ToleranceViolation:
    """首个超出容差的关节及其误差。

    joint: 关节名；error: 带符号误差（弧度），正负表示偏大/偏小方向；
    abs_error: 绝对值，便于与容差直接比较。
    """

    joint: str
    error: float
    abs_error: float


def interpolate_trajectory(points: Sequence[TrajectoryPoint], elapsed: float) -> list[float]:
    """按线性插值取 ``elapsed`` 时刻的关节角（单位秒/弧度）。

    边界行为：空轨迹抛 ValueError；早于首点或晚于末点分别返回首、末点位置
    （即保持端点姿态，不外推）。两点时间相同时用 ratio=0 防止除零。
    """
    if not points:
        raise ValueError("trajectory contains no points")
    if elapsed <= points[0].time_from_start:
        return list(points[0].positions)
    if elapsed >= points[-1].time_from_start:
        return list(points[-1].positions)

    # bisect_left 得到第一个时间 >= elapsed 的点作为区间右端，左端为其前一点。
    times = [point.time_from_start for point in points]
    right = bisect.bisect_left(times, elapsed)
    left = max(0, right - 1)
    t0 = points[left].time_from_start
    t1 = points[right].time_from_start
    ratio = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
    return [
        float(p0) + (float(p1) - float(p0)) * ratio
        for p0, p1 in zip(points[left].positions, points[right].positions)
    ]


def normalize_trajectory_points(
    *,
    source_joint_names: Sequence[str],
    target_joint_names: Sequence[str],
    points: Iterable[object],
    current_positions: Sequence[float],
) -> list[TrajectoryPoint]:
    """把任意关节顺序的轨迹点重排为目标关节顺序。

    参数：
      source_joint_names: 轨迹点内部的位置顺序（例如规划器输出的关节名）。
      target_joint_names: 期望顺序（仿真模型的 qpos 顺序）。
      points: 轨迹点序列，支持字典（键 positions/time_from_start）或对象属性形式；
              ``time_from_start`` 既接受浮点秒数，也接受带 sec/nanosec 字段的时长对象。
      current_positions: 与 target_joint_names 对齐的当前位置，用于给缺失关节兜底
                         （单位弧度）。
    返回：按目标顺序排列、时间戳严格递增的轨迹点列表。
    异常：source 与 target 没有任何交集时抛 ValueError；轨迹为空同样抛错。
    安全语义：缺失关节一律保持当前位置而非归零，避免未规划关节突然回零造成撞机
    或超限报警。
    """
    source_names = [str(name) for name in source_joint_names]
    target_names = [str(name) for name in target_joint_names]
    # 只保留双方都有的关节；source 里出现但 target 没有的关节会被忽略。
    supported = [name for name in source_names if name in target_names]
    if not supported:
        raise ValueError("trajectory contains no supported joints")
    source_index = {name: index for index, name in enumerate(source_names)}
    # 目标关节没有对应位置时兜底为 0.0（只可能发生在调用方给的 current_positions 过短时）。
    target_current = {
        name: float(current_positions[index])
        for index, name in enumerate(target_names)
        if index < len(current_positions)
    }

    normalized: list[TrajectoryPoint] = []
    # 最小时间间隔 1e-3 秒：保证时间戳严格递增，供插值二分查找使用；
    # last_time 初值略小于 0，使首点允许 t=0，同时负时间戳会被抬到 0 附近。
    last_time = -1e-6
    for raw_point in points:
        positions = _positions_from_point(raw_point)
        target_positions: list[float] = []
        for name in target_names:
            if name in source_index and source_index[name] < len(positions):
                target_positions.append(float(positions[source_index[name]]))
            else:
                target_positions.append(float(target_current.get(name, 0.0)))
        time_from_start = max(_time_from_point(raw_point), last_time + 1e-3)
        last_time = time_from_start
        normalized.append(TrajectoryPoint(time_from_start=time_from_start, positions=target_positions))

    if not normalized:
        raise ValueError("trajectory contains no points")
    # 首点不在 t=0 时补一个"当前姿态"起点：否则插值会立刻跳到第一个目标点，
    # 表现为速度无穷大的阶跃。
    if normalized[0].time_from_start > 0.0:
        normalized.insert(
            0,
            TrajectoryPoint(
                time_from_start=0.0,
                positions=[float(target_current.get(name, 0.0)) for name in target_names],
            ),
        )
    return normalized


def gripper_width_to_ctrl(width: float, *, min_width: float = 0.0, max_width: float = 0.09) -> float:
    """把夹爪总开口宽度（米）换算为 MuJoCo 的位置控制量。

    物理关系：模型里两只手指由 equality 约束反向联动，只驱动左手指，
    因此单侧控制量 = 总开口 / 2。宽度先按给定的上下限排序夹紧
    （参数写反也能工作），默认范围 0~0.09 m 对应控制量 0~0.045。
    """
    lower = min(float(min_width), float(max_width))
    upper = max(float(min_width), float(max_width))
    clamped = min(max(float(width), lower), upper)
    return clamped * 0.5


def consume_sim_steps(
    *,
    wall_delta: float,
    timestep: float,
    pending_sim_seconds: float,
) -> tuple[int, float]:
    """把挂钟时间增量换算为"这次要推进多少物理步"，并保留不足一步的余量。

    参数：wall_delta 为本次回调自上一帧以来的挂钟秒数；timestep 为物理步长（秒）；
    pending_sim_seconds 为上次遗留的、不足一步的时间。
    返回：(整数步数, 新的余量秒数)。余量必须由调用方保存并在下次相加，
    否则慢回调下仿真会持续走慢。负的输入一律按 0 处理以抵抗时钟回退。
    异常：timestep <= 0 抛 ValueError（物理步长必须为正）。
    """
    if timestep <= 0.0:
        raise ValueError("MuJoCo timestep must be positive")
    pending = max(0.0, float(pending_sim_seconds)) + max(0.0, float(wall_delta))
    steps = int(pending / float(timestep))
    remainder = pending - float(steps) * float(timestep)
    return steps, remainder


def trajectory_error_code_for_stop_reason(stop_reason: str, *, result_type) -> int:
    """把内部停止原因映射为轨迹执行结果的错误码。

    映射：``finished``→SUCCESSFUL；``goal_tolerance_violated``→GOAL_TOLERANCE_VIOLATED；
    其余（stopped/canceled/timeout 等接近或超出路径容差的情况）→PATH_TOLERANCE_VIOLATED。
    ``result_type`` 由调用方注入（消息包里定义的结果常量），避免本模块依赖 ROS 消息。
    """
    if stop_reason == "finished":
        return int(result_type.SUCCESSFUL)
    if stop_reason == "goal_tolerance_violated":
        return int(result_type.GOAL_TOLERANCE_VIOLATED)
    return int(result_type.PATH_TOLERANCE_VIOLATED)


def execution_timeout_seconds(end_time: float, *, margin_sec: float = 2.0) -> float:
    """返回一次轨迹执行的挂钟超时预算（秒）。

    轨迹时长是以仿真秒计的；额外留出 margin_sec 以吸收定时器抖动与
    挂钟执行偏慢的余量，同时仍能给卡死的动作回调设下界限。
    结果为 max(时长 + 余量, 1e-6)，保证始终为正。
    """
    duration = max(float(end_time), 0.0)
    margin = max(float(margin_sec), 0.0)
    return max(duration + margin, 1e-6)


def first_tolerance_violation(
    *,
    joint_names: Sequence[str],
    errors: Sequence[float],
    tolerance: float,
) -> ToleranceViolation | None:
    """返回第一个绝对误差超过 ``tolerance`` 的关节（弧度）；都不超则 None。

    容差 <= 0 视为"不做检查"直接返回 None。joint_names 与 errors 按下标配对，
    较短的一方决定比较长度；一旦命中即返回，不做全部扫描。
    """
    if float(tolerance) <= 0.0:
        return None
    for joint, error in zip(joint_names, errors):
        abs_error = abs(float(error))
        if abs_error > float(tolerance):
            return ToleranceViolation(joint=str(joint), error=float(error), abs_error=abs_error)
    return None


def default_step_response_targets(motor_profiles: Iterable[object]) -> dict[str, float]:
    """为每个臂关节算一个"安全的阶跃测试目标位置"。

    算法：取该关节控制范围的中点，再朝上限方向走半程（即中点到上限的一半），
    即位于范围 3/4 处；若上限 <= 0（范围整体在负半轴），则改为取中点的 1/2。
    夹爪（left_finger）不参与阶跃测试，直接被跳过。
    返回：关节名 -> 目标角（弧度）。该结果只用于仿真整定/自检，不能作为运动指令。
    """
    targets: dict[str, float] = {}
    for profile in motor_profiles:
        joint = str(getattr(profile, "joint"))
        if joint == "left_finger":
            continue
        lower, upper = _parse_range(str(getattr(profile, "ctrlrange")))
        midpoint = (lower + upper) * 0.5
        target = midpoint + (upper - midpoint) * 0.5
        if upper <= 0.0:
            target = midpoint * 0.5
        targets[joint] = float(target)
    return targets


class MuJoCoArmAdapter:
    """MuJoCo 模型/数据的薄封装，供仿真执行器与无头检查共用。

    生命周期：构造时导入 MuJoCo、加载模型 XML、解析关节与执行器 id，
    并复位到 ``zero`` 关键帧；之后由调用方按固定节奏循环
    ``set_arm_targets`` → ``step`` → 读取状态。所有方法都是阻塞式的，
    不做线程保护——并发访问必须由调用方（仿真访问锁）串行化，否则会读到
    半更新的物理状态。

    索引约定：模型里的 qpos/qvel/ctrl 与关节、执行器 id 的映射在构造时一次性算好，
    运行期只按索引取值，避免每帧按名字查找的开销。

    安全语义：``healthy`` 只保证数值有限，不保证物理合理；仿真侧的任何异常都不应
    被当作真机可用性的证据。
    """

    def __init__(
        self,
        model_xml: Path,
        *,
        arm_joint_names: Sequence[str] = ARM_JOINT_NAMES,
    ) -> None:
        # 延迟导入：只有真正构造适配器时才需要 MuJoCo 运行库，
        # 让本模块的纯函数部分可以在未安装仿真依赖的环境中测试。
        import mujoco

        self._mujoco = mujoco
        self.model_xml = Path(model_xml).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_xml))
        self.data = mujoco.MjData(self.model)
        self.arm_joint_names = [str(name) for name in arm_joint_names]
        # 关节 id → qpos（位置）与 qvel（速度）地址；滑动关节与转动关节都只有 1 个自由度。
        self._joint_ids = [self._require_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.arm_joint_names]
        self._qpos_indices = [int(self.model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids]
        self._dof_indices = [int(self.model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids]
        self._actuator_ids = [self._require_actuator_for_joint(joint_id, name) for joint_id, name in zip(self._joint_ids, self.arm_joint_names)]
        # 夹爪与手指为可选对象：模型裁剪掉夹爪时适配器仍可用，只是宽度读取退化为记录值。
        self._gripper_actuator_id = self._optional_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
        self._left_finger_joint_id = self._optional_id(mujoco.mjtObj.mjOBJ_JOINT, "left_finger")
        self._right_finger_joint_id = self._optional_id(mujoco.mjtObj.mjOBJ_JOINT, "right_finger")
        self._last_gripper_width = 0.0
        self.reset("zero")

    @property
    def timestep(self) -> float:
        # 模型积分步长，单位秒；由 XML 的 option/timestep 决定（当前为 0.0025）。
        return float(self.model.opt.timestep)

    @property
    def sim_time(self) -> float:
        # 仿真内部累计时间，单位秒；与挂钟时间不同步，仅供轨迹采样使用。
        return float(self.data.time)

    def reset(self, keyframe: str | None = None) -> None:
        """复位物理状态：优先复位到指定关键帧，找不到该关键帧或未指定时归零。"""
        if keyframe:
            key_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_KEY, keyframe)
            if key_id >= 0:
                self._mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            else:
                self._mujoco.mj_resetData(self.model, self.data)
        else:
            self._mujoco.mj_resetData(self.model, self.data)
        self._mujoco.mj_forward(self.model, self.data)

    def step(self, steps: int = 1) -> None:
        """推进给定数量的物理步；步数下限为 1，避免无意义调用。"""
        for _ in range(max(1, int(steps))):
            self._mujoco.mj_step(self.model, self.data)

    def set_arm_targets(self, positions: Sequence[float]) -> None:
        """写入六个臂关节的位置控制量（弧度）。短于关节数的输入只写前几个，
        多出的元素被忽略；控制量随后由执行器的 ctrlrange 截断。"""
        for actuator_id, position in zip(self._actuator_ids, positions):
            self.data.ctrl[actuator_id] = float(position)

    def arm_positions(self) -> list[float]:
        # 关节角，单位弧度，顺序同 arm_joint_names。
        return [float(self.data.qpos[index]) for index in self._qpos_indices]

    def arm_velocities(self) -> list[float]:
        # 关节角速度，单位弧度/秒。
        return [float(self.data.qvel[index]) for index in self._dof_indices]

    def arm_actuator_forces(self) -> list[float]:
        # 执行器实际输出力/力矩（正值表示该执行器当前出力），单位牛·米。
        return [float(self.data.actuator_force[index]) for index in self._actuator_ids]

    def arm_actuator_force_limits(self) -> list[float | None]:
        """各关节的力矩限幅绝对值；执行器未开启限幅时为 None。

        取力限区间上下限绝对值的较大者，便于调用方统一按"是否接近上限"判断过载。
        """
        limits: list[float | None] = []
        for actuator_id in self._actuator_ids:
            if not int(self.model.actuator_forcelimited[actuator_id]):
                limits.append(None)
                continue
            lower, upper = (float(value) for value in self.model.actuator_forcerange[actuator_id])
            limits.append(max(abs(lower), abs(upper)))
        return limits

    def set_gripper_width(self, width: float, *, min_width: float = 0.0, max_width: float = 0.09) -> float:
        """设置夹爪总开口宽度（米），返回夹紧后的实际指令宽度。

        返回的是按参数范围夹紧后的宽度（= 控制量 × 2），反映"下发值"；
        真实开口请用 ``gripper_width`` 读取，两者在手指被物体挡住时可能不同。
        """
        ctrl = gripper_width_to_ctrl(width, min_width=min_width, max_width=max_width)
        if self._gripper_actuator_id is not None:
            self.data.ctrl[self._gripper_actuator_id] = ctrl
        self._last_gripper_width = ctrl * 2.0
        return self._last_gripper_width

    def gripper_width(self) -> float:
        """读取夹爪总开口（米）：左手指位移 × 2。

        模型无左手指关节时退回最近一次下发的宽度；单个手指的负位移按 0 处理，
        避免读取到未初始化状态时出现负宽度。
        """
        if self._left_finger_joint_id is None:
            return self._last_gripper_width
        qpos_index = int(self.model.jnt_qposadr[self._left_finger_joint_id])
        return max(0.0, float(self.data.qpos[qpos_index]) * 2.0)

    def joint_state_snapshot(self) -> JointStateSnapshot:
        """抓取一份臂关节状态快照（位置/速度/力矩），用于构造反馈消息。"""
        positions = self.arm_positions()
        velocities = self.arm_velocities()
        efforts = self.arm_actuator_forces()
        return JointStateSnapshot(
            names=list(self.arm_joint_names),
            positions=positions,
            velocities=velocities,
            efforts=efforts,
        )

    def healthy(self) -> bool:
        """所有 qpos/qvel 均为有限值时为 True。

        用于在把仿真状态对外发布前拦住数值发散（NaN/Inf：求解器炸开、物体飞出、
        参数不合法等），避免污染下游消费者。
        """
        values = list(self.data.qpos) + list(self.data.qvel)
        return all(math.isfinite(float(value)) for value in values)

    def _require_id(self, obj_type, name: str) -> int:
        obj_id = self._mujoco.mj_name2id(self.model, obj_type, name)
        if obj_id < 0:
            raise ValueError(f"MuJoCo object not found: {name}")
        return int(obj_id)

    def _optional_id(self, obj_type, name: str) -> int | None:
        obj_id = self._mujoco.mj_name2id(self.model, obj_type, name)
        return int(obj_id) if obj_id >= 0 else None

    def _require_actuator_for_joint(self, joint_id: int, name: str) -> int:
        # 扫描执行器，找传动类型为"关节"且目标关节 id 匹配的那一个。
        # 关节没有执行器时抛错（位置控制无法生效，属于配置错误，必须早失败）。
        for actuator_id in range(int(self.model.nu)):
            if (
                int(self.model.actuator_trntype[actuator_id]) == int(self._mujoco.mjtTrn.mjTRN_JOINT)
                and int(self.model.actuator_trnid[actuator_id, 0]) == int(joint_id)
            ):
                return actuator_id
        raise ValueError(f"MuJoCo actuator not found for joint: {name}")


def _positions_from_point(point: object) -> list[float]:
    # 兼容两种轨迹点载体：字典（消息反序列化后的结果）与带属性的对象。
    if isinstance(point, dict):
        return [float(value) for value in point.get("positions", [])]
    return [float(value) for value in getattr(point, "positions", [])]


def _time_from_point(point: object) -> float:
    # 时间字段同样兼容两种形式：float 秒数，或带 sec/nanosec 字段的时长对象
    # （后者按 1e-9 换算），缺失时按 0.0 处理。
    if isinstance(point, dict):
        return float(point.get("time_from_start", 0.0))
    duration = getattr(point, "time_from_start", None)
    if duration is None:
        return 0.0
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) * 1e-9


def _parse_range(value: str) -> tuple[float, float]:
    # 解析 MuJoCo 风格的 "下限 上限" 字符串；不是两个数就抛错，避免静默取错范围。
    parts = [float(part) for part in str(value).split()]
    if len(parts) != 2:
        raise ValueError(f"expected two range values, got: {value}")
    return parts[0], parts[1]

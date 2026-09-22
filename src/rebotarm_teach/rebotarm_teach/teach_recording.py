"""示教记录的数据模型、质量校验与预处理算法（示教包的核心算法层）。

职责与在系统中的位置
    本模块只处理"数据"：示教样本的定义与 JSONL 编解码、记录文件列举与巡检、轨迹质量
    评估、起点误差分档、以及回放前的预处理流水线（平滑 → 低通滤波 → 重采样 → 重定时）。
    它是纯计算 + 纯文件读写的库：不创建 ROS 节点、不发布话题、不调用服务或动作、不接触
    硬件。真正决定"是否放行真实回放"的节点与服务工作流在自己的模块里调用这里的结果。

    调用关系：
      - 采集侧（``TeachRecorderNode``）用 :func:`encode_teach_sample` / :func:`is_quit_key`
        把重力补偿示教过程中采到的样本逐行写成 JSONL；
      - 回放侧（回放节点与回放工作流）用 :func:`load_teach_samples` 读原始记录，用
        :func:`inspect_teach_record` 做文件巡检，用 :func:`prepare_teach_replay_samples`
        生成"预处理轨迹"，再用 :func:`validate_teach_dry_run_request` /
        :func:`validate_teach_replay_execute_request` / :func:`validate_teach_replay_stop_request`
        做 dry-run 与真实执行的门控；
      - 原始示教数据只是输入数据：真实回放必须执行预处理（过滤/重采样/重定时/校验）后的
        轨迹，而不是原始样本。

对外接口（全部为纯函数或不可变数据类）
    数据类：:class:`TeachSample`（单个采样点）、:class:`TeachTrajectoryEvent`（异常事件）、
    :class:`TeachTrajectoryQuality`（质量评估）、:class:`RetimedTeachPoint`（重定时后的
    轨迹点）、:class:`ReplayStartDecision` / :class:`TeachRecordInfo` /
    :class:`TeachDryRunDecision`（巡检与门控结果）、:class:`PreparedTeachReplay`（预处理
    结果汇总）。
    枚举：:class:`ReplayStartBand`，取值 ``direct`` / ``align`` / ``moveit_align`` /
    ``reject``；其中前三个是"文件检查通过、允许继续"的分档，``reject`` 表示起点误差过大，
    只能人工把手臂拖到记录起点附近后重来。

单位与时间基准（全模块统一，不再逐处重复）
    ``stamp`` 为记录时间（秒，来自机械臂状态消息的时间戳；预处理后的重定时轨迹改为
    "相对回放起点的时间"，同样以秒计，且必须严格单调递增）；关节位置 rad（仅旋转关节），
    速度 rad/s，加速度 rad/s^2，加加速度 jerk rad/s^3；阈值型参数（跳变、跟踪误差等）
    单位同为 rad。角度量一律不做角度制换算。

安全语义
    1. 质量分档 ``green`` / ``yellow`` / ``red`` 是"能不能真的动"的判据来源：
       ``red`` 直接禁止真实回放，``yellow`` 必须先做安全重定时并降速。
    2. 结构性异常（关节名不一致、位置向量长度不一致、时间戳非单调）只判 ``red``，
       绝不做"自动修复后继续"。
    3. 本模块的判定函数只返回决策对象，不抛异常给调用方（除少数参数长度不匹配的编程错误
       会显式 ``raise ValueError``）；``unknown`` / ``missing`` / ``invalid`` 等"说不清"
       的分档一律不得被当作可放行。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from rebotarm_motion.trajectory_time_parameterization import parameterize_teach_samples


@dataclass(frozen=True)
class TeachSample:
    """一帧重力补偿示教采样（不可变；关节顺序即 ``joint_names`` 的顺序）。

    ``stamp``        采样时间（秒），单调递增，用于差分出速度/加速度/加加速度；
    ``joint_names``  本帧包含的关节名，整条记录内应保持一致，否则判为结构性异常；
    ``positions``    关节位置（rad，仅旋转关节），与 ``joint_names`` 一一对应且等长；
    ``velocities``   关节速度（rad/s），可能为空元组（重采样后会清空）；
    ``efforts``      关节力矩/电流读数（原始采样单位，来自硬件反馈），可能为空；
    ``motor_status`` 电机状态码字典，键为电机标识、值为整型状态码（硬件定义），可能为空；
    ``arm_state``    采集当时的机械臂状态字符串（由采集侧写入，回放侧只透传不解释）。

    注意：``velocities`` / ``efforts`` / ``motor_status`` 允许为空，凡是要用到它们的逻辑都必须
    先判空，不能假定一定存在。
    """

    stamp: float
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    motor_status: dict[str, int]
    arm_state: str


@dataclass(frozen=True)
class TeachTrajectoryEvent:
    """轨迹质量评估中的一条异常事件（一条事件只对应某一帧的某一个关节）。

    ``sample``            发生异常的样本下标（从 0 开始；首帧不参与差分，故不会是 0）；
    ``joint_name``        出问题的关节名；
    ``level``             事件等级字符串：``red``（跳变超黄线，禁止回放）或 ``yellow``
                          （跳变超绿线 / 速度 / 加速度 / 加加速度越限，需降速或重定时）；
    ``message``           人类可读的事件描述（同时被收集进 anomalies 列表）；
    ``delta_rad``         该帧该关节的位置增量绝对值（rad）；
    ``velocity_rad_s``    差分速度绝对值（rad/s）；时间差为 0 等无法差分时为 None；
    ``acceleration_rad_s2`` 差分加速度绝对值（rad/s^2）；无法差分时为 None；
    ``jerk_rad_s3``       差分加加速度绝对值（rad/s^3）；无法差分时为 None。
    """

    sample: int
    joint_name: str
    level: str
    message: str
    delta_rad: float
    velocity_rad_s: float | None
    acceleration_rad_s2: float | None = None
    jerk_rad_s3: float | None = None


@dataclass(frozen=True)
class TeachTrajectoryQuality:
    """一条示教轨迹的质量评估结果（由 :func:`analyze_teach_trajectory` 产生）。

    ``risk_level``       总风险等级：``green``（正常回放）/ ``yellow``（需安全重定时）/
                         ``red``（禁止真实回放）；
    ``replay_policy``    与等级对应的策略说明文本（英文，供上层 UI 直接展示）；
    ``allow_real_replay`` 是否允许真实回放，等价于 ``risk_level != "red"``；
    ``requires_safe_retiming`` 是否必须先做安全重定时，等价于 ``risk_level == "yellow"``；
    ``max_jump_rad``     全轨迹最大单帧关节增量（rad）；
    ``max_velocity_rad_s`` / ``max_acceleration_rad_s2`` / ``max_jerk_rad_s3``
                         差分得到的最大速度 / 加速度 / 加加速度（rad/s、rad/s^2、rad/s^3）；
    ``worst_joint`` / ``worst_sample`` 最大跳变所在关节与样本下标（无跳变时分别
                         为空串和 -1）；
    ``anomalies``        全部异常描述（去重前），含结构性异常与越限事件；
    ``events``           结构化的异常事件元组；
    ``green_jump_rad`` / ``yellow_jump_rad`` 判定时使用的绿/黄跳变阈值（rad），随结果一起
                         回传，便于上层复现判定条件；
    ``velocity_limit_rad_s`` 生效的速度上限（rad/s，多关节时取各关节上限中的最大值）；
    ``acceleration_limit_rad_s2`` / ``jerk_limit_rad_s3`` 生效的加速度、加加速度上限；
    ``max_jerk_rad_s3``  实测最大 jerk（rad/s^3），与 ``jerk_limit_rad_s3``（阈值）区分。
    """

    risk_level: str
    replay_policy: str
    allow_real_replay: bool
    requires_safe_retiming: bool
    max_jump_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    worst_joint: str
    worst_sample: int
    anomalies: tuple[str, ...]
    events: tuple[TeachTrajectoryEvent, ...]
    green_jump_rad: float
    yellow_jump_rad: float
    velocity_limit_rad_s: float
    acceleration_limit_rad_s2: float
    max_jerk_rad_s3: float = 0.0
    jerk_limit_rad_s3: float = 999.0


@dataclass(frozen=True)
class RetimedTeachPoint:
    """重定时后的一段轨迹点（时间轴已重排，位置仍取自原始示教样本）。

    ``time_from_start`` 相对本段轨迹起点的时间（秒），严格递增；
    ``positions``       关节位置（rad），顺序与来源记录的 ``joint_names`` 一致；
    ``source_sample``   对应的原始样本下标；由软启动/对齐等步骤插入的合成点为 -1；
    ``velocities``      重定时算出的关节速度（rad/s，位置对时间的差分），可为空元组。
    """

    time_from_start: float
    positions: tuple[float, ...]
    source_sample: int
    velocities: tuple[float, ...] = ()


@dataclass(frozen=True)
class PreparedTeachReplay:
    """一条示教记录的完整预处理结果（原始质量 → 滤波后质量 → 重定时后质量）。

    中间数据：
    ``samples``            预处理流水线最终得到的样本序列（平滑 + 滤波 + 重采样之后）；
    ``raw_quality``        原始记录的质量评估（回放前的"before"）；
    ``filtered_quality``   平滑 + 滤波之后、重采样/重定时之前的质量评估；
    ``retimed_quality``    重定时之后（若未重定时则为 ``samples``）的质量评估，也就是
                           真实回放实际执行的那条轨迹的"after"；
    各步骤是否真正生效（供 UI 展示实际生效的处理链，注意 ``*_applied`` 表示"这一步跑过"，
    不等同于"数据被改变了"）：
    ``smoothing_applied`` / ``filter_applied`` / ``resample_applied`` / ``retime_applied``；
    生效参数（都是被夹到安全下限之后的实际值）：
    ``smoothing_window``   平滑窗口长度（奇数，样本数）；
    ``filter_cutoff_hz``   低通滤波截止频率（Hz）；
    ``filter_sample_rate_hz`` 滤波假定的采样率（Hz）；
    ``resample_rate_hz``   重采样目标频率（Hz）；
    ``retimed_points``     重定时轨迹点列表；为空表示未做重定时；
    大幅度运动与降速信息：
    ``large_motion``       轨迹行程是否达到"大幅度"判据；
    ``max_joint_span_rad`` 单关节最大行程（rad，各关节位置极差的最大值）；
    ``total_joint_motion_rad`` 全轨迹各关节增量绝对值之和（rad），反映总运动量；
    ``requested_replay_speed`` 请求的回放倍速（已夹到 (0, 1]）；
    ``effective_replay_speed`` 实际生效倍速（当前实现等于请求值，保留字段以便后续限速）；
    ``large_motion_max_speed`` 大幅度运动允许的最大倍速（越小越慢越安全）；
    时间参数化后端信息：
    ``time_parameterization_requested_method`` 请求的方法名（``auto`` / ``ruckig`` /
                          ``current_jerk_retime`` 等）；
    ``time_parameterization_used_method`` 实际使用的方法名，未重定时时为 ``none``；
    ``time_parameterization_message`` 后端给出的说明（英文，含回退原因）。

    属性 ``before_quality`` / ``after_quality`` 是门控代码使用的简写：分别指向原始质量与
    重定时后质量。
    """

    samples: list[TeachSample]
    raw_quality: TeachTrajectoryQuality
    filtered_quality: TeachTrajectoryQuality
    retimed_quality: TeachTrajectoryQuality
    smoothing_applied: bool
    filter_applied: bool
    resample_applied: bool
    retime_applied: bool
    smoothing_window: int
    filter_cutoff_hz: float
    filter_sample_rate_hz: float
    resample_rate_hz: float
    retimed_points: list[RetimedTeachPoint]
    large_motion: bool = False
    max_joint_span_rad: float = 0.0
    total_joint_motion_rad: float = 0.0
    requested_replay_speed: float = 1.0
    effective_replay_speed: float = 1.0
    large_motion_max_speed: float = 1.0
    time_parameterization_requested_method: str = "auto"
    time_parameterization_used_method: str = "current_jerk_retime"
    time_parameterization_message: str = ""

    @property
    def before_quality(self) -> TeachTrajectoryQuality:
        return self.raw_quality

    @property
    def after_quality(self) -> TeachTrajectoryQuality:
        return self.retimed_quality


def is_quit_key(key: str | None, *, quit_key: str = "q") -> bool:
    """判断键盘输入是否等同于"退出示教采集"。

    忽略大小写与首尾空白；``key`` 为 None（例如非阻塞读取没读到按键）时返回 False。
    只做纯字符串比较，不读取键盘。
    """
    if key is None:
        return False
    return str(key).strip().lower() == str(quit_key).strip().lower()


def encode_teach_sample(sample: TeachSample) -> str:
    """把一个示教样本编码成单行 JSON（JSONL 的一行）。

    分隔符使用最紧凑形式 ``(",", ":")``：采样率高、记录文件大，这里省掉空格。
    元组被转成列表以便 JSON 序列化；键名是记录文件格式的一部分，不可改动。
    """
    return json.dumps(
        {
            "stamp": sample.stamp,
            "joint_names": list(sample.joint_names),
            "positions": list(sample.positions),
            "velocities": list(sample.velocities),
            "efforts": list(sample.efforts),
            "motor_status": sample.motor_status,
            "arm_state": sample.arm_state,
        },
        separators=(",", ":"),
    )


def decode_teach_sample(payload: str) -> TeachSample:
    """把一行 JSON 解析回示教样本（与 :func:`encode_teach_sample` 互为逆操作）。

    ``velocities`` / ``efforts`` / ``motor_status`` / ``arm_state`` 都是可选字段，缺失时
    分别退化为空元组、空元组、空字典和空串，以兼容早期版本写出的记录文件。
    数值字段强制转 float / int，字符串字段强制转 str；JSON 本身非法时异常会上抛，
    由调用方（:func:`inspect_teach_record`）决定如何降级处理。
    """
    data = json.loads(payload)
    return TeachSample(
        stamp=float(data["stamp"]),
        joint_names=tuple(str(v) for v in data["joint_names"]),
        positions=tuple(float(v) for v in data["positions"]),
        velocities=tuple(float(v) for v in data.get("velocities", [])),
        efforts=tuple(float(v) for v in data.get("efforts", [])),
        motor_status={str(k): int(v) for k, v in data.get("motor_status", {}).items()},
        arm_state=str(data.get("arm_state", "")),
    )


def load_teach_samples(path: str | Path) -> list[TeachSample]:
    """按行读取 JSONL 示教记录，返回样本列表（文件内的顺序即时间顺序）。

    以 ``utf-8-sig`` 打开：兼容带 BOM 的文件（采集侧可能由不同工具写入）。
    空行被跳过；单行解析失败会直接抛出异常，不会静默丢弃那一帧——记录缺帧会改变轨迹
    时序，必须让上层察觉。
    """
    samples: list[TeachSample] = []
    with Path(path).open("r", encoding="utf-8-sig") as handle:
        for line in handle:
            line = line.strip()
            if line:
                samples.append(decode_teach_sample(line))
    return samples


def prepared_record_path(raw_path: str | Path) -> Path:
    """由原始记录路径推导"预处理记录"的落盘路径。

    规则：在扩展名前插入 ``.prepared``（``a.jsonl`` → ``a.prepared.jsonl``）；
    没有扩展名时直接追加 ``.prepared.jsonl``。这样预处理结果与原始记录同目录、同名前缀，
    列举原始记录时可以按后缀把它们排除掉。
    """
    path = Path(raw_path)
    if path.suffix:
        return path.with_name(f"{path.stem}.prepared{path.suffix}")
    return path.with_name(f"{path.name}.prepared.jsonl")


def write_prepared_teach_record(
    raw_path: str | Path,
    prepared: PreparedTeachReplay,
    *,
    output_path: str | Path | None = None,
) -> Path:
    """把预处理结果写成 JSONL 文件（真实回放实际执行的那条轨迹），返回落盘路径。

    写出的内容分两种情况：
      1. 有重定点（``prepared.retimed_points`` 非空）时，以重定点为准：``stamp`` 写
         ``time_from_start``（相对回放起点的时间，秒），速度写重定时算出的速度，
         力矩/电机状态清空，``arm_state`` 标记为 ``PREPARED_REPLAY``；
      2. 没有重定点时，按重采样频率把预处理样本重新打时间戳（第 i 点 = i / 采样率），
         位置取预处理样本，``arm_state`` 同样标记为 ``PREPARED_REPLAY``。
    ``output_path`` 为空时落到 :func:`prepared_record_path` 推导出的默认路径；父目录会自动
    创建。注意：这里写的是**派生数据**，原始记录文件不会被改写；``arm_state`` 标记用于让
    下游一眼看出该文件不是手工示教采集的原始记录。
    """
    target = Path(output_path) if output_path is not None else prepared_record_path(raw_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    samples: list[TeachSample]
    if prepared.retimed_points:
        joint_names = prepared.samples[0].joint_names if prepared.samples else ()
        samples = [
            TeachSample(
                stamp=float(point.time_from_start),
                joint_names=joint_names,
                positions=point.positions,
                velocities=point.velocities,
                efforts=(),
                motor_status={},
                arm_state="PREPARED_REPLAY",
            )
            for point in prepared.retimed_points
        ]
    else:
        samples = [
            TeachSample(
                # 没有重定时信息时只能按目标采样率等间隔重建时间轴；采样率下限 1 Hz，
                # 避免错误的 0 值导致除零。
                stamp=float(index) / max(float(prepared.resample_rate_hz), 1.0),
                joint_names=sample.joint_names,
                positions=sample.positions,
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=sample.motor_status,
                arm_state="PREPARED_REPLAY",
            )
            for index, sample in enumerate(prepared.samples)
        ]
    with target.open("w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(encode_teach_sample(sample) + "\n")
    return target


class ReplayStartBand(str, Enum):
    """回放起点误差分档（也是真实回放的第一道门）。

    ``direct``       当前位姿已足够接近记录起点，可直接回放，无需对齐；
    ``align``        误差较小，允许自动对齐（软启动/对齐段）后再回放；
    ``moveit_align`` 需要走 MoveIt 规划对齐（由上层工作流实现），本模块只把它当作合法分档；
    ``reject``       误差过大，禁止回放，必须人工把机械臂拖到记录起点附近。
    """

    DIRECT = "direct"
    ALIGN = "align"
    MOVEIT_ALIGN = "moveit_align"
    REJECT = "reject"


@dataclass(frozen=True)
class ReplayStartDecision:
    """起点误差分档结果。

    ``band``            分档（见 :class:`ReplayStartBand`）；
    ``max_error``       各关节误差绝对值的最大值（rad）；关节数不匹配时为正无穷；
    ``per_joint_error`` 逐关节误差绝对值（rad），顺序与传入的关节向量一致；分档为
                        ``reject`` 且因长度不匹配时为空元组；
    ``allow_replay``    是否允许（含自动对齐后）回放；
    ``allow_auto_align`` 是否允许自动对齐（只有 ``align`` 档为 True）；
    ``message``         说明文本（英文，供 UI 直接展示）。
    """

    band: ReplayStartBand
    max_error: float
    per_joint_error: tuple[float, ...]
    allow_replay: bool
    allow_auto_align: bool
    message: str


@dataclass(frozen=True)
class TeachRecordInfo:
    """一条示教记录文件的巡检结果（文件是否存在、能否解析、以及质量与起点误差）。

    ``path``            记录文件路径（字符串形式）；
    ``exists``          文件是否存在；
    ``samples``         解析出的样本数（文件缺失/损坏/为空时为 0）；
    ``duration_sec``    记录时长（秒，末帧时间戳减首帧时间戳，负数被夹到 0）；
    ``joint_names``     关节顺序（取自首帧）；
    ``start_positions`` / ``end_positions`` 首帧与末帧的关节位置（rad）；
    ``start_band``      起点分档字符串：``direct``/``align``/``moveit_align``/``reject``，
                        以及异常取值 ``missing``（文件不存在）、``invalid``（JSONL 解析
                        失败）、``empty``（没有样本）、``unknown``（没有当前关节状态或缺少
                        记录里的关节，无法比较）；
    ``max_error``       起点最大关节误差（rad）；无法比较时为 None；
    ``worst_joint``     误差最大的关节名（无误差信息时为空串）；
    ``per_joint_error`` 逐关节误差（rad），键为关节名；
    ``anomalies``       异常描述元组（结构巡检与质量评估结果合并去重后）；
    ``message``         说明文本（英文）；
    ``quality``         轨迹质量评估；文件缺失或损坏时为 None。
    """

    path: str
    exists: bool
    samples: int
    duration_sec: float
    joint_names: tuple[str, ...]
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]
    start_band: str
    max_error: float | None
    worst_joint: str
    per_joint_error: dict[str, float]
    anomalies: tuple[str, ...]
    message: str
    quality: TeachTrajectoryQuality | None = None


@dataclass(frozen=True)
class TeachDryRunDecision:
    """dry-run / 真实回放 / 停止请求的统一门控结果。

    ``accepted`` 是否接受该请求；
    ``state``    请求被接受后应进入的状态字符串（如 ``dry_run``、``replaying``、
                 ``cancel_requested``），被拒绝时为 ``blocked``（无活动目标时停止请求返回
                 ``idle``）；
    ``message``  拒绝或接受的原因（英文，供 UI 直接展示）。
    """

    accepted: bool
    state: str
    message: str


def classify_replay_start(
    *,
    current_positions: tuple[float, ...],
    start_positions: tuple[float, ...],
    direct_threshold: float,
    align_threshold: float,
) -> ReplayStartDecision:
    """按最大关节误差给回放起点分档（本模块的第一道安全门）。

    参数均为 rad：``current_positions`` 当前关节位置，``start_positions`` 记录起点关节位置，
    ``direct_threshold`` 直接回放上限，``align_threshold`` 允许自动对齐的上限（应大于前者）。
    判定用严格小于：``max_error < direct_threshold`` → ``direct``；
    ``max_error < align_threshold`` → ``align``；否则 ``reject``。
    两个关节向量长度不一致时直接 ``reject``（正无穷误差），因为逐关节比较没有意义。
    """
    if len(current_positions) != len(start_positions):
        return ReplayStartDecision(
            band=ReplayStartBand.REJECT,
            max_error=float("inf"),
            per_joint_error=(),
            allow_replay=False,
            allow_auto_align=False,
            message="current and start joint vectors have different lengths",
        )
    errors = tuple(abs(float(a) - float(b)) for a, b in zip(current_positions, start_positions))
    max_error = max(errors, default=0.0)
    if max_error < float(direct_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.DIRECT,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=False,
            message="current pose is close enough to replay start",
        )
    if max_error < float(align_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.ALIGN,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=True,
            message="small return_to_start alignment required",
        )
    return ReplayStartDecision(
        band=ReplayStartBand.REJECT,
        max_error=max_error,
        per_joint_error=errors,
        allow_replay=False,
        allow_auto_align=False,
        message="start error too large; manually drag the arm near the recording start",
    )


def interpolate_joint_positions(
    *,
    current_positions: tuple[float, ...],
    target_positions: tuple[float, ...],
    steps: int,
) -> list[tuple[float, ...]]:
    """在两组关节位置之间做等参数线性插值，返回含首末点的插值序列（关节角，rad）。

    ``steps`` 是**总点数**而非段数，且被夹到至少 2（否则无法同时包含首末点）；
    第 i 个点的系数为 ``i / (steps - 1)``，因此首点等于 ``current_positions``、
    末点等于 ``target_positions``。两个向量长度必须一致，否则抛 ``ValueError``
    （这是调用方的编程错误，不是可恢复的运行状态）。
    """
    if len(current_positions) != len(target_positions):
        raise ValueError("current and target joint vectors have different lengths")
    count = max(int(steps), 2)
    points: list[tuple[float, ...]] = []
    for index in range(count):
        alpha = float(index) / float(count - 1)
        points.append(
            tuple(
                float(current + (target - current) * alpha)
                for current, target in zip(current_positions, target_positions)
            )
        )
    return points


def build_replay_start_soft_points(
    *,
    current_positions: tuple[float, ...],
    first_positions: tuple[float, ...],
    start_band: str,
    start_hold_sec: float = 0.8,
    soft_start_duration: float = 1.0,
    soft_start_steps: int = 30,
    align_duration: float = 3.0,
    align_steps: int = 30,
    first_hold_sec: float = 0.3,
) -> list[RetimedTeachPoint]:
    """构造回放开始前的"保持 → 对齐 → 再保持"引导段（位置单位 rad，时间单位秒）。

    段结构（合成点的 ``source_sample`` 一律为 -1，便于与真实示教帧区分）：
      1. 起始保持段：时长 ``start_hold_sec``，停在当前位姿，给控制器和操作者一个缓冲；
      2. 对齐段：从当前位姿线性插值到记录首帧位姿。分档为 ``align`` 时使用
         ``align_duration`` / ``align_steps``（较慢较长，属于真正的位置对齐）；其余分档
         （如 ``direct``）使用 ``soft_start_duration`` / ``soft_start_steps`` 的软启动；
      3. 首帧保持段：时长 ``first_hold_sec``，停在记录首帧，确保进入示教轨迹时速度为零。
    时长与点数都被夹到非负/至少 2；时间戳必须严格递增，若某合成点的时间不大于上一点则
    直接丢弃，避免生成零时长或时间回退的点（重定时/执行要求时间严格单调）。
    """
    if len(current_positions) != len(first_positions):
        raise ValueError("current and first joint vectors have different lengths")
    elapsed = 0.0
    points: list[RetimedTeachPoint] = []
    hold = max(float(start_hold_sec), 0.0)
    if hold > 0.0:
        elapsed += hold
        points.append(
            RetimedTeachPoint(
                time_from_start=elapsed,
                positions=tuple(float(v) for v in current_positions),
                source_sample=-1,
            )
        )
    band = str(start_band or "").strip().lower()
    if band == ReplayStartBand.ALIGN.value:
        # 真正的"回起始点"对齐：用更长的时长和更多的点，避免对齐本身产生速度冲击。
        duration = max(float(align_duration), 0.0)
        steps = int(align_steps)
    else:
        # 起点已经足够接近：只做一段短软启动，把速度从 0 平滑拉起来。
        duration = max(float(soft_start_duration), 0.0)
        steps = int(soft_start_steps)
    align_points = interpolate_joint_positions(
        current_positions=tuple(float(v) for v in current_positions),
        target_positions=tuple(float(v) for v in first_positions),
        steps=steps,
    )
    for index, positions in enumerate(align_points):
        ratio = float(index) / float(max(len(align_points) - 1, 1))
        timestamp = elapsed + duration * ratio
        # 时长为 0 或点数过多时会算出与上一点相同/更早的时间戳，必须跳过。
        if points and timestamp <= points[-1].time_from_start:
            continue
        points.append(
            RetimedTeachPoint(
                time_from_start=timestamp,
                positions=tuple(float(v) for v in positions),
                source_sample=-1,
            )
        )
    elapsed += duration
    first_hold = max(float(first_hold_sec), 0.0)
    if first_hold > 0.0:
        elapsed += first_hold
        if not points or elapsed > points[-1].time_from_start:
            points.append(
                RetimedTeachPoint(
                    time_from_start=elapsed,
                    positions=tuple(float(v) for v in first_positions),
                    source_sample=-1,
                )
            )
    return points


def _max_position_delta(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    """两组关节位置之间逐关节差值的最大绝对值（rad）；长度不一致返回正无穷。"""
    if len(a) != len(b):
        return float("inf")
    return max((abs(float(current) - float(last)) for current, last in zip(a, b)), default=0.0)


def compute_auto_align_duration(
    max_error_rad: float | None,
    *,
    target_speed_rad_s: float = 0.15,
    min_duration_sec: float = 3.0,
    max_duration_sec: float = 10.0,
) -> float:
    """按"误差 / 目标速度"估算自动对齐所需时长，并夹在给定区间内（返回秒）。

    参数：``max_error_rad`` 起点最大关节误差（rad，None 或非数值按 0 处理）；
    ``target_speed_rad_s`` 对齐时允许的平均关节速度（rad/s，下限 0.01，防止除零）；
    ``min_duration_sec`` / ``max_duration_sec`` 时长下上限（秒）。
    误差为 0 或无法解析时直接返回下限时长；误差很大时时长被 ``max_duration_sec`` 截断，
    即"宁可慢也绝不为了赶时间提速"。
    """
    try:
        error = abs(float(max_error_rad))
    except (TypeError, ValueError):
        error = 0.0
    speed = max(float(target_speed_rad_s), 0.01)
    duration = error / speed if error > 0.0 else float(min_duration_sec)
    return min(max(duration, float(min_duration_sec)), float(max_duration_sec))


def _motion_scope(samples: list[TeachSample]) -> tuple[float, float]:
    """统计轨迹运动规模，返回 ``(单关节最大行程, 全轨迹总运动量)``，单位均为 rad。

    单关节最大行程 = 每个关节位置极差（最大减最小）中的最大值；
    总运动量 = 相邻帧之间所有关节增量绝对值之和（反映"走了多远的折线距离"）。
    样本为空、关节数为 0 或某帧位置长度与首帧不一致时返回 ``(inf, inf)``——长度不一致属于
    结构性异常，返回无穷大可以让上层按"大幅度运动"从严处理，而不是低估风险。
    """
    if not samples:
        return 0.0, 0.0
    joint_count = len(samples[0].positions)
    if joint_count == 0:
        return 0.0, 0.0
    mins = [float("inf") for _ in range(joint_count)]
    maxs = [float("-inf") for _ in range(joint_count)]
    total = 0.0
    previous: TeachSample | None = None
    for sample in samples:
        if len(sample.positions) != joint_count:
            return float("inf"), float("inf")
        for index, position in enumerate(sample.positions):
            value = float(position)
            mins[index] = min(mins[index], value)
            maxs[index] = max(maxs[index], value)
        if previous is not None:
            total += sum(
                abs(float(current) - float(last))
                for current, last in zip(sample.positions, previous.positions)
            )
        previous = sample
    max_span = max((upper - lower for lower, upper in zip(mins, maxs)), default=0.0)
    return max_span, total


def _velocity_limits_for_joints(
    max_velocity_rad_s,
    joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    """把速度上限参数展开成与 ``joint_names`` 等长的逐关节上限（rad/s）。

    支持三种写法：字典（键为关节名，``"*"`` 为缺省值，默认 2.0 rad/s）、与关节数等长的
    列表/元组、以及单值（所有关节同限）。每个上限都被夹到至少 0.01 rad/s，防止 0 或负数
    导致后续按速度推算时间时除零；列表长度不等于关节数时抛 ``ValueError``。
    """
    if isinstance(max_velocity_rad_s, dict):
        fallback = float(max_velocity_rad_s.get("*", 2.0))
        return tuple(max(float(max_velocity_rad_s.get(name, fallback)), 0.01) for name in joint_names)
    if isinstance(max_velocity_rad_s, (list, tuple)):
        if len(max_velocity_rad_s) != len(joint_names):
            raise ValueError("max_velocity_rad_s length must match joint_names")
        return tuple(max(float(value), 0.01) for value in max_velocity_rad_s)
    return tuple(max(float(max_velocity_rad_s), 0.01) for _ in joint_names)


def _velocity_limit_summary(max_velocity_rad_s) -> float:
    """把速度上限参数压成一个标量用于回填质量结果：多关节时取最大值，单位 rad/s。"""
    if isinstance(max_velocity_rad_s, dict):
        values = [float(value) for value in max_velocity_rad_s.values()]
        return max(values, default=0.0)
    if isinstance(max_velocity_rad_s, (list, tuple)):
        return max((float(value) for value in max_velocity_rad_s), default=0.0)
    return float(max_velocity_rad_s)


def analyze_teach_trajectory(
    samples: list[TeachSample],
    *,
    green_jump_rad: float = 0.03,
    yellow_jump_rad: float = 0.05,
    max_velocity_rad_s: float = 2.0,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
) -> TeachTrajectoryQuality:
    """逐帧差分评估示教轨迹质量，给出 green/yellow/red 分档与异常清单。

    判定规则（阈值单位：rad、rad/s、rad/s^2、rad/s^3）：
      - 单帧关节增量 > ``yellow_jump_rad`` 记 ``red`` 事件（并按 red 处理，禁止真实回放）；
      - 增量在 ``green_jump_rad`` 与 ``yellow_jump_rad`` 之间记 ``yellow`` 事件；
      - 速度 / 加速度 / 加加速度超过各自上限各记一条 ``yellow`` 异常；
      - 关节名不一致、位置向量长度不一致、时间戳非单调（``dt <= 0``）记结构性异常并直接
        判 ``red``：这类记录连差分都不可信，不能靠重定时"抢救"。
    风险等级只升不降（已经 red 不会被后续 yellow 覆盖）。速度/加速度/加加速度用相邻帧差分，
    因此对采集噪声敏感，这也是后续要做滤波与重采样的原因。
    ``samples`` 为空时返回 red 且 ``replay_policy`` 为 "record contains no samples"。
    """
    anomalies: list[str] = []
    events: list[TeachTrajectoryEvent] = []
    risk_level = "green"
    max_jump = 0.0
    max_velocity = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    worst_joint = ""
    worst_sample = -1
    if not samples:
        return TeachTrajectoryQuality(
            risk_level="red",
            replay_policy="record contains no samples",
            allow_real_replay=False,
            requires_safe_retiming=False,
            max_jump_rad=0.0,
            max_velocity_rad_s=0.0,
            max_acceleration_rad_s2=0.0,
            max_jerk_rad_s3=0.0,
            worst_joint="",
            worst_sample=-1,
            anomalies=("empty record",),
            events=(),
            green_jump_rad=float(green_jump_rad),
            yellow_jump_rad=float(yellow_jump_rad),
            velocity_limit_rad_s=_velocity_limit_summary(max_velocity_rad_s),
            acceleration_limit_rad_s2=float(max_acceleration_rad_s2),
            jerk_limit_rad_s3=float(max_jerk_rad_s3),
        )
    expected_joints = samples[0].joint_names
    expected_len = len(expected_joints)
    velocity_limits = _velocity_limits_for_joints(max_velocity_rad_s, expected_joints)
    previous = samples[0]
    # 首帧没有前一帧，差分初值取零：首帧速度按 0 处理，避免伪造出一个"跳跃起步"。
    previous_velocities = tuple(0.0 for _ in range(expected_len))
    previous_accelerations = tuple(0.0 for _ in range(expected_len))
    if len(previous.positions) != expected_len:
        anomalies.append("positions length mismatch at sample 0")
        risk_level = "red"
    for index, sample in enumerate(samples[1:], start=1):
        if sample.joint_names != expected_joints:
            anomalies.append(f"joint_names mismatch at sample {index}")
            risk_level = "red"
        if len(sample.positions) != expected_len:
            anomalies.append(f"positions length mismatch at sample {index}")
            risk_level = "red"
        dt = float(sample.stamp) - float(previous.stamp)
        if dt <= 0.0:
            # 时间戳不前进：差分方向无意义，整条记录判 red，且下面不再使用该 dt。
            anomalies.append(f"timestamp not monotonic at sample {index}")
            risk_level = "red"
        current_velocities = tuple(0.0 for _ in range(expected_len))
        current_accelerations = tuple(0.0 for _ in range(expected_len))
        if dt > 0.0 and len(sample.positions) == expected_len and len(previous.positions) == expected_len:
            # 一阶差分得速度 (rad/s)，再对速度一阶差分得加速度 (rad/s^2)。
            current_velocities = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(sample.positions, previous.positions)
            )
            current_accelerations = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(current_velocities, previous_velocities)
            )
        for joint_index, (joint_name, current, last) in enumerate(zip(expected_joints, sample.positions, previous.positions)):
            delta = abs(float(current) - float(last))
            velocity = abs(current_velocities[joint_index]) if dt > 0.0 else None
            # 只记录"更差"的极值，用于回填 worst_joint / worst_sample（最大单帧跳变点）。
            if delta > max_jump:
                max_jump = delta
                worst_joint = joint_name
                worst_sample = index
            if velocity is not None and velocity > max_velocity:
                max_velocity = velocity
            acceleration = None
            jerk = None
            if velocity is not None and dt > 0.0:
                acceleration = abs(float(current_accelerations[joint_index]))
                if acceleration > max_acceleration:
                    max_acceleration = acceleration
                # 加加速度 = 相邻两帧加速度之差除以 dt (rad/s^3)。
                jerk = abs(float(current_accelerations[joint_index]) - float(previous_accelerations[joint_index])) / dt
                if jerk > max_jerk:
                    max_jerk = jerk
            level = ""
            message = ""
            if delta > float(yellow_jump_rad):
                level = "red"
                message = f"{joint_name} jump {delta:.4f} rad at sample {index}"
                risk_level = "red"
                anomalies.append(message)
            elif delta > float(green_jump_rad):
                level = "yellow"
                message = f"{joint_name} jump {delta:.4f} rad at sample {index}"
                if risk_level != "red":
                    risk_level = "yellow"
            velocity_limit = velocity_limits[joint_index]
            if velocity is not None and velocity > velocity_limit:
                # 同一帧已有更严重的事件（跳变）时保留原 message/level，只追加异常行。
                velocity_message = f"{joint_name} velocity {velocity:.4f} rad/s at sample {index}"
                if not message:
                    message = velocity_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(velocity_message)
            if acceleration is not None and acceleration > float(max_acceleration_rad_s2):
                acceleration_message = f"{joint_name} acceleration {acceleration:.4f} rad/s^2 at sample {index}"
                if not message:
                    message = acceleration_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(acceleration_message)
            if jerk is not None and jerk > float(max_jerk_rad_s3):
                jerk_message = f"{joint_name} jerk {jerk:.4f} rad/s^3 at sample {index}"
                if not message:
                    message = jerk_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(jerk_message)
            if level:
                events.append(
                    TeachTrajectoryEvent(
                        sample=index,
                        joint_name=joint_name,
                        level=level,
                        message=message,
                        delta_rad=delta,
                        velocity_rad_s=velocity,
                        acceleration_rad_s2=acceleration,
                        jerk_rad_s3=jerk,
                    )
                )
        if dt > 0.0:
            # 只有 dt 有效时才滚动差分状态，避免非法 dt 污染后续帧的加速度/jerk。
            previous_velocities = current_velocities
            previous_accelerations = current_accelerations
        previous = sample
    if risk_level == "green":
        replay_policy = "normal replay allowed"
    elif risk_level == "yellow":
        replay_policy = "safe retiming required before real replay"
    else:
        replay_policy = "real replay blocked; record a cleaner teach trajectory"
    return TeachTrajectoryQuality(
        risk_level=risk_level,
        replay_policy=replay_policy,
        allow_real_replay=risk_level != "red",
        requires_safe_retiming=risk_level == "yellow",
        max_jump_rad=max_jump,
        max_velocity_rad_s=max_velocity,
        max_acceleration_rad_s2=max_acceleration,
        max_jerk_rad_s3=max_jerk,
        worst_joint=worst_joint,
        worst_sample=worst_sample,
        anomalies=tuple(anomalies),
        events=tuple(events),
        green_jump_rad=float(green_jump_rad),
        yellow_jump_rad=float(yellow_jump_rad),
        velocity_limit_rad_s=max(velocity_limits, default=0.0),
        acceleration_limit_rad_s2=float(max_acceleration_rad_s2),
        jerk_limit_rad_s3=float(max_jerk_rad_s3),
    )


def teach_trajectory_quality_to_dict(quality: TeachTrajectoryQuality) -> dict:
    """把质量评估转成可直接 JSON 序列化的字典（字段名即对外 payload 键，不可改动）。"""
    return {
        "risk_level": quality.risk_level,
        "replay_policy": quality.replay_policy,
        "allow_real_replay": quality.allow_real_replay,
        "requires_safe_retiming": quality.requires_safe_retiming,
        "max_jump_rad": quality.max_jump_rad,
        "max_velocity_rad_s": quality.max_velocity_rad_s,
        "max_acceleration_rad_s2": quality.max_acceleration_rad_s2,
        "worst_joint": quality.worst_joint,
        "worst_sample": quality.worst_sample,
        "anomalies": list(quality.anomalies),
        "events": [
            {
                "sample": event.sample,
                "joint_name": event.joint_name,
                "level": event.level,
                "message": event.message,
                "delta_rad": event.delta_rad,
                "velocity_rad_s": event.velocity_rad_s,
                "acceleration_rad_s2": event.acceleration_rad_s2,
                "jerk_rad_s3": event.jerk_rad_s3,
            }
            for event in quality.events
        ],
        "green_jump_rad": quality.green_jump_rad,
        "yellow_jump_rad": quality.yellow_jump_rad,
        "velocity_limit_rad_s": quality.velocity_limit_rad_s,
        "acceleration_limit_rad_s2": quality.acceleration_limit_rad_s2,
        "max_jerk_rad_s3": quality.max_jerk_rad_s3,
        "jerk_limit_rad_s3": quality.jerk_limit_rad_s3,
    }


def retime_teach_samples(
    samples: list[TeachSample],
    *,
    replay_speed: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
    initial_delay_sec: float = 0.2,
    boundary_zero_velocity: bool = True,
) -> list[RetimedTeachPoint]:
    """保留示教位置点、只重排时间轴，使速度/加速度/加加速度满足上限（本模块的兜底重定时）。

    做法：按原时间间隔除以 ``replay_speed`` 得到候选 dt，再保证 dt 不小于"按关节速度上限
    所需的最短时间"（各关节 ``|Δq| / 速度上限`` 的最大值），然后最多迭代 24 次、每次把 dt
    放大 1.25 倍，直到加速度与 jerk 同时不越界为止；因此它只放慢、绝不加速，是保守的安全
    重定时。位置点本身不做改动，只在时间上被拉开。
    参数单位：``replay_speed`` 倍速（下限 0.01）、``max_velocity_rad_s`` rad/s（可为逐关节
    字典/列表）、``max_acceleration_rad_s2`` rad/s^2、``max_jerk_rad_s3`` rad/s^3、
    ``initial_delay_sec`` 首点相对回放起点的延时（秒）。
    ``boundary_zero_velocity=True`` 时把首点速度强制为 0，并同样用"放大 1.25 倍、最多 24 次"
    的方式推迟末点时间，使末点速度也为 0——保证起停无速度冲击（否则末点残留速度会让
    控制器在轨迹结束时急停）。
    关节名或位置向量长度不一致时抛 ``ValueError``（结构性异常，重定时无法修复）。
    """
    if not samples:
        return []
    expected_joints = samples[0].joint_names
    expected_len = len(expected_joints)
    for index, sample in enumerate(samples):
        if sample.joint_names != expected_joints:
            raise ValueError(f"joint_names mismatch at sample {index}")
        if len(sample.positions) != expected_len:
            raise ValueError(f"positions length mismatch at sample {index}")
    speed = max(float(replay_speed), 0.01)
    velocity_limits = _velocity_limits_for_joints(max_velocity_rad_s, expected_joints)
    acceleration_limit = max(float(max_acceleration_rad_s2), 0.01)
    jerk_limit = max(float(max_jerk_rad_s3), 0.01)
    zero_velocity = tuple(0.0 for _ in samples[0].positions)
    retimed = [
        RetimedTeachPoint(
            time_from_start=max(float(initial_delay_sec), 0.0),
            positions=tuple(float(v) for v in samples[0].positions),
            source_sample=0,
            velocities=zero_velocity,
        )
    ]
    previous = samples[0]
    previous_velocity = zero_velocity
    previous_acceleration = zero_velocity
    elapsed = retimed[0].time_from_start
    for index, sample in enumerate(samples[1:], start=1):
        # 倍速只作用于原始时间间隔；真正的下限由速度上限和下面的加速度/jerk 迭代决定。
        recorded_dt = max(0.0, float(sample.stamp) - float(previous.stamp)) / speed
        min_velocity_dt = max(
            (
                abs(float(current) - float(last)) / velocity_limit
                for current, last, velocity_limit in zip(sample.positions, previous.positions, velocity_limits)
            ),
            default=0.0,
        )
        # 三段合取：原始(倍速后)间隔、速度上限允许的最短时间、1 ms 硬下限（防止零时长点）。
        dt = max(recorded_dt, min_velocity_dt, 0.001)
        delta = tuple(
            float(current) - float(last)
            for current, last in zip(sample.positions, previous.positions)
        )
        # 放慢迭代：最多 24 轮、每轮 ×1.25（约可放大 100 倍），仍未收敛就接受当前 dt，
        # 避免在极端数据上无限循环。
        for _ in range(24):
            velocity = tuple(value / dt for value in delta)
            acceleration = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(velocity, previous_velocity)
            )
            max_acceleration = max((abs(value) for value in acceleration), default=0.0)
            max_jerk = max(
                (
                    abs(float(current) - float(last)) / dt
                    for current, last in zip(acceleration, previous_acceleration)
                ),
                default=0.0,
            )
            # 加 1e-12 容差，避免浮点误差导致刚满足上限时再多迭代一轮。
            if (
                max_acceleration <= acceleration_limit + 1e-12
                and max_jerk <= jerk_limit + 1e-12
            ):
                break
            dt *= 1.25
        velocity = tuple(value / dt for value in delta)
        acceleration = tuple(
            (float(current) - float(last)) / dt
            for current, last in zip(velocity, previous_velocity)
        )
        elapsed += dt
        retimed.append(
            RetimedTeachPoint(
                time_from_start=elapsed,
                positions=tuple(float(v) for v in sample.positions),
                source_sample=index,
                velocities=velocity,
            )
        )
        previous = sample
        previous_velocity = velocity
        previous_acceleration = acceleration
    if boundary_zero_velocity and retimed:
        # 首点速度归零：起点是保持段，直接置零不会引入误差。
        retimed[0] = RetimedTeachPoint(
            time_from_start=retimed[0].time_from_start,
            positions=retimed[0].positions,
            source_sample=retimed[0].source_sample,
            velocities=zero_velocity,
        )
        if len(retimed) > 1:
            previous_point = retimed[-2]
            last_point = retimed[-1]
            prior_acceleration = zero_velocity
            if len(retimed) > 2:
                # 用倒数第三点估算末点之前的加速度，作为 jerk 迭代的比较基准。
                before_previous = retimed[-3]
                prior_dt = max(previous_point.time_from_start - before_previous.time_from_start, 1e-9)
                prior_acceleration = tuple(
                    (float(current) - float(last)) / prior_dt
                    for current, last in zip(previous_point.velocities, before_previous.velocities)
                )
            current_dt = last_point.time_from_start - previous_point.time_from_start
            adjusted_dt = max(current_dt, 0.001)
            # 末点速度从 v 拉到 0 会带来 v/dt 的减速度：若越限就继续拉长末段（×1.25，最多 24 轮）。
            for _ in range(24):
                final_acceleration = tuple(
                    (0.0 - float(value)) / adjusted_dt
                    for value in previous_point.velocities
                )
                max_acceleration = max((abs(value) for value in final_acceleration), default=0.0)
                max_jerk = max(
                    (
                        abs(float(current) - float(last)) / adjusted_dt
                        for current, last in zip(final_acceleration, prior_acceleration)
                    ),
                    default=0.0,
                )
                if (
                    max_acceleration <= acceleration_limit + 1e-12
                    and max_jerk <= jerk_limit + 1e-12
                ):
                    break
                adjusted_dt *= 1.25
            adjusted_time = previous_point.time_from_start + adjusted_dt
            retimed[-1] = RetimedTeachPoint(
                time_from_start=adjusted_time,
                positions=last_point.positions,
                source_sample=last_point.source_sample,
                velocities=tuple(0.0 for _ in last_point.positions),
            )
    return retimed


def lowpass_filter_teach_samples(
    samples: list[TeachSample],
    *,
    sample_rate_hz: float,
    cutoff_hz: float,
    preserve_start_end: bool = True,
) -> list[TeachSample]:
    """对关节位置做零相位一阶低通滤波，抑制示教拖动与编码器噪声（时间戳与速度/力矩原样保留）。

    实现是一阶 RC 低通：``alpha = dt / (RC + dt)``，``RC = 1 / (2π·截止频率)``，
    ``dt = 1 / 采样率``。先正向滤波一遍、再反向滤波一遍（filtfilt），使整体相位延迟相互
    抵消——否则滤波后的轨迹会相对原始记录滞后，起点对齐与终点判定都会失真。
    参数：``sample_rate_hz`` 采样率（Hz，下限 1）、``cutoff_hz`` 截止频率（Hz，下限 0.01，
    越小越平滑但越容易削掉真实运动）、``preserve_start_end`` 是否强制首末点等于原始值
    （默认 True：起点决定回放起点误差，终点决定停位，不允许被滤波改动）。
    样本数 ≤ 2 时直接原样返回（无法滤波）。位置之外的所有字段都不修改。
    """
    if len(samples) <= 2:
        return list(samples)
    rate = max(float(sample_rate_hz), 1.0)
    cutoff = max(float(cutoff_hz), 0.01)
    dt = 1.0 / rate
    rc = 1.0 / (2.0 * math.pi * cutoff)
    alpha = dt / (rc + dt)
    filtered_positions: list[tuple[float, ...]] = [tuple(float(v) for v in samples[0].positions)]
    for sample in samples[1:]:
        previous = filtered_positions[-1]
        filtered_positions.append(
            tuple(
                float(last) + alpha * (float(current) - float(last))
                for current, last in zip(sample.positions, previous)
            )
        )
    # 反向再滤一遍：与正向结果级联得到零相位响应（代价是幅度被平方衰减）。
    backward = [filtered_positions[-1]]
    for positions in reversed(filtered_positions[:-1]):
        previous = backward[-1]
        backward.append(
            tuple(
                float(last) + alpha * (float(current) - float(last))
                for current, last in zip(positions, previous)
            )
        )
    filtered_positions = list(reversed(backward))
    if preserve_start_end:
        # 首末点还原为原始采样：起点/终点位置必须与记录严格一致。
        filtered_positions[0] = samples[0].positions
        filtered_positions[-1] = samples[-1].positions
    result: list[TeachSample] = []
    for sample, positions in zip(samples, filtered_positions):
        result.append(
            TeachSample(
                stamp=sample.stamp,
                joint_names=sample.joint_names,
                positions=tuple(float(v) for v in positions),
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=dict(sample.motor_status),
                arm_state=sample.arm_state,
            )
        )
    return result


def smooth_teach_samples(
    samples: list[TeachSample],
    *,
    window: int = 5,
    preserve_start_end: bool = True,
) -> list[TeachSample]:
    """对关节位置做滑动均值平滑（窗口不足时自动缩短，边缘不做补零）。

    ``window`` 为窗口长度（样本数），会被夹到至少 1 并**向上取整为奇数**，使窗口关于当前
    样本对称、不引入整体相位偏移；窗口半径 ``radius = width // 2``，首尾样本处窗口自动截断
    到有效范围后取平均（因此边缘平滑力度弱于中间，不会像补零那样把边缘拉向 0）。
    ``preserve_start_end=True`` 时首末帧原样保留。非位置字段不做任何修改。
    样本数 ≤ 2 时直接原样返回。
    """
    if len(samples) <= 2:
        return list(samples)
    width = max(int(window), 1)
    if width % 2 == 0:
        width += 1
    radius = width // 2
    smoothed: list[TeachSample] = []
    for index, sample in enumerate(samples):
        if preserve_start_end and index in (0, len(samples) - 1):
            smoothed.append(sample)
            continue
        start = max(0, index - radius)
        end = min(len(samples), index + radius + 1)
        segment = samples[start:end]
        positions = tuple(
            sum(float(item.positions[joint_index]) for item in segment) / float(len(segment))
            for joint_index in range(len(sample.positions))
        )
        smoothed.append(
            TeachSample(
                stamp=sample.stamp,
                joint_names=sample.joint_names,
                positions=positions,
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=dict(sample.motor_status),
                arm_state=sample.arm_state,
            )
        )
    return smoothed


def resample_teach_samples(
    samples: list[TeachSample],
    *,
    rate_hz: float = 50.0,
) -> list[TeachSample]:
    """把示教样本线性插值到等间隔时间网格上（重采样），保证下游可按固定周期执行。

    输出点数由记录时长与目标频率决定（``round(时长 / 周期) + 1``，至少 1 段），网格从首帧
    时间戳开始、按目标周期递增，网格点超出记录末尾时夹到末帧时间戳。
    查找区间时只向前推进 ``source_index``（样本时间单调，单次线性扫描即可），区间内按时间
    比例线性插值；区间退化时 ``alpha`` 取 0。
    输出样本的 ``velocities`` 与 ``efforts`` 被清空（插值后的速度/力矩不再可信，重定时会
    重新计算速度），``joint_names`` / ``motor_status`` / ``arm_state`` 沿用区间左端点。
    首末样本用原始对象整体替换，确保端点位置与状态标记严格等于原始记录。
    ``rate_hz`` 下限 1 Hz；样本数 ≤ 1 或时长非正时原样返回。
    """
    if len(samples) <= 1:
        return list(samples)
    rate = max(float(rate_hz), 1.0)
    period = 1.0 / rate
    start_stamp = float(samples[0].stamp)
    end_stamp = float(samples[-1].stamp)
    duration = max(0.0, end_stamp - start_stamp)
    if duration <= 0.0:
        return list(samples)
    result: list[TeachSample] = []
    source_index = 0
    count = max(int(round(duration / period)), 1)
    for output_index in range(count + 1):
        stamp = start_stamp + min(float(output_index) * period, duration)
        while source_index + 1 < len(samples) and float(samples[source_index + 1].stamp) < stamp:
            source_index += 1
        previous = samples[source_index]
        following = samples[min(source_index + 1, len(samples) - 1)]
        # 左右端点相同（区间退化）时直接取该点，不做除零。
        span = max(float(following.stamp) - float(previous.stamp), 1e-9)
        alpha = 0.0 if following is previous else (stamp - float(previous.stamp)) / span
        alpha = min(max(alpha, 0.0), 1.0)
        positions = tuple(
            float(a) + (float(b) - float(a)) * alpha
            for a, b in zip(previous.positions, following.positions)
        )
        result.append(
            TeachSample(
                stamp=stamp,
                joint_names=previous.joint_names,
                positions=positions,
                velocities=(),
                efforts=(),
                motor_status=dict(previous.motor_status),
                arm_state=previous.arm_state,
            )
        )
    result[0] = samples[0]
    result[-1] = samples[-1]
    return result


def _has_structural_teach_anomaly(quality: TeachTrajectoryQuality) -> bool:
    """判断质量结果里是否含结构性异常（关节名不一致 / 位置长度不一致 / 时间戳非单调）。

    这类异常无法靠重定时修复，因此预处理流水线据此跳过重定时；通过匹配英文异常文案实现，
    文案由 :func:`analyze_teach_trajectory` 生成，两处必须同步修改。
    """
    return any(
        "joint_names mismatch" in item
        or "positions length mismatch" in item
        or "timestamp not monotonic" in item
        for item in quality.anomalies
    )


def prepare_teach_replay_samples(
    samples: list[TeachSample],
    *,
    smoothing_enabled: bool = True,
    smoothing_window: int = 7,
    filter_enabled: bool = True,
    filter_cutoff_hz: float = 5.0,
    filter_sample_rate_hz: float = 50.0,
    resample_enabled: bool = True,
    resample_rate_hz: float = 100.0,
    retime_enabled: bool = False,
    replay_speed: float = 1.0,
    max_velocity_rad_s: float = 1.5,
    max_acceleration_rad_s2: float = 5.0,
    max_jerk_rad_s3: float = 20.0,
    time_parameterization_method: str = "auto",
    large_motion_span_rad: float = 0.8,
    large_motion_total_rad: float = 2.5,
    large_motion_max_speed: float = 1.0,
) -> PreparedTeachReplay:
    """生成真实回放要执行的预处理轨迹：原始 → 平滑 → 低通滤波 → 重采样 → 重定时。

    流水线顺序固定，每一步都受开关控制，且每一步之后/之前都会重新做一次质量评估，最终返回
    :class:`PreparedTeachReplay`（含三段质量、各步骤是否生效、实际参数与大幅度运动信息）。
    参数含义与单位：
      ``smoothing_enabled`` / ``smoothing_window``   滑动均值平滑开关与窗口长度（样本数，奇数）；
      ``filter_enabled`` / ``filter_cutoff_hz``      零相位低通滤波开关与截止频率（Hz）；
      ``filter_sample_rate_hz``                      滤波假定的采样率（Hz，需与实际记录采样率相符，
                                                     否则 RC 系数失真）；
      ``resample_enabled`` / ``resample_rate_hz``    重采样开关与目标频率（Hz）；
      ``retime_enabled``                             是否做时间参数化；只有滤波后质量没有结构性
                                                     异常时才会执行；
      ``replay_speed``                               回放倍速，被夹到 [0.01, 1.0]（不允许超过 1 倍）；
      ``max_velocity_rad_s`` / ``max_acceleration_rad_s2`` / ``max_jerk_rad_s3``
                                                     速度/加速度/加加速度上限，同时用于质量评估
                                                     与重定时（rad/s、rad/s^2、rad/s^3）；
      ``time_parameterization_method``               时间参数化后端（``auto`` 等），实际用哪个由
                                                     运动包的时间参数化模块决定，并把结果记入
                                                     ``time_parameterization_*`` 字段；
      ``large_motion_span_rad`` / ``large_motion_total_rad``
                                                     大幅度运动判据：单关节行程或总运动量任一达到
                                                     阈值即视为大幅度运动（rad）；
      ``large_motion_max_speed``                     大幅度运动时的最大倍速（下限 0.01；当前实现
                                                     只回填该值，未用它改写 ``effective_speed``）。
    安全语义：真实回放决策必须基于 ``after_quality``（重定时后的质量，等于 ``retimed_quality``）；
    ``before_quality`` 只用于展示"处理前有多差"。
    """
    max_joint_span, total_joint_motion = _motion_scope(samples)
    # 回放倍速只允许减速（≤1.0），避免上层误配置出比示教更快的回放。
    requested_speed = min(max(float(replay_speed), 0.01), 1.0)
    large_motion = (
        max_joint_span >= float(large_motion_span_rad)
        or total_joint_motion >= float(large_motion_total_rad)
    )
    effective_speed = requested_speed
    raw_quality = analyze_teach_trajectory(
        samples,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    prepared = list(samples)
    smoothing_applied = False
    filter_applied = False
    resample_applied = False
    retime_applied = False
    if smoothing_enabled and prepared:
        prepared = smooth_teach_samples(prepared, window=smoothing_window)
        smoothing_applied = len(prepared) > 0
    if filter_enabled and len(prepared) > 2:
        # 只有超过 2 个样本才滤波：少于此数无明显噪声可滤，且 filtfilt 需要前后向都有数据。
        prepared = lowpass_filter_teach_samples(
            prepared,
            sample_rate_hz=filter_sample_rate_hz,
            cutoff_hz=filter_cutoff_hz,
        )
        filter_applied = len(prepared) > 0
    filtered_quality = analyze_teach_trajectory(
        prepared,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    if resample_enabled and len(prepared) > 1:
        prepared = resample_teach_samples(prepared, rate_hz=resample_rate_hz)
        # 以"样本数是否变化"判定重采样是否真的生效（等间隔记录可能点数不变）。
        resample_applied = len(prepared) != len(samples)
    retimed_points: list[RetimedTeachPoint] = []
    if retime_enabled and not _has_structural_teach_anomaly(filtered_quality):
        # 时间参数化后端由运动包提供；这里把兜底重定时函数作为回调传入，后端不可用时自动回退。
        time_parameterization = parameterize_teach_samples(
            prepared,
            method=time_parameterization_method,
            fallback_retime=retime_teach_samples,
            replay_speed=effective_speed,
            max_velocity_rad_s=max_velocity_rad_s,
            max_acceleration_rad_s2=max_acceleration_rad_s2,
            max_jerk_rad_s3=max_jerk_rad_s3,
            initial_delay_sec=0.0,
            boundary_zero_velocity=True,
        )
        retimed_points = time_parameterization.points
        retime_applied = len(retimed_points) > 0
    else:
        time_parameterization = None
    # 把重定点包装成样本以便复用统一的质量评估（efforts/motor_status 无意义故清空）。
    retimed_samples = [
        TeachSample(
            stamp=point.time_from_start,
            joint_names=prepared[0].joint_names if prepared else (),
            positions=point.positions,
            velocities=point.velocities,
            efforts=(),
            motor_status={},
            arm_state="RETIMED",
        )
        for point in retimed_points
    ]
    # 没有重定点时退回评估预处理样本本身，保证 after_quality 始终反映"将要执行的东西"。
    retimed_quality = analyze_teach_trajectory(
        retimed_samples if retimed_samples else prepared,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    return PreparedTeachReplay(
        samples=prepared,
        raw_quality=raw_quality,
        filtered_quality=filtered_quality,
        retimed_quality=retimed_quality,
        smoothing_applied=smoothing_applied,
        filter_applied=filter_applied,
        resample_applied=resample_applied,
        retime_applied=retime_applied,
        # 以下参数一律回填"夹紧之后的实际值"，与真正参与计算的值保持一致。
        smoothing_window=max(int(smoothing_window), 1),
        filter_cutoff_hz=max(float(filter_cutoff_hz), 0.01),
        filter_sample_rate_hz=max(float(filter_sample_rate_hz), 1.0),
        resample_rate_hz=max(float(resample_rate_hz), 1.0),
        retimed_points=retimed_points,
        large_motion=large_motion,
        max_joint_span_rad=max_joint_span,
        total_joint_motion_rad=total_joint_motion,
        requested_replay_speed=requested_speed,
        effective_replay_speed=effective_speed,
        large_motion_max_speed=max(float(large_motion_max_speed), 0.01),
        time_parameterization_requested_method=(
            time_parameterization.requested_method if time_parameterization is not None else str(time_parameterization_method or "auto")
        ),
        time_parameterization_used_method=(
            time_parameterization.used_method if time_parameterization is not None else "none"
        ),
        time_parameterization_message=(
            time_parameterization.message if time_parameterization is not None else "retime disabled"
        ),
    )


def prepared_teach_replay_to_dict(prepared: PreparedTeachReplay) -> dict:
    """把预处理结果转成 JSON 可序列化的状态 payload（键名为对外接口，不可改动）。

    只输出摘要与质量数据，不输出全部轨迹点（点数很大）；``prepared_samples`` /
    ``retimed_points`` 是数量而非数组。``before_quality`` 与 ``after_quality`` 是门控与实际
    判定的两个关键字段，``raw/filtered/retimed_quality`` 用于排查是哪一步导致了等级变化。
    """
    return {
        "smoothing_applied": prepared.smoothing_applied,
        "filter_applied": prepared.filter_applied,
        "resample_applied": prepared.resample_applied,
        "retime_applied": prepared.retime_applied,
        "smoothing_window": prepared.smoothing_window,
        "filter_cutoff_hz": prepared.filter_cutoff_hz,
        "filter_sample_rate_hz": prepared.filter_sample_rate_hz,
        "resample_rate_hz": prepared.resample_rate_hz,
        "prepared_samples": len(prepared.samples),
        "retimed_points": len(prepared.retimed_points),
        "time_parameterization": {
            "requested_method": prepared.time_parameterization_requested_method,
            "used_method": prepared.time_parameterization_used_method,
            "message": prepared.time_parameterization_message,
        },
        "before_quality": teach_trajectory_quality_to_dict(prepared.before_quality),
        "after_quality": teach_trajectory_quality_to_dict(prepared.after_quality),
        "raw_quality": teach_trajectory_quality_to_dict(prepared.raw_quality),
        "filtered_quality": teach_trajectory_quality_to_dict(prepared.filtered_quality),
        "retimed_quality": teach_trajectory_quality_to_dict(prepared.retimed_quality),
        "large_motion": {
            "enabled": prepared.large_motion,
            "max_joint_span_rad": prepared.max_joint_span_rad,
            "total_joint_motion_rad": prepared.total_joint_motion_rad,
            "requested_speed": prepared.requested_replay_speed,
            "effective_speed": prepared.effective_replay_speed,
            "large_motion_max_speed": prepared.large_motion_max_speed,
        },
    }


def teach_trajectory_preview_to_dict(
    samples: list[TeachSample],
    *,
    max_points: int = 500,
) -> dict:
    """把（可能很长的）示教轨迹压缩成给界面画曲线用的预览 payload。

    点数超过 ``max_points``（下限 1）时按 ``round(i * (n-1) / (limit-1))`` 均匀抽取，
    并用集合去重后排序，因此首末点一定包含、下标严格递增；实际使用的抽样步长以
    ``downsample_step`` 回填，便于界面说明"这是抽样后的曲线"。
    每个点的时间 ``t`` 是相对首帧的秒数（负数被夹到 0），位置按关节名成字典输出（rad）。
    ``sample`` 保留原始下标，便于把质量事件里的 ``sample`` 直接映射到曲线位置。
    样本为空时返回空结构（但仍带回一份 red 质量结果）。
    """
    quality = analyze_teach_trajectory(samples)
    if not samples:
        return {
            "joint_names": [],
            "raw_samples": 0,
            "returned_samples": 0,
            "downsample_step": 1,
            "duration_sec": 0.0,
            "quality": teach_trajectory_quality_to_dict(quality),
            "points": [],
            "events": [],
        }
    limit = max(int(max_points), 1)
    # 向上取整的采样步长，仅用于展示"原始点 → 抽样点"的稀疏程度。
    step = max(1, (len(samples) + limit - 1) // limit)
    first_stamp = float(samples[0].stamp)
    if len(samples) <= limit:
        selected_indices = list(range(len(samples)))
    elif limit == 1:
        selected_indices = [0]
    else:
        selected_indices = sorted(
            {
                round(index * (len(samples) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
    points = []
    for index in selected_indices:
        sample = samples[index]
        points.append(
            {
                "sample": index,
                "t": max(0.0, float(sample.stamp) - first_stamp),
                "positions": {
                    name: float(position)
                    for name, position in zip(sample.joint_names, sample.positions)
                },
                "arm_state": sample.arm_state,
            }
        )
    return {
        "joint_names": list(samples[0].joint_names),
        "raw_samples": len(samples),
        "returned_samples": len(points),
        "downsample_step": step,
        "duration_sec": max(0.0, float(samples[-1].stamp) - first_stamp),
        "quality": teach_trajectory_quality_to_dict(quality),
        "points": points,
        "events": [
            {
                "sample": event.sample,
                "joint_name": event.joint_name,
                "level": event.level,
                "message": event.message,
                "delta_rad": event.delta_rad,
                "velocity_rad_s": event.velocity_rad_s,
            }
            for event in quality.events
        ],
    }


def detect_teach_record_anomalies(
    samples: list[TeachSample],
    *,
    max_jump_rad: float = 0.75,
    max_velocity_rad_s: float = 2.0,
) -> tuple[str, ...]:
    """文件巡检用的轻量异常扫描：只看跳变、速度、关节名与时间戳单调性。

    与 :func:`analyze_teach_trajectory` 的区别是阈值更宽松（默认跳变 0.75 rad、速度
    2.0 rad/s）且只返回文本清单，用于"文件是否值得拿来用"的粗筛；质量分档以
    :func:`analyze_teach_trajectory` 的结果为准。
    """
    anomalies: list[str] = []
    if not samples:
        return ()
    expected_joints = samples[0].joint_names
    previous = samples[0]
    for index, sample in enumerate(samples[1:], start=1):
        if sample.joint_names != expected_joints:
            anomalies.append(f"joint_names mismatch at sample {index}")
        dt = float(sample.stamp) - float(previous.stamp)
        if dt <= 0.0:
            anomalies.append(f"timestamp not monotonic at sample {index}")
        for joint_name, current, last in zip(expected_joints, sample.positions, previous.positions):
            delta = abs(float(current) - float(last))
            if delta > float(max_jump_rad):
                anomalies.append(f"{joint_name} jump {delta:.4f} rad at sample {index}")
            if dt > 0.0:
                velocity = delta / dt
                if velocity > float(max_velocity_rad_s):
                    anomalies.append(f"{joint_name} velocity {velocity:.4f} rad/s at sample {index}")
        previous = sample
    return tuple(anomalies)


def inspect_teach_record(
    path: str | Path,
    *,
    current_positions: dict[str, float] | None = None,
    direct_threshold: float = 0.01,
    align_threshold: float = 0.25,
) -> TeachRecordInfo:
    """巡检一条示教记录文件：能否读取、轨迹质量如何、与当前位姿的起点误差多大。

    ``path``               记录文件（JSONL）；
    ``current_positions``  当前关节位置（rad，键为关节名）；为 None 时不做起点比较，
                           结果中 ``start_band`` 为 ``unknown``；
    ``direct_threshold`` / ``align_threshold`` 起点分档阈值（rad），透传给
                           :func:`classify_replay_start`。
    一切异常都转成结构化的"坏结果"而不是抛异常：
      文件不存在 → ``start_band="missing"``；解析失败 → ``"invalid"``（异常文本进 message
      与 anomalies）；没有样本 → ``"empty"``；缺少记录里的某些关节 → ``"unknown"`` 并在
      message 里列出缺失关节名。
    ``anomalies`` 是轻量巡检结果与质量评估异常的合并去重（``dict.fromkeys`` 保序去重）。
    """
    record_path = Path(path)
    if not record_path.exists():
        return TeachRecordInfo(
            path=str(record_path),
            exists=False,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="missing",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=(),
            message="record file does not exist",
        )
    try:
        samples = load_teach_samples(record_path)
    except Exception as exc:
        # 记录文件损坏时只报告，不猜测内容，也不让异常穿到 ROS 回调里。
        return TeachRecordInfo(
            path=str(record_path),
            exists=True,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="invalid",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=(f"invalid jsonl: {exc}",),
            message=f"failed to read record: {exc}",
        )
    if not samples:
        return TeachRecordInfo(
            path=str(record_path),
            exists=True,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="empty",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=("empty record",),
            message="record contains no samples",
        )
    first = samples[0]
    last = samples[-1]
    duration_sec = max(0.0, float(last.stamp) - float(first.stamp))
    per_joint_error: dict[str, float] = {}
    max_error: float | None = None
    worst_joint = ""
    start_band = "unknown"
    message = "current joint state unavailable"
    if current_positions is not None:
        # 记录里的关节必须都能在当前状态里找到，否则起点比较不可信 → unknown。
        missing = [name for name in first.joint_names if name not in current_positions]
        if missing:
            message = f"current joint state missing: {', '.join(missing)}"
        else:
            current = tuple(float(current_positions[name]) for name in first.joint_names)
            decision = classify_replay_start(
                current_positions=current,
                start_positions=first.positions,
                direct_threshold=direct_threshold,
                align_threshold=align_threshold,
            )
            per_joint_error = {
                name: float(error)
                for name, error in zip(first.joint_names, decision.per_joint_error)
            }
            max_error = float(decision.max_error)
            if per_joint_error:
                worst_joint = max(per_joint_error, key=per_joint_error.get)
            start_band = str(decision.band.value)
            message = decision.message
    quality = analyze_teach_trajectory(samples)
    return TeachRecordInfo(
        path=str(record_path),
        exists=True,
        samples=len(samples),
        duration_sec=duration_sec,
        joint_names=first.joint_names,
        start_positions=first.positions,
        end_positions=last.positions,
        start_band=start_band,
        max_error=max_error,
        worst_joint=worst_joint,
        per_joint_error=per_joint_error,
        anomalies=tuple(dict.fromkeys((*detect_teach_record_anomalies(samples), *quality.anomalies))),
        message=message,
        quality=quality,
    )


def teach_record_info_to_dict(info: TeachRecordInfo) -> dict:
    """把巡检结果转成 JSON 可序列化的字典（键名为对外 payload 接口，不可改动）。

    ``start_positions`` / ``end_positions`` 在这里从与关节名对齐的元组转成"关节名 → 位置"的
    字典（rad）；``quality`` 只在有质量结果时出现（文件缺失/损坏时没有该键）。
    """
    payload = {
        "path": info.path,
        "exists": info.exists,
        "samples": info.samples,
        "duration_sec": info.duration_sec,
        "joint_names": list(info.joint_names),
        "start_positions": {
            name: float(position)
            for name, position in zip(info.joint_names, info.start_positions)
        },
        "end_positions": {
            name: float(position)
            for name, position in zip(info.joint_names, info.end_positions)
        },
        "start_band": info.start_band,
        "max_error": info.max_error,
        "worst_joint": info.worst_joint,
        "per_joint_error": dict(info.per_joint_error),
        "anomalies": list(info.anomalies),
        "message": info.message,
    }
    if info.quality is not None:
        payload["quality"] = teach_trajectory_quality_to_dict(info.quality)
    return payload


def list_teach_record_files(directory: str | Path) -> list[dict]:
    """列出目录下的原始示教记录文件（``*.jsonl``），按文件名排序返回。

    文件名以 ``.prepared.jsonl`` 结尾的是 :func:`write_prepared_teach_record` 生成的派生
    文件，会被跳过，避免把预处理结果当成可再次回放的原始示教记录。
    返回项包含路径、文件名、大小（字节）、修改时间（Unix 秒）、样本数、时长（秒）、
    起点分档与异常清单；每条记录都会完整巡检一次，因此目录很大时本函数开销与文件大小成正比。
    目录不存在时返回空列表。
    """
    base = Path(directory)
    if not base.exists():
        return []
    records: list[dict] = []
    for path in sorted(base.glob("*.jsonl")):
        if path.name.endswith(".prepared.jsonl"):
            continue
        stat = path.stat()
        info = teach_record_info_to_dict(inspect_teach_record(path))
        records.append(
            {
                "path": str(path),
                "name": path.name,
                "size_bytes": int(stat.st_size),
                "modified_time": float(stat.st_mtime),
                "samples": int(info["samples"]),
                "duration_sec": float(info["duration_sec"]),
                "start_band": str(info["start_band"]),
                "anomalies": list(info["anomalies"]),
            }
        )
    return records


def validate_teach_dry_run_request(start_band: str) -> TeachDryRunDecision:
    """dry-run（空跑校验）请求的门控：文件巡检分档合法才接受。

    只有 ``direct`` / ``align`` / ``moveit_align`` 三档被接受（进入 ``dry_run`` 状态）；
    文件缺失、损坏、为空、起点误差过大（``reject``）等一律拒绝并返回 ``blocked``。
    dry-run 不驱动真实硬件，是真实回放的前置条件。分档字符串不区分大小写。
    """
    band = str(start_band or "").strip().lower()
    if band in (
        ReplayStartBand.DIRECT.value,
        ReplayStartBand.ALIGN.value,
        ReplayStartBand.MOVEIT_ALIGN.value,
    ):
        return TeachDryRunDecision(
            accepted=True,
            state="dry_run",
            message=f"dry-run accepted for {band} replay check",
        )
    return TeachDryRunDecision(
        accepted=False,
        state="blocked",
        message=f"dry-run blocked because file check is {band or 'unknown'}",
    )


def validate_teach_replay_execute_request(
    start_band: str,
    *,
    dry_run_passed: bool,
    risk_level: str = "green",
    prepared_risk_level: str | None = None,
    prepared_max_jump_rad: float | None = None,
    max_prepared_jump_rad: float = 0.02,
    retimed_max_acceleration_rad_s2: float | None = None,
    max_replay_acceleration_rad_s2: float = 5.0,
    retimed_max_jerk_rad_s3: float | None = None,
    max_replay_jerk_rad_s3: float = 20.0,
    replay_speed: float = 1.0,
    yellow_max_speed: float = 0.6,
) -> TeachDryRunDecision:
    """真实回放执行请求的门控（本模块最重要的一道安全闸门，条件按固定顺序短路判断）。

    参数：
      ``start_band``                    文件巡检分档，必须是 ``direct`` / ``align`` /
                                        ``moveit_align`` 之一；
      ``dry_run_passed``                是否已经成功空跑过；真实回放要求先 dry-run；
      ``risk_level``                    原始轨迹风险等级（``green``/``yellow``/``red``）；
      ``prepared_risk_level``           预处理后轨迹的风险等级；为 None 时退回用
                                        ``risk_level``（此时错误信息里不会出现"prepared"字样）；
      ``prepared_max_jump_rad``         预处理轨迹的最大单帧跳变（rad），None 表示不检查；
      ``max_prepared_jump_rad``         允许的预处理最大跳变（rad，默认 0.02）；
      ``retimed_max_acceleration_rad_s2`` / ``retimed_max_jerk_rad_s3``
                                        重定时后的实测最大加速度/加加速度（rad/s^2、rad/s^3），
                                        None 表示不检查；
      ``max_replay_acceleration_rad_s2`` / ``max_replay_jerk_rad_s3``
                                        对应的允许上限（默认 5.0、20.0）；
      ``replay_speed``                  请求的回放倍速；
      ``yellow_max_speed``              yellow 等级允许的最大倍速（默认 0.6）。
    判定顺序（先查最基础的分档，再查等级，再查数值，最后查 dry-run）：
      分档非法 → 拒绝；预处理/原始等级为 ``red`` → 拒绝；预处理跳变、重定时加速度、重定时
      加加速度超限 → 拒绝；``yellow`` 且倍速超过 ``yellow_max_speed`` → 拒绝；
      未通过 dry-run → 拒绝；全部通过才返回 ``replaying``。
    注意：所有阈值检查都是"None 即跳过"，因此调用方有责任把真实测量值传进来，否则该层保护
    不会生效。
    """
    band = str(start_band or "").strip().lower()
    raw_risk = str(risk_level or "unknown").strip().lower()
    risk = str(prepared_risk_level or raw_risk).strip().lower()
    if band not in (
        ReplayStartBand.DIRECT.value,
        ReplayStartBand.ALIGN.value,
        ReplayStartBand.MOVEIT_ALIGN.value,
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"real replay blocked because file check is {band or 'unknown'}",
        )
    if risk == "red":
        # 等级字段存在时优先采信预处理后的等级，错误信息据此区分措辞。
        quality_name = "prepared trajectory quality" if prepared_risk_level else "trajectory quality"
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"real replay blocked because {quality_name} is red",
        )
    if prepared_max_jump_rad is not None and float(prepared_max_jump_rad) > float(max_prepared_jump_rad):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because prepared max jump "
                f"{float(prepared_max_jump_rad):.4f} rad exceeds {float(max_prepared_jump_rad):.4f} rad"
            ),
        )
    if (
        retimed_max_jerk_rad_s3 is not None
        and float(retimed_max_jerk_rad_s3) > float(max_replay_jerk_rad_s3)
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because retimed max jerk "
                f"{float(retimed_max_jerk_rad_s3):.4f} rad/s^3 exceeds "
                f"{float(max_replay_jerk_rad_s3):.4f} rad/s^3"
            ),
        )
    if (
        retimed_max_acceleration_rad_s2 is not None
        and float(retimed_max_acceleration_rad_s2) > float(max_replay_acceleration_rad_s2)
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because retimed max acceleration "
                f"{float(retimed_max_acceleration_rad_s2):.4f} rad/s^2 exceeds "
                f"{float(max_replay_acceleration_rad_s2):.4f} rad/s^2"
            ),
        )
    if risk == "yellow" and float(replay_speed) > float(yellow_max_speed):
        # yellow 轨迹本身带瑕疵：即使用户请求 1.0 倍速也必须降到 yellow_max_speed 以内。
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"yellow replay speed must be <= {float(yellow_max_speed):.2f}",
        )
    if not dry_run_passed:
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message="real replay requires a successful dry-run first",
        )
    return TeachDryRunDecision(
        accepted=True,
        state="replaying",
        message=f"real replay accepted for {band} file check",
    )


def validate_teach_replay_stop_request(has_active_goal: bool) -> TeachDryRunDecision:
    """回放停止（取消）请求的门控：没有活动回放目标时返回 ``idle`` 且不接受。

    有活动目标时返回 ``cancel_requested``，由上层据此去取消动作目标；本函数只做判定，
    不执行取消动作，也不会自行下发停止指令。
    """
    if not has_active_goal:
        return TeachDryRunDecision(
            accepted=False,
            state="idle",
            message="no active teach replay goal",
        )
    return TeachDryRunDecision(
        accepted=True,
        state="cancel_requested",
        message="teach replay cancel requested",
    )


def normalize_teach_replay_settings(
    *,
    replay_speed: float,
    align_duration: float,
    align_steps: int,
    final_hold_sec: float = 1.0,
) -> dict[str, float | int]:
    """把用户请求的回放设置夹到安全范围内（返回可直接使用的设置字典）。

    夹取规则：``replay_speed`` → [0.1, 1.0]（只允许减速，且不能慢到 0.1 倍以下）；
    ``align_duration`` → [1.0, 10.0] 秒；``align_steps`` → [2, 200] 个点。
    ``final_hold_sec`` 是形参但**结果恒为 1.0**：回放结束保持时长被硬编码为 1 秒，
    传入值只用于保持接口兼容，不参与计算。
    """
    return {
        "replay_speed": min(max(float(replay_speed), 0.1), 1.0),
        "align_duration": min(max(float(align_duration), 1.0), 10.0),
        "align_steps": min(max(int(align_steps), 2), 200),
        "final_hold_sec": 1.0,
    }


def estimate_teach_replay(
    *,
    samples: int,
    record_duration_sec: float,
    start_band: str,
    replay_speed: float,
    align_duration: float,
    align_steps: int,
    final_hold_sec: float = 0.0,
) -> dict[str, float | int | bool]:
    """预估一次回放的时长与轨迹点数（用于界面提示，不参与安全判定）。

    参数：``samples`` 记录样本数；``record_duration_sec`` 记录时长（秒）；``start_band``
    分档（只有 ``align`` 档才会计入对齐时长与对齐点）；``replay_speed`` 倍速；
    ``align_duration`` / ``align_steps`` 对齐时长（秒）与对齐点数；``final_hold_sec``
    结束保持时长（秒）。
    计算：回放时长 = 记录时长 / 倍速（倍速下限 0.01 防止除零）；总时长 = 对齐时长 +
    回放时长 + 结束保持时长；总点数 = 样本数 + 对齐点（仅 ``align`` 档）+
    1 个结束保持点（仅在保持时长 > 0 且样本数 > 0 时计入）。
    返回的 ``replay_speed`` / ``align_duration`` / ``align_steps`` / ``final_hold_sec`` 都是
    :func:`normalize_teach_replay_settings` 夹紧之后的实际值；注意 ``final_hold_sec`` 被该
    函数硬编码为 1.0，因此总时长里的保持时长实际恒为 1 秒。
    """
    settings = normalize_teach_replay_settings(
        replay_speed=replay_speed,
        align_duration=align_duration,
        align_steps=align_steps,
        final_hold_sec=final_hold_sec,
    )
    speed = float(settings["replay_speed"])
    use_align = str(start_band or "").lower() == ReplayStartBand.ALIGN.value
    replay_duration = max(0.0, float(record_duration_sec)) / max(speed, 0.01)
    alignment_duration = float(settings["align_duration"]) if use_align else 0.0
    final_hold = float(settings["final_hold_sec"])
    return {
        "use_align": use_align,
        "alignment_duration_sec": alignment_duration,
        "estimated_duration_sec": alignment_duration + replay_duration + final_hold,
        "trajectory_points": max(int(samples), 0)
        + (int(settings["align_steps"]) if use_align else 0)
        + (1 if final_hold > 0.0 and int(samples) > 0 else 0),
        "replay_speed": speed,
        "align_duration": float(settings["align_duration"]),
        "align_steps": int(settings["align_steps"]),
        "final_hold_sec": final_hold,
    }

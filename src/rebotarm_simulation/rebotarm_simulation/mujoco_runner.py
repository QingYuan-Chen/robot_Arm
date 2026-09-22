"""离线 MuJoCo 物理体检套件：不依赖 ROS，直接加载 MJCF 模型跑正动力学。

职责与位置
    本模块属于仿真层的底层"体检"工具：给定一份 MJCF 模型文件，加载后推进若干物理
    步，把模型规模、数值有限性、接触数与步进结果整理成不可变的结果对象。上层
    （健康检查、模型配置校验、成对轨迹对照脚本、pytest）只读这些结果做判定，
    本模块自身不做阈值决策、不下发任何真实硬件指令。

四类检查
    1. :func:`run_smoke`           冒烟：只看模型能否加载并稳定步进（数值不发散）；
    2. :func:`run_step_response`   单关节阶跃响应：位置执行器 + 限力下的跟踪质量；
    3. :func:`run_step_response_suite` 把第 2 项按关节批量执行并汇总最坏值；
    4. :func:`run_grasp_benchmark` 抓取场景：接触与抬升判定（委托抓取质量模块）。

依赖与约束
    物理引擎与数值库在 :func:`_load_mujoco` 内延迟导入，因此缺少该可选依赖时本模块
    仍可被导入（调用方需自行跳过）；抓取质量判定复用同目录的质量模块；结果类型全部
    ``frozen=True``，保证证据一旦生成就不会被后续代码改写。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time

from .mujoco_grasp_quality import evaluate_grasp_quality


@dataclass(frozen=True)
class SmokeResult:
    """一次冒烟检查的输出。

    字段：
    - ``xml_path``：被检查 MJCF 文件的绝对路径（已 resolve）；
    - ``nq``：广义坐标维数（关节位置自由度，含夹爪与被抓物体）；
    - ``nv``：广义速度维数（自由度，等于 nq 减去被约束的坐标）；
    - ``nu``：执行器个数；
    - ``finite``：步进结束后 qpos/qvel 是否全部为有限数（NaN/Inf 说明数值发散）；
    - ``contacts``：步进结束瞬间的接触点数量；
    - ``sim_time``：步进结束时的仿真时间，单位 s。
    """

    xml_path: Path
    nq: int
    nv: int
    nu: int
    finite: bool
    contacts: int
    sim_time: float


@dataclass(frozen=True)
class StepResponseResult:
    """单个关节阶跃响应的输出。

    字段：
    - ``joint``：被测试的关节名（如 joint2）；
    - ``target``：阶跃目标角，单位 rad；
    - ``final_position``：仿真结束时的实际关节角，单位 rad；
    - ``final_abs_error``：结束时刻的绝对跟踪误差，单位 rad；
    - ``max_abs_error``：整个过程中出现的最大绝对跟踪误差，单位 rad；
    - ``rms_error``：全过程跟踪误差的均方根，单位 rad（既能反映偏差大小，
      又不会像最大值那样被单个毛刺完全主导）；
    - ``max_abs_velocity``：过程中关节角速度绝对值的峰值，单位 rad/s；
    - ``max_abs_actuator_force``：过程中执行器广义力的绝对值峰值，单位 N·m，
      用于确认限力没有失效；
    - ``sim_time``：仿真结束时间，单位 s。
    """

    joint: str
    target: float
    final_position: float
    final_abs_error: float
    max_abs_error: float
    rms_error: float
    max_abs_velocity: float
    max_abs_actuator_force: float
    sim_time: float


@dataclass(frozen=True)
class StepResponseSuiteResult:
    """多关节阶跃响应套件的汇总输出。

    字段：
    - ``xml_path``：被测模型的绝对路径；
    - ``results``：各关节单独的响应结果，顺序与调用方给出的目标字典一致；
    - 其余 ``max_*`` 字段：对全部子结果取最大值，即"最坏关节"的指标，
      供上层只用一个数就能与限值比较；目标字典为空时取 0.0。
    """

    xml_path: Path
    results: list[StepResponseResult]
    max_final_abs_error: float
    max_abs_error: float
    max_rms_error: float
    max_abs_velocity: float
    max_abs_actuator_force: float


@dataclass(frozen=True)
class GraspBenchmarkResult:
    """抓取场景基准检查的输出。

    字段：
    - ``xml_path``：场景 MJCF 的绝对路径；
    - ``finite``：结束时状态是否数值有限；
    - ``initial_box_height_m`` / ``box_height_m``：被测物体（body 名 ``box``）
      在初始关键帧与结束时刻的世界坐标 z，单位 m；模型中找不到该 body 时为 None；
    - ``max_contacts``：整个过程中出现过的最大接触点数（用于判断是否真的接触过）；
    - ``final_contacts``：结束瞬间的接触点数；
    - ``contact_detected`` / ``lift_detected`` / ``grasp_success``：抓取质量模块
      给出的判定（有接触、抬升超过阈值、两者同时成立）；
    - ``grasp_status``：判定结论字符串，取值与质量模块一致；
    - ``sim_time``：仿真结束时间，单位 s。
    """

    xml_path: Path
    finite: bool
    initial_box_height_m: float | None
    box_height_m: float | None
    max_contacts: int
    final_contacts: int
    contact_detected: bool
    lift_detected: bool
    grasp_success: bool
    grasp_status: str
    sim_time: float


def run_smoke(xml_path: Path, *, seconds: float = 3.0) -> SmokeResult:
    """对 MJCF 模型做冒烟步进：加载 -> 推进 -> 汇报模型规模与数值稳定性。

    不做任何控制输入，只让模型在自带初始状态下自由步进，因此适合快速回答
    "这份模型文件是否可用、有没有明显数值发散"。

    参数：
    - ``xml_path``：MJCF 文件路径，内部会 resolve 成绝对路径；
    - ``seconds``：仿真时长，单位 s，默认 3.0。

    返回 :class:`SmokeResult`。若模型文件缺失或解析失败，底层引擎会抛出异常，
    本函数不吞异常、也不做降级处理。
    """
    mujoco, np = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    # 时长换算成物理步数：按模型自带的时间步长取整，至少 1 步，避免 seconds 过小
    # 时出现"一步都不跑却报告步进成功"的空检查。
    steps = max(1, int(seconds / float(model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    return SmokeResult(
        xml_path=xml_path.resolve(),
        nq=int(model.nq),
        nv=int(model.nv),
        nu=int(model.nu),
        finite=finite,
        contacts=int(data.ncon),
        sim_time=float(data.time),
    )


def run_step_response(
    xml_path: Path,
    *,
    joint: str = "joint2",
    target: float = -0.6,
    seconds: float = 3.0,
) -> StepResponseResult:
    """对单个关节施加位置阶跃并统计跟踪质量（限力、有限时长）。

    流程：重置到名为 ``zero`` 的关键帧（模型没有该关键帧时退化为第 0 个关键帧，
    再退化为默认状态）并做一次前向计算，使重力等派生量生效；随后把目标角写进
    该关节位置执行器的控制量，逐步推进并记录误差、速度和执行器力。

    参数：
    - ``xml_path``：MJCF 文件路径；
    - ``joint``：目标关节名，默认 joint2，同时也是"关节未找到"报错里的名字；
    - ``target``：阶跃目标角，单位 rad，默认 -0.6，需要落在该关节的关节限位内，
      否则会被模型限位截断、末端误差不会收敛到 0；
    - ``seconds``：仿真时长，单位 s，默认 3.0。时长决定能否记录到稳态，太短会
      把尚未收敛的瞬态当成最终结果。

    返回 :class:`StepResponseResult`。关节不存在或该关节没有位置执行器时抛
    :class:`ValueError`。
    """
    mujoco, np = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    _reset_keyframe_if_present(mujoco, model, data, "zero")

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    if joint_id < 0:
        raise ValueError(f"joint not found in MuJoCo model: {joint}")
    qpos_index = int(model.jnt_qposadr[joint_id])
    actuator_id = _actuator_id_for_joint(mujoco, model, joint_id)
    if actuator_id is None:
        raise ValueError(f"position actuator not found for joint: {joint}")

    # 位置执行器的控制量就是目标角（单位与 qpos 一致，即 rad），由模型内部的位置
    # 伺服与限力参数产生实际力矩。
    data.ctrl[actuator_id] = float(target)
    # 与冒烟检查相同的步数换算；这里逐帧累计误差，因此 steps 也决定了统计样本量。
    steps = max(1, int(seconds / float(model.opt.timestep)))
    errors: list[float] = []
    max_abs_velocity = 0.0
    max_abs_force = 0.0
    for _ in range(steps):
        mujoco.mj_step(model, data)
        error = float(target) - float(data.qpos[qpos_index])
        errors.append(error)
        # 同一个关节在 qpos 与 qvel 中的存放位置不同：速度要按自由度地址 jnt_dofadr 取。
        dof_index = int(model.jnt_dofadr[joint_id])
        max_abs_velocity = max(max_abs_velocity, abs(float(data.qvel[dof_index])))
        max_abs_force = max(max_abs_force, abs(float(data.actuator_force[actuator_id])))

    rms_error = math.sqrt(sum(error * error for error in errors) / float(len(errors)))
    return StepResponseResult(
        joint=joint,
        target=float(target),
        final_position=float(data.qpos[qpos_index]),
        final_abs_error=abs(float(target) - float(data.qpos[qpos_index])),
        max_abs_error=max(abs(error) for error in errors),
        rms_error=float(rms_error),
        max_abs_velocity=float(max_abs_velocity),
        max_abs_actuator_force=float(max_abs_force),
        sim_time=float(data.time),
    )


def run_step_response_suite(
    xml_path: Path,
    *,
    targets: dict[str, float],
    seconds: float = 3.0,
) -> StepResponseSuiteResult:
    """按关节批量执行阶跃响应，并汇总各指标的"最坏值"。

    参数：
    - ``xml_path``：MJCF 文件路径；
    - ``targets``：关节名 -> 目标角（rad）的映射，遍历顺序（即字典插入顺序）就是
      执行顺序，也是结果列表顺序；空字典会得到空结果列表；
    - ``seconds``：每个关节各自的仿真时长，单位 s，默认 3.0。

    每个关节都从同一份模型文件重新加载并重置关键帧，因此各关节互不影响。返回
    :class:`StepResponseSuiteResult`，其中 ``max_*`` 字段用 ``default=0.0`` 覆盖
    空输入，保证不会对空序列调用 max()。
    """
    results = [
        run_step_response(xml_path, joint=joint, target=target, seconds=seconds)
        for joint, target in targets.items()
    ]
    return StepResponseSuiteResult(
        xml_path=xml_path,
        results=results,
        max_final_abs_error=max((result.final_abs_error for result in results), default=0.0),
        max_abs_error=max((result.max_abs_error for result in results), default=0.0),
        max_rms_error=max((result.rms_error for result in results), default=0.0),
        max_abs_velocity=max((result.max_abs_velocity for result in results), default=0.0),
        max_abs_actuator_force=max((result.max_abs_actuator_force for result in results), default=0.0),
    )


def run_grasp_benchmark(xml_path: Path, *, seconds: float = 5.0) -> GraspBenchmarkResult:
    """在抓取场景上自由步进，统计接触与抬升，形成抓取成功率证据。

    流程：重置到关键帧 ``"0"``（注意这里是字符串 ``"0"``，与阶跃检查使用的 ``zero``
    不同；模型没有该关键帧时会退化为第 0 个关键帧或默认状态），记录被夹物体初始
    高度，然后自由步进并跟踪接触数峰值，最后交给抓取质量模块判定。

    参数：
    - ``xml_path``：包含机械臂、夹爪与被夹物体的场景 MJCF；
    - ``seconds``：仿真时长，单位 s，默认 5.0。抓取过程比单关节阶跃慢，时长过短
      会来不及产生抬升，从而把成功抓取误判成失败。

    返回 :class:`GraspBenchmarkResult`。注意本函数只做仿真判定，不代表真实硬件
    抓取能力；物体 body 名固定为 ``box``，缺失时高度字段为 None。
    """
    mujoco, np = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    _reset_keyframe_if_present(mujoco, model, data, "0")

    initial_box_z = _body_z_position(mujoco, model, data, "box")
    max_contacts = 0
    # 接触数在抓取过程中是脉冲式的：判定用过程峰值，而不是结束瞬间的瞬时值。
    steps = max(1, int(seconds / float(model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
        max_contacts = max(max_contacts, int(data.ncon))

    box_z = _body_z_position(mujoco, model, data, "box")
    quality = evaluate_grasp_quality(
        contact_count=max_contacts,
        initial_box_height_m=initial_box_z,
        final_box_height_m=box_z,
    )
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    return GraspBenchmarkResult(
        xml_path=xml_path.resolve(),
        finite=finite,
        initial_box_height_m=initial_box_z,
        box_height_m=box_z,
        max_contacts=max_contacts,
        final_contacts=int(data.ncon),
        contact_detected=quality.contact_detected,
        lift_detected=quality.lift_detected,
        grasp_success=quality.success,
        grasp_status=quality.status,
        sim_time=float(data.time),
    )


def _load_mujoco():
    """延迟导入物理引擎与数值库，返回 (mujoco, numpy) 二元组。

    放在函数内导入是为了让本模块在未安装该可选依赖时仍可被导入（测试据此跳过），
    也避免 ROS 节点导入本模块时立刻付出加载重库的代价。
    """
    import mujoco
    import numpy as np

    return mujoco, np


def _reset_keyframe_if_present(mujoco, model, data, name: str) -> None:
    """按名字重置到关键帧，找不到时逐级降级，最后做一次前向计算。

    降级顺序：指定名字的关键帧 -> 第 0 个关键帧（模型存在关键帧但名字不匹配）
    -> 引擎默认初始状态。重置后必须调用 mj_forward 刷新派生量（接触、传感器等），
    否则首次读取的派生数据仍是重置前的。
    """
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    elif int(getattr(model, "nkey", 0)) > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)


def _actuator_id_for_joint(mujoco, model, joint_id: int) -> int | None:
    """返回直接驱动指定关节的执行器下标，找不到时返回 None。

    判定条件是执行器的传动类型为"关节"且传动对象正是该关节；用遍历而不是假设
    "执行器顺序等于关节顺序"，这样模型中混入夹爪执行器或额外传动时也不会错位。
    """
    for actuator_id in range(int(model.nu)):
        if (
            int(model.actuator_trntype[actuator_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[actuator_id, 0]) == int(joint_id)
        ):
            return actuator_id
    return None


def _body_z_position(mujoco, model, data, body_name: str) -> float | None:
    """返回指定 body 在世界坐标系中的 z 高度，单位 m；body 不存在时返回 None。"""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return None
    return float(data.xpos[body_id][2])


def benchmark_wall_time(command) -> tuple[float, object]:
    """执行无参可调用对象并返回 (真实耗时秒数, 其返回值)。

    用单调时钟测量墙钟时间，专门用来把"仿真时长"与"实际算力开销"区分开；
    被调用对象的异常照常向上抛出，不做捕获。
    """
    start = time.monotonic()
    result = command()
    return time.monotonic() - start, result

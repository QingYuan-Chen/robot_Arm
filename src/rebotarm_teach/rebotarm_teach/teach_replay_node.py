"""示教回放节点：把示教记录预处理后回放到真实机械臂。

职责与在系统中的位置
--------------------
本节点是示教链路的“执行端”。它读取示教录制阶段落盘的原始记录（JSONL），先做质量
分析，再做平滑 / 低通滤波 / 重采样 / 重定时等预处理并写出“预处理记录”，最后把
预处理后的轨迹交给底层关节轨迹控制器回放。**原始示教数据只当作输入数据**，绝不
直接用于执行。

对外接口（``arm_namespace`` 默认取 "rebotarm"，下列名称中的该段可被参数替换）
----------------------------------------------------------------------------
- 订阅 ``/{arm_namespace}/joint_states``：关节位置/速度反馈，BEST_EFFORT +
  KEEP_LAST，用于起点误差判定与运行期跟踪监控。
- 发布 ``/{arm_namespace}/teleop/replay_status``：JSON 文本状态；RELIABLE +
  TRANSIENT_LOCAL，保证上层状态面板晚于本节点启动时也能收到最后一次状态。
- 动作客户端 ``/{arm_namespace}/follow_joint_trajectory``：回放轨迹的唯一执行出口。
- 服务客户端 ``/{arm_namespace}/trajectory_stop``：停止时请求控制器中断当前轨迹。
- 服务客户端 ``collision_check_service``（默认 ``/check_state_validity``）：轨迹碰撞预检。
- 规划服务 ``moveit_planning_service``（默认 ``/plan_kinematic_path``）：起点对齐规划。

关键流程（一次性入口 :meth:`TeachReplayNode._maybe_start`）
----------------------------------------------------------
1. 等第一帧 joint_states，再加载示教记录；任一前置条件不满足即发布 ``failed``。
2. 起点分类：按最大关节误差落入 ``direct`` / ``align`` / ``reject`` 三个区间；``reject``
   且允许起点对齐时升级为 ``moveit_align``。
3. 原始记录质量分析 → 预处理（平滑/滤波/重采样/重定时）→ 写预处理记录。
4. 真实执行门控：风险等级、预处理后跳变/加速度/加加速度、黄色风险限速。
5. 组装完整轨迹（起点对齐段 + 预处理重定时段 + 末尾保持段）。
6. 碰撞预检。
7. ``dry_run`` 为真时只上报校验结果；否则发送动作目标并等待结果。
8. 回放期间由定时器监控跟踪误差与实时速度，超差即请求停止。

安全约束
--------
1. ``dry_run`` 默认为真：只有在下发端显式关闭后才会真正驱动机械臂。
2. 预处理后质量判定 ``allow_real_replay`` 为假时，真实回放被直接拒绝。
3. 碰撞预检结果为 ``collision`` 或 ``unknown`` 时禁止真实回放（``disabled`` 不拦截）。
4. 运行期监控在起步宽限期之后生效，连续超差超过违约宽限期即停控制器并取消目标。
"""

from __future__ import annotations

import json
import signal
import time
from contextlib import suppress
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .parameter_helpers import sensor_qos_kwargs
from .teach_recording import (
    ReplayStartBand,
    analyze_teach_trajectory,
    build_replay_start_soft_points,
    classify_replay_start,
    compute_auto_align_duration,
    load_teach_samples,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    retime_teach_samples,
    teach_trajectory_quality_to_dict,
    write_prepared_teach_record,
)
from rebotarm_motion.trajectory_safety_monitor import evaluate_replay_tracking
from rebotarm_motion.moveit_planner import MoveItMotionPlanner


def _set_duration(duration_msg, seconds: float) -> None:
    """把浮点秒数写入 ROS 的 Duration 消息。

    时间消息由整秒 ``sec`` 与纳秒 ``nanosec`` 两个整数字段组成，这里直接截断取整秒，
    再把小数部分换算成纳秒（1e9 ns/s）。调用方只传入非负的累计时间，因此不需要处理
    借位；本文件所有轨迹点的时间戳都是“相对轨迹起点”的累计秒。
    """
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


def _select_collision_points(points: list[tuple[float, ...]], *, max_samples: int) -> list[tuple[int, tuple[float, ...]]]:
    """从整条轨迹中均匀抽取至多 ``max_samples`` 个点用于碰撞预检。

    返回 ``(原始轨迹点索引, 关节位置)`` 列表：索引用于在状态里报告碰撞发生在第几个
    轨迹点，位置向量用于构造状态有效性请求。

    采样公式 ``round(i * (n - 1) / (limit - 1))`` 保证首尾点一定被选中（``i=0`` 得 0、
    ``i=limit-1`` 得 ``n-1``），因为起点与终点是回放中最关键的两个位形；同时用集合
    去重后排序，保证索引唯一且严格升序。

    边界处理：``points`` 为空返回空列表；``max_samples`` 小于 1 时按下限 1 处理；
    点总数不超过上限时全量返回（不再抽样）；上限为 1 时只取第一个点。
    """
    if not points:
        return []
    limit = max(int(max_samples), 1)
    if len(points) <= limit:
        return list(enumerate(points))
    if limit == 1:
        return [(0, points[0])]
    indices = sorted(
        {
            round(index * (len(points) - 1) / (limit - 1))
            for index in range(limit)
        }
    )
    return [(index, points[index]) for index in indices]


class TeachReplayNode(Node):
    """示教回放节点。

    生命期：节点启动后声明参数、创建动作/服务客户端、发布器、订阅与两个定时器，随后
    只回放一次——:meth:`_maybe_start` 由定时器反复调用，但内部 ``self._started`` 是一次
    性门闩：一旦进入启动流程（无论最终是回放、拒绝还是失败）都不会再触发第二次回放。
    要换记录文件回放必须重启节点或由上层重新拉起。

    回调模型：订阅回调、定时器回调、动作结果回调都跑在同一个 executor 线程上，因此对
    实例状态的读写无需加锁；代价是任何回调都不能长时间阻塞。例外是两处同步等待：
    :meth:`_request_controller_trajectory_stop`（会被运行期监控定时器间接调用，等待无参
    停止服务返回）与 :meth:`cancel_active_goal`（由 ``main`` 的停止路径调用，等待取消
    future），二者都会在回调内自旋 executor，因此只能在不会造成重入的调用点使用。

    状态机（通过 ``teleop/replay_status`` 的 ``state`` 字段对外表达，取值见各
    :meth:`_publish_status` 调用点）：``ready`` → ``waiting`` →（``failed`` /
    ``rejected`` / ``blocked`` / ``dry_run``）或 ``replaying`` →（``done`` /
    ``canceled`` / ``failed`` / ``safety_stop``）；``cancel_requested`` 是收到停止请求
    后的过渡状态。

    安全语义：``dry_run`` 为真时只做校验与状态上报，绝不发送动作目标；真实回放必须
    依次通过风险等级、预处理后跳变/加速度/加加速度、黄色风险限速与碰撞预检四项门控。
    """

    def __init__(self) -> None:
        # 参数含义（声明顺序与下方 declare_parameter 一致；标量单位见括号）：
        # 名称空间与数据源
        #   arm_namespace：话题/服务/动作名中的机械臂命名空间段，去首尾斜杠后拼成
        #     /{arm_namespace}/...；留空表示直接使用顶层名称。
        #   record_path：原始示教记录 JSONL 路径（相对路径按节点工作目录解析），
        #     预处理记录会写到同目录下的派生文件名。
        #   dry_run（bool）：默认 True，只校验不执行；真实驱动机械臂必须显式置 False。
        # 速度与起点分类
        #   speed（倍率）：回放速度倍率，无量纲；预处理内部夹到 [0.01, 1.0]，
        #     因此大于 1.0 不会加速，只会按 1.0 处理。
        #   direct_threshold（rad）：起点分类下界，最大关节误差 < 该值判为 direct，
        #     直接回放不做对齐。
        #   align_threshold（rad）：起点分类上界，误差在 [direct, align) 判为 align
        #     （关节空间插值对齐）；≥ 该值判为 reject。
        #   align_duration（s）：固定对齐时长，仅当 align_duration_auto 为假时使用。
        #   align_duration_auto（bool）：为真时按误差与目标速度自动计算对齐时长。
        #   align_target_speed_rad_s（rad/s）：自动对齐的期望关节速度，内部下限 0.01。
        #   align_min_duration / align_max_duration（s）：自动对齐时长的下限/上限，
        #     误差很小或很大时对齐时长都被夹在该区间内。
        #   align_steps（点数）：align 带的插值点数，内部至少 2 点。
        # 轨迹质量阈值
        #   green_jump_rad（rad）：质量分析判为“绿色”的相邻样本最大跳变。
        #   yellow_jump_rad（rad）：判为“黄色”的阈值；跳变越大风险等级越高。
        #   yellow_max_speed（倍率）：黄色风险允许的最大回放速度，真实回放时预处理后
        #     的有效速度超过该值即拒绝。
        # 动力学上限（同时用于质量分析与重定时）
        #   max_replay_velocity_rad_s（rad/s）：关节速度上限标量，仅在逐关节列表长度
        #     与记录的关节数不匹配时作为回退值。
        #   max_replay_velocity_rad_s_by_joint（rad/s，列表）：逐关节速度上限，顺序与
        #     记录中的关节顺序一致；默认前 3 个大关节 3.0、后 3 个腕关节 1.8。
        #   max_replay_acceleration_rad_s2（rad/s²）：关节加速度上限。
        #   max_replay_jerk_rad_s3（rad/s³）：关节加加速度上限。
        # 大范围运动判定
        #   large_motion_span_rad（rad）：单关节最大行程阈值，超过即判定为大范围运动。
        #   large_motion_total_rad（rad）：各关节累计行程阈值，超过即判定为大范围运动。
        #   large_motion_max_speed（倍率）：大范围运动推荐速度上限，随预处理结果与状态
        #     payload 上报；本文件内不再据此二次限速（限速见 yellow_max_speed 分支）。
        # 起停保持段
        #   start_hold_sec（s）：轨迹开头保持“当前位形”的时长。
        #   soft_start_duration（s）：起点判为 direct 时软启动插值的时长。
        #   soft_start_steps（点数）：软启动插值点数。
        #   first_hold_sec（s）：到达记录首点后的保持时长，两种起点对齐路径都会追加。
        #   final_hold_sec（s）：轨迹末尾保持末点位置的时长，0 表示不追加。
        #   initial_replay_delay_sec（s）：预处理轨迹整体向后平移的起始延时，用于等待
        #     控制器进入跟踪状态。
        # 起点对齐（改用规划服务）
        #   use_moveit_start_align（bool）：为真时起点对齐走规划服务（moveit_align），
        #     为假时退回关节空间插值。
        #   moveit_start_skip_threshold（rad）：当前位形与首点误差小于该值时跳过规划，
        #     只保留保持点。
        #   moveit_group_name：规划用的规划组名。
        #   collision_group_name：碰撞预检用的规划组名；等于 "arm_with_gripper" 时
        #     会自动补夹爪关节（见 :meth:`_with_collision_default_joints`）。
        #   moveit_planning_service / moveit_planning_pipeline / moveit_planner_id：
        #     规划服务名、流水线 id（默认 "ompl"）、规划器 id（空串表示由 MoveIt 选默认）。
        #   moveit_planning_time（s）/ moveit_num_planning_attempts（次）：单次规划允许
        #     时间与尝试次数。
        #   moveit_joint_goal_tolerance（rad）：关节目标容差。
        #   moveit_velocity_scaling / moveit_acceleration_scaling（无量纲，(0,1]）：
        #     规划请求的速度/加速度缩放系数，越小越保守。
        # 碰撞预检
        #   collision_check_enabled（bool）：为假时预检直接返回 disabled，不拦截真实回放。
        #   collision_check_service：状态有效性服务名。
        #   collision_check_max_samples（点数）：最多抽样的轨迹点数，内部至少 1。
        #   collision_check_timeout_sec（s）：预检总超时，内部下限 0.1。
        # 预处理
        #   smoothing_enabled（bool）/ smoothing_window（点数）：是否做滑动平均平滑及窗口长度。
        #   filter_enabled（bool）/ filter_cutoff_hz（Hz）/ filter_sample_rate_hz（Hz）：
        #     是否低通滤波、截止频率、滤波使用的采样率；截止频率必须低于采样率的一半。
        #   resample_enabled（bool）/ resample_rate_hz（Hz）：是否重采样及目标采样率。
        #   time_parameterization_method：重定时方法选择，取值 "auto" /
        #     "current_jerk_retime" / "ruckig"，非法值回落为 "auto"。
        #   max_prepared_jump_rad（rad）：预处理后允许的最大相邻点跳变，真实回放门控之一。
        # 运行期跟踪监控
        #   replay_monitor_enabled（bool）：是否启用回放期间的跟踪监控。
        #   replay_monitor_period_sec（s）：监控定时器周期，内部下限 0.02。
        #   replay_monitor_start_grace_sec（s）：起步宽限期，期间不判超差。
        #   replay_monitor_violation_grace_sec（s）：连续超差需持续超过该时长才触发停止，
        #     用于滤掉单帧抖动造成的误停。
        #   max_tracking_error_rad（rad）：允许的关节跟踪误差（目标位形与反馈之差）。
        #   max_live_velocity_rad_s（rad/s）：允许的反馈关节速度。
        super().__init__("teach_replay_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("direct_threshold", 0.01)
        self.declare_parameter("align_threshold", 0.25)
        self.declare_parameter("align_duration", 3.0)
        self.declare_parameter("align_duration_auto", True)
        self.declare_parameter("align_target_speed_rad_s", 0.15)
        self.declare_parameter("align_min_duration", 3.0)
        self.declare_parameter("align_max_duration", 10.0)
        self.declare_parameter("align_steps", 30)
        self.declare_parameter("green_jump_rad", 0.03)
        self.declare_parameter("yellow_jump_rad", 0.05)
        self.declare_parameter("yellow_max_speed", 0.6)
        self.declare_parameter("max_replay_velocity_rad_s", 3.0)
        self.declare_parameter(
            "max_replay_velocity_rad_s_by_joint",
            [3.0, 3.0, 3.0, 1.8, 1.8, 1.8],
        )
        self.declare_parameter("max_replay_acceleration_rad_s2", 5.0)
        self.declare_parameter("max_replay_jerk_rad_s3", 20.0)
        self.declare_parameter("large_motion_span_rad", 0.8)
        self.declare_parameter("large_motion_total_rad", 2.5)
        self.declare_parameter("large_motion_max_speed", 1.0)
        self.declare_parameter("start_hold_sec", 0.8)
        self.declare_parameter("soft_start_duration", 1.0)
        self.declare_parameter("soft_start_steps", 30)
        self.declare_parameter("first_hold_sec", 0.3)
        self.declare_parameter("final_hold_sec", 1.0)
        self.declare_parameter("initial_replay_delay_sec", 0.2)
        self.declare_parameter("use_moveit_start_align", True)
        self.declare_parameter("moveit_start_skip_threshold", 0.005)
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_planning_time", 3.0)
        self.declare_parameter("moveit_num_planning_attempts", 3)
        self.declare_parameter("moveit_joint_goal_tolerance", 0.005)
        self.declare_parameter("moveit_velocity_scaling", 0.1)
        self.declare_parameter("moveit_acceleration_scaling", 0.1)
        self.declare_parameter("collision_check_enabled", True)
        self.declare_parameter("collision_check_service", "/check_state_validity")
        self.declare_parameter("collision_check_max_samples", 80)
        self.declare_parameter("collision_check_timeout_sec", 2.0)
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("smoothing_window", 7)
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("filter_cutoff_hz", 5.0)
        self.declare_parameter("filter_sample_rate_hz", 150.0)
        self.declare_parameter("resample_enabled", True)
        self.declare_parameter("resample_rate_hz", 150.0)
        self.declare_parameter("time_parameterization_method", "auto")
        self.declare_parameter("max_prepared_jump_rad", 0.02)
        self.declare_parameter("replay_monitor_enabled", True)
        self.declare_parameter("replay_monitor_period_sec", 0.05)
        self.declare_parameter("replay_monitor_start_grace_sec", 1.0)
        self.declare_parameter("replay_monitor_violation_grace_sec", 0.30)
        self.declare_parameter("max_tracking_error_rad", 0.25)
        self.declare_parameter("max_live_velocity_rad_s", 3.0)
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._record_path = Path(str(self.get_parameter("record_path").value))
        self._dry_run = bool(self.get_parameter("dry_run").value)
        # 运行期状态：反馈缓存、一次性启动门闩、加载/预处理产物与起点判定结果。
        self._latest_joint_state: JointState | None = None
        self._started = False
        self._samples = []
        self._start_band = ""
        self._max_error: float | None = None
        self._per_joint_error: tuple[float, ...] = ()
        self._trajectory_points = 0
        self._quality = None
        self._prepared_replay = None
        self._prepared_record_path: Path | None = None
        # 活动回放句柄与时间基准：_active_replay_started_at 使用 time.monotonic()，
        # 与 _tracking_violation_since 同源，避免系统时钟跳变干扰时延判断。
        self._goal_handle = None
        self._active_replay_trajectory: JointTrajectory | None = None
        self._active_replay_started_at: float | None = None
        self._tracking_violation_since: float | None = None
        # 两个“只触发一次”的标志：运行期监控停止、外部停止请求（信号/上层调用）。
        self._monitor_stop_requested = False
        self._stop_requested = False
        self._stop_reason = ""
        self._moveit_align_message = ""
        # 碰撞预检结果占位；真实执行前一定会被 _check_trajectory_collision 覆盖。
        self._collision_precheck = {"state": "not_run", "message": "collision precheck not run"}
        # 回放轨迹的唯一执行出口：底层关节轨迹控制器。
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        # 停止请求走控制器自己的停止服务，比单纯取消动作目标更快、更可靠。
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
        )
        # 起点对齐用的规划适配器；坐标系固定为 base_link → end_link，本节点只做关节
        # 空间规划，位置/姿态容差（0.005 m / 0.02 rad）为写死的保守值，未由参数暴露。
        self._moveit_planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id="end_link",
            frame_id="base_link",
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=0.005,
            goal_orientation_tolerance=0.02,
        )
        # 状态话题用 TRANSIENT_LOCAL：上层面板可能晚于本节点启动，必须能补到最后一帧状态。
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/replay_status",
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        sensor_qos_spec = sensor_qos_kwargs()
        # 关节反馈按传感器流处理：BEST_EFFORT + KEEP_LAST，允许丢帧，不重传历史帧。
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        # 0.2 s 轮询启动条件（等反馈 + 加载记录只做一次）；监控定时器周期由参数控制，
        # 但强制不低于 0.02 s，防止把 executor 拖死。
        self.create_timer(0.2, self._maybe_start)
        self.create_timer(
            max(float(self.get_parameter("replay_monitor_period_sec").value), 0.02),
            self._check_active_replay_tracking,
        )
        self._publish_status("ready", f"waiting to replay {self._record_path}")

    def _max_replay_velocity_limits(self, joint_names: tuple[str, ...]):
        """返回本次回放使用的关节速度上限。

        优先使用逐关节列表 ``max_replay_velocity_rad_s_by_joint``，但仅当其长度与记录的
        关节数一致时才生效；否则回退为标量 ``max_replay_velocity_rad_s``。返回值可能是
        ``tuple[float, ...]``（逐关节）或 ``float``（统一上限），下游两者都能接受。
        长度不一致时不报错而是静默回退，是为了避免参数配置错误直接让回放失败。
        """
        scalar_limit = float(self.get_parameter("max_replay_velocity_rad_s").value)
        values = self.get_parameter("max_replay_velocity_rad_s_by_joint").value
        if isinstance(values, (list, tuple)) and len(values) == len(joint_names):
            return tuple(float(value) for value in values)
        return scalar_limit

    def _auto_align_duration_for_positions(
        self,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        """计算从当前位形插值到记录首点所用的对齐时长（秒）。

        ``align_duration_auto`` 为假时直接返回固定值 ``align_duration``；为真时先求各关节
        误差绝对值的最大值（最坏关节决定对齐时间），再按目标速度换算时长，并由
        :func:`compute_auto_align_duration` 夹到 [align_min_duration, align_max_duration]。
        """
        if not bool(self.get_parameter("align_duration_auto").value):
            return float(self.get_parameter("align_duration").value)
        max_error = max(
            (abs(float(current) - float(first)) for current, first in zip(current_positions, first_positions)),
            default=0.0,
        )
        return compute_auto_align_duration(
            max_error,
            target_speed_rad_s=float(self.get_parameter("align_target_speed_rad_s").value),
            min_duration_sec=float(self.get_parameter("align_min_duration").value),
            max_duration_sec=float(self.get_parameter("align_max_duration").value),
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """缓存最新一帧关节反馈（位置/速度），供起点判定与运行期监控读取。"""
        self._latest_joint_state = msg

    def _maybe_start(self) -> None:
        """启动回放的一次性入口，由 0.2 s 定时器反复调用。

        完整流程：等反馈 → 加载原始记录 → 用记录首样本的关节顺序对齐当前反馈 →
        起点分带判定 → 原始记录质量分析 → 预处理并写预处理记录 → 真实执行门控 →
        组装轨迹 → 碰撞预检 → ``dry_run`` 上报或发送动作目标。

        所有失败路径都只发布状态后 ``return``，不会抛异常中断节点；``_started`` 在任何
        实质动作之前就已置位，因此每进程生命周期内最多回放一次。
        """
        if self._started:
            return
        if self._latest_joint_state is None:
            self._publish_status("waiting", "waiting for current joint_states")
            return
        self._started = True
        try:
            self._samples = load_teach_samples(self._record_path)
        except Exception as exc:
            self._publish_status("failed", f"failed to load record: {exc}")
            return
        if not self._samples:
            self._publish_status("failed", "record contains no samples")
            return

        first = self._samples[0]
        # 反馈可能包含夹爪等额外关节，而记录只含手臂关节；这里按“记录首样本的关节顺序”
        # 重建当前位形向量，保证两边顺序严格一致（顺序错位会直接变成错误的起点误差）。
        current_map = {
            str(name): float(pos)
            for name, pos in zip(self._latest_joint_state.name, self._latest_joint_state.position)
        }
        missing = [name for name in first.joint_names if name not in current_map]
        if missing:
            self._publish_status("failed", f"current joint_state missing: {', '.join(missing)}")
            return
        current = tuple(current_map[name] for name in first.joint_names)
        decision = classify_replay_start(
            current_positions=current,
            start_positions=first.positions,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        # 起点误差过大被 reject 时，若允许起点对齐则降级为“先规划到记录首点”；
        # 不允许对齐的配置下直接拒绝，不做任何运动。
        start_band = decision.band
        if decision.band == ReplayStartBand.REJECT and bool(self.get_parameter("use_moveit_start_align").value):
            start_band = ReplayStartBand.MOVEIT_ALIGN
        if decision.band == ReplayStartBand.REJECT:
            if start_band != ReplayStartBand.MOVEIT_ALIGN:
                self._start_band = str(decision.band.value)
                self._max_error = decision.max_error
                self._per_joint_error = decision.per_joint_error
                self._publish_status("rejected", decision.message, max_error=decision.max_error)
                return

        # 起点误差信息统一记录（下面两行与上一分支的赋值结果相同，属重复赋值）。
        self._start_band = str(start_band.value)
        if start_band == ReplayStartBand.MOVEIT_ALIGN:
            self.get_logger().warn(
                "start error exceeds joint-space align threshold; using MoveIt start alignment"
            )
        else:
            self._start_band = str(decision.band.value)
            self._max_error = decision.max_error
            self._per_joint_error = decision.per_joint_error
        self._max_error = decision.max_error
        self._per_joint_error = decision.per_joint_error
        # 先对原始记录做质量分析（green/yellow 跳变阈值 + 速度/加速度/加加速度上限），
        # 结果用于状态上报与风险等级判定。
        self._quality = analyze_teach_trajectory(
            self._samples,
            green_jump_rad=float(self.get_parameter("green_jump_rad").value),
            yellow_jump_rad=float(self.get_parameter("yellow_jump_rad").value),
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(self._samples[0].joint_names)),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
        )
        # 预处理链固定开启重定时（retime_enabled=True）；重定时结果非空时即后续真正下发的
        # 轨迹点，为空（例如记录存在结构性异常）则退回 :meth:`_build_trajectory` 的兜底重定时。
        self._prepared_replay = prepare_teach_replay_samples(
            self._samples,
            smoothing_enabled=bool(self.get_parameter("smoothing_enabled").value),
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            filter_enabled=bool(self.get_parameter("filter_enabled").value),
            filter_cutoff_hz=float(self.get_parameter("filter_cutoff_hz").value),
            filter_sample_rate_hz=float(self.get_parameter("filter_sample_rate_hz").value),
            resample_enabled=bool(self.get_parameter("resample_enabled").value),
            resample_rate_hz=float(self.get_parameter("resample_rate_hz").value),
            retime_enabled=True,
            replay_speed=float(self.get_parameter("speed").value),
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(self._samples[0].joint_names)),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            time_parameterization_method=str(self.get_parameter("time_parameterization_method").value),
            large_motion_span_rad=float(self.get_parameter("large_motion_span_rad").value),
            large_motion_total_rad=float(self.get_parameter("large_motion_total_rad").value),
            large_motion_max_speed=float(self.get_parameter("large_motion_max_speed").value),
        )
        self._prepared_record_path = write_prepared_teach_record(
            self._record_path,
            self._prepared_replay,
        )
        # 以下门控一律只看“预处理之后”的质量（after_quality），因为真正下发的是预处理
        # 轨迹；原始记录的风险等级只用于状态展示与对比。
        speed = float(self.get_parameter("speed").value)
        yellow_max_speed = float(self.get_parameter("yellow_max_speed").value)
        replay_quality = self._prepared_replay.after_quality
        max_prepared_jump_rad = float(self.get_parameter("max_prepared_jump_rad").value)
        max_replay_acceleration = float(self.get_parameter("max_replay_acceleration_rad_s2").value)
        max_replay_jerk = float(self.get_parameter("max_replay_jerk_rad_s3").value)
        # 门控 1：预处理后仍不允许真实回放（结构性异常/风险过高）→ 真实执行拒绝。
        if not self._dry_run and not replay_quality.allow_real_replay:
            self._publish_status(
                "rejected",
                replay_quality.replay_policy,
                max_error=decision.max_error,
            )
            return
        # dry_run 下同一情况只记为 dry_run（已校验但真实执行被拦），便于上层区分“参数问题”
        # 与“试运行结论”。
        if self._dry_run and not replay_quality.allow_real_replay:
            self._publish_status(
                "dry_run",
                (
                    "validated replay but real execution is blocked; "
                    f"quality={self._quality.risk_level}->{self._prepared_replay.after_quality.risk_level}"
                ),
                max_error=decision.max_error,
            )
            return
        # 门控 2：预处理后相邻点跳变仍超限 → 拒绝（说明平滑/重定时没能压住阶跃）。
        if not self._dry_run and float(replay_quality.max_jump_rad) > max_prepared_jump_rad:
            self._publish_status(
                "rejected",
                (
                    "prepared replay jump is still too large: "
                    f"{float(replay_quality.max_jump_rad):.4f} rad > {max_prepared_jump_rad:.4f} rad"
                ),
                max_error=decision.max_error,
            )
            return
        # 门控 3：重定时后的加速度仍超上限 → 拒绝。
        if not self._dry_run and float(replay_quality.max_acceleration_rad_s2) > max_replay_acceleration:
            self._publish_status(
                "rejected",
                (
                    "retimed replay acceleration is still too large: "
                    f"{float(replay_quality.max_acceleration_rad_s2):.4f} rad/s^2 > "
                    f"{max_replay_acceleration:.4f} rad/s^2"
                ),
                max_error=decision.max_error,
            )
            return
        # 门控 4：重定时后的加加速度仍超上限 → 拒绝。
        if not self._dry_run and float(replay_quality.max_jerk_rad_s3) > max_replay_jerk:
            self._publish_status(
                "rejected",
                (
                    "retimed replay jerk is still too large: "
                    f"{float(replay_quality.max_jerk_rad_s3):.4f} rad/s^3 > "
                    f"{max_replay_jerk:.4f} rad/s^3"
                ),
                max_error=decision.max_error,
            )
            return
        # 门控 5：黄色风险下有效回放速度不得超过 yellow_max_speed——这里是直接拒绝，
        # 不自动降速（自动降速只出现在 _build_trajectory 的兜底重定时路径）。
        effective_speed = (
            float(self._prepared_replay.effective_replay_speed)
            if self._prepared_replay is not None
            else speed
        )
        if not self._dry_run and replay_quality.risk_level == "yellow" and effective_speed > yellow_max_speed:
            self._publish_status(
                "rejected",
                f"prepared yellow replay speed must be <= {yellow_max_speed:.2f}",
                max_error=decision.max_error,
            )
            return
        # 组装轨迹失败（例如起点对齐规划失败/缺关节）只上报 blocked，绝不下发半成品轨迹。
        try:
            trajectory = self._build_trajectory(current, start_band)
        except RuntimeError as exc:
            self._publish_status("blocked", str(exc), max_error=decision.max_error)
            return
        self._trajectory_points = len(trajectory.points)
        # 门控 6：碰撞预检。只有明确 pass 才允许真实执行；collision / unknown 都拦截，
        # 而 disabled（预检被显式关闭）不拦截。
        self._collision_precheck = self._check_trajectory_collision(trajectory)
        collision_state = str(self._collision_precheck.get("state", "")).lower()
        if collision_state in ("collision", "unknown") and not self._dry_run:
            self._publish_status(
                "blocked",
                f"collision precheck blocked replay: {self._collision_precheck.get('message', collision_state)}",
                max_error=decision.max_error,
            )
            return
        # dry_run 分支：不做任何运动，只回报校验结论（碰撞/未知则记为 blocked）。
        if self._dry_run:
            dry_state = "dry_run" if collision_state not in ("collision", "unknown") else "blocked"
            collision_suffix = (
                ""
                if dry_state == "dry_run"
                else f"; collision precheck blocked replay: {self._collision_precheck.get('message', collision_state)}"
            )
            self._publish_status(
                dry_state,
                (
                    f"validated replay with {len(trajectory.points)} trajectory points; "
                    f"quality={self._quality.risk_level}->{self._prepared_replay.after_quality.risk_level}"
                    f"{collision_suffix}"
                ),
                max_error=decision.max_error,
            )
            return
        # 真实执行路径：动作服务器 2 s 内不可用即失败，不重试、不降级为直接发布。
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self._publish_status("failed", "follow_joint_trajectory action unavailable")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, trajectory))

    def _on_goal_response(self, future, trajectory: JointTrajectory) -> None:
        """动作目标响应回调：记录活动句柄与时间基准，并注册结果回调。"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("failed", f"failed to send replay trajectory: {exc}")
            return
        if not goal_handle.accepted:
            self._publish_status("rejected", "replay trajectory goal rejected")
            return
        self._goal_handle = goal_handle
        # 记录活动回放上下文：轨迹、单调时钟起点与违约计时复位，运行期监控据此工作。
        self._active_replay_trajectory = trajectory
        self._active_replay_started_at = time.monotonic()
        self._tracking_violation_since = None
        self._monitor_stop_requested = False
        self._publish_status("replaying", "trajectory goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_replay_result)

    def _on_replay_result(self, future) -> None:
        """动作结果回调：把目标状态映射为对外状态并清理活动回放上下文。

        这里直接比较 ``status`` 的原始整数，避免为两个常量额外引入动作状态消息类型：
        4 为 ``SUCCEEDED``、5 为 ``CANCELED``（沿用动作目标状态枚举的取值）。只有
        “成功且 ``error_code`` 为 0”才记为 ``done``：控制器可能返回成功状态但同时
        带上非零错误码，这种情况按 ``failed`` 处理，避免把异常回放报成正常完成。
        """
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._publish_status("failed", f"replay result retrieval failed: {exc}")
            self._goal_handle = None
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        self._publish_status(
            state,
            f"replay result status={status}, error_code={error_code}: {error_string}",
        )
        self._goal_handle = None
        self._active_replay_trajectory = None
        self._active_replay_started_at = None
        self._tracking_violation_since = None
        self._monitor_stop_requested = False

    def request_stop(self, reason: str) -> None:
        """线程/信号安全地登记停止请求；真正的停止动作由 :meth:`cancel_active_goal` 执行。

        这里只置标志与原因，不做任何 ROS 调用——信号处理器里不能自旋 executor。
        """
        self._stop_requested = True
        self._stop_reason = reason

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    def cancel_active_goal(self, *, timeout_sec: float = 2.0) -> bool:
        """停止当前回放：先停控制器轨迹，再取消动作目标。

        ``timeout_sec`` 是取消 future 的等待上限（秒）；控制器停止服务的等待被额外夹到
        0.8 s，避免“先等控制器、再等取消”把总停机时间拖到两倍超时。返回 ``True`` 表示
        控制器停止请求或目标取消请求至少有一条成功发出；注意它只表示“请求已发出”，
        不代表机械臂已经静止。

        本方法会在回调线程内自旋 executor 等待取消结果，因此只能在 ``main`` 的收尾
        路径调用，不能在订阅/定时器回调里调用。
        """
        goal_handle = self._goal_handle
        reason = self._stop_reason or "stop requested"
        self._publish_status("cancel_requested", f"{reason}; stopping replay trajectory")
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=min(timeout_sec, 0.8))
        cancel_requested = False
        if goal_handle is not None:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_requested = True
            except Exception as exc:
                self._publish_status("failed", f"failed to request replay cancel: {exc}")
                return stop_requested
            with suppress(Exception, KeyboardInterrupt):
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=timeout_sec)
        return stop_requested or cancel_requested

    def _check_active_replay_tracking(self) -> None:
        """回放期间的运行期安全监控，按 ``replay_monitor_period_sec`` 周期触发。

        先过一串“不适用”门：未启用监控、``dry_run``、没有活动目标或轨迹、没有反馈、
        没有起始时刻，任一成立就直接返回。起步宽限期内也返回，因为轨迹刚开始跟踪时
        误差天然偏大。

        违约判定用“持续超差”而不是“瞬时超差”：第一次不通过只记录时间戳，之后每次
        检查都重新评估，只有不通过状态持续超过 ``replay_monitor_violation_grace_sec``
        才触发停止，这样可以滤掉单帧反馈抖动造成的误停。停止只执行一次
        （``_monitor_stop_requested`` 是门闩），并先发 ``safety_stop`` 状态再停控制器。
        """
        if not bool(self.get_parameter("replay_monitor_enabled").value):
            return
        if self._dry_run or self._goal_handle is None or self._active_replay_trajectory is None:
            return
        if self._latest_joint_state is None or self._active_replay_started_at is None:
            return
        now = time.monotonic()
        elapsed = now - self._active_replay_started_at
        if elapsed < float(self.get_parameter("replay_monitor_start_grace_sec").value):
            return
        result = evaluate_replay_tracking(
            self._active_replay_trajectory,
            joint_names=tuple(self._latest_joint_state.name),
            positions=tuple(float(v) for v in self._latest_joint_state.position),
            velocities=tuple(float(v) for v in self._latest_joint_state.velocity),
            elapsed_sec=elapsed,
            max_tracking_error_rad=float(self.get_parameter("max_tracking_error_rad").value),
            max_live_velocity_rad_s=float(self.get_parameter("max_live_velocity_rad_s").value),
        )
        if result.ok:
            # 恢复正常即清零违约计时，下一次超差重新开始累计。
            self._tracking_violation_since = None
            return
        if self._tracking_violation_since is None:
            self._tracking_violation_since = now
            return
        if now - self._tracking_violation_since < float(self.get_parameter("replay_monitor_violation_grace_sec").value):
            return
        if self._monitor_stop_requested:
            return
        # 安全停止：先上报原因（含评估器给出的具体关节/数值），再停控制器、最后取消目标。
        # 停止服务只等 0.2 s——监控回调在 executor 线程上，不能长时间阻塞。
        self._monitor_stop_requested = True
        self._publish_status(
            "safety_stop",
            f"runtime replay monitor stopped trajectory: {result.message}",
        )
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        try:
            self._goal_handle.cancel_goal_async()
        except Exception as exc:
            self._publish_status("failed", f"failed to cancel after runtime monitor stop: {exc}")

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        """调用控制器的无参停止服务，返回服务是否在超时内返回。

        服务发现只等 0.1 s（快路径，避免拖慢调用方）；服务不存在、调用抛异常、或 future
        未在 ``timeout_sec`` 内完成都返回 ``False``，由调用方决定后续动作。本方法会在
        回调线程内自旋 executor，是运行期监控触发停止时唯一允许的阻塞点。
        """
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=0.1):
                return False
            future = self._trajectory_stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return bool(future.done())
        except Exception:
            return False

    def _build_trajectory(
        self,
        current_positions: tuple[float, ...],
        start_band: ReplayStartBand,
    ) -> JointTrajectory:
        """组装最终下发的关节轨迹：起点对齐段 + 回放主体 + 末尾保持段。

        关节顺序与名字一律取自记录首样本（``trajectory.joint_names``），后面所有段都按
        这个顺序排列。时间戳全部是相对轨迹起点的累计秒。

        起点对齐有两条互斥路径：``use_moveit_start_align`` 为真时用规划服务规划到记录
        首点；为假时用关节空间插值（``direct`` 带用软启动参数，``align`` 带用对齐参数），
        插值段速度恒为 0。

        回放主体优先使用预处理产出的重定时点（正常路径，见
        :meth:`_append_prepared_replay_points`，不再二次重定时）；只有预处理没有产出
        重定点时才回退到本函数内现场重定时（兜底路径），并在黄色风险时把速度夹到
        ``yellow_max_speed``。
        """
        first = self._samples[0]
        trajectory = JointTrajectory()
        trajectory.joint_names = list(first.joint_names)
        elapsed = 0.0
        if bool(self.get_parameter("use_moveit_start_align").value):
            elapsed = self._append_moveit_start_alignment(
                trajectory,
                current_positions=current_positions,
                first_positions=first.positions,
            )
        else:
            # 插值段的时长由起点误差决定：align 带按误差自动算，direct 带用软启动时长。
            start_points = build_replay_start_soft_points(
                current_positions=current_positions,
                first_positions=first.positions,
                start_band=start_band.value,
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                soft_start_duration=float(self.get_parameter("soft_start_duration").value),
                soft_start_steps=int(self.get_parameter("soft_start_steps").value),
                align_duration=self._auto_align_duration_for_positions(current_positions, first.positions),
                align_steps=int(self.get_parameter("align_steps").value),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
            )
            for start_point in start_points:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in start_point.positions]
                point.velocities = [0.0 for _ in start_point.positions]
                _set_duration(point.time_from_start, start_point.time_from_start)
                trajectory.points.append(point)
            if start_points:
                elapsed = start_points[-1].time_from_start
        if self._prepared_replay is not None and self._prepared_replay.retimed_points:
            self._append_prepared_replay_points(trajectory, elapsed=elapsed)
        else:
            # 兜底路径：预处理没有产出重定点点位时现场重定时。速度取请求值与预处理有效
            # 值的较大者（下限 0.01，避免零速导致时间轴除零），黄色风险再夹到限速值。
            speed = max(float(self.get_parameter("speed").value), 0.01)
            if self._prepared_replay is not None:
                speed = max(float(self._prepared_replay.effective_replay_speed), 0.01)
            replay_quality = self._prepared_replay.after_quality if self._prepared_replay is not None else self._quality
            if replay_quality is not None and replay_quality.risk_level == "yellow":
                speed = min(speed, float(self.get_parameter("yellow_max_speed").value))
            replay_samples = self._prepared_replay.samples if self._prepared_replay is not None else self._samples
            for retimed in retime_teach_samples(
                replay_samples,
                replay_speed=speed,
                max_velocity_rad_s=self._max_replay_velocity_limits(tuple(replay_samples[0].joint_names)),
                max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
                initial_delay_sec=float(self.get_parameter("initial_replay_delay_sec").value),
                boundary_zero_velocity=True,
            ):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in retimed.positions]
                if retimed.velocities:
                    point.velocities = [float(v) for v in retimed.velocities]
                _set_duration(point.time_from_start, elapsed + retimed.time_from_start)
                trajectory.points.append(point)
        self._append_final_hold(trajectory)
        return trajectory

    def _check_trajectory_collision(self, trajectory: JointTrajectory) -> dict:
        """对轨迹做抽样碰撞预检，返回状态字典（键：``state`` / ``message`` / 可选计数与接触信息）。

        ``state`` 取值与含义：

        - ``disabled``：``collision_check_enabled`` 为假，未做检查（**不拦截**真实回放）；
        - ``unknown``：轨迹为空、服务不可用、调用异常或超时/未完成，结论不可用
          （真实回放被拦截）；
        - ``collision``：某个抽样位形无效，附带 ``point``（轨迹点索引）与前 5 组接触体对；
        - ``pass``：全部抽样点都有效。

        抽样点数由 ``collision_check_max_samples`` 决定（首尾点保证被抽到）。总耗时由
        ``collision_check_timeout_sec`` 封顶（内部下限 0.1 s）：每发一个请求就轮询等待
        future（10 ms 一次），一旦越过 deadline 立即返回 ``unknown``，绝不无限等待。
        发现第一个碰撞点即停止后续抽样——预检只需判定“有碰撞”，无需穷举。
        """
        if not bool(self.get_parameter("collision_check_enabled").value):
            return {"state": "disabled", "message": "collision precheck disabled"}
        points = [
            tuple(float(v) for v in point.positions)
            for point in trajectory.points
            if getattr(point, "positions", None)
        ]
        if not trajectory.joint_names or not points:
            return {"state": "unknown", "message": "no trajectory points to collision check"}
        try:
            available = bool(self._state_validity_client.service_is_ready())
            if not available:
                available = bool(self._state_validity_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            available = False
        if not available:
            return {
                "state": "unknown",
                "message": "MoveIt state validity service unavailable",
                "service": str(self.get_parameter("collision_check_service").value),
                "checked_samples": 0,
            }
        selected = _select_collision_points(
            points,
            max_samples=int(self.get_parameter("collision_check_max_samples").value),
        )
        collision_joint_names, selected = self._with_collision_default_joints(
            tuple(trajectory.joint_names),
            selected,
        )
        timeout_sec = max(float(self.get_parameter("collision_check_timeout_sec").value), 0.1)
        # deadline 用单调时钟计算，服务端慢或卡死时也能保证在总超时内返回 unknown。
        deadline = time.monotonic() + timeout_sec
        checked = 0
        collisions = []
        for point_index, positions in selected:
            if time.monotonic() >= deadline:
                return {
                    "state": "unknown",
                    "message": "collision precheck timed out",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            request = GetStateValidity.Request()
            # 只填组名与关节状态；未填的字段由 MoveIt 按当前规划场景补全。
            request.group_name = str(self.get_parameter("collision_group_name").value)
            request.robot_state.joint_state.name = list(collision_joint_names)
            request.robot_state.joint_state.position = [float(v) for v in positions]
            future = self._state_validity_client.call_async(request)
            # 同步轮询等待：这是启动路径（定时器回调）里的有限阻塞，总时长受 deadline 约束。
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                break
            try:
                response = future.result()
            except Exception as exc:
                return {
                    "state": "unknown",
                    "message": f"collision precheck failed: {exc}",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            checked += 1
            if not bool(getattr(response, "valid", False)):
                # 只保留前 5 组接触体对：状态 payload 需要保持精简，且第一处碰撞已足够定位。
                contacts = []
                for contact in list(getattr(response, "contacts", []))[:5]:
                    contacts.append(
                        {
                            "body_1": str(getattr(contact, "contact_body_1", "")),
                            "body_2": str(getattr(contact, "contact_body_2", "")),
                        }
                    )
                collisions.append({"point": point_index, "contacts": contacts})
                break
        if collisions:
            return {
                "state": "collision",
                "message": "collision detected in replay trajectory",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": collisions,
            }
        if checked < len(selected):
            # 走完循环但没检查完（超时 break），结论不可信，按 unknown 处理而非 pass。
            return {
                "state": "unknown",
                "message": "collision precheck incomplete",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": [],
            }
        return {
            "state": "pass",
            "message": "no collision detected in sampled replay trajectory",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": [],
        }

    def _with_collision_default_joints(
        self,
        joint_names: tuple[str, ...],
        selected: list[tuple[int, tuple[float, ...]]],
    ) -> tuple[tuple[str, ...], list[tuple[int, tuple[float, ...]]]]:
        """给碰撞检查请求补齐夹爪关节，使关节向量与所选规划组匹配。

        只在 ``collision_group_name`` 严格等于 "arm_with_gripper" 时生效：该组包含两个
        夹爪关节，而示教记录只含 6 个手臂关节；若请求里缺关节，MoveIt 会用默认值补齐，
        得到的位形与实际不符。因此这里用最近一帧 joint_states 中的夹爪位置，缺失时用
        记录文件里的零位约定（左指 0.0、右指 -0.0，两指方向相反）。

        返回新的 (关节名元组, 抽样点列表)：关节名在末尾追加补入的名字，每个抽样点的
        位置向量同步追加对应位置；轨迹点索引保持不变，因此碰撞报告仍能指向轨迹中的点。
        """
        if str(self.get_parameter("collision_group_name").value) != "arm_with_gripper":
            return joint_names, selected
        existing = set(joint_names)
        defaults: list[tuple[str, float]] = []
        latest = self._latest_joint_state
        latest_positions = {}
        if latest is not None:
            latest_positions = {
                str(name): float(latest.position[index])
                for index, name in enumerate(latest.name)
                if index < len(latest.position)
            }
        for name, fallback in (("left_finger_joint", 0.0), ("right_finger_joint", -0.0)):
            if name not in existing:
                defaults.append((name, float(latest_positions.get(name, fallback))))
        if not defaults:
            return joint_names, selected
        added_names = tuple(name for name, _ in defaults)
        added_positions = tuple(position for _, position in defaults)
        return (
            joint_names + added_names,
            [(index, tuple(positions) + added_positions) for index, positions in selected],
        )

    def _append_final_hold(self, trajectory: JointTrajectory) -> None:
        """在轨迹末尾追加一段“保持末点位置、速度 0”的点，持续 ``final_hold_sec`` 秒。

        目的是让控制器在轨迹结束后仍有明确的目标位形可跟踪，避免刚到位就失去目标。
        ``final_hold_sec`` 不大于 0 或轨迹为空时不追加。上一末点的时刻由 ``sec`` 与
        ``nanosec`` 两段还原为浮点秒（1e-9 s/ns）后再叠加保持时长。
        """
        final_hold = max(float(self.get_parameter("final_hold_sec").value), 0.0)
        if final_hold <= 0.0 or not trajectory.points:
            return
        last_point = trajectory.points[-1]
        last_time = float(last_point.time_from_start.sec) + float(last_point.time_from_start.nanosec) * 1e-9
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in last_point.positions]
        hold_point.velocities = [0.0 for _ in hold_point.positions]
        _set_duration(hold_point.time_from_start, last_time + final_hold)
        trajectory.points.append(hold_point)

    def _append_prepared_replay_points(
        self,
        trajectory: JointTrajectory,
        *,
        elapsed: float,
    ) -> None:
        """把预处理产出的重定时点直接追加进轨迹（回放的正常路径）。

        预处理的 ``retimed_points`` 已经满足速度/加速度/加加速度上限并带前馈速度，因此
        这里不再做任何重定时或限速，只做时间轴平移：``elapsed``（起点对齐段耗时）+
        ``initial_replay_delay_sec``（等待控制器进入跟踪的静止延时）+ 该点自身的时间。
        缺少速度的点按 0 速度下发。预处理为空时直接返回，不影响已有轨迹。
        """
        if self._prepared_replay is None:
            return
        initial_delay = max(float(self.get_parameter("initial_replay_delay_sec").value), 0.0)
        for retimed in self._prepared_replay.retimed_points:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in retimed.positions]
            if retimed.velocities:
                point.velocities = [float(v) for v in retimed.velocities]
            else:
                point.velocities = [0.0 for _ in point.positions]
            _set_duration(point.time_from_start, elapsed + initial_delay + float(retimed.time_from_start))
            trajectory.points.append(point)

    def _append_moveit_start_alignment(
        self,
        trajectory: JointTrajectory,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        """用规划服务把机械臂从当前位形规划到记录首点，并追加进轨迹。

        返回起点对齐段的累计时长（秒），后续回放段从该时刻接续。

        步骤与约定：

        1. 先追加一段 ``start_hold_sec`` 的“保持当前位置”点，让控制器先收到一个明确目标。
        2. 只有当前位形与首点的最大关节误差达到 ``moveit_start_skip_threshold`` 才真正
           发起规划——已经足够接近时省掉一次规划调用，直接靠保持点衔接。
        3. 规划失败或缺关节时抛 ``RuntimeError``（由 :meth:`_maybe_start` 转成 ``blocked``
           状态），绝不退化成“跳过对齐直接回放”。
        4. 规划结果的关节顺序可能与记录不同：按关节名建立索引映射后重排成
           ``trajectory.joint_names`` 的顺序；规划点与上一个点时间戳完全相同时跳过，
           避免出现零时长重复点。
        5. 最后追加 ``first_hold_sec`` 的首点保持段，并把 ``elapsed`` 推进到该点时刻。
        """
        elapsed = max(float(self.get_parameter("start_hold_sec").value), 0.0)
        # 保持点用当前位置，速度 0，时刻即 start_hold_sec。
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in current_positions]
        hold_point.velocities = [0.0 for _ in current_positions]
        _set_duration(hold_point.time_from_start, elapsed)
        trajectory.points.append(hold_point)
        # 最坏关节决定是否需要规划，与起点分类用的是同一判据（最大绝对误差）。
        max_error = max(
            (abs(float(a) - float(b)) for a, b in zip(current_positions, first_positions)),
            default=0.0,
        )
        if max_error >= float(self.get_parameter("moveit_start_skip_threshold").value):
            plan = self._moveit_planner.plan_joint_positions(
                joint_names=tuple(trajectory.joint_names),
                target_positions=first_positions,
                tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            )
            if not plan.success or plan.trajectory is None:
                self._moveit_align_message = plan.message
                raise RuntimeError(f"moveit start alignment failed: {plan.message}")
            self._moveit_align_message = plan.message
            # 规划轨迹可能只含部分关节或顺序不同，先建名字→下标映射，缺关节即失败。
            source_names = list(getattr(plan.trajectory, "joint_names", []))
            index_by_name = {name: index for index, name in enumerate(source_names)}
            missing = [name for name in trajectory.joint_names if name not in index_by_name]
            if missing:
                raise RuntimeError(f"moveit start alignment missing joints: {', '.join(missing)}")
            for source_point in getattr(plan.trajectory, "points", []):
                # 时间戳由 sec + nanosec 还原为浮点秒，再整体平移到保持段之后。
                source_time = float(source_point.time_from_start.sec) + float(source_point.time_from_start.nanosec) * 1e-9
                point = JointTrajectoryPoint()
                point.positions = [
                    float(source_point.positions[index_by_name[name]])
                    for name in trajectory.joint_names
                ]
                if getattr(source_point, "velocities", None):
                    point.velocities = [
                        float(source_point.velocities[index_by_name[name]])
                        for name in trajectory.joint_names
                    ]
                _set_duration(point.time_from_start, elapsed + source_time)
                if trajectory.points and point.time_from_start.sec == trajectory.points[-1].time_from_start.sec and point.time_from_start.nanosec == trajectory.points[-1].time_from_start.nanosec:
                    continue
                trajectory.points.append(point)
            if trajectory.points:
                last = trajectory.points[-1].time_from_start
                elapsed = float(last.sec) + float(last.nanosec) * 1e-9
        # 首点保持段：无论是否规划，都让机械臂在记录首点稍作停留再进入回放主体。
        first_hold = max(float(self.get_parameter("first_hold_sec").value), 0.0)
        if first_hold > 0.0:
            elapsed += first_hold
            first_point = JointTrajectoryPoint()
            first_point.positions = [float(v) for v in first_positions]
            first_point.velocities = [0.0 for _ in first_positions]
            _set_duration(first_point.time_from_start, elapsed)
            trajectory.points.append(first_point)
        return elapsed

    def _publish_status(
        self,
        state: str,
        message: str,
        *,
        max_error: float | None = None,
    ) -> None:
        """把当前状态以 JSON 文本发布到 ``teleop/replay_status``。

        ``state`` 是状态机取值（见类文档），``message`` 是面向运维的说明文本；两者分别写入
        payload 的 ``state`` 与 ``message`` 键，是状态面板的主要显示内容。payload 其余键按
        用途分组：记录与配置回显（路径、回放速度、样本数、阈值、保持时长、预处理开关）、
        起点判定（``start_band`` 与逐关节误差）、预处理产物路径，以及碰撞预检结果。

        三个字段是惰性附加的：原始记录质量（``quality`` / ``risk_level``）、预处理结果
        （``prepared_replay`` 及其风险等级）、预处理记录路径——它们只在对应阶段完成后才
        出现，因此读 payload 时必须允许缺键。

        ``max_error`` 显式传入时优先（用于回报“本次判定用的起点误差”），否则回落到内部
        缓存的 ``_max_error``；两者都没有时不输出该键。JSON 使用紧凑分隔符以缩小消息体积。
        """
        msg = String()
        payload = {
            "state": state,
            "message": message,
            "record_path": str(self._record_path),
            "dry_run": self._dry_run,
            "speed": float(self.get_parameter("speed").value),
            "samples": len(self._samples),
            "trajectory_points": self._trajectory_points,
            "start_band": self._start_band,
            "per_joint_error": list(self._per_joint_error),
            "direct_threshold": float(self.get_parameter("direct_threshold").value),
            "align_threshold": float(self.get_parameter("align_threshold").value),
            "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
            "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
            "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
            "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
            "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
            "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
            "moveit_start_align_message": self._moveit_align_message,
            "collision_precheck": self._collision_precheck,
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "filter_enabled": bool(self.get_parameter("filter_enabled").value),
            "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
            "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
            "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
            "time_parameterization_method": str(self.get_parameter("time_parameterization_method").value),
        }
        if self._quality is not None:
            payload["quality"] = teach_trajectory_quality_to_dict(self._quality)
            payload["risk_level"] = self._quality.risk_level
        if self._prepared_replay is not None:
            # prepared_risk_level 与 effective_risk_level 当前取同一个值（预处理后质量），
            # 同时保留两个键是为了兼容既有的状态读取方。
            payload["prepared_replay"] = prepared_teach_replay_to_dict(self._prepared_replay)
            payload["prepared_risk_level"] = self._prepared_replay.after_quality.risk_level
            payload["effective_risk_level"] = self._prepared_replay.after_quality.risk_level
        if self._prepared_record_path is not None:
            payload["prepared_record_path"] = str(self._prepared_record_path)
        if max_error is not None:
            payload["max_error"] = max_error
        elif self._max_error is not None:
            payload["max_error"] = self._max_error
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)


def main(args=None) -> None:
    """节点入口：自旋直到收到停止信号，并做有序停机。

    这里不用 ``rclpy.spin`` 而用 0.1 s 一次的 ``spin_once`` 循环，是为了能在收到
    SIGINT/SIGTERM 后及时跳出循环，再执行“请求控制器停轨迹 + 取消动作目标”的有序停机；
    信号处理器本身只登记停止原因，不做任何 ROS 调用。

    收尾阶段依次恢复原有信号处理器、销毁节点、关闭上下文，并吞掉 ``KeyboardInterrupt``，
    保证重复 Ctrl-C 不会让清理逻辑中断而留下未释放的资源。
    """
    rclpy.init(args=args)
    node = TeachReplayNode()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def _request_signal_stop(signum, _frame) -> None:
        node.request_stop(f"signal {int(signum)} received")

    signal.signal(signal.SIGINT, _request_signal_stop)
    signal.signal(signal.SIGTERM, _request_signal_stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.stop_requested:
            node.cancel_active_goal(timeout_sec=2.0)
    except KeyboardInterrupt:
        node.request_stop("KeyboardInterrupt received")
        node.cancel_active_goal(timeout_sec=2.0)
    finally:
        with suppress(Exception):
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()

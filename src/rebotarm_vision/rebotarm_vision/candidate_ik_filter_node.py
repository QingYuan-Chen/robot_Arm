"""抓取候选的 IK 与碰撞可行性过滤节点（视觉链路中候选生成之后、执行之前的一层）。

职责与在系统中的位置：订阅上游视觉节点给出的抓取候选数组，为每个候选展开多组
"接近点 + 抓取点"目标位姿，逐个调用 MoveIt 的逆解服务求关节解、再用状态有效性服务
做碰撞检查，最后按几何闸门与可动性代价打分排序，发布过滤后的候选数组与一份抓取计划。
本节点只做校验与排序：既不发送轨迹，也不直接驱动硬件；真正的规划、碰撞检查与执行
门控由下游执行器负责。

对外接口：
- 订阅 ``input_topic``（默认 /grasp/candidates）：上游候选数组，位姿坐标系由消息
  header.frame_id 给出；
- 订阅 ``joint_state_topic``（默认 /rebotarm/visual_joint_states）：当前关节角，
  既作 IK 的种子状态，也用于计算关节位移代价；
- 发布 ``output_topic``（默认 /grasp/filtered_candidates）：保留下来并按得分降序的候选；
- 发布 ``output_plan_topic``（默认 /grasp/filtered_plan）：最优候选的完整计划
  （source=candidate_ik_filter），供执行器消费；
- 调用 ``moveit_ik_service``（默认 /compute_ik）与 ``collision_check_service``
  （默认 /check_state_validity）。

单帧处理流程：候选级预检（置信度、夹爪宽度）→ 位姿变体展开（位姿策略、偏航角偏移、
高度偏移、平行夹爪对称姿态）→ 几何闸门（夹爪宽度、最低抓取高度、可选工作空间盒）→
逐变体 IK + 碰撞检查 → 关节位移与 joint6 变化量评估 → 同一候选取最高分变体 → 整帧排序发布。

安全约束：帧处理串行化并只保留最新一帧，避免服务调用堆积；没有有效关节状态时直接
跳过 IK，不把空的机器人状态交给求解器；服务不可用、超时、返回空等异常一律按"不可行"
处理（保守失败），绝不因为校验环节故障而放行候选。
"""

from __future__ import annotations

from copy import deepcopy
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan

from .candidate_filter_policy import filter_candidate_array_by_reachability
from .candidate_gate_policy import CandidateGateConfig, evaluate_candidate_gate
from .candidate_motion_policy import JointMotionPolicyConfig, evaluate_joint_motion
from .candidate_precheck_policy import CandidatePrecheckConfig, evaluate_candidate_precheck
from .candidate_scoring_policy import CandidateScoringInput, score_candidate
from .candidate_target_policy import CandidateTargetPolicyConfig, build_candidate_target_variants
from .candidate_tf_adapter import transform_candidate_pose_to_target_frame
from .motion_feasibility_policy import evaluate_motion_feasibility
from .latest_only_work_queue import LatestOnlyWorkQueue
from .pose_variant_policy import PoseVariantConfig
from .visual_grasp_sequence import PoseTarget


def _pose_from_target(target: PoseTarget) -> Pose:
    """把内部位姿目标转换为 ROS 位姿消息。

    内部约定：位置为 (x, y, z) 三元组、单位 m；姿态为 (x, y, z, w) 四元数，顺序与 ROS
    字段一致，因此此处只做逐字段拷贝与 float 转换，不做任何坐标变换。
    """
    pose = Pose()
    pose.position.x = float(target.position[0])
    pose.position.y = float(target.position[1])
    pose.position.z = float(target.position[2])
    pose.orientation.x = float(target.orientation[0])
    pose.orientation.y = float(target.orientation[1])
    pose.orientation.z = float(target.orientation[2])
    pose.orientation.w = float(target.orientation[3])
    return pose


class CandidateIkFilterNode(Node):
    """抓取候选 IK / 碰撞可行性过滤节点。

    生命周期：构造时声明全部参数、建立 TF 缓冲与监听、创建逆解与状态有效性服务客户端、
    两个发布者和两个订阅者；``main`` 中用 3 线程多线程执行器自旋。节点不保存跨帧的抓取
    状态，处理完一帧立即发布结果，因此随时可以重启而不遗留状态。

    线程模型：候选回调与关节状态回调位于同一个可重入回调组，可以被并发调用；但耗时处理
    由"只保留最新一帧"的串行队列保护，同一时刻只有一个工作项在跑，处理期间到达的帧被
    合并丢弃，从而限制服务调用速率、保证下一帧永远基于最新输入。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_candidate_ik_filter")
        self._callback_group = ReentrantCallbackGroup()
        # ---- 话题、坐标系与服务 ----
        # 输入候选话题：上游视觉节点发布的抓取候选数组，位姿坐标系见消息 header.frame_id
        self.declare_parameter("input_topic", "/grasp/candidates")
        # 输出候选话题：过滤并排序后的候选数组，header 沿用输入帧
        self.declare_parameter("output_topic", "/grasp/filtered_candidates")
        # 输出计划话题：只含最优候选的抓取计划（接近点 + 抓取点），供下游执行器消费
        self.declare_parameter("output_plan_topic", "/grasp/filtered_plan")
        # 统一的求解与发布坐标系（默认机械臂基座系 base_link），所有位姿单位 m
        self.declare_parameter("target_frame", "base_link")
        # 逆解服务名（MoveIt 的 compute_ik）；该服务只求关节解，不产生任何运动
        self.declare_parameter("moveit_ik_service", "/compute_ik")
        # 关节状态话题：IK 的种子状态与关节位移代价的当前值，必须来自有反馈的关节源
        self.declare_parameter("joint_state_topic", "/rebotarm/visual_joint_states")
        # 是否调用状态有效性服务做碰撞/越限检查；关闭后仅凭 IK 有解判定可行，属调试手段
        self.declare_parameter("collision_check_enabled", True)
        # 状态有效性服务名（MoveIt 的 check_state_validity）
        self.declare_parameter("collision_check_service", "/check_state_validity")
        # 碰撞检查使用的规划组：必须包含夹爪，否则夹爪与目标/桌面的干涉查不出来
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        # 逆解使用的规划组名（只含手臂关节）
        self.declare_parameter("moveit_group_name", "arm")
        # 逆解的目标连杆（TCP 参考连杆），与 tcp_offset_xyz 配套解读
        self.declare_parameter("ee_frame_id", "end_link")
        # 服务等待与单次调用超时，单位 s；调小会让慢响应被误判为无解，调大会拖慢整帧
        self.declare_parameter("service_timeout_sec", 5.0)
        # ---- 候选位姿变体生成 ----
        # 位姿策略：preserve_candidate_pose 保留候选原始 6D 姿态；hybrid_geometry 只保留候选姿态
        # 的偏航角（丢弃俯仰/滚转）；official_geometry 用候选自身 x 轴作为接近轴；base_axis 完全
        # 使用固定姿态。带 _with_base_axis_fallback 后缀的取值会在前一策略之外再追加 base_axis 变体
        self.declare_parameter("pose_policy", "hybrid_geometry_with_base_axis_fallback")
        # base_axis 策略使用的固定末端姿态（四元数 x,y,z,w）。当前安装把上游 +X 工作区绕基座 Z
        # 旋转了 -90°，因此默认值是绕 Z 轴 -90° 的四元数；只在 base_axis 及其回退变体中生效
        self.declare_parameter(
            "fixed_grasp_orientation_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )
        # 基座系进给轴方向（单位向量）：接近点 = 抓取点沿该方向后退 base_pregrasp_distance_m，
        # 默认沿基座 -Y 进入，与上面旋转后的工作区朝向一致
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        # 接近点到抓取点的距离，单位 m：太小会侧向蹭到目标，太大会拉长接近行程与节拍
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        # 偏航角偏移列表，单位 rad；每个元素生成一组姿态变体，用于绕竖直轴尝试不同抓取朝向
        self.declare_parameter("orientation_yaw_offsets_rad", [0.0])
        # 抓取高度附加偏移列表，单位 m；每个元素生成一组变体，用于在目标高度方向微调
        self.declare_parameter("candidate_grasp_z_offsets_m", [0.0])
        # 每帧最多处理的候选数（下限被钳到 1）；超出部分直接丢弃，用于限制单帧 IK 调用量
        self.declare_parameter("max_candidates_per_frame", 20)
        # 置信度下限，无量纲 [0, 1]；低于该值的候选在生成位姿前就被丢弃
        self.declare_parameter("candidate_min_confidence", 0.0)
        # 过滤统计日志的最小间隔，单位 s；<= 0 表示每帧都打印
        self.declare_parameter("filter_stats_log_interval_sec", 5.0)
        # TCP 相对末端连杆的偏移（末端连杆局部坐标系，单位 m）；求逆解前会先从目标点减去该
        # 偏移，把"TCP 应到达的点"换算成"末端连杆应到达的位姿"
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        # 目标点在基座系下的平移修正，单位 m；用于补偿手眼/TCP 标定残差
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        # 接近点最低高度钳位，单位 m；低于该值会被抬高到该值，避免接近点插进桌面
        self.declare_parameter("candidate_pregrasp_min_z_m", 0.04)
        # 抓取点整体高度偏移，单位 m；与每个变体的高度偏移叠加
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        # ---- 几何闸门与打分权重 ----
        # 夹爪可信行程下限，单位 m；小于该开口认为抓不稳目标
        self.declare_parameter("candidate_min_jaw_width_m", 0.006)
        # 夹爪可信行程上限，单位 m；大于该开口认为夹爪张不到位（会被夹爪行程截断）
        self.declare_parameter("candidate_max_jaw_width_m", 0.085)
        # 抓取点最低允许高度，单位 m（基座系）；低于该值判为贴地或穿桌
        self.declare_parameter("candidate_min_grasp_z_m", 0.0)
        # 是否启用工作空间盒闸门；默认关闭，启用后抓取点必须落在下面的盒范围内
        self.declare_parameter("candidate_workspace_gate_enabled", False)
        # 工作空间盒最小角（基座系，单位 m）；沿用旧仓库 +X 工作区。
        self.declare_parameter("candidate_workspace_min_xyz", [0.18, -0.35, 0.0])
        # 工作空间盒最大角（基座系，单位 m）
        self.declare_parameter("candidate_workspace_max_xyz", [0.64, 0.35, 0.45])
        # 抓取点到目标中心的最大允许距离，单位 m；超出说明抓取点已偏离物体（深度或分割异常）
        self.declare_parameter("candidate_max_grasp_to_object_center_m", 0.15)
        # 关节位移在评分中的权重（每 rad 的扣分）；越大越偏好关节动作小的解
        self.declare_parameter("candidate_score_joint_distance_weight", 0.15)
        # joint6 变化量在评分中的权重（每 rad 的扣分）；腕部旋转代价单独加权，抑制绕线
        self.declare_parameter("candidate_score_joint6_weight", 0.35)
        # joint6 变化量上限，单位 rad（默认 1.5708 ≈ 90°）；超过直接判该解不可行，防止腕部大幅翻转
        self.declare_parameter("candidate_max_joint6_delta_rad", 1.5708)
        # 是否追加平行夹爪对称姿态变体：平行夹爪绕末端 x 轴转 180° 后仍是同一次抓取
        self.declare_parameter("candidate_joint6_symmetry_enabled", True)
        # 对称变体的旋转角，单位 rad（默认 π）；只有平行夹爪的 180° 对称才物理等价
        self.declare_parameter("candidate_joint6_symmetry_angle_rad", math.pi)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        # 最近一帧有效关节状态，作为 IK 种子；None 表示尚未收到可用反馈
        self._latest_joint_state: JointState | None = None
        # 关节状态缺失告警的去重标志，避免每帧刷屏
        self._warned_missing_joint_state = False
        # 只保留最新一帧的串行工作队列：处理耗时的帧运行期间到达的帧会被合并丢弃
        self._work_queue: LatestOnlyWorkQueue[GraspCandidateArray] = LatestOnlyWorkQueue()
        # 上次打印过滤统计的时刻（单调时钟，s），配合 filter_stats_log_interval_sec 限流
        self._last_filter_stats_log_at = 0.0
        # TF 缓冲与监听：用于把候选位姿从相机系等来源坐标系变换到 target_frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # 逆解客户端：仅用于可行性校验，绝不触发任何运动
        self._ik_client = self.create_client(
            GetPositionIK,
            str(self.get_parameter("moveit_ik_service").value),
            callback_group=self._callback_group,
        )
        # 状态有效性客户端：对 IK 解做碰撞/越限检查
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
            callback_group=self._callback_group,
        )
        # 过滤后的候选数组发布者
        self._candidates_pub = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        # 抓取计划发布者：只有最优候选的接近点/抓取点，是执行器真正消费的消息
        self._plan_pub = self.create_publisher(
            GraspPlan,
            str(self.get_parameter("output_plan_topic").value),
            10,
        )
        # 候选订阅：回调可重入，实际重活由下面的串行队列限制并发
        self.create_subscription(
            GraspCandidateArray,
            self._input_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        # 关节状态订阅：只缓存最新有效帧，供逆解与位移代价使用
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "candidate IK filter ready: "
            f"input={self._input_topic}, output={str(self.get_parameter('output_topic').value)}, "
            f"plan={str(self.get_parameter('output_plan_topic').value)}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        """读取三元组向量参数并转为 float；长度不是 3 时立即抛错，避免静默使用错参数。"""
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))

    def _tuple4(self, name: str) -> tuple[float, float, float, float]:
        """读取四元数参数并转为 float；长度不是 4 时立即抛错（四元数必须完整给出 x,y,z,w）。"""
        values = list(self.get_parameter(name).value)
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly 4 values")
        return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))

    def _on_joint_state(self, msg: JointState) -> None:
        """缓存最近一帧有效关节状态；空或不完整的帧只告警一次，不污染 IK 种子。"""
        if self._valid_joint_state(msg):
            self._latest_joint_state = msg
            self._warned_missing_joint_state = False
            return
        if not self._warned_missing_joint_state:
            self.get_logger().warn(
                "candidate IK filter ignored empty or incomplete joint state; "
                "waiting before calling MoveIt IK"
            )
            self._warned_missing_joint_state = True

    def _valid_joint_state(self, msg: JointState | None) -> bool:
        """判断关节状态能否作为 IK 种子。

        要求：消息非空、name 与 position 都非空、position 数量不少于 name 数量，且 name
        对应的 position 全为有限值。数量不足会让求解器拿到缺关节的机器人状态；NaN/Inf
        则会让逆解直接失败或返回不可预期的解，因此都必须在入口挡住。
        """
        if msg is None:
            return False
        if not msg.name or not msg.position:
            return False
        if len(msg.position) < len(msg.name):
            return False
        return all(math.isfinite(float(value)) for value in msg.position[: len(msg.name)])

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        """候选数组订阅回调：交给最新优先队列后串行处理。

        队列保证同一时刻只有一个工作项在跑，处理期间新到的帧只保留最新一帧，因此这里用
        while 循环把"当前帧 + 处理期间积压的最新帧"依次处理完，直到队列空闲。
        单帧内部异常只记录错误日志，不打断后续帧的处理。
        """
        work = self._work_queue.submit(msg, received_at=time.monotonic())
        while work is not None:
            started_at = time.monotonic()
            counts: dict[str, int] = {}
            try:
                counts = self._on_candidates_unlocked(work.item)
            except Exception as exc:
                self.get_logger().error(f"candidate IK filter frame failed: {exc}")
            completed_at = time.monotonic()
            # 单帧实际处理耗时（含服务调用等待），单位 ms
            processing_ms = (completed_at - started_at) * 1000.0
            # 出队即表示本帧处理结束；返回不为空说明处理期间又积压了更新的一帧
            work = self._work_queue.complete(completed_at=completed_at)
            self._log_filter_stats(
                processing_ms=processing_ms,
                pending_age_ms=0.0 if work is None else work.pending_age_ms,
                counts=counts,
            )

    def _log_filter_stats(
        self,
        *,
        processing_ms: float,
        pending_age_ms: float,
        counts: dict[str, int],
    ) -> None:
        """按 filter_stats_log_interval_sec 限流打印过滤统计。

        统计内容包括队列的收到/开始/完成/合并帧数与忙碌状态、本帧耗时、积压帧年龄，
        以及本帧各阶段的候选计数；限流是为了避免高频相机帧把日志刷爆。
        """
        now = time.monotonic()
        # 间隔 <= 0 视为不限流（每帧都打印）；否则未到间隔直接返回
        interval = max(0.0, float(self.get_parameter("filter_stats_log_interval_sec").value))
        if interval > 0.0 and now - self._last_filter_stats_log_at < interval:
            return
        self._last_filter_stats_log_at = now
        stats = self._work_queue.snapshot()
        self.get_logger().info(
            "candidate_filter_stats "
            f"received={stats.received} started={stats.started} completed={stats.completed} "
            f"coalesced={stats.coalesced} busy={str(stats.busy).lower()} "
            f"pending={str(stats.pending).lower()} processing_ms={processing_ms:.1f} "
            f"pending_age_ms={pending_age_ms:.1f} "
            f"input_candidates={counts.get('input_candidates', 0)} "
            f"precheck_passed={counts.get('precheck_passed', 0)} "
            f"geometry_variants_passed={counts.get('geometry_variants_passed', 0)} "
            f"ranked={counts.get('ranked', 0)}"
        )

    def _on_candidates_unlocked(self, msg: GraspCandidateArray) -> dict[str, int]:
        """对一帧候选完成"预检 → 变体 → IK/碰撞 → 打分排序"的全过程。

        方法名中的 unlocked 表示调用方（最新优先队列）已保证不会并发重入，因此内部不再
        加锁。返回各阶段计数，供统计日志使用：输入候选数、预检通过数、几何变体通过数、
        最终入选数。
        """
        if not msg.candidates:
            self._publish_filtered(msg, [])
            return {
                "input_candidates": 0,
                "precheck_passed": 0,
                "geometry_variants_passed": 0,
                "ranked": 0,
            }
        # 每条记录：(得分, 原始下标, 候选消息, (接近点, 抓取点), 变体标签, 可动性说明)
        ranked: list[tuple[float, int, object, tuple[PoseTarget, PoseTarget], str, str]] = []
        # 至少处理 1 个候选，防止参数被写成 0 或负数导致整帧被丢弃
        max_candidates = max(1, int(self.get_parameter("max_candidates_per_frame").value))
        input_candidates = min(len(msg.candidates), max_candidates)
        precheck_passed = 0
        geometry_variants_passed = 0
        for original_index, candidate in enumerate(msg.candidates[:max_candidates]):
            try:
                # 预检最便宜：置信度或夹爪宽度不合法时不必生成位姿，也不调用任何服务
                if not self._candidate_precheck_allows(candidate):
                    continue
                precheck_passed += 1
                best: tuple[float, tuple[PoseTarget, PoseTarget], str, str] | None = None
                variants = self._candidate_target_variants(msg, candidate.pose)
                # 逐变体做几何闸门、IK + 碰撞检查与可动性评估，保留该候选得分最高的变体
                for pregrasp, grasp, variant_label in variants:
                    if not self._candidate_gate_allows(candidate, grasp=grasp):
                        continue
                    geometry_variants_passed += 1
                    feasibility = evaluate_motion_feasibility(
                        pregrasp=pregrasp,
                        grasp=grasp,
                        variant_label=variant_label,
                        check_target=self._check_ik_and_collision,
                        motion_penalty=self._joint_motion_penalty,
                    )
                    if not feasibility.accepted or feasibility.motion_penalty is None:
                        continue
                    # 打分只依赖上游排序位置、变体标签与可动性代价，得分越高越优先执行
                    scoring = score_candidate(
                        CandidateScoringInput(
                            original_index=original_index,
                            variant_label=variant_label,
                            motion_penalty=feasibility.motion_penalty,
                        )
                    )
                    score_value = scoring.score
                    # 同分时保持先出现的变体（> 而不是 >=），排序结果稳定可复现
                    if best is None or float(score_value) > float(best[0]):
                        best = (float(score_value), (pregrasp, grasp), variant_label, feasibility.reason)
                if best is not None:
                    score_value, targets, label, motion_reason = best
                    ranked.append((score_value, original_index, candidate, targets, label, motion_reason))
                    self.get_logger().info(
                        f"candidate IK filter accepted candidate={original_index} {label}: "
                        f"score={score_value:.2f}, {motion_reason}"
                    )
            except Exception as exc:
                # 单个候选异常（例如 TF 查询失败）只丢弃该候选，不放弃整帧
                self.get_logger().warn(f"candidate IK filter rejected candidate: {exc}")
        self._publish_ranked(msg, ranked)
        return {
            "input_candidates": input_candidates,
            "precheck_passed": precheck_passed,
            "geometry_variants_passed": geometry_variants_passed,
            "ranked": len(ranked),
        }

    def _candidate_precheck_allows(self, candidate) -> bool:
        """候选级预检：置信度有限且不低于下限，夹爪宽度有限且落在可信行程内。"""
        result = evaluate_candidate_precheck(
            confidence=float(getattr(candidate, "confidence", 0.0)),
            jaw_width_m=float(getattr(candidate, "jaw_width", 0.0)),
            config=CandidatePrecheckConfig(
                min_confidence=float(self.get_parameter("candidate_min_confidence").value),
                min_jaw_width_m=float(self.get_parameter("candidate_min_jaw_width_m").value),
                max_jaw_width_m=float(self.get_parameter("candidate_max_jaw_width_m").value),
            ),
        )
        return result.accepted

    def _candidate_gate_allows(self, candidate, *, grasp: PoseTarget) -> bool:
        """几何闸门：夹爪宽度、抓取点最低高度，以及可选的工作空间盒检查。

        工作空间闸门默认关闭：关闭时既不判断坐标范围，也不查询 TF 求目标中心，省掉一次
        变换；开启时把候选自身位姿变换到 target_frame 当作目标中心，并用它与抓取点的
        距离判断抓取点是否已明显偏离物体。
        """
        try:
            workspace_enabled = bool(self.get_parameter("candidate_workspace_gate_enabled").value)
        except Exception:
            # 参数缺失或类型异常时按"不启用"处理，与默认配置保持一致
            workspace_enabled = False
        object_center_xyz = None
        # 闸门关闭时使用与参数默认值相同的占位范围（该分支下不会被用到）
        workspace_min_xyz = (-0.35, -0.64, 0.0)
        workspace_max_xyz = (0.35, -0.18, 0.45)
        max_grasp_to_object_center_m = 0.15
        if workspace_enabled:
            object_center_xyz = self._candidate_object_center_in_target_frame(candidate)
            workspace_min_xyz = self._tuple3("candidate_workspace_min_xyz")
            workspace_max_xyz = self._tuple3("candidate_workspace_max_xyz")
            max_grasp_to_object_center_m = float(
                self.get_parameter("candidate_max_grasp_to_object_center_m").value
            )
        result = evaluate_candidate_gate(
            jaw_width_m=float(getattr(candidate, "jaw_width", 0.0)),
            grasp_position_xyz=tuple(float(v) for v in grasp.position),
            object_center_xyz=object_center_xyz,
            config=CandidateGateConfig(
                min_jaw_width_m=float(self.get_parameter("candidate_min_jaw_width_m").value),
                max_jaw_width_m=float(self.get_parameter("candidate_max_jaw_width_m").value),
                min_grasp_z_m=float(self.get_parameter("candidate_min_grasp_z_m").value),
                workspace_gate_enabled=workspace_enabled,
                workspace_min_xyz=workspace_min_xyz,
                workspace_max_xyz=workspace_max_xyz,
                max_grasp_to_object_center_m=max_grasp_to_object_center_m,
            ),
        )
        if not result.accepted:
            self.get_logger().warn(f"candidate IK filter rejected reachable grasp: {result.reason}")
            return False
        return True

    def _candidate_safety_gate(self, candidate, *, grasp: PoseTarget) -> bool:
        """兼容入口：语义与几何闸门完全一致，仅转发调用，保证两条调用路径不会分叉。"""
        return CandidateIkFilterNode._candidate_gate_allows(self, candidate, grasp=grasp)

    def _candidate_object_center_in_target_frame(self, candidate) -> tuple[float, float, float] | None:
        """取候选位姿作为目标中心并变换到 target_frame；变换失败返回 None。

        候选消息没有独立的目标中心字段，这里用候选位姿近似目标中心，仅用于"抓取点是否
        离物体太远"这一粗粒度判断。
        """
        pose = getattr(candidate, "pose", None)
        if pose is None:
            return None
        source_frame = str(getattr(getattr(candidate, "header", None), "frame_id", "") or "")
        try:
            stamp = getattr(getattr(candidate, "header", None), "stamp", None)
            object_pose = self._transform_pose_to_target_frame(pose, source_frame, stamp)
        except Exception:
            return None
        return (
            float(object_pose.position.x),
            float(object_pose.position.y),
            float(object_pose.position.z),
        )

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp):
        """查询 TF（取最新可用时刻），超时 0.2 s。

        取不到变换时本方法会抛异常，由调用方按"无法校验"处理：候选级异常会丢弃该候选，
        目标中心变换失败则跳过依赖该变换的距离检查。
        """
        if stamp is None or (int(stamp.sec) == 0 and int(stamp.nanosec) == 0):
            raise RuntimeError("candidate has no sensor timestamp for TF lookup")
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time.from_msg(stamp),
            timeout=rclpy.duration.Duration(seconds=0.2),
        )

    def _transform_pose_to_target_frame(self, pose: Pose, source_frame: str, stamp) -> Pose:
        """把位姿从 source_frame 变换到 target_frame；两者相同或为空时返回原样副本。"""
        return transform_candidate_pose_to_target_frame(
            pose,
            source_frame=source_frame,
            target_frame=self._target_frame,
            stamp=stamp,
            lookup_transform=self._lookup_transform,
        )

    def _list_float_parameter(self, name: str) -> list[float]:
        """读取浮点数组参数（偏航角偏移、高度偏移等），统一转换为 float 列表。"""
        values = list(self.get_parameter(name).value)
        return [float(value) for value in values]

    def _candidate_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        """生成该候选全部待校验的 (接近点, 抓取点, 变体标签) 组合。

        先把候选位姿从消息声明的坐标系变换到 target_frame，再交给位姿策略按 pose_policy、
        偏航角偏移、高度偏移、TCP 偏移与平行夹爪对称配置展开变体；标签用于日志与打分，
        其中 yaw/z 下标对应参数列表下标。
        """
        source_frame = str(candidates.header.frame_id)
        grasp_pose = self._transform_pose_to_target_frame(pose, source_frame, candidates.header.stamp)
        position_xyz = (
            float(grasp_pose.position.x),
            float(grasp_pose.position.y),
            float(grasp_pose.position.z),
        )
        variants = build_candidate_target_variants(
            grasp_position_xyz=position_xyz,
            candidate_orientation_xyzw=(
                float(grasp_pose.orientation.x),
                float(grasp_pose.orientation.y),
                float(grasp_pose.orientation.z),
                float(grasp_pose.orientation.w),
            ),
            config=CandidateTargetPolicyConfig(
                pose_policy=str(self.get_parameter("pose_policy").value),
                fixed_grasp_orientation_xyzw=self._tuple4("fixed_grasp_orientation_xyzw"),
                base_approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                base_pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tuple3("tcp_offset_xyz"),
                target_base_offset_xyz=self._tuple3("target_base_offset_xyz"),
                pregrasp_min_z_m=float(self.get_parameter("candidate_pregrasp_min_z_m").value),
                grasp_base_z_offset_m=float(self.get_parameter("grasp_base_z_offset_m").value),
                orientation_yaw_offsets_rad=tuple(self._list_float_parameter("orientation_yaw_offsets_rad")),
                candidate_grasp_z_offsets_m=tuple(self._list_float_parameter("candidate_grasp_z_offsets_m")),
                pose_variant_config=PoseVariantConfig(
                    joint6_symmetry_enabled=bool(self.get_parameter("candidate_joint6_symmetry_enabled").value),
                    joint6_symmetry_angle_rad=float(self.get_parameter("candidate_joint6_symmetry_angle_rad").value),
                ),
            ),
        )
        return [(variant.pregrasp, variant.grasp, variant.label) for variant in variants]

    def _check_ik_and_collision(self, target: PoseTarget, label: str):
        """对单个目标位姿做完整可行性检查：逆解有解且状态有效才返回解，否则返回 None。

        返回值为逆解得到的机器人状态；碰撞检查关闭时只做逆解。
        """
        solution = self._solve_ik(target, label)
        if solution is None:
            return None
        if not self._check_state_validity(solution, label):
            return None
        return solution

    def _joint_motion_penalty(self, robot_state) -> tuple[float | None, str]:
        """计算逆解相对当前关节状态的位移代价，并拦截 joint6 过大的解。

        返回 (代价, 说明)；当 joint6 变化超过 candidate_max_joint6_delta_rad 时返回
        (None, 说明) 表示该解不可行——这类解虽然能到达目标位姿，但腕部需要转过很大的角度，
        通常说明还存在更省力的等效解（例如平行夹爪的 180° 对称姿态）。当前状态缺失或
        找不到公共关节名时返回 0 代价，即"无法比较就不惩罚"，避免因为反馈缺失而否决全部候选。
        """
        current = self._joint_positions_by_name(self._latest_joint_state)
        solution_joint_state = getattr(robot_state, "joint_state", None)
        target = self._joint_positions_by_name(solution_joint_state)
        # 只统计以 joint 开头的臂关节，夹爪等执行器关节不参与位移代价
        common_names = [name for name in current if name in target and name.startswith("joint")]
        if not common_names:
            return 0.0, "joint_delta=unknown"
        evaluation = evaluate_joint_motion(
            current_positions=current,
            target_positions=target,
            config=JointMotionPolicyConfig(
                joint_distance_weight=float(self.get_parameter("candidate_score_joint_distance_weight").value),
                joint6_weight=float(self.get_parameter("candidate_score_joint6_weight").value),
                max_joint6_delta_rad=float(self.get_parameter("candidate_max_joint6_delta_rad").value),
            ),
        )
        if not evaluation.accepted:
            self.get_logger().warn(
                "candidate IK filter rejected reachable grasp: "
                f"joint6 delta too large "
                f"({evaluation.joint6_delta:.3f}rad > "
                f"{float(self.get_parameter('candidate_max_joint6_delta_rad').value):.3f}rad)"
            )
            return None, evaluation.reason
        return evaluation.penalty, evaluation.reason

    def _joint_positions_by_name(self, joint_state: JointState | None) -> dict[str, float]:
        """把关节状态整理成 {关节名: 弧度} 字典，非有限值直接跳过。"""
        if joint_state is None:
            return {}
        names = list(getattr(joint_state, "name", []))
        positions = list(getattr(joint_state, "position", []))
        return {
            str(name): float(position)
            for name, position in zip(names, positions)
            if math.isfinite(float(position))
        }

    def _target_debug_text(self, target: PoseTarget) -> str:
        """把目标位姿格式化成日志文本（位置 3 位小数，单位 m；四元数 4 位小数）。"""
        return (
            f"target=({target.position[0]:.3f}, {target.position[1]:.3f}, {target.position[2]:.3f}), "
            f"orientation=({target.orientation[0]:.4f}, {target.orientation[1]:.4f}, "
            f"{target.orientation[2]:.4f}, {target.orientation[3]:.4f})"
        )

    def _solve_ik(self, target: PoseTarget, label: str):
        """调用逆解服务求一个目标位姿的关节解，失败返回 None。

        安全前置：服务可用且已有有效关节状态才发请求——机器人状态为空会让求解器在缺关节
        的情况下猜测，结果不可信。请求中显式关闭求解器内置的碰撞检查，因为碰撞判定由随后
        的状态有效性服务单独负责，两处检查重复且可能给出不一致结论。
        """
        # 服务未就绪：直接判不可行，不阻塞整帧
        if not self._ik_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn("candidate IK filter IK service unavailable")
            return None
        # 没有有效关节状态时不发请求，避免把空的机器人状态当作种子
        if not self._valid_joint_state(self._latest_joint_state):
            if not self._warned_missing_joint_state:
                self.get_logger().warn(
                    "candidate IK filter has no valid joint state yet; "
                    "skipping IK to avoid empty MoveIt RobotState"
                )
                self._warned_missing_joint_state = True
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = str(self.get_parameter("moveit_group_name").value)
        request.ik_request.ik_link_name = str(self.get_parameter("ee_frame_id").value)
        request.ik_request.robot_state.joint_state = deepcopy(self._latest_joint_state)
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self._target_frame
        request.ik_request.pose_stamped.pose = _pose_from_target(target)
        # 关闭求解器内置碰撞检查：碰撞由后面的状态有效性服务统一判定
        request.ik_request.avoid_collisions = False
        future = self._ik_client.call_async(request)
        # 在可重入回调组里同步轮询等待；deadline 至少 0.1 s，避免超时参数被写成 0 时立即放弃
        deadline = time.monotonic() + max(self._service_timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            # 超时只放弃本候选，不取消 future，也不影响其他候选
            self.get_logger().warn(
                f"candidate IK filter IK timed out for {label}: "
                f"{self._target_debug_text(target)}"
            )
            return None
        result = future.result()
        if result is None:
            self.get_logger().warn(f"candidate IK filter IK failed for {label}: empty service result")
            return None
        error_code = int(getattr(result.error_code, "val", 99999))
        # MoveIt 错误码 1 = SUCCESS；1 以外（含取不到字段时的 99999 兜底）一律视为失败
        if error_code != 1:
            self.get_logger().warn(
                f"candidate IK filter IK failed for {label}: error_code={error_code}, "
                f"{self._target_debug_text(target)}"
            )
            return None
        return getattr(result, "solution", None)

    def _check_state_validity(self, robot_state, label: str) -> bool:
        """调用状态有效性服务判断逆解是否碰撞/越限，True 表示通过。

        关闭 collision_check_enabled 时直接放行（仅调试用）。服务不可用、超时、返回空或
        valid=False 都按不通过处理：宁可漏掉可行解，也不放行可疑解。
        """
        if not bool(self.get_parameter("collision_check_enabled").value):
            return True
        if robot_state is None:
            return False
        if not self._state_validity_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn("candidate IK filter state validity service unavailable")
            return False
        request = GetStateValidity.Request()
        # 用包含夹爪的规划组检查，才能发现夹爪与目标/桌面的干涉
        request.group_name = str(self.get_parameter("collision_group_name").value)
        request.robot_state = robot_state
        future = self._state_validity_client.call_async(request)
        deadline = time.monotonic() + max(self._service_timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            # 超时按不通过处理，绝不在未确认无碰撞的情况下放行
            self.get_logger().warn(f"candidate IK filter state validity timed out for {label}")
            return False
        result = future.result()
        if result is None:
            self.get_logger().warn(f"candidate IK filter state validity failed for {label}: empty service result")
            return False
        valid = bool(getattr(result, "valid", False))
        if not valid:
            # 状态无效通常意味着碰撞或关节越限，需在日志里保留标签以便回溯是哪组变体
            self.get_logger().warn(f"candidate IK filter state validity failed for {label}: state invalid")
        return valid

    def _publish_filtered(
        self,
        original: GraspCandidateArray,
        reachable: list[bool],
        reachable_targets: list[tuple[PoseTarget, PoseTarget]] | None = None,
    ) -> None:
        """按可达标记过滤后发布候选数组，并发布由过滤结果组装的抓取计划。

        reachable 是与输入候选一一对应的布尔标记（本节点在无候选可处理时传入空列表）；
        reachable_targets 为空时，计划退化为候选原始位姿，不带接近点/抓取点。
        """
        filtered = filter_candidate_array_by_reachability(original, reachable)
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, reachable_targets or [])
        self._plan_pub.publish(plan)

    def _publish_ranked(
        self,
        original: GraspCandidateArray,
        ranked: list[tuple[float, int, object, tuple[PoseTarget, PoseTarget], str, str]],
    ) -> None:
        """按得分降序发布候选数组与最优抓取计划。

        排序键为 (-得分, 原始下标)：得分高者在前，同分时保持上游顺序，结果稳定可复现。
        计划中的 best_index 在列表非空时固定为 0（已排序），空列表时为 -1。
        """
        ranked = sorted(ranked, key=lambda item: (-float(item[0]), int(item[1])))
        filtered = GraspCandidateArray()
        filtered.header = original.header
        filtered.best_index = 0 if ranked else -1
        # 候选消息整体深拷贝：下游可能改写内容，不能与输入消息共用对象
        filtered.candidates = [deepcopy(item[2]) for item in ranked]
        targets = [item[3] for item in ranked]
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, targets)
        if ranked:
            score, original_index, _candidate, _targets, label, motion_reason = ranked[0]
            # 计划里的 reason 在有效时用于记录选择依据：原始下标、得分、变体标签与可动性说明
            plan.reason = (
                f"best_candidate original_index={original_index}, score={score:.2f}, "
                f"variant={label}, {motion_reason}"
            )
            self.get_logger().info(f"candidate IK filter best: {plan.reason}")
        self._plan_pub.publish(plan)

    def _plan_from_filtered(
        self,
        filtered: GraspCandidateArray,
        reachable_targets: list[tuple[PoseTarget, PoseTarget]],
    ) -> GraspPlan:
        """由过滤后的候选数组与对应的可达目标点组装抓取计划。

        best_index < 0 或候选为空时返回 valid=False 的计划（reason 说明原因），下游必须
        据此放弃抓取。有可达目标点时，计划头改用 target_frame 并写入该变体的接近点/抓取点；
        没有目标点时退化为候选原始位姿（接近点与抓取点相同）。
        """
        plan = GraspPlan()
        plan.header = filtered.header
        # 计划来源标识：下游执行器据此判断位姿已由本节点在 target_frame 中生成，无需再套用位姿策略
        plan.source = "candidate_ik_filter"
        if filtered.best_index < 0 or not filtered.candidates:
            plan.valid = False
            plan.reason = "no IK-reachable grasp candidates"
            return plan
        candidate = filtered.candidates[int(filtered.best_index)]
        plan.candidate = candidate
        if reachable_targets:
            pregrasp, grasp = reachable_targets[int(filtered.best_index)]
            plan.pregrasp_pose = _pose_from_target(pregrasp)
            plan.grasp_pose = _pose_from_target(grasp)
            # 位姿已在 target_frame 下生成，计划头必须同步改写，否则下游会按原坐标系解读
            plan.header.frame_id = self._target_frame
        else:
            # 没有可达目标点时只能退化为候选原始位姿，接近点与抓取点相同
            plan.grasp_pose = candidate.pose
            plan.pregrasp_pose = candidate.pose
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.reason = ""
        return plan


def main(args=None) -> None:
    """节点入口：用 3 线程的多线程执行器自旋，Ctrl+C 或外部关闭时正常退出。"""
    rclpy.init(args=args)
    node = CandidateIkFilterNode()
    # 3 线程：候选回调在等待 IK / 状态有效性服务返回期间，仍需处理关节状态与 TF 回调
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # 用户中断与外部关闭都按正常退出处理，不再向上抛异常
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # 仅在上下文仍有效时关闭，避免二次 shutdown 抛错
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

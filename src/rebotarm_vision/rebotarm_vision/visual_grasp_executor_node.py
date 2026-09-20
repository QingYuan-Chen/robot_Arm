from __future__ import annotations

from copy import deepcopy
import time
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan
from rebotarm_msgs.srv import ExecutePose, GraspGripper, PublishTrajectoryPreview, SetGripper

from .grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices
from .grasp_verification_policy import (
    GraspVerificationConfig,
    GraspVerificationInput,
    verify_grasp_after_lift,
)
from .grasp_preview_sender_node import (
    _transform_from_msg,
    apply_tcp_offset_to_pose,
    transform_pose_message,
)
from .gripper_quality import close_contact_success
from .gripper_policy import GripperPolicyConfig, resolve_gripper_command
from .message_freshness import is_message_fresh, message_age_sec
from .place_task_policy import PlaceTaskConfig, build_place_stages
from .retreat_policy import RetreatPolicyConfig
from .trajectory_recovery_policy import RecoveryConfig, recovery_decision_for_stage
from .visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets
from .visual_grasp_sequence import (
    PoseTarget,
    VisualGraspSequenceConfig,
    VisualGraspStage,
    build_visual_grasp_sequence,
)
from .visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step

# 视觉抓取执行节点：把"视觉给出的抓取计划"翻译成一条受门控的机械臂动作序列。
#
# 本节点是视觉链路的执行入口，本身不做感知，也不直接驱动硬件，只做编排。
#
# 输入
#   - 订阅 `input_topic`（默认 `/grasp/filtered_plan`，类型为抓取计划消息）：
#     上游已过滤/校验过的抓取计划，含接近点位姿、抓取点位姿、类别、置信度与夹爪开口宽度。
#   - 订阅 `candidates_topic`（默认 `/grasp/filtered_candidates`，类型为抓取候选数组消息）：
#     同一次规划的候选列表，用于失败后按顺序自动换候选重试。
#
# 输出（全部为服务调用，服务名前缀 `/{arm_namespace}`，默认命名空间 `rebotarm`）
#   - `/visual_grasp/execute`（Trigger）：收到请求后同步跑完整条抓取序列，返回值即最终结论。
#   - `/visual_grasp/stop`（Trigger）：请求中止，置运行标志为假并调用运动停止。
#   - 向下游调用 `/motion_execution/execute_pose`（末端位姿规划/执行）、
#     `/motion_execution/stop` 与 `/trajectory_stop`（急停）、`/safe_home`（回安全位）、
#     `/gripper/set`（夹爪位置控制）与 `/gripper/grasp`（力闭合抓取）。
#
# 典型阶段序列（由序列构建策略按参数生成）：
#   [open_gripper] → move_to_pregrasp → approach_grasp → close_gripper → lift
#   → [safe_retreat] → [safe_home] → [move_to_place → open_gripper_at_place → place_retreat]
#   方括号表示由参数开关控制的可选阶段。
#
# 安全设计（阅读时请重点注意）
#   1. `execution_mode` 决定一切动作是否真正下发：只有取值 execute / real 才视为"执行"；
#      其余（含默认 plan_only）下所有运动只做规划干跑、夹爪与回零命令一律跳过，
#      各段规划成功后由运动层一次性发布连续 RViz 预览；不会逐段重启动画。
#   2. 抓取计划具有时效性：`max_plan_age_sec` 之外的旧计划、时间戳未设置的计划一律拒收，
#      避免用几秒前的检测结果去驱动现在的机械臂。
#   3. 抓取点高度受 `min_grasp_z_m` 下限保护（在序列构建策略中校验），防止规划到桌面以下。
#   4. 每个阶段失败都会走恢复策略：先请求运动停止，再（可选）撤到接近点，
#      只有"允许自动重试 + 该阶段可重试 + 还有剩余候选"三者同时成立才会换候选继续。
#   5. 抬起阶段之后做抓取验证（接触 + 闭合行程，可选视觉抬升证据），验证不过即判本次抓取失败。
#
# 线程模型：全部回调注册在可重入回调组上，主函数使用三线程执行器。执行服务回调会长时间
# 阻塞（内部用 `rclpy.ok()` 与 `_running` 标志轮询等待），停止服务可在另一个线程里打断它。
# 坐标系：`target_frame`（默认 `base_link`）是下发给运动层的目标坐标系；计划消息里
# `header.frame_id` 与它不一致时用 TF 换算，查询超时 0.2 s、取最新可用变换。


def pose_to_target(pose: Pose) -> PoseTarget:
    """把位姿消息转成内部轻量元组表示（位置 3 元组 + 四元数 4 元组，单位均为 m / 无量纲）。"""

    return PoseTarget(
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def target_to_pose_stamped(target: PoseTarget, frame_id: str) -> PoseStamped:
    """把内部位姿目标打包成带坐标系的位姿消息。

    `frame_id` 即目标坐标系（本节点默认 `base_link`），下游运动层按该坐标系解释位置。
    时间戳刻意填 0（rclpy 的零时刻），表示"使用最新可用变换"，避免用本地时钟去要求
    运动层做时间对齐；位置单位为 m，姿态为四元数。
    """

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = float(target.position[0])
    msg.pose.position.y = float(target.position[1])
    msg.pose.position.z = float(target.position[2])
    msg.pose.orientation.x = float(target.orientation[0])
    msg.pose.orientation.y = float(target.orientation[1])
    msg.pose.orientation.z = float(target.orientation[2])
    msg.pose.orientation.w = float(target.orientation[3])
    return msg


class VisualGraspExecutorNode(Node):
    """抓取执行器节点：订阅抓取计划/候选，对外提供执行与中止两个服务。

    生命周期：进程启动即建立 TF 监听、六个下游服务客户端、两个订阅与两个服务；
    节点本身不保存"机械臂状态"，只保存最近一次有效计划与候选，以及本轮执行过程的
    快照（最近夹爪到位位置、是否检出接触、闭合行程、抬起起始高度），供失败诊断使用。

    回调模型：所有订阅与服务都在同一个可重入回调组，主线程池为三线程。抓取序列在
    `/visual_grasp/execute` 的回调里同步执行，期间由 `_running` 标志与 `rclpy.ok()`
    控制所有等待循环；`/visual_grasp/stop` 只需把该标志置假即可让序列在下一个检查点退出。

    安全语义：本类不对硬件做任何直接操作，所有动作都必须经过下游运动执行服务的
    规划与执行门控；执行模式不是 `execute`/`real` 时，运动命令降级为纯规划干跑，
    夹爪与回零命令被跳过，因此默认参数下不会让机械臂真实运动。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_visual_grasp_executor")
        self._callback_group = ReentrantCallbackGroup()

        # ── 输入话题与坐标系 ────────────────────────────────────────────────
        # 机械臂命名空间，用于拼接所有下游服务名；前导斜杠会被 strip 掉
        self.declare_parameter("arm_namespace", "rebotarm")
        # 抓取计划订阅话题；上游候选过滤节点的输出，必须与本节点收到的计划格式一致
        self.declare_parameter("input_topic", "/grasp/filtered_plan")
        # 候选数组订阅话题；仅用于失败后换候选重试，缺省时退化为只执行最优计划
        self.declare_parameter("candidates_topic", "/grasp/filtered_candidates")
        # 下发给运动层的目标坐标系；计划消息 frame_id 与之不同则先用 TF 换算
        self.declare_parameter("target_frame", "base_link")

        # ── 位姿换算与接近点构造 ────────────────────────────────────────────
        # TCP（夹持中心）相对末端法兰 end_link 的偏移（m），表达在末端自身坐标系中：
        # 先用目标姿态把它旋转到基座系，再从目标位姿里扣掉，得到法兰应到达的位置。
        # 本站实测量为 [-0.04, 0, 0]，即夹持中心位于法兰沿末端 X 轴前伸 4 cm 处
        # （与仿真模型里 end_link 下的 ee_site 位姿一致）
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        # 目标点整体平移补偿（m），用于补偿标定/安装残差；默认全 0 表示不补偿
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        # 接近点（预抓取点）额外抬高的 z 偏移（m），只有旧式 visual_pose 策略使用
        self.declare_parameter("pregrasp_base_z_offset_m", 0.05)
        # 抓取点额外的 z 偏移（m），同上，仅旧式策略使用
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        # 位姿生成策略：base_axis = 用固定姿态 + 基座接近轴现算接近点；
        # visual_pose/source_pose/legacy = 直接采用计划里的位姿并叠加上述 z 偏移
        self.declare_parameter("pose_policy", "base_axis")
        # 固定抓取姿态四元数 xyzw，默认绕 Z 转 -90°（本站工作区相对上游 +X 布局旋转而来），
        # 使夹爪闭合方向与目标摆放方向一致
        self.declare_parameter(
            "fixed_grasp_orientation_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )
        # 基座系下的接近方向单位向量（会归一化）：接近点 = 抓取点沿该轴反向退开一段距离。
        # 默认 [0, -1, 0] 表示从 -Y 侧进给
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        # 接近点与抓取点的距离（m），越大越保守，但要求工作空间更大
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        # 抓取点 z 下限（m），低于此值直接判序列构建失败，属桌面碰撞护栏
        self.declare_parameter("min_grasp_z_m", 0.0)
        # 闭合后抬升高度（m）；同时受安全撤退的 min_lift_z_m 抬高保护
        self.declare_parameter("lift_z_m", 0.08)

        # ── 夹爪开合与宽度自适应 ────────────────────────────────────────────
        # 是否在接近前先张开夹爪；张开会提前占用空间，狭窄场景可关掉
        self.declare_parameter("open_before_approach", False)
        # 固定张开宽度（m，指两指间距），auto_gripper_width 为真且检测到宽度时被覆写
        self.declare_parameter("open_position_m", 0.09)
        # 固定闭合宽度（m）；同上，可能被自适应结果覆写
        self.declare_parameter("close_position_m", 0.025)
        # 闭合阶段最大力矩（N·m），是实际夹持力旋钮（越大越紧、越容易顶坏目标）
        self.declare_parameter("close_max_effort", 0.4)
        # 是否按检测到的目标宽度自动推算张开/闭合宽度；关闭或未测到宽度时用上下两行的固定值
        self.declare_parameter("auto_gripper_width", True)
        # 张开宽度在检测宽度基础上额外留的余量（m），默认 0 表示刚好等于检测宽度
        self.declare_parameter("open_clearance_m", 0.0)
        # 闭合宽度比检测宽度收窄的量（m），保证闭合时对目标有预压
        self.declare_parameter("close_margin_m", 0.012)
        # 自适应张开宽度下限（m），防止算出的开口过小夹不住
        self.declare_parameter("min_open_position_m", 0.035)
        # 自适应张开宽度上限（m），受夹爪行程限制
        self.declare_parameter("max_open_position_m", 0.09)
        # 自适应闭合宽度下限（m），0 附近即完全闭合
        self.declare_parameter("min_close_position_m", 0.006)
        # 自适应闭合宽度上限（m），防止闭合指令等于张开指令而夹不住
        self.declare_parameter("max_close_position_m", 0.08)
        # 是否按统一上下限约束夹持力矩
        self.declare_parameter("auto_gripper_effort", True)
        # 力矩下限（N·m），太小会夹不住、滑落
        self.declare_parameter("min_gripper_effort", 0.22)
        # 力矩上限（N·m），太大会压坏目标或让夹爪堵转报警
        self.declare_parameter("max_gripper_effort", 0.60)
        # 允许抓取的最大目标宽度（m），超过直接判策略不允许抓（不算夹爪行程上限，留了余量）
        self.declare_parameter("max_allowed_grasp_width_m", 0.085)

        # ── 位置闭合的"接触"判据（无接触传感器，靠行程与位移推断）────────────
        # 是否启用"闭合未到位但疑似夹到"的放宽判据
        self.declare_parameter("close_contact_success_enabled", True)
        # 允许停目标外侧的余量（m）：实际到位 >= 目标 + 该余量即认为被目标挡住
        self.declare_parameter("close_contact_margin_m", 0.004)
        # 相对上次张开位置的最小闭合行程（m），行程不足说明没夹到东西
        self.declare_parameter("close_contact_min_closure_delta_m", 0.015)

        # ── 力闭合抓取服务（堵转推断接触 + 有界保持）────────────────────────
        # close_gripper 阶段是否改用专用抓取服务而非普通位置服务
        self.declare_parameter("gripper_grasp_enabled", True)
        # 闭合阶段力矩（N·m）；硬件层会夹到 [0.05, 1.0]
        self.declare_parameter("gripper_grasp_close_force", 0.4)
        # 闭合超时（s），超时未堵转即判失败
        self.declare_parameter("gripper_grasp_timeout_sec", 8.0)
        # 最短闭合时间（s），防止刚起步速度未建立就被误判为接触
        self.declare_parameter("gripper_grasp_min_close_time_sec", 0.08)
        # 判定"已停住"的角速度阈值（rad/s），低于它才算堵转
        self.declare_parameter("gripper_grasp_velocity_threshold", 0.04)
        # 判定接触所需的最小闭合行程（m）
        self.declare_parameter("gripper_grasp_min_closure_distance_m", 0.006)

        # ── 抓取后安全撤退 ─────────────────────────────────────────────────
        # 是否在抬起后额外撤到远离目标的方向
        self.declare_parameter("safe_retreat_enabled", True)
        # 撤退前的最低抬升高度（m），与 lift_z_m 取较大者，保证先离台再平移
        self.declare_parameter("safe_retreat_min_lift_z_m", 0.12)
        # 撤退平移距离（m）
        self.declare_parameter("safe_retreat_distance_m", 0.06)
        # 撤退方向（基座系，会归一化）；默认沿 +Y 并向上 0.5，即斜向后上方退开
        self.declare_parameter("safe_retreat_axis_xyz", [0.0, 1.0, 0.5])
        # 是否在序列末尾回安全位（默认关，避免未经授权的大范围回零动作）
        self.declare_parameter("safe_home_after_grasp", False)

        # ── 超时与阶段间停顿 ───────────────────────────────────────────────
        # 等待下游服务可用的超时（s）
        self.declare_parameter("service_timeout_sec", 20.0)
        # move_to_pregrasp 之后的稳定等待（s）
        self.declare_parameter("pregrasp_wait_sec", 0.5)
        # approach_grasp 之后的稳定等待（s）
        self.declare_parameter("approach_wait_sec", 0.2)
        # 是否真正下发夹爪命令；计划模式或仿真下会被强制跳过
        self.declare_parameter("execute_gripper", True)
        # 夹爪开/合阶段的等待（s）
        self.declare_parameter("gripper_wait_sec", 1.0)
        # lift 与 safe_retreat 阶段的等待（s）
        self.declare_parameter("lift_wait_sec", 0.5)
        # 单次运动执行的服务端超时（s），会随请求一起下发，因此要覆盖规划耗时
        self.declare_parameter("motion_result_timeout_sec", 45.0)

        # ── 执行模式与速度缩放 ─────────────────────────────────────────────
        # plan_only = 只规划干跑（默认）；execute/real = 真正下发轨迹与夹爪命令
        self.declare_parameter("execution_mode", "plan_only")
        # 常规移动的速度缩放，无量纲 (0, 1]；越小越慢越安全
        self.declare_parameter("move_velocity_scaling", 0.10)
        # 接近段速度缩放；比常规更慢，因为此时离目标与台面最近
        self.declare_parameter("approach_velocity_scaling", 0.04)
        # 抬起与撤退段速度缩放；比接近段稍快，兼顾"已夹住"与效率
        self.declare_parameter("lift_velocity_scaling", 0.08)
        # 加速度缩放，所有阶段共用
        self.declare_parameter("acceleration_scaling", 0.08)
        # plan_only 阶段间诊断等待（s）；默认 0，连续预览无需人为停顿
        self.declare_parameter("plan_only_stage_pause_sec", 0.0)

        # ── 到接近点后刷新计划 ─────────────────────────────────────────────
        # 是否在到达接近点后等待更新版本的抓取计划（近距离视觉更准）
        self.declare_parameter("refresh_plan_at_pregrasp_enabled", True)
        # 刷新是否强制：开启且等不到新计划时本次尝试判失败（而不是沿用旧计划）
        self.declare_parameter("refresh_plan_at_pregrasp_required", True)
        # 等待新计划的最长时间（s）
        self.declare_parameter("refresh_plan_timeout_sec", 1.0)

        # ── 接近段视觉伺服（小步逼近 + 用新计划纠偏）──────────────────────
        # 是否用迭代小步替代一次到位的 approach_grasp
        self.declare_parameter("approach_visual_servo_enabled", False)
        # 最大迭代步数，至少为 1
        self.declare_parameter("approach_visual_servo_max_iterations", 5)
        # 单步最大位移（m），限制每步风险
        self.declare_parameter("approach_visual_servo_max_step_m", 0.02)
        # 位置误差容差（m），误差小于它就认为已到位
        self.declare_parameter("approach_visual_servo_position_tolerance_m", 0.008)
        # 是否要求每一步都基于更新的计划；关掉后允许沿用最近一次计划
        self.declare_parameter("approach_visual_servo_require_fresh_plan", True)

        # ── 失败恢复与抓取验证 ─────────────────────────────────────────────
        # 是否允许失败后自动换下一个候选重试（默认关，保持"一次命令一次动作"）
        self.declare_parameter("auto_retry_enabled", False)
        # 自动重试的候选数上限
        self.declare_parameter("auto_retry_max_attempts", 3)
        # 重试前是否先撤到接近点（避免贴着目标换位形）
        self.declare_parameter("safe_retreat_before_retry", True)
        # 是否在抬起后做抓取成功性验证
        self.declare_parameter("grasp_verification_enabled", True)
        # 判定"确实夹住"所需的最小闭合行程（m）
        self.declare_parameter("grasp_verification_min_closure_distance_m", 0.006)
        # 是否要求检出接触，缺失即判验证失败
        self.declare_parameter("grasp_verification_require_contact", True)
        # 是否引入视觉抬升证据（用最新计划的目标高度变化佐证物体被抬起）
        self.declare_parameter("visual_lift_check_enabled", False)
        # 视觉抬升证据的最小高度变化（m）
        self.declare_parameter("visual_lift_min_delta_m", 0.03)

        # ── 抓取后放置（可选）──────────────────────────────────────────────
        # 是否在抓取成功后继续执行放置序列
        self.declare_parameter("place_after_grasp_enabled", False)
        # 放置点位置（基座系，m）
        self.declare_parameter("place_position_xyz", [-0.20, -0.20, 0.25])
        # 放置点姿态四元数 xyzw，默认同样是绕 Z 转 -90°，与抓取姿态保持一致
        self.declare_parameter(
            "place_orientation_xyzw",
            [0.0, 0.0, 0.0, 1.0],
        )
        # 放置点松开夹爪的开口宽度（m）
        self.declare_parameter("place_open_position_m", 0.08)
        # 放置点松开夹爪的力矩（N·m）
        self.declare_parameter("place_open_max_effort", 0.25)
        # 松爪后向上撤开的高度（m）
        self.declare_parameter("place_retreat_z_m", 0.06)

        # ── 执行前预检与计划时效 ───────────────────────────────────────────
        # 真正执行前先用 execute=False 走一次规划预检，规划不通过就不执行
        self.declare_parameter("trajectory_precheck_enabled", True)
        # 抓取计划的最大允许时延（s），超时视为过期并拒收
        self.declare_parameter("max_plan_age_sec", 1.0)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._candidates_topic = str(self.get_parameter("candidates_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self._target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self._pregrasp_base_z_offset_m = float(self.get_parameter("pregrasp_base_z_offset_m").value)
        self._grasp_base_z_offset_m = float(self.get_parameter("grasp_base_z_offset_m").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self._motion_result_timeout_sec = float(self.get_parameter("motion_result_timeout_sec").value)
        self._max_plan_age_sec = float(self.get_parameter("max_plan_age_sec").value)
        self._execution_mode = str(self.get_parameter("execution_mode").value).strip().lower()
        # 阶段名 → 该阶段成功后的固定等待（s）。等待只用于让机械/夹爪稳定，未列出的阶段不等待；
        # 具体时长来自参数；plan_only 不需要机械稳定等待，只使用其专用诊断等待参数
        self._stage_waits = {
            "move_to_pregrasp": float(self.get_parameter("pregrasp_wait_sec").value),
            "approach_grasp": float(self.get_parameter("approach_wait_sec").value),
            "close_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "open_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "lift": float(self.get_parameter("lift_wait_sec").value),
            "safe_retreat": float(self.get_parameter("lift_wait_sec").value),
        }

        # ── 运行状态 ───────────────────────────────────────────────────────
        self._latest_plan: GraspPlan | None = None  # 最近一次通过校验的抓取计划
        self._last_plan_rejection: str = ""  # 服务诊断：最新被丢弃计划的原因，不参与安全判定
        self._latest_candidates: GraspCandidateArray | None = None  # 最近一次非空候选数组
        self._plan_revision = 0  # 计划版本号，每次接受新计划自增；用于判断"是否来了更新的计划"
        self._last_gripper_reached_position: float | None = None  # 最近一次张爪实际到位开口（m），用于估算闭合行程
        self._last_grasp_contact_detected = False  # 最近一次抓取服务是否检出接触（堵转推断）
        self._last_grasp_closure_distance_m = 0.0  # 本次闭合行程（m）= 张爪到位值 - 抓取到位值，下限截到 0
        self._last_lift_start_z_m: float | None = None  # 抬起阶段起点 z（m），用作视觉抬升证据的基准
        self._retry_retreat_stage: VisualGraspStage | None = None  # 已完成的接近点阶段，重试前撤回到这里
        self._run_counter = 0  # 累计执行次数，只增不减，用于给每轮日志编号
        self._current_run_id = 0  # 当前执行轮次编号
        self._current_attempt_index = 0  # 当前尝试序号（从 1 开始），仅用于日志
        self._current_candidate_index = -1  # 当前候选下标；-1 表示用的上游最优计划而非候选重试
        self._current_attempt_plan: GraspPlan | None = None  # 本轮尝试使用的计划，失败时用于打印快照
        self._preview_start_joint_state: JointState | None = None  # 只在本轮纯规划中传递上一段轨迹终点
        self._preview_trajectories = []  # 本轮纯规划的各段结果；全部成功后才一次发布给 RViz
        self._running = False  # 运行标志：所有等待循环的取消点，停止服务与异常都会把它置假
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 下游服务客户端（全部挂在可重入回调组上）────────────────────────
        self._execute_pose_client = self.create_client(
            ExecutePose,
            f"/{self._arm_namespace}/motion_execution/execute_pose",
            callback_group=self._callback_group,
        )
        self._publish_preview_client = self.create_client(
            PublishTrajectoryPreview,
            f"/{self._arm_namespace}/motion_execution/publish_trajectory_preview",
            callback_group=self._callback_group,
        )
        self._motion_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            callback_group=self._callback_group,
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        self._safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
            callback_group=self._callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            f"/{self._arm_namespace}/gripper/set",
            callback_group=self._callback_group,
        )
        self._gripper_grasp_client = self.create_client(
            GraspGripper,
            f"/{self._arm_namespace}/gripper/grasp",
            callback_group=self._callback_group,
        )
        # 队列深度 10：计划/候选都是"最新值优先"语义，积压没有意义，_on_plan 会自行丢弃过期消息
        self.create_subscription(GraspPlan, self._input_topic, self._on_plan, 10, callback_group=self._callback_group)
        self.create_subscription(
            GraspCandidateArray,
            self._candidates_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/execute", self._execute_visual_grasp, callback_group=self._callback_group)
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/stop", self._stop_visual_grasp, callback_group=self._callback_group)
        self.get_logger().info(
            "visual grasp executor ready: "
            f"input={self._input_topic}, namespace=/{self._arm_namespace}, target_frame={self._target_frame}, "
            "motion_execution=/motion_execution/execute_pose"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = self._tuple_n(name, 3)
        return (values[0], values[1], values[2])

    def _tuple_n(self, name: str, expected_len: int) -> tuple[float, ...]:
        values = list(self.get_parameter(name).value)
        if len(values) != expected_len:
            raise ValueError(f"{name} must contain exactly {expected_len} values")
        return tuple(float(value) for value in values)

    def _on_plan(self, plan: GraspPlan) -> None:
        """抓取计划订阅回调：只缓存"有效且未过期"的计划。

        无效计划（`valid` 为假）直接丢弃，不覆盖已有缓存；通过校验的计划做深拷贝保存，
        因为消息对象在回调返回后可能被中间件复用。每接受一次就自增 `_plan_revision`，
        该版本号是判断"是否收到更新计划"的唯一依据（接近点刷新与视觉伺服都依赖它）。
        """

        if not plan.valid:
            self._last_plan_rejection = f"invalid grasp plan: {plan.reason or 'upstream reported invalid'}"
            return
        if not self._plan_is_fresh(plan):
            age = message_age_sec(plan.header.stamp, now_ns=int(self.get_clock().now().nanoseconds))
            self._last_plan_rejection = (
                f"grasp plan expired on arrival: age_sec={age}, max_plan_age_sec={self._max_plan_age_sec}"
            )
            return
        self._latest_plan = deepcopy(plan)
        self._last_plan_rejection = ""
        self._plan_revision += 1

    def _plan_is_fresh(self, plan: GraspPlan | None) -> bool:
        """判断计划是否可作为执行依据：时间戳必须已设置，且时延不超过 `max_plan_age_sec`。

        时间戳为 0（未设置）一律视为不新鲜；允许轻微的"未来时间戳"以容忍时钟偏差。
        拒绝时打印 age 与阈值，便于排查"有检测但执行器说没有计划"的问题。
        """

        if plan is None:
            return False
        header = getattr(plan, "header", None)
        stamp = getattr(header, "stamp", None)
        now_ns = int(self.get_clock().now().nanoseconds)
        age_sec = message_age_sec(
            stamp,
            now_ns=now_ns,
        )
        if not is_message_fresh(
            stamp,
            now_ns=now_ns,
            max_age_sec=self._max_plan_age_sec,
        ):
            self.get_logger().warn(
                "visual grasp executor rejected stale or unset plan: "
                f"age_sec={age_sec} max_plan_age_sec={self._max_plan_age_sec}"
            )
            return False
        return True

    def _on_candidates(self, candidates: GraspCandidateArray) -> None:
        """候选数组订阅回调：只保留最近一次非空候选，空数组不会清掉已有结果。"""

        if candidates.candidates:
            self._latest_candidates = deepcopy(candidates)

    def _execute_visual_grasp(self, _request, response):
        """`/visual_grasp/execute` 服务回调：同步执行整条抓取序列并返回最终结论。

        请求内容被忽略（Trigger 无字段），执行对象是缓存的最近计划。返回的 `success`
        表示"整条序列走完且抓取验证通过"，`message` 为英文结果说明。

        进入前有两道门：已有任务在跑（`_running`）直接拒绝，避免两个抓取序列并发；
        没有新鲜有效的计划也拒绝，避免用过期的检测结果驱动机械臂。进入后按候选顺序
        逐个尝试，每次尝试都会重建完整阶段序列（含可选的放置阶段）。失败时调用恢复
        策略决定是"停止 + 安全撤退 + 换候选继续"还是"停止 + 直接失败返回"。
        无论成功、失败还是抛异常，`finally` 都会清空本轮计划快照并复位运行标志；
        异常路径额外请求一次运动停止，保证不会把机械臂留在运动状态。
        """

        if self._running:
            response.success = False
            response.message = "visual grasp already running"
            return response
        if self._latest_plan is None:
            response.success = False
            response.message = self._last_plan_rejection or "no grasp plan received yet"
            return response
        if not self._plan_is_fresh(self._latest_plan):
            age = message_age_sec(
                self._latest_plan.header.stamp, now_ns=int(self.get_clock().now().nanoseconds)
            )
            response.success = False
            response.message = f"cached grasp plan expired: age_sec={age}, max_plan_age_sec={self._max_plan_age_sec}"
            return response
        self._running = True
        self._run_counter += 1
        self._current_run_id = self._run_counter
        try:
            self._log_diagnostic("detect", "ok", f"input_topic={self._input_topic}, plan_revision={self._plan_revision}")
            attempts = self._candidate_plans_for_attempts()
            if not attempts:
                response.success = False
                response.message = "no candidate attempts available"
                self._log_diagnostic("filter", "fail", response.message)
                return response
            self._log_diagnostic("filter", "ok", f"attempts={len(attempts)}")
            for attempt_index, (candidate_index, plan) in enumerate(attempts):
                self._preview_start_joint_state = None
                self._preview_trajectories = []
                self._current_attempt_index = attempt_index + 1
                self._current_candidate_index = int(candidate_index)
                self._current_attempt_plan = deepcopy(plan)
                self.get_logger().info(
                    f"{self._diagnostic_prefix('attempt')} start: "
                    f"attempt={attempt_index + 1}/{len(attempts)}, candidate={candidate_index}"
                )
                self._log_plan_snapshot(plan)
                # 每轮尝试都把上一轮的抓取证据清零，防止上一轮的结果误判本轮
                self._last_grasp_contact_detected = False
                self._last_grasp_closure_distance_m = 0.0
                self._last_lift_start_z_m = None
                self._retry_retreat_stage = None
                stages = self._append_place_stages(self._build_sequence_from_plan(plan))
                ok, message, failed_stage = self._execute_stages(stages)
                if ok:
                    if not self._execution_enabled():
                        ok, message = self._publish_preview_sequence()
                        if not ok:
                            response.success = False
                            response.message = f"preview publication failed: {message}"
                            self._log_diagnostic("preview", "fail", message)
                            return response
                    response.success = True
                    response.message = "visual grasp sequence finished"
                    self._log_diagnostic("result", "success", response.message)
                    return response
                # 走到这里说明某个阶段失败：先按恢复策略收拾现场，再决定是否换候选
                remaining_attempts = len(attempts) - attempt_index - 1
                decision = recovery_decision_for_stage(
                    failed_stage,
                    attempt_index=attempt_index,
                    remaining_attempts=remaining_attempts,
                    config=RecoveryConfig(
                        auto_retry_enabled=bool(self.get_parameter("auto_retry_enabled").value),
                        safe_retreat_before_retry=bool(self.get_parameter("safe_retreat_before_retry").value),
                    ),
                )
                if decision.request_stop:
                    self._request_stop()
                if decision.request_safe_retreat:
                    self._request_retry_retreat()
                if decision.retry:
                    self.get_logger().warn(f"{decision.reason}: {message}")
                    continue
                response.success = False
                response.message = f"{failed_stage} failed: {message}"
                self._log_failure_snapshot(failed_stage, message)
                return response
            # 循环体每条路径都会 return（成功、失败或换候选继续），能走到这里说明没有可执行的尝试
            response.success = True
            response.message = "visual grasp sequence finished"
            self._log_diagnostic("result", "success", response.message)
            return response
        except Exception as exc:
            # 任何未预料的异常都必须先停运动再返回，避免机械臂停在未知状态
            self._request_stop()
            response.success = False
            response.message = f"visual grasp failed: {exc}"
            self._log_failure_snapshot("executor", str(exc))
            return response
        finally:
            self._current_attempt_plan = None
            self._preview_start_joint_state = None
            self._preview_trajectories = []
            self._running = False

    def _stop_visual_grasp(self, _request, response):
        """`/visual_grasp/stop` 服务回调：请求中止当前序列。

        只置运行标志为假并请求运动停止，不做等待：真正的序列退出发生在执行回调的
        下一个检查点（阶段后的等待或服务结果等待循环）。即使当前没有任务在跑，
        也照常返回成功并发出停止请求，属于幂等的"保险动作"。
        """

        self._running = False
        self._request_stop()
        response.success = True
        response.message = "visual grasp stop requested"
        return response

    def _build_sequence_from_plan(self, plan: GraspPlan) -> list[VisualGraspStage]:
        """按抓取计划与当前参数生成完整阶段序列（不含可选的放置阶段）。

        流程：先算接近点/抓取点（位姿策略），再让夹爪策略根据检测宽度解析出张开、闭合宽度
        与力矩（可能直接拒绝抓取），最后交给序列构建策略生成阶段列表。若策略判定"目标
        过宽不可抓"，序列构建会抛 ValueError，由上游执行回调的异常分支统一处理。
        """

        pregrasp, grasp = self._build_motion_targets(plan)
        gripper_command = resolve_gripper_command(
            jaw_width_m=self._detected_jaw_width(plan),
            object_length_m=self._detected_object_length(plan),
            class_name=str(getattr(plan.candidate, "class_name", "") or ""),
            config=GripperPolicyConfig(
                auto_width=bool(self.get_parameter("auto_gripper_width").value),
                auto_effort=bool(self.get_parameter("auto_gripper_effort").value),
                default_open_width_m=float(self.get_parameter("open_position_m").value),
                default_close_width_m=float(self.get_parameter("close_position_m").value),
                default_max_effort=float(self.get_parameter("close_max_effort").value),
                open_clearance_m=float(self.get_parameter("open_clearance_m").value),
                close_margin_m=float(self.get_parameter("close_margin_m").value),
                min_open_width_m=float(self.get_parameter("min_open_position_m").value),
                max_open_width_m=float(self.get_parameter("max_open_position_m").value),
                min_close_width_m=float(self.get_parameter("min_close_position_m").value),
                max_close_width_m=float(self.get_parameter("max_close_position_m").value),
                min_effort=float(self.get_parameter("min_gripper_effort").value),
                max_effort=float(self.get_parameter("max_gripper_effort").value),
                max_allowed_width_m=float(self.get_parameter("max_allowed_grasp_width_m").value),
            ),
        )
        config = VisualGraspSequenceConfig(
            open_before_approach=bool(self.get_parameter("open_before_approach").value),
            open_position_m=float(self.get_parameter("open_position_m").value),
            close_position_m=float(self.get_parameter("close_position_m").value),
            close_max_effort=float(self.get_parameter("close_max_effort").value),
            lift_z_m=float(self.get_parameter("lift_z_m").value),
            min_grasp_z_m=float(self.get_parameter("min_grasp_z_m").value),
            auto_gripper_width=bool(self.get_parameter("auto_gripper_width").value),
            detected_jaw_width_m=self._detected_jaw_width(plan),
            open_clearance_m=float(self.get_parameter("open_clearance_m").value),
            close_margin_m=float(self.get_parameter("close_margin_m").value),
            min_open_position_m=float(self.get_parameter("min_open_position_m").value),
            max_open_position_m=float(self.get_parameter("max_open_position_m").value),
            min_close_position_m=float(self.get_parameter("min_close_position_m").value),
            max_close_position_m=float(self.get_parameter("max_close_position_m").value),
            gripper_command=gripper_command,
            retreat_policy=RetreatPolicyConfig(
                enabled=bool(self.get_parameter("safe_retreat_enabled").value),
                min_lift_z_m=float(self.get_parameter("safe_retreat_min_lift_z_m").value),
                retreat_distance_m=float(self.get_parameter("safe_retreat_distance_m").value),
                retreat_axis_xyz=self._tuple3("safe_retreat_axis_xyz"),
            ),
            include_safe_home=bool(self.get_parameter("safe_home_after_grasp").value),
        )
        return build_visual_grasp_sequence(pregrasp, grasp, config)

    def _candidate_plans_for_attempts(self) -> list[tuple[int, GraspPlan]]:
        """组装本轮执行的尝试序列，元素为 (候选下标, 计划)。

        第 0 项永远是上游最优计划本身，候选下标记为 -1；若候选数组可用且启用了自动重试，
        再按重试策略给出的顺序追加其它候选（跳过最优候选本身，避免重复执行同一目标）。
        重试策略关闭时排序函数只返回最优下标，而这里会把它过滤掉，结果仍只有第 0 项，
        等价于"只执行一次"。
        """

        if self._latest_plan is None:
            return []
        attempts: list[tuple[int, GraspPlan]] = [(-1, deepcopy(self._latest_plan))]
        candidates = self._latest_candidates
        if candidates is None or not candidates.candidates:
            return attempts
        indices = ordered_candidate_indices(
            candidate_count=len(candidates.candidates),
            best_index=int(candidates.best_index),
            failed_indices=set(),
            config=RetryPolicyConfig(
                enabled=bool(self.get_parameter("auto_retry_enabled").value),
                max_attempts=int(self.get_parameter("auto_retry_max_attempts").value),
            ),
        )
        for index in indices:
            if index == int(candidates.best_index):
                continue
            attempts.append((index, self._plan_from_candidate(candidates, index)))
        return attempts

    def _plan_from_candidate(self, candidates: GraspCandidateArray, index: int) -> GraspPlan:
        """把候选数组中的第 index 个候选包装成可直接执行的抓取计划。

        候选本身只给一个 TCP 位姿，没有独立接近点，因此这里把接近点与抓取点都设为该位姿；
        真正的进给路径由下游运动层规划决定，不代表本节点会走一条直线接近段。
        计划来源标记为 `visual_grasp_executor_retry`，方便在日志里区分"上游计划"与"重试计划"。
        """

        candidate = candidates.candidates[int(index)]
        plan = GraspPlan()
        plan.header = candidates.header
        plan.candidate = deepcopy(candidate)
        plan.pregrasp_pose = deepcopy(candidate.pose)
        plan.grasp_pose = deepcopy(candidate.pose)
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.source = "visual_grasp_executor_retry"
        plan.reason = ""
        return plan

    def _execute_stages(self, stages: list[VisualGraspStage]) -> tuple[bool, str, str]:
        """顺序执行阶段列表，返回 (是否全部成功, 说明, 失败阶段名)。

        阶段列表在运行中可能被替换：到达接近点后若拿到更新的计划，就用新计划重建剩余阶段；
        若启用了视觉伺服逼近，则删掉原来的 approach_grasp 阶段（已由伺服步替代）。替换只
        影响尚未执行的阶段，已完成阶段不会重放。

        两个特殊阶段：
          - move_to_pregrasp：记录本阶段为"重试前撤回点"，并按参数等待更新的计划；刷新被
            设为必需却等不到时，本次尝试直接判失败。
          - lift：抬起完成后做抓取验证，验证不通过按失败返回，失败阶段名仍为 lift。
        """

        stage_index = 0
        while stage_index < len(stages):
            stage = stages[stage_index]
            stage_start_revision = self._plan_revision
            # 抬起目标高度 = 抓取点 z + lift_z_m，反推即抬起起点 z，供视觉抬升证据对比使用
            if stage.name == "lift" and stage.pose is not None:
                self._last_lift_start_z_m = float(stage.pose.position[2]) - float(self.get_parameter("lift_z_m").value)
            ok, message = self._run_stage(stage)
            if not ok:
                return False, message, stage.name
            if stage.name == "move_to_pregrasp":
                self._retry_retreat_stage = stage
                refreshed_plan = self._wait_for_refreshed_plan(stage_start_revision)
                if (
                    refreshed_plan is None
                    and bool(self.get_parameter("refresh_plan_at_pregrasp_enabled").value)
                    and bool(self.get_parameter("refresh_plan_at_pregrasp_required").value)
                ):
                    return False, "fresh grasp plan unavailable after pregrasp", stage.name
                if refreshed_plan is not None:
                    refreshed_stages = self._append_place_stages(self._build_sequence_from_plan(refreshed_plan))
                    stages = self._replace_remaining_after_pregrasp(stages, refreshed_stages, stage_index)
                if self._approach_visual_servo_enabled() and stage.pose is not None:
                    ok, message = self._run_visual_servo_approach(stage.pose)
                    if not ok:
                        return False, message, "visual_servo_approach"
                    stages = self._remove_approach_after_pregrasp(stages, stage_index)
            if stage.name == "lift":
                verified, reason = self._verify_after_lift()
                if not verified:
                    return False, reason, stage.name
            stage_index += 1
        return True, "ok", ""

    def _diagnostic_prefix(self, stage: str) -> str:
        """构造统一日志前缀，把轮次、尝试序号、候选下标与阶段名串起来便于排障。"""

        return (
            f"[visual_grasp][run={self._current_run_id}]"
            f"[attempt={self._current_attempt_index}]"
            f"[candidate={self._current_candidate_index}]"
            f"[stage={stage}]"
        )

    def _log_diagnostic(self, stage: str, status: str, details: str = "") -> None:
        """输出一条带前缀的诊断日志；`details` 为空时不追加冒号后缀。"""

        suffix = f": {details}" if details else ""
        self.get_logger().info(f"{self._diagnostic_prefix(stage)} {status}{suffix}")

    def _log_plan_snapshot(self, plan: GraspPlan) -> None:
        """打印计划快照：有效性、来源、原因、类别、置信度、夹爪宽度与接近/抓取位姿。

        计划与候选字段都按可选字段读取（缺失回落到候选值或 0），因此对缺字段的旧消息也安全。
        """

        candidate = plan.candidate
        self.get_logger().info(
            f"{self._diagnostic_prefix('plan')} "
            f"valid={bool(plan.valid)}, source={plan.source}, reason={plan.reason}, "
            f"class={getattr(candidate, 'class_name', '')}, confidence={float(getattr(candidate, 'confidence', 0.0)):.3f}, "
            f"jaw_width={float(getattr(plan, 'jaw_width', 0.0) or getattr(candidate, 'jaw_width', 0.0) or 0.0):.4f}"
        )
        self.get_logger().info(f"{self._diagnostic_prefix('pregrasp_pose')} {self._format_pose(plan.pregrasp_pose)}")
        self.get_logger().info(f"{self._diagnostic_prefix('grasp_pose')} {self._format_pose(plan.grasp_pose)}")

    def _log_failure_snapshot(self, failed_stage: str, message: str) -> None:
        """失败时的汇总日志：失败阶段与原因 + 计划快照 + 夹爪证据。

        夹爪证据用于区分"没夹到"与"夹到但被判定不合规"：`last_gripper_reached_position`
        是最近一次张爪的实际到位开口（m），从未张过爪时打印 unknown；
        `closure_distance` 是本次闭合行程（m）。两者配合接触标志判断失败原因。
        """

        self.get_logger().error(f"{self._diagnostic_prefix(failed_stage)} fail: {message}")
        if self._current_attempt_plan is not None:
            self._log_plan_snapshot(self._current_attempt_plan)
        reached = "unknown" if self._last_gripper_reached_position is None else f"{self._last_gripper_reached_position:.4f}"
        self.get_logger().error(
            f"{self._diagnostic_prefix('failure_summary')} "
            f"failed_stage={failed_stage}, message={message}, "
            f"last_gripper_reached_position={reached}, "
            f"contact={self._last_grasp_contact_detected}, "
            f"closure_distance={self._last_grasp_closure_distance_m:.4f}"
        )

    def _format_pose(self, pose: Pose) -> str:
        """把位姿格式化成定长小数文本（位置 4 位小数，单位 m），便于日志逐行比对。"""

        return (
            "position=("
            f"{float(pose.position.x):.4f}, {float(pose.position.y):.4f}, {float(pose.position.z):.4f}"
            "), orientation=("
            f"{float(pose.orientation.x):.4f}, {float(pose.orientation.y):.4f}, "
            f"{float(pose.orientation.z):.4f}, {float(pose.orientation.w):.4f}"
            ")"
        )

    def _wait_for_refreshed_plan(self, min_revision: int) -> GraspPlan | None:
        """轮询等待"比 min_revision 更新的计划"，返回深拷贝；未启用、超时或被中止时返回 None。

        等待条件是计划版本号增大（而不是时间），因此只要订阅回调接受了新计划就能立刻返回；
        轮询间隔 20 ms 是对实时性与占用之间的折中。整个等待循环受 `rclpy.ok()` 与
        `_running` 控制，停止服务可在等待期间立即打断本次执行。
        """

        if not bool(self.get_parameter("refresh_plan_at_pregrasp_enabled").value):
            return None
        timeout_sec = float(self.get_parameter("refresh_plan_timeout_sec").value)
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and self._running and time.monotonic() < deadline:
            if self._plan_revision > min_revision and self._latest_plan is not None:
                self.get_logger().info(
                    "using refreshed grasp plan after pregrasp: "
                    f"revision={self._plan_revision}, source={self._latest_plan.source}"
                )
                return deepcopy(self._latest_plan)
            time.sleep(0.02)
        return None

    def _replace_remaining_after_pregrasp(
        self,
        current_stages: list[VisualGraspStage],
        refreshed_stages: list[VisualGraspStage],
        completed_pregrasp_index: int,
    ) -> list[VisualGraspStage]:
        """用新计划生成的阶段替换"接近点之后"的剩余阶段。

        保留已经执行完的部分（含接近点阶段本身），只替换其后内容；若新阶段列表里找不到
        接近点阶段（即没有可替换的剩余部分），则原样返回，避免把整个序列清空。
        """

        refreshed_remaining = self._remaining_stages_after_pregrasp(refreshed_stages)
        if not refreshed_remaining:
            return current_stages
        return current_stages[: completed_pregrasp_index + 1] + refreshed_remaining

    def _remaining_stages_after_pregrasp(self, stages: list[VisualGraspStage]) -> list[VisualGraspStage]:
        """截取接近点阶段之后的阶段；列表中若没有接近点阶段，则整体返回（视为全部都是剩余部分）。"""

        for index, stage in enumerate(stages):
            if stage.name == "move_to_pregrasp":
                return stages[index + 1 :]
        return stages

    def _remove_approach_after_pregrasp(
        self,
        stages: list[VisualGraspStage],
        completed_pregrasp_index: int,
    ) -> list[VisualGraspStage]:
        """删掉接近点之后紧跟的 approach_grasp 阶段。

        仅在视觉伺服已经用小步逼近代替了这一步时调用；只删紧跟的那一个，其余阶段顺序不变。
        """

        remaining = stages[completed_pregrasp_index + 1 :]
        if remaining and remaining[0].name == "approach_grasp":
            remaining = remaining[1:]
        return stages[: completed_pregrasp_index + 1] + remaining

    def _approach_visual_servo_enabled(self) -> bool:
        """是否启用接近段视觉伺服（用迭代小步替代一次到位的接近）。"""

        return bool(self.get_parameter("approach_visual_servo_enabled").value)

    def _run_visual_servo_approach(self, current: PoseTarget) -> tuple[bool, str]:
        """迭代逼近抓取点，返回 (是否收敛, 说明)。

        每次迭代先用最新计划算出期望抓取点，再由伺服策略给出"朝目标走一步"的位姿：
        误差进入容差即判收敛；否则按单步上限截断后执行一次移动。要求"每步都用新计划"
        时，等不到新计划立即失败，避免拿旧观测反复逼近同一点。

        失败语义：单步移动失败直接透传原因；迭代次数用尽仍未收敛返回 not converged，
        并在消息里带上最后一次误差。抛出的异常由上层执行回调统一处理。
        """

        max_iterations = max(1, int(self.get_parameter("approach_visual_servo_max_iterations").value))
        config = VisualServoApproachConfig(
            max_step_m=float(self.get_parameter("approach_visual_servo_max_step_m").value),
            position_tolerance_m=float(self.get_parameter("approach_visual_servo_position_tolerance_m").value),
        )
        last_error = 0.0
        require_fresh_plan = bool(self.get_parameter("approach_visual_servo_require_fresh_plan").value)
        for iteration in range(max_iterations):
            plan_revision = self._plan_revision
            refreshed_plan = self._wait_for_refreshed_plan(plan_revision)
            if require_fresh_plan and refreshed_plan is None:
                return False, "fresh visual servo plan unavailable"
            plan = refreshed_plan if refreshed_plan is not None else self._latest_plan
            if plan is None:
                return False, "no refreshed grasp plan available"
            _, desired_grasp = self._build_motion_targets(plan)
            step = build_visual_servo_step(current, desired_grasp, config)
            last_error = step.error_m
            if step.reached:
                return True, f"target reached: error={step.error_m:.4f}"
            ok, message = self._run_stage(
                VisualGraspStage(name="visual_servo_approach", kind="move", pose=step.target)
            )
            if not ok:
                return False, message
            current = step.target
            self.get_logger().info(
                "visual servo approach step "
                f"{iteration + 1}/{max_iterations}: error={step.error_m:.4f}"
            )
        return False, f"not converged after {max_iterations} steps: error={last_error:.4f}"

    def _verify_after_lift(self) -> tuple[bool, str]:
        """抬起后判定本次抓取是否真的成功，返回 (是否通过, 原因)。

        只在"真正执行 + 夹爪命令未禁用"时才有意义：plan_only 或夹爪被禁用时直接放行并注明跳过，
        因为此时根本没有真实夹持行为可供判定。

        证据来源：
          - 夹爪侧：是否检出接触（力闭合服务由堵转推断），以及闭合行程是否够大；
          - 视觉侧（可选）：最新计划里的目标高度相对抬起起点的高度变化，用来佐证目标被带起来。
        视觉证据在启用时是硬条件：拿不到证据或变化量不足都判失败。计算视觉证据可能因
        位姿换算失败抛异常，此处捕获后仅告警并标记为不可用，交由验证策略决定后果。
        """

        if not self._execution_enabled():
            return True, "plan_only: grasp verification skipped"
        if not self._gripper_execution_enabled():
            return True, "gripper disabled: grasp verification skipped"
        visual_delta = 0.0
        visual_available = False
        if bool(self.get_parameter("visual_lift_check_enabled").value) and self._latest_plan is not None:
            try:
                _, latest_grasp = self._build_motion_targets(self._latest_plan)
                if self._last_lift_start_z_m is not None:
                    visual_delta = float(latest_grasp.position[2]) - float(self._last_lift_start_z_m)
                    visual_available = True
            except Exception as exc:
                self.get_logger().warn(f"visual lift verification unavailable: {exc}")
        result = verify_grasp_after_lift(
            GraspVerificationInput(
                gripper_contact_detected=bool(self._last_grasp_contact_detected),
                closure_distance_m=float(self._last_grasp_closure_distance_m),
                visual_lift_delta_m=visual_delta,
                visual_lift_evidence_available=visual_available,
            ),
            GraspVerificationConfig(
                enabled=bool(self.get_parameter("grasp_verification_enabled").value),
                min_closure_distance_m=float(self.get_parameter("grasp_verification_min_closure_distance_m").value),
                require_gripper_contact=bool(self.get_parameter("grasp_verification_require_contact").value),
                visual_lift_check_enabled=bool(self.get_parameter("visual_lift_check_enabled").value),
                min_visual_lift_delta_m=float(self.get_parameter("visual_lift_min_delta_m").value),
            ),
        )
        return bool(result.success), str(result.reason)

    def _append_place_stages(self, stages: list[VisualGraspStage]) -> list[VisualGraspStage]:
        """在抓取序列末尾追加放置阶段（移动到位 → 松爪 → 抬升撤开）；未启用时追加空列表。"""

        return stages + build_place_stages(
            PlaceTaskConfig(
                enabled=bool(self.get_parameter("place_after_grasp_enabled").value),
                place_position_xyz=self._tuple3("place_position_xyz"),
                place_orientation_xyzw=self._tuple_n("place_orientation_xyzw", 4),
                open_position_m=float(self.get_parameter("place_open_position_m").value),
                open_max_effort=float(self.get_parameter("place_open_max_effort").value),
                retreat_z_m=float(self.get_parameter("place_retreat_z_m").value),
            )
        )

    def _request_retry_retreat(self) -> None:
        """重试前撤回：把末端退回本轮的接近点，避免贴着目标换位形。

        撤回点直接复用已执行过的 `move_to_pregrasp` 阶段位姿；若该阶段还没跑过（例如首个
        阶段就失败），只告警不动作。撤回本身失败也只记录告警：此时已经决定重试或放弃，
        不应该因为撤回失败再改变上层决策。
        """

        if self._retry_retreat_stage is None:
            self.get_logger().warn("safe retreat before retry requested, but no pregrasp retreat stage is available")
            return
        retreat = VisualGraspStage(
            name="retry_safe_retreat",
            kind="move",
            pose=self._retry_retreat_stage.pose,
        )
        ok, message = self._run_stage(retreat)
        if not ok:
            self.get_logger().warn(f"retry safe retreat failed: {message}")


    def _build_motion_targets(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        """按计划与策略算出 (接近点, 抓取点) 两个目标。

        三种分支：
          - 计划来源为 `candidate_ik_filter`：上游已做过 IK 与碰撞过滤，位姿就是最终答案，
            只做坐标系换算，不再叠加任何策略（见 `_build_motion_targets_from_filtered_plan`）。
          - `pose_policy` 为 visual_pose / source_pose / legacy：直接使用计划自带位姿，
            经坐标系换算、TCP 偏移扣减后再叠加各自的 z 偏移。
          - `pose_policy` 为 base_axis（默认）：只用计划里的抓取位置，姿态取固定值，
            接近点由基座接近轴与距离现算，因此不依赖上游给出的接近点质量。
        其它取值一律抛 ValueError，交由上层按执行失败处理。
        """

        if str(getattr(plan, "source", "")).strip() == "candidate_ik_filter":
            return self._build_motion_targets_from_filtered_plan(plan)
        policy = str(self.get_parameter("pose_policy").value).strip().lower()
        if policy in ("visual_pose", "source_pose", "legacy"):
            return (
                self._convert_plan_pose(plan, plan.pregrasp_pose, self._pregrasp_base_z_offset_m),
                self._convert_plan_pose(plan, plan.grasp_pose, self._grasp_base_z_offset_m),
            )
        if policy != "base_axis":
            raise ValueError(f"unsupported pose_policy: {policy}")
        grasp_pose = self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)
        return build_base_axis_grasp_targets(
            grasp_position_xyz=(
                float(grasp_pose.position.x),
                float(grasp_pose.position.y),
                float(grasp_pose.position.z),
            ),
            config=BaseAxisGraspPolicyConfig(
                fixed_orientation_xyzw=self._tuple_n("fixed_grasp_orientation_xyzw", 4),
                approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tcp_offset_xyz,
                target_base_offset_xyz=self._target_base_offset_xyz,
                pregrasp_z_offset_m=self._pregrasp_base_z_offset_m,
                grasp_z_offset_m=self._grasp_base_z_offset_m,
            ),
        )

    def _build_motion_targets_from_filtered_plan(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        """已过滤计划专用：直接采用计划里的接近点与抓取点位姿，只做坐标系换算。

        该分支不套用固定姿态、不叠加 TCP/基座偏移——上游过滤节点已经把这些处理完并做过
        可达性校验，再处理一次会让执行位姿偏离被校验过的那一个。
        """

        return (
            pose_to_target(self._transform_plan_pose_to_target_frame(plan, plan.pregrasp_pose)),
            pose_to_target(self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)),
        )

    def _transform_plan_pose_to_target_frame(self, plan: GraspPlan, pose: Pose) -> Pose:
        """把计划中的位姿换算到 `target_frame`。

        计划未带坐标系（空 frame_id）或本来就与目标坐标系相同时原样返回（仍做深拷贝，
        避免修改调用方持有的消息）；否则查询最新可用变换，查询超时 0.2 s，超时抛出
        异常（TF 查不到变换属于不可恢复状态，不能拿未换算的位姿去执行）。
        """

        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            converted = transform_pose_message(converted, _transform_from_msg(tf_msg))
        return converted

    def _convert_plan_pose(self, plan: GraspPlan, pose: Pose, base_z_offset_m: float) -> PoseTarget:
        """旧式策略的位姿处理：坐标系换算 → 扣减 TCP 偏移 → 叠加基座偏移与 z 偏移。

        位置偏移结果保留 6 位小数，避免浮点累加产生 1e-17 级别的噪声位姿；
        z 方向同时叠加 `target_base_offset_xyz[2]` 与传入的 `base_z_offset_m`
        （接近点与抓取点使用不同的 z 偏移参数）。
        """

        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            converted = transform_pose_message(converted, _transform_from_msg(tf_msg))
        converted = apply_tcp_offset_to_pose(converted, self._tcp_offset_xyz)
        converted.position.x = round(float(converted.position.x) + self._target_base_offset_xyz[0], 6)
        converted.position.y = round(float(converted.position.y) + self._target_base_offset_xyz[1], 6)
        converted.position.z = round(float(converted.position.z) + self._target_base_offset_xyz[2] + base_z_offset_m, 6)
        return pose_to_target(converted)

    def _run_stage(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """执行单个阶段并返回 (是否成功, 说明)。

        按 `kind` 分派：
          - move：调用运动执行服务；plan_only 模式下只规划干跑。
          - gripper：未开启执行时跳过；开启时闭合阶段优先走力闭合抓取服务，其余走位置服务。
          - safe_home：回安全位（未开启执行时直接跳过）。
        未知 `kind` 视为失败，避免静默漏掉某个阶段。

        真正执行时按 `_stage_waits` 做机械稳定等待；plan_only 不发生机械动作，因此不使用
        这些等待，只保留可选的 `plan_only_stage_pause_sec` 诊断节拍（默认 0）。等待结束后
        再检查一次 `_running`：若期间收到停止请求，本阶段返回失败 `stopped`。
        """

        self._log_diagnostic(stage.name, "start")
        if stage.kind == "move":
            if stage.pose is None:
                return False, "missing move pose"
            ok, message = self._call_execute_pose(stage)
        elif stage.kind == "gripper":
            if not self._execution_enabled():
                ok, message = True, "plan_only: gripper command skipped"
            elif self._gripper_execution_enabled():
                if stage.name == "close_gripper" and bool(self.get_parameter("gripper_grasp_enabled").value):
                    ok, message = self._call_grasp_gripper(stage)
                else:
                    ok, message = self._call_gripper(stage)
            else:
                ok, message = True, "simulation: gripper command skipped"
        elif stage.kind == "safe_home":
            ok, message = self._call_safe_home()
        else:
            return False, f"unsupported stage kind: {stage.kind}"
        if not ok:
            self._log_diagnostic(stage.name, "fail", message)
            return False, message
        if self._execution_enabled():
            wait_sec = self._stage_waits.get(stage.name, 0.0)
        else:
            wait_sec = float(self.get_parameter("plan_only_stage_pause_sec").value)
        time.sleep(max(0.0, wait_sec))
        if not self._running:
            return False, "stopped"
        self._log_diagnostic(stage.name, "ok", message)
        return True, message

    def _call_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """把运动阶段交给运动执行服务。

        真正执行且开启了 `trajectory_precheck_enabled` 时，先用 `execute=False` 干跑一次做
        预检：规划不通过就不下发执行请求，避免"先动了才发现后半段规划失败"。
        预检通过（或未启用）后，再按执行模式发出正式请求。
        """

        if stage.pose is None:
            return False, "missing move pose"
        if not self._execute_pose_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "motion execution service unavailable"
        if self._execution_enabled() and bool(self.get_parameter("trajectory_precheck_enabled").value):
            ok, message = self._precheck_execute_pose(stage)
            if not ok:
                return False, f"trajectory precheck failed: {message}"
        return self._send_execute_pose(stage, execute=self._execution_enabled())

    def _precheck_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """执行前的规划干跑：只规划、不执行。"""

        return self._send_execute_pose(stage, execute=False)

    def _send_execute_pose(self, stage: VisualGraspStage, *, execute: bool) -> tuple[bool, str]:
        """组装并发送一次运动执行请求，返回 (是否成功, 阶段说明)。

        `execute=False` 即干跑。速度缩放按阶段选择，加速度缩放全阶段共用；超时字段同时作为
        服务端执行超时下发（`motion_result_timeout_sec`），客户端等待时间取
        "服务可用超时 + 执行超时"之和，保证等待窗口不小于服务端可能花费的时间。
        返回文本拼上服务端返回的阶段名，便于从日志判断失败发生在规划还是执行。
        """

        if stage.pose is None:
            return False, "missing move pose"
        request = ExecutePose.Request()
        request.target_pose = target_to_pose_stamped(stage.pose, self._target_frame)
        request.velocity_scaling = self._velocity_scaling_for_stage(stage.name)
        request.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        request.timeout_sec = self._motion_result_timeout_sec
        request.execute = bool(execute)
        request.suppress_preview = bool(not execute and not self._execution_enabled())
        if not execute and not self._execution_enabled() and self._preview_start_joint_state is not None:
            request.preview_start_joint_state = deepcopy(self._preview_start_joint_state)
        future = self._execute_pose_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec + self._motion_result_timeout_sec):
            return False, "motion execution service call timed out"
        result = future.result()
        if result is None:
            return False, "motion execution returned no result"
        if result.success and not execute and not self._execution_enabled():
            trajectory = result.planned_trajectory
            names = list(trajectory.joint_names)
            points = list(trajectory.points)
            if (
                len(names) != 6
                or set(names) != {f"joint{i}" for i in range(1, 7)}
                or not points
                or len(points[-1].positions) != 6
                or not all(math.isfinite(value) for value in points[-1].positions)
            ):
                return False, "plan-only response has no valid arm trajectory"
            next_start = JointState()
            next_start.name = names
            next_start.position = list(points[-1].positions)
            self._preview_start_joint_state = next_start
            self._preview_trajectories.append(deepcopy(trajectory))
        return bool(result.success), f"{result.stage}: {result.message}"

    def _publish_preview_sequence(self) -> tuple[bool, str]:
        """全部阶段规划成功后，请运动层一次发布完整 RViz 预览序列。"""

        if not self._preview_trajectories:
            return False, "no planned trajectories collected"
        if not self._publish_preview_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "trajectory preview service unavailable"
        request = PublishTrajectoryPreview.Request()
        request.trajectories = deepcopy(self._preview_trajectories)
        future = self._publish_preview_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec):
            return False, "trajectory preview service call timed out"
        result = future.result()
        if result is None:
            return False, "trajectory preview service returned no result"
        return bool(result.success), str(result.message)

    def _execution_enabled(self) -> bool:
        """是否处于真正下发动作的模式：只有 execute / real 算数，其余（含 plan_only）都只做规划。"""

        return self._execution_mode in ("execute", "real")

    def _gripper_execution_enabled(self) -> bool:
        """夹爪命令是否真正下发：要求处于执行模式且 `execute_gripper` 未被关闭。"""

        return self._execution_enabled() and bool(self.get_parameter("execute_gripper").value)

    def _detected_jaw_width(self, plan: GraspPlan) -> float:
        """取检测到的夹爪开口宽度（m）：计划字段优先，缺失或非正时回落到候选字段，仍无则为 0。"""

        plan_width = float(getattr(plan, "jaw_width", 0.0) or 0.0)
        candidate_width = float(getattr(plan.candidate, "jaw_width", 0.0) or 0.0)
        return plan_width if plan_width > 0.0 else candidate_width

    def _detected_object_length(self, plan: GraspPlan) -> float:
        """取候选给出的目标长度估计（m）；缺失时为 0。"""

        return float(getattr(plan.candidate, "object_length", 0.0) or 0.0)

    def _velocity_scaling_for_stage(self, name: str) -> float:
        """按阶段名选择速度缩放：接近类阶段最慢，抬起/撤退居中，其余用常规值。"""

        if name == "visual_servo_approach":
            return float(self.get_parameter("approach_velocity_scaling").value)
        if name == "approach_grasp":
            return float(self.get_parameter("approach_velocity_scaling").value)
        if name == "lift":
            return float(self.get_parameter("lift_velocity_scaling").value)
        if name == "safe_retreat":
            return float(self.get_parameter("lift_velocity_scaling").value)
        return float(self.get_parameter("move_velocity_scaling").value)

    def _call_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """调用夹爪位置服务开到指定开口，返回 (是否成功, 说明)。

        副作用（供后续抓取验证使用）：
          - 张开阶段成功时记录实际到位开口 `_last_gripper_reached_position`，它是计算
            闭合行程的基准；张开失败则不更新，避免用错误基准估算行程。
          - 闭合阶段失败但启用"接触判据"时，若"停在了目标外侧一定余量"且"相对上次张开
            位有明显行程"，则按"夹到了目标"处理并返回成功；这是无接触传感器下的替代判据，
            返回文本会标注 contact assumed。
        失败信息统一带上实际到位开口（m），便于区分"没动"与"动到一半"。
        """

        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper target"
        if not self._gripper_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "gripper service unavailable"
        request = SetGripper.Request()
        request.position = float(stage.gripper_position_m)
        request.max_effort = float(stage.gripper_max_effort)
        future = self._gripper_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec):
            return False, "gripper service call timed out"
        result = future.result()
        if result is None:
            return False, "gripper service returned no result"
        reached_position = float(result.reached_position)
        command_success = bool(result.success)
        if stage.name == "open_gripper" and command_success:
            self._last_gripper_reached_position = reached_position
        if stage.name == "open_gripper_at_place" and command_success:
            self._last_gripper_reached_position = reached_position
        if stage.name == "close_gripper" and not command_success:
            contact_ok = bool(self.get_parameter("close_contact_success_enabled").value) and close_contact_success(
                command_success=command_success,
                target_position_m=float(stage.gripper_position_m),
                reached_position_m=reached_position,
                previous_open_position_m=self._last_gripper_reached_position,
                contact_margin_m=float(self.get_parameter("close_contact_margin_m").value),
                min_closure_delta_m=float(self.get_parameter("close_contact_min_closure_delta_m").value),
            )
            if contact_ok:
                return True, (
                    f"contact assumed: reached_position={reached_position:.4f}, "
                    f"target={float(stage.gripper_position_m):.4f}"
                )
        return command_success, f"reached_position={reached_position:.4f}"

    def _call_safe_home(self) -> tuple[bool, str]:
        """请求运动层回到安全位；非执行模式下直接跳过（返回成功并注明 skipped）。

        这是序列末尾可选的大范围回零动作，等待窗口同样按"服务可用超时 + 执行超时"计算。
        """

        if not self._execution_enabled():
            return True, "plan_only: safe_home skipped"
        if not self._safe_home_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "safe_home service unavailable"
        future = self._safe_home_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, self._service_timeout_sec + self._motion_result_timeout_sec):
            return False, "safe_home service call timed out"
        result = future.result()
        if result is None:
            return False, "safe_home returned no result"
        return bool(result.success), str(result.message)

    def _call_grasp_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        """调用力闭合抓取服务闭合夹爪并保持，返回 (是否成功, 说明)。

        请求字段映射：闭合力矩取 `gripper_grasp_close_force`（夹持力旋钮），保持力矩取阶段
        计算出的最大力矩（两者都截断为非负）；超时、最短闭合时间、停住速度阈值与最小闭合
        行程全部来自抓取参数。注意本夹爪没有力传感器，"接触"是闭合行程 + 速度堵转推断的
        结果，不是实测接触力。

        副作用（供抬起后抓取验证使用）：记录是否检出接触，并把闭合行程记为
        "上次张爪到位开口 - 本次到位开口"，结果截断到非负；若从未张过爪（基准为 None），
        基准按 0 处理，因此行程会偏大——正常序列里闭合前一定先张开过。
        """

        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper grasp target"
        if not self._gripper_grasp_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "gripper grasp service unavailable"
        request = GraspGripper.Request()
        request.close_force = max(float(self.get_parameter("gripper_grasp_close_force").value), 0.0)
        request.hold_force = max(float(stage.gripper_max_effort), 0.0)
        request.close_timeout_sec = float(self.get_parameter("gripper_grasp_timeout_sec").value)
        request.min_close_time_sec = float(self.get_parameter("gripper_grasp_min_close_time_sec").value)
        request.velocity_threshold = float(self.get_parameter("gripper_grasp_velocity_threshold").value)
        request.min_closure_distance_m = float(self.get_parameter("gripper_grasp_min_closure_distance_m").value)
        future = self._gripper_grasp_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec + request.close_timeout_sec):
            return False, "gripper grasp service call timed out"
        result = future.result()
        if result is None:
            return False, "gripper grasp service returned no result"
        self._last_grasp_contact_detected = bool(result.contact_detected)
        self._last_grasp_closure_distance_m = max(
            0.0,
            float(self._last_gripper_reached_position or 0.0) - float(result.reached_position),
        )
        if result.success:
            return True, (
                f"{result.message}: contact={result.contact_detected}, "
                f"contact_position={result.contact_position:.4f}, "
                f"hold_force={result.hold_force:.3f}"
            )
        return False, (
            f"{result.message}: contact={result.contact_detected}, "
            f"reached_position={result.reached_position:.4f}"
        )

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """轮询等待异步调用完成，返回是否已完成。

        返回 False 有两种含义：超时，或等待期间被停止（`_running` 变假）；调用方把两者
        统一按失败处理，调用结果由调用方自行读取。轮询间隔 20 ms，避免忙等占满 CPU。
        """

        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if not self._running:
                return False
            time.sleep(0.02)
        return future.done()

    def _request_stop(self) -> None:
        """同时向运动执行与轨迹层发出停止请求，双通道保险，任一可用即可生效。"""

        self._request_stop_service(self._motion_stop_client, "motion execution stop")
        self._request_stop_service(self._trajectory_stop_client, "trajectory_stop")

    def _request_stop_service(self, client, label: str) -> None:
        """尽力而为地调用一个停止服务：服务端 0.2 s 内不可用就放弃，异常只告警不抛出。

        停止路径本身不能失败退出，否则会把异常传播到执行回调，掩盖真正的失败原因。
        """

        try:
            if not client.wait_for_service(timeout_sec=0.2):
                return
            client.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().warn(f"failed to request {label}: {exc}")


def main(args=None) -> None:
    """进程入口：初始化 rclpy，跑一个三线程执行器。

    线程数 3 是为了让长时间运行的执行服务回调（内含轮询等待）不至于饿死订阅回调与
    停止服务；`KeyboardInterrupt` 与外部关闭异常按正常退出处理（不打印堆栈），
    退出时先关执行器再销毁节点，最后只在 rclpy 仍有效时调用 shutdown。
    """

    rclpy.init(args=args)
    node = VisualGraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""move_relative 意图的仿真运动适配器：规划 + 轨迹下发。

本模块把一次"相对当前位置沿 x/y/z 平移 distance_m"的意图，转换成运动包
``rebotarm_motion`` 的 ``PoseTarget``，调用注入的规划器得到轨迹，再通过
FollowJointTrajectory 动作把轨迹下发给仿真控制器。它只负责"规划与派发"，
不做安全上限判断（上限在工具调用校验、安全门与仿真动作节点中各自校验），
也不等待轨迹执行完成。

坐标与单位：位置单位米，姿态用 roll/pitch/yaw（弧度）；位移轴取
base_link / end_link / tool0 三种坐标系之一，但当前实现只按轴名做平移，
不做坐标系间的旋转变换。速度缩放 speed_scale 通过拉伸轨迹时间实现，见
``build_follow_joint_trajectory_goal``。

对外接口：``build_relative_pose_target``、``MoveRelativeSimMotionAdapter``、
``build_follow_joint_trajectory_goal``。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from rebotarm_motion.command_models import PoseTarget

from .models import SafetyViolationError


@dataclass(frozen=True)
class MoveRelativePlanResult:
    """一次相对运动的结果：是否成功、说明文本、轨迹与目标位姿。"""

    success: bool  # 规划与派发是否都成功
    message: str  # 失败原因或成功说明，会透传给动作结果
    trajectory: Any | None  # 规划出的关节轨迹；失败时为 None
    final_pose: PoseTarget | None  # 本次相对运动的目标位姿，失败时也保留以便诊断


class TrajectoryGoalSender(Protocol):
    def send_goal_async(self, goal_msg: Any) -> Any:
        """异步发送轨迹目标到仿真控制器，返回 future。"""


def build_relative_pose_target(
    *,
    axis: str,
    distance_m: float,
    frame_id: str,
    current_pose: dict[str, float],
) -> PoseTarget:
    """由当前位姿计算沿指定轴平移后的目标位姿。

    axis 只接受 x/y/z，frame_id 只接受 base_link/end_link/tool0（空字符串
    表示沿用默认），任一不合法即抛 SafetyViolationError。当前位姿字典中
    缺失的姿态字段按 0.0 处理，位置字段缺失则直接 KeyError；返回位姿的
    姿态与当前位姿一致，即只做纯平移。
    """
    if axis not in {"x", "y", "z"}:
        raise SafetyViolationError("move_relative axis must be x, y, or z")
    if frame_id and frame_id not in {"base_link", "end_link", "tool0"}:
        raise SafetyViolationError("move_relative frame_id must be base_link, end_link, or tool0")

    base = PoseTarget(
        x=float(current_pose["x"]),
        y=float(current_pose["y"]),
        z=float(current_pose["z"]),
        roll=float(current_pose.get("roll", 0.0)),
        pitch=float(current_pose.get("pitch", 0.0)),
        yaw=float(current_pose.get("yaw", 0.0)),
    )
    dx, dy, dz = _axis_offset(axis, float(distance_m))
    return PoseTarget(
        x=base.x + dx,
        y=base.y + dy,
        z=base.z + dz,
        roll=base.roll,
        pitch=base.pitch,
        yaw=base.yaw,
    )


class MoveRelativeSimMotionAdapter:
    """把相对运动意图接到"规划器 + 轨迹客户端"上的适配器。

    依赖全部通过构造注入，便于在没有 ROS 2 的环境里用假规划器测试：
    planner 需提供 plan_preview(preview) 并返回带 success / trajectory 的
    结果；trajectory_client 需提供 send_goal_async；current_pose_supplier
    返回当前末端位姿字典；goal_builder 把轨迹与速度缩放构造成动作目标。
    """

    def __init__(
        self,
        *,
        planner: Any,
        trajectory_client: TrajectoryGoalSender,
        current_pose_supplier: Any,
        goal_builder: Any,
        future_waiter: Any | None = None,
    ) -> None:
        self._planner = planner
        self._trajectory_client = trajectory_client
        self._current_pose_supplier = current_pose_supplier
        self._goal_builder = goal_builder
        self._future_waiter = future_waiter

    def execute_move_relative(
        self,
        *,
        axis: str,
        distance_m: float,
        frame_id: str,
        speed_scale: float,
    ) -> MoveRelativePlanResult:
        """执行一次相对运动：读取当前位姿 → 规划 → 下发轨迹目标。

        参数：axis 为平移轴（x/y/z）；distance_m 为位移，单位米，符号表示
        方向；frame_id 为参考坐标系；speed_scale 为速度缩放，越大越快。
        返回 MoveRelativePlanResult：规划失败或目标被控制器拒绝时 success
        为 False，但仍返回算出的目标位姿与（若已规划出的）轨迹以便诊断。
        副作用：调用规划服务并向仿真控制器发送动作目标；本方法不等待轨迹
        执行完成，只等到目标被接受。
        """
        current_pose = self._current_pose_supplier()
        pose_target = build_relative_pose_target(
            axis=axis,
            distance_m=distance_m,
            frame_id=frame_id,
            current_pose=current_pose,
        )
        # 规划器只依赖 pose_target 与 speed_scale 两个属性，这里用轻量匿名
        # 对象满足其预览接口，避免为一次性调用定义正式数据模型。
        preview = type(
            "Preview",
            (),
            {"pose_target": pose_target, "speed_scale": float(speed_scale)},
        )()
        plan_result = self._planner.plan_preview(preview)
        # 规划失败或轨迹为空都不下发，直接返回失败结果。
        if not plan_result.success or plan_result.trajectory is None:
            return MoveRelativePlanResult(
                success=False,
                message=plan_result.message,
                trajectory=None,
                final_pose=pose_target,
            )

        goal_msg = self._goal_builder(plan_result.trajectory, float(speed_scale))
        future = self._trajectory_client.send_goal_async(goal_msg)
        # 未注入等待器时由调用方自行自旋；注入时在此阻塞至目标响应。
        if self._future_waiter is not None:
            self._future_waiter(future, timeout_sec=5.0)
        goal_handle = future.result()
        # getattr 兜底：等待超时后 future 结果可能为 None。
        if not bool(getattr(goal_handle, "accepted", False)):
            return MoveRelativePlanResult(
                success=False,
                message="sim trajectory goal rejected",
                trajectory=plan_result.trajectory,
                final_pose=pose_target,
            )
        return MoveRelativePlanResult(
            success=True,
            message="planned and dispatched to simulation controller",
            trajectory=plan_result.trajectory,
            final_pose=pose_target,
        )


def _axis_offset(axis: str, distance_m: float) -> tuple[float, float, float]:
    """把轴名与位移换算成 (dx, dy, dz) 三元组。

    调用方已保证 axis 合法；这里对未知轴按 z 处理，与原有实现一致。
    """
    if axis == "x":
        return distance_m, 0.0, 0.0
    if axis == "y":
        return 0.0, distance_m, 0.0
    return 0.0, 0.0, distance_m


def build_follow_joint_trajectory_goal(trajectory: Any, speed_scale: float) -> Any:
    """把轨迹包装成 FollowJointTrajectory 目标，并按速度缩放拉伸各点时间。

    speed_scale 是速度倍率：大于 1 表示更快，因此每个轨迹点的
    time_from_start 要乘以 1/speed_scale。分母用 max(speed_scale, 0.05)
    兜底，避免 0 或负值导致除零或时间倒流，等效于最多把时长拉伸 20 倍。
    goal_time_tolerance 恒为 0，表示要求控制器严格按时完成，不额外放宽。
    需要环境提供 control_msgs 的 FollowJointTrajectory 类型，缺失时抛
    SafetyViolationError。
    """
    try:
        from control_msgs.action import FollowJointTrajectory
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("control_msgs is required for sim trajectory dispatch") from exc

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance.sec = 0
    goal.goal_time_tolerance.nanosec = 0
    # 速度倍率的倒数即时间拉伸系数；下限 0.05 防止除零。
    scale = 1.0 / max(float(speed_scale), 0.05)
    for point in getattr(goal.trajectory, "points", []):
        duration = getattr(point, "time_from_start", None)
        if duration is not None:
            # 先统一换算成纳秒再缩放，避免 sec 与 nanosec 分别取整产生误差。
            total_ns = int((int(duration.sec) * 1_000_000_000 + int(duration.nanosec)) * scale)
            duration.sec = total_ns // 1_000_000_000
            duration.nanosec = total_ns % 1_000_000_000
    return goal

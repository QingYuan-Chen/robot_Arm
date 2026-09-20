"""MoveIt 规划适配器：把关节目标或末端位姿目标翻译成规划请求并同步取回轨迹。

职责与位置
    本模块属于运动层，是运动包调用外部运动规划能力的唯一薄封装：只负责"请求规划、返回
    关节轨迹"，不执行轨迹、不访问硬件（轨迹下发与停止由上层执行节点负责）。上层（位姿
    执行节点、示教回放节点、语音相对运动适配器）通过构造参数注入配置，再调用
    :meth:`MoveItMotionPlanner.plan_pose`、:meth:`~MoveItMotionPlanner.plan_joint_positions`
    或 :meth:`~MoveItMotionPlanner.plan_preview`。

对外接口
    规划服务：``GetMotionPlan`` 接口（服务名由调用方给出，默认形如 ``/plan_kinematic_path``）。
    三个入口都返回 :class:`MotionPlanResult`：失败原因写在结果的 ``message`` 里，不抛异常，
    便于上层把它直接转成状态 payload。

安全语义（读代码时最容易忽略的几点）
    1. 默认 ``start_state.is_diff = True``，规划起点取当前状态；纯规划预览可显式提供上一段
       轨迹终点作为完整虚拟起点，从而连续规划，但该状态不得用于真实执行；
    2. 速度/加速度缩放因子被夹到 [0.01, 1.0]，关节容差与位置容差都有下限，防止调用方传 0
       导致按满速规划或得到零容差的不可解约束；
    3. 只有 ``error_code == 1``（成功）且关节轨迹含轨迹点才算成功；服务异常、等待超时、
       空轨迹一律按失败返回，绝不把"没有结果"当成"安全路径"；
    4. 本类不自旋：:meth:`MoveItMotionPlanner._spin_until_future` 用 ``sleep`` 轮询 future，
       调用方必须保证另有线程在自旋执行器，否则请求只能等到超时。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from math import cos, sin
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.duration import Duration
from shape_msgs.msg import SolidPrimitive


@dataclass(frozen=True)
class MotionPlanResult:
    """一次规划请求的结果。

    ``success``：是否拿到可执行的关节轨迹；``message``：英文原因文本（对外接口，日志与
    上层 payload 会原样转发，不要翻译）；``trajectory``：成功时为关节轨迹消息对象，失败时
    为 ``None``。

    冻结 dataclass：按值传递与比较，结果不允许被就地修改。
    """

    success: bool
    message: str
    trajectory: object | None


class MoveItMotionPlanner:
    """规划客户端适配器：一次调用 = 一次规划服务请求。

    构造时在传入的节点上创建服务客户端，之后可长期复用。实例不保存单次请求的状态，
    配置在构造后固定，因此多个调用方（位姿执行、示教回放、语音相对运动）可以共用同一实例；
    但 ROS 客户端本身的并发调用仍受执行器线程模型约束——调用线程必须与自旋线程分离，
    否则 future 无法完成。规划失败不抛异常，而是返回 ``success=False`` 的结果。

    线程/回调模型：本类不注册任何回调，全部为同步阻塞式调用（内部轮询 future）。
    """

    def __init__(
        self,
        node,
        *,
        group_name: str,
        ee_frame_id: str,
        frame_id: str,
        planning_service: str,
        planning_pipeline: str,
        planner_id: str,
        planning_time: float,
        num_attempts: int,
        goal_position_tolerance: float,
        goal_orientation_tolerance: float,
    ) -> None:
        """保存规划配置并创建服务客户端。

        参数：
        - ``node``：用于创建客户端与读取时钟的节点（本类不接管其生命周期）；
        - ``group_name``：规划组名，需与运动规划配置中的组名一致（本仓库为手臂组）；
        - ``ee_frame_id``：位置/姿态约束作用的末端 link 名；
        - ``frame_id``：目标位姿的参考坐标系（如 base_link）；
        - ``planning_service``：规划服务名（接口为 GetMotionPlan）；
        - ``planning_pipeline``：规划管线 id（如 ompl），空串表示用默认管线；
        - ``planner_id``：管线内的规划器 id，空串表示用该管线的默认规划器；
        - ``planning_time``：单次规划允许的最长时间，单位 s；同时决定等待 future 的上限；
        - ``num_attempts``：规划尝试次数，调大提高成功率但成倍增加耗时；
        - ``goal_position_tolerance``：末端位置约束容差，单位 m（作为约束盒边长使用）；
        - ``goal_orientation_tolerance``：末端姿态各轴容差，单位 rad。
        """
        self._node = node
        self._group_name = group_name
        self._ee_frame_id = ee_frame_id
        self._frame_id = frame_id
        self._planning_pipeline = planning_pipeline
        self._planner_id = planner_id
        self._planning_time = float(planning_time)
        self._num_attempts = int(num_attempts)
        self._goal_position_tolerance = float(goal_position_tolerance)
        self._goal_orientation_tolerance = float(goal_orientation_tolerance)
        self._client = node.create_client(GetMotionPlan, planning_service)

    def service_is_ready(self) -> bool:
        """规划服务当前是否已连接（不阻塞，立即返回）。"""
        return bool(self._client.service_is_ready())

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        """等待规划服务可用，超时返回 ``False``；``timeout_sec`` 单位 s。"""
        return bool(self._client.wait_for_service(timeout_sec=timeout_sec))

    def plan_joint_positions(
        self,
        *,
        joint_names: tuple[str, ...],
        target_positions: tuple[float, ...],
        tolerance: float = 0.005,
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
    ) -> MotionPlanResult:
        """规划到一组关节目标（关节空间的目标约束）。

        参数：
        - ``joint_names`` / ``target_positions``：关节名与目标角度（rad），两者长度必须
          一致，否则直接返回失败（不截断、不补齐）；
        - ``tolerance``：每个关节的上/下对称容差，单位 rad，下限 1e-5；
        - ``velocity_scaling`` / ``acceleration_scaling``：无量纲缩放，夹到 [0.01, 1.0]；
          默认 0.1 表示只用额定速度/加速度的 10%。

        服务在 0.5 s 内不可用、规划失败或返回空轨迹时返回失败结果，不抛异常。
        """
        if len(joint_names) != len(target_positions):
            # 长度不一致说明调用方的关节顺序/数量搞错了；按较短一侧 zip 会静默丢约束。
            return MotionPlanResult(
                success=False,
                message="moveit joint target length mismatch",
                trajectory=None,
            )
        if not self._client.wait_for_service(timeout_sec=0.5):
            return MotionPlanResult(
                success=False,
                message="moveit planning service unavailable",
                trajectory=None,
            )

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self._group_name
        motion_request.pipeline_id = self._planning_pipeline
        motion_request.planner_id = self._planner_id
        motion_request.num_planning_attempts = self._num_attempts
        motion_request.allowed_planning_time = self._planning_time
        # 缩放因子夹到 [0.01, 1.0]：0 会被规划侧当成非法或满速，>1 会超出额定动态限值。
        motion_request.max_velocity_scaling_factor = min(max(float(velocity_scaling), 0.01), 1.0)
        motion_request.max_acceleration_scaling_factor = min(max(float(acceleration_scaling), 0.01), 1.0)
        # diff 语义：起点取"当前状态"，不使用规划场景里缓存的旧起点。
        motion_request.start_state.is_diff = True
        constraints = Constraints()
        # 容差下限：0 容差会让关节约束退化成等式，在采样类规划器上几乎必然无解。
        tol = max(float(tolerance), 1e-5)
        for name, position in zip(joint_names, target_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = str(name)
            joint_constraint.position = float(position)
            # 上下容差对称、权重 1.0：属于硬约束，六个关节都必须落在容差带内。
            joint_constraint.tolerance_above = tol
            joint_constraint.tolerance_below = tol
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        motion_request.goal_constraints = [constraints]
        return self._call_plan_service(request)

    def plan_preview(self, preview: Any) -> MotionPlanResult:
        """按预览对象的位姿目标规划（末端笛卡尔位姿约束）。

        ``preview.pose_target`` 需提供 ``x/y/z``（m）与 ``roll/pitch/yaw``（rad）属性；
        为 ``None`` 时直接返回失败。本入口的速度/加速度缩放固定写死为 0.1，不接受调用方
        覆盖（预览与语音相对运动走这条路径）。
        """
        pose_target = preview.pose_target
        if pose_target is None:
            return MotionPlanResult(
                success=False,
                message="preview has no pose target for moveit planning",
                trajectory=None,
            )

        if not self._client.wait_for_service(timeout_sec=0.5):
            return MotionPlanResult(
                success=False,
                message="moveit planning service unavailable",
                trajectory=None,
            )

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self._group_name
        motion_request.pipeline_id = self._planning_pipeline
        motion_request.planner_id = self._planner_id
        motion_request.num_planning_attempts = self._num_attempts
        motion_request.allowed_planning_time = self._planning_time
        # 固定在 0.1：预览入口不接受调用方覆盖缩放，始终按额定动态的 10% 规划。
        motion_request.max_velocity_scaling_factor = 0.1
        motion_request.max_acceleration_scaling_factor = 0.1
        motion_request.start_state.is_diff = True
        motion_request.goal_constraints = [self._build_goal_constraints(preview)]

        return self._call_plan_service(request)

    def plan_pose(
        self,
        pose: PoseStamped,
        *,
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
        start_joint_state=None,
    ) -> MotionPlanResult:
        """按给定的末端位姿目标规划。

        ``pose`` 是带参考坐标系的位姿消息（本仓库用 base_link 系）：其 ``header`` 会原样
        写进位置/姿态约束，因此调用方必须保证 ``frame_id`` 是规划器认识的坐标系。
        ``velocity_scaling`` / ``acceleration_scaling`` 夹到 [0.01, 1.0]，默认 0.1。
        ``start_joint_state`` 仅供调用层构造连续纯规划预览；为空时仍使用当前状态。
        """
        if not self._client.wait_for_service(timeout_sec=0.5):
            return MotionPlanResult(
                success=False,
                message="moveit planning service unavailable",
                trajectory=None,
            )

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self._group_name
        motion_request.pipeline_id = self._planning_pipeline
        motion_request.planner_id = self._planner_id
        motion_request.num_planning_attempts = self._num_attempts
        motion_request.allowed_planning_time = self._planning_time
        # 与关节目标入口一致：缩放夹到 [0.01, 1.0]，避免 0 或超限值直达规划器。
        motion_request.max_velocity_scaling_factor = min(max(float(velocity_scaling), 0.01), 1.0)
        motion_request.max_acceleration_scaling_factor = min(max(float(acceleration_scaling), 0.01), 1.0)
        if start_joint_state is None or not start_joint_state.name:
            motion_request.start_state.is_diff = True
        else:
            # 指定完整的虚拟臂状态；规划服务不得再将其与当前假关节状态混用。
            motion_request.start_state.is_diff = False
            motion_request.start_state.joint_state = start_joint_state
        motion_request.goal_constraints = [self._build_pose_constraints(pose)]

        return self._call_plan_service(request)

    def _call_plan_service(self, request: GetMotionPlan.Request) -> MotionPlanResult:
        """发起异步规划调用并同步等待，把结果统一归类为"成功"或"失败 + 原因"。

        返回失败而不抛异常，``message`` 区分四类原因：等待超时、调用抛异常、规划器返回非
        成功错误码、返回空关节轨迹。``error_code == 1`` 是规划成功码，其它数值原样带进消息
        便于定位。成功时只返回 ``joint_trajectory``（上层直接用它构造轨迹目标）。
        """
        future = self._client.call_async(request)
        self._spin_until_future(future)
        if not future.done():
            # 未在 planning_time + 1.0 s 内完成：按超时失败处理，不去读未完成的结果。
            return MotionPlanResult(
                success=False,
                message="moveit planning request timed out",
                trajectory=None,
            )

        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            # 服务端异常（通信中断、类型/序列化问题）不能让规划请求把上层进程带崩。
            return MotionPlanResult(
                success=False,
                message=f"moveit planning request failed: {exc}",
                trajectory=None,
            )

        error_code = int(response.motion_plan_response.error_code.val)
        if error_code != 1:
            # 1 = SUCCESS；其余（无解、规划失败、超时等）都按失败处理。
            return MotionPlanResult(
                success=False,
                message=f"moveit planning failed: error_code={error_code}",
                trajectory=None,
            )

        trajectory = response.motion_plan_response.trajectory
        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        # 有成功码但没有轨迹点，等于没有可执行内容：绝不能当成成功下发。
        if joint_trajectory is None or not joint_trajectory.points:
            return MotionPlanResult(
                success=False,
                message="moveit planning returned empty joint trajectory",
                trajectory=None,
            )

        return MotionPlanResult(
            success=True,
            message="moveit trajectory planned",
            trajectory=joint_trajectory,
        )

    def _build_goal_constraints(self, preview: Any) -> Constraints:
        """把预览对象的位姿目标转成规划位姿约束。

        预览目标是简单的 ``rpy + xyz`` 结构（``preview.pose_target`` 非 ``None`` 由调用方
        保证），这里先组装成带 ``frame_id`` 的位姿消息，再复用
        :meth:`_build_pose_constraints`。``header.stamp`` 保持默认 0，约定为"使用该坐标系的
        最新可用变换"。
        """
        pose_target = preview.pose_target
        assert pose_target is not None

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = float(pose_target.x)
        pose.pose.position.y = float(pose_target.y)
        pose.pose.position.z = float(pose_target.z)
        qx, qy, qz, qw = rpy_to_quaternion(
            float(pose_target.roll),
            float(pose_target.pitch),
            float(pose_target.yaw),
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return self._build_pose_constraints(pose)

    def _build_pose_constraints(self, pose: PoseStamped) -> Constraints:
        """构造位置 + 姿态两个约束，两者都作用在 ``self._ee_frame_id`` 上。

        位置约束用边长 ``tol``（m）的立方体：约束区域是以目标位姿为中心的盒子，因此实际
        允许的位置偏差约为 ±tol/2。``tol`` 取 ``goal_position_tolerance`` 与 1e-4 的较大
        者，避免配置成 0 时出现零尺寸盒。

        姿态约束用各轴绝对容差（rad），三个轴共用 ``goal_orientation_tolerance``。两个约束
        权重都是 1.0，即硬约束。
        """
        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = self._ee_frame_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # BOX 的 dimensions 是各方向全长（非半长）：位置容差实际是盒子边长的一半。
        tol = max(self._goal_position_tolerance, 1e-4)
        primitive.dimensions = [tol, tol, tol]
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self._ee_frame_id
        orientation_constraint.orientation = pose.pose.orientation
        # 三个轴共用同一容差（rad）；不设置 parameterization，保持规划侧默认。
        orientation_constraint.absolute_x_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def _spin_until_future(self, future) -> None:
        """阻塞等待 future 完成，上限为 ``planning_time + 1.0`` s。

        ``+1.0`` s 是留给服务往返的余量。本方法不调用自旋接口，只是每 50 ms 查询一次
        ``future.done()``，因此调用方必须有另一个线程在自旋执行器，否则请求只能等到超时。
        计时使用节点时钟（可能是仿真时间），deadline 在进入循环前一次算好。
        """
        deadline = self._node.get_clock().now() + Duration(seconds=self._planning_time + 1.0)
        while not future.done() and self._node.get_clock().now() < deadline:
            # 50 ms 轮询：远小于规划时间，既不占 CPU 又能及时取回结果。
            time.sleep(0.05)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """把外旋 XYZ 欧拉角转成单位四元数，返回 ``(x, y, z, w)``。

    约定为外旋 XYZ，即旋转矩阵 ``R = Rz(yaw) · Ry(pitch) · Rx(roll)``（常说的"先 roll 再
    pitch 再 yaw"）；角度单位 rad。返回顺序是 x, y, z, w（``w`` 在最后，与位姿消息的字段
    顺序一致）。同包 ``pose_math`` 中有一个数学等价的同名实现。
    """
    # 0.5 是半角系数：四元数各分量用旋转角的一半做三角运算。
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

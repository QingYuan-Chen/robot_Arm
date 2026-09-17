# 仿真「相对移动」动作服务器节点。
#
# 职责：对外提供一个动作服务，接收「沿 x/y/z 轴平移多少米」的请求，先用仿真限值
# 校验（本文件内的 validate_move_relative_goal），再经运动规划适配器生成轨迹，
# 最后转发给仿真侧关节轨迹控制器执行。
#
# 数据流：
#   动作客户端 → /<arm_namespace>/sim/move_relative（本节点）
#     → 运动规划服务（默认 /plan_kinematic_path）
#     → 关节轨迹动作（默认 /rebotarm/follow_joint_trajectory）
#
# 仿真环境的安全限值（见 validate_move_relative_goal）：
#   - 单次位移不超过 0.05 m；
#   - speed_scale 限定在 [0.05, 1.0]；
#   - 轴向只能是 x / y / z。
# 这些限值在目标接收（goal_callback）与执行（execute_callback）两处各校验一次，
# 防止外部绕过或在排队期间被篡改。
#
# 依赖可降级说明：缺少 ROS 2 运行环境时，本模块仍可被导入（rclpy 置空），
# 仅纯校验逻辑可用，节点无法实例化。

from __future__ import annotations

try:
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
except ImportError:  # pragma: no cover - lets pure validation tests run without ROS2
    # 无 ROS 2 时的占位：让本模块可以只用于纯函数校验测试
    rclpy = None
    ActionClient = None
    ActionServer = None
    CancelResponse = None
    GoalResponse = None
    # 用普通 Exception 占位，保证 main() 中的 except 子句在无 ROS 2 时也可解析
    ExternalShutdownException = Exception
    MultiThreadedExecutor = None
    # Node 用 object 占位，类定义仍然合法
    Node = object

from .models import SafetyViolationError
from .sim_motion_adapter import (
    MoveRelativeSimMotionAdapter,
    build_follow_joint_trajectory_goal,
)


def validate_move_relative_goal(goal) -> dict[str, float | str]:
    """校验并归一化一条相对移动请求，返回可直接展开成关键字参数的字典。

    仿真限值（安全门，不可放宽）：
    - ``axis``：仅接受 x / y / z，大小写与首尾空白会被规整，其它值抛错；
    - ``distance_m``：位移绝对值不得超过 0.05 m，超出即拒绝——仿真中一次大步
      位移容易穿过桌面或自身，宁可要求多次小步；
    - ``speed_scale``：必须落在 [0.05, 1.0]，下界避免慢到看不出动作，上界禁止
      超过额定速度；
    - ``frame_id``：空串回落到 base_link（基座坐标系最直观、最安全）。

    返回值键：``axis``、``distance_m``、``frame_id``、``speed_scale``。
    校验不通过时抛 ``SafetyViolationError``。
    """
    # getattr + 默认值：容忍请求里缺少可选字段
    axis = str(getattr(goal, "axis", "")).strip().lower()
    distance_m = float(getattr(goal, "distance_m", 0.0))
    frame_id = str(getattr(goal, "frame_id", "")).strip() or "base_link"
    speed_scale = float(getattr(goal, "speed_scale", 0.2))

    if axis not in {"x", "y", "z"}:
        raise SafetyViolationError("move_relative axis must be x, y, or z")
    if abs(distance_m) > 0.05:
        raise SafetyViolationError("move_relative distance exceeds 0.05 m in sim")
    if not 0.05 <= speed_scale <= 1.0:
        raise SafetyViolationError("move_relative speed_scale must be within [0.05, 1.0]")

    return {
        "axis": axis,
        "distance_m": distance_m,
        "frame_id": frame_id,
        "speed_scale": speed_scale,
    }


class SimMoveRelativeActionNode(Node):
    """相对移动动作服务器：接收目标、规划、转发执行。

    线程模型：使用多线程执行器（见 ``main``），因此动作回调与轨迹客户端回调
    可以并行推进，长耗时的规划不会阻塞其它回调。

    参数（均通过 ROS 参数声明，可在 launch 中覆盖）：
    - ``arm_namespace``：机械臂命名空间前缀，决定动作服务名；
    - ``group_name``：规划组名，缺省 arm；
    - ``ee_frame_id`` / ``frame_id``：末端坐标系与规划参考坐标系；
    - ``planning_service`` / ``planning_pipeline`` / ``planner_id``：规划器接入点；
    - ``planning_time``、``num_attempts``：单次规划时间上限（秒）与尝试次数；
    - ``goal_position_tolerance``（米）、``goal_orientation_tolerance``（弧度）：
      规划目标容差，容差过大会导致「看起来没走到位」，过小则容易规划失败；
    - ``sim_trajectory_action``：仿真侧关节轨迹动作名。
    """

    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("ROS2 rclpy is required to run SimMoveRelativeActionNode")
        super().__init__("rebotarm_sim_move_relative_action")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ee_frame_id", "tool0")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("planning_service", "/plan_kinematic_path")
        self.declare_parameter("planning_pipeline", "ompl")
        self.declare_parameter("planner_id", "RRTConnect")
        # 规划时间 3 秒：仿真里够用，又能避免长时间卡死
        self.declare_parameter("planning_time", 3.0)
        self.declare_parameter("num_attempts", 3)
        # 位置容差 1 cm、姿态容差 0.05 rad（约 2.9°）
        self.declare_parameter("goal_position_tolerance", 0.01)
        self.declare_parameter("goal_orientation_tolerance", 0.05)
        self.declare_parameter("sim_trajectory_action", "/rebotarm/follow_joint_trajectory")

        # 归一化命名空间，去掉可能写在前后的斜杠，避免拼出双斜杠动作名
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._move_relative_type = self._resolve_action_type()
        self._follow_joint_trajectory_type = self._resolve_follow_joint_trajectory_type()
        self._planner = self._build_planner()
        self._trajectory_client = ActionClient(
            self,
            self._follow_joint_trajectory_type,
            str(self.get_parameter("sim_trajectory_action").value),
        )
        self._adapter = MoveRelativeSimMotionAdapter(
            planner=self._planner,
            trajectory_client=self._trajectory_client,
            current_pose_supplier=self._current_pose_supplier,
            goal_builder=build_follow_joint_trajectory_goal,
            future_waiter=self._wait_for_future,
        )
        self._action_server = ActionServer(
            self,
            self._move_relative_type,
            f"/{self._arm_namespace}/sim/move_relative",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            f"sim move_relative action ready on /{self._arm_namespace}/sim/move_relative"
        )

    def _resolve_action_type(self):
        """解析本节点对外提供的动作消息类型。"""
        try:
            from rebotarm_msgs.action import MoveRelative
        except ImportError as exc:  # pragma: no cover - depends on ROS2 workspace
            raise SafetyViolationError("rebotarm_msgs.action.MoveRelative is required") from exc
        return MoveRelative

    def _resolve_follow_joint_trajectory_type(self):
        """解析仿真控制器所用的关节轨迹动作消息类型。"""
        try:
            from control_msgs.action import FollowJointTrajectory
        except ImportError as exc:  # pragma: no cover - depends on ROS2 workspace
            raise SafetyViolationError("control_msgs.action.FollowJointTrajectory is required") from exc
        return FollowJointTrajectory

    def _build_planner(self):
        """构造运动规划器实例，参数全部来自本节点声明的 ROS 参数。"""
        from rebotarm_motion.moveit_planner import MoveItMotionPlanner

        planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("group_name").value),
            ee_frame_id=str(self.get_parameter("ee_frame_id").value),
            frame_id=str(self.get_parameter("frame_id").value),
            planning_service=str(self.get_parameter("planning_service").value),
            planning_pipeline=str(self.get_parameter("planning_pipeline").value),
            planner_id=str(self.get_parameter("planner_id").value),
            planning_time=float(self.get_parameter("planning_time").value),
            num_attempts=int(self.get_parameter("num_attempts").value),
            goal_position_tolerance=float(self.get_parameter("goal_position_tolerance").value),
            goal_orientation_tolerance=float(self.get_parameter("goal_orientation_tolerance").value),
        )
        return planner

    def _goal_callback(self, goal_request) -> GoalResponse:
        """目标准入检查：不合法直接拒绝，连排队都不进。

        在这里拦一道是为了尽早反馈，避免无效目标占用执行器与规划时间。
        """
        try:
            validate_move_relative_goal(goal_request)
        except SafetyViolationError as exc:
            self.get_logger().warn(f"reject move_relative goal: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        """取消请求一律接受：相对小位移可随时中止，不应被拒绝。"""
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """执行一次相对移动，并分阶段发布反馈。

        反馈进度约定：``planning`` 0.2 → ``dispatching`` 0.6 → ``executing`` 0.9，
        最终结果给出 success/message。任何失败路径都以 ``abort()`` 结束，
        保证客户端能明确区分「成功」与「中断」。
        """
        result_msg = self._move_relative_type.Result()
        feedback_msg = self._move_relative_type.Feedback()
        try:
            # 二次校验：准入之后目标仍在队列中等待，此处重新校验以防变化
            request = validate_move_relative_goal(goal_handle.request)
            feedback_msg.state = "planning"
            feedback_msg.progress = 0.2
            goal_handle.publish_feedback(feedback_msg)

            # 3 秒内等不到仿真控制器就放弃，避免无限等待
            if not self._trajectory_client.wait_for_server(timeout_sec=3.0):
                result_msg.success = False
                result_msg.message = "sim trajectory controller unavailable"
                goal_handle.abort()
                return result_msg

            feedback_msg.state = "dispatching"
            feedback_msg.progress = 0.6
            goal_handle.publish_feedback(feedback_msg)

            adapter_result = self._adapter.execute_move_relative(**request)
            if not adapter_result.success:
                result_msg.success = False
                result_msg.message = adapter_result.message
                goal_handle.abort()
                return result_msg

            feedback_msg.state = "executing"
            feedback_msg.progress = 0.9
            goal_handle.publish_feedback(feedback_msg)

            result_msg.success = True
            result_msg.message = adapter_result.message
            goal_handle.succeed()
            return result_msg
        except SafetyViolationError as exc:
            # 参数类错误：直接把原因回给客户端
            result_msg.success = False
            result_msg.message = str(exc)
            goal_handle.abort()
            return result_msg
        except Exception as exc:  # pragma: no cover - runtime safety
            # 兜底：任何未预期异常都必须转成 abort，绝不能让目标悬空
            self.get_logger().exception("move_relative execution failed")
            result_msg.success = False
            result_msg.message = f"move_relative execution failed: {exc}"
            goal_handle.abort()
            return result_msg

    def _current_pose_supplier(self) -> dict[str, float]:
        """提供当前末端位姿（位置米、姿态欧拉角弧度）。

        当前为固定占位值：x=0.2, y=0.0, z=0.2，姿态全零。仿真自检阶段用它定义
        一个确定的固定起始位姿，使相对位移结果可复现；接入真实反馈后应替换。
        """
        return {"x": 0.2, "y": 0.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def _wait_for_future(self, future, *, timeout_sec: float) -> None:
        """在单线程模式下等待 future 完成（阻塞至多 timeout_sec 秒）。"""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)


def main(args=None) -> None:
    """节点进程入口：启动多线程执行器并常驻，直到中断或外部关闭。"""
    if rclpy is None:
        raise RuntimeError("ROS2 rclpy is required to run rebotarm_sim_move_relative_action")
    rclpy.init(args=args)
    node = SimMoveRelativeActionNode()
    # 4 线程：规划、轨迹客户端回调与动作服务回调需要并发推进
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # 正常退出路径：Ctrl-C 或外部关闭，都不算错误
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

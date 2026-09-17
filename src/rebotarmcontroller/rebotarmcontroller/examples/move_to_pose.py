#!/usr/bin/env python3
"""末端位姿点到点演示：让末端执行器走到给定位姿。

用途：现场手动验证/调试用的小工具（控制台入口 ``MoveToPose``），不属于生产流程。
它只负责把目标位姿发出去并打印反馈：IK、轨迹规划与限速全部由控制器侧完成，
控制器在硬件未显式使能时会直接拒绝目标，因此运行前必须已完成显式使能。

目标的表达：位置为米（m），姿态为单位四元数 ``(x, y, z, w)``，参考系为基座
（base_link）；``duration`` 为期望运动时长（s），``<= 0`` 时由底层按末端移动
距离自行估算。

退出码：成功 0，任一环节失败 1。
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.action import MoveToPose
from sensor_msgs.msg import JointState


class DemoMoveToPose(Node):
    """把命令行给的位姿组装成动作目标并等待结果的动作客户端节点。

    生命周期：构造时订阅关节状态（仅用于等待控制器反馈就绪）并创建动作客户端；
    ``run`` 内用单线程 ``spin_until_future_complete`` 顺序等待"服务器就绪 -> 目标
    被接受 -> 结果返回"，因此不需要执行器线程。节点自身不使能硬件，也不做碰撞检查。
    """

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("move_to_pose")
        # 命名空间统一去掉首尾斜杠，避免拼出双斜杠话题名。
        self._namespace = args.namespace.strip("/")
        # 直接持有 argparse 结果：目标字段在 run() 里逐项取出。
        self._target = args
        self._latest_joint_state: JointState | None = None
        # 订阅关节状态只是为了让演示脚本等到控制器已经在发反馈再下发目标。
        self.create_subscription(
            JointState,
            f"/{self._namespace}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        # 控制器的位姿动作服务器；硬件未显式使能时目标会被拒绝。
        self._move_to_pose = ActionClient(
            self,
            MoveToPose,
            f"/{self._namespace}/move_to_pose",
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def run(self) -> bool:
        """等待反馈与动作服务器、发送位姿目标并等待结果；成功返回 True。

        返回 False 的四种情况：拿不到 joint_states、动作服务器 5 s 内不可用、
        目标被拒绝（通常是硬件未使能）、或结果里 ``success`` 为假
        （规划失败、取消或被抢占）。
        """
        if not self._wait_for_joint_state():
            self.get_logger().error("joint_states not available")
            return False

        if not self._move_to_pose.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_to_pose action not available")
            return False

        goal = MoveToPose.Goal()
        goal.target_pose = Pose()
        # 位置单位为米，姿态为单位四元数（默认 0,0,0,1 表示不改变姿态）。
        goal.target_pose.position.x = float(self._target.x)
        goal.target_pose.position.y = float(self._target.y)
        goal.target_pose.position.z = float(self._target.z)
        goal.target_pose.orientation.x = float(self._target.qx)
        goal.target_pose.orientation.y = float(self._target.qy)
        goal.target_pose.orientation.z = float(self._target.qz)
        goal.target_pose.orientation.w = float(self._target.qw)
        # 期望运动时长（s）；<= 0 表示交给底层按末端移动距离估算。
        goal.duration = float(self._target.duration)

        send_future = self._move_to_pose.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        # 阻塞式自旋等待：本演示是单线程脚本，不需要额外的执行器。
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"success={result.success} message={result.message}")
        return bool(result.success)

    def _feedback_cb(self, feedback_msg) -> None:
        """动作反馈回调：打印进度与已用时长。

        ``progress`` 为 0~1 的估算值（给了期望时长就按时长比例，否则按已发送的
        轨迹点占比）；``time_elapsed`` 为自收到目标起的秒数。这里不节流，因为
        控制器侧本身以 20 Hz 限频发布反馈。
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"progress={feedback.progress:.2f} elapsed={feedback.time_elapsed:.2f}s"
        )

    def _wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        """在 timeout_sec 内轮询等待第一帧关节状态；超时返回 False。"""
        # 用单调时钟做死线，避免系统时间跳变影响超时判定。
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self._latest_joint_state is None:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state is not None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    # 控制器命名空间，与 arm_namespace 参数一致；统一去掉首尾斜杠。
    parser.add_argument("--namespace", default="rebotarm")
    # 目标位置（m），参考系为基座 base_link：默认机械臂前方 0.30 m、高 0.30 m。
    parser.add_argument("--x", type=float, default=0.30)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.30)
    # 目标姿态四元数 (x, y, z, w)，默认单位四元数表示保持基座姿态（不旋转）。
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    # 期望运动时长（s），默认 2.0；<= 0 时由底层按末端移动距离估算。
    parser.add_argument("--duration", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    """控制台入口：解析参数、运行演示，并按结果设置退出码（成功 0 / 失败 1）。"""
    args = _parse_args()
    rclpy.init()
    node = DemoMoveToPose(args)
    try:
        ok = node.run()
    except Exception as exc:
        # 任何异常都按失败处理，但节点与 rclpy 仍要在 finally 里关闭。
        node.get_logger().error(str(exc))
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

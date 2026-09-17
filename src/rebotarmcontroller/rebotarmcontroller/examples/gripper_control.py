#!/usr/bin/env python3
"""夹爪开合交互演示：可直接运行的现场调试客户端。

流程
    1. 调用 ``/rebotarm/enable`` 显式使能硬件（真机默认失能，不使能任何夹爪命令都会被
       硬件层拒绝）；
    2. 等待 ``/rebotarm/gripper/set`` 服务可用，然后进入命令行循环，输入 o/open 张开、
       c/close 闭合、q/quit/exit 退出；
    3. 退出时（含异常与 Ctrl+C）一定走 cleanup：先把夹爪合上，再调用
       ``/rebotarm/disable``，避免演示结束后夹具停在张开姿态或电机继续使能。

职责边界
    这是运维/调试用的演示客户端，不参与生产链路：只通过服务访问硬件，不直接碰串口、
    不做规划、不发布命令话题。它同步等待每个服务响应，因此不要当作可组合节点复用。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rebotarm_msgs.srv import SetGripper
from std_srvs.srv import Trigger

_NAMESPACE = "rebotarm"  # 机械臂命名空间：服务名统一为 /<namespace>/...
_OPEN_POSITION_M = 0.09  # "全开"请求开度（米）；硬件层会把超过实测机械上限 0.085 m 的请求裁剪掉
_CLOSE_POSITION_M = 0.0  # "闭合"请求开度（米），0 表示完全闭合
_MAX_EFFORT = 0.0  # 最大夹持力矩（N·m）；<=0 表示由硬件层取默认夹持力


class DemoGripperControl(Node):
    """演示节点：持有使能/失能与夹爪设置三类服务的客户端。

    生命周期很短，全部动作在 run() 中同步完成，因此成员只在单线程中被读写。
    ``_enabled_by_demo`` 记录"使能是本演示打开的"：只有它成立时 cleanup 才会去失能，
    避免误动别人已经开启的硬件状态。
    """

    def __init__(self) -> None:
        super().__init__("gripper_control")
        self._enabled_by_demo = False
        self._enable = self.create_client(Trigger, f"/{_NAMESPACE}/enable")
        self._disable = self.create_client(Trigger, f"/{_NAMESPACE}/disable")
        self._client = self.create_client(SetGripper, f"/{_NAMESPACE}/gripper/set")

    def run(self) -> bool:
        """执行一次交互式演示。

        返回 True 表示用户正常退出，False 表示使能失败或夹爪服务不可用；调用方据此
        决定进程退出码。期间会阻塞在标准输入上，属于前台交互程序。
        """
        # 真机默认失能：必须显式使能后才允许下发夹爪命令
        if not self._call_trigger(self._enable, "enable"):
            return False
        self._enabled_by_demo = True

        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("gripper set service not available")
            return False

        self.get_logger().info("commands: o/open, c/close, q/quit")
        while rclpy.ok():
            try:
                command = input("gripper> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                # 非交互输入（管道/重定向）或 Ctrl+C：按退出处理，先换行避免日志粘在提示符后
                print()
                break

            if command in ("q", "quit", "exit"):
                break
            if command in ("o", "open"):
                self._set_gripper(_OPEN_POSITION_M, "open")
                continue
            if command in ("c", "close"):
                self._set_gripper(_CLOSE_POSITION_M, "close")
                continue
            self.get_logger().info("commands: o/open, c/close, q/quit")
        return True

    def cleanup(self) -> None:
        """收尾：仅在本次演示使能过时才动硬件，先合上夹爪再失能。

        夹爪先闭合是为了不把夹具留在张开姿态；失能后电机不再输出力矩，姿态由重力与
        机械结构决定。
        """
        if self._enabled_by_demo:
            self._set_gripper(_CLOSE_POSITION_M, "close")
            self._call_trigger(self._disable, "disable")

    def _call_trigger(self, client, label: str, timeout_sec: float = 5.0) -> bool:
        """同步调用一个标准触发器服务。

        返回 True 仅当服务返回 success；服务不可用、等待响应超时、success 为假都返回
        False 并记录错误日志。注意：服务可用性探测固定等待 5.0 秒，形参 timeout_sec
        只作用于等待响应（默认 5.0 秒）。
        """
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{label} service not available")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(f"{label} timed out")
            return False
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.get_logger().error(f"{label} failed: {message}")
            return False
        self.get_logger().info(message if (message := result.message) else f"{label} OK")
        return True

    def _set_gripper(self, position: float, label: str) -> bool:
        """请求夹爪运动到指定开度。

        position 单位为米（0 = 闭合），max_effort 单位为 N·m，0 表示用硬件层默认夹持力，
        硬件层会先把开度裁剪到可信行程 [0, 0.085] m。返回 True 表示服务报告在超时内
        到达目标；失败时也会打印实际到达开度，便于区分"夹到物体停住"和"完全没动作"。
        等待响应未设超时：服务端长时间不回复时本调用会一直阻塞。
        """
        request = SetGripper.Request()
        request.position = float(position)
        request.max_effort = _MAX_EFFORT

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error(f"{label} failed: no response")
            return False
        if not result.success:
            self.get_logger().warn(
                f"{label} not reached, current={result.reached_position:.3f}m"
            )
            return False
        self.get_logger().info(
            f"{label} reached, current={result.reached_position:.3f}m"
        )
        return True


def main() -> None:
    rclpy.init()
    node = DemoGripperControl()
    try:
        ok = node.run()
    except Exception as exc:
        # 交互过程中的任何异常都折算成失败退出，日志保留原始信息
        node.get_logger().error(str(exc))
        ok = False
    finally:
        # 无论成败都必须收尾，否则可能把硬件留在使能状态
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()

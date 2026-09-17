"""语音控制包的常驻节点骨架（当前为演练模式占位节点）。

职责：向系统提供一个稳定存在的节点名，供启动组合包与监控确认语音控制栈已就绪；
节点本身不订阅话题、不发布消息、不创建服务，也不读取 execution_mode 之外的
运行参数；它只记录自身处于"演练模式"日志，并提示交互式文本调试应使用独立的
命令行入口（本节点自身不含解析与路由逻辑）。

配置目录在构造时解析为 <包根>/config，与各命令行入口保持一致，便于后续在此
节点内接入解析/路由逻辑时不改变路径约定。

可导入性：ROS 2 运行时不可用时（如仅做纯离线配置校验的环境），导入期回退为
``rclpy = None``、``Node = object``，使模块仍可被导入而不抛 ImportError；
真正运行入口会在 ``main`` 中显式检查并报错。
"""

from __future__ import annotations

from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None
    Node = object


class VoiceControlNode(Node):
    """占位节点：仅声明节点名与配置目录，不承载业务逻辑。

    节点名固定为 "rebotarm_voice_control_node"，与启动文件中 ``name`` 及
    setup.py 注册的可执行文件名一致，改名会影响启动与监控。
    """

    def __init__(self):
        super().__init__("rebotarm_voice_control_node")
        package_root = Path(__file__).resolve().parents[1]
        self.config_root = package_root / "config"
        self.get_logger().info(
            "rebotarm_voice_control_node ready in dry-run mode; "
            "use rebotarm_text_input for interactive text MVP"
        )


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run rebotarm_voice_control_node")
    rclpy.init()
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    finally:
        # 无论 spin 因何结束都做资源清理，避免残留节点与未关闭的上下文。
        node.destroy_node()
        rclpy.shutdown()

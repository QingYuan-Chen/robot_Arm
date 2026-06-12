from __future__ import annotations

from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None
    Node = object


class VoiceControlNode(Node):
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
        node.destroy_node()
        rclpy.shutdown()

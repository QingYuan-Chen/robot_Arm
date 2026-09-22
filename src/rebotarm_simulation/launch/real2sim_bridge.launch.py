"""Start only the read-only Real2Sim bridge; no hardware node is launched."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("rebotarm_simulation"))
        / "config"
        / "real2sim_bridge.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_real2sim_bridge",
                name="rebotarm_real2sim_bridge",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )

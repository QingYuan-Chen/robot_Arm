"""Start only the explicitly selected, headless MuJoCo ROS adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _default_python_executable() -> str:
    workspace_root = Path(__file__).resolve().parents[3]
    pinned = workspace_root / "third_party" / "rebotarm_mujoco_venv" / "bin" / "python"
    return str(pinned) if pinned.is_file() else "python3"


def generate_launch_description():
    config = Path(get_package_share_directory("rebotarm_simulation")) / "config" / "mujoco_sim.yaml"
    python_executable = LaunchConfiguration("python_executable")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "python_executable",
                default_value=_default_python_executable(),
                description="Python interpreter containing MuJoCo and ROS 2 dependencies",
            ),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                prefix=python_executable,
                parameters=[str(config), {"backend": "mujoco", "headless": True}],
            )
        ]
    )

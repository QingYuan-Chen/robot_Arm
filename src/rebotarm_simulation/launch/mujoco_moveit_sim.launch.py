from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_python_executable() -> str:
    workspace_root = Path(__file__).resolve().parents[3]
    pinned = workspace_root / "third_party" / "rebotarm_mujoco_venv" / "bin" / "python"
    return str(pinned) if pinned.is_file() else "python3"


def generate_launch_description():
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    simulation_share = FindPackageShare("rebotarm_simulation")
    arm_namespace = LaunchConfiguration("arm_namespace")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    python_executable = LaunchConfiguration("python_executable")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    upstream_model = PathJoinSubstitution(
        [simulation_share, "models", "rebotarm", "scene.xml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("python_executable", default_value=_default_python_executable()),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                prefix=python_executable,
                parameters=[
                    {
                        "backend": "mujoco",
                        "headless": True,
                        "model_path": upstream_model,
                        "arm_namespace": arm_namespace,
                        "publish_rate_hz": publish_rate_hz,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])
                ),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": "false",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )

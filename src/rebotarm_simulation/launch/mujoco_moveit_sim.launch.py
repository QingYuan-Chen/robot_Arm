from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    simulation_share = FindPackageShare("rebotarm_simulation")
    arm_namespace = LaunchConfiguration("arm_namespace")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    python_executable = LaunchConfiguration("python_executable")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")
    upstream_model = PathJoinSubstitution(
        [simulation_share, "models", "rebotarm", "scene.xml"]
    )

    default_mujoco_python = PathJoinSubstitution(
        [EnvironmentVariable("PWD", default_value="."), "third_party", "rebotarm_mujoco_venv", "bin", "python"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "use_mujoco_viewer",
                default_value="true",
                description="Open a native MuJoCo viewer for visual inspection",
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument(
                "python_executable",
                default_value=EnvironmentVariable(
                    "REBOTARM_MUJOCO_PYTHON",
                    default_value=default_mujoco_python,
                ),
                description=(
                    "Python interpreter containing MuJoCo; set an absolute path "
                    "or REBOTARM_MUJOCO_PYTHON when launching from another directory"
                ),
            ),
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
                        "show_viewer": use_mujoco_viewer,
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

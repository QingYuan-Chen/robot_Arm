"""Start the MuJoCo and MoveIt integration without desktop windows."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation_share = FindPackageShare("rebotarm_simulation")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [simulation_share, "launch", "mujoco_moveit_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_rviz": "false",
                    "use_mujoco_viewer": "false",
                }.items(),
            )
        ]
    )

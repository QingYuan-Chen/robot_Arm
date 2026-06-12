from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "interactive_system.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_hardware": "false",
                    "use_local_rviz": "true",
                    "use_moveit_preview": "true",
                    "start_passive_joint_state_publisher": "true",
                    "use_moveit_fake_joint_states": "true",
                }.items(),
            )
        ]
    )

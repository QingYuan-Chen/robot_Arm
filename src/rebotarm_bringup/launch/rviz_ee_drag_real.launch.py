from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_channel(context: LaunchContext) -> str:
    channel = LaunchConfiguration("channel").perform(context)
    if channel and channel != "auto":
        return channel
    for candidate in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if Path(candidate).exists():
            return candidate
    return "/dev/ttyACM0"


def _include_system(context: LaunchContext):
    bringup_share = FindPackageShare("rebotarm_bringup")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "interactive_system.launch.py"])
            ),
            launch_arguments={
                "channel": _resolve_channel(context),
                "use_hardware": "true",
                "use_local_rviz": "true",
                "use_moveit_preview": "true",
                "start_passive_joint_state_publisher": "false",
                "use_moveit_fake_joint_states": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("channel", default_value="auto"),
            OpaqueFunction(function=_include_system),
        ]
    )

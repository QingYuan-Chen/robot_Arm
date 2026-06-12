from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    execution_mode = LaunchConfiguration("execution_mode")
    event_jsonl = LaunchConfiguration("event_jsonl")
    return LaunchDescription(
        [
            DeclareLaunchArgument("execution_mode", default_value="dry_run"),
            DeclareLaunchArgument("event_jsonl", default_value=""),
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_realtime_gateway",
                name="rebotarm_realtime_gateway",
                output="screen",
                arguments=[
                    "--event-jsonl",
                    event_jsonl,
                    "--mode",
                    execution_mode,
                ],
            ),
        ]
    )

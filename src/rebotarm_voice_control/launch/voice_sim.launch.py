from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    execution_mode = LaunchConfiguration("execution_mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument("execution_mode", default_value="sim"),
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_voice_control_node",
                name="rebotarm_voice_control_node",
                output="screen",
                parameters=[
                    {
                        "execution_mode": execution_mode,
                    }
                ],
            ),
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_sim_move_relative_action",
                name="rebotarm_sim_move_relative_action",
                output="screen",
            ),
        ]
    )

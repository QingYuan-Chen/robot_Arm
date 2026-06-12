from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    record_path = LaunchConfiguration("record_path")
    auto_start_gravity_comp = LaunchConfiguration("auto_start_gravity_comp")
    teleop_config = LaunchConfiguration("teleop_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    interactive_share = FindPackageShare("rebotarm_interactive_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument("auto_start_gravity_comp", default_value="false"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="",
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=PathJoinSubstitution(
                    [interactive_share, "config", "teleop_control.yaml"]
                ),
            ),
            Node(
                package="rebotarm_teach",
                executable="TeachRecorderNode",
                name="teach_recorder_node",
                output="screen",
                prefix=keyboard_prefix,
                parameters=[
                    teleop_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "auto_start_gravity_comp": auto_start_gravity_comp,
                    },
                ],
            ),
        ]
    )

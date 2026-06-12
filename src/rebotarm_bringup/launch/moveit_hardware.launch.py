import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    moveit_share = FindPackageShare("rebotarm_moveit_config")

    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    arm_namespace = LaunchConfiguration("arm_namespace")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    teach_record_path = LaunchConfiguration("teach_record_path")
    teach_record_rate_hz = LaunchConfiguration("teach_record_rate_hz")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    use_rviz = LaunchConfiguration("use_rviz")

    demo_launch = PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "gripper.yaml"]
                ),
            ),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="200.0"),
            DeclareLaunchArgument("teach_record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument("teach_record_rate_hz", default_value="150.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[
                    {
                        "arm_config": arm_config,
                        "gripper_config": gripper_config,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "teach_record_path": teach_record_path,
                        "teach_record_rate_hz": teach_record_rate_hz,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": "false",
                }.items(),
            ),
        ]
    )

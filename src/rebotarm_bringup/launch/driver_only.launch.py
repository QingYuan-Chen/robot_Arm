# 该启动文件仅启动机械臂控制器节点，适用于需要单独控制机械臂的场景。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    grasp_hold_timeout_sec = LaunchConfiguration("grasp_hold_timeout_sec")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="0.5"),
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
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
                        "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
                        "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
                        "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
                        "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
                        "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
                        "grasp_hold_timeout_sec": grasp_hold_timeout_sec,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
        ]
    )

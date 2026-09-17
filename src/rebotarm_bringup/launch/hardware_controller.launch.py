"""真实机械臂控制器的唯一底层启动片段。

本文件只声明硬件公共参数并启动 ``reBotArmController``。它不启动 MoveIt、RViz、
robot_state_publisher、示教、遥操作、视觉或网页节点。所有需要真实硬件的 bringup
组合都应包含本文件，避免复制控制器节点和安全参数。

安全语义保持不变：控制器启动后处于失能状态，必须获得新鲜反馈、完成现场检查并显式
调用 ``/<arm_namespace>/enable`` 后才允许运动；默认退出不自动回位。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")

    controller_parameters = {
        "arm_config": LaunchConfiguration("arm_config"),
        "gripper_config": LaunchConfiguration("gripper_config"),
        "channel": LaunchConfiguration("channel"),
        "shutdown_safe_home": LaunchConfiguration("shutdown_safe_home"),
        "joint_state_rate": LaunchConfiguration("joint_state_rate"),
        "hardware_feedback_rate_hz": LaunchConfiguration("hardware_feedback_rate_hz"),
        "gripper_position_torque_cap_nm": LaunchConfiguration(
            "gripper_position_torque_cap_nm"
        ),
        "gripper_position_max_speed_rad_s": LaunchConfiguration(
            "gripper_position_max_speed_rad_s"
        ),
        "gripper_position_timeout_margin_sec": LaunchConfiguration(
            "gripper_position_timeout_margin_sec"
        ),
        "gripper_feedback_stale_timeout_sec": LaunchConfiguration(
            "gripper_feedback_stale_timeout_sec"
        ),
        "grasp_hold_timeout_sec": LaunchConfiguration("grasp_hold_timeout_sec"),
        "cmd_arbitration": LaunchConfiguration("cmd_arbitration"),
        "arm_namespace": LaunchConfiguration("arm_namespace"),
        "frame_id": LaunchConfiguration("frame_id"),
        "ee_frame_id": LaunchConfiguration("ee_frame_id"),
    }

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
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[controller_parameters],
            ),
        ]
    )

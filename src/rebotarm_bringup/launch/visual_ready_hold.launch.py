from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")

    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    visual_ready_duration_sec = LaunchConfiguration("visual_ready_duration_sec")
    visual_ready_wait_timeout_sec = LaunchConfiguration("visual_ready_wait_timeout_sec")
    visual_ready_max_start_delta_rad = LaunchConfiguration("visual_ready_max_start_delta_rad")
    visual_ready_startup_delay_sec = LaunchConfiguration("visual_ready_startup_delay_sec")
    visual_ready_params = PathJoinSubstitution([vision_share, "config", "visual_ready.yaml"])

    controller = Node(
        package="rebotarmcontroller",
        executable="reBotArmController",
        name="reBotArmController",
        output="screen",
        parameters=[
            {
                "arm_config": arm_config,
                "gripper_config": gripper_config,
                "channel": channel,
                "shutdown_safe_home": shutdown_safe_home,
                "joint_state_rate": joint_state_rate,
                "cmd_arbitration": cmd_arbitration,
                "arm_namespace": arm_namespace,
            }
        ],
    )
    visual_ready_startup = Node(
        package="rebotarm_vision",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready_startup",
        output="screen",
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": True,
                "exit_after_startup_move": True,
                "startup_delay_sec": visual_ready_startup_delay_sec,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )
    visual_ready_service = Node(
        package="rebotarm_vision",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready",
        output="screen",
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": False,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )

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
            DeclareLaunchArgument("channel", default_value="auto"),
            DeclareLaunchArgument("shutdown_safe_home", default_value="true"),
            DeclareLaunchArgument("joint_state_rate", default_value="200.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("visual_ready_joint_positions", default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]"),
            DeclareLaunchArgument("visual_ready_duration_sec", default_value="4.0"),
            DeclareLaunchArgument("visual_ready_wait_timeout_sec", default_value="12.0"),
            DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="2.5"),
            DeclareLaunchArgument("visual_ready_startup_delay_sec", default_value="0.0"),
            controller,
            visual_ready_startup,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=visual_ready_startup,
                    on_exit=[visual_ready_service],
                )
            ),
        ]
    )

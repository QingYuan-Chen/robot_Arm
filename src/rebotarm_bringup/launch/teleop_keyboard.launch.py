from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_hardware = LaunchConfiguration("use_hardware")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    teach_record_path = LaunchConfiguration("teach_record_path")
    teach_record_rate_hz = LaunchConfiguration("teach_record_rate_hz")
    teleop_config = LaunchConfiguration("teleop_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    bringup_share = FindPackageShare("rebotarm_bringup")
    interactive_share = FindPackageShare("rebotarm_interactive_control")
    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.urdf"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_hardware", default_value="false"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="200.0"),
            DeclareLaunchArgument("teach_record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument("teach_record_rate_hz", default_value="150.0"),
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=PathJoinSubstitution(
                    [interactive_share, "config", "teleop_control.yaml"]
                ),
            ),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                condition=IfCondition(use_hardware),
                parameters=[
                    {
                        "arm_config": arm_config,
                        "gripper_config": gripper_config,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "teach_record_path": teach_record_path,
                        "teach_record_rate_hz": teach_record_rate_hz,
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
            Node(
                package="rebotarm_teleop",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[teleop_config, {"arm_namespace": arm_namespace}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="teleop_joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_hardware),
                parameters=[{"robot_description": robot_description}, {"rate": 30.0}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rebotarm_teleop",
                executable="TeleopKeyboardNode",
                name="teleop_keyboard_node",
                output="screen",
                prefix=keyboard_prefix,
                parameters=[teleop_config, {"arm_namespace": arm_namespace}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_local_rviz),
            ),
        ]
    )

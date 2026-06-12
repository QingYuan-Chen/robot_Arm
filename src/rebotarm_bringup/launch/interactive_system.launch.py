import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    bringup_share = FindPackageShare("rebotarm_bringup")
    interactive_share = FindPackageShare("rebotarm_interactive_control")
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    use_moveit_preview = LaunchConfiguration("use_moveit_preview")
    use_hardware = LaunchConfiguration("use_hardware")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    interactive_config = LaunchConfiguration("interactive_config")
    start_passive_joint_state_publisher = LaunchConfiguration("start_passive_joint_state_publisher")
    use_moveit_fake_joint_states = LaunchConfiguration("use_moveit_fake_joint_states")
    rviz_config = LaunchConfiguration("rviz_config")

    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.urdf"]
    )
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    ompl_planning_yaml = load_yaml(
        "rebotarm_moveit_config", "config/ompl_planning.yaml"
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
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("shutdown_safe_home", default_value="true"),
            DeclareLaunchArgument("joint_state_rate", default_value="200.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("use_moveit_preview", default_value="false"),
            DeclareLaunchArgument("use_hardware", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("start_passive_joint_state_publisher", default_value="true"),
            DeclareLaunchArgument("use_moveit_fake_joint_states", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([bringup_share, "rviz", "interactive_system.rviz"]),
            ),
            DeclareLaunchArgument(
                "interactive_config",
                default_value=PathJoinSubstitution(
                    [interactive_share, "config", "interactive_control.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])
                ),
                condition=IfCondition(use_moveit_preview),
                launch_arguments={
                    "use_rviz": "false",
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": PythonExpression(
                        [
                            "'false' if '",
                            use_hardware,
                            "'.lower() == 'true' or '",
                            use_moveit_fake_joint_states,
                            "'.lower() != 'true' else 'true'",
                        ]
                    ),
                }.items(),
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
                        "shutdown_safe_home": shutdown_safe_home,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
                    }
                ],
            ),
            Node(
                package="rebotarm_teleop",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[interactive_config, {"arm_namespace": arm_namespace}],
                condition=UnlessCondition(use_moveit_preview),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                condition=UnlessCondition(use_moveit_preview),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="interactive_joint_state_publisher",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_hardware,
                            "'.lower() != 'true' and '",
                            start_passive_joint_state_publisher,
                            "'.lower() == 'true'",
                        ]
                    )
                ),
                parameters=[
                    {"robot_description": robot_description},
                    {"rate": 30.0},
                ],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    ompl_planning_yaml,
                ],
                condition=IfCondition(use_local_rviz),
            ),
        ]
    )

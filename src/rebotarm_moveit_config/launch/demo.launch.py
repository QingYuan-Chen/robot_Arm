import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_fake_joint_states = LaunchConfiguration("use_fake_joint_states")
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    rviz_config = PathJoinSubstitution([moveit_share, "rviz", "moveit.rviz"])

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

    sensors_3d = {
        "sensors": ["no_depth_sensor"],
        "no_depth_sensor": {"sensor_plugin": "~"},
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_debug_summary = yaml.safe_dump(
        {
            "planning_plugins": ompl_planning_yaml.get("planning_plugins"),
            "request_adapters": ompl_planning_yaml.get("request_adapters"),
            "response_adapters": ompl_planning_yaml.get("response_adapters"),
            "planner_configs": list(
                (ompl_planning_yaml.get("planner_configs") or {}).keys()
            ),
            "arm": ompl_planning_yaml.get("arm"),
        },
        sort_keys=False,
        allow_unicode=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_fake_joint_states", default_value="true"),
            LogInfo(msg="move_group planning params:\n" + planning_debug_summary),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="screen",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            ),
            Node(
                package="rebotarm_interactive_control",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="moveit_demo_joint_state_publisher",
                output="screen",
                condition=IfCondition(use_fake_joint_states),
                parameters=[
                    moveit_config.robot_description,
                    {"rate": 30.0},
                ],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                parameters=[
                    moveit_config.to_dict(),
                    ompl_planning_yaml,
                    trajectory_execution,
                    sensors_3d,
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    ompl_planning_yaml,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

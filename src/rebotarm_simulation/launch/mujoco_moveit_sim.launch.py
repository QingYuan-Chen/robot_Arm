from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    arm_namespace = LaunchConfiguration("arm_namespace")
    model_xml = LaunchConfiguration("model_xml")
    metrics_dir = LaunchConfiguration("metrics_dir")
    metrics_sample_stride = LaunchConfiguration("metrics_sample_stride")
    use_mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    execution_timeout_margin_sec = LaunchConfiguration("execution_timeout_margin_sec")
    joint_limits_yaml = LaunchConfiguration("joint_limits_yaml")
    motor_profile = LaunchConfiguration("motor_profile")
    python_executable = LaunchConfiguration("python_executable")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("model_xml", default_value="build/mujoco_models/reBot-DevArm_gripper_physics.xml"),
            DeclareLaunchArgument("metrics_dir", default_value="build/mujoco_runs/latest"),
            DeclareLaunchArgument("metrics_sample_stride", default_value="1"),
            DeclareLaunchArgument("use_mujoco_viewer", default_value="false"),
            DeclareLaunchArgument("control_rate_hz", default_value="200.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("execution_timeout_margin_sec", default_value="2.0"),
            DeclareLaunchArgument(
                "joint_limits_yaml",
                default_value="src/rebotarm_moveit_config/config/joint_limits.yaml",
            ),
            DeclareLaunchArgument("motor_profile", default_value="current"),
            DeclareLaunchArgument("python_executable", default_value="third_party/rebotarm_mujoco_venv/bin/python"),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_adapter",
                name="rebotarm_mujoco_adapter",
                output="screen",
                prefix=python_executable,
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "model_xml": model_xml,
                        "metrics_dir": metrics_dir,
                        "metrics_sample_stride": metrics_sample_stride,
                        "use_mujoco_viewer": use_mujoco_viewer,
                        "control_rate_hz": control_rate_hz,
                        "publish_rate_hz": publish_rate_hz,
                        "execution_timeout_margin_sec": execution_timeout_margin_sec,
                        "joint_limits_yaml": joint_limits_yaml,
                        "motor_profile": motor_profile,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])
                ),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": "false",
                }.items(),
            ),
        ]
    )

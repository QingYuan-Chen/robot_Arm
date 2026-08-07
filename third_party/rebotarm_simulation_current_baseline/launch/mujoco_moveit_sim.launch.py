from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_repo_root() -> str:
    source_candidate = Path(__file__).resolve().parents[3]
    snapshot = source_candidate / "third_party/robotarm_ros2_mujoco_snapshot"
    return str(source_candidate if snapshot.is_dir() else Path.cwd())


def _validate_simulation_backend(context, *args, **kwargs):
    backend = LaunchConfiguration("simulation_backend").perform(context).strip().lower()
    if backend not in {"upstream", "current"}:
        raise RuntimeError("simulation_backend must be 'upstream' or 'current'")
    return []


def generate_launch_description():
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    simulation_backend = LaunchConfiguration("simulation_backend")
    repo_root = LaunchConfiguration("repo_root")
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    upstream_condition = IfCondition(
        PythonExpression(["'", simulation_backend, "' == 'upstream'"])
    )
    current_condition = IfCondition(
        PythonExpression(["'", simulation_backend, "' == 'current'"])
    )
    backend_use_sim_time = PythonExpression(
        ["'false' if '", simulation_backend, "' == 'current' else '", use_sim_time, "'"]
    )
    upstream_model = PathJoinSubstitution(
        [
            repo_root,
            "third_party",
            "robotarm_ros2_mujoco_snapshot",
            "src",
            "rebotarm_simulation",
            "models",
            "rebotarm",
            "scene.xml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_backend", default_value="upstream"),
            DeclareLaunchArgument("repo_root", default_value=_default_repo_root()),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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
            OpaqueFunction(function=_validate_simulation_backend),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_upstream_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                prefix=python_executable,
                condition=upstream_condition,
                additional_env={
                    "REBOTARM_REPO_ROOT": repo_root,
                    "REBOTARM_MUJOCO_PYTHON": python_executable,
                },
                parameters=[
                    {
                        "backend": "mujoco",
                        "headless": True,
                        "model_path": upstream_model,
                        "arm_namespace": arm_namespace,
                        "publish_rate_hz": publish_rate_hz,
                    }
                ],
            ),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_adapter",
                name="rebotarm_mujoco_adapter",
                output="screen",
                prefix=python_executable,
                condition=current_condition,
                parameters=[
                    {
                        "use_sim_time": False,
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
                    "use_sim_time": backend_use_sim_time,
                }.items(),
            ),
        ]
    )

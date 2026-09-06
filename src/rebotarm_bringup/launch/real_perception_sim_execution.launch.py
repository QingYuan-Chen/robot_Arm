from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import yaml


def _mujoco_parameters() -> dict:
    config_path = (
        Path(get_package_share_directory("rebotarm_simulation"))
        / "config"
        / "mujoco_sim.yaml"
    )
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    node_payload = payload.get("rebotarm_mujoco_node", {})
    parameters = node_payload.get("ros__parameters", {}) if isinstance(node_payload, dict) else {}
    if not isinstance(parameters, dict):
        raise RuntimeError(f"invalid MuJoCo parameters: {config_path}")
    return dict(parameters)


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    sim_arm_namespace = LaunchConfiguration("sim_arm_namespace")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_joint_positions = LaunchConfiguration("initial_joint_positions")
    mujoco_python_executable = LaunchConfiguration("mujoco_python_executable")
    mujoco_parameters = _mujoco_parameters()
    mujoco_parameters.update(
        {
            "backend": "mujoco",
            "headless": True,
            "arm_namespace": sim_arm_namespace,
            "initial_joint_positions": initial_joint_positions,
        }
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_arm_namespace", default_value="rebotarm_sim"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "initial_joint_positions",
                default_value="[-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            DeclareLaunchArgument(
                "mujoco_python_executable",
                default_value=EnvironmentVariable("REBOTARM_MUJOCO_PYTHON", default_value="python3"),
            ),
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            DeclareLaunchArgument(
                "graspnet_python_executable",
                default_value=EnvironmentVariable("GRASPNET_PYTHON", default_value="python3"),
            ),
            GroupAction(
                [
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    Node(
                        package="rebotarm_simulation",
                        executable="rebotarm_mujoco_node",
                        name="rebotarm_mujoco_node",
                        output="screen",
                        prefix=mujoco_python_executable,
                        parameters=[mujoco_parameters],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [bringup_share, "launch", "visual_grasp_system.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "arm_namespace": sim_arm_namespace,
                            "vision_python_executable": LaunchConfiguration("vision_python_executable"),
                            "graspnet_python_executable": LaunchConfiguration("graspnet_python_executable"),
                            "use_hardware": "false",
                            "start_sim_trajectory_controller": "false",
                            "use_local_rviz": use_local_rviz,
                            "execution_mode": "execute",
                            "start_vision": "true",
                            "vision_profile": "ubuntu_native",
                            "start_visual_ready": "false",
                            "ordinary_depth_quality_enabled": "true",
                            "start_graspnet_baseline": "true",
                            "graspnet_source_mode": "in_process",
                            "graspnet_config": PathJoinSubstitution(
                                [vision_share, "config", "graspnet_ubuntu.yaml"]
                            ),
                            "candidate_ik_input_topic": "/grasp/graspnet_candidates",
                            "start_candidate_ik_filter": "true",
                            "candidate_pose_policy": "preserve_candidate_pose",
                            "candidate_max_candidates_per_frame": "20",
                            "candidate_workspace_gate_enabled": "true",
                            "candidate_max_joint6_delta_rad": "0.0",
                            "candidate_joint_state_topic": [
                                "/",
                                sim_arm_namespace,
                                "/visual_joint_states",
                            ],
                            # The active MuJoCo model already expresses the measured
                            # ee_site=-0.04 m, so do not apply it a second time.
                            "tcp_offset_xyz": "[0.0, 0.0, 0.0]",
                            "executor_input_topic": "/grasp/filtered_plan",
                            "trajectory_precheck_enabled": "true",
                            # Measured local GraspNet + IK latency is about
                            # 1.0-1.4 s on this host. Keep the real-system
                            # default unchanged and widen only this hybrid path.
                            "max_plan_age_sec": "3.0",
                            "open_before_approach": "true",
                            "auto_gripper_width": "true",
                            "auto_gripper_effort": "true",
                            "gripper_grasp_enabled": "false",
                            "grasp_verification_enabled": "false",
                            "grasp_verification_require_contact": "false",
                            "safe_retreat_enabled": "true",
                            "safe_retreat_min_lift_z_m": "0.12",
                            "lift_z_m": "0.04",
                            "moveit_planning_time": "8.0",
                            "moveit_num_planning_attempts": "5",
                            "base_pregrasp_distance_m": "0.06",
                            "safe_home_after_grasp": "false",
                        }.items(),
                    ),
                ]
            ),
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    sim_arm_namespace = LaunchConfiguration("sim_arm_namespace")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    graspnet_candidates_url = LaunchConfiguration("graspnet_candidates_url")

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_arm_namespace", default_value="rebotarm_sim"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("graspnet_candidates_url", default_value="http://192.168.145.1:8081/graspnet_candidates.json"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "visual_grasp_system.launch.py"])
                ),
                launch_arguments={
                    "arm_namespace": sim_arm_namespace,
                    "use_hardware": "false",
                    "use_local_rviz": use_local_rviz,
                    "execution_mode": "execute",
                    "start_vision": "true",
                    "start_visual_ready": "false",
                    "ordinary_depth_quality_enabled": "true",
                    "start_graspnet_baseline": "true",
                    "graspnet_source_mode": "network",
                    "graspnet_candidates_url": graspnet_candidates_url,
                    "graspnet_network_poll_hz": "0.5",
                    "candidate_ik_input_topic": "/grasp/graspnet_candidates",
                    "start_candidate_ik_filter": "true",
                    "candidate_pose_policy": "preserve_candidate_pose",
                    "candidate_max_candidates_per_frame": "20",
                    "candidate_workspace_gate_enabled": "true",
                    "candidate_max_joint6_delta_rad": "0.0",
                    "candidate_joint_state_topic": ["/", sim_arm_namespace, "/visual_joint_states"],
                    "tcp_offset_xyz": "[0.0, 0.0, 0.0]",
                    "executor_input_topic": "/grasp/filtered_plan",
                    "trajectory_precheck_enabled": "true",
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
    )

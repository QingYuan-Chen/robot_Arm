from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")

    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])
    table_safety_params = PathJoinSubstitution([vision_share, "config", "table_safety.yaml"])
    graspnet_policy_params = PathJoinSubstitution([vision_share, "config", "graspnet_policy.yaml"])

    use_local_rviz = LaunchConfiguration("use_local_rviz")
    ordinary_depth_quality_enabled = LaunchConfiguration("ordinary_depth_quality_enabled")
    start_vision = LaunchConfiguration("start_vision")
    start_graspnet_baseline = LaunchConfiguration("start_graspnet_baseline")
    start_candidate_ik_filter = LaunchConfiguration("start_candidate_ik_filter")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")

    graspnet_candidates_topic = LaunchConfiguration("graspnet_candidates_topic")
    graspnet_source_mode = LaunchConfiguration("graspnet_source_mode")
    graspnet_candidates_url = LaunchConfiguration("graspnet_candidates_url")
    graspnet_network_timeout_ms = LaunchConfiguration("graspnet_network_timeout_ms")
    graspnet_network_poll_hz = LaunchConfiguration("graspnet_network_poll_hz")
    graspnet_model_root = LaunchConfiguration("graspnet_model_root")
    graspnet_checkpoint_path = LaunchConfiguration("graspnet_checkpoint_path")
    graspnet_device = LaunchConfiguration("graspnet_device")
    graspnet_backend_module = LaunchConfiguration("graspnet_backend_module")
    graspnet_max_grasps = LaunchConfiguration("graspnet_max_grasps")
    graspnet_max_points = LaunchConfiguration("graspnet_max_points")

    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    filtered_plan_topic = LaunchConfiguration("filtered_plan_topic")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    candidate_joint_state_topic = LaunchConfiguration("candidate_joint_state_topic")
    candidate_filter_service_timeout_sec = LaunchConfiguration("candidate_filter_service_timeout_sec")
    candidate_collision_check_enabled = LaunchConfiguration("candidate_collision_check_enabled")
    candidate_collision_check_service = LaunchConfiguration("candidate_collision_check_service")
    candidate_collision_group_name = LaunchConfiguration("candidate_collision_group_name")
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    candidate_orientation_yaw_offsets_rad = LaunchConfiguration("candidate_orientation_yaw_offsets_rad")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    candidate_max_candidates_per_frame = LaunchConfiguration("candidate_max_candidates_per_frame")
    candidate_min_jaw_width_m = LaunchConfiguration("candidate_min_jaw_width_m")
    candidate_max_jaw_width_m = LaunchConfiguration("candidate_max_jaw_width_m")
    candidate_min_grasp_z_m = LaunchConfiguration("candidate_min_grasp_z_m")
    candidate_safe_lift_min_z_m = LaunchConfiguration("candidate_safe_lift_min_z_m")
    candidate_workspace_gate_enabled = LaunchConfiguration("candidate_workspace_gate_enabled")
    candidate_workspace_min_xyz = LaunchConfiguration("candidate_workspace_min_xyz")
    candidate_workspace_max_xyz = LaunchConfiguration("candidate_workspace_max_xyz")
    candidate_max_grasp_to_object_center_m = LaunchConfiguration("candidate_max_grasp_to_object_center_m")
    candidate_score_joint_distance_weight = LaunchConfiguration("candidate_score_joint_distance_weight")
    candidate_score_joint6_weight = LaunchConfiguration("candidate_score_joint6_weight")
    candidate_max_joint6_delta_rad = LaunchConfiguration("candidate_max_joint6_delta_rad")
    candidate_joint6_symmetry_enabled = LaunchConfiguration("candidate_joint6_symmetry_enabled")
    candidate_joint6_symmetry_angle_rad = LaunchConfiguration("candidate_joint6_symmetry_angle_rad")

    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    base_z_offset_m = LaunchConfiguration("base_z_offset_m")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    lift_z_m = LaunchConfiguration("lift_z_m")
    gripper_open_axis_local_xyz = LaunchConfiguration("gripper_open_axis_local_xyz")
    show_tcp_markers = LaunchConfiguration("show_tcp_markers")
    show_approach_arrow = LaunchConfiguration("show_approach_arrow")
    show_gripper_open_axis = LaunchConfiguration("show_gripper_open_axis")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("start_vision", default_value="true"),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            DeclareLaunchArgument("start_graspnet_baseline", default_value="true"),
            DeclareLaunchArgument("start_candidate_ik_filter", default_value="true"),
            DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),
            DeclareLaunchArgument("graspnet_candidates_topic", default_value="/grasp/graspnet_candidates"),
            DeclareLaunchArgument("graspnet_source_mode", default_value="network"),
            DeclareLaunchArgument("graspnet_candidates_url", default_value="http://192.168.145.1:8081/graspnet_candidates.json"),
            DeclareLaunchArgument("graspnet_network_timeout_ms", default_value="1000"),
            DeclareLaunchArgument("graspnet_network_poll_hz", default_value="0.5"),
            DeclareLaunchArgument("graspnet_model_root", default_value=""),
            DeclareLaunchArgument("graspnet_checkpoint_path", default_value=""),
            DeclareLaunchArgument("graspnet_device", default_value="cuda:0"),
            DeclareLaunchArgument("graspnet_backend_module", default_value="graspnet_baseline_inference"),
            DeclareLaunchArgument("graspnet_max_grasps", default_value="10"),
            DeclareLaunchArgument("graspnet_max_points", default_value="20000"),
            DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates"),
            DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states"),
            DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("candidate_collision_check_enabled", default_value="false"),
            DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity"),
            DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper"),
            DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose"),
            DeclareLaunchArgument("fixed_grasp_orientation_xyzw", default_value="[0.0, 0.0, 0.0, 1.0]"),
            DeclareLaunchArgument("base_approach_axis_xyz", default_value="[1.0, 0.0, 0.0]"),
            DeclareLaunchArgument("base_pregrasp_distance_m", default_value="0.06"),
            DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0]"),
            DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0]"),
            DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20"),
            DeclareLaunchArgument("candidate_min_jaw_width_m", default_value="0.006"),
            DeclareLaunchArgument("candidate_max_jaw_width_m", default_value="0.088"),
            DeclareLaunchArgument("candidate_min_grasp_z_m", default_value="0.0"),
            DeclareLaunchArgument("candidate_safe_lift_min_z_m", default_value="0.120"),
            DeclareLaunchArgument("candidate_workspace_gate_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_workspace_min_xyz", default_value="[0.18, -0.35, 0.0]"),
            DeclareLaunchArgument("candidate_workspace_max_xyz", default_value="[0.64, 0.35, 0.45]"),
            DeclareLaunchArgument("candidate_max_grasp_to_object_center_m", default_value="0.15"),
            DeclareLaunchArgument("candidate_score_joint_distance_weight", default_value="0.15"),
            DeclareLaunchArgument("candidate_score_joint6_weight", default_value="0.35"),
            DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="0.0"),
            DeclareLaunchArgument("candidate_joint6_symmetry_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_joint6_symmetry_angle_rad", default_value="3.141592653589793"),
            DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]"),
            DeclareLaunchArgument("target_base_offset_xyz", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("base_z_offset_m", default_value="0.05"),
            DeclareLaunchArgument("grasp_base_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("lift_z_m", default_value="0.04"),
            DeclareLaunchArgument("gripper_open_axis_local_xyz", default_value="[0.0, 1.0, 0.0]"),
            DeclareLaunchArgument("show_tcp_markers", default_value="true"),
            DeclareLaunchArgument("show_approach_arrow", default_value="true"),
            DeclareLaunchArgument("show_gripper_open_axis", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
                ),
                condition=IfCondition(start_vision),
                launch_arguments={
                    "ordinary_depth_quality_enabled": ordinary_depth_quality_enabled,
                }.items(),
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_graspnet_baseline_node",
                name="rebotarm_graspnet_baseline_node",
                output="screen",
                condition=IfCondition(start_graspnet_baseline),
                parameters=[
                    graspnet_policy_params,
                    {
                        "input_color_topic": "/camera/color/image_raw",
                        "input_depth_topic": "/camera/depth/image_raw",
                        "input_detections_topic": "/grasp/detections",
                        "output_candidates_topic": graspnet_candidates_topic,
                        "output_frame_id": "camera_depth_frame",
                        "source_mode": graspnet_source_mode,
                        "network_candidates_url": graspnet_candidates_url,
                        "network_timeout_ms": graspnet_network_timeout_ms,
                        "network_poll_hz": graspnet_network_poll_hz,
                        "model_root": graspnet_model_root,
                        "checkpoint_path": graspnet_checkpoint_path,
                        "device": graspnet_device,
                        "backend_module": graspnet_backend_module,
                        "max_grasps": graspnet_max_grasps,
                        "max_points": graspnet_max_points,
                    },
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_grasp_candidate_ik_filter",
                name="rebotarm_grasp_candidate_ik_filter",
                output="screen",
                condition=IfCondition(start_candidate_ik_filter),
                parameters=[
                    grasp_pose_policy_params,
                    table_safety_params,
                    {
                        "input_topic": graspnet_candidates_topic,
                        "output_topic": filtered_candidates_topic,
                        "output_plan_topic": filtered_plan_topic,
                        "joint_state_topic": candidate_joint_state_topic,
                        "service_timeout_sec": candidate_filter_service_timeout_sec,
                        "target_frame": "base_link",
                        "moveit_ik_service": "/compute_ik",
                        "collision_check_enabled": candidate_collision_check_enabled,
                        "collision_check_service": candidate_collision_check_service,
                        "collision_group_name": candidate_collision_group_name,
                        "moveit_group_name": "arm",
                        "ee_frame_id": "end_link",
                        "pose_policy": candidate_pose_policy,
                        "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                        "base_approach_axis_xyz": base_approach_axis_xyz,
                        "base_pregrasp_distance_m": base_pregrasp_distance_m,
                        "orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad,
                        "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                        "max_candidates_per_frame": candidate_max_candidates_per_frame,
                        "lift_z_m": lift_z_m,
                        "candidate_min_jaw_width_m": candidate_min_jaw_width_m,
                        "candidate_max_jaw_width_m": candidate_max_jaw_width_m,
                        "candidate_min_grasp_z_m": candidate_min_grasp_z_m,
                        "candidate_safe_lift_min_z_m": candidate_safe_lift_min_z_m,
                        "candidate_workspace_gate_enabled": candidate_workspace_gate_enabled,
                        "candidate_workspace_min_xyz": candidate_workspace_min_xyz,
                        "candidate_workspace_max_xyz": candidate_workspace_max_xyz,
                        "candidate_max_grasp_to_object_center_m": candidate_max_grasp_to_object_center_m,
                        "candidate_score_joint_distance_weight": candidate_score_joint_distance_weight,
                        "candidate_score_joint6_weight": candidate_score_joint6_weight,
                        "candidate_max_joint6_delta_rad": candidate_max_joint6_delta_rad,
                        "candidate_joint6_symmetry_enabled": candidate_joint6_symmetry_enabled,
                        "candidate_joint6_symmetry_angle_rad": candidate_joint6_symmetry_angle_rad,
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "target_base_offset_xyz": target_base_offset_xyz,
                        "pregrasp_base_z_offset_m": base_z_offset_m,
                        "grasp_base_z_offset_m": grasp_base_z_offset_m,
                    },
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_visual_grasp_markers",
                name="rebotarm_visual_grasp_markers",
                output="screen",
                condition=IfCondition(start_visual_grasp_markers),
                parameters=[
                    grasp_pose_policy_params,
                    {
                        "input_topic": executor_input_topic,
                        "output_topic": "/grasp/visual_markers",
                        "target_frame": "base_link",
                        "object_min_diameter_m": 0.06,
                        "object_min_height_m": 0.12,
                        "upright_object_marker": True,
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "gripper_open_axis_local_xyz": gripper_open_axis_local_xyz,
                        "show_tcp_markers": show_tcp_markers,
                        "show_approach_arrow": show_approach_arrow,
                        "show_gripper_open_axis": show_gripper_open_axis,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_visual_grasp_preview",
                output="screen",
                condition=IfCondition(use_local_rviz),
                arguments=["-d", PathJoinSubstitution([bringup_share, "rviz", "visual_grasp.rviz"])],
            ),
        ]
    )


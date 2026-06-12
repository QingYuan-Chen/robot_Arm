from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])
    gripper_policy_params = PathJoinSubstitution([vision_share, "config", "gripper_policy.yaml"])
    retry_policy_params = PathJoinSubstitution([vision_share, "config", "retry_policy.yaml"])
    retreat_policy_params = PathJoinSubstitution([vision_share, "config", "retreat_policy.yaml"])
    visual_servo_params = PathJoinSubstitution([vision_share, "config", "visual_servo.yaml"])
    table_safety_params = PathJoinSubstitution([vision_share, "config", "table_safety.yaml"])
    graspnet_policy_params = PathJoinSubstitution([vision_share, "config", "graspnet_policy.yaml"])
    visual_ready_params = PathJoinSubstitution([vision_share, "config", "visual_ready.yaml"])
    flat_graspnet_params = PathJoinSubstitution([vision_share, "config", "flat_graspnet.yaml"])

    arm_namespace = LaunchConfiguration("arm_namespace")
    channel = LaunchConfiguration("channel")
    use_hardware = LaunchConfiguration("use_hardware")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    execution_mode = LaunchConfiguration("execution_mode")
    start_vision = LaunchConfiguration("start_vision")
    ordinary_depth_quality_enabled = LaunchConfiguration("ordinary_depth_quality_enabled")
    start_graspnet_baseline = LaunchConfiguration("start_graspnet_baseline")
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
    start_grasp_preview = LaunchConfiguration("start_grasp_preview")
    start_candidate_ik_filter = LaunchConfiguration("start_candidate_ik_filter")
    candidate_ik_input_topic = LaunchConfiguration("candidate_ik_input_topic")
    start_visual_ready = LaunchConfiguration("start_visual_ready")
    move_to_visual_ready_on_start = LaunchConfiguration("move_to_visual_ready_on_start")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    visual_ready_duration_sec = LaunchConfiguration("visual_ready_duration_sec")
    visual_ready_wait_timeout_sec = LaunchConfiguration("visual_ready_wait_timeout_sec")
    visual_ready_max_start_delta_rad = LaunchConfiguration("visual_ready_max_start_delta_rad")
    visual_ready_startup_delay_sec = LaunchConfiguration("visual_ready_startup_delay_sec")
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")
    start_motion_execution = LaunchConfiguration("start_motion_execution")
    gripper_open_axis_local_xyz = LaunchConfiguration("gripper_open_axis_local_xyz")
    show_tcp_markers = LaunchConfiguration("show_tcp_markers")
    show_approach_arrow = LaunchConfiguration("show_approach_arrow")
    show_gripper_open_axis = LaunchConfiguration("show_gripper_open_axis")
    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    filtered_plan_topic = LaunchConfiguration("filtered_plan_topic")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    candidate_joint_state_topic = LaunchConfiguration("candidate_joint_state_topic")
    candidate_filter_service_timeout_sec = LaunchConfiguration("candidate_filter_service_timeout_sec")
    candidate_collision_check_enabled = LaunchConfiguration("candidate_collision_check_enabled")
    candidate_collision_check_service = LaunchConfiguration("candidate_collision_check_service")
    candidate_collision_group_name = LaunchConfiguration("candidate_collision_group_name")
    pose_mode = LaunchConfiguration("pose_mode")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    base_z_offset_m = LaunchConfiguration("base_z_offset_m")
    min_target_z_m = LaunchConfiguration("min_target_z_m")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    pose_policy = LaunchConfiguration("pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    candidate_orientation_yaw_offsets_rad = LaunchConfiguration("candidate_orientation_yaw_offsets_rad")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    candidate_max_candidates_per_frame = LaunchConfiguration("candidate_max_candidates_per_frame")
    candidate_min_jaw_width_m = LaunchConfiguration("candidate_min_jaw_width_m")
    candidate_max_jaw_width_m = LaunchConfiguration("candidate_max_jaw_width_m")
    candidate_min_grasp_z_m = LaunchConfiguration("candidate_min_grasp_z_m")
    candidate_pregrasp_min_z_m = LaunchConfiguration("candidate_pregrasp_min_z_m")
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
    lift_z_m = LaunchConfiguration("lift_z_m")
    close_position_m = LaunchConfiguration("close_position_m")
    close_max_effort = LaunchConfiguration("close_max_effort")
    open_before_approach = LaunchConfiguration("open_before_approach")
    auto_gripper_width = LaunchConfiguration("auto_gripper_width")
    auto_gripper_effort = LaunchConfiguration("auto_gripper_effort")
    open_clearance_m = LaunchConfiguration("open_clearance_m")
    close_margin_m = LaunchConfiguration("close_margin_m")
    min_gripper_effort = LaunchConfiguration("min_gripper_effort")
    max_gripper_effort = LaunchConfiguration("max_gripper_effort")
    max_allowed_grasp_width_m = LaunchConfiguration("max_allowed_grasp_width_m")
    gripper_grasp_enabled = LaunchConfiguration("gripper_grasp_enabled")
    gripper_grasp_close_force = LaunchConfiguration("gripper_grasp_close_force")
    gripper_grasp_timeout_sec = LaunchConfiguration("gripper_grasp_timeout_sec")
    gripper_grasp_min_close_time_sec = LaunchConfiguration("gripper_grasp_min_close_time_sec")
    gripper_grasp_velocity_threshold = LaunchConfiguration("gripper_grasp_velocity_threshold")
    gripper_grasp_min_closure_distance_m = LaunchConfiguration("gripper_grasp_min_closure_distance_m")
    safe_retreat_enabled = LaunchConfiguration("safe_retreat_enabled")
    safe_retreat_min_lift_z_m = LaunchConfiguration("safe_retreat_min_lift_z_m")
    safe_retreat_distance_m = LaunchConfiguration("safe_retreat_distance_m")
    safe_retreat_axis_xyz = LaunchConfiguration("safe_retreat_axis_xyz")
    safe_home_after_grasp = LaunchConfiguration("safe_home_after_grasp")
    moveit_planning_time = LaunchConfiguration("moveit_planning_time")
    moveit_num_planning_attempts = LaunchConfiguration("moveit_num_planning_attempts")
    plan_only_stage_pause_sec = LaunchConfiguration("plan_only_stage_pause_sec")
    approach_visual_servo_enabled = LaunchConfiguration("approach_visual_servo_enabled")
    approach_visual_servo_max_iterations = LaunchConfiguration("approach_visual_servo_max_iterations")
    approach_visual_servo_max_step_m = LaunchConfiguration("approach_visual_servo_max_step_m")
    approach_visual_servo_position_tolerance_m = LaunchConfiguration("approach_visual_servo_position_tolerance_m")
    approach_visual_servo_require_fresh_plan = LaunchConfiguration("approach_visual_servo_require_fresh_plan")
    auto_retry_enabled = LaunchConfiguration("auto_retry_enabled")
    auto_retry_max_attempts = LaunchConfiguration("auto_retry_max_attempts")
    safe_retreat_before_retry = LaunchConfiguration("safe_retreat_before_retry")
    grasp_verification_enabled = LaunchConfiguration("grasp_verification_enabled")
    grasp_verification_min_closure_distance_m = LaunchConfiguration("grasp_verification_min_closure_distance_m")
    grasp_verification_require_contact = LaunchConfiguration("grasp_verification_require_contact")
    visual_lift_check_enabled = LaunchConfiguration("visual_lift_check_enabled")
    visual_lift_min_delta_m = LaunchConfiguration("visual_lift_min_delta_m")
    place_after_grasp_enabled = LaunchConfiguration("place_after_grasp_enabled")
    place_position_xyz = LaunchConfiguration("place_position_xyz")
    place_orientation_xyzw = LaunchConfiguration("place_orientation_xyzw")
    place_open_position_m = LaunchConfiguration("place_open_position_m")
    place_open_max_effort = LaunchConfiguration("place_open_max_effort")
    place_retreat_z_m = LaunchConfiguration("place_retreat_z_m")
    trajectory_precheck_enabled = LaunchConfiguration("trajectory_precheck_enabled")

    interactive_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "interactive_system.launch.py"])
        ),
        launch_arguments={
            "arm_namespace": arm_namespace,
            "use_moveit_preview": "true",
            "use_hardware": use_hardware,
            "channel": channel,
            "shutdown_safe_home": shutdown_safe_home,
            "use_local_rviz": use_local_rviz,
            "start_passive_joint_state_publisher": "false",
            "use_moveit_fake_joint_states": "false",
            "rviz_config": PathJoinSubstitution([bringup_share, "rviz", "visual_grasp.rviz"]),
        }.items(),
    )
    visual_ready_startup = Node(
        package="rebotarm_vision",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready_startup",
        output="screen",
        condition=IfCondition(start_visual_ready),
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": move_to_visual_ready_on_start,
                "exit_after_startup_move": True,
                "startup_delay_sec": visual_ready_startup_delay_sec,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )
    post_visual_ready_actions = [
        Node(
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
        ),
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
            executable="rebotarm_send_grasp_preview",
            name="rebotarm_grasp_preview_sender",
            output="screen",
            condition=IfCondition(start_grasp_preview),
            parameters=[
                grasp_pose_policy_params,
                {
                    "input_topic": executor_input_topic,
                    "output_topic": ["/", arm_namespace, "/interactive_control/pose_target"],
                    "pose_mode": pose_mode,
                    "target_frame": "base_link",
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "target_base_offset_xyz": target_base_offset_xyz,
                    "base_z_offset_m": base_z_offset_m,
                    "min_target_z_m": min_target_z_m,
                    "publish_count": 5,
                    "exit_after_publish": False,
                }
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
                }
            ],
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
                }
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
                    "input_topic": candidate_ik_input_topic,
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
                    "candidate_pregrasp_min_z_m": candidate_pregrasp_min_z_m,
                    "grasp_base_z_offset_m": grasp_base_z_offset_m,
                }
            ],
        ),
        Node(
            package="rebotarm_simulation",
            executable="rebotarm_sim_trajectory_controller",
            name="rebotarm_sim_trajectory_controller",
            output="screen",
            condition=UnlessCondition(use_hardware),
            parameters=[
                {
                    "arm_namespace": arm_namespace,
                    "initial_joint_positions": visual_ready_joint_positions,
                }
            ],
        ),
        Node(
            package="rebotarm_motion",
            executable="PoseExecutionNode",
            name="motion_execution",
            output="screen",
            condition=IfCondition(start_motion_execution),
            parameters=[
                {
                    "arm_namespace": arm_namespace,
                    "frame_id": "base_link",
                    "ee_frame_id": "end_link",
                    "moveit_planning_time": moveit_planning_time,
                    "moveit_num_planning_attempts": moveit_num_planning_attempts,
                    "default_velocity_scaling": 0.10,
                    "default_acceleration_scaling": 0.08,
                }
            ],
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_visual_grasp_executor",
            name="rebotarm_visual_grasp_executor",
            output="screen",
            condition=IfCondition(start_visual_grasp_executor),
            parameters=[
                grasp_pose_policy_params,
                gripper_policy_params,
                retry_policy_params,
                retreat_policy_params,
                visual_servo_params,
                table_safety_params,
                {
                    "arm_namespace": arm_namespace,
                    "input_topic": executor_input_topic,
                    "candidates_topic": filtered_candidates_topic,
                    "target_frame": "base_link",
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "target_base_offset_xyz": target_base_offset_xyz,
                    "pregrasp_base_z_offset_m": base_z_offset_m,
                    "grasp_base_z_offset_m": grasp_base_z_offset_m,
                    "pose_policy": pose_policy,
                    "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                    "base_approach_axis_xyz": base_approach_axis_xyz,
                    "base_pregrasp_distance_m": base_pregrasp_distance_m,
                    "min_grasp_z_m": min_target_z_m,
                    "lift_z_m": lift_z_m,
                    "close_position_m": close_position_m,
                    "close_max_effort": close_max_effort,
                    "open_before_approach": open_before_approach,
                    "auto_gripper_width": auto_gripper_width,
                    "auto_gripper_effort": auto_gripper_effort,
                    "open_clearance_m": open_clearance_m,
                    "close_margin_m": close_margin_m,
                    "min_gripper_effort": min_gripper_effort,
                    "max_gripper_effort": max_gripper_effort,
                    "max_allowed_grasp_width_m": max_allowed_grasp_width_m,
                    "gripper_grasp_enabled": gripper_grasp_enabled,
                    "gripper_grasp_close_force": gripper_grasp_close_force,
                    "gripper_grasp_timeout_sec": gripper_grasp_timeout_sec,
                    "gripper_grasp_min_close_time_sec": gripper_grasp_min_close_time_sec,
                    "gripper_grasp_velocity_threshold": gripper_grasp_velocity_threshold,
                    "gripper_grasp_min_closure_distance_m": gripper_grasp_min_closure_distance_m,
                    "safe_retreat_enabled": safe_retreat_enabled,
                    "safe_retreat_min_lift_z_m": safe_retreat_min_lift_z_m,
                    "safe_retreat_distance_m": safe_retreat_distance_m,
                    "safe_retreat_axis_xyz": safe_retreat_axis_xyz,
                    "safe_home_after_grasp": safe_home_after_grasp,
                    "execute_gripper": True,
                    "execution_mode": execution_mode,
                    "plan_only_stage_pause_sec": plan_only_stage_pause_sec,
                    "refresh_plan_at_pregrasp_enabled": False,
                    "refresh_plan_at_pregrasp_required": False,
                    "refresh_plan_timeout_sec": 1.0,
                    "approach_visual_servo_enabled": approach_visual_servo_enabled,
                    "approach_visual_servo_max_iterations": approach_visual_servo_max_iterations,
                    "approach_visual_servo_max_step_m": approach_visual_servo_max_step_m,
                    "approach_visual_servo_position_tolerance_m": approach_visual_servo_position_tolerance_m,
                    "approach_visual_servo_require_fresh_plan": approach_visual_servo_require_fresh_plan,
                    "auto_retry_enabled": auto_retry_enabled,
                    "auto_retry_max_attempts": auto_retry_max_attempts,
                    "safe_retreat_before_retry": safe_retreat_before_retry,
                    "grasp_verification_enabled": grasp_verification_enabled,
                    "grasp_verification_min_closure_distance_m": grasp_verification_min_closure_distance_m,
                    "grasp_verification_require_contact": grasp_verification_require_contact,
                    "visual_lift_check_enabled": visual_lift_check_enabled,
                    "visual_lift_min_delta_m": visual_lift_min_delta_m,
                    "place_after_grasp_enabled": place_after_grasp_enabled,
                    "place_position_xyz": place_position_xyz,
                    "place_orientation_xyzw": place_orientation_xyzw,
                    "place_open_position_m": place_open_position_m,
                    "place_open_max_effort": place_open_max_effort,
                    "place_retreat_z_m": place_retreat_z_m,
                    "trajectory_precheck_enabled": trajectory_precheck_enabled,
                }
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value="auto"),
            DeclareLaunchArgument("use_hardware", default_value="true"),
            DeclareLaunchArgument("shutdown_safe_home", default_value="true"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("execution_mode", default_value="execute"),
            DeclareLaunchArgument("start_vision", default_value="true"),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            DeclareLaunchArgument("start_graspnet_baseline", default_value="true"),
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
            DeclareLaunchArgument("start_grasp_preview", default_value="false"),
            DeclareLaunchArgument("start_candidate_ik_filter", default_value="true"),
            DeclareLaunchArgument("candidate_ik_input_topic", default_value="/grasp/graspnet_candidates"),
            DeclareLaunchArgument("start_visual_ready", default_value="true"),
            DeclareLaunchArgument("move_to_visual_ready_on_start", default_value="true"),
            DeclareLaunchArgument("visual_ready_joint_positions", default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]"),
            DeclareLaunchArgument("visual_ready_duration_sec", default_value="4.0"),
            DeclareLaunchArgument("visual_ready_wait_timeout_sec", default_value="12.0"),
            DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="2.5"),
            DeclareLaunchArgument("visual_ready_startup_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("start_visual_grasp_executor", default_value="true"),
            DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),
            DeclareLaunchArgument("start_motion_execution", default_value="true"),
            DeclareLaunchArgument("gripper_open_axis_local_xyz", default_value="[0.0, 1.0, 0.0]"),
            DeclareLaunchArgument("show_tcp_markers", default_value="true"),
            DeclareLaunchArgument("show_approach_arrow", default_value="true"),
            DeclareLaunchArgument("show_gripper_open_axis", default_value="true"),
            DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates"),
            DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states"),
            DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("candidate_collision_check_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity"),
            DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper"),
            DeclareLaunchArgument("pose_mode", default_value="pregrasp"),
            DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]"),
            DeclareLaunchArgument("target_base_offset_xyz", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("base_z_offset_m", default_value="0.05"),
            DeclareLaunchArgument("min_target_z_m", default_value="0.0"),
            DeclareLaunchArgument("grasp_base_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("pose_policy", default_value="base_axis"),
            DeclareLaunchArgument("fixed_grasp_orientation_xyzw", default_value="[0.0, 0.0, 0.0, 1.0]"),
            DeclareLaunchArgument("base_approach_axis_xyz", default_value="[1.0, 0.0, 0.0]"),
            DeclareLaunchArgument("base_pregrasp_distance_m", default_value="0.06"),
            DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose"),
            DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0]"),
            DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0]"),
            DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20"),
            DeclareLaunchArgument("candidate_min_jaw_width_m", default_value="0.006"),
            DeclareLaunchArgument("candidate_max_jaw_width_m", default_value="0.082"),
            DeclareLaunchArgument("candidate_min_grasp_z_m", default_value="0.0"),
            DeclareLaunchArgument("candidate_pregrasp_min_z_m", default_value="0.120"),
            DeclareLaunchArgument("candidate_safe_lift_min_z_m", default_value="0.120"),
            DeclareLaunchArgument("candidate_workspace_gate_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_workspace_min_xyz", default_value="[0.18, -0.35, 0.0]"),
            DeclareLaunchArgument("candidate_workspace_max_xyz", default_value="[0.64, 0.35, 0.45]"),
            DeclareLaunchArgument("candidate_max_grasp_to_object_center_m", default_value="0.15"),
            DeclareLaunchArgument("candidate_score_joint_distance_weight", default_value="0.15"),
            DeclareLaunchArgument("candidate_score_joint6_weight", default_value="0.35"),
            DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="1.5708"),
            DeclareLaunchArgument("candidate_joint6_symmetry_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_joint6_symmetry_angle_rad", default_value="3.141592653589793"),
            DeclareLaunchArgument("lift_z_m", default_value="0.04"),
            DeclareLaunchArgument("close_position_m", default_value="0.025"),
            DeclareLaunchArgument("close_max_effort", default_value="0.4"),
            DeclareLaunchArgument("open_before_approach", default_value="true"),
            DeclareLaunchArgument("auto_gripper_width", default_value="true"),
            DeclareLaunchArgument("auto_gripper_effort", default_value="true"),
            DeclareLaunchArgument("open_clearance_m", default_value="0.0"),
            DeclareLaunchArgument("close_margin_m", default_value="0.012"),
            DeclareLaunchArgument("min_gripper_effort", default_value="0.22"),
            DeclareLaunchArgument("max_gripper_effort", default_value="0.60"),
            DeclareLaunchArgument("max_allowed_grasp_width_m", default_value="0.082"),
            DeclareLaunchArgument("gripper_grasp_enabled", default_value="true"),
            DeclareLaunchArgument("gripper_grasp_close_force", default_value="0.4"),
            DeclareLaunchArgument("gripper_grasp_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("gripper_grasp_min_close_time_sec", default_value="0.08"),
            DeclareLaunchArgument("gripper_grasp_velocity_threshold", default_value="0.04"),
            DeclareLaunchArgument("gripper_grasp_min_closure_distance_m", default_value="0.006"),
            DeclareLaunchArgument("safe_retreat_enabled", default_value="true"),
            DeclareLaunchArgument("safe_retreat_min_lift_z_m", default_value="0.12"),
            DeclareLaunchArgument("safe_retreat_distance_m", default_value="0.06"),
            DeclareLaunchArgument("safe_retreat_axis_xyz", default_value="[-1.0, 0.0, 0.5]"),
            DeclareLaunchArgument("safe_home_after_grasp", default_value="false"),
            DeclareLaunchArgument("moveit_planning_time", default_value="8.0"),
            DeclareLaunchArgument("moveit_num_planning_attempts", default_value="5"),
            DeclareLaunchArgument("plan_only_stage_pause_sec", default_value="3.0"),
            DeclareLaunchArgument("approach_visual_servo_enabled", default_value="false"),
            DeclareLaunchArgument("approach_visual_servo_max_iterations", default_value="5"),
            DeclareLaunchArgument("approach_visual_servo_max_step_m", default_value="0.02"),
            DeclareLaunchArgument("approach_visual_servo_position_tolerance_m", default_value="0.008"),
            DeclareLaunchArgument("approach_visual_servo_require_fresh_plan", default_value="true"),
            DeclareLaunchArgument("auto_retry_enabled", default_value="false"),
            DeclareLaunchArgument("auto_retry_max_attempts", default_value="3"),
            DeclareLaunchArgument("safe_retreat_before_retry", default_value="true"),
            DeclareLaunchArgument("grasp_verification_enabled", default_value="true"),
            DeclareLaunchArgument("grasp_verification_min_closure_distance_m", default_value="0.006"),
            DeclareLaunchArgument("grasp_verification_require_contact", default_value="true"),
            DeclareLaunchArgument("visual_lift_check_enabled", default_value="false"),
            DeclareLaunchArgument("visual_lift_min_delta_m", default_value="0.03"),
            DeclareLaunchArgument("place_after_grasp_enabled", default_value="false"),
            DeclareLaunchArgument("place_position_xyz", default_value="[0.20, -0.20, 0.25]"),
            DeclareLaunchArgument("place_orientation_xyzw", default_value="[0.0, 0.0, 0.0, 1.0]"),
            DeclareLaunchArgument("place_open_position_m", default_value="0.08"),
            DeclareLaunchArgument("place_open_max_effort", default_value="0.25"),
            DeclareLaunchArgument("place_retreat_z_m", default_value="0.06"),
            DeclareLaunchArgument("trajectory_precheck_enabled", default_value="true"),
            interactive_system,
            visual_ready_startup,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=visual_ready_startup,
                    on_exit=post_visual_ready_actions,
                )
            ),
            GroupAction(
                condition=UnlessCondition(start_visual_ready),
                actions=post_visual_ready_actions,
            ),
        ]
    )


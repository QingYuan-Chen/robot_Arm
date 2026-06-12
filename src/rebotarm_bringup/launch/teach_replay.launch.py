from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    record_path = LaunchConfiguration("record_path")
    dry_run = LaunchConfiguration("dry_run")
    speed = LaunchConfiguration("speed")
    yellow_max_speed = LaunchConfiguration("yellow_max_speed")
    max_replay_velocity_rad_s = LaunchConfiguration("max_replay_velocity_rad_s")
    max_replay_acceleration_rad_s2 = LaunchConfiguration("max_replay_acceleration_rad_s2")
    max_replay_jerk_rad_s3 = LaunchConfiguration("max_replay_jerk_rad_s3")
    large_motion_span_rad = LaunchConfiguration("large_motion_span_rad")
    large_motion_total_rad = LaunchConfiguration("large_motion_total_rad")
    large_motion_max_speed = LaunchConfiguration("large_motion_max_speed")
    start_hold_sec = LaunchConfiguration("start_hold_sec")
    soft_start_duration = LaunchConfiguration("soft_start_duration")
    soft_start_steps = LaunchConfiguration("soft_start_steps")
    first_hold_sec = LaunchConfiguration("first_hold_sec")
    final_hold_sec = LaunchConfiguration("final_hold_sec")
    use_moveit_start_align = LaunchConfiguration("use_moveit_start_align")
    moveit_start_skip_threshold = LaunchConfiguration("moveit_start_skip_threshold")
    moveit_planning_service = LaunchConfiguration("moveit_planning_service")
    moveit_planning_time = LaunchConfiguration("moveit_planning_time")
    collision_check_enabled = LaunchConfiguration("collision_check_enabled")
    collision_check_service = LaunchConfiguration("collision_check_service")
    collision_check_max_samples = LaunchConfiguration("collision_check_max_samples")
    collision_check_timeout_sec = LaunchConfiguration("collision_check_timeout_sec")
    smoothing_enabled = LaunchConfiguration("smoothing_enabled")
    smoothing_window = LaunchConfiguration("smoothing_window")
    filter_enabled = LaunchConfiguration("filter_enabled")
    filter_cutoff_hz = LaunchConfiguration("filter_cutoff_hz")
    filter_sample_rate_hz = LaunchConfiguration("filter_sample_rate_hz")
    resample_enabled = LaunchConfiguration("resample_enabled")
    resample_rate_hz = LaunchConfiguration("resample_rate_hz")
    time_parameterization_method = LaunchConfiguration("time_parameterization_method")
    max_prepared_jump_rad = LaunchConfiguration("max_prepared_jump_rad")
    replay_monitor_enabled = LaunchConfiguration("replay_monitor_enabled")
    replay_monitor_period_sec = LaunchConfiguration("replay_monitor_period_sec")
    replay_monitor_start_grace_sec = LaunchConfiguration("replay_monitor_start_grace_sec")
    replay_monitor_violation_grace_sec = LaunchConfiguration("replay_monitor_violation_grace_sec")
    max_tracking_error_rad = LaunchConfiguration("max_tracking_error_rad")
    max_live_velocity_rad_s = LaunchConfiguration("max_live_velocity_rad_s")
    teleop_config = LaunchConfiguration("teleop_config")
    interactive_share = FindPackageShare("rebotarm_interactive_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument("yellow_max_speed", default_value="0.6"),
            DeclareLaunchArgument("max_replay_velocity_rad_s", default_value="3.0"),
            DeclareLaunchArgument("max_replay_acceleration_rad_s2", default_value="5.0"),
            DeclareLaunchArgument("max_replay_jerk_rad_s3", default_value="20.0"),
            DeclareLaunchArgument("large_motion_span_rad", default_value="0.8"),
            DeclareLaunchArgument("large_motion_total_rad", default_value="2.5"),
            DeclareLaunchArgument("large_motion_max_speed", default_value="1.0"),
            DeclareLaunchArgument("start_hold_sec", default_value="0.8"),
            DeclareLaunchArgument("soft_start_duration", default_value="1.0"),
            DeclareLaunchArgument("soft_start_steps", default_value="30"),
            DeclareLaunchArgument("first_hold_sec", default_value="0.3"),
            DeclareLaunchArgument("final_hold_sec", default_value="1.0"),
            DeclareLaunchArgument("use_moveit_start_align", default_value="true"),
            DeclareLaunchArgument("moveit_start_skip_threshold", default_value="0.005"),
            DeclareLaunchArgument("moveit_planning_service", default_value="/plan_kinematic_path"),
            DeclareLaunchArgument("moveit_planning_time", default_value="3.0"),
            DeclareLaunchArgument("collision_check_enabled", default_value="true"),
            DeclareLaunchArgument("collision_check_service", default_value="/check_state_validity"),
            DeclareLaunchArgument("collision_check_max_samples", default_value="80"),
            DeclareLaunchArgument("collision_check_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("smoothing_enabled", default_value="true"),
            DeclareLaunchArgument("smoothing_window", default_value="7"),
            DeclareLaunchArgument("filter_enabled", default_value="true"),
            DeclareLaunchArgument("filter_cutoff_hz", default_value="5.0"),
            DeclareLaunchArgument("filter_sample_rate_hz", default_value="150.0"),
            DeclareLaunchArgument("resample_enabled", default_value="true"),
            DeclareLaunchArgument("resample_rate_hz", default_value="150.0"),
            DeclareLaunchArgument("time_parameterization_method", default_value="auto"),
            DeclareLaunchArgument("max_prepared_jump_rad", default_value="0.02"),
            DeclareLaunchArgument("replay_monitor_enabled", default_value="true"),
            DeclareLaunchArgument("replay_monitor_period_sec", default_value="0.05"),
            DeclareLaunchArgument("replay_monitor_start_grace_sec", default_value="1.0"),
            DeclareLaunchArgument("replay_monitor_violation_grace_sec", default_value="0.30"),
            DeclareLaunchArgument("max_tracking_error_rad", default_value="0.25"),
            DeclareLaunchArgument("max_live_velocity_rad_s", default_value="3.0"),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=PathJoinSubstitution(
                    [interactive_share, "config", "teleop_control.yaml"]
                ),
            ),
            Node(
                package="rebotarm_teach",
                executable="TeachReplayNode",
                name="teach_replay_node",
                output="screen",
                parameters=[
                    teleop_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "dry_run": dry_run,
                        "speed": speed,
                        "yellow_max_speed": yellow_max_speed,
                        "max_replay_velocity_rad_s": max_replay_velocity_rad_s,
                        "max_replay_acceleration_rad_s2": max_replay_acceleration_rad_s2,
                        "max_replay_jerk_rad_s3": max_replay_jerk_rad_s3,
                        "large_motion_span_rad": large_motion_span_rad,
                        "large_motion_total_rad": large_motion_total_rad,
                        "large_motion_max_speed": large_motion_max_speed,
                        "start_hold_sec": start_hold_sec,
                        "soft_start_duration": soft_start_duration,
                        "soft_start_steps": soft_start_steps,
                        "first_hold_sec": first_hold_sec,
                        "final_hold_sec": final_hold_sec,
                        "use_moveit_start_align": use_moveit_start_align,
                        "moveit_start_skip_threshold": moveit_start_skip_threshold,
                        "moveit_planning_service": moveit_planning_service,
                        "moveit_planning_time": moveit_planning_time,
                        "collision_check_enabled": collision_check_enabled,
                        "collision_check_service": collision_check_service,
                        "collision_check_max_samples": collision_check_max_samples,
                        "collision_check_timeout_sec": collision_check_timeout_sec,
                        "smoothing_enabled": smoothing_enabled,
                        "smoothing_window": smoothing_window,
                        "filter_enabled": filter_enabled,
                        "filter_cutoff_hz": filter_cutoff_hz,
                        "filter_sample_rate_hz": filter_sample_rate_hz,
                        "resample_enabled": resample_enabled,
                        "resample_rate_hz": resample_rate_hz,
                        "time_parameterization_method": time_parameterization_method,
                        "max_prepared_jump_rad": max_prepared_jump_rad,
                        "replay_monitor_enabled": replay_monitor_enabled,
                        "replay_monitor_period_sec": replay_monitor_period_sec,
                        "replay_monitor_start_grace_sec": replay_monitor_start_grace_sec,
                        "replay_monitor_violation_grace_sec": replay_monitor_violation_grace_sec,
                        "max_tracking_error_rad": max_tracking_error_rad,
                        "max_live_velocity_rad_s": max_live_velocity_rad_s,
                    },
                ],
            ),
        ]
    )

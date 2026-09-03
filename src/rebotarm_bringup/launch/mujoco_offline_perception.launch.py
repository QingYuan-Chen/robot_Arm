"""Compose MuJoCo virtual RGB-D with the local plan-only perception chain."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _vision_pythonpath() -> str:
    source = Path(__file__).resolve()
    for parent in source.parents:
        candidates = sorted((parent / ".venv-vision" / "lib").glob("python*/site-packages"))
        if candidates:
            current = os.environ.get("PYTHONPATH", "")
            return os.pathsep.join([str(candidates[0]), current]) if current else str(candidates[0])
    return os.environ.get("PYTHONPATH", "")


def _mujoco_python_executable() -> str:
    source = Path(__file__).resolve()
    for parent in source.parents:
        candidate = parent / "third_party" / "rebotarm_mujoco_venv" / "bin" / "python"
        if candidate.is_file():
            return str(candidate)
    return "python3"


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
    virtual_camera_width = LaunchConfiguration("virtual_camera_width")
    virtual_camera_height = LaunchConfiguration("virtual_camera_height")
    virtual_camera_rate_hz = LaunchConfiguration("virtual_camera_rate_hz")
    virtual_camera_frame_id = LaunchConfiguration("virtual_camera_frame_id")
    virtual_camera_annotation_topic = LaunchConfiguration(
        "virtual_camera_annotation_topic"
    )
    offline_yolo_enabled = LaunchConfiguration("offline_yolo_enabled")
    offline_yolo_model_path = LaunchConfiguration("offline_yolo_model_path")
    offline_yolo_device = LaunchConfiguration("offline_yolo_device")
    offline_yolo_target_classes = LaunchConfiguration("offline_yolo_target_classes")
    offline_yolo_use_world = LaunchConfiguration("offline_yolo_use_world")
    offline_yolo_detection_topic = LaunchConfiguration("offline_yolo_detection_topic")
    offline_yolo_conf_threshold = LaunchConfiguration("offline_yolo_conf_threshold")
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    start_motion_execution = LaunchConfiguration("start_motion_execution")
    execution_mode = LaunchConfiguration("execution_mode")
    execute_gripper = LaunchConfiguration("execute_gripper")
    max_plan_age_sec = LaunchConfiguration("max_plan_age_sec")
    grasp_verification_enabled = LaunchConfiguration("grasp_verification_enabled")
    grasp_verification_require_contact = LaunchConfiguration(
        "grasp_verification_require_contact"
    )
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    mujoco_parameters = _mujoco_parameters()
    mujoco_parameters.update(
        {
            "backend": "mujoco",
            "headless": True,
            "arm_namespace": sim_arm_namespace,
            "initial_joint_positions": initial_joint_positions,
            "virtual_camera.enabled": True,
            "virtual_camera.width": ParameterValue(virtual_camera_width, value_type=int),
            "virtual_camera.height": ParameterValue(virtual_camera_height, value_type=int),
            "virtual_camera.rate_hz": ParameterValue(virtual_camera_rate_hz, value_type=float),
            "virtual_camera.frame_id": virtual_camera_frame_id,
            "virtual_camera.parent_frame_id": "base_link",
            "virtual_camera.annotation_topic": virtual_camera_annotation_topic,
        }
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_arm_namespace", default_value="rebotarm_sim"),
            DeclareLaunchArgument("use_local_rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "initial_joint_positions",
                default_value="[-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            DeclareLaunchArgument(
                "mujoco_python_executable",
                default_value=_mujoco_python_executable(),
            ),
            DeclareLaunchArgument("virtual_camera_width", default_value="640"),
            DeclareLaunchArgument("virtual_camera_height", default_value="480"),
            DeclareLaunchArgument("virtual_camera_rate_hz", default_value="10.0"),
            DeclareLaunchArgument(
                "virtual_camera_frame_id",
                default_value="mujoco_fixed_camera_optical_frame",
            ),
            DeclareLaunchArgument(
                "virtual_camera_annotation_topic",
                default_value="/grasp/ground_truth_detections",
            ),
            DeclareLaunchArgument("offline_yolo_enabled", default_value="true"),
            DeclareLaunchArgument(
                "offline_yolo_model_path",
                default_value=PathJoinSubstitution(
                    [
                        vision_share,
                        "models",
                        "yolo26m-seg-fp16-b1-640-linux.engine",
                    ]
                ),
            ),
            DeclareLaunchArgument("offline_yolo_device", default_value="0"),
            DeclareLaunchArgument("offline_yolo_target_classes", default_value="['bottle']"),
            DeclareLaunchArgument("offline_yolo_use_world", default_value="false"),
            DeclareLaunchArgument(
                "offline_yolo_detection_topic",
                default_value="/grasp/detections",
            ),
            DeclareLaunchArgument("offline_yolo_conf_threshold", default_value="0.05"),
            DeclareLaunchArgument("start_visual_grasp_executor", default_value="false"),
            DeclareLaunchArgument("start_motion_execution", default_value="false"),
            DeclareLaunchArgument("execution_mode", default_value="plan_only"),
            DeclareLaunchArgument("execute_gripper", default_value="false"),
            DeclareLaunchArgument("max_plan_age_sec", default_value="1.0"),
            DeclareLaunchArgument("grasp_verification_enabled", default_value="true"),
            DeclareLaunchArgument(
                "grasp_verification_require_contact",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "candidate_pose_policy",
                default_value="preserve_candidate_pose",
            ),
            DeclareLaunchArgument(
                "fixed_grasp_orientation_xyzw",
                default_value="[0.0, 0.0, 0.0, 1.0]",
            ),
            DeclareLaunchArgument(
                "candidate_grasp_z_offsets_m",
                default_value="[0.0]",
            ),
            GroupAction(
                [
                    SetEnvironmentVariable(name="PYTHONPATH", value=_vision_pythonpath()),
                    SetEnvironmentVariable(name="MUJOCO_GL", value="egl"),
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    Node(
                        package="rebotarm_simulation",
                        executable="rebotarm_mujoco_node",
                        name="rebotarm_mujoco_node",
                        output="screen",
                        prefix=mujoco_python_executable,
                        parameters=[mujoco_parameters],
                    ),
                    Node(
                        package="rebotarm_vision",
                        executable="rebotarm_offline_yolo_node",
                        name="rebotarm_offline_yolo_node",
                        output="screen",
                        condition=IfCondition(offline_yolo_enabled),
                        parameters=[
                            {
                                "offline_yolo.input_topic": "/camera/color/image_raw",
                                "offline_yolo.raw_detection_topic": "/grasp/offline_yolo/raw_detections",
                                "offline_yolo.detection_topic": offline_yolo_detection_topic,
                                "offline_yolo.annotated_topic": "/camera/color/offline_yolo_annotated",
                                "offline_yolo.model_path": offline_yolo_model_path,
                                "offline_yolo.device": offline_yolo_device,
                                "offline_yolo.target_classes": ParameterValue(
                                    offline_yolo_target_classes, value_type=str
                                ),
                                "offline_yolo.use_world": ParameterValue(
                                    PythonExpression(
                                        ["'", offline_yolo_use_world, "'.lower() == 'true'"]
                                    ),
                                    value_type=bool,
                                ),
                                "offline_yolo.conf_threshold": ParameterValue(
                                    offline_yolo_conf_threshold, value_type=float
                                ),
                                "offline_yolo.iou_threshold": 0.45,
                                "offline_yolo.publish_annotated": True,
                            }
                        ],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [bringup_share, "launch", "visual_grasp_system.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "arm_namespace": sim_arm_namespace,
                            "use_hardware": "false",
                            "shutdown_safe_home": "false",
                            "start_sim_trajectory_controller": "false",
                            "use_local_rviz": use_local_rviz,
                            "execution_mode": execution_mode,
                            "start_vision": "false",
                            "start_ordinary_grasp": "false",
                            "start_visual_ready": "false",
                            "start_graspnet_baseline": "true",
                            "graspnet_source_mode": "in_process",
                            "graspnet_config": PathJoinSubstitution(
                                [vision_share, "config", "graspnet_ubuntu.yaml"]
                            ),
                            "graspnet_output_frame_id": virtual_camera_frame_id,
                            "candidate_ik_input_topic": "/grasp/graspnet_candidates",
                            "start_candidate_ik_filter": "true",
                            "candidate_pose_policy": candidate_pose_policy,
                            "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                            "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                            "candidate_joint_state_topic": [
                                "/",
                                sim_arm_namespace,
                                "/visual_joint_states",
                            ],
                            "candidate_workspace_gate_enabled": "true",
                            "candidate_workspace_min_xyz": "[-0.10, -0.15, 0.0]",
                            "candidate_workspace_max_xyz": "[0.60, 0.15, 0.50]",
                            "candidate_pregrasp_min_z_m": "0.05",
                            "candidate_safe_lift_min_z_m": "0.08",
                            "base_approach_axis_xyz": "[0.0, 0.0, -1.0]",
                            "candidate_max_joint6_delta_rad": "0.0",
                            "candidate_collision_check_enabled": "true",
                            "tcp_offset_xyz": "[-0.04, 0.0, 0.0]",
                            "start_grasp_preview": "false",
                            "start_visual_grasp_markers": "false",
                            "start_visual_grasp_executor": start_visual_grasp_executor,
                            "start_motion_execution": start_motion_execution,
                            "execution_mode": execution_mode,
                            "execute_gripper": execute_gripper,
                            "max_plan_age_sec": max_plan_age_sec,
                            "grasp_verification_enabled": grasp_verification_enabled,
                            "grasp_verification_require_contact": grasp_verification_require_contact,
                        }.items(),
                    ),
                ]
            ),
        ]
    )

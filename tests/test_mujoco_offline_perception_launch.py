from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_offline_perception_launch_uses_only_active_mujoco_and_virtual_sensor():
    text = (
        ROOT / "src/rebotarm_bringup/launch/mujoco_offline_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_node"' in text
    assert '"virtual_camera.enabled": True' in text
    assert '"virtual_camera.parent_frame_id": "base_link"' in text
    assert '"virtual_camera.annotation_topic": virtual_camera_annotation_topic' in text
    assert 'DeclareLaunchArgument(' in text and 'virtual_camera_annotation_topic' in text
    assert 'default_value="/grasp/ground_truth_detections"' in text
    assert '"start_vision": "false"' in text
    assert '"graspnet_source_mode": "in_process"' in text
    assert "graspnet_local_infer_url" not in text
    assert '"graspnet_output_frame_id": virtual_camera_frame_id' in text
    assert '"start_sim_trajectory_controller": "false"' in text
    assert 'DeclareLaunchArgument("start_visual_grasp_executor", default_value="false")' in text
    assert 'DeclareLaunchArgument("start_motion_execution", default_value="false")' in text
    assert 'DeclareLaunchArgument("max_plan_age_sec", default_value="1.0")' in text
    assert 'DeclareLaunchArgument("offline_yolo_use_world", default_value="false")' in text
    assert '"yolo26m-seg-fp16-b1-640-linux.engine"' in text
    assert 'DeclareLaunchArgument("offline_yolo_device", default_value="0")' in text
    assert 'DeclareLaunchArgument("offline_yolo_target_classes", default_value="[\'bottle\']")' in text
    assert '"offline_yolo.use_world": ParameterValue(' in text
    assert 'DeclareLaunchArgument(' in text and 'offline_yolo_detection_topic' in text
    assert '"offline_yolo.detection_topic": offline_yolo_detection_topic' in text
    assert 'default_value="/grasp/detections"' in text
    assert '"candidate_pose_policy",' in text
    assert 'default_value="preserve_candidate_pose"' in text
    assert '"fixed_grasp_orientation_xyzw",' in text
    assert '"candidate_grasp_z_offsets_m",' in text
    assert '"candidate_pregrasp_min_z_m": "0.05"' in text
    assert '"base_approach_axis_xyz": "[0.0, 0.0, -1.0]"' in text
    assert '"tcp_offset_xyz": "[-0.04, 0.0, 0.0]"' in text
    assert "rebotarm_sim_trajectory_controller" not in text
    assert "rebotarmcontroller" not in text
    assert "camera_ubuntu.yaml" not in text


def test_visual_grasp_system_allows_nonphysical_camera_output_frame_override():
    text = (
        ROOT / "src/rebotarm_bringup/launch/visual_grasp_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'graspnet_output_frame_id = LaunchConfiguration("graspnet_output_frame_id")' in text
    assert '"output_frame_id": graspnet_output_frame_id' in text
    assert 'DeclareLaunchArgument("graspnet_output_frame_id", default_value="camera_depth_frame")' in text


def test_visual_grasp_system_defaults_ubuntu_native_to_packaged_tensorrt_engine():
    text = (
        ROOT / "src/rebotarm_bringup/launch/visual_grasp_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'vision_yolo_model_path = LaunchConfiguration("vision_yolo_model_path")' in text
    assert '"yolo_model_path": vision_yolo_model_path' in text
    assert '"yolo26m-seg-fp16-b1-640-linux.engine"' in text


def test_candidate_filter_uses_one_complete_launch_parameter_profile():
    text = (
        ROOT / "src/rebotarm_bringup/launch/visual_grasp_system.launch.py"
    ).read_text(encoding="utf-8")
    node_start = text.index('executable="rebotarm_grasp_candidate_ik_filter"')
    node_end = text.index('executable="rebotarm_sim_trajectory_controller"', node_start)
    candidate_node = text[node_start:node_end]

    assert '"pose_policy": candidate_pose_policy' in candidate_node
    assert '"candidate_workspace_gate_enabled": candidate_workspace_gate_enabled' in candidate_node
    assert "grasp_pose_policy_params" not in candidate_node
    assert "table_safety_params" not in candidate_node

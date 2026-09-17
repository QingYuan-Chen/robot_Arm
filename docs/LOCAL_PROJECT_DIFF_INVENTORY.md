# 本地迁移清单

> 历史迁移清单：后续 Windows 和 HTTP 视觉路线的移除记录在 LOCAL_PROJECT_DIFF.md。下方路径是对比证据，不是当前安装清单。

本清单根据新旧工作树中的 Git 跟踪文件生成，并包含旧工作树未提交的修改。通过 Git 跟踪文件清单排除了 build/install/log/venv/cache 目录。仅路径发生的新增或删除可能只是资源移动，不代表功能新增或移除。

旧版跟踪文件：539；新版基线跟踪文件：604。仅新版存在：260；仅旧版存在：195；已变更：123；完全相同：221。

## 新版包、可执行文件、启动文件与依赖

### rebotarm_bringup

**Console entries**

- None (resource/interface package).

**launch**

- `bringup.launch.py`
- `driver_only.launch.py`
- `interactive_system.launch.py`
- `moveit_hardware.launch.py`
- `mujoco_offline_perception.launch.py`
- `real_perception_sim_execution.launch.py`
- `rebotarm_app.launch.py`
- `rviz_ee_drag_real.launch.py`
- `rviz_ee_drag_sim.launch.py`
- `teach_record.launch.py`
- `teach_replay.launch.py`
- `teleop_keyboard.launch.py`
- `teleop_system.launch.py`
- `visual_grasp_perception_preview.launch.py`
- `visual_grasp_system.launch.py`
- `visual_ready_hold.launch.py`

**config**

- `config/arm.yaml`
- `config/driver_params.yaml`
- `config/gripper.yaml`
- `config/interactive_control.yaml`
- `config/replay_profiles.yaml`
- `config/teleop_control.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `joint_state_publisher`
- `launch`
- `launch_ros`
- `moveit_configs_utils`
- `python3-yaml`
- `rebotarm_calibration`
- `rebotarm_dashboard`
- `rebotarm_motion`
- `rebotarm_moveit_config`
- `rebotarm_simulation`
- `rebotarm_teach`
- `rebotarm_teleop`
- `rebotarm_vision`
- `rebotarmcontroller`
- `robot_state_publisher`
- `rviz2`

### rebotarm_calibration

**Console entries**

- `rebotarm_handeye_residual = rebotarm_calibration.handeye_residual_cli:main`
- `rebotarm_tcp_calibration = rebotarm_calibration.tcp_calibration_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `cv_bridge`
- `python3-numpy`
- `python3-opencv`
- `rclpy`
- `sensor_msgs`
- `tf2_ros`

### rebotarm_dashboard

**Console entries**

- `TeleopStatusPanelNode = rebotarm_dashboard.teleop_status_panel_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_index_python`
- `ament_python`
- `control_msgs`
- `moveit_msgs`
- `rclpy`
- `rebotarm_motion`
- `rebotarm_moveit_config`
- `rebotarm_msgs`
- `rebotarm_teach`
- `rebotarm_teleop`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_interactive_control

**Console entries**

- `TeleopKeyboardNode = rebotarm_interactive_control.teleop_keyboard_node:main`
- `TeachRecorderNode = rebotarm_interactive_control.teach_recorder_node:main`
- `TeachReplayNode = rebotarm_interactive_control.teach_replay_node:main`
- `TeleopStatusPanelNode = rebotarm_interactive_control.teleop_status_panel_node:main`
- `GripperVisualJointStateNode = rebotarm_interactive_control.gripper_visual_joint_state_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_index_python`
- `ament_python`
- `control_msgs`
- `geometry_msgs`
- `moveit_msgs`
- `rclpy`
- `rebotarm_dashboard`
- `rebotarm_motion`
- `rebotarm_msgs`
- `rebotarm_teach`
- `rebotarm_teleop`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_motion

**Console entries**

- `PoseExecutionNode = rebotarm_motion.pose_execution_node:main`
- `rebotarm_visual_ready = rebotarm_motion.visual_ready_node:main`

**launch**

- None.

**config**

- `config/visual_ready.yaml`

**dependencies**

- `ament_python`
- `builtin_interfaces`
- `control_msgs`
- `geometry_msgs`
- `moveit_msgs`
- `pinocchio`
- `python3-yaml`
- `rclpy`
- `rebotarm_msgs`
- `ruckig`
- `sensor_msgs`
- `shape_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_moveit_config

**Console entries**

- None (resource/interface package).

**launch**

- `demo.launch.py`

**config**

- `config/joint_limits.yaml`
- `config/kinematics.yaml`
- `config/moveit_controllers.yaml`
- `config/moveit_cpp.yaml`
- `config/ompl_planning.yaml`
- `config/pilz_cartesian_limits.yaml`
- `config/reBot-DevArm_fixend.srdf`
- `config/rebotarm.srdf`
- `config/rebotarm.urdf`
- `config/sensors_3d.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `joint_state_publisher`
- `launch`
- `launch_ros`
- `moveit_configs_utils`
- `moveit_kinematics`
- `moveit_ros_move_group`
- `moveit_ros_planning`
- `moveit_ros_planning_interface`
- `moveit_ros_visualization`
- `python3-yaml`
- `rebotarm_teleop`
- `robot_state_publisher`
- `rviz2`
- `tf2_ros`

### rebotarm_msgs

**Console entries**

- None (resource/interface package).

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_cmake`
- `builtin_interfaces`
- `geometry_msgs`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `std_msgs`
- `trajectory_msgs`

### rebotarm_simulation

**Console entries**

- `rebotarm_sim_trajectory_controller = rebotarm_simulation.sim_trajectory_controller_node:main`
- `rebotarm_mujoco_health = rebotarm_simulation.mujoco_health:main`
- `rebotarm_mujoco_cli = rebotarm_simulation.mujoco_cli:main`
- `rebotarm_mujoco = rebotarm_simulation.mujoco_cli:main`
- `rebotarm_mujoco_viewer = rebotarm_simulation.mujoco_viewer:main`
- `rebotarm_mujoco_node = rebotarm_simulation.mujoco_ros_node:main`
- `rebotarm_urdf_to_mjcf = rebotarm_simulation.urdf_to_mjcf:main`

**launch**

- `mujoco_moveit_sim.launch.py`
- `mujoco_sim.launch.py`

**config**

- `config/motor_control_calibration.yaml`
- `config/mujoco_collision_baseline.json`
- `config/mujoco_sim.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `builtin_interfaces`
- `control_msgs`
- `geometry_msgs`
- `launch`
- `launch_ros`
- `rclpy`
- `rebotarm_moveit_config`
- `rebotarm_msgs`
- `rosgraph_msgs`
- `sensor_msgs`
- `std_srvs`
- `tf2_ros`
- `trajectory_msgs`

### rebotarm_teach

**Console entries**

- `TeachRecorderNode = rebotarm_teach.teach_recorder_node:main`
- `TeachReplayNode = rebotarm_teach.teach_replay_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `control_msgs`
- `moveit_msgs`
- `rclpy`
- `rebotarm_motion`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_teleop

**Console entries**

- `TeleopKeyboardNode = rebotarm_teleop.teleop_keyboard_node:main`
- `GripperVisualJointStateNode = rebotarm_teleop.gripper_visual_joint_state_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `control_msgs`
- `geometry_msgs`
- `rclpy`
- `rebotarm_motion`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_vision

**Console entries**

- `rebotarm_vision_node = rebotarm_vision.vision_node:main`
- `rebotarm_ordinary_grasp_node = rebotarm_vision.ordinary_grasp_node:main`
- `rebotarm_graspnet_baseline_node = rebotarm_vision.graspnet_baseline_node:main`
- `rebotarm_send_grasp_preview = rebotarm_vision.grasp_preview_sender_node:main`
- `rebotarm_visual_grasp_markers = rebotarm_vision.visual_grasp_marker_node:main`
- `rebotarm_visual_grasp_executor = rebotarm_vision.visual_grasp_executor_node:main`
- `rebotarm_grasp_candidate_ik_filter = rebotarm_vision.candidate_ik_filter_node:main`
- `rebotarm_grasp_tcp_frame = rebotarm_vision.grasp_tcp_frame_node:main`
- `rebotarm_visual_ready = rebotarm_vision.visual_ready_node:main`
- `rebotarm_visual_grasp_benchmark = rebotarm_vision.visual_grasp_benchmark:main`
- `rebotarm_hybrid_grasp_sim_benchmark = rebotarm_vision.hybrid_grasp_sim_benchmark:main`
- `rebotarm_tcp_calibration = rebotarm_vision.tcp_calibration_node:main`
- `rebotarm_debug_camera_preview = rebotarm_vision.debug_camera_preview:main`
- `rebotarm_grasp_depth_probe = rebotarm_vision.grasp_depth_probe_node:main`
- `rebotarm_offline_yolo_node = rebotarm_vision.offline_yolo_node:main`

**launch**

- `vision.launch.py`
- `vision_ubuntu.launch.py`

**config**

- `config/camera.yaml`
- `config/camera_ubuntu.yaml`
- `config/flat_graspnet.yaml`
- `config/grasp_pose_policy.yaml`
- `config/graspnet_policy.yaml`
- `config/graspnet_ubuntu.yaml`
- `config/gripper_policy.yaml`
- `config/handeye.yaml`
- `config/retreat_policy.yaml`
- `config/retry_policy.yaml`
- `config/table_safety.yaml`
- `config/visual_servo.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `geometry_msgs`
- `launch`
- `launch_ros`
- `moveit_msgs`
- `rclpy`
- `rebotarm_calibration`
- `rebotarm_motion`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `tf2_ros`
- `visualization_msgs`

### rebotarm_voice_control

**Console entries**

- `rebotarm_text_input = rebotarm_voice_control.text_input_node:main`
- `rebotarm_llm_tool = rebotarm_voice_control.llm_tool_node:main`
- `rebotarm_tool_call = rebotarm_voice_control.tool_call_node:main`
- `rebotarm_realtime_event = rebotarm_voice_control.realtime_event_node:main`
- `rebotarm_realtime_gateway = rebotarm_voice_control.realtime_voice_gateway_node:main`
- `rebotarm_sim_move_relative_action = rebotarm_voice_control.sim_move_relative_action_node:main`
- `rebotarm_sim_executor = rebotarm_voice_control.sim_executor:main`
- `rebotarm_voice_file = rebotarm_voice_control.voice_file_node:main`
- `rebotarm_voice_control_node = rebotarm_voice_control.voice_control_node:main`

**launch**

- `voice_control.launch.py`
- `voice_real.launch.py`
- `voice_realtime.launch.py`
- `voice_sim.launch.py`

**config**

- `config/intents.yaml`
- `config/llm_config.yaml`
- `config/named_poses.yaml`
- `config/safety_limits.yaml`
- `config/sim_config.yaml`
- `config/task_templates.yaml`

**dependencies**

- `ament_copyright`
- `ament_flake8`
- `ament_pep257`
- `ament_python`
- `control_msgs`
- `geometry_msgs`
- `launch`
- `launch_ros`
- `python3-pytest`
- `python3-yaml`
- `rclpy`
- `rebotarm_motion`
- `rebotarm_msgs`
- `std_msgs`
- `std_srvs`

### rebotarmcontroller

**Console entries**

- `reBotArmController = rebotarmcontroller.rebotarm_controller:main`
- `GravityCompensation = rebotarmcontroller.examples.gravity_compensation:main`
- `GripperControl = rebotarmcontroller.examples.gripper_control:main`
- `MoveTo = rebotarmcontroller.examples.move_to:main`
- `MoveToPose = rebotarmcontroller.examples.move_to_pose:main`
- `p0_gate_bc_acceptance = rebotarmcontroller.examples.p0_gate_bc_acceptance:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `builtin_interfaces`
- `control_msgs`
- `geometry_msgs`
- `pinocchio`
- `rclpy`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `tf_transformations`
- `trajectory_msgs`

## 旧版包、可执行文件、启动文件与依赖

### rebotarm_bringup

**Console entries**

- None (resource/interface package).

**launch**

- `bringup.launch.py`
- `core.launch.py`
- `driver_only.launch.py`
- `interactive_system.launch.py`
- `moveit_hardware.launch.py`
- `real_perception_sim_execution.launch.py`
- `rebotarm_app.launch.py`
- `rviz.launch.py`
- `rviz_ee_drag_real.launch.py`
- `rviz_ee_drag_sim.launch.py`
- `teach_record.launch.py`
- `teach_replay.launch.py`
- `teleop_keyboard.launch.py`
- `teleop_system.launch.py`
- `visual_grasp_perception_preview.launch.py`
- `visual_grasp_system.launch.py`
- `visual_ready_hold.launch.py`

**config**

- `config/controller_runtime.yaml`
- `config/controller_safety.yaml`
- `config/interactive_control.yaml`
- `config/rviz_real.yaml`
- `config/teleop_control.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `joint_state_publisher`
- `launch`
- `launch_ros`
- `moveit_configs_utils`
- `python3-yaml`
- `rebotarm_dashboard`
- `rebotarm_description`
- `rebotarm_motion`
- `rebotarm_moveit_config`
- `rebotarm_simulation`
- `rebotarm_teach`
- `rebotarm_teleop`
- `rebotarm_vision`
- `rebotarmcontroller`
- `robot_state_publisher`
- `rviz2`

### rebotarm_calibration

**Console entries**

- None (resource/interface package).

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `python3-yaml`

### rebotarm_dashboard

**Console entries**

- `TeleopStatusPanelNode = rebotarm_dashboard.teleop_status_panel_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_index_python`
- `ament_python`
- `control_msgs`
- `moveit_msgs`
- `python3-yaml`
- `rclpy`
- `rebotarm_description`
- `rebotarm_motion`
- `rebotarm_moveit_config`
- `rebotarm_msgs`
- `rebotarm_teach`
- `rebotarm_teleop`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_description

**Console entries**

- None (resource/interface package).

**launch**

- None.

**config**

- `config/arm.yaml`
- `config/gripper.yaml`

**dependencies**

- `ament_python`

### rebotarm_interactive_control

**Console entries**

- `TeleopKeyboardNode = rebotarm_interactive_control.teleop_keyboard_node:main`
- `TeachRecorderNode = rebotarm_interactive_control.teach_recorder_node:main`
- `TeachReplayNode = rebotarm_interactive_control.teach_replay_node:main`
- `TeleopStatusPanelNode = rebotarm_interactive_control.teleop_status_panel_node:main`
- `GripperVisualJointStateNode = rebotarm_interactive_control.gripper_visual_joint_state_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_index_python`
- `ament_python`
- `control_msgs`
- `geometry_msgs`
- `moveit_msgs`
- `rclpy`
- `rebotarm_dashboard`
- `rebotarm_motion`
- `rebotarm_msgs`
- `rebotarm_teach`
- `rebotarm_teleop`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_motion

**Console entries**

- `PoseExecutionNode = rebotarm_motion.pose_execution_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `control_msgs`
- `geometry_msgs`
- `moveit_msgs`
- `rclpy`
- `rebotarm_msgs`
- `sensor_msgs`
- `shape_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_moveit_config

**Console entries**

- None (resource/interface package).

**launch**

- `demo.launch.py`

**config**

- `config/joint_limits.yaml`
- `config/kinematics.yaml`
- `config/moveit_controllers.yaml`
- `config/moveit_cpp.yaml`
- `config/ompl_planning.yaml`
- `config/pilz_cartesian_limits.yaml`
- `config/reBot-DevArm_fixend.srdf`
- `config/rebotarm.srdf`
- `config/sensors_3d.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `joint_state_publisher`
- `launch`
- `launch_ros`
- `moveit_configs_utils`
- `moveit_kinematics`
- `moveit_planners_ompl`
- `moveit_ros_move_group`
- `moveit_ros_planning`
- `moveit_ros_planning_interface`
- `moveit_ros_visualization`
- `moveit_simple_controller_manager`
- `python3-yaml`
- `rebotarm_description`
- `rebotarm_teleop`
- `robot_state_publisher`
- `rviz2`
- `tf2_ros`

### rebotarm_msgs

**Console entries**

- None (resource/interface package).

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_cmake`
- `builtin_interfaces`
- `geometry_msgs`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `std_msgs`
- `trajectory_msgs`

### rebotarm_simulation

**Console entries**

- `rebotarm_rviz_fake_controller = rebotarm_simulation.sim_trajectory_controller_node:main`
- `rebotarm_sim_trajectory_controller = rebotarm_simulation.sim_trajectory_controller_node:main`
- `rebotarm_mujoco_health = rebotarm_simulation.mujoco_health:main`
- `rebotarm_mujoco_acceptance = rebotarm_simulation.mujoco_acceptance:main`
- `rebotarm_mujoco_batch = rebotarm_simulation.mujoco_batch:main`
- `rebotarm_mujoco_contact_check = rebotarm_simulation.mujoco_contact_check:main`
- `rebotarm_mujoco_ros_acceptance = rebotarm_simulation.mujoco_ros_acceptance:main`
- `rebotarm_real2sim_acceptance = rebotarm_simulation.real2sim_acceptance:main`
- `rebotarm_real2sim_bridge = rebotarm_simulation.real2sim_ros_node:main`
- `rebotarm_real2sim_viewer = rebotarm_simulation.real2sim_viewer:main`
- `rebotarm_mujoco_moveit_acceptance = rebotarm_simulation.mujoco_moveit_acceptance:main`
- `rebotarm_mujoco_pick_batch = rebotarm_simulation.mujoco_pick_batch:main`
- `rebotarm_mujoco_cli = rebotarm_simulation.mujoco_cli:main`
- `rebotarm_mujoco_viewer = rebotarm_simulation.mujoco_viewer:main`
- `rebotarm_mujoco_node = rebotarm_simulation.mujoco_ros_node:main`
- `rebotarm_sim2real = rebotarm_simulation.sim2real_cli:main`
- `rebotarm_urdf_to_mjcf = rebotarm_simulation.urdf_to_mjcf:main`

**launch**

- `mujoco_sim.launch.py`
- `real2sim_bridge.launch.py`

**config**

- `config/motor_control_calibration.yaml`
- `config/mujoco_collision.yaml`
- `config/mujoco_collision_baseline.json`
- `config/mujoco_sim.yaml`
- `config/real2sim_bridge.yaml`
- `config/real2sim_mapping.yaml`
- `config/sim2real_randomization.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `builtin_interfaces`
- `control_msgs`
- `diagnostic_msgs`
- `launch`
- `launch_ros`
- `moveit_msgs`
- `python3-numpy`
- `python3-yaml`
- `rclpy`
- `rebotarm_description`
- `rebotarm_msgs`
- `rosgraph_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_teach

**Console entries**

- `TeachRecorderNode = rebotarm_teach.teach_recorder_node:main`
- `TeachReplayNode = rebotarm_teach.teach_replay_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `control_msgs`
- `moveit_msgs`
- `rclpy`
- `rebotarm_motion`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_teleop

**Console entries**

- `TeleopKeyboardNode = rebotarm_teleop.teleop_keyboard_node:main`
- `GripperVisualJointStateNode = rebotarm_teleop.gripper_visual_joint_state_node:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `control_msgs`
- `geometry_msgs`
- `rclpy`
- `rebotarm_motion`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `trajectory_msgs`

### rebotarm_vision

**Console entries**

- `rebotarm_vision_node = rebotarm_vision.vision_node:main`
- `rebotarm_yolo_node = rebotarm_vision.yolo_node:main`
- `rebotarm_graspnet_baseline_node = rebotarm_vision.graspnet_baseline_node:main`
- `rebotarm_graspnet_viewer = rebotarm_vision.graspnet_viewer_node:main`
- `rebotarm_send_grasp_preview = rebotarm_vision.grasp_preview_sender_node:main`
- `rebotarm_visual_grasp_markers = rebotarm_vision.visual_grasp_marker_node:main`
- `rebotarm_visual_grasp_executor = rebotarm_vision.visual_grasp_executor_node:main`
- `rebotarm_grasp_candidate_ik_filter = rebotarm_vision.candidate_ik_filter_node:main`
- `rebotarm_grasp_tcp_frame = rebotarm_vision.grasp_tcp_frame_node:main`
- `rebotarm_visual_ready = rebotarm_vision.visual_ready_node:main`
- `rebotarm_visual_grasp_benchmark = rebotarm_vision.visual_grasp_benchmark:main`
- `rebotarm_hybrid_grasp_sim_benchmark = rebotarm_vision.hybrid_grasp_sim_benchmark:main`
- `rebotarm_tcp_calibration = rebotarm_vision.tcp_calibration_node:main`
- `rebotarm_debug_camera_preview = rebotarm_vision.debug_camera_preview:main`
- `rebotarm_grasp_depth_probe = rebotarm_vision.grasp_depth_probe_node:main`

**launch**

- `graspnet_viewer.launch.py`
- `vision.launch.py`

**config**

- `config/camera.yaml`
- `config/flat_graspnet.yaml`
- `config/grasp_pose_policy.yaml`
- `config/graspnet_policy.yaml`
- `config/graspnet_viewer.yaml`
- `config/gripper_policy.yaml`
- `config/handeye.yaml`
- `config/retreat_policy.yaml`
- `config/retry_policy.yaml`
- `config/table_safety.yaml`
- `config/visual_ready.yaml`
- `config/visual_servo.yaml`

**dependencies**

- `ament_index_python`
- `ament_python`
- `builtin_interfaces`
- `control_msgs`
- `geometry_msgs`
- `launch`
- `launch_ros`
- `message_filters`
- `moveit_msgs`
- `python3-numpy`
- `python3-opencv`
- `python3-yaml`
- `rclpy`
- `rebotarm_calibration`
- `rebotarm_motion`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `tf2_ros`
- `trajectory_msgs`
- `visualization_msgs`

### rebotarmcontroller

**Console entries**

- `reBotArmController = rebotarmcontroller.rebotarm_controller:main`
- `GravityCompensation = rebotarmcontroller.examples.gravity_compensation:main`
- `GripperControl = rebotarmcontroller.examples.gripper_control:main`
- `MoveTo = rebotarmcontroller.examples.move_to:main`
- `MoveToPose = rebotarmcontroller.examples.move_to_pose:main`

**launch**

- None.

**config**

- None.

**dependencies**

- `ament_python`
- `builtin_interfaces`
- `control_msgs`
- `geometry_msgs`
- `python3-numpy`
- `python3-yaml`
- `rclpy`
- `rebotarm_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `tf_transformations`
- `trajectory_msgs`

## 仅新版存在的路径

- `AGENTS.md`
- `Agent/ACTIVITY_LOG.md`
- `Agent/EXECUTION_FLOW.md`
- `Agent/MEMORY.md`
- `Agent/PROJECT_STATUS.md`
- `Agent/README.md`
- `Agent/STATE.json`
- `Agent/update_state.py`
- `CONTEXT.md`
- `LICENSE`
- `README.md`
- `README_zh.md`
- `THIRD_PARTY_NOTICES.md`
- `docs/LOCAL_PROJECT_DIFF.md`
- `docs/LOCAL_PROJECT_DIFF_INVENTORY.md`
- `docs/README_zh.md`
- `docs/architecture.md`
- `docs/coupling_migration.md`
- `docs/launch_python_configuration.md`
- `docs/local_setup_zh.md`
- `docs/mujoco_sim.md`
- `docs/mujoco_sim_to_real_params.md`
- `docs/mujoco_upstream_sources.md`
- `docs/node_topology.mermaid`
- `docs/rebotarm_common_commands.md`
- `docs/ubuntu_vision_setup_zh.md`
- `docs/visual_grasp_commands.md`
- `docs/visual_grasp_seven_layer_params.md`
- `docs/voice_control_stage4_zh.md`
- `requirements-graspnet.txt`
- `requirements-mujoco.txt`
- `requirements-tensorrt.txt`
- `requirements-vision.txt`
- `src/rebotarm_bringup/config/arm.yaml`
- `src/rebotarm_bringup/config/driver_params.yaml`
- `src/rebotarm_bringup/config/gripper.yaml`
- `src/rebotarm_bringup/config/replay_profiles.yaml`
- `src/rebotarm_bringup/launch/mujoco_offline_perception.launch.py`
- `src/rebotarm_calibration/rebotarm_calibration/aruco_pose.py`
- `src/rebotarm_calibration/rebotarm_calibration/handeye_residual.py`
- `src/rebotarm_calibration/rebotarm_calibration/handeye_residual_cli.py`
- `src/rebotarm_calibration/rebotarm_calibration/handeye_solver.py`
- `src/rebotarm_calibration/rebotarm_calibration/tcp_calibration_node.py`
- `src/rebotarm_interactive_control/rebotarm_interactive_control/pose_preview_solver.py`
- `src/rebotarm_motion/config/visual_ready.yaml`
- `src/rebotarm_motion/rebotarm_motion/pose_preview_solver.py`
- `src/rebotarm_motion/rebotarm_motion/trajectory_runtime_limits.py`
- `src/rebotarm_motion/rebotarm_motion/visual_ready_node.py`
- `src/rebotarm_moveit_config/config/rebotarm.urdf`
- `src/rebotarm_moveit_config/meshes/base_link.STL`
- `src/rebotarm_moveit_config/meshes/end_link.STL`
- `src/rebotarm_moveit_config/meshes/gripper_base.stl`
- `src/rebotarm_moveit_config/meshes/gripper_hardware.stl`
- `src/rebotarm_moveit_config/meshes/left_finger.stl`
- `src/rebotarm_moveit_config/meshes/link1.STL`
- `src/rebotarm_moveit_config/meshes/link2.STL`
- `src/rebotarm_moveit_config/meshes/link3.STL`
- `src/rebotarm_moveit_config/meshes/link4.STL`
- `src/rebotarm_moveit_config/meshes/link5.STL`
- `src/rebotarm_moveit_config/meshes/link6.STL`
- `src/rebotarm_moveit_config/meshes/right_finger.stl`
- `src/rebotarm_simulation/launch/mujoco_moveit_sim.launch.py`
- `src/rebotarm_simulation/rebotarm_simulation/assets/rebotarm_base.xml`
- `src/rebotarm_simulation/rebotarm_simulation/assets/rebotarm_grasp_scene.xml`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_adapter_core.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_grasp_quality.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_legacy_cli.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_legacy_health.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_limit_checks.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_metrics.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_model_profile.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_runner.py`
- `src/rebotarm_simulation/rebotarm_simulation/paired_trajectory_analysis.py`
- `src/rebotarm_simulation/rebotarm_simulation/resource_paths.py`
- `src/rebotarm_simulation/rebotarm_simulation/virtual_camera.py`
- `src/rebotarm_teach/rebotarm_teach/recording_feedback.py`
- `src/rebotarm_vision/config/camera_ubuntu.yaml`
- `src/rebotarm_vision/config/graspnet_ubuntu.yaml`
- `src/rebotarm_vision/launch/vision_ubuntu.launch.py`
- `src/rebotarm_vision/rebotarm_vision/camera/network_mjpeg_driver.py`
- `src/rebotarm_vision/rebotarm_vision/candidate_precheck_policy.py`
- `src/rebotarm_vision/rebotarm_vision/converters/network_detection_msgs.py`
- `src/rebotarm_vision/rebotarm_vision/converters/ordinary_grasp_adapter.py`
- `src/rebotarm_vision/rebotarm_vision/detector/network_detection_client.py`
- `src/rebotarm_vision/rebotarm_vision/grasp_verification_policy.py`
- `src/rebotarm_vision/rebotarm_vision/graspnet_service_contract.py`
- `src/rebotarm_vision/rebotarm_vision/latest_only_work_queue.py`
- `src/rebotarm_vision/rebotarm_vision/local_graspnet_client.py`
- `src/rebotarm_vision/rebotarm_vision/message_freshness.py`
- `src/rebotarm_vision/rebotarm_vision/network_graspnet_client.py`
- `src/rebotarm_vision/rebotarm_vision/offline_yolo.py`
- `src/rebotarm_vision/rebotarm_vision/offline_yolo_node.py`
- `src/rebotarm_vision/rebotarm_vision/ordinary_grasp_node.py`
- `src/rebotarm_vision/rebotarm_vision/timestamp_policy.py`
- `src/rebotarm_voice_control/config/intents.yaml`
- `src/rebotarm_voice_control/config/llm_config.yaml`
- `src/rebotarm_voice_control/config/named_poses.yaml`
- `src/rebotarm_voice_control/config/safety_limits.yaml`
- `src/rebotarm_voice_control/config/sim_config.yaml`
- `src/rebotarm_voice_control/config/task_templates.yaml`
- `src/rebotarm_voice_control/launch/voice_control.launch.py`
- `src/rebotarm_voice_control/launch/voice_real.launch.py`
- `src/rebotarm_voice_control/launch/voice_realtime.launch.py`
- `src/rebotarm_voice_control/launch/voice_sim.launch.py`
- `src/rebotarm_voice_control/package.xml`
- `src/rebotarm_voice_control/rebotarm_voice_control/__init__.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/asr_client.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/command_router.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/config_loader.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/execution_modes.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/intent_parser.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/llm_providers.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/llm_tool_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/models.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/realtime_bridge.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/realtime_event_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/realtime_session_client.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/realtime_voice_gateway_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/ros2_action_transport.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/safety_guard.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/sim_action_bindings.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/sim_executor.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/sim_motion_adapter.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/sim_move_relative_action_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/task_planner.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/text_input_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/tool_call_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/tool_call_schema.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/voice_control_node.py`
- `src/rebotarm_voice_control/rebotarm_voice_control/voice_file_node.py`
- `src/rebotarm_voice_control/resource/rebotarm_voice_control`
- `src/rebotarm_voice_control/setup.cfg`
- `src/rebotarm_voice_control/setup.py`
- `src/rebotarmcontroller/rebotarmcontroller/examples/p0_gate_bc_acceptance.py`
- `src/rebotarmcontroller/rebotarmcontroller/p0_acceptance_core.py`
- `tests/conftest.py`
- `tests/test_agent_state.py`
- `tests/test_approach_policy.py`
- `tests/test_aruco_reference.py`
- `tests/test_calibration_aruco_pose.py`
- `tests/test_camera_info_msgs.py`
- `tests/test_candidate_filter_scheduling.py`
- `tests/test_candidate_gate_policy.py`
- `tests/test_candidate_motion_policy.py`
- `tests/test_candidate_scoring_policy.py`
- `tests/test_candidate_target_policy.py`
- `tests/test_candidate_tf_adapter.py`
- `tests/test_controller_feedback_cache.py`
- `tests/test_detection_msgs.py`
- `tests/test_gemini2_driver_metadata.py`
- `tests/test_grasp_preview_sender.py`
- `tests/test_grasp_tcp_frame.py`
- `tests/test_graspnet_baseline_adapter.py`
- `tests/test_graspnet_baseline_inference_wrapper.py`
- `tests/test_gravity_compensation_core.py`
- `tests/test_gripper_action_completion.py`
- `tests/test_gripper_safety_configuration.py`
- `tests/test_handeye_config.py`
- `tests/test_handeye_residual.py`
- `tests/test_handeye_solver.py`
- `tests/test_hardware_manager_p0_safety.py`
- `tests/test_interactive_control_phase1.py`
- `tests/test_launch_interpreters.py`
- `tests/test_motion_feasibility_policy.py`
- `tests/test_motorbridge_freshness_setup.py`
- `tests/test_motorbridge_transport_safety.py`
- `tests/test_moveit_gripper_srdf.py`
- `tests/test_mujoco_adapter_core.py`
- `tests/test_mujoco_bottle_scene.py`
- `tests/test_mujoco_grasp_quality.py`
- `tests/test_mujoco_headless.py`
- `tests/test_mujoco_health.py`
- `tests/test_mujoco_limit_consistency.py`
- `tests/test_mujoco_metrics.py`
- `tests/test_mujoco_model_profile.py`
- `tests/test_mujoco_offline_perception_launch.py`
- `tests/test_mujoco_ros_adapter_launch.py`
- `tests/test_mujoco_runner_keyframes.py`
- `tests/test_mujoco_runner_suite.py`
- `tests/test_mujoco_virtual_camera.py`
- `tests/test_network_depth_driver.py`
- `tests/test_network_detection_json.py`
- `tests/test_network_graspnet_client.py`
- `tests/test_network_mjpeg_driver.py`
- `tests/test_offline_yolo.py`
- `tests/test_ordinary_grasp_adapter.py`
- `tests/test_p0_gate_bc_acceptance.py`
- `tests/test_p3_ubuntu_native_profile.py`
- `tests/test_p5_handeye_failure_recovery.py`
- `tests/test_p6_bottle_candidate_confidence.py`
- `tests/test_package_layering.py`
- `tests/test_package_resources.py`
- `tests/test_paired_trajectory_protocol.py`
- `tests/test_pose_variant_policy.py`
- `tests/test_rebotarm_app_launch.py`
- `tests/test_recording_feedback.py`
- `tests/test_safe_home_pose_consistency.py`
- `tests/test_sim_trajectory_controller.py`
- `tests/test_simulation_source_swap.py`
- `tests/test_status_panel_layering.py`
- `tests/test_tcp_calibration.py`
- `tests/test_teach_replay_workflow.py`
- `tests/test_teleop_teach_core.py`
- `tests/test_trajectory_runtime_limits.py`
- `tests/test_transform_points.py`
- `tests/test_ubuntu_vision_setup.py`
- `tests/test_upstream_backend_selection.py`
- `tests/test_vision_timestamp_policy.py`
- `tests/test_visual_grasp_launch_cleanup.py`
- `tests/test_visual_grasp_markers.py`
- `tests/test_visual_grasp_pose_policy.py`
- `tests/test_visual_grasp_sequence.py`
- `tests/test_visual_grasp_wiring.py`
- `tests/test_visual_ready_motion.py`
- `tests/test_voice_command_router.py`
- `tests/test_voice_config_loader.py`
- `tests/test_voice_execution_modes.py`
- `tests/test_voice_file_pipeline.py`
- `tests/test_voice_intent_parser.py`
- `tests/test_voice_llm_pipeline.py`
- `tests/test_voice_llm_providers.py`
- `tests/test_voice_move_relative_action.py`
- `tests/test_voice_online_asr.py`
- `tests/test_voice_real_launch.py`
- `tests/test_voice_realtime_bridge.py`
- `tests/test_voice_realtime_gateway.py`
- `tests/test_voice_realtime_launch.py`
- `tests/test_voice_ros2_action_transport.py`
- `tests/test_voice_safety_guard.py`
- `tests/test_voice_sim_action_bindings.py`
- `tests/test_voice_sim_config.py`
- `tests/test_voice_sim_executor.py`
- `tests/test_voice_sim_launch.py`
- `tests/test_voice_sim_motion_adapter.py`
- `tests/test_voice_sim_move_relative_action_node.py`
- `tests/test_voice_task_planner.py`
- `tests/test_voice_text_pipeline.py`
- `tests/test_voice_tool_call_schema.py`
- `tests/test_voice_tool_cli_pipeline.py`
- `third_party/COLCON_IGNORE`
- `tools/check_ubuntu_graspnet_env.py`
- `tools/graspnet_baseline_inference.py`
- `tools/install_orbbec_udev_rules.sh`
- `tools/p5_build_paired_report.py`
- `tools/p5_extract_handeye_prior.py`
- `tools/p5_extract_handeye_result.py`
- `tools/p5_generate_paired_commands.py`
- `tools/p5_handeye_recalibration_runner.py`
- `tools/p5_handeye_replacement_pose_runner.py`
- `tools/p5_handeye_residual_runner.py`
- `tools/p5_paired_trajectory_runner.py`
- `tools/p5_solve_handeye_recalibration.py`
- `tools/p6_single_bottle_grasp_runner.py`
- `tools/run_ubuntu_graspnet_service.sh`
- `tools/run_ubuntu_vision.sh`
- `tools/setup_ubuntu_graspnet.sh`
- `tools/setup_ubuntu_vision.sh`
- `tools/source_local_environment.bash`
- `tools/ubuntu_graspnet_service.py`

## 仅旧版存在的路径

- `.gitattributes`
- `.github/workflows/architecture.yml`
- `Log/OrbbecSDK.log.txt`
- `docs/README.md`
- `docs/acceptance_criteria_zh.md`
- `docs/acceptance_records/2026-09-11-baseline.md`
- `docs/acceptance_records/2026-09-11-boundary-migration-tests.xml`
- `docs/acceptance_records/2026-09-11-boundary-migration.md`
- `docs/acceptance_records/2026-09-11-functional-fixes-tests.xml`
- `docs/acceptance_records/2026-09-11-functional-fixes.md`
- `docs/acceptance_records/2026-09-11-functional-regression-tests.xml`
- `docs/acceptance_records/2026-09-11-functional-regression.md`
- `docs/acceptance_records/2026-09-11-local-ros-perception-tests.xml`
- `docs/acceptance_records/2026-09-11-local-ros-perception.md`
- `docs/acceptance_records/2026-09-11-open3d-viewer-tests.xml`
- `docs/acceptance_records/2026-09-11-open3d-viewer.md`
- `docs/acceptance_records/2026-09-11-perception-runtime-tests.xml`
- `docs/acceptance_records/2026-09-11-perception-runtime.md`
- `docs/acceptance_records/TEMPLATE.md`
- `docs/functional_specification_zh.md`
- `docs/local_ros_vision_zh.md`
- `docs/project_architecture_zh.md`
- `docs/project_stage_zh.md`
- `models/MANIFEST.sha256`
- `models/README.md`
- `patches/graspnet-baseline/0001-use-fixed-width-knn-indices.patch`
- `patches/graspnetAPI/0001-optional-evaluation-imports.patch`
- `patches/graspnetAPI/0002-separate-inference-and-evaluation-dependencies.patch`
- `patches/rebotarm_control_py/0001-feedback-and-safe-home-safety.patch`
- `src/rebotarm_bringup/README.md`
- `src/rebotarm_bringup/config/controller_runtime.yaml`
- `src/rebotarm_bringup/config/controller_safety.yaml`
- `src/rebotarm_bringup/config/rviz_real.yaml`
- `src/rebotarm_bringup/launch/README.md`
- `src/rebotarm_bringup/launch/core.launch.py`
- `src/rebotarm_calibration/README.md`
- `src/rebotarm_calibration/rebotarm_calibration/geometry.py`
- `src/rebotarm_calibration/rebotarm_calibration/handeye_config.py`
- `src/rebotarm_dashboard/README.md`
- `src/rebotarm_dashboard/rebotarm_dashboard/panel_config.py`
- `src/rebotarm_description/README.md`
- `src/rebotarm_description/config/arm.yaml`
- `src/rebotarm_description/config/gripper.yaml`
- `src/rebotarm_description/description/meshes/base_link.STL`
- `src/rebotarm_description/description/meshes/end_link.STL`
- `src/rebotarm_description/description/meshes/gripper_base.stl`
- `src/rebotarm_description/description/meshes/gripper_hardware.stl`
- `src/rebotarm_description/description/meshes/left_finger.stl`
- `src/rebotarm_description/description/meshes/link1.STL`
- `src/rebotarm_description/description/meshes/link2.STL`
- `src/rebotarm_description/description/meshes/link3.STL`
- `src/rebotarm_description/description/meshes/link4.STL`
- `src/rebotarm_description/description/meshes/link5.STL`
- `src/rebotarm_description/description/meshes/link6.STL`
- `src/rebotarm_description/description/meshes/right_finger.stl`
- `src/rebotarm_description/description/urdf/reBot-DevArm_fixend.urdf`
- `src/rebotarm_description/package.xml`
- `src/rebotarm_description/resource/rebotarm_description`
- `src/rebotarm_description/setup.py`
- `src/rebotarm_motion/README.md`
- `src/rebotarm_motion/rebotarm_motion/replay_start_policy.py`
- `src/rebotarm_motion/rebotarm_motion/teach_sample_processing.py`
- `src/rebotarm_moveit_config/README.md`
- `src/rebotarm_msgs/README.md`
- `src/rebotarm_simulation/README.md`
- `src/rebotarm_simulation/config/mujoco_collision.yaml`
- `src/rebotarm_simulation/config/real2sim_bridge.yaml`
- `src/rebotarm_simulation/config/real2sim_mapping.yaml`
- `src/rebotarm_simulation/config/sim2real_randomization.yaml`
- `src/rebotarm_simulation/launch/real2sim_bridge.launch.py`
- `src/rebotarm_simulation/rebotarm_simulation/gym_adapter.py`
- `src/rebotarm_simulation/rebotarm_simulation/model_contract.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_acceptance.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_adapters.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_batch.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_cartesian.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_commands.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_contact_check.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_contact_reader.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_dashboard.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_env.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_jog.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_model_index.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_moveit_acceptance.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_pick_batch.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_pick_env.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_ros_acceptance.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_scene_runtime.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_telemetry.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_visualization.py`
- `src/rebotarm_simulation/rebotarm_simulation/pick_task.py`
- `src/rebotarm_simulation/rebotarm_simulation/reach_task.py`
- `src/rebotarm_simulation/rebotarm_simulation/real2sim.py`
- `src/rebotarm_simulation/rebotarm_simulation/real2sim_acceptance.py`
- `src/rebotarm_simulation/rebotarm_simulation/real2sim_ros_node.py`
- `src/rebotarm_simulation/rebotarm_simulation/real2sim_viewer.py`
- `src/rebotarm_simulation/rebotarm_simulation/rl_schema.py`
- `src/rebotarm_simulation/rebotarm_simulation/ros_diagnostics.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real/__init__.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real/randomization.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real/replay_compare.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real/schemas.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real/trajectory_log.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real/validation.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim2real_cli.py`
- `src/rebotarm_simulation/rebotarm_simulation/simulation_config.py`
- `src/rebotarm_simulation/rebotarm_simulation/simulation_protocol.py`
- `src/rebotarm_simulation/rebotarm_simulation/trajectory_execution.py`
- `src/rebotarm_simulation/rebotarm_simulation/vector_env.py`
- `src/rebotarm_simulation/rebotarm_simulation/viewer_app.py`
- `src/rebotarm_simulation/rebotarm_simulation/viewer_input.py`
- `src/rebotarm_simulation/rebotarm_simulation/viewer_runtime.py`
- `src/rebotarm_simulation/rebotarm_simulation/viewer_state.py`
- `src/rebotarm_teach/README.md`
- `src/rebotarm_teach/rebotarm_teach/record_joint_selection.py`
- `src/rebotarm_teach/rebotarm_teach/teach_record_repository.py`
- `src/rebotarm_teach/rebotarm_teach/teach_record_types.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_parameter_adapter.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_parameters.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_payload.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_session.py`
- `src/rebotarm_teleop/README.md`
- `src/rebotarm_teleop/rebotarm_teleop/web_execute_session.py`
- `src/rebotarm_teleop/rebotarm_teleop/web_gripper_client.py`
- `src/rebotarm_teleop/rebotarm_teleop/web_keyboard_client.py`
- `src/rebotarm_vision/README.md`
- `src/rebotarm_vision/config/graspnet_viewer.yaml`
- `src/rebotarm_vision/config/visual_ready.yaml`
- `src/rebotarm_vision/constraints-ubuntu.txt`
- `src/rebotarm_vision/launch/graspnet_viewer.launch.py`
- `src/rebotarm_vision/models/yolo11n-seg.pt`
- `src/rebotarm_vision/rebotarm_vision/depth_utils.py`
- `src/rebotarm_vision/rebotarm_vision/freshness.py`
- `src/rebotarm_vision/rebotarm_vision/grasp_plan_store.py`
- `src/rebotarm_vision/rebotarm_vision/graspnet_inference.py`
- `src/rebotarm_vision/rebotarm_vision/graspnet_viewer_node.py`
- `src/rebotarm_vision/rebotarm_vision/graspnet_visualization.py`
- `src/rebotarm_vision/rebotarm_vision/model_paths.py`
- `src/rebotarm_vision/rebotarm_vision/perception_frames.py`
- `src/rebotarm_vision/rebotarm_vision/visual_failure_recovery.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_execution_state.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_parameter_adapter.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_plan_builder.py`
- `src/rebotarm_vision/rebotarm_vision/visual_gripper_gateway.py`
- `src/rebotarm_vision/rebotarm_vision/visual_motion_gateway.py`
- `src/rebotarm_vision/rebotarm_vision/visual_trigger_gateway.py`
- `src/rebotarm_vision/rebotarm_vision/yolo_node.py`
- `src/rebotarm_vision/requirements-local.txt`
- `src/rebotarmcontroller/README.md`
- `src/rebotarmcontroller/rebotarmcontroller/bus_synchronization.py`
- `src/rebotarmcontroller/rebotarmcontroller/command_arbiter.py`
- `src/rebotarmcontroller/rebotarmcontroller/feedback_sequence.py`
- `src/rebotarmcontroller/rebotarmcontroller/gravity_compensation_state.py`
- `src/rebotarmcontroller/rebotarmcontroller/gravity_dynamics.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_coordinates.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_grasp.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_motion_policy.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_motor_commands.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_position.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_runtime_state.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_safety.py`
- `src/rebotarmcontroller/rebotarmcontroller/gripper_sdk_adapter.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_disable.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_feedback.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_feedback_validation.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_lifecycle_state.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_runtime_config.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_sdk_runtime.py`
- `src/rebotarmcontroller/rebotarmcontroller/joint_motor_commands.py`
- `src/rebotarmcontroller/rebotarmcontroller/runtime_parameters.py`
- `src/rebotarmcontroller/rebotarmcontroller/sdk_runtime.py`
- `src/rebotarmcontroller/rebotarmcontroller/trajectory_safety.py`
- `tests/test_architecture_guard.py`
- `tests/test_architecture_migration.py`
- `tests/test_aruco_opencv_runtime.py`
- `tests/test_functional_fixes.py`
- `tests/test_graspnet_viewer.py`
- `tests/test_local_ros_perception.py`
- `tests/test_rviz_execute_workflow.py`
- `tools/apply_rebotarm_control_safety_patch.py`
- `tools/architecture_rules.json`
- `tools/bootstrap_ubuntu_dependencies.sh`
- `tools/build_graspnet_extensions.bash`
- `tools/build_workspace.bash`
- `tools/check_architecture.py`
- `tools/check_local_perception.py`
- `tools/check_python_entrypoints.py`
- `tools/convert_arm_teach_record.py`
- `tools/source_graspnet_build_env.bash`
- `tools/source_ubuntu_env.bash`
- `tools/udev/70-rebotarm-gemini2.rules`
- `tools/verify_live_vision.py`
- `tools/verify_perception_models.py`
- `tools/verify_perception_ros.py`
- `tools/verify_sim_workflows.py`

## 已变更的路径

- `.gitignore`
- `docs/rebotarm_feature_commands.md`
- `rebotarm_dependencies.repos`
- `src/rebotarm_bringup/config/interactive_control.yaml`
- `src/rebotarm_bringup/config/teleop_control.yaml`
- `src/rebotarm_bringup/launch/bringup.launch.py`
- `src/rebotarm_bringup/launch/driver_only.launch.py`
- `src/rebotarm_bringup/launch/interactive_system.launch.py`
- `src/rebotarm_bringup/launch/moveit_hardware.launch.py`
- `src/rebotarm_bringup/launch/real_perception_sim_execution.launch.py`
- `src/rebotarm_bringup/launch/rebotarm_app.launch.py`
- `src/rebotarm_bringup/launch/rviz_ee_drag_real.launch.py`
- `src/rebotarm_bringup/launch/rviz_ee_drag_sim.launch.py`
- `src/rebotarm_bringup/launch/teach_record.launch.py`
- `src/rebotarm_bringup/launch/teach_replay.launch.py`
- `src/rebotarm_bringup/launch/teleop_keyboard.launch.py`
- `src/rebotarm_bringup/launch/teleop_system.launch.py`
- `src/rebotarm_bringup/launch/visual_grasp_perception_preview.launch.py`
- `src/rebotarm_bringup/launch/visual_grasp_system.launch.py`
- `src/rebotarm_bringup/launch/visual_ready_hold.launch.py`
- `src/rebotarm_bringup/package.xml`
- `src/rebotarm_bringup/rviz/interactive_system.rviz`
- `src/rebotarm_bringup/rviz/rebotarm.rviz`
- `src/rebotarm_bringup/rviz/web_teleop_status.rviz`
- `src/rebotarm_bringup/setup.py`
- `src/rebotarm_calibration/package.xml`
- `src/rebotarm_calibration/rebotarm_calibration/__init__.py`
- `src/rebotarm_calibration/rebotarm_calibration/tcp_calibration.py`
- `src/rebotarm_calibration/setup.py`
- `src/rebotarm_dashboard/package.xml`
- `src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html`
- `src/rebotarm_dashboard/rebotarm_dashboard/status_panel_state.py`
- `src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py`
- `src/rebotarm_dashboard/rebotarm_dashboard/web_robot_assets.py`
- `src/rebotarm_interactive_control/README.md`
- `src/rebotarm_interactive_control/package.xml`
- `src/rebotarm_interactive_control/setup.py`
- `src/rebotarm_motion/package.xml`
- `src/rebotarm_motion/rebotarm_motion/moveit_planner.py`
- `src/rebotarm_motion/rebotarm_motion/teach_replay_start_align_precheck.py`
- `src/rebotarm_motion/rebotarm_motion/teach_replay_start_alignment.py`
- `src/rebotarm_motion/setup.py`
- `src/rebotarm_moveit_config/README_zh.md`
- `src/rebotarm_moveit_config/config/reBot-DevArm_fixend.srdf`
- `src/rebotarm_moveit_config/config/rebotarm.srdf`
- `src/rebotarm_moveit_config/launch/demo.launch.py`
- `src/rebotarm_moveit_config/package.xml`
- `src/rebotarm_moveit_config/rviz/moveit.rviz`
- `src/rebotarm_moveit_config/setup.py`
- `src/rebotarm_simulation/README_mujoco.md`
- `src/rebotarm_simulation/config/motor_control_calibration.yaml`
- `src/rebotarm_simulation/config/mujoco_sim.yaml`
- `src/rebotarm_simulation/launch/mujoco_sim.launch.py`
- `src/rebotarm_simulation/models/rebotarm/robot.xml`
- `src/rebotarm_simulation/models/rebotarm/scene.xml`
- `src/rebotarm_simulation/package.xml`
- `src/rebotarm_simulation/rebotarm_simulation/motor_control.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_cli.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_health.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_ros_node.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_sim.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_types.py`
- `src/rebotarm_simulation/rebotarm_simulation/mujoco_viewer.py`
- `src/rebotarm_simulation/rebotarm_simulation/sim_trajectory_controller_node.py`
- `src/rebotarm_simulation/rebotarm_simulation/trajectory_sampler.py`
- `src/rebotarm_simulation/rebotarm_simulation/urdf_to_mjcf.py`
- `src/rebotarm_simulation/setup.py`
- `src/rebotarm_teach/rebotarm_teach/teach_recorder_node.py`
- `src/rebotarm_teach/rebotarm_teach/teach_recording.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_node.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_settings.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_trajectory_builder.py`
- `src/rebotarm_teach/rebotarm_teach/teach_replay_workflow.py`
- `src/rebotarm_teleop/rebotarm_teleop/gripper_visual_joint_state_node.py`
- `src/rebotarm_teleop/rebotarm_teleop/web_teleop_client.py`
- `src/rebotarm_vision/README_zh.md`
- `src/rebotarm_vision/config/camera.yaml`
- `src/rebotarm_vision/config/flat_graspnet.yaml`
- `src/rebotarm_vision/config/grasp_pose_policy.yaml`
- `src/rebotarm_vision/config/graspnet_policy.yaml`
- `src/rebotarm_vision/config/gripper_policy.yaml`
- `src/rebotarm_vision/config/handeye.yaml`
- `src/rebotarm_vision/config/retreat_policy.yaml`
- `src/rebotarm_vision/config/retry_policy.yaml`
- `src/rebotarm_vision/config/table_safety.yaml`
- `src/rebotarm_vision/launch/vision.launch.py`
- `src/rebotarm_vision/package.xml`
- `src/rebotarm_vision/rebotarm_vision/aruco_reference.py`
- `src/rebotarm_vision/rebotarm_vision/camera/gemini2_driver.py`
- `src/rebotarm_vision/rebotarm_vision/candidate_ik_filter_node.py`
- `src/rebotarm_vision/rebotarm_vision/candidate_target_policy.py`
- `src/rebotarm_vision/rebotarm_vision/converters/detection_msgs.py`
- `src/rebotarm_vision/rebotarm_vision/detector/yolo_detector.py`
- `src/rebotarm_vision/rebotarm_vision/grasp_depth_probe_node.py`
- `src/rebotarm_vision/rebotarm_vision/grasp_preview_sender_node.py`
- `src/rebotarm_vision/rebotarm_vision/graspnet_baseline_adapter.py`
- `src/rebotarm_vision/rebotarm_vision/graspnet_baseline_node.py`
- `src/rebotarm_vision/rebotarm_vision/gripper_policy.py`
- `src/rebotarm_vision/rebotarm_vision/handeye_config.py`
- `src/rebotarm_vision/rebotarm_vision/retreat_policy.py`
- `src/rebotarm_vision/rebotarm_vision/tcp_calibration.py`
- `src/rebotarm_vision/rebotarm_vision/tcp_calibration_node.py`
- `src/rebotarm_vision/rebotarm_vision/vision_node.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_marker_node.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_pose_policy.py`
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_sequence.py`
- `src/rebotarm_vision/rebotarm_vision/visual_ready_node.py`
- `src/rebotarm_vision/setup.py`
- `src/rebotarmcontroller/package.xml`
- `src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py`
- `src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`
- `src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py`
- `src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py`
- `src/rebotarmcontroller/rebotarmcontroller/ros_actions.py`
- `src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py`
- `src/rebotarmcontroller/rebotarmcontroller/ros_services.py`
- `src/rebotarmcontroller/setup.py`
- `star_arm_102_rebot_b601_follow/Python_SDK/rebot_b601_mapping/README.md`
- `star_arm_102_rebot_b601_follow/Python_SDK/rebot_b601_mapping/follower_controller.py`
- `star_arm_102_rebot_b601_follow/Python_SDK/rebot_b601_mapping/hardware_specs.py`
- `tools/setup_motorbridge_fresh_feedback.py`

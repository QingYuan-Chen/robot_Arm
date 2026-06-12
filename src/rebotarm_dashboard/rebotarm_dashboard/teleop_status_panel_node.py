from __future__ import annotations

import json
import math
import threading
import time
from contextlib import suppress
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetTeachRecordPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_control_client import ArmControlClient
from .arm_command_api import (
    arm_command_is_replay_locked,
    normalize_arm_command,
    should_stop_trajectory_before_arm_command,
    status_state,
)
from rebotarm_motion.collision_precheck import CollisionPrecheckConfig, CollisionPrechecker
from .parameter_helpers import build_joint_limits
from .parameter_helpers import sensor_qos_kwargs
from rebotarm_motion.replay_runtime_monitor import ReplayRuntimeMonitor, ReplayRuntimeMonitorConfig
from .status_panel_http import create_status_panel_server
from .status_panel_page import HTML_PAGE
from .status_panel_state import TeleopStatusStore
from .web_command_gateway import WebCommandGateway, WebCommandRequest
from rebotarm_teleop.teleop_core import validate_web_keyboard_command
from rebotarm_teach.teach_record_client import TeachRecordClient
from rebotarm_teach.teach_replay_coordinator import TeachReplayCoordinator, TeachReplayLimits
from rebotarm_teach.teach_replay_client import TeachReplayClient
from rebotarm_teach.teach_recording import (
    ReplayStartBand,
    inspect_teach_record,
    list_teach_record_files,
    load_teach_samples,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    teach_record_info_to_dict,
    teach_trajectory_preview_to_dict,
    validate_teach_dry_run_request,
    write_prepared_teach_record,
)
from rebotarm_teach.teach_replay_settings import TeachReplaySettingsProvider
from rebotarm_motion.teach_replay_start_align_precheck import (
    MoveItStartAlignPrecheckConfig,
    MoveItStartAlignPrechecker,
)
from rebotarm_motion.teach_replay_start_alignment import MoveItStartAligner, MoveItStartAlignmentConfig
from rebotarm_teach.teach_replay_trajectory_builder import (
    TeachReplayTrajectoryBuilder,
    TeachReplayTrajectoryConfig,
    set_duration,
)
from rebotarm_motion.moveit_planner import MoveItMotionPlanner
from .web_robot_assets import (
    DEFAULT_GRIPPER_LIMITS_M,
    gripper_opening_to_finger_joint_positions,
    load_gripper_limits,
    load_moveit_velocity_limits,
    load_urdf_joint_limits,
    merge_velocity_limits,
    merge_joint_limits,
)
from rebotarm_teleop.web_execute import (
    WebExecuteDecision,
    WebGripperDecision,
)
from rebotarm_teleop.web_teleop_client import WebTeleopClient, decision_response, gripper_decision_response


def _set_duration(duration_msg, seconds: float) -> None:
    set_duration(duration_msg, seconds)


def _is_number_like(value) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def _number_or_default(value, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not _is_number_like(number) or not math.isfinite(number):
        return float(default)
    return float(number)


def _decision_response(decision: WebExecuteDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "max_delta": float(decision.max_delta),
        "max_delta_limit": float(decision.max_delta_limit),
        "duration": float(decision.duration),
    }


def _keyboard_decision_response(decision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "key": str(decision.key),
        "joint_name": str(decision.joint_name),
        "step_rad": float(decision.step_rad),
        "duration": float(decision.duration),
        "max_joint_speed_rad_s": float(decision.max_joint_speed_rad_s),
    }




class TeleopStatusPanelNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_status_panel_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8088)
        self.declare_parameter("sse_rate_hz", 10.0)
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("direct_threshold", 0.01)
        self.declare_parameter("align_threshold", 0.25)
        self.declare_parameter("align_duration", 3.0)
        self.declare_parameter("align_duration_auto", True)
        self.declare_parameter("align_target_speed_rad_s", 0.15)
        self.declare_parameter("align_min_duration", 3.0)
        self.declare_parameter("align_max_duration", 10.0)
        self.declare_parameter("align_steps", 30)
        self.declare_parameter("replay_speed", 1.0)
        self.declare_parameter("green_jump_rad", 0.03)
        self.declare_parameter("yellow_jump_rad", 0.05)
        self.declare_parameter("yellow_max_speed", 0.6)
        self.declare_parameter("max_replay_velocity_rad_s", 3.0)
        self.declare_parameter(
            "max_replay_velocity_rad_s_by_joint",
            [3.0, 3.0, 3.0, 1.8, 1.8, 1.8],
        )
        self.declare_parameter("max_replay_acceleration_rad_s2", 5.0)
        self.declare_parameter("max_replay_jerk_rad_s3", 20.0)
        self.declare_parameter("large_motion_span_rad", 0.8)
        self.declare_parameter("large_motion_total_rad", 2.5)
        self.declare_parameter("large_motion_max_speed", 1.0)
        self.declare_parameter("start_hold_sec", 0.8)
        self.declare_parameter("soft_start_duration", 1.0)
        self.declare_parameter("soft_start_steps", 30)
        self.declare_parameter("first_hold_sec", 0.3)
        self.declare_parameter("final_hold_sec", 1.0)
        self.declare_parameter("initial_replay_delay_sec", 0.2)
        self.declare_parameter("use_moveit_start_align", True)
        self.declare_parameter("moveit_start_skip_threshold", 0.005)
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_planning_time", 3.0)
        self.declare_parameter("moveit_num_planning_attempts", 3)
        self.declare_parameter("moveit_joint_goal_tolerance", 0.005)
        self.declare_parameter("moveit_velocity_scaling", 0.1)
        self.declare_parameter("moveit_acceleration_scaling", 0.1)
        self.declare_parameter("collision_check_enabled", True)
        self.declare_parameter("collision_check_service", "/check_state_validity")
        self.declare_parameter("collision_check_max_samples", 80)
        self.declare_parameter("collision_check_timeout_sec", 2.0)
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("smoothing_window", 7)
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("filter_cutoff_hz", 5.0)
        self.declare_parameter("filter_sample_rate_hz", 150.0)
        self.declare_parameter("resample_enabled", True)
        self.declare_parameter("resample_rate_hz", 150.0)
        self.declare_parameter("time_parameterization_method", "auto")
        self.declare_parameter("max_prepared_jump_rad", 0.02)
        self.declare_parameter("replay_monitor_enabled", True)
        self.declare_parameter("replay_monitor_period_sec", 0.05)
        self.declare_parameter("replay_monitor_start_grace_sec", 1.0)
        self.declare_parameter("replay_monitor_violation_grace_sec", 0.30)
        self.declare_parameter("max_tracking_error_rad", 0.25)
        self.declare_parameter("max_live_velocity_rad_s", 3.0)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("execution_mode", "dry_run")
        self.declare_parameter("panel_mode", "control")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        self.declare_parameter("web_execute_enabled", False)
        self.declare_parameter("web_execute_max_delta_rad", 1.5)
        self.declare_parameter("web_execute_min_duration", 1.0)
        self.declare_parameter("web_execute_max_duration", 8.0)
        self.declare_parameter("web_execute_max_joint_speed_rad_s", 1.5)
        self.declare_parameter("web_keyboard_default_step_rad", 0.02)
        self.declare_parameter("web_keyboard_min_step_rad", 0.005)
        self.declare_parameter("web_keyboard_max_step_rad", 0.10)
        self.declare_parameter("web_keyboard_default_duration", 0.2)
        self.declare_parameter("web_keyboard_min_duration", 0.1)
        self.declare_parameter("web_keyboard_max_duration", 2.0)
        self.declare_parameter("web_keyboard_default_speed_rad_s", 0.5)
        self.declare_parameter("gripper_lower_limit_m", DEFAULT_GRIPPER_LIMITS_M[0])
        self.declare_parameter("gripper_upper_limit_m", DEFAULT_GRIPPER_LIMITS_M[1])
        self.declare_parameter("web_gripper_max_effort", 0.3)
        self.declare_parameter("web_gripper_max_effort_limit", 1.5)
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        bringup_share = Path(get_package_share_directory("rebotarm_bringup"))
        self._urdf_path = bringup_share / "description" / "urdf" / "reBot-DevArm_fixend.urdf"
        self._mesh_dir = bringup_share / "description" / "meshes"
        lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        fallback_limits = build_joint_limits(
            joint_names=self._joint_names,
            lower_limits=lower,
            upper_limits=upper,
        )
        try:
            urdf_limits = load_urdf_joint_limits(self._urdf_path, self._joint_names)
        except Exception as exc:
            self.get_logger().warn(f"failed to load URDF joint limits, using parameter limits: {exc}")
            urdf_limits = {}
        self._joint_limits = merge_joint_limits(
            joint_names=self._joint_names,
            fallback_limits=fallback_limits,
            preferred_limits=urdf_limits,
        )
        moveit_share = Path(get_package_share_directory("rebotarm_moveit_config"))
        moveit_velocity_limits = load_moveit_velocity_limits(
            moveit_share / "config" / "joint_limits.yaml",
            self._joint_names,
        )
        self._joint_velocity_limits = merge_velocity_limits(
            joint_names=self._joint_names,
            default_limit=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            preferred_limits=moveit_velocity_limits,
        )
        gripper_lower = float(self.get_parameter("gripper_lower_limit_m").value)
        gripper_upper = float(self.get_parameter("gripper_upper_limit_m").value)
        if gripper_upper < gripper_lower:
            gripper_lower, gripper_upper = gripper_upper, gripper_lower
        self._gripper_limits = (gripper_lower, gripper_upper)
        self._use_hardware = bool(self.get_parameter("use_hardware").value)
        self._web_command_gateway = WebCommandGateway()
        self._sim_gripper_position = gripper_lower
        self._store = TeleopStatusStore()
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._moveit_planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id="end_link",
            frame_id="base_link",
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=0.005,
            goal_orientation_tolerance=0.02,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
        )
        self._collision_prechecker = CollisionPrechecker(
            client=self._state_validity_client,
            request_factory=GetStateValidity.Request,
        )
        self._gripper_action_client = ActionClient(
            self,
            GripperCommand,
            f"/{self._arm_namespace}/gripper/command",
        )
        self._gravity_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/start",
        )
        self._gravity_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/stop",
        )
        self._teach_record_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/start",
        )
        self._teach_record_set_path_client = self.create_client(
            SetTeachRecordPath,
            f"/{self._arm_namespace}/teleop/teach_record/set_path",
        )
        self._teach_record_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/stop",
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._arm_enable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/enable",
        )
        self._arm_disable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/disable",
        )
        self._arm_safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
        )
        self._arm_control_client = ArmControlClient(
            enable_client=self._arm_enable_client,
            disable_client=self._arm_disable_client,
            safe_home_client=self._arm_safe_home_client,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        self._teach_record_client = TeachRecordClient(
            set_path_client=self._teach_record_set_path_client,
            start_client=self._teach_record_start_client,
            stop_client=self._teach_record_stop_client,
            gravity_start_client=self._gravity_start_client,
            gravity_stop_client=self._gravity_stop_client,
            record_path_request_factory=SetTeachRecordPath.Request,
        )
        self._teach_replay_client = TeachReplayClient()
        self._teach_replay_coordinator = TeachReplayCoordinator()
        self._teach_replay_trajectory_builder = TeachReplayTrajectoryBuilder(
            trajectory_factory=JointTrajectory,
            trajectory_point_factory=JointTrajectoryPoint,
        )
        self._moveit_start_aligner = MoveItStartAligner(
            planner=self._moveit_planner,
            trajectory_point_factory=JointTrajectoryPoint,
        )
        self._moveit_start_align_prechecker = MoveItStartAlignPrechecker(
            planner=self._moveit_planner,
            service_client=self._moveit_planner._client,  # noqa: SLF001
        )
        self._teach_replay_settings_provider = TeachReplaySettingsProvider(
            replay_speed=float(self.get_parameter("replay_speed").value),
            align_duration=float(self.get_parameter("align_duration").value),
            align_duration_auto=bool(self.get_parameter("align_duration_auto").value),
            align_target_speed_rad_s=float(self.get_parameter("align_target_speed_rad_s").value),
            align_min_duration=float(self.get_parameter("align_min_duration").value),
            align_max_duration=float(self.get_parameter("align_max_duration").value),
            align_steps=int(self.get_parameter("align_steps").value),
        )
        self._web_teleop_client = WebTeleopClient(
            action_client=self._action_client,
            joint_names=self._joint_names,
            joint_limits=self._joint_limits,
            joint_velocity_limits=self._joint_velocity_limits,
            trajectory_factory=JointTrajectory,
            trajectory_point_factory=JointTrajectoryPoint,
            follow_goal_factory=FollowJointTrajectory.Goal,
            gripper_action_client=self._gripper_action_client,
            gripper_goal_factory=GripperCommand.Goal,
        )
        self._execute_lock = threading.Lock()
        self._execute_goal_handle = None
        self._web_keyboard_lock = threading.Lock()
        self._web_keyboard_enabled = False
        self._web_keyboard_step_rad = float(self.get_parameter("web_keyboard_default_step_rad").value)
        self._web_keyboard_duration = float(self.get_parameter("web_keyboard_default_duration").value)
        self._web_keyboard_speed = float(self.get_parameter("web_keyboard_default_speed_rad_s").value)
        self._teach_replay_lock = threading.Lock()
        self._teach_replay_goal_handle = None
        self._active_teach_replay_trajectory: JointTrajectory | None = None
        self._active_teach_replay_started_at: float | None = None
        self._replay_runtime_monitor = ReplayRuntimeMonitor()
        self._last_teach_dry_run: dict | None = None
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_subscription(ArmStatus, f"/{self._arm_namespace}/arm_status", self._on_arm_status, arm_status_qos)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/status", lambda msg: self._on_status("status", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/recording_status", lambda msg: self._on_status("recording", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/replay_status", lambda msg: self._on_status("replay", msg), 10)
        for joint_name in self._joint_names:
            self.create_subscription(
                JointMotorState,
                f"/{self._arm_namespace}/joints/{joint_name}/state",
                self._on_motor_state,
                sensor_qos,
            )
        self.create_subscription(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            self._on_gripper_state,
            sensor_qos,
        )
        self._sim_gripper_state_pub = None
        if not self._use_hardware:
            self._sim_gripper_state_pub = self.create_publisher(
                JointMotorState,
                f"/{self._arm_namespace}/gripper/state",
                sensor_qos,
            )
            self.create_timer(0.1, self._publish_simulated_gripper_state)
        self.create_timer(1.0, self._update_gravity_comp_status)
        self.create_timer(
            max(float(self.get_parameter("replay_monitor_period_sec").value), 0.02),
            self._check_active_replay_tracking,
        )
        self._server = self._make_server()
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.get_logger().info(f"teleop status panel available at http://{host}:{port}/")

    def _make_server(self):
        interval = 1.0 / max(float(self.get_parameter("sse_rate_hz").value), 1.0)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        return create_status_panel_server(
            host=host,
            port=port,
            node=self,
            store=self._store,
            html_page=HTML_PAGE,
            urdf_path=self._urdf_path,
            mesh_dir=self._mesh_dir,
            sse_interval_sec=interval,
        )

    def _panel_config(self) -> dict:
        return {
            "joint_names": list(self._joint_names),
            "joint_limits": {
                name: [float(lower), float(upper)]
                for name, (lower, upper) in self._joint_limits.items()
            },
            "joint_velocity_limits": {
                name: float(limit)
                for name, limit in self._joint_velocity_limits.items()
            },
            "gripper_limits": [float(self._gripper_limits[0]), float(self._gripper_limits[1])],
            "web_execute": {
                "enabled": bool(self.get_parameter("web_execute_enabled").value),
                "max_delta_rad": float(self.get_parameter("web_execute_max_delta_rad").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
                "min_duration": float(self.get_parameter("web_execute_min_duration").value),
                "max_duration": float(self.get_parameter("web_execute_max_duration").value),
            },
            "web_keyboard": {
                "step_rad": float(self.get_parameter("web_keyboard_default_step_rad").value),
                "min_step_rad": float(self.get_parameter("web_keyboard_min_step_rad").value),
                "max_step_rad": float(self.get_parameter("web_keyboard_max_step_rad").value),
                "duration": float(self.get_parameter("web_keyboard_default_duration").value),
                "min_duration": float(self.get_parameter("web_keyboard_min_duration").value),
                "max_duration": float(self.get_parameter("web_keyboard_max_duration").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_keyboard_default_speed_rad_s").value),
            },
            "web_gripper": {
                "max_effort": float(self.get_parameter("web_gripper_max_effort").value),
                "max_effort_limit": float(self.get_parameter("web_gripper_max_effort_limit").value),
            },
            "teach": {
                "record_path": str(self.get_parameter("record_path").value),
                "direct_threshold": float(self.get_parameter("direct_threshold").value),
                "align_threshold": float(self.get_parameter("align_threshold").value),
                "align_duration": float(self.get_parameter("align_duration").value),
                "align_duration_auto": bool(self.get_parameter("align_duration_auto").value),
                "align_target_speed_rad_s": float(self.get_parameter("align_target_speed_rad_s").value),
                "align_min_duration": float(self.get_parameter("align_min_duration").value),
                "align_max_duration": float(self.get_parameter("align_max_duration").value),
                "align_steps": int(self.get_parameter("align_steps").value),
                "replay_speed": float(self.get_parameter("replay_speed").value),
                "green_jump_rad": float(self.get_parameter("green_jump_rad").value),
                "yellow_jump_rad": float(self.get_parameter("yellow_jump_rad").value),
                "yellow_max_speed": float(self.get_parameter("yellow_max_speed").value),
                "max_replay_velocity_rad_s": float(self.get_parameter("max_replay_velocity_rad_s").value),
                "max_replay_velocity_rad_s_by_joint": [
                    float(value)
                    for value in self.get_parameter("max_replay_velocity_rad_s_by_joint").value
                ],
                "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
                "large_motion_span_rad": float(self.get_parameter("large_motion_span_rad").value),
                "large_motion_total_rad": float(self.get_parameter("large_motion_total_rad").value),
                "large_motion_max_speed": float(self.get_parameter("large_motion_max_speed").value),
                "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
                "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
                "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
                "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
                "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
                "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
                "moveit_start_skip_threshold": float(self.get_parameter("moveit_start_skip_threshold").value),
                "collision_check_enabled": bool(self.get_parameter("collision_check_enabled").value),
                "collision_check_max_samples": int(self.get_parameter("collision_check_max_samples").value),
                "smoothing_enabled": bool(self.get_parameter("smoothing_enabled").value),
                "smoothing_window": int(self.get_parameter("smoothing_window").value),
                "filter_enabled": bool(self.get_parameter("filter_enabled").value),
                "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
                "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
                "resample_enabled": bool(self.get_parameter("resample_enabled").value),
                "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
                "time_parameterization_method": str(self.get_parameter("time_parameterization_method").value),
                "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
                "use_hardware": bool(self.get_parameter("use_hardware").value) if self.has_parameter("use_hardware") else False,
            },
            "panel_mode": str(self.get_parameter("panel_mode").value),
            "execution_mode": str(self.get_parameter("execution_mode").value),
        }

    def _teach_record_info(self, record_path: str | None = None) -> dict:
        snapshot = self._store.snapshot()
        path = record_path or str(self.get_parameter("record_path").value)
        for key in ("recording", "replay"):
            value = snapshot.teleop.get(key)
            if record_path is None and isinstance(value, dict) and value.get("record_path"):
                path = str(value["record_path"])
                break
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if name in self._joint_names and "position" in data
        }
        info = inspect_teach_record(
            path,
            current_positions=current_positions if current_positions else None,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        payload = teach_record_info_to_dict(info)
        if (
            str(payload.get("start_band", "")).lower() == ReplayStartBand.REJECT.value
            and bool(self.get_parameter("use_moveit_start_align").value)
            and _is_number_like(payload.get("max_error"))
        ):
            payload["start_band"] = ReplayStartBand.MOVEIT_ALIGN.value
            payload["message"] = "start error requires MoveIt start alignment"
        payload["direct_threshold"] = float(self.get_parameter("direct_threshold").value)
        payload["align_threshold"] = float(self.get_parameter("align_threshold").value)
        return self._compact_replay_payload(payload)

    @staticmethod
    def _compact_list(items, *, limit: int = 12) -> list:
        values = list(items) if isinstance(items, (list, tuple)) else []
        return values[: max(int(limit), 0)]

    @classmethod
    def _compact_quality_payload(cls, quality: dict, *, limit: int = 12) -> dict:
        compact = dict(quality)
        if isinstance(compact.get("events"), list):
            compact["events_total"] = len(compact["events"])
            compact["events"] = cls._compact_list(compact["events"], limit=limit)
            compact["events_truncated"] = compact["events_total"] > len(compact["events"])
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        return compact

    @classmethod
    def _compact_replay_payload(cls, payload: dict, *, limit: int = 12) -> dict:
        compact = dict(payload)
        for key in (
            "quality",
            "before_quality",
            "after_quality",
            "raw_quality",
            "filtered_quality",
            "retimed_quality",
        ):
            if isinstance(compact.get(key), dict):
                compact[key] = cls._compact_quality_payload(compact[key], limit=limit)
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        if isinstance(compact.get("prepared_replay"), dict):
            compact["prepared_replay"] = cls._compact_replay_payload(compact["prepared_replay"], limit=limit)
        return compact

    def _max_replay_velocity_limits(self, joint_names: tuple[str, ...]):
        scalar_limit = float(self.get_parameter("max_replay_velocity_rad_s").value)
        values = self.get_parameter("max_replay_velocity_rad_s_by_joint").value
        if isinstance(values, (list, tuple)) and len(values) == len(joint_names):
            return tuple(float(value) for value in values)
        return scalar_limit

    def _prepare_teach_replay_samples(self, samples, settings: dict[str, float | int] | None = None):
        replay_speed = float(settings["replay_speed"]) if settings else float(self.get_parameter("replay_speed").value)
        return prepare_teach_replay_samples(
            samples,
            smoothing_enabled=bool(self.get_parameter("smoothing_enabled").value),
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            filter_enabled=bool(self.get_parameter("filter_enabled").value),
            filter_cutoff_hz=float(self.get_parameter("filter_cutoff_hz").value),
            filter_sample_rate_hz=float(self.get_parameter("filter_sample_rate_hz").value),
            resample_enabled=bool(self.get_parameter("resample_enabled").value),
            resample_rate_hz=float(self.get_parameter("resample_rate_hz").value),
            retime_enabled=True,
            replay_speed=replay_speed,
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(samples[0].joint_names) if samples else ()),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            time_parameterization_method=str(self.get_parameter("time_parameterization_method").value),
            large_motion_span_rad=float(self.get_parameter("large_motion_span_rad").value),
            large_motion_total_rad=float(self.get_parameter("large_motion_total_rad").value),
            large_motion_max_speed=float(self.get_parameter("large_motion_max_speed").value),
        )

    def _moveit_align_summary(self, info_payload: dict, samples=None, *, plan: bool = False) -> dict:
        return self._moveit_start_align_prechecker.summary(
            info_payload,
            config=MoveItStartAlignPrecheckConfig(
                enabled=bool(self.get_parameter("use_moveit_start_align").value),
                service=str(self.get_parameter("moveit_planning_service").value),
                skip_threshold=float(self.get_parameter("moveit_start_skip_threshold").value),
                joint_goal_tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            ),
            samples=samples,
            plan=plan,
        )

    def _collision_precheck(self, samples) -> dict:
        if not samples:
            return self._collision_precheck_positions((), [])
        first = samples[0]
        positions = [tuple(sample.positions) for sample in samples]
        return self._collision_precheck_positions(tuple(first.joint_names), positions)

    def _collision_precheck_trajectory(self, trajectory: JointTrajectory) -> dict:
        positions = [
            tuple(point.positions)
            for point in getattr(trajectory, "points", [])
            if getattr(point, "positions", None)
        ]
        return self._collision_precheck_positions(tuple(trajectory.joint_names), positions)

    def _collision_precheck_positions(self, joint_names: tuple[str, ...], positions_list: list[tuple[float, ...]]) -> dict:
        default_joint_positions = self._collision_default_joint_positions(joint_names)
        return self._collision_prechecker.check_positions(
            joint_names=joint_names,
            positions_list=positions_list,
            config=CollisionPrecheckConfig(
                enabled=bool(self.get_parameter("collision_check_enabled").value),
                service=str(self.get_parameter("collision_check_service").value),
                group_name=str(self.get_parameter("collision_group_name").value),
                max_samples=max(int(self.get_parameter("collision_check_max_samples").value), 1),
                timeout_sec=max(float(self.get_parameter("collision_check_timeout_sec").value), 0.1),
                default_joint_positions=default_joint_positions,
            ),
        )

    def _collision_default_joint_positions(self, joint_names: tuple[str, ...]) -> tuple[tuple[str, float], ...]:
        if str(self.get_parameter("collision_group_name").value) != "arm_with_gripper":
            return ()
        if {"left_finger_joint", "right_finger_joint"}.issubset(set(joint_names)):
            return ()
        joints = self._store.snapshot().joints
        gripper_state = joints.get("gripper", {})
        gripper_position = gripper_state.get("position", self._sim_gripper_position)
        left, right = gripper_opening_to_finger_joint_positions(
            float(gripper_position),
            self._gripper_limits,
        )
        return (("left_finger_joint", left), ("right_finger_joint", right))

    def _teach_replay_limits(self) -> TeachReplayLimits:
        return TeachReplayLimits(
            max_prepared_jump_rad=float(self.get_parameter("max_prepared_jump_rad").value),
            max_replay_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_replay_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
        )

    def _target_runtime(self) -> str:
        return "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation"

    def _route_web_command(self, intent: str, payload: dict | None = None) -> dict | None:
        request_payload = dict(payload or {})
        request_payload["intent"] = intent
        result = self._web_command_gateway.route(
            WebCommandRequest.from_payload(
                request_payload,
                execution_mode=str(self.get_parameter("execution_mode").value),
            )
        )
        if result.get("state") == "dry_run":
            return result
        if not result.get("accepted", False):
            return result
        return None

    def _legacy_blocked_response(self, intent: str, message: str, payload: dict | None = None) -> dict:
        payload = dict(payload or {})
        return self._web_command_gateway.blocked_legacy_response(
            intent=intent,
            message=message,
            execution_mode=str(self.get_parameter("execution_mode").value),
            request_id=str(payload.get("request_id", "") or ""),
            blocked_legacy_execution=True,
        )

    def _teach_trajectory(self, record_path: str | None = None, max_points: int = 500) -> dict:
        path = record_path or str(self._teach_record_info(None).get("path", self.get_parameter("record_path").value))
        try:
            samples = load_teach_samples(path)
        except Exception as exc:
            return {
                "accepted": False,
                "message": f"failed to load teach trajectory: {exc}",
                "path": str(path),
                "points": [],
            }
        prepared = self._prepare_teach_replay_samples(samples)
        prepared_path = write_prepared_teach_record(path, prepared)
        preview_samples = load_teach_samples(prepared_path)
        payload = teach_trajectory_preview_to_dict(preview_samples, max_points=max_points)
        payload["prepared_replay"] = prepared_teach_replay_to_dict(prepared)
        payload["collision_precheck"] = self._collision_precheck(preview_samples)
        payload["accepted"] = True
        payload["curve_source"] = "prepared"
        payload["path"] = str(prepared_path)
        payload["raw_record_path"] = str(path)
        payload["prepared_record_path"] = str(prepared_path)
        payload["info"] = self._teach_record_info(str(path))
        return payload

    def _teach_records(self) -> dict:
        record_path = Path(str(self.get_parameter("record_path").value))
        directory = record_path.parent if str(record_path.parent) else Path("teleop_records")
        records = list_teach_record_files(directory)
        return {
            "directory": str(directory),
            "default_record_path": str(record_path),
            "records": records,
        }

    def _handle_teach_dry_run(self, payload: dict) -> dict:
        record_path = payload.get("record_path")
        info_payload = self._teach_record_info(str(record_path) if record_path else None)
        settings = self._teach_replay_settings_from_payload(
            payload,
            max_error=info_payload.get("max_error"),
        )
        decision = validate_teach_dry_run_request(str(info_payload.get("start_band", "")))
        prepared_payload = {}
        prepared_record_path = ""
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        samples_for_precheck = []
        trajectory_points = 0
        try:
            samples_for_precheck = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(samples_for_precheck, settings)
            prepared_record_path = str(write_prepared_teach_record(str(info_payload.get("path", "")), prepared))
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            moveit_align = self._moveit_align_summary(info_payload, samples_for_precheck, plan=decision.accepted)
            if decision.accepted and str(moveit_align.get("state", "")).lower() not in ("failed", "unavailable", "unknown"):
                trajectory = self._build_teach_replay_trajectory(
                    samples_for_precheck,
                    str(info_payload.get("start_band", "")),
                    settings,
                )
                trajectory_points = len(trajectory.points)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        result = self._teach_replay_coordinator.build_dry_run_result(
            info_payload=info_payload,
            settings=settings,
            decision=decision,
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=trajectory_points,
            limits=self._teach_replay_limits(),
            target_runtime=self._target_runtime(),
            compact_payload=self._compact_replay_payload,
        )
        self._last_teach_dry_run = result if result["accepted"] else None
        self._store.update_teleop_status("replay", result)
        return result

    def _handle_teach_replay_execute(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web teach replay disabled; launch with web_execute_enabled:=true"
            result = self._legacy_blocked_response("teach_replay_execute", message, payload)
            self._store.update_teleop_status("replay", result)
            return result
        gateway_result = self._route_web_command("teach_replay_execute", payload)
        if gateway_result is not None:
            self._store.update_teleop_status("replay", gateway_result)
            return gateway_result
        record_path = payload.get("record_path")
        info_payload = self._teach_record_info(str(record_path) if record_path else None)
        settings = self._teach_replay_settings_from_payload(
            payload,
            max_error=info_payload.get("max_error"),
        )
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        prepared_payload = {}
        prepared_quality = {}
        prepared_record_path = ""
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        trajectory = None
        try:
            source_samples = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(source_samples, settings)
            prepared_record_path = str(write_prepared_teach_record(str(info_payload.get("path", "")), prepared))
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            prepared_quality = prepared_payload.get("after_quality") if isinstance(prepared_payload.get("after_quality"), dict) else {}
            moveit_align = self._moveit_align_summary(info_payload, source_samples, plan=False)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        decision = self._teach_replay_coordinator.evaluate_execute_request(
            info_payload=info_payload,
            settings=settings,
            prepared_quality=prepared_quality,
            dry_run_token=self._last_teach_dry_run or {},
            limits=self._teach_replay_limits(),
            yellow_max_speed=float(self.get_parameter("yellow_max_speed").value),
        )
        moveit_state = str(moveit_align.get("state", "")).lower()
        if decision.accepted and moveit_state in ("failed", "unavailable", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"MoveIt start alignment not ready: {moveit_align.get('message', moveit_state)}",
            )
        if decision.accepted:
            try:
                samples = load_teach_samples(str(info_payload["path"]))
                if not samples:
                    raise ValueError("record contains no samples")
                trajectory = self._build_teach_replay_trajectory(samples, str(info_payload.get("start_band", "")), settings)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
            except Exception as exc:
                collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        precheck_state = str(collision_precheck.get("state", "")).lower()
        if decision.accepted and precheck_state in ("collision", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"collision precheck blocked replay: {collision_precheck.get('message', precheck_state)}",
            )
        if not decision.accepted:
            result = self._teach_replay_coordinator.build_execute_result(
                info_payload=info_payload,
                settings=settings,
                decision=decision,
                prepared_payload=prepared_payload,
                prepared_record_path=prepared_record_path,
                moveit_align=moveit_align,
                collision_precheck=collision_precheck,
                trajectory_points=0,
                limits=self._teach_replay_limits(),
                target_runtime=self._target_runtime(),
                compact_payload=self._compact_replay_payload,
            )
            self._store.update_teleop_status("replay", result)
            return result
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("replay", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}
        if trajectory is None:
            result = {
                "accepted": False,
                "state": "blocked",
                "message": "failed to build replay trajectory",
                "record_path": str(info_payload.get("path", "")),
                "prepared_record_path": prepared_record_path,
                "start_band": str(info_payload.get("start_band", "")),
                "moveit_start_align": moveit_align,
                "collision_precheck": collision_precheck,
                "prepared_replay": prepared_payload,
                "dry_run": False,
            }
            result = self._compact_replay_payload(result)
            self._store.update_teleop_status("replay", result)
            return result
        prepared_payload = getattr(self, "_last_teach_prepared_payload", prepared_payload)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_teach_replay_goal_response(fut, info_payload, len(trajectory.points), trajectory))
        result = self._teach_replay_coordinator.build_execute_result(
            info_payload=info_payload,
            settings=settings,
            decision=decision,
            prepared_payload=prepared_payload,
            prepared_record_path=prepared_record_path,
            moveit_align=moveit_align,
            collision_precheck=collision_precheck,
            trajectory_points=len(trajectory.points),
            limits=self._teach_replay_limits(),
            target_runtime=self._target_runtime(),
            compact_payload=self._compact_replay_payload,
        )
        self._store.update_teleop_status("replay", result)
        return result

    def _handle_teach_replay_stop(self) -> dict:
        gateway_result = self._route_web_command("stop_robot", {"command": "teach_replay_stop"})
        if gateway_result is not None:
            self._store.update_teleop_status("replay", gateway_result)
            return gateway_result
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
        result = self._teach_replay_client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.pop("cancel_future", None)
        if future is not None:
            future.add_done_callback(self._on_teach_replay_cancel_response)
        self._store.update_teleop_status("replay", result)
        return result

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
                return False
            self._trajectory_stop_client.call_async(Trigger.Request())
            return True
        except Exception:
            return False

    def _call_trigger_service(self, client, *, timeout_sec: float) -> tuple[bool, str]:
        return ArmControlClient.call_trigger_service(client, timeout_sec=timeout_sec)

    def _handle_arm_service_command(self, command: str) -> dict:
        command = normalize_arm_command(command) or ""
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web arm command disabled; launch with web_execute_enabled:=true"
            gateway_intent = "move_home" if command == "safe_home" else "stop_robot"
            result = {**self._legacy_blocked_response(gateway_intent, message, {"command": command}), "command": command}
            self._store.update_teleop_status("arm_command", result)
            return result
        replay_state = status_state(self._store.snapshot().teleop.get("replay", {}))
        if arm_command_is_replay_locked(replay_state):
            message = "arm command blocked during teach replay"
            result = {"accepted": False, "state": "blocked", "command": command, "message": message}
            self._store.update_teleop_status("arm_command", result)
            return result
        if not command:
            result = {"accepted": False, "state": "rejected", "command": command, "message": "unknown arm command"}
            self._store.update_teleop_status("arm_command", result)
            return result
        gateway_intent = "move_home" if command == "safe_home" else "stop_robot"
        gateway_result = self._route_web_command(gateway_intent, {"command": command})
        if gateway_result is not None:
            gateway_result = {**gateway_result, "command": command}
            self._store.update_teleop_status("arm_command", gateway_result)
            return gateway_result
        if should_stop_trajectory_before_arm_command(command):
            with self._execute_lock:
                self._execute_goal_handle = None
        result = self._arm_control_client.execute(command)
        result.setdefault("trajectory_stop_requested", False)
        if result["accepted"]:
            arm = self._store.snapshot().arm
            enabled = bool(arm.get("enabled", False))
            mode = str(arm.get("mode", ""))
            if command == "disable":
                enabled = False
            elif command in ("enable", "safe_home"):
                enabled = True
            if command == "safe_home":
                mode = mode or "pos_vel"
            self._store.update_arm_status(
                mode=mode,
                enabled=enabled,
                state_machine="IDLE",
                error_codes=tuple(str(v) for v in arm.get("error_codes", [])),
            )
        self._store.update_teleop_status("arm_command", result)
        return result

    def _handle_teach_record_start(self, payload: dict | None = None) -> dict:
        result = self._teach_record_client.start(payload)
        self._store.update_teleop_status("recording", result)
        return result

    def _handle_teach_record_stop(self) -> dict:
        result = self._teach_record_client.stop()
        self._store.update_teleop_status("recording", result)
        return result

    def _auto_align_duration_from_error(self, max_error: float | None) -> float:
        return float(self._teach_replay_settings_provider.auto_align_duration(max_error))

    def _teach_replay_settings_from_payload(
        self,
        payload: dict,
        *,
        max_error: float | None = None,
    ) -> dict[str, float | int]:
        return self._teach_replay_settings_provider.from_payload(payload, max_error=max_error)

    def _build_teach_replay_trajectory(self, samples, start_band: str, settings: dict[str, float | int]) -> JointTrajectory:
        prepared = self._prepare_teach_replay_samples(samples, settings)
        self._last_teach_prepared_payload = prepared_teach_replay_to_dict(prepared)
        first = prepared.samples[0]
        snapshot = self._store.snapshot()
        current_map = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        result = self._teach_replay_trajectory_builder.build(
            prepared=prepared,
            current_positions=current_map,
            start_band=start_band,
            settings=settings,
            config=TeachReplayTrajectoryConfig(
                use_moveit_start_align=bool(self.get_parameter("use_moveit_start_align").value),
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                soft_start_duration=float(self.get_parameter("soft_start_duration").value),
                soft_start_steps=int(self.get_parameter("soft_start_steps").value),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
                yellow_max_speed=float(self.get_parameter("yellow_max_speed").value),
                initial_replay_delay_sec=float(self.get_parameter("initial_replay_delay_sec").value),
                max_velocity_rad_s=self._max_replay_velocity_limits(tuple(first.joint_names)),
                max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            ),
            moveit_start_alignment=self._append_moveit_start_alignment,
        )
        return result.trajectory

    def _append_final_hold(self, trajectory: JointTrajectory, *, final_hold_sec: float) -> None:
        self._teach_replay_trajectory_builder.append_final_hold(
            trajectory,
            final_hold_sec=final_hold_sec,
        )

    def _append_moveit_start_alignment(
        self,
        trajectory: JointTrajectory,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        return self._moveit_start_aligner.append(
            trajectory,
            current_positions=current_positions,
            first_positions=first_positions,
            config=MoveItStartAlignmentConfig(
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
                skip_threshold=float(self.get_parameter("moveit_start_skip_threshold").value),
                joint_goal_tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            ),
        )

    def _on_teach_replay_goal_response(self, future, info_payload: dict, points: int, trajectory: JointTrajectory) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("replay", {"state": "rejected", "message": "teach replay goal rejected"})
            return
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = goal_handle
            self._active_teach_replay_trajectory = trajectory
            self._active_teach_replay_started_at = time.monotonic()
            self._replay_runtime_monitor.reset()
        self._store.update_teleop_status(
            "replay",
            {
                "state": "replaying",
                "message": "teach replay goal accepted",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": {
                    "enabled": bool(self.get_parameter("replay_monitor_enabled").value),
                    "max_tracking_error_rad": float(self.get_parameter("max_tracking_error_rad").value),
                    "max_live_velocity_rad_s": float(self.get_parameter("max_live_velocity_rad_s").value),
                },
                "dry_run": False,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_teach_replay_result(fut, info_payload, points))

    def _on_teach_replay_cancel_response(self, future) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "teach replay cancel accepted"
            if goals_canceling
            else "teach replay already finished before cancel"
        )
        self._store.update_teleop_status("replay", {"state": state, "message": message})
        if not goals_canceling:
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._replay_runtime_monitor.reset()

    def _on_teach_replay_result(self, future, info_payload: dict, points: int) -> None:
        previous_replay = self._store.snapshot().teleop.get("replay", {})
        with self._teach_replay_lock:
            monitor_stop_requested = self._replay_runtime_monitor.stop_requested
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._replay_runtime_monitor.reset()
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "safety_stop" if monitor_stop_requested else "canceled"
        else:
            state = "failed"
        message = f"teach replay result status={status}, error_code={error_code}: {error_string}"
        runtime_monitor = previous_replay.get("runtime_monitor") if isinstance(previous_replay, dict) else None
        if status == 5 and monitor_stop_requested:
            previous_message = str(previous_replay.get("message", "")) if isinstance(previous_replay, dict) else ""
            message = (
                f"action canceled after runtime monitor stop: {previous_message}"
                if previous_message
                else "action canceled after runtime monitor stop"
            )
        self._store.update_teleop_status(
            "replay",
            {
                "state": state,
                "message": message,
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": runtime_monitor,
                "dry_run": False,
            },
        )
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = None
            self._active_teach_replay_trajectory = None
            self._active_teach_replay_started_at = None
            self._replay_runtime_monitor.reset()

    def _check_active_replay_tracking(self) -> None:
        snapshot = self._store.snapshot()
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
            trajectory = self._active_teach_replay_trajectory
            started_at = self._active_teach_replay_started_at
        if goal_handle is None:
            return
        decision = self._replay_runtime_monitor.check(
            trajectory=trajectory,
            started_at=started_at,
            joints=snapshot.joints,
            now=time.monotonic(),
            config=ReplayRuntimeMonitorConfig(
                enabled=bool(self.get_parameter("replay_monitor_enabled").value),
                start_grace_sec=float(self.get_parameter("replay_monitor_start_grace_sec").value),
                violation_grace_sec=float(self.get_parameter("replay_monitor_violation_grace_sec").value),
                max_tracking_error_rad=float(self.get_parameter("max_tracking_error_rad").value),
                max_live_velocity_rad_s=float(self.get_parameter("max_live_velocity_rad_s").value),
            ),
        )
        if not decision.should_stop:
            return
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        with suppress(Exception):
            goal_handle.cancel_goal_async()
        self._store.update_teleop_status("replay", decision.status)

    def _handle_keyboard_enable(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web keyboard disabled; launch with web_execute_enabled:=true"
            result = self._legacy_blocked_response("keyboard_step", message, payload)
            self._store.update_teleop_status("status", {**result, "source": "web_keyboard"})
            return result
        if str(self.get_parameter("panel_mode").value).lower() == "check":
            message = "web keyboard blocked: check mode is read-only"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "blocked", "message": message})
            return {"accepted": False, "message": message}
        step = _number_or_default(payload.get("step_rad"), self._web_keyboard_step_rad)
        duration = _number_or_default(payload.get("duration"), self._web_keyboard_duration)
        speed = _number_or_default(payload.get("max_joint_speed_rad_s"), self._web_keyboard_speed)
        with self._web_keyboard_lock:
            self._web_keyboard_step_rad = min(
                max(step, float(self.get_parameter("web_keyboard_min_step_rad").value)),
                float(self.get_parameter("web_keyboard_max_step_rad").value),
            )
            self._web_keyboard_duration = min(
                max(duration, float(self.get_parameter("web_keyboard_min_duration").value)),
                float(self.get_parameter("web_keyboard_max_duration").value),
            )
            self._web_keyboard_speed = min(
                max(speed, 0.05),
                float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            )
            self._web_keyboard_enabled = True
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "ready",
            "message": "web keyboard teleop enabled",
            "step_rad": self._web_keyboard_step_rad,
            "duration": self._web_keyboard_duration,
            "max_joint_speed_rad_s": self._web_keyboard_speed,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_disable(self) -> dict:
        gateway_result = self._route_web_command("stop_robot", {"command": "keyboard_disable"})
        if gateway_result is not None:
            self._store.update_teleop_status("status", {**gateway_result, "source": "web_keyboard"})
            return gateway_result
        with self._web_keyboard_lock:
            self._web_keyboard_enabled = False
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.2)
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "disabled",
            "message": "web keyboard teleop disabled; trajectory_stop requested" if stop_requested else "web keyboard teleop disabled",
            "trajectory_stop_requested": stop_requested,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_key(self, payload: dict) -> dict:
        with self._web_keyboard_lock:
            enabled = self._web_keyboard_enabled
            step_rad = self._web_keyboard_step_rad
            duration = self._web_keyboard_duration
            speed = self._web_keyboard_speed
        request_payload = dict(payload)
        request_payload.setdefault("step_rad", step_rad)
        request_payload.setdefault("duration", duration)
        request_payload.setdefault("max_joint_speed_rad_s", speed)
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        decision = validate_web_keyboard_command(
            request_payload,
            enabled=enabled,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            default_step_rad=float(self.get_parameter("web_keyboard_default_step_rad").value),
            min_step_rad=float(self.get_parameter("web_keyboard_min_step_rad").value),
            max_step_rad=float(self.get_parameter("web_keyboard_max_step_rad").value),
            default_duration=float(self.get_parameter("web_keyboard_default_duration").value),
            min_duration=float(self.get_parameter("web_keyboard_min_duration").value),
            max_duration=float(self.get_parameter("web_keyboard_max_duration").value),
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status(
                "status",
                {"source": "web_keyboard", "state": "rejected", "message": decision.message, "last_key": payload.get("key")},
            )
            return _keyboard_decision_response(decision)
        gateway_result = self._route_web_command("keyboard_step", request_payload)
        if gateway_result is not None:
            self._store.update_teleop_status("status", {**gateway_result, "source": "web_keyboard", "last_key": payload.get("key")})
            return gateway_result
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "unavailable", "message": message, "last_key": decision.key})
            return {"accepted": False, "message": message}
        trajectory = JointTrajectory()
        trajectory.joint_names = list(decision.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in decision.positions]
        _set_duration(point.time_from_start, decision.duration)
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_keyboard_goal_response(fut, decision))
        result = _keyboard_decision_response(decision)
        self._store.update_teleop_status(
            "status",
            {
                **result,
                "source": "web_keyboard",
                "state": "active",
                "last_key": decision.key,
            },
        )
        return result

    def _on_keyboard_goal_response(self, future, decision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "failed", "message": str(exc), "last_key": decision.key})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "rejected", "message": "keyboard trajectory goal rejected", "last_key": decision.key})
            return
        self._store.update_teleop_status(
            "status",
            {
                "source": "web_keyboard",
                "state": "accepted",
                "message": decision.message,
                "last_key": decision.key,
                "joint_name": decision.joint_name,
                "step_rad": decision.step_rad,
                "duration": decision.duration,
                "max_joint_speed_rad_s": decision.max_joint_speed_rad_s,
            },
        )

    def _handle_execute_preview(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            return self._legacy_blocked_response(
                "move_relative",
                "web execute disabled; launch with web_execute_enabled:=true",
                payload,
            )
        gateway_result = self._route_web_command("move_relative", payload)
        if gateway_result is not None:
            self._store.update_teleop_status("web_execute", gateway_result)
            return gateway_result
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        execution = self._web_teleop_client.execute(
            payload,
            current_positions=current_positions,
            max_delta_rad=float(self.get_parameter("web_execute_max_delta_rad").value),
            min_duration=float(self.get_parameter("web_execute_min_duration").value),
            max_duration=float(self.get_parameter("web_execute_max_duration").value),
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        decision = execution["decision"]
        if not execution["accepted"]:
            self._store.update_teleop_status("web_execute", execution["status"])
            return execution["response"]
        future = execution["goal_future"]
        future.add_done_callback(lambda fut: self._on_execute_goal_response(fut, decision))
        self._store.update_teleop_status("web_execute", execution["status"])
        return decision_response(decision)

    def _handle_stop_execute(self) -> dict:
        gateway_result = self._route_web_command("stop_robot", {"command": "stop_execute"})
        if gateway_result is not None:
            self._store.update_teleop_status("web_execute", gateway_result)
            return gateway_result
        with self._execute_lock:
            goal_handle = self._execute_goal_handle
        result = self._web_teleop_client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.get("cancel_future")
        if future is not None:
            future.add_done_callback(self._on_execute_cancel_response)
        self._store.update_teleop_status("web_execute", result["status"])
        if result.get("clear_goal_handle"):
            with self._execute_lock:
                self._execute_goal_handle = None
        return {
            "accepted": bool(result["accepted"]),
            "state": result.get("state", result["status"].get("state", "")),
            "message": str(result.get("message", "")),
            "trajectory_stop_requested": bool(result.get("trajectory_stop_requested", False)),
        }

    def _handle_set_gripper(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            return self._legacy_blocked_response(
                "set_gripper",
                "web gripper disabled; launch with web_execute_enabled:=true",
                payload,
            )
        gateway_result = self._route_web_command("set_gripper", payload)
        if gateway_result is not None:
            self._store.update_teleop_status("web_gripper", gateway_result)
            return gateway_result
        result = self._web_teleop_client.set_gripper(
            payload,
            use_hardware=self._use_hardware,
            gripper_limits=self._gripper_limits,
            default_max_effort=float(self.get_parameter("web_gripper_max_effort").value),
            max_effort_limit=float(self.get_parameter("web_gripper_max_effort_limit").value),
        )
        decision = result["decision"]
        if not decision.accepted:
            self._store.update_teleop_status("web_gripper", result["status"])
            return result["response"]
        if result.get("simulated_position") is not None:
            self._sim_gripper_position = float(result["simulated_position"])
            self._publish_simulated_gripper_state()
            self._store.update_teleop_status("web_gripper", result["status"])
            return gripper_decision_response(decision)
        if not result["accepted"]:
            self._store.update_teleop_status("web_gripper", result["status"])
            return result["response"]
        future = result["goal_future"]
        future.add_done_callback(lambda fut: self._on_gripper_goal_response(fut, decision))
        self._store.update_teleop_status("web_gripper", result["status"])
        return gripper_decision_response(decision)

    def _on_gripper_goal_response(self, future, decision: WebGripperDecision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_gripper",
                {"state": "rejected", "message": "gripper goal rejected"},
            )
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "accepted",
                "message": "gripper goal accepted by controller",
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_gripper_result(fut, decision))

    def _on_gripper_result(self, future, decision: WebGripperDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            reached = bool(getattr(result, "reached_goal", False))
            position = float(getattr(result, "position", decision.position))
            effort = float(getattr(result, "effort", 0.0))
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "done" if reached else "failed",
                "message": f"gripper result reached={reached}",
                "position": position,
                "max_effort": decision.max_effort,
                "effort": effort,
            },
        )

    def _publish_simulated_gripper_state(self) -> None:
        if self._sim_gripper_state_pub is None:
            return
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(self._sim_gripper_position)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 1
        self._sim_gripper_state_pub.publish(msg)

    def _on_execute_cancel_response(self, future) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "trajectory cancel accepted"
            if goals_canceling
            else "trajectory already finished before cancel"
        )
        self._store.update_teleop_status("web_execute", {"state": state, "message": message})

    def _on_execute_goal_response(self, future, decision: WebExecuteDecision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_execute",
                {"state": "rejected", "message": "trajectory goal rejected"},
            )
            return
        with self._execute_lock:
            self._execute_goal_handle = goal_handle
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "accepted",
                "message": "trajectory goal accepted by controller",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_execute_result(fut, decision))

    def _on_execute_result(self, future, decision: WebExecuteDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            status = int(result_response.status)
            error_code = int(getattr(result, "error_code", 0))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            with self._execute_lock:
                self._execute_goal_handle = None
            return
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        with self._execute_lock:
            self._execute_goal_handle = None
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": state,
                "message": f"trajectory result status={status}, error_code={error_code}: {error_string}",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._store.update_joint_state(
            names=tuple(str(v) for v in msg.name),
            positions=tuple(float(v) for v in msg.position),
            velocities=tuple(float(v) for v in msg.velocity),
            efforts=tuple(float(v) for v in msg.effort),
        )

    def _on_motor_state(self, msg: JointMotorState) -> None:
        self._store.update_motor_state(
            joint_name=str(msg.joint_name),
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )

    def _on_gripper_state(self, msg: JointMotorState) -> None:
        joint_name = str(msg.joint_name).strip() or "gripper"
        self._store.update_motor_state(
            joint_name=joint_name,
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )
        if joint_name != "gripper":
            self._store.update_motor_state(
                joint_name="gripper",
                position=float(msg.position),
                velocity=float(msg.velocity),
                torque=float(msg.torque),
                status_code=int(msg.status_code),
            )

    def _on_arm_status(self, msg: ArmStatus) -> None:
        self._store.update_arm_status(
            mode=str(msg.mode),
            enabled=bool(msg.enabled),
            state_machine=str(msg.state_machine),
            error_codes=tuple(str(v) for v in msg.error_codes),
        )
        self._update_gravity_comp_status()

    def _on_status(self, key: str, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except Exception:
            value = msg.data
        self._store.update_teleop_status(key, value)

    def _update_gravity_comp_status(self) -> None:
        snapshot = self._store.snapshot()
        arm_state = str(snapshot.arm.get("state_machine", ""))
        try:
            start_available = bool(self._gravity_start_client.service_is_ready())
            if not start_available:
                start_available = bool(self._gravity_start_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            start_available = False
        try:
            stop_available = bool(self._gravity_stop_client.service_is_ready())
            if not stop_available:
                stop_available = bool(self._gravity_stop_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            stop_available = False
        active = arm_state == "GRAVITY_COMP"
        if active:
            state = "active"
            message = "gravity compensation is active"
        elif start_available:
            state = "ready"
            message = "gravity compensation start service is available"
        else:
            state = "unavailable"
            message = "gravity compensation services unavailable"
        recording = snapshot.teleop.get("recording")
        require_gravity = bool(recording.get("require_gravity_comp")) if isinstance(recording, dict) else True
        self._store.update_teleop_status(
            "gravity_comp",
            {
                "state": state,
                "message": message,
                "arm_state": arm_state,
                "start_service_available": start_available,
                "stop_service_available": stop_available,
                "active": active,
                "ready_for_teach_recording": (not require_gravity) or active,
                "recording_requires_gravity_comp": require_gravity,
            },
        )

    def destroy_node(self) -> bool:
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopStatusPanelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()


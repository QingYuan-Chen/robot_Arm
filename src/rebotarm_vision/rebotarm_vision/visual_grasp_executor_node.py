from __future__ import annotations

from copy import deepcopy
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan
from rebotarm_msgs.srv import ExecutePose, GraspGripper, SetGripper

from .grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices
from .grasp_verification_policy import (
    GraspVerificationConfig,
    GraspVerificationInput,
    verify_grasp_after_lift,
)
from .grasp_preview_sender_node import (
    _transform_from_msg,
    apply_tcp_offset_to_pose,
    transform_pose_message,
)
from .gripper_quality import close_contact_success
from .gripper_policy import GripperPolicyConfig, resolve_gripper_command
from .place_task_policy import PlaceTaskConfig, build_place_stages
from .retreat_policy import RetreatPolicyConfig
from .trajectory_recovery_policy import RecoveryConfig, recovery_decision_for_stage
from .visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets
from .visual_grasp_sequence import (
    PoseTarget,
    VisualGraspSequenceConfig,
    VisualGraspStage,
    build_visual_grasp_sequence,
)
from .visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step


def pose_to_target(pose: Pose) -> PoseTarget:
    return PoseTarget(
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def target_to_pose_stamped(target: PoseTarget, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = float(target.position[0])
    msg.pose.position.y = float(target.position[1])
    msg.pose.position.z = float(target.position[2])
    msg.pose.orientation.x = float(target.orientation[0])
    msg.pose.orientation.y = float(target.orientation[1])
    msg.pose.orientation.z = float(target.orientation[2])
    msg.pose.orientation.w = float(target.orientation[3])
    return msg


class VisualGraspExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_visual_grasp_executor")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("input_topic", "/grasp/filtered_plan")
        self.declare_parameter("candidates_topic", "/grasp/filtered_candidates")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pregrasp_base_z_offset_m", 0.05)
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        self.declare_parameter("pose_policy", "base_axis")
        self.declare_parameter("fixed_grasp_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        self.declare_parameter("min_grasp_z_m", 0.0)
        self.declare_parameter("lift_z_m", 0.08)
        self.declare_parameter("open_before_approach", False)
        self.declare_parameter("open_position_m", 0.09)
        self.declare_parameter("close_position_m", 0.025)
        self.declare_parameter("close_max_effort", 0.4)
        self.declare_parameter("auto_gripper_width", True)
        self.declare_parameter("open_clearance_m", 0.0)
        self.declare_parameter("close_margin_m", 0.012)
        self.declare_parameter("min_open_position_m", 0.035)
        self.declare_parameter("max_open_position_m", 0.09)
        self.declare_parameter("min_close_position_m", 0.006)
        self.declare_parameter("max_close_position_m", 0.08)
        self.declare_parameter("auto_gripper_effort", True)
        self.declare_parameter("min_gripper_effort", 0.22)
        self.declare_parameter("max_gripper_effort", 0.60)
        self.declare_parameter("max_allowed_grasp_width_m", 0.082)
        self.declare_parameter("close_contact_success_enabled", True)
        self.declare_parameter("close_contact_margin_m", 0.004)
        self.declare_parameter("close_contact_min_closure_delta_m", 0.015)
        self.declare_parameter("gripper_grasp_enabled", True)
        self.declare_parameter("gripper_grasp_close_force", 0.4)
        self.declare_parameter("gripper_grasp_timeout_sec", 8.0)
        self.declare_parameter("gripper_grasp_min_close_time_sec", 0.08)
        self.declare_parameter("gripper_grasp_velocity_threshold", 0.04)
        self.declare_parameter("gripper_grasp_min_closure_distance_m", 0.006)
        self.declare_parameter("safe_retreat_enabled", True)
        self.declare_parameter("safe_retreat_min_lift_z_m", 0.12)
        self.declare_parameter("safe_retreat_distance_m", 0.06)
        self.declare_parameter("safe_retreat_axis_xyz", [-1.0, 0.0, 0.5])
        self.declare_parameter("safe_home_after_grasp", False)
        self.declare_parameter("service_timeout_sec", 20.0)
        self.declare_parameter("pregrasp_wait_sec", 0.5)
        self.declare_parameter("approach_wait_sec", 0.2)
        self.declare_parameter("execute_gripper", True)
        self.declare_parameter("gripper_wait_sec", 1.0)
        self.declare_parameter("lift_wait_sec", 0.5)
        self.declare_parameter("motion_result_timeout_sec", 45.0)
        self.declare_parameter("execution_mode", "plan_only")
        self.declare_parameter("move_velocity_scaling", 0.10)
        self.declare_parameter("approach_velocity_scaling", 0.04)
        self.declare_parameter("lift_velocity_scaling", 0.08)
        self.declare_parameter("acceleration_scaling", 0.08)
        self.declare_parameter("plan_only_stage_pause_sec", 3.0)
        self.declare_parameter("refresh_plan_at_pregrasp_enabled", True)
        self.declare_parameter("refresh_plan_at_pregrasp_required", True)
        self.declare_parameter("refresh_plan_timeout_sec", 1.0)
        self.declare_parameter("approach_visual_servo_enabled", False)
        self.declare_parameter("approach_visual_servo_max_iterations", 5)
        self.declare_parameter("approach_visual_servo_max_step_m", 0.02)
        self.declare_parameter("approach_visual_servo_position_tolerance_m", 0.008)
        self.declare_parameter("approach_visual_servo_require_fresh_plan", True)
        self.declare_parameter("auto_retry_enabled", False)
        self.declare_parameter("auto_retry_max_attempts", 3)
        self.declare_parameter("safe_retreat_before_retry", True)
        self.declare_parameter("grasp_verification_enabled", True)
        self.declare_parameter("grasp_verification_min_closure_distance_m", 0.006)
        self.declare_parameter("grasp_verification_require_contact", True)
        self.declare_parameter("visual_lift_check_enabled", False)
        self.declare_parameter("visual_lift_min_delta_m", 0.03)
        self.declare_parameter("place_after_grasp_enabled", False)
        self.declare_parameter("place_position_xyz", [0.20, -0.20, 0.25])
        self.declare_parameter("place_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("place_open_position_m", 0.08)
        self.declare_parameter("place_open_max_effort", 0.25)
        self.declare_parameter("place_retreat_z_m", 0.06)
        self.declare_parameter("trajectory_precheck_enabled", True)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._candidates_topic = str(self.get_parameter("candidates_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value).strip()
        self._tcp_offset_xyz = self._tuple3("tcp_offset_xyz")
        self._target_base_offset_xyz = self._tuple3("target_base_offset_xyz")
        self._pregrasp_base_z_offset_m = float(self.get_parameter("pregrasp_base_z_offset_m").value)
        self._grasp_base_z_offset_m = float(self.get_parameter("grasp_base_z_offset_m").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self._motion_result_timeout_sec = float(self.get_parameter("motion_result_timeout_sec").value)
        self._execution_mode = str(self.get_parameter("execution_mode").value).strip().lower()
        self._stage_waits = {
            "move_to_pregrasp": float(self.get_parameter("pregrasp_wait_sec").value),
            "approach_grasp": float(self.get_parameter("approach_wait_sec").value),
            "close_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "open_gripper": float(self.get_parameter("gripper_wait_sec").value),
            "lift": float(self.get_parameter("lift_wait_sec").value),
            "safe_retreat": float(self.get_parameter("lift_wait_sec").value),
        }

        self._latest_plan: GraspPlan | None = None
        self._latest_candidates: GraspCandidateArray | None = None
        self._plan_revision = 0
        self._last_gripper_reached_position: float | None = None
        self._last_grasp_contact_detected = False
        self._last_grasp_closure_distance_m = 0.0
        self._last_lift_start_z_m: float | None = None
        self._retry_retreat_stage: VisualGraspStage | None = None
        self._run_counter = 0
        self._current_run_id = 0
        self._current_attempt_index = 0
        self._current_candidate_index = -1
        self._current_attempt_plan: GraspPlan | None = None
        self._running = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._execute_pose_client = self.create_client(
            ExecutePose,
            f"/{self._arm_namespace}/motion_execution/execute_pose",
            callback_group=self._callback_group,
        )
        self._motion_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            callback_group=self._callback_group,
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        self._safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
            callback_group=self._callback_group,
        )
        self._gripper_client = self.create_client(
            SetGripper,
            f"/{self._arm_namespace}/gripper/set",
            callback_group=self._callback_group,
        )
        self._gripper_grasp_client = self.create_client(
            GraspGripper,
            f"/{self._arm_namespace}/gripper/grasp",
            callback_group=self._callback_group,
        )
        self.create_subscription(GraspPlan, self._input_topic, self._on_plan, 10, callback_group=self._callback_group)
        self.create_subscription(
            GraspCandidateArray,
            self._candidates_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/execute", self._execute_visual_grasp, callback_group=self._callback_group)
        self.create_service(Trigger, f"/{self._arm_namespace}/visual_grasp/stop", self._stop_visual_grasp, callback_group=self._callback_group)
        self.get_logger().info(
            "visual grasp executor ready: "
            f"input={self._input_topic}, namespace=/{self._arm_namespace}, target_frame={self._target_frame}, "
            "motion_execution=/motion_execution/execute_pose"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = self._tuple_n(name, 3)
        return (values[0], values[1], values[2])

    def _tuple_n(self, name: str, expected_len: int) -> tuple[float, ...]:
        values = list(self.get_parameter(name).value)
        if len(values) != expected_len:
            raise ValueError(f"{name} must contain exactly {expected_len} values")
        return tuple(float(value) for value in values)

    def _on_plan(self, plan: GraspPlan) -> None:
        if plan.valid:
            self._latest_plan = deepcopy(plan)
            self._plan_revision += 1

    def _on_candidates(self, candidates: GraspCandidateArray) -> None:
        if candidates.candidates:
            self._latest_candidates = deepcopy(candidates)

    def _execute_visual_grasp(self, _request, response):
        if self._running:
            response.success = False
            response.message = "visual grasp already running"
            return response
        if self._latest_plan is None:
            response.success = False
            response.message = "no valid grasp plan received"
            return response
        self._running = True
        self._run_counter += 1
        self._current_run_id = self._run_counter
        try:
            self._log_diagnostic("detect", "ok", f"input_topic={self._input_topic}, plan_revision={self._plan_revision}")
            attempts = self._candidate_plans_for_attempts()
            if not attempts:
                response.success = False
                response.message = "no candidate attempts available"
                self._log_diagnostic("filter", "fail", response.message)
                return response
            self._log_diagnostic("filter", "ok", f"attempts={len(attempts)}")
            for attempt_index, (candidate_index, plan) in enumerate(attempts):
                self._current_attempt_index = attempt_index + 1
                self._current_candidate_index = int(candidate_index)
                self._current_attempt_plan = deepcopy(plan)
                self.get_logger().info(
                    f"{self._diagnostic_prefix('attempt')} start: "
                    f"attempt={attempt_index + 1}/{len(attempts)}, candidate={candidate_index}"
                )
                self._log_plan_snapshot(plan)
                self._last_grasp_contact_detected = False
                self._last_grasp_closure_distance_m = 0.0
                self._last_lift_start_z_m = None
                self._retry_retreat_stage = None
                stages = self._append_place_stages(self._build_sequence_from_plan(plan))
                ok, message, failed_stage = self._execute_stages(stages)
                if ok:
                    response.success = True
                    response.message = "visual grasp sequence finished"
                    self._log_diagnostic("result", "success", response.message)
                    return response
                remaining_attempts = len(attempts) - attempt_index - 1
                decision = recovery_decision_for_stage(
                    failed_stage,
                    attempt_index=attempt_index,
                    remaining_attempts=remaining_attempts,
                    config=RecoveryConfig(
                        auto_retry_enabled=bool(self.get_parameter("auto_retry_enabled").value),
                        safe_retreat_before_retry=bool(self.get_parameter("safe_retreat_before_retry").value),
                    ),
                )
                if decision.request_stop:
                    self._request_stop()
                if decision.request_safe_retreat:
                    self._request_retry_retreat()
                if decision.retry:
                    self.get_logger().warn(f"{decision.reason}: {message}")
                    continue
                response.success = False
                response.message = f"{failed_stage} failed: {message}"
                self._log_failure_snapshot(failed_stage, message)
                return response
            response.success = True
            response.message = "visual grasp sequence finished"
            self._log_diagnostic("result", "success", response.message)
            return response
        except Exception as exc:
            self._request_stop()
            response.success = False
            response.message = f"visual grasp failed: {exc}"
            self._log_failure_snapshot("executor", str(exc))
            return response
        finally:
            self._current_attempt_plan = None
            self._running = False

    def _stop_visual_grasp(self, _request, response):
        self._running = False
        self._request_stop()
        response.success = True
        response.message = "visual grasp stop requested"
        return response

    def _build_sequence_from_plan(self, plan: GraspPlan) -> list[VisualGraspStage]:
        pregrasp, grasp = self._build_motion_targets(plan)
        gripper_command = resolve_gripper_command(
            jaw_width_m=self._detected_jaw_width(plan),
            object_length_m=self._detected_object_length(plan),
            class_name=str(getattr(plan.candidate, "class_name", "") or ""),
            config=GripperPolicyConfig(
                auto_width=bool(self.get_parameter("auto_gripper_width").value),
                auto_effort=bool(self.get_parameter("auto_gripper_effort").value),
                default_open_width_m=float(self.get_parameter("open_position_m").value),
                default_close_width_m=float(self.get_parameter("close_position_m").value),
                default_max_effort=float(self.get_parameter("close_max_effort").value),
                open_clearance_m=float(self.get_parameter("open_clearance_m").value),
                close_margin_m=float(self.get_parameter("close_margin_m").value),
                min_open_width_m=float(self.get_parameter("min_open_position_m").value),
                max_open_width_m=float(self.get_parameter("max_open_position_m").value),
                min_close_width_m=float(self.get_parameter("min_close_position_m").value),
                max_close_width_m=float(self.get_parameter("max_close_position_m").value),
                min_effort=float(self.get_parameter("min_gripper_effort").value),
                max_effort=float(self.get_parameter("max_gripper_effort").value),
                max_allowed_width_m=float(self.get_parameter("max_allowed_grasp_width_m").value),
            ),
        )
        config = VisualGraspSequenceConfig(
            open_before_approach=bool(self.get_parameter("open_before_approach").value),
            open_position_m=float(self.get_parameter("open_position_m").value),
            close_position_m=float(self.get_parameter("close_position_m").value),
            close_max_effort=float(self.get_parameter("close_max_effort").value),
            lift_z_m=float(self.get_parameter("lift_z_m").value),
            min_grasp_z_m=float(self.get_parameter("min_grasp_z_m").value),
            auto_gripper_width=bool(self.get_parameter("auto_gripper_width").value),
            detected_jaw_width_m=self._detected_jaw_width(plan),
            open_clearance_m=float(self.get_parameter("open_clearance_m").value),
            close_margin_m=float(self.get_parameter("close_margin_m").value),
            min_open_position_m=float(self.get_parameter("min_open_position_m").value),
            max_open_position_m=float(self.get_parameter("max_open_position_m").value),
            min_close_position_m=float(self.get_parameter("min_close_position_m").value),
            max_close_position_m=float(self.get_parameter("max_close_position_m").value),
            gripper_command=gripper_command,
            retreat_policy=RetreatPolicyConfig(
                enabled=bool(self.get_parameter("safe_retreat_enabled").value),
                min_lift_z_m=float(self.get_parameter("safe_retreat_min_lift_z_m").value),
                retreat_distance_m=float(self.get_parameter("safe_retreat_distance_m").value),
                retreat_axis_xyz=self._tuple3("safe_retreat_axis_xyz"),
            ),
            include_safe_home=bool(self.get_parameter("safe_home_after_grasp").value),
        )
        return build_visual_grasp_sequence(pregrasp, grasp, config)

    def _candidate_plans_for_attempts(self) -> list[tuple[int, GraspPlan]]:
        if self._latest_plan is None:
            return []
        attempts: list[tuple[int, GraspPlan]] = [(-1, deepcopy(self._latest_plan))]
        candidates = self._latest_candidates
        if candidates is None or not candidates.candidates:
            return attempts
        indices = ordered_candidate_indices(
            candidate_count=len(candidates.candidates),
            best_index=int(candidates.best_index),
            failed_indices=set(),
            config=RetryPolicyConfig(
                enabled=bool(self.get_parameter("auto_retry_enabled").value),
                max_attempts=int(self.get_parameter("auto_retry_max_attempts").value),
            ),
        )
        for index in indices:
            if index == int(candidates.best_index):
                continue
            attempts.append((index, self._plan_from_candidate(candidates, index)))
        return attempts

    def _plan_from_candidate(self, candidates: GraspCandidateArray, index: int) -> GraspPlan:
        candidate = candidates.candidates[int(index)]
        plan = GraspPlan()
        plan.header = candidates.header
        plan.candidate = deepcopy(candidate)
        plan.pregrasp_pose = deepcopy(candidate.pose)
        plan.grasp_pose = deepcopy(candidate.pose)
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.source = "visual_grasp_executor_retry"
        plan.reason = ""
        return plan

    def _execute_stages(self, stages: list[VisualGraspStage]) -> tuple[bool, str, str]:
        stage_index = 0
        while stage_index < len(stages):
            stage = stages[stage_index]
            stage_start_revision = self._plan_revision
            if stage.name == "lift" and stage.pose is not None:
                self._last_lift_start_z_m = float(stage.pose.position[2]) - float(self.get_parameter("lift_z_m").value)
            ok, message = self._run_stage(stage)
            if not ok:
                return False, message, stage.name
            if stage.name == "move_to_pregrasp":
                self._retry_retreat_stage = stage
                refreshed_plan = self._wait_for_refreshed_plan(stage_start_revision)
                if (
                    refreshed_plan is None
                    and bool(self.get_parameter("refresh_plan_at_pregrasp_enabled").value)
                    and bool(self.get_parameter("refresh_plan_at_pregrasp_required").value)
                ):
                    return False, "fresh grasp plan unavailable after pregrasp", stage.name
                if refreshed_plan is not None:
                    refreshed_stages = self._append_place_stages(self._build_sequence_from_plan(refreshed_plan))
                    stages = self._replace_remaining_after_pregrasp(stages, refreshed_stages, stage_index)
                if self._approach_visual_servo_enabled() and stage.pose is not None:
                    ok, message = self._run_visual_servo_approach(stage.pose)
                    if not ok:
                        return False, message, "visual_servo_approach"
                    stages = self._remove_approach_after_pregrasp(stages, stage_index)
            if stage.name == "lift":
                verified, reason = self._verify_after_lift()
                if not verified:
                    return False, reason, stage.name
            stage_index += 1
        return True, "ok", ""

    def _diagnostic_prefix(self, stage: str) -> str:
        return (
            f"[visual_grasp][run={self._current_run_id}]"
            f"[attempt={self._current_attempt_index}]"
            f"[candidate={self._current_candidate_index}]"
            f"[stage={stage}]"
        )

    def _log_diagnostic(self, stage: str, status: str, details: str = "") -> None:
        suffix = f": {details}" if details else ""
        self.get_logger().info(f"{self._diagnostic_prefix(stage)} {status}{suffix}")

    def _log_plan_snapshot(self, plan: GraspPlan) -> None:
        candidate = plan.candidate
        self.get_logger().info(
            f"{self._diagnostic_prefix('plan')} "
            f"valid={bool(plan.valid)}, source={plan.source}, reason={plan.reason}, "
            f"class={getattr(candidate, 'class_name', '')}, confidence={float(getattr(candidate, 'confidence', 0.0)):.3f}, "
            f"jaw_width={float(getattr(plan, 'jaw_width', 0.0) or getattr(candidate, 'jaw_width', 0.0) or 0.0):.4f}"
        )
        self.get_logger().info(f"{self._diagnostic_prefix('pregrasp_pose')} {self._format_pose(plan.pregrasp_pose)}")
        self.get_logger().info(f"{self._diagnostic_prefix('grasp_pose')} {self._format_pose(plan.grasp_pose)}")

    def _log_failure_snapshot(self, failed_stage: str, message: str) -> None:
        self.get_logger().error(f"{self._diagnostic_prefix(failed_stage)} fail: {message}")
        if self._current_attempt_plan is not None:
            self._log_plan_snapshot(self._current_attempt_plan)
        reached = "unknown" if self._last_gripper_reached_position is None else f"{self._last_gripper_reached_position:.4f}"
        self.get_logger().error(
            f"{self._diagnostic_prefix('failure_summary')} "
            f"failed_stage={failed_stage}, message={message}, "
            f"last_gripper_reached_position={reached}, "
            f"contact={self._last_grasp_contact_detected}, "
            f"closure_distance={self._last_grasp_closure_distance_m:.4f}"
        )

    def _format_pose(self, pose: Pose) -> str:
        return (
            "position=("
            f"{float(pose.position.x):.4f}, {float(pose.position.y):.4f}, {float(pose.position.z):.4f}"
            "), orientation=("
            f"{float(pose.orientation.x):.4f}, {float(pose.orientation.y):.4f}, "
            f"{float(pose.orientation.z):.4f}, {float(pose.orientation.w):.4f}"
            ")"
        )

    def _wait_for_refreshed_plan(self, min_revision: int) -> GraspPlan | None:
        if not bool(self.get_parameter("refresh_plan_at_pregrasp_enabled").value):
            return None
        timeout_sec = float(self.get_parameter("refresh_plan_timeout_sec").value)
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and self._running and time.monotonic() < deadline:
            if self._plan_revision > min_revision and self._latest_plan is not None:
                self.get_logger().info(
                    "using refreshed grasp plan after pregrasp: "
                    f"revision={self._plan_revision}, source={self._latest_plan.source}"
                )
                return deepcopy(self._latest_plan)
            time.sleep(0.02)
        return None

    def _replace_remaining_after_pregrasp(
        self,
        current_stages: list[VisualGraspStage],
        refreshed_stages: list[VisualGraspStage],
        completed_pregrasp_index: int,
    ) -> list[VisualGraspStage]:
        refreshed_remaining = self._remaining_stages_after_pregrasp(refreshed_stages)
        if not refreshed_remaining:
            return current_stages
        return current_stages[: completed_pregrasp_index + 1] + refreshed_remaining

    def _remaining_stages_after_pregrasp(self, stages: list[VisualGraspStage]) -> list[VisualGraspStage]:
        for index, stage in enumerate(stages):
            if stage.name == "move_to_pregrasp":
                return stages[index + 1 :]
        return stages

    def _remove_approach_after_pregrasp(
        self,
        stages: list[VisualGraspStage],
        completed_pregrasp_index: int,
    ) -> list[VisualGraspStage]:
        remaining = stages[completed_pregrasp_index + 1 :]
        if remaining and remaining[0].name == "approach_grasp":
            remaining = remaining[1:]
        return stages[: completed_pregrasp_index + 1] + remaining

    def _approach_visual_servo_enabled(self) -> bool:
        return bool(self.get_parameter("approach_visual_servo_enabled").value)

    def _run_visual_servo_approach(self, current: PoseTarget) -> tuple[bool, str]:
        max_iterations = max(1, int(self.get_parameter("approach_visual_servo_max_iterations").value))
        config = VisualServoApproachConfig(
            max_step_m=float(self.get_parameter("approach_visual_servo_max_step_m").value),
            position_tolerance_m=float(self.get_parameter("approach_visual_servo_position_tolerance_m").value),
        )
        last_error = 0.0
        require_fresh_plan = bool(self.get_parameter("approach_visual_servo_require_fresh_plan").value)
        for iteration in range(max_iterations):
            plan_revision = self._plan_revision
            refreshed_plan = self._wait_for_refreshed_plan(plan_revision)
            if require_fresh_plan and refreshed_plan is None:
                return False, "fresh visual servo plan unavailable"
            plan = refreshed_plan if refreshed_plan is not None else self._latest_plan
            if plan is None:
                return False, "no refreshed grasp plan available"
            _, desired_grasp = self._build_motion_targets(plan)
            step = build_visual_servo_step(current, desired_grasp, config)
            last_error = step.error_m
            if step.reached:
                return True, f"target reached: error={step.error_m:.4f}"
            ok, message = self._run_stage(
                VisualGraspStage(name="visual_servo_approach", kind="move", pose=step.target)
            )
            if not ok:
                return False, message
            current = step.target
            self.get_logger().info(
                "visual servo approach step "
                f"{iteration + 1}/{max_iterations}: error={step.error_m:.4f}"
            )
        return False, f"not converged after {max_iterations} steps: error={last_error:.4f}"

    def _verify_after_lift(self) -> tuple[bool, str]:
        if not self._execution_enabled():
            return True, "plan_only: grasp verification skipped"
        visual_delta = 0.0
        visual_available = False
        if bool(self.get_parameter("visual_lift_check_enabled").value) and self._latest_plan is not None:
            try:
                _, latest_grasp = self._build_motion_targets(self._latest_plan)
                if self._last_lift_start_z_m is not None:
                    visual_delta = float(latest_grasp.position[2]) - float(self._last_lift_start_z_m)
                    visual_available = True
            except Exception as exc:
                self.get_logger().warn(f"visual lift verification unavailable: {exc}")
        result = verify_grasp_after_lift(
            GraspVerificationInput(
                gripper_contact_detected=bool(self._last_grasp_contact_detected),
                closure_distance_m=float(self._last_grasp_closure_distance_m),
                visual_lift_delta_m=visual_delta,
                visual_lift_evidence_available=visual_available,
            ),
            GraspVerificationConfig(
                enabled=bool(self.get_parameter("grasp_verification_enabled").value),
                min_closure_distance_m=float(self.get_parameter("grasp_verification_min_closure_distance_m").value),
                require_gripper_contact=bool(self.get_parameter("grasp_verification_require_contact").value),
                visual_lift_check_enabled=bool(self.get_parameter("visual_lift_check_enabled").value),
                min_visual_lift_delta_m=float(self.get_parameter("visual_lift_min_delta_m").value),
            ),
        )
        return bool(result.success), str(result.reason)

    def _append_place_stages(self, stages: list[VisualGraspStage]) -> list[VisualGraspStage]:
        return stages + build_place_stages(
            PlaceTaskConfig(
                enabled=bool(self.get_parameter("place_after_grasp_enabled").value),
                place_position_xyz=self._tuple3("place_position_xyz"),
                place_orientation_xyzw=self._tuple_n("place_orientation_xyzw", 4),
                open_position_m=float(self.get_parameter("place_open_position_m").value),
                open_max_effort=float(self.get_parameter("place_open_max_effort").value),
                retreat_z_m=float(self.get_parameter("place_retreat_z_m").value),
            )
        )

    def _request_retry_retreat(self) -> None:
        if self._retry_retreat_stage is None:
            self.get_logger().warn("safe retreat before retry requested, but no pregrasp retreat stage is available")
            return
        retreat = VisualGraspStage(
            name="retry_safe_retreat",
            kind="move",
            pose=self._retry_retreat_stage.pose,
        )
        ok, message = self._run_stage(retreat)
        if not ok:
            self.get_logger().warn(f"retry safe retreat failed: {message}")


    def _build_motion_targets(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        if str(getattr(plan, "source", "")).strip() == "candidate_ik_filter":
            return self._build_motion_targets_from_filtered_plan(plan)
        policy = str(self.get_parameter("pose_policy").value).strip().lower()
        if policy in ("visual_pose", "source_pose", "legacy"):
            return (
                self._convert_plan_pose(plan, plan.pregrasp_pose, self._pregrasp_base_z_offset_m),
                self._convert_plan_pose(plan, plan.grasp_pose, self._grasp_base_z_offset_m),
            )
        if policy != "base_axis":
            raise ValueError(f"unsupported pose_policy: {policy}")
        grasp_pose = self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)
        return build_base_axis_grasp_targets(
            grasp_position_xyz=(
                float(grasp_pose.position.x),
                float(grasp_pose.position.y),
                float(grasp_pose.position.z),
            ),
            config=BaseAxisGraspPolicyConfig(
                fixed_orientation_xyzw=self._tuple_n("fixed_grasp_orientation_xyzw", 4),
                approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tcp_offset_xyz,
                target_base_offset_xyz=self._target_base_offset_xyz,
                pregrasp_z_offset_m=self._pregrasp_base_z_offset_m,
                grasp_z_offset_m=self._grasp_base_z_offset_m,
            ),
        )

    def _build_motion_targets_from_filtered_plan(self, plan: GraspPlan) -> tuple[PoseTarget, PoseTarget]:
        return (
            pose_to_target(self._transform_plan_pose_to_target_frame(plan, plan.pregrasp_pose)),
            pose_to_target(self._transform_plan_pose_to_target_frame(plan, plan.grasp_pose)),
        )

    def _transform_plan_pose_to_target_frame(self, plan: GraspPlan, pose: Pose) -> Pose:
        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            converted = transform_pose_message(converted, _transform_from_msg(tf_msg))
        return converted

    def _convert_plan_pose(self, plan: GraspPlan, pose: Pose, base_z_offset_m: float) -> PoseTarget:
        converted = deepcopy(pose)
        source_frame = str(plan.header.frame_id)
        if self._target_frame and source_frame and source_frame != self._target_frame:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            converted = transform_pose_message(converted, _transform_from_msg(tf_msg))
        converted = apply_tcp_offset_to_pose(converted, self._tcp_offset_xyz)
        converted.position.x = round(float(converted.position.x) + self._target_base_offset_xyz[0], 6)
        converted.position.y = round(float(converted.position.y) + self._target_base_offset_xyz[1], 6)
        converted.position.z = round(float(converted.position.z) + self._target_base_offset_xyz[2] + base_z_offset_m, 6)
        return pose_to_target(converted)

    def _run_stage(self, stage: VisualGraspStage) -> tuple[bool, str]:
        self._log_diagnostic(stage.name, "start")
        if stage.kind == "move":
            if stage.pose is None:
                return False, "missing move pose"
            ok, message = self._call_execute_pose(stage)
        elif stage.kind == "gripper":
            if not self._execution_enabled():
                ok, message = True, "plan_only: gripper command skipped"
            elif self._gripper_execution_enabled():
                if stage.name == "close_gripper" and bool(self.get_parameter("gripper_grasp_enabled").value):
                    ok, message = self._call_grasp_gripper(stage)
                else:
                    ok, message = self._call_gripper(stage)
            else:
                ok, message = True, "simulation: gripper command skipped"
        elif stage.kind == "safe_home":
            ok, message = self._call_safe_home()
        else:
            return False, f"unsupported stage kind: {stage.kind}"
        if not ok:
            self._log_diagnostic(stage.name, "fail", message)
            return False, message
        wait_sec = self._stage_waits.get(stage.name, 0.0)
        if not self._execution_enabled() and stage.kind == "move":
            wait_sec = max(wait_sec, float(self.get_parameter("plan_only_stage_pause_sec").value))
        time.sleep(max(0.0, wait_sec))
        if not self._running:
            return False, "stopped"
        self._log_diagnostic(stage.name, "ok", message)
        return True, message

    def _call_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        if stage.pose is None:
            return False, "missing move pose"
        if not self._execute_pose_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "motion execution service unavailable"
        if self._execution_enabled() and bool(self.get_parameter("trajectory_precheck_enabled").value):
            ok, message = self._precheck_execute_pose(stage)
            if not ok:
                return False, f"trajectory precheck failed: {message}"
        return self._send_execute_pose(stage, execute=self._execution_enabled())

    def _precheck_execute_pose(self, stage: VisualGraspStage) -> tuple[bool, str]:
        return self._send_execute_pose(stage, execute=False)

    def _send_execute_pose(self, stage: VisualGraspStage, *, execute: bool) -> tuple[bool, str]:
        if stage.pose is None:
            return False, "missing move pose"
        request = ExecutePose.Request()
        request.target_pose = target_to_pose_stamped(stage.pose, self._target_frame)
        request.velocity_scaling = self._velocity_scaling_for_stage(stage.name)
        request.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        request.timeout_sec = self._motion_result_timeout_sec
        request.execute = bool(execute)
        future = self._execute_pose_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec + self._motion_result_timeout_sec):
            return False, "motion execution service call timed out"
        result = future.result()
        if result is None:
            return False, "motion execution returned no result"
        return bool(result.success), f"{result.stage}: {result.message}"

    def _execution_enabled(self) -> bool:
        return self._execution_mode in ("execute", "real")

    def _gripper_execution_enabled(self) -> bool:
        return self._execution_enabled() and bool(self.get_parameter("execute_gripper").value)

    def _detected_jaw_width(self, plan: GraspPlan) -> float:
        plan_width = float(getattr(plan, "jaw_width", 0.0) or 0.0)
        candidate_width = float(getattr(plan.candidate, "jaw_width", 0.0) or 0.0)
        return plan_width if plan_width > 0.0 else candidate_width

    def _detected_object_length(self, plan: GraspPlan) -> float:
        return float(getattr(plan.candidate, "object_length", 0.0) or 0.0)

    def _velocity_scaling_for_stage(self, name: str) -> float:
        if name == "visual_servo_approach":
            return float(self.get_parameter("approach_velocity_scaling").value)
        if name == "approach_grasp":
            return float(self.get_parameter("approach_velocity_scaling").value)
        if name == "lift":
            return float(self.get_parameter("lift_velocity_scaling").value)
        if name == "safe_retreat":
            return float(self.get_parameter("lift_velocity_scaling").value)
        return float(self.get_parameter("move_velocity_scaling").value)

    def _call_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper target"
        if not self._gripper_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "gripper service unavailable"
        request = SetGripper.Request()
        request.position = float(stage.gripper_position_m)
        request.max_effort = float(stage.gripper_max_effort)
        future = self._gripper_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec):
            return False, "gripper service call timed out"
        result = future.result()
        if result is None:
            return False, "gripper service returned no result"
        reached_position = float(result.reached_position)
        command_success = bool(result.success)
        if stage.name == "open_gripper" and command_success:
            self._last_gripper_reached_position = reached_position
        if stage.name == "open_gripper_at_place" and command_success:
            self._last_gripper_reached_position = reached_position
        if stage.name == "close_gripper" and not command_success:
            contact_ok = bool(self.get_parameter("close_contact_success_enabled").value) and close_contact_success(
                command_success=command_success,
                target_position_m=float(stage.gripper_position_m),
                reached_position_m=reached_position,
                previous_open_position_m=self._last_gripper_reached_position,
                contact_margin_m=float(self.get_parameter("close_contact_margin_m").value),
                min_closure_delta_m=float(self.get_parameter("close_contact_min_closure_delta_m").value),
            )
            if contact_ok:
                return True, (
                    f"contact assumed: reached_position={reached_position:.4f}, "
                    f"target={float(stage.gripper_position_m):.4f}"
                )
        return command_success, f"reached_position={reached_position:.4f}"

    def _call_safe_home(self) -> tuple[bool, str]:
        if not self._execution_enabled():
            return True, "plan_only: safe_home skipped"
        if not self._safe_home_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "safe_home service unavailable"
        future = self._safe_home_client.call_async(Trigger.Request())
        if not self._wait_for_future(future, self._service_timeout_sec + self._motion_result_timeout_sec):
            return False, "safe_home service call timed out"
        result = future.result()
        if result is None:
            return False, "safe_home returned no result"
        return bool(result.success), str(result.message)

    def _call_grasp_gripper(self, stage: VisualGraspStage) -> tuple[bool, str]:
        if stage.gripper_position_m is None or stage.gripper_max_effort is None:
            return False, "missing gripper grasp target"
        if not self._gripper_grasp_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, "gripper grasp service unavailable"
        request = GraspGripper.Request()
        request.close_force = max(float(self.get_parameter("gripper_grasp_close_force").value), 0.0)
        request.hold_force = max(float(stage.gripper_max_effort), 0.0)
        request.close_timeout_sec = float(self.get_parameter("gripper_grasp_timeout_sec").value)
        request.min_close_time_sec = float(self.get_parameter("gripper_grasp_min_close_time_sec").value)
        request.velocity_threshold = float(self.get_parameter("gripper_grasp_velocity_threshold").value)
        request.min_closure_distance_m = float(self.get_parameter("gripper_grasp_min_closure_distance_m").value)
        future = self._gripper_grasp_client.call_async(request)
        if not self._wait_for_future(future, self._service_timeout_sec + request.close_timeout_sec):
            return False, "gripper grasp service call timed out"
        result = future.result()
        if result is None:
            return False, "gripper grasp service returned no result"
        self._last_grasp_contact_detected = bool(result.contact_detected)
        self._last_grasp_closure_distance_m = max(
            0.0,
            float(self._last_gripper_reached_position or 0.0) - float(result.reached_position),
        )
        if result.success:
            return True, (
                f"{result.message}: contact={result.contact_detected}, "
                f"contact_position={result.contact_position:.4f}, "
                f"hold_force={result.hold_force:.3f}"
            )
        return False, (
            f"{result.message}: contact={result.contact_detected}, "
            f"reached_position={result.reached_position:.4f}"
        )

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if not self._running:
                return False
            time.sleep(0.02)
        return future.done()

    def _request_stop(self) -> None:
        self._request_stop_service(self._motion_stop_client, "motion execution stop")
        self._request_stop_service(self._trajectory_stop_client, "trajectory_stop")

    def _request_stop_service(self, client, label: str) -> None:
        try:
            if not client.wait_for_service(timeout_sec=0.2):
                return
            client.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().warn(f"failed to request {label}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualGraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

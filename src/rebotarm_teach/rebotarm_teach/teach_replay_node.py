from __future__ import annotations

import json
import signal
import time
from contextlib import suppress
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .parameter_helpers import sensor_qos_kwargs
from .teach_recording import (
    ReplayStartBand,
    analyze_teach_trajectory,
    build_replay_start_soft_points,
    classify_replay_start,
    compute_auto_align_duration,
    load_teach_samples,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    retime_teach_samples,
    teach_trajectory_quality_to_dict,
    write_prepared_teach_record,
)
from rebotarm_motion.trajectory_safety_monitor import evaluate_replay_tracking
from rebotarm_motion.moveit_planner import MoveItMotionPlanner


def _set_duration(duration_msg, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


def _select_collision_points(points: list[tuple[float, ...]], *, max_samples: int) -> list[tuple[int, tuple[float, ...]]]:
    if not points:
        return []
    limit = max(int(max_samples), 1)
    if len(points) <= limit:
        return list(enumerate(points))
    if limit == 1:
        return [(0, points[0])]
    indices = sorted(
        {
            round(index * (len(points) - 1) / (limit - 1))
            for index in range(limit)
        }
    )
    return [(index, points[index]) for index in indices]


class TeachReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("teach_replay_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("direct_threshold", 0.01)
        self.declare_parameter("align_threshold", 0.25)
        self.declare_parameter("align_duration", 3.0)
        self.declare_parameter("align_duration_auto", True)
        self.declare_parameter("align_target_speed_rad_s", 0.15)
        self.declare_parameter("align_min_duration", 3.0)
        self.declare_parameter("align_max_duration", 10.0)
        self.declare_parameter("align_steps", 30)
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
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._record_path = Path(str(self.get_parameter("record_path").value))
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._latest_joint_state: JointState | None = None
        self._started = False
        self._samples = []
        self._start_band = ""
        self._max_error: float | None = None
        self._per_joint_error: tuple[float, ...] = ()
        self._trajectory_points = 0
        self._quality = None
        self._prepared_replay = None
        self._prepared_record_path: Path | None = None
        self._goal_handle = None
        self._active_replay_trajectory: JointTrajectory | None = None
        self._active_replay_started_at: float | None = None
        self._tracking_violation_since: float | None = None
        self._monitor_stop_requested = False
        self._stop_requested = False
        self._stop_reason = ""
        self._moveit_align_message = ""
        self._collision_precheck = {"state": "not_run", "message": "collision precheck not run"}
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
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
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/replay_status",
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_timer(0.2, self._maybe_start)
        self.create_timer(
            max(float(self.get_parameter("replay_monitor_period_sec").value), 0.02),
            self._check_active_replay_tracking,
        )
        self._publish_status("ready", f"waiting to replay {self._record_path}")

    def _max_replay_velocity_limits(self, joint_names: tuple[str, ...]):
        scalar_limit = float(self.get_parameter("max_replay_velocity_rad_s").value)
        values = self.get_parameter("max_replay_velocity_rad_s_by_joint").value
        if isinstance(values, (list, tuple)) and len(values) == len(joint_names):
            return tuple(float(value) for value in values)
        return scalar_limit

    def _auto_align_duration_for_positions(
        self,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        if not bool(self.get_parameter("align_duration_auto").value):
            return float(self.get_parameter("align_duration").value)
        max_error = max(
            (abs(float(current) - float(first)) for current, first in zip(current_positions, first_positions)),
            default=0.0,
        )
        return compute_auto_align_duration(
            max_error,
            target_speed_rad_s=float(self.get_parameter("align_target_speed_rad_s").value),
            min_duration_sec=float(self.get_parameter("align_min_duration").value),
            max_duration_sec=float(self.get_parameter("align_max_duration").value),
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _maybe_start(self) -> None:
        if self._started:
            return
        if self._latest_joint_state is None:
            self._publish_status("waiting", "waiting for current joint_states")
            return
        self._started = True
        try:
            self._samples = load_teach_samples(self._record_path)
        except Exception as exc:
            self._publish_status("failed", f"failed to load record: {exc}")
            return
        if not self._samples:
            self._publish_status("failed", "record contains no samples")
            return

        first = self._samples[0]
        current_map = {
            str(name): float(pos)
            for name, pos in zip(self._latest_joint_state.name, self._latest_joint_state.position)
        }
        missing = [name for name in first.joint_names if name not in current_map]
        if missing:
            self._publish_status("failed", f"current joint_state missing: {', '.join(missing)}")
            return
        current = tuple(current_map[name] for name in first.joint_names)
        decision = classify_replay_start(
            current_positions=current,
            start_positions=first.positions,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        start_band = decision.band
        if decision.band == ReplayStartBand.REJECT and bool(self.get_parameter("use_moveit_start_align").value):
            start_band = ReplayStartBand.MOVEIT_ALIGN
        if decision.band == ReplayStartBand.REJECT:
            if start_band != ReplayStartBand.MOVEIT_ALIGN:
                self._start_band = str(decision.band.value)
                self._max_error = decision.max_error
                self._per_joint_error = decision.per_joint_error
                self._publish_status("rejected", decision.message, max_error=decision.max_error)
                return

        self._start_band = str(start_band.value)
        if start_band == ReplayStartBand.MOVEIT_ALIGN:
            self.get_logger().warn(
                "start error exceeds joint-space align threshold; using MoveIt start alignment"
            )
        else:
            self._start_band = str(decision.band.value)
            self._max_error = decision.max_error
            self._per_joint_error = decision.per_joint_error
        self._max_error = decision.max_error
        self._per_joint_error = decision.per_joint_error
        self._quality = analyze_teach_trajectory(
            self._samples,
            green_jump_rad=float(self.get_parameter("green_jump_rad").value),
            yellow_jump_rad=float(self.get_parameter("yellow_jump_rad").value),
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(self._samples[0].joint_names)),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
        )
        self._prepared_replay = prepare_teach_replay_samples(
            self._samples,
            smoothing_enabled=bool(self.get_parameter("smoothing_enabled").value),
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            filter_enabled=bool(self.get_parameter("filter_enabled").value),
            filter_cutoff_hz=float(self.get_parameter("filter_cutoff_hz").value),
            filter_sample_rate_hz=float(self.get_parameter("filter_sample_rate_hz").value),
            resample_enabled=bool(self.get_parameter("resample_enabled").value),
            resample_rate_hz=float(self.get_parameter("resample_rate_hz").value),
            retime_enabled=True,
            replay_speed=float(self.get_parameter("speed").value),
            max_velocity_rad_s=self._max_replay_velocity_limits(tuple(self._samples[0].joint_names)),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            time_parameterization_method=str(self.get_parameter("time_parameterization_method").value),
            large_motion_span_rad=float(self.get_parameter("large_motion_span_rad").value),
            large_motion_total_rad=float(self.get_parameter("large_motion_total_rad").value),
            large_motion_max_speed=float(self.get_parameter("large_motion_max_speed").value),
        )
        self._prepared_record_path = write_prepared_teach_record(
            self._record_path,
            self._prepared_replay,
        )
        speed = float(self.get_parameter("speed").value)
        yellow_max_speed = float(self.get_parameter("yellow_max_speed").value)
        replay_quality = self._prepared_replay.after_quality
        max_prepared_jump_rad = float(self.get_parameter("max_prepared_jump_rad").value)
        max_replay_acceleration = float(self.get_parameter("max_replay_acceleration_rad_s2").value)
        max_replay_jerk = float(self.get_parameter("max_replay_jerk_rad_s3").value)
        if not self._dry_run and not replay_quality.allow_real_replay:
            self._publish_status(
                "rejected",
                replay_quality.replay_policy,
                max_error=decision.max_error,
            )
            return
        if self._dry_run and not replay_quality.allow_real_replay:
            self._publish_status(
                "dry_run",
                (
                    "validated replay but real execution is blocked; "
                    f"quality={self._quality.risk_level}->{self._prepared_replay.after_quality.risk_level}"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._dry_run and float(replay_quality.max_jump_rad) > max_prepared_jump_rad:
            self._publish_status(
                "rejected",
                (
                    "prepared replay jump is still too large: "
                    f"{float(replay_quality.max_jump_rad):.4f} rad > {max_prepared_jump_rad:.4f} rad"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._dry_run and float(replay_quality.max_acceleration_rad_s2) > max_replay_acceleration:
            self._publish_status(
                "rejected",
                (
                    "retimed replay acceleration is still too large: "
                    f"{float(replay_quality.max_acceleration_rad_s2):.4f} rad/s^2 > "
                    f"{max_replay_acceleration:.4f} rad/s^2"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._dry_run and float(replay_quality.max_jerk_rad_s3) > max_replay_jerk:
            self._publish_status(
                "rejected",
                (
                    "retimed replay jerk is still too large: "
                    f"{float(replay_quality.max_jerk_rad_s3):.4f} rad/s^3 > "
                    f"{max_replay_jerk:.4f} rad/s^3"
                ),
                max_error=decision.max_error,
            )
            return
        effective_speed = (
            float(self._prepared_replay.effective_replay_speed)
            if self._prepared_replay is not None
            else speed
        )
        if not self._dry_run and replay_quality.risk_level == "yellow" and effective_speed > yellow_max_speed:
            self._publish_status(
                "rejected",
                f"prepared yellow replay speed must be <= {yellow_max_speed:.2f}",
                max_error=decision.max_error,
            )
            return
        try:
            trajectory = self._build_trajectory(current, start_band)
        except RuntimeError as exc:
            self._publish_status("blocked", str(exc), max_error=decision.max_error)
            return
        self._trajectory_points = len(trajectory.points)
        self._collision_precheck = self._check_trajectory_collision(trajectory)
        collision_state = str(self._collision_precheck.get("state", "")).lower()
        if collision_state in ("collision", "unknown") and not self._dry_run:
            self._publish_status(
                "blocked",
                f"collision precheck blocked replay: {self._collision_precheck.get('message', collision_state)}",
                max_error=decision.max_error,
            )
            return
        if self._dry_run:
            dry_state = "dry_run" if collision_state not in ("collision", "unknown") else "blocked"
            collision_suffix = (
                ""
                if dry_state == "dry_run"
                else f"; collision precheck blocked replay: {self._collision_precheck.get('message', collision_state)}"
            )
            self._publish_status(
                dry_state,
                (
                    f"validated replay with {len(trajectory.points)} trajectory points; "
                    f"quality={self._quality.risk_level}->{self._prepared_replay.after_quality.risk_level}"
                    f"{collision_suffix}"
                ),
                max_error=decision.max_error,
            )
            return
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self._publish_status("failed", "follow_joint_trajectory action unavailable")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, trajectory))

    def _on_goal_response(self, future, trajectory: JointTrajectory) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("failed", f"failed to send replay trajectory: {exc}")
            return
        if not goal_handle.accepted:
            self._publish_status("rejected", "replay trajectory goal rejected")
            return
        self._goal_handle = goal_handle
        self._active_replay_trajectory = trajectory
        self._active_replay_started_at = time.monotonic()
        self._tracking_violation_since = None
        self._monitor_stop_requested = False
        self._publish_status("replaying", "trajectory goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_replay_result)

    def _on_replay_result(self, future) -> None:
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._publish_status("failed", f"replay result retrieval failed: {exc}")
            self._goal_handle = None
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        self._publish_status(
            state,
            f"replay result status={status}, error_code={error_code}: {error_string}",
        )
        self._goal_handle = None
        self._active_replay_trajectory = None
        self._active_replay_started_at = None
        self._tracking_violation_since = None
        self._monitor_stop_requested = False

    def request_stop(self, reason: str) -> None:
        self._stop_requested = True
        self._stop_reason = reason

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    def cancel_active_goal(self, *, timeout_sec: float = 2.0) -> bool:
        goal_handle = self._goal_handle
        reason = self._stop_reason or "stop requested"
        self._publish_status("cancel_requested", f"{reason}; stopping replay trajectory")
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=min(timeout_sec, 0.8))
        cancel_requested = False
        if goal_handle is not None:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_requested = True
            except Exception as exc:
                self._publish_status("failed", f"failed to request replay cancel: {exc}")
                return stop_requested
            with suppress(Exception, KeyboardInterrupt):
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=timeout_sec)
        return stop_requested or cancel_requested

    def _check_active_replay_tracking(self) -> None:
        if not bool(self.get_parameter("replay_monitor_enabled").value):
            return
        if self._dry_run or self._goal_handle is None or self._active_replay_trajectory is None:
            return
        if self._latest_joint_state is None or self._active_replay_started_at is None:
            return
        now = time.monotonic()
        elapsed = now - self._active_replay_started_at
        if elapsed < float(self.get_parameter("replay_monitor_start_grace_sec").value):
            return
        result = evaluate_replay_tracking(
            self._active_replay_trajectory,
            joint_names=tuple(self._latest_joint_state.name),
            positions=tuple(float(v) for v in self._latest_joint_state.position),
            velocities=tuple(float(v) for v in self._latest_joint_state.velocity),
            elapsed_sec=elapsed,
            max_tracking_error_rad=float(self.get_parameter("max_tracking_error_rad").value),
            max_live_velocity_rad_s=float(self.get_parameter("max_live_velocity_rad_s").value),
        )
        if result.ok:
            self._tracking_violation_since = None
            return
        if self._tracking_violation_since is None:
            self._tracking_violation_since = now
            return
        if now - self._tracking_violation_since < float(self.get_parameter("replay_monitor_violation_grace_sec").value):
            return
        if self._monitor_stop_requested:
            return
        self._monitor_stop_requested = True
        self._publish_status(
            "safety_stop",
            f"runtime replay monitor stopped trajectory: {result.message}",
        )
        self._request_controller_trajectory_stop(timeout_sec=0.2)
        try:
            self._goal_handle.cancel_goal_async()
        except Exception as exc:
            self._publish_status("failed", f"failed to cancel after runtime monitor stop: {exc}")

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=0.1):
                return False
            future = self._trajectory_stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return bool(future.done())
        except Exception:
            return False

    def _build_trajectory(
        self,
        current_positions: tuple[float, ...],
        start_band: ReplayStartBand,
    ) -> JointTrajectory:
        first = self._samples[0]
        trajectory = JointTrajectory()
        trajectory.joint_names = list(first.joint_names)
        elapsed = 0.0
        if bool(self.get_parameter("use_moveit_start_align").value):
            elapsed = self._append_moveit_start_alignment(
                trajectory,
                current_positions=current_positions,
                first_positions=first.positions,
            )
        else:
            start_points = build_replay_start_soft_points(
                current_positions=current_positions,
                first_positions=first.positions,
                start_band=start_band.value,
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                soft_start_duration=float(self.get_parameter("soft_start_duration").value),
                soft_start_steps=int(self.get_parameter("soft_start_steps").value),
                align_duration=self._auto_align_duration_for_positions(current_positions, first.positions),
                align_steps=int(self.get_parameter("align_steps").value),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
            )
            for start_point in start_points:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in start_point.positions]
                point.velocities = [0.0 for _ in start_point.positions]
                _set_duration(point.time_from_start, start_point.time_from_start)
                trajectory.points.append(point)
            if start_points:
                elapsed = start_points[-1].time_from_start
        if self._prepared_replay is not None and self._prepared_replay.retimed_points:
            self._append_prepared_replay_points(trajectory, elapsed=elapsed)
        else:
            speed = max(float(self.get_parameter("speed").value), 0.01)
            if self._prepared_replay is not None:
                speed = max(float(self._prepared_replay.effective_replay_speed), 0.01)
            replay_quality = self._prepared_replay.after_quality if self._prepared_replay is not None else self._quality
            if replay_quality is not None and replay_quality.risk_level == "yellow":
                speed = min(speed, float(self.get_parameter("yellow_max_speed").value))
            replay_samples = self._prepared_replay.samples if self._prepared_replay is not None else self._samples
            for retimed in retime_teach_samples(
                replay_samples,
                replay_speed=speed,
                max_velocity_rad_s=self._max_replay_velocity_limits(tuple(replay_samples[0].joint_names)),
                max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
                initial_delay_sec=float(self.get_parameter("initial_replay_delay_sec").value),
                boundary_zero_velocity=True,
            ):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in retimed.positions]
                if retimed.velocities:
                    point.velocities = [float(v) for v in retimed.velocities]
                _set_duration(point.time_from_start, elapsed + retimed.time_from_start)
                trajectory.points.append(point)
        self._append_final_hold(trajectory)
        return trajectory

    def _check_trajectory_collision(self, trajectory: JointTrajectory) -> dict:
        if not bool(self.get_parameter("collision_check_enabled").value):
            return {"state": "disabled", "message": "collision precheck disabled"}
        points = [
            tuple(float(v) for v in point.positions)
            for point in trajectory.points
            if getattr(point, "positions", None)
        ]
        if not trajectory.joint_names or not points:
            return {"state": "unknown", "message": "no trajectory points to collision check"}
        try:
            available = bool(self._state_validity_client.service_is_ready())
            if not available:
                available = bool(self._state_validity_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            available = False
        if not available:
            return {
                "state": "unknown",
                "message": "MoveIt state validity service unavailable",
                "service": str(self.get_parameter("collision_check_service").value),
                "checked_samples": 0,
            }
        selected = _select_collision_points(
            points,
            max_samples=int(self.get_parameter("collision_check_max_samples").value),
        )
        collision_joint_names, selected = self._with_collision_default_joints(
            tuple(trajectory.joint_names),
            selected,
        )
        timeout_sec = max(float(self.get_parameter("collision_check_timeout_sec").value), 0.1)
        deadline = time.monotonic() + timeout_sec
        checked = 0
        collisions = []
        for point_index, positions in selected:
            if time.monotonic() >= deadline:
                return {
                    "state": "unknown",
                    "message": "collision precheck timed out",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            request = GetStateValidity.Request()
            request.group_name = str(self.get_parameter("collision_group_name").value)
            request.robot_state.joint_state.name = list(collision_joint_names)
            request.robot_state.joint_state.position = [float(v) for v in positions]
            future = self._state_validity_client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                break
            try:
                response = future.result()
            except Exception as exc:
                return {
                    "state": "unknown",
                    "message": f"collision precheck failed: {exc}",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            checked += 1
            if not bool(getattr(response, "valid", False)):
                contacts = []
                for contact in list(getattr(response, "contacts", []))[:5]:
                    contacts.append(
                        {
                            "body_1": str(getattr(contact, "contact_body_1", "")),
                            "body_2": str(getattr(contact, "contact_body_2", "")),
                        }
                    )
                collisions.append({"point": point_index, "contacts": contacts})
                break
        if collisions:
            return {
                "state": "collision",
                "message": "collision detected in replay trajectory",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": collisions,
            }
        if checked < len(selected):
            return {
                "state": "unknown",
                "message": "collision precheck incomplete",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": [],
            }
        return {
            "state": "pass",
            "message": "no collision detected in sampled replay trajectory",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": [],
        }

    def _with_collision_default_joints(
        self,
        joint_names: tuple[str, ...],
        selected: list[tuple[int, tuple[float, ...]]],
    ) -> tuple[tuple[str, ...], list[tuple[int, tuple[float, ...]]]]:
        if str(self.get_parameter("collision_group_name").value) != "arm_with_gripper":
            return joint_names, selected
        existing = set(joint_names)
        defaults: list[tuple[str, float]] = []
        latest = self._latest_joint_state
        latest_positions = {}
        if latest is not None:
            latest_positions = {
                str(name): float(latest.position[index])
                for index, name in enumerate(latest.name)
                if index < len(latest.position)
            }
        for name, fallback in (("left_finger_joint", 0.0), ("right_finger_joint", -0.0)):
            if name not in existing:
                defaults.append((name, float(latest_positions.get(name, fallback))))
        if not defaults:
            return joint_names, selected
        added_names = tuple(name for name, _ in defaults)
        added_positions = tuple(position for _, position in defaults)
        return (
            joint_names + added_names,
            [(index, tuple(positions) + added_positions) for index, positions in selected],
        )

    def _append_final_hold(self, trajectory: JointTrajectory) -> None:
        final_hold = max(float(self.get_parameter("final_hold_sec").value), 0.0)
        if final_hold <= 0.0 or not trajectory.points:
            return
        last_point = trajectory.points[-1]
        last_time = float(last_point.time_from_start.sec) + float(last_point.time_from_start.nanosec) * 1e-9
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in last_point.positions]
        hold_point.velocities = [0.0 for _ in hold_point.positions]
        _set_duration(hold_point.time_from_start, last_time + final_hold)
        trajectory.points.append(hold_point)

    def _append_prepared_replay_points(
        self,
        trajectory: JointTrajectory,
        *,
        elapsed: float,
    ) -> None:
        if self._prepared_replay is None:
            return
        initial_delay = max(float(self.get_parameter("initial_replay_delay_sec").value), 0.0)
        for retimed in self._prepared_replay.retimed_points:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in retimed.positions]
            if retimed.velocities:
                point.velocities = [float(v) for v in retimed.velocities]
            else:
                point.velocities = [0.0 for _ in point.positions]
            _set_duration(point.time_from_start, elapsed + initial_delay + float(retimed.time_from_start))
            trajectory.points.append(point)

    def _append_moveit_start_alignment(
        self,
        trajectory: JointTrajectory,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        elapsed = max(float(self.get_parameter("start_hold_sec").value), 0.0)
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in current_positions]
        hold_point.velocities = [0.0 for _ in current_positions]
        _set_duration(hold_point.time_from_start, elapsed)
        trajectory.points.append(hold_point)
        max_error = max(
            (abs(float(a) - float(b)) for a, b in zip(current_positions, first_positions)),
            default=0.0,
        )
        if max_error >= float(self.get_parameter("moveit_start_skip_threshold").value):
            plan = self._moveit_planner.plan_joint_positions(
                joint_names=tuple(trajectory.joint_names),
                target_positions=first_positions,
                tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            )
            if not plan.success or plan.trajectory is None:
                self._moveit_align_message = plan.message
                raise RuntimeError(f"moveit start alignment failed: {plan.message}")
            self._moveit_align_message = plan.message
            source_names = list(getattr(plan.trajectory, "joint_names", []))
            index_by_name = {name: index for index, name in enumerate(source_names)}
            missing = [name for name in trajectory.joint_names if name not in index_by_name]
            if missing:
                raise RuntimeError(f"moveit start alignment missing joints: {', '.join(missing)}")
            for source_point in getattr(plan.trajectory, "points", []):
                source_time = float(source_point.time_from_start.sec) + float(source_point.time_from_start.nanosec) * 1e-9
                point = JointTrajectoryPoint()
                point.positions = [
                    float(source_point.positions[index_by_name[name]])
                    for name in trajectory.joint_names
                ]
                if getattr(source_point, "velocities", None):
                    point.velocities = [
                        float(source_point.velocities[index_by_name[name]])
                        for name in trajectory.joint_names
                    ]
                _set_duration(point.time_from_start, elapsed + source_time)
                if trajectory.points and point.time_from_start.sec == trajectory.points[-1].time_from_start.sec and point.time_from_start.nanosec == trajectory.points[-1].time_from_start.nanosec:
                    continue
                trajectory.points.append(point)
            if trajectory.points:
                last = trajectory.points[-1].time_from_start
                elapsed = float(last.sec) + float(last.nanosec) * 1e-9
        first_hold = max(float(self.get_parameter("first_hold_sec").value), 0.0)
        if first_hold > 0.0:
            elapsed += first_hold
            first_point = JointTrajectoryPoint()
            first_point.positions = [float(v) for v in first_positions]
            first_point.velocities = [0.0 for _ in first_positions]
            _set_duration(first_point.time_from_start, elapsed)
            trajectory.points.append(first_point)
        return elapsed

    def _publish_status(
        self,
        state: str,
        message: str,
        *,
        max_error: float | None = None,
    ) -> None:
        msg = String()
        payload = {
            "state": state,
            "message": message,
            "record_path": str(self._record_path),
            "dry_run": self._dry_run,
            "speed": float(self.get_parameter("speed").value),
            "samples": len(self._samples),
            "trajectory_points": self._trajectory_points,
            "start_band": self._start_band,
            "per_joint_error": list(self._per_joint_error),
            "direct_threshold": float(self.get_parameter("direct_threshold").value),
            "align_threshold": float(self.get_parameter("align_threshold").value),
            "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
            "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
            "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
            "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
            "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
            "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
            "moveit_start_align_message": self._moveit_align_message,
            "collision_precheck": self._collision_precheck,
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "filter_enabled": bool(self.get_parameter("filter_enabled").value),
            "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
            "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
            "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
            "time_parameterization_method": str(self.get_parameter("time_parameterization_method").value),
        }
        if self._quality is not None:
            payload["quality"] = teach_trajectory_quality_to_dict(self._quality)
            payload["risk_level"] = self._quality.risk_level
        if self._prepared_replay is not None:
            payload["prepared_replay"] = prepared_teach_replay_to_dict(self._prepared_replay)
            payload["prepared_risk_level"] = self._prepared_replay.after_quality.risk_level
            payload["effective_risk_level"] = self._prepared_replay.after_quality.risk_level
        if self._prepared_record_path is not None:
            payload["prepared_record_path"] = str(self._prepared_record_path)
        if max_error is not None:
            payload["max_error"] = max_error
        elif self._max_error is not None:
            payload["max_error"] = self._max_error
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachReplayNode()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def _request_signal_stop(signum, _frame) -> None:
        node.request_stop(f"signal {int(signum)} received")

    signal.signal(signal.SIGINT, _request_signal_stop)
    signal.signal(signal.SIGTERM, _request_signal_stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.stop_requested:
            node.cancel_active_goal(timeout_sec=2.0)
    except KeyboardInterrupt:
        node.request_stop("KeyboardInterrupt received")
        node.cancel_active_goal(timeout_sec=2.0)
    finally:
        with suppress(Exception):
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()

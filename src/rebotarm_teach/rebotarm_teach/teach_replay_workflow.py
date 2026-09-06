from __future__ import annotations

import threading
import time
from contextlib import suppress
from pathlib import Path
from typing import Callable, Protocol, Mapping

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rebotarm_motion.collision_precheck import CollisionPrecheckConfig, CollisionPrechecker
from rebotarm_motion.replay_runtime_monitor import ReplayRuntimeMonitor, ReplayRuntimeMonitorConfig
from .teach_replay_client import TeachReplayClient
from .teach_replay_coordinator import TeachReplayCoordinator, TeachReplayLimits
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
)
from rebotarm_motion.moveit_planner import MoveItMotionPlanner


class ReplaySnapshot(Protocol):
    joints: Mapping[str, dict]
    teleop: Mapping[str, dict]


def _is_number_like(value) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


class TeachReplayWorkflow:
    """Own replay preparation, validation, execution and tracking, independent of HTTP."""

    def __init__(
        self, *, node, joint_names: tuple[str, ...],
        snapshot: Callable[[], ReplaySnapshot],
        publish_status: Callable[[str, dict], None],
        collision_defaults: Callable[[tuple[str, ...]], tuple[tuple[str, float], ...]],
        action_client, trajectory_stop_client, request_stop: Callable[..., bool],
    ):
        self.get_parameter = node.get_parameter
        self.create_client = node.create_client
        self._joint_names = joint_names
        self._snapshot = snapshot
        self._publish_status = publish_status
        self._collision_default_joint_positions = collision_defaults
        self._action_client = action_client
        self._trajectory_stop_client = trajectory_stop_client
        self._request_controller_trajectory_stop = request_stop
        self._moveit_planner = MoveItMotionPlanner(
            node,
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
        self._teach_replay_lock = threading.Lock()
        self._teach_replay_goal_handle = None
        self._active_teach_replay_trajectory: JointTrajectory | None = None
        self._active_teach_replay_started_at: float | None = None
        self._replay_runtime_monitor = ReplayRuntimeMonitor()
        self._last_teach_dry_run: dict | None = None

    def _target_runtime(self) -> str:
        return "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation"

    def record_info(self, record_path: str | None = None) -> dict:
        snapshot = self._snapshot()
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

    def _teach_replay_limits(self) -> TeachReplayLimits:
        return TeachReplayLimits(
            max_prepared_jump_rad=float(self.get_parameter("max_prepared_jump_rad").value),
            max_replay_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_replay_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
        )

    def trajectory_preview(self, record_path: str | None = None, max_points: int = 500) -> dict:
        path = record_path or str(self.record_info(None).get("path", self.get_parameter("record_path").value))
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
        payload["info"] = self.record_info(str(path))
        return payload

    def records(self) -> dict:
        record_path = Path(str(self.get_parameter("record_path").value))
        directory = record_path.parent if str(record_path.parent) else Path("teleop_records")
        records = list_teach_record_files(directory)
        return {
            "directory": str(directory),
            "default_record_path": str(record_path),
            "records": records,
        }

    def dry_run(self, payload: dict) -> dict:
        record_path = payload.get("record_path")
        info_payload = self.record_info(str(record_path) if record_path else None)
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
        self._publish_status("replay", result)
        return result

    def execute(self, payload: dict) -> dict:
        record_path = payload.get("record_path")
        info_payload = self.record_info(str(record_path) if record_path else None)
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
            self._publish_status("replay", result)
            return result
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._publish_status("replay", {"state": "unavailable", "message": message})
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
            self._publish_status("replay", result)
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
        self._publish_status("replay", result)
        return result

    def stop(self) -> dict:
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
        result = self._teach_replay_client.stop(
            goal_handle,
            trajectory_stop_client=self._trajectory_stop_client,
        )
        future = result.pop("cancel_future", None)
        if future is not None:
            future.add_done_callback(self._on_teach_replay_cancel_response)
        self._publish_status("replay", result)
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
        snapshot = self._snapshot()
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
            self._publish_status("replay", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("replay", {"state": "rejected", "message": "teach replay goal rejected"})
            return
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = goal_handle
            self._active_teach_replay_trajectory = trajectory
            self._active_teach_replay_started_at = time.monotonic()
            self._replay_runtime_monitor.reset()
        self._publish_status(
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
            self._publish_status("replay", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "teach replay cancel accepted"
            if goals_canceling
            else "teach replay already finished before cancel"
        )
        self._publish_status("replay", {"state": state, "message": message})
        if not goals_canceling:
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._replay_runtime_monitor.reset()

    def _on_teach_replay_result(self, future, info_payload: dict, points: int) -> None:
        previous_replay = self._snapshot().teleop.get("replay", {})
        with self._teach_replay_lock:
            monitor_stop_requested = self._replay_runtime_monitor.stop_requested
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._publish_status("replay", {"state": "failed", "message": str(exc)})
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
        self._publish_status(
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

    def check_tracking(self) -> None:
        snapshot = self._snapshot()
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
        self._publish_status("replay", decision.status)

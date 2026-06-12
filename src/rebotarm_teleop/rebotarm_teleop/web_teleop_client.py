from __future__ import annotations

from typing import Any, Callable

from .service_call_helpers import call_trigger_service
from .web_execute import (
    WebExecuteDecision,
    WebGripperDecision,
    interpolate_joint_points,
    validate_web_gripper_request,
    validate_web_execute_request,
)


def set_duration(duration_msg: Any, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


def decision_response(decision: WebExecuteDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "positions": dict(zip(decision.joint_names, decision.positions)),
        "duration": decision.duration,
        "max_delta": decision.max_delta,
        "max_delta_limit": decision.max_delta_limit,
    }


def gripper_decision_response(decision: WebGripperDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "position": decision.position,
        "max_effort": decision.max_effort,
    }


class WebTeleopClient:
    """Builds and sends web point-to-point joint trajectories."""

    def __init__(
        self,
        *,
        action_client: Any,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        joint_velocity_limits: dict[str, float],
        trajectory_factory: Callable[[], Any],
        trajectory_point_factory: Callable[[], Any],
        follow_goal_factory: Callable[[], Any],
        gripper_action_client: Any | None = None,
        gripper_goal_factory: Callable[[], Any] | None = None,
    ) -> None:
        self._action_client = action_client
        self._joint_names = tuple(joint_names)
        self._joint_limits = dict(joint_limits)
        self._joint_velocity_limits = dict(joint_velocity_limits)
        self._trajectory_factory = trajectory_factory
        self._trajectory_point_factory = trajectory_point_factory
        self._follow_goal_factory = follow_goal_factory
        self._gripper_action_client = gripper_action_client
        self._gripper_goal_factory = gripper_goal_factory

    def execute(
        self,
        payload: dict,
        *,
        current_positions: dict[str, float],
        max_delta_rad: float,
        min_duration: float,
        max_duration: float,
        max_joint_speed_rad_s: float,
    ) -> dict:
        decision = validate_web_execute_request(
            payload,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            max_delta_rad=max_delta_rad,
            min_duration=min_duration,
            max_duration=max_duration,
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
        )
        if not decision.accepted:
            return {
                "accepted": False,
                "decision": decision,
                "response": decision_response(decision),
                "status": {"state": "rejected", "message": decision.message},
                "goal_future": None,
            }
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            return {
                "accepted": False,
                "decision": decision,
                "response": {"accepted": False, "message": message},
                "status": {"state": "unavailable", "message": message},
                "goal_future": None,
            }

        trajectory = self._trajectory_factory()
        trajectory.joint_names = list(decision.joint_names)
        current = tuple(current_positions[name] for name in decision.joint_names)
        for elapsed, positions in interpolate_joint_points(
            current=current,
            target=decision.positions,
            duration=decision.duration,
        ):
            point = self._trajectory_point_factory()
            point.positions = [float(v) for v in positions]
            set_duration(point.time_from_start, elapsed)
            trajectory.points.append(point)
        goal = self._follow_goal_factory()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        return {
            "accepted": True,
            "decision": decision,
            "response": decision_response(decision),
            "status": {
                "state": "active",
                "message": decision.message,
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(max_joint_speed_rad_s),
                "points": len(trajectory.points),
            },
            "goal_future": future,
            "trajectory": trajectory,
        }

    def stop(self, goal_handle: Any | None, *, trajectory_stop_client: Any) -> dict:
        stop_requested, _stop_message = call_trigger_service(
            trajectory_stop_client,
            timeout_sec=0.2,
        )
        if goal_handle is None:
            message = (
                "no active web execute goal; controller trajectory_stop requested"
                if stop_requested
                else "no active web execute goal; controller trajectory_stop unavailable"
            )
            state = "cancel_requested" if stop_requested else "idle"
            return {
                "accepted": bool(stop_requested),
                "state": state,
                "message": message,
                "trajectory_stop_requested": stop_requested,
                "status": {
                    "state": state,
                    "message": message,
                    "trajectory_stop_requested": stop_requested,
                },
                "cancel_future": None,
                "clear_goal_handle": False,
            }
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as exc:
            message = f"failed to request trajectory cancel: {exc}"
            if stop_requested:
                message = f"{message}; controller trajectory_stop requested"
                return {
                    "accepted": True,
                    "message": message,
                    "trajectory_stop_requested": True,
                    "status": {
                        "state": "cancel_requested",
                        "message": message,
                        "trajectory_stop_requested": True,
                    },
                    "cancel_future": None,
                    "clear_goal_handle": False,
                }
            return {
                "accepted": False,
                "message": message,
                "trajectory_stop_requested": False,
                "status": {"state": "failed", "message": message},
                "cancel_future": None,
                "clear_goal_handle": False,
            }
        message = (
            "trajectory cancel requested; controller trajectory_stop requested"
            if stop_requested
            else "trajectory cancel requested; controller trajectory_stop unavailable"
        )
        return {
            "accepted": True,
            "message": message,
            "trajectory_stop_requested": stop_requested,
            "status": {
                "state": "cancel_requested",
                "message": message,
                "trajectory_stop_requested": stop_requested,
            },
            "cancel_future": future,
            "clear_goal_handle": True,
        }

    def set_gripper(
        self,
        payload: dict,
        *,
        use_hardware: bool,
        gripper_limits: tuple[float, float],
        default_max_effort: float,
        max_effort_limit: float,
    ) -> dict:
        decision = validate_web_gripper_request(
            payload,
            gripper_limits=gripper_limits,
            default_max_effort=default_max_effort,
            max_effort_limit=max_effort_limit,
        )
        if not decision.accepted:
            return {
                "accepted": False,
                "decision": decision,
                "response": gripper_decision_response(decision),
                "status": {"state": "rejected", "message": decision.message},
                "goal_future": None,
                "simulated_position": None,
            }
        if not use_hardware:
            return {
                "accepted": True,
                "decision": decision,
                "response": gripper_decision_response(decision),
                "status": {
                    "state": "done",
                    "message": f"simulated gripper position={decision.position:.4f} m",
                    "position": decision.position,
                    "max_effort": decision.max_effort,
                    "simulated": True,
                },
                "goal_future": None,
                "simulated_position": float(decision.position),
            }
        if self._gripper_action_client is None or self._gripper_goal_factory is None:
            message = "gripper command action unavailable"
            return {
                "accepted": False,
                "decision": decision,
                "response": {"accepted": False, "message": message},
                "status": {"state": "unavailable", "message": message},
                "goal_future": None,
                "simulated_position": None,
            }
        if not self._gripper_action_client.wait_for_server(timeout_sec=0.1):
            message = "gripper command action unavailable"
            return {
                "accepted": False,
                "decision": decision,
                "response": {"accepted": False, "message": message},
                "status": {"state": "unavailable", "message": message},
                "goal_future": None,
                "simulated_position": None,
            }
        goal = self._gripper_goal_factory()
        goal.command.position = float(decision.position)
        goal.command.max_effort = float(decision.max_effort)
        future = self._gripper_action_client.send_goal_async(goal)
        return {
            "accepted": True,
            "decision": decision,
            "response": gripper_decision_response(decision),
            "status": {
                "state": "active",
                "message": decision.message,
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
            "goal_future": future,
            "simulated_position": None,
        }

from __future__ import annotations

from typing import Any, Callable

from .models import SafetyViolationError


PoseFactory = Callable[[], Any]


def _default_pose_factory() -> Any:
    try:
        from geometry_msgs.msg import Pose
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("geometry_msgs is required for sim action bindings") from exc
    return Pose()


def _load_move_to_pose_type() -> Any:
    try:
        from rebotarm_msgs.action import MoveToPose
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rebotarm_msgs.action.MoveToPose is required") from exc
    return MoveToPose


def _load_move_relative_type() -> Any:
    try:
        from rebotarm_msgs.action import MoveRelative
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rebotarm_msgs.action.MoveRelative is required") from exc
    return MoveRelative


def _load_execute_grasp_type() -> Any:
    try:
        from rebotarm_msgs.action import ExecuteGrasp
    except ImportError as exc:  # pragma: no cover - depends on ROS2 environment
        raise SafetyViolationError("rebotarm_msgs.action.ExecuteGrasp is required") from exc
    return ExecuteGrasp


def _as_pose(pose_data: dict[str, Any], pose_factory: PoseFactory) -> Any:
    pose = pose_factory()
    position = pose_data.get("position", {})
    orientation = pose_data.get("orientation", {})
    pose.position.x = float(position.get("x", 0.0))
    pose.position.y = float(position.get("y", 0.0))
    pose.position.z = float(position.get("z", 0.0))
    pose.orientation.x = float(orientation.get("x", 0.0))
    pose.orientation.y = float(orientation.get("y", 0.0))
    pose.orientation.z = float(orientation.get("z", 0.0))
    pose.orientation.w = float(orientation.get("w", 1.0))
    return pose


def build_move_to_pose_goal(
    move_to_pose_type: Any,
    pose_factory: PoseFactory,
    goal: dict[str, Any],
) -> Any:
    pose_data = goal.get("pose")
    if not isinstance(pose_data, dict):
        raise SafetyViolationError("move_to_pose sim goal requires pose")
    request = move_to_pose_type.Goal()
    request.target_pose = _as_pose(pose_data, pose_factory)
    request.duration = float(goal.get("duration", 2.0))
    return request


def build_move_relative_goal(
    move_relative_type: Any,
    goal: dict[str, Any],
) -> Any:
    axis = str(goal.get("axis", ""))
    if axis not in {"x", "y", "z"}:
        raise SafetyViolationError("move_relative sim goal axis must be x, y, or z")
    request = move_relative_type.Goal()
    request.axis = axis
    request.distance_m = float(goal.get("distance_m", 0.0))
    request.frame_id = str(goal.get("frame_id", "base_link"))
    request.speed_scale = float(goal.get("speed_scale", 0.2))
    return request


def build_execute_grasp_goal(
    execute_grasp_type: Any,
    pose_factory: PoseFactory,
    goal: dict[str, Any],
) -> Any:
    request = execute_grasp_type.Goal()
    label = str(goal.get("label", "") or goal.get("target_label", ""))
    pose_data = goal.get("pose")
    if label:
        request.target_label = label
        request.use_label = True
    if isinstance(pose_data, dict):
        request.target_pose = _as_pose(pose_data, pose_factory)
        request.use_pose = True
    if not request.use_label and not request.use_pose:
        raise SafetyViolationError("execute_grasp sim goal requires label or pose")
    return request


def resolve_sim_action_type(
    action_name: str,
    move_relative_type: Any | None = None,
    move_to_pose_type: Any | None = None,
    execute_grasp_type: Any | None = None,
) -> Any:
    if action_name.endswith("/move_relative"):
        return move_relative_type or _load_move_relative_type()
    if action_name.endswith("/move_to_pose"):
        return move_to_pose_type or _load_move_to_pose_type()
    if action_name.endswith("/pick_object") or action_name.endswith("/place_object"):
        return execute_grasp_type or _load_execute_grasp_type()
    raise SafetyViolationError(f"unsupported sim action: {action_name}")


def build_sim_goal(
    action_name: str,
    goal: dict[str, Any],
    move_relative_type: Any | None = None,
    move_to_pose_type: Any | None = None,
    execute_grasp_type: Any | None = None,
    pose_factory: PoseFactory | None = None,
) -> Any:
    pose_factory = pose_factory or _default_pose_factory
    if action_name.endswith("/move_relative"):
        return build_move_relative_goal(
            move_relative_type or _load_move_relative_type(),
            goal,
        )
    if action_name.endswith("/move_to_pose"):
        return build_move_to_pose_goal(
            move_to_pose_type or _load_move_to_pose_type(),
            pose_factory,
            goal,
        )
    if action_name.endswith("/pick_object") or action_name.endswith("/place_object"):
        return build_execute_grasp_goal(
            execute_grasp_type or _load_execute_grasp_type(),
            pose_factory,
            goal,
        )
    raise SafetyViolationError(f"unsupported sim action: {action_name}")

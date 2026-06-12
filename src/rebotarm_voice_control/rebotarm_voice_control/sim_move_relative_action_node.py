from __future__ import annotations

try:
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
except ImportError:  # pragma: no cover - lets pure validation tests run without ROS2
    rclpy = None
    ActionClient = None
    ActionServer = None
    CancelResponse = None
    GoalResponse = None
    ExternalShutdownException = Exception
    MultiThreadedExecutor = None
    Node = object

from .models import SafetyViolationError
from .sim_motion_adapter import (
    MoveRelativeSimMotionAdapter,
    build_follow_joint_trajectory_goal,
)


def validate_move_relative_goal(goal) -> dict[str, float | str]:
    axis = str(getattr(goal, "axis", "")).strip().lower()
    distance_m = float(getattr(goal, "distance_m", 0.0))
    frame_id = str(getattr(goal, "frame_id", "")).strip() or "base_link"
    speed_scale = float(getattr(goal, "speed_scale", 0.2))

    if axis not in {"x", "y", "z"}:
        raise SafetyViolationError("move_relative axis must be x, y, or z")
    if abs(distance_m) > 0.05:
        raise SafetyViolationError("move_relative distance exceeds 0.05 m in sim")
    if not 0.05 <= speed_scale <= 1.0:
        raise SafetyViolationError("move_relative speed_scale must be within [0.05, 1.0]")

    return {
        "axis": axis,
        "distance_m": distance_m,
        "frame_id": frame_id,
        "speed_scale": speed_scale,
    }


class SimMoveRelativeActionNode(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("ROS2 rclpy is required to run SimMoveRelativeActionNode")
        super().__init__("rebotarm_sim_move_relative_action")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ee_frame_id", "tool0")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("planning_service", "/plan_kinematic_path")
        self.declare_parameter("planning_pipeline", "ompl")
        self.declare_parameter("planner_id", "RRTConnect")
        self.declare_parameter("planning_time", 3.0)
        self.declare_parameter("num_attempts", 3)
        self.declare_parameter("goal_position_tolerance", 0.01)
        self.declare_parameter("goal_orientation_tolerance", 0.05)
        self.declare_parameter("sim_trajectory_action", "/rebotarm/follow_joint_trajectory")

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._move_relative_type = self._resolve_action_type()
        self._follow_joint_trajectory_type = self._resolve_follow_joint_trajectory_type()
        self._planner = self._build_planner()
        self._trajectory_client = ActionClient(
            self,
            self._follow_joint_trajectory_type,
            str(self.get_parameter("sim_trajectory_action").value),
        )
        self._adapter = MoveRelativeSimMotionAdapter(
            planner=self._planner,
            trajectory_client=self._trajectory_client,
            current_pose_supplier=self._current_pose_supplier,
            goal_builder=build_follow_joint_trajectory_goal,
            future_waiter=self._wait_for_future,
        )
        self._action_server = ActionServer(
            self,
            self._move_relative_type,
            f"/{self._arm_namespace}/sim/move_relative",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            f"sim move_relative action ready on /{self._arm_namespace}/sim/move_relative"
        )

    def _resolve_action_type(self):
        try:
            from rebotarm_msgs.action import MoveRelative
        except ImportError as exc:  # pragma: no cover - depends on ROS2 workspace
            raise SafetyViolationError("rebotarm_msgs.action.MoveRelative is required") from exc
        return MoveRelative

    def _resolve_follow_joint_trajectory_type(self):
        try:
            from control_msgs.action import FollowJointTrajectory
        except ImportError as exc:  # pragma: no cover - depends on ROS2 workspace
            raise SafetyViolationError("control_msgs.action.FollowJointTrajectory is required") from exc
        return FollowJointTrajectory

    def _build_planner(self):
        from rebotarm_motion.moveit_planner import MoveItMotionPlanner

        planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("group_name").value),
            ee_frame_id=str(self.get_parameter("ee_frame_id").value),
            frame_id=str(self.get_parameter("frame_id").value),
            planning_service=str(self.get_parameter("planning_service").value),
            planning_pipeline=str(self.get_parameter("planning_pipeline").value),
            planner_id=str(self.get_parameter("planner_id").value),
            planning_time=float(self.get_parameter("planning_time").value),
            num_attempts=int(self.get_parameter("num_attempts").value),
            goal_position_tolerance=float(self.get_parameter("goal_position_tolerance").value),
            goal_orientation_tolerance=float(self.get_parameter("goal_orientation_tolerance").value),
        )
        return planner

    def _goal_callback(self, goal_request) -> GoalResponse:
        try:
            validate_move_relative_goal(goal_request)
        except SafetyViolationError as exc:
            self.get_logger().warn(f"reject move_relative goal: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        result_msg = self._move_relative_type.Result()
        feedback_msg = self._move_relative_type.Feedback()
        try:
            request = validate_move_relative_goal(goal_handle.request)
            feedback_msg.state = "planning"
            feedback_msg.progress = 0.2
            goal_handle.publish_feedback(feedback_msg)

            if not self._trajectory_client.wait_for_server(timeout_sec=3.0):
                result_msg.success = False
                result_msg.message = "sim trajectory controller unavailable"
                goal_handle.abort()
                return result_msg

            feedback_msg.state = "dispatching"
            feedback_msg.progress = 0.6
            goal_handle.publish_feedback(feedback_msg)

            adapter_result = self._adapter.execute_move_relative(**request)
            if not adapter_result.success:
                result_msg.success = False
                result_msg.message = adapter_result.message
                goal_handle.abort()
                return result_msg

            feedback_msg.state = "executing"
            feedback_msg.progress = 0.9
            goal_handle.publish_feedback(feedback_msg)

            result_msg.success = True
            result_msg.message = adapter_result.message
            goal_handle.succeed()
            return result_msg
        except SafetyViolationError as exc:
            result_msg.success = False
            result_msg.message = str(exc)
            goal_handle.abort()
            return result_msg
        except Exception as exc:  # pragma: no cover - runtime safety
            self.get_logger().exception("move_relative execution failed")
            result_msg.success = False
            result_msg.message = f"move_relative execution failed: {exc}"
            goal_handle.abort()
            return result_msg

    def _current_pose_supplier(self) -> dict[str, float]:
        return {"x": 0.2, "y": 0.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def _wait_for_future(self, future, *, timeout_sec: float) -> None:
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 rclpy is required to run rebotarm_sim_move_relative_action")
    rclpy.init(args=args)
    node = SimMoveRelativeActionNode()
    executor = MultiThreadedExecutor(num_threads=4)
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

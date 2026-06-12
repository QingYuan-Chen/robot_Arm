from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from rebotarm_msgs.srv import ExecutePose

from .moveit_planner import MoveItMotionPlanner


class PoseExecutionNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_execution")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_planning_time", 2.0)
        self.declare_parameter("moveit_num_planning_attempts", 1)
        self.declare_parameter("goal_position_tolerance", 0.005)
        self.declare_parameter("goal_orientation_tolerance", 0.02)
        self.declare_parameter("default_velocity_scaling", 0.10)
        self.declare_parameter("default_acceleration_scaling", 0.08)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._active_goal_handle = None
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            callback_group=self._callback_group,
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        self._planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id=str(self.get_parameter("ee_frame_id").value),
            frame_id=str(self.get_parameter("frame_id").value),
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=float(self.get_parameter("goal_position_tolerance").value),
            goal_orientation_tolerance=float(self.get_parameter("goal_orientation_tolerance").value),
        )
        self.create_service(
            ExecutePose,
            f"/{self._arm_namespace}/motion_execution/execute_pose",
            self._execute_pose,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            self._stop,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"pose motion execution ready: /{self._arm_namespace}/motion_execution/execute_pose"
        )

    def _execute_pose(self, request: ExecutePose.Request, response: ExecutePose.Response):
        response.stage = "planning"
        velocity = float(request.velocity_scaling) or float(self.get_parameter("default_velocity_scaling").value)
        acceleration = float(request.acceleration_scaling) or float(
            self.get_parameter("default_acceleration_scaling").value
        )
        plan = self._planner.plan_pose(
            request.target_pose,
            velocity_scaling=velocity,
            acceleration_scaling=acceleration,
        )
        if not plan.success or plan.trajectory is None:
            response.success = False
            response.message = plan.message
            return response

        response.planned_trajectory = plan.trajectory
        if not bool(request.execute):
            response.success = True
            response.stage = "planning"
            response.message = plan.message
            return response

        response.stage = "execution"
        if not self._trajectory_client.wait_for_server(timeout_sec=max(float(request.timeout_sec), 1.0)):
            response.success = False
            response.message = "follow_joint_trajectory action unavailable"
            return response

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = plan.trajectory
        send_future = self._trajectory_client.send_goal_async(goal)
        self._wait_future(send_future, max(float(request.timeout_sec), 1.0))
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            response.success = False
            response.message = "trajectory goal rejected"
            return response

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._wait_future(result_future, max(float(request.timeout_sec), 30.0))
        self._active_goal_handle = None
        if not result_future.done():
            response.success = False
            response.message = "trajectory result timeout"
            return response

        wrapped = result_future.result()
        status = int(getattr(wrapped, "status", 0))
        result = getattr(wrapped, "result", None)
        error_string = str(getattr(result, "error_string", "") or "")
        error_code = int(getattr(result, "error_code", 0) or 0)
        if status != 4 or error_code != int(FollowJointTrajectory.Result.SUCCESSFUL):
            response.success = False
            response.message = (
                f"trajectory failed: status={status}, error_code={error_code}, {error_string}".strip()
            )
            return response

        response.success = True
        response.message = "trajectory executed"
        return response

    def _stop(self, _request: Trigger.Request, response: Trigger.Response):
        goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"failed to cancel active trajectory: {exc}")
        if self._trajectory_stop_client.wait_for_service(timeout_sec=0.2):
            try:
                self._trajectory_stop_client.call_async(Trigger.Request())
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"failed to request trajectory_stop: {exc}")
        response.success = True
        response.message = "motion execution stop requested"
        return response

    def _wait_future(self, future, timeout_sec: float) -> None:
        deadline = time.monotonic() + max(float(timeout_sec), 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseExecutionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

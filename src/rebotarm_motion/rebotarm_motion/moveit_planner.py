from __future__ import annotations

import time
from dataclasses import dataclass
from math import cos, sin
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.duration import Duration
from shape_msgs.msg import SolidPrimitive


@dataclass(frozen=True)
class MotionPlanResult:
    success: bool
    message: str
    trajectory: object | None


class MoveItMotionPlanner:
    """Requests a full MoveIt motion plan for the latest preview target."""

    def __init__(
        self,
        node,
        *,
        group_name: str,
        ee_frame_id: str,
        frame_id: str,
        planning_service: str,
        planning_pipeline: str,
        planner_id: str,
        planning_time: float,
        num_attempts: int,
        goal_position_tolerance: float,
        goal_orientation_tolerance: float,
    ) -> None:
        self._node = node
        self._group_name = group_name
        self._ee_frame_id = ee_frame_id
        self._frame_id = frame_id
        self._planning_pipeline = planning_pipeline
        self._planner_id = planner_id
        self._planning_time = float(planning_time)
        self._num_attempts = int(num_attempts)
        self._goal_position_tolerance = float(goal_position_tolerance)
        self._goal_orientation_tolerance = float(goal_orientation_tolerance)
        self._client = node.create_client(GetMotionPlan, planning_service)

    def plan_joint_positions(
        self,
        *,
        joint_names: tuple[str, ...],
        target_positions: tuple[float, ...],
        tolerance: float = 0.005,
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
    ) -> MotionPlanResult:
        if len(joint_names) != len(target_positions):
            return MotionPlanResult(
                success=False,
                message="moveit joint target length mismatch",
                trajectory=None,
            )
        if not self._client.wait_for_service(timeout_sec=0.5):
            return MotionPlanResult(
                success=False,
                message="moveit planning service unavailable",
                trajectory=None,
            )

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self._group_name
        motion_request.pipeline_id = self._planning_pipeline
        motion_request.planner_id = self._planner_id
        motion_request.num_planning_attempts = self._num_attempts
        motion_request.allowed_planning_time = self._planning_time
        motion_request.max_velocity_scaling_factor = min(max(float(velocity_scaling), 0.01), 1.0)
        motion_request.max_acceleration_scaling_factor = min(max(float(acceleration_scaling), 0.01), 1.0)
        motion_request.start_state.is_diff = True
        constraints = Constraints()
        tol = max(float(tolerance), 1e-5)
        for name, position in zip(joint_names, target_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = str(name)
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = tol
            joint_constraint.tolerance_below = tol
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        motion_request.goal_constraints = [constraints]
        return self._call_plan_service(request)

    def plan_preview(self, preview: Any) -> MotionPlanResult:
        pose_target = preview.pose_target
        if pose_target is None:
            return MotionPlanResult(
                success=False,
                message="preview has no pose target for moveit planning",
                trajectory=None,
            )

        if not self._client.wait_for_service(timeout_sec=0.5):
            return MotionPlanResult(
                success=False,
                message="moveit planning service unavailable",
                trajectory=None,
            )

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self._group_name
        motion_request.pipeline_id = self._planning_pipeline
        motion_request.planner_id = self._planner_id
        motion_request.num_planning_attempts = self._num_attempts
        motion_request.allowed_planning_time = self._planning_time
        motion_request.max_velocity_scaling_factor = 0.1
        motion_request.max_acceleration_scaling_factor = 0.1
        motion_request.start_state.is_diff = True
        motion_request.goal_constraints = [self._build_goal_constraints(preview)]

        return self._call_plan_service(request)

    def plan_pose(
        self,
        pose: PoseStamped,
        *,
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
    ) -> MotionPlanResult:
        if not self._client.wait_for_service(timeout_sec=0.5):
            return MotionPlanResult(
                success=False,
                message="moveit planning service unavailable",
                trajectory=None,
            )

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self._group_name
        motion_request.pipeline_id = self._planning_pipeline
        motion_request.planner_id = self._planner_id
        motion_request.num_planning_attempts = self._num_attempts
        motion_request.allowed_planning_time = self._planning_time
        motion_request.max_velocity_scaling_factor = min(max(float(velocity_scaling), 0.01), 1.0)
        motion_request.max_acceleration_scaling_factor = min(max(float(acceleration_scaling), 0.01), 1.0)
        motion_request.start_state.is_diff = True
        motion_request.goal_constraints = [self._build_pose_constraints(pose)]

        return self._call_plan_service(request)

    def _call_plan_service(self, request: GetMotionPlan.Request) -> MotionPlanResult:
        future = self._client.call_async(request)
        self._spin_until_future(future)
        if not future.done():
            return MotionPlanResult(
                success=False,
                message="moveit planning request timed out",
                trajectory=None,
            )

        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            return MotionPlanResult(
                success=False,
                message=f"moveit planning request failed: {exc}",
                trajectory=None,
            )

        error_code = int(response.motion_plan_response.error_code.val)
        if error_code != 1:
            return MotionPlanResult(
                success=False,
                message=f"moveit planning failed: error_code={error_code}",
                trajectory=None,
            )

        trajectory = response.motion_plan_response.trajectory
        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        if joint_trajectory is None or not joint_trajectory.points:
            return MotionPlanResult(
                success=False,
                message="moveit planning returned empty joint trajectory",
                trajectory=None,
            )

        return MotionPlanResult(
            success=True,
            message="moveit trajectory planned",
            trajectory=joint_trajectory,
        )

    def _build_goal_constraints(self, preview: Any) -> Constraints:
        pose_target = preview.pose_target
        assert pose_target is not None

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = float(pose_target.x)
        pose.pose.position.y = float(pose_target.y)
        pose.pose.position.z = float(pose_target.z)
        qx, qy, qz, qw = rpy_to_quaternion(
            float(pose_target.roll),
            float(pose_target.pitch),
            float(pose_target.yaw),
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return self._build_pose_constraints(pose)

    def _build_pose_constraints(self, pose: PoseStamped) -> Constraints:
        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = self._ee_frame_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        tol = max(self._goal_position_tolerance, 1e-4)
        primitive.dimensions = [tol, tol, tol]
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self._ee_frame_id
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self._goal_orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def _spin_until_future(self, future) -> None:
        deadline = self._node.get_clock().now() + Duration(seconds=self._planning_time + 1.0)
        while not future.done() and self._node.get_clock().now() < deadline:
            time.sleep(0.05)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

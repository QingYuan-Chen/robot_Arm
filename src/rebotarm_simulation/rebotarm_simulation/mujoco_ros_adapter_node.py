from __future__ import annotations

from pathlib import Path
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rebotarm_msgs.msg import JointMotorState
from rebotarm_msgs.srv import SetGripper
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .mujoco_adapter_core import (
    ARM_JOINT_NAMES,
    MuJoCoArmAdapter,
    consume_sim_steps,
    execution_timeout_seconds,
    first_tolerance_violation,
    interpolate_trajectory,
    normalize_trajectory_points,
    trajectory_error_code_for_stop_reason,
)
from .mujoco_metrics import TrajectoryMetricsRecorder
from .mujoco_model_profile import DEFAULT_GRIPPER_XML, write_physics_profile, xml_asset_references_are_readable


class MuJoCoRosAdapterNode(Node):
    """ROS 2 adapter that drives the reBotArm MuJoCo model with MoveIt trajectories."""

    def __init__(self) -> None:
        super().__init__("rebotarm_mujoco_adapter")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("model_xml", "build/mujoco_models/reBot-DevArm_gripper_physics.xml")
        self.declare_parameter("source_xml", str(DEFAULT_GRIPPER_XML))
        self.declare_parameter("auto_generate_model", True)
        self.declare_parameter("control_rate_hz", 200.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("metrics_dir", "build/mujoco_runs/latest")
        self.declare_parameter("gripper_min_width_m", 0.0)
        self.declare_parameter("gripper_max_width_m", 0.09)
        self.declare_parameter("path_tolerance_rad", 0.12)
        self.declare_parameter("goal_tolerance_rad", 0.06)
        self.declare_parameter("execution_timeout_margin_sec", 2.0)
        self.declare_parameter("metrics_sample_stride", 1)
        self.declare_parameter("use_mujoco_viewer", False)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self._publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._metrics_dir = Path(str(self.get_parameter("metrics_dir").value))
        self._gripper_min_width_m = float(self.get_parameter("gripper_min_width_m").value)
        self._gripper_max_width_m = float(self.get_parameter("gripper_max_width_m").value)
        self._path_tolerance_rad = max(float(self.get_parameter("path_tolerance_rad").value), 0.0)
        self._goal_tolerance_rad = max(float(self.get_parameter("goal_tolerance_rad").value), 0.0)
        self._execution_timeout_margin_sec = max(
            float(self.get_parameter("execution_timeout_margin_sec").value), 0.0
        )
        self._metrics_sample_stride = max(int(self.get_parameter("metrics_sample_stride").value), 1)
        self._use_mujoco_viewer = bool(self.get_parameter("use_mujoco_viewer").value)
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()

        model_xml = self._ensure_model_xml()
        self._adapter = MuJoCoArmAdapter(model_xml)
        self._last_publish_time = 0.0
        self._last_step_wall_time = time.monotonic()
        self._pending_sim_seconds = 0.0
        self._latest_targets = self._adapter.arm_positions()

        self._joint_state_pub = self.create_publisher(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            10,
        )
        self._gripper_state_pub = self.create_publisher(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            10,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.create_service(Trigger, f"/{self._arm_namespace}/trajectory_stop", self._stop_service)
        self.create_service(SetGripper, f"/{self._arm_namespace}/gripper/set", self._set_gripper_service)
        self.create_timer(1.0 / self._control_rate_hz, self._step_and_publish)
        if self._use_mujoco_viewer:
            threading.Thread(target=self._run_viewer, daemon=True).start()
        self.get_logger().info(
            f"MuJoCo adapter ready: /{self._arm_namespace}/follow_joint_trajectory, "
            f"/{self._arm_namespace}/joint_states"
        )

    def _run_viewer(self) -> None:
        try:
            import mujoco.viewer

            with mujoco.viewer.launch_passive(self._adapter.model, self._adapter.data) as viewer:
                while rclpy.ok() and viewer.is_running():
                    with self._lock:
                        viewer.sync()
                    time.sleep(1.0 / max(self._publish_rate_hz, 1.0))
        except Exception as exc:
            self.get_logger().error(f"MuJoCo viewer failed: {exc}")

    def _ensure_model_xml(self) -> Path:
        model_xml = Path(str(self.get_parameter("model_xml").value))
        auto_generate_model = bool(self.get_parameter("auto_generate_model").value)
        if model_xml.exists() and xml_asset_references_are_readable(model_xml):
            return model_xml
        if not auto_generate_model:
            raise FileNotFoundError(f"MuJoCo model XML does not exist: {model_xml}")
        source_xml = Path(str(self.get_parameter("source_xml").value))
        self.get_logger().info(f"generating MuJoCo model XML at {model_xml}")
        return write_physics_profile(model_xml, source_xml)

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not goal_request.trajectory.joint_names or not goal_request.trajectory.points:
            self.get_logger().warn("rejecting empty MuJoCo trajectory goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._stop_requested.set()
        return CancelResponse.ACCEPT

    def _stop_service(self, _request, response):
        self._stop_requested.set()
        response.success = True
        response.message = "MuJoCo trajectory stop requested"
        return response

    def _set_gripper_service(self, request, response):
        with self._lock:
            reached = self._adapter.set_gripper_width(
                float(request.position),
                min_width=self._gripper_min_width_m,
                max_width=self._gripper_max_width_m,
            )
            self._publish_gripper_state()
        response.success = True
        response.reached_position = float(reached)
        return response

    def _execute_goal(self, goal_handle):
        self._stop_requested.clear()
        result = FollowJointTrajectory.Result()
        try:
            with self._lock:
                current_positions = self._adapter.arm_positions()
            trajectory = normalize_trajectory_points(
                source_joint_names=list(goal_handle.request.trajectory.joint_names),
                target_joint_names=ARM_JOINT_NAMES,
                points=list(goal_handle.request.trajectory.points),
                current_positions=current_positions,
            )
        except ValueError as exc:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result

        recorder = TrajectoryMetricsRecorder(
            self._metrics_dir,
            joint_names=ARM_JOINT_NAMES,
            sample_stride=self._metrics_sample_stride,
        )
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(ARM_JOINT_NAMES)
        with self._lock:
            start_sim_time = self._adapter.sim_time
        end_time = trajectory[-1].time_from_start
        deadline = time.monotonic() + execution_timeout_seconds(
            end_time,
            margin_sec=self._execution_timeout_margin_sec,
        )
        stop_reason = "finished"
        violated_joint = None
        violated_tolerance = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                stop_reason = "canceled"
                goal_handle.canceled()
                break
            if self._stop_requested.is_set():
                stop_reason = "stopped"
                goal_handle.canceled()
                break

            with self._lock:
                elapsed = min(self._adapter.sim_time - start_sim_time, end_time)
            if elapsed < end_time and time.monotonic() >= deadline:
                stop_reason = "timeout"
                goal_handle.abort()
                break

            with self._lock:
                targets = interpolate_trajectory(trajectory, elapsed)
                self._adapter.set_arm_targets(targets)
                self._latest_targets = list(targets)
                actual = self._adapter.arm_positions()
                velocities = self._adapter.arm_velocities()
                forces = self._adapter.arm_actuator_forces()
            errors = [target - position for target, position in zip(targets, actual)]
            recorder.record(
                elapsed=elapsed,
                targets=list(targets),
                actual=actual,
                velocities=velocities,
                actuator_forces=forces,
            )
            feedback.desired.positions = list(targets)
            feedback.actual.positions = actual
            feedback.error.positions = errors
            goal_handle.publish_feedback(feedback)
            if elapsed >= end_time:
                violation = first_tolerance_violation(
                    joint_names=ARM_JOINT_NAMES,
                    errors=errors,
                    tolerance=self._goal_tolerance_rad,
                )
                if violation is not None:
                    stop_reason = "goal_tolerance_violated"
                    violated_joint = violation.joint
                    violated_tolerance = self._goal_tolerance_rad
                    goal_handle.abort()
                else:
                    goal_handle.succeed()
                break
            violation = first_tolerance_violation(
                joint_names=ARM_JOINT_NAMES,
                errors=errors,
                tolerance=self._path_tolerance_rad,
            )
            if violation is not None:
                stop_reason = "path_tolerance_violated"
                violated_joint = violation.joint
                violated_tolerance = self._path_tolerance_rad
                goal_handle.abort()
                break
            time.sleep(1.0 / self._control_rate_hz)

        if stop_reason == "finished" and not rclpy.ok():
            stop_reason = "stopped"
        if stop_reason != "finished":
            with self._lock:
                hold_position = self._adapter.arm_positions()
                self._adapter.set_arm_targets(hold_position)
                self._latest_targets = list(hold_position)

        success = stop_reason == "finished"
        recorder.finish(
            success=success,
            stop_reason=stop_reason,
            violated_joint=violated_joint,
            tolerance=violated_tolerance,
        )
        result.error_code = trajectory_error_code_for_stop_reason(
            stop_reason,
            result_type=FollowJointTrajectory.Result,
        )
        result.error_string = (
            "MuJoCo trajectory finished"
            if success
            else f"MuJoCo trajectory {stop_reason}"
        )
        return result

    def _step_and_publish(self) -> None:
        with self._lock:
            now = time.monotonic()
            wall_delta = now - self._last_step_wall_time
            self._last_step_wall_time = now
            steps, self._pending_sim_seconds = consume_sim_steps(
                wall_delta=wall_delta,
                timestep=self._adapter.timestep,
                pending_sim_seconds=self._pending_sim_seconds,
            )
            if steps > 0:
                self._adapter.step(steps)
            if now - self._last_publish_time >= 1.0 / self._publish_rate_hz:
                self._publish_joint_state()
                self._publish_gripper_state()
                self._last_publish_time = now

    def _publish_joint_state(self) -> None:
        snapshot = self._adapter.joint_state_snapshot()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = snapshot.names
        msg.position = snapshot.positions
        msg.velocity = snapshot.velocities
        msg.effort = snapshot.efforts
        self._joint_state_pub.publish(msg)

    def _publish_gripper_state(self) -> None:
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(self._adapter.gripper_width())
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 0
        self._gripper_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MuJoCoRosAdapterNode()
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

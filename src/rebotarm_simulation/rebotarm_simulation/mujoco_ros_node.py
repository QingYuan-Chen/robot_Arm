"""Safe ROS 2 adapter for the headless reBotArm MuJoCo simulation.

The validation helpers in this module intentionally have no ROS imports so
trajectory inputs can be fuzzed and unit tested on development hosts without
a ROS installation. ROS types are imported only when the node is constructed.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Sequence

from .trajectory_sampler import ARM_JOINT_NAMES, NamedTrajectoryPoint, TrajectorySampler
from .virtual_camera import VirtualCameraConfig, VirtualCameraWorker


DEFAULT_MAX_TRAJECTORY_POINTS = 10_000
DEFAULT_MAX_TRAJECTORY_DURATION_SEC = 300.0


@dataclass(frozen=True)
class GoalSettlingPolicy:
    position_tolerance: float = 0.02
    velocity_tolerance: float = 0.05
    time_tolerance_sec: float = 5.0

    def __post_init__(self) -> None:
        values = (self.position_tolerance, self.velocity_tolerance, self.time_tolerance_sec)
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values):
            raise ValueError("goal tolerances must be positive finite values")

    def evaluate(self, desired, actual, velocities, settle_elapsed: float) -> str:
        vectors = (tuple(desired), tuple(actual), tuple(velocities))
        if any(len(vector) != len(ARM_JOINT_NAMES) for vector in vectors):
            raise ValueError("goal state must contain six arm values")
        numeric = tuple(tuple(float(value) for value in vector) for vector in vectors)
        elapsed = float(settle_elapsed)
        if any(not math.isfinite(value) for vector in numeric for value in vector) or not math.isfinite(elapsed):
            raise ValueError("goal state must be finite")
        if elapsed < 0.0:
            raise ValueError("settling time must be non-negative")
        position_error = max(abs(target - reached) for target, reached in zip(numeric[0], numeric[1]))
        max_velocity = max(abs(value) for value in numeric[2])
        if position_error <= self.position_tolerance and max_velocity <= self.velocity_tolerance:
            return "succeeded"
        if elapsed >= self.time_tolerance_sec:
            return "timed_out"
        return "settling"


def duration_to_seconds(duration: Any) -> float:
    try:
        value = float(duration.sec) + float(duration.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("trajectory time is invalid") from exc
    if not math.isfinite(value):
        raise ValueError("trajectory time must be finite")
    return value


def trajectory_to_sampler(
    trajectory: Any,
    *,
    initial_positions: Sequence[float],
    max_points: int = DEFAULT_MAX_TRAJECTORY_POINTS,
    max_duration_sec: float = DEFAULT_MAX_TRAJECTORY_DURATION_SEC,
) -> TrajectorySampler:
    """Validate an untrusted JointTrajectory-like object and build a sampler."""
    if isinstance(max_points, bool) or int(max_points) <= 0:
        raise ValueError("max points must be positive")
    duration_cap = float(max_duration_sec)
    if not math.isfinite(duration_cap) or duration_cap <= 0.0:
        raise ValueError("max duration must be finite and positive")
    try:
        names = tuple(trajectory.joint_names)
        raw_points = tuple(trajectory.points)
    except (AttributeError, TypeError) as exc:
        raise ValueError("trajectory structure is invalid") from exc
    if len(raw_points) > int(max_points):
        raise ValueError("trajectory contains too many points")

    points = tuple(
        NamedTrajectoryPoint(duration_to_seconds(point.time_from_start), point.positions)
        for point in raw_points
    )
    # TrajectorySampler owns canonical-name, duplicate, length, finite-value,
    # non-empty, and strictly-increasing-time validation.
    sampler = TrajectorySampler(names, points, initial_positions=initial_positions)
    if sampler.duration > duration_cap:
        raise ValueError("trajectory duration exceeds configured limit")
    return sampler


def validate_gripper_width(width: Any) -> float:
    try:
        value = float(width)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper width must be finite") from exc
    if not math.isfinite(value):
        raise ValueError("gripper width must be finite")
    return value


def seconds_to_stamp_parts(simulation_time: Any) -> tuple[int, int]:
    try:
        value = float(simulation_time)
    except (TypeError, ValueError) as exc:
        raise ValueError("simulation time must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    seconds = math.floor(value)
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return int(seconds), nanoseconds


class MonotonicStamp:
    def __init__(self) -> None:
        self._nanoseconds = 0
        self._lock = threading.Lock()

    def update(self, simulation_time: Any) -> tuple[int, int]:
        seconds, nanoseconds = seconds_to_stamp_parts(simulation_time)
        candidate = seconds * 1_000_000_000 + nanoseconds
        with self._lock:
            self._nanoseconds = max(self._nanoseconds, candidate)
            return divmod(self._nanoseconds, 1_000_000_000)


class FeedbackRateLimiter:
    def __init__(self, rate_hz: Any) -> None:
        try:
            rate = float(rate_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError("feedback rate must be finite and in (0, 200]") from exc
        if not math.isfinite(rate) or rate <= 0.0 or rate > 200.0:
            raise ValueError("feedback rate must be finite and in (0, 200]")
        self.rate_hz = rate
        self._period = 1.0 / rate
        self._last_publish: float | None = None

    def should_publish(self, monotonic_time: Any, *, final: bool = False) -> bool:
        now = float(monotonic_time)
        if not math.isfinite(now):
            raise ValueError("feedback clock must be finite")
        due = self._last_publish is None or now - self._last_publish >= self._period
        if final or due:
            self._last_publish = now
            return True
        return False


class ActiveTrajectory:
    """Thread-safe single-goal admission and cooperative cancellation gate."""

    def __init__(self, lock: threading.RLock | None = None) -> None:
        self._lock = lock or threading.RLock()
        self._token: object | None = None
        self._cancel_requested = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._token is not None

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    @property
    def token(self) -> object | None:
        with self._lock:
            return self._token

    def try_start(self, token: object) -> bool:
        with self._lock:
            if self._token is not None:
                return False
            self._token = token
            self._cancel_requested = False
            return True

    def stop(self) -> bool:
        with self._lock:
            if self._token is None:
                return False
            self._cancel_requested = True
            return True

    def finish(self, token: object) -> None:
        with self._lock:
            if self._token is token:
                self._token = None
                self._cancel_requested = False


class GateOutcome(Enum):
    APPLIED = auto()
    ACTION_CANCEL = auto()
    SERVICE_STOP = auto()
    INACTIVE = auto()
    SUCCEEDED = auto()


def terminal_disposition(outcome: GateOutcome, action_cancel_requested: bool) -> str:
    if outcome is GateOutcome.ACTION_CANCEL and bool(action_cancel_requested):
        return "canceled"
    return "aborted"


class TrajectoryCommandGate:
    """Atomically arbitrate trajectory commands and stop/hold operations."""

    def __init__(self, active: ActiveTrajectory) -> None:
        self._active = active

    def apply_if_active(self, token, cancel_requested, apply, hold) -> bool:
        return self.apply_with_reason(token, cancel_requested, apply, hold) is GateOutcome.APPLIED

    def apply_with_reason(self, token, action_cancel_requested, apply, hold) -> GateOutcome:
        with self._active._lock:
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            apply()
            return GateOutcome.APPLIED

    def stop_and_hold(self, hold) -> bool:
        with self._active._lock:
            stopped = self._active._token is not None
            if stopped:
                self._active._cancel_requested = True
                hold()
            return stopped

    def complete_if_active(self, token, cancel_requested, hold, succeed) -> bool:
        return self.complete_with_reason(
            token, cancel_requested, hold, succeed
        ) is GateOutcome.SUCCEEDED

    def complete_with_reason(self, token, action_cancel_requested, hold, succeed) -> GateOutcome:
        """Linearize cancellation versus the terminal success transition."""
        with self._active._lock:
            # Evaluate the external hook first, then re-read internal state: a
            # cancel callback may mark the goal through a reentrant test hook
            # or immediately before this critical section.
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            # No simulation lock is held while calling the ROS transition.
            succeed()
            self._active._token = None
            self._active._cancel_requested = False
            return GateOutcome.SUCCEEDED


class ExecutionLifecycle:
    """Best-effort failure cleanup that never leaks an admitted goal token."""

    def __init__(self, active: ActiveTrajectory, gate: TrajectoryCommandGate) -> None:
        self._active = active
        self._gate = gate

    def fail(self, token, hold, abort) -> None:
        try:
            self._gate.stop_and_hold(hold)
        except Exception:
            pass
        try:
            abort()
        finally:
            self._active.finish(token)


class SerializedSimulationAccess:
    """Serialize every operation touching one simulation instance."""

    def __init__(self, simulation: Any, lock: threading.RLock | None = None) -> None:
        self._simulation = simulation
        self._lock = lock or threading.RLock()

    def run(self, operation):
        with self._lock:
            return operation(self._simulation)


def create_node_class():
    """Import ROS lazily and return the concrete node class."""
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import TransformStamped
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock as RclpyClock
    from rclpy.clock import ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rebotarm_msgs.msg import Detection2D, Detection2DArray, JointMotorState
    from rebotarm_msgs.srv import SetGripper
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from trajectory_msgs.msg import JointTrajectoryPoint

    from .mujoco_sim import RebotArmMujoco

    class RebotArmMujocoNode(Node):
        def __init__(self) -> None:
            super().__init__("rebotarm_mujoco_node")
            self.declare_parameter("backend", "mujoco")
            self.declare_parameter("headless", True)
            self.declare_parameter("model_path", "")
            self.declare_parameter("arm_namespace", "rebotarm")
            self.declare_parameter("publish_rate_hz", 30.0)
            self.declare_parameter("max_trajectory_points", DEFAULT_MAX_TRAJECTORY_POINTS)
            self.declare_parameter("max_trajectory_duration_sec", DEFAULT_MAX_TRAJECTORY_DURATION_SEC)
            self.declare_parameter("initial_joint_positions", [0.0] * 6)
            self.declare_parameter("goal_position_tolerance", 0.02)
            self.declare_parameter("goal_velocity_tolerance", 0.05)
            self.declare_parameter("goal_time_tolerance_sec", 5.0)
            self.declare_parameter("feedback_rate_hz", 20.0)
            self.declare_parameter("virtual_camera.enabled", False)
            self.declare_parameter("virtual_camera.camera_name", "fixed_camera")
            self.declare_parameter(
                "virtual_camera.frame_id", "mujoco_fixed_camera_optical_frame"
            )
            self.declare_parameter("virtual_camera.parent_body_name", "base_link")
            self.declare_parameter("virtual_camera.parent_frame_id", "base_link")
            self.declare_parameter("virtual_camera.width", 640)
            self.declare_parameter("virtual_camera.height", 480)
            self.declare_parameter("virtual_camera.rate_hz", 15.0)
            self.declare_parameter("virtual_camera.max_depth_m", 2.0)
            self.declare_parameter("virtual_camera.annotation_bodies", ["bottle"])
            self.declare_parameter(
                "virtual_camera.annotation_topic", "/grasp/ground_truth_detections"
            )
            if self.get_parameter("backend").value != "mujoco":
                raise ValueError("simulation backend must be mujoco")
            if self.get_parameter("headless").value is not True:
                raise ValueError("ROS adapter is headless; run the viewer separately")

            namespace = str(self.get_parameter("arm_namespace").value).strip("/")
            if not namespace or any(part in namespace for part in ("//", " ")):
                raise ValueError("arm namespace is invalid")
            self._arm_namespace = namespace
            rate = float(self.get_parameter("publish_rate_hz").value)
            if not math.isfinite(rate) or rate <= 0.0 or rate > 1000.0:
                raise ValueError("publish rate must be finite and in (0, 1000]")
            self._max_points = int(self.get_parameter("max_trajectory_points").value)
            self._max_duration = float(self.get_parameter("max_trajectory_duration_sec").value)
            # Per-goal JointTolerance arrays are intentionally not interpreted:
            # this simulation backend uses these bounded node-wide defaults so
            # clients cannot weaken settling guarantees on individual goals.
            self._settling_policy = GoalSettlingPolicy(
                float(self.get_parameter("goal_position_tolerance").value),
                float(self.get_parameter("goal_velocity_tolerance").value),
                float(self.get_parameter("goal_time_tolerance_sec").value),
            )
            self._feedback_rate_hz = float(self.get_parameter("feedback_rate_hz").value)
            # Constructor performs finite/positive/range validation.
            FeedbackRateLimiter(self._feedback_rate_hz)
            self._execute_wait_sec = min(0.01, 1.0 / self._feedback_rate_hz)
            initial = tuple(float(v) for v in self.get_parameter("initial_joint_positions").value)
            if len(initial) != 6 or any(not math.isfinite(v) for v in initial):
                raise ValueError("initial positions must contain six finite values")

            model_path = str(self.get_parameter("model_path").value).strip()
            self._sim = RebotArmMujoco(model_path or None)
            self._lock = threading.RLock()
            self._sim_access = SerializedSimulationAccess(self._sim, self._lock)
            self._sim_access.run(lambda sim: sim.reset_joint_positions(initial))
            # Gate/active lock is intentionally distinct. Lock order is always
            # gate first, then simulation; the timer only takes simulation.
            self._active = ActiveTrajectory()
            self._command_gate = TrajectoryCommandGate(self._active)
            self._lifecycle = ExecutionLifecycle(self._active, self._command_gate)
            self._pending_sampler: TrajectorySampler | None = None
            self._callback_group = ReentrantCallbackGroup()
            self._timer_callback_group = MutuallyExclusiveCallbackGroup()
            self._physics_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
            self._stamp = MonotonicStamp()
            self._virtual_camera_worker = None
            self._virtual_camera_config = None
            self._next_virtual_camera_time = 0.0
            if bool(self.get_parameter("virtual_camera.enabled").value):
                self._virtual_camera_config = VirtualCameraConfig(
                    camera_name=str(
                        self.get_parameter("virtual_camera.camera_name").value
                    ),
                    frame_id=str(self.get_parameter("virtual_camera.frame_id").value),
                    parent_body_name=str(
                        self.get_parameter("virtual_camera.parent_body_name").value
                    ),
                    parent_frame_id=str(
                        self.get_parameter("virtual_camera.parent_frame_id").value
                    ),
                    width=int(self.get_parameter("virtual_camera.width").value),
                    height=int(self.get_parameter("virtual_camera.height").value),
                    rate_hz=float(self.get_parameter("virtual_camera.rate_hz").value),
                    max_depth_m=float(
                        self.get_parameter("virtual_camera.max_depth_m").value
                    ),
                    annotation_bodies=tuple(
                        self.get_parameter("virtual_camera.annotation_bodies").value
                    ),
                )

            self._joint_pub = self.create_publisher(
                JointState, f"/{self._arm_namespace}/joint_states", 10
            )
            self._gripper_pub = self.create_publisher(
                JointMotorState, f"/{self._arm_namespace}/gripper/state", 10
            )
            self._clock_pub = self.create_publisher(Clock, "/clock", 10)
            self._color_pub = None
            self._depth_pub = None
            self._color_info_pub = None
            self._depth_info_pub = None
            self._annotation_pub = None
            self._virtual_camera_tf_broadcaster = None
            if self._virtual_camera_config is not None:
                self._color_pub = self.create_publisher(
                    Image, "/camera/color/image_raw", qos_profile_sensor_data
                )
                self._depth_pub = self.create_publisher(
                    Image, "/camera/depth/image_raw", qos_profile_sensor_data
                )
                self._color_info_pub = self.create_publisher(
                    CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data
                )
                self._depth_info_pub = self.create_publisher(
                    CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data
                )
                annotation_topic = str(
                    self.get_parameter("virtual_camera.annotation_topic").value
                ).strip()
                if not annotation_topic:
                    raise ValueError("virtual_camera.annotation_topic must be non-empty")
                self._annotation_pub = self.create_publisher(
                    Detection2DArray, annotation_topic, qos_profile_sensor_data
                )
                self._virtual_camera_tf_broadcaster = StaticTransformBroadcaster(self)
                self._virtual_camera_worker = VirtualCameraWorker(
                    self._sim_access,
                    self._virtual_camera_config,
                    on_frame=self._publish_virtual_frame,
                    on_ready=self._virtual_camera_ready,
                    on_error=self._virtual_camera_error,
                )
                self.get_logger().info(
                    "MuJoCo virtual RGB-D configured: "
                    f"camera={self._virtual_camera_config.camera_name}, "
                    f"size={self._virtual_camera_config.width}x"
                    f"{self._virtual_camera_config.height}, "
                    f"rate={self._virtual_camera_config.rate_hz:g} Hz, "
                    f"frame={self._virtual_camera_config.frame_id}"
                )
            self._action_server = ActionServer(
                self,
                FollowJointTrajectory,
                f"/{self._arm_namespace}/follow_joint_trajectory",
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                execute_callback=self._execute_goal,
                callback_group=self._callback_group,
            )
            self.create_service(
                Trigger,
                f"/{self._arm_namespace}/trajectory_stop",
                self._stop_service,
                callback_group=self._callback_group,
            )
            self.create_service(
                SetGripper,
                f"/{self._arm_namespace}/gripper/set",
                self._gripper_service,
                callback_group=self._callback_group,
            )
            # Each callback advances enough fixed physics steps to match the
            # configured publication period; simulation time remains the
            # authoritative trajectory clock.
            self._steps_per_tick = max(1, round((1.0 / rate) / self._sim.timestep))
            self.create_timer(
                1.0 / rate,
                self._timer_callback,
                callback_group=self._timer_callback_group,
                clock=self._physics_clock,
            )

        def _current_arm_positions(self) -> tuple[float, ...]:
            return self._sim_access.run(
                lambda sim: tuple(sim.get_state().joint_positions[:6])
            )

        def _hold_current_position(self) -> None:
            self._sim_access.run(lambda _sim: self._hold_current_position_unlocked())

        def _hold_current_position_unlocked(self) -> None:
            current = tuple(self._sim.get_state().joint_positions[:6])
            self._sim.set_joint_position_targets(current)

        def _apply_target_threadsafe(self, operation) -> None:
            self._sim_access.run(lambda _sim: operation())

        def _goal_callback(self, goal_request):
            token = goal_request
            try:
                sampler = trajectory_to_sampler(
                    goal_request.trajectory,
                    initial_positions=self._current_arm_positions(),
                    max_points=self._max_points,
                    max_duration_sec=self._max_duration,
                )
            except (TypeError, ValueError):
                self.get_logger().warning("rejected invalid trajectory goal")
                return GoalResponse.REJECT
            if not self._active.try_start(token):
                self.get_logger().warning("rejected trajectory goal while controller is busy")
                return GoalResponse.REJECT
            self._pending_sampler = sampler
            return GoalResponse.ACCEPT

        def _cancel_callback(self, _goal_handle):
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            return CancelResponse.ACCEPT if stopped else CancelResponse.REJECT

        def _stop_service(self, _request, response):
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            response.success = True
            response.message = "simulation trajectory stop requested" if stopped else "no active trajectory"
            return response

        def _gripper_service(self, request, response):
            try:
                width = validate_gripper_width(request.position)
                reached = self._sim_access.run(lambda sim: sim.set_gripper_width(width))
            except (TypeError, ValueError):
                response.success = False
                response.reached_position = 0.0
                return response
            response.success = True
            response.reached_position = float(reached)
            return response

        @staticmethod
        def _terminate_goal(goal_handle, result, outcome):
            disposition = terminal_disposition(outcome, goal_handle.is_cancel_requested)
            if disposition == "canceled":
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "simulation trajectory canceled"
                return result
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            if outcome is GateOutcome.SERVICE_STOP:
                result.error_string = "simulation trajectory stopped by service"
            else:
                result.error_string = "simulation trajectory is no longer active"
            return result

        def _execute_goal(self, goal_handle):
            # rclpy does not promise that the goal-request wrapper passed to
            # admission and the one exposed by the goal handle are identical.
            # Preserve the admission token so cleanup cannot leave the server
            # permanently busy after an otherwise valid goal.
            token = self._active.token
            result = FollowJointTrajectory.Result()
            sampler, self._pending_sampler = self._pending_sampler, None
            if sampler is None:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory was not admitted"
                goal_handle.abort()
                self._active.finish(token)
                return result
            try:
                start_time = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
                feedback_limiter = FeedbackRateLimiter(self._feedback_rate_hz)
                while rclpy.ok():
                    command: dict[str, Any] = {}

                    def apply_target() -> None:
                        state = self._sim.get_state()
                        elapsed = max(0.0, state.simulation_time - start_time)
                        desired = sampler.sample(min(elapsed, sampler.duration))
                        self._sim.set_joint_position_targets(desired)
                        command.update(state=state, elapsed=elapsed, desired=desired)

                    outcome = self._command_gate.apply_with_reason(
                        token,
                        lambda: goal_handle.is_cancel_requested,
                        lambda: self._apply_target_threadsafe(apply_target),
                        self._hold_current_position,
                    )
                    if outcome is not GateOutcome.APPLIED:
                        return self._terminate_goal(goal_handle, result, outcome)
                    state = command["state"]
                    elapsed = command["elapsed"]
                    desired = command["desired"]
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.joint_names = list(ARM_JOINT_NAMES)
                    feedback.desired = JointTrajectoryPoint()
                    feedback.actual = JointTrajectoryPoint()
                    feedback.error = JointTrajectoryPoint()
                    feedback.desired.positions = list(desired)
                    actual = tuple(state.joint_positions[:6])
                    feedback.actual.positions = list(actual)
                    feedback.actual.velocities = list(state.joint_velocities[:6])
                    feedback.error.positions = [d - a for d, a in zip(desired, actual)]
                    settling = self._settling_policy.evaluate(
                        sampler.sample(sampler.duration),
                        actual,
                        tuple(state.joint_velocities[:6]),
                        max(0.0, elapsed - sampler.duration),
                    ) if elapsed >= sampler.duration else "tracking"
                    if feedback_limiter.should_publish(
                        time.monotonic(), final=settling in ("succeeded", "timed_out")
                    ):
                        goal_handle.publish_feedback(feedback)
                    if settling == "succeeded":
                        completion = self._command_gate.complete_with_reason(
                            token,
                            lambda: goal_handle.is_cancel_requested,
                            self._hold_current_position,
                            goal_handle.succeed,
                        )
                        if completion is not GateOutcome.SUCCEEDED:
                            return self._terminate_goal(goal_handle, result, completion)
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "simulation trajectory finished"
                        return result
                    if settling == "timed_out":
                        self._command_gate.stop_and_hold(self._hold_current_position)
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = "simulation goal did not settle within configured tolerance"
                        return result
                    time.sleep(self._execute_wait_sec)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation shutting down"
                return result
            except Exception as exc:
                self.get_logger().error(
                    f"trajectory execution exception: {type(exc).__name__}: {exc}"
                )
                self._lifecycle.fail(
                    token,
                    self._hold_current_position,
                    lambda: goal_handle.abort() if getattr(goal_handle, "is_active", True) else None,
                )
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation trajectory execution failed"
                return result
            finally:
                self._active.finish(token)

        def _timer_callback(self) -> None:
            state = self._sim_access.run(lambda sim: sim.step(self._steps_per_tick))
            stamp = Clock()
            seconds, nanoseconds = self._stamp.update(state.simulation_time)
            stamp.clock.sec = seconds
            stamp.clock.nanosec = nanoseconds
            self._clock_pub.publish(stamp)

            joint = JointState()
            joint.header.stamp = stamp.clock
            joint.name = list(state.joint_names)
            joint.position = list(state.joint_positions)
            joint.velocity = list(state.joint_velocities)
            joint.effort = list(state.actuator_forces)
            self._joint_pub.publish(joint)

            gripper = JointMotorState()
            gripper.header.stamp = stamp.clock
            gripper.joint_name = "gripper"
            gripper.position = float(state.gripper_width)
            gripper.velocity = 0.0
            gripper.torque = float(sum(abs(v) for v in state.actuator_forces[-2:]))
            gripper.status_code = 0
            self._gripper_pub.publish(gripper)

            self._publish_virtual_camera(state.simulation_time, stamp.clock)

        def _publish_virtual_camera(self, simulation_time: float, stamp) -> None:
            if self._virtual_camera_worker is None or self._virtual_camera_config is None:
                return
            if simulation_time + 1e-12 < self._next_virtual_camera_time:
                return
            period = 1.0 / self._virtual_camera_config.rate_hz
            self._next_virtual_camera_time += period
            if self._next_virtual_camera_time <= simulation_time:
                skipped = math.floor(
                    (simulation_time - self._next_virtual_camera_time) / period
                ) + 1
                self._next_virtual_camera_time += skipped * period
            self._virtual_camera_worker.submit((stamp.sec, stamp.nanosec))

        def _publish_virtual_frame(self, frame, intrinsics, stamp_parts) -> None:
            stamp_sec, stamp_nanosec = stamp_parts
            color = Image()
            color.header.stamp.sec = stamp_sec
            color.header.stamp.nanosec = stamp_nanosec
            color.header.frame_id = self._virtual_camera_config.frame_id
            color.height = self._virtual_camera_config.height
            color.width = self._virtual_camera_config.width
            color.encoding = "rgb8"
            color.is_bigendian = 0
            color.step = self._virtual_camera_config.width * 3
            color.data = frame.rgb.tobytes()
            self._color_pub.publish(color)

            depth = Image()
            depth.header.stamp.sec = stamp_sec
            depth.header.stamp.nanosec = stamp_nanosec
            depth.header.frame_id = self._virtual_camera_config.frame_id
            depth.height = self._virtual_camera_config.height
            depth.width = self._virtual_camera_config.width
            depth.encoding = "mono16"
            depth.is_bigendian = 0
            depth.step = self._virtual_camera_config.width * 2
            depth.data = frame.depth_mm.tobytes()
            self._depth_pub.publish(depth)

            self._color_info_pub.publish(
                self._camera_info_message(stamp_parts, intrinsics)
            )
            self._depth_info_pub.publish(
                self._camera_info_message(stamp_parts, intrinsics)
            )

            annotations = Detection2DArray()
            annotations.header.stamp.sec = stamp_sec
            annotations.header.stamp.nanosec = stamp_nanosec
            annotations.header.frame_id = self._virtual_camera_config.frame_id
            for item in frame.annotations:
                detection = Detection2D()
                detection.header.stamp.sec = stamp_sec
                detection.header.stamp.nanosec = stamp_nanosec
                detection.header.frame_id = self._virtual_camera_config.frame_id
                detection.class_name = item.class_name
                detection.confidence = 1.0
                detection.center_u = item.center_u
                detection.center_v = item.center_v
                detection.x_min = item.x_min
                detection.y_min = item.y_min
                detection.x_max = item.x_max
                detection.y_max = item.y_max
                detection.has_obb = False
                detection.obb_points_xy = []
                detection.has_mask = True
                detection.mask_polygon_xy = list(item.mask_polygon_xy)
                annotations.detections.append(detection)
            self._annotation_pub.publish(annotations)

        def _camera_info_message(self, stamp_parts, intrinsics):
            message = CameraInfo()
            message.header.stamp.sec = stamp_parts[0]
            message.header.stamp.nanosec = stamp_parts[1]
            message.header.frame_id = self._virtual_camera_config.frame_id
            message.height = intrinsics.height
            message.width = intrinsics.width
            message.distortion_model = "plumb_bob"
            message.d = [0.0] * 5
            message.k = [
                intrinsics.fx, 0.0, intrinsics.cx,
                0.0, intrinsics.fy, intrinsics.cy,
                0.0, 0.0, 1.0,
            ]
            message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            message.p = [
                intrinsics.fx, 0.0, intrinsics.cx, 0.0,
                0.0, intrinsics.fy, intrinsics.cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            return message

        def _virtual_camera_ready(self, intrinsics, extrinsics) -> None:
            transform = TransformStamped()
            transform.header.frame_id = extrinsics.parent_frame_id
            transform.child_frame_id = extrinsics.child_frame_id
            transform.transform.translation.x = extrinsics.translation_xyz[0]
            transform.transform.translation.y = extrinsics.translation_xyz[1]
            transform.transform.translation.z = extrinsics.translation_xyz[2]
            transform.transform.rotation.x = extrinsics.rotation_xyzw[0]
            transform.transform.rotation.y = extrinsics.rotation_xyzw[1]
            transform.transform.rotation.z = extrinsics.rotation_xyzw[2]
            transform.transform.rotation.w = extrinsics.rotation_xyzw[3]
            self._virtual_camera_tf_broadcaster.sendTransform(transform)
            self.get_logger().info(
                "MuJoCo virtual RGB-D renderer ready: "
                f"fx={intrinsics.fx:.3f}, fy={intrinsics.fy:.3f}, "
                f"cx={intrinsics.cx:.3f}, cy={intrinsics.cy:.3f}, "
                f"tf={extrinsics.parent_frame_id}->{extrinsics.child_frame_id}"
            )

        def _virtual_camera_error(self, exc: BaseException) -> None:
            self.get_logger().error(
                "MuJoCo virtual camera worker failed: "
                f"{type(exc).__name__}: {exc}"
            )

        def destroy_node(self):
            self._action_server.destroy()
            if self._virtual_camera_worker is not None:
                stopped = self._virtual_camera_worker.close()
                if not stopped:
                    self.get_logger().error(
                        "MuJoCo virtual camera worker did not stop within timeout"
                    )
            self._sim_access.run(lambda sim: sim.close())
            return super().destroy_node()

    return RebotArmMujocoNode


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = create_node_class()()
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

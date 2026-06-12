from __future__ import annotations

from copy import deepcopy
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan

from .candidate_filter_policy import filter_candidate_array_by_reachability
from .candidate_gate_policy import CandidateGateConfig, evaluate_candidate_gate
from .candidate_motion_policy import JointMotionPolicyConfig, evaluate_joint_motion
from .candidate_scoring_policy import CandidateScoringInput, score_candidate
from .candidate_target_policy import CandidateTargetPolicyConfig, build_candidate_target_variants
from .candidate_tf_adapter import transform_candidate_pose_to_target_frame
from .motion_feasibility_policy import evaluate_motion_feasibility
from .pose_variant_policy import PoseVariantConfig
from .visual_grasp_sequence import PoseTarget


def _pose_from_target(target: PoseTarget) -> Pose:
    pose = Pose()
    pose.position.x = float(target.position[0])
    pose.position.y = float(target.position[1])
    pose.position.z = float(target.position[2])
    pose.orientation.x = float(target.orientation[0])
    pose.orientation.y = float(target.orientation[1])
    pose.orientation.z = float(target.orientation[2])
    pose.orientation.w = float(target.orientation[3])
    return pose


class CandidateIkFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_candidate_ik_filter")
        self._callback_group = ReentrantCallbackGroup()
        self.declare_parameter("input_topic", "/grasp/candidates")
        self.declare_parameter("output_topic", "/grasp/filtered_candidates")
        self.declare_parameter("output_plan_topic", "/grasp/filtered_plan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("moveit_ik_service", "/compute_ik")
        self.declare_parameter("joint_state_topic", "/rebotarm/visual_joint_states")
        self.declare_parameter("collision_check_enabled", True)
        self.declare_parameter("collision_check_service", "/check_state_validity")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("pose_policy", "hybrid_geometry_with_base_axis_fallback")
        self.declare_parameter("fixed_grasp_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("base_approach_axis_xyz", [1.0, 0.0, 0.0])
        self.declare_parameter("base_pregrasp_distance_m", 0.08)
        self.declare_parameter("orientation_yaw_offsets_rad", [0.0])
        self.declare_parameter("candidate_grasp_z_offsets_m", [0.0])
        self.declare_parameter("max_candidates_per_frame", 20)
        self.declare_parameter("lift_z_m", 0.08)
        self.declare_parameter("tcp_offset_xyz", [-0.04, 0.0, 0.0])
        self.declare_parameter("target_base_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pregrasp_base_z_offset_m", 0.05)
        self.declare_parameter("candidate_pregrasp_min_z_m", 0.120)
        self.declare_parameter("grasp_base_z_offset_m", 0.0)
        self.declare_parameter("candidate_min_jaw_width_m", 0.006)
        self.declare_parameter("candidate_max_jaw_width_m", 0.082)
        self.declare_parameter("candidate_min_grasp_z_m", 0.0)
        self.declare_parameter("candidate_safe_lift_min_z_m", 0.120)
        self.declare_parameter("candidate_workspace_gate_enabled", False)
        self.declare_parameter("candidate_workspace_min_xyz", [0.18, -0.35, 0.0])
        self.declare_parameter("candidate_workspace_max_xyz", [0.64, 0.35, 0.45])
        self.declare_parameter("candidate_max_grasp_to_object_center_m", 0.15)
        self.declare_parameter("candidate_score_joint_distance_weight", 0.15)
        self.declare_parameter("candidate_score_joint6_weight", 0.35)
        self.declare_parameter("candidate_max_joint6_delta_rad", 1.5708)
        self.declare_parameter("candidate_joint6_symmetry_enabled", True)
        self.declare_parameter("candidate_joint6_symmetry_angle_rad", math.pi)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self._latest_joint_state: JointState | None = None
        self._warned_missing_joint_state = False
        self._filter_busy = False
        self._warned_filter_busy = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ik_client = self.create_client(
            GetPositionIK,
            str(self.get_parameter("moveit_ik_service").value),
            callback_group=self._callback_group,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
            callback_group=self._callback_group,
        )
        self._candidates_pub = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._plan_pub = self.create_publisher(
            GraspPlan,
            str(self.get_parameter("output_plan_topic").value),
            10,
        )
        self.create_subscription(
            GraspCandidateArray,
            self._input_topic,
            self._on_candidates,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "candidate IK filter ready: "
            f"input={self._input_topic}, output={str(self.get_parameter('output_topic').value)}, "
            f"plan={str(self.get_parameter('output_plan_topic').value)}"
        )

    def _tuple3(self, name: str) -> tuple[float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return (float(values[0]), float(values[1]), float(values[2]))

    def _tuple4(self, name: str) -> tuple[float, float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly 4 values")
        return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))

    def _on_joint_state(self, msg: JointState) -> None:
        if self._valid_joint_state(msg):
            self._latest_joint_state = msg
            self._warned_missing_joint_state = False
            return
        if not self._warned_missing_joint_state:
            self.get_logger().warn(
                "candidate IK filter ignored empty or incomplete joint state; "
                "waiting before calling MoveIt IK"
            )
            self._warned_missing_joint_state = True

    def _valid_joint_state(self, msg: JointState | None) -> bool:
        if msg is None:
            return False
        if not msg.name or not msg.position:
            return False
        if len(msg.position) < len(msg.name):
            return False
        return all(math.isfinite(float(value)) for value in msg.position[: len(msg.name)])

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        if self._filter_busy:
            if not self._warned_filter_busy:
                self.get_logger().warn(
                    "candidate IK filter is still processing previous candidates; dropping this frame"
                )
                self._warned_filter_busy = True
            return
        self._filter_busy = True
        try:
            self._on_candidates_unlocked(msg)
        finally:
            self._filter_busy = False
            self._warned_filter_busy = False

    def _on_candidates_unlocked(self, msg: GraspCandidateArray) -> None:
        if not msg.candidates:
            self._publish_filtered(msg, [])
            return
        ranked: list[tuple[float, int, object, tuple[PoseTarget, PoseTarget], str, str]] = []
        max_candidates = max(1, int(self.get_parameter("max_candidates_per_frame").value))
        for original_index, candidate in enumerate(msg.candidates[:max_candidates]):
            try:
                best: tuple[float, tuple[PoseTarget, PoseTarget], str, str] | None = None
                variants = self._candidate_target_variants(msg, candidate.pose)
                for pregrasp, grasp, variant_label in variants:
                    feasibility = evaluate_motion_feasibility(
                        pregrasp=pregrasp,
                        grasp=grasp,
                        variant_label=variant_label,
                        check_target=self._check_ik_and_collision,
                        motion_penalty=self._joint_motion_penalty,
                    )
                    if not feasibility.accepted or feasibility.motion_penalty is None:
                        continue
                    if not self._candidate_gate_allows(candidate, grasp=grasp):
                        continue
                    scoring = score_candidate(
                        CandidateScoringInput(
                            original_index=original_index,
                            variant_label=variant_label,
                            motion_penalty=feasibility.motion_penalty,
                        )
                    )
                    score_value = scoring.score
                    if best is None or float(score_value) > float(best[0]):
                        best = (float(score_value), (pregrasp, grasp), variant_label, feasibility.reason)
                if best is not None:
                    score_value, targets, label, motion_reason = best
                    ranked.append((score_value, original_index, candidate, targets, label, motion_reason))
                    self.get_logger().info(
                        f"candidate IK filter accepted candidate={original_index} {label}: "
                        f"score={score_value:.2f}, {motion_reason}"
                    )
            except Exception as exc:
                self.get_logger().warn(f"candidate IK filter rejected candidate: {exc}")
        self._publish_ranked(msg, ranked)

    def _candidate_gate_allows(self, candidate, *, grasp: PoseTarget) -> bool:
        try:
            workspace_enabled = bool(self.get_parameter("candidate_workspace_gate_enabled").value)
        except Exception:
            workspace_enabled = False
        object_center_xyz = None
        workspace_min_xyz = (0.18, -0.35, 0.0)
        workspace_max_xyz = (0.64, 0.35, 0.45)
        max_grasp_to_object_center_m = 0.15
        if workspace_enabled:
            object_center_xyz = self._candidate_object_center_in_target_frame(candidate)
            workspace_min_xyz = self._tuple3("candidate_workspace_min_xyz")
            workspace_max_xyz = self._tuple3("candidate_workspace_max_xyz")
            max_grasp_to_object_center_m = float(
                self.get_parameter("candidate_max_grasp_to_object_center_m").value
            )
        result = evaluate_candidate_gate(
            jaw_width_m=float(getattr(candidate, "jaw_width", 0.0)),
            grasp_position_xyz=tuple(float(v) for v in grasp.position),
            object_center_xyz=object_center_xyz,
            config=CandidateGateConfig(
                min_jaw_width_m=float(self.get_parameter("candidate_min_jaw_width_m").value),
                max_jaw_width_m=float(self.get_parameter("candidate_max_jaw_width_m").value),
                min_grasp_z_m=float(self.get_parameter("candidate_min_grasp_z_m").value),
                workspace_gate_enabled=workspace_enabled,
                workspace_min_xyz=workspace_min_xyz,
                workspace_max_xyz=workspace_max_xyz,
                max_grasp_to_object_center_m=max_grasp_to_object_center_m,
            ),
        )
        if not result.accepted:
            self.get_logger().warn(f"candidate IK filter rejected reachable grasp: {result.reason}")
            return False
        return True

    def _candidate_safety_gate(self, candidate, *, grasp: PoseTarget) -> bool:
        return CandidateIkFilterNode._candidate_gate_allows(self, candidate, grasp=grasp)

    def _candidate_object_center_in_target_frame(self, candidate) -> tuple[float, float, float] | None:
        pose = getattr(candidate, "pose", None)
        if pose is None:
            return None
        source_frame = str(getattr(getattr(candidate, "header", None), "frame_id", "") or "")
        try:
            object_pose = self._transform_pose_to_target_frame(pose, source_frame)
        except Exception:
            return None
        return (
            float(object_pose.position.x),
            float(object_pose.position.y),
            float(object_pose.position.z),
        )

    def _lookup_transform(self, target_frame: str, source_frame: str):
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2),
        )

    def _transform_pose_to_target_frame(self, pose: Pose, source_frame: str) -> Pose:
        return transform_candidate_pose_to_target_frame(
            pose,
            source_frame=source_frame,
            target_frame=self._target_frame,
            lookup_transform=self._lookup_transform,
        )

    def _list_float_parameter(self, name: str) -> list[float]:
        values = list(self.get_parameter(name).value)
        return [float(value) for value in values]

    def _candidate_target_variants(
        self,
        candidates: GraspCandidateArray,
        pose: Pose,
    ) -> list[tuple[PoseTarget, PoseTarget, str]]:
        source_frame = str(candidates.header.frame_id)
        grasp_pose = self._transform_pose_to_target_frame(pose, source_frame)
        position_xyz = (
            float(grasp_pose.position.x),
            float(grasp_pose.position.y),
            float(grasp_pose.position.z),
        )
        variants = build_candidate_target_variants(
            grasp_position_xyz=position_xyz,
            candidate_orientation_xyzw=(
                float(grasp_pose.orientation.x),
                float(grasp_pose.orientation.y),
                float(grasp_pose.orientation.z),
                float(grasp_pose.orientation.w),
            ),
            config=CandidateTargetPolicyConfig(
                pose_policy=str(self.get_parameter("pose_policy").value),
                fixed_grasp_orientation_xyzw=self._tuple4("fixed_grasp_orientation_xyzw"),
                base_approach_axis_xyz=self._tuple3("base_approach_axis_xyz"),
                base_pregrasp_distance_m=float(self.get_parameter("base_pregrasp_distance_m").value),
                tcp_offset_xyz=self._tuple3("tcp_offset_xyz"),
                target_base_offset_xyz=self._tuple3("target_base_offset_xyz"),
                pregrasp_base_z_offset_m=float(self.get_parameter("pregrasp_base_z_offset_m").value),
                pregrasp_min_z_m=float(self.get_parameter("candidate_pregrasp_min_z_m").value),
                grasp_base_z_offset_m=float(self.get_parameter("grasp_base_z_offset_m").value),
                orientation_yaw_offsets_rad=tuple(self._list_float_parameter("orientation_yaw_offsets_rad")),
                candidate_grasp_z_offsets_m=tuple(self._list_float_parameter("candidate_grasp_z_offsets_m")),
                pose_variant_config=PoseVariantConfig(
                    joint6_symmetry_enabled=bool(self.get_parameter("candidate_joint6_symmetry_enabled").value),
                    joint6_symmetry_angle_rad=float(self.get_parameter("candidate_joint6_symmetry_angle_rad").value),
                ),
            ),
        )
        return [(variant.pregrasp, variant.grasp, variant.label) for variant in variants]

    def _check_ik_and_collision(self, target: PoseTarget, label: str):
        solution = self._solve_ik(target, label)
        if solution is None:
            return None
        if not self._check_state_validity(solution, label):
            return None
        return solution

    def _joint_motion_penalty(self, robot_state) -> tuple[float | None, str]:
        current = self._joint_positions_by_name(self._latest_joint_state)
        solution_joint_state = getattr(robot_state, "joint_state", None)
        target = self._joint_positions_by_name(solution_joint_state)
        common_names = [name for name in current if name in target and name.startswith("joint")]
        if not common_names:
            return 0.0, "joint_delta=unknown"
        evaluation = evaluate_joint_motion(
            current_positions=current,
            target_positions=target,
            config=JointMotionPolicyConfig(
                joint_distance_weight=float(self.get_parameter("candidate_score_joint_distance_weight").value),
                joint6_weight=float(self.get_parameter("candidate_score_joint6_weight").value),
                max_joint6_delta_rad=float(self.get_parameter("candidate_max_joint6_delta_rad").value),
            ),
        )
        if not evaluation.accepted:
            self.get_logger().warn(
                "candidate IK filter rejected reachable grasp: "
                f"joint6 delta too large "
                f"({evaluation.joint6_delta:.3f}rad > "
                f"{float(self.get_parameter('candidate_max_joint6_delta_rad').value):.3f}rad)"
            )
            return None, evaluation.reason
        return evaluation.penalty, evaluation.reason

    def _joint_positions_by_name(self, joint_state: JointState | None) -> dict[str, float]:
        if joint_state is None:
            return {}
        names = list(getattr(joint_state, "name", []))
        positions = list(getattr(joint_state, "position", []))
        return {
            str(name): float(position)
            for name, position in zip(names, positions)
            if math.isfinite(float(position))
        }

    def _target_debug_text(self, target: PoseTarget) -> str:
        return (
            f"target=({target.position[0]:.3f}, {target.position[1]:.3f}, {target.position[2]:.3f}), "
            f"orientation=({target.orientation[0]:.4f}, {target.orientation[1]:.4f}, "
            f"{target.orientation[2]:.4f}, {target.orientation[3]:.4f})"
        )

    def _solve_ik(self, target: PoseTarget, label: str):
        if not self._ik_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn("candidate IK filter IK service unavailable")
            return None
        if not self._valid_joint_state(self._latest_joint_state):
            if not self._warned_missing_joint_state:
                self.get_logger().warn(
                    "candidate IK filter has no valid joint state yet; "
                    "skipping IK to avoid empty MoveIt RobotState"
                )
                self._warned_missing_joint_state = True
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = str(self.get_parameter("moveit_group_name").value)
        request.ik_request.ik_link_name = str(self.get_parameter("ee_frame_id").value)
        request.ik_request.robot_state.joint_state = deepcopy(self._latest_joint_state)
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self._target_frame
        request.ik_request.pose_stamped.pose = _pose_from_target(target)
        request.ik_request.avoid_collisions = False
        future = self._ik_client.call_async(request)
        deadline = time.monotonic() + max(self._service_timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(
                f"candidate IK filter IK timed out for {label}: "
                f"{self._target_debug_text(target)}"
            )
            return None
        result = future.result()
        if result is None:
            self.get_logger().warn(f"candidate IK filter IK failed for {label}: empty service result")
            return None
        error_code = int(getattr(result.error_code, "val", 99999))
        if error_code != 1:
            self.get_logger().warn(
                f"candidate IK filter IK failed for {label}: error_code={error_code}, "
                f"{self._target_debug_text(target)}"
            )
            return None
        return getattr(result, "solution", None)

    def _check_state_validity(self, robot_state, label: str) -> bool:
        if not bool(self.get_parameter("collision_check_enabled").value):
            return True
        if robot_state is None:
            return False
        if not self._state_validity_client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn("candidate IK filter state validity service unavailable")
            return False
        request = GetStateValidity.Request()
        request.group_name = str(self.get_parameter("collision_group_name").value)
        request.robot_state = robot_state
        future = self._state_validity_client.call_async(request)
        deadline = time.monotonic() + max(self._service_timeout_sec, 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().warn(f"candidate IK filter state validity timed out for {label}")
            return False
        result = future.result()
        if result is None:
            self.get_logger().warn(f"candidate IK filter state validity failed for {label}: empty service result")
            return False
        valid = bool(getattr(result, "valid", False))
        if not valid:
            self.get_logger().warn(f"candidate IK filter state validity failed for {label}: state invalid")
        return valid

    def _publish_filtered(
        self,
        original: GraspCandidateArray,
        reachable: list[bool],
        reachable_targets: list[tuple[PoseTarget, PoseTarget]] | None = None,
    ) -> None:
        filtered = filter_candidate_array_by_reachability(original, reachable)
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, reachable_targets or [])
        self._plan_pub.publish(plan)

    def _publish_ranked(
        self,
        original: GraspCandidateArray,
        ranked: list[tuple[float, int, object, tuple[PoseTarget, PoseTarget], str, str]],
    ) -> None:
        ranked = sorted(ranked, key=lambda item: (-float(item[0]), int(item[1])))
        filtered = GraspCandidateArray()
        filtered.header = original.header
        filtered.best_index = 0 if ranked else -1
        filtered.candidates = [deepcopy(item[2]) for item in ranked]
        targets = [item[3] for item in ranked]
        self._candidates_pub.publish(filtered)
        plan = self._plan_from_filtered(filtered, targets)
        if ranked:
            score, original_index, _candidate, _targets, label, motion_reason = ranked[0]
            plan.reason = (
                f"best_candidate original_index={original_index}, score={score:.2f}, "
                f"variant={label}, {motion_reason}"
            )
            self.get_logger().info(f"candidate IK filter best: {plan.reason}")
        self._plan_pub.publish(plan)

    def _plan_from_filtered(
        self,
        filtered: GraspCandidateArray,
        reachable_targets: list[tuple[PoseTarget, PoseTarget]],
    ) -> GraspPlan:
        plan = GraspPlan()
        plan.header = filtered.header
        plan.source = "candidate_ik_filter"
        if filtered.best_index < 0 or not filtered.candidates:
            plan.valid = False
            plan.reason = "no IK-reachable grasp candidates"
            return plan
        candidate = filtered.candidates[int(filtered.best_index)]
        plan.candidate = candidate
        if reachable_targets:
            pregrasp, grasp = reachable_targets[int(filtered.best_index)]
            plan.pregrasp_pose = _pose_from_target(pregrasp)
            plan.grasp_pose = _pose_from_target(grasp)
            plan.header.frame_id = self._target_frame
        else:
            plan.grasp_pose = candidate.pose
            plan.pregrasp_pose = candidate.pose
        plan.jaw_width = float(candidate.jaw_width)
        plan.valid = True
        plan.reason = ""
        return plan


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CandidateIkFilterNode()
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

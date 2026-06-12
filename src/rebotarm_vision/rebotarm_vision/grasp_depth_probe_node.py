from __future__ import annotations

import math
from statistics import mean, pstdev
from typing import Optional

import rclpy
from rclpy.node import Node

from rebotarm_msgs.msg import GraspCandidateArray


class GraspDepthProbeNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_grasp_depth_probe")
        self.declare_parameter("input_topic", "/grasp/candidates")
        self.declare_parameter("sample_count", 20)
        self.declare_parameter("min_reliable_depth_m", 0.20)
        self.declare_parameter("absolute_min_depth_m", 0.15)
        self.declare_parameter("max_reliable_depth_m", 5.0)
        self.declare_parameter("max_depth_std_m", 0.02)
        self.declare_parameter("target_class", "")
        self._samples: list[float] = []
        self._last_widths: list[float] = []
        self._last_lengths: list[float] = []
        self._sample_count = max(int(self.get_parameter("sample_count").value), 1)
        self._target_class = str(self.get_parameter("target_class").value).strip()
        self.create_subscription(
            GraspCandidateArray,
            str(self.get_parameter("input_topic").value),
            self._on_candidates,
            10,
        )
        self.get_logger().info(
            "grasp depth probe ready: "
            f"input={self.get_parameter('input_topic').value}, samples={self._sample_count}"
        )

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        candidate = self._select_candidate(msg)
        if candidate is None:
            self.get_logger().warn("no grasp candidate received in this frame")
            return
        z = float(candidate.pose.position.z)
        width = float(candidate.jaw_width)
        length = float(candidate.object_length)
        self._samples.append(z)
        self._last_widths.append(width)
        self._last_lengths.append(length)
        status = self._depth_status(z)
        self.get_logger().info(
            f"sample {len(self._samples)}/{self._sample_count}: "
            f"class={candidate.class_name} conf={float(candidate.confidence):.3f} "
            f"camera_xyz=({float(candidate.pose.position.x):.3f}, "
            f"{float(candidate.pose.position.y):.3f}, {z:.3f}) "
            f"jaw={width:.3f} object_length={length:.3f} depth_status={status}"
        )
        if len(self._samples) >= self._sample_count:
            self._print_summary()
            rclpy.shutdown()

    def _select_candidate(self, msg: GraspCandidateArray):
        if not msg.candidates:
            return None
        if self._target_class:
            for candidate in msg.candidates:
                if str(candidate.class_name) == self._target_class:
                    return candidate
            return None
        best = int(getattr(msg, "best_index", -1))
        if 0 <= best < len(msg.candidates):
            return msg.candidates[best]
        return msg.candidates[0]

    def _depth_status(self, depth_m: float) -> str:
        if not math.isfinite(depth_m) or depth_m <= 0.0:
            return "invalid"
        absolute_min = float(self.get_parameter("absolute_min_depth_m").value)
        min_reliable = float(self.get_parameter("min_reliable_depth_m").value)
        max_reliable = float(self.get_parameter("max_reliable_depth_m").value)
        if depth_m < absolute_min:
            return "too_close_unreliable"
        if depth_m < min_reliable:
            return "near_limit"
        if depth_m > max_reliable:
            return "too_far"
        return "reliable"

    def _print_summary(self) -> None:
        depths = [value for value in self._samples if math.isfinite(value) and value > 0.0]
        if not depths:
            self.get_logger().error("summary: no valid depth samples")
            return
        avg = mean(depths)
        std = pstdev(depths) if len(depths) > 1 else 0.0
        min_depth = min(depths)
        max_depth = max(depths)
        max_std = float(self.get_parameter("max_depth_std_m").value)
        reliable_count = sum(1 for value in depths if self._depth_status(value) == "reliable")
        usable = reliable_count == len(depths) and std <= max_std
        self.get_logger().info(
            "summary: "
            f"samples={len(depths)}, depth_min={min_depth:.3f}, depth_max={max_depth:.3f}, "
            f"depth_mean={avg:.3f}, depth_std={std:.3f}, "
            f"jaw_mean={mean(self._last_widths):.3f}, object_length_mean={mean(self._last_lengths):.3f}, "
            f"gemini2_close_depth_usable={usable}"
        )
        if usable:
            self.get_logger().info("result: depth appears usable for close-range re-localization")
        else:
            self.get_logger().warn(
                "result: depth is not reliable enough for pregrasp re-localization; "
                "prefer one-shot grasping or a farther coarse-pregrasp"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspDepthProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

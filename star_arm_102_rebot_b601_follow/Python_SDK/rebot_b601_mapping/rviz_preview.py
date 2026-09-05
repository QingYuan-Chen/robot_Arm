from __future__ import annotations

import math
import statistics
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

from .leader_reader import LeaderReader
from .mapping import map_virtual_follower
from .models import Baseline, LeaderSample, MappingConfig, load_mapping_config
from .ports import assert_ports_unoccupied


JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


class RvizJointStatePublisher:
    """向隔离 ROS 域发布 RViz 预览关节状态。"""

    def __init__(self, topic: str) -> None:
        import rclpy
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self._joint_state_type = JointState
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self._node = rclpy.create_node("stararm_rebot_mapping_preview")
        self._publisher = self._node.create_publisher(JointState, str(topic), 10)

    def publish(self, names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        message = self._joint_state_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = list(names)
        message.position = list(positions)
        self._publisher.publish(message)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


def _virtual_midpoint_positions(config: MappingConfig) -> tuple[float, ...]:
    positions: list[float] = []
    for joint in config.arm_joints + (config.gripper,):
        if joint.lower_rad is None or joint.upper_rad is None:
            positions.append(0.0)
        else:
            positions.append((float(joint.lower_rad) + float(joint.upper_rad)) / 2.0)
    return tuple(positions)


def _capture_leader_baseline(
    samples: Sequence[LeaderSample],
    config: MappingConfig,
    *,
    captured_at_s: float,
) -> Baseline:
    if not samples:
        raise ValueError("引导臂基线样本不得为空")
    for sample in samples:
        if len(sample.angles_deg) != len(config.leader_ids):
            raise ValueError("引导臂样本必须包含 ID 0..6 的七个角度")
        if not all(math.isfinite(float(value)) for value in sample.angles_deg):
            raise ValueError("引导臂基线包含非有限数值")
    angles = tuple(
        float(statistics.median(sample.angles_deg[index] for sample in samples))
        for index in range(len(config.leader_ids))
    )
    return Baseline(
        captured_at_s=float(captured_at_s),
        leader_angles_deg=angles,
        follower_positions_rad=_virtual_midpoint_positions(config),
    )


def run_rviz_preview(
    *,
    leader_port: str,
    config_path: Path,
    topic: str = "/mapping_preview/joint_states",
    baseline_samples: int = 5,
    rate_hz: float = 20.0,
    max_samples: int | None = None,
    leader_factory: Callable[[str], Any] = LeaderReader,
    publisher_factory: Callable[[str], Any] = RvizJointStatePublisher,
    port_checker: Callable[[Sequence[str]], None] = assert_ports_unoccupied,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    print_fn: Callable[[str], None] = print,
) -> dict[str, Any]:
    """只读取引导臂，并把候选映射发布给 RViz 虚拟从臂。"""

    if baseline_samples < 1:
        raise ValueError("baseline_samples 必须大于零")
    if not math.isfinite(float(rate_hz)) or rate_hz <= 0.0:
        raise ValueError("rate_hz 必须是正有限数值")
    if max_samples is not None and max_samples < 1:
        raise ValueError("max_samples 必须大于零或不设置")

    config = load_mapping_config(Path(config_path))
    port_checker((leader_port,))
    leader = leader_factory(leader_port)
    publisher = None
    interval_s = 1.0 / float(rate_hz)
    published = 0
    try:
        leader.open()
        baseline_window: list[LeaderSample] = []
        for index in range(baseline_samples):
            baseline_window.append(leader.read_sample())
            if index + 1 < baseline_samples:
                sleep(interval_s)
        baseline = _capture_leader_baseline(
            baseline_window,
            config,
            captured_at_s=float(clock()),
        )
        publisher = publisher_factory(topic)
        print_fn("RViz 虚拟从臂预览已启动：仅读取引导臂，不访问真实从臂。")
        print_fn(f"候选方向：{[joint.sign for joint in config.arm_joints]}")

        while max_samples is None or published < max_samples:
            leader_sample = leader.read_sample()
            mapped = map_virtual_follower(leader_sample, baseline, config)
            publisher.publish(JOINT_NAMES, mapped.positions_rad)
            published += 1
            if published == 1 or published % 10 == 0:
                deltas_deg = [round(math.degrees(value), 2) for value in mapped.leader_deltas_rad]
                positions = [round(value, 3) for value in mapped.positions_rad]
                print_fn(
                    f"样本 {published}：引导增量(度)={deltas_deg}；"
                    f"虚拟从臂(弧度)={positions}"
                )
            if max_samples is None or published < max_samples:
                sleep(interval_s)

        return {
            "mode": "rviz-preview",
            "leader_port": str(leader_port),
            "topic": str(topic),
            "published_samples": published,
            "leader_baseline_deg": list(baseline.leader_angles_deg),
            "virtual_baseline_rad": list(baseline.follower_positions_rad[:6]),
            "candidate_signs": [joint.sign for joint in config.arm_joints],
        }
    finally:
        if publisher is not None:
            publisher.close()
        leader.close()

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import GraspPlan
from std_srvs.srv import Trigger

from .visual_grasp_benchmark import classify_failure_stage


@dataclass
class HybridBenchmarkStats:
    total: int = 0
    success: int = 0
    failures_by_stage: dict[str, int] = field(default_factory=dict)
    messages: list[str] = field(default_factory=list)

    def record(self, *, success: bool, message: str) -> None:
        self.total += 1
        self.messages.append(str(message))
        if success:
            self.success += 1
            return
        stage = classify_failure_stage(message)
        self.failures_by_stage[stage] = self.failures_by_stage.get(stage, 0) + 1

    @property
    def success_rate(self) -> float:
        if self.total <= 0:
            return 0.0
        return 100.0 * float(self.success) / float(self.total)

    def summary_lines(self) -> list[str]:
        lines = [
            (
                f"total={self.total} success={self.success} failed={self.total - self.success} "
                f"success_rate={self.success_rate:.1f}%"
            )
        ]
        if self.failures_by_stage:
            lines.append("failed_stage:")
            for stage, count in sorted(self.failures_by_stage.items()):
                lines.append(f"  {stage}: {count}")
        return lines


class HybridGraspSimBenchmark(Node):
    def __init__(
        self,
        *,
        namespace: str,
        plan_topic: str,
        service_timeout_sec: float,
        plan_timeout_sec: float,
    ) -> None:
        super().__init__("rebotarm_hybrid_grasp_sim_benchmark")
        namespace = namespace.strip("/")
        self._plan_topic = str(plan_topic)
        self._service_timeout_sec = float(service_timeout_sec)
        self._plan_timeout_sec = float(plan_timeout_sec)
        self._latest_plan: GraspPlan | None = None
        self._plan_revision = 0

        self._ready_client = self.create_client(Trigger, f"/{namespace}/visual_ready/move")
        self._grasp_client = self.create_client(Trigger, f"/{namespace}/visual_grasp/execute")
        self.create_subscription(GraspPlan, self._plan_topic, self._on_plan, 10)

    def _on_plan(self, plan: GraspPlan) -> None:
        self._latest_plan = plan
        self._plan_revision += 1

    def _wait_for_fresh_valid_plan(self, *, min_revision: int, timeout_sec: float) -> GraspPlan | None:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            plan = self._latest_plan
            if self._plan_revision <= min_revision or plan is None:
                continue
            if bool(plan.valid):
                return plan
        return None

    def call_trigger(self, client, label: str) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, f"{label} service unavailable"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_sec)
        if not future.done():
            return False, f"{label} service call timed out"
        result = future.result()
        if result is None:
            return False, f"{label} returned no result"
        return bool(result.success), str(result.message)

    def run_attempts(
        self,
        *,
        attempts: int,
        wait_enter: bool,
        return_ready_before_each: bool,
        return_ready_after_each: bool,
    ) -> HybridBenchmarkStats:
        stats = HybridBenchmarkStats()
        for index in range(1, int(attempts) + 1):
            self.get_logger().info(f"hybrid grasp sim benchmark attempt {index}/{attempts}")

            if return_ready_before_each:
                ok, message = self.call_trigger(self._ready_client, "visual_ready")
                self.get_logger().info(f"visual_ready: success={ok}, message={message}")
                if not ok:
                    stats.record(success=False, message=f"visual_ready failed: {message}")
                    continue

            if wait_enter:
                input(
                    f"[{index}/{attempts}] Confirm real arm is at visual_ready, real camera sees the object, "
                    "then press Enter to run simulated grasp..."
                )

            min_revision = self._plan_revision
            plan = self._wait_for_fresh_valid_plan(
                min_revision=min_revision,
                timeout_sec=self._plan_timeout_sec,
            )
            if plan is None:
                message = f"no valid fresh grasp plan received on {self._plan_topic}"
                stats.record(success=False, message=message)
                self.get_logger().warn(message)
                continue

            self._log_plan_snapshot(plan)
            ok, message = self.call_trigger(self._grasp_client, "visual_grasp")
            stats.record(success=ok, message=message)
            self.get_logger().info(f"visual_grasp(sim): success={ok}, message={message}")

            if return_ready_after_each:
                ready_ok, ready_message = self.call_trigger(self._ready_client, "visual_ready")
                self.get_logger().info(f"visual_ready(after): success={ready_ok}, message={ready_message}")
                if not ready_ok:
                    stats.record(success=False, message=f"visual_ready failed: {ready_message}")
        return stats

    def _log_plan_snapshot(self, plan: GraspPlan) -> None:
        pre = plan.pregrasp_pose.position
        grasp = plan.grasp_pose.position
        self.get_logger().info(
            "plan source="
            f"{plan.source}, candidate_source={plan.candidate.source}, "
            f"jaw_width={float(plan.jaw_width):.4f}, "
            f"pregrasp=({pre.x:.3f}, {pre.y:.3f}, {pre.z:.3f}), "
            f"grasp=({grasp.x:.3f}, {grasp.y:.3f}, {grasp.z:.3f}), "
            f"reason={plan.reason}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run repeated real-perception plus simulated-execution visual grasp attempts and summarize success rate."
        )
    )
    parser.add_argument("--attempts", type=int, default=20)
    parser.add_argument("--namespace", default="rebotarm_sim")
    parser.add_argument("--plan-topic", default="/grasp/filtered_plan")
    parser.add_argument("--service-timeout-sec", type=float, default=180.0)
    parser.add_argument("--plan-timeout-sec", type=float, default=10.0)
    parser.add_argument("--min-success-rate", type=float, default=0.0)
    parser.add_argument("--wait-enter", action="store_true", default=True)
    parser.add_argument("--no-wait-enter", action="store_false", dest="wait_enter")
    parser.add_argument("--return-ready-before-each", action="store_true", default=False)
    parser.add_argument("--return-ready-after-each", action="store_true", default=False)
    return parser


def main(argv: Iterable[str] | None = None) -> None:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    rclpy.init()
    node = HybridGraspSimBenchmark(
        namespace=args.namespace,
        plan_topic=args.plan_topic,
        service_timeout_sec=args.service_timeout_sec,
        plan_timeout_sec=args.plan_timeout_sec,
    )
    exit_code = 0
    try:
        stats = node.run_attempts(
            attempts=max(1, int(args.attempts)),
            wait_enter=bool(args.wait_enter),
            return_ready_before_each=bool(args.return_ready_before_each),
            return_ready_after_each=bool(args.return_ready_after_each),
        )
        for line in stats.summary_lines():
            node.get_logger().info(line)
        if stats.success_rate < float(args.min_success_rate):
            exit_code = 2
            node.get_logger().error(
                f"success_rate {stats.success_rate:.1f}% < min_success_rate {float(args.min_success_rate):.1f}%"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv[1:])

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def classify_failure_stage(message: str) -> str:
    text = str(message or "").strip()
    if not text:
        return "unknown"
    if " failed:" in text:
        return text.split(" failed:", 1)[0].strip() or "unknown"
    if text.startswith("no valid grasp plan"):
        return "detect"
    if text.startswith("no candidate attempts"):
        return "filter"
    if text.startswith("visual grasp already running"):
        return "busy"
    if text.startswith("visual grasp failed"):
        return "executor"
    return "unknown"


@dataclass
class BenchmarkStats:
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


class VisualGraspBenchmark(Node):
    def __init__(self, *, namespace: str, service_timeout_sec: float) -> None:
        super().__init__("rebotarm_visual_grasp_benchmark")
        namespace = namespace.strip("/")
        self._service_timeout_sec = float(service_timeout_sec)
        self._ready_client = self.create_client(Trigger, f"/{namespace}/visual_ready/move")
        self._grasp_client = self.create_client(Trigger, f"/{namespace}/visual_grasp/execute")

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

    def run_attempts(self, *, attempts: int, return_ready_before_each: bool, wait_enter: bool) -> BenchmarkStats:
        stats = BenchmarkStats()
        for index in range(1, int(attempts) + 1):
            self.get_logger().info(f"benchmark attempt {index}/{attempts}")
            if return_ready_before_each:
                ok, message = self.call_trigger(self._ready_client, "visual_ready")
                self.get_logger().info(f"visual_ready: success={ok}, message={message}")
                if not ok:
                    stats.record(success=False, message=f"visual_ready failed: {message}")
                    continue
            if wait_enter:
                input(
                    f"[{index}/{attempts}] Place object, open gripper if needed, confirm the area is safe, "
                    "then press Enter to grasp..."
                )
            ok, message = self.call_trigger(self._grasp_client, "visual_grasp")
            stats.record(success=ok, message=message)
            self.get_logger().info(f"visual_grasp: success={ok}, message={message}")
        return stats


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run repeated visual grasp attempts and summarize success rate.")
    parser.add_argument("--attempts", type=int, default=20)
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--service-timeout-sec", type=float, default=180.0)
    parser.add_argument("--wait-enter", action="store_true", default=True)
    parser.add_argument("--no-wait-enter", action="store_false", dest="wait_enter")
    parser.add_argument("--return-ready-before-each", action="store_true", default=True)
    parser.add_argument("--no-return-ready-before-each", action="store_false", dest="return_ready_before_each")
    return parser


def main(argv: Iterable[str] | None = None) -> None:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    rclpy.init()
    node = VisualGraspBenchmark(namespace=args.namespace, service_timeout_sec=args.service_timeout_sec)
    try:
        stats = node.run_attempts(
            attempts=max(1, int(args.attempts)),
            return_ready_before_each=bool(args.return_ready_before_each),
            wait_enter=bool(args.wait_enter),
        )
        for line in stats.summary_lines():
            node.get_logger().info(line)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

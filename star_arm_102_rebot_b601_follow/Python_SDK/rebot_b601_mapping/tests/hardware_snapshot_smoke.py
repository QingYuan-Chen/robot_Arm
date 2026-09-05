from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from rebot_b601_mapping.cli import run_snapshot


DEFAULT_CONFIG = Path(__file__).parents[1] / "mapping.example.json"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="两台机械臂只读快照真机冒烟测试")
    parser.add_argument("--leader-port", default="/dev/ttyUSB0")
    parser.add_argument("--follower-port", default="/dev/ttyACM0")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    print("本程序严格只读，不会使能、失能、置零或控制机械臂。")
    evidence = run_snapshot(
        leader_port=args.leader_port,
        follower_port=args.follower_port,
        config_path=args.config,
        output_path=args.output,
        baseline_samples=5,
        sample_count=args.samples,
        interval_s=0.02,
    )
    print(f"只读真机快照完成：{evidence['sample_count']} 个有效样本。")
    print(f"证据文件：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

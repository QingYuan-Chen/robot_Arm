#!/usr/bin/env python3
"""Generate deterministic outbound/return P5 command JSON files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "rebotarm_motion"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

from rebotarm_motion.paired_trajectory_protocol import build_quintic_command


def _positions(value: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value.split(","))
    if len(result) != 6:
        raise argparse.ArgumentTypeError("expected six comma-separated joint positions")
    return result


def _write(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=_positions, required=True)
    parser.add_argument("--target", type=_positions, required=True)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--cadence-sec", type=float, default=0.05)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    outbound = build_quintic_command(
        args.start,
        args.target,
        duration_sec=args.duration_sec,
        cadence_sec=args.cadence_sec,
        label="safe_posture_outbound",
    )
    return_command = build_quintic_command(
        args.target,
        args.start,
        duration_sec=args.duration_sec,
        cadence_sec=args.cadence_sec,
        label="safe_posture_return",
    )
    _write(args.output_dir / "outbound.json", outbound)
    _write(args.output_dir / "return.json", return_command)
    print(
        json.dumps(
            {
                "outbound_sha256": outbound["command_sha256"],
                "return_sha256": return_command["command_sha256"],
            }
        )
    )


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .handeye_residual import analyze_handeye_residual


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze multi-pose hand-eye residual samples")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--min-translation-span-m", type=float, default=0.05)
    parser.add_argument("--min-rotation-span-deg", type=float, default=20.0)
    parser.add_argument("--max-position-rms-m", type=float, default=0.005)
    parser.add_argument("--max-position-residual-m", type=float, default=0.010)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=1.5)
    parser.add_argument("--max-rotation-residual-deg", type=float, default=3.0)
    args = parser.parse_args()
    payload = json.loads(args.input.read_text(encoding="utf-8"))
    report = analyze_handeye_residual(
        payload,
        min_samples=args.min_samples,
        min_end_translation_span_m=args.min_translation_span_m,
        min_end_rotation_span_deg=args.min_rotation_span_deg,
        max_position_rms_m=args.max_position_rms_m,
        max_position_residual_m=args.max_position_residual_m,
        max_rotation_rms_deg=args.max_rotation_rms_deg,
        max_rotation_residual_deg=args.max_rotation_residual_deg,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Extract durable aggregate-only hand-eye captures from a raw runner report."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


REQUIRED_LABELS = ("baseline", "pose_1", "pose_2", "safe", "pose_4")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raw_bytes = args.input.read_bytes()
    source = json.loads(raw_bytes)
    captures = source.get("pose_captures")
    if not isinstance(captures, list):
        raise SystemExit("input has no pose_captures list")
    by_label = {str(item.get("label")): item for item in captures}
    output_captures = []
    for label in REQUIRED_LABELS:
        if label not in by_label:
            raise SystemExit(f"input is missing capture {label}")
        capture = by_label[label]
        if int(capture.get("accepted", 0)) < 30 or float(capture.get("detection_rate", 0.0)) < 0.90:
            raise SystemExit(f"capture {label} does not pass the detection gate")
        output_captures.append(
            {
                "label": label,
                "attempts": int(capture["attempts"]),
                "accepted": int(capture["accepted"]),
                "detection_rate": float(capture["detection_rate"]),
                "aggregate": capture["aggregate"],
            }
        )
    output = {
        "schema_version": 1,
        "kind": "p5_handeye_prior_aggregates",
        "source_sha256": hashlib.sha256(raw_bytes).hexdigest(),
        "source_started_at": source.get("started_at"),
        "source_finished_at": source.get("finished_at"),
        "source_success": bool(source.get("success", False)),
        "source_failure": source.get("failure"),
        "pose_captures": output_captures,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(args.output), "capture_count": len(output_captures)}))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Collect one deterministic safe-posture round trip from ROS 2.

This task-level tool talks only to ROS topics, services, and the existing
FollowJointTrajectory action. It never imports the motor SDK directly.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import rclpy


ROOT = Path(__file__).resolve().parents[1]
for package in ("rebotarm_motion", "rebotarm_simulation"):
    source = ROOT / "src" / package
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from rebotarm_motion.guarded_trajectory_client import (
    GuardedTrajectoryNode as PairedTrajectoryNode,
)
from rebotarm_motion.real_failure_recovery import recover_real_failure
from rebotarm_motion.paired_trajectory_protocol import (
    validate_command,
)


REAL_CONFIRMATION = "REAL_PAIRED_SAFE_POSTURE"


def _load_command(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    validate_command(payload)
    return payload


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "real"), required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--outbound-command", type=Path)
    parser.add_argument("--return-command", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hold-sec", type=float, default=2.0)
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="record disabled feedback/status and exit without enabling or sending a goal",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not args.preflight_only and (args.outbound_command is None or args.return_command is None):
        raise SystemExit("trajectory run requires --outbound-command and --return-command")
    if args.backend == "real" and not args.preflight_only and args.confirm != REAL_CONFIRMATION:
        raise SystemExit(f"real run requires --confirm {REAL_CONFIRMATION}")
    outbound = None if args.preflight_only else _load_command(args.outbound_command)
    return_command = None if args.preflight_only else _load_command(args.return_command)
    rclpy.init()
    node = PairedTrajectoryNode(namespace=args.namespace, backend=args.backend)
    report: dict[str, object] = {
        "schema_version": 1,
        "backend": args.backend,
        "namespace": args.namespace.strip("/"),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "services": [],
        "legs": [],
        "success": False,
    }
    hardware_enabled = False
    allow_controlled_return = False
    captured_baseline: tuple[float, ...] | None = None
    try:
        node.wait_for_preflight()
        captured_baseline = node.canonical_positions()
        report["preflight_positions"] = list(captured_baseline)
        report["preflight_status"] = node._status_payload()
        if args.preflight_only:
            report["success"] = True
            return
        assert outbound is not None and return_command is not None
        command_start = tuple(float(value) for value in outbound["points"][0]["positions"])
        start_errors = [target - actual for target, actual in zip(command_start, node.canonical_positions())]
        report["command_start_errors"] = start_errors
        if max(abs(value) for value in start_errors) > 0.01:
            raise RuntimeError(f"live start differs from command by more than 0.01 rad: {start_errors}")
        if args.backend == "real":
            report["services"].append(node.call_trigger(node.enable_client, "enable"))
            hardware_enabled = True
            allow_controlled_return = True
            node.hold_and_collect(0.5)
        allow_controlled_return = False
        outbound_run = node.execute_leg(outbound)
        report["legs"].append(outbound_run)
        if not outbound_run["success"]:
            raise RuntimeError(f"outbound failed: {outbound_run['result']}")
        allow_controlled_return = True
        node.hold_and_collect(args.hold_sec)
        allow_controlled_return = False
        return_run = node.execute_leg(return_command)
        report["legs"].append(return_run)
        if not return_run["success"]:
            raise RuntimeError(f"return failed: {return_run['result']}")
        allow_controlled_return = True
        node.hold_and_collect(0.5)
        final_positions = node.canonical_positions()
        baseline = tuple(float(value) for value in return_command["points"][-1]["positions"])
        final_errors = [target - actual for target, actual in zip(baseline, final_positions)]
        report["enabled_final_positions"] = list(final_positions)
        report["enabled_final_errors"] = final_errors
        if max(abs(value) for value in final_errors) > 0.02:
            raise RuntimeError(f"return baseline error exceeds 0.02 rad: {final_errors}")
        if args.backend == "real":
            report["services"].append(node.call_trigger(node.disable_client, "disable"))
            hardware_enabled = False
            node.hold_and_collect(0.5)
        report["final_status"] = node._status_payload()
        report["success"] = True
    except Exception as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        if args.backend == "real" and hardware_enabled:
            try:
                if captured_baseline is None:
                    raise RuntimeError("baseline unavailable for failure recovery")
                hardware_enabled = recover_real_failure(
                    node=node,
                    report=report,
                    baseline=captured_baseline,
                    allow_controlled_return=allow_controlled_return,
                    legs_key="legs",
                    command_label="paired_failure_return_baseline",
                )
            except Exception as recovery_exc:
                report["recovery_failure"] = f"{type(recovery_exc).__name__}: {recovery_exc}"
                report["recovery_final_status"] = node._status_payload()
        raise
    finally:
        report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        _write_json(args.output, report)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

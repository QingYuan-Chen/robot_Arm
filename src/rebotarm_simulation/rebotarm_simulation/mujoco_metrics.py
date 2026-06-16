from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math


@dataclass(frozen=True)
class MetricRow:
    elapsed: float
    joint: str
    target_position: float
    actual_position: float
    position_error: float
    velocity: float
    actuator_force: float


class TrajectoryMetricsRecorder:
    def __init__(self, output_dir: Path, *, joint_names: list[str]) -> None:
        self.output_dir = Path(output_dir)
        self.joint_names = [str(name) for name in joint_names]
        self.rows: list[MetricRow] = []

    def record(
        self,
        *,
        elapsed: float,
        targets: list[float],
        actual: list[float],
        velocities: list[float],
        actuator_forces: list[float],
    ) -> None:
        for index, joint in enumerate(self.joint_names):
            target = float(targets[index])
            position = float(actual[index])
            velocity = float(velocities[index]) if index < len(velocities) else 0.0
            force = float(actuator_forces[index]) if index < len(actuator_forces) else 0.0
            self.rows.append(
                MetricRow(
                    elapsed=float(elapsed),
                    joint=joint,
                    target_position=target,
                    actual_position=position,
                    position_error=target - position,
                    velocity=velocity,
                    actuator_force=force,
                )
            )

    def finish(self, *, success: bool, stop_reason: str) -> dict[str, object]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._write_csv(self.output_dir / "trajectory_metrics.csv")
        summary = self._summary(success=success, stop_reason=stop_reason)
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return summary

    def _write_csv(self, path: Path) -> None:
        with path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(
                file,
                fieldnames=[
                    "elapsed",
                    "joint",
                    "target_position",
                    "actual_position",
                    "position_error",
                    "velocity",
                    "actuator_force",
                ],
            )
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row.__dict__)

    def _summary(self, *, success: bool, stop_reason: str) -> dict[str, object]:
        errors = [abs(row.position_error) for row in self.rows]
        velocities = [abs(row.velocity) for row in self.rows]
        forces = [abs(row.actuator_force) for row in self.rows]
        rms_error = 0.0
        if errors:
            rms_error = math.sqrt(sum(error * error for error in errors) / float(len(errors)))
        return {
            "success": bool(success),
            "stop_reason": str(stop_reason),
            "sample_count": len(self.rows),
            "joint_count": len(self.joint_names),
            "max_abs_error": max(errors) if errors else 0.0,
            "rms_error": rms_error,
            "max_abs_velocity": max(velocities) if velocities else 0.0,
            "max_abs_actuator_force": max(forces) if forces else 0.0,
        }

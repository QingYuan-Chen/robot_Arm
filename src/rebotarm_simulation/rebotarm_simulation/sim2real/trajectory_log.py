from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

from .schemas import TrajectorySample


class TrajectoryRecorder:
    def __init__(self, *, episode_id: str, source: str, schema_version: int = 1) -> None:
        if not isinstance(episode_id, str) or not episode_id:
            raise ValueError("episode_id must be a non-empty string")
        if source not in {"sim", "real"}:
            raise ValueError("source must be 'sim' or 'real'")
        self.episode_id = episode_id
        self.source = source
        self.schema_version = int(schema_version)
        self._samples: list[TrajectorySample] = []

    @property
    def samples(self) -> tuple[TrajectorySample, ...]:
        return tuple(self._samples)

    def __len__(self) -> int:
        return len(self._samples)

    def append(self, sample: TrajectorySample) -> None:
        if not isinstance(sample, TrajectorySample):
            raise TypeError("sample must be a TrajectorySample")
        if sample.episode_id != self.episode_id:
            raise ValueError("sample episode_id does not match recorder")
        if sample.source != self.source:
            raise ValueError("sample source does not match recorder")
        if sample.schema_version != self.schema_version:
            raise ValueError("sample schema_version does not match recorder")
        if self._samples and sample.simulation_time <= self._samples[-1].simulation_time:
            raise ValueError("sample simulation_time must be strictly increasing")
        if self._samples and sample.step_index <= self._samples[-1].step_index:
            raise ValueError("sample step_index must be strictly increasing")
        self._samples.append(sample)

    def extend(self, samples: Iterable[TrajectorySample]) -> None:
        for sample in samples:
            self.append(sample)

    def summary(self) -> dict[str, str | int | float]:
        if not self._samples:
            return {
                "episode_id": self.episode_id,
                "source": self.source,
                "sample_count": 0,
                "start_time": None,
                "end_time": None,
                "max_joint_velocity": 0.0,
                "max_actuator_torque": 0.0,
                "max_contact_force": 0.0,
            }
        return {
            "episode_id": self.episode_id,
            "source": self.source,
            "sample_count": len(self._samples),
            "start_time": self._samples[0].simulation_time,
            "end_time": self._samples[-1].simulation_time,
            "max_joint_velocity": max(
                max(abs(value) for value in sample.joint_velocities)
                for sample in self._samples
            ),
            "max_actuator_torque": max(
                max(abs(value) for value in sample.actuator_torques)
                for sample in self._samples
            ),
            "max_contact_force": max(sample.max_contact_force for sample in self._samples),
        }

    def to_jsonl(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8", newline="\n") as stream:
            for sample in self._samples:
                stream.write(json.dumps(sample.to_dict(), ensure_ascii=False, sort_keys=True))
                stream.write("\n")

    @classmethod
    def from_jsonl(cls, path: str | Path) -> "TrajectoryRecorder":
        samples = []
        with Path(path).open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    samples.append(TrajectorySample.from_dict(json.loads(line)))
                except (TypeError, ValueError, json.JSONDecodeError) as exc:
                    raise ValueError(f"invalid trajectory JSONL line {line_number}") from exc
        if not samples:
            raise ValueError("trajectory JSONL contains no samples")
        recorder = cls(
            episode_id=samples[0].episode_id,
            source=samples[0].source,
            schema_version=samples[0].schema_version,
        )
        recorder.extend(samples)
        return recorder

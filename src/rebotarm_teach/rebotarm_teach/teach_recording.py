from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from rebotarm_motion.trajectory_time_parameterization import parameterize_teach_samples


@dataclass(frozen=True)
class TeachSample:
    stamp: float
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    motor_status: dict[str, int]
    arm_state: str


@dataclass(frozen=True)
class TeachTrajectoryEvent:
    sample: int
    joint_name: str
    level: str
    message: str
    delta_rad: float
    velocity_rad_s: float | None
    acceleration_rad_s2: float | None = None
    jerk_rad_s3: float | None = None


@dataclass(frozen=True)
class TeachTrajectoryQuality:
    risk_level: str
    replay_policy: str
    allow_real_replay: bool
    requires_safe_retiming: bool
    max_jump_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    worst_joint: str
    worst_sample: int
    anomalies: tuple[str, ...]
    events: tuple[TeachTrajectoryEvent, ...]
    green_jump_rad: float
    yellow_jump_rad: float
    velocity_limit_rad_s: float
    acceleration_limit_rad_s2: float
    max_jerk_rad_s3: float = 0.0
    jerk_limit_rad_s3: float = 999.0


@dataclass(frozen=True)
class RetimedTeachPoint:
    time_from_start: float
    positions: tuple[float, ...]
    source_sample: int
    velocities: tuple[float, ...] = ()


@dataclass(frozen=True)
class PreparedTeachReplay:
    samples: list[TeachSample]
    raw_quality: TeachTrajectoryQuality
    filtered_quality: TeachTrajectoryQuality
    retimed_quality: TeachTrajectoryQuality
    smoothing_applied: bool
    filter_applied: bool
    resample_applied: bool
    retime_applied: bool
    smoothing_window: int
    filter_cutoff_hz: float
    filter_sample_rate_hz: float
    resample_rate_hz: float
    retimed_points: list[RetimedTeachPoint]
    large_motion: bool = False
    max_joint_span_rad: float = 0.0
    total_joint_motion_rad: float = 0.0
    requested_replay_speed: float = 1.0
    effective_replay_speed: float = 1.0
    large_motion_max_speed: float = 1.0
    time_parameterization_requested_method: str = "auto"
    time_parameterization_used_method: str = "current_jerk_retime"
    time_parameterization_message: str = ""

    @property
    def before_quality(self) -> TeachTrajectoryQuality:
        return self.raw_quality

    @property
    def after_quality(self) -> TeachTrajectoryQuality:
        return self.retimed_quality


def is_quit_key(key: str | None, *, quit_key: str = "q") -> bool:
    if key is None:
        return False
    return str(key).strip().lower() == str(quit_key).strip().lower()


def encode_teach_sample(sample: TeachSample) -> str:
    return json.dumps(
        {
            "stamp": sample.stamp,
            "joint_names": list(sample.joint_names),
            "positions": list(sample.positions),
            "velocities": list(sample.velocities),
            "efforts": list(sample.efforts),
            "motor_status": sample.motor_status,
            "arm_state": sample.arm_state,
        },
        separators=(",", ":"),
    )


def decode_teach_sample(payload: str) -> TeachSample:
    data = json.loads(payload)
    return TeachSample(
        stamp=float(data["stamp"]),
        joint_names=tuple(str(v) for v in data["joint_names"]),
        positions=tuple(float(v) for v in data["positions"]),
        velocities=tuple(float(v) for v in data.get("velocities", [])),
        efforts=tuple(float(v) for v in data.get("efforts", [])),
        motor_status={str(k): int(v) for k, v in data.get("motor_status", {}).items()},
        arm_state=str(data.get("arm_state", "")),
    )


def load_teach_samples(path: str | Path) -> list[TeachSample]:
    samples: list[TeachSample] = []
    with Path(path).open("r", encoding="utf-8-sig") as handle:
        for line in handle:
            line = line.strip()
            if line:
                samples.append(decode_teach_sample(line))
    return samples


def prepared_record_path(raw_path: str | Path) -> Path:
    path = Path(raw_path)
    if path.suffix:
        return path.with_name(f"{path.stem}.prepared{path.suffix}")
    return path.with_name(f"{path.name}.prepared.jsonl")


def write_prepared_teach_record(
    raw_path: str | Path,
    prepared: PreparedTeachReplay,
    *,
    output_path: str | Path | None = None,
) -> Path:
    target = Path(output_path) if output_path is not None else prepared_record_path(raw_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    samples: list[TeachSample]
    if prepared.retimed_points:
        joint_names = prepared.samples[0].joint_names if prepared.samples else ()
        samples = [
            TeachSample(
                stamp=float(point.time_from_start),
                joint_names=joint_names,
                positions=point.positions,
                velocities=point.velocities,
                efforts=(),
                motor_status={},
                arm_state="PREPARED_REPLAY",
            )
            for point in prepared.retimed_points
        ]
    else:
        samples = [
            TeachSample(
                stamp=float(index) / max(float(prepared.resample_rate_hz), 1.0),
                joint_names=sample.joint_names,
                positions=sample.positions,
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=sample.motor_status,
                arm_state="PREPARED_REPLAY",
            )
            for index, sample in enumerate(prepared.samples)
        ]
    with target.open("w", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(encode_teach_sample(sample) + "\n")
    return target


class ReplayStartBand(str, Enum):
    DIRECT = "direct"
    ALIGN = "align"
    MOVEIT_ALIGN = "moveit_align"
    REJECT = "reject"


@dataclass(frozen=True)
class ReplayStartDecision:
    band: ReplayStartBand
    max_error: float
    per_joint_error: tuple[float, ...]
    allow_replay: bool
    allow_auto_align: bool
    message: str


@dataclass(frozen=True)
class TeachRecordInfo:
    path: str
    exists: bool
    samples: int
    duration_sec: float
    joint_names: tuple[str, ...]
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]
    start_band: str
    max_error: float | None
    worst_joint: str
    per_joint_error: dict[str, float]
    anomalies: tuple[str, ...]
    message: str
    quality: TeachTrajectoryQuality | None = None


@dataclass(frozen=True)
class TeachDryRunDecision:
    accepted: bool
    state: str
    message: str


def classify_replay_start(
    *,
    current_positions: tuple[float, ...],
    start_positions: tuple[float, ...],
    direct_threshold: float,
    align_threshold: float,
) -> ReplayStartDecision:
    if len(current_positions) != len(start_positions):
        return ReplayStartDecision(
            band=ReplayStartBand.REJECT,
            max_error=float("inf"),
            per_joint_error=(),
            allow_replay=False,
            allow_auto_align=False,
            message="current and start joint vectors have different lengths",
        )
    errors = tuple(abs(float(a) - float(b)) for a, b in zip(current_positions, start_positions))
    max_error = max(errors, default=0.0)
    if max_error < float(direct_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.DIRECT,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=False,
            message="current pose is close enough to replay start",
        )
    if max_error < float(align_threshold):
        return ReplayStartDecision(
            band=ReplayStartBand.ALIGN,
            max_error=max_error,
            per_joint_error=errors,
            allow_replay=True,
            allow_auto_align=True,
            message="small return_to_start alignment required",
        )
    return ReplayStartDecision(
        band=ReplayStartBand.REJECT,
        max_error=max_error,
        per_joint_error=errors,
        allow_replay=False,
        allow_auto_align=False,
        message="start error too large; manually drag the arm near the recording start",
    )


def interpolate_joint_positions(
    *,
    current_positions: tuple[float, ...],
    target_positions: tuple[float, ...],
    steps: int,
) -> list[tuple[float, ...]]:
    if len(current_positions) != len(target_positions):
        raise ValueError("current and target joint vectors have different lengths")
    count = max(int(steps), 2)
    points: list[tuple[float, ...]] = []
    for index in range(count):
        alpha = float(index) / float(count - 1)
        points.append(
            tuple(
                float(current + (target - current) * alpha)
                for current, target in zip(current_positions, target_positions)
            )
        )
    return points


def build_replay_start_soft_points(
    *,
    current_positions: tuple[float, ...],
    first_positions: tuple[float, ...],
    start_band: str,
    start_hold_sec: float = 0.8,
    soft_start_duration: float = 1.0,
    soft_start_steps: int = 30,
    align_duration: float = 3.0,
    align_steps: int = 30,
    first_hold_sec: float = 0.3,
) -> list[RetimedTeachPoint]:
    if len(current_positions) != len(first_positions):
        raise ValueError("current and first joint vectors have different lengths")
    elapsed = 0.0
    points: list[RetimedTeachPoint] = []
    hold = max(float(start_hold_sec), 0.0)
    if hold > 0.0:
        elapsed += hold
        points.append(
            RetimedTeachPoint(
                time_from_start=elapsed,
                positions=tuple(float(v) for v in current_positions),
                source_sample=-1,
            )
        )
    band = str(start_band or "").strip().lower()
    if band == ReplayStartBand.ALIGN.value:
        duration = max(float(align_duration), 0.0)
        steps = int(align_steps)
    else:
        duration = max(float(soft_start_duration), 0.0)
        steps = int(soft_start_steps)
    align_points = interpolate_joint_positions(
        current_positions=tuple(float(v) for v in current_positions),
        target_positions=tuple(float(v) for v in first_positions),
        steps=steps,
    )
    for index, positions in enumerate(align_points):
        ratio = float(index) / float(max(len(align_points) - 1, 1))
        timestamp = elapsed + duration * ratio
        if points and timestamp <= points[-1].time_from_start:
            continue
        points.append(
            RetimedTeachPoint(
                time_from_start=timestamp,
                positions=tuple(float(v) for v in positions),
                source_sample=-1,
            )
        )
    elapsed += duration
    first_hold = max(float(first_hold_sec), 0.0)
    if first_hold > 0.0:
        elapsed += first_hold
        if not points or elapsed > points[-1].time_from_start:
            points.append(
                RetimedTeachPoint(
                    time_from_start=elapsed,
                    positions=tuple(float(v) for v in first_positions),
                    source_sample=-1,
                )
            )
    return points


def _max_position_delta(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    if len(a) != len(b):
        return float("inf")
    return max((abs(float(current) - float(last)) for current, last in zip(a, b)), default=0.0)


def compute_auto_align_duration(
    max_error_rad: float | None,
    *,
    target_speed_rad_s: float = 0.15,
    min_duration_sec: float = 3.0,
    max_duration_sec: float = 10.0,
) -> float:
    try:
        error = abs(float(max_error_rad))
    except (TypeError, ValueError):
        error = 0.0
    speed = max(float(target_speed_rad_s), 0.01)
    duration = error / speed if error > 0.0 else float(min_duration_sec)
    return min(max(duration, float(min_duration_sec)), float(max_duration_sec))


def _motion_scope(samples: list[TeachSample]) -> tuple[float, float]:
    if not samples:
        return 0.0, 0.0
    joint_count = len(samples[0].positions)
    if joint_count == 0:
        return 0.0, 0.0
    mins = [float("inf") for _ in range(joint_count)]
    maxs = [float("-inf") for _ in range(joint_count)]
    total = 0.0
    previous: TeachSample | None = None
    for sample in samples:
        if len(sample.positions) != joint_count:
            return float("inf"), float("inf")
        for index, position in enumerate(sample.positions):
            value = float(position)
            mins[index] = min(mins[index], value)
            maxs[index] = max(maxs[index], value)
        if previous is not None:
            total += sum(
                abs(float(current) - float(last))
                for current, last in zip(sample.positions, previous.positions)
            )
        previous = sample
    max_span = max((upper - lower for lower, upper in zip(mins, maxs)), default=0.0)
    return max_span, total


def _velocity_limits_for_joints(
    max_velocity_rad_s,
    joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    if isinstance(max_velocity_rad_s, dict):
        fallback = float(max_velocity_rad_s.get("*", 2.0))
        return tuple(max(float(max_velocity_rad_s.get(name, fallback)), 0.01) for name in joint_names)
    if isinstance(max_velocity_rad_s, (list, tuple)):
        if len(max_velocity_rad_s) != len(joint_names):
            raise ValueError("max_velocity_rad_s length must match joint_names")
        return tuple(max(float(value), 0.01) for value in max_velocity_rad_s)
    return tuple(max(float(max_velocity_rad_s), 0.01) for _ in joint_names)


def _velocity_limit_summary(max_velocity_rad_s) -> float:
    if isinstance(max_velocity_rad_s, dict):
        values = [float(value) for value in max_velocity_rad_s.values()]
        return max(values, default=0.0)
    if isinstance(max_velocity_rad_s, (list, tuple)):
        return max((float(value) for value in max_velocity_rad_s), default=0.0)
    return float(max_velocity_rad_s)


def analyze_teach_trajectory(
    samples: list[TeachSample],
    *,
    green_jump_rad: float = 0.03,
    yellow_jump_rad: float = 0.05,
    max_velocity_rad_s: float = 2.0,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
) -> TeachTrajectoryQuality:
    anomalies: list[str] = []
    events: list[TeachTrajectoryEvent] = []
    risk_level = "green"
    max_jump = 0.0
    max_velocity = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    worst_joint = ""
    worst_sample = -1
    if not samples:
        return TeachTrajectoryQuality(
            risk_level="red",
            replay_policy="record contains no samples",
            allow_real_replay=False,
            requires_safe_retiming=False,
            max_jump_rad=0.0,
            max_velocity_rad_s=0.0,
            max_acceleration_rad_s2=0.0,
            max_jerk_rad_s3=0.0,
            worst_joint="",
            worst_sample=-1,
            anomalies=("empty record",),
            events=(),
            green_jump_rad=float(green_jump_rad),
            yellow_jump_rad=float(yellow_jump_rad),
            velocity_limit_rad_s=_velocity_limit_summary(max_velocity_rad_s),
            acceleration_limit_rad_s2=float(max_acceleration_rad_s2),
            jerk_limit_rad_s3=float(max_jerk_rad_s3),
        )
    expected_joints = samples[0].joint_names
    expected_len = len(expected_joints)
    velocity_limits = _velocity_limits_for_joints(max_velocity_rad_s, expected_joints)
    previous = samples[0]
    previous_velocities = tuple(0.0 for _ in range(expected_len))
    previous_accelerations = tuple(0.0 for _ in range(expected_len))
    if len(previous.positions) != expected_len:
        anomalies.append("positions length mismatch at sample 0")
        risk_level = "red"
    for index, sample in enumerate(samples[1:], start=1):
        if sample.joint_names != expected_joints:
            anomalies.append(f"joint_names mismatch at sample {index}")
            risk_level = "red"
        if len(sample.positions) != expected_len:
            anomalies.append(f"positions length mismatch at sample {index}")
            risk_level = "red"
        dt = float(sample.stamp) - float(previous.stamp)
        if dt <= 0.0:
            anomalies.append(f"timestamp not monotonic at sample {index}")
            risk_level = "red"
        current_velocities = tuple(0.0 for _ in range(expected_len))
        current_accelerations = tuple(0.0 for _ in range(expected_len))
        if dt > 0.0 and len(sample.positions) == expected_len and len(previous.positions) == expected_len:
            current_velocities = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(sample.positions, previous.positions)
            )
            current_accelerations = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(current_velocities, previous_velocities)
            )
        for joint_index, (joint_name, current, last) in enumerate(zip(expected_joints, sample.positions, previous.positions)):
            delta = abs(float(current) - float(last))
            velocity = abs(current_velocities[joint_index]) if dt > 0.0 else None
            if delta > max_jump:
                max_jump = delta
                worst_joint = joint_name
                worst_sample = index
            if velocity is not None and velocity > max_velocity:
                max_velocity = velocity
            acceleration = None
            jerk = None
            if velocity is not None and dt > 0.0:
                acceleration = abs(float(current_accelerations[joint_index]))
                if acceleration > max_acceleration:
                    max_acceleration = acceleration
                jerk = abs(float(current_accelerations[joint_index]) - float(previous_accelerations[joint_index])) / dt
                if jerk > max_jerk:
                    max_jerk = jerk
            level = ""
            message = ""
            if delta > float(yellow_jump_rad):
                level = "red"
                message = f"{joint_name} jump {delta:.4f} rad at sample {index}"
                risk_level = "red"
                anomalies.append(message)
            elif delta > float(green_jump_rad):
                level = "yellow"
                message = f"{joint_name} jump {delta:.4f} rad at sample {index}"
                if risk_level != "red":
                    risk_level = "yellow"
            velocity_limit = velocity_limits[joint_index]
            if velocity is not None and velocity > velocity_limit:
                velocity_message = f"{joint_name} velocity {velocity:.4f} rad/s at sample {index}"
                if not message:
                    message = velocity_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(velocity_message)
            if acceleration is not None and acceleration > float(max_acceleration_rad_s2):
                acceleration_message = f"{joint_name} acceleration {acceleration:.4f} rad/s^2 at sample {index}"
                if not message:
                    message = acceleration_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(acceleration_message)
            if jerk is not None and jerk > float(max_jerk_rad_s3):
                jerk_message = f"{joint_name} jerk {jerk:.4f} rad/s^3 at sample {index}"
                if not message:
                    message = jerk_message
                    level = "yellow"
                if risk_level != "red":
                    risk_level = "yellow"
                anomalies.append(jerk_message)
            if level:
                events.append(
                    TeachTrajectoryEvent(
                        sample=index,
                        joint_name=joint_name,
                        level=level,
                        message=message,
                        delta_rad=delta,
                        velocity_rad_s=velocity,
                        acceleration_rad_s2=acceleration,
                        jerk_rad_s3=jerk,
                    )
                )
        if dt > 0.0:
            previous_velocities = current_velocities
            previous_accelerations = current_accelerations
        previous = sample
    if risk_level == "green":
        replay_policy = "normal replay allowed"
    elif risk_level == "yellow":
        replay_policy = "safe retiming required before real replay"
    else:
        replay_policy = "real replay blocked; record a cleaner teach trajectory"
    return TeachTrajectoryQuality(
        risk_level=risk_level,
        replay_policy=replay_policy,
        allow_real_replay=risk_level != "red",
        requires_safe_retiming=risk_level == "yellow",
        max_jump_rad=max_jump,
        max_velocity_rad_s=max_velocity,
        max_acceleration_rad_s2=max_acceleration,
        max_jerk_rad_s3=max_jerk,
        worst_joint=worst_joint,
        worst_sample=worst_sample,
        anomalies=tuple(anomalies),
        events=tuple(events),
        green_jump_rad=float(green_jump_rad),
        yellow_jump_rad=float(yellow_jump_rad),
        velocity_limit_rad_s=max(velocity_limits, default=0.0),
        acceleration_limit_rad_s2=float(max_acceleration_rad_s2),
        jerk_limit_rad_s3=float(max_jerk_rad_s3),
    )


def teach_trajectory_quality_to_dict(quality: TeachTrajectoryQuality) -> dict:
    return {
        "risk_level": quality.risk_level,
        "replay_policy": quality.replay_policy,
        "allow_real_replay": quality.allow_real_replay,
        "requires_safe_retiming": quality.requires_safe_retiming,
        "max_jump_rad": quality.max_jump_rad,
        "max_velocity_rad_s": quality.max_velocity_rad_s,
        "max_acceleration_rad_s2": quality.max_acceleration_rad_s2,
        "worst_joint": quality.worst_joint,
        "worst_sample": quality.worst_sample,
        "anomalies": list(quality.anomalies),
        "events": [
            {
                "sample": event.sample,
                "joint_name": event.joint_name,
                "level": event.level,
                "message": event.message,
                "delta_rad": event.delta_rad,
                "velocity_rad_s": event.velocity_rad_s,
                "acceleration_rad_s2": event.acceleration_rad_s2,
                "jerk_rad_s3": event.jerk_rad_s3,
            }
            for event in quality.events
        ],
        "green_jump_rad": quality.green_jump_rad,
        "yellow_jump_rad": quality.yellow_jump_rad,
        "velocity_limit_rad_s": quality.velocity_limit_rad_s,
        "acceleration_limit_rad_s2": quality.acceleration_limit_rad_s2,
        "max_jerk_rad_s3": quality.max_jerk_rad_s3,
        "jerk_limit_rad_s3": quality.jerk_limit_rad_s3,
    }


def retime_teach_samples(
    samples: list[TeachSample],
    *,
    replay_speed: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float = 999.0,
    max_jerk_rad_s3: float = 999.0,
    initial_delay_sec: float = 0.2,
    boundary_zero_velocity: bool = True,
) -> list[RetimedTeachPoint]:
    if not samples:
        return []
    expected_joints = samples[0].joint_names
    expected_len = len(expected_joints)
    for index, sample in enumerate(samples):
        if sample.joint_names != expected_joints:
            raise ValueError(f"joint_names mismatch at sample {index}")
        if len(sample.positions) != expected_len:
            raise ValueError(f"positions length mismatch at sample {index}")
    speed = max(float(replay_speed), 0.01)
    velocity_limits = _velocity_limits_for_joints(max_velocity_rad_s, expected_joints)
    acceleration_limit = max(float(max_acceleration_rad_s2), 0.01)
    jerk_limit = max(float(max_jerk_rad_s3), 0.01)
    zero_velocity = tuple(0.0 for _ in samples[0].positions)
    retimed = [
        RetimedTeachPoint(
            time_from_start=max(float(initial_delay_sec), 0.0),
            positions=tuple(float(v) for v in samples[0].positions),
            source_sample=0,
            velocities=zero_velocity,
        )
    ]
    previous = samples[0]
    previous_velocity = zero_velocity
    previous_acceleration = zero_velocity
    elapsed = retimed[0].time_from_start
    for index, sample in enumerate(samples[1:], start=1):
        recorded_dt = max(0.0, float(sample.stamp) - float(previous.stamp)) / speed
        min_velocity_dt = max(
            (
                abs(float(current) - float(last)) / velocity_limit
                for current, last, velocity_limit in zip(sample.positions, previous.positions, velocity_limits)
            ),
            default=0.0,
        )
        dt = max(recorded_dt, min_velocity_dt, 0.001)
        delta = tuple(
            float(current) - float(last)
            for current, last in zip(sample.positions, previous.positions)
        )
        for _ in range(24):
            velocity = tuple(value / dt for value in delta)
            acceleration = tuple(
                (float(current) - float(last)) / dt
                for current, last in zip(velocity, previous_velocity)
            )
            max_acceleration = max((abs(value) for value in acceleration), default=0.0)
            max_jerk = max(
                (
                    abs(float(current) - float(last)) / dt
                    for current, last in zip(acceleration, previous_acceleration)
                ),
                default=0.0,
            )
            if (
                max_acceleration <= acceleration_limit + 1e-12
                and max_jerk <= jerk_limit + 1e-12
            ):
                break
            dt *= 1.25
        velocity = tuple(value / dt for value in delta)
        acceleration = tuple(
            (float(current) - float(last)) / dt
            for current, last in zip(velocity, previous_velocity)
        )
        elapsed += dt
        retimed.append(
            RetimedTeachPoint(
                time_from_start=elapsed,
                positions=tuple(float(v) for v in sample.positions),
                source_sample=index,
                velocities=velocity,
            )
        )
        previous = sample
        previous_velocity = velocity
        previous_acceleration = acceleration
    if boundary_zero_velocity and retimed:
        retimed[0] = RetimedTeachPoint(
            time_from_start=retimed[0].time_from_start,
            positions=retimed[0].positions,
            source_sample=retimed[0].source_sample,
            velocities=zero_velocity,
        )
        if len(retimed) > 1:
            previous_point = retimed[-2]
            last_point = retimed[-1]
            prior_acceleration = zero_velocity
            if len(retimed) > 2:
                before_previous = retimed[-3]
                prior_dt = max(previous_point.time_from_start - before_previous.time_from_start, 1e-9)
                prior_acceleration = tuple(
                    (float(current) - float(last)) / prior_dt
                    for current, last in zip(previous_point.velocities, before_previous.velocities)
                )
            current_dt = last_point.time_from_start - previous_point.time_from_start
            adjusted_dt = max(current_dt, 0.001)
            for _ in range(24):
                final_acceleration = tuple(
                    (0.0 - float(value)) / adjusted_dt
                    for value in previous_point.velocities
                )
                max_acceleration = max((abs(value) for value in final_acceleration), default=0.0)
                max_jerk = max(
                    (
                        abs(float(current) - float(last)) / adjusted_dt
                        for current, last in zip(final_acceleration, prior_acceleration)
                    ),
                    default=0.0,
                )
                if (
                    max_acceleration <= acceleration_limit + 1e-12
                    and max_jerk <= jerk_limit + 1e-12
                ):
                    break
                adjusted_dt *= 1.25
            adjusted_time = previous_point.time_from_start + adjusted_dt
            retimed[-1] = RetimedTeachPoint(
                time_from_start=adjusted_time,
                positions=last_point.positions,
                source_sample=last_point.source_sample,
                velocities=tuple(0.0 for _ in last_point.positions),
            )
    return retimed


def lowpass_filter_teach_samples(
    samples: list[TeachSample],
    *,
    sample_rate_hz: float,
    cutoff_hz: float,
    preserve_start_end: bool = True,
) -> list[TeachSample]:
    if len(samples) <= 2:
        return list(samples)
    rate = max(float(sample_rate_hz), 1.0)
    cutoff = max(float(cutoff_hz), 0.01)
    dt = 1.0 / rate
    rc = 1.0 / (2.0 * math.pi * cutoff)
    alpha = dt / (rc + dt)
    filtered_positions: list[tuple[float, ...]] = [tuple(float(v) for v in samples[0].positions)]
    for sample in samples[1:]:
        previous = filtered_positions[-1]
        filtered_positions.append(
            tuple(
                float(last) + alpha * (float(current) - float(last))
                for current, last in zip(sample.positions, previous)
            )
        )
    backward = [filtered_positions[-1]]
    for positions in reversed(filtered_positions[:-1]):
        previous = backward[-1]
        backward.append(
            tuple(
                float(last) + alpha * (float(current) - float(last))
                for current, last in zip(positions, previous)
            )
        )
    filtered_positions = list(reversed(backward))
    if preserve_start_end:
        filtered_positions[0] = samples[0].positions
        filtered_positions[-1] = samples[-1].positions
    result: list[TeachSample] = []
    for sample, positions in zip(samples, filtered_positions):
        result.append(
            TeachSample(
                stamp=sample.stamp,
                joint_names=sample.joint_names,
                positions=tuple(float(v) for v in positions),
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=dict(sample.motor_status),
                arm_state=sample.arm_state,
            )
        )
    return result


def smooth_teach_samples(
    samples: list[TeachSample],
    *,
    window: int = 5,
    preserve_start_end: bool = True,
) -> list[TeachSample]:
    if len(samples) <= 2:
        return list(samples)
    width = max(int(window), 1)
    if width % 2 == 0:
        width += 1
    radius = width // 2
    smoothed: list[TeachSample] = []
    for index, sample in enumerate(samples):
        if preserve_start_end and index in (0, len(samples) - 1):
            smoothed.append(sample)
            continue
        start = max(0, index - radius)
        end = min(len(samples), index + radius + 1)
        segment = samples[start:end]
        positions = tuple(
            sum(float(item.positions[joint_index]) for item in segment) / float(len(segment))
            for joint_index in range(len(sample.positions))
        )
        smoothed.append(
            TeachSample(
                stamp=sample.stamp,
                joint_names=sample.joint_names,
                positions=positions,
                velocities=sample.velocities,
                efforts=sample.efforts,
                motor_status=dict(sample.motor_status),
                arm_state=sample.arm_state,
            )
        )
    return smoothed


def resample_teach_samples(
    samples: list[TeachSample],
    *,
    rate_hz: float = 50.0,
) -> list[TeachSample]:
    if len(samples) <= 1:
        return list(samples)
    rate = max(float(rate_hz), 1.0)
    period = 1.0 / rate
    start_stamp = float(samples[0].stamp)
    end_stamp = float(samples[-1].stamp)
    duration = max(0.0, end_stamp - start_stamp)
    if duration <= 0.0:
        return list(samples)
    result: list[TeachSample] = []
    source_index = 0
    count = max(int(round(duration / period)), 1)
    for output_index in range(count + 1):
        stamp = start_stamp + min(float(output_index) * period, duration)
        while source_index + 1 < len(samples) and float(samples[source_index + 1].stamp) < stamp:
            source_index += 1
        previous = samples[source_index]
        following = samples[min(source_index + 1, len(samples) - 1)]
        span = max(float(following.stamp) - float(previous.stamp), 1e-9)
        alpha = 0.0 if following is previous else (stamp - float(previous.stamp)) / span
        alpha = min(max(alpha, 0.0), 1.0)
        positions = tuple(
            float(a) + (float(b) - float(a)) * alpha
            for a, b in zip(previous.positions, following.positions)
        )
        result.append(
            TeachSample(
                stamp=stamp,
                joint_names=previous.joint_names,
                positions=positions,
                velocities=(),
                efforts=(),
                motor_status=dict(previous.motor_status),
                arm_state=previous.arm_state,
            )
        )
    result[0] = samples[0]
    result[-1] = samples[-1]
    return result


def _has_structural_teach_anomaly(quality: TeachTrajectoryQuality) -> bool:
    return any(
        "joint_names mismatch" in item
        or "positions length mismatch" in item
        or "timestamp not monotonic" in item
        for item in quality.anomalies
    )


def prepare_teach_replay_samples(
    samples: list[TeachSample],
    *,
    smoothing_enabled: bool = True,
    smoothing_window: int = 7,
    filter_enabled: bool = True,
    filter_cutoff_hz: float = 5.0,
    filter_sample_rate_hz: float = 50.0,
    resample_enabled: bool = True,
    resample_rate_hz: float = 100.0,
    retime_enabled: bool = False,
    replay_speed: float = 1.0,
    max_velocity_rad_s: float = 1.5,
    max_acceleration_rad_s2: float = 5.0,
    max_jerk_rad_s3: float = 20.0,
    time_parameterization_method: str = "auto",
    large_motion_span_rad: float = 0.8,
    large_motion_total_rad: float = 2.5,
    large_motion_max_speed: float = 1.0,
) -> PreparedTeachReplay:
    max_joint_span, total_joint_motion = _motion_scope(samples)
    requested_speed = min(max(float(replay_speed), 0.01), 1.0)
    large_motion = (
        max_joint_span >= float(large_motion_span_rad)
        or total_joint_motion >= float(large_motion_total_rad)
    )
    effective_speed = requested_speed
    raw_quality = analyze_teach_trajectory(
        samples,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    prepared = list(samples)
    smoothing_applied = False
    filter_applied = False
    resample_applied = False
    retime_applied = False
    if smoothing_enabled and prepared:
        prepared = smooth_teach_samples(prepared, window=smoothing_window)
        smoothing_applied = len(prepared) > 0
    if filter_enabled and len(prepared) > 2:
        prepared = lowpass_filter_teach_samples(
            prepared,
            sample_rate_hz=filter_sample_rate_hz,
            cutoff_hz=filter_cutoff_hz,
        )
        filter_applied = len(prepared) > 0
    filtered_quality = analyze_teach_trajectory(
        prepared,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    if resample_enabled and len(prepared) > 1:
        prepared = resample_teach_samples(prepared, rate_hz=resample_rate_hz)
        resample_applied = len(prepared) != len(samples)
    retimed_points: list[RetimedTeachPoint] = []
    if retime_enabled and not _has_structural_teach_anomaly(filtered_quality):
        time_parameterization = parameterize_teach_samples(
            prepared,
            method=time_parameterization_method,
            fallback_retime=retime_teach_samples,
            replay_speed=effective_speed,
            max_velocity_rad_s=max_velocity_rad_s,
            max_acceleration_rad_s2=max_acceleration_rad_s2,
            max_jerk_rad_s3=max_jerk_rad_s3,
            initial_delay_sec=0.0,
            boundary_zero_velocity=True,
        )
        retimed_points = time_parameterization.points
        retime_applied = len(retimed_points) > 0
    else:
        time_parameterization = None
    retimed_samples = [
        TeachSample(
            stamp=point.time_from_start,
            joint_names=prepared[0].joint_names if prepared else (),
            positions=point.positions,
            velocities=point.velocities,
            efforts=(),
            motor_status={},
            arm_state="RETIMED",
        )
        for point in retimed_points
    ]
    retimed_quality = analyze_teach_trajectory(
        retimed_samples if retimed_samples else prepared,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
    )
    return PreparedTeachReplay(
        samples=prepared,
        raw_quality=raw_quality,
        filtered_quality=filtered_quality,
        retimed_quality=retimed_quality,
        smoothing_applied=smoothing_applied,
        filter_applied=filter_applied,
        resample_applied=resample_applied,
        retime_applied=retime_applied,
        smoothing_window=max(int(smoothing_window), 1),
        filter_cutoff_hz=max(float(filter_cutoff_hz), 0.01),
        filter_sample_rate_hz=max(float(filter_sample_rate_hz), 1.0),
        resample_rate_hz=max(float(resample_rate_hz), 1.0),
        retimed_points=retimed_points,
        large_motion=large_motion,
        max_joint_span_rad=max_joint_span,
        total_joint_motion_rad=total_joint_motion,
        requested_replay_speed=requested_speed,
        effective_replay_speed=effective_speed,
        large_motion_max_speed=max(float(large_motion_max_speed), 0.01),
        time_parameterization_requested_method=(
            time_parameterization.requested_method if time_parameterization is not None else str(time_parameterization_method or "auto")
        ),
        time_parameterization_used_method=(
            time_parameterization.used_method if time_parameterization is not None else "none"
        ),
        time_parameterization_message=(
            time_parameterization.message if time_parameterization is not None else "retime disabled"
        ),
    )


def prepared_teach_replay_to_dict(prepared: PreparedTeachReplay) -> dict:
    return {
        "smoothing_applied": prepared.smoothing_applied,
        "filter_applied": prepared.filter_applied,
        "resample_applied": prepared.resample_applied,
        "retime_applied": prepared.retime_applied,
        "smoothing_window": prepared.smoothing_window,
        "filter_cutoff_hz": prepared.filter_cutoff_hz,
        "filter_sample_rate_hz": prepared.filter_sample_rate_hz,
        "resample_rate_hz": prepared.resample_rate_hz,
        "prepared_samples": len(prepared.samples),
        "retimed_points": len(prepared.retimed_points),
        "time_parameterization": {
            "requested_method": prepared.time_parameterization_requested_method,
            "used_method": prepared.time_parameterization_used_method,
            "message": prepared.time_parameterization_message,
        },
        "before_quality": teach_trajectory_quality_to_dict(prepared.before_quality),
        "after_quality": teach_trajectory_quality_to_dict(prepared.after_quality),
        "raw_quality": teach_trajectory_quality_to_dict(prepared.raw_quality),
        "filtered_quality": teach_trajectory_quality_to_dict(prepared.filtered_quality),
        "retimed_quality": teach_trajectory_quality_to_dict(prepared.retimed_quality),
        "large_motion": {
            "enabled": prepared.large_motion,
            "max_joint_span_rad": prepared.max_joint_span_rad,
            "total_joint_motion_rad": prepared.total_joint_motion_rad,
            "requested_speed": prepared.requested_replay_speed,
            "effective_speed": prepared.effective_replay_speed,
            "large_motion_max_speed": prepared.large_motion_max_speed,
        },
    }


def teach_trajectory_preview_to_dict(
    samples: list[TeachSample],
    *,
    max_points: int = 500,
) -> dict:
    quality = analyze_teach_trajectory(samples)
    if not samples:
        return {
            "joint_names": [],
            "raw_samples": 0,
            "returned_samples": 0,
            "downsample_step": 1,
            "duration_sec": 0.0,
            "quality": teach_trajectory_quality_to_dict(quality),
            "points": [],
            "events": [],
        }
    limit = max(int(max_points), 1)
    step = max(1, (len(samples) + limit - 1) // limit)
    first_stamp = float(samples[0].stamp)
    if len(samples) <= limit:
        selected_indices = list(range(len(samples)))
    elif limit == 1:
        selected_indices = [0]
    else:
        selected_indices = sorted(
            {
                round(index * (len(samples) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
    points = []
    for index in selected_indices:
        sample = samples[index]
        points.append(
            {
                "sample": index,
                "t": max(0.0, float(sample.stamp) - first_stamp),
                "positions": {
                    name: float(position)
                    for name, position in zip(sample.joint_names, sample.positions)
                },
                "arm_state": sample.arm_state,
            }
        )
    return {
        "joint_names": list(samples[0].joint_names),
        "raw_samples": len(samples),
        "returned_samples": len(points),
        "downsample_step": step,
        "duration_sec": max(0.0, float(samples[-1].stamp) - first_stamp),
        "quality": teach_trajectory_quality_to_dict(quality),
        "points": points,
        "events": [
            {
                "sample": event.sample,
                "joint_name": event.joint_name,
                "level": event.level,
                "message": event.message,
                "delta_rad": event.delta_rad,
                "velocity_rad_s": event.velocity_rad_s,
            }
            for event in quality.events
        ],
    }


def detect_teach_record_anomalies(
    samples: list[TeachSample],
    *,
    max_jump_rad: float = 0.75,
    max_velocity_rad_s: float = 2.0,
) -> tuple[str, ...]:
    anomalies: list[str] = []
    if not samples:
        return ()
    expected_joints = samples[0].joint_names
    previous = samples[0]
    for index, sample in enumerate(samples[1:], start=1):
        if sample.joint_names != expected_joints:
            anomalies.append(f"joint_names mismatch at sample {index}")
        dt = float(sample.stamp) - float(previous.stamp)
        if dt <= 0.0:
            anomalies.append(f"timestamp not monotonic at sample {index}")
        for joint_name, current, last in zip(expected_joints, sample.positions, previous.positions):
            delta = abs(float(current) - float(last))
            if delta > float(max_jump_rad):
                anomalies.append(f"{joint_name} jump {delta:.4f} rad at sample {index}")
            if dt > 0.0:
                velocity = delta / dt
                if velocity > float(max_velocity_rad_s):
                    anomalies.append(f"{joint_name} velocity {velocity:.4f} rad/s at sample {index}")
        previous = sample
    return tuple(anomalies)


def inspect_teach_record(
    path: str | Path,
    *,
    current_positions: dict[str, float] | None = None,
    direct_threshold: float = 0.01,
    align_threshold: float = 0.25,
) -> TeachRecordInfo:
    record_path = Path(path)
    if not record_path.exists():
        return TeachRecordInfo(
            path=str(record_path),
            exists=False,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="missing",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=(),
            message="record file does not exist",
        )
    try:
        samples = load_teach_samples(record_path)
    except Exception as exc:
        return TeachRecordInfo(
            path=str(record_path),
            exists=True,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="invalid",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=(f"invalid jsonl: {exc}",),
            message=f"failed to read record: {exc}",
        )
    if not samples:
        return TeachRecordInfo(
            path=str(record_path),
            exists=True,
            samples=0,
            duration_sec=0.0,
            joint_names=(),
            start_positions=(),
            end_positions=(),
            start_band="empty",
            max_error=None,
            worst_joint="",
            per_joint_error={},
            anomalies=("empty record",),
            message="record contains no samples",
        )
    first = samples[0]
    last = samples[-1]
    duration_sec = max(0.0, float(last.stamp) - float(first.stamp))
    per_joint_error: dict[str, float] = {}
    max_error: float | None = None
    worst_joint = ""
    start_band = "unknown"
    message = "current joint state unavailable"
    if current_positions is not None:
        missing = [name for name in first.joint_names if name not in current_positions]
        if missing:
            message = f"current joint state missing: {', '.join(missing)}"
        else:
            current = tuple(float(current_positions[name]) for name in first.joint_names)
            decision = classify_replay_start(
                current_positions=current,
                start_positions=first.positions,
                direct_threshold=direct_threshold,
                align_threshold=align_threshold,
            )
            per_joint_error = {
                name: float(error)
                for name, error in zip(first.joint_names, decision.per_joint_error)
            }
            max_error = float(decision.max_error)
            if per_joint_error:
                worst_joint = max(per_joint_error, key=per_joint_error.get)
            start_band = str(decision.band.value)
            message = decision.message
    quality = analyze_teach_trajectory(samples)
    return TeachRecordInfo(
        path=str(record_path),
        exists=True,
        samples=len(samples),
        duration_sec=duration_sec,
        joint_names=first.joint_names,
        start_positions=first.positions,
        end_positions=last.positions,
        start_band=start_band,
        max_error=max_error,
        worst_joint=worst_joint,
        per_joint_error=per_joint_error,
        anomalies=tuple(dict.fromkeys((*detect_teach_record_anomalies(samples), *quality.anomalies))),
        message=message,
        quality=quality,
    )


def teach_record_info_to_dict(info: TeachRecordInfo) -> dict:
    payload = {
        "path": info.path,
        "exists": info.exists,
        "samples": info.samples,
        "duration_sec": info.duration_sec,
        "joint_names": list(info.joint_names),
        "start_positions": {
            name: float(position)
            for name, position in zip(info.joint_names, info.start_positions)
        },
        "end_positions": {
            name: float(position)
            for name, position in zip(info.joint_names, info.end_positions)
        },
        "start_band": info.start_band,
        "max_error": info.max_error,
        "worst_joint": info.worst_joint,
        "per_joint_error": dict(info.per_joint_error),
        "anomalies": list(info.anomalies),
        "message": info.message,
    }
    if info.quality is not None:
        payload["quality"] = teach_trajectory_quality_to_dict(info.quality)
    return payload


def list_teach_record_files(directory: str | Path) -> list[dict]:
    base = Path(directory)
    if not base.exists():
        return []
    records: list[dict] = []
    for path in sorted(base.glob("*.jsonl")):
        if path.name.endswith(".prepared.jsonl"):
            continue
        stat = path.stat()
        info = teach_record_info_to_dict(inspect_teach_record(path))
        records.append(
            {
                "path": str(path),
                "name": path.name,
                "size_bytes": int(stat.st_size),
                "modified_time": float(stat.st_mtime),
                "samples": int(info["samples"]),
                "duration_sec": float(info["duration_sec"]),
                "start_band": str(info["start_band"]),
                "anomalies": list(info["anomalies"]),
            }
        )
    return records


def validate_teach_dry_run_request(start_band: str) -> TeachDryRunDecision:
    band = str(start_band or "").strip().lower()
    if band in (
        ReplayStartBand.DIRECT.value,
        ReplayStartBand.ALIGN.value,
        ReplayStartBand.MOVEIT_ALIGN.value,
    ):
        return TeachDryRunDecision(
            accepted=True,
            state="dry_run",
            message=f"dry-run accepted for {band} replay check",
        )
    return TeachDryRunDecision(
        accepted=False,
        state="blocked",
        message=f"dry-run blocked because file check is {band or 'unknown'}",
    )


def validate_teach_replay_execute_request(
    start_band: str,
    *,
    dry_run_passed: bool,
    risk_level: str = "green",
    prepared_risk_level: str | None = None,
    prepared_max_jump_rad: float | None = None,
    max_prepared_jump_rad: float = 0.02,
    retimed_max_acceleration_rad_s2: float | None = None,
    max_replay_acceleration_rad_s2: float = 5.0,
    retimed_max_jerk_rad_s3: float | None = None,
    max_replay_jerk_rad_s3: float = 20.0,
    replay_speed: float = 1.0,
    yellow_max_speed: float = 0.6,
) -> TeachDryRunDecision:
    band = str(start_band or "").strip().lower()
    raw_risk = str(risk_level or "unknown").strip().lower()
    risk = str(prepared_risk_level or raw_risk).strip().lower()
    if band not in (
        ReplayStartBand.DIRECT.value,
        ReplayStartBand.ALIGN.value,
        ReplayStartBand.MOVEIT_ALIGN.value,
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"real replay blocked because file check is {band or 'unknown'}",
        )
    if risk == "red":
        quality_name = "prepared trajectory quality" if prepared_risk_level else "trajectory quality"
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"real replay blocked because {quality_name} is red",
        )
    if prepared_max_jump_rad is not None and float(prepared_max_jump_rad) > float(max_prepared_jump_rad):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because prepared max jump "
                f"{float(prepared_max_jump_rad):.4f} rad exceeds {float(max_prepared_jump_rad):.4f} rad"
            ),
        )
    if (
        retimed_max_jerk_rad_s3 is not None
        and float(retimed_max_jerk_rad_s3) > float(max_replay_jerk_rad_s3)
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because retimed max jerk "
                f"{float(retimed_max_jerk_rad_s3):.4f} rad/s^3 exceeds "
                f"{float(max_replay_jerk_rad_s3):.4f} rad/s^3"
            ),
        )
    if (
        retimed_max_acceleration_rad_s2 is not None
        and float(retimed_max_acceleration_rad_s2) > float(max_replay_acceleration_rad_s2)
    ):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=(
                "real replay blocked because retimed max acceleration "
                f"{float(retimed_max_acceleration_rad_s2):.4f} rad/s^2 exceeds "
                f"{float(max_replay_acceleration_rad_s2):.4f} rad/s^2"
            ),
        )
    if risk == "yellow" and float(replay_speed) > float(yellow_max_speed):
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message=f"yellow replay speed must be <= {float(yellow_max_speed):.2f}",
        )
    if not dry_run_passed:
        return TeachDryRunDecision(
            accepted=False,
            state="blocked",
            message="real replay requires a successful dry-run first",
        )
    return TeachDryRunDecision(
        accepted=True,
        state="replaying",
        message=f"real replay accepted for {band} file check",
    )


def validate_teach_replay_stop_request(has_active_goal: bool) -> TeachDryRunDecision:
    if not has_active_goal:
        return TeachDryRunDecision(
            accepted=False,
            state="idle",
            message="no active teach replay goal",
        )
    return TeachDryRunDecision(
        accepted=True,
        state="cancel_requested",
        message="teach replay cancel requested",
    )


def normalize_teach_replay_settings(
    *,
    replay_speed: float,
    align_duration: float,
    align_steps: int,
    final_hold_sec: float = 1.0,
) -> dict[str, float | int]:
    return {
        "replay_speed": min(max(float(replay_speed), 0.1), 1.0),
        "align_duration": min(max(float(align_duration), 1.0), 10.0),
        "align_steps": min(max(int(align_steps), 2), 200),
        "final_hold_sec": 1.0,
    }


def estimate_teach_replay(
    *,
    samples: int,
    record_duration_sec: float,
    start_band: str,
    replay_speed: float,
    align_duration: float,
    align_steps: int,
    final_hold_sec: float = 0.0,
) -> dict[str, float | int | bool]:
    settings = normalize_teach_replay_settings(
        replay_speed=replay_speed,
        align_duration=align_duration,
        align_steps=align_steps,
        final_hold_sec=final_hold_sec,
    )
    speed = float(settings["replay_speed"])
    use_align = str(start_band or "").lower() == ReplayStartBand.ALIGN.value
    replay_duration = max(0.0, float(record_duration_sec)) / max(speed, 0.01)
    alignment_duration = float(settings["align_duration"]) if use_align else 0.0
    final_hold = float(settings["final_hold_sec"])
    return {
        "use_align": use_align,
        "alignment_duration_sec": alignment_duration,
        "estimated_duration_sec": alignment_duration + replay_duration + final_hold,
        "trajectory_points": max(int(samples), 0)
        + (int(settings["align_steps"]) if use_align else 0)
        + (1 if final_hold > 0.0 and int(samples) > 0 else 0),
        "replay_speed": speed,
        "align_duration": float(settings["align_duration"]),
        "align_steps": int(settings["align_steps"]),
        "final_hold_sec": final_hold,
    }

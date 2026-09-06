from __future__ import annotations

from bisect import bisect_right
import math
from statistics import fmean
from typing import Mapping, Sequence


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def analyze_run(run: Mapping[str, object]) -> dict[str, object]:
    command = run.get("command")
    samples = run.get("samples")
    if not isinstance(command, Mapping) or not isinstance(samples, list) or not samples:
        raise ValueError("run must contain command and non-empty samples")
    if tuple(command.get("joint_names", ())) != ARM_JOINT_NAMES:
        raise ValueError("run command uses unexpected joint names")
    normalized = [_normalize_sample(sample) for sample in samples]
    normalized.sort(key=lambda sample: sample["elapsed_sec"])
    duration = float(command["duration_sec"])
    start = tuple(float(value) for value in command["points"][0]["positions"])
    target = tuple(float(value) for value in command["points"][-1]["positions"])

    per_joint: dict[str, object] = {}
    for index, joint in enumerate(ARM_JOINT_NAMES):
        errors = [sample["desired_positions"][index] - sample["observed_positions"][index] for sample in normalized]
        observed = [sample["observed_positions"][index] for sample in normalized]
        velocities = [sample["velocities"][index] for sample in normalized]
        efforts = [sample["efforts"][index] for sample in normalized]
        per_joint[joint] = {
            "rms_tracking_error_rad": _rms(errors),
            "max_abs_tracking_error_rad": max(abs(value) for value in errors),
            "final_error_rad": target[index] - observed[-1],
            "peak_abs_velocity_rad_s": max(abs(value) for value in velocities),
            "peak_window_velocity_rad_s": _peak_window_velocity(normalized, index, 0.20),
            "settling_time_sec": _settling_time(normalized, index, duration),
            "overshoot_rad": _overshoot(observed, start[index], target[index]),
            "peak_abs_effort": max(abs(value) for value in efforts),
            "mean_abs_effort": fmean(abs(value) for value in efforts),
            "final_observed_position_rad": observed[-1],
        }
    return {
        "sample_count": len(normalized),
        "duration_sec": duration,
        "first_elapsed_sec": normalized[0]["elapsed_sec"],
        "last_elapsed_sec": normalized[-1]["elapsed_sec"],
        "per_joint": per_joint,
    }


def compare_runs(sim_run: Mapping[str, object], real_run: Mapping[str, object]) -> dict[str, object]:
    sim_command = sim_run.get("command", {})
    real_command = real_run.get("command", {})
    sim_hash = str(sim_command.get("command_sha256", ""))
    real_hash = str(real_command.get("command_sha256", ""))
    if not sim_hash or sim_hash != real_hash:
        raise ValueError("paired runs must use the same command_sha256")
    sim_samples = sorted(
        (_normalize_sample(sample) for sample in sim_run["samples"]),
        key=lambda sample: sample["elapsed_sec"],
    )
    real_samples = sorted(
        (_normalize_sample(sample) for sample in real_run["samples"]),
        key=lambda sample: sample["elapsed_sec"],
    )
    paired: dict[str, object] = {}
    for index, joint in enumerate(ARM_JOINT_NAMES):
        position_deltas = []
        effort_pairs = []
        for real_sample in real_samples:
            sim_position, sim_effort = _interpolate_values(
                sim_samples, real_sample["elapsed_sec"], index
            )
            position_deltas.append(real_sample["observed_positions"][index] - sim_position)
            effort_pairs.append((sim_effort, real_sample["efforts"][index]))
        paired[joint] = {
            "rms_real_minus_sim_position_rad": _rms(position_deltas),
            "max_abs_real_minus_sim_position_rad": max(abs(value) for value in position_deltas),
            "effort_trend_correlation": _correlation(effort_pairs),
        }
    return {
        "command_sha256": sim_hash,
        "effort_semantics": (
            "real effort is Damiao feedback torque; MuJoCo effort is applied generalized "
            "actuator force. Both arm fields are nominally N.m but are not directly "
            "calibrated as identical measurements."
        ),
        "sim": analyze_run(sim_run),
        "real": analyze_run(real_run),
        "paired": paired,
    }


def _normalize_sample(sample: Mapping[str, object]) -> dict[str, object]:
    if not isinstance(sample, Mapping):
        raise ValueError("samples must be mappings")
    elapsed = float(sample["elapsed_sec"])
    if not math.isfinite(elapsed) or elapsed < 0.0:
        raise ValueError("sample elapsed_sec must be finite and non-negative")
    return {
        "elapsed_sec": elapsed,
        "desired_positions": _vector6(sample["desired_positions"], "desired_positions"),
        "observed_positions": _vector6(sample["observed_positions"], "observed_positions"),
        "velocities": _vector6(sample["velocities"], "velocities"),
        "efforts": _vector6(sample["efforts"], "efforts"),
    }


def _peak_window_velocity(samples: Sequence[Mapping[str, object]], index: int, window: float) -> float:
    peak = 0.0
    start = 0
    for end in range(1, len(samples)):
        while start + 1 < end and samples[end]["elapsed_sec"] - samples[start + 1]["elapsed_sec"] >= window:
            start += 1
        dt = samples[end]["elapsed_sec"] - samples[start]["elapsed_sec"]
        if dt <= 0.0:
            continue
        delta = samples[end]["observed_positions"][index] - samples[start]["observed_positions"][index]
        peak = max(peak, abs(delta / dt))
    return peak


def _settling_time(samples: Sequence[Mapping[str, object]], index: int, duration: float) -> float | None:
    for position, sample in enumerate(samples):
        if sample["elapsed_sec"] + 1e-9 < duration:
            continue
        error = abs(sample["desired_positions"][index] - sample["observed_positions"][index])
        velocity = abs(sample["velocities"][index])
        if error > 0.02 or velocity > 0.05:
            continue
        horizon = sample["elapsed_sec"] + 0.25
        stable = [later for later in samples[position:] if later["elapsed_sec"] <= horizon + 1e-9]
        if stable and stable[-1]["elapsed_sec"] + 1e-9 >= horizon and all(
            abs(later["desired_positions"][index] - later["observed_positions"][index]) <= 0.02
            and abs(later["velocities"][index]) <= 0.05
            for later in stable
        ):
            return sample["elapsed_sec"] - duration
    return None


def _overshoot(observed: Sequence[float], start: float, target: float) -> float:
    delta = target - start
    if delta > 0.0:
        return max(0.0, max(observed) - target)
    if delta < 0.0:
        return max(0.0, target - min(observed))
    return max(abs(value - target) for value in observed)


def _interpolate_values(samples: Sequence[Mapping[str, object]], elapsed: float, index: int) -> tuple[float, float]:
    times = [sample["elapsed_sec"] for sample in samples]
    if elapsed <= times[0]:
        return samples[0]["observed_positions"][index], samples[0]["efforts"][index]
    if elapsed >= times[-1]:
        return samples[-1]["observed_positions"][index], samples[-1]["efforts"][index]
    upper = bisect_right(times, elapsed)
    lower = samples[upper - 1]
    higher = samples[upper]
    ratio = (elapsed - lower["elapsed_sec"]) / (higher["elapsed_sec"] - lower["elapsed_sec"])
    position = lower["observed_positions"][index] + (
        higher["observed_positions"][index] - lower["observed_positions"][index]
    ) * ratio
    effort = lower["efforts"][index] + (
        higher["efforts"][index] - lower["efforts"][index]
    ) * ratio
    return float(position), float(effort)


def _correlation(pairs: Sequence[tuple[float, float]]) -> float | None:
    if len(pairs) < 2:
        return None
    left = [pair[0] for pair in pairs]
    right = [pair[1] for pair in pairs]
    left_mean = fmean(left)
    right_mean = fmean(right)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in pairs)
    left_energy = sum((a - left_mean) ** 2 for a in left)
    right_energy = sum((b - right_mean) ** 2 for b in right)
    denominator = math.sqrt(left_energy * right_energy)
    return None if denominator <= 1e-15 else numerator / denominator


def _rms(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / float(len(values)))


def _vector6(values: Sequence[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != len(ARM_JOINT_NAMES) or any(not math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain six finite values")
    return result

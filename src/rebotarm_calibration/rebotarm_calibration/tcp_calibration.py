from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
import math

import numpy as np


Vector3 = tuple[float, float, float]


def _vector3(values: Sequence[float], name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def quaternion_to_rotation_matrix(values: Sequence[float]) -> np.ndarray:
    if len(values) != 4:
        raise ValueError("orientation must contain exactly 4 xyzw values")
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("orientation quaternion must be finite and non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def estimate_sample_offset(
    *,
    end_link_position: Sequence[float],
    end_link_orientation_xyzw: Sequence[float],
    tcp_reference_position: Sequence[float],
) -> Vector3:
    end_position = np.asarray(_vector3(end_link_position, "end_link_position"))
    reference_position = np.asarray(_vector3(tcp_reference_position, "tcp_reference_position"))
    rotation = quaternion_to_rotation_matrix(end_link_orientation_xyzw)
    offset = rotation.T @ (reference_position - end_position)
    return tuple(float(value) for value in offset)  # type: ignore[return-value]


def average_offsets(offsets: Iterable[Sequence[float]]) -> Vector3:
    samples = np.asarray([_vector3(offset, "offset") for offset in offsets], dtype=np.float64)
    if samples.size == 0:
        raise ValueError("at least one offset sample is required")
    result = np.mean(samples, axis=0)
    return tuple(float(value) for value in result)  # type: ignore[return-value]


def _rotation_distance_deg(left: np.ndarray, right: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(left.T @ right) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def analyze_tcp_samples(
    samples: Sequence[Mapping[str, Sequence[float]]],
    *,
    minimum_samples: int = 5,
    minimum_rotation_span_deg: float = 20.0,
    maximum_rms_residual_m: float = 0.003,
    maximum_residual_m: float = 0.006,
    maximum_axis_std_m: float = 0.003,
) -> dict[str, object]:
    if not samples:
        raise ValueError("at least one TCP sample is required")

    offsets: list[np.ndarray] = []
    rotations: list[np.ndarray] = []
    end_positions: list[np.ndarray] = []
    references: list[np.ndarray] = []
    for index, sample in enumerate(samples):
        try:
            end_position = np.asarray(_vector3(sample["end_link_position"], "end_link_position"))
            reference = np.asarray(_vector3(sample["tcp_reference_position"], "tcp_reference_position"))
            rotation = quaternion_to_rotation_matrix(sample["end_link_orientation_xyzw"])
        except KeyError as exc:
            raise ValueError(f"sample {index} missing field: {exc.args[0]}") from exc
        offsets.append(rotation.T @ (reference - end_position))
        rotations.append(rotation)
        end_positions.append(end_position)
        references.append(reference)

    offset_array = np.stack(offsets)
    estimate = np.mean(offset_array, axis=0)
    residual_vectors = np.stack(
        [end + rotation @ estimate - reference for end, rotation, reference in zip(end_positions, rotations, references)]
    )
    residual_norms = np.linalg.norm(residual_vectors, axis=1)
    rms = float(np.sqrt(np.mean(np.square(residual_norms))))
    maximum = float(np.max(residual_norms))
    axis_std = np.std(offset_array, axis=0)
    rotation_span = max(
        (_rotation_distance_deg(left, right) for i, left in enumerate(rotations) for right in rotations[i + 1 :]),
        default=0.0,
    )
    gates = {
        "sample_count": len(samples) >= int(minimum_samples),
        "rotation_span": rotation_span >= float(minimum_rotation_span_deg),
        "rms_residual": rms <= float(maximum_rms_residual_m),
        "maximum_residual": maximum <= float(maximum_residual_m),
        "axis_std": float(np.max(axis_std)) <= float(maximum_axis_std_m),
    }
    return {
        "sample_count": len(samples),
        "tcp_offset_xyz": estimate.tolist(),
        "per_sample_offset_xyz": offset_array.tolist(),
        "offset_axis_std_m": axis_std.tolist(),
        "reference_residual_vectors_m": residual_vectors.tolist(),
        "reference_residual_norms_m": residual_norms.tolist(),
        "rms_residual_m": rms,
        "maximum_residual_m": maximum,
        "rotation_span_deg": rotation_span,
        "limits": {
            "minimum_samples": int(minimum_samples),
            "minimum_rotation_span_deg": float(minimum_rotation_span_deg),
            "maximum_rms_residual_m": float(maximum_rms_residual_m),
            "maximum_residual_m": float(maximum_residual_m),
            "maximum_axis_std_m": float(maximum_axis_std_m),
        },
        "gates": gates,
        "passed": all(gates.values()),
    }


def analyze_tcp_pivot_samples(
    samples: Sequence[Mapping[str, Sequence[float]]],
    *,
    minimum_samples: int = 5,
    minimum_rotation_span_deg: float = 30.0,
    maximum_condition_number: float = 100.0,
    maximum_rms_residual_m: float = 0.003,
    maximum_residual_m: float = 0.006,
) -> dict[str, object]:
    """Solve ``p_end + R_end * tcp_offset = fixed_pivot``.

    Both the local TCP offset and the fixed pivot position in the base frame are
    unknown. This is the classic pivot/four-point TCP calibration model and does
    not depend on hand-eye calibration or a visually measured reference point.
    """

    if not samples:
        raise ValueError("at least one TCP pivot sample is required")
    rotations: list[np.ndarray] = []
    positions: list[np.ndarray] = []
    for index, sample in enumerate(samples):
        try:
            positions.append(
                np.asarray(_vector3(sample["end_link_position"], "end_link_position"))
            )
            rotations.append(
                quaternion_to_rotation_matrix(sample["end_link_orientation_xyzw"])
            )
        except KeyError as exc:
            raise ValueError(f"sample {index} missing field: {exc.args[0]}") from exc

    identity = np.eye(3, dtype=np.float64)
    matrix = np.vstack(
        [np.hstack((rotation, -identity)) for rotation in rotations]
    )
    target = np.concatenate([-position for position in positions])
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, target, rcond=None)
    tcp_offset = solution[:3]
    pivot_position = solution[3:]
    residual_vectors = np.stack(
        [
            position + rotation @ tcp_offset - pivot_position
            for position, rotation in zip(positions, rotations)
        ]
    )
    residual_norms = np.linalg.norm(residual_vectors, axis=1)
    rms = float(np.sqrt(np.mean(np.square(residual_norms))))
    maximum = float(np.max(residual_norms))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if len(singular_values) == 6 and singular_values[-1] > 1e-12
        else float("inf")
    )
    rotation_span = max(
        (
            _rotation_distance_deg(left, right)
            for index, left in enumerate(rotations)
            for right in rotations[index + 1 :]
        ),
        default=0.0,
    )
    gates = {
        "sample_count": len(samples) >= int(minimum_samples),
        "full_rank": int(rank) == 6,
        "condition_number": condition <= float(maximum_condition_number),
        "rotation_span": rotation_span >= float(minimum_rotation_span_deg),
        "rms_residual": rms <= float(maximum_rms_residual_m),
        "maximum_residual": maximum <= float(maximum_residual_m),
    }
    return {
        "method": "classic_tcp_pivot_unknown_reference",
        "sample_count": len(samples),
        "tcp_offset_xyz": tcp_offset.tolist(),
        "pivot_position_base_xyz": pivot_position.tolist(),
        "matrix_rank": int(rank),
        "condition_number": condition,
        "singular_values": singular_values.tolist(),
        "rotation_span_deg": rotation_span,
        "pivot_residual_vectors_m": residual_vectors.tolist(),
        "pivot_residual_norms_m": residual_norms.tolist(),
        "rms_residual_m": rms,
        "maximum_residual_m": maximum,
        "limits": {
            "minimum_samples": int(minimum_samples),
            "minimum_rotation_span_deg": float(minimum_rotation_span_deg),
            "maximum_condition_number": float(maximum_condition_number),
            "maximum_rms_residual_m": float(maximum_rms_residual_m),
            "maximum_residual_m": float(maximum_residual_m),
        },
        "gates": gates,
        "passed": all(gates.values()),
    }
def format_tcp_offset_yaml(offset: Sequence[float]) -> str:
    ox, oy, oz = _vector3(offset, "offset")
    return f"tcp_offset_xyz: [{ox:.6f}, {oy:.6f}, {oz:.6f}]"

from __future__ import annotations

import math

import numpy as np
import pytest

from rebot_b601_mapping.command_shaper import (
    CommandShaper,
    PointToPointTrajectory,
    TargetViolation,
)


LIMITS = (
    (-2.8, 2.8),
    (-3.14, 0.02),
    (-3.14, 0.0),
    (-1.87, 1.57),
    (-1.57, 1.57),
    (-3.14, 3.14),
)
BASELINE = (0.0, -1.0, -1.0, 0.0, 0.0, 0.0)


def make_shaper(baseline=BASELINE) -> CommandShaper:
    return CommandShaper(
        joint_limits=LIMITS,
        baseline_rad=baseline,
        max_speed_rad_s=0.5,
        max_acceleration_rad_s2=5.0,
        max_jerk_rad_s3=20.0,
    )


def test_first_step_starts_exactly_at_follower_baseline() -> None:
    shaper = make_shaper()
    shaper.reset(BASELINE)

    first = shaper.step(BASELINE, 0.02)

    assert first.position_rad == pytest.approx(BASELINE)
    assert first.velocity_rad_s == pytest.approx((0.0,) * 6)
    assert first.acceleration_rad_s2 == pytest.approx((0.0,) * 6)


def test_sequence_respects_speed_acceleration_and_jerk() -> None:
    shaper = make_shaper()
    shaper.reset(BASELINE)

    samples = [
        shaper.step((1.0, -1.0, -1.0, 0.0, 0.0, 0.0), 0.02)
        for _ in range(300)
    ]

    assert max(abs(sample.velocity_rad_s[0]) for sample in samples) <= 0.5 + 1e-9
    assert max(abs(sample.acceleration_rad_s2[0]) for sample in samples) <= 5.0 + 1e-9
    accelerations = [0.0] + [sample.acceleration_rad_s2[0] for sample in samples]
    jerks = np.diff(accelerations) / 0.02
    assert float(np.max(np.abs(jerks))) <= 20.0 + 1e-8
    assert abs(samples[-1].position_rad[0] - 1.0) < 0.02
    assert abs(samples[-1].velocity_rad_s[0]) < 0.05


def test_recorded_joint5_target_converges_without_limit_cycle() -> None:
    recorded_start = (
        -1.6096363067626953,
        -0.00667572021484375,
        -0.061226844787597656,
        0.11844825744628906,
        0.12378883361816406,
        0.10547828674316406,
    )
    historical_target = (-math.pi / 2.0, -0.1, -0.2, 0.2, 0.0, 0.0)
    shaper = make_shaper(recorded_start)
    shaper.reset(recorded_start)

    samples = [shaper.step(historical_target, 0.02) for _ in range(1500)]
    joint5_error = np.asarray(
        [sample.position_rad[4] - historical_target[4] for sample in samples]
    )
    nonzero_signs = np.sign(joint5_error[joint5_error != 0.0])
    target_crossings = int(np.count_nonzero(np.diff(nonzero_signs)))

    assert target_crossings <= 1
    assert max(abs(value) for value in joint5_error[-50:]) < 0.02
    assert max(abs(sample.velocity_rad_s[4]) for sample in samples[-50:]) < 0.05


def test_point_to_point_is_monotonic_and_stops_at_target() -> None:
    recorded_start = (
        -1.6096363067626953,
        -0.00667572021484375,
        -0.061226844787597656,
        0.11844825744628906,
        0.12378883361816406,
        0.10547828674316406,
    )
    historical_target = (-math.pi / 2.0, -0.1, -0.2, 0.2, 0.0, 0.0)
    trajectory = PointToPointTrajectory(
        start_rad=recorded_start,
        target_rad=historical_target,
        max_speed_rad_s=0.5,
        max_acceleration_rad_s2=5.0,
        max_jerk_rad_s3=20.0,
    )

    dt = 0.002
    samples = [trajectory.sample(index * dt) for index in range(15001)]
    joint5_positions = np.asarray([sample.position_rad[4] for sample in samples])
    accelerations = np.asarray([sample.acceleration_rad_s2 for sample in samples])
    numerical_jerk = np.diff(accelerations, axis=0) / dt

    assert np.all(np.diff(joint5_positions) <= 1e-12)
    assert samples[-1].position_rad == pytest.approx(historical_target)
    assert samples[-1].velocity_rad_s == pytest.approx((0.0,) * 6)
    assert samples[-1].acceleration_rad_s2 == pytest.approx((0.0,) * 6)
    assert (
        max(abs(value) for sample in samples for value in sample.velocity_rad_s)
        <= 0.5 + 1e-9
    )
    assert (
        max(
            abs(value)
            for sample in samples
            for value in sample.acceleration_rad_s2
        )
        <= 5.0 + 1e-9
    )
    assert float(np.max(np.abs(numerical_jerk))) <= 20.0 + 0.1


def test_point_to_point_factory_preserves_command_safety_validation() -> None:
    shaper = make_shaper((0.0, 0.014, -1.0, 0.0, 0.0, 0.0))

    with pytest.raises(TargetViolation, match="joint2"):
        shaper.point_to_point((0.0, 0.021, -1.0, 0.0, 0.0, 0.0))


def test_targets_at_web_joint_limits_are_allowed_but_beyond_are_rejected() -> None:
    baseline = (0.0, 0.014, -0.02002716064453125, 0.0, 0.0, 0.0)
    shaper = make_shaper(baseline)
    shaper.reset(baseline)

    shaper.step((0.0, 0.02, 0.0, 0.0, 0.0, 0.0), 0.02)

    with pytest.raises(TargetViolation, match="joint2"):
        shaper.step((0.0, 0.021, 0.0, 0.0, 0.0, 0.0), 0.02)
    with pytest.raises(TargetViolation, match="joint3"):
        shaper.step((0.0, 0.02, 0.001, 0.0, 0.0, 0.0), 0.02)


def test_target_inside_web_limit_is_allowed_beyond_startup_baseline_delta() -> None:
    shaper = make_shaper()
    shaper.reset(BASELINE)

    command = shaper.step((2.0, -1.0, -1.0, 0.0, 0.0, 0.0), 0.02)

    assert command.position_rad[0] > BASELINE[0]


@pytest.mark.parametrize(
    ("target", "message"),
    [
        ((0.0, np.nextafter(0.02, math.inf), -1.0, 0.0, 0.0, 0.0), "joint2"),
        ((0.0, -1.0, np.nextafter(0.0, math.inf), 0.0, 0.0, 0.0), "joint3"),
    ],
)
def test_smallest_float_beyond_web_joint_limit_is_rejected(
    target: tuple[float, ...],
    message: str,
) -> None:
    shaper = make_shaper()
    shaper.reset(BASELINE)

    with pytest.raises(TargetViolation, match=message):
        shaper.step(target, 0.02)


def test_boundary_projection_clears_outward_dynamic_state() -> None:
    baseline = (0.0, 0.019, -1.0, 0.0, 0.0, 0.0)
    shaper = make_shaper(baseline)
    shaper.reset(baseline)

    at_limit = shaper.step((0.0, 0.02, -1.0, 0.0, 0.0, 0.0), 1.0)

    assert at_limit.position_rad[1] == pytest.approx(0.02)
    assert at_limit.velocity_rad_s[1] == pytest.approx(0.0)
    assert at_limit.acceleration_rad_s2[1] == pytest.approx(0.0)

    inward = shaper.step((0.0, 0.019, -1.0, 0.0, 0.0, 0.0), 0.02)
    assert inward.position_rad[1] < 0.02


@pytest.mark.parametrize(
    ("target", "message"),
    [
        ((math.nan, -1.0, -1.0, 0.0, 0.0, 0.0), "有限"),
        ((0.0,) * 5, "六个"),
        ((0.0, 0.021, -1.0, 0.0, 0.0, 0.0), "joint2"),
    ],
)
def test_rejects_invalid_or_unsafe_raw_targets(target, message: str) -> None:
    shaper = make_shaper()
    shaper.reset(BASELINE)

    with pytest.raises(TargetViolation, match=message):
        shaper.step(target, 0.02)


@pytest.mark.parametrize("dt", [0.0, -0.01, math.inf])
def test_rejects_invalid_time_step(dt: float) -> None:
    shaper = make_shaper()
    shaper.reset(BASELINE)

    with pytest.raises(ValueError, match="dt"):
        shaper.step(BASELINE, dt)

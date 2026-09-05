from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from rebot_b601_mapping.mapping import (
    apply_confirmation,
    capture_baseline,
    infer_direction,
    map_virtual_follower,
    validate_paired_sample,
)
from rebot_b601_mapping.models import (
    Baseline,
    FollowerSample,
    LeaderSample,
    MotorFeedback,
    load_mapping_config,
)


CONFIG = load_mapping_config(Path(__file__).parents[1] / "mapping.example.json")
NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")


def leader(
    angles=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    *,
    timestamp=10.0,
) -> LeaderSample:
    return LeaderSample(timestamp_s=timestamp, angles_deg=tuple(angles))


def follower(
    positions=(0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0),
    *,
    velocities=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    statuses=(0, 0, 0, 0, 0, 0, 0),
    timestamp=10.0,
) -> FollowerSample:
    motors = tuple(
        MotorFeedback(
            name=name,
            position_rad=float(position),
            velocity_rad_s=float(velocity),
            torque_nm=0.0,
            status_code=int(status),
        )
        for name, position, velocity, status in zip(
            NAMES, positions, velocities, statuses, strict=True
        )
    )
    return FollowerSample(timestamp_s=timestamp, motors=motors)


def baseline() -> Baseline:
    return Baseline(
        captured_at_s=10.0,
        leader_angles_deg=(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0),
        follower_positions_rad=(1.0, -1.0, -1.0, 0.5, 0.5, 1.0, -1.0),
    )


def test_virtual_mapping_uses_relative_baseline_and_follower_signs() -> None:
    sample = leader((20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0))

    result = map_virtual_follower(sample, baseline(), CONFIG)

    delta = math.radians(10.0)
    assert result.leader_deltas_rad == pytest.approx((delta,) * 6)
    assert result.positions_rad == pytest.approx(
        (1.0 - delta, -1.0 - delta, -1.0 + delta, 0.5 + delta, 0.5 + delta, 1.0 - delta)
    )
    assert len(result.positions_rad) == 6


@pytest.mark.parametrize(
    ("leader_sample", "follower_sample", "now_s", "message"),
    [
        (leader(timestamp=9.0), follower(), 10.0, "引导臂样本已过期"),
        (leader(), follower(timestamp=9.0), 10.0, "从臂样本已过期"),
        (leader((math.nan, 0, 0, 0, 0, 0, 0)), follower(), 10.0, "非有限"),
        (
            leader(),
            follower(statuses=(0, 0, 1, 0, 0, 0, 0)),
            10.0,
            "joint3 status_code=1",
        ),
        (
            leader(),
            follower(positions=(3.0, -1, -1, 0, 0, 0, -1)),
            10.0,
            "joint1.*软限位",
        ),
    ],
)
def test_validate_paired_sample_rejects_unsafe_feedback(
    leader_sample: LeaderSample,
    follower_sample: FollowerSample,
    now_s: float,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        validate_paired_sample(leader_sample, follower_sample, CONFIG, now_s=now_s)


def test_capture_baseline_uses_axis_medians() -> None:
    pairs = []
    for offset in (0.02, -0.01, 0.0, 0.01, -0.02):
        pairs.append(
            (
                leader((10 + offset, 20, 30, 40, 50, 60, 70), timestamp=10.0),
                follower(
                    (1 + offset, -1, -1, 0.5, 0.5, 1, -1),
                    timestamp=10.0,
                ),
            )
        )

    result = capture_baseline(pairs, CONFIG, now_s=10.0)

    assert result.captured_at_s == 10.0
    assert result.leader_angles_deg == pytest.approx((10, 20, 30, 40, 50, 60, 70))
    assert result.follower_positions_rad == pytest.approx((1, -1, -1, 0.5, 0.5, 1, -1))


def test_capture_baseline_rejects_motion() -> None:
    moving = follower(velocities=(0, 0, 0.06, 0, 0, 0, 0))

    with pytest.raises(ValueError, match="joint3.*基线速度"):
        capture_baseline([(leader(), moving)] * 5, CONFIG, now_s=10.0)


def test_infer_direction_accepts_consistent_selected_joint_window() -> None:
    base = Baseline(
        captured_at_s=10.0,
        leader_angles_deg=(0.0,) * 7,
        follower_positions_rad=(0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0),
    )
    pairs = [
        (
            leader((10 + index, 0, 0, 0, 0, 0, 0), timestamp=10.1),
            follower(
                (-0.2 - index * 0.01, -1, -1, 0, 0, 0, -1),
                timestamp=10.1,
            ),
        )
        for index in range(5)
    ]

    evidence = infer_direction(base, pairs, "joint1", CONFIG, now_s=10.1)

    assert evidence.follower_name == "joint1"
    assert evidence.inferred_sign == -1
    assert evidence.candidate_sign == -1
    assert evidence.consistent is True
    assert evidence.verified is False


def test_infer_direction_rejects_other_joint_motion() -> None:
    base = Baseline(10.0, (0.0,) * 7, (0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0))
    pairs = [
        (
            leader((10, 2, 0, 0, 0, 0, 0), timestamp=10.1),
            follower((-0.2, -1, -1, 0, 0, 0, -1), timestamp=10.1),
        )
    ] * 5

    with pytest.raises(ValueError, match="joint2.*非选定关节"):
        infer_direction(base, pairs, "joint1", CONFIG, now_s=10.1)


def test_infer_direction_rejects_small_selected_motion() -> None:
    base = Baseline(10.0, (0.0,) * 7, (0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0))
    pairs = [
        (
            leader((1, 0, 0, 0, 0, 0, 0), timestamp=10.1),
            follower((-0.01, -1, -1, 0, 0, 0, -1), timestamp=10.1),
        )
    ] * 5

    with pytest.raises(ValueError, match="joint1.*运动幅度过小"):
        infer_direction(base, pairs, "joint1", CONFIG, now_s=10.1)


def test_infer_direction_rejects_inconsistent_signs() -> None:
    base = Baseline(10.0, (0.0,) * 7, (0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0))
    follower_deltas = (-0.2, -0.2, 0.2, -0.2, -0.2)
    pairs = [
        (
            leader((10, 0, 0, 0, 0, 0, 0), timestamp=10.1),
            follower((delta, -1, -1, 0, 0, 0, -1), timestamp=10.1),
        )
        for delta in follower_deltas
    ]

    with pytest.raises(ValueError, match="joint1.*符号不一致"):
        infer_direction(base, pairs, "joint1", CONFIG, now_s=10.1)


def test_confirmation_verifies_only_matching_candidate_sign() -> None:
    base = Baseline(10.0, (0.0,) * 7, (0.0, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0))
    pairs = [
        (
            leader((10, 0, 0, 0, 0, 0, 0), timestamp=10.1),
            follower((-0.2, -1, -1, 0, 0, 0, -1), timestamp=10.1),
        )
    ] * 5
    evidence = infer_direction(base, pairs, "joint1", CONFIG, now_s=10.1)

    assert apply_confirmation(evidence, False).verified is False
    assert apply_confirmation(evidence, True).verified is True
    assert apply_confirmation(replace(evidence, candidate_sign=1), True).verified is False

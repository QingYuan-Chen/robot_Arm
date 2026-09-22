from __future__ import annotations

from pathlib import Path

import pytest

from rebotarm_simulation.sim2real.randomization import (
    RandomizationConfig,
    RandomizationRange,
)


CONFIG = (
    Path(__file__).parents[1]
    / "src"
    / "rebotarm_simulation"
    / "config"
    / "sim2real_randomization.yaml"
)


def test_default_yaml_is_identity_and_zero_noise():
    config = RandomizationConfig.from_yaml(CONFIG)
    sample = config.sample(seed=7)

    assert sample.mass_scale == 1.0
    assert sample.damping_scale == 1.0
    assert sample.friction_scale == 1.0
    assert sample.torque_scale == 1.0
    assert sample.control_latency_steps == 0
    assert sample.action_noise_std == 0.0
    assert sample.position_noise_std == 0.0
    assert sample.velocity_noise_std == 0.0


def test_training_profile_is_seeded_and_stays_inside_ranges():
    config = RandomizationConfig.from_yaml(CONFIG, profile="training_profile")

    first = config.sample(seed=7)
    second = config.sample(seed=7)
    other = config.sample(seed=8)

    assert first == second
    assert first != other
    for value, bounds in (
        (first.mass_scale, config.mass_scale),
        (first.damping_scale, config.damping_scale),
        (first.friction_scale, config.friction_scale),
        (first.torque_scale, config.torque_scale),
        (first.action_noise_std, config.action_noise_std),
        (first.position_noise_std, config.position_noise_std),
        (first.velocity_noise_std, config.velocity_noise_std),
    ):
        assert bounds.minimum <= value <= bounds.maximum
    assert config.control_latency_steps.minimum <= first.control_latency_steps <= config.control_latency_steps.maximum


def test_range_rejects_reversed_nonfinite_and_nonpositive_physical_values():
    with pytest.raises(ValueError, match="ordered"):
        RandomizationRange(2.0, 1.0)
    with pytest.raises(ValueError, match="finite"):
        RandomizationRange(float("nan"), 1.0)
    with pytest.raises(ValueError, match="positive"):
        RandomizationRange(0.0, 1.0, strictly_positive=True)


def test_yaml_rejects_unknown_profile_and_invalid_range(tmp_path):
    config_path = tmp_path / "bad.yaml"
    config_path.write_text(
        "default:\n"
        "  mass_scale: [1.0, 0.9]\n"
        "  damping_scale: [1.0, 1.0]\n"
        "  friction_scale: [1.0, 1.0]\n"
        "  torque_scale: [1.0, 1.0]\n"
        "  control_latency_steps: [0, 0]\n"
        "  action_noise_std: [0.0, 0.0]\n"
        "  position_noise_std: [0.0, 0.0]\n"
        "  velocity_noise_std: [0.0, 0.0]\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="ordered"):
        RandomizationConfig.from_yaml(config_path)

    config_path.write_text("default: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="profile"):
        RandomizationConfig.from_yaml(config_path, profile="missing")

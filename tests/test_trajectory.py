"""Tests for the zero-derivative quintic trajectory."""

from __future__ import annotations

import numpy as np
import pytest

from src.trajectory import QuinticTrajectory


def test_quintic_has_exact_zero_endpoint_velocity_and_acceleration() -> None:
    start = np.zeros(7)
    goal = np.linspace(-0.2, 0.2, 7)
    trajectory = QuinticTrajectory(start, goal, duration=5.0)

    first = trajectory.sample(0.0)
    last = trajectory.sample(5.0)
    assert np.array_equal(first.position, start)
    assert np.array_equal(last.position, goal)
    assert np.array_equal(first.velocity, np.zeros(7))
    assert np.array_equal(last.velocity, np.zeros(7))
    assert np.array_equal(first.acceleration, np.zeros(7))
    assert np.array_equal(last.acceleration, np.zeros(7))


def test_entire_quintic_path_stays_between_endpoints() -> None:
    start = np.array([0.0, -0.1, 0.2, 0.3, -0.2, 0.1, 0.0])
    goal = np.array([-0.2, -0.3, 0.4, 0.5, -0.1, 0.2, 0.15])
    trajectory = QuinticTrajectory(start, goal, duration=5.0)
    lower = np.minimum(start, goal) - 0.01
    upper = np.maximum(start, goal) + 0.01

    trajectory.validate_limits(lower, upper)
    samples = np.asarray(
        [trajectory.sample(t).position for t in np.linspace(0, 5, 501)]
    )
    assert np.all(samples >= lower)
    assert np.all(samples <= upper)


def test_out_of_limit_endpoint_is_rejected() -> None:
    trajectory = QuinticTrajectory(np.zeros(7), np.ones(7), duration=5.0)
    with pytest.raises(ValueError, match="planning limit"):
        trajectory.validate_limits(np.full(7, -0.5), np.full(7, 0.5))


@pytest.mark.parametrize("duration", [0.0, -1.0, np.inf, np.nan])
def test_invalid_duration_is_rejected(duration: float) -> None:
    with pytest.raises(ValueError, match="duration"):
        QuinticTrajectory(np.zeros(7), np.ones(7), duration=duration)

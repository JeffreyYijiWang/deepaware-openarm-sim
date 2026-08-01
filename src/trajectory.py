"""Smooth deterministic joint-space trajectories."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    """Position, velocity, and acceleration at one trajectory time."""

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


@dataclass(frozen=True)
class QuinticTrajectory:
    """Point-to-point quintic interpolation with zero endpoint derivatives."""

    start: np.ndarray
    goal: np.ndarray
    duration: float

    def __post_init__(self) -> None:
        start = np.asarray(self.start, dtype=float)
        goal = np.asarray(self.goal, dtype=float)
        if start.shape != goal.shape or start.ndim != 1:
            raise ValueError("Trajectory start and goal must be matching vectors")
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
            raise ValueError("Trajectory endpoints must be finite")
        if not np.isfinite(self.duration) or self.duration <= 0:
            raise ValueError("Trajectory duration must be finite and positive")
        object.__setattr__(self, "start", start.copy())
        object.__setattr__(self, "goal", goal.copy())

    def sample(self, time_s: float) -> TrajectorySample:
        """Sample the trajectory, holding exactly at either endpoint."""
        if not np.isfinite(time_s):
            raise ValueError("Trajectory time must be finite")
        if time_s <= 0:
            zeros = np.zeros_like(self.start)
            return TrajectorySample(self.start.copy(), zeros, zeros.copy())
        if time_s >= self.duration:
            zeros = np.zeros_like(self.goal)
            return TrajectorySample(self.goal.copy(), zeros, zeros.copy())

        phase = time_s / self.duration
        blend = 10 * phase**3 - 15 * phase**4 + 6 * phase**5
        blend_rate = (30 * phase**2 - 60 * phase**3 + 30 * phase**4) / self.duration
        blend_accel = (60 * phase - 180 * phase**2 + 120 * phase**3) / self.duration**2
        delta = self.goal - self.start
        return TrajectorySample(
            position=self.start + blend * delta,
            velocity=blend_rate * delta,
            acceleration=blend_accel * delta,
        )

    def validate_limits(self, lower: np.ndarray, upper: np.ndarray) -> None:
        """Prove the monotonic quintic path remains inside supplied limits."""
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if lower.shape != self.start.shape or upper.shape != self.start.shape:
            raise ValueError("Trajectory limits must match endpoint shape")
        endpoint_min = np.minimum(self.start, self.goal)
        endpoint_max = np.maximum(self.start, self.goal)
        if np.any(endpoint_min < lower) or np.any(endpoint_max > upper):
            raise ValueError("Quintic trajectory crosses a planning limit")

"""Minimal joint-space PD controller with external bias compensation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TorqueCommand:
    """Separated feedback, bias, and total requested torque."""

    feedback: np.ndarray
    bias: np.ndarray
    requested: np.ndarray


@dataclass(frozen=True)
class JointSpacePD:
    """Vector PD feedback; MuJoCo bias is supplied explicitly per DoF."""

    kp: np.ndarray
    kd: np.ndarray

    def __post_init__(self) -> None:
        kp = np.asarray(self.kp, dtype=float)
        kd = np.asarray(self.kd, dtype=float)
        if kp.shape != (7,) or kd.shape != (7,):
            raise ValueError("Controller gains must contain exactly seven values")
        if not np.all(np.isfinite(kp)) or not np.all(np.isfinite(kd)):
            raise ValueError("Controller gains must be finite")
        if np.any(kp <= 0) or np.any(kd <= 0):
            raise ValueError("Controller gains must be positive")
        object.__setattr__(self, "kp", kp.copy())
        object.__setattr__(self, "kd", kd.copy())

    def compute(
        self,
        desired_position: np.ndarray,
        desired_velocity: np.ndarray,
        measured_position: np.ndarray,
        measured_velocity: np.ndarray,
        bias_force: np.ndarray,
    ) -> TorqueCommand:
        """Compute PD feedback and add the named-DoF MuJoCo bias force."""
        arrays = tuple(
            np.asarray(value, dtype=float)
            for value in (
                desired_position,
                desired_velocity,
                measured_position,
                measured_velocity,
                bias_force,
            )
        )
        if any(value.shape != (7,) for value in arrays):
            raise ValueError("Controller inputs must each contain seven values")
        (
            desired_position,
            desired_velocity,
            measured_position,
            measured_velocity,
            bias,
        ) = arrays
        feedback = self.kp * (desired_position - measured_position) + self.kd * (
            desired_velocity - measured_velocity
        )
        return TorqueCommand(
            feedback=feedback,
            bias=bias.copy(),
            requested=feedback + bias,
        )

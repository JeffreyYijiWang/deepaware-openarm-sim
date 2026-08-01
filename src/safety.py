"""Pre-step software safety checks for the selected seven joints."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class SafetyFault(RuntimeError):
    """A named rule violation that must stop the experiment."""

    def __init__(self, rule: str, detail: str) -> None:
        self.rule = rule
        self.detail = detail
        super().__init__(f"{rule}: {detail}")


@dataclass(frozen=True)
class SafetyLimits:
    """Hard, planning, velocity, torque, and divergence limits."""

    hard_position_lower: np.ndarray
    hard_position_upper: np.ndarray
    planning_position_lower: np.ndarray
    planning_position_upper: np.ndarray
    velocity: np.ndarray
    normal_torque: np.ndarray
    hard_torque: np.ndarray
    tracking_divergence: float

    def __post_init__(self) -> None:
        names = (
            "hard_position_lower",
            "hard_position_upper",
            "planning_position_lower",
            "planning_position_upper",
            "velocity",
            "normal_torque",
            "hard_torque",
        )
        for name in names:
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (7,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must contain seven finite values")
            object.__setattr__(self, name, value.copy())
        if np.any(self.hard_position_lower >= self.hard_position_upper):
            raise ValueError("Hard lower limits must be below upper limits")
        if np.any(self.planning_position_lower < self.hard_position_lower) or np.any(
            self.planning_position_upper > self.hard_position_upper
        ):
            raise ValueError("Planning limits must stay inside hard limits")
        if np.any(self.planning_position_lower >= self.planning_position_upper):
            raise ValueError("Planning lower limits must be below upper limits")
        if np.any(self.velocity <= 0) or np.any(self.normal_torque <= 0):
            raise ValueError("Velocity and torque limits must be positive")
        if np.any(self.normal_torque > self.hard_torque):
            raise ValueError("Normal torque limits cannot exceed hard torque limits")
        if not np.isfinite(self.tracking_divergence) or self.tracking_divergence <= 0:
            raise ValueError("Tracking-divergence threshold must be positive")


@dataclass(frozen=True)
class SafetySnapshot:
    """Safe clamped command and margins for logging."""

    applied_torque: np.ndarray
    torque_saturated: np.ndarray
    position_limit_margin: np.ndarray
    velocity_limit_margin: np.ndarray


class SafetyMonitor:
    """Evaluate all safety rules before every MuJoCo step."""

    def __init__(self, limits: SafetyLimits) -> None:
        self.limits = limits
        self.violation_count = 0
        self.nonfinite_sample_count = 0

    def _fault(self, rule: str, detail: str, *, nonfinite: bool = False) -> None:
        self.violation_count += 1
        if nonfinite:
            self.nonfinite_sample_count += 1
        raise SafetyFault(rule, detail)

    def evaluate(
        self,
        *,
        desired_position: np.ndarray,
        desired_velocity: np.ndarray,
        measured_position: np.ndarray,
        measured_velocity: np.ndarray,
        feedback_torque: np.ndarray,
        bias_torque: np.ndarray,
        requested_torque: np.ndarray,
    ) -> SafetySnapshot:
        """Check state/command safety and apply the normal torque derating cap."""
        signals = {
            "desired_position": np.asarray(desired_position, dtype=float),
            "desired_velocity": np.asarray(desired_velocity, dtype=float),
            "measured_position": np.asarray(measured_position, dtype=float),
            "measured_velocity": np.asarray(measured_velocity, dtype=float),
            "feedback_torque": np.asarray(feedback_torque, dtype=float),
            "bias_torque": np.asarray(bias_torque, dtype=float),
            "requested_torque": np.asarray(requested_torque, dtype=float),
        }
        for name, value in signals.items():
            if value.shape != (7,):
                self._fault("shape", f"{name} has shape {value.shape}, expected (7,)")
            if not np.all(np.isfinite(value)):
                self._fault(
                    "non_finite", f"{name} contains a non-finite value", nonfinite=True
                )

        desired_position = signals["desired_position"]
        measured_position = signals["measured_position"]
        measured_velocity = signals["measured_velocity"]
        requested_torque = signals["requested_torque"]

        if np.any(desired_position < self.limits.planning_position_lower) or np.any(
            desired_position > self.limits.planning_position_upper
        ):
            self._fault(
                "desired_planning_limit", "desired position crossed planning limits"
            )
        if np.any(measured_position < self.limits.hard_position_lower) or np.any(
            measured_position > self.limits.hard_position_upper
        ):
            self._fault(
                "measured_hard_position_limit", "measured position crossed hard limits"
            )
        if np.any(np.abs(measured_velocity) > self.limits.velocity):
            self._fault(
                "measured_velocity_limit", "measured velocity exceeded software limit"
            )
        if np.any(np.abs(requested_torque) > self.limits.hard_torque):
            self._fault(
                "requested_hard_torque_limit", "requested torque exceeded peak limit"
            )
        if np.any(
            np.abs(desired_position - measured_position)
            > self.limits.tracking_divergence
        ):
            self._fault(
                "tracking_divergence", "tracking error exceeded divergence threshold"
            )

        applied_torque = np.clip(
            requested_torque, -self.limits.normal_torque, self.limits.normal_torque
        )
        torque_saturated = ~np.isclose(
            applied_torque, requested_torque, atol=1e-12, rtol=0
        )
        if np.any(np.abs(applied_torque) > self.limits.normal_torque + 1e-12):
            self._fault("internal_torque_clamp", "applied torque escaped normal limits")

        position_margin = np.minimum(
            measured_position - self.limits.hard_position_lower,
            self.limits.hard_position_upper - measured_position,
        )
        velocity_margin = self.limits.velocity - np.abs(measured_velocity)
        return SafetySnapshot(
            applied_torque=applied_torque,
            torque_saturated=torque_saturated,
            position_limit_margin=position_margin,
            velocity_limit_margin=velocity_margin,
        )

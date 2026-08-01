"""Explicit, stateful software safety checks for the selected seven joints.

This module protects a simulation experiment. It is not a certified hardware
safety system and cannot replace an independent physical E-stop and
power-isolation path.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import mujoco

    from .model_mapping import ArmMapping


JOINT_COUNT = 7


def _seven_finite(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (JOINT_COUNT,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain seven finite values")
    return array.copy()


@dataclass(frozen=True)
class PlanningLimits:
    """Limits used only to construct and validate desired trajectories."""

    position_lower: np.ndarray
    position_upper: np.ndarray
    velocity: np.ndarray
    max_position_step: np.ndarray

    def __post_init__(self) -> None:
        for name in (
            "position_lower",
            "position_upper",
            "velocity",
            "max_position_step",
        ):
            object.__setattr__(self, name, _seven_finite(getattr(self, name), name))
        if np.any(self.position_lower >= self.position_upper):
            raise ValueError("Planning lower limits must be below upper limits")
        if np.any(self.velocity <= 0) or np.any(self.max_position_step <= 0):
            raise ValueError(
                "Planning velocity and position-step limits must be positive"
            )


@dataclass(frozen=True)
class NormalCommandLimits:
    """Non-faulting limits applied to ordinary controller commands."""

    torque: np.ndarray
    torque_rate: np.ndarray

    def __post_init__(self) -> None:
        for name in ("torque", "torque_rate"):
            object.__setattr__(self, name, _seven_finite(getattr(self, name), name))
        if np.any(self.torque <= 0) or np.any(self.torque_rate <= 0):
            raise ValueError("Normal torque and torque-rate limits must be positive")


@dataclass(frozen=True)
class AbsoluteFaultLimits:
    """Bounds whose violation latches the subsystem in FAULT."""

    position_lower: np.ndarray
    position_upper: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray
    tracking_error: np.ndarray

    def __post_init__(self) -> None:
        for name in (
            "position_lower",
            "position_upper",
            "velocity",
            "torque",
            "tracking_error",
        ):
            object.__setattr__(self, name, _seven_finite(getattr(self, name), name))
        if np.any(self.position_lower >= self.position_upper):
            raise ValueError("Hard lower limits must be below upper limits")
        if (
            np.any(self.velocity <= 0)
            or np.any(self.torque <= 0)
            or np.any(self.tracking_error <= 0)
        ):
            raise ValueError(
                "Absolute velocity, torque, and tracking limits must be positive"
            )


@dataclass(frozen=True)
class TemporalSafetyConfig:
    """Timing, persistence, and watchdog policy."""

    nominal_timestep: float
    timestep_tolerance_fraction: float
    tracking_persistence_samples: int
    command_timeout: float
    feedback_timeout: float

    def __post_init__(self) -> None:
        scalars = {
            "nominal_timestep": self.nominal_timestep,
            "timestep_tolerance_fraction": self.timestep_tolerance_fraction,
            "command_timeout": self.command_timeout,
            "feedback_timeout": self.feedback_timeout,
        }
        if any(not np.isfinite(value) or value <= 0 for value in scalars.values()):
            raise ValueError("Temporal safety values must be finite and positive")
        if self.tracking_persistence_samples < 2:
            raise ValueError("Tracking persistence must require at least two samples")


@dataclass(frozen=True)
class SafetyConfig:
    """Explicitly separated planning, normal, absolute, and timing policy."""

    planning: PlanningLimits
    normal: NormalCommandLimits
    absolute: AbsoluteFaultLimits
    temporal: TemporalSafetyConfig
    safe_behavior: str = "zero_torque"

    def __post_init__(self) -> None:
        if np.any(
            self.planning.position_lower < self.absolute.position_lower
        ) or np.any(self.planning.position_upper > self.absolute.position_upper):
            raise ValueError("Planning limits must stay inside hard position limits")
        if np.any(self.normal.torque > self.absolute.torque):
            raise ValueError(
                "Normal torque limits cannot exceed absolute torque limits"
            )
        if self.safe_behavior != "zero_torque":
            raise ValueError(
                "Only the verified zero_torque simulation behavior is supported"
            )


class SafetyState(str, Enum):
    RUNNING = "RUNNING"
    FAULT = "FAULT"


@dataclass(frozen=True)
class SafetyEvent:
    reason: str
    timestamp: float
    detail: str


class SafetyFault(RuntimeError):
    """A timestamped, named violation that must stop the active trajectory."""

    def __init__(self, event: SafetyEvent, safe_torque: np.ndarray) -> None:
        self.event = event
        self.rule = event.reason
        self.detail = event.detail
        self.timestamp = event.timestamp
        self.safe_torque = np.asarray(safe_torque, dtype=float).copy()
        super().__init__(f"{event.reason} at t={event.timestamp:.9f}s: {event.detail}")


@dataclass(frozen=True)
class SafetySnapshot:
    """Safe command and observable non-faulting interventions for logging."""

    applied_torque: np.ndarray
    torque_saturated: np.ndarray
    normal_torque_saturated: np.ndarray
    torque_rate_limited: np.ndarray
    position_limit_margin: np.ndarray
    velocity_limit_margin: np.ndarray


class SafetyMonitor:
    """Evaluate safety before every step and latch on the first fault."""

    def __init__(self, config: SafetyConfig) -> None:
        self.config = config
        self.state = SafetyState.RUNNING
        self.active_trajectory = True
        self.events: list[SafetyEvent] = []
        self.violation_count = 0
        self.nonfinite_sample_count = 0
        self.saturation_sample_count = 0
        self._last_simulation_time: float | None = None
        self._last_command_timestamp: float | None = None
        self._last_feedback_timestamp: float | None = None
        self._last_desired_position: np.ndarray | None = None
        self._last_applied_torque: np.ndarray | None = None
        self._tracking_divergence_samples = 0

    @property
    def safe_torque(self) -> np.ndarray:
        """Configured safe simulated response after a fault."""
        return np.zeros(JOINT_COUNT, dtype=float)

    def _fault(
        self,
        rule: str,
        detail: str,
        timestamp: float,
        *,
        nonfinite: bool = False,
    ) -> None:
        event = SafetyEvent(rule, float(timestamp), detail)
        self.state = SafetyState.FAULT
        self.active_trajectory = False
        self.events.append(event)
        self.violation_count += 1
        if nonfinite:
            self.nonfinite_sample_count += 1
        raise SafetyFault(event, self.safe_torque)

    def _require_running(self, timestamp: float) -> None:
        if self.state is SafetyState.FAULT:
            first = self.events[0]
            raise SafetyFault(first, self.safe_torque)
        if not np.isfinite(timestamp):
            self._fault(
                "non_finite", "simulation time is not finite", 0.0, nonfinite=True
            )

    def trip(self, rule: str, detail: str, timestamp: float) -> None:
        """Latch an integration-level fault through the same event path."""
        self._require_running(timestamp)
        self._fault(rule, detail, timestamp)

    def _signal(self, value: np.ndarray, name: str, timestamp: float) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (JOINT_COUNT,):
            self._fault(
                "shape", f"{name} has shape {array.shape}, expected (7,)", timestamp
            )
        if not np.all(np.isfinite(array)):
            self._fault(
                "non_finite",
                f"{name} contains a non-finite value",
                timestamp,
                nonfinite=True,
            )
        return array

    def _check_timing(
        self,
        simulation_time: float,
        command_timestamp: float | None,
        feedback_timestamp: float | None,
        simulation_timestep: float,
    ) -> None:
        if not np.isfinite(simulation_timestep) or simulation_timestep <= 0:
            self._fault(
                "invalid_timestep",
                "simulation timestep must be positive",
                simulation_time,
            )
        expected = self.config.temporal.nominal_timestep
        tolerance = expected * self.config.temporal.timestep_tolerance_fraction
        if abs(simulation_timestep - expected) > tolerance:
            self._fault(
                "abnormal_timestep",
                f"timestep {simulation_timestep} differs from nominal {expected}",
                simulation_time,
            )
        if self._last_simulation_time is not None:
            advance = simulation_time - self._last_simulation_time
            if advance <= 0:
                self._fault(
                    "simulation_time_not_advancing",
                    "simulation time did not increase",
                    simulation_time,
                )
            if abs(advance - expected) > tolerance:
                self._fault(
                    "abnormal_time_advance",
                    f"simulation advanced by {advance}, expected {expected}",
                    simulation_time,
                )

        self._check_source_timestamp(
            "command",
            command_timestamp,
            simulation_time,
            self._last_command_timestamp,
            self.config.temporal.command_timeout,
        )
        self._check_source_timestamp(
            "feedback",
            feedback_timestamp,
            simulation_time,
            self._last_feedback_timestamp,
            self.config.temporal.feedback_timeout,
        )

    def _check_source_timestamp(
        self,
        source: str,
        timestamp: float | None,
        simulation_time: float,
        previous: float | None,
        timeout: float,
    ) -> None:
        if timestamp is None:
            self._fault(
                f"missing_{source}", f"{source} timestamp is missing", simulation_time
            )
        assert timestamp is not None
        if not np.isfinite(timestamp):
            self._fault(
                "non_finite",
                f"{source} timestamp is not finite",
                simulation_time,
                nonfinite=True,
            )
        if timestamp > simulation_time + 1e-12:
            self._fault(
                f"future_{source}",
                f"{source} timestamp is in the future",
                simulation_time,
            )
        if previous is not None and timestamp <= previous:
            self._fault(
                f"nonmonotonic_{source}_timestamp",
                f"{source} timestamp did not increase",
                simulation_time,
            )
        if simulation_time - timestamp > timeout + 1e-12:
            self._fault(
                f"stale_{source}", f"{source} age exceeded {timeout}s", simulation_time
            )

    def evaluate(
        self,
        *,
        simulation_time: float,
        command_timestamp: float | None,
        feedback_timestamp: float | None,
        simulation_timestep: float,
        desired_position: np.ndarray,
        desired_velocity: np.ndarray,
        measured_position: np.ndarray,
        measured_velocity: np.ndarray,
        feedback_torque: np.ndarray,
        bias_torque: np.ndarray,
        requested_torque: np.ndarray,
    ) -> SafetySnapshot:
        """Validate one sample and return the constrained ordinary command."""
        self._require_running(simulation_time)
        self._check_timing(
            simulation_time, command_timestamp, feedback_timestamp, simulation_timestep
        )
        signals = {
            name: self._signal(value, name, simulation_time)
            for name, value in {
                "desired_position": desired_position,
                "desired_velocity": desired_velocity,
                "measured_position": measured_position,
                "measured_velocity": measured_velocity,
                "feedback_torque": feedback_torque,
                "bias_torque": bias_torque,
                "requested_torque": requested_torque,
            }.items()
        }
        desired_position = signals["desired_position"]
        desired_velocity = signals["desired_velocity"]
        measured_position = signals["measured_position"]
        measured_velocity = signals["measured_velocity"]
        requested_torque = signals["requested_torque"]

        if np.any(desired_position < self.config.planning.position_lower) or np.any(
            desired_position > self.config.planning.position_upper
        ):
            self._fault(
                "desired_planning_limit",
                "desired position crossed planning limits",
                simulation_time,
            )
        if np.any(np.abs(desired_velocity) > self.config.planning.velocity):
            self._fault(
                "desired_velocity_limit",
                "desired velocity crossed trajectory limits",
                simulation_time,
            )
        if self._last_desired_position is not None and np.any(
            np.abs(desired_position - self._last_desired_position)
            > self.config.planning.max_position_step
        ):
            self._fault(
                "desired_discontinuity",
                "desired position changed discontinuously",
                simulation_time,
            )

        if np.any(measured_position < self.config.absolute.position_lower) or np.any(
            measured_position > self.config.absolute.position_upper
        ):
            self._fault(
                "measured_hard_position_limit",
                "measured position crossed hard limits",
                simulation_time,
            )
        if np.any(np.abs(measured_velocity) > self.config.absolute.velocity):
            self._fault(
                "measured_velocity_limit",
                "measured velocity crossed hard limits",
                simulation_time,
            )
        if np.any(np.abs(requested_torque) > self.config.absolute.torque):
            self._fault(
                "requested_hard_torque_limit",
                "requested torque crossed absolute limits",
                simulation_time,
            )

        divergent = np.any(
            np.abs(desired_position - measured_position)
            > self.config.absolute.tracking_error
        )
        self._tracking_divergence_samples = (
            self._tracking_divergence_samples + 1 if divergent else 0
        )
        if (
            self._tracking_divergence_samples
            >= self.config.temporal.tracking_persistence_samples
        ):
            self._fault(
                "tracking_divergence",
                f"tracking error persisted for {self._tracking_divergence_samples} samples",
                simulation_time,
            )

        normal_clipped = np.clip(
            requested_torque, -self.config.normal.torque, self.config.normal.torque
        )
        normal_saturated = ~np.isclose(
            normal_clipped, requested_torque, atol=1e-12, rtol=0
        )
        if self._last_applied_torque is None:
            applied_torque = normal_clipped.copy()
            rate_limited = np.zeros(JOINT_COUNT, dtype=bool)
        else:
            max_change = self.config.normal.torque_rate * simulation_timestep
            applied_torque = np.clip(
                normal_clipped,
                self._last_applied_torque - max_change,
                self._last_applied_torque + max_change,
            )
            rate_limited = ~np.isclose(
                applied_torque, normal_clipped, atol=1e-12, rtol=0
            )
        self.verify_applied_torque(applied_torque, simulation_time)
        saturated = normal_saturated | rate_limited
        if np.any(saturated):
            self.saturation_sample_count += 1

        position_margin = np.minimum(
            measured_position - self.config.absolute.position_lower,
            self.config.absolute.position_upper - measured_position,
        )
        velocity_margin = self.config.absolute.velocity - np.abs(measured_velocity)
        self._last_simulation_time = float(simulation_time)
        self._last_command_timestamp = float(command_timestamp)
        self._last_feedback_timestamp = float(feedback_timestamp)
        self._last_desired_position = desired_position.copy()
        self._last_applied_torque = applied_torque.copy()
        return SafetySnapshot(
            applied_torque=applied_torque,
            torque_saturated=saturated,
            normal_torque_saturated=normal_saturated,
            torque_rate_limited=rate_limited,
            position_limit_margin=position_margin,
            velocity_limit_margin=velocity_margin,
        )

    def verify_applied_torque(
        self,
        torque: np.ndarray,
        timestamp: float,
        *,
        expected: np.ndarray | None = None,
    ) -> None:
        """Validate actuator-force feedback after the command reaches MuJoCo."""
        applied = self._signal(torque, "applied_torque", timestamp)
        if np.any(np.abs(applied) > self.config.normal.torque + 1e-12):
            self._fault(
                "applied_torque_limit",
                "applied torque escaped normal limits",
                timestamp,
            )
        if expected is not None:
            expected_torque = self._signal(
                expected, "expected_applied_torque", timestamp
            )
            if not np.allclose(applied, expected_torque, atol=1e-12, rtol=0):
                self._fault(
                    "actuator_force_mismatch",
                    "applied actuator force differs from constrained command",
                    timestamp,
                )

    def verify_mapping(
        self, model: mujoco.MjModel, mapping: ArmMapping, timestamp: float
    ) -> None:
        """Fault if current named lookup differs from the startup fingerprint."""
        from .model_mapping import ModelMappingError, verify_mapping_unchanged

        try:
            verify_mapping_unchanged(model, mapping)
        except ModelMappingError as exc:
            self._fault("mapping_drift", str(exc), timestamp)

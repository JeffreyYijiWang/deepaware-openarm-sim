"""Deterministic positive and negative tests for every software safety rule."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from src.safety import (
    AbsoluteFaultLimits,
    NormalCommandLimits,
    PlanningLimits,
    SafetyConfig,
    SafetyFault,
    SafetyMonitor,
    SafetyState,
    TemporalSafetyConfig,
)


@pytest.fixture
def config() -> SafetyConfig:
    return SafetyConfig(
        planning=PlanningLimits(
            position_lower=np.full(7, -0.8),
            position_upper=np.full(7, 0.8),
            velocity=np.full(7, 1.0),
            max_position_step=np.full(7, 0.1),
        ),
        normal=NormalCommandLimits(
            torque=np.full(7, 1.0), torque_rate=np.full(7, 10.0)
        ),
        absolute=AbsoluteFaultLimits(
            position_lower=np.full(7, -1.0),
            position_upper=np.full(7, 1.0),
            velocity=np.full(7, 2.0),
            torque=np.full(7, 2.0),
            tracking_error=np.full(7, 0.5),
        ),
        temporal=TemporalSafetyConfig(
            nominal_timestep=0.002,
            timestep_tolerance_fraction=0.25,
            tracking_persistence_samples=3,
            command_timeout=0.01,
            feedback_timeout=0.01,
        ),
    )


def safe_inputs(time_s: float = 0.0) -> dict[str, object]:
    zeros = np.zeros(7)
    return {
        "simulation_time": time_s,
        "command_timestamp": time_s,
        "feedback_timestamp": time_s,
        "simulation_timestep": 0.002,
        "desired_position": zeros.copy(),
        "desired_velocity": zeros.copy(),
        "measured_position": zeros.copy(),
        "measured_velocity": zeros.copy(),
        "feedback_torque": zeros.copy(),
        "bias_torque": zeros.copy(),
        "requested_torque": zeros.copy(),
    }


def test_safe_sample_passes_and_reports_positive_margins(config: SafetyConfig) -> None:
    snapshot = SafetyMonitor(config).evaluate(**safe_inputs())
    assert not np.any(snapshot.torque_saturated)
    assert np.all(snapshot.position_limit_margin == 1.0)
    assert np.all(snapshot.velocity_limit_margin == 2.0)


def test_every_joint_vector_requires_shape_seven(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["desired_velocity"] = np.zeros(6)
    with pytest.raises(SafetyFault, match="shape"):
        SafetyMonitor(config).evaluate(**inputs)


def test_nan_command_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["requested_torque"][3] = np.nan
    monitor = SafetyMonitor(config)
    with pytest.raises(SafetyFault, match="non_finite"):
        monitor.evaluate(**inputs)
    assert monitor.nonfinite_sample_count == 1


def test_infinity_state_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["measured_position"][2] = np.inf
    with pytest.raises(SafetyFault, match="non_finite"):
        SafetyMonitor(config).evaluate(**inputs)


def test_nonpositive_timestep_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["simulation_timestep"] = 0.0
    with pytest.raises(SafetyFault, match="invalid_timestep"):
        SafetyMonitor(config).evaluate(**inputs)


def test_abnormal_configured_timestep_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["simulation_timestep"] = 0.003
    with pytest.raises(SafetyFault, match="abnormal_timestep"):
        SafetyMonitor(config).evaluate(**inputs)


def test_nonfinite_simulation_timestamp_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["simulation_time"] = np.nan
    with pytest.raises(SafetyFault, match="non_finite"):
        SafetyMonitor(config).evaluate(**inputs)


@pytest.mark.parametrize("source", ["command_timestamp", "feedback_timestamp"])
def test_nonfinite_source_timestamp_faults(config: SafetyConfig, source: str) -> None:
    inputs = safe_inputs()
    inputs[source] = np.nan
    with pytest.raises(SafetyFault, match="non_finite"):
        SafetyMonitor(config).evaluate(**inputs)


@pytest.mark.parametrize(
    ("source", "rule"),
    [
        ("command_timestamp", "future_command"),
        ("feedback_timestamp", "future_feedback"),
    ],
)
def test_future_source_timestamp_faults(
    config: SafetyConfig, source: str, rule: str
) -> None:
    inputs = safe_inputs()
    inputs[source] = 0.001
    with pytest.raises(SafetyFault, match=rule):
        SafetyMonitor(config).evaluate(**inputs)


def test_nonmonotonic_command_timestamp_faults(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    monitor.evaluate(**safe_inputs(0.0))
    inputs = safe_inputs(0.002)
    inputs["command_timestamp"] = 0.0
    with pytest.raises(SafetyFault, match="nonmonotonic_command_timestamp"):
        monitor.evaluate(**inputs)


def test_nonmonotonic_feedback_timestamp_faults(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    monitor.evaluate(**safe_inputs(0.0))
    inputs = safe_inputs(0.002)
    inputs["feedback_timestamp"] = 0.0
    with pytest.raises(SafetyFault, match="nonmonotonic_feedback_timestamp"):
        monitor.evaluate(**inputs)


def test_simulation_time_must_advance_normally(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    monitor.evaluate(**safe_inputs(0.0))
    with pytest.raises(SafetyFault, match="simulation_time_not_advancing"):
        monitor.evaluate(**safe_inputs(0.0))


def test_abnormal_simulation_time_jump_faults(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    monitor.evaluate(**safe_inputs(0.0))
    with pytest.raises(SafetyFault, match="abnormal_time_advance"):
        monitor.evaluate(**safe_inputs(0.004))


def test_desired_planning_limit_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["desired_position"][0] = 0.81
    with pytest.raises(SafetyFault, match="desired_planning_limit"):
        SafetyMonitor(config).evaluate(**inputs)


def test_desired_velocity_limit_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["desired_velocity"][0] = 1.01
    with pytest.raises(SafetyFault, match="desired_velocity_limit"):
        SafetyMonitor(config).evaluate(**inputs)


def test_desired_position_discontinuity_faults(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    monitor.evaluate(**safe_inputs(0.0))
    inputs = safe_inputs(0.002)
    inputs["desired_position"][0] = 0.11
    with pytest.raises(SafetyFault, match="desired_discontinuity"):
        monitor.evaluate(**inputs)


def test_positive_torque_is_clipped_and_saturation_logged(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["requested_torque"][6] = 1.5
    monitor = SafetyMonitor(config)
    snapshot = monitor.evaluate(**inputs)
    assert snapshot.applied_torque[6] == pytest.approx(1.0)
    assert snapshot.normal_torque_saturated[6]
    assert snapshot.torque_saturated[6]
    assert monitor.saturation_sample_count == 1


def test_negative_torque_is_clipped(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["requested_torque"][1] = -1.5
    snapshot = SafetyMonitor(config).evaluate(**inputs)
    assert snapshot.applied_torque[1] == pytest.approx(-1.0)
    assert snapshot.normal_torque_saturated[1]


def test_absolute_torque_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["requested_torque"][4] = 2.01
    with pytest.raises(SafetyFault, match="requested_hard_torque_limit"):
        SafetyMonitor(config).evaluate(**inputs)


def test_torque_rate_is_limited_and_logged(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    monitor.evaluate(**safe_inputs(0.0))
    inputs = safe_inputs(0.002)
    inputs["requested_torque"][0] = 0.5
    snapshot = monitor.evaluate(**inputs)
    assert snapshot.applied_torque[0] == pytest.approx(0.02)
    assert snapshot.torque_rate_limited[0]
    assert snapshot.torque_saturated[0]


def test_nonfinite_applied_torque_feedback_faults(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    actual = np.zeros(7)
    actual[0] = np.nan
    with pytest.raises(SafetyFault, match="non_finite"):
        monitor.verify_applied_torque(actual, 0.0)


def test_applied_torque_above_normal_limit_faults(config: SafetyConfig) -> None:
    actual = np.zeros(7)
    actual[0] = 1.01
    with pytest.raises(SafetyFault, match="applied_torque_limit"):
        SafetyMonitor(config).verify_applied_torque(actual, 0.0)


def test_applied_torque_mismatch_faults(config: SafetyConfig) -> None:
    actual = np.zeros(7)
    actual[0] = 0.1
    with pytest.raises(SafetyFault, match="actuator_force_mismatch"):
        SafetyMonitor(config).verify_applied_torque(actual, 0.0, expected=np.zeros(7))


def test_hard_position_limit_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["measured_position"][1] = -1.01
    with pytest.raises(SafetyFault, match="measured_hard_position_limit"):
        SafetyMonitor(config).evaluate(**inputs)


def test_hard_velocity_limit_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["measured_velocity"][2] = 2.01
    with pytest.raises(SafetyFault, match="measured_velocity_limit"):
        SafetyMonitor(config).evaluate(**inputs)


def test_persistent_tracking_divergence_faults(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    for index in range(2):
        inputs = safe_inputs(index * 0.002)
        inputs["desired_position"][5] = 0.6
        monitor.evaluate(**inputs)
    inputs = safe_inputs(0.004)
    inputs["desired_position"][5] = 0.6
    with pytest.raises(SafetyFault, match="tracking_divergence"):
        monitor.evaluate(**inputs)


def test_single_tracking_transient_does_not_fault(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    inputs = safe_inputs(0.0)
    inputs["measured_position"][5] = -0.6
    monitor.evaluate(**inputs)
    monitor.evaluate(**safe_inputs(0.002))
    assert monitor.state is SafetyState.RUNNING


def test_controller_sign_error_causes_persistent_divergence_fault(
    config: SafetyConfig,
) -> None:
    monitor = SafetyMonitor(config)
    for index in range(3):
        inputs = safe_inputs(index * 0.002)
        inputs["desired_position"][0] = 0.3
        inputs["measured_position"][0] = -0.3
        if index < 2:
            monitor.evaluate(**inputs)
        else:
            with pytest.raises(SafetyFault, match="tracking_divergence"):
                monitor.evaluate(**inputs)


def test_stale_command_watchdog_fault_stops_and_logs(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    inputs = safe_inputs(0.012)
    inputs["command_timestamp"] = 0.0
    with pytest.raises(SafetyFault, match="stale_command") as raised:
        monitor.evaluate(**inputs)
    assert monitor.state is SafetyState.FAULT
    assert not monitor.active_trajectory
    assert np.array_equal(raised.value.safe_torque, np.zeros(7))
    assert monitor.events[-1].reason == "stale_command"
    assert monitor.events[-1].timestamp == pytest.approx(0.012)


def test_missing_command_watchdog_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["command_timestamp"] = None
    with pytest.raises(SafetyFault, match="missing_command"):
        SafetyMonitor(config).evaluate(**inputs)


def test_stale_feedback_watchdog_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs(0.012)
    inputs["feedback_timestamp"] = 0.0
    with pytest.raises(SafetyFault, match="stale_feedback"):
        SafetyMonitor(config).evaluate(**inputs)


def test_missing_feedback_watchdog_faults(config: SafetyConfig) -> None:
    inputs = safe_inputs()
    inputs["feedback_timestamp"] = None
    with pytest.raises(SafetyFault, match="missing_feedback"):
        SafetyMonitor(config).evaluate(**inputs)


def test_fault_is_latched(config: SafetyConfig) -> None:
    monitor = SafetyMonitor(config)
    inputs = safe_inputs()
    inputs["requested_torque"][0] = 3.0
    with pytest.raises(SafetyFault, match="requested_hard_torque_limit"):
        monitor.evaluate(**inputs)
    with pytest.raises(SafetyFault, match="requested_hard_torque_limit"):
        monitor.evaluate(**safe_inputs(0.002))


def test_invalid_limit_relationship_is_rejected(config: SafetyConfig) -> None:
    invalid_normal = replace(config.normal, torque=np.full(7, 3.0))
    with pytest.raises(ValueError, match="Normal torque"):
        replace(config, normal=invalid_normal)

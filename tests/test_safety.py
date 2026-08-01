"""Negative tests proving every pre-step safety fault is detected."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from src.safety import SafetyFault, SafetyLimits, SafetyMonitor


@pytest.fixture
def limits() -> SafetyLimits:
    return SafetyLimits(
        hard_position_lower=np.full(7, -1.0),
        hard_position_upper=np.full(7, 1.0),
        planning_position_lower=np.full(7, -0.8),
        planning_position_upper=np.full(7, 0.8),
        velocity=np.full(7, 2.0),
        normal_torque=np.full(7, 1.0),
        hard_torque=np.full(7, 2.0),
        tracking_divergence=0.5,
    )


def safe_inputs() -> dict[str, np.ndarray]:
    zeros = np.zeros(7)
    return {
        "desired_position": zeros.copy(),
        "desired_velocity": zeros.copy(),
        "measured_position": zeros.copy(),
        "measured_velocity": zeros.copy(),
        "feedback_torque": zeros.copy(),
        "bias_torque": zeros.copy(),
        "requested_torque": zeros.copy(),
    }


def test_safe_sample_passes_and_reports_positive_margins(limits: SafetyLimits) -> None:
    snapshot = SafetyMonitor(limits).evaluate(**safe_inputs())
    assert not np.any(snapshot.torque_saturated)
    assert np.all(snapshot.position_limit_margin == 1.0)
    assert np.all(snapshot.velocity_limit_margin == 2.0)


def test_nonfinite_signal_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["bias_torque"][3] = np.nan
    monitor = SafetyMonitor(limits)
    with pytest.raises(SafetyFault, match="non_finite"):
        monitor.evaluate(**inputs)
    assert monitor.nonfinite_sample_count == 1
    assert monitor.violation_count == 1


def test_wrong_signal_shape_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["desired_velocity"] = np.zeros(6)
    with pytest.raises(SafetyFault, match="shape"):
        SafetyMonitor(limits).evaluate(**inputs)


def test_desired_planning_limit_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["desired_position"][0] = 0.81
    with pytest.raises(SafetyFault, match="desired_planning_limit"):
        SafetyMonitor(limits).evaluate(**inputs)


def test_measured_hard_position_limit_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["measured_position"][1] = -1.01
    with pytest.raises(SafetyFault, match="measured_hard_position_limit"):
        SafetyMonitor(limits).evaluate(**inputs)


def test_measured_velocity_limit_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["measured_velocity"][2] = 2.01
    with pytest.raises(SafetyFault, match="measured_velocity_limit"):
        SafetyMonitor(limits).evaluate(**inputs)


def test_requested_hard_torque_limit_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["requested_torque"][4] = 2.01
    with pytest.raises(SafetyFault, match="requested_hard_torque_limit"):
        SafetyMonitor(limits).evaluate(**inputs)


def test_tracking_divergence_faults(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["desired_position"][5] = 0.6
    with pytest.raises(SafetyFault, match="tracking_divergence"):
        SafetyMonitor(limits).evaluate(**inputs)


def test_normal_torque_saturation_is_detected_and_clamped(limits: SafetyLimits) -> None:
    inputs = safe_inputs()
    inputs["requested_torque"][6] = 1.5
    snapshot = SafetyMonitor(limits).evaluate(**inputs)
    assert snapshot.torque_saturated[6]
    assert snapshot.applied_torque[6] == pytest.approx(1.0)


def test_invalid_limit_relationship_is_rejected(limits: SafetyLimits) -> None:
    with pytest.raises(ValueError, match="Normal torque"):
        replace(limits, normal_torque=np.full(7, 3.0))

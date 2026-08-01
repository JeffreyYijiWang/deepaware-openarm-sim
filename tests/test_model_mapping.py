"""Regression tests for named mapping and the torque actuator interface."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from src.experiment import load_parameters
from src.model_mapping import (
    LEFT_ACTUATOR_NAMES,
    LEFT_JOINT_NAMES,
    ModelMappingError,
    adapt_selected_actuators_to_torque,
    load_bimanual_model,
    resolve_arm_mapping,
    verify_torque_interface,
)


def test_exact_named_mapping_is_unique_and_targets_expected_joints() -> None:
    model, _ = load_bimanual_model()
    mapping = resolve_arm_mapping(model)

    assert mapping.joint_names == LEFT_JOINT_NAMES
    assert mapping.actuator_names == LEFT_ACTUATOR_NAMES
    assert len(np.unique(mapping.joint_ids)) == 7
    assert len(np.unique(mapping.qpos_addresses)) == 7
    assert len(np.unique(mapping.dof_addresses)) == 7
    assert len(np.unique(mapping.actuator_ids)) == 7
    for channel in mapping.channels:
        actuator = model.actuator(channel.actuator_name)
        assert int(actuator.trnid[0]) == channel.joint_id
        assert float(actuator.gear[0]) == pytest.approx(1.0)


def test_duplicate_named_selection_is_rejected() -> None:
    model, _ = load_bimanual_model()
    duplicated = (*LEFT_JOINT_NAMES[:-1], LEFT_JOINT_NAMES[0])
    with pytest.raises(ModelMappingError, match="unique"):
        resolve_arm_mapping(model, duplicated, LEFT_ACTUATOR_NAMES)


def test_selected_actuators_become_direct_positive_joint_torque() -> None:
    model, _ = load_bimanual_model()
    mapping = resolve_arm_mapping(model)
    parameters = load_parameters()
    adapt_selected_actuators_to_torque(model, mapping, parameters.normal_torque)
    data = mujoco.MjData(model)
    data.qpos[mapping.qpos_addresses] = parameters.start_position
    mujoco.mj_forward(model, data)

    verify_torque_interface(model, data, mapping, parameters.normal_torque)
    for channel, limit in zip(mapping.channels, parameters.normal_torque, strict=True):
        actuator = model.actuator(channel.actuator_name)
        assert int(actuator.biastype[0]) == int(mujoco.mjtBias.mjBIAS_NONE)
        assert np.allclose(actuator.biasprm, 0.0)
        assert float(actuator.gainprm[0]) == pytest.approx(1.0)
        assert actuator.ctrlrange.tolist() == pytest.approx([-limit, limit])


def test_normal_torque_above_model_force_limit_is_rejected() -> None:
    model, _ = load_bimanual_model()
    mapping = resolve_arm_mapping(model)
    excessive = np.full(7, 1000.0)
    with pytest.raises(ModelMappingError, match="exceeds"):
        adapt_selected_actuators_to_torque(model, mapping, excessive)

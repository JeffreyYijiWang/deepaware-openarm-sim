"""Named OpenArm model mapping and torque-interface validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
import numpy as np
from openarm_mujoco import v2

if TYPE_CHECKING:
    from collections.abc import Sequence


LEFT_JOINT_NAMES = tuple(f"openarm_left_joint{i}" for i in range(1, 8))
LEFT_ACTUATOR_NAMES = tuple(f"left_joint{i}_ctrl" for i in range(1, 8))


class ModelMappingError(ValueError):
    """Raised when the official model does not match the expected arm mapping."""


@dataclass(frozen=True)
class JointChannel:
    """Resolved identifiers for one named joint/actuator channel."""

    joint_name: str
    actuator_name: str
    joint_id: int
    qpos_address: int
    dof_address: int
    actuator_id: int


@dataclass(frozen=True)
class ArmMapping:
    """The seven named channels and their resolved model-array addresses."""

    channels: tuple[JointChannel, ...]
    joint_ids: np.ndarray
    qpos_addresses: np.ndarray
    dof_addresses: np.ndarray
    actuator_ids: np.ndarray

    def __post_init__(self) -> None:
        validate_mapping_uniqueness(
            self.joint_ids,
            self.qpos_addresses,
            self.dof_addresses,
            self.actuator_ids,
        )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(channel.joint_name for channel in self.channels)

    @property
    def actuator_names(self) -> tuple[str, ...]:
        return tuple(channel.actuator_name for channel in self.channels)

    def read_state(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        """Read selected position and velocity using startup-resolved addresses."""
        return (
            data.qpos[self.qpos_addresses].copy(),
            data.qvel[self.dof_addresses].copy(),
        )

    def read_bias(self, data: mujoco.MjData) -> np.ndarray:
        """Read gravity/Coriolis/centrifugal bias at selected DoF addresses."""
        return data.qfrc_bias[self.dof_addresses].copy()


def resolve_bimanual_model_path() -> Path:
    """Resolve the official v2 bimanual asset, with a packaging fallback."""
    candidate = Path(v2.openarm_bimanual_xml()).resolve()
    if candidate.is_file():
        return candidate

    # Microsoft Store Python can report a data prefix outside an active venv.
    # Search ancestors of the imported package without assuming a venv name.
    package_file = Path(v2.__file__).resolve()
    relative = Path("share/openarm_mujoco/v2/openarm_bimanual.xml")
    for parent in package_file.parents:
        fallback = parent / relative
        if fallback.is_file():
            return fallback.resolve()
    raise FileNotFoundError(f"OpenArm v2 bimanual model not found: {candidate}")


def load_bimanual_model() -> tuple[mujoco.MjModel, Path]:
    """Load the selected official model without modifying its XML."""
    model_path = resolve_bimanual_model_path()
    return mujoco.MjModel.from_xml_path(str(model_path)), model_path


def _scalar_address(values: np.ndarray, label: str) -> int:
    flat = np.asarray(values).reshape(-1)
    if flat.size != 1:
        raise ModelMappingError(f"{label} is not a one-DoF scalar address: {flat}")
    return int(flat[0])


def validate_mapping_uniqueness(
    joint_ids: np.ndarray,
    qpos_addresses: np.ndarray,
    dof_addresses: np.ndarray,
    actuator_ids: np.ndarray,
) -> None:
    """Reject malformed seven-channel mappings, including duplicate addresses."""
    for label, raw_values in (
        ("joint IDs", joint_ids),
        ("qpos addresses", qpos_addresses),
        ("DoF addresses", dof_addresses),
        ("actuator IDs", actuator_ids),
    ):
        values = np.asarray(raw_values, dtype=int)
        if values.shape != (7,):
            raise ModelMappingError(f"Selected {label} must have shape (7,): {values}")
        if np.unique(values).size != 7:
            raise ModelMappingError(f"Selected {label} are not unique: {values}")


def resolve_arm_mapping(
    model: mujoco.MjModel,
    joint_names: Sequence[str] = LEFT_JOINT_NAMES,
    actuator_names: Sequence[str] = LEFT_ACTUATOR_NAMES,
) -> ArmMapping:
    """Resolve and validate seven joint channels once through named access."""
    if len(joint_names) != 7 or len(actuator_names) != 7:
        raise ModelMappingError("Exactly seven joint and actuator names are required")
    if len(set(joint_names)) != 7 or len(set(actuator_names)) != 7:
        raise ModelMappingError("Joint and actuator names must be unique")

    channels: list[JointChannel] = []
    for joint_name, actuator_name in zip(joint_names, actuator_names, strict=True):
        try:
            joint = model.joint(joint_name)
            actuator = model.actuator(actuator_name)
        except KeyError as exc:
            raise ModelMappingError(f"Missing named model object: {exc}") from exc

        joint_id = int(joint.id)
        actuator_id = int(actuator.id)
        qpos_address = _scalar_address(joint.qposadr, f"{joint_name}.qposadr")
        dof_address = _scalar_address(joint.dofadr, f"{joint_name}.dofadr")

        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise ModelMappingError(f"{joint_name} is not a hinge joint")
        if int(actuator.trntype[0]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            raise ModelMappingError(f"{actuator_name} is not a joint transmission")
        if int(actuator.trnid[0]) != joint_id:
            raise ModelMappingError(
                f"{actuator_name} targets joint {int(actuator.trnid[0])}, "
                f"expected named joint {joint_id}"
            )
        if not np.isclose(float(actuator.gear[0]), 1.0) or not np.allclose(
            actuator.gear[1:], 0.0
        ):
            raise ModelMappingError(f"{actuator_name} does not have scalar gear +1")

        channels.append(
            JointChannel(
                joint_name=joint_name,
                actuator_name=actuator_name,
                joint_id=joint_id,
                qpos_address=qpos_address,
                dof_address=dof_address,
                actuator_id=actuator_id,
            )
        )

    joint_ids = np.asarray([channel.joint_id for channel in channels], dtype=int)
    qpos_addresses = np.asarray(
        [channel.qpos_address for channel in channels], dtype=int
    )
    dof_addresses = np.asarray([channel.dof_address for channel in channels], dtype=int)
    actuator_ids = np.asarray([channel.actuator_id for channel in channels], dtype=int)
    return ArmMapping(
        channels=tuple(channels),
        joint_ids=joint_ids,
        qpos_addresses=qpos_addresses,
        dof_addresses=dof_addresses,
        actuator_ids=actuator_ids,
    )


def verify_mapping_unchanged(model: mujoco.MjModel, startup: ArmMapping) -> None:
    """Re-resolve named channels and reject any index/transmission drift."""
    current = resolve_arm_mapping(model, startup.joint_names, startup.actuator_names)
    comparisons = (
        ("joint ID", startup.joint_ids, current.joint_ids),
        ("qpos address", startup.qpos_addresses, current.qpos_addresses),
        ("DoF address", startup.dof_addresses, current.dof_addresses),
        ("actuator ID", startup.actuator_ids, current.actuator_ids),
    )
    for label, expected, actual in comparisons:
        if not np.array_equal(expected, actual):
            raise ModelMappingError(
                f"Named {label} mapping changed: startup={expected}, current={actual}"
            )


def adapt_selected_actuators_to_torque(
    model: mujoco.MjModel,
    mapping: ArmMapping,
    normal_torque_limits: np.ndarray,
) -> None:
    """Convert selected compiled position servos to direct-torque actuators."""
    limits = np.asarray(normal_torque_limits, dtype=float)
    if limits.shape != (7,) or not np.all(np.isfinite(limits)) or np.any(limits <= 0):
        raise ModelMappingError("Normal torque limits must be seven finite positives")

    for channel, normal_limit in zip(mapping.channels, limits, strict=True):
        actuator = model.actuator(channel.actuator_name)
        peak_limit = min(
            abs(float(actuator.forcerange[0])), float(actuator.forcerange[1])
        )
        if normal_limit > peak_limit:
            raise ModelMappingError(
                f"Normal torque {normal_limit} exceeds {channel.actuator_name} "
                f"model force limit {peak_limit}"
            )

        actuator.dyntype[0] = mujoco.mjtDyn.mjDYN_NONE
        actuator.dynprm[:] = 0.0
        actuator.gaintype[0] = mujoco.mjtGain.mjGAIN_FIXED
        actuator.gainprm[:] = 0.0
        actuator.gainprm[0] = 1.0
        actuator.biastype[0] = mujoco.mjtBias.mjBIAS_NONE
        actuator.biasprm[:] = 0.0
        actuator.ctrllimited[0] = 1
        actuator.ctrlrange[:] = (-normal_limit, normal_limit)

        if int(actuator.biastype[0]) != int(mujoco.mjtBias.mjBIAS_NONE):
            raise ModelMappingError(f"{channel.actuator_name} still has actuator bias")
        if not np.allclose(actuator.biasprm, 0.0):
            raise ModelMappingError(
                f"{channel.actuator_name} still has built-in position feedback"
            )
        if not np.isclose(float(actuator.gainprm[0]), 1.0):
            raise ModelMappingError(f"{channel.actuator_name} torque gain is not one")


def verify_torque_interface(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mapping: ArmMapping,
    normal_torque_limits: np.ndarray,
) -> None:
    """Prove command-to-force magnitude, target, and sign for every channel."""
    saved_ctrl = data.ctrl.copy()
    limits = np.asarray(normal_torque_limits, dtype=float)
    try:
        data.ctrl[mapping.actuator_ids] = 0.0
        for channel, normal_limit in zip(mapping.channels, limits, strict=True):
            probe = min(0.1, normal_limit * 0.1)
            data.ctrl[mapping.actuator_ids] = 0.0
            data.ctrl[channel.actuator_id] = probe
            mujoco.mj_forward(model, data)
            actuator_force = float(data.actuator_force[channel.actuator_id])
            joint_force = float(data.qfrc_actuator[channel.dof_address])
            if not np.isclose(actuator_force, probe, atol=1e-12, rtol=1e-12):
                raise ModelMappingError(
                    f"{channel.actuator_name} ctrl is not direct force: "
                    f"command={probe}, force={actuator_force}"
                )
            if not np.isclose(joint_force, probe, atol=1e-12, rtol=1e-12):
                raise ModelMappingError(
                    f"{channel.actuator_name} sign/target mismatch: "
                    f"command={probe}, joint force={joint_force}"
                )
    finally:
        data.ctrl[:] = saved_ctrl
        mujoco.mj_forward(model, data)

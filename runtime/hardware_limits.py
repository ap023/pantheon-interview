"""Hardware limits (DESIGN.md section 1: "Hardware — arm model, gripper,
declared sensor set (static-ish)").

Two different sources of truth here, deliberately not conflated:

- Position range and torque/force range ARE encoded in the MJCF
  (jnt_range, actuator_forcerange) — read them from the loaded model,
  never hardcode a second copy that could drift out of sync with it.
- Max joint VELOCITY is not encoded in either menagerie MJCF at all
  (checked both panda.xml and ur5e.xml directly — no velocity fields
  anywhere). It only exists as a manufacturer datasheet spec, so it has
  to be hardcoded here, keyed by robot name.

The velocity numbers below are transcribed from commonly-cited public
spec sheets, not the primary manufacturer datasheets — see TODO.md's
"Low priority" section.
"""
from typing import Dict

import numpy as np

# rad/s per joint, arm joints only (excludes the gripper actuator).
# Panda: franka_description joint_limits.yaml convention (joints 1-4 slower
# than 5-7). UR5e: e-Series datasheet, 180 deg/s uniform across all 6 joints.
MAX_JOINT_VELOCITY_RAD_S: Dict[str, np.ndarray] = {
    "panda": np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]),
    "ur5e": np.full(6, np.pi),  # 180 deg/s on every joint
}


def max_joint_velocity(robot_name: str) -> np.ndarray:
    try:
        return MAX_JOINT_VELOCITY_RAD_S[robot_name]
    except KeyError:
        raise KeyError(
            f"no velocity limits recorded for {robot_name!r}; "
            f"known robots: {list(MAX_JOINT_VELOCITY_RAD_S)}"
        )


def actuated_position_range(model) -> np.ndarray:
    """(nu, 2) min/max position per actuator, read from the model's own
    joint definitions — this is the model's actual position limit, not a
    duplicated constant."""
    joint_ids = model.actuator_trnid[:, 0]
    return model.jnt_range[joint_ids].copy()


def actuated_torque_range(model) -> np.ndarray:
    """(nu, 2) min/max torque/force per actuator, read directly from
    actuator_forcerange."""
    return model.actuator_forcerange.copy()

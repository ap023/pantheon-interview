"""Proportional joint-space controller.

Nudges each actuator's controlled joint toward a target position. Reads
the joint each actuator drives via the model's actuator transmission
(actuator_trnid -> jnt_qposadr) rather than assuming qpos and ctrl share
an index order — this is what lets the same controller run unmodified
against cells with different actuator counts (Panda nu=8 vs UR5e nu=6).
"""
import numpy as np


def actuated_qpos(model, data) -> np.ndarray:
    """Current position of the joint each actuator drives, in actuator order."""
    joint_ids = model.actuator_trnid[:, 0]
    qpos_addrs = model.jnt_qposadr[joint_ids]
    return data.qpos[qpos_addrs].copy()


def actuated_qvel(model, data) -> np.ndarray:
    """Current velocity of the joint each actuator drives, in actuator order.

    Addressed via jnt_dofadr, not jnt_qposadr — position and velocity use
    separate address spaces in MuJoCo (they only happen to coincide here
    because every joint on these arms is a single-dof hinge/slide; a ball
    or free joint would make qpos and qvel different sizes).
    """
    joint_ids = model.actuator_trnid[:, 0]
    dof_addrs = model.jnt_dofadr[joint_ids]
    return data.qvel[dof_addrs].copy()


class ProportionalController:
    def __init__(self, kp: float):
        self.kp = kp

    def step(self, model, data, target_qpos: np.ndarray) -> np.ndarray:
        """Apply one proportional control update toward target_qpos.

        target_qpos is per-actuator (length model.nu), same ordering as
        actuated_qpos(). Returns the position error before this step's
        control was applied, for the caller's success/failure check.
        """
        current = actuated_qpos(model, data)
        error = target_qpos - current
        ctrl = self.kp * error
        ctrl_range = model.actuator_ctrlrange
        data.ctrl[:] = np.clip(ctrl, ctrl_range[:, 0], ctrl_range[:, 1])
        return error

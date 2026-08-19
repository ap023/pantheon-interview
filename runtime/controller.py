"""Proportional joint-space controller.

Nudges each actuator's controlled joint toward a target position. Reads
the joint each actuator drives via the model's actuator transmission
(actuator_trnid -> jnt_qposadr) rather than assuming qpos and ctrl share
an index order — this is what lets the same controller run unmodified
against cells with different actuator counts (Panda nu=8 vs UR5e nu=6).

Not every actuator drives a joint directly: Panda's real gripper
actuator is tendon-driven (actuator_trntype is mjTRN_TENDON, not
mjTRN_JOINT), so actuator_trnid[:, 0] for it is a tendon id, not a
joint id — silently reading it as one aliases the actuator onto
whatever joint happens to share that id. controlled_actuator_ids()
and require_joint_transmission() exist to exclude/guard that case
rather than mis-address it.
"""
import mujoco
import numpy as np


def controlled_actuator_ids(model) -> np.ndarray:
    """Actuator ids whose transmission is a joint. Excludes tendon/site/
    body-transmission actuators (e.g. Panda's tendon-driven gripper) that
    this controller doesn't know how to address."""
    return np.where(model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)[0]


def require_joint_transmission(model, actuator_ids) -> None:
    """Fail loud if any of actuator_ids isn't joint-transmitted, instead of
    silently reading actuator_trnid[:, 0] as a joint id when it isn't one."""
    actuator_ids = np.asarray(actuator_ids)
    trntypes = model.actuator_trntype[actuator_ids]
    bad = actuator_ids[trntypes != mujoco.mjtTrn.mjTRN_JOINT]
    if bad.size:
        raise ValueError(
            f"actuator id(s) {bad.tolist()} do not use joint transmission "
            "(e.g. a tendon-driven gripper) — not addressable via "
            "jnt_qposadr/jnt_dofadr; exclude them with controlled_actuator_ids()"
        )


def actuated_qpos(model, data, actuator_ids=None) -> np.ndarray:
    """Current position of the joint each actuator drives, in actuator_ids
    order (default: every actuator, model.nu)."""
    if actuator_ids is None:
        actuator_ids = np.arange(model.nu)
    require_joint_transmission(model, actuator_ids)
    joint_ids = model.actuator_trnid[actuator_ids, 0]
    qpos_addrs = model.jnt_qposadr[joint_ids]
    return data.qpos[qpos_addrs].copy()


def actuated_qvel(model, data, actuator_ids=None) -> np.ndarray:
    """Current velocity of the joint each actuator drives, in actuator_ids
    order (default: every actuator, model.nu).

    Addressed via jnt_dofadr, not jnt_qposadr — position and velocity use
    separate address spaces in MuJoCo (they only happen to coincide here
    because every joint on these arms is a single-dof hinge/slide; a ball
    or free joint would make qpos and qvel different sizes).
    """
    if actuator_ids is None:
        actuator_ids = np.arange(model.nu)
    require_joint_transmission(model, actuator_ids)
    joint_ids = model.actuator_trnid[actuator_ids, 0]
    dof_addrs = model.jnt_dofadr[joint_ids]
    return data.qvel[dof_addrs].copy()


class ProportionalController:
    def __init__(self, kp: float):
        self.kp = kp

    def step(self, model, data, target_qpos: np.ndarray, actuator_ids=None) -> np.ndarray:
        """Apply one control update toward target_qpos.

        target_qpos is per actuator_ids (default: every actuator, model.nu),
        same ordering as actuated_qpos(). Actuators outside actuator_ids
        (e.g. a tendon-driven gripper) are left untouched. Returns the
        position error before this step's control was applied, for the
        caller's success/failure check.

        ctrl semantics differ by actuator type, and writing the wrong
        kind converges by accident at best (verified empirically —
        wrapping an outer kp*error loop around the menagerie arms'
        built-in position servos limit-cycles and never settles):

        - torque actuators (<motor>, biastype none): ctrl is a torque —
          close the position loop here, ctrl = kp * error.
        - position servos (<position>/<general> with affine bias, which
          is what both menagerie arms use, servo gains tuned in the
          XML): ctrl IS a target position — command target_qpos directly
          and let the model's own servo close the loop.
        """
        if actuator_ids is None:
            actuator_ids = np.arange(model.nu)
        current = actuated_qpos(model, data, actuator_ids)
        error = target_qpos - current
        servo = model.actuator_biastype[actuator_ids] == mujoco.mjtBias.mjBIAS_AFFINE
        ctrl = np.where(servo, target_qpos, self.kp * error)
        ctrl_range = model.actuator_ctrlrange[actuator_ids]
        data.ctrl[actuator_ids] = np.clip(ctrl, ctrl_range[:, 0], ctrl_range[:, 1])
        return error

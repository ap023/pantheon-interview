import numpy as np
import pytest

from runtime.controller import (
    ProportionalController,
    actuated_qpos,
    actuated_qvel,
    controlled_actuator_ids,
    require_joint_transmission,
)


def test_actuated_qpos_follows_actuator_order_not_declaration_order(two_joint_model, two_joint_data):
    # actuator 0 drives joint2, actuator 1 drives joint1 (see conftest) —
    # proves the lookup isn't just slicing qpos by index.
    two_joint_data.qpos[:] = [1.0, 2.0]  # [joint1, joint2]
    result = actuated_qpos(two_joint_model, two_joint_data)
    np.testing.assert_allclose(result, [2.0, 1.0])


def test_actuated_qvel_follows_actuator_order(two_joint_model, two_joint_data):
    two_joint_data.qvel[:] = [0.5, -0.5]  # [joint1, joint2]
    result = actuated_qvel(two_joint_model, two_joint_data)
    np.testing.assert_allclose(result, [-0.5, 0.5])


def test_controller_step_computes_error_and_ctrl_in_actuator_order(two_joint_model, two_joint_data):
    controller = ProportionalController(kp=3.0)
    two_joint_data.qpos[:] = [0.0, 0.0]

    error = controller.step(two_joint_model, two_joint_data, np.array([2.0, 1.0]))

    np.testing.assert_allclose(error, [2.0, 1.0])
    ctrl_range = two_joint_model.actuator_ctrlrange
    expected_ctrl = np.clip(3.0 * np.array([2.0, 1.0]), ctrl_range[:, 0], ctrl_range[:, 1])
    np.testing.assert_allclose(two_joint_data.ctrl, expected_ctrl)


def test_controller_step_clips_to_ctrlrange(two_joint_model, two_joint_data):
    controller = ProportionalController(kp=1000.0)  # forces clipping
    controller.step(two_joint_model, two_joint_data, np.array([3.0, 3.0]))
    ctrl_range = two_joint_model.actuator_ctrlrange
    assert np.all(two_joint_data.ctrl <= ctrl_range[:, 1] + 1e-9)
    assert np.all(two_joint_data.ctrl >= ctrl_range[:, 0] - 1e-9)


def test_controlled_actuator_ids_excludes_tendon_actuator(tendon_model):
    # act_joint2 (0), act_joint1 (1) are joint-transmitted; act_tendon1 (2)
    # is tendon-transmitted (stand-in for Panda's real gripper actuator).
    assert tendon_model.nu == 3
    np.testing.assert_array_equal(controlled_actuator_ids(tendon_model), [0, 1])


def test_require_joint_transmission_raises_for_tendon_actuator(tendon_model):
    with pytest.raises(ValueError, match="joint transmission"):
        require_joint_transmission(tendon_model, [0, 1, 2])


def test_actuated_qpos_raises_if_tendon_actuator_addressed(tendon_model, tendon_data):
    with pytest.raises(ValueError, match="joint transmission"):
        actuated_qpos(tendon_model, tendon_data)


def test_actuated_qpos_with_controlled_ids_skips_tendon_actuator(tendon_model, tendon_data):
    ids = controlled_actuator_ids(tendon_model)
    tendon_data.qpos[:] = [1.0, 2.0]  # [joint1, joint2]
    result = actuated_qpos(tendon_model, tendon_data, ids)
    np.testing.assert_allclose(result, [2.0, 1.0])  # act_joint2, act_joint1


def test_controller_step_leaves_tendon_actuator_ctrl_untouched(tendon_model, tendon_data):
    ids = controlled_actuator_ids(tendon_model)
    controller = ProportionalController(kp=3.0)
    tendon_data.ctrl[2] = 5.0  # gripper slot, pre-existing value

    controller.step(tendon_model, tendon_data, np.array([1.0, 1.0]), ids)

    assert tendon_data.ctrl[2] == 5.0

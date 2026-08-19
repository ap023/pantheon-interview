import numpy as np

from runtime.controller import ProportionalController, actuated_qpos, actuated_qvel


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

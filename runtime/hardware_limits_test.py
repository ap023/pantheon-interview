import numpy as np
import pytest

from runtime.hardware_limits import (
    actuated_position_range,
    actuated_torque_range,
    max_joint_velocity,
)


def test_max_joint_velocity_known_robots():
    assert max_joint_velocity("panda").shape == (7,)
    assert max_joint_velocity("ur5e").shape == (6,)
    np.testing.assert_allclose(max_joint_velocity("ur5e"), np.pi)


def test_max_joint_velocity_unknown_robot_raises():
    with pytest.raises(KeyError):
        max_joint_velocity("not_a_real_robot")


def test_actuated_position_range_follows_actuator_order(two_joint_model):
    # actuator 0 -> joint2 (range -2 2), actuator 1 -> joint1 (range -1 1)
    result = actuated_position_range(two_joint_model)
    assert result.shape == (2, 2)
    np.testing.assert_allclose(result[0], [-2, 2])
    np.testing.assert_allclose(result[1], [-1, 1])


def test_actuated_torque_range_matches_model(two_joint_model):
    result = actuated_torque_range(two_joint_model)
    np.testing.assert_allclose(result, two_joint_model.actuator_forcerange)

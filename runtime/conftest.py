"""Shared test fixtures.

A tiny synthetic 2-joint model stands in for Panda/UR5e in unit tests, so
tests don't depend on network access or robot_descriptions' cache. The
two joints are deliberately given different ranges, and the actuators
are declared in *reversed* order relative to the joints (actuator 0
drives joint2, actuator 1 drives joint1) — this is what makes tests
against actuated_qpos/actuated_qvel actually prove the address-lookup
logic, instead of passing by coincidence because everything lines up by
index.
"""
import mujoco
import pytest

TWO_JOINT_XML = """
<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="link1">
      <joint name="joint1" type="hinge" axis="0 0 1" range="-1 1" damping="2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/>
      <body name="link2" pos="0.3 0 0">
        <joint name="joint2" type="hinge" axis="0 0 1" range="-2 2" damping="2"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="act_joint2" joint="joint2" ctrlrange="-20 20" forcerange="-20 20"/>
    <motor name="act_joint1" joint="joint1" ctrlrange="-20 20" forcerange="-20 20"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def two_joint_model():
    return mujoco.MjModel.from_xml_string(TWO_JOINT_XML)


@pytest.fixture
def two_joint_data(two_joint_model):
    data = mujoco.MjData(two_joint_model)
    mujoco.mj_resetData(two_joint_model, data)
    return data


@pytest.fixture
def two_joint_mjcf_path(tmp_path):
    path = tmp_path / "two_joint.xml"
    path.write_text(TWO_JOINT_XML)
    return path

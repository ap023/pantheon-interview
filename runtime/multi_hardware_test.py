"""Phase 2's actual deliverable: the same Cell/controller code running
unmodified against two arms with different actuator counts (Panda
nu=8, UR5e nu=6). Every other test file uses a synthetic 2-joint model
specifically to avoid needing real hardware models — this one is the
exception on purpose, since "does it actually work on Panda and UR5e"
is the claim that needs checking directly, not inferred from the
synthetic fixture. Uses the real fleet_defaults.yaml too (no config
mocking) — this is the acceptance test for "the abstraction holds," not
a unit test of one piece of it.

Needs network access on first run (robot_descriptions clones/caches the
mujoco_menagerie repo); cached locally after that, same tradeoff
runtime/demo.py already has.
"""
import numpy as np
import pytest
from robot_descriptions import panda_mj_description, ur5e_mj_description

from runtime import records as records_module
from runtime.cell import Cell


@pytest.fixture(autouse=True)
def isolate_records_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(records_module, "RECORDS_DIR", tmp_path / "records")
    monkeypatch.setattr(records_module, "CONFIG_SNAPSHOTS_DIR", tmp_path / "records" / "config_snapshots")


def _nudge_target(cell: Cell) -> np.ndarray:
    """A small, physically reachable delta from wherever the arm
    currently is — same recipe as runtime/demo.py."""
    start = cell.current_qpos()
    ctrl_range = cell.model.actuator_ctrlrange
    return np.clip(start + 0.15, ctrl_range[:, 0], ctrl_range[:, 1])


@pytest.mark.parametrize(
    "mjcf_path,expected_nu",
    [
        (panda_mj_description.MJCF_PATH, 8),
        (ur5e_mj_description.MJCF_PATH, 6),
    ],
    ids=["panda", "ur5e"],
)
def test_run_cycle_succeeds_on_real_hardware_with_different_actuator_counts(mjcf_path, expected_nu):
    cell = Cell("cell_hardware_check", mjcf_path)
    # Sanity check that we're actually still exercising the
    # actuator-count gap the design is built around, not two arms that
    # happen to have converged to the same nu.
    assert cell.model.nu == expected_nu

    record = cell.run_cycle(_nudge_target(cell), part_id="part_1")

    assert record.outcome == "success"
    assert record.reason is None
    assert cell.done is True
    # Real config resolution happened (fleet_defaults.yaml), not a mock.
    assert record.config["control_gain_kp"]["source"] == "fleet_default"

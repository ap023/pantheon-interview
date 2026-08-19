"""Manual verification: spin up one Panda cell and run a cycle.

    python -m runtime.demo
"""
import numpy as np
from robot_descriptions import panda_mj_description

from runtime.cell import Cell


def main():
    cell = Cell("cell_panda_001", panda_mj_description.MJCF_PATH)

    start = cell.current_qpos()
    ctrl_range = cell.model.actuator_ctrlrange[cell.controlled_actuator_ids]
    target = np.clip(start + 0.5, ctrl_range[:, 0], ctrl_range[:, 1])

    print(f"start qpos:  {np.round(start, 3)}")
    print(f"target qpos: {np.round(target, 3)}")

    record = cell.run_cycle(target, part_id="part_demo_001", variant="default")

    end = cell.current_qpos()
    print(f"end qpos:    {np.round(end, 3)}")
    print(f"outcome: {record.outcome} (reason={record.reason})")
    print(f"sim steps: {record.sim_steps}, duration_s: {record.duration_s:.3f} / takt_s: {record.takt_s}")


if __name__ == "__main__":
    main()

"""Live status board over a running Line, driven by the file channels.

    python -m runtime.line_runner                     # run forever
    python -m runtime.line_runner --ticks 20          # bounded run
    python -m runtime.line_runner --interval 1.0      # faster ticks

Each tick, every cell runs its default task (alternating between its
home pose and a small fixed offset from it — passes under the
fleet-default config) unless an instruction is waiting in its inbox. Drive it from a second terminal while it runs:

Inbox — task_input/task_{cell_id}.json (DESIGN.md section 1b). Consumed
(deleted) the moment it's read; the instruction then STAYS active for
that cell until cleared or replaced. An adversarial instruction (e.g. an
out-of-range target_qpos -> logic_fault) therefore keeps failing tick
after tick, holding its part in place, until you clear it:

    echo '{"target_qpos": [9, 9, 9, 9, 9, 9, 9]}' > task_input/task_cell_panda_001.json

Commands — commands/{command}_{cell_id}.json, polled once per tick at
the boundary, consumed on read (section 1c):

    touch commands/clear_failure_cell_panda_001.json   # unhalt + drop active instruction -> back to default/inbox
    touch commands/obstruct_cell_panda_001.json        # inject an obstruction (halts the cell — needs clear_failure after)
    touch commands/kill_cell_panda_001.json            # mid-cycle kill (checked every physics step, not per tick; halts the cell — needs clear_failure after)
    touch commands/drop_clear_cell_panda_001.json      # part dropped OUTSIDE the workspace (one-shot fail, no halt — contrast with obstruct)

While one cell is stuck failing, watch the part flow do the right thing:
its part never advances (peek-not-pop), its downstream neighbor finishes
whatever part it already had and then reads STARVED, and once the stuck
cell's upstream buffer fills the Line stops releasing new parts at all.
"""
import argparse
from typing import Dict, Optional

import numpy as np
from robot_descriptions import panda_mj_description, ur5e_mj_description

from runtime import commands as commands_module
from runtime import task_input as task_input_module
from runtime.cell import Cell
from runtime.line import AutomaticTickTrigger, Line, Part, TickResult

TAKT_S = 4.0

STATUS_LABEL = {"success": "PASS", "failure": "FAIL", "refusal": "REFUSED"}

# ANSI color for terminal readability — green success, red hard failure,
# yellow refusal (a decision, not a crash), dim gray for idle
# (starved/blocked, nobody's fault). No effect on non-tty output (piped
# to a file, `less`, etc.) beyond the raw escape codes being present;
# harmless either way for this demo script.
_COLOR = {"success": "\033[32m", "failure": "\033[31m", "refusal": "\033[33m", "idle": "\033[2m"}
_RESET = "\033[0m"


def _status_line(result: TickResult, instructed: bool) -> str:
    part = result.part_id or "-"
    if not result.ran:
        state = "STARVED" if result.starved else "BLOCKED"
        return f"  {result.cell_id:<16} {'idle':<12} {_COLOR['idle']}{state}{_RESET}"
    label = STATUS_LABEL[result.outcome]
    reason = f" ({result.reason})" if result.reason else ""
    takt = f" [{result.duration_s:.3f}s/{result.takt_s:.1f}s takt, step {result.sim_steps}]"
    src = " <- inbox instruction" if instructed else ""
    color = _COLOR[result.outcome]
    return f"  {result.cell_id:<16} {part:<12} {color}{label}{reason}{_RESET}{takt}{src}"


class LineRunner:
    """Wraps a Line with the two file channels: the task_input/ inbox
    (per-cell instruction override, consumed on read, held in memory
    until cleared or replaced) and the once-per-tick commands/ poll
    (clear_failure, obstruct)."""

    def __init__(self, cells, trigger):
        # cell_id -> active instruction target. Set from the inbox,
        # dropped by clear_failure. The file itself is deleted the
        # moment it's read — this dict is what keeps the instruction
        # live across ticks afterward.
        self.active_targets: Dict[str, np.ndarray] = {}
        # Default task: alternate between two ABSOLUTE poses — the
        # cell's home pose and home+0.15 — flipping on each success.
        # Absolute, not "current + 0.15": a relative nudge compounds
        # every cycle, walking the arm further out until gravity's
        # steady-state error beats the fleet-default gain and every
        # cycle goes over-takt (observed empirically). Two fixed poses
        # near home can't drift.
        self._home: Dict[str, np.ndarray] = {}
        self._outbound: Dict[str, bool] = {}
        for cell in cells:
            ctrl_range = cell.model.actuator_ctrlrange[cell.controlled_actuator_ids]
            self._home[cell.cell_id] = np.clip(cell.current_qpos(), ctrl_range[:, 0], ctrl_range[:, 1])
            self._outbound[cell.cell_id] = True
        self.line = Line(cells=cells, trigger=trigger, target_qpos_fn=self._target_for)

    def _target_for(self, cell: Cell, part: Part) -> np.ndarray:
        override = self.active_targets.get(cell.cell_id)
        if override is not None:
            return override
        home = self._home[cell.cell_id]
        if not self._outbound[cell.cell_id]:
            return home
        ctrl_range = cell.model.actuator_ctrlrange[cell.controlled_actuator_ids]
        return np.clip(home + 0.15, ctrl_range[:, 0], ctrl_range[:, 1])

    def poll_channels(self) -> None:
        """Once per tick, before the tick runs: apply any waiting
        commands, then pull any waiting inbox instructions."""
        for cell in self.line.cells:
            if commands_module.check_and_consume_clear_failure(cell.cell_id):
                cell.clear_failure()
                dropped = self.active_targets.pop(cell.cell_id, None)
                extra = " and active instruction dropped -> back to default/inbox" if dropped is not None else ""
                print(f"  >>> command: clear_failure {cell.cell_id} — halt cleared{extra}")
            if commands_module.check_and_consume_obstruct(cell.cell_id):
                cell.obstruct(reason="obstruction_injected_via_commands")
                print(f"  >>> command: obstruct {cell.cell_id}")

            instruction = task_input_module.check_and_consume_task(cell.cell_id)
            if instruction is None:
                continue
            if "error" in instruction:
                print(f"  >>> inbox: {cell.cell_id}: {instruction['error']} (ignored)")
                continue
            target = instruction.get("target_qpos")
            n = len(cell.controlled_actuator_ids)
            if not isinstance(target, list) or len(target) != n:
                print(
                    f"  >>> inbox: {cell.cell_id}: instruction needs a "
                    f"target_qpos list of length {n}, got {target!r} (ignored)"
                )
                continue
            self.active_targets[cell.cell_id] = np.asarray(target, dtype=float)
            print(f"  >>> inbox: {cell.cell_id} instruction consumed (file deleted), target={target}")

    def run(self, n_ticks: Optional[int]) -> None:
        tick = 0
        while n_ticks is None or tick < n_ticks:
            tick += 1
            self.poll_channels()
            results = self.line.tick()

            for result in results:
                if result.outcome == "success" and result.cell_id not in self.active_targets:
                    # Default task leg completed — head for the other pose
                    # next cycle (out -> home -> out -> ...).
                    self._outbound[result.cell_id] = not self._outbound[result.cell_id]

            print(f"\n=== tick {self.line.tick_count} ===")
            for result in results:
                print(_status_line(result, result.cell_id in self.active_targets))

            occupancy = " | ".join(
                f"{self.line._edge_id(i)}: [{', '.join(p.id for p in buf.contents) or 'empty'}]"
                for i, buf in enumerate(self.line.buffers)
            )
            print(f"  buffers: {occupancy}")
            done = self.line.completed
            done_ids = ", ".join(p.id for p in done[-3:])
            print(f"  completed: {len(done)}" + (f" (latest: {done_ids})" if done else ""))

            stuck = [r for r in results if r.outcome in ("failure", "refusal")]
            if stuck:
                reasons = ", ".join(f"{r.cell_id}={r.reason}" for r in stuck)
                print(f"  >>> holding: {reasons} — part stays put, retried next tick")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=None, help="number of ticks (default: run forever)")
    parser.add_argument("--interval", type=float, default=TAKT_S, help="seconds between ticks")
    args = parser.parse_args()

    cells = [
        # robot_name omitted: the tendon-addressing crash is fixed, but
        # the fleet-default kp isn't tuned for real dynamics and would
        # trip the velocity safety check immediately — see TODO.md.
        # site_id="site_a" + config/per_unit/cell_panda_001.yaml
        # demonstrate real per-cell config layering (DESIGN.md section
        # 2) — this Panda alone gets a slower takt and looser tolerance;
        # UR5e gets pure fleet defaults.
        Cell("cell_panda_001", panda_mj_description.MJCF_PATH, site_id="site_a"),
        Cell("cell_ur5e_001", ur5e_mj_description.MJCF_PATH),
    ]
    runner = LineRunner(cells, AutomaticTickTrigger(interval_s=args.interval))
    line = runner.line
    print(f"Line started: {[c.cell_id for c in line.cells]}, tick interval={args.interval}s")
    print("inbox:    task_input/task_{cell_id}.json   (consumed on read, stays active until cleared/replaced)")
    print("commands: commands/{clear_failure|obstruct|kill|drop_clear}_{cell_id}.json  (consumed on read)")
    print(f"records:  records/runs/{line.run_id}/   (every cycle this run, in tick order)")
    print(f"          records/{{cell_id}}/{{cycle_id}}.json is the canonical copy; the above is an index into it")
    try:
        runner.run(args.ticks)
    except KeyboardInterrupt:
        pass
    print("\nShift complete.")


if __name__ == "__main__":
    main()

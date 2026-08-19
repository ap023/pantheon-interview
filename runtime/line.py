"""Line: the takt clock, topology, buffer ownership, variant registry, and
part release (DESIGN.md section 1, 1a, 1c).

This is the "script that emulates takt" — each tick it checks every
cell's upstream buffer, calls run_cycle if the cell isn't starved or
blocked, and reads `done` at the tick boundary as the source of truth for
over-takt (DESIGN.md section 3, TODO.md). No Cell holds a reference to
another Cell; they only ever touch shared state (buffers, the current
variant) through the Line.
"""
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

import numpy as np

from runtime import line_config as line_config_module
from runtime import variants as variants_module
from runtime.buffer import Buffer
from runtime.cell import Cell


class LineTopologyError(Exception):
    """Raised by the preflight validator (DESIGN.md section 1a) when the
    line as configured can never run. Runs once at construction, before
    any part is released — a bad topology fails at startup, not partway
    through a shift."""


@dataclass(frozen=True)
class Part:
    """DESIGN.md section 1: released by the Line at each takt tick,
    carries an id and a target variant, flows through buffers along the
    topology. No physically simulated box (TODO.md) — this is the whole
    of it; success stays pure joint-space convergence."""

    id: str
    variant: str


@dataclass
class TickResult:
    """One cell's outcome for a single tick — what the Line decided
    (starved/blocked/ran) and what run_cycle reported back, if it ran."""

    cell_id: str
    ran: bool
    starved: bool
    blocked: bool
    over_takt: bool
    outcome: Optional[str]  # "success" | "failure" | "refusal" | None if not attempted
    reason: Optional[str]


class TickTrigger:
    """Pluggable trigger source for the takt clock (DESIGN.md section
    1c). The clock's job — decide a tick happened, then check every
    cell's done flag at that boundary — is identical either way; only
    wait() differs between manual and automatic. One clock interface,
    not two Line implementations."""

    def wait(self) -> None:
        raise NotImplementedError


class ManualTickTrigger(TickTrigger):
    """Section 1c's natural default to build first: deterministic, and
    lets whoever's driving a demo/test control exactly when the next
    tick lands relative to a task-input override they just wrote."""

    def __init__(
        self,
        prompt: str = "[Line] press Enter to advance tick...",
        input_fn: Callable[[str], str] = input,
    ):
        self._prompt = prompt
        self._input_fn = input_fn

    def wait(self) -> None:
        self._input_fn(self._prompt)


class AutomaticTickTrigger(TickTrigger):
    """Stands in for a background timer: fires every interval_s seconds.
    interval_s is passed explicitly rather than pulled from config here
    — the Line doesn't assume there's exactly one takt shared by every
    cell."""

    def __init__(self, interval_s: float, sleep_fn: Callable[[float], None] = time.sleep):
        if interval_s <= 0:
            raise ValueError(f"interval_s must be > 0, got {interval_s}")
        self.interval_s = interval_s
        self._sleep_fn = sleep_fn

    def wait(self) -> None:
        self._sleep_fn(self.interval_s)


def _default_target_qpos(cell: Cell, part: Part) -> np.ndarray:
    """Stand-in Task Instruction deriver (DESIGN.md section 1: 'derived
    automatically from Part + active Variant'). Multi-waypoint cycles and
    real per-variant task specs aren't built yet (TODO.md) — this mirrors
    demo.py's existing pattern (nudge every actuator by a fixed offset,
    clipped to range) so the Line is runnable today without inventing new
    physics. Swap it via Line(..., target_qpos_fn=...).
    """
    start = cell.current_qpos()
    ctrl_range = cell.model.actuator_ctrlrange
    return np.clip(start + 0.5, ctrl_range[:, 0], ctrl_range[:, 1])


class Line:
    """Owns anything shared across stations (DESIGN.md section 1: "the
    clock, the topology, the buffers, the current variant, and part
    release"). Topology is a linear chain (section 6: graph topology
    deliberately deferred):

        release -> buffers[0] -> cells[0] -> buffers[1] -> cells[1]
                -> ... -> buffers[N] (sink)

    buffers[0] is where the Line releases new parts; buffers[-1] is the
    sink the preflight validator checks for. A Cell is only ever handed
    its own upstream/downstream buffer's occupancy through tick() — it
    never sees another Cell or its place in the chain.
    """

    def __init__(
        self,
        cells: Sequence[Cell],
        trigger: Optional[TickTrigger] = None,
        target_qpos_fn: Callable[[Cell, Part], np.ndarray] = _default_target_qpos,
        initial_variant: str = "default",
    ):
        if not cells:
            raise LineTopologyError("a line needs at least one cell")

        self.cells: List[Cell] = list(cells)
        self.trigger = trigger or ManualTickTrigger()
        self._target_qpos_fn = target_qpos_fn

        variants_module.requirements(initial_variant)  # raises KeyError if unknown
        self.current_variant = initial_variant

        self.buffers: List[Buffer] = self._build_buffers()
        self.sink = self._validate_topology()

        self.tick_count = 0
        self._next_part_seq = 0

    def _edge_id(self, index: int) -> str:
        """index 0 is the source edge (release point -> cells[0]); index
        len(cells) is the sink edge (cells[-1] -> sink). Used as the
        line_config.resolve() key — DESIGN.md section 1a: buffer size is
        a property of an edge, a second config surface keyed by edge id,
        not by cell_id."""
        upstream = "source" if index == 0 else self.cells[index - 1].cell_id
        downstream = "sink" if index == len(self.cells) else self.cells[index].cell_id
        return f"{upstream}->{downstream}"

    def _build_buffers(self) -> List[Buffer]:
        buffers = []
        for i in range(len(self.cells) + 1):
            edge_id = self._edge_id(i)
            size = line_config_module.resolve(edge_id)["buffer_size"]["value"]
            try:
                buffers.append(Buffer(size=size))
            except ValueError as exc:
                # Preflight check "any buffer has size <= 0" (section 1a)
                # — Buffer already refuses to construct at all, so this
                # just re-raises it as the Line's own topology error
                # instead of a bare ValueError from a module the caller
                # of Line shouldn't need to know about.
                raise LineTopologyError(f"edge {edge_id!r}: {exc}") from exc
        return buffers

    def _validate_topology(self) -> Buffer:
        """DESIGN.md section 1a preflight validator: a one-time hard stop
        before the shift starts, not a partway-through discovery. Buffer
        size <= 0 is already caught in _build_buffers; what's left here
        is the chain-link and sink checks."""
        if len(self.buffers) != len(self.cells) + 1:
            raise LineTopologyError(
                "buffer count must be exactly one more than cell count (source + one per gap + sink)"
            )
        for i, cell in enumerate(self.cells):
            if self.buffers[i] is None or self.buffers[i + 1] is None:
                raise LineTopologyError(
                    f"cell {cell.cell_id!r} is missing an upstream or downstream buffer reference"
                )
        sink = self.buffers[-1]
        if sink is None:
            raise LineTopologyError("chain does not terminate at a defined sink")
        return sink

    def set_variant(self, variant: str) -> None:
        """Changeover event (DESIGN.md section 1: 'switched by Line, at
        changeover events'). Only affects parts released after this call
        — parts already sitting in a buffer keep the variant they were
        released with, since each Part carries its own target variant."""
        variants_module.requirements(variant)  # raises KeyError if unknown
        self.current_variant = variant

    def _release_part(self) -> Optional[Part]:
        part = Part(id=f"part_{self._next_part_seq:06d}", variant=self.current_variant)
        if not self.buffers[0].push(part):
            return None  # source blocked: can't release faster than cells[0] consumes
        self._next_part_seq += 1
        return part

    def tick(self) -> List[TickResult]:
        """Advance the takt clock by one tick.

        1. Blocks on self.trigger.wait() — the only thing manual vs.
           automatic mode changes (DESIGN.md section 1c).
        2. Releases one new part into the source buffer.
        3. Snapshots starved/blocked for every cell off buffer state
           taken at this one instant, before any cell runs — decisions
           for this tick are made off one consistent snapshot, not off
           buffers a neighboring cell mutates mid-pass (section 1a: "all
           cells run on the same shared takt clock, so cycles start in
           lockstep").
        4. Runs one cycle for every cell that's neither starved nor
           blocked, reads its `done` flag at this tick boundary as the
           source of truth for over-takt (section 3 / TODO.md), and
           advances the part into the downstream buffer on success.
        """
        self.trigger.wait()
        self.tick_count += 1
        self._release_part()

        starved_snapshot = [self.buffers[i].starved for i in range(len(self.cells))]
        blocked_snapshot = [self.buffers[i + 1].blocked for i in range(len(self.cells))]

        results: List[TickResult] = []
        for i, cell in enumerate(self.cells):
            starved = starved_snapshot[i]
            blocked = blocked_snapshot[i]
            if starved or blocked:
                # Not attempted at all, not even a refusal — a refusal is
                # a cell's own decision, starvation/blockage here is its
                # neighbor's fault (section 1a readiness rule).
                results.append(
                    TickResult(
                        cell_id=cell.cell_id,
                        ran=False,
                        starved=starved,
                        blocked=blocked,
                        over_takt=False,
                        outcome=None,
                        reason=None,
                    )
                )
                continue

            # peek, not pop: a part is only actually removed from
            # upstream once its cycle completes. A failed or refused
            # attempt leaves it exactly where it was, so the same part
            # is retried next tick instead of being silently scrapped —
            # this is also what makes a stuck cell (obstructed, stale
            # calibration, missing sensor — anything that keeps refusing)
            # naturally stall everything behind it: nothing ever pops,
            # so downstream starves and, once the upstream buffer fills,
            # the Line stops releasing new parts too (DESIGN.md section
            # 1a's backpressure, extended one step further).
            part = self.buffers[i].peek()
            target_qpos = self._target_qpos_fn(cell, part)
            record = cell.run_cycle(target_qpos, part_id=part.id, variant=part.variant)

            if cell.done:
                self.buffers[i].pop()
                self.buffers[i + 1].push(part)

            # done is set by run_cycle in lockstep with reason ==
            # "over_takt" (self.done = reached, right before the same
            # branch picks that reason) — reading it here is the tick-
            # boundary read TODO.md asks for, not a second opinion; the
            # internal decision inside run_cycle still exists too and is
            # flagged there as the redundant half still to collapse.
            over_takt = not cell.done and record.reason == "over_takt"

            results.append(
                TickResult(
                    cell_id=cell.cell_id,
                    ran=True,
                    starved=False,
                    blocked=False,
                    over_takt=over_takt,
                    outcome=record.outcome,
                    reason=record.reason,
                )
            )

        return results

    def run_shift(self, n_ticks: Optional[int] = None) -> List[List[TickResult]]:
        """Run tick() repeatedly — n_ticks times, or until interrupted
        (Ctrl-C) if n_ticks is None. In manual mode each tick blocks on
        Enter, so this loop is how an operator actually steps the whole
        line; in automatic mode each tick blocks on the
        AutomaticTickTrigger's sleep instead."""
        history: List[List[TickResult]] = []
        if n_ticks is None:
            try:
                while True:
                    history.append(self.tick())
            except KeyboardInterrupt:
                pass
        else:
            for _ in range(n_ticks):
                history.append(self.tick())
        return history

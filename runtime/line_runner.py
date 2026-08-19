"""Live status board over a running Line — a demo/manual-test entrypoint,
not part of DESIGN.md's scoped runtime, built to actually watch section
1c's manual-vs-automatic trigger and section 5's failure classification
play out tick by tick instead of just reading about them.

    python -m runtime.line_runner
    python -m runtime.line_runner --ticks 20 --interval 0.5

Ticks a two-cell (Panda + UR5e) line automatically and reprints a
one-line-per-cell status board every tick. A refused cell now holds onto
its part instead of scrapping it (Line.tick() peeks, only popping on
success) — so its refusal reason prints again every tick until something
clears it. That repeating "waiting on X" line *is* the flash this script
exists to produce, not a description of it.

There's no real commands/ terminal to drive a fault from yet (that
channel isn't wired up — see TODO.md), so this script injects an
obstruction on the first cell at a fixed tick and clears it a few ticks
later, purely to make the wait-for-clear flow visible on a bounded run.
Swap OBSTRUCT_AT_TICK / CLEAR_AT_TICK for real commands/ polling once
that exists.
"""
import argparse
import asyncio

from robot_descriptions import panda_mj_description, ur5e_mj_description

from runtime.cell import Cell
from runtime.line import AutomaticTickTrigger, Line, TickResult

TAKT_S = 4.0
OBSTRUCT_AT_TICK = 3
CLEAR_AT_TICK = 7

STATUS_LABEL = {"success": "PASS", "failure": "FAIL", "refusal": "REFUSED"}


def _status_line(result: TickResult) -> str:
    if not result.ran:
        state = "STARVED" if result.starved else "BLOCKED"
        return f"  {result.cell_id:<16} {state}"
    label = STATUS_LABEL[result.outcome]
    reason = f" ({result.reason})" if result.reason else ""
    return f"  {result.cell_id:<16} {label}{reason}"


def _build_line(interval_s: float) -> Line:
    cells = [
        # robot_name intentionally omitted for the Panda cell here, same
        # as demo.py — hardware_limits.MAX_JOINT_VELOCITY_RAD_S["panda"]
        # only has 7 entries (arm joints) but this model's nu is 8 (it
        # bundles a gripper actuator), so passing robot_name="panda"
        # crashes the mid-cycle velocity check on a shape mismatch. Newly
        # flagged in TODO.md as a known gap rather than fixed here — it's
        # a pre-existing hardware_limits/controller issue, not a Line one.
        Cell("cell_panda_001", panda_mj_description.MJCF_PATH),
        Cell("cell_ur5e_001", ur5e_mj_description.MJCF_PATH, robot_name="ur5e"),
    ]
    return Line(cells=cells, trigger=AutomaticTickTrigger(interval_s=interval_s))


async def run(n_ticks: int, interval_s: float) -> None:
    line = _build_line(interval_s)

    print(f"Line started: {[c.cell_id for c in line.cells]}, takt={interval_s}s")
    print(
        f"(cell_panda_001 will be obstructed at tick {OBSTRUCT_AT_TICK} and "
        f"manually cleared at tick {CLEAR_AT_TICK}, to demonstrate the "
        "wait-for-clear flow)"
    )

    for _ in range(n_ticks):
        if line.tick_count + 1 == OBSTRUCT_AT_TICK:
            line.cells[0].obstruct(reason="part_dropped_in_workspace")
        if line.tick_count + 1 == CLEAR_AT_TICK:
            line.cells[0].clear_failure()

        results = line.tick()

        print(f"\n=== tick {line.tick_count} ===")
        for result in results:
            print(_status_line(result))

        refused = [r for r in results if r.outcome == "refusal"]
        if refused:
            reasons = ", ".join(f"{r.cell_id}={r.reason}" for r in refused)
            print(f">>> waiting on: {reasons} — held part will retry next tick")

        # Nothing else needs to run concurrently in this simple demo —
        # tick() is a fast, synchronous call — but yielding here is what
        # makes this loop an actual coroutine rather than asyncio.run()
        # wrapping a body that never touches the event loop at all.
        await asyncio.sleep(0)

    print("\nShift complete.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", type=int, default=12, help="number of ticks to run")
    parser.add_argument("--interval", type=float, default=TAKT_S, help="seconds between ticks")
    args = parser.parse_args()

    try:
        asyncio.run(run(args.ticks, args.interval))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

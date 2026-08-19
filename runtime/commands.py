"""commands/ file-polling.

DESIGN.md sections 1b/1c describe commands/ as a directory a separate
process drops files into: `commands/{command}_{cell_id}.json`, polled
and consumed (deleted) once applied. Two polling cadences on the same
directory, deliberately:

- `kill` is checked every physics step inside Cell.run_cycle's loop —
  it's the one command that has to interrupt a cycle already in
  progress, not wait for the next tick boundary.
- Everything else (`clear_failure`, `obstruct`) is polled once per tick
  at the tick boundary by whoever drives the Line (line_runner) — a
  clear typed mid-cycle takes effect at the next tick, which is the
  section 1c "operator types clear_failure {cell_id} on the command
  terminal" flow.
"""
from pathlib import Path

COMMANDS_DIR = Path(__file__).resolve().parent.parent / "commands"


def _check_and_consume(command: str, cell_id: str) -> bool:
    """True if commands/{command}_{cell_id}.json exists, and delete it —
    consumed immediately on detection (DESIGN.md section 1b)."""
    path = COMMANDS_DIR / f"{command}_{cell_id}.json"
    if path.exists():
        path.unlink()
        return True
    return False


def check_and_consume_kill(cell_id: str) -> bool:
    return _check_and_consume("kill", cell_id)


def check_and_consume_clear_failure(cell_id: str) -> bool:
    return _check_and_consume("clear_failure", cell_id)


def check_and_consume_obstruct(cell_id: str) -> bool:
    return _check_and_consume("obstruct", cell_id)


def check_and_consume_drop_clear(cell_id: str) -> bool:
    """DESIGN.md section 5, "Failure — task outcome (clear)": a part
    dropped/misplaced but landing OUTSIDE the cell's workspace/buffer
    bounds — logged, that one cycle fails, no halt (contrast with
    obstruct(), which is the same physical event landing INSIDE the
    workspace and halting the cell). One-shot, same as kill: it voids
    the next cycle attempt only, not a standing state."""
    return _check_and_consume("drop_clear", cell_id)

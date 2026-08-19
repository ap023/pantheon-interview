"""task_input/ polling — the per-cell instruction inbox.

DESIGN.md section 1b: `task_input/task_{cell_id}.json` is an explicit
per-cycle instruction override, written by a separate script/terminal
for manual or automated adversarial testing, polled at cycle start and
consumed (deleted) once read — the same file-polling pattern as
commands/ and config/, no new IPC mechanism.

An instruction file is a YAML/JSON mapping (YAML is a superset of JSON,
so one parser covers both) with a `target_qpos` list, sized to the
cell's controlled actuators:

    {"target_qpos": [0.3, 0.1, 0.0, -0.5, 0.2, 0.1, 0.4]}

Adversarial values (out-of-range positions -> logic_fault, etc.) are the
point — the file is how you inject a bad instruction from a second
terminal while the line is running.
"""
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

TASK_INPUT_DIR = Path(__file__).resolve().parent.parent / "task_input"


def check_and_consume_task(cell_id: str) -> Optional[Dict[str, Any]]:
    """Return the parsed instruction for cell_id and delete the file, or
    None if no instruction is waiting. Looks for task_{cell_id}.json
    first (the DESIGN.md name), then task_{cell_id}.yaml.

    A file that fails to parse, or parses to something other than a
    mapping, is still consumed (deleted) — a malformed instruction
    shouldn't wedge the inbox by being re-read forever — and reported as
    an instruction with an "error" key rather than raised, so the caller
    can surface it without crashing the tick loop.
    """
    for suffix in (".json", ".yaml"):
        path = TASK_INPUT_DIR / f"task_{cell_id}{suffix}"
        if not path.exists():
            continue
        try:
            raw = path.read_text()
        finally:
            path.unlink()
        try:
            parsed = yaml.safe_load(raw)
        except yaml.YAMLError as exc:
            return {"error": f"unparseable instruction file {path.name}: {exc}"}
        if not isinstance(parsed, dict):
            return {"error": f"instruction file {path.name} must be a mapping, got {type(parsed).__name__}"}
        return parsed
    return None

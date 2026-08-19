"""Cycle record persistence (DESIGN.md section 4).

Each attempt gets a JSON record, written once under
records/{cell_id}/{cycle_id}.json — that's the single canonical copy.
The resolved config at run time is stored as a hash pointing to a
separately-stored snapshot, so identical configs across many cycles
aren't duplicated on disk.

records/runs/{run_id}/ is a browsable INDEX on top of that, not a second
copy: one symlink per cycle, named so `ls` sorts it into tick order and
shows the outcome at a glance, so a whole run's history can be read
chronologically without hunting through per-cell directories keyed by
random cycle_id.
"""
import hashlib
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

RECORDS_DIR = Path(__file__).resolve().parent.parent / "records"
CONFIG_SNAPSHOTS_DIR = RECORDS_DIR / "config_snapshots"
RUNS_DIR = RECORDS_DIR / "runs"

# TODO: derive from git commit hash / tagged release once this is a git repo.
ORCHESTRATOR_VERSION = "phase2-dev"
CONTROLLER_VERSION = "proportional-v1"


def hash_config(config: Dict[str, Any]) -> str:
    canonical = json.dumps(config, sort_keys=True)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def _write_config_snapshot(config: Dict[str, Any]) -> str:
    config_hash = hash_config(config)
    CONFIG_SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)
    snapshot_path = CONFIG_SNAPSHOTS_DIR / f"{config_hash}.json"
    if not snapshot_path.exists():
        snapshot_path.write_text(json.dumps(config, indent=2, sort_keys=True))
    return config_hash


@dataclass
class CycleRecord:
    cell_id: str
    cycle_id: str
    outcome: str  # "success" | "failure" | "refusal"
    duration_s: float
    takt_s: float
    config: Dict[str, Any]
    part_id: Optional[str] = None
    variant: Optional[str] = None
    task_spec_version: str = "phase2-stub"
    reason: Optional[str] = None
    calibration_value: float = 0.0
    calibration_age_s: float = 0.0
    sim_steps: int = 0
    timestamp: float = field(default_factory=time.time)
    orchestrator_version: str = ORCHESTRATOR_VERSION
    controller_version: str = CONTROLLER_VERSION

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["config_hash"] = _write_config_snapshot(self.config)
        del d["config"]
        return d


def write_cycle_record(record: CycleRecord) -> Path:
    cell_dir = RECORDS_DIR / record.cell_id
    cell_dir.mkdir(parents=True, exist_ok=True)
    out_path = cell_dir / f"{record.cycle_id}.json"
    out_path.write_text(json.dumps(record.to_dict(), indent=2, sort_keys=True))
    return out_path


def record_path(cell_id: str, cycle_id: str) -> Path:
    return RECORDS_DIR / cell_id / f"{cycle_id}.json"


def link_into_run(run_id: str, tick_number: int, cell_id: str, cycle_id: str, outcome: str) -> Path:
    """Symlink one cycle's canonical record into records/runs/{run_id}/.
    Filename order (tick, then cell) is what makes `ls records/runs/{run_id}/`
    readable as one run's timeline; the outcome suffix lets you spot
    failures with a glance or a `grep`, without opening every file."""
    run_dir = RUNS_DIR / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    link_path = run_dir / f"tick_{tick_number:04d}_{cell_id}_{outcome}.json"
    target = record_path(cell_id, cycle_id)
    if link_path.exists() or link_path.is_symlink():
        link_path.unlink()
    link_path.symlink_to(os.path.relpath(target, run_dir))
    return link_path


def read_config_snapshot(config_hash: str) -> Dict[str, Any]:
    """The reverse of hash_config(): look up the actual resolved config
    (value + source per field) a cycle record's config_hash points to."""
    snapshot_path = CONFIG_SNAPSHOTS_DIR / f"{config_hash}.json"
    if not snapshot_path.exists():
        raise FileNotFoundError(f"no config snapshot for hash {config_hash!r} at {snapshot_path}")
    return json.loads(snapshot_path.read_text())


def _main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="Look up a config snapshot by hash, or by a cycle record's config_hash field."
    )
    parser.add_argument(
        "target",
        help="a config_hash (16 hex chars), or the path to a records/{cell_id}/{cycle_id}.json cycle record",
    )
    args = parser.parse_args()

    target_path = Path(args.target)
    if target_path.exists():
        record = json.loads(target_path.read_text())
        print(f"cycle {record['cycle_id']} ({record['cell_id']}, outcome={record['outcome']}, reason={record.get('reason')}):")
        config_hash = record["config_hash"]
    else:
        config_hash = args.target

    config = read_config_snapshot(config_hash)
    print(f"config_hash {config_hash}:")
    for field_name, entry in sorted(config.items()):
        print(f"  {field_name:<24} = {entry['value']!r:<10} ({entry['source']})")


if __name__ == "__main__":
    _main()

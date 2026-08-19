"""Cycle record persistence (DESIGN.md section 4).

Each attempt gets a JSON record. The resolved config at run time is
stored as a hash pointing to a separately-stored snapshot, so identical
configs across many cycles aren't duplicated on disk.
"""
import hashlib
import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

RECORDS_DIR = Path(__file__).resolve().parent.parent / "records"
CONFIG_SNAPSHOTS_DIR = RECORDS_DIR / "config_snapshots"

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

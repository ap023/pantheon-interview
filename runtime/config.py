"""Config resolver — Phase 2 stub.

Real resolver merges fleet defaults -> site overrides -> per-unit
corrections, field by field, with provenance (DESIGN.md section 2). This
stub only loads fleet_defaults.yaml; site/per-unit merging is a TODO.
"""
from pathlib import Path
from typing import Any, Dict

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
FLEET_DEFAULTS_PATH = CONFIG_DIR / "fleet_defaults.yaml"


def _load_fleet_defaults() -> Dict[str, Any]:
    with open(FLEET_DEFAULTS_PATH) as f:
        return yaml.safe_load(f) or {}


def resolve(cell_id: str) -> Dict[str, Dict[str, Any]]:
    """Return cell_id's resolved config, shaped like the real resolver's
    output: {field: {"value": ..., "source": ...}}.

    TODO: merge config/site_overrides/{site}.yaml and
    config/per_unit/{cell_id}.yaml on top of fleet defaults (DESIGN.md
    section 2) instead of returning fleet defaults unconditionally.
    """
    defaults = _load_fleet_defaults()
    return {
        field: {"value": value, "source": "fleet_default"}
        for field, value in defaults.items()
    }

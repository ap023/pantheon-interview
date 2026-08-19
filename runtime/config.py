"""Config resolver — per-cell settings (DESIGN.md section 2).

Three layers, applied field by field, not whole-file replacement:

    fleet defaults  <  site overrides  <  per-unit corrections

A site or per-unit file only needs to contain the field(s) it's
overriding — the resolver starts from fleet defaults and merges each
later layer's fields on top, one field at a time, tagging each field's
source by whichever file last set it (not by whichever file happened to
produce a different number — see the tie-value rule below).

Re-read and re-merged fresh on every call (DESIGN.md section 2:
"config resolution happens live every cycle") — a file edited on disk
mid-shift takes effect on the very next cycle, no push mechanism needed.
"""
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
FLEET_DEFAULTS_PATH = CONFIG_DIR / "fleet_defaults.yaml"
SITE_OVERRIDES_DIR = CONFIG_DIR / "site_overrides"
PER_UNIT_DIR = CONFIG_DIR / "per_unit"


def _load_yaml(path: Path) -> Dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _merge(resolved: Dict[str, Dict[str, Any]], layer: Dict[str, Any], source: str) -> None:
    """Apply one layer's fields on top of resolved, in place. Field by
    field — a layer that only sets one field never touches any other
    field's existing value or source."""
    for field, value in layer.items():
        resolved[field] = {"value": value, "source": source}


def resolve(cell_id: str, site_id: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
    """Return cell_id's resolved config: {field: {"value": ..., "source": ...}}.

    site_id selects which config/site_overrides/{site_id}.yaml applies,
    if any — a cell with no site_id, or whose site has no override file,
    just skips that layer. config/per_unit/{cell_id}.yaml always applies
    if it exists, regardless of site_id.

    Tie values don't change precedence (DESIGN.md section 2): if a site
    override and a per-unit correction set the same field to the same
    value, the resolved source still reports "per_unit_correction" —
    provenance reports which layer's decision currently governs the
    field, not whether the numbers happen to match right now.
    """
    resolved: Dict[str, Dict[str, Any]] = {}
    _merge(resolved, _load_yaml(FLEET_DEFAULTS_PATH), "fleet_default")

    if site_id is not None:
        site_path = SITE_OVERRIDES_DIR / f"{site_id}.yaml"
        if site_path.exists():
            _merge(resolved, _load_yaml(site_path), "site_override")

    per_unit_path = PER_UNIT_DIR / f"{cell_id}.yaml"
    if per_unit_path.exists():
        _merge(resolved, _load_yaml(per_unit_path), "per_unit_correction")

    return resolved

"""Line-level config resolver — the second config surface, separate from
the per-cell resolver in config.py.

config.py resolves settings that describe one cell (takt, tolerance,
gain, ...), keyed by cell_id, through fleet -> site -> per-unit
precedence. This resolves settings that describe the Line/topology
itself — buffer size between two stations is the first example
(DESIGN.md section 1a) — which no single cell owns and which don't fit
config.py's per-cell shape at all.

Phase 2 stub, same posture as config.py: line_defaults.yaml only, one
layer. The real version would need a second precedence chain keyed by
edge/site rather than cell_id (buffer size is "configurable per
site/edge" per section 1a) — that layering is a TODO, same as
config.py's.
"""
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
LINE_DEFAULTS_PATH = CONFIG_DIR / "line_defaults.yaml"


def _load_line_defaults() -> Dict[str, Any]:
    with open(LINE_DEFAULTS_PATH) as f:
        return yaml.safe_load(f) or {}


def resolve(edge_id: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
    """Return the resolved Line-level config, shaped like the per-cell
    resolver's output: {field: {"value": ..., "source": ...}}.

    edge_id is accepted now (unused) so the signature already matches
    where this is headed — per-edge overrides once the Line exists —
    rather than needing a breaking signature change later.

    TODO: merge site/per-edge overrides on top of line defaults, field
    by field, mirroring config.py's fleet -> site -> per-unit chain.
    """
    defaults = _load_line_defaults()
    return {
        field: {"value": value, "source": "line_default"}
        for field, value in defaults.items()
    }

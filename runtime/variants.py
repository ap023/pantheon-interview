"""Variant / task-spec requirements.

DESIGN.md section 1 puts the real Variant/Task Spec registry under the
Line — switched at changeover events, read by every cell every cycle.
No Line exists yet, so this is a static lookup rather than something
that changes at runtime (see TODO.md). It exists so Cell's readiness
checks have something concrete to check a variant's sensor requirements
against.
"""
from typing import Dict, Set

VARIANT_REQUIREMENTS: Dict[str, Dict[str, Set[str]]] = {
    "default": {"required_sensors": set()},
    "vision_pick": {"required_sensors": {"wrist_camera"}},
}


def requirements(variant: str) -> Dict[str, Set[str]]:
    try:
        return VARIANT_REQUIREMENTS[variant]
    except KeyError:
        raise KeyError(f"unknown variant {variant!r}; known: {list(VARIANT_REQUIREMENTS)}")

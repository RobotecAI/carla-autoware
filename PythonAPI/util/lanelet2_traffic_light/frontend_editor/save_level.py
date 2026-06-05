"""Resolve the save-level flag for placement runs.

Pure helper (no unreal import) so it is unit-testable outside the editor.
The EUW passes an explicit bool from its "Save Level" checkbox; console
flows leave it None and opt in via the LANELET2_SAVE_LEVEL environment
variable (mirrors the LANELET2_OSM_PATH pattern).
"""
import os
from typing import Mapping, Optional

_TRUTHY = ("1", "true", "yes", "on")


def resolve_save_level(
    explicit: Optional[bool] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> bool:
    """Resolve whether the level should be saved after placement.

    Args:
        explicit: True/False forces the decision (EUW checkbox); None defers
            to the environment.
        environ: mapping used for resolution (defaults to os.environ).

    Returns:
        bool: True when the level should be saved after placement.
    """
    if explicit is not None:
        return bool(explicit)
    env = environ if environ is not None else os.environ
    return env.get("LANELET2_SAVE_LEVEL", "").strip().lower() in _TRUTHY

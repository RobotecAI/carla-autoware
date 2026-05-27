"""OSM path validation for the lanelet2 traffic light placement tool.

Pure-Python (no unreal dependency) so it can be unit-tested. Used by
run_placement to fail early with a helpful message when the OSM path is
unset or missing.
"""
import os
from typing import Optional


def validate_osm_path(osm_path: Optional[str]) -> Optional[str]:
    """Return an error message if the OSM path is unset or missing, else None.

    Args:
        osm_path: lanelet2 .osm file path, typically from the
            LANELET2_OSM_PATH environment variable.

    Returns:
        None if the path is valid (non-empty and exists on disk).
        Otherwise a human-readable error message instructing the user to
        set LANELET2_OSM_PATH.
    """
    if not osm_path or not os.path.exists(osm_path):
        return (
            "lanelet2 OSM file not found.\n"
            "Set the LANELET2_OSM_PATH environment variable to your .osm path "
            "and restart the editor, or pass a valid path via the EUW.\n"
            "Example: export LANELET2_OSM_PATH=/path/to/NishishinjukuMap.osm\n"
            f"Current value: '{osm_path or '(unset)'}'"
        )
    return None

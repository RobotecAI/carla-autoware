"""Per-map parent-BP overrides for derived traffic-light BP generation.

Thin wrapper over `map_profile` (single source of truth). Inc4: Odaiba pedestrians
(subtype "red_green") use a dedicated T4 parent BP that drives the Scene_NNN lamp face
via a runtime material override. Other maps keep the profile default.
"""
from lanelet2_traffic_light.frontend_editor.map_profile import (
    get_map_profile,
    SCENE_FIGURE_PARENT_BP,  # re-exported for backward compatibility
)

__all__ = ["parent_bp_override_for_map", "SCENE_FIGURE_PARENT_BP"]


def parent_bp_override_for_map(map_name) -> dict:
    """Return {subtype: parent_bp_path} overrides for the map (empty = defaults)."""
    return get_map_profile(map_name).parent_bp_override

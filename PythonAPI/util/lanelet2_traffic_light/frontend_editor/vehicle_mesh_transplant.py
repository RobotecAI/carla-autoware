"""Per-map vehicle mesh prefix + transplant policy (pure, no `unreal`).

Thin wrappers over `map_profile` (single source of truth). NishiShinjuku vehicle
meshes are named `TrafficLightsA*` and have no per-color slots (a single LED face with
shared UVs). So vehicles snap to the native mesh for pose, but the StaticMesh is
swapped to a T4 mesh that does have Green/Yellow/Red slots (a copy of Odaiba's Scene_842,
SM_JPVehicleLamp); the existing BP_VehicleTrafficLight slot-name lighting then works.

Inc1 de-risk: the pose offset is baked into the ACTOR world rotation (a component
relative rotation is wiped by the BP construction rerun), the snap Z-gate is disabled,
and material overrides are cleared. See map_profile.MAP_PROFILES for all knobs.
"""
from lanelet2_traffic_light.frontend_editor.map_profile import (
    get_map_profile,
    TRANSPLANT_VEHICLE_MESH,  # re-exported for backward compatibility
)

__all__ = [
    "vehicle_mesh_prefixes_for_map",
    "transplant_mesh_for_map",
    "TRANSPLANT_VEHICLE_MESH",
]


def vehicle_mesh_prefixes_for_map(map_name):
    """Existing-mesh label prefixes to snap vehicle signals to (map-aware)."""
    return get_map_profile(map_name).vehicle_mesh_prefixes


def transplant_mesh_for_map(map_name):
    """Vehicle transplant config dict for the map, or None to keep the native mesh."""
    return get_map_profile(map_name).vehicle_transplant

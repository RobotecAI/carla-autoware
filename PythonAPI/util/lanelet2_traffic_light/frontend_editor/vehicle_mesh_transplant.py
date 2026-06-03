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


def green_arrow_dirs(arrows):
    """Directions of the GREEN arrows from a PlacementSpec.arrows ((color,direction),...).

    The lighting side only handles green arrows (the only color present in the
    target maps and the only color the meshes have slots for). Returns a frozenset
    of "left"/"straight"/"right".
    """
    return frozenset(d for (c, d) in arrows if c == "green")


def native_led_capable(slot_names, led_slot):
    """True when a snapped native mesh can have its R/Y/G lit directly: the map
    declares a native LED slot (MapProfile.vehicle_native_led_slot) and the mesh
    carries it. Decides native-vs-transplant per signal AFTER snap (e.g.
    NishiShinjuku's 14 odd 1x-scale heads lack the slot and keep the transplant).
    """
    return led_slot is not None and led_slot in slot_names


def select_vehicle_transplant(transplant, transplant_arrow, green_dirs, arrow_native=False):
    """Pick the per-signal transplant config.

    arrow_native (per-map): arrow-bearing signals keep their NATIVE mesh -- the map's
    arrow units carry per-direction arrow slots, so no transplant is needed and the
    slots are lit directly (returns None -> the plain native snap path). Otherwise a
    signal with green arrows uses the 6-light arrow transplant when configured, and
    every other signal uses the plain transplant (which may itself be None = native).
    """
    if green_dirs and arrow_native:
        return None
    if green_dirs and transplant_arrow is not None:
        return transplant_arrow
    return transplant

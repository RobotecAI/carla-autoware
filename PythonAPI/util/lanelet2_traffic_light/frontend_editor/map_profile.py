"""Per-map configuration registry (pure, no `unreal`).

Centralizes the map-specific decisions (existing-mesh naming prefixes, vehicle
transplant, parent-BP overrides, ...) in one place. Adding a new map is a single
entry in `MAP_PROFILES`: override only the axes that differ; the rest fall back to
`MapProfile` defaults (generic / Odaiba-like).

When a new decision axis is needed, add a field to `MapProfile` (existing maps keep
the default, so they are unaffected). The frontend helpers are thin wrappers that
just read this registry (`vehicle_mesh_transplant`, `bp_parent_override`).
"""
from dataclasses import dataclass, field
from typing import Optional, Tuple


# --- Shared asset paths (single source of truth) -------------------------------

# T4 parent BP that drives the Scene_NNN lamp face via material override
# (Odaiba pedestrian Spec A).
SCENE_FIGURE_PARENT_BP = (
    "/T4/TrafficLightSample/PedestrianTrafficLight/"
    "BP_PedestrianTrafficLightSceneFigure.BP_PedestrianTrafficLightSceneFigure"
)

# T4 mesh used to replace vehicle signals on maps whose native meshes lack
# per-color material slots (a copy of Odaiba's Scene_842).
TRANSPLANT_VEHICLE_MESH = (
    "/T4/TrafficLightSample/VehicleTrafficLight/SM_JPVehicleLamp.SM_JPVehicleLamp"
)

# 6-light variant (R/Y/G + 3 green arrow slots) for arrow-bearing vehicle signals
# on transplant maps. A copy of an Odaiba `_Modified` Scene mesh (Inc 0 de-risk).
TRANSPLANT_VEHICLE_MESH_ARROW = (
    "/T4/TrafficLightSample/VehicleTrafficLight/SM_JPVehicleLampArrow.SM_JPVehicleLampArrow"
)

# NishiShinjuku vehicle transplant settings (values confirmed in the Inc1 de-risk).
#   mesh:               replacement mesh asset path.
#   scale:              component relative scale (snap does not adopt native scale).
#   relrot_deg:         (pitch, yaw, roll) mesh-convention offset, baked into the
#                       ACTOR world rotation via compose_rotators (a component
#                       relative rotation gets wiped by the BP construction rerun).
#   world_z_offset_cm:  height pivot correction, added to the actor world Z.
#   local_offset_cm:    (x, y, z) depth/lateral pivot correction in the mesh local frame.
#   ignore_snap_z_gate: disable the snap Z-validity check (the spec Z is wrong here,
#                       and snap adopts the mesh Z anyway).
_NISHISHINJUKU_VEHICLE_TRANSPLANT = {
    "mesh": TRANSPLANT_VEHICLE_MESH,
    "scale": 1.0,
    "relrot_deg": (0.0, 120.0, -90.0),
    "world_z_offset_cm": -24.0,
    "local_offset_cm": (0.0, 0.0, 0.0),
    "ignore_snap_z_gate": True,
}

# Arrow (6-light) transplant pose for NishiShinjuku. The 6-light geometry differs
# from the 3-light SM_JPVehicleLamp, so the pose is a separate set (values are the
# 3-light starting point; confirmed/updated in the Inc 0 de-risk).
_NISHISHINJUKU_VEHICLE_TRANSPLANT_ARROW = {
    "mesh": TRANSPLANT_VEHICLE_MESH_ARROW,
    "scale": 1.0,
    "relrot_deg": (0.0, 120.0, -90.0),
    "world_z_offset_cm": -24.0,
    "local_offset_cm": (0.0, 0.0, 0.0),
    "ignore_snap_z_gate": True,
}


# --- Map profile ----------------------------------------------------------------

@dataclass(frozen=True)
class MapProfile:
    """Placement/lighting decision knobs for one map. Defaults are generic (Odaiba-like)."""

    # Label prefixes used to find existing meshes by startswith() during snap.
    vehicle_mesh_prefixes: Tuple[str, ...] = ("Traffic_Lights",)
    pedestrian_mesh_prefixes: Tuple[str, ...] = ("Pedestrian_Lights",)

    # For maps whose native meshes lack per-color slots: snap for pose but swap the
    # mesh to a T4 mesh (settings dict). None keeps the native Scene_NNN (e.g. Odaiba).
    vehicle_transplant: Optional[dict] = None

    # For transplant maps: the 6-light arrow mesh config used for signals that have
    # green arrows (per-signal selection). None keeps the plain transplant / native.
    vehicle_transplant_arrow: Optional[dict] = None

    # subtype -> parent BP path override for derived-BP generation
    # (e.g. Odaiba pedestrian -> SceneFigure).
    parent_bp_override: dict = field(default_factory=dict)

    # Whether pedestrians snap to an existing native mesh. False keeps the canonical
    # parent-BP mesh (snap=False): NishiShinjuku's native TrafficLightB meshes are
    # ~10x scale, so snapping to them would produce giant signals; the canonical
    # TrafficLightPedestrian mesh is the correct real-world size.
    pedestrian_uses_snap: bool = True


_DEFAULT = MapProfile()

MAP_PROFILES = {
    "Odaiba": MapProfile(
        # Vehicle/pedestrian both use Odaiba naming (same as default, kept explicit).
        parent_bp_override={"red_green": SCENE_FIGURE_PARENT_BP},
    ),
    "NishishinjukuMap": MapProfile(
        vehicle_mesh_prefixes=("TrafficLightsA",),
        pedestrian_mesh_prefixes=("TrafficLightB",),
        vehicle_transplant=_NISHISHINJUKU_VEHICLE_TRANSPLANT,
        vehicle_transplant_arrow=_NISHISHINJUKU_VEHICLE_TRANSPLANT_ARROW,
        # Pedestrians are canonical (snap=False): the native TrafficLightB meshes are
        # ~10x scale, so they keep the correctly-sized canonical parent-BP mesh and are
        # placed at the mesh-derived OSM pose. No parent override (canonical BP).
        pedestrian_uses_snap=False,
    ),
}


def get_map_profile(map_name) -> MapProfile:
    """Return the MapProfile for map_name, or the generic default for unknown maps."""
    return MAP_PROFILES.get(map_name, _DEFAULT)

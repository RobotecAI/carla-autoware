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

# Arrow (6-light) transplant pose. Superseded for NishiShinjuku by the native arrow
# lighting below (2026-06-03); kept as a registry option for future maps whose native
# arrow units lack per-direction slots.
_NISHISHINJUKU_VEHICLE_TRANSPLANT_ARROW = {
    "mesh": TRANSPLANT_VEHICLE_MESH_ARROW,
    "scale": 1.0,
    "relrot_deg": (0.0, 120.0, -90.0),
    "world_z_offset_cm": -24.0,
    "local_offset_cm": (0.0, 0.0, 0.0),
    "ignore_snap_z_gate": True,
}

# Native arrow lighting for NishiShinjuku (de-risk 2026-06-03). The arrow-bearing native
# units (33 units / 21 mesh variants, slot names fully uniform) carry per-direction arrow
# slots, so arrow signals keep their native mesh (snap pose AND world scale: the meshes
# are ~10x-authored and shrunk by a parent) and light the slots with M_JPArrowLit using
# the map's own per-direction arrow textures. black/white_level differ from the Odaiba
# texture family (these textures saturate at the Odaiba defaults).
_NISHISHINJUKU_TRAFFIC_DIR = (
    "/Game/Carla/Maps/Nishishinjuku/Content/NishishinjukuMap/SJK01_P03/"
    "SJK01_P03_All_GP01/TrafficAssets_Root01_All_GP/TrafficLightA01_Root01_ALL_GP01"
)
_NISHISHINJUKU_VEHICLE_ARROW_NATIVE = {
    "slot_by_dir": {
        "left": "TrafficLightsLeftArrow",
        "straight": "TrafficLightsUpArrow",
        "right": "TrafficLightsRightArrow",
    },
    "tex_by_dir": {
        "left": (_NISHISHINJUKU_TRAFFIC_DIR
                 + "/TrafficLightsA03_Root01_ALL_GP01/TrafficLightsA01_Led01_Share01_col_arrow_left"),
        "straight": (_NISHISHINJUKU_TRAFFIC_DIR
                     + "/TrafficLightsA01_Root01_All_GP01/TrafficLightsA01_Led01_Share01_col_arrow_up"),
        "right": (_NISHISHINJUKU_TRAFFIC_DIR
                  + "/TrafficLightsA02_Root01_ALL_GP01/TrafficLightsA01_Led01_Share01_col_arrow_right"),
    },
    "black_level": 0.10,
    "white_level": 0.40,
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

    # Native arrow lighting config (slot_by_dir / tex_by_dir / black_level / white_level).
    # When set, arrow-bearing vehicle signals skip the transplant entirely: they snap the
    # native mesh (pose AND world scale) and light its per-direction arrow slots. None
    # falls back to the arrow_lighting module defaults (Odaiba slots + T4 textures).
    vehicle_arrow_native: Optional[dict] = None

    # Bypass the snap Z-validity gate for ALL vehicle snaps (native and transplant).
    # A MAP property, not a transplant property: on maps whose vehicle spec Z is
    # unreliable (NishiShinjuku source data yields a uniform bogus Z), gating by
    # |mesh_z - spec_z| rejects every candidate; snap adopts the mesh Z anyway, so
    # the gate is XY-only there. (The per-transplant ignore_snap_z_gate flag is
    # still honored for backward compatibility.)
    vehicle_snap_ignore_z_gate: bool = False

    # Native LED (R/Y/G) slot name for hybrid per-signal native lighting. When set, a
    # vehicle signal whose SNAPPED mesh carries this slot skips the transplant: it keeps
    # the native mesh (pose AND world scale) and the vehicle BP drives M_JPVehicleLedRYG
    # on this slot per set_state (self-gating, same gate as the BP's Get Material Index).
    # Signals snapping to meshes WITHOUT the slot fall back to the transplant. None
    # disables (vehicle_transplant decides as before).
    vehicle_native_led_slot: Optional[str] = None

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
        # Vehicle spec Z is a uniform bogus value on this map -> XY-only snap gate
        # for every vehicle signal (native arrows/LEDs would otherwise all be
        # rejected by the |mesh_z - spec_z| fallback gate; 2026-06-03 full run).
        vehicle_snap_ignore_z_gate=True,
        # Arrow signals are NATIVE (no transplant): per-direction arrow slots exist on the
        # native units, so they keep their own mesh and light those slots (2026-06-03).
        vehicle_arrow_native=_NISHISHINJUKU_VEHICLE_ARROW_NATIVE,
        # 3-light signals are ALSO native when possible (level scan 2026-06-03: all 51
        # non-arrow head variants carry TrafficLightsLed with the same lamp-row geometry
        # as the arrow units). The 14 odd 1x-scale heads (slot ..._Led01_Share01_col)
        # lack the slot and keep the transplant (per-signal fallback after snap).
        vehicle_native_led_slot="TrafficLightsLed",
        # Pedestrians are canonical (snap=False): the native TrafficLightB meshes are
        # ~10x scale, so they keep the correctly-sized canonical parent-BP mesh and are
        # placed at the mesh-derived OSM pose. No parent override (canonical BP).
        pedestrian_uses_snap=False,
    ),
}


def get_map_profile(map_name) -> MapProfile:
    """Return the MapProfile for map_name, or the generic default for unknown maps."""
    return MAP_PROFILES.get(map_name, _DEFAULT)

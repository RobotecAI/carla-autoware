"""Asset Path construction for per-subtype derived Blueprints.

No unreal dependency (pure Python). Fully testable.

Added in Phase 5 step 1: consolidates the path naming convention for
auto-generating derived BPs on different maps.
The default convention is `/Game/Carla/Blueprints/{map_name}/BP_{map_name}{Role}TL.BP_...`,
but output_dir or name_template can be overridden to change it.
"""
from typing import Optional


# subtype -> role name for the derived BP (used in BP naming)
_SUBTYPE_TO_ROLE = {
    "red_yellow_green": "Vehicle",
    "red_green":        "Pedestrian",
}


def role_for_subtype(subtype: str) -> str:
    """Return the role name ("Vehicle" / "Pedestrian") corresponding to the given subtype."""
    if subtype not in _SUBTYPE_TO_ROLE:
        raise KeyError(f"unknown subtype: {subtype}")
    return _SUBTYPE_TO_ROLE[subtype]


def derive_group_bp_path(
    map_name: str,
    output_dir: Optional[str] = None,
    name_template: str = "BP_{map_name}TrafficLightGroup",
) -> str:
    """Build the Asset Path of an `ATrafficLightGroup` derived BP from map_name.

    Example: map_name="Odaiba"
        -> /Game/Carla/Blueprints/Odaiba/BP_OdaibaTrafficLightGroup.BP_OdaibaTrafficLightGroup

    Added in Phase 5 step 2: auto-generates a C++ ATrafficLightGroup derived BP for each
    map so that snap-mode placed TLs can be connected to Carla's standard traffic light
    control via Controller -> Group.
    """
    bp_name = name_template.format(map_name=map_name)
    dir_path = output_dir if output_dir is not None else f"/Game/Carla/Blueprints/{map_name}"
    return f"{dir_path}/{bp_name}.{bp_name}"


def derive_subtype_bp_path(
    subtype: str,
    map_name: str,
    output_dir: Optional[str] = None,
    name_template: str = "BP_{map_name}{role}TL",
) -> str:
    """Build the derived BP Asset Path (without `_C`) from subtype + map_name.

    Args:
        subtype: lanelet2 subtype. "red_yellow_green" or "red_green".
        map_name: Map name (key for BP naming). e.g. "Odaiba".
        output_dir: Directory in `/Game/...` format.
            Defaults to `/Game/Carla/Blueprints/{map_name}` if None.
        name_template: Naming template. Must include `{map_name}` and `{role}`.

    Returns:
        Asset Path string. e.g.:
        ``/Game/Carla/Blueprints/Odaiba/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL``
    """
    role = role_for_subtype(subtype)
    bp_name = name_template.format(map_name=map_name, role=role)
    dir_path = output_dir if output_dir is not None else f"/Game/Carla/Blueprints/{map_name}"
    return f"{dir_path}/{bp_name}.{bp_name}"

"""Auto-generation of per-subtype derived Blueprints (Phase 5 step 1).

When deploying to a different map, derived BPs (e.g. BP_ChibaVehicleTL) are
automatically created from the Editor as children of the T4 sample BP.

In snap mode, StaticMesh / Rotation are overwritten dynamically, so the
derived BP body can be mostly empty. What matters is inheriting the parent class:
using the T4 sample BP as parent inherits Carla's standard traffic light lighting
logic; when unspecified (None), it becomes a direct C++ TrafficLightBase subclass.

Depends on the unreal module (runs inside the Editor only). Path construction
logic is separated into bp_naming.py for testability.
"""
from typing import Optional

import unreal

from lanelet2_traffic_light.frontend_editor.bp_naming import (
    derive_group_bp_path,
    derive_subtype_bp_path,
)


def _asset_path_only(asset_path: str) -> str:
    """Return the package path portion `/X/Y/BP_Foo` from a `/X/Y/BP_Foo.BP_Foo` format string."""
    if "." in asset_path:
        return asset_path.split(".")[0]
    if asset_path.endswith("_C"):
        return asset_path[:-2]
    return asset_path


def _load_parent_class(parent_bp_path: Optional[str]):
    """Load the Generated Class for parent_bp_path.
    Returns C++ TrafficLightBase if None is given.
    """
    if parent_bp_path is None:
        return unreal.TrafficLightBase
    asset_path = _asset_path_only(parent_bp_path)
    cls = unreal.EditorAssetLibrary.load_blueprint_class(asset_path)
    if cls is None:
        # T4 sample BP not found. Emit warning and fall back.
        unreal.log_warning(
            f"bp_factory: parent Blueprint not found at '{asset_path}', "
            f"falling back to C++ TrafficLightBase. Lighting logic will be absent."
        )
        return unreal.TrafficLightBase
    return cls


def create_derived_bp(parent_bp_path: Optional[str], target_asset_path: str) -> bool:
    """Create a derived BP of the parent BP at target_asset_path.

    Args:
        parent_bp_path: Asset Path of the parent BP. None means C++ TrafficLightBase.
        target_asset_path: Destination Asset Path.
            e.g. `/Game/Carla/Blueprints/Chiba/BP_ChibaVehicleTL.BP_ChibaVehicleTL`.

    Returns:
        True if newly created, False if an existing asset was reused.

    Raises:
        RuntimeError: If asset creation fails.
    """
    target_pkg = _asset_path_only(target_asset_path)
    if unreal.EditorAssetLibrary.does_asset_exist(target_pkg):
        return False

    parent_cls = _load_parent_class(parent_bp_path)

    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", parent_cls)

    package_path, asset_name = target_pkg.rsplit("/", 1)
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    new_asset = asset_tools.create_asset(
        asset_name=asset_name,
        package_path=package_path,
        asset_class=unreal.Blueprint,
        factory=factory,
    )
    if new_asset is None:
        raise RuntimeError(
            f"bp_factory: AssetTools.create_asset failed for {target_pkg}"
        )

    unreal.EditorAssetLibrary.save_asset(target_pkg)
    return True


def ensure_group_bp(
    map_name: str,
    output_dir: Optional[str] = None,
) -> str:
    """Create or reuse a derived BP of C++ `ATrafficLightGroup` per map.

    Added in Phase 5 step 2: auto-generates BP_<MapName>TrafficLightGroup for each map
    to connect snap-mode placed TLs to Carla's standard traffic light control
    (Group -> Controller -> Component). A direct C++ `unreal.TrafficLightGroup`
    subclass is sufficient as the parent class (Tick logic lives in C++).

    Args:
        map_name: e.g. "Odaiba" / "Chiba".
        output_dir: Destination directory (defaults to /Game/Carla/Blueprints/{map_name}).

    Returns:
        Asset Path of the created or reused Group derived BP.
        Passed to `_place_groups()` as the spawn source class.
    """
    target = derive_group_bp_path(map_name, output_dir)
    target_pkg = _asset_path_only(target)
    if unreal.EditorAssetLibrary.does_asset_exist(target_pkg):
        unreal.log(f"bp_factory: reused existing Group BP {target}")
        return target

    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", unreal.TrafficLightGroup)
    package_path, asset_name = target_pkg.rsplit("/", 1)
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    new_asset = asset_tools.create_asset(
        asset_name=asset_name,
        package_path=package_path,
        asset_class=unreal.Blueprint,
        factory=factory,
    )
    if new_asset is None:
        raise RuntimeError(
            f"bp_factory: AssetTools.create_asset failed for Group BP {target_pkg}"
        )
    unreal.EditorAssetLibrary.save_asset(target_pkg)
    unreal.log(
        f"bp_factory: created Group BP {target} "
        f"(parent=C++ ATrafficLightGroup)"
    )
    return target


def ensure_subtype_bps(
    profile,
    map_name: str,
    output_dir: Optional[str] = None,
    parent_override: Optional[dict] = None,
) -> dict:
    """Ensure derived BPs for each subtype in the profile and return {subtype: target_path}.

    Reuses existing BPs if found (idempotent). Returns a dict that can be passed
    directly as the `bp_override` argument to core.api.generate_placements.

    Args:
        profile: A profile (e.g. PROFILE_JP) that has `parent_bp_path_for(subtype)`
            and `_subtype_to_bp`. The latter is used to enumerate subtypes to process.
        map_name: Map name (key for derived BP naming). e.g. "Odaiba" / "Chiba".
        output_dir: Destination directory (defaults to /Game/Carla/Blueprints/{map_name}).

    Returns:
        dict of {subtype: target_asset_path}.
    """
    result = {}
    # Enumerate subtypes from the profile's _subtype_to_bp keys
    subtypes = list(profile._subtype_to_bp.keys())
    for subtype in subtypes:
        target = derive_subtype_bp_path(subtype, map_name, output_dir)
        # parent_override (per-map) takes precedence over the profile default.
        parent = (parent_override or {}).get(subtype) or profile.parent_bp_path_for(subtype)
        try:
            created = create_derived_bp(parent, target)
            if created:
                unreal.log(
                    f"bp_factory: created derived BP {target} "
                    f"(parent={parent or '<C++ TrafficLightBase>'})"
                )
            else:
                unreal.log(f"bp_factory: reused existing BP {target}")
            result[subtype] = target
        except Exception as e:
            unreal.log_error(
                f"bp_factory: failed to ensure {target}: {e}. "
                f"Falling back to profile default."
            )
            # Fallback: use the existing _subtype_to_bp path from the profile
            result[subtype] = profile.bp_class_for(subtype)
    return result

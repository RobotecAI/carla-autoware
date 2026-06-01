"""UE5 Editor Python frontend.

Converts `PlacementSpec` output from the core package into Actor placements in the level.
This is the only file that imports the `unreal` module.

Design spec: docs/superpowers/specs/2026-05-20-lanelet2-traffic-light-design.md
Implementation plan: docs/superpowers/plans/2026-05-20-lanelet2-traffic-light.md
Phase 0 findings: PythonAPI/util/lanelet2_traffic_light/docs/phase0_validation.md
"""
from dataclasses import dataclass, field
from typing import Optional

import unreal

from lanelet2_traffic_light.corelib.ir.traffic_light_ir import PlacementSpec, GroupSpec
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
from lanelet2_traffic_light.frontend_editor.snap_stats import (
    MeshZStats, compute_z_stats,
)
from lanelet2_traffic_light.frontend_editor.mesh_override_policy import (
    should_skip_mesh_override,
)
from lanelet2_traffic_light.frontend_editor.map_profile import get_map_profile


# Default Group BP path prior to Phase 5-2 (no longer used;
# per-map derived BP passed via argument to `_place_groups()`).
# The constant itself is kept for compatibility, but references are deprecated.
BP_TRAFFIC_LIGHT_GROUP_PATH = "/Carla/Blueprints/TrafficLight/BP_TrafficLightGroup.BP_TrafficLightGroup_C"


@dataclass
class PlacementReport:
    """Execution result summary of place_from_specs()."""
    created: int = 0
    updated: int = 0
    failed: list = field(default_factory=list)  # list of (sign_id, reason_str) tuples
    # Specs skipped because no matching existing mesh was found when
    # snap_to_existing_mesh=True. Traffic lights outside the 3D cityscape
    # coverage of Odaiba.umap end up here (lanelet2 covers the entire Tokyo bay
    # area, but meshes only cover the Odaiba center area).
    snap_skipped: list = field(default_factory=list)  # (sign_id, target_xyz_cm)
    snap_skipped_records: list = field(default_factory=list)  # dict form (for key=value output)
    # Reverse mismatch: existing signal meshes present as Odaiba meshes but not
    # matched to any lanelet2 way — list of (label, (x_cm, y_cm, z_cm)).
    # Useful for detecting leftover meshes from removed signals, redundant map
    # design, or missing lanelet2 edits (added in Phase 4.2).
    unused_existing_meshes: list = field(default_factory=list)
    # Snap detail records for successfully placed specs (for report file output).
    # Each element is a dict: {sign_id, was_created, target_label, xy_dist_cm, mesh_name}
    placed_records: list = field(default_factory=list)
    # Snapshot of Z statistics (for report output)
    z_stats_snapshot: dict = field(default_factory=dict)
    groups_created: int = 0
    groups_updated: int = 0
    pedestrian_material_stats: list = field(default_factory=list)
    # stores the return value list of aggregate_material_stats


# ---------------------------------------------------------------------------
# 3.1: get_world_mgrs_data
# ---------------------------------------------------------------------------

def _get_editor_world():
    """Return the world currently open in the editor.

    In UE5.5+, EditorLevelLibrary.get_editor_world() is deprecated, so
    UnrealEditorSubsystem is preferred; the legacy API is used as fallback.
    """
    subsys = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    if subsys is not None and hasattr(subsys, "get_editor_world"):
        return subsys.get_editor_world()
    # Fallback: for environments prior to UE5.4
    return unreal.EditorLevelLibrary.get_editor_world()


def _resolve_soft_object(soft_ref):
    """Resolve a TSoftObjectPtr / SoftObjectPath and return the UObject.

    In unreal Python, a TSoftObjectPtr may behave directly as a UObject or
    may arrive as a SoftObjectPath. Both cases are handled here.
    """
    if soft_ref is None:
        return None
    # For SoftObjectPath, extract the path string and load it
    if isinstance(soft_ref, unreal.SoftObjectPath):
        path = soft_ref.get_path_name() if hasattr(soft_ref, "get_path_name") else str(soft_ref)
        return unreal.EditorAssetLibrary.load_asset(path)
    # Already a resolved UObject (TSoftObjectPtr auto-resolved case) — return as-is
    try:
        if hasattr(soft_ref, "get_path_name"):
            return soft_ref
    except Exception:
        pass
    return soft_ref


def get_world_mgrs_data() -> MgrsTransformer:
    """Extract MgrsDataAsset from the current level's WorldSettings and
    construct a MgrsTransformer using the formula confirmed in Phase 0.

    Actual property name discovered in Phase 0: `mgrs_data_asset_soft_ptr` (TSoftObjectPtr)
    Unit of MgrsOffsetPosition: m (not cm)

    Raises:
        RuntimeError: If the expected property does not exist in WorldSettings,
                      or if the asset fails to load.
    """
    world = _get_editor_world()
    ws = world.get_world_settings()

    # Confirmed in Phase 0: property name is mgrs_data_asset_soft_ptr
    # mgrs_data_asset / MgrsDataAsset from the old plan draft are incorrect
    raw = ws.get_editor_property("mgrs_data_asset_soft_ptr")
    if raw is None:
        raise RuntimeError(
            "WorldSettings has no 'mgrs_data_asset_soft_ptr'. "
            "Confirm the level uses AutowareWorldSettings and the asset is assigned."
        )

    da = _resolve_soft_object(raw)
    if da is None:
        raise RuntimeError("MgrsDataAsset could not be loaded from soft reference.")

    offset = da.get_editor_property("mgrs_offset_position")
    if offset is None:
        raise RuntimeError("MgrsDataAsset.MgrsOffsetPosition is null.")

    # Confirmed in Phase 0: offset.x / offset.y / offset.z are in metres
    # x_sign=+1, y_sign=-1: standard CARLA Z-up with Y-only inversion (confirmed in Phase 0)
    return MgrsTransformer(
        offset_x_m=float(offset.x),
        offset_y_m=float(offset.y),
        offset_z_m=float(offset.z),
        x_sign=+1,
        y_sign=-1,   # standard CARLA Z-up with Y-only inversion
    )


# ---------------------------------------------------------------------------
# 3.2: find_actor_by_sign_id
# ---------------------------------------------------------------------------

def find_actor_by_sign_id(sign_id: str) -> Optional[unreal.Actor]:
    """Return the TrafficLightBase actor in the level whose sign_id matches.

    Scans all actors in the level via EditorActorSubsystem, looking for one
    that is a TrafficLightBase subclass and whose
    TrafficLightComponent.get_sign_id() matches the given value.

    If multiple actors match, a warning is logged for debugging and the first
    one is returned.  Returns None if no match is found.
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    matches = []

    for a in actor_subsys.get_all_level_actors():
        # Skip actors that are not TrafficLightBase subclasses
        if not isinstance(a, unreal.TrafficLightBase):
            continue

        tlc = a.get_traffic_light_component()
        if tlc is None:
            continue

        try:
            sid = tlc.get_sign_id()
        except Exception:
            # Safe fallback for old BP classes that lack get_sign_id
            continue

        if sid == sign_id:
            matches.append(a)

    if len(matches) > 1:
        labels = [m.get_actor_label() for m in matches]
        unreal.log_warning(
            f"find_actor_by_sign_id: multiple actors share sign_id={sign_id}: {labels}. "
            "Returning first. Consider removing duplicates."
        )

    return matches[0] if matches else None


# ---------------------------------------------------------------------------
# 3.3: spawn / update single placement
# ---------------------------------------------------------------------------

def _load_bp_class(class_path: str):
    """Load a GeneratedClass (UClass) from an ObjectPath.

    `EditorAssetLibrary.load_blueprint_class()` expects an asset path
    (`/Game/.../BP_X.BP_X`) and returns `BP_X_C` (the GeneratedClass).
    Any `_C` suffix passed by the caller is stripped before forwarding.

    Raises:
        RuntimeError: If loading fails.
    """
    asset_path = class_path[:-2] if class_path.endswith("_C") else class_path
    cls = unreal.EditorAssetLibrary.load_blueprint_class(asset_path)
    if cls is None:
        raise RuntimeError(f"Failed to load blueprint class: {class_path}")
    return cls


def _collect_unused_meshes(used_labels: set,
                           label_prefixes=("Traffic_Lights", "Pedestrian_Lights")) -> list:
    """Return a list of existing signal meshes not included in `used_labels` (Poles excluded).

    Added in Phase 4.2: detects existing meshes that were missed in lanelet2 association.
    Useful for finding leftover meshes from removed signals, redundant map design,
    or missing lanelet2 edits.

    Returns:
        [(label, (x_cm, y_cm, z_cm)), ...] list of tuples including world position.
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    out = []
    for a in actor_subsys.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor):
            continue
        label = a.get_actor_label()
        if "Pole" in label:
            continue
        if not any(label.startswith(p) for p in label_prefixes):
            continue
        if label in used_labels:
            continue
        loc = a.get_actor_location()
        out.append((label, (loc.x, loc.y, loc.z)))
    return out


def _collect_mesh_z_stats(label_prefixes=("Traffic_Lights", "Pedestrian_Lights")) -> dict:
    """Collect per-prefix Z statistics for existing meshes from the Editor.

    Added in Phase 4.2: called once at the start of a Full Run to absorb
    per-map Z reference offsets (sea level / ellipsoid / custom datum) and
    pole_height differences.  The result is passed to
    `_find_nearest_existing_signal_mesh` for Z validity checks.

    Args:
        label_prefixes: List of prefixes to collect. Labels containing "Pole" are excluded.

    Returns:
        Dict of {prefix: MeshZStats}. Prefixes with no matching meshes are omitted.
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    by_prefix: dict = {p: [] for p in label_prefixes}
    for a in actor_subsys.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor):
            continue
        label = a.get_actor_label()
        if "Pole" in label:
            continue
        for p in label_prefixes:
            if label.startswith(p):
                by_prefix[p].append(a.get_actor_location().z)
                break
    return {p: compute_z_stats(zs) for p, zs in by_prefix.items() if zs}


def _collect_pedestrian_mesh_materials() -> list:
    """Aggregate the Material element composition of Pedestrian_Lights_* actors in the level.

    Returns:
        Output list from aggregate_material_stats():
        [{"mesh": str, "num_elements": int, "elements": tuple, "count": int}, ...]
    """
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats

    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    records: list = []
    for a in actor_subsys.get_all_level_actors():
        label = a.get_actor_label()
        if not label.startswith("Pedestrian_Lights"):
            continue
        smc = None
        try:
            smc = a.static_mesh_component
        except AttributeError:
            # Skip non-StaticMeshActor actors
            try:
                smc = a.get_component_by_class(unreal.StaticMeshComponent)
            except Exception:
                smc = None
        if smc is None:
            continue
        try:
            mesh_asset = smc.static_mesh
        except Exception:
            mesh_asset = None
        if mesh_asset is None:
            continue
        try:
            mesh_name = mesh_asset.get_name()
        except Exception:
            continue
        # Retrieve Material element names in order
        try:
            num = smc.get_num_materials()
        except Exception:
            num = 0
        element_names: list = []
        for i in range(num):
            try:
                mat = smc.get_material(i)
                element_names.append(mat.get_name() if mat is not None else "<None>")
            except Exception:
                element_names.append("<error>")
        records.append((mesh_name, tuple(element_names)))
    return aggregate_material_stats(records)


def _label_prefixes_for_bp_class(bp_class_path: str, map_name: str = None) -> tuple:
    """Determine the existing mesh label prefix that corresponds to a BP class path.

    Existing signal meshes come in two families, with map-specific naming:

    | subtype           | BP class              | Odaiba mesh      | NishiShinjuku    |
    |-------------------|-----------------------|------------------|------------------|
    | red_yellow_green  | BP_*VehicleTL         | Traffic_Lights_* | TrafficLightsA*  |
    | red_green         | BP_*PedestrianTL      | Pedestrian_Lights_* | (snap=False)  |

    Prefixes are kept strictly separate to prevent a pedestrian BP from
    accidentally snapping to a vehicle mesh. The vehicle prefix is map-aware
    (NishiShinjuku uses TrafficLightsA*); pedestrians keep Pedestrian_Lights.
    """
    prof = get_map_profile(map_name)
    if "Pedestrian" in bp_class_path:
        return prof.pedestrian_mesh_prefixes
    return prof.vehicle_mesh_prefixes


def _label_prefix_for_subtype(subtype: str) -> str:
    """Return the actor label prefix for a given subtype.

    - red_yellow_green (vehicle)  → "TLV_"
    - red_green (pedestrian)      → "TLP_"
    - other (unknown subtype)     → "TL_" (fallback, compatible with pre-Phase 5-2)
    """
    if subtype == "red_yellow_green":
        return "TLV_"
    if subtype == "red_green":
        return "TLP_"
    return "TL_"


def _find_nearest_existing_signal_mesh(target: unreal.Vector,
                                       max_distance_cm: float = 300.0,
                                       label_prefixes: tuple = ("Traffic_Lights",),
                                       max_z_diff_cm: float = 700.0,
                                       mesh_z_stats: Optional[dict] = None):
    """Return the nearest existing signal mesh StaticMeshActor to target (XY-distance based).

    Distance is computed using **XY plane distance** (changed in Phase 4.2).
    Pedestrian signals are placed at 7-10 m above ground, which differs from
    the core-computed pole_height (12.3 m for vehicles) by several metres; using
    3D distance would put them outside radius=300 cm and cause snap failures.
    XY distance resolves this.  Z validity is verified separately to prevent
    incorrect snapping:

    - If `mesh_z_stats` is provided: meshes outside the Tukey 1.5 IQR range
      per prefix are excluded (map-independent, adaptive).
    - Otherwise: meshes whose Z difference from target exceeds `max_z_diff_cm`
      are excluded (fixed value, fallback).

    Args:
        target: Search center (Unreal world cm).
        max_distance_cm: XY plane distance limit (cm). Unlimited if <= 0.
        label_prefixes: Allowed actor label prefixes.
        max_z_diff_cm: Absolute Z difference limit when mesh_z_stats is None.
        mesh_z_stats: Dict of `{prefix: MeshZStats}` obtained from
            `_collect_mesh_z_stats()`. Takes priority over the fixed fallback.

    Returns:
        (actor or None, xy_distance_cm)
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    best_actor = None
    best_xy_sq = float("inf")
    max_xy_sq = (max_distance_cm * max_distance_cm) if max_distance_cm > 0 else float("inf")
    abs_max_z = max_z_diff_cm if max_z_diff_cm > 0 else float("inf")

    for a in actor_subsys.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor):
            continue
        label = a.get_actor_label()
        matched_prefix = None
        for p in label_prefixes:
            if label.startswith(p):
                matched_prefix = p
                break
        if matched_prefix is None:
            continue
        loc = a.get_actor_location()

        # Z validity check: stats take priority; static fallback otherwise
        if mesh_z_stats and matched_prefix in mesh_z_stats:
            stats = mesh_z_stats[matched_prefix]
            if loc.z < stats.tukey_low or loc.z > stats.tukey_high:
                continue
        else:
            dz = loc.z - target.z
            if dz > abs_max_z or dz < -abs_max_z:
                continue

        dx = loc.x - target.x
        dy = loc.y - target.y
        xy_sq = dx * dx + dy * dy
        if xy_sq < best_xy_sq and xy_sq <= max_xy_sq:
            best_xy_sq = xy_sq
            best_actor = a
    return best_actor, (best_xy_sq ** 0.5 if best_actor is not None else float("inf"))


def _spawn_or_update(spec: PlacementSpec,
                     snap_to_existing_mesh: bool = False,
                     snap_radius_cm: float = 300.0,
                     skip_when_snap_fails: bool = True,
                     mesh_z_stats: Optional[dict] = None,
                     map_name: str = None,
                     transplant: Optional[dict] = None) -> tuple:
    """Update location/rotation if spec.sign_id already exists in the level; otherwise spawn new.

    Core idempotency logic: running with the same sign_id any number of times
    guarantees no duplicate actors.

    Spawned as an independent actor (confirmed in Phase 0: existing signal meshes
    are attached to OdaibaFinaL_ver7_lights, but actors spawned here are placed
    without a parent attachment).

    Args:
        spec: PlacementSpec produced by core.api.generate_placements().
              location_cm is (X_cm, Y_cm, Z_cm);
              rotation_deg is in **(roll, pitch, yaw)** order.
              core.api generates rotation_deg=(0.0, 0.0, yaw_deg).
        snap_to_existing_mesh: If True, adopts the position and rotation of a
              nearby existing mesh (Traffic_Lights_* for vehicles,
              Pedestrian_Lights_* for pedestrians) when one is found.
              Absorbs a few cm / a few degrees of drift from core calculations
              (Odaiba-specific fine-tuning mode).
        snap_radius_cm: Maximum distance for snap matching. Meshes farther
              than this are ignored and the core-computed value is used.
        skip_when_snap_fails: If snap_to_existing_mesh=True and no nearby mesh
              is found, skip the new spawn (added in Phase 4.2).
              Prevents signals outside the 3D cityscape coverage of Odaiba.umap
              from appearing in mid-air.
              Does not affect updates to existing actors (update runs regardless).

    Returns:
        (actor, was_created, snap_info):
          - Normal placement: (actor, True/False, dict or None)
          - Skip due to snap failure: (None, False, None)
        snap_info is a detail dict for the adopted existing mesh in snap mode:
          {"label": str, "xy_dist_cm": float, "mesh_name": str}
        None when snap mode is off or no snap target was found.
        Used for report file output and reverse-mismatch aggregation.

    Raises:
        RuntimeError: If BP class loading or spawn fails.
    """
    # PlacementSpec.location_cm is a (X, Y, Z) tuple
    location = unreal.Vector(spec.location_cm[0], spec.location_cm[1], spec.location_cm[2])
    # PlacementSpec.rotation_deg is in (roll, pitch, yaw) order.
    # `unreal.Rotator` positional arguments are (pitch, yaw, roll), so
    # keyword arguments are used explicitly to avoid axis mix-ups.
    roll_deg, pitch_deg, yaw_deg = spec.rotation_deg
    rotation = unreal.Rotator(roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg)

    # Snap mode: adopt position, rotation, and StaticMesh asset from existing mesh to absorb drift.
    # Search prefix is switched based on BP class path (Pedestrian → Pedestrian_Lights).
    # Inc4: pedestrians and vehicles both take the snap path (adopt Scene_NNN mesh
    # + natural orientation). Figures are driven via runtime material override.
    skip_mesh_override = should_skip_mesh_override(spec.actor_class_path)  # always False
    snapped_mesh_asset = None  # StaticMesh asset to adopt in snap mode
    snap_target_found = False
    snap_info = None  # snap detail dict (for report output & reverse-mismatch aggregation)
    # Vehicle transplant: snap for pose but swap the mesh to a per-map T4 mesh
    # (maps whose native vehicle meshes lack per-color slots, e.g. NishiShinjuku).
    # Pedestrians never transplant.
    use_transplant = transplant is not None and "Pedestrian" not in spec.actor_class_path
    if snap_to_existing_mesh:
        label_prefixes = _label_prefixes_for_bp_class(spec.actor_class_path, map_name)
        # When transplanting, the spec Z is unreliable (profile pole height) but the
        # mesh Z is adopted via snap, so the Z-validity gate is bypassed (XY only).
        if use_transplant and transplant.get("ignore_snap_z_gate"):
            nearest, dist = _find_nearest_existing_signal_mesh(
                location, snap_radius_cm, label_prefixes,
                max_z_diff_cm=float("inf"), mesh_z_stats=None,
            )
        else:
            nearest, dist = _find_nearest_existing_signal_mesh(
                location, snap_radius_cm, label_prefixes,
                mesh_z_stats=mesh_z_stats,
            )
        if nearest is not None:
            snap_target_found = True
            snapped_loc = nearest.get_actor_location()
            snapped_rot = nearest.get_actor_rotation()
            # Also retrieve the snap target's StaticMesh — if its pivot differs
            # from the BP's fixed Scene_1024, replace it with the corresponding
            # source mesh so positions align correctly
            try:
                target_sm_comp = nearest.get_component_by_class(unreal.StaticMeshComponent)
                if target_sm_comp is not None:
                    snapped_mesh_asset = target_sm_comp.get_editor_property("static_mesh")
            except Exception:
                snapped_mesh_asset = None
            mesh_name = snapped_mesh_asset.get_name() if snapped_mesh_asset is not None else "<n/a>"
            snap_info = {
                "label": nearest.get_actor_label(),
                "xy_dist_cm": dist,
                "mesh_name": mesh_name,
            }
            unreal.log(
                f"_spawn_or_update[snap]: sign_id={spec.sign_id} "
                f"snapped to '{nearest.get_actor_label()}' "
                f"(xy_dist={dist:.1f} cm, mesh={mesh_name})"
            )
            location = snapped_loc
            # Adopt the snap target's world rotation so the lamp mesh aligns
            # with its natural mounting orientation (vehicle + pedestrian).
            rotation = snapped_rot
        else:
            prefix_str = "/".join(p + "_*" for p in label_prefixes)
            unreal.log_warning(
                f"_spawn_or_update[snap]: sign_id={spec.sign_id} "
                f"no {prefix_str} within {snap_radius_cm} cm of target."
            )

    existing = find_actor_by_sign_id(spec.sign_id)
    if existing is not None:
        # Update only position and rotation of existing actor (label and sign_id are preserved)
        existing.set_actor_location(location, sweep=False, teleport=True)
        existing.set_actor_rotation(rotation, teleport_physics=True)
        if snap_to_existing_mesh:
            _zero_out_static_mesh_relative_rotation(existing)
            if use_transplant and snap_target_found:
                _apply_vehicle_transplant(existing, rotation, transplant)
            elif snapped_mesh_asset is not None:
                _override_static_mesh(existing, snapped_mesh_asset)
                if "Pedestrian" in spec.actor_class_path:
                    _clear_static_mesh_material_overrides(existing)
        return existing, False, snap_info

    # Skip snap failures before new spawn
    # (existing actor updates never reach this point, so they are unaffected)
    if snap_to_existing_mesh and skip_when_snap_fails and not snap_target_found:
        return None, False, None

    # New spawn
    bp_cls = _load_bp_class(spec.actor_class_path)
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actor = actor_subsys.spawn_actor_from_class(bp_cls, location, rotation)
    if actor is None:
        raise RuntimeError(
            f"spawn_actor_from_class failed for {spec.actor_class_path} "
            f"(sign_id={spec.sign_id})"
        )

    # In snap mode, the Relative Rotation baked into the BP's StaticMeshComponent
    # (e.g. Roll=-90, Pitch=85.4) would be double-applied on top of the snap World
    # Rotation, so it is zeroed out here.  Additionally, each Traffic_Lights_* uses
    # an individual Scene_NNNN asset, so the mesh is also replaced with the snap
    # target's mesh to align pivot positions.
    if snap_to_existing_mesh:
        _zero_out_static_mesh_relative_rotation(actor)
        if use_transplant and snap_target_found:
            _apply_vehicle_transplant(actor, rotation, transplant)
        elif snapped_mesh_asset is not None:
            _override_static_mesh(actor, snapped_mesh_asset)
            if "Pedestrian" in spec.actor_class_path:
                _clear_static_mesh_material_overrides(actor)

    # Write sign_id to TrafficLightComponent (so it can be found later by find_actor_by_sign_id)
    tlc = actor.get_traffic_light_component()
    if tlc is not None:
        tlc.set_sign_id(spec.sign_id)
    else:
        unreal.log_warning(
            f"_spawn_or_update: actor {spec.actor_class_path} has no TrafficLightComponent. "
            f"sign_id={spec.sign_id} will not be persisted on the component."
        )

    # Assign a human-readable label in the editor (subtype-specific prefix: TLV_/TLP_/TL_)
    prefix = _label_prefix_for_subtype(spec.subtype)
    actor.set_actor_label(f"{prefix}{spec.sign_id}")

    return actor, True, snap_info


def _override_static_mesh(actor, new_mesh) -> None:
    """Replace the StaticMesh on every StaticMeshComponent under the actor.

    In snap mode, this overwrites the BP default mesh (fixed Scene_1024) that
    is present right after spawning with the mesh used by the snap target's
    Traffic_Lights_* (which varies per Scene_NNNN), eliminating subtle position
    offsets caused by pivot differences.
    """
    if new_mesh is None:
        return
    try:
        comps = actor.get_components_by_class(unreal.StaticMeshComponent)
    except Exception:
        return
    for comp in comps:
        try:
            comp.set_static_mesh(new_mesh)
        except Exception as e:
            unreal.log_warning(
                f"_override_static_mesh: set_static_mesh failed on "
                f"'{actor.get_actor_label()}': {e}"
            )


def _clear_static_mesh_material_overrides(actor) -> None:
    """Remove per-component material overrides so the snapped mesh's own default
    materials are used.

    Pedestrian SceneFigure BPs are duplicated from the canonical
    BP_PedestrianTrafficLight, whose StaticMeshComponent carries leftover
    canonical material overrides (Walk/Frame/Stop, slot 0-2). Snap mesh-override
    swaps the mesh to Scene_NNN but the component's index-based overrides remain
    and mask the Scene_NNN Overlapped_Green/Body materials — which breaks the
    BP's "find the Overlapped_Green element" lookup. Clearing the overrides
    restores the Scene_NNN defaults so the lookup (and figure override) work.
    """
    try:
        comps = actor.get_components_by_class(unreal.StaticMeshComponent)
    except Exception:
        return
    for comp in comps:
        try:
            comp.empty_override_materials()
        except Exception:
            # Fallback: reset each slot to the mesh's own default material.
            try:
                sm = comp.get_editor_property("static_mesh")
                for i in range(comp.get_num_materials()):
                    comp.set_material(i, sm.get_material(i) if sm is not None else None)
            except Exception:
                pass


def _apply_vehicle_transplant(actor, snapped_rotation, transplant: dict) -> None:
    """Replace the snapped vehicle mesh with the per-map transplant mesh and
    apply the de-risked pose/position corrections.

    Used for maps (e.g. NishiShinjuku) whose native vehicle meshes lack per-color
    material slots. The actor snaps to the native mesh for pose, but its StaticMesh
    is swapped to a T4 mesh that *does* have Green/Yellow/Red slots so the existing
    BP_VehicleTrafficLight lighting works.

    Inc1 de-risk findings baked in here:
      - The mesh-orientation offset is composed into the ACTOR world rotation, not
        the component relative rotation: set_static_mesh / empty_override_materials
        trigger a BP construction-script rerun that resets component relative
        rotation to the BP default, so a component offset would be wiped.
      - World Z offset corrects the pivot height difference; a local offset (mesh
        frame) corrects depth/lateral pivot difference.
      - Material overrides are cleared so the transplant mesh's own Green/Yellow/Red
        materials drive the lighting (the canonical BP leaves index-based overrides).
    """
    mesh = unreal.load_asset(transplant["mesh"])
    if mesh is not None:
        _override_static_mesh(actor, mesh)
        _clear_static_mesh_material_overrides(actor)
        scale = transplant.get("scale", 1.0)
        if scale != 1.0:
            for c in actor.get_components_by_class(unreal.StaticMeshComponent):
                try:
                    c.set_relative_scale3d(unreal.Vector(scale, scale, scale))
                except Exception:
                    pass
    # Pose: bake the mesh-convention offset into the ACTOR world rotation.
    p, y, r = transplant.get("relrot_deg", (0.0, 0.0, 0.0))
    offset = unreal.Rotator(roll=r, pitch=p, yaw=y)
    actor.set_actor_rotation(
        unreal.MathLibrary.compose_rotators(offset, snapped_rotation),
        teleport_physics=True,
    )
    # Depth/lateral pivot correction in the mesh's local frame.
    lx, ly, lz = transplant.get("local_offset_cm", (0.0, 0.0, 0.0))
    if (lx, ly, lz) != (0.0, 0.0, 0.0):
        try:
            actor.add_actor_local_offset(unreal.Vector(lx, ly, lz), False, True)
        except Exception:
            pass
    # Height pivot correction in world Z (component is rotated, so local Z is unusable).
    wz = transplant.get("world_z_offset_cm", 0.0)
    if wz != 0.0:
        loc = actor.get_actor_location()
        actor.set_actor_location(
            unreal.Vector(loc.x, loc.y, loc.z + wz), False, True)


def _zero_out_static_mesh_relative_rotation(actor) -> None:
    """Reset the Relative Rotation of StaticMeshComponents inside a freshly spawned Actor to (0, 0, 0).

    In snap mode the Actor's World Rotation is snapped to an existing mesh, so
    double-application of the Component Relative Rotation baked into the BP
    must be avoided.
    """
    try:
        comps = actor.get_components_by_class(unreal.StaticMeshComponent)
    except Exception:
        return
    for comp in comps:
        try:
            comp.set_relative_rotation(unreal.Rotator(0.0, 0.0, 0.0), sweep=False, teleport=True)
        except TypeError:
            # Guard against overload signature mismatch
            try:
                comp.set_relative_rotation(unreal.Rotator(0.0, 0.0, 0.0))
            except Exception:
                pass
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 3.4: group binding
# ---------------------------------------------------------------------------

def _place_groups(groups: list, sign_id_to_actor: dict,
                  sign_id_to_subtype: dict,
                  group_bp_path: Optional[str] = None) -> tuple:
    """Place an ATrafficLightGroup actor for each GroupSpec and bind 2 Controllers
    per subtype (Vehicle/Pedestrian) as defined in Phase 6.

    Phase 5-2's 1 Group = 1 Controller design caused `set_state` to sync across
    all TLs when subtypes were mixed.  Phase 6 changed the design to split
    member_actors by subtype and call add_controller twice per Group.
    `set_state` is written directly to the Component, enabling independent
    per-subtype control.

    Args:
        groups: Groups list from corelib.api.generate_placements().
        sign_id_to_actor: Map of sign_id → Actor for already-placed TLs.
        sign_id_to_subtype: Map of sign_id → subtype string (red_yellow_green / red_green).
            Built in place_from_specs as {p.sign_id: p.subtype for p in placements}.
        group_bp_path: Asset Path of the derived Group BP (obtained from
            bp_factory.ensure_group_bp). If None, C++ ATrafficLightGroup is spawned directly.

    Returns:
        (created_count, updated_count)
    """
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype

    # Determine the class for Group spawn: derived BP > direct C++ subclass
    group_cls = None
    if group_bp_path:
        try:
            group_cls = _load_bp_class(group_bp_path)
        except RuntimeError as e:
            unreal.log_warning(
                f"_place_groups: Group BP load failed at {group_bp_path}: {e}. "
                f"Falling back to C++ ATrafficLightGroup."
            )
    if group_cls is None:
        group_cls = unreal.TrafficLightGroup

    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    # Label map of existing TLGroup_* actors (for idempotent re-runs)
    existing_labels: dict = {
        a.get_actor_label(): a
        for a in actor_subsys.get_all_level_actors()
        if a.get_actor_label().startswith("TLGroup_")
    }

    created = 0
    updated = 0

    for g in groups:
        # Split member_actors by subtype
        members_by_subtype = split_members_by_subtype(
            refers=g.refers,
            sign_id_to_actor=sign_id_to_actor,
            sign_id_to_subtype=sign_id_to_subtype,
        )
        # Also consider level-search fallback via find_actor_by_sign_id
        # (for cases not in sign_id_to_actor but present in the level)
        for way_id in g.refers:
            sid = str(way_id)
            if sid in sign_id_to_actor:
                continue
            actor = find_actor_by_sign_id(sid)
            if actor is not None:
                subtype = sign_id_to_subtype.get(sid, "")
                members_by_subtype.setdefault(subtype, []).append(actor)

        if not members_by_subtype:
            # Skip groups where all members were not placed (e.g. snap_skipped)
            continue

        label = f"TLGroup_{g.relation_id}"
        if label in existing_labels:
            group_actor = existing_labels[label]
            # Clear existing Controllers and redo (for idempotency)
            try:
                group_actor.set_editor_property("controllers", [])
            except Exception:
                pass
            updated += 1
        else:
            # Place the group origin at the first member of the first subtype encountered
            first_actor = next(iter(members_by_subtype.values()))[0]
            group_actor = actor_subsys.spawn_actor_from_class(
                group_cls,
                first_actor.get_actor_location(),
                unreal.Rotator(0, 0, 0),
            )
            if group_actor is None:
                unreal.log_warning(
                    f"_place_groups: Failed to spawn group actor for label={label}. Skipping."
                )
                continue
            group_actor.set_actor_label(label)
            created += 1

        # Reuse lanelet2 relation_id as JunctionId
        try:
            group_actor.set_editor_property("junction_id", int(g.relation_id))
        except Exception:
            pass

        # Create one Controller per subtype
        for subtype, members in members_by_subtype.items():
            try:
                controller = unreal.new_object(
                    unreal.TrafficLightController, outer=group_actor
                )
            except Exception as e:
                unreal.log_warning(
                    f"_place_groups: failed to create UTrafficLightController for "
                    f"group {g.relation_id} subtype={subtype}: {e}. Skipping."
                )
                continue
            controller_id = f"{g.relation_id}_{subtype}" if subtype else str(g.relation_id)
            try:
                controller.set_controller_id(controller_id)
            except Exception:
                pass
            try:
                controller.set_red_time(10.0)
                controller.set_yellow_time(2.0)
                controller.set_green_time(10.0)
            except Exception:
                pass

            try:
                group_actor.add_controller(controller)
            except Exception as e:
                unreal.log_warning(
                    f"_place_groups: group_actor.add_controller failed for "
                    f"group {g.relation_id} subtype={subtype}: {e}. Skipping subtype."
                )
                continue

            for actor in members:
                try:
                    tlc = actor.get_traffic_light_component()
                except Exception:
                    tlc = None
                if tlc is None:
                    continue
                try:
                    controller.add_traffic_light(tlc)
                except Exception as e:
                    unreal.log_warning(
                        f"_place_groups: add_traffic_light failed for {actor.get_actor_label()} "
                        f"in group {g.relation_id} subtype={subtype}: {e}"
                    )

    return created, updated


# ---------------------------------------------------------------------------
# Report file output (added in Phase 4.2)
# ---------------------------------------------------------------------------

def _write_full_run_report(report: "PlacementReport", path: str,
                           n_input_placements: int,
                           metadata: Optional[dict] = None) -> None:
    """Write the full run report to a text file after place_from_specs completes.

    Outputs the following sections completely (multiple sections in one file):
      0. METADATA: full paths of the input osm file, CarlaUE5 directory, etc.
      1. SUMMARY: aggregated counts
      2. Z STATS: per-prefix Z statistics (Tukey range)
      3. PLACED: sign_id, snap target, xy_dist, mesh for successfully placed specs
      4. SNAP_SKIPPED: lanelet2 way exists but skipped because outside Odaiba cityscape
      5. FAILED: placement failures due to exceptions
      6. UNUSED_MESHES: Odaiba mesh exists but no lanelet2 way matched (reverse mismatch)

    Output is overwritten. Expected to be written outside git control (e.g. directly under T4Fork.odaiba).

    Args:
        metadata: Arbitrary key/value pairs written at the top of the report.
            Passing full paths for `osm_path`, `carla_ue5_dir`, etc. aids later tracing.
    """
    import datetime
    import os
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("# Lanelet2 Traffic Light Full Run Report\n")
        f.write(f"# generated: {datetime.datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# input placements (lanelet2 parsed): {n_input_placements}\n")
        f.write(f"# report file path: {os.path.abspath(path)}\n")
        f.write("\n")

        # 0. METADATA (full paths of input files / directories)
        if metadata:
            f.write("## METADATA\n")
            for k, v in metadata.items():
                f.write(f"{k:24s}: {v}\n")
            f.write("\n")

        # 1. SUMMARY
        f.write("## SUMMARY\n")
        f.write(f"created          : {report.created}\n")
        f.write(f"updated          : {report.updated}\n")
        f.write(f"snap_skipped     : {len(report.snap_skipped)}\n")
        f.write(f"failed           : {len(report.failed)}\n")
        f.write(f"unused_meshes    : {len(report.unused_existing_meshes)}\n")
        f.write(f"groups_created   : {report.groups_created}\n")
        f.write(f"groups_updated   : {report.groups_updated}\n")
        f.write(f"placed           : {report.created + report.updated}\n")
        f.write(f"pedestrian_mesh_materials: {len(report.pedestrian_material_stats)}\n")
        f.write("\n")

        # 2. Z STATS
        f.write("## Z STATS (existing meshes, by prefix)\n")
        if report.z_stats_snapshot:
            for prefix, st in report.z_stats_snapshot.items():
                f.write(
                    f"{prefix:24s} n={st['n']:4d} "
                    f"min={st['z_min']:8.1f} q25={st['z_q25']:8.1f} "
                    f"median={st['z_median']:8.1f} q75={st['z_q75']:8.1f} "
                    f"max={st['z_max']:8.1f} "
                    f"tukey=[{st['tukey_low']:8.1f}, {st['tukey_high']:8.1f}]\n"
                )
        else:
            f.write("(snap_to_existing_mesh disabled, no stats collected)\n")
        f.write("\n")

        # 3. PLACED
        f.write(f"## PLACED ({len(report.placed_records)} entries)\n")
        f.write("\n# placed\n")
        for rec in report.placed_records:
            wx, wy, wz = rec.get("world_xyz", (0.0, 0.0, 0.0))
            parts = [
                f"sign_id={rec['sign_id']}",
                f"actor={rec.get('actor_label', '')}",
                f"subtype={rec.get('subtype', '')}",
                f"bp={rec.get('bp_class', '')}",
                f"snap={'true' if rec.get('target_label') else 'false'}",
            ]
            if rec.get("target_label"):
                parts.append(f"target={rec['target_label']}")
            if rec.get("mesh_name"):
                parts.append(f"mesh={rec['mesh_name']}")
            if rec.get("xy_dist_cm") is not None:
                parts.append(f"xy_dist_cm={rec['xy_dist_cm']:.1f}")
            parts += [
                f"world_x={wx:.1f}", f"world_y={wy:.1f}", f"world_z={wz:.1f}",
            ]
            if rec.get("lat") is not None:
                parts.append(f"lat={rec['lat']:.6f}")
            if rec.get("lon") is not None:
                parts.append(f"lon={rec['lon']:.6f}")
            if rec.get("ele") is not None:
                parts.append(f"ele={rec['ele']:.3f}")
            if rec.get("local_x") is not None:
                parts.append(f"local_x={rec['local_x']:.2f}")
            if rec.get("local_y") is not None:
                parts.append(f"local_y={rec['local_y']:.2f}")
            if rec.get("mgrs_code"):
                parts.append(f"mgrs={rec['mgrs_code']}")
            f.write(" ".join(parts) + "\n")

        # 4. SNAP_SKIPPED
        f.write(f"\n## SNAP_SKIPPED ({len(report.snap_skipped)} entries)\n")
        f.write("# lanelet2 way exists but no Odaiba mesh within snap_radius\n")
        f.write("\n# snap_skipped\n")
        for rec in getattr(report, "snap_skipped_records", []):
            parts = [
                f"sign_id={rec['sign_id']}",
                f"subtype={rec.get('subtype', '')}",
                f"reason={rec.get('reason', 'no_snap_target')}",
                f"target_x={rec['target_x']:.1f}",
                f"target_y={rec['target_y']:.1f}",
                f"target_z={rec['target_z']:.1f}",
            ]
            if rec.get("lat") is not None:
                parts.append(f"lat={rec['lat']:.6f}")
            if rec.get("lon") is not None:
                parts.append(f"lon={rec['lon']:.6f}")
            if rec.get("ele") is not None:
                parts.append(f"ele={rec['ele']:.3f}")
            if rec.get("local_x") is not None:
                parts.append(f"local_x={rec['local_x']:.2f}")
            if rec.get("local_y") is not None:
                parts.append(f"local_y={rec['local_y']:.2f}")
            if rec.get("mgrs_code"):
                parts.append(f"mgrs={rec['mgrs_code']}")
            f.write(" ".join(parts) + "\n")

        # 5. FAILED
        f.write(f"\n## FAILED ({len(report.failed)} entries)\n")
        f.write("\n# failed\n")
        for sign_id, error in report.failed:
            err_escaped = error.replace('"', '\\"').replace('\n', '\\n')
            f.write(f'sign_id={sign_id} error="{err_escaped}"\n')

        # 6. UNUSED_MESHES
        f.write(f"\n## UNUSED_MESHES ({len(report.unused_existing_meshes)} entries)\n")
        f.write("# Odaiba mesh exists but no lanelet2 way matched (reverse mismatch)\n")
        f.write("\n# unused_meshes\n")
        for label, loc in report.unused_existing_meshes:
            wx, wy, wz = loc[0], loc[1], loc[2]
            parts = [
                f"label={label}",
                f"world_x={wx:.1f}",
                f"world_y={wy:.1f}",
                f"world_z={wz:.1f}",
            ]
            f.write(" ".join(parts) + "\n")
        f.write("\n")

        # 7. PEDESTRIAN_MESH_MATERIALS
        if report.pedestrian_material_stats:
            f.write("\n# pedestrian_mesh_materials\n")
            for row in report.pedestrian_material_stats:
                f.write(
                    f"mesh={row['mesh']} num_elements={row['num_elements']} "
                    f"elements=\"{','.join(row['elements'])}\" count={row['count']}\n"
                )
            # summary line
            total = sum(r["count"] for r in report.pedestrian_material_stats)
            unique = len(report.pedestrian_material_stats)
            n3 = sum(1 for r in report.pedestrian_material_stats if r["num_elements"] == 3)
            n2 = sum(1 for r in report.pedestrian_material_stats if r["num_elements"] == 2)
            f.write("\n# pedestrian_mesh_summary\n")
            f.write(
                f"total_actors={total} unique_meshes={unique} "
                f"meshes_with_3_elements={n3} meshes_with_2_elements={n2}\n"
            )


# ---------------------------------------------------------------------------
# main entry: place_from_specs
# ---------------------------------------------------------------------------

def place_from_specs(
    placements: list,
    groups: list,
    save_level: bool = False,
    snap_to_existing_mesh: bool = False,
    snap_radius_cm: float = 300.0,
    skip_when_snap_fails: bool = True,
    report_path: Optional[str] = None,
    report_metadata: Optional[dict] = None,
    group_bp_path: Optional[str] = None,
    map_name: str = None,
) -> PlacementReport:
    """Idempotently place the list of PlacementSpecs into the level.

    Expects the output of core.api.generate_placements() to be passed directly.
    All operations are wrapped in a single ScopedEditorTransaction so that
    failures can be undone in bulk with Ctrl+Z.

    Args:
        placements: Return value [0] of core.api.generate_placements() (list of PlacementSpec)
        groups:     Return value [1] of core.api.generate_placements() (list of GroupSpec)
        save_level: If True, saves the current level to disk after placement.
                    Default is False (manual Ctrl+S recommended).
        snap_to_existing_mesh: If True, adopts the position and rotation of the
                    corresponding existing mesh (`Traffic_Lights_*` for vehicles,
                    `Pedestrian_Lights_*` for pedestrians) before each spawn
                    (Odaiba-specific fine-tuning mode).
                    Absorbs yaw drift of ~3° and Z drift of a few cm.
                    Only meshes within `snap_radius_cm` are adopted.
        snap_radius_cm: Maximum distance for snap matching (cm).
        skip_when_snap_fails: If snap_to_existing_mesh=True and no snap target is
                    found, skip the new spawn. Prevents signals outside the 3D
                    cityscape coverage of Odaiba.umap from appearing in mid-air.
                    Skipped specs are aggregated in report.snap_skipped.

    Returns:
        PlacementReport: Summary of placement results.
                         report.failed holds a list of (sign_id, reason) tuples for failures.
                         report.snap_skipped holds a list of (sign_id, location_cm)
                         for specs skipped due to snap failure.
    """
    report = PlacementReport()

    with unreal.ScopedEditorTransaction("Generate Traffic Lights from lanelet2"):
        sign_id_to_actor: dict = {}

        # Phase 4.2: collect Z statistics for snap target meshes once.
        # Passed to _find_nearest_existing_signal_mesh for per-prefix Tukey range
        # Z validity checks (map-independent). Falls back to static threshold if empty.
        mesh_z_stats = None
        if snap_to_existing_mesh:
            mesh_z_stats = _collect_mesh_z_stats()
            for prefix, st in mesh_z_stats.items():
                unreal.log(
                    f"place_from_specs[Z stats]: {prefix} n={st.n} "
                    f"min={st.z_min:.1f} q25={st.z_q25:.1f} "
                    f"median={st.z_median:.1f} q75={st.z_q75:.1f} "
                    f"max={st.z_max:.1f} | tukey=[{st.tukey_low:.1f}, {st.tukey_high:.1f}]"
                )
            # Store snapshot in report (for report output)
            report.z_stats_snapshot = {
                prefix: {
                    "n": st.n,
                    "z_min": st.z_min, "z_max": st.z_max,
                    "z_median": st.z_median,
                    "z_q25": st.z_q25, "z_q75": st.z_q75,
                    "tukey_low": st.tukey_low, "tukey_high": st.tukey_high,
                }
                for prefix, st in mesh_z_stats.items()
            }

        # Phase 6 issue D: aggregate Material composition of pedestrian meshes
        try:
            report.pedestrian_material_stats = _collect_pedestrian_mesh_materials()
            for row in report.pedestrian_material_stats:
                unreal.log(
                    f"pedestrian_mesh: mesh={row['mesh']} num_elements={row['num_elements']} "
                    f"elements=\"{','.join(row['elements'])}\" count={row['count']}"
                )
        except Exception as e:
            unreal.log_warning(
                f"_collect_pedestrian_mesh_materials failed: {e}. continuing without stats."
            )
            report.pedestrian_material_stats = []

        # Set of existing mesh labels adopted in snap mode (for reverse-mismatch detection)
        used_mesh_labels: set = set()

        # Per-map vehicle transplant config (None for maps that adopt the native mesh).
        transplant = get_map_profile(map_name).vehicle_transplant

        for spec in placements:
            try:
                actor, was_created, snap_info = _spawn_or_update(
                    spec,
                    snap_to_existing_mesh=snap_to_existing_mesh,
                    snap_radius_cm=snap_radius_cm,
                    skip_when_snap_fails=skip_when_snap_fails,
                    mesh_z_stats=mesh_z_stats,
                    map_name=map_name,
                    transplant=transplant,
                )
                if snap_info is not None:
                    used_mesh_labels.add(snap_info["label"])
                if actor is None:
                    # Skipped due to snap failure (new spawns only; existing updates always return actor)
                    report.snap_skipped.append((spec.sign_id, spec.location_cm))
                    report.snap_skipped_records.append({
                        "sign_id": spec.sign_id,
                        "subtype": spec.subtype,
                        "reason": "no_snap_target",
                        "target_x": spec.location_cm[0],
                        "target_y": spec.location_cm[1],
                        "target_z": spec.location_cm[2],
                        "lat": spec.lat,
                        "lon": spec.lon,
                        "ele": spec.ele,
                        "local_x": spec.local_x,
                        "local_y": spec.local_y,
                        "mgrs_code": spec.mgrs_code,
                    })
                    continue
                sign_id_to_actor[spec.sign_id] = actor
                # Placement record (for report output)
                rec = {
                    "sign_id": spec.sign_id,
                    "actor_label": actor.get_actor_label(),
                    "subtype": spec.subtype,
                    "was_created": was_created,
                    "bp_class": spec.actor_class_path.split("/")[-1].split(".")[0],
                    "target_label": (snap_info["label"] if snap_info else None),
                    "xy_dist_cm": (snap_info["xy_dist_cm"] if snap_info else None),
                    "mesh_name": (snap_info["mesh_name"] if snap_info else None),
                    "world_xyz": (
                        actor.get_actor_location().x,
                        actor.get_actor_location().y,
                        actor.get_actor_location().z,
                    ),
                    "lat": spec.lat,
                    "lon": spec.lon,
                    "ele": spec.ele,
                    "local_x": spec.local_x,
                    "local_y": spec.local_y,
                    "mgrs_code": spec.mgrs_code,
                }
                report.placed_records.append(rec)
                if was_created:
                    report.created += 1
                else:
                    report.updated += 1
            except Exception as e:
                report.failed.append((spec.sign_id, str(e)))
                unreal.log_error(
                    f"place_from_specs: sign_id={spec.sign_id} failed: {e}"
                )

        # Reverse mismatch: report signal meshes that exist as Odaiba meshes but
        # were not matched to any lanelet2 way (added in Phase 4.2).
        if snap_to_existing_mesh:
            report.unused_existing_meshes = _collect_unused_meshes(used_mesh_labels)
            unreal.log(
                f"editor_placer[unused]: existing meshes not snapped by any "
                f"lanelet2 way = {len(report.unused_existing_meshes)}"
            )
            for label, (x, y, z) in report.unused_existing_meshes[:10]:
                unreal.log(
                    f"  unused: {label} at ({x:.1f}, {y:.1f}, {z:.1f})"
                )
            if len(report.unused_existing_meshes) > 10:
                unreal.log(
                    f"  ... and {len(report.unused_existing_meshes) - 10} more"
                )

        # Group placement (must run after individual actor placement)
        # Phase 6: build sign_id → subtype map for per-subtype Controllers
        sign_id_to_subtype = {p.sign_id: p.subtype for p in placements}
        gc, gu = _place_groups(
            groups,
            sign_id_to_actor,
            sign_id_to_subtype,
            group_bp_path=group_bp_path,
        )
        report.groups_created = gc
        report.groups_updated = gu

        unreal.log(
            f"editor_placer: TL created={report.created} updated={report.updated} "
            f"snap_skipped={len(report.snap_skipped)} failed={len(report.failed)} "
            f"unused_meshes={len(report.unused_existing_meshes)} | "
            f"groups created={gc} updated={gu}"
        )

    if save_level:
        # Note: saving does not flush the Undo stack, but to require explicit
        # user approval, save_level=True is only executed when passed explicitly.
        unreal.EditorLevelLibrary.save_current_level()

    # Report file output (added in Phase 4.2): complete, no omissions
    if report_path:
        try:
            _write_full_run_report(
                report, report_path,
                n_input_placements=len(placements),
                metadata=report_metadata,
            )
            unreal.log(f"[lanelet2_tl] wrote {report_path}")
        except Exception as e:
            unreal.log_error(f"[lanelet2_tl] failed to write report {report_path}: {e}")
    else:
        unreal.log("[lanelet2_tl] report_path not set; skipping report file output")

    return report

"""Green-arrow lighting via material override.

Inc 0 de-risk (2026-06-02) established that the map lamp materials carry no
parameters (so an emissive scalar cannot be raised on them) and that
``Actor.rerun_construction_scripts`` is unavailable in this build. Arrows are
therefore lit by overriding each active green-arrow mesh slot with a dynamic
instance of the T4 material ``M_JPArrowLit`` (arrow texture masked + black-level
subtract -> green emissive). Map ``.uasset`` files are never edited; the override
is applied to the placed BP actor's component.

The pure part (direction maps + accessors) is unit-tested. The editor functions
import ``unreal`` lazily so the module imports without the engine.
"""

# T4 emissive material applied to arrow slots (built by arrow_material_setup.py).
ARROW_LIT_MATERIAL = (
    "/T4/TrafficLightSample/VehicleTrafficLight/M_JPArrowLit.M_JPArrowLit"
)

# Direction -> mesh material slot name (Odaiba `_Modified` / SM_JPVehicleLampArrow).
ARROW_SLOT_BY_DIR = {
    "left": "Green_Left_Arrow",
    "straight": "Green_Straight_Arrow",
    "right": "Green_Right_Arrow",
}

# Direction -> T4 arrow texture. Odaiba's arrow mesh encodes DIRECTION via the
# slot orientation (left slot is rotated to show a left arrow, right slot a right
# arrow), so every slot uses the same UP-pointing 2-dot texture; the mesh makes
# it left/up/right. (Per-direction figure textures like T_JPArrow2_Left would be
# rotated twice and come out wrong.) NishiShinjuku uses its own per-map config
# (_NISHISHINJUKU_VEHICLE_ARROW_NATIVE) so it is unaffected.
ARROW_TEX_BY_DIR = {
    "left": "/T4/TrafficLightSample/VehicleTrafficLight/T_JPArrow2_Straight.T_JPArrow2_Straight",
    "straight": "/T4/TrafficLightSample/VehicleTrafficLight/T_JPArrow2_Straight.T_JPArrow2_Straight",
    "right": "/T4/TrafficLightSample/VehicleTrafficLight/T_JPArrow2_Straight.T_JPArrow2_Straight",
}

# Default ON value for the material's Intensity scalar (PIE-tuned; the material
# asset carries the same default).
ARROW_INTENSITY_ON = 30.0


def arrow_slot_for_dir(direction):
    """Mesh material slot name for a green-arrow direction, or None if unknown."""
    return ARROW_SLOT_BY_DIR.get(direction)


def arrow_texture_for_dir(direction):
    """T4 arrow texture path for a green-arrow direction, or None if unknown."""
    return ARROW_TEX_BY_DIR.get(direction)


def resolve_arrow_config(config):
    """Merge a per-map arrow lighting config with the module defaults (Odaiba).

    config keys (all optional): slot_by_dir, tex_by_dir, black_level, white_level.
    black/white_level None means "keep the material defaults" (the M_JPArrowLit asset
    defaults fit the Odaiba texture family; e.g. NishiShinjuku needs 0.10/0.40).
    """
    config = config or {}
    return {
        "slot_by_dir": config.get("slot_by_dir", ARROW_SLOT_BY_DIR),
        "tex_by_dir": config.get("tex_by_dir", ARROW_TEX_BY_DIR),
        "black_level": config.get("black_level"),
        "white_level": config.get("white_level"),
    }


def arrow_slots_present(slot_names, cfg):
    """True when the mesh carries ANY per-direction arrow slot of the (resolved)
    config. Used to prefer arrow-CAPABLE snap targets for arrow-bearing signals:
    the nearest native actor may be a plain 3-light head (or an odd compact unit)
    whose mesh cannot light any arrow (pure).
    """
    return any(slot in slot_names for slot in cfg["slot_by_dir"].values())


def _slot_names(component):
    """Material slot names of the component's static mesh (editor-only)."""
    sm = component.get_editor_property("static_mesh")
    if sm is None:
        return []
    return [str(e.get_editor_property("material_slot_name"))
            for e in sm.get_editor_property("static_materials")]


def apply_arrow_lighting(actor, green_dirs, config=None):
    """Override each active green-arrow slot with a DMI of M_JPArrowLit.

    For every direction in ``green_dirs`` whose slot exists on the actor's static
    mesh, create a dynamic instance of M_JPArrowLit, set its ``ArrowTex`` to the
    per-direction texture, and assign it to that slot. Slots absent on the mesh
    are skipped (no green arrow there). ``config`` is the per-map override from
    MapProfile.vehicle_arrow_native (slot/texture maps + black/white levels);
    None uses the Odaiba defaults. Editor-only. Returns the number of slots lit.
    """
    import unreal

    cfg = resolve_arrow_config(config)
    component = actor.get_component_by_class(unreal.StaticMeshComponent)
    if component is None:
        return 0
    base_mat = unreal.load_asset(ARROW_LIT_MATERIAL)
    if base_mat is None:
        unreal.log_warning(f"apply_arrow_lighting: material not found {ARROW_LIT_MATERIAL}")
        return 0
    names = _slot_names(component)
    lit = 0
    for direction in green_dirs:
        slot = cfg["slot_by_dir"].get(direction)
        if slot is None or slot not in names:
            continue
        idx = names.index(slot)
        mid = component.create_dynamic_material_instance(idx, base_mat)
        if mid is None:
            unreal.log_warning(f"apply_arrow_lighting: DMI creation failed at slot {slot}")
            continue
        tex_path = cfg["tex_by_dir"].get(direction)
        tex = unreal.load_asset(tex_path) if tex_path else None
        if tex is not None:
            mid.set_texture_parameter_value("ArrowTex", tex)
        if cfg["black_level"] is not None:
            mid.set_scalar_parameter_value("BlackLevel", cfg["black_level"])
        if cfg["white_level"] is not None:
            mid.set_scalar_parameter_value("WhiteLevel", cfg["white_level"])
        lit += 1
    return lit


def arrow_slot_indices(actor, config=None):
    """Per-direction material slot index of the actor's CURRENT static mesh
    ({direction: index}, directions without a face omitted). Editor-only.
    Used by the placer to stamp runtime arrow properties (BP slot vars and
    the capabilities mask)."""
    import unreal

    cfg = resolve_arrow_config(config)
    component = actor.get_component_by_class(unreal.StaticMeshComponent)
    if component is None:
        return {}
    names = _slot_names(component)
    return {d: names.index(slot)
            for d, slot in cfg["slot_by_dir"].items() if slot in names}


def set_arrow_intensity_on_actor(actor, enabled):
    """Toggle the arrows on one actor by setting Intensity on every applied
    M_JPArrowLit DMI (0 = off, ARROW_INTENSITY_ON = on). Editor-only.
    Returns the number of arrow DMIs touched.
    """
    import unreal

    component = actor.get_component_by_class(unreal.StaticMeshComponent)
    if component is None:
        return 0
    value = ARROW_INTENSITY_ON if enabled else 0.0
    touched = 0
    for idx in range(component.get_num_materials()):
        mat = component.get_material(idx)
        if not isinstance(mat, unreal.MaterialInstanceDynamic):
            continue
        base = mat.get_base_material()
        if base is None or base.get_name() != "M_JPArrowLit":
            continue
        mat.set_scalar_parameter_value("Intensity", value)
        touched += 1
    return touched

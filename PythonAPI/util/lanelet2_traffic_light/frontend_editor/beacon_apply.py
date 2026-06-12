"""One-shot apply: light up the Odaiba flashing beacons.

Per beacon actor (Orange_Light slot), measures the local-Y lamp gap of the
lens section, buckets the gap midpoint into a per-family material instance
(MI_JPFlashingBeacon_* with the YSplit override), assigns it to the slot,
and finally ensures exactly one BP_FlashingBeaconManager exists in the level.

Idempotent: a second run finds no Orange_Light slots left (they now hold
MI_JPFlashingBeacon_*) and skips the manager spawn when one is present.
Does NOT save the level -- review in PIE first, then save manually (Ctrl+S)
to persist (same discipline as Delete Native).

Run from the editor Output Log:
    py "<repo>/PythonAPI/util/lanelet2_traffic_light/frontend_editor/beacon_apply.py"
"""
import importlib
import os
import sys

import unreal

_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _PKG_ROOT not in sys.path:
    sys.path.append(_PKG_ROOT)

import lanelet2_traffic_light.frontend_editor.beacon_select as _beacon_select  # noqa: E402

_beacon_select = importlib.reload(_beacon_select)
beacon_element_indices = _beacon_select.beacon_element_indices
largest_gap = _beacon_select.largest_gap

NATIVE_MARKER = "Orange_Light"
APPLIED_MARKER = "JPFlashingBeacon"
# Runtime selection handle for BP_FlashingBeaconManager (string matching on
# material names proved too fragile to wire reliably in BP, 2026-06-12).
ACTOR_TAG = "FlashingBeacon"
MATERIAL_DIR = "/T4/TrafficLightSample/FlashingBeacon"
BASE_MATERIAL = "%s/M_JPFlashingBeaconLit" % MATERIAL_DIR
MANAGER_BP = "%s/BP_FlashingBeaconManager" % MATERIAL_DIR
MANAGER_LABEL = "FlashingBeaconManager"

mel = unreal.MaterialEditingLibrary


def _material_names(comp):
    return [comp.get_material(i).get_name() if comp.get_material(i) else None
            for i in range(comp.get_num_materials())]


def _section_y_split(mesh, section_index):
    """Midpoint of the largest local-Y gap of a section (the lamp boundary)."""
    try:
        verts = unreal.ProceduralMeshLibrary.get_section_from_static_mesh(
            mesh, 0, section_index)[0]
    except Exception as e:
        unreal.log_warning("[beacon_apply] section %d read failed: %s"
                           % (section_index, e))
        return None
    if not verts:
        return None
    return largest_gap([v.y for v in verts])[1]


def _mic_name(split):
    # +2.808 -> MI_JPFlashingBeacon_Yp028; -2.808 -> ..._Yn028 (0.1 cm buckets)
    return "MI_JPFlashingBeacon_Y%s%03d" % (
        "p" if split >= 0 else "n", int(round(abs(split) * 10)))


def _mic_for_split(split, base, cache):
    name = _mic_name(split)
    if name in cache:
        return cache[name]
    path = "%s/%s" % (MATERIAL_DIR, name)
    mic = unreal.load_asset(path)
    if mic is None:
        at = unreal.AssetToolsHelpers.get_asset_tools()
        mic = at.create_asset(name, MATERIAL_DIR,
                              unreal.MaterialInstanceConstant,
                              unreal.MaterialInstanceConstantFactoryNew())
    if mic is None:
        unreal.log_error("[beacon_apply] could not create %s" % path)
        return None
    mel.set_material_instance_parent(mic, base)
    mel.set_material_instance_scalar_parameter_value(mic, "YSplit", split)
    mel.update_material_instance(mic)
    unreal.EditorAssetLibrary.save_loaded_asset(mic)
    cache[name] = mic
    return mic


def _ensure_tag(actor):
    """Add ACTOR_TAG to the actor (idempotent). Returns 1 when added."""
    tags = [str(t) for t in actor.tags]
    if ACTOR_TAG in tags:
        return 0
    actor.modify()
    actor.set_editor_property("tags", list(actor.tags) + [unreal.Name(ACTOR_TAG)])
    return 1


def run():
    base = unreal.load_asset(BASE_MATERIAL)
    if base is None:
        unreal.log_error("[beacon_apply] missing material %s" % BASE_MATERIAL)
        return
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = list(eas.get_all_level_actors())

    mics = {}
    beacons = elements = already = tagged = 0
    for actor in actors:
        if not isinstance(actor, unreal.StaticMeshActor):
            continue
        comp = actor.static_mesh_component
        if comp is None or comp.static_mesh is None:
            continue
        names = _material_names(comp)
        if beacon_element_indices(names, APPLIED_MARKER):
            already += 1
            tagged += _ensure_tag(actor)
            continue
        idx = beacon_element_indices(names, NATIVE_MARKER)
        if not idx:
            continue
        tagged += _ensure_tag(actor)
        comp.modify()
        for i in idx:
            split = _section_y_split(comp.static_mesh, i)
            if split is None:
                split = 0.0
                unreal.log_warning(
                    "[beacon_apply] %s section %d: no split measured, using 0"
                    % (actor.get_actor_label(), i))
            mic = _mic_for_split(split, base, mics)
            if mic is None:
                continue
            comp.set_material(i, mic)
            elements += 1
        beacons += 1

    has_manager = any(a.get_actor_label() == MANAGER_LABEL for a in actors)
    spawned = 0
    if not has_manager:
        cls = unreal.EditorAssetLibrary.load_blueprint_class(MANAGER_BP)
        if cls is None:
            unreal.log_error("[beacon_apply] missing class %s" % MANAGER_BP)
        else:
            mgr = eas.spawn_actor_from_class(cls, unreal.Vector(0, 0, 0))
            mgr.set_actor_label(MANAGER_LABEL)
            spawned = 1

    unreal.log(
        "[beacon_apply] beacons=%d elements=%d mics=%d already_applied=%d "
        "tagged=%d manager_spawned=%d (save the level to persist)"
        % (beacons, elements, len(mics), already, tagged, spawned))


run()

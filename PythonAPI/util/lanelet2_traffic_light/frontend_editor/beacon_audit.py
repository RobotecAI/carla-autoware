"""Read-only audit of flashing-beacon candidate meshes (Orange_Light slots).

Reports one key=value line per candidate actor plus per-section vertex Z and
UV V ranges, so the upper/lower lamp layout (separate elements vs single
shared element) can be decided before building the material. Editor-only.

Run from the editor Output Log:
    py "<repo>/PythonAPI/util/lanelet2_traffic_light/frontend_editor/beacon_audit.py"
"""
import os
import sys

import unreal

_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _PKG_ROOT not in sys.path:
    sys.path.append(_PKG_ROOT)

# Reload: the editor's Python caches modules across `py` runs, so a stale
# beacon_select from an earlier run would shadow on-disk edits.
import importlib  # noqa: E402

import lanelet2_traffic_light.frontend_editor.beacon_select as _beacon_select  # noqa: E402

_beacon_select = importlib.reload(_beacon_select)
beacon_element_indices = _beacon_select.beacon_element_indices
largest_gap = _beacon_select.largest_gap

NATIVE_MARKER = "Orange_Light"


def _material_names(comp):
    names = []
    for i in range(comp.get_num_materials()):
        mat = comp.get_material(i)
        names.append(mat.get_name() if mat is not None else None)
    return names


def _axis_line(name, values):
    gap, mid = largest_gap(values)
    return ("%s_min=%.3f %s_max=%.3f %s_gap=%.3f %s_split=%.3f"
            % (name, min(values), name, max(values), name, gap, name, mid))


def _section_report(mesh, section_index):
    """Per-axis range/gap lines for a static-mesh section, or None."""
    try:
        verts, _tris, _normals, uvs, _tangents = (
            unreal.ProceduralMeshLibrary.get_section_from_static_mesh(
                mesh, 0, section_index))
    except Exception as e:  # plugin missing etc. -- audit stays useful without UVs
        unreal.log_warning("[beacon_audit] section %d read failed: %s"
                           % (section_index, e))
        return None
    if not verts:
        return None
    parts = [_axis_line("x", [v.x for v in verts]),
             _axis_line("y", [v.y for v in verts]),
             _axis_line("z", [v.z for v in verts])]
    if uvs:
        parts.append(_axis_line("u", [u.x for u in uvs]))
        parts.append(_axis_line("v", [u.y for u in uvs]))
    return "verts=%d %s" % (len(verts), " ".join(parts))


def run():
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    world = unreal.get_editor_subsystem(
        unreal.UnrealEditorSubsystem).get_editor_world()
    map_name = world.get_path_name().split(".")[0].rsplit("/", 1)[-1]
    report_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "..",
        "flashing_beacon_audit.%s.txt" % map_name))

    lines = ["# flashing beacon audit map=%s marker=%s" % (map_name, NATIVE_MARKER)]
    count = 0
    for actor in eas.get_all_level_actors():
        if not isinstance(actor, unreal.StaticMeshActor):
            continue
        comp = actor.static_mesh_component
        if comp is None or comp.static_mesh is None:
            continue
        names = _material_names(comp)
        idx = beacon_element_indices(names, NATIVE_MARKER)
        if not idx:
            continue
        count += 1
        loc = actor.get_actor_location()
        mesh = comp.static_mesh
        lines.append(
            "label=%s mesh=%s elements=%d matched=%s names=%s "
            "world_x=%.1f world_y=%.1f world_z=%.1f"
            % (actor.get_actor_label(), mesh.get_name(), len(names),
               ",".join(map(str, idx)), "|".join(str(n) for n in names),
               loc.x, loc.y, loc.z))
        for i in idx:
            r = _section_report(mesh, i)
            if r is None:
                lines.append("  section=%d ranges=unavailable" % i)
            else:
                lines.append("  section=%d %s" % (i, r))
    lines.append("# total candidates=%d" % count)
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    unreal.log("[beacon_audit] candidates=%d report=%s" % (count, report_path))


run()

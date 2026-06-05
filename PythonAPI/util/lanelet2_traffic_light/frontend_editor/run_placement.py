"""Placement orchestration for the lanelet2 traffic light tool.

Wraps the full Quick/Full Run flow (parse -> ensure BPs -> place -> report)
into a single callable so both the Tools menu and the EUW can invoke it with
only the OSM path differing. Editor-only (depends on the unreal module).
"""
import os

import unreal

from lanelet2_traffic_light.corelib.api import generate_placements
from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
from lanelet2_traffic_light.frontend_editor.editor_placer import (
    get_world_mgrs_data, place_from_specs,
)
from lanelet2_traffic_light.frontend_editor.bp_factory import (
    ensure_subtype_bps, ensure_group_bp,
)
from lanelet2_traffic_light.frontend_editor.bp_parent_override import (
    parent_bp_override_for_map,
)
from lanelet2_traffic_light.frontend_editor.osm_validation import validate_osm_path
from lanelet2_traffic_light.frontend_editor.save_level import resolve_save_level


def run_placement(osm_path, limit=None, save_level=None):
    """Run the lanelet2 -> CARLA traffic light placement.

    Args:
        osm_path: lanelet2 .osm file path. Validated up front; an invalid
            path shows an error dialog and aborts.
        limit: place only the first `limit` placements (smoke test) when an
            int; place all when None.
        save_level: save the current level to disk after placement. True or
            False forces the decision; None (default) resolves from the
            LANELET2_SAVE_LEVEL environment variable ("1"/"true"/"yes"/"on"
            enables saving).

    Returns:
        The PlacementReport from place_from_specs, or None if aborted.
    """
    err = validate_osm_path(osm_path)
    if err is not None:
        unreal.log_error("[lanelet2_tl] " + err)
        try:
            unreal.EditorDialog.show_message(
                "lanelet2 Traffic Light", err, unreal.AppMsgType.OK)
        except Exception:
            pass
        return None

    project_dir = unreal.Paths.project_dir()

    # Resolve the currently open level's path and map name (basename) first,
    # so the report file name can include the map name.
    try:
        world = unreal.get_editor_subsystem(
            unreal.UnrealEditorSubsystem).get_editor_world()
        asset_path = world.get_path_name() if world is not None else ""
        if asset_path.startswith("/Game/"):
            rel = asset_path[len("/Game/"):].split(".")[0]
            map_file_path = os.path.abspath(
                os.path.join(project_dir, "Content", rel + ".umap"))
            map_name = rel.rsplit("/", 1)[-1]
        else:
            map_file_path = asset_path or "<unknown>"
            map_name = "Unknown"
    except Exception:
        map_file_path = "<unknown>"
        map_name = "Unknown"

    # Report file name includes the map name, e.g.
    # lanelet2_tl_full_run_report.Odaiba.txt
    report_path = os.path.abspath(os.path.join(
        project_dir, "..", "..", "..",
        f"lanelet2_tl_full_run_report.{map_name}.txt",
    ))

    unreal.log(f"[lanelet2_tl] ensuring subtype BPs for map '{map_name}'...")
    parent_override = parent_bp_override_for_map(map_name)
    bp_override = ensure_subtype_bps(PROFILE_JP, map_name, parent_override=parent_override)
    for st, bp in bp_override.items():
        unreal.log(f"[lanelet2_tl]   subtype={st} -> {bp}")

    unreal.log(f"[lanelet2_tl] ensuring TrafficLightGroup BP for map '{map_name}'...")
    group_bp_path = ensure_group_bp(map_name)
    unreal.log(f"[lanelet2_tl]   group_bp -> {group_bp_path}")

    transformer = get_world_mgrs_data()
    placements, groups, report = generate_placements(
        osm_path=osm_path,
        profile=PROFILE_JP,
        sign_id_resolver=WayIdResolver(),
        transformer=transformer,
        bp_override=bp_override,
    )
    unreal.log(
        f"[lanelet2_tl] parsed TL={report.parsed_traffic_lights} "
        f"groups={report.parsed_groups} skipped={len(report.placements_skipped)}"
    )

    save_level_resolved = resolve_save_level(save_level)
    preview = placements if limit is None else placements[:limit]
    scope = "all" if limit is None else f"first {limit}"
    unreal.log(f"[lanelet2_tl] placing {len(preview)} traffic lights ({scope})...")
    preport = place_from_specs(
        preview, groups,
        save_level=save_level_resolved,
        snap_to_existing_mesh=True,
        snap_radius_cm=300.0,
        report_path=report_path,
        report_metadata={
            "osm_path"      : osm_path,
            "map_file_path" : map_file_path,
            "map_name"      : map_name,
            "ue_project_dir": os.path.abspath(project_dir),
            "report_path"   : report_path,
            "group_bp_path" : group_bp_path,
        },
        group_bp_path=group_bp_path,
        map_name=map_name,
    )
    unreal.log(
        f"[lanelet2_tl] result: created={preport.created} updated={preport.updated} "
        f"snap_skipped={len(preport.snap_skipped)} failed={len(preport.failed)} "
        f"unused_meshes={len(preport.unused_existing_meshes)} | "
        f"groups created={preport.groups_created} updated={preport.groups_updated} | "
        f"save_level={save_level_resolved}"
    )
    unreal.log(f"[lanelet2_tl] full report file: {report_path}")
    return preport


def run_placement_and_summarize(osm_path, limit=None, save_level=None):
    """Run placement and return a one-line summary string for UI display (EUW).

    Thin wrapper around run_placement() that turns the PlacementReport into a
    short string suitable for an Editor Utility Widget status label. Intended to
    be called as a single expression via PythonScriptLibrary in EVALUATE mode so
    the returned string can be shown in the widget.

    Args:
        osm_path: lanelet2 .osm file path.
        limit: place only the first `limit` placements when an int; all when None.
        save_level: save the current level to disk after placement. True or
            False forces the decision; None (default) resolves from the
            LANELET2_SAVE_LEVEL environment variable ("1"/"true"/"yes"/"on"
            enables saving).

    Returns:
        A one-line summary string (also when aborted).
    """
    report = run_placement(osm_path, limit, save_level=save_level)
    if report is None:
        return "Aborted: invalid OSM path (see Output Log)."
    saved = "yes" if resolve_save_level(save_level) else "no"
    return (
        f"Done: created={report.created} updated={report.updated} "
        f"snap_skipped={len(report.snap_skipped)} failed={len(report.failed)} "
        f"unused_meshes={len(report.unused_existing_meshes)} "
        f"groups={report.groups_created} saved={saved}"
    )

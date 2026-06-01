"""Hide / show / delete the native signal meshes that placed signals snap onto.

Each placed signal actor (label `TLV_*` / `TLP_*`) is positioned right on top of the
native mesh it snapped to, so the original native mesh overlaps (and may occlude or
z-fight) the lit signal. This module hides (reversible), un-hides, or deletes
(irreversible) only the native meshes that sit under a placed signal — native meshes
with no placed signal (lanelet2-unregistered decoratives) are left in place so the
scene does not develop gaps.

Map-agnostic: the native mesh label prefixes come from the open map's `MapProfile`
(`map_profile.MAP_PROFILES`), so a new map only needs its MAP_PROFILES entry to make
both placement and this cleanup work. Editor-only (depends on the `unreal` module).
"""
import unreal

from lanelet2_traffic_light.frontend_editor.map_profile import get_map_profile

# Labels of the placed signal actors (map-independent; set by _label_prefix_for_subtype).
SIGNAL_PREFIXES = ("TLV_", "TLP_")
# Substrings of native labels that are never signal heads (e.g. road-surface markings).
DEFAULT_EXCLUDE = ("Ground",)
# Snap places a signal exactly on its native mesh, so a tight XY radius matches only
# that native (not neighbouring signals at the same intersection).
DEFAULT_MATCH_RADIUS_CM = 80.0


def _log(msg):
    unreal.log("[native_cleanup] " + msg)


def _actor_label(actor):
    try:
        return actor.get_actor_label()
    except Exception:
        return ""


def _open_map_native_prefixes():
    """Return the native signal-mesh label prefixes for the currently open map.

    Derived from the map's MapProfile (vehicle + pedestrian mesh prefixes), so it
    stays in sync with placement. Falls back to the generic default profile prefixes
    when the open map cannot be resolved.
    """
    map_name = "Unknown"
    try:
        world = unreal.get_editor_subsystem(
            unreal.UnrealEditorSubsystem).get_editor_world()
        asset_path = world.get_path_name() if world is not None else ""
        if asset_path.startswith("/Game/"):
            map_name = asset_path.split(".")[0].rsplit("/", 1)[-1]
    except Exception:
        pass
    prof = get_map_profile(map_name)
    prefixes = tuple(prof.vehicle_mesh_prefixes) + tuple(prof.pedestrian_mesh_prefixes)
    return map_name, prefixes


def _is_native(label, native_prefixes, exclude_substrings):
    return label.startswith(native_prefixes) and not any(
        x in label for x in exclude_substrings)


def _placed_signal_xy(actors):
    xy = []
    for a in actors:
        if _actor_label(a).startswith(SIGNAL_PREFIXES):
            loc = a.get_actor_location()
            xy.append((loc.x, loc.y))
    return xy


def _snapped_native_actors(actors, native_prefixes, signal_xy,
                           match_radius_cm, exclude_substrings):
    """Native actors whose XY is within match_radius_cm of any placed signal."""
    r2 = match_radius_cm * match_radius_cm
    out = []
    for a in actors:
        if not _is_native(_actor_label(a), native_prefixes, exclude_substrings):
            continue
        loc = a.get_actor_location()
        if any((loc.x - sx) ** 2 + (loc.y - sy) ** 2 <= r2 for sx, sy in signal_xy):
            out.append(a)
    return out


def hide_snapped_native_meshes(match_radius_cm=DEFAULT_MATCH_RADIUS_CM,
                               exclude_substrings=DEFAULT_EXCLUDE):
    """Hide native meshes that sit under a placed signal (reversible).

    Returns the number of hidden actors.
    """
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = list(eas.get_all_level_actors())
    map_name, native_prefixes = _open_map_native_prefixes()
    signal_xy = _placed_signal_xy(actors)
    targets = _snapped_native_actors(
        actors, native_prefixes, signal_xy, match_radius_cm, exclude_substrings)
    for a in targets:
        try:
            a.set_actor_hidden_in_game(True)
        except Exception:
            pass
        try:
            a.set_is_temporarily_hidden_in_editor(True)
        except Exception:
            pass
    _log(f"hide: map={map_name} prefixes={native_prefixes} signals={len(signal_xy)} "
         f"hidden={len(targets)} (unregistered natives left visible)")
    return len(targets)


def show_native_meshes(match_radius_cm=None, exclude_substrings=DEFAULT_EXCLUDE):
    """Un-hide all native signal meshes for the open map (reverse of hide).

    `match_radius_cm` is unused (kept for signature symmetry); show always restores
    every native mesh of the map's prefixes. Deleted actors cannot be restored.
    """
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = list(eas.get_all_level_actors())
    map_name, native_prefixes = _open_map_native_prefixes()
    n = 0
    for a in actors:
        if not _is_native(_actor_label(a), native_prefixes, exclude_substrings):
            continue
        try:
            a.set_actor_hidden_in_game(False)
        except Exception:
            pass
        try:
            a.set_is_temporarily_hidden_in_editor(False)
        except Exception:
            pass
        n += 1
    _log(f"show: map={map_name} un-hid={n}")
    return n


def delete_snapped_native_meshes(confirm=False,
                                 match_radius_cm=DEFAULT_MATCH_RADIUS_CM,
                                 exclude_substrings=DEFAULT_EXCLUDE):
    """Delete native meshes that sit under a placed signal (IRREVERSIBLE).

    Requires `confirm=True`; otherwise it only reports the count that would be
    deleted (dry run). Persisting the deletion requires saving the level.
    Returns the number of deleted actors, or the would-delete count on a dry run.
    """
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = list(eas.get_all_level_actors())
    map_name, native_prefixes = _open_map_native_prefixes()
    signal_xy = _placed_signal_xy(actors)
    targets = _snapped_native_actors(
        actors, native_prefixes, signal_xy, match_radius_cm, exclude_substrings)
    if not confirm:
        _log(f"delete DRY-RUN: map={map_name} would delete {len(targets)} co-located "
             f"native meshes. Pass confirm=True to delete (irreversible).")
        return len(targets)
    for a in targets:
        try:
            eas.destroy_actor(a)
        except Exception:
            pass
    _log(f"delete: map={map_name} deleted={len(targets)} (IRREVERSIBLE; "
         f"save the level to persist)")
    return len(targets)


# --- EUW summary wrappers (return a one-line status string; never raise) ---------

def hide_snapped_native_meshes_summary():
    """EUW wrapper: hide co-located native meshes; return a one-line status string."""
    try:
        n = hide_snapped_native_meshes()
        return f"Hid {n} native meshes (lanelet2-unregistered ones kept visible)."
    except Exception as e:
        return f"ERROR: {e}"


def show_native_meshes_summary():
    """EUW wrapper: un-hide native meshes; return a one-line status string."""
    try:
        n = show_native_meshes()
        return f"Un-hid {n} native meshes."
    except Exception as e:
        return f"ERROR: {e}"


def delete_snapped_native_meshes_summary():
    """EUW wrapper: confirm via a Yes/No dialog, then delete (IRREVERSIBLE)."""
    try:
        count = delete_snapped_native_meshes(confirm=False)  # dry-run count
        if count == 0:
            return "No co-located native meshes to delete."
        result = unreal.EditorDialog.show_message(
            "Delete native meshes",
            f"Delete {count} native meshes under placed signals?\n"
            f"This is irreversible (save the level to persist).",
            unreal.AppMsgType.YES_NO,
        )
        if result != unreal.AppReturnType.YES:
            return "Cancelled."
        deleted = delete_snapped_native_meshes(confirm=True)
        return f"Deleted {deleted} native meshes. Save the level (Ctrl+S) to persist."
    except Exception as e:
        return f"ERROR: {e}"

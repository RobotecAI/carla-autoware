"""Toggle green-arrow lighting on all vehicle traffic-light actors.

The pure part (`is_vehicle_tl_label`) is unit-tested; `set_arrows_enabled`
is the editor-time wrapper (imports unreal lazily so the module is importable
without the engine for tests).
"""


def is_vehicle_tl_label(label: str) -> bool:
    """Vehicle TL actors are labelled `TLV_<sign_id>` (see editor_placer label prefix)."""
    return bool(label) and label.startswith("TLV_")


def set_arrows_enabled(enabled: bool) -> int:
    """Set ArrowsEnabled on every vehicle TL actor and rerun their construction
    scripts. Returns the number of actors updated. Editor-time only.
    """
    import unreal

    eas = unreal.get_editor_actor_subsystem()
    count = 0
    for actor in eas.get_all_level_actors():
        if not is_vehicle_tl_label(actor.get_actor_label()):
            continue
        try:
            actor.set_editor_property("ArrowsEnabled", bool(enabled))
            actor.rerun_construction_scripts()
            count += 1
        except Exception as e:
            unreal.log_warning(f"set_arrows_enabled: {actor.get_actor_label()} failed: {e}")
    unreal.log(f"set_arrows_enabled({enabled}): updated {count} vehicle TL actors")
    return count

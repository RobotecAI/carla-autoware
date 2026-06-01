"""Editor runner: feed TrafficLightB* pedestrian mesh positions back into lanelet2.

Run in the UE5 Editor Output Log (target map open):
    import importlib
    import lanelet2_traffic_light.frontend_editor.pedestrian_feedback_runner as r
    importlib.reload(r); r.run_pedestrian_feedback()

Enumerates TrafficLightB* pedestrian mesh actors, inverse-transforms their world
XY to lanelet2 local coordinates, and writes a copy of the lanelet2 OSM with
added pedestrian traffic_light (subtype=red_green) entries so the standard
placement pipeline can place them. Editor-only (depends on the unreal module).

Pure logic (transform inverse, id allocation, ele estimation, osm serialization)
lives in corelib and is unit-tested; this file only enumerates actors and wires
those pure functions together.
"""
import os

import unreal

from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
from lanelet2_traffic_light.corelib.osm_feedback.id_allocator import allocate_id_base
from lanelet2_traffic_light.corelib.osm_feedback.pedestrian_osm_feedback import (
    parse_existing, estimate_ped_ele, nearest_latlon, derive_output_path,
    build_feedback_osm, PedSignal,
)
from lanelet2_traffic_light.frontend_editor.editor_placer import get_world_mgrs_data

PED_PREFIX = "TrafficLightB"          # pedestrian signal meshes
EXCLUDE_SUBSTR = ("Root", "_All_", "GP")  # group/root helper actors, not signals
DEDUP_CM = 50.0                        # quantize world XY to merge near-coincident meshes
SEGMENT_LENGTH_M = 0.5                 # generated traffic_light way length


def _log(msg):
    unreal.log("[ped_feedback] " + msg)


def _is_pedestrian_signal_label(label):
    if not label.startswith(PED_PREFIX):
        return False
    return not any(s in label for s in EXCLUDE_SUBSTR)


def run_pedestrian_feedback(osm_path=None):
    """Generate <map>.pedestrianSignalAdded.osm from TrafficLightB* meshes.

    Args:
        osm_path: lanelet2 .osm path. Defaults to env var LANELET2_OSM_PATH.
    """
    osm_path = osm_path or os.environ.get("LANELET2_OSM_PATH", "")
    if not osm_path or not os.path.isfile(osm_path):
        _log("ERROR LANELET2_OSM_PATH not set or file missing: %r" % osm_path)
        return

    tf = get_world_mgrs_data()
    if tf is None:
        _log("ERROR no MGRS transformer (open the target map first)")
        return

    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    raw = []
    for a in actor_subsys.get_all_level_actors():
        try:
            label = a.get_actor_label()
        except Exception:
            continue
        if not _is_pedestrian_signal_label(label):
            continue
        loc = a.get_actor_location()
        yaw = a.get_actor_rotation().yaw
        raw.append((loc.x, loc.y, loc.z, yaw))
    _log("found pedestrian actors=%d (prefix=%s, excluded=%s)"
         % (len(raw), PED_PREFIX, ",".join(EXCLUDE_SUBSTR)))

    # Deduplicate near-coincident meshes by quantized world XY.
    seen = set()
    meshes = []
    for (x, y, z, yaw) in raw:
        key = (round(x / DEDUP_CM), round(y / DEDUP_CM))
        if key in seen:
            continue
        seen.add(key)
        meshes.append((x, y, z, yaw))
    _log("after dedup=%d" % len(meshes))
    if not meshes:
        _log("no pedestrian meshes; nothing to do")
        return

    parsed = parse_existing(osm_path)
    diff = (PROFILE_JP.pole_height_m("red_yellow_green")
            - PROFILE_JP.pole_height_m("red_green"))
    _log("existing max_id=%d nodes=%d vehicle_tl_nodes=%d pole_height_diff_m=%.3f"
         % (parsed.max_id, len(parsed.nodes), len(parsed.vehicle_tl_nodes), diff))

    signals = []
    for i, (x, y, z, yaw) in enumerate(meshes):
        # abs_h round-trips the mesh's own UE Z (forward transform reproduces it
        # exactly), so the canonical mesh lands at the original TrafficLightB Z.
        lx, ly, abs_h = tf.unreal_cm_to_local(x, y, z)
        est = estimate_ped_ele((lx, ly), parsed.vehicle_tl_nodes, diff)  # alt Z source
        lat, lon, mgrs = nearest_latlon((lx, ly), parsed.nodes)
        if i < 3:
            _log("z-source mesh_z_cm=%.1f mesh_abs_h_m=%.2f vehicle_est_m=%.2f (using mesh_abs_h)"
                 % (z, abs_h, est))
        signals.append(PedSignal(
            local_x=lx, local_y=ly, ele=abs_h,
            placed_yaw_deg=yaw, lat=lat, lon=lon, mgrs_code=mgrs,
            pole_height=abs_h,
        ))

    id_base = allocate_id_base(parsed.max_id, num_new=4 * len(signals))
    with open(osm_path, "r", encoding="utf-8") as f:
        original = f.read()
    out_text = build_feedback_osm(
        original, signals,
        id_base=id_base,
        yaw_offset_deg=PROFILE_JP.yaw_offset_deg(),
        segment_length_m=SEGMENT_LENGTH_M,
    )
    out_path = derive_output_path(osm_path)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(out_text)
    _log("WROTE signals=%d id_base=%d out=%s" % (len(signals), id_base, out_path))
    _log("NEXT set LANELET2_OSM_PATH=%s, reopen editor / run placement, "
         "verify TLP_ appear near TrafficLightB XY." % out_path)
    return out_path


def hide_existing_pedestrian_meshes(label_prefixes=("TrafficLightB", "Pedestrian_Lights"),
                                    delete=False):
    """Hide (or delete) the map's original pedestrian-signal meshes.

    The canonical TLP_ actors now overlap and are partly masked by the original
    meshes (TrafficLightB* on NishiShinjuku, Pedestrian_Lights_* on Odaiba).
    Hiding (default) is reversible (in-game + editor visibility); delete=True is
    permanent and requires saving the level. Vehicle meshes (TrafficLightsA*,
    Traffic_Lights*) and generated TLP_/TLV_ actors are not matched.
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    n = 0
    for a in list(actor_subsys.get_all_level_actors()):
        try:
            label = a.get_actor_label()
        except Exception:
            continue
        if not any(label.startswith(p) for p in label_prefixes):
            continue
        if delete:
            actor_subsys.destroy_actor(a)
        else:
            try:
                a.set_actor_hidden_in_game(True)
            except Exception:
                pass
            try:
                a.set_is_temporarily_hidden_in_editor(True)
            except Exception:
                pass
        n += 1
    _log("%s %d existing meshes (prefixes=%s, delete=%s)"
         % ("deleted" if delete else "hid", n, ",".join(label_prefixes), delete))
    return n


def show_existing_pedestrian_meshes(label_prefixes=("TrafficLightB", "Pedestrian_Lights")):
    """Reverse of hide_existing_pedestrian_meshes: un-hide matching actors.

    Restores both in-game and editor viewport visibility for the original
    pedestrian-signal meshes. Has no effect on actors that were deleted
    (delete=True is irreversible).
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    n = 0
    for a in list(actor_subsys.get_all_level_actors()):
        try:
            label = a.get_actor_label()
        except Exception:
            continue
        if not any(label.startswith(p) for p in label_prefixes):
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
    _log("shown %d existing meshes (prefixes=%s)" % (n, ",".join(label_prefixes)))
    return n


def run_pedestrian_feedback_summary(osm_path=None):
    """EUW wrapper: run the mesh->osm feedback; return the output OSM path on success.

    Returns the absolute path of the written *.pedestrianSignalAdded.osm so the EUW
    can put it back into the OSM-path field, or an "ERROR: ..." string on failure.
    Never raises.
    """
    try:
        out = run_pedestrian_feedback(osm_path)
        if not out:
            return ("ERROR: no output written (check the OSM path, the open map, and "
                    "the TrafficLightB meshes; see the Output Log).")
        return out
    except Exception as e:
        return f"ERROR: {e}"

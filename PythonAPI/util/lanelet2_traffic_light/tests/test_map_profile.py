from lanelet2_traffic_light.frontend_editor.map_profile import (
    get_map_profile, MapProfile, MAP_PROFILES,
    SCENE_FIGURE_PARENT_BP, TRANSPLANT_VEHICLE_MESH,
)


def test_unknown_map_returns_generic_default():
    prof = get_map_profile("SomeFutureMap")
    assert prof.vehicle_mesh_prefixes == ("Traffic_Lights",)
    assert prof.pedestrian_mesh_prefixes == ("Pedestrian_Lights",)
    assert prof.vehicle_transplant is None
    assert prof.parent_bp_override == {}
    assert prof.pedestrian_uses_snap is True


def test_none_map_returns_default():
    assert get_map_profile(None).vehicle_transplant is None


def test_odaiba_profile():
    prof = get_map_profile("Odaiba")
    assert prof.vehicle_mesh_prefixes == ("Traffic_Lights",)
    assert prof.pedestrian_mesh_prefixes == ("Pedestrian_Lights",)
    assert prof.vehicle_transplant is None  # adopts the native Scene_NNN mesh
    assert prof.parent_bp_override == {"red_green": SCENE_FIGURE_PARENT_BP}
    assert prof.pedestrian_uses_snap is True  # Odaiba peds snap (SceneFigure)


def test_nishishinjuku_profile():
    prof = get_map_profile("NishishinjukuMap")
    assert prof.vehicle_mesh_prefixes == ("TrafficLightsA",)
    assert prof.pedestrian_mesh_prefixes == ("TrafficLightB",)
    t = prof.vehicle_transplant
    assert t is not None
    assert t["mesh"] == TRANSPLANT_VEHICLE_MESH
    assert t["relrot_deg"] == (0.0, 120.0, -90.0)
    assert t["world_z_offset_cm"] == -24.0
    assert t["ignore_snap_z_gate"] is True
    assert prof.parent_bp_override == {}  # pedestrians are canonical (snap=False)
    assert prof.pedestrian_uses_snap is False  # native TrafficLightB is ~10x; keep canonical


def test_adding_a_new_map_is_one_entry():
    # Extensibility: a new map is a single MAP_PROFILES entry; unset axes use defaults.
    prof = MapProfile(vehicle_mesh_prefixes=("FooLights",))
    assert prof.vehicle_mesh_prefixes == ("FooLights",)
    assert prof.pedestrian_mesh_prefixes == ("Pedestrian_Lights",)  # default
    assert prof.vehicle_transplant is None  # default
    assert prof.parent_bp_override == {}  # default


def test_nishishinjuku_has_arrow_transplant():
    from lanelet2_traffic_light.frontend_editor.map_profile import (
        get_map_profile, TRANSPLANT_VEHICLE_MESH_ARROW,
    )
    t = get_map_profile("NishishinjukuMap").vehicle_transplant_arrow
    assert t is not None
    assert t["mesh"] == TRANSPLANT_VEHICLE_MESH_ARROW
    assert t["ignore_snap_z_gate"] is True


def test_odaiba_has_no_arrow_transplant():
    from lanelet2_traffic_light.frontend_editor.map_profile import get_map_profile
    assert get_map_profile("Odaiba").vehicle_transplant_arrow is None


def test_default_has_no_arrow_transplant():
    from lanelet2_traffic_light.frontend_editor.map_profile import get_map_profile
    assert get_map_profile("Unknown").vehicle_transplant_arrow is None
    assert get_map_profile(None).vehicle_transplant_arrow is None

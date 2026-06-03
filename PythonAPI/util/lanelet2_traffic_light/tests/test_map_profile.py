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


def test_nishishinjuku_arrow_signals_are_native():
    from lanelet2_traffic_light.frontend_editor.map_profile import get_map_profile
    prof = get_map_profile("NishishinjukuMap")
    # Native arrow lighting replaces the 6-light transplant (2026-06-03 pivot): the
    # arrow-bearing native units carry per-direction arrow slots, so arrow signals
    # keep their native mesh (snap, incl. world scale) and light those slots directly.
    assert prof.vehicle_transplant_arrow is None
    cfg = prof.vehicle_arrow_native
    assert cfg is not None
    assert cfg["slot_by_dir"] == {
        "left": "TrafficLightsLeftArrow",
        "straight": "TrafficLightsUpArrow",
        "right": "TrafficLightsRightArrow",
    }
    assert set(cfg["tex_by_dir"]) == {"left", "straight", "right"}
    assert all(p.startswith("/Game/") for p in cfg["tex_by_dir"].values())
    assert cfg["black_level"] == 0.10
    assert cfg["white_level"] == 0.40


def test_odaiba_and_default_have_no_arrow_native():
    from lanelet2_traffic_light.frontend_editor.map_profile import get_map_profile
    assert get_map_profile("Odaiba").vehicle_arrow_native is None
    assert get_map_profile("Odaiba").vehicle_transplant_arrow is None
    assert get_map_profile("Unknown").vehicle_arrow_native is None
    assert get_map_profile(None).vehicle_arrow_native is None


def test_nishishinjuku_native_led_slot():
    # Hybrid native LED (level scan 2026-06-03): all 51 non-arrow native heads carry
    # the TrafficLightsLed slot with the same lamp-row geometry as the 21 arrow units,
    # so 3-light signals also go native WHEN the snapped mesh has this slot. The 14
    # odd 1x-scale heads (slot TrafficLightsA01_Led01_Share01_col) keep the transplant.
    assert get_map_profile("NishishinjukuMap").vehicle_native_led_slot == "TrafficLightsLed"


def test_odaiba_and_default_have_no_native_led_slot():
    assert get_map_profile("Odaiba").vehicle_native_led_slot is None
    assert get_map_profile("Unknown").vehicle_native_led_slot is None
    assert get_map_profile(None).vehicle_native_led_slot is None

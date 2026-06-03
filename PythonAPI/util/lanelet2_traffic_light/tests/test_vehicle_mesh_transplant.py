from lanelet2_traffic_light.frontend_editor.vehicle_mesh_transplant import (
    vehicle_mesh_prefixes_for_map,
    transplant_mesh_for_map,
    TRANSPLANT_VEHICLE_MESH,
)


def test_nishishinjuku_uses_trafficlightsa_prefix():
    assert vehicle_mesh_prefixes_for_map("NishishinjukuMap") == ("TrafficLightsA",)


def test_odaiba_uses_traffic_lights_prefix():
    assert vehicle_mesh_prefixes_for_map("Odaiba") == ("Traffic_Lights",)


def test_unknown_map_defaults_to_traffic_lights():
    assert vehicle_mesh_prefixes_for_map("Unknown") == ("Traffic_Lights",)
    assert vehicle_mesh_prefixes_for_map(None) == ("Traffic_Lights",)


def test_nishishinjuku_has_transplant_config():
    cfg = transplant_mesh_for_map("NishishinjukuMap")
    assert cfg is not None
    assert cfg["mesh"] == TRANSPLANT_VEHICLE_MESH
    assert cfg["scale"] == 1.0
    assert cfg["relrot_deg"] == (0.0, 120.0, -90.0)
    assert cfg["world_z_offset_cm"] == -24.0
    assert cfg["local_offset_cm"] == (0.0, 0.0, 0.0)
    assert cfg["ignore_snap_z_gate"] is True


def test_odaiba_has_no_transplant():
    assert transplant_mesh_for_map("Odaiba") is None


def test_unknown_map_has_no_transplant():
    assert transplant_mesh_for_map("Unknown") is None
    assert transplant_mesh_for_map(None) is None


from lanelet2_traffic_light.frontend_editor.vehicle_mesh_transplant import (
    green_arrow_dirs,
    select_vehicle_transplant,
)


def test_green_arrow_dirs_filters_green_only():
    arrows = (("green", "right"), ("green", "straight"), ("yellow", "left"))
    assert green_arrow_dirs(arrows) == frozenset({"right", "straight"})


def test_green_arrow_dirs_empty():
    assert green_arrow_dirs(()) == frozenset()


def test_select_vehicle_transplant_picks_arrow_when_green_dirs():
    plain = {"mesh": "plain"}
    arrow = {"mesh": "arrow"}
    assert select_vehicle_transplant(plain, arrow, frozenset({"right"})) is arrow


def test_select_vehicle_transplant_plain_when_no_green_dirs():
    plain = {"mesh": "plain"}
    arrow = {"mesh": "arrow"}
    assert select_vehicle_transplant(plain, arrow, frozenset()) is plain


def test_select_vehicle_transplant_plain_when_arrow_none():
    plain = {"mesh": "plain"}
    assert select_vehicle_transplant(plain, None, frozenset({"right"})) is plain


def test_select_vehicle_transplant_native_arrow_returns_none():
    # arrow_native: arrow-bearing signals keep their NATIVE mesh (no transplant at all),
    # so their per-direction arrow slots can be lit directly (NishiShinjuku 2026-06-03).
    plain = {"mesh": "plain"}
    arrow = {"mesh": "arrow"}
    assert select_vehicle_transplant(plain, arrow, frozenset({"right"}), arrow_native=True) is None
    # signals without green arrows are unaffected by the flag
    assert select_vehicle_transplant(plain, arrow, frozenset(), arrow_native=True) is plain


from lanelet2_traffic_light.frontend_editor.vehicle_mesh_transplant import (
    native_led_capable,
)


def test_native_led_capable_when_slot_present():
    slots = ["TrafficLightsLed", "TrafficLightsA01_Head01_White01_MT1"]
    assert native_led_capable(slots, "TrafficLightsLed") is True


def test_native_led_capable_false_for_odd_head_family():
    # NishiShinjuku's 14 odd 1x-scale heads use a different slot naming and geometry:
    # they must keep the transplant (no native LED drive possible).
    slots = ["TrafficLightsA01_Head01_Black01_col", "TrafficLightsA01_Led01_Share01_col"]
    assert native_led_capable(slots, "TrafficLightsLed") is False


def test_native_led_capable_false_when_map_does_not_declare_a_slot():
    assert native_led_capable(["TrafficLightsLed"], None) is False

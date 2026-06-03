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

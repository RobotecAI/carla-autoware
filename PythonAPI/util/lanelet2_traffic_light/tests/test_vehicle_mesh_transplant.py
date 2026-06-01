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

from lanelet2_traffic_light.frontend_editor.arrow_lighting import (
    ARROW_SLOT_BY_DIR, ARROW_TEX_BY_DIR, ARROW_LIT_MATERIAL,
    arrow_slot_for_dir, arrow_texture_for_dir,
)


def test_arrow_slot_for_dir():
    assert arrow_slot_for_dir("left") == "Green_Left_Arrow"
    assert arrow_slot_for_dir("straight") == "Green_Straight_Arrow"
    assert arrow_slot_for_dir("right") == "Green_Right_Arrow"
    assert arrow_slot_for_dir("unknown") is None


def test_arrow_texture_for_dir_points_to_t4():
    for d in ("left", "straight", "right"):
        assert arrow_texture_for_dir(d).startswith("/T4/")
    assert arrow_texture_for_dir("nope") is None


def test_lit_material_is_t4():
    assert ARROW_LIT_MATERIAL.startswith("/T4/")
    assert "M_JPArrowLit" in ARROW_LIT_MATERIAL


def test_maps_cover_exactly_three_directions():
    assert set(ARROW_SLOT_BY_DIR) == {"left", "straight", "right"}
    assert set(ARROW_TEX_BY_DIR) == {"left", "straight", "right"}


def test_resolve_arrow_config_defaults():
    from lanelet2_traffic_light.frontend_editor.arrow_lighting import resolve_arrow_config
    cfg = resolve_arrow_config(None)
    assert cfg["slot_by_dir"] == ARROW_SLOT_BY_DIR
    assert cfg["tex_by_dir"] == ARROW_TEX_BY_DIR
    assert cfg["black_level"] is None and cfg["white_level"] is None


def test_arrow_slots_present():
    from lanelet2_traffic_light.frontend_editor.arrow_lighting import (
        arrow_slots_present, resolve_arrow_config,
    )
    cfg = resolve_arrow_config({"slot_by_dir": {
        "left": "TrafficLightsLeftArrow",
        "straight": "TrafficLightsUpArrow",
        "right": "TrafficLightsRightArrow",
    }})
    # arrow unit (any direction slot present)
    assert arrow_slots_present(
        ["TrafficLightsLed", "TrafficLightsRightArrow", "Body"], cfg) is True
    # plain 3-light native head: no arrow slot
    assert arrow_slots_present(["TrafficLightsLed", "Body"], cfg) is False
    # odd 1x compact head: different slot family entirely
    assert arrow_slots_present(
        ["TrafficLightsA01_Head01_Black01_col", "TrafficLightsA01_Led01_Share01_col"],
        cfg) is False


def test_resolve_arrow_config_per_map_override():
    from lanelet2_traffic_light.frontend_editor.arrow_lighting import resolve_arrow_config
    cfg = resolve_arrow_config({
        "slot_by_dir": {"right": "TrafficLightsRightArrow"},
        "tex_by_dir": {"right": "/Game/somewhere/tex"},
        "black_level": 0.10,
        "white_level": 0.40,
    })
    assert cfg["slot_by_dir"] == {"right": "TrafficLightsRightArrow"}
    assert cfg["tex_by_dir"] == {"right": "/Game/somewhere/tex"}
    assert cfg["black_level"] == 0.10
    assert cfg["white_level"] == 0.40

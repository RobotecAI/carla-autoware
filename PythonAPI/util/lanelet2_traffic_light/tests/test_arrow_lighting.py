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

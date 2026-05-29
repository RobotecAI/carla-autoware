from lanelet2_traffic_light.frontend_editor.bp_parent_override import (
    SCENE_FIGURE_PARENT_BP,
    parent_bp_override_for_map,
)


def test_odaiba_pedestrian_uses_scene_figure_parent():
    ov = parent_bp_override_for_map("Odaiba")
    assert ov["red_green"] == SCENE_FIGURE_PARENT_BP


def test_other_maps_have_no_override():
    assert parent_bp_override_for_map("NishishinjukuMap") == {}
    assert parent_bp_override_for_map("Unknown") == {}

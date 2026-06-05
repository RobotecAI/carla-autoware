from lanelet2_traffic_light.frontend_editor.arrow_runtime import (
    ARROW_FLAG_BY_DIR, initial_arrow_mask, capabilities_mask,
)


def test_flags_match_rpc_green_row():
    # Must mirror carla/rpc/TrafficLightArrowState.h (GreenLeft/Straight/Right).
    assert ARROW_FLAG_BY_DIR == {"left": 0x1, "straight": 0x2, "right": 0x4}


def test_initial_arrow_mask():
    assert initial_arrow_mask(frozenset()) == 0
    assert initial_arrow_mask({"right"}) == 0x4
    assert initial_arrow_mask({"left", "straight", "right"}) == 0x7
    assert initial_arrow_mask({"right", "unknown"}) == 0x4  # unknown dirs ignored


def test_capabilities_mask_from_slot_indices():
    assert capabilities_mask({}) == 0
    assert capabilities_mask({"right": 2}) == 0x4
    assert capabilities_mask({"left": 1, "straight": 3, "right": 2}) == 0x7
    assert capabilities_mask({"left": -1, "right": 0}) == 0x4  # -1 = no face

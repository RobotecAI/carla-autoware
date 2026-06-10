from lanelet2_traffic_light.frontend_editor.arrow_runtime import (
    ARROW_FLAG_BY_DIR, initial_arrow_mask, capabilities_mask,
)


def test_flags_match_rpc_green_row():
    # FROZEN layout lock: exact-equality on purpose -- update only when
    # carla/rpc/TrafficLightArrowState.h is renumbered (it never should be).
    # Must mirror carla/rpc/TrafficLightArrowState.h green row (bits 0-7).
    # Direction index: 0=left 1=straight 2=right 3=up_left 4=up_right
    # 5=down (reserved, intentionally absent here) 6=down_left 7=down_right.
    assert ARROW_FLAG_BY_DIR == {
        "left": 0x1,
        "straight": 0x2,
        "right": 0x4,
        "up_left": 0x8,
        "up_right": 0x10,
        "down_left": 0x40,
        "down_right": 0x80,
    }


def test_initial_arrow_mask():
    assert initial_arrow_mask(frozenset()) == 0
    assert initial_arrow_mask({"right"}) == 0x4
    assert initial_arrow_mask({"left", "straight", "right"}) == 0x7
    assert initial_arrow_mask({"right", "unknown"}) == 0x4  # unknown dirs ignored
    assert initial_arrow_mask({"right", "down_right"}) == 0x84
    assert initial_arrow_mask({"up_left", "up_right"}) == 0x18


def test_capabilities_mask_from_slot_indices():
    # Note: diagonal dirs compute bits here, but arrow_lighting.py's
    # ARROW_SLOT_BY_DIR only maps left/straight/right to physical mesh slots,
    # so real maps never produce diagonal capabilities yet (visuals are out of
    # scope; see specs/2026-06-10-arrow-enum-renumber-design.md §5).
    assert capabilities_mask({}) == 0
    assert capabilities_mask({"right": 2}) == 0x4
    assert capabilities_mask({"left": 1, "straight": 3, "right": 2}) == 0x7
    assert capabilities_mask({"left": -1, "right": 0}) == 0x4  # -1 = no face
    assert capabilities_mask({"down_left": 4, "down_right": 5}) == 0xC0

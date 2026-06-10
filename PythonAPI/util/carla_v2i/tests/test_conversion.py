"""Pure-logic tests for CARLA -> TrafficLightGroupArray conversion (no ROS)."""
import os

from carla_v2i.conversion import (
    COLOR_RED, COLOR_AMBER, COLOR_GREEN,
    SHAPE_CIRCLE, SHAPE_LEFT, SHAPE_UP, SHAPE_RIGHT, SHAPE_DOWN_RIGHT,
    STATUS_SOLID_ON, STATUS_FLASHING, CONFIDENCE,
    elements_for,
    build_relation_to_ways, assemble_groups,
    predict_states,
)


def test_vehicle_red():
    assert elements_for("red", 0, "vehicle") == [
        (COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]


def test_vehicle_yellow_is_amber():
    assert elements_for("yellow", 0, "vehicle") == [
        (COLOR_AMBER, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]


def test_vehicle_off_is_empty():
    assert elements_for("off", 0, "vehicle") == []


def test_pedestrian_yellow_is_green_flashing():
    assert elements_for("yellow", 0, "pedestrian") == [
        (COLOR_GREEN, SHAPE_CIRCLE, STATUS_FLASHING, CONFIDENCE)]


def test_pedestrian_red_and_green():
    assert elements_for("red", 0, "pedestrian") == [
        (COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]
    assert elements_for("green", 0, "pedestrian") == [
        (COLOR_GREEN, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)]


def test_green_arrows_appended():
    # GreenLeft(bit0) | GreenStraight(bit1) with red circle
    got = elements_for("red", 0b11, "vehicle")
    assert (COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_GREEN, SHAPE_LEFT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_GREEN, SHAPE_UP, STATUS_SOLID_ON, CONFIDENCE) in got
    assert len(got) == 3


def test_arrow_rows_and_diagonal():
    # GreenDownRight(bit7) + YellowLeft(bit8) + RedRight(bit18)
    mask = (1 << 7) | (1 << 8) | (1 << 18)
    got = elements_for("off", mask, "vehicle")
    assert (COLOR_GREEN, SHAPE_DOWN_RIGHT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_AMBER, SHAPE_LEFT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert (COLOR_RED, SHAPE_RIGHT, STATUS_SOLID_ON, CONFIDENCE) in got
    assert len(got) == 3


def test_user_bits_ignored():
    # User-defined bits (>=24) fall outside the 0-7 direction range the
    # row loop iterates, so they are never converted to elements.
    assert elements_for("off", 1 << 24, "vehicle") == []


# --- relation grouping ---


def _mini_osm():
    return os.path.join(os.path.dirname(__file__), "fixtures", "mini_map.osm")


def test_build_relation_to_ways():
    rel2ways = build_relation_to_ways(_mini_osm())
    assert rel2ways == {901: [100, 101], 902: [102]}


def test_assemble_groups_representative_first_resolvable():
    rel2ways = {901: [100, 101], 902: [102]}
    # way 100 is absent from CARLA; 101 acts as representative for 901.
    states = {101: ("red", 0, "vehicle"), 102: ("green", 0, "vehicle")}
    groups = assemble_groups(rel2ways, states)
    by_id = {gid: (rep, elems) for gid, rep, elems in groups}
    assert set(by_id) == {901, 902}
    assert by_id[901] == (101, [(COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)])
    assert by_id[902] == (102, [(COLOR_GREEN, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)])


def test_assemble_groups_skips_unresolvable_relation():
    groups = assemble_groups({901: [100]}, {})
    assert groups == []


def test_assemble_groups_way_id_mode():
    states = {101: ("red", 0, "vehicle")}
    groups = assemble_groups({901: [101]}, states, id_mode="way")
    assert groups == [(101, 101, [(COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)])]


def test_assemble_groups_way_id_mode_dedups_shared_way():
    # One physical light referred by two regulatory elements -> one group.
    states = {101: ("red", 0, "vehicle")}
    groups = assemble_groups({901: [101], 903: [101]}, states, id_mode="way")
    assert groups == [(101, 101, [(COLOR_RED, SHAPE_CIRCLE, STATUS_SOLID_ON, CONFIDENCE)])]


# --- predictions (Phase 2) ---


def test_predict_states_walks_carla_cycle():
    # CARLA cycle: green -> yellow -> red -> green. green=10 yellow=3 red=10.
    # Now: green, elapsed 3s -> remaining 7s.
    got = predict_states("green", elapsed=3.0, green_t=10.0, yellow_t=3.0,
                         red_t=10.0, steps=3)
    assert got == [(7.0, "yellow"), (10.0, "red"), (20.0, "green")]


def test_predict_states_from_red():
    got = predict_states("red", elapsed=9.0, green_t=10.0, yellow_t=3.0,
                         red_t=10.0, steps=2)
    assert got == [(1.0, "green"), (11.0, "yellow")]


def test_predict_states_off_returns_empty():
    assert predict_states("off", 0.0, 10.0, 3.0, 10.0, 6) == []


def test_predict_states_steps_zero_returns_empty():
    assert predict_states("green", 0.0, 10.0, 3.0, 10.0, 0) == []


def test_predict_states_from_yellow():
    # yellow elapsed=1s of 3s -> remaining 2s -> red, then green
    got = predict_states("yellow", elapsed=1.0, green_t=10.0, yellow_t=3.0,
                         red_t=10.0, steps=2)
    assert got == [(2.0, "red"), (12.0, "green")]


def test_predict_states_zero_duration_phase_returns_empty():
    assert predict_states("green", 0.0, 10.0, 0.0, 10.0, 3) == []


def test_predict_states_negative_first_offset_passes_through():
    # elapsed overshoots the phase: first offset is negative; the caller
    # (node.py) clamps it -- the pure function reports the raw walk.
    got = predict_states("green", elapsed=11.0, green_t=10.0, yellow_t=3.0,
                         red_t=10.0, steps=1)
    assert got == [(-1.0, "yellow")]

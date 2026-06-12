"""Tests for beacon_select (pure, no unreal dependency)."""
from lanelet2_traffic_light.frontend_editor.beacon_select import (
    beacon_element_indices,
)


def test_finds_single_orange_element():
    names = ["Yellow_Metal", "Orange_Light"]
    assert beacon_element_indices(names, "Orange_Light") == [1]


def test_finds_multiple_elements_in_order():
    names = ["Orange_Light", "Yellow_Metal", "Orange_Light_2"]
    assert beacon_element_indices(names, "Orange_Light") == [0, 2]


def test_marker_is_substring_match():
    # Post-apply detection: slots hold M_JPFlashingBeaconLit (or its MID name).
    names = ["Yellow_Metal", "M_JPFlashingBeaconLit"]
    assert beacon_element_indices(names, "JPFlashingBeacon") == [1]


def test_none_material_names_are_skipped():
    # Unassigned slots surface as None from the editor wrapper.
    names = [None, "Orange_Light"]
    assert beacon_element_indices(names, "Orange_Light") == [1]


def test_no_match_returns_empty():
    assert beacon_element_indices(["Red", "Green"], "Orange_Light") == []

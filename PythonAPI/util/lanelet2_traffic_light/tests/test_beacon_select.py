"""Tests for beacon_select (pure, no unreal dependency)."""
from lanelet2_traffic_light.frontend_editor.beacon_select import (
    beacon_element_indices,
    largest_gap,
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


def test_largest_gap_two_clusters():
    # Two lamp discs: vertices cluster at ~0.1-0.2 and ~0.8-0.9.
    gap, mid = largest_gap([0.1, 0.15, 0.2, 0.8, 0.85, 0.9])
    assert abs(gap - 0.6) < 1e-9
    assert abs(mid - 0.5) < 1e-9


def test_largest_gap_single_cluster_is_small():
    gap, _mid = largest_gap([0.40, 0.45, 0.50, 0.55])
    assert abs(gap - 0.05) < 1e-9


def test_largest_gap_degenerate_inputs():
    assert largest_gap([]) == (0.0, 0.0)
    assert largest_gap([0.7]) == (0.0, 0.7)

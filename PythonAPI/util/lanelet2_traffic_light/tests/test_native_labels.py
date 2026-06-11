"""Signal-head label predicate shared by snap targeting and native cleanup."""
from lanelet2_traffic_light.frontend_editor.native_labels import (
    NON_HEAD_SUBSTRINGS, is_signal_head_label,
)


def test_accepts_plain_lamp_labels():
    prefixes = ("Traffic_Lights", "Pedestrian_Lights")
    assert is_signal_head_label("Traffic_Lights_249", prefixes)
    assert is_signal_head_label("Pedestrian_Lights_107", prefixes)


def test_rejects_pole_and_ground_variants():
    prefixes = ("Traffic_Lights", "Pedestrian_Lights")
    # Pole/arm assemblies and ground cabinets share the prefix but are not
    # signal heads; snapping to them or deleting them broke the Odaiba map.
    assert not is_signal_head_label("Traffic_Lights_Pole_035", prefixes)
    assert not is_signal_head_label("Traffic_Lights_Ground_030", prefixes)
    assert not is_signal_head_label("Pedestrian_Lights_Pole_002", prefixes)


def test_rejects_non_matching_prefix():
    assert not is_signal_head_label("StreetLamp_01", ("Traffic_Lights",))
    assert not is_signal_head_label("", ("Traffic_Lights",))


def test_default_exclusions_are_pole_and_ground():
    assert set(NON_HEAD_SUBSTRINGS) == {"Pole", "Ground"}


def test_custom_exclusions():
    assert not is_signal_head_label(
        "Traffic_Lights_Marker_01", ("Traffic_Lights",),
        exclude_substrings=("Marker",))
    assert is_signal_head_label(
        "Traffic_Lights_Pole_035", ("Traffic_Lights",),
        exclude_substrings=("Marker",))  # explicit list overrides defaults

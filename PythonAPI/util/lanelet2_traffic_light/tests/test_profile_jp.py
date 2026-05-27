import pytest
from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP


def test_profile_has_required_subtypes():
    assert PROFILE_JP.bp_class_for("red_yellow_green") != ""
    assert PROFILE_JP.bp_class_for("red_green") != ""


def test_profile_unknown_subtype_raises():
    with pytest.raises(KeyError):
        PROFILE_JP.bp_class_for("totally_unknown")


def test_profile_defaults():
    assert PROFILE_JP.default_pole_height_m() > 0
    assert isinstance(PROFILE_JP.yaw_offset_deg(), float)
    assert isinstance(PROFILE_JP.front_is_left_to_right(), bool)


def test_pole_height_per_subtype():
    # Values determined from the Z-median of 190 Odaiba placements in Phase 4.2.
    assert PROFILE_JP.pole_height_m("red_yellow_green") == pytest.approx(11.76)
    assert PROFILE_JP.pole_height_m("red_green") == pytest.approx(9.18)


def test_pole_height_unknown_subtype_falls_back_to_default():
    # Unknown subtypes fall back to default_pole_height_m (no KeyError raised).
    assert PROFILE_JP.pole_height_m("totally_unknown") == PROFILE_JP.default_pole_height_m()


def test_parent_bp_path_for_known_subtypes():
    # Phase 5: parent path used for derived BP auto-generation is defined.
    v = PROFILE_JP.parent_bp_path_for("red_yellow_green")
    p = PROFILE_JP.parent_bp_path_for("red_green")
    assert v is not None and "VehicleTrafficLight" in v
    assert p is not None and "PedestrianTrafficLight" in p


def test_parent_bp_path_for_unknown_subtype_returns_none():
    # Unknown subtypes return None (fall back to deriving from C++ TrafficLightBase).
    assert PROFILE_JP.parent_bp_path_for("arrow_left") is None

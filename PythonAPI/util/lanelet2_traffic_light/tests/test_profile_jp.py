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
    # Phase 4.2 で Odaiba 配置 190 件の Z 中央値から確定した値
    assert PROFILE_JP.pole_height_m("red_yellow_green") == pytest.approx(11.76)
    assert PROFILE_JP.pole_height_m("red_green") == pytest.approx(9.18)


def test_pole_height_unknown_subtype_falls_back_to_default():
    # 未知 subtype は default_pole_height_m に縮退する (KeyError は出さない)
    assert PROFILE_JP.pole_height_m("totally_unknown") == PROFILE_JP.default_pole_height_m()


def test_parent_bp_path_for_known_subtypes():
    # Phase 5 ①: 派生 BP 自動生成で使う parent path が定義済み
    v = PROFILE_JP.parent_bp_path_for("red_yellow_green")
    p = PROFILE_JP.parent_bp_path_for("red_green")
    assert v is not None and "VehicleTrafficLight" in v
    assert p is not None and "PedestrianTrafficLight" in p


def test_parent_bp_path_for_unknown_subtype_returns_none():
    # 未知 subtype は None (C++ TrafficLightBase 直派生にフォールバック)
    assert PROFILE_JP.parent_bp_path_for("arrow_left") is None

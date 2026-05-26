"""bp_naming.derive_subtype_bp_path のテスト。pure Python なので unreal 不要。"""
import pytest

from lanelet2_traffic_light.frontend_editor.bp_naming import (
    derive_group_bp_path, derive_subtype_bp_path, role_for_subtype,
)


def test_role_for_subtype():
    assert role_for_subtype("red_yellow_green") == "Vehicle"
    assert role_for_subtype("red_green") == "Pedestrian"


def test_role_unknown_subtype_raises():
    with pytest.raises(KeyError):
        role_for_subtype("arrow_left")


def test_default_path_for_odaiba_vehicle():
    p = derive_subtype_bp_path("red_yellow_green", "Odaiba")
    assert p == "/Game/Carla/Blueprints/Odaiba/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL"


def test_default_path_for_odaiba_pedestrian():
    p = derive_subtype_bp_path("red_green", "Odaiba")
    assert p == "/Game/Carla/Blueprints/Odaiba/BP_OdaibaPedestrianTL.BP_OdaibaPedestrianTL"


def test_default_path_for_other_map():
    p = derive_subtype_bp_path("red_yellow_green", "Chiba")
    assert p == "/Game/Carla/Blueprints/Chiba/BP_ChibaVehicleTL.BP_ChibaVehicleTL"


def test_custom_output_dir():
    p = derive_subtype_bp_path(
        "red_yellow_green", "Odaiba",
        output_dir="/Game/MyMod/TL",
    )
    assert p == "/Game/MyMod/TL/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL"


def test_custom_name_template():
    p = derive_subtype_bp_path(
        "red_green", "TestMap",
        name_template="TL_{role}_{map_name}",
    )
    assert p == "/Game/Carla/Blueprints/TestMap/TL_Pedestrian_TestMap.TL_Pedestrian_TestMap"


def test_group_path_for_odaiba():
    p = derive_group_bp_path("Odaiba")
    assert p == "/Game/Carla/Blueprints/Odaiba/BP_OdaibaTrafficLightGroup.BP_OdaibaTrafficLightGroup"


def test_group_path_for_other_map():
    p = derive_group_bp_path("Chiba")
    assert p == "/Game/Carla/Blueprints/Chiba/BP_ChibaTrafficLightGroup.BP_ChibaTrafficLightGroup"


def test_group_path_custom_dir():
    p = derive_group_bp_path("Odaiba", output_dir="/Game/MyMod/TL")
    assert p == "/Game/MyMod/TL/BP_OdaibaTrafficLightGroup.BP_OdaibaTrafficLightGroup"

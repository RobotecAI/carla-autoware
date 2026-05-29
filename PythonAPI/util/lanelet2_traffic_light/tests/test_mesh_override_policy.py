"""Unit tests for mesh_override_policy."""
from lanelet2_traffic_light.frontend_editor.mesh_override_policy import (
    should_skip_mesh_override,
)


def test_pedestrian_bp_does_not_skip_override():
    # Inc4: pedestrians now adopt the snapped Scene_NNN mesh (natural orientation),
    # figures are driven via runtime material override. Never skip snap-override.
    assert should_skip_mesh_override(
        "/Game/Carla/Blueprints/Odaiba/BP_OdaibaPedestrianTL.BP_OdaibaPedestrianTL"
    ) is False


def test_vehicle_bp_does_not_skip_override():
    assert should_skip_mesh_override(
        "/Game/Carla/Blueprints/Odaiba/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL"
    ) is False


def test_empty_path_does_not_skip():
    assert should_skip_mesh_override("") is False

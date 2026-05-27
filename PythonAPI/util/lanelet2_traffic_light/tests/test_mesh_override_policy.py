"""Unit tests for mesh_override_policy (Phase 8 Increment 1)."""
from lanelet2_traffic_light.frontend_editor.mesh_override_policy import (
    should_skip_mesh_override,
)


def test_pedestrian_bp_skips_override():
    # Pedestrian BPs must keep the parent 3-element canonical mesh.
    assert should_skip_mesh_override(
        "/Game/Carla/Blueprints/Odaiba/BP_OdaibaPedestrianTL.BP_OdaibaPedestrianTL"
    ) is True


def test_vehicle_bp_keeps_override():
    # Vehicle path is unchanged: snap-override stays enabled.
    assert should_skip_mesh_override(
        "/Game/Carla/Blueprints/Odaiba/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL"
    ) is False


def test_pedestrian_match_is_case_sensitive_substring():
    assert should_skip_mesh_override("X/BP_NishishinjukuMapPedestrianTL") is True
    assert should_skip_mesh_override("") is False

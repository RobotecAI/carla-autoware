import pytest

from lanelet2_traffic_light.core.ir.traffic_light_ir import (
    Node, TrafficLightSpec, LightBulbsSpec, GroupSpec, PlacementSpec,
)


def test_node_fields():
    n = Node(id=1, lat=35.0, lon=139.0, local_x=89430.7, local_y=43191.1,
             mgrs_code="54SUE894431")
    assert n.id == 1
    assert n.mgrs_code == "54SUE894431"


def test_traffic_light_spec_immutable():
    n0 = Node(1, 35.0, 139.0, 0.0, 0.0, "54SUE")
    n1 = Node(2, 35.0, 139.0, 1.0, 0.0, "54SUE")
    spec = TrafficLightSpec(way_id=6621, subtype="red_yellow_green",
                            p0=n0, p1=n1, height=0.45, raw_tags={})
    with pytest.raises(Exception):
        spec.way_id = 9999      # frozen


def test_placement_spec_optional_group():
    p = PlacementSpec(sign_id="6621", actor_class_path="/Game/Foo.Foo_C",
                      location_cm=(0,0,0), rotation_deg=(0,0,0))
    assert p.group_relation_id is None
    assert p.source_way_id == 0

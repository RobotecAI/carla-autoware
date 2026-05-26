import pytest

from lanelet2_traffic_light.corelib.ir.traffic_light_ir import (
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


def test_node_has_ele_field_default_none():
    from lanelet2_traffic_light.corelib.ir.traffic_light_ir import Node
    n = Node(id=1, lat=35.0, lon=139.0, local_x=0.0, local_y=0.0, mgrs_code="54SUE")
    assert n.ele is None


def test_node_accepts_explicit_ele():
    from lanelet2_traffic_light.corelib.ir.traffic_light_ir import Node
    n = Node(id=1, lat=35.0, lon=139.0, local_x=0.0, local_y=0.0, mgrs_code="54SUE", ele=6.083)
    assert n.ele == 6.083


def test_placement_spec_has_subtype_field_default_empty():
    from lanelet2_traffic_light.corelib.ir.traffic_light_ir import PlacementSpec
    p = PlacementSpec(
        sign_id="1",
        actor_class_path="/Game/foo",
        location_cm=(0.0, 0.0, 0.0),
        rotation_deg=(0.0, 0.0, 0.0),
    )
    assert p.subtype == ""


def test_placement_spec_has_lanelet2_fields_defaults():
    from lanelet2_traffic_light.corelib.ir.traffic_light_ir import PlacementSpec
    p = PlacementSpec(
        sign_id="1",
        actor_class_path="/Game/foo",
        location_cm=(0.0, 0.0, 0.0),
        rotation_deg=(0.0, 0.0, 0.0),
    )
    assert p.lat == 0.0
    assert p.lon == 0.0
    assert p.ele is None
    assert p.local_x == 0.0
    assert p.local_y == 0.0
    assert p.mgrs_code == ""


def test_placement_spec_accepts_lanelet2_fields():
    from lanelet2_traffic_light.corelib.ir.traffic_light_ir import PlacementSpec
    p = PlacementSpec(
        sign_id="1",
        actor_class_path="/Game/foo",
        location_cm=(0.0, 0.0, 0.0),
        rotation_deg=(0.0, 0.0, 0.0),
        subtype="red_yellow_green",
        lat=35.62,
        lon=139.77,
        ele=6.083,
        local_x=89430.77,
        local_y=43191.12,
        mgrs_code="54SUE894431",
    )
    assert p.subtype == "red_yellow_green"
    assert p.lat == 35.62
    assert p.lon == 139.77
    assert p.ele == 6.083
    assert p.local_x == 89430.77
    assert p.local_y == 43191.12
    assert p.mgrs_code == "54SUE894431"

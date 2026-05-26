from lanelet2_traffic_light.corelib.ir.traffic_light_ir import Node, TrafficLightSpec
from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver


def _spec(way_id):
    n0 = Node(1, 0, 0, 0, 0, "")
    n1 = Node(2, 0, 0, 0, 0, "")
    return TrafficLightSpec(way_id, "red_yellow_green", n0, n1, 0.45, {})


def test_way_id_resolver_returns_str():
    r = WayIdResolver()
    assert r.resolve(_spec(6621), None) == "6621"


def test_way_id_resolver_independent_of_group():
    r = WayIdResolver()
    assert r.resolve(_spec(42), None) == "42"

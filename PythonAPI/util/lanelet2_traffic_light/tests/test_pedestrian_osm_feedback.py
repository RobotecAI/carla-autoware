import math
from lanelet2_traffic_light.corelib.osm_feedback import pedestrian_osm_feedback as pf
from lanelet2_traffic_light.corelib.ir.traffic_light_ir import TrafficLightSpec, Node
from lanelet2_traffic_light.corelib.geometry.pose_estimator import estimate_center_and_yaw


def test_output_path():
    assert pf.derive_output_path("/a/b/Map.osm") == "/a/b/Map.pedestrianSignalAdded.osm"
    assert pf.derive_output_path("/a/b/c.OSM") == "/a/b/c.pedestrianSignalAdded.osm"


def test_segment_recovers_yaw():
    cx, cy = 100.0, 200.0
    placed_yaw = 37.0
    yaw_offset = -90.0
    (x0, y0), (x1, y1) = pf.make_segment_endpoints(cx, cy, placed_yaw, yaw_offset, length_m=0.5)
    spec = TrafficLightSpec(
        way_id=1, subtype="red_green",
        p0=Node(1, 0, 0, x0, y0, "M", 0.0),
        p1=Node(2, 0, 0, x1, y1, "M", 0.0),
        height=0.0)
    _cx, _cy, yaw = estimate_center_and_yaw(spec, yaw_offset)
    assert abs(_cx - cx) < 1e-6 and abs(_cy - cy) < 1e-6
    d = (yaw - placed_yaw) % 360.0
    assert min(d, 360.0 - d) < 1e-4


def test_estimate_ped_ele():
    veh = [((10.0, 10.0), 44.0), ((100.0, 100.0), 46.0)]  # ((local_x, local_y), ele)
    ele = pf.estimate_ped_ele((11.0, 11.0), veh, diff_m=2.58)
    assert abs(ele - (44.0 - 2.58)) < 1e-6


def test_parse_existing_max_id_and_nodes():
    osm = (
        '<?xml version="1.0"?><osm version="0.6">'
        '<node id="5" lat="35.0" lon="139.0"><tag k="local_x" v="10.0"/>'
        '<tag k="local_y" v="20.0"/><tag k="ele" v="40.0"/><tag k="mgrs_code" v="X"/></node>'
        '<way id="7"><nd ref="5"/><nd ref="5"/><tag k="type" v="traffic_light"/>'
        '<tag k="subtype" v="red_yellow_green"/></way>'
        '<relation id="9"><tag k="type" v="regulatory_element"/></relation>'
        '</osm>')
    info = pf.parse_existing(osm)
    assert info.max_id == 9
    assert len(info.nodes) == 1
    assert info.vehicle_tl_nodes == [((10.0, 20.0), 40.0)]


def test_build_and_insert_well_formed():
    import xml.etree.ElementTree as ET
    osm = '<osm version="0.6"><node id="1" lat="0" lon="0"/></osm>'
    signals = [pf.PedSignal(local_x=10.0, local_y=20.0, ele=41.0, placed_yaw_deg=0.0,
                            lat=35.0, lon=139.0, mgrs_code="X")]
    out = pf.build_feedback_osm(osm, signals, id_base=10_000_000, yaw_offset_deg=-90.0,
                                segment_length_m=0.5, subtype="red_green")
    root = ET.fromstring(out)
    ways = [w for w in root.findall("way")
            if any(t.get("k") == "type" and t.get("v") == "traffic_light" for t in w.findall("tag"))]
    assert len(ways) == 1
    assert any(t.get("k") == "subtype" and t.get("v") == "red_green"
               for t in ways[0].findall("tag"))
    rels = root.findall("relation")
    assert len(rels) == 1

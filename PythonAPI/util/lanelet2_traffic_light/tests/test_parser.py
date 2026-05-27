import pytest
from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm


def test_parse_returns_traffic_lights_and_groups(minimal_osm_path):
    tls, groups = parse_osm(minimal_osm_path)
    assert len(tls) == 3
    assert len(groups) == 1


def test_traffic_light_fields(minimal_osm_path):
    tls, _ = parse_osm(minimal_osm_path)
    by_id = {t.way_id: t for t in tls}
    assert 1001 in by_id
    assert by_id[1001].subtype == "red_yellow_green"
    assert by_id[1001].height == pytest.approx(0.45)
    assert by_id[1001].p0.local_x == pytest.approx(100.0)
    assert by_id[1001].p1.local_x == pytest.approx(101.0)
    assert by_id[1003].subtype == "red_green"


def test_group_refers(minimal_osm_path):
    _, groups = parse_osm(minimal_osm_path)
    g = groups[0]
    assert g.relation_id == 2001
    assert set(g.refers) == {1001, 1002}


def test_file_not_found():
    with pytest.raises(FileNotFoundError):
        parse_osm("/nonexistent/path.osm")


def test_skip_way_with_less_than_two_nodes(tmp_path):
    p = tmp_path / "short_way.osm"
    p.write_text("""<?xml version='1.0'?>
<osm version='0.6'>
  <node id='1' lat='0' lon='0'><tag k='local_x' v='0'/><tag k='local_y' v='0'/></node>
  <way id='100'>
    <nd ref='1'/>
    <tag k='type' v='traffic_light'/>
    <tag k='subtype' v='red_yellow_green'/>
    <tag k='height' v='0.45'/>
  </way>
</osm>""")
    with pytest.warns(UserWarning, match="needs >=2 nd refs"):
        tls, _ = parse_osm(str(p))
    assert tls == []


def test_skip_way_with_missing_node_ref(tmp_path):
    p = tmp_path / "missing_node.osm"
    p.write_text("""<?xml version='1.0'?>
<osm version='0.6'>
  <node id='1' lat='0' lon='0'><tag k='local_x' v='0'/><tag k='local_y' v='0'/></node>
  <way id='100'>
    <nd ref='1'/>
    <nd ref='9999'/>
    <tag k='type' v='traffic_light'/>
    <tag k='subtype' v='red_yellow_green'/>
    <tag k='height' v='0.45'/>
  </way>
</osm>""")
    with pytest.warns(UserWarning, match="missing node refs"):
        tls, _ = parse_osm(str(p))
    assert tls == []


def test_skip_malformed_node(tmp_path):
    p = tmp_path / "bad_node.osm"
    p.write_text("""<?xml version='1.0'?>
<osm version='0.6'>
  <node id='1' lat='0' lon='0'><tag k='local_x' v='0'/></node>
</osm>""")
    with pytest.warns(UserWarning, match="missing local_x or local_y"):
        tls, _ = parse_osm(str(p))
    assert tls == []


def test_node_reads_ele_tag():
    """A node with an ele tag is read as a float value."""
    import os
    from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
    fixture = os.path.join(os.path.dirname(__file__), "fixtures", "minimal.osm")
    tls, groups = parse_osm(fixture)
    # Check the ele value of the node tagged with ele in the fixture (any TL p0/p1).
    found_ele = False
    for tl in tls:
        for node in (tl.p0, tl.p1):
            if node.ele is not None:
                assert isinstance(node.ele, float)
                assert node.ele == 6.083  # must match the fixture value
                found_ele = True
    assert found_ele, "The fixture must contain a node with an ele tag"


def test_node_ele_missing_returns_none():
    """A node without an ele tag returns None for ele."""
    import os
    from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
    fixture = os.path.join(os.path.dirname(__file__), "fixtures", "minimal.osm")
    tls, groups = parse_osm(fixture)
    found_none = False
    for tl in tls:
        for node in (tl.p0, tl.p1):
            if node.ele is None:
                found_none = True
    assert found_none, "The fixture must contain a node without an ele tag"

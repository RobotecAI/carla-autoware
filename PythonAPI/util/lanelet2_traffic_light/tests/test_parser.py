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


from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_arrow_bulbs


@pytest.fixture
def arrows_osm_path():
    import os
    return os.path.join(os.path.dirname(__file__), "fixtures", "arrows.osm")


def test_parse_arrow_bulbs_extracts_color_and_direction(arrows_osm_path):
    bulbs = parse_arrow_bulbs(arrows_osm_path)
    # 1001 has green-right, green-up(->straight); red bulb has no arrow (excluded)
    assert bulbs[1001] == frozenset({("green", "right"), ("green", "straight")})


def test_parse_arrow_bulbs_no_arrow_tl_absent_or_empty(arrows_osm_path):
    bulbs = parse_arrow_bulbs(arrows_osm_path)
    # 1002 light_bulbs way has no arrow bulbs -> empty set (or key absent)
    assert bulbs.get(1002, frozenset()) == frozenset()


def test_parse_arrow_bulbs_warns_on_unknown_direction(arrows_osm_path):
    # lower_left (fixture node 14) is a genuinely unknown direction;
    # the reserved 'down' has its own dedicated test below.
    with pytest.warns(UserWarning, match="unknown arrow direction 'lower_left'"):
        parse_arrow_bulbs(arrows_osm_path)


def test_parse_arrow_bulbs_accepts_diagonal_directions(arrows_osm_path):
    bulbs = parse_arrow_bulbs(arrows_osm_path)
    # 1003 has green up_left / down_right / literal "straight" (new Odaiba map
    # vocabulary, 2026-06 update); 'down' is a reserved slot -> skipped.
    assert bulbs[1003] == frozenset(
        {("green", "up_left"), ("green", "down_right"), ("green", "straight")}
    )


def test_parse_arrow_bulbs_skips_reserved_down(arrows_osm_path):
    with pytest.warns(UserWarning, match="unknown arrow direction 'down'"):
        parse_arrow_bulbs(arrows_osm_path)


def test_parse_arrow_bulbs_file_not_found():
    with pytest.raises(FileNotFoundError):
        parse_arrow_bulbs("/nonexistent/x.osm")

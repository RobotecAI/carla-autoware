import os

import pytest
from lanelet2_traffic_light.corelib.api import generate_placements, GenerationReport
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver


def test_generate_placements_from_minimal(minimal_osm_path):
    transformer = MgrsTransformer(0.0, 0.0, 0.0, x_sign=+1, y_sign=-1)
    placements, groups, report = generate_placements(
        osm_path=minimal_osm_path,
        profile=PROFILE_JP,
        sign_id_resolver=WayIdResolver(),
        transformer=transformer,
    )
    assert len(placements) == 3
    assert len(groups) == 1
    sign_ids = {p.sign_id for p in placements}
    assert sign_ids == {"1001", "1002", "1003"}
    assert report.parsed_traffic_lights == 3
    assert report.parsed_groups == 1
    assert report.placements_created == 3


def test_z_uses_subtype_pole_height(minimal_osm_path):
    """Phase 4.2: 各 placement の Z は profile.pole_height_m(subtype) を反映する。"""
    transformer = MgrsTransformer(0.0, 0.0, 0.0, x_sign=+1, y_sign=-1)
    placements, _, _ = generate_placements(
        osm_path=minimal_osm_path,
        profile=PROFILE_JP,
        sign_id_resolver=WayIdResolver(),
        transformer=transformer,
    )
    # minimal フィクスチャの subtype は parser/fixture 側に依存するが、いずれにせよ
    # subtype 別 pole_height が反映されていることが重要。
    # 車両用と歩行者用が混在していれば Z 値が複数現れる、単一なら 1 種類。
    zs = {round(p.location_cm[2], 2) for p in placements}
    # subtype 別の pole_height (cm) のいずれかに完全一致するはず
    expected_zs = {
        PROFILE_JP.pole_height_m("red_yellow_green") * 100.0,
        PROFILE_JP.pole_height_m("red_green") * 100.0,
    }
    expected_zs = {round(z, 2) for z in expected_zs}
    assert zs <= expected_zs, f"unexpected Z values: {zs - expected_zs}"


def test_generate_placements_collision_detected(minimal_osm_path):
    class CollidingResolver:
        def resolve(self, tl, group): return "FIXED"
    transformer = MgrsTransformer(0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="SignID collision"):
        generate_placements(
            osm_path=minimal_osm_path,
            profile=PROFILE_JP,
            sign_id_resolver=CollidingResolver(),
            transformer=transformer,
        )


def test_unknown_subtype_skipped(tmp_path):
    # 未知 subtype の way だけのフィクスチャを動的生成
    p = tmp_path / "unknown.osm"
    p.write_text("""<?xml version='1.0'?>
<osm version='0.6'>
  <node id='1' lat='0' lon='0'><tag k='local_x' v='0'/><tag k='local_y' v='0'/></node>
  <node id='2' lat='0' lon='0'><tag k='local_x' v='1'/><tag k='local_y' v='0'/></node>
  <way id='100'>
    <nd ref='1'/><nd ref='2'/>
    <tag k='type' v='traffic_light'/>
    <tag k='subtype' v='arrow_left'/>
    <tag k='height' v='0.45'/>
  </way>
</osm>""")
    transformer = MgrsTransformer(0.0, 0.0, 0.0)
    placements, _, report = generate_placements(
        osm_path=str(p),
        profile=PROFILE_JP,
        sign_id_resolver=WayIdResolver(),
        transformer=transformer,
    )
    assert len(placements) == 0
    assert len(report.placements_skipped) == 1
    assert report.placements_skipped[0][0] == 100   # way_id
    assert "subtype" in report.placements_skipped[0][1]
    assert len(report.warnings) == 1


def test_placement_carries_subtype_from_traffic_light_spec():
    """PlacementSpec.subtype が TrafficLightSpec.subtype と一致する。"""
    from lanelet2_traffic_light.corelib.api import generate_placements
    from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
    from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
    from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
    import os
    fixture = os.path.join(os.path.dirname(__file__), "fixtures", "minimal.osm")
    transformer = MgrsTransformer(0.0, 0.0, 0.0)
    placements, _, _ = generate_placements(fixture, PROFILE_JP, WayIdResolver(), transformer)
    assert placements, "minimal.osm に traffic_light way が必要"
    for p in placements:
        assert p.subtype in ("red_yellow_green", "red_green"), \
            f"unexpected subtype: {p.subtype}"


def test_placement_carries_lat_lon_midpoint():
    """PlacementSpec.lat/lon が TL の p0/p1 中点。"""
    from lanelet2_traffic_light.corelib.api import generate_placements
    from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
    from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
    from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
    from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
    import os
    fixture = os.path.join(os.path.dirname(__file__), "fixtures", "minimal.osm")
    tls, _ = parse_osm(fixture)
    transformer = MgrsTransformer(0.0, 0.0, 0.0)
    placements, _, _ = generate_placements(fixture, PROFILE_JP, WayIdResolver(), transformer)
    tl_by_wayid = {tl.way_id: tl for tl in tls}
    for p in placements:
        tl = tl_by_wayid[p.source_way_id]
        expected_lat = (tl.p0.lat + tl.p1.lat) / 2.0
        expected_lon = (tl.p0.lon + tl.p1.lon) / 2.0
        assert abs(p.lat - expected_lat) < 1e-9
        assert abs(p.lon - expected_lon) < 1e-9


def test_placement_carries_local_xy_midpoint():
    from lanelet2_traffic_light.corelib.api import generate_placements
    from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
    from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
    from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
    from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
    import os
    fixture = os.path.join(os.path.dirname(__file__), "fixtures", "minimal.osm")
    tls, _ = parse_osm(fixture)
    transformer = MgrsTransformer(0.0, 0.0, 0.0)
    placements, _, _ = generate_placements(fixture, PROFILE_JP, WayIdResolver(), transformer)
    tl_by_wayid = {tl.way_id: tl for tl in tls}
    for p in placements:
        tl = tl_by_wayid[p.source_way_id]
        expected_x = (tl.p0.local_x + tl.p1.local_x) / 2.0
        expected_y = (tl.p0.local_y + tl.p1.local_y) / 2.0
        assert abs(p.local_x - expected_x) < 1e-6
        assert abs(p.local_y - expected_y) < 1e-6


def test_placement_carries_mgrs_code_from_p0():
    from lanelet2_traffic_light.corelib.api import generate_placements
    from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
    from lanelet2_traffic_light.corelib.profile.profile_jp import PROFILE_JP
    from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
    from lanelet2_traffic_light.corelib.sign_id.way_id_resolver import WayIdResolver
    import os
    fixture = os.path.join(os.path.dirname(__file__), "fixtures", "minimal.osm")
    tls, _ = parse_osm(fixture)
    transformer = MgrsTransformer(0.0, 0.0, 0.0)
    placements, _, _ = generate_placements(fixture, PROFILE_JP, WayIdResolver(), transformer)
    tl_by_wayid = {tl.way_id: tl for tl in tls}
    for p in placements:
        tl = tl_by_wayid[p.source_way_id]
        assert p.mgrs_code == tl.p0.mgrs_code


def test_placement_ele_midpoint_both_present():
    """ele が両端にあれば中点。pure-Python での中点計算ロジック検証。"""
    from lanelet2_traffic_light.corelib.api import _compute_ele_midpoint
    assert _compute_ele_midpoint(6.0, 8.0) == 7.0


def test_placement_ele_p0_only():
    from lanelet2_traffic_light.corelib.api import _compute_ele_midpoint
    assert _compute_ele_midpoint(6.0, None) == 6.0


def test_placement_ele_p1_only():
    from lanelet2_traffic_light.corelib.api import _compute_ele_midpoint
    assert _compute_ele_midpoint(None, 8.0) == 8.0


def test_placement_ele_both_none():
    from lanelet2_traffic_light.corelib.api import _compute_ele_midpoint
    assert _compute_ele_midpoint(None, None) is None


REAL_OSM = "/mnt/dsk0/wk0/CARLA/autoware_map/odaiba_autoware_map_2025_01_16/lanelet2_map.osm"


@pytest.mark.skipif(not os.path.exists(REAL_OSM),
                    reason="real Odaiba lanelet2 not available in CI")
def test_real_odaiba_osm_parses_and_generates():
    """実 Odaiba .osm が解析でき、それなりの件数の PlacementSpec が返ること。"""
    transformer = MgrsTransformer(0.0, 0.0, 0.0, x_sign=+1, y_sign=-1)
    placements, groups, report = generate_placements(
        osm_path=REAL_OSM,
        profile=PROFILE_JP,
        sign_id_resolver=WayIdResolver(),
        transformer=transformer,
    )
    # 件数の桁感だけアサート（実値は phase0_validation.md と突合）
    assert report.parsed_traffic_lights > 10
    assert report.parsed_groups > 0
    assert len(placements) <= report.parsed_traffic_lights
    # SignID 集合が一意
    assert len({p.sign_id for p in placements}) == len(placements)
    # 座標範囲が異常でないこと（Odaiba の半径 km スケール想定）
    for p in placements:
        x, y, z = p.location_cm
        assert -1e8 < x < 1e8
        assert -1e8 < y < 1e8

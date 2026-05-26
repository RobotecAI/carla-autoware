"""subtype_splitter.split_members_by_subtype のユニットテスト。

editor_placer._place_groups の subtype 別 member 分割ロジックを pure-Python
として切り出したもの。Editor 依存無しで unit test 可能。
"""


def test_split_empty_refers_returns_empty_dict():
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(refers=[], sign_id_to_actor={}, sign_id_to_subtype={})
    assert result == {}


def test_split_skips_actors_not_in_map():
    """sign_id_to_actor に存在しない way は無視される。"""
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(
        refers=[100, 200, 300],
        sign_id_to_actor={"100": "actorA", "300": "actorC"},  # 200 が無い
        sign_id_to_subtype={"100": "red_yellow_green", "200": "red_green", "300": "red_yellow_green"},
    )
    assert result == {"red_yellow_green": ["actorA", "actorC"]}


def test_split_groups_vehicle_only():
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(
        refers=[1, 2, 3],
        sign_id_to_actor={"1": "A", "2": "B", "3": "C"},
        sign_id_to_subtype={"1": "red_yellow_green", "2": "red_yellow_green", "3": "red_yellow_green"},
    )
    assert result == {"red_yellow_green": ["A", "B", "C"]}


def test_split_groups_pedestrian_only():
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(
        refers=[1, 2],
        sign_id_to_actor={"1": "A", "2": "B"},
        sign_id_to_subtype={"1": "red_green", "2": "red_green"},
    )
    assert result == {"red_green": ["A", "B"]}


def test_split_groups_mixed_subtypes():
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(
        refers=[1, 2, 3, 4],
        sign_id_to_actor={"1": "Av", "2": "Bp", "3": "Cv", "4": "Dp"},
        sign_id_to_subtype={
            "1": "red_yellow_green",
            "2": "red_green",
            "3": "red_yellow_green",
            "4": "red_green",
        },
    )
    assert result == {
        "red_yellow_green": ["Av", "Cv"],
        "red_green": ["Bp", "Dp"],
    }


def test_split_unknown_subtype_grouped_separately():
    """未知 subtype (空文字含む) は独立した群として残す。"""
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(
        refers=[1, 2],
        sign_id_to_actor={"1": "A", "2": "B"},
        sign_id_to_subtype={"1": "red_yellow_green", "2": ""},  # 2 は subtype 不明
    )
    assert result == {
        "red_yellow_green": ["A"],
        "": ["B"],
    }


def test_split_preserves_order_within_group():
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype
    result = split_members_by_subtype(
        refers=[3, 1, 2],
        sign_id_to_actor={"1": "A", "2": "B", "3": "C"},
        sign_id_to_subtype={"1": "red_yellow_green", "2": "red_yellow_green", "3": "red_yellow_green"},
    )
    assert result == {"red_yellow_green": ["C", "A", "B"]}

"""material_stats.aggregate_material_stats のユニットテスト。

editor_placer._collect_pedestrian_mesh_materials から切り出した pure-Python
集計関数。unreal 依存無しで unit test 可能にするのが目的。
"""


def test_aggregate_empty_input_returns_empty():
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    assert aggregate_material_stats([]) == []


def test_aggregate_single_actor():
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    rows = aggregate_material_stats([
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
    ])
    assert rows == [
        {
            "mesh": "Scene_805",
            "num_elements": 2,
            "elements": ("TrafficLightsWalk", "Frame"),
            "count": 1,
        },
    ]


def test_aggregate_groups_by_mesh_and_elements():
    """同じ (mesh, elements) は count をインクリメント。"""
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    rows = aggregate_material_stats([
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
        ("Scene_812", ("TrafficLightsWalk", "Frame", "TrafficLightsStop")),
    ])
    # 順序: count 降順、tie は mesh アルファベット順
    assert rows == [
        {
            "mesh": "Scene_805",
            "num_elements": 2,
            "elements": ("TrafficLightsWalk", "Frame"),
            "count": 3,
        },
        {
            "mesh": "Scene_812",
            "num_elements": 3,
            "elements": ("TrafficLightsWalk", "Frame", "TrafficLightsStop"),
            "count": 1,
        },
    ]


def test_aggregate_distinguishes_element_order():
    """同じ mesh でも element 順序が違えば別エントリ。"""
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    rows = aggregate_material_stats([
        ("Scene_999", ("A", "B")),
        ("Scene_999", ("B", "A")),
    ])
    # 別エントリとして count=1 が 2 件
    counts = sorted(r["count"] for r in rows)
    assert counts == [1, 1]
    elements = sorted([r["elements"] for r in rows])
    assert elements == [("A", "B"), ("B", "A")]


def test_aggregate_same_mesh_different_meshes_separate():
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    rows = aggregate_material_stats([
        ("Scene_A", ("X",)),
        ("Scene_B", ("X",)),
    ])
    # 同じ elements でも mesh が違えば別エントリ
    meshes = sorted(r["mesh"] for r in rows)
    assert meshes == ["Scene_A", "Scene_B"]

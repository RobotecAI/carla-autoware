"""Unit tests for material_stats.aggregate_material_stats.

A pure-Python aggregation function extracted from
editor_placer._collect_pedestrian_mesh_materials so it can be unit-tested
without any Unreal dependency.
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
    """Identical (mesh, elements) pairs increment the count."""
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    rows = aggregate_material_stats([
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
        ("Scene_805", ("TrafficLightsWalk", "Frame")),
        ("Scene_812", ("TrafficLightsWalk", "Frame", "TrafficLightsStop")),
    ])
    # Order: count descending; ties broken by mesh name alphabetically.
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
    """Same mesh with different element order produces separate entries."""
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats
    rows = aggregate_material_stats([
        ("Scene_999", ("A", "B")),
        ("Scene_999", ("B", "A")),
    ])
    # Two separate entries each with count=1.
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
    # Even with identical elements, different meshes are separate entries.
    meshes = sorted(r["mesh"] for r in rows)
    assert meshes == ["Scene_A", "Scene_B"]

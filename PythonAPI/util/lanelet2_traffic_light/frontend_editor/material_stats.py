"""歩行者用メッシュの Material 構成集計 (Phase 6 課題 D)。

editor_placer._collect_pedestrian_mesh_materials から切り出した pure-Python
集計関数。unreal 依存無しで unit test 可能にするのが目的。
"""
from collections import Counter


def aggregate_material_stats(actor_records):
    """(mesh_name, elements_tuple) を集計し、頻度順のレコードリストを返す。

    Args:
        actor_records: [(mesh_name: str, elements: tuple[str, ...]), ...]
            各 entry は 1 actor に対応。mesh_name は StaticMesh アセット名
            (例 "Scene_805")、elements は Material element 名の tuple (順序保持)。

    Returns:
        [
            {"mesh": str, "num_elements": int, "elements": tuple, "count": int},
            ...
        ]
        順序: count 降順、tie は mesh アルファベット順 + elements 辞書順。
    """
    counter = Counter(actor_records)
    rows = []
    for (mesh, elements), count in counter.items():
        rows.append({
            "mesh": mesh,
            "num_elements": len(elements),
            "elements": elements,
            "count": count,
        })
    rows.sort(key=lambda r: (-r["count"], r["mesh"], r["elements"]))
    return rows

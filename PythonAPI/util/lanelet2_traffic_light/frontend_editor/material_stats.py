"""Aggregation of Material composition for pedestrian meshes (Phase 6 issue D).

Pure-Python aggregation function extracted from editor_placer._collect_pedestrian_mesh_materials.
Designed to be unit-testable without the unreal dependency.
"""
from collections import Counter


def aggregate_material_stats(actor_records):
    """Aggregate (mesh_name, elements_tuple) entries and return a frequency-sorted list of records.

    Args:
        actor_records: [(mesh_name: str, elements: tuple[str, ...]), ...]
            Each entry corresponds to one actor. mesh_name is the StaticMesh asset name
            (e.g. "Scene_805"); elements is an order-preserving tuple of Material element names.

    Returns:
        [
            {"mesh": str, "num_elements": int, "elements": tuple, "count": int},
            ...
        ]
        Order: descending by count; ties broken by mesh alphabetical order then elements lexicographic order.
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

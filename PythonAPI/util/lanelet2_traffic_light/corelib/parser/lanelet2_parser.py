"""lanelet2(.osm) を解析して中間表現に変換するモジュール。"""
import os
import warnings
import xml.etree.ElementTree as ET

from lanelet2_traffic_light.corelib.ir.traffic_light_ir import (
    Node, TrafficLightSpec, GroupSpec,
)


def _read_node(node_elem: ET.Element) -> Node:
    """node 要素 → Node IR。local_x/local_y は必須、欠落時 KeyError。"""
    tags = {t.get("k"): t.get("v") for t in node_elem.findall("tag")}
    if "local_x" not in tags or "local_y" not in tags:
        raise KeyError(
            f"node id={node_elem.get('id')} missing local_x or local_y tag"
        )
    return Node(
        id=int(node_elem.get("id")),
        lat=float(node_elem.get("lat")),
        lon=float(node_elem.get("lon")),
        local_x=float(tags["local_x"]),
        local_y=float(tags["local_y"]),
        mgrs_code=tags.get("mgrs_code", ""),
    )


def parse_osm(osm_path: str) -> tuple[list[TrafficLightSpec], list[GroupSpec]]:
    """lanelet2 .osm を解析。

    Returns:
        (traffic_light_specs, group_specs)

    way 単位の構造欠落（nd refs < 2, 不正な node 参照、座標タグ欠落）は
    当該 way をスキップし `warnings.warn` で警告。
    """
    if not os.path.exists(osm_path):
        raise FileNotFoundError(osm_path)

    tree = ET.parse(osm_path)
    root = tree.getroot()

    nodes: dict[int, Node] = {}
    for n in root.findall("node"):
        try:
            node = _read_node(n)
        except (KeyError, ValueError) as e:
            warnings.warn(f"skip malformed node: {e}", stacklevel=2)
            continue
        nodes[node.id] = node

    traffic_lights: list[TrafficLightSpec] = []
    for way in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        if tags.get("type") != "traffic_light":
            continue
        way_id = int(way.get("id"))
        nd_refs = [int(nd.get("ref")) for nd in way.findall("nd")]
        if len(nd_refs) < 2:
            warnings.warn(
                f"skip traffic_light way={way_id}: needs >=2 nd refs, got {len(nd_refs)}",
                stacklevel=2,
            )
            continue
        if not all(nid in nodes for nid in nd_refs):
            missing = [n for n in nd_refs if n not in nodes]
            warnings.warn(
                f"skip traffic_light way={way_id}: missing node refs {missing}",
                stacklevel=2,
            )
            continue
        try:
            height = float(tags.get("height", "0"))
        except ValueError:
            height = 0.0
        traffic_lights.append(TrafficLightSpec(
            way_id=way_id,
            subtype=tags.get("subtype", "unknown"),
            p0=nodes[nd_refs[0]],
            p1=nodes[nd_refs[-1]],
            height=height,
            raw_tags=tags,
        ))

    groups: list[GroupSpec] = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "traffic_light":
            continue
        refers: list[int] = []
        bulbs: list[int] = []
        ref_line: int | None = None
        for m in rel.findall("member"):
            role = m.get("role")
            ref = int(m.get("ref"))
            if role == "refers":
                refers.append(ref)
            elif role == "light_bulbs":
                bulbs.append(ref)
            elif role == "ref_line":
                ref_line = ref
        groups.append(GroupSpec(
            relation_id=int(rel.get("id")),
            refers=refers,
            light_bulbs=bulbs,
            ref_line=ref_line,
        ))

    return traffic_lights, groups

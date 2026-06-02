"""Module for parsing a lanelet2 (.osm) file and converting it to an intermediate representation."""
import os
import warnings
import xml.etree.ElementTree as ET

from lanelet2_traffic_light.corelib.ir.traffic_light_ir import (
    Node, TrafficLightSpec, GroupSpec,
)


def _read_node(node_elem: ET.Element) -> Node:
    """Convert a node element to a Node IR. local_x/local_y are required; raises KeyError if missing.
    The ele tag is optional (lanelet2 .osm <tag k="ele">; returns None if absent).
    """
    tags = {t.get("k"): t.get("v") for t in node_elem.findall("tag")}
    if "local_x" not in tags or "local_y" not in tags:
        raise KeyError(
            f"node id={node_elem.get('id')} missing local_x or local_y tag"
        )
    ele_str = tags.get("ele")
    ele = float(ele_str) if ele_str is not None else None
    return Node(
        id=int(node_elem.get("id")),
        lat=float(node_elem.get("lat")),
        lon=float(node_elem.get("lon")),
        local_x=float(tags["local_x"]),
        local_y=float(tags["local_y"]),
        mgrs_code=tags.get("mgrs_code", ""),
        ele=ele,
    )


def parse_osm(osm_path: str) -> tuple[list[TrafficLightSpec], list[GroupSpec]]:
    """Parse a lanelet2 .osm file.

    Returns:
        (traffic_light_specs, group_specs)

    Ways with structural defects (nd refs < 2, invalid node references, missing coordinate tags)
    are skipped with a `warnings.warn` warning.
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


_ARROW_DIRECTION_MAP = {"left": "left", "right": "right", "up": "straight"}


def parse_arrow_bulbs(osm_path: str) -> dict[int, frozenset[tuple[str, str]]]:
    """Parse light_bulbs ways into per-traffic_light arrow bulbs.

    A `light_bulbs` way carries a `traffic_light_id` tag (the traffic_light way it
    belongs to) and references bulb nodes via `nd`. Each bulb node may carry a
    `color` (red/yellow/green) and an `arrow` (left/right/up) tag. Only bulbs with
    an `arrow` tag are returned. Color is preserved (not filtered) so a future
    non-green arrow needs no parser change; the lighting side filters to green.
    `arrow=up` normalizes to "straight"; unknown directions are warned and skipped
    (no mesh slot exists for them).

    Returns: {traffic_light_way_id: frozenset[(color, direction)]}.
    """
    if not os.path.exists(osm_path):
        raise FileNotFoundError(osm_path)
    tree = ET.parse(osm_path)
    root = tree.getroot()

    node_tags: dict[int, dict[str, str]] = {}
    for n in root.findall("node"):
        node_tags[int(n.get("id"))] = {t.get("k"): t.get("v") for t in n.findall("tag")}

    result: dict[int, set[tuple[str, str]]] = {}
    for way in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        if tags.get("type") != "light_bulbs":
            continue
        tl_id_raw = tags.get("traffic_light_id")
        if tl_id_raw is None:
            warnings.warn(
                f"skip light_bulbs way={way.get('id')}: no traffic_light_id tag",
                stacklevel=2,
            )
            continue
        tl_id = int(tl_id_raw)
        bucket = result.setdefault(tl_id, set())
        for nd in way.findall("nd"):
            ntags = node_tags.get(int(nd.get("ref")), {})
            raw_arrow = ntags.get("arrow")
            if raw_arrow is None:
                continue
            direction = _ARROW_DIRECTION_MAP.get(raw_arrow)
            if direction is None:
                warnings.warn(
                    f"skip unknown arrow direction '{raw_arrow}' in light_bulbs way={way.get('id')}",
                    stacklevel=2,
                )
                continue
            bucket.add((ntags.get("color", ""), direction))
    return {k: frozenset(v) for k, v in result.items()}

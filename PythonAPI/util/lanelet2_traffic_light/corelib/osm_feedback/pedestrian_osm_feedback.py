"""Pure-logic module for generating pedestrian traffic light (red_green subtype) entries
in a lanelet2 OSM file from UE pedestrian mesh placement data.

No Unreal imports. All geometry is in lanelet2 local coordinates (meters).
"""
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional


# ---------------------------------------------------------------------------
# Output path helper
# ---------------------------------------------------------------------------

def derive_output_path(osm_path: str) -> str:
    """Replace the trailing .osm extension (case-insensitive) with .pedestrianSignalAdded.osm.

    Args:
        osm_path: Absolute or relative path ending with .osm or .OSM etc.

    Returns:
        New path string with the extension replaced.
    """
    return re.sub(r'\.osm$', '.pedestrianSignalAdded.osm', osm_path, flags=re.IGNORECASE)


# ---------------------------------------------------------------------------
# Segment geometry
# ---------------------------------------------------------------------------

def make_segment_endpoints(
    cx: float,
    cy: float,
    placed_yaw_deg: float,
    yaw_offset_deg: float,
    length_m: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Compute the two lanelet2 local-coordinate endpoints of a traffic_light way
    such that estimate_center_and_yaw(spec, yaw_offset_deg) recovers (cx, cy, placed_yaw_deg).

    The formula is derived from the pose_estimator convention:
        way_yaw = atan2(-dy, dx)
        returned_yaw = way_yaw + yaw_offset_deg

    To satisfy returned_yaw == placed_yaw_deg:
        way_yaw_target = placed_yaw_deg - yaw_offset_deg
        dx = cos(way_yaw_target) * length_m
        dy = -sin(way_yaw_target) * length_m   (sign-inversion from -dy in atan2)

    Args:
        cx: Center local_x (m).
        cy: Center local_y (m).
        placed_yaw_deg: Desired UE Yaw angle that estimate_center_and_yaw should recover (degrees).
        yaw_offset_deg: The same offset passed to estimate_center_and_yaw (e.g. -90.0).
        length_m: Total length of the way segment (m).

    Returns:
        ((x0, y0), (x1, y1)) where p0=(x0,y0), p1=(x1,y1).
    """
    way_yaw = math.radians(placed_yaw_deg - yaw_offset_deg)
    dx = math.cos(way_yaw) * length_m
    dy = -math.sin(way_yaw) * length_m
    p0 = (cx - dx / 2.0, cy - dy / 2.0)
    p1 = (cx + dx / 2.0, cy + dy / 2.0)
    return p0, p1


# ---------------------------------------------------------------------------
# Parsed OSM representation
# ---------------------------------------------------------------------------

@dataclass
class ParsedOsm:
    """Result of parsing an existing lanelet2 OSM file."""

    max_id: int
    """Maximum node/way/relation id found in the file."""

    nodes: list[dict]
    """Nodes that have local_x and local_y tags.
    Each dict has keys: id, local_x, local_y, lat, lon, ele (float or None), mgrs_code (str or None).
    """

    vehicle_tl_nodes: list[tuple[tuple[float, float], float]]
    """For each way whose type=traffic_light tag is present, collect the local coordinates and
    elevation of all referenced nodes that have local_x/local_y tags.
    Each entry is ((local_x, local_y), ele) where ele defaults to 0.0 if absent.
    """


def parse_existing(osm_text_or_path: str) -> ParsedOsm:
    """Parse an existing lanelet2 OSM file (given as XML text or file path).

    If the argument contains a newline or '<', it is treated as XML text.
    Otherwise it is treated as a file path.

    Returns:
        ParsedOsm with max_id, nodes list, and vehicle_tl_nodes list.
    """
    if '\n' in osm_text_or_path or '<' in osm_text_or_path:
        root = ET.fromstring(osm_text_or_path)
    else:
        root = ET.parse(osm_text_or_path).getroot()

    max_id = 0

    # Collect all node elements and build id -> node info lookup
    node_map: dict[int, dict] = {}
    for el in root.findall("node"):
        nid = int(el.get("id", 0))
        max_id = max(max_id, nid)

        tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            ele_raw = tags.get("ele")
            node_map[nid] = {
                "id": nid,
                "local_x": float(tags["local_x"]),
                "local_y": float(tags["local_y"]),
                "lat": float(el.get("lat", 0.0)),
                "lon": float(el.get("lon", 0.0)),
                "ele": float(ele_raw) if ele_raw is not None else None,
                "mgrs_code": tags.get("mgrs_code"),
            }

    for el in root.findall("way"):
        wid = int(el.get("id", 0))
        max_id = max(max_id, wid)

    for el in root.findall("relation"):
        rid = int(el.get("id", 0))
        max_id = max(max_id, rid)

    nodes = list(node_map.values())

    # Collect vehicle traffic light nodes (way[type=traffic_light] nd references).
    # Deduplicate by node id so that a way referencing the same node twice does not
    # produce duplicate entries in vehicle_tl_nodes.
    vehicle_tl_nodes: list[tuple[tuple[float, float], float]] = []
    seen_refs: set[int] = set()
    for way in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        if tags.get("type") != "traffic_light":
            continue
        for nd in way.findall("nd"):
            ref = int(nd.get("ref", 0))
            if ref in node_map and ref not in seen_refs:
                seen_refs.add(ref)
                n = node_map[ref]
                ele = n["ele"] if n["ele"] is not None else 0.0
                vehicle_tl_nodes.append(((n["local_x"], n["local_y"]), ele))

    return ParsedOsm(max_id=max_id, nodes=nodes, vehicle_tl_nodes=vehicle_tl_nodes)


# ---------------------------------------------------------------------------
# Elevation estimation
# ---------------------------------------------------------------------------

def estimate_ped_ele(
    ped_local_xy: tuple[float, float],
    vehicle_tl_nodes: list[tuple[tuple[float, float], float]],
    diff_m: float,
) -> float:
    """Estimate pedestrian signal elevation from the nearest vehicle TL node.

    Finds the vehicle TL node whose (local_x, local_y) is closest (Euclidean)
    to ped_local_xy, then returns that node's elevation minus diff_m.

    Args:
        ped_local_xy: (local_x, local_y) of the pedestrian signal center (m).
        vehicle_tl_nodes: List of ((local_x, local_y), ele) for vehicle TL nodes.
        diff_m: Height difference to subtract from the nearest vehicle node's ele.

    Returns:
        Estimated elevation in meters, or float('nan') if vehicle_tl_nodes is empty.
    """
    if not vehicle_tl_nodes:
        return float('nan')

    px, py = ped_local_xy
    best_dist2 = float('inf')
    best_ele = 0.0
    for (vx, vy), ele in vehicle_tl_nodes:
        d2 = (vx - px) ** 2 + (vy - py) ** 2
        if d2 < best_dist2:
            best_dist2 = d2
            best_ele = ele

    return best_ele - diff_m


# ---------------------------------------------------------------------------
# Nearest lat/lon helper
# ---------------------------------------------------------------------------

def nearest_latlon(
    local_xy: tuple[float, float],
    nodes: list[dict],
) -> tuple[float, float, str]:
    """Find the node closest to local_xy and return its (lat, lon, mgrs_code).

    Args:
        local_xy: (local_x, local_y) query point (m).
        nodes: List of node dicts from ParsedOsm.nodes.

    Returns:
        (lat, lon, mgrs_code). Returns (0.0, 0.0, "") if nodes is empty.
    """
    if not nodes:
        return (0.0, 0.0, "")

    px, py = local_xy
    best_dist2 = float('inf')
    best = nodes[0]
    for n in nodes:
        d2 = (n["local_x"] - px) ** 2 + (n["local_y"] - py) ** 2
        if d2 < best_dist2:
            best_dist2 = d2
            best = n

    return (best["lat"], best["lon"], best.get("mgrs_code") or "")


# ---------------------------------------------------------------------------
# Signal data container
# ---------------------------------------------------------------------------

@dataclass
class PedSignal:
    """Data for a single pedestrian signal to be written into the OSM file."""

    local_x: float
    local_y: float
    ele: float
    placed_yaw_deg: float
    lat: float
    lon: float
    mgrs_code: str
    # Absolute placement height (m, same datum as the MgrsTransformer Z offset).
    # Emitted as a per-way "pole_height" tag and honored by api._resolve_pole_height
    # so placement Z does not fall back to the (Odaiba-specific) profile height.
    pole_height: float = None


# ---------------------------------------------------------------------------
# OSM builder
# ---------------------------------------------------------------------------

def _make_tag(k: str, v: str) -> ET.Element:
    t = ET.Element("tag")
    t.set("k", k)
    t.set("v", v)
    return t


def _make_node_el(
    node_id: int,
    lat: float,
    lon: float,
    local_x: float,
    local_y: float,
    ele: float,
    mgrs_code: str,
) -> ET.Element:
    el = ET.Element("node")
    el.set("id", str(node_id))
    el.set("lat", str(lat))
    el.set("lon", str(lon))
    el.append(_make_tag("local_x", str(local_x)))
    el.append(_make_tag("local_y", str(local_y)))
    el.append(_make_tag("ele", str(ele)))
    el.append(_make_tag("mgrs_code", mgrs_code))
    return el


def build_feedback_osm(
    original_text: str,
    signals: list[PedSignal],
    *,
    id_base: int,
    yaw_offset_deg: float,
    segment_length_m: float,
    subtype: str = "red_green",
) -> str:
    """Build a modified OSM XML string that adds pedestrian traffic light entries.

    For each PedSignal, creates:
      - 2 <node> elements (p0, p1) with local_x/y/ele/mgrs_code tags
      - 1 <way> element with nd refs to p0/p1 and type=traffic_light, subtype tags
      - 1 <relation> element with type=regulatory_element, subtype=traffic_light,
        and a member pointing to the way with role=refers

    All new elements use sequential ids starting from id_base.

    The generated XML block is inserted immediately before the closing </osm> tag.

    Args:
        original_text: The full original OSM file content as a string.
        signals: List of PedSignal objects to insert.
        id_base: Starting id for the new elements (4 ids per signal: node0, node1, way, relation).
        yaw_offset_deg: Yaw offset passed to make_segment_endpoints (e.g. -90.0).
        segment_length_m: Length of each traffic_light way segment in meters.
        subtype: OSM subtype tag value for the traffic_light way (default "red_green").

    Returns:
        Modified OSM XML string with the new elements inserted.
    """
    current_id = id_base
    fragments: list[str] = []

    for sig in signals:
        (x0, y0), (x1, y1) = make_segment_endpoints(
            sig.local_x, sig.local_y,
            sig.placed_yaw_deg, yaw_offset_deg,
            segment_length_m,
        )

        node0_id = current_id
        node1_id = current_id + 1
        way_id = current_id + 2
        rel_id = current_id + 3
        current_id += 4

        # Node p0
        n0 = _make_node_el(node0_id, sig.lat, sig.lon, x0, y0, sig.ele, sig.mgrs_code)
        # Node p1
        n1 = _make_node_el(node1_id, sig.lat, sig.lon, x1, y1, sig.ele, sig.mgrs_code)

        # Way
        way = ET.Element("way")
        way.set("id", str(way_id))
        nd0 = ET.SubElement(way, "nd")
        nd0.set("ref", str(node0_id))
        nd1 = ET.SubElement(way, "nd")
        nd1.set("ref", str(node1_id))
        way.append(_make_tag("type", "traffic_light"))
        way.append(_make_tag("subtype", subtype))
        if sig.pole_height is not None:
            way.append(_make_tag("pole_height", str(sig.pole_height)))

        # Relation
        rel = ET.Element("relation")
        rel.set("id", str(rel_id))
        rel.append(_make_tag("type", "regulatory_element"))
        rel.append(_make_tag("subtype", "traffic_light"))
        member = ET.SubElement(rel, "member")
        member.set("type", "way")
        member.set("role", "refers")
        member.set("ref", str(way_id))

        for el in (n0, n1, way, rel):
            fragments.append(ET.tostring(el, encoding="unicode"))

    block = "".join(fragments)

    # Insert before the last </osm>
    idx = original_text.rfind("</osm>")
    if idx == -1:
        return original_text + block
    return original_text[:idx] + block + original_text[idx:]

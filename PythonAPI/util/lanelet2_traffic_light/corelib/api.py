"""Unified entry point. Converts a lanelet2 .osm file into a list of PlacementSpecs.

The editor frontend calls this function and converts the resulting PlacementSpecs
into actual Actor placements.
"""
import time
from dataclasses import dataclass, field
from typing import Optional

from lanelet2_traffic_light.corelib.parser.lanelet2_parser import parse_osm
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
from lanelet2_traffic_light.corelib.geometry.pose_estimator import estimate_center_and_yaw
from lanelet2_traffic_light.corelib.profile.profile_base import TrafficLightProfile
from lanelet2_traffic_light.corelib.ir.traffic_light_ir import (
    PlacementSpec, GroupSpec,
)


def _compute_ele_midpoint(ele_p0, ele_p1):
    """Midpoint of ele between p0 and p1. Returns the non-None value if only one is None; returns None if both are None."""
    if ele_p0 is not None and ele_p1 is not None:
        return (ele_p0 + ele_p1) / 2.0
    if ele_p0 is not None:
        return ele_p0
    if ele_p1 is not None:
        return ele_p1
    return None


@dataclass
class GenerationReport:
    parsed_traffic_lights: int = 0
    parsed_groups: int = 0
    placements_created: int = 0
    placements_skipped: list = field(default_factory=list)  # (way_id, reason)
    warnings: list = field(default_factory=list)
    errors: list = field(default_factory=list)
    duration_seconds: float = 0.0


def generate_placements(
    osm_path: str,
    profile: TrafficLightProfile,
    sign_id_resolver,
    transformer: MgrsTransformer,
    bp_override: Optional[dict[str, str]] = None,
) -> tuple[list[PlacementSpec], list[GroupSpec], GenerationReport]:
    """lanelet2 -> list of PlacementSpecs + list of GroupSpecs + execution report.

    bp_override: dict of subtype -> bp_path. Takes precedence over profile defaults.
    """
    start = time.time()
    report = GenerationReport()
    bp_override = bp_override or {}

    tls, groups = parse_osm(osm_path)
    report.parsed_traffic_lights = len(tls)
    report.parsed_groups = len(groups)

    # Reverse lookup: way_id -> GroupSpec (for SignID strategies that need to reference the group)
    wayid_to_group: dict[int, GroupSpec] = {}
    for g in groups:
        for w in g.refers:
            wayid_to_group[w] = g

    placements: list[PlacementSpec] = []
    seen_sign_ids: dict[str, int] = {}    # sign_id -> way_id (collision detection)

    for tl in tls:
        # Resolve BP
        try:
            bp_path = bp_override.get(tl.subtype) or profile.bp_class_for(tl.subtype)
        except KeyError as e:
            report.placements_skipped.append((tl.way_id, f"unknown subtype: {tl.subtype}"))
            report.warnings.append(str(e))
            continue

        group_for_tl = wayid_to_group.get(tl.way_id)

        # Resolve SignID
        sign_id = sign_id_resolver.resolve(tl, group_for_tl)
        if sign_id in seen_sign_ids:
            raise ValueError(
                f"SignID collision: '{sign_id}' for way_id={tl.way_id} "
                f"already used by way_id={seen_sign_ids[sign_id]}"
            )
        seen_sign_ids[sign_id] = tl.way_id

        # Estimate center + yaw
        cx_m, cy_m, yaw_deg = estimate_center_and_yaw(tl, profile.yaw_offset_deg())
        # Height: lanelet2 height represents the signal face size, so use per-subtype pole_height.
        # Based on Z-distribution of 190 placed actors in Odaiba (confirmed in Phase 4.2):
        #   red_yellow_green (vehicle)   -> median 11.76 m
        #   red_green        (pedestrian)-> median  9.18 m
        # Falls back to profile.default_pole_height_m() for unknown subtypes.
        z_m = profile.pole_height_m(tl.subtype)

        x_cm, y_cm, z_cm = transformer.local_to_unreal_cm(cx_m, cy_m, z_m)

        # Compute representative point from lanelet2 (midpoint of p0/p1)
        lat_mid = (tl.p0.lat + tl.p1.lat) / 2.0
        lon_mid = (tl.p0.lon + tl.p1.lon) / 2.0
        local_x_mid = (tl.p0.local_x + tl.p1.local_x) / 2.0
        local_y_mid = (tl.p0.local_y + tl.p1.local_y) / 2.0
        ele_mid = _compute_ele_midpoint(tl.p0.ele, tl.p1.ele)
        # Use p0 for mgrs_code (MGRS grid is normally identical for both ends of a TL)
        mgrs_code = tl.p0.mgrs_code

        placements.append(PlacementSpec(
            sign_id=sign_id,
            actor_class_path=bp_path,
            location_cm=(x_cm, y_cm, z_cm),
            rotation_deg=(0.0, 0.0, yaw_deg),
            group_relation_id=(group_for_tl.relation_id if group_for_tl else None),
            source_way_id=tl.way_id,
            subtype=tl.subtype,
            lat=lat_mid,
            lon=lon_mid,
            ele=ele_mid,
            local_x=local_x_mid,
            local_y=local_y_mid,
            mgrs_code=mgrs_code,
        ))
        report.placements_created += 1

    report.duration_seconds = time.time() - start
    return placements, groups, report

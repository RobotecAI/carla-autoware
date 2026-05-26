"""統合エントリーポイント。lanelet2 .osm → PlacementSpec のリストを返す。

Editor フロントエンドはこの関数を呼び、得られた PlacementSpec を実 Actor 配置に
変換する。
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
    """p0/p1 の ele の中点。片方 None なら他方、両方 None なら None。"""
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
    """lanelet2 → PlacementSpec リスト + GroupSpec リスト + 実行レポート。

    bp_override: subtype -> bp_path の dict。プロファイル既定値より優先される。
    """
    start = time.time()
    report = GenerationReport()
    bp_override = bp_override or {}

    tls, groups = parse_osm(osm_path)
    report.parsed_traffic_lights = len(tls)
    report.parsed_groups = len(groups)

    # way_id -> GroupSpec 逆引き（SignID 戦略が group を参照したいケース用）
    wayid_to_group: dict[int, GroupSpec] = {}
    for g in groups:
        for w in g.refers:
            wayid_to_group[w] = g

    placements: list[PlacementSpec] = []
    seen_sign_ids: dict[str, int] = {}    # sign_id -> way_id (衝突検出)

    for tl in tls:
        # BP 解決
        try:
            bp_path = bp_override.get(tl.subtype) or profile.bp_class_for(tl.subtype)
        except KeyError as e:
            report.placements_skipped.append((tl.way_id, f"unknown subtype: {tl.subtype}"))
            report.warnings.append(str(e))
            continue

        group_for_tl = wayid_to_group.get(tl.way_id)

        # SignID 解決
        sign_id = sign_id_resolver.resolve(tl, group_for_tl)
        if sign_id in seen_sign_ids:
            raise ValueError(
                f"SignID collision: '{sign_id}' for way_id={tl.way_id} "
                f"already used by way_id={seen_sign_ids[sign_id]}"
            )
        seen_sign_ids[sign_id] = tl.way_id

        # 中心+yaw 推定
        cx_m, cy_m, yaw_deg = estimate_center_and_yaw(tl, profile.yaw_offset_deg())
        # 高さ: lanelet2 height は信号面サイズなので、subtype 別の pole_height を使う。
        # Phase 4.2 で Odaiba 配置済 190 件の Z 分布から:
        #   red_yellow_green (車両)  → median 11.76 m
        #   red_green        (歩行者)→ median  9.18 m
        # subtype 不明時は profile.default_pole_height_m() にフォールバック。
        z_m = profile.pole_height_m(tl.subtype)

        x_cm, y_cm, z_cm = transformer.local_to_unreal_cm(cx_m, cy_m, z_m)

        # lanelet2 由来の代表点を計算 (p0/p1 中点)
        lat_mid = (tl.p0.lat + tl.p1.lat) / 2.0
        lon_mid = (tl.p0.lon + tl.p1.lon) / 2.0
        local_x_mid = (tl.p0.local_x + tl.p1.local_x) / 2.0
        local_y_mid = (tl.p0.local_y + tl.p1.local_y) / 2.0
        ele_mid = _compute_ele_midpoint(tl.p0.ele, tl.p1.ele)
        # mgrs_code は p0 を採用 (TL 両端の MGRS グリッドは通常同一)
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

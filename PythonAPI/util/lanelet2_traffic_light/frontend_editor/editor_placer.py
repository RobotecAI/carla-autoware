"""UE5 Editor Python フロントエンド。

core パッケージが出力した `PlacementSpec` をレベル内の Actor 配置に変換する。
`unreal` モジュールを import する唯一のファイル。

設計仕様: docs/superpowers/specs/2026-05-20-lanelet2-traffic-light-design.md
実装プラン: docs/superpowers/plans/2026-05-20-lanelet2-traffic-light.md
Phase 0 知見: PythonAPI/util/lanelet2_traffic_light/docs/phase0_validation.md
"""
from dataclasses import dataclass, field
from typing import Optional

import unreal

from lanelet2_traffic_light.corelib.ir.traffic_light_ir import PlacementSpec, GroupSpec
from lanelet2_traffic_light.corelib.geometry.mgrs_transform import MgrsTransformer
from lanelet2_traffic_light.frontend_editor.snap_stats import (
    MeshZStats, compute_z_stats,
)


# Phase 5 ② 以前のデフォルト Group BP path (現在は使われない、
# `_place_groups()` に引数で渡される map 別の派生 BP を使う)。
# 互換性のため定数自体は残すが、参照は廃止。
BP_TRAFFIC_LIGHT_GROUP_PATH = "/Carla/Blueprints/TrafficLight/BP_TrafficLightGroup.BP_TrafficLightGroup_C"


@dataclass
class PlacementReport:
    """place_from_specs() の実行結果サマリ。"""
    created: int = 0
    updated: int = 0
    failed: list = field(default_factory=list)  # (sign_id, reason_str) のタプル
    # snap_to_existing_mesh=True で対応する既存メッシュが見つからずに配置を
    # skip した spec のリスト。Odaiba.umap の 3D 街並みカバー範囲外の信号機
    # (lanelet2 は東京湾岸全域カバーするがメッシュは Odaiba 中心のみ) が
    # ここに入る。
    snap_skipped: list = field(default_factory=list)  # (sign_id, target_xyz_cm)
    snap_skipped_records: list = field(default_factory=list)  # dict 形式 (key=value 出力用)
    # 逆方向の漏れ: Odaiba メッシュとしては存在するが、どの lanelet2 way とも
    # マッチしなかった既存信号機メッシュ (label, (x_cm, y_cm, z_cm)) のリスト。
    # 撤去された信号機の残置、マップ設計時の冗長、lanelet2 編集漏れ等の
    # 検出に使える (Phase 4.2 追加)。
    unused_existing_meshes: list = field(default_factory=list)
    # 配置成功した spec の snap 詳細レコード。レポートファイル出力用。
    # 各要素は dict: {sign_id, was_created, target_label, xy_dist_cm, mesh_name}
    placed_records: list = field(default_factory=list)
    # Z 統計のスナップショット (レポート出力用)
    z_stats_snapshot: dict = field(default_factory=dict)
    groups_created: int = 0
    groups_updated: int = 0
    pedestrian_material_stats: list = field(default_factory=list)
    # aggregate_material_stats の返り値リストを格納


# ---------------------------------------------------------------------------
# 3.1: get_world_mgrs_data
# ---------------------------------------------------------------------------

def _get_editor_world():
    """現在エディタで開いているワールドを取得する。

    UE5.5+ では EditorLevelLibrary.get_editor_world() が deprecated になったため、
    UnrealEditorSubsystem 経由を優先し、失敗した場合のみ旧 API にフォールバックする。
    """
    subsys = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    if subsys is not None and hasattr(subsys, "get_editor_world"):
        return subsys.get_editor_world()
    # フォールバック: UE5.4 以前の環境向け
    return unreal.EditorLevelLibrary.get_editor_world()


def _resolve_soft_object(soft_ref):
    """TSoftObjectPtr / SoftObjectPath を解決して UObject を返す。

    unreal Python では TSoftObjectPtr がそのまま UObject として振る舞う場合と、
    SoftObjectPath として届く場合の両方がある。いずれにも対応する。
    """
    if soft_ref is None:
        return None
    # SoftObjectPath の場合はパス文字列を取り出して load する
    if isinstance(soft_ref, unreal.SoftObjectPath):
        path = soft_ref.get_path_name() if hasattr(soft_ref, "get_path_name") else str(soft_ref)
        return unreal.EditorAssetLibrary.load_asset(path)
    # 既に解決済み UObject (TSoftObjectPtr が自動解決されたケース) はそのまま返す
    try:
        if hasattr(soft_ref, "get_path_name"):
            return soft_ref
    except Exception:
        pass
    return soft_ref


def get_world_mgrs_data() -> MgrsTransformer:
    """現在開いているレベルの WorldSettings から MgrsDataAsset を取り出し、
    Phase 0 で確定した式に従う MgrsTransformer を構築して返す。

    Phase 0 で判明した実プロパティ名: `mgrs_data_asset_soft_ptr` (TSoftObjectPtr)
    MgrsOffsetPosition の単位: m (cm ではない)

    Raises:
        RuntimeError: WorldSettings に期待するプロパティが存在しない、
                      またはアセットのロードに失敗した場合。
    """
    world = _get_editor_world()
    ws = world.get_world_settings()

    # Phase 0 で確定: プロパティ名は mgrs_data_asset_soft_ptr
    # 旧プラン案にあった mgrs_data_asset / MgrsDataAsset は誤りなので使わない
    raw = ws.get_editor_property("mgrs_data_asset_soft_ptr")
    if raw is None:
        raise RuntimeError(
            "WorldSettings has no 'mgrs_data_asset_soft_ptr'. "
            "Confirm the level uses AutowareWorldSettings and the asset is assigned."
        )

    da = _resolve_soft_object(raw)
    if da is None:
        raise RuntimeError("MgrsDataAsset could not be loaded from soft reference.")

    offset = da.get_editor_property("mgrs_offset_position")
    if offset is None:
        raise RuntimeError("MgrsDataAsset.MgrsOffsetPosition is null.")

    # Phase 0 で確定: offset.x / offset.y / offset.z は m 単位
    # x_sign=+1, y_sign=-1 は標準 CARLA Z-up かつ Y のみ反転 (Phase 0 で確定)
    return MgrsTransformer(
        offset_x_m=float(offset.x),
        offset_y_m=float(offset.y),
        offset_z_m=float(offset.z),
        x_sign=+1,
        y_sign=-1,   # CARLA 標準 Z-up かつ Y のみ反転
    )


# ---------------------------------------------------------------------------
# 3.2: find_actor_by_sign_id
# ---------------------------------------------------------------------------

def find_actor_by_sign_id(sign_id: str) -> Optional[unreal.Actor]:
    """レベル内の TrafficLightBase アクターから sign_id が一致するものを返す。

    EditorActorSubsystem でレベル内の全アクターを走査し、TrafficLightBase 派生
    かつ TrafficLightComponent.get_sign_id() が一致するものを探す。

    複数ヒット時はデバッグ用に警告ログを出して最初の 1 件を返す。
    見つからない場合は None を返す。
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    matches = []

    for a in actor_subsys.get_all_level_actors():
        # TrafficLightBase 派生でないアクターはスキップ
        if not isinstance(a, unreal.TrafficLightBase):
            continue

        tlc = a.get_traffic_light_component()
        if tlc is None:
            continue

        try:
            sid = tlc.get_sign_id()
        except Exception:
            # get_sign_id が存在しない古い BP クラスなどへの安全なフォールバック
            continue

        if sid == sign_id:
            matches.append(a)

    if len(matches) > 1:
        labels = [m.get_actor_label() for m in matches]
        unreal.log_warning(
            f"find_actor_by_sign_id: multiple actors share sign_id={sign_id}: {labels}. "
            "Returning first. Consider removing duplicates."
        )

    return matches[0] if matches else None


# ---------------------------------------------------------------------------
# 3.3: spawn / update single placement
# ---------------------------------------------------------------------------

def _load_bp_class(class_path: str):
    """ObjectPath から GeneratedClass (UClass) をロードする。

    `EditorAssetLibrary.load_blueprint_class()` は Asset path
    (`/Game/.../BP_X.BP_X`) を期待し、戻り値が `BP_X_C` (GeneratedClass)。
    呼び出し側が `_C` サフィックス付きで渡してきても剥がしてから渡す。

    Raises:
        RuntimeError: ロードに失敗した場合。
    """
    asset_path = class_path[:-2] if class_path.endswith("_C") else class_path
    cls = unreal.EditorAssetLibrary.load_blueprint_class(asset_path)
    if cls is None:
        raise RuntimeError(f"Failed to load blueprint class: {class_path}")
    return cls


def _collect_unused_meshes(used_labels: set,
                           label_prefixes=("Traffic_Lights", "Pedestrian_Lights")) -> list:
    """`used_labels` に含まれない既存信号機メッシュ一覧を返す (Pole 除く)。

    Phase 4.2 追加: lanelet2 紐付けから漏れた既存メッシュを検出する。
    撤去された信号機の残置、マップ設計時の冗長、lanelet2 編集漏れ等の発見に使う。

    Returns:
        [(label, (x_cm, y_cm, z_cm)), ...] World 位置を含むタプルのリスト。
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    out = []
    for a in actor_subsys.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor):
            continue
        label = a.get_actor_label()
        if "Pole" in label:
            continue
        if not any(label.startswith(p) for p in label_prefixes):
            continue
        if label in used_labels:
            continue
        loc = a.get_actor_location()
        out.append((label, (loc.x, loc.y, loc.z)))
    return out


def _collect_mesh_z_stats(label_prefixes=("Traffic_Lights", "Pedestrian_Lights")) -> dict:
    """既存メッシュの prefix 別 Z 統計を Editor から収集する。

    Phase 4.2 で追加: マップごとの Z 基準ズレ (海抜/楕円体/独自基準) や
    pole_height の差を吸収するため、Full Run 開始時に 1 回呼び出して
    `_find_nearest_existing_signal_mesh` の Z 妥当性判定に使う。

    Args:
        label_prefixes: 収集対象の prefix 一覧。"Pole" を含むラベルは除外。

    Returns:
        {prefix: MeshZStats} の辞書。該当メッシュが無い prefix は含まない。
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    by_prefix: dict = {p: [] for p in label_prefixes}
    for a in actor_subsys.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor):
            continue
        label = a.get_actor_label()
        if "Pole" in label:
            continue
        for p in label_prefixes:
            if label.startswith(p):
                by_prefix[p].append(a.get_actor_location().z)
                break
    return {p: compute_z_stats(zs) for p, zs in by_prefix.items() if zs}


def _collect_pedestrian_mesh_materials() -> list:
    """レベル上の Pedestrian_Lights_* アクターの Material element 構成を集計。

    Returns:
        aggregate_material_stats() の出力リスト
        [{"mesh": str, "num_elements": int, "elements": tuple, "count": int}, ...]
    """
    from lanelet2_traffic_light.frontend_editor.material_stats import aggregate_material_stats

    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    records: list = []
    for a in actor_subsys.get_all_level_actors():
        label = a.get_actor_label()
        if not label.startswith("Pedestrian_Lights"):
            continue
        smc = None
        try:
            smc = a.static_mesh_component
        except AttributeError:
            # StaticMeshActor 以外はスキップ
            try:
                smc = a.get_component_by_class(unreal.StaticMeshComponent)
            except Exception:
                smc = None
        if smc is None:
            continue
        try:
            mesh_asset = smc.static_mesh
        except Exception:
            mesh_asset = None
        if mesh_asset is None:
            continue
        try:
            mesh_name = mesh_asset.get_name()
        except Exception:
            continue
        # Material element 名を順序保持で取得
        try:
            num = smc.get_num_materials()
        except Exception:
            num = 0
        element_names: list = []
        for i in range(num):
            try:
                mat = smc.get_material(i)
                element_names.append(mat.get_name() if mat is not None else "<None>")
            except Exception:
                element_names.append("<error>")
        records.append((mesh_name, tuple(element_names)))
    return aggregate_material_stats(records)


def _label_prefixes_for_bp_class(bp_class_path: str) -> tuple:
    """BP class path から、対応する既存メッシュのラベル prefix を決定する。

    Odaiba.umap の既存信号機メッシュは 2 系統あり、subtype に応じて
    異なるラベル prefix が使われている:

    | subtype           | BP class              | 既存メッシュ命名     |
    |-------------------|------------------------|-----------------------|
    | red_yellow_green  | BP_OdaibaVehicleTL    | Traffic_Lights_*     |
    | red_green         | BP_OdaibaPedestrianTL | Pedestrian_Lights_*  |

    車両用メッシュに歩行者用 BP を誤って snap させないよう、prefix を
    厳密に分離する。
    """
    if "Pedestrian" in bp_class_path:
        return ("Pedestrian_Lights",)
    return ("Traffic_Lights",)


def _label_prefix_for_subtype(subtype: str) -> str:
    """subtype 別の actor label prefix を返す。

    - red_yellow_green (車両用) → "TLV_"
    - red_green (歩行者用)     → "TLP_"
    - その他 (未知 subtype)    → "TL_" (フォールバック、Phase 5② 以前と互換)
    """
    if subtype == "red_yellow_green":
        return "TLV_"
    if subtype == "red_green":
        return "TLP_"
    return "TL_"


def _find_nearest_existing_signal_mesh(target: unreal.Vector,
                                       max_distance_cm: float = 300.0,
                                       label_prefixes: tuple = ("Traffic_Lights",),
                                       max_z_diff_cm: float = 700.0,
                                       mesh_z_stats: Optional[dict] = None):
    """target に最も近い既存の信号機メッシュ StaticMeshActor を返す (XY 距離ベース)。

    距離計算は **XY 平面距離** を使う (Phase 4.2 で変更)。
    歩行者用信号機は地上 7-10m と core 計算の pole_height (車両用 12.3m)
    と数 m ズレるため、3D 距離だと radius=300cm 外になって snap fail
    していた。XY 距離なら一致する。誤 snap 防止に Z 妥当性を別途確認:

    - `mesh_z_stats` が与えられた場合: prefix 別に Tukey の 1.5 IQR 範囲外
      の mesh を除外 (マップ非依存、自動適応)。
    - 与えられない場合: target.z との差が `max_z_diff_cm` 超のものを除外
      (固定値、fallback)。

    Args:
        target: 検索の中心 (Unreal world cm)。
        max_distance_cm: XY 平面距離の上限 (cm)。0 以下なら無制限。
        label_prefixes: 許容するアクター prefix。
        max_z_diff_cm: mesh_z_stats=None 時の Z 差絶対値上限。
        mesh_z_stats: `{prefix: MeshZStats}` の dict。`_collect_mesh_z_stats()`
            で取得。stats があるならそれを優先する。

    Returns:
        (actor or None, xy_distance_cm)
    """
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    best_actor = None
    best_xy_sq = float("inf")
    max_xy_sq = (max_distance_cm * max_distance_cm) if max_distance_cm > 0 else float("inf")
    abs_max_z = max_z_diff_cm if max_z_diff_cm > 0 else float("inf")

    for a in actor_subsys.get_all_level_actors():
        if not isinstance(a, unreal.StaticMeshActor):
            continue
        label = a.get_actor_label()
        matched_prefix = None
        for p in label_prefixes:
            if label.startswith(p):
                matched_prefix = p
                break
        if matched_prefix is None:
            continue
        loc = a.get_actor_location()

        # Z 妥当性チェック: stats 優先、なければ静的 fallback
        if mesh_z_stats and matched_prefix in mesh_z_stats:
            stats = mesh_z_stats[matched_prefix]
            if loc.z < stats.tukey_low or loc.z > stats.tukey_high:
                continue
        else:
            dz = loc.z - target.z
            if dz > abs_max_z or dz < -abs_max_z:
                continue

        dx = loc.x - target.x
        dy = loc.y - target.y
        xy_sq = dx * dx + dy * dy
        if xy_sq < best_xy_sq and xy_sq <= max_xy_sq:
            best_xy_sq = xy_sq
            best_actor = a
    return best_actor, (best_xy_sq ** 0.5 if best_actor is not None else float("inf"))


def _spawn_or_update(spec: PlacementSpec,
                     snap_to_existing_mesh: bool = False,
                     snap_radius_cm: float = 300.0,
                     skip_when_snap_fails: bool = True,
                     mesh_z_stats: Optional[dict] = None) -> tuple:
    """spec.sign_id が既にレベル内にあれば位置・回転を更新し、なければ新規 spawn する。

    冪等性を保つための中心ロジック。同じ sign_id で何度実行しても
    アクターが重複しないことが保証される。

    独立アクターとして spawn する (Phase 0 確認事項: 既存信号機メッシュは
    OdaibaFinaL_ver7_lights に attach されているが、今回 spawn するアクターは
    親 attach なしの独立配置)。

    Args:
        spec: core.api.generate_placements() が生成した PlacementSpec。
              location_cm は (X_cm, Y_cm, Z_cm)、
              rotation_deg は **(roll, pitch, yaw)** 順。
              core.api では rotation_deg=(0.0, 0.0, yaw_deg) として生成される。
        snap_to_existing_mesh: True ならば、計算した位置の近傍に既存の
              対応メッシュ (車両用は Traffic_Lights_*、歩行者用は
              Pedestrian_Lights_*) があれば、その位置・回転を採用する。
              core 計算の数 cm / 数度のズレを吸収できる (Odaiba 専用の
              微調整モード)。
        snap_radius_cm: スナップ判定の最大距離。離れすぎたメッシュは
              無視して core 計算値を使う。
        skip_when_snap_fails: snap_to_existing_mesh=True かつ近傍に対応メッシュが
              無い場合、新規 spawn を skip する (Phase 4.2 で追加)。
              Odaiba.umap の 3D 街並みカバー外の信号機が空中に出現するのを防ぐ。
              既存アクターの更新には影響しない (snap target なくても更新は走る)。

    Returns:
        (actor, was_created, snap_info):
          - 正常配置: (actor, True/False, dict or None)
          - snap fail で skip: (None, False, None)
        snap_info は snap モードで採用された既存メッシュの詳細 dict:
          {"label": str, "xy_dist_cm": float, "mesh_name": str}
        snap モード OFF または snap target なしの場合は None。
        レポートファイル出力と逆方向漏れ集計に使う。

    Raises:
        RuntimeError: BP クラスのロードまたは spawn に失敗した場合。
    """
    # PlacementSpec.location_cm は (X, Y, Z) のタプル
    location = unreal.Vector(spec.location_cm[0], spec.location_cm[1], spec.location_cm[2])
    # PlacementSpec.rotation_deg は (roll, pitch, yaw) 順。
    # `unreal.Rotator` のポジショナル引数は (pitch, yaw, roll) なので
    # 軸の取り違えを避けるため keyword 引数で明示する。
    roll_deg, pitch_deg, yaw_deg = spec.rotation_deg
    rotation = unreal.Rotator(roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg)

    # Snap モード: 既存メッシュの位置・回転・StaticMesh アセットを採用してドリフトを吸収。
    # BP class path に応じて検索対象 prefix を切り替える (Pedestrian → Pedestrian_Lights)。
    snapped_mesh_asset = None  # snap モードで採用する StaticMesh アセット
    snap_target_found = False
    snap_info = None  # snap 詳細 dict (レポート出力 & 逆方向漏れ集計用)
    if snap_to_existing_mesh:
        label_prefixes = _label_prefixes_for_bp_class(spec.actor_class_path)
        nearest, dist = _find_nearest_existing_signal_mesh(
            location, snap_radius_cm, label_prefixes,
            mesh_z_stats=mesh_z_stats,
        )
        if nearest is not None:
            snap_target_found = True
            snapped_loc = nearest.get_actor_location()
            snapped_rot = nearest.get_actor_rotation()
            # snap target の StaticMesh も取得 — BP 固定の Scene_1024 と pivot が
            # 違うと位置が完全に合わないので、対応する元 mesh に置き換える
            try:
                target_sm_comp = nearest.get_component_by_class(unreal.StaticMeshComponent)
                if target_sm_comp is not None:
                    snapped_mesh_asset = target_sm_comp.get_editor_property("static_mesh")
            except Exception:
                snapped_mesh_asset = None
            mesh_name = snapped_mesh_asset.get_name() if snapped_mesh_asset is not None else "<n/a>"
            snap_info = {
                "label": nearest.get_actor_label(),
                "xy_dist_cm": dist,
                "mesh_name": mesh_name,
            }
            unreal.log(
                f"_spawn_or_update[snap]: sign_id={spec.sign_id} "
                f"snapped to '{nearest.get_actor_label()}' "
                f"(xy_dist={dist:.1f} cm, mesh={mesh_name})"
            )
            location = snapped_loc
            rotation = snapped_rot
        else:
            prefix_str = "/".join(p + "_*" for p in label_prefixes)
            unreal.log_warning(
                f"_spawn_or_update[snap]: sign_id={spec.sign_id} "
                f"no {prefix_str} within {snap_radius_cm} cm of target."
            )

    existing = find_actor_by_sign_id(spec.sign_id)
    if existing is not None:
        # 既存アクターの位置・回転のみ更新 (ラベルや sign_id は維持)
        existing.set_actor_location(location, sweep=False, teleport=True)
        existing.set_actor_rotation(rotation, teleport_physics=True)
        if snap_to_existing_mesh:
            _zero_out_static_mesh_relative_rotation(existing)
            # Phase 6 課題 A: 歩行者用 (subtype=red_green) は snap した既存メッシュ
            # ではなく、親 BP_PedestrianTrafficLight の StaticMesh (3 element:
            # Walk/Frame/Stop) をそのまま使う。snap mesh (Scene_805 等の 2 element)
            # で override すると MID 生成失敗で不点灯になるため、位置/向きだけ
            # snap target から取り、mesh override はスキップする。
            if snapped_mesh_asset is not None and spec.subtype != "red_green":
                _override_static_mesh(existing, snapped_mesh_asset)
        return existing, False, snap_info

    # 新規 spawn の手前で snap fail を skip する
    # (既存アクター更新はここに到達しないので影響なし)
    if snap_to_existing_mesh and skip_when_snap_fails and not snap_target_found:
        return None, False, None

    # 新規 spawn
    bp_cls = _load_bp_class(spec.actor_class_path)
    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actor = actor_subsys.spawn_actor_from_class(bp_cls, location, rotation)
    if actor is None:
        raise RuntimeError(
            f"spawn_actor_from_class failed for {spec.actor_class_path} "
            f"(sign_id={spec.sign_id})"
        )

    # Snap モードでは BP の StaticMeshComponent に baked-in されている
    # Relative Rotation (例: Roll=-90, Pitch=85.4) が snap World Rotation と
    # 二重適用されてしまうため、ここで打ち消す。さらに、各 Traffic_Lights_* は
    # 個別の Scene_NNNN アセットを使うため、その mesh も snap target のものに
    # 差し替えてピボット位置を一致させる。
    # Phase 6 課題 A: 歩行者用 (subtype=red_green) は snap した既存メッシュ
    # ではなく、親 BP_PedestrianTrafficLight の StaticMesh (3 element:
    # Walk/Frame/Stop) をそのまま使う。snap mesh (Scene_805 等の 2 element)
    # で override すると MID 生成失敗で不点灯になるため、位置/向きだけ
    # snap target から取り、mesh override はスキップする。
    if snap_to_existing_mesh:
        _zero_out_static_mesh_relative_rotation(actor)
        if snapped_mesh_asset is not None and spec.subtype != "red_green":
            _override_static_mesh(actor, snapped_mesh_asset)

    # TrafficLightComponent に sign_id を書き込む (後から find_actor_by_sign_id で検索できるように)
    tlc = actor.get_traffic_light_component()
    if tlc is not None:
        tlc.set_sign_id(spec.sign_id)
    else:
        unreal.log_warning(
            f"_spawn_or_update: actor {spec.actor_class_path} has no TrafficLightComponent. "
            f"sign_id={spec.sign_id} will not be persisted on the component."
        )

    # エディタ上で識別しやすいラベルを付ける (subtype 別 prefix: TLV_/TLP_/TL_)
    prefix = _label_prefix_for_subtype(spec.subtype)
    actor.set_actor_label(f"{prefix}{spec.sign_id}")

    return actor, True, snap_info


def _override_static_mesh(actor, new_mesh) -> None:
    """Actor 配下の全 StaticMeshComponent の StaticMesh を差し替える。

    snap モードでは spawn 直後の BP デフォルト mesh (Scene_1024 固定) を
    snap target の Traffic_Lights_* が使っている mesh (Scene_NNNN ごとに異なる)
    に上書きすることで、pivot 差による微妙な位置ズレを解消する。
    """
    if new_mesh is None:
        return
    try:
        comps = actor.get_components_by_class(unreal.StaticMeshComponent)
    except Exception:
        return
    for comp in comps:
        try:
            comp.set_static_mesh(new_mesh)
        except Exception as e:
            unreal.log_warning(
                f"_override_static_mesh: set_static_mesh failed on "
                f"'{actor.get_actor_label()}': {e}"
            )


def _zero_out_static_mesh_relative_rotation(actor) -> None:
    """spawn 直後の Actor 内の StaticMeshComponent の Relative Rotation を
    (0, 0, 0) にリセットする。

    snap モードでは Actor の World Rotation を既存メッシュにスナップするため、
    BP に baked-in されている Component Relative Rotation を含む二重適用を
    避ける必要がある。
    """
    try:
        comps = actor.get_components_by_class(unreal.StaticMeshComponent)
    except Exception:
        return
    for comp in comps:
        try:
            comp.set_relative_rotation(unreal.Rotator(0.0, 0.0, 0.0), sweep=False, teleport=True)
        except TypeError:
            # 引数違いのオーバーロード対策
            try:
                comp.set_relative_rotation(unreal.Rotator(0.0, 0.0, 0.0))
            except Exception:
                pass
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 3.4: group binding
# ---------------------------------------------------------------------------

def _place_groups(groups: list, sign_id_to_actor: dict,
                  sign_id_to_subtype: dict,
                  group_bp_path: Optional[str] = None) -> tuple:
    """各 GroupSpec ごとに ATrafficLightGroup actor を配置し、subtype 別に
    Controller を 2 個 (Vehicle/Pedestrian) 紐付ける (Phase 6)。

    Phase 5② の 1 Group = 1 Controller では subtype 混在で `set_state` が
    全 TL に同期してしまう問題があったため、Phase 6 で member_actors を
    subtype 別に分割し、Group に Controller を 2 個 add_controller する設計に
    変更した。`set_state` は直接 Component に書くので subtype 独立操作が可能。

    Args:
        groups: corelib.api.generate_placements() の groups リスト。
        sign_id_to_actor: 配置済 TL の sign_id → Actor マップ。
        sign_id_to_subtype: sign_id → subtype 文字列 (red_yellow_green / red_green)。
            place_from_specs で {p.sign_id: p.subtype for p in placements} として構築する。
        group_bp_path: 派生 Group BP の Asset Path (bp_factory.ensure_group_bp で
            得られた値)。None の場合は C++ ATrafficLightGroup を直接 spawn。

    Returns:
        (created_count, updated_count)
    """
    from lanelet2_traffic_light.frontend_editor.subtype_splitter import split_members_by_subtype

    # Group spawn 用クラスを決定: 派生 BP > C++ 直派生
    group_cls = None
    if group_bp_path:
        try:
            group_cls = _load_bp_class(group_bp_path)
        except RuntimeError as e:
            unreal.log_warning(
                f"_place_groups: Group BP load failed at {group_bp_path}: {e}. "
                f"Falling back to C++ ATrafficLightGroup."
            )
    if group_cls is None:
        group_cls = unreal.TrafficLightGroup

    actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    # 既存 TLGroup_* アクターのラベルマップ (冪等再実行のため)
    existing_labels: dict = {
        a.get_actor_label(): a
        for a in actor_subsys.get_all_level_actors()
        if a.get_actor_label().startswith("TLGroup_")
    }

    created = 0
    updated = 0

    for g in groups:
        # subtype 別に member_actors を分割
        members_by_subtype = split_members_by_subtype(
            refers=g.refers,
            sign_id_to_actor=sign_id_to_actor,
            sign_id_to_subtype=sign_id_to_subtype,
        )
        # find_actor_by_sign_id によるレベル探索フォールバックも考慮
        # (sign_id_to_actor に無いが、レベル上には存在する場合)
        for way_id in g.refers:
            sid = str(way_id)
            if sid in sign_id_to_actor:
                continue
            actor = find_actor_by_sign_id(sid)
            if actor is not None:
                subtype = sign_id_to_subtype.get(sid, "")
                members_by_subtype.setdefault(subtype, []).append(actor)

        if not members_by_subtype:
            # 全 member が snap_skipped 等で配置されなかったグループはスキップ
            continue

        label = f"TLGroup_{g.relation_id}"
        if label in existing_labels:
            group_actor = existing_labels[label]
            # 既存 Controllers を空にしてやり直す (冪等性のため)
            try:
                group_actor.set_editor_property("controllers", [])
            except Exception:
                pass
            updated += 1
        else:
            # グループの原点は最初に出現する subtype 群の先頭メンバーに合わせる
            first_actor = next(iter(members_by_subtype.values()))[0]
            group_actor = actor_subsys.spawn_actor_from_class(
                group_cls,
                first_actor.get_actor_location(),
                unreal.Rotator(0, 0, 0),
            )
            if group_actor is None:
                unreal.log_warning(
                    f"_place_groups: Failed to spawn group actor for label={label}. Skipping."
                )
                continue
            group_actor.set_actor_label(label)
            created += 1

        # JunctionId に lanelet2 relation_id を流用
        try:
            group_actor.set_editor_property("junction_id", int(g.relation_id))
        except Exception:
            pass

        # subtype 別に Controller を 1 個ずつ作成
        for subtype, members in members_by_subtype.items():
            try:
                controller = unreal.new_object(
                    unreal.TrafficLightController, outer=group_actor
                )
            except Exception as e:
                unreal.log_warning(
                    f"_place_groups: failed to create UTrafficLightController for "
                    f"group {g.relation_id} subtype={subtype}: {e}. Skipping."
                )
                continue
            controller_id = f"{g.relation_id}_{subtype}" if subtype else str(g.relation_id)
            try:
                controller.set_controller_id(controller_id)
            except Exception:
                pass
            try:
                controller.set_red_time(10.0)
                controller.set_yellow_time(2.0)
                controller.set_green_time(10.0)
            except Exception:
                pass

            try:
                group_actor.add_controller(controller)
            except Exception as e:
                unreal.log_warning(
                    f"_place_groups: group_actor.add_controller failed for "
                    f"group {g.relation_id} subtype={subtype}: {e}. Skipping subtype."
                )
                continue

            for actor in members:
                try:
                    tlc = actor.get_traffic_light_component()
                except Exception:
                    tlc = None
                if tlc is None:
                    continue
                try:
                    controller.add_traffic_light(tlc)
                except Exception as e:
                    unreal.log_warning(
                        f"_place_groups: add_traffic_light failed for {actor.get_actor_label()} "
                        f"in group {g.relation_id} subtype={subtype}: {e}"
                    )

    return created, updated


# ---------------------------------------------------------------------------
# レポートファイル出力 (Phase 4.2 追加)
# ---------------------------------------------------------------------------

def _write_full_run_report(report: "PlacementReport", path: str,
                           n_input_placements: int,
                           metadata: Optional[dict] = None) -> None:
    """place_from_specs 完了後の全件レポートをテキストファイルに書き出す。

    省略なく以下を出力 (1 ファイル内に複数セクション):
      0. METADATA: 入力 osm ファイル・CarlaUE5 ディレクトリ等のフルパス
      1. SUMMARY: 集計値
      2. Z STATS: prefix 別の Z 統計 (Tukey range)
      3. PLACED: 配置成功した spec の sign_id, snap target, xy_dist, mesh
      4. SNAP_SKIPPED: lanelet2 way ありだが Odaiba 街並み外で skip
      5. FAILED: 例外発生で配置失敗
      6. UNUSED_MESHES: Odaiba メッシュありだが lanelet2 way 無し (逆方向漏れ)

    出力先は上書き。git 管理外の場所を想定 (例 T4Fork.odaiba 直下)。

    Args:
        metadata: 任意の key/value をレポート冒頭に記載。`osm_path`,
            `carla_ue5_dir` などのフルパスを渡すと後追跡しやすい。
    """
    import datetime
    import os
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.write("# Lanelet2 Traffic Light Full Run Report\n")
        f.write(f"# generated: {datetime.datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# input placements (lanelet2 parsed): {n_input_placements}\n")
        f.write(f"# report file path: {os.path.abspath(path)}\n")
        f.write("\n")

        # 0. METADATA (入力ファイル/ディレクトリのフルパス)
        if metadata:
            f.write("## METADATA\n")
            for k, v in metadata.items():
                f.write(f"{k:24s}: {v}\n")
            f.write("\n")

        # 1. SUMMARY
        f.write("## SUMMARY\n")
        f.write(f"created          : {report.created}\n")
        f.write(f"updated          : {report.updated}\n")
        f.write(f"snap_skipped     : {len(report.snap_skipped)}\n")
        f.write(f"failed           : {len(report.failed)}\n")
        f.write(f"unused_meshes    : {len(report.unused_existing_meshes)}\n")
        f.write(f"groups_created   : {report.groups_created}\n")
        f.write(f"groups_updated   : {report.groups_updated}\n")
        f.write("\n")

        # 2. Z STATS
        f.write("## Z STATS (existing meshes, by prefix)\n")
        if report.z_stats_snapshot:
            for prefix, st in report.z_stats_snapshot.items():
                f.write(
                    f"{prefix:24s} n={st['n']:4d} "
                    f"min={st['z_min']:8.1f} q25={st['z_q25']:8.1f} "
                    f"median={st['z_median']:8.1f} q75={st['z_q75']:8.1f} "
                    f"max={st['z_max']:8.1f} "
                    f"tukey=[{st['tukey_low']:8.1f}, {st['tukey_high']:8.1f}]\n"
                )
        else:
            f.write("(snap_to_existing_mesh disabled, no stats collected)\n")
        f.write("\n")

        # 3. PLACED
        f.write(f"## PLACED ({len(report.placed_records)} entries)\n")
        f.write("\n# placed\n")
        for rec in report.placed_records:
            wx, wy, wz = rec.get("world_xyz", (0.0, 0.0, 0.0))
            parts = [
                f"sign_id={rec['sign_id']}",
                f"subtype={rec.get('subtype', '')}",
                f"bp={rec.get('bp_class', '')}",
                f"snap={'true' if rec.get('target_label') else 'false'}",
            ]
            if rec.get("target_label"):
                parts.append(f"target={rec['target_label']}")
            if rec.get("mesh_name"):
                parts.append(f"mesh={rec['mesh_name']}")
            if rec.get("xy_dist_cm") is not None:
                parts.append(f"xy_dist_cm={rec['xy_dist_cm']:.1f}")
            parts += [
                f"world_x={wx:.1f}", f"world_y={wy:.1f}", f"world_z={wz:.1f}",
            ]
            if rec.get("lat") is not None:
                parts.append(f"lat={rec['lat']:.6f}")
            if rec.get("lon") is not None:
                parts.append(f"lon={rec['lon']:.6f}")
            if rec.get("ele") is not None:
                parts.append(f"ele={rec['ele']:.3f}")
            if rec.get("local_x") is not None:
                parts.append(f"local_x={rec['local_x']:.2f}")
            if rec.get("local_y") is not None:
                parts.append(f"local_y={rec['local_y']:.2f}")
            if rec.get("mgrs_code"):
                parts.append(f"mgrs={rec['mgrs_code']}")
            f.write(" ".join(parts) + "\n")

        # 4. SNAP_SKIPPED
        f.write(f"\n## SNAP_SKIPPED ({len(report.snap_skipped)} entries)\n")
        f.write("# lanelet2 way exists but no Odaiba mesh within snap_radius\n")
        f.write("\n# snap_skipped\n")
        for rec in getattr(report, "snap_skipped_records", []):
            parts = [
                f"sign_id={rec['sign_id']}",
                f"subtype={rec.get('subtype', '')}",
                f"reason={rec.get('reason', 'no_snap_target')}",
                f"target_x={rec['target_x']:.1f}",
                f"target_y={rec['target_y']:.1f}",
                f"target_z={rec['target_z']:.1f}",
            ]
            if rec.get("lat") is not None:
                parts.append(f"lat={rec['lat']:.6f}")
            if rec.get("lon") is not None:
                parts.append(f"lon={rec['lon']:.6f}")
            if rec.get("ele") is not None:
                parts.append(f"ele={rec['ele']:.3f}")
            if rec.get("local_x") is not None:
                parts.append(f"local_x={rec['local_x']:.2f}")
            if rec.get("local_y") is not None:
                parts.append(f"local_y={rec['local_y']:.2f}")
            if rec.get("mgrs_code"):
                parts.append(f"mgrs={rec['mgrs_code']}")
            f.write(" ".join(parts) + "\n")

        # 5. FAILED
        f.write(f"\n## FAILED ({len(report.failed)} entries)\n")
        f.write("\n# failed\n")
        for sign_id, error in report.failed:
            err_escaped = error.replace('"', '\\"').replace('\n', '\\n')
            f.write(f'sign_id={sign_id} error="{err_escaped}"\n')

        # 6. UNUSED_MESHES
        f.write(f"\n## UNUSED_MESHES ({len(report.unused_existing_meshes)} entries)\n")
        f.write("# Odaiba mesh exists but no lanelet2 way matched (reverse mismatch)\n")
        f.write("\n# unused_meshes\n")
        for label, loc in report.unused_existing_meshes:
            wx, wy, wz = loc[0], loc[1], loc[2]
            parts = [
                f"label={label}",
                f"world_x={wx:.1f}",
                f"world_y={wy:.1f}",
                f"world_z={wz:.1f}",
            ]
            f.write(" ".join(parts) + "\n")
        f.write("\n")

        # 7. PEDESTRIAN_MESH_MATERIALS
        if report.pedestrian_material_stats:
            f.write("\n# pedestrian_mesh_materials\n")
            for row in report.pedestrian_material_stats:
                f.write(
                    f"mesh={row['mesh']} num_elements={row['num_elements']} "
                    f"elements=\"{','.join(row['elements'])}\" count={row['count']}\n"
                )
            # summary 行
            total = sum(r["count"] for r in report.pedestrian_material_stats)
            unique = len(report.pedestrian_material_stats)
            n3 = sum(1 for r in report.pedestrian_material_stats if r["num_elements"] == 3)
            n2 = sum(1 for r in report.pedestrian_material_stats if r["num_elements"] == 2)
            f.write("\n# pedestrian_mesh_summary\n")
            f.write(
                f"total_actors={total} unique_meshes={unique} "
                f"meshes_with_3_elements={n3} meshes_with_2_elements={n2}\n"
            )


# ---------------------------------------------------------------------------
# main entry: place_from_specs
# ---------------------------------------------------------------------------

def place_from_specs(
    placements: list,
    groups: list,
    save_level: bool = False,
    snap_to_existing_mesh: bool = False,
    snap_radius_cm: float = 300.0,
    skip_when_snap_fails: bool = True,
    report_path: Optional[str] = None,
    report_metadata: Optional[dict] = None,
    group_bp_path: Optional[str] = None,
) -> PlacementReport:
    """PlacementSpec のリストをレベルに冪等に配置する。

    core.api.generate_placements() の出力をそのまま渡すことを想定している。
    全処理を 1 つの ScopedEditorTransaction で囲むため、失敗時は Ctrl+Z で
    一括 Undo が可能。

    Args:
        placements: core.api.generate_placements() の戻り値 [0] (PlacementSpec のリスト)
        groups:     core.api.generate_placements() の戻り値 [1] (GroupSpec のリスト)
        save_level: True なら配置後に現在のレベルをディスクに保存する。
                    デフォルトは False (手動で Ctrl+S を推奨)。
        snap_to_existing_mesh: True ならば、各 spawn 前に対応する既存メッシュ
                    (車両用は `Traffic_Lights_*`、歩行者用は
                    `Pedestrian_Lights_*`) の位置・回転を採用する
                    (Odaiba 専用の微調整モード)。yaw 3°前後・Z 数 cm の
                    ドリフトを吸収。`snap_radius_cm` 以内のメッシュのみ採用。
        snap_radius_cm: スナップ判定の最大距離 (cm)。
        skip_when_snap_fails: snap_to_existing_mesh=True かつ snap target が
                    見つからない場合、新規 spawn を skip する。Odaiba.umap の
                    3D 街並みカバー外の信号機が空中に出現するのを防ぐ。
                    skip された spec は report.snap_skipped に集計される。

    Returns:
        PlacementReport: 配置結果のサマリ。
                         report.failed に失敗した (sign_id, 理由) タプルのリストが入る。
                         report.snap_skipped に snap fail で skip した
                         (sign_id, location_cm) のリストが入る。
    """
    report = PlacementReport()

    with unreal.ScopedEditorTransaction("Generate Traffic Lights from lanelet2"):
        sign_id_to_actor: dict = {}

        # Phase 4.2: snap 対象メッシュの Z 統計を 1 回だけ収集。
        # _find_nearest_existing_signal_mesh に渡すと、prefix 別の Tukey range
        # で Z 妥当性を判定する (マップ非依存)。stats が空なら静的 fallback。
        mesh_z_stats = None
        if snap_to_existing_mesh:
            mesh_z_stats = _collect_mesh_z_stats()
            for prefix, st in mesh_z_stats.items():
                unreal.log(
                    f"place_from_specs[Z stats]: {prefix} n={st.n} "
                    f"min={st.z_min:.1f} q25={st.z_q25:.1f} "
                    f"median={st.z_median:.1f} q75={st.z_q75:.1f} "
                    f"max={st.z_max:.1f} | tukey=[{st.tukey_low:.1f}, {st.tukey_high:.1f}]"
                )
            # スナップショットを report に保持 (レポート出力用)
            report.z_stats_snapshot = {
                prefix: {
                    "n": st.n,
                    "z_min": st.z_min, "z_max": st.z_max,
                    "z_median": st.z_median,
                    "z_q25": st.z_q25, "z_q75": st.z_q75,
                    "tukey_low": st.tukey_low, "tukey_high": st.tukey_high,
                }
                for prefix, st in mesh_z_stats.items()
            }

        # Phase 6 課題 D: 歩行者用メッシュの Material 構成を集計
        try:
            report.pedestrian_material_stats = _collect_pedestrian_mesh_materials()
            for row in report.pedestrian_material_stats:
                unreal.log(
                    f"pedestrian_mesh: mesh={row['mesh']} num_elements={row['num_elements']} "
                    f"elements=\"{','.join(row['elements'])}\" count={row['count']}"
                )
        except Exception as e:
            unreal.log_warning(
                f"_collect_pedestrian_mesh_materials failed: {e}. continuing without stats."
            )
            report.pedestrian_material_stats = []

        # snap モードで採用された既存メッシュのラベル集合 (逆方向漏れ検出用)
        used_mesh_labels: set = set()

        for spec in placements:
            try:
                actor, was_created, snap_info = _spawn_or_update(
                    spec,
                    snap_to_existing_mesh=snap_to_existing_mesh,
                    snap_radius_cm=snap_radius_cm,
                    skip_when_snap_fails=skip_when_snap_fails,
                    mesh_z_stats=mesh_z_stats,
                )
                if snap_info is not None:
                    used_mesh_labels.add(snap_info["label"])
                if actor is None:
                    # snap fail で skip された (new spawn のみ; 既存更新は actor を返す)
                    report.snap_skipped.append((spec.sign_id, spec.location_cm))
                    report.snap_skipped_records.append({
                        "sign_id": spec.sign_id,
                        "subtype": spec.subtype,
                        "reason": "no_snap_target",
                        "target_x": spec.location_cm[0],
                        "target_y": spec.location_cm[1],
                        "target_z": spec.location_cm[2],
                        "lat": spec.lat,
                        "lon": spec.lon,
                        "ele": spec.ele,
                        "local_x": spec.local_x,
                        "local_y": spec.local_y,
                        "mgrs_code": spec.mgrs_code,
                    })
                    continue
                sign_id_to_actor[spec.sign_id] = actor
                # 配置レコード (レポート出力用)
                rec = {
                    "sign_id": spec.sign_id,
                    "subtype": spec.subtype,
                    "was_created": was_created,
                    "bp_class": spec.actor_class_path.split("/")[-1].split(".")[0],
                    "target_label": (snap_info["label"] if snap_info else None),
                    "xy_dist_cm": (snap_info["xy_dist_cm"] if snap_info else None),
                    "mesh_name": (snap_info["mesh_name"] if snap_info else None),
                    "world_xyz": (
                        actor.get_actor_location().x,
                        actor.get_actor_location().y,
                        actor.get_actor_location().z,
                    ),
                    "lat": spec.lat,
                    "lon": spec.lon,
                    "ele": spec.ele,
                    "local_x": spec.local_x,
                    "local_y": spec.local_y,
                    "mgrs_code": spec.mgrs_code,
                }
                report.placed_records.append(rec)
                if was_created:
                    report.created += 1
                else:
                    report.updated += 1
            except Exception as e:
                report.failed.append((spec.sign_id, str(e)))
                unreal.log_error(
                    f"place_from_specs: sign_id={spec.sign_id} failed: {e}"
                )

        # 逆方向の漏れ: 既存メッシュとして存在するが、どの lanelet2 way とも
        # マッチしなかった信号機メッシュをレポート (Phase 4.2 追加)。
        if snap_to_existing_mesh:
            report.unused_existing_meshes = _collect_unused_meshes(used_mesh_labels)
            unreal.log(
                f"editor_placer[unused]: existing meshes not snapped by any "
                f"lanelet2 way = {len(report.unused_existing_meshes)}"
            )
            for label, (x, y, z) in report.unused_existing_meshes[:10]:
                unreal.log(
                    f"  unused: {label} at ({x:.1f}, {y:.1f}, {z:.1f})"
                )
            if len(report.unused_existing_meshes) > 10:
                unreal.log(
                    f"  ... and {len(report.unused_existing_meshes) - 10} more"
                )

        # グループ配置 (individual アクター配置後に実行する必要がある)
        # Phase 6: subtype 別 Controller のために sign_id → subtype マップを構築
        sign_id_to_subtype = {p.sign_id: p.subtype for p in placements}
        gc, gu = _place_groups(
            groups,
            sign_id_to_actor,
            sign_id_to_subtype,
            group_bp_path=group_bp_path,
        )
        report.groups_created = gc
        report.groups_updated = gu

        unreal.log(
            f"editor_placer: TL created={report.created} updated={report.updated} "
            f"snap_skipped={len(report.snap_skipped)} failed={len(report.failed)} "
            f"unused_meshes={len(report.unused_existing_meshes)} | "
            f"groups created={gc} updated={gu}"
        )

    if save_level:
        # 注意: 保存は Undo スタックをフラッシュしないが、ユーザーに明示的に
        # 承認させるため save_level=True は明示的に渡した場合のみ実行する。
        unreal.EditorLevelLibrary.save_current_level()

    # レポートファイル出力 (Phase 4.2 追加): 省略なし全件
    if report_path:
        try:
            _write_full_run_report(
                report, report_path,
                n_input_placements=len(placements),
                metadata=report_metadata,
            )
            unreal.log(f"[lanelet2_tl] wrote {report_path}")
        except Exception as e:
            unreal.log_error(f"[lanelet2_tl] failed to write report {report_path}: {e}")
    else:
        unreal.log("[lanelet2_tl] report_path not set; skipping report file output")

    return report

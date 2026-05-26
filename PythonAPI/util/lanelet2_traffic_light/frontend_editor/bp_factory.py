"""subtype 別 派生 Blueprint の自動生成 (Phase 5 ①)。

別マップに展開する際、各マップ向けの派生 BP (例 BP_ChibaVehicleTL) を
T4 サンプル BP の派生として Editor から自動生成する。

snap モードでは StaticMesh / Rotation が dynamic 上書きされるため、
派生 BP の中身はほぼ空でよい。重要なのは parent クラスを継ぐこと:
T4 サンプル BP を親にすれば Carla 標準の信号機点灯ロジックが受け継がれ、
未指定 (None) 時は C++ TrafficLightBase 直派生となる。

unreal モジュール依存 (Editor 内でのみ動作)。パス組み立てロジックは
bp_naming.py に分離してテスト可能。
"""
from typing import Optional

import unreal

from lanelet2_traffic_light.frontend_editor.bp_naming import (
    derive_group_bp_path,
    derive_subtype_bp_path,
)


def _asset_path_only(asset_path: str) -> str:
    """`/X/Y/BP_Foo.BP_Foo` 形式から package path 部 `/X/Y/BP_Foo` を返す。"""
    if "." in asset_path:
        return asset_path.split(".")[0]
    if asset_path.endswith("_C"):
        return asset_path[:-2]
    return asset_path


def _load_parent_class(parent_bp_path: Optional[str]):
    """parent_bp_path の Generated Class をロード。
    None なら C++ TrafficLightBase を返す。
    """
    if parent_bp_path is None:
        return unreal.TrafficLightBase
    asset_path = _asset_path_only(parent_bp_path)
    cls = unreal.EditorAssetLibrary.load_blueprint_class(asset_path)
    if cls is None:
        # T4 サンプル BP が見つからないケース。warning を出して fallback。
        unreal.log_warning(
            f"bp_factory: parent Blueprint not found at '{asset_path}', "
            f"falling back to C++ TrafficLightBase. Lighting logic will be absent."
        )
        return unreal.TrafficLightBase
    return cls


def create_derived_bp(parent_bp_path: Optional[str], target_asset_path: str) -> bool:
    """parent BP の派生 BP を target_asset_path に作成する。

    Args:
        parent_bp_path: 親 BP の Asset Path。None なら C++ TrafficLightBase。
        target_asset_path: 作成先 Asset Path。
            例 `/Game/Carla/Blueprints/Chiba/BP_ChibaVehicleTL.BP_ChibaVehicleTL`。

    Returns:
        True なら新規作成、False なら既存ありで再利用。

    Raises:
        RuntimeError: asset 作成に失敗した場合。
    """
    target_pkg = _asset_path_only(target_asset_path)
    if unreal.EditorAssetLibrary.does_asset_exist(target_pkg):
        return False

    parent_cls = _load_parent_class(parent_bp_path)

    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", parent_cls)

    package_path, asset_name = target_pkg.rsplit("/", 1)
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    new_asset = asset_tools.create_asset(
        asset_name=asset_name,
        package_path=package_path,
        asset_class=unreal.Blueprint,
        factory=factory,
    )
    if new_asset is None:
        raise RuntimeError(
            f"bp_factory: AssetTools.create_asset failed for {target_pkg}"
        )

    unreal.EditorAssetLibrary.save_asset(target_pkg)
    return True


def ensure_group_bp(
    map_name: str,
    output_dir: Optional[str] = None,
) -> str:
    """C++ `ATrafficLightGroup` の派生 BP を map ごとに作成 or 再利用する。

    Phase 5 ② 追加: snap モード配置の TL を Carla 標準の信号機制御
    (Group → Controller → Component) に接続するため、各マップで使う
    BP_<MapName>TrafficLightGroup を自動生成する。parent class は C++ の
    `unreal.TrafficLightGroup` 直派生で十分 (Tick 処理は C++ 側にある)。

    Args:
        map_name: 例 "Odaiba" / "Chiba"。
        output_dir: 派生先 (省略時 /Game/Carla/Blueprints/{map_name})。

    Returns:
        作成 or 再利用された Group 派生 BP の Asset Path。
        `_place_groups()` に渡して spawn 元クラスとして使う。
    """
    target = derive_group_bp_path(map_name, output_dir)
    target_pkg = _asset_path_only(target)
    if unreal.EditorAssetLibrary.does_asset_exist(target_pkg):
        unreal.log(f"bp_factory: reused existing Group BP {target}")
        return target

    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", unreal.TrafficLightGroup)
    package_path, asset_name = target_pkg.rsplit("/", 1)
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    new_asset = asset_tools.create_asset(
        asset_name=asset_name,
        package_path=package_path,
        asset_class=unreal.Blueprint,
        factory=factory,
    )
    if new_asset is None:
        raise RuntimeError(
            f"bp_factory: AssetTools.create_asset failed for Group BP {target_pkg}"
        )
    unreal.EditorAssetLibrary.save_asset(target_pkg)
    unreal.log(
        f"bp_factory: created Group BP {target} "
        f"(parent=C++ ATrafficLightGroup)"
    )
    return target


def ensure_subtype_bps(
    profile,
    map_name: str,
    output_dir: Optional[str] = None,
) -> dict:
    """profile の各 subtype について派生 BP を ensure し、{subtype: target_path} を返す。

    既存 BP があれば再利用 (冪等)。core.api.generate_placements の
    `bp_override` 引数にそのまま渡せる形式で返す。

    Args:
        profile: `parent_bp_path_for(subtype)` と `_subtype_to_bp` を持つ
            プロファイル (例 PROFILE_JP)。後者から処理対象 subtype を列挙。
        map_name: マップ名 (派生 BP 命名のキー)。例 "Odaiba" / "Chiba"。
        output_dir: 派生先ディレクトリ (省略時 /Game/Carla/Blueprints/{map_name})。

    Returns:
        {subtype: target_asset_path} の dict。
    """
    result = {}
    # subtype 列挙: profile の _subtype_to_bp の keys を使う
    subtypes = list(profile._subtype_to_bp.keys())
    for subtype in subtypes:
        target = derive_subtype_bp_path(subtype, map_name, output_dir)
        parent = profile.parent_bp_path_for(subtype)
        try:
            created = create_derived_bp(parent, target)
            if created:
                unreal.log(
                    f"bp_factory: created derived BP {target} "
                    f"(parent={parent or '<C++ TrafficLightBase>'})"
                )
            else:
                unreal.log(f"bp_factory: reused existing BP {target}")
            result[subtype] = target
        except Exception as e:
            unreal.log_error(
                f"bp_factory: failed to ensure {target}: {e}. "
                f"Falling back to profile default."
            )
            # フォールバック: profile の既存 _subtype_to_bp パス
            result[subtype] = profile.bp_class_for(subtype)
    return result

"""subtype 別 派生 Blueprint の Asset Path 組み立て。

unreal 非依存 (pure Python)。テスト可能。

Phase 5 ① 追加: 別マップで派生 BP を自動生成するためのパス命名規則を集約。
既定で `/Game/Carla/Blueprints/{map_name}/BP_{map_name}{Role}TL.BP_...` という
規則を採用するが、output_dir や name_template を差し替えれば変更可能。
"""
from typing import Optional


# subtype → 派生 BP の役割名 (BP 命名で使用)
_SUBTYPE_TO_ROLE = {
    "red_yellow_green": "Vehicle",
    "red_green":        "Pedestrian",
}


def role_for_subtype(subtype: str) -> str:
    """subtype に対応する役割名 ("Vehicle" / "Pedestrian") を返す。"""
    if subtype not in _SUBTYPE_TO_ROLE:
        raise KeyError(f"unknown subtype: {subtype}")
    return _SUBTYPE_TO_ROLE[subtype]


def derive_group_bp_path(
    map_name: str,
    output_dir: Optional[str] = None,
    name_template: str = "BP_{map_name}TrafficLightGroup",
) -> str:
    """map_name から `ATrafficLightGroup` 派生 BP の Asset Path を組み立てる。

    例: map_name="Odaiba"
        → /Game/Carla/Blueprints/Odaiba/BP_OdaibaTrafficLightGroup.BP_OdaibaTrafficLightGroup

    Phase 5 ② で追加: snap モード配置の TL を Controller→Group 経由で Carla 標準の
    信号機制御に接続するため、各マップ用に C++ ATrafficLightGroup の派生 BP を
    自動生成する。
    """
    bp_name = name_template.format(map_name=map_name)
    dir_path = output_dir if output_dir is not None else f"/Game/Carla/Blueprints/{map_name}"
    return f"{dir_path}/{bp_name}.{bp_name}"


def derive_subtype_bp_path(
    subtype: str,
    map_name: str,
    output_dir: Optional[str] = None,
    name_template: str = "BP_{map_name}{role}TL",
) -> str:
    """subtype + map_name から派生 BP の Asset Path (no `_C`) を組み立てる。

    Args:
        subtype: lanelet2 subtype。"red_yellow_green" or "red_green"。
        map_name: マップ名 (BP 命名のキー)。例 "Odaiba"。
        output_dir: `/Game/...` 形式のディレクトリ。
            None なら `/Game/Carla/Blueprints/{map_name}`。
        name_template: 命名テンプレート。`{map_name}` と `{role}` を含める。

    Returns:
        Asset Path 文字列。例:
        ``/Game/Carla/Blueprints/Odaiba/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL``
    """
    role = role_for_subtype(subtype)
    bp_name = name_template.format(map_name=map_name, role=role)
    dir_path = output_dir if output_dir is not None else f"/Game/Carla/Blueprints/{map_name}"
    return f"{dir_path}/{bp_name}.{bp_name}"

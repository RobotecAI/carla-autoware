"""日本向け既定プロファイル。

BP パスは Phase 2 で作成する Odaiba 用 BP をデフォルトに指す。
別マップで使う場合は generate_traffic_lights(bp_override=...) で上書きする。

Phase 0 / Phase 4.2 検証で確定した値:
- default_pole_height_m: 12.3 m (Phase 0 — Odaiba overhead gantry)
- subtype 別 pole_height (Phase 4.2 で配置済 190 件の Z 分布実測):
    - red_yellow_green (車両): 11.76 m (median)
    - red_green (歩行者):       9.18 m (median)
- yaw_offset_deg: +90.0 deg (Phase 0 実機確認済み)
- front_is_left_to_right: True (暫定値、Phase 2 で再確認)
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class _ProfileJP:
    name: str = "jp"
    # Blueprint asset path (without the "_C" Generated Class suffix).
    # `EditorAssetLibrary.load_blueprint_class()` expects the asset path
    # (the `_C` is appended internally to return the GeneratedClass).
    _subtype_to_bp: dict[str, str] = field(default_factory=lambda: {
        "red_yellow_green": "/Game/Carla/Blueprints/Odaiba/BP_OdaibaVehicleTL.BP_OdaibaVehicleTL",
        "red_green":        "/Game/Carla/Blueprints/Odaiba/BP_OdaibaPedestrianTL.BP_OdaibaPedestrianTL",
    })
    # subtype 別 pole_height (Phase 4.2 で実測した Odaiba の中央値)。
    # 別マップでは将来 map_override で上書きする想定。
    _subtype_to_pole_height_m: dict[str, float] = field(default_factory=lambda: {
        "red_yellow_green": 11.76,  # Odaiba 車両用 median
        "red_green":         9.18,  # Odaiba 歩行者用 median
    })
    # Phase 5 ①: 派生 BP を自動生成する際の親 Blueprint Asset Path。
    # T4 サンプル BP を継承することで、Carla 標準の信号機点灯ロジックを
    # 受け継いだ派生 BP がマップごとに作れる。
    # parent_bp_path=None の subtype は C++ TrafficLightBase 直派生になる
    # (点灯ロジック無し、snap モードでメッシュだけ機能する最小構成)。
    #
    # Plugin Content の UE5 Asset Path は `/<PluginName>/<ContentSubdir>/...` 形式。
    # T4 Plugin の場合: /T4/TrafficLightSample/VehicleTrafficLight/BP_VehicleTrafficLight
    # (`/All/Plugins/T4/...` は Content Browser の表示 path で内部 Asset Path ではない)
    _subtype_to_parent_bp_path: dict[str, str] = field(default_factory=lambda: {
        "red_yellow_green":
            "/T4/TrafficLightSample/VehicleTrafficLight/BP_VehicleTrafficLight.BP_VehicleTrafficLight",
        "red_green":
            "/T4/TrafficLightSample/PedestrianTrafficLight/BP_PedestrianTrafficLight.BP_PedestrianTrafficLight",
    })
    _default_pole_height_m: float = 12.3    # subtype 不明時のフォールバック (Phase 0 値)
    _yaw_offset_deg: float = +90.0          # Phase 0 で確定
    _front_is_left_to_right: bool = True    # Phase 0.3 で確定 (暫定値)

    def bp_class_for(self, subtype: str) -> str:
        if subtype not in self._subtype_to_bp:
            raise KeyError(f"unknown subtype: {subtype}")
        return self._subtype_to_bp[subtype]

    def default_pole_height_m(self) -> float:
        return self._default_pole_height_m

    def pole_height_m(self, subtype: str) -> float:
        return self._subtype_to_pole_height_m.get(subtype, self._default_pole_height_m)

    def parent_bp_path_for(self, subtype: str):
        return self._subtype_to_parent_bp_path.get(subtype)

    def yaw_offset_deg(self) -> float:
        return self._yaw_offset_deg

    def front_is_left_to_right(self) -> bool:
        return self._front_is_left_to_right


PROFILE_JP = _ProfileJP()

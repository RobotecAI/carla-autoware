"""Default profile for Japan.

BP paths point to the Odaiba BPs created in Phase 2 by default.
Use generate_traffic_lights(bp_override=...) to override when using a different map.

Values confirmed in Phase 0 / Phase 4.2 validation:
- default_pole_height_m: 12.3 m (Phase 0 -- Odaiba overhead gantry)
- pole_height per subtype (measured from Z-distribution of 190 placed actors in Odaiba, confirmed in Phase 4.2):
    - red_yellow_green (vehicle):   11.76 m (median)
    - red_green (pedestrian):        9.18 m (median)
- yaw_offset_deg: +90.0 deg (confirmed on real hardware in Phase 0)
- front_is_left_to_right: True (provisional value, to be re-confirmed in Phase 2)
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
    # Pole height per subtype (median values measured for Odaiba in Phase 4.2).
    # Intended to be overridden via map_override for other maps in the future.
    _subtype_to_pole_height_m: dict[str, float] = field(default_factory=lambda: {
        "red_yellow_green": 11.76,  # Odaiba vehicle median
        "red_green":         9.18,  # Odaiba pedestrian median
    })
    # Phase 5 step 1: Parent Blueprint Asset Path used when auto-generating derived BPs.
    # By inheriting from the T4 sample BP, derived BPs per map will carry over
    # the standard CARLA traffic light lighting logic.
    # Subtypes with parent_bp_path=None become direct subclasses of C++ TrafficLightBase
    # (no lighting logic; minimal configuration where only the mesh works in snap mode).
    #
    # UE5 Asset Paths for Plugin Content follow the `/<PluginName>/<ContentSubdir>/...` format.
    # For the T4 Plugin: /T4/TrafficLightSample/VehicleTrafficLight/BP_VehicleTrafficLight
    # (`/All/Plugins/T4/...` is the Content Browser display path, not the internal Asset Path)
    _subtype_to_parent_bp_path: dict[str, str] = field(default_factory=lambda: {
        "red_yellow_green":
            "/T4/TrafficLightSample/VehicleTrafficLight/BP_VehicleTrafficLight.BP_VehicleTrafficLight",
        "red_green":
            "/T4/TrafficLightSample/PedestrianTrafficLight/BP_PedestrianTrafficLight.BP_PedestrianTrafficLight",
    })
    _default_pole_height_m: float = 12.3    # Fallback for unknown subtypes (Phase 0 value)
    _yaw_offset_deg: float = +90.0          # Confirmed in Phase 0
    _front_is_left_to_right: bool = True    # Confirmed in Phase 0.3 (provisional)

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

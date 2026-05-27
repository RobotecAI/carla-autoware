"""Protocol definition for country/region profiles."""
from typing import Optional, Protocol


class TrafficLightProfile(Protocol):
    name: str

    def bp_class_for(self, subtype: str) -> str: ...
    def default_pole_height_m(self) -> float: ...
    def pole_height_m(self, subtype: str) -> float:
        """Pole height (m) per subtype.
        Must return default_pole_height_m() if the subtype is not defined.
        """
        ...
    def parent_bp_path_for(self, subtype: str) -> Optional[str]:
        """Asset Path of the Blueprint to use as parent when auto-generating derived BPs.
        If None is returned, the BP is created as a direct subclass of C++ TrafficLightBase
        (Phase 5 step 1 -- for use with bp_factory.ensure_subtype_bps).
        """
        ...
    def yaw_offset_deg(self) -> float: ...
    def front_is_left_to_right(self) -> bool: ...

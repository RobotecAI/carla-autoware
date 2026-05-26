"""国/地域プロファイルの Protocol 定義。"""
from typing import Optional, Protocol


class TrafficLightProfile(Protocol):
    name: str

    def bp_class_for(self, subtype: str) -> str: ...
    def default_pole_height_m(self) -> float: ...
    def pole_height_m(self, subtype: str) -> float:
        """subtype 別の信号機ポール高 (m)。
        subtype 未定義の場合は default_pole_height_m() を返すこと。
        """
        ...
    def parent_bp_path_for(self, subtype: str) -> Optional[str]:
        """派生 BP を自動生成する際に親とする Blueprint の Asset Path。
        None を返した場合、C++ TrafficLightBase を親として作成する想定
        (Phase 5 ① — bp_factory.ensure_subtype_bps 用)。
        """
        ...
    def yaw_offset_deg(self) -> float: ...
    def front_is_left_to_right(self) -> bool: ...

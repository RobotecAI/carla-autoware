"""SignID 生成戦略。初期実装は lanelet2 way_id をそのまま str 化する。

将来の差し替え候補:
- MappingTableResolver: JSON 対応表で way_id → 任意 SignID
- CompoundResolver: way_id + 命名規則 (例 "TL_<wayid>")
"""
from typing import Optional
from lanelet2_traffic_light.corelib.ir.traffic_light_ir import TrafficLightSpec, GroupSpec


class WayIdResolver:
    def resolve(self, tl: TrafficLightSpec, group: Optional[GroupSpec]) -> str:
        return str(tl.way_id)

"""SignID generation strategy. The initial implementation simply converts the lanelet2 way_id to a string.

Future replacement candidates:
- MappingTableResolver: way_id -> arbitrary SignID via a JSON mapping table
- CompoundResolver: way_id + naming convention (e.g. "TL_<wayid>")
"""
from typing import Optional
from lanelet2_traffic_light.corelib.ir.traffic_light_ir import TrafficLightSpec, GroupSpec


class WayIdResolver:
    def resolve(self, tl: TrafficLightSpec, group: Optional[GroupSpec]) -> str:
        return str(tl.way_id)

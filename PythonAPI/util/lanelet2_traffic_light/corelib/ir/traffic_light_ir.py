from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class Node:
    id: int
    lat: float
    lon: float
    local_x: float
    local_y: float
    mgrs_code: str
    ele: Optional[float] = None


@dataclass(frozen=True)
class TrafficLightSpec:
    way_id: int
    subtype: str
    p0: Node
    p1: Node
    height: float
    raw_tags: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class LightBulbsSpec:
    way_id: int
    bulbs: list[Node]


@dataclass(frozen=True)
class GroupSpec:
    relation_id: int
    refers: list[int]
    light_bulbs: list[int]
    ref_line: Optional[int] = None


@dataclass(frozen=True)
class PlacementSpec:
    sign_id: str
    actor_class_path: str
    # location_cm: Unreal world coordinates in centimeters (X, Y, Z).
    location_cm: tuple[float, float, float]
    # rotation_deg: (roll, pitch, yaw) in degrees.
    # NOTE: this is *not* the order `unreal.Rotator(pitch, yaw, roll)` uses
    # positionally — frontends must map explicitly when constructing FRotator.
    rotation_deg: tuple[float, float, float]
    group_relation_id: Optional[int] = None
    source_way_id: int = 0
    subtype: str = ""
    # Representative point from lanelet2 (midpoint of TL's p0/p1)
    lat: float = 0.0
    lon: float = 0.0
    ele: Optional[float] = None
    local_x: float = 0.0
    local_y: float = 0.0
    mgrs_code: str = ""

"""Module for estimating the center position (local coordinates, m) and yaw (degrees)
of a traffic light from two points of a traffic_light way plus a yaw_offset.
Z is computed separately from height_m via mgrs_transform.

CARLA-Odaiba conversion confirmed in Phase 0:
- The Y axis (Northing) in lanelet2 is sign-inverted relative to the Unreal Y axis
  (same reason as y_sign=-1 in mgrs_transform)
- Therefore the yaw calculation also inverts the sign of dy, using `atan2(-dy, dx)`
"""
import math
from lanelet2_traffic_light.corelib.ir.traffic_light_ir import TrafficLightSpec


def estimate_center_and_yaw(spec: TrafficLightSpec, yaw_offset_deg: float = -90.0) -> tuple[float, float, float]:
    """Compute the midpoint and Unreal world Yaw from the two points of a way.

    Returns:
        (center_x_m, center_y_m, yaw_deg)
        - center is in lanelet2 local_x/local_y units (m); Unreal conversion is handled by the caller.
        - yaw is around the Unreal world Z axis (degrees).
          Internal formula: math.degrees(atan2(-dy, dx)) + yaw_offset_deg
          - (-dy) accounts for the lanelet2 Y inversion
          - yaw_offset_deg corrects for the forward-facing direction of the signal face
    """
    cx = (spec.p0.local_x + spec.p1.local_x) / 2.0
    cy = (spec.p0.local_y + spec.p1.local_y) / 2.0
    dx = spec.p1.local_x - spec.p0.local_x
    dy = spec.p1.local_y - spec.p0.local_y
    # Y is inverted when converting lanelet2 -> Unreal, so use -dy in yaw calculation as well
    way_yaw_deg = math.degrees(math.atan2(-dy, dx))
    return (cx, cy, way_yaw_deg + yaw_offset_deg)
